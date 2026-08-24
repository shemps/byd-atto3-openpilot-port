import copy
import os

from opendbc.car import Bus, create_button_events, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.byd.values import DBC, CarControllerParams as CCP
from opendbc.car.interfaces import CarStateBase

GearShifter = structs.CarState.GearShifter
ButtonType = structs.CarState.ButtonEvent.Type

# BYD gear enum from DRIVE_STATE.GEAR
GEAR_MAP = {
  1: GearShifter.park,
  2: GearShifter.reverse,
  3: GearShifter.neutral,
  4: GearShifter.drive,
}

# Speed-rocker states for create_button_events. If the rocker direction is reversed on
# your car, exchange the two ButtonTypes here.
ROCKER_DOWN, ROCKER_UP = 1, 2
ROCKER_BUTTONS = {ROCKER_DOWN: ButtonType.decelCruise, ROCKER_UP: ButtonType.accelCruise}


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.lkas_hud = {}
    # Button engage without the MVS4 in the loop. cruiseState.enabled latches on a button
    # press and clears on brake, gas, LKAS or ACC button. It is independent of the stock
    # ACC state. The panda safety param 0x1000 latch does the same (modes/byd.h). The
    # environment variable POC_BTN_ENGAGE=1 enables both.
    self.poc_btn_engage = os.getenv("POC_BTN_ENGAGE") == "1"
    self.btn_latch = False
    self.btn_prev = False
    # openpilot longitudinal. The cruise helper (selfdrive/car/cruise.py through card.py)
    # owns the set speed. It reads standard buttonEvents. It does not read
    # cruiseState.speed, which openpilot ignores when pcmCruise is False. The rocker
    # therefore sends buttonEvents.
    self.op_long = os.getenv("USE_OP_LONG") == "1"
    self.rocker_prev = False
    self.rocker_pending = False
    self.rocker_state = 0    # 0=released, else ROCKER_DOWN/ROCKER_UP latched at confirm
    # EPS state-11 debounce, see the dropout block in update()
    self.eps_dropout_frames = 0
    self.btn_engage_edge = False  # latch rising edge, consumed by update_button_enable()

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    # speed
    speed_kph = cp.vl["WHEELSPEED_CLEAN"]["WHEELSPEED_CLEAN"]
    ret.vEgoRaw = speed_kph * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = speed_kph < 0.1
    ret.vEgoCluster = ret.vEgo

    # steering wheel
    ret.steeringAngleDeg = cp.vl["STEER_MODULE_2"]["STEER_ANGLE_2"]
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]["MAIN_TORQUE"]        # EPS motor effort, not the driver
    ret.steeringTorqueEps = cp.vl["STEER_MODULE_2"]["DRIVER_EPS_TORQUE"]  # masked in EPS state 10, logs only
    # Driver override reads 0x1FC STEERING_TORQUE.DRIVER_TORQUE, at 0.1 Nm/bit. The car masks
    # DRIVER_EPS_TORQUE while the EPS is in state 10, which is the window openpilot steers, so
    # that signal cannot carry override. steerOverride is ET.OVERRIDE_LATERAL: openpilot gives
    # lateral control to the driver and continues when the driver relaxes. It does not disengage.
    driver_torque = cp.vl["STEERING_TORQUE"]["DRIVER_TORQUE"]
    ret.steeringPressed = self.update_steering_pressed(abs(driver_torque) > CCP.STEER_DRIVER_TORQUE_OVERRIDE_NM, 5)
    # Do not set steeringDisengage on this car. It is a NO_ENTRY "Steering Pressed" plus an
    # immediate USER_DISABLE. steeringPressed above gives the soft lateral-override alert,
    # which is the correct response.

    # LKAS_STATE uses the BYD TJA_STATE_* enum: 0=Off, 1=Passive, 2/3=Active, 4=Fault. The
    # camera fault state is the reliable indicator. LKS_PREPARED on the EPS is also 1 at route
    # start before engagement, so it cannot be a fault flag on its own.
    # Without the MVS4 the camera is not the steering authority. Its fault state must not
    # prevent engagement. Read the EPS health directly from 0x1FC instead.
    ret.steerFaultTemporary = (int(cp_cam.vl["LKAS_HUD_ADAS"]["LKAS_STATE"]) == 4) and not self.poc_btn_engage
    # EPS state 11 on the 0x1FC byte-0 low nibble means the command stream stopped while the
    # EPS was active. The state latches. To re-arm 9->10 the EPS needs an idle REQ=0 stream,
    # then a new STEER_REQ rising edge.
    # byd_atto3.dbc decodes the nibble as bits, not as an enum: 9=PREPARED(+bit3),
    # 10=ACTIVATED(+bit3), 11=PREPARED+ACTIVATED(+bit3). Both bits set is the dropout
    # signature. Bit 3 is undecoded and is constant 1 in every capture.
    # Debounce state 11. It is transitional and is not always a fault. It occurs for 100 to
    # 200 ms when the 0x1E2 source changes, and at the engage handover when the panda stops
    # to forward the camera frames. Without the debounce the steerTempUnavailable NO_ENTRY
    # prevents engagement, because state 11 starts before openpilot transmits. A true latched
    # dropout stays set, so 0.5 s is sufficient. card runs at 100 Hz.
    lks_prepared = bool(cp.vl["STEERING_TORQUE"]["LKS_PREPARED"])
    lks_activated = bool(cp.vl["STEERING_TORQUE"]["CRUISE_ACTIVATED"])
    self.eps_dropout_frames = self.eps_dropout_frames + 1 if (lks_prepared and lks_activated) else 0
    ret.steerFaultTemporary = ret.steerFaultTemporary or (self.eps_dropout_frames > 50)

    # gas / brake
    # gasPressed reads the accelerator pedal, 0x342 GAS_PEDAL, which reads 0 at rest.
    # Do not use DRIVE_STATE.RAW_THROTTLE. That signal is powertrain throttle demand. It
    # pulses 0 to 28 on its own during acceleration while the pedal reads 0.00, which
    # suspends openpilot longitudinal control about every 0.5 s. The two signals agree
    # during braking. The 0.10 deadband ignores light contact and noise. The brake still
    # disengages openpilot.
    ret.gasPressed = cp.vl["PEDAL"]["GAS_PEDAL"] > 0.10
    # brakePressed must read the same signal as the panda latch. If the two differ, a light
    # touch on the brake clears one latch and not the other, and openpilot shows "Controls
    # Mismatch". Both sides read the debounced DRIVE_STATE.BRAKE_PRESSED bit, 0x242 byte 4
    # bit 5, car threshold about 9 counts. Keep the two in agreement.
    ret.brakePressed = bool(cp.vl["DRIVE_STATE"]["BRAKE_PRESSED"])

    # gear
    ret.gearShifter = GEAR_MAP.get(int(cp.vl["DRIVE_STATE"]["GEAR"]), GearShifter.unknown)

    # blinkers
    ret.leftBlinker = bool(cp.vl["STALKS"]["LEFT_BLINKER"])
    ret.rightBlinker = bool(cp.vl["STALKS"]["RIGHT_BLINKER"])

    # blind spot monitor
    ret.leftBlindspot = cp.vl["BSD_RADAR"]["LEFT_APPROACH"] != 0
    ret.rightBlindspot = cp.vl["BSD_RADAR"]["RIGHT_APPROACH"] != 0

    # doors / belt
    ret.doorOpen = any((
      cp.vl["METER_CLUSTER"]["FRONT_LEFT_DOOR"],
      cp.vl["METER_CLUSTER"]["FRONT_RIGHT_DOOR"],
      cp.vl["METER_CLUSTER"]["BACK_LEFT_DOOR"],
      cp.vl["METER_CLUSTER"]["BACK_RIGHT_DOOR"],
    ))
    ret.seatbeltUnlatched = not bool(cp.vl["METER_CLUSTER"]["SEATBELT_DRIVER"])

    # cruise state: ACC messages come from camera bus on Atto 3
    # ACC_STATE: 0=OFF, 2=ACC_ON (available), 3=ACC_ACTIVE (enabled), 5=FORCE_ACCEL, 7=ERROR
    ret.cruiseState.speed = cp_cam.vl["ACC_HUD_ADAS"]["SET_SPEED"] * CV.KPH_TO_MS
    acc_state = int(cp_cam.vl["ACC_HUD_ADAS"]["ACC_STATE"])
    ret.cruiseState.available = acc_state in (2, 3, 5)
    ret.cruiseState.enabled = acc_state in (3, 5)
    ret.cruiseState.standstill = bool(cp_cam.vl["ACC_CMD"]["STANDSTILL_STATE"])

    if self.poc_btn_engage:
      # The LKAS button, LKAS_ON_BTN = 0x3B0 byte 0 bit 6, toggles the engage latch. The
      # brake clears it. Use the LKAS button and not the speed rocker, because the rocker
      # changes the car ACC. The panda latch in modes/byd.h toggles on the same rising edge.
      prev_latch = self.btn_latch
      lkas_btn = bool(cp.vl["PCM_BUTTONS"]["LKAS_ON_BTN"])
      if lkas_btn and not self.btn_prev and not ret.brakePressed:
        # The cruise helper sets the initial cruise target from vEgo.
        self.btn_latch = not self.btn_latch
      if ret.brakePressed:  # only the brake disengages. The driver drives on the gas pedal.
        self.btn_latch = False
      self.btn_prev = lkas_btn
      ret.cruiseState.available = True
      ret.cruiseState.enabled = ret.cruiseState.enabled or self.btn_latch
      # With pcmCruise=False, openpilot engages only on buttonEnable and disengages on a
      # cancel buttonEvent. This is the pcm_enable gate in car_specific.py. Map the toggle
      # onto that: a rising latch gives buttonEnable, and a latch that the button drops
      # gives a cancel event. The brake needs nothing here, because selfdrived reads
      # pedalPressed and the latch clears above and in the panda.
      # Do not assign ret.buttonEnable here. interfaces.py calls update_button_enable()
      # after update() returns and overwrites the field, so a direct write has no effect.
      # The update_button_enable() override below delivers the rising edge.
      self.btn_engage_edge = self.btn_latch and not prev_latch
      button_events = []
      if prev_latch and not self.btn_latch and not ret.brakePressed:
        button_events.append(structs.CarState.ButtonEvent(pressed=True, type=ButtonType.cancel))
      if self.op_long and self.btn_latch:
        # The cruise-speed rocker on 0x3B0 sends standard buttonEvents. The cruise helper
        # steps on the release edge and makes +10 steps on a long hold. On this car
        # SET_BTN, byte 0 bit 3, means "rocker pressed" in both directions. RES_BTN, bit 4,
        # is the direction and is never set alone. The direction latches one frame after
        # the press edge, so a skew between the two bits cannot give a wrong direction.
        # A one-frame pulse is debounced. btn_latch gates this block, so a rocker press
        # cannot be an engage request. The panda controls_allowed latch toggles only on
        # the LKAS button.
        rocker = bool(cp.vl["PCM_BUTTONS"]["SET_BTN"])       # bit 3 = rocker pressed, both directions
        rocker_up = bool(cp.vl["PCM_BUTTONS"]["RES_BTN"])    # bit 4 = direction
        prev_state = self.rocker_state
        if rocker and not self.rocker_prev:
          self.rocker_pending = True
        elif rocker and self.rocker_pending:
          self.rocker_state = ROCKER_UP if rocker_up else ROCKER_DOWN
          self.rocker_pending = False
        elif not rocker:
          self.rocker_state = 0
          self.rocker_pending = False
        self.rocker_prev = rocker
        button_events.extend(create_button_events(self.rocker_state, prev_state, ROCKER_BUTTONS))
      else:
        self.rocker_state = 0
        self.rocker_prev = self.rocker_pending = False
      ret.buttonEvents = button_events
      if self.btn_latch and ret.cruiseState.speed < 8.3:
        ret.cruiseState.speed = 8.3  # floor for when the stock ACC is off and SET_SPEED is 0

    # forward stock LKAS HUD
    self.lkas_hud = copy.copy(cp_cam.vl["LKAS_HUD_ADAS"])

    return ret

  def update_button_enable(self, buttonEvents):
    # interfaces.py calls this after update() and writes the result to
    # CarState.buttonEnable. On the button-engage path the engage request is the rising
    # edge of the LKAS latch, not the falling edge of a cruise button that the base
    # implementation uses. The base version must not run while the latch is set, or the
    # release of a rocker set-speed press reads as an engage request.
    if self.poc_btn_engage:
      return self.btn_engage_edge
    return super().update_button_enable(buttonEvents)

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
