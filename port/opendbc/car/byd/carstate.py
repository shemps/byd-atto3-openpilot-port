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

# speed-rocker states for create_button_events (direction bit4=up still TBD on-car:
# if the physical rocker works backwards, swap the two ButtonTypes here)
ROCKER_DOWN, ROCKER_UP = 1, 2
ROCKER_BUTTONS = {ROCKER_DOWN: ButtonType.decelCruise, ROCKER_UP: ButtonType.accelCruise}


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.lkas_hud = {}
    # POC no-MVS4 button engage: cruiseState.enabled latches on a SET/RES press and clears
    # on brake/gas/LKAS/ACC button, independent of the stock ACC state. Mirrors the panda
    # safety param 0x1000 latch (see modes/byd.h) - both are enabled by POC_BTN_ENGAGE=1.
    self.poc_btn_engage = os.getenv("POC_BTN_ENGAGE") == "1"
    self.btn_latch = False
    self.btn_prev = False
    # openpilot longitudinal (LONG_PLAN.md LG2): the set-speed is owned by the cruise
    # helper (selfdrive/car/cruise.py via card.py), which reads standard buttonEvents -
    # NOT cruiseState.speed (ignored when pcmCruise=False). The rocker therefore emits
    # buttonEvents; route 00000058 proved an internal set-speed counter steps fine but
    # never reaches the planner or the UI.
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
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]["MAIN_TORQUE"]        # EPS MOTOR effort (not the driver)
    ret.steeringTorqueEps = cp.vl["STEER_MODULE_2"]["DRIVER_EPS_TORQUE"]  # masked in EPS state 10 - kept for logs only
    # OVERRIDE off the REAL driver signal: 0x1FC STEERING_TORQUE.DRIVER_TORQUE (0.1 Nm/bit,
    # already parsed for LKS_PREPARED below), NOT DRIVER_EPS_TORQUE which is masked exactly
    # when openpilot is steering (route 0000015d defect A). steerOverride is ET.OVERRIDE_LATERAL:
    # openpilot yields lateral to the driver and resumes when they relax - it does NOT disengage.
    driver_torque = cp.vl["STEERING_TORQUE"]["DRIVER_TORQUE"]
    ret.steeringPressed = self.update_steering_pressed(abs(driver_torque) > CCP.STEER_DRIVER_TORQUE_OVERRIDE_NM, 5)
    # NO steeringDisengage on this car (the wip does not set it either - it came from
    # the July structural branch and I restored it in the 0.11.2 port by mistake).
    # steerDisengage is NO_ENTRY "Steering Pressed" + an immediate USER_DISABLE, and
    # DRIVER_EPS_TORQUE is not trustworthy here: it is masked/noisy once the EPS is in
    # state 10, so the >30 threshold fired without the driver touching the wheel and
    # blocked engagement outright (in-car 2026-08-02, route 000000f2: 3 steerDisengage
    # events). steeringPressed above still raises the soft lateral-override alert.

    # LKAS_STATE follows the BYD TJA_STATE_* enum: 0=Off, 1=Passive, 2/3=Active, 4=Fault.
    # LKS_PREPARED on the EPS is also 1 at route start before any engagement, so it can't be
    # used directly as a fault flag - the camera-side fault state is the reliable indicator.
    # POC no-MVS4: the camera is not the steering authority - do not let its fault state
    # (LKAS_STATE=4, e.g. the interpose-bench ADAS error) veto engagement. EPS health is
    # observed directly on 0x1FC during the test.
    ret.steerFaultTemporary = (int(cp_cam.vl["LKAS_HUD_ADAS"]["LKAS_STATE"]) == 4) and not self.poc_btn_engage
    # EPS 0x1FC byte0-low-nibble "STATE" 11 = command stream lost while active (latched;
    # needs an idle REQ=0 stream then a fresh STEER_REQ rising edge to re-arm 9->10).
    # Surface a real fault instead of silently not steering with the UI green (drive
    # 2026-07-17: every stop latched 11, lateral dead through move-off). byd_atto3.dbc
    # decodes the nibble as bits, not a STATE enum: 9=PREPARED(+bit3), 10=ACTIVATED(+bit3),
    # 11=PREPARED+ACTIVATED(+bit3) -> both bits set is the latched-dropout signature
    # (bit3 is undecoded and constant-1 in every capture; bench name: byd_general_pt STATE).
    # DEBOUNCED: state 11 (both bits) is TRANSITIONAL, not automatically a fault - it
    # appears for ~100-200 ms whenever the 0x1E2 source changes or briefly gaps, e.g.
    # at the engage handover when the panda stops forwarding the camera's frames.
    # Reporting it instantly was a deadlock: in-car 2026-08-02 route 000000f5, EVERY
    # state-11 onset landed 0.22-0.24 s after an LKAS press (the press blocks the
    # camera before openpilot streams), so steerTempUnavailable NO_ENTRY refused the
    # very engagement that would have started our stream. A REAL latched dropout
    # persists indefinitely, so require ~0.5 s (card runs at 100 Hz).
    lks_prepared = bool(cp.vl["STEERING_TORQUE"]["LKS_PREPARED"])
    lks_activated = bool(cp.vl["STEERING_TORQUE"]["CRUISE_ACTIVATED"])
    self.eps_dropout_frames = self.eps_dropout_frames + 1 if (lks_prepared and lks_activated) else 0
    ret.steerFaultTemporary = ret.steerFaultTemporary or (self.eps_dropout_frames > 50)
    # NOTE: the wip exposed this nibble on carState.gearStep for the drive-review
    # dashboard; 0.11.2's schema has no gearStep, so the 0112 dashboard decodes
    # the state straight from raw CAN 0x1FC in the rlog instead (server_0112).

    # gas / brake
    # gasPressed from the real accelerator pedal (0x342 GAS_PEDAL, reads 0 at rest), NOT
    # DRIVE_STATE.RAW_THROTTLE: that signal is powertrain throttle DEMAND, which pulses 0->28
    # on its own while accelerating (route 00000053: it fired 104x while the pedal read 0.00,
    # falsely suspending OP long ~every 0.5s = the "still jerky" churn). Both signals agree
    # during braking (rt51 22=22); RAW_THROTTLE only lies under acceleration. 0.10 deadband
    # ignores light contact/noise; brake still disengages for safety.
    ret.gasPressed = cp.vl["PEDAL"]["GAS_PEDAL"] > 0.10
    # (0.11.2 removed CarState.brake - the raw pedal float the wip set here; nothing
    # in this stack reads it, brakePressed below is the control-relevant signal)
    # brakePressed MUST read the exact signal the panda latch reads or the two engage
    # latches desync on a light brake graze -> "Controls Mismatch" (routes 5e/5f,
    # 2026-07-17, mono 747.8: a 1-7 count feather touch cleared only the panda latch).
    # Both sides now read DRIVE_STATE's debounced BRAKE_PRESSED bit (0x242 byte4 bit5,
    # car threshold ~9 counts; modes/byd.h moved off PEDAL byte1 != 0 same day, fw
    # flashed via pandad auto-reflash). Keep these two in lockstep forever.
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
      # POC engage: the LKAS button (LKAS_ON_BTN = 0x3B0 byte0 bit6, on-car verified)
      # TOGGLES the engage latch; brake/gas clears it. LKAS chosen over the speed
      # rocker (SET/RES bits) because it doesn't nudge the car's ACC. Mirrors the
      # panda latch in modes/byd.h (both toggle on the same LKAS rising edge).
      prev_latch = self.btn_latch
      lkas_btn = bool(cp.vl["PCM_BUTTONS"]["LKAS_ON_BTN"])
      if lkas_btn and not self.btn_prev and not ret.brakePressed:
        # engage-init of the cruise target is the cruise helper's job (it inits from vEgo)
        self.btn_latch = not self.btn_latch
      if ret.brakePressed:  # only brake disengages; gas must not (driver drives on gas)
        self.btn_latch = False
      self.btn_prev = lkas_btn
      ret.cruiseState.available = True
      ret.cruiseState.enabled = ret.cruiseState.enabled or self.btn_latch
      # Stock engagement wiring. Carrot engaged off the latched cruiseState.enabled
      # even with pcmCruise=False - STOCK does not (pcm_enable gate in
      # car_specific.py): with pcmCruise=False it engages ONLY on buttonEnable and
      # disengages on a cancel buttonEvent. Map the toggle onto exactly that:
      # latch rising -> buttonEnable; latch dropped by the button -> cancel event.
      # Brake disengage needs nothing here (selfdrived pedalPressed + the latch
      # clears above and in the panda). Same on-car behavior as the wip.
      # NOTE: do NOT assign ret.buttonEnable here - interfaces.py overwrites it
      # with update_button_enable() after update() returns, so a direct write is
      # dead (in-car 2026-08-02 route 000000f6: the latch rose on all 27 presses,
      # cancel events came through, but every buttonEnable was erased and the
      # state machine never left 'disabled'). The rising edge is delivered
      # through our update_button_enable() override below instead.
      self.btn_engage_edge = self.btn_latch and not prev_latch
      button_events = []
      if prev_latch and not self.btn_latch and not ret.brakePressed:
        button_events.append(structs.CarState.ButtonEvent(pressed=True, type=ButtonType.cancel))
      if self.op_long and self.btn_latch:
        # Cruise-speed UP/DOWN ROCKER on 0x3B0 -> standard buttonEvents, which is what
        # the cruise helper consumes: it steps on the RELEASE edge and does +10 jumps on a
        # long hold. Rocker RE (btn_map.log): SET_BTN (byte0 bit3) = "rocker pressed"
        # both ways; RES_BTN (bit4) = direction modifier, never set alone. Direction
        # latches one frame after the press edge so a rising-edge skew between the two
        # bits can't misread it (a 1-frame blip is debounced away entirely). Gated on
        # btn_latch so a rocker press can't be taken as an engage request - the panda
        # controls_allowed latch only toggles on the LKAS button.
        rocker = bool(cp.vl["PCM_BUTTONS"]["SET_BTN"])       # bit3 = rocker pressed (either way)
        rocker_up = bool(cp.vl["PCM_BUTTONS"]["RES_BTN"])    # bit4 = direction (TBD on-car)
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
        ret.cruiseState.speed = 8.3  # POC: sane floor when stock ACC is off (SET_SPEED=0)

    # forward stock LKAS HUD
    self.lkas_hud = copy.copy(cp_cam.vl["LKAS_HUD_ADAS"])

    return ret

  def update_button_enable(self, buttonEvents):
    # interfaces.py calls this after update() and overwrites CarState.buttonEnable
    # with the result. On the POC path the engage request is the LKAS-latch rising
    # edge, not a cruise-button falling edge (the base implementation) - and the
    # base version must NOT run while latched either, or a rocker set-speed
    # release (accel/decelCruise falling edge) would read as an engage request.
    if self.poc_btn_engage:
      return self.btn_engage_edge
    return super().update_button_enable(buttonEvents)

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
