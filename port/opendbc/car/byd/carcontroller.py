import os

import numpy as np

from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_vm_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd import bydcan
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel

LongCtrlState = structs.CarControl.Actuators.LongControlState

# LG3 acceptance probe: a FIXED accel (m/s^2) that bypasses the longitudinal planner so
# the very first on-car longitudinal test commands a deterministic value instead of the
# planner chasing the set-speed. Tightly clamped to +-1.0 (a probe range well inside the
# safety cap), and a typo falls back to 0.0 = coast (never a surprise accel). Unset = the
# planner drives (LG4). See LONG_PLAN.md LG3.
def _long_test_accel():
  v = os.getenv("POC_LONG_TEST_ACCEL")
  if v is None:
    return None
  try:
    a = float(v)
  except ValueError:
    print(f"byd carcontroller: bad POC_LONG_TEST_ACCEL={v!r} -> 0.0 (coast)")
    return 0.0
  return float(np.clip(a, -1.0, 1.0))


def get_safety_CP():
  from opendbc.car.byd.interface import CarInterface
  return CarInterface.get_non_essential_params("BYD_ATTO_3")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.apply_angle_last = 0.0
    self.accel_last = 0.0
    self.long_test_accel = _long_test_accel()
    if self.long_test_accel is not None:
      print(f"byd carcontroller: LG3 fixed-accel probe active, accel={self.long_test_accel:+.2f} m/s^2 (planner bypassed)")

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    hud_control = CC.hudControl

    if self.frame % 2:
      self.apply_angle_last = apply_vm_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                          CS.out.steeringAngleDeg, CC.latActive, CarControllerParams, self.VM,
                                                          MAX_ANGLE_RATE=CarControllerParams.MAX_ANGLE_RATE)

      # The validated carrot-era safety FORWARDS the camera's own 0x1E2/0x316 to the car
      # while disengaged (factory LKS stays healthy) and blocks them only when engaged.
      # Send our regenerated frames ONLY when engaged, or both sources collide on the bus.
      #
      # "Engaged" must be CC.enabled, NOT CC.latActive: at a stop latActive drops while
      # controls_allowed stays true, so gating on latActive made the 0x1E2 stream VANISH
      # mid-assist (panda's disengaged-synth only runs when !controls_allowed). The EPS
      # then latches STATE=11 + dash beep and stays dead through the move-off - REQ=1
      # alone does not recover it; it re-arms 9->10 only off a REQ rising edge on a
      # continuous stream (route 5e 2026-07-17: stop mono 599-628 = 0 tx/s, 10->11 at
      # 598.2, 7 s of REQ=1 creep did NOT recover it). So while enabled-but-lat-inactive
      # we stream idle STEER_REQ=0 frames (always allowed by the safety TX hook), which
      # ends assist gracefully (10->9) and gives move-off its rising edge.
      if CC.enabled or CC.latActive:
        cntr = (self.frame // 2) % 16
        can_sends.append(bydcan.create_steering_control(self.packer, self.apply_angle_last, CC.latActive, cntr))
        can_sends.append(bydcan.create_lkas_hud(self.packer, CC.latActive, cntr, CS.lkas_hud, hud_control))

    # Longitudinal: inject 0x32E ACC_CMD at ~33 Hz (stock rate) ONLY when longActive.
    # Mirrors the lateral passthrough design exactly: send our frame only while engaged
    # (the safety then blocks the camera's own ACC_CMD, except AEB); when disengaged we
    # send nothing and the safety forwards the camera's ACC_CMD (its `!controls_allowed`
    # clause). Sending a disengaged frame here would collide with that forwarded frame.
    accel = 0.0
    if self.CP.openpilotLongitudinalControl and CC.longActive and self.frame % 3 == 0:
      # target: planner accel (or the fixed probe knob if set)
      target = self.long_test_accel if self.long_test_accel is not None else actuators.accel
      # LG4 jerk limiter: ease the command toward target so re-engagement after a gas
      # override (the +0.2 jerk, SESSION section 18) and any planner step ramp in rather
      # than jump. Braking is allowed to build faster than accel for response/safety.
      dt = 0.03  # 33 Hz command
      # pull-away carve-out: faster up-ramp below walking pace so the start kick beats
      # the IPB lag off the line (honk feedback 2026-07-18); JERK_UP resumes above 2 m/s.
      launch = CS.out.vEgo < 2.0 and target > 0.0
      up = (CarControllerParams.JERK_UP_LAUNCH if launch else CarControllerParams.JERK_UP) * dt
      down = CarControllerParams.JERK_DOWN * dt
      target = float(np.clip(target, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      # stop-and-go: hold at a full stop; resume/starting pulls away
      lcs = actuators.longControlState
      stopping = (lcs == LongCtrlState.stopping) or (CS.out.standstill and target <= 0.0)
      resume = (lcs == LongCtrlState.starting) or CC.cruiseControl.resume
      # launch: the hold parks the ramp at the stopping brake (~-0.9); climbing back through
      # the negative band at JERK_UP costs ~0.4s while the car is stationary and ESC-held
      # (no physical jerk to limit). Snap the anchor to 0 on resume so the start kick
      # applies immediately (route 00000065: cmd->move ~1.0s median, worst 1.7s).
      if resume and CS.out.standstill and self.accel_last < 0.0:
        self.accel_last = 0.0
      accel = float(np.clip(target, self.accel_last - down, self.accel_last + up))
      can_sends.append(bydcan.create_acc_cmd(self.packer, accel, True, (self.frame // 3) % 16,
                                             standstill=stopping and CS.out.standstill, resume=resume))
      self.accel_last = accel
    elif self.frame % 3 == 0:
      # not commanding: keep the ramp anchored to the car's real accel so the first
      # frame after (re)engage eases in from reality, not a stale value.
      self.accel_last = float(np.clip(CS.out.aEgo, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

    # No button TX (SAFETY_REVIEW M1): the old cancel spam went to bus 0 while the
    # allowlist only ever permitted bus 2, so it was 100% blocked - and spoofing wheel
    # buttons at the camera is unproven on this car. In no-MVS4 mode there is no stock
    # ACC to cancel; disengage = stop sending commanding frames (the fwd hook then
    # forwards the camera's own ACC_CMD again). Bench-prove a real button spoof first
    # if a stock-ACC-active configuration ever returns.

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = float(self.apply_angle_last)
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends
