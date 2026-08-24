import os

import numpy as np

from opendbc.can.packer import CANPacker
from opendbc.car import Bus, rate_limit, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd import bydcan
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel

LongCtrlState = structs.CarControl.Actuators.LongControlState


def eps_angle_cap(v_ego_raw: float) -> float:
  """Largest |wheel angle| in deg the EPS holds in lane keeping at this speed.

  See CarControllerParams.EPS_YAW_LIMIT_DEG_KMH. Never returns more than
  STEER_ANGLE_MAX, and is finite at a standstill.
  """
  v_kmh = max(v_ego_raw * CV.MS_TO_KPH, 1.0)
  return min(CarControllerParams.EPS_YAW_LIMIT_DEG_KMH / v_kmh,
             CarControllerParams.ANGLE_LIMITS.STEER_ANGLE_MAX)

# Acceptance probe. It sets a fixed accel in m/s^2 and bypasses the longitudinal planner,
# so a first on-car longitudinal test commands a known value. The value is clamped to
# +-1.0, which is well inside the safety limit. A bad value gives 0.0, which is coast, and
# never an unwanted accel. If the variable is not set, the planner controls the accel.
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
      new_angle = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                              CS.out.steeringAngleDeg, CC.latActive, CarControllerParams, self.VM)
      # Low-speed comfort slew limit, applied after the limits above. Neither of those
      # limits acts at low speed: MAX_ANGLE_RATE is a constant 3 deg/20 ms, which is a
      # fault limit, and the ISO lateral-jerk limit scales with 1/v^2, about 8.6 deg per
      # frame at 5 m/s. 0.11.2 sends the model curvature direct to the wheel, so model
      # noise reaches the wheel without a filter. 2.5 deg/20 ms, 125 deg/s, keeps a margin
      # below the constant 3.0 and still permits a tight turn. A tighter value makes the
      # car run wide, because the wheel cannot turn fast enough.
      # This limit is a mitigation. The correct fix is a low-pass filter on the model
      # curvature, because a rate limit cannot separate noise from a true turn.
      # Apply it only while lateral control is active. The inactive path must follow the
      # measured angle for the handover.
      if CC.latActive:
        max_rate = float(np.interp(CS.out.vEgoRaw, [3.0, 10.0], [2.5, 3.0]))
        new_angle = rate_limit(new_angle, self.apply_angle_last, -max_rate, max_rate)
        # EPS authority clamp. Do not command an angle the EPS will not hold. Past its
        # limit the EPS drops lane keeping mid-corner and hands the wheel back with the
        # command tens of degrees past the angle it refused. This clip only ever reduces
        # the command, so the panda angle-rate and angle-error checks cannot newly fail.
        # When it binds, the car runs wide and openpilot raises steerSaturated. It
        # declines the turn instead of losing assist in it. It does not make the turn:
        # that is a planner change, not this clamp.
        cap = eps_angle_cap(CS.out.vEgoRaw)
        new_angle = float(np.clip(new_angle, -cap, cap))
      self.apply_angle_last = new_angle

      # The passthrough safety forwards the camera 0x1E2 and 0x316 to the car while
      # openpilot is disengaged, which keeps the factory LKS healthy. It blocks them only
      # while openpilot is engaged. Send the openpilot frames only while engaged, or the
      # two sources collide on the bus.
      #
      # Use CC.enabled here, not CC.latActive. At a stop latActive goes false while
      # controls_allowed stays true. With latActive as the gate the 0x1E2 stream stops
      # during assist, because the panda synthesizes frames only when controls_allowed is
      # false. The EPS then latches state 11 and stays dead through the move-off. REQ=1
      # alone does not recover it. The EPS re-arms 9->10 only on a REQ rising edge in a
      # continuous stream. While enabled but lateral-inactive, send idle STEER_REQ=0
      # frames. The safety TX hook always permits them, they end assist correctly with
      # 10->9, and they give the move-off its rising edge.
      if CC.enabled or CC.latActive:
        cntr = (self.frame // 2) % 16
        can_sends.append(bydcan.create_steering_control(self.packer, self.apply_angle_last, CC.latActive, cntr))
        can_sends.append(bydcan.create_lkas_hud(self.packer, CC.latActive, cntr, CS.lkas_hud, hud_control))

    # Longitudinal control sends 0x32E ACC_CMD at about 33 Hz, the stock rate, only while
    # longActive. This follows the lateral passthrough design. Send the openpilot frame only
    # while engaged. The safety then blocks the camera ACC_CMD, except AEB. While disengaged
    # send nothing, and the safety forwards the camera ACC_CMD. A frame sent while disengaged
    # collides with that forwarded frame.
    accel = 0.0
    if self.CP.openpilotLongitudinalControl and CC.longActive and self.frame % 3 == 0:
      # The target is the planner accel, or the fixed probe value if it is set.
      target = self.long_test_accel if self.long_test_accel is not None else actuators.accel
      # The jerk limiter moves the command toward the target, so that a re-engage after a
      # gas override and any planner step ramp instead of step. Braking builds faster than
      # acceleration, for response and safety.
      dt = 0.03  # 33 Hz command
      # Pull-away uses a faster up-ramp below walking pace, so the start covers the IPB lag.
      # JERK_UP applies again above 2 m/s.
      launch = CS.out.vEgo < 2.0 and target > 0.0
      up = (CarControllerParams.JERK_UP_LAUNCH if launch else CarControllerParams.JERK_UP) * dt
      down = CarControllerParams.JERK_DOWN * dt
      target = float(np.clip(target, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      # Stop and go: hold at a full stop, and pull away on resume or starting.
      lcs = actuators.longControlState
      stopping = (lcs == LongCtrlState.stopping) or (CS.out.standstill and target <= 0.0)
      resume = (lcs == LongCtrlState.starting) or CC.cruiseControl.resume
      # The hold leaves the ramp at the stopping brake value, about -0.9. To climb back
      # through the negative band at JERK_UP takes about 0.4 s while the car is stationary
      # and the ESC holds it, where there is no physical jerk to limit. Set the anchor to 0
      # on resume, so that the start applies immediately.
      if resume and CS.out.standstill and self.accel_last < 0.0:
        self.accel_last = 0.0
      accel = float(np.clip(target, self.accel_last - down, self.accel_last + up))
      # Hysteresis on the ACC_CMD value. The PID closes on wheel-speed aEgo noise, so the
      # command carries 0.05 to 0.1 m/s^2 of churn. The wire cannot express it, because the
      # ACCEL_CMD scale is 0.05 m/s^2 per LSB, and the drivetrain does not respond to it.
      # Hold the last sent value for a change below one LSB. A true demand passes the band
      # on its first sample, so there is no cost in response. Apply this above walking pace
      # only, so that the stop, the launch and the stop bite do not change.
      if CS.out.vEgo > 2.0 and abs(accel - self.accel_last) < 0.04:
        accel = self.accel_last
      can_sends.append(bydcan.create_acc_cmd(self.packer, accel, True, (self.frame // 3) % 16,
                                             standstill=stopping and CS.out.standstill, resume=resume))
      self.accel_last = accel
    elif self.frame % 3 == 0:
      # While not commanding, keep the ramp at the true accel of the car. The first frame
      # after an engage then starts from the true value and not from an old value.
      self.accel_last = float(np.clip(CS.out.aEgo, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

    # This port transmits no button frames. Without the MVS4 there is no stock ACC to
    # cancel. To disengage, stop to send the commanding frames. The forward hook then
    # forwards the camera ACC_CMD again. A button spoof at the camera is not proven on this
    # car. Test one on a bench first if a configuration with the stock ACC active is needed.

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = float(self.apply_angle_last)
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends
