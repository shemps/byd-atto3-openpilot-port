import os

from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.radar_interface import RadarInterface


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "byd"

    safety_param = 0x10
    if os.getenv("POC_BTN_ENGAGE") == "1":
      safety_param |= 0x1000  # POC no-MVS4 button engage latch (see modes/byd.h)
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd, safety_param)]

    ret.dashcamOnly = False

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 0.4

    # The stock ACC is NOT the authority on this port: openpilot engages off the
    # LKAS-button latch and owns the set speed. Leaving the base default
    # (pcmCruise=True) made cruise.py read the set speed from the car instead of
    # the rocker - and since carstate floors cruiseState.speed at 8.3 m/s when the
    # stock ACC is off, the UI sat at "MAX 30" and the rocker did nothing (the
    # non-pcm button path is only called when pcmCruise is False). In-car
    # 2026-08-02. carrot hid this behind its own speed_from_pcm knob, which is why
    # the wip worked with the base default.
    ret.pcmCruise = not (os.getenv("POC_BTN_ENGAGE") == "1")

    # Veoneer private-bus radar (bus 1). Opt-in via env for a supervised radar
    # drive: parked, the Veoneer stops TXing -> no track frames -> radarState
    # would go stale and radarFault (NO_ENTRY) could block engagement, so keep
    # it OFF by default until the not-transmitting case is handled in radard.
    # (RADAR_TRACK slots repeat ~13.3 Hz; 0.11.2 dropped CarParams.radarTimeStep,
    # radard now derives its own rate.)
    ret.radarUnavailable = os.getenv("USE_VEONEER_RADAR") != "1"
    ret.alphaLongitudinalAvailable = False

    # openpilot longitudinal on the no-MVS4 path (LONG_PLAN.md LG2), env-gated for a
    # supervised bring-up drive - default OFF keeps the lat-only POC unchanged. We
    # inject 0x32E ACC_CMD (stock ACC off); safety enforces the accel bounds.
    ret.openpilotLongitudinalControl = os.getenv("USE_OP_LONG") == "1"
    if ret.openpilotLongitudinalControl:
      ret.longitudinalActuatorDelay = 0.5   # RE'd plant lag: best-fit +550ms (byd_general CM_ 814)
      ret.stopAccel = -0.5
      # NOTE vs the wip/carrot port: 0.11.2 deprecated vEgoStopping/vEgoStarting/
      # startAccel/startingState (longcontrol never enters `starting` anymore, it goes
      # stopping->pid). The startAccel launch kick is gone with it; move-off relies on
      # CC.cruiseControl.resume + the carcontroller launch snap-to-zero + JERK_UP_LAUNCH.
      # If launches feel lazy on this stack, that is where to look first.
      # 0x32E ACCEL_CMD is a FEEDFORWARD accel request (the car's IPB closes its own loop with
      # ~0.5s lag). So lean on feedforward (kf=1) with only light PID trim - a high feedback gain
      # on a 0.5s-lag actuator winds the integrator up to the ACCEL_MAX ceiling when the car
      # hasn't responded yet (route 00000053: command ran to +2.0 while kW~0). Gentle gains keep
      # accel tracking the planner's aTarget. Tune up from here once a no-gas drive lands (LG4).
      #
      # NO P TERM: upstream deprecated longitudinalTuning.kpBP/kpV (opendbc #3681,
      # openpilot #38614) - longcontrol now builds its PID with kp = 0 for every car.
      # This continues the direction of our own 2026-08-02 gain cut (0.5/0.4/0.3 ->
      # 0.2/0.15/0.12): 0.11.2's longcontrol closes on `a_target - CS.aEgo`, our wheel-
      # speed-derived aEgo carries p95 jerk ~15 m/s^3 of noise, and kp turned that noise
      # into command churn the driver felt as "always correcting itself" (routes f9/fb;
      # the planner target itself was smooth at p95 1.1). The IPB closes its own loop on
      # the commanded accel with ~0.5 s lag, so kf = 1 does the work and ki trims the
      # remaining IPB model error. Watch steady-state accel error on the next long drive.
      ret.longitudinalTuning.kiBP = [0.0, 35.0]
      ret.longitudinalTuning.kiV = [0.01, 0.01]

    return ret
