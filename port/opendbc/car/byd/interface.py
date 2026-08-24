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

    # The stock ACC is not the authority on this port. openpilot engages on the LKAS-button
    # latch and owns the set speed, so pcmCruise must be False. With pcmCruise True, cruise.py
    # reads the set speed from the car and not from the rocker, and the non-pcm button path
    # never runs. The UI then shows the carstate floor of 8.3 m/s and the rocker has no effect.
    ret.pcmCruise = not (os.getenv("POC_BTN_ENGAGE") == "1")

    # Veoneer radar on the private bus, panda bus 1. The environment variable enables it for a
    # supervised drive. Keep it off by default: when the car is parked the Veoneer stops to
    # transmit, radarState becomes stale, and the radarFault NO_ENTRY can prevent engagement.
    # Enable it after radard handles the no-transmission case.
    # The RADAR_TRACK slots repeat at about 13.3 Hz. 0.11.2 has no CarParams.radarTimeStep,
    # because radard calculates its own rate.
    ret.radarUnavailable = os.getenv("USE_VEONEER_RADAR") != "1"
    ret.alphaLongitudinalAvailable = False

    # openpilot longitudinal control. The environment variable enables it for a supervised
    # drive. Default off keeps the port lateral only. openpilot sends 0x32E ACC_CMD with the
    # stock ACC off, and the panda safety enforces the accel limits.
    ret.openpilotLongitudinalControl = os.getenv("USE_OP_LONG") == "1"
    if ret.openpilotLongitudinalControl:
      ret.longitudinalActuatorDelay = 0.5   # measured plant lag, best fit +550 ms
      ret.stopAccel = -0.5
      # 0.11.2 has no vEgoStopping, vEgoStarting, startAccel or startingState. longcontrol
      # goes from stopping to pid and never enters a starting state, so there is no launch
      # kick. Move-off uses CC.cruiseControl.resume, the launch snap-to-zero in the car
      # controller, and JERK_UP_LAUNCH. Examine those three first if move-off is slow.
      #
      # 0x32E ACCEL_CMD is a feedforward accel request. The car IPB closes its own loop with
      # about 0.5 s of lag. Use feedforward, kf = 1, with a light PID trim. A high feedback
      # gain on an actuator with 0.5 s of lag winds the integrator to the ACCEL_MAX limit
      # before the car responds.
      #
      # There is no P term. 0.11.2 has no longitudinalTuning.kpBP or kpV, and longcontrol
      # builds its PID with kp = 0 for every car. longcontrol closes on a_target minus
      # CS.aEgo. On this car aEgo comes from wheel speed and carries about 15 m/s^3 of p95
      # jerk noise, which a P term turns into command churn. ki trims the remaining IPB
      # model error. Check the steady-state accel error on a long drive.
      ret.longitudinalTuning.kiBP = [0.0, 35.0]
      ret.longitudinalTuning.kiV = [0.01, 0.01]

    return ret
