import os

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
      safety_param |= 0x1000  # POC no-MVS4 button engage latch (see safety_byd.h)
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd, safety_param)]

    ret.dashcamOnly = False

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 0.4

    # Veoneer private-bus radar (bus 1). Opt-in via env for a supervised radar
    # drive: parked, the Veoneer stops TXing -> no track frames -> radarState
    # would go stale and radarFault (NO_ENTRY) could block engagement, so keep
    # it OFF by default until the not-transmitting case is handled in radard.
    ret.radarUnavailable = os.getenv("USE_VEONEER_RADAR") != "1"
    if not ret.radarUnavailable:
      ret.radarTimeStep = 0.075   # RADAR_TRACK slots repeat ~13.3 Hz
    ret.alphaLongitudinalAvailable = False

    # openpilot longitudinal on the no-MVS4 path (LONG_PLAN.md LG2), env-gated for a
    # supervised bring-up drive - default OFF keeps the lat-only POC unchanged. We
    # inject 0x32E ACC_CMD (stock ACC off); safety enforces the accel bounds.
    ret.openpilotLongitudinalControl = os.getenv("USE_OP_LONG") == "1"
    if ret.openpilotLongitudinalControl:
      ret.longitudinalActuatorDelay = 0.5   # RE'd plant lag: best-fit +550ms (byd_general CM_ 814)
      ret.vEgoStopping = 0.3
      ret.vEgoStarting = 0.3
      ret.stopAccel = -0.5
      ret.startAccel = 1.5  # launch kick (fw cap +2.0); 0.5 was creep-slow off the line (honk feedback 2026-07-18)
      # starting state ON: without it longcontrol goes stopping->pid directly, so the
      # startAccel kick and the STANDSTILL_RESUME pulse (carcontroller resume wiring)
      # never fire - route 00000065 launches: RESUME bit 0 all drive, car moved ~1.0s
      # (worst 1.7s) after the lead. ESC releases the hold lazily without the pulse.
      ret.startingState = True
      # 0x32E ACCEL_CMD is a FEEDFORWARD accel request (the car's IPB closes its own loop with
      # ~0.5s lag). So lean on feedforward (kf=1) with only light PID trim - a high feedback gain
      # on a 0.5s-lag actuator winds the integrator up to the ACCEL_MAX ceiling when the car
      # hasn't responded yet (route 00000053: command ran to +2.0 while kW~0). Gentle gains keep
      # accel tracking the planner's aTarget. Tune up from here once a no-gas drive lands (LG4).
      ret.longitudinalTuning.kpBP = [0.0, 5.0, 35.0]
      ret.longitudinalTuning.kpV = [0.5, 0.4, 0.3]
      ret.longitudinalTuning.kiBP = [0.0, 35.0]
      ret.longitudinalTuning.kiV = [0.03, 0.02]

    return ret
