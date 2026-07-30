from dataclasses import dataclass, field
from enum import IntFlag, StrEnum

from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, structs
from opendbc.car.lateral import AngleSteeringLimits, ISO_LATERAL_ACCEL
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
from opendbc.car.vin import Vin

Ecu = structs.CarParams.Ecu


# Add extra tolerance for average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation. higher actual roll lowers lateral acceleration


class CarControllerParams:
  STEER_STEP = 2  # Angle command is sent at 50 Hz

  # On a fault STEERING_TORQUE.LKS_PREPARED goes from 0 to 1.
  # STEERING_TORQUE.MAIN_TORQUE is saturated at -300 for around 900ms,
  # while the wheel sits 15-26 deg past the commanded TARGET_ANGLE.
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    390,  # deg
    # BYD uses a vehicle model instead, check carcontroller.py for details
    ([], []),
    ([], []),

    # Vehicle model angle limits
    # Add extra tolerance for average banked road since safety doesn't have the roll
    MAX_LATERAL_ACCEL=ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),  # ~3.6 m/s^2
    MAX_LATERAL_JERK=3.0 + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),  # ~3.6 m/s^3
  )
  # limit angle rate to both prevent a fault and for low speed comfort (deg/20ms frame);
  # carrot's AngleSteeringLimits has no such field - passed to apply_vm_steer_angle_limits instead
  MAX_ANGLE_RATE = 3  # deg/20ms; raised 2->3 for tighter sharp-curve tracking (rt56 hit the 2-cap on
                     # the sharpest ~1%, wheel lagged cmd 27-40deg p90 -> cut wide). Stock Veoneer max=4.8
                     # (5 caused 29deg spikes/shaky wheel), so 3 stays within the safe stock range.

  STEER_DRIVER_OVERRIDE = 10   # EPS torque threshold for soft override
  STEER_DRIVER_DISENGAGE = 30  # EPS torque threshold for hard disengage

  # Longitudinal (LONG_PLAN.md LG2). Comfort envelope, well inside the safety cap
  # (safety_byd.h enforces -3.5..+2.0 on 0x32E). ISO 15622 defaults; tune in LG4.
  # 0x32E ACCEL_CMD field itself resolves to 0.05 m/s^2/LSB over -5..+7.75.
  ACCEL_MIN = -3.0  # m/s^2  (stock camera comfort-braked to -2.55 max on tape)
  ACCEL_MAX = 1.5   # m/s^2  (comfort cap; was 2.0 - the PID wound to +2.0 on rt 53. tune up in LG4)
  # LG4 jerk limiter on the accel command (m/s^3). The planner already jerk-limits its
  # output; this only bites on transition steps (re-engage after a gas override). Braking
  # allowed to build ~2x faster than accel for response. ISO 15622 comfort ~2.5. Tune LG4.
  JERK_UP = 2.5     # accel increasing (toward accel / less braking)
  JERK_UP_LAUNCH = 4.0  # pull-away only (vEgo < 2, cmd > 0): beat the ~0.5s IPB lag off the line
  JERK_DOWN = 5.0   # accel decreasing (toward braking)


class BydSafetyFlags(IntFlag):
  LONG_CONTROL = 1


class WMI(StrEnum):
  BYD_AUTO = "LGX"  # BYD Auto Co., Ltd. (Shenzhen)


class ModelYear(StrEnum):
  N_2022 = "N"
  P_2023 = "P"
  R_2024 = "R"
  S_2025 = "S"


@dataclass
class BydCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class BydPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'byd_atto3',
  })
  wmis: set[WMI] = field(default_factory=set)
  years: set[ModelYear] = field(default_factory=set)


class CAR(Platforms):
  BYD_ATTO_3 = BydPlatformConfig(
    [BydCarDocs("BYD Atto 3 2022-25")],
    CarSpecs(mass=1750, wheelbase=2.72, steerRatio=19.8),
    wmis={WMI.BYD_AUTO},
    years={ModelYear.N_2022, ModelYear.P_2023, ModelYear.R_2024, ModelYear.S_2025},
  )


def match_fw_to_car_fuzzy(live_fw_versions, vin, offline_fw_versions) -> set[str]:
  # BYD Atto 3 VIN: LGX (WMI) + <VDS> + <year><plant><seq> (VIS).
  # TODO: currently we only match on WMI + model year
  vin_obj = Vin(vin)
  year = vin_obj.vis[:1]

  candidates = set()
  for platform in CAR:
    if vin_obj.wmi in platform.config.wmis and year in platform.config.years:
      candidates.add(platform)

  return {str(c) for c in candidates}


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # BYD ECUs don't answer MANUFACTURER_SOFTWARE_VERSION (0xF188, NRC on all 24
    # bus-0 ECUs of a real Atto 3); they do answer SUPPLIER_SOFTWARE_VERSION (0xF195)
    Request(
      [StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
  # the MPC camera answers OBD DTC scans but not the bus-0 DID sweep, so its FW may be missing
  non_essential_ecus={Ecu.fwdCamera: [CAR.BYD_ATTO_3]},
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)


DBC = CAR.create_dbc_map()
