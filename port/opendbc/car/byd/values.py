from dataclasses import dataclass, field
from enum import IntFlag, StrEnum

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, structs
from opendbc.car.lateral import AngleSteeringLimitsVM
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
from opendbc.car.vin import Vin

Ecu = structs.CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 2  # Angle command is sent at 50 Hz

  # On a fault STEERING_TORQUE.LKS_PREPARED goes from 0 to 1.
  # STEERING_TORQUE.MAIN_TORQUE is saturated at -300 for around 900ms,
  # while the wheel sits 15-26 deg past the commanded TARGET_ANGLE.
  # MAX_LATERAL_ACCEL/JERK: upstream defaults (~3.6 = ISO + average-banked-road roll
  # tolerance) match the values this port validated on-car.
  ANGLE_LIMITS: AngleSteeringLimitsVM = AngleSteeringLimitsVM(
    390,  # deg
    # limit angle rate to both prevent a fault and for low speed comfort (deg/20ms frame);
    # raised 2->3 for tighter sharp-curve tracking (rt56 hit the 2-cap on the sharpest ~1%,
    # wheel lagged cmd 27-40deg p90 -> cut wide). Stock Veoneer max=4.8 (5 caused 29deg
    # spikes/shaky wheel), so 3 stays within the safe stock range.
    MAX_ANGLE_RATE=3,  # deg/20ms
  )

  STEER_DRIVER_OVERRIDE = 10   # EPS torque threshold for soft override (LEGACY - DRIVER_EPS_TORQUE)
  STEER_DRIVER_DISENGAGE = 30  # EPS torque threshold for hard disengage (LEGACY - DRIVER_EPS_TORQUE)
  # Real driver-torque override threshold, in Nm, on 0x1FC STEERING_TORQUE.DRIVER_TORQUE.
  # steeringPressed keys on THIS, not on DRIVER_EPS_TORQUE, which is masked/noisy once the
  # EPS is in state 10 - the exact window openpilot is steering (route 0000015d: the driver
  # fought a wrong U-turn at 23-29 Nm and DRIVER_EPS_TORQUE stayed 0-34 noise, so no override
  # fired). Measured: DRIVER_TORQUE median 1.3 Nm engaged (18.8 min), so 18 Nm is well clear
  # of hands-on noise (5 override events, all sustained, median 6.3 s) yet catches a real
  # fight instantly. FINDINGS §17 / TASKS #56 defect A.
  STEER_DRIVER_TORQUE_OVERRIDE_NM = 18

  # Longitudinal (LONG_PLAN.md LG2). Comfort envelope, well inside the safety cap
  # (safety modes/byd.h enforces -3.5..+2.0 on 0x32E). ISO 15622 defaults; tune in LG4.
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
  # BYD F195 supplier-software-version payloads are 6 raw bytes (BCD-ish date +
  # revision fields, not ASCII) - see fingerprints.py. Full-match any 6 bytes.
  fw_version_regex=rb"[\x00-\xff]{6}",
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)


DBC = CAR.create_dbc_map()
