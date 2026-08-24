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

  # MAX_LATERAL_ACCEL and MAX_LATERAL_JERK use the upstream defaults, about 3.6, which is
  # the ISO value plus an allowance for average road bank. Note that the EPS itself stops
  # lane keeping near 1.3 to 2.5 m/s^2 of lateral acceleration, so this limit is higher than
  # the car delivers. See "EPS limits" in the top-level README.
  ANGLE_LIMITS: AngleSteeringLimitsVM = AngleSteeringLimitsVM(
    390,  # deg
    # Angle-rate limit per 20 ms frame. It prevents an EPS fault and gives low-speed comfort.
    # The stock Veoneer camera uses a maximum of 4.8. A value of 5 gives 29 deg spikes and an
    # unsteady wheel, so 3 stays inside the stock range.
    MAX_ANGLE_RATE=3,  # deg/20ms
  )

  # Thresholds on DRIVER_EPS_TORQUE. That signal is masked while the EPS steers, so these
  # two are not used for override. Keep them only for compatibility.
  STEER_DRIVER_OVERRIDE = 10
  STEER_DRIVER_DISENGAGE = 30
  # Driver-torque override threshold, in Nm, on 0x1FC STEERING_TORQUE.DRIVER_TORQUE.
  # steeringPressed uses this signal. DRIVER_TORQUE has a median of 1.3 Nm while openpilot
  # steers, so 18 Nm is clear of hands-on noise and still detects a sustained fight.
  STEER_DRIVER_TORQUE_OVERRIDE_NM = 18

  # Longitudinal comfort envelope. It stays inside the safety limit, because modes/byd.h
  # enforces -3.5 to +2.0 on 0x32E. The 0x32E ACCEL_CMD field resolves to 0.05 m/s^2 per LSB
  # over -5 to +7.75. The values are the ISO 15622 defaults.
  ACCEL_MIN = -3.0  # m/s^2. The stock camera brakes to -2.55 maximum.
  ACCEL_MAX = 1.5   # m/s^2. Comfort limit.
  # Jerk limiter on the accel command, in m/s^3. The planner limits its own jerk, so this
  # acts only on a step change, such as a re-engage after a gas override. Braking builds
  # about two times faster than acceleration, for response. ISO 15622 comfort is about 2.5.
  JERK_UP = 2.5     # acceleration increases, toward accel or less braking
  JERK_UP_LAUNCH = 4.0  # pull-away only, vEgo < 2 and cmd > 0. It covers the 0.5 s IPB lag.
  JERK_DOWN = 5.0   # acceleration decreases, toward braking


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
