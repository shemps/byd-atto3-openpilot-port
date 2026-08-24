def byd_checksum(address: int, sig, d: bytearray) -> int:
  return (~sum(d[:7])) & 0xFF


def create_steering_control(packer, apply_angle: float, lat_active: bool, counter: int):
  # Stock saturates the rate limits at ±299 when engaged, 0 when disengaged
  rate_limit = 299 if lat_active else 0
  values = {
    "STEER_REQ": 1 if lat_active else 0,
    "STEER_REQ_ACTIVE_LOW": 0 if lat_active else 1,
    "STEER_ANGLE": apply_angle,
    "ANGLE_RATE_LIMIT_UPPER": 251 if lat_active else 0,
    "ANGLE_RATE_LIMIT_LOWER": -252 if lat_active else 0,
    "E2E_ALIVE_1": 1,
    "E2E_ALIVE_2": 1,
    "SET_ME_FF": 0xFF,
    "SET_ME_F": 0xF,
    "COUNTER": counter,
  }
  return packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)


def create_acc_cmd(packer, accel: float, long_active: bool, counter: int,
                   standstill: bool = False, resume: bool = False):
  # 0x32E ACC_CMD for openpilot longitudinal control. It reproduces the "actively
  # commanding" bit pattern of the stock camera: ON1, ON2 and CTRL = 1,
  # REQ_NOT_STANDSTILL = 1, CMD_REQ_ACTIVE_LOW = 0.
  # While not longActive, send the disengaged pattern. The safety forward hook then
  # forwards the camera ACC_CMD, because the block acts only while CONTROLLABLE_AND_ON = 1.
  # standstill holds the car at a full stop. The stock sequence clears REQ_NOT_STANDSTILL
  # and sets OVERRIDE_OR_STANDSTILL and STANDSTILL_STATE.
  # resume sends a STANDSTILL_RESUME pulse to pull away from the hold.
  # ACCEL_FACTOR and DECEL_FACTOR are a pair. They move with ACCEL_CMD and tell the IPB
  # which gain profile to use: coast (0,0), soft accel (12,5), soft decel (13,1), and
  # sustained hard brake (1,1). Measurement on the car gives a delivered-to-commanded
  # ratio of 1.02 to 1.12 across -0.25 to -1.25. A survey of 78.9k stock-ACC frames shows
  # (1,1) as the stock pair for every bin from -1.75 to -4.0. Do not use the pair (1,2).
  # It is only a transition pair in stock, and it causes a brake transient at regime entry.
  holding = long_active and standstill and not resume
  if not long_active or abs(accel) < 0.1:
    accel_fac, decel_fac = 0, 0
  elif accel > 0:
    accel_fac, decel_fac = 12, 5
  elif accel > -1.5:
    accel_fac, decel_fac = 13, 1
  else:
    accel_fac, decel_fac = 1, 1
  values = {
    "ACCEL_CMD": accel if long_active else 0.0,
    "ACC_ON_1": 1 if long_active else 0,
    "ACC_ON_2": 1 if long_active else 0,
    "ACC_CONTROLLABLE_AND_ON": 1 if long_active else 0,
    "ACC_REQ_NOT_STANDSTILL": 0 if holding else (1 if long_active else 0),
    "CMD_REQ_ACTIVE_LOW": 0 if long_active else 1,
    "ACC_OVERRIDE_OR_STANDSTILL": 1 if holding else 0,
    "STANDSTILL_RESUME": 1 if (long_active and resume) else 0,
    "STANDSTILL_STATE": 1 if holding else 0,
    "ACCEL_FACTOR": accel_fac,
    "DECEL_FACTOR": decel_fac,
    "SET_ME_25_1": 25,
    "SET_ME_25_2": 25,
    "SET_ME_1": 1,
    "SET_ME_X8": 8,
    "SET_ME_XF": 15,
    "COUNTER": counter,
  }
  return packer.make_can_msg("ACC_CMD", 0, values)


def create_lkas_hud(packer, lat_active: bool, counter: int, stock_lkas_hud: dict, hud_control):
  # 0x316 is a validated safety frame: the ADAS modules cross-check its exact bit
  # pattern every frame and fail-safe on any mismatch (AEB/ACC/LKS/LDWS "limited" +
  # latched DTCs across 8 modules, measured on a 2024 Atto 3 MVS4). Pass every stock
  # bit through untouched - including HANDS_ON_WHEEL_REQ, the camera's hands-on nag.
  values = {**stock_lkas_hud, "COUNTER": counter}
  if lat_active:
    # Assert ONLY the on-car-proven active bits (pattern a000ff1f29), preserving every
    # other stock bit inside these multi-bit fields:
    # - bit 37 set / bit 36 cleared (low 2 bits of LKAS_STATE): the EPS-arming pair;
    #   without them the EPS sees "LKAS disabled" and refuses to actuate. Bits 38-39
    #   (stock state, e.g. fault flags) pass through.
    # - bits 5/35 (high bit of LANE_STATE): lane bits the stock camera sets when it
    #   steers (this DBC's GREEN=1/ORANGE=2 labels don't match this car). Bits 4/34
    #   pass through. No other bit may change until proven on-car.
    values["LKAS_STATE"] = (int(stock_lkas_hud["LKAS_STATE"]) & 0b1100) | 0b0010
    values["LEFT_LANE_STATE"] = int(stock_lkas_hud["LEFT_LANE_STATE"]) | 2
    values["RIGHT_LANE_STATE"] = int(stock_lkas_hud["RIGHT_LANE_STATE"]) | 2

  return packer.make_can_msg("LKAS_HUD_ADAS", 0, values)
