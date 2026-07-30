#!/usr/bin/env python3
"""CAN->IMU bridge: publish the Atto 3's yaw-rate + accel sensors as openpilot
`gyroscope`/`accelerometer` SensorEventData, substituting for a device IMU
(a PC has none -> sensord silent -> locationd/livePose degraded).

Car sensors on the chassis bus:
  0x222 YAW_RATE: YawRate 0|12 *0.002133 -2.094 rad/s (minus YawRateOffset 12|12)
  0x223 AXAY:     Ax      0|12 *0.027167 -21.593 m/s^2 (minus AxOffset 12|12)
                  AY     24|12 *0.027127 -21.593 m/s^2 (minus AYOffset 36|12)

locationd contract (selfdrive/locationd/locationd.py handle_log):
  meas = [-v[2], -v[1], -v[0]]  -> we pack v = [-meas2, -meas1, -meas0] so that
  meas comes out in device frame (x fwd, y right, z down). Accel must include
  gravity (az ~= -9.81 at rest, specific force, z-down). Source must not be bmx055.

Run inside the openpilot venv, after pandad:  python can_imu_bridge.py
Check the signs first:                        python can_imu_bridge.py --print
  stationary: yaw~0, ax~0, ay~0 | accelerate: ax>0 | LEFT turn: yaw<0 (z-down)
  Flip the *_SIGN knobs below if a check fails. locationd's own gyro-vs-camera-
  odometry cross-check rejects a wrong yaw sign (livePose stays invalid).

There is no vertical sensor on these messages, so az is a constant. Roll and
pitch are therefore weakly observable; yaw and the planar dynamics are not.
"""
import argparse
try:
  import cereal.messaging as messaging
  from cereal import log
except ImportError:  # no cereal outside the openpilot venv
  messaging = log = None

# Sign knobs, kept as calibration points rather than constants - check them against
# your own drive log before you drive on them (yaw against steering angle, ax against
# d(speed)/dt, ay against v*yaw). On the car these came from, the car frame is ISO
# (yaw and AY positive = left), so AY flips for a device y-axis pointing right.
YAW_SIGN = 1.0   # +1 if the car reports a left turn positive (z-up); bridge negates to z-down
AX_SIGN = 1.0    # +1 if Ax is positive when accelerating forward
AY_SIGN = -1.0   # car AY positive-LEFT (ISO); device y = right -> flip
GRAVITY = 9.81

YAW_RATE_ADDR = 0x222
AXAY_ADDR = 0x223
BUS = 0  # chassis


def u12(dat: bytes, start_bit: int) -> int:
  # little-endian 12-bit unsigned field at bit offset (byte-aligned nibble packing)
  byte, shift = divmod(start_bit, 8)
  raw = int.from_bytes(dat[byte:byte + 3], 'little')
  return (raw >> shift) & 0xFFF


def decode_yaw(dat: bytes) -> float:
  yaw = u12(dat, 0) * 0.002133 - 2.094
  off = u12(dat, 12) * 0.002133 - 0.13
  return yaw - off  # rad/s, car convention


def decode_axay(dat: bytes) -> tuple[float, float]:
  ax = (u12(dat, 0) - u12(dat, 12)) * 0.027167   # offsets share scale+bias -> subtract raw
  ay = (u12(dat, 24) - u12(dat, 36)) * 0.027127
  return ax, ay  # m/s^2, car convention


def sensor_msg(which: str, union_field: str, vec: list[float], ts_ns: int):
  msg = messaging.new_message(which)
  msg.valid = True
  ev = getattr(msg, which)
  ev.source = log.SensorEventData.SensorSource.velodyne  # any source but bmx055 passes
  ev.timestamp = ts_ns
  ev.init(union_field).v = vec  # init() selects the union member, then fill SensorVec.v
  return msg


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--print', action='store_true', dest='show', help='print decoded values, publish nothing')
  args = ap.parse_args()

  pm = None if args.show else messaging.PubMaster(['gyroscope', 'accelerometer'])
  sock = messaging.sub_sock('can', timeout=100)

  while True:
    for can_msg in messaging.drain_sock(sock, wait_for_one=True):
      ts = can_msg.logMonoTime
      for frame in can_msg.can:
        if frame.src != BUS:
          continue
        if frame.address == YAW_RATE_ADDR and len(frame.dat) >= 3:
          gz = YAW_SIGN * -decode_yaw(frame.dat)  # car z-up -> device z-down
          if args.show:
            print(f"yaw {gz:+.4f} rad/s")
          else:
            # meas = [roll=0, pitch=0, yaw=gz] -> v = [-gz, 0, 0]
            pm.send('gyroscope', sensor_msg('gyroscope', 'gyroUncalibrated', [-gz, 0.0, 0.0], ts))
        elif frame.address == AXAY_ADDR and len(frame.dat) >= 6:
          ax_raw, ay_raw = decode_axay(frame.dat)
          ax, ay, az = AX_SIGN * ax_raw, AY_SIGN * ay_raw, -GRAVITY  # no vertical sensor: gravity only
          if args.show:
            print(f"ax {ax:+.3f}  ay {ay:+.3f} m/s^2")
          else:
            # meas = [ax, ay, az] -> v = [-az, -ay, -ax]
            pm.send('accelerometer', sensor_msg('accelerometer', 'acceleration', [-az, -ay, -ax], ts))


if __name__ == '__main__':
  # self-check: 12-bit field extraction + zero-point decode
  d = bytes([0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
  assert u12(d, 0) == 0xFFF and u12(d, 12) == 0x000
  zero = bytes(8)
  assert abs(decode_yaw(zero) - (-2.094 + 0.13)) < 1e-9
  main()
