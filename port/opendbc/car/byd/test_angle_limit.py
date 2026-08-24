#!/usr/bin/env python3
"""Unit test for the BYD EPS authority clamp (carcontroller.eps_angle_cap).

Guards the constant against drift: the cap must stay under every observed EPS
drop-out, and must leave the tightest turn that did NOT drop out unclipped.

Run: .venv/bin/python opendbc_repo/opendbc/car/byd/test_angle_limit.py
"""
import sys

import numpy as np

from opendbc.car.byd.carcontroller import eps_angle_cap
from opendbc.car.byd.values import CAR, CarControllerParams as CCP

checks = []
def chk(name, cond):
  checks.append((name, bool(cond)))
  print(("  PASS  " if cond else "  FAIL  ") + name)

KMH = 1 / 3.6

# (speed km/h, |wheel angle| deg) at each EPS drop-out. The clamp must be under all of them.
EPS_DROPOUTS = [(22.0, 153.6), (19.8, 170.0), (23.0, 163.1), (22.2, 148.4), (24.0, 167.1),
                (14.2, 244.0), (22.3, 177.4), (23.7, 161.3), (15.4, 204.9)]
# The tightest engaged turn that did NOT drop out. The clamp must not clip it.
NO_DROPOUT = (21.7, 125.0)

print("== cap value ==")
chk("150 deg at 20 km/h", abs(eps_angle_cap(20 * KMH) - 150.0) < 0.1)
chk("100 deg at 30 km/h", abs(eps_angle_cap(30 * KMH) - 100.0) < 0.1)

print("== bounded at both ends ==")
chk("never over STEER_ANGLE_MAX at a standstill",
    eps_angle_cap(0.0) <= CCP.ANGLE_LIMITS.STEER_ANGLE_MAX)
chk("finite and positive at every speed",
    all(0.0 < eps_angle_cap(v) <= CCP.ANGLE_LIMITS.STEER_ANGLE_MAX
        for v in np.linspace(0.0, 60.0, 601)))
chk("monotonically non-increasing with speed",
    all(eps_angle_cap(a) >= eps_angle_cap(b)
        for a, b in zip(np.linspace(0, 60, 601)[:-1], np.linspace(0, 60, 601)[1:])))

print("== under every observed EPS drop-out ==")
for v, ang in EPS_DROPOUTS:
  chk(f"clips {ang} deg at {v} km/h", ang > eps_angle_cap(v * KMH))

print("== leaves a turn the EPS did hold ==")
v, ang = NO_DROPOUT
chk(f"{ang} deg at {v} km/h not clipped", ang < eps_angle_cap(v * KMH))

print("== the clamp only ever reduces the command ==")
rng = np.random.default_rng(0)
ok = True
for _ in range(2000):
  v = rng.uniform(0.0, 40.0)
  a = rng.uniform(-500.0, 500.0)
  cap = eps_angle_cap(v)
  ok &= abs(float(np.clip(a, -cap, cap))) <= abs(a) + 1e-9
chk("|clipped| <= |commanded|", ok)

print("== the ISO lateral-accel limiter takes over at speed ==")
specs = CAR.BYD_ATTO_3.config.specs
def lat_cap(v_ms):
  return np.degrees(CCP.ANGLE_LIMITS.MAX_LATERAL_ACCEL * specs.wheelbase / max(v_ms ** 2, 1e-9)) * specs.steerRatio
chk("lateral-accel limiter is tighter at 60 km/h", lat_cap(60 * KMH) < eps_angle_cap(60 * KMH))
chk("this clamp is tighter at 20 km/h", eps_angle_cap(20 * KMH) < lat_cap(20 * KMH))

bad = [n for n, ok_ in checks if not ok_]
print(f"\n{len(checks) - len(bad)}/{len(checks)} passed")
sys.exit(1 if bad else 0)
