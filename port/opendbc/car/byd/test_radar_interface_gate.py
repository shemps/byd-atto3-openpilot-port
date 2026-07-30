#!/usr/bin/env python3
"""Unit test for the BYD RadarInterface cut-in gate (2026-07-21).

Drives RadarInterface.update() directly with a fake CANParser (the .vl dict the
real parser exposes), so every gate branch is exercised deterministically —
the replay validation (radar_replay_validate.py §7, cutin_births_scan.py)
covers real-data behavior; this covers the deployed class itself.
Run: .venv/bin/python opendbc_repo/opendbc/car/byd/test_radar_interface_gate.py
"""
import sys

from opendbc.car import structs
from opendbc.car.byd.radar_interface import RadarInterface, TRIGGER_MSG, N_SLOTS

checks = []
def chk(name, cond):
    checks.append((name, bool(cond)))
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")


class FakeRcp:
    def __init__(self):
        self.vl = {f"RADAR_TRACK_{i:02d}": self.empty() for i in range(N_SLOTS)}
        self.can_valid = True

    @staticmethod
    def empty():
        return {"TRACK_ID": 255, "LONG_DIST": 0.0, "LAT_DIST": 0.0,
                "VLEAD": 0.0, "ALEAD": 0.0, "CONFIDENCE": 0.0}

    def update(self, _):
        return {TRIGGER_MSG}

    def set(self, slot, tid, long_dist, lat, vlead, conf):
        self.vl[f"RADAR_TRACK_{slot:02d}"] = {
            "TRACK_ID": tid, "LONG_DIST": long_dist, "LAT_DIST": lat,
            "VLEAD": vlead, "ALEAD": 0.0, "CONFIDENCE": conf}


def make_ri():
    CP = structs.CarParams()
    CP.radarUnavailable = False
    ri = RadarInterface(CP)
    ri.rcp = FakeRcp()
    ri.v_ego = 10.0
    return ri


def cycle(ri):
    ret = ri.update(b"")
    return ret, ri.pts[0].measured


print("== confident track (regression) ==")
ri = make_ri()
ri.rcp.set(0, 7, 50.0, 0.5, 12.0, 1.0)
ret, m = cycle(ri)
chk("conf 1.0 accepted on first cycle", m)
chk("dRel/vRel/vLead mapped", abs(ri.pts[0].dRel - 50.0) < 1e-6 and abs(ri.pts[0].vRel - 2.0) < 1e-6)

print("== cut-in gate: low-conf close co-moving newborn ==")
ri = make_ri()
ri.rcp.set(0, 9, 8.0, 3.0, 12.0, 0.0)
_, m1 = cycle(ri)
_, m2 = cycle(ri)
chk("cycle 1 rejected (debounce)", not m1)
chk("cycle 2 accepted (seen >= 2)", m2)

print("== cut-in gate: each rejection condition alone ==")
for name, args, cycles in [
    ("stationary (vlead 0.5) never accepted",   (0, 9, 8.0, 3.0, 0.5, 0.0), 5),
    ("oncoming (vlead -5) never accepted",      (0, 9, 8.0, 3.0, -5.0, 0.0), 5),
    ("far (25 m) rejected",                     (0, 9, 25.0, 3.0, 12.0, 0.0), 3),
    ("wide (|lat| 3.6) rejected",               (0, 9, 8.0, 3.6, 12.0, 0.0), 3),
]:
    ri = make_ri()
    ri.rcp.set(*args)
    m = any(cycle(ri)[1] for _ in range(cycles))
    chk(name, not m)
ri = make_ri()
ri.rcp.set(0, 9, 8.0, -3.4, 12.0, 0.0)
cycle(ri); _, m = cycle(ri)
chk("lat -3.4 (inside corridor) accepted after debounce", m)

print("== seen-counter lifecycle ==")
ri = make_ri()
ri.rcp.set(0, 7, 8.0, 1.0, 12.0, 0.0)
cycle(ri)
ri.rcp.set(0, 8, 8.0, 1.0, 12.0, 0.0)   # tid change resets
_, m = cycle(ri)
chk("tid change resets debounce", not m)
_, m = cycle(ri)
chk("...and re-arms on the next cycle", m)
ri.rcp.set(0, 255, 0.0, 0.0, 0.0, 0.0)  # track leaves
cycle(ri)
chk("empty tid clears slot state", 0 not in ri.seen and not ri.pts[0].measured)
ri.rcp.set(0, 8, 8.0, 1.0, 12.0, 0.0)   # same tid returns
_, m = cycle(ri)
chk("returning tid needs fresh debounce", not m)

print("== bounds + tid 0 ==")
ri = make_ri()
ri.rcp.set(0, 7, 0.0, 0.0, 12.0, 1.0)
_, m = cycle(ri)
chk("long 0.0 rejected (bounds)", not m)
ri.rcp.set(0, 7, 250.0, 0.0, 12.0, 1.0)
_, m = cycle(ri)
chk("long 250 rejected (bounds)", not m)
ri = make_ri()
ri.rcp.set(0, 0, 40.0, 0.0, 12.0, 1.0)
_, m = cycle(ri)
chk("tid 0 is a VALID track (regression for the `or EMPTY` latent bug)", m)

print("== canError flag ==")
ri = make_ri()
ri.rcp.set(0, 7, 50.0, 0.5, 12.0, 1.0)
ri.rcp.can_valid = False
ret, _ = cycle(ri)
chk("can_valid False -> errors.canError", ret.errors.canError)

fails = [n for n, ok in checks if not ok]
print(f"\n{len(checks) - len(fails)}/{len(checks)} passed")
if fails:
    print("FAILED:", *fails, sep="\n  ")
    sys.exit(1)
print("ALL PASS")
