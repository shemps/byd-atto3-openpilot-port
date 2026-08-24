#!/usr/bin/env python3
"""Unit test for the BYD RadarInterface cut-in gate.

The test calls RadarInterface.update() with a fake CANParser, which supplies the
.vl dict that the real parser gives. Each gate branch therefore runs with a known
input. An offline replay covers the behavior on real data. This test covers the
class itself.
Run: python opendbc/car/byd/test_radar_interface_gate.py
"""
import sys

from opendbc.car import structs
from opendbc.car.byd.radar_interface import RadarInterface, TRIGGER_MSG, N_SLOTS, BUMPER_TO_RADAR_ORIGIN_M

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
    return ret, 0 in ri.pts


print("== confident track (regression) ==")
ri = make_ri()
ri.rcp.set(0, 7, 50.0, 0.5, 12.0, 1.0)
ret, m = cycle(ri)
chk("conf 1.0 accepted on first cycle", m)
# dRel is BUMPER-referenced while LONG_DIST comes from the ego rear-axle origin
# (FINDINGS 16), so a 50 m track publishes 50 - BUMPER_TO_RADAR_ORIGIN_M
# tolerance 1e-3, not 1e-6: RadarPoint.dRel is stored as float32, and unlike the
# old exact 50.0 the corrected value is not representable (46.4 -> 46.4000015)
chk("dRel origin-corrected, vRel mapped",
    abs(ri.pts[0].dRel - (50.0 - BUMPER_TO_RADAR_ORIGIN_M)) < 1e-3
    and abs(ri.pts[0].vRel - 2.0) < 1e-6)

print("== range origin: a track inside the offset is not a lead behind the bumper ==")
ri = make_ri()
ri.rcp.set(0, 7, BUMPER_TO_RADAR_ORIGIN_M - 0.6, 0.0, 12.0, 1.0)   # 3.0 m raw
_, m = cycle(ri)
chk("raw range under the origin offset dropped", not m)
ri = make_ri()
ri.rcp.set(0, 7, BUMPER_TO_RADAR_ORIGIN_M + 1.4, 0.0, 12.0, 1.0)   # 5.0 m raw
_, m = cycle(ri)
chk("just outside the offset kept, 1.4 m ahead of the bumper",
    m and abs(ri.pts[0].dRel - 1.4) < 1e-3)

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
chk("empty tid clears slot state", 0 not in ri.seen and 0 not in ri.pts)
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
