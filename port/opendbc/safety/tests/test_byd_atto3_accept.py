#!/usr/bin/env python3
"""Offline acceptance test for the Atto3-only safety_byd.h (2026-07-17 rewrite).

Exercises every safety rule against libsafety.so BEFORE the fw reaches the car:
latch engage/disengage, the H3 brake gate, angle rate + error bounds (H4), the
restored no-steer-while-disengaged rule, ACC_CMD bounds, allowlist, fwd hook.

Build libsafety first (from opendbc/safety/tests/libsafety/):
  gcc -Wall -Werror -nostdlib -fno-builtin -std=gnu11 -Wfatal-errors \
      -Wno-pointer-to-int-cast -include carrot_test_prelude.h -fPIC \
      -I. -I../../board/ -I../../ -shared -o libsafety.so safety.c
Run:  .venv/bin/python opendbc_repo/opendbc/safety/tests/test_byd_atto3_accept.py
"""
import struct
import sys

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py

s = libsafety_py.libsafety

# This carrot fork changed safety_fwd_hook to take a CANPacket_t* (the AEB pass-through
# inspects frame data), but libsafety_py's cdef still declares the upstream (int, int)
# signature - calling that segfaults. Open a second handle with the real signature.
from cffi import FFI  # noqa: E402
_ffi = FFI()
_ffi.cdef("""
typedef struct {
  unsigned char fd : 1;
  unsigned char bus : 3;
  unsigned char data_len_code : 4;
  unsigned char rejected : 1;
  unsigned char returned : 1;
  unsigned char extended : 1;
  unsigned int addr : 29;
  unsigned char checksum;
  unsigned char data[64];
} CANPacket_t;
int safety_fwd_hook(CANPacket_t *to_send);
void set_timer(unsigned int t);
void safety_tick_current_safety_config(void);
void set_alternative_experience(int mode);
""", packed=True)
import opendbc.safety.tests.libsafety.libsafety_py as _lp  # noqa: E402
_lib = _ffi.dlopen(_lp.libsafety_fn)
from opendbc.safety import LEN_TO_DLC  # noqa: E402

def fwd(bus, addr, dat=b"\x00" * 8):
    p = _ffi.new("CANPacket_t*")
    p.bus = bus
    p.addr = addr
    p.data_len_code = LEN_TO_DLC[len(dat)]
    for i, b in enumerate(dat):
        p.data[i] = b
    return _lib.safety_fwd_hook(p)

PARAM = 0x10 | 0x1000  # legacy Atto3 bit (ignored) + button-engage latch

checks = []
def chk(name, cond):
    checks.append((name, bool(cond)))
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")

def pkt(addr, bus, dat):
    return libsafety_py.make_CANPacket(addr, bus, bytes(dat))

def rx(addr, bus, dat):
    return s.safety_rx_hook(pkt(addr, bus, dat))

def tx(addr, bus, dat):
    return s.safety_tx_hook(pkt(addr, bus, dat))

def steer_frame(angle_can, req):
    d = bytearray(8)
    if req:
        d[2] |= 0x20  # STEER_REQ bit 21
    d[3:5] = struct.pack("<h", angle_can)
    return d

def acc_frame(raw_accel, active):
    d = bytearray(8)
    d[0] = raw_accel
    if active:
        d[5] |= 0x10  # AccControlActive byte5 bit4
    return d

def rx_btn(lkas):
    return rx(0x3B0, 0, [0x40 if lkas else 0x00] + [0]*7)

def rx_brake(pressed):
    d = [0]*8
    d[4] = 0x20 if pressed else 0
    return rx(0x242, 0, d)

def rx_accstate(state):
    d = [0]*8
    d[2] = (state & 0x7) << 3
    return rx(0x32D, 2, d)

def rx_speed(ms):
    raw = int(ms * 3.6 / 0.0758)
    return rx(0x1F0, 0, list(struct.pack("<H", raw)) + [0]*6)

def rx_angle(deg):
    return rx(0x11F, 0, list(struct.pack("<h", int(deg * 10))) + [0]*3)

def latch_engage():
    rx_brake(False)
    rx_btn(False)
    rx_btn(True)   # rising edge -> latch
    rx_btn(False)
    rx_accstate(0) # level-track picks the latch up

s.set_safety_hooks(CarParams.SafetyModel.byd, PARAM)
s.init_tests()

print("== init ==")
chk("controls not allowed at boot", not s.get_controls_allowed())
rx_accstate(0)
chk("0x32D alone does not engage", not s.get_controls_allowed())

print("== latch engage / brake clear ==")
latch_engage()
chk("LKAS toggle latches -> controls allowed", s.get_controls_allowed())
rx_brake(True)
rx_accstate(0)
chk("debounced brake clears latch -> controls not allowed", not s.get_controls_allowed())
rx_brake(False)
rx_accstate(0)
chk("brake release does NOT re-engage (latch stays cleared)", not s.get_controls_allowed())

print("== H3: brake gates the level-track (stock-ACC path) ==")
rx_brake(True)
rx_accstate(3)  # stock ACC active while brake held
chk("ACC_STATE=3 with brake held -> controls NOT allowed (H3)", not s.get_controls_allowed())
rx_brake(False)
rx_accstate(3)
chk("ACC_STATE=3, brake released -> controls allowed", s.get_controls_allowed())
rx_accstate(0)
chk("ACC_STATE=0 -> controls not allowed again", not s.get_controls_allowed())

print("== angle safety setup ==")
for _ in range(8):
    rx_speed(15.0)
    rx_angle(0.0)
chk("vehicle_speed fed (~15 m/s)", 13.5 <= s.get_vehicle_speed_min() <= 15.5)
chk("angle_meas fed (~0)", abs(s.get_angle_meas_max()) <= 1)

print("== H4: rate + error + max bounds on 0x1E2 ==")
latch_engage()
chk("re-engaged for steering tests", s.get_controls_allowed())
s.set_desired_angle_last(0)
chk("REQ=1 delta +4.0 deg allowed (<= 4.0/frame + margin)", tx(0x1E2, 0, steer_frame(40, 1)))
s.set_desired_angle_last(0)
chk("REQ=1 delta +6.0 deg blocked (rate)", not tx(0x1E2, 0, steer_frame(60, 1)))
s.set_desired_angle_last(495)
chk("cmd 50.0 deg from meas allowed (error bound edge)", tx(0x1E2, 0, steer_frame(500, 1)))
s.set_desired_angle_last(495)
chk("cmd 54.0 deg from meas blocked (error bound)", not tx(0x1E2, 0, steer_frame(540, 1)))

print("== no active steering while disengaged ==")
rx_brake(True)   # clear latch
rx_accstate(0)
rx_brake(False)
chk("disengaged", not s.get_controls_allowed())
s.set_desired_angle_last(0)
chk("REQ=1 blocked while disengaged (even tiny delta)", not tx(0x1E2, 0, steer_frame(1, 1)))
chk("REQ=0 idle frame allowed while disengaged", tx(0x1E2, 0, steer_frame(300, 0)))
chk("REQ=0 idle frame allowed with any angle content", tx(0x1E2, 0, steer_frame(-3000, 0)))

print("== ACC_CMD bounds ==")
chk("commanding frame blocked while disengaged", not tx(0x32E, 0, acc_frame(100, 1)))
chk("idle ACC frame allowed while disengaged", tx(0x32E, 0, acc_frame(100, 0)))
latch_engage()
chk("commanding accel raw=100 (0 m/s^2) allowed", tx(0x32E, 0, acc_frame(100, 1)))
chk("raw=30 (-3.5) allowed (lower edge)", tx(0x32E, 0, acc_frame(30, 1)))
chk("raw=140 (+2.0) allowed (upper edge)", tx(0x32E, 0, acc_frame(140, 1)))
chk("raw=29 (< -3.5) blocked", not tx(0x32E, 0, acc_frame(29, 1)))
chk("raw=141 (> +2.0) blocked", not tx(0x32E, 0, acc_frame(141, 1)))

print("== allowlist ==")
chk("0x316 TX allowed (allowlist)", tx(0x316, 0, [0]*8))
chk("0x3B0 TX blocked on bus 0 (no button TX anymore)", not tx(0x3B0, 0, [0]*8))
chk("0x3B0 TX blocked on bus 2", not tx(0x3B0, 2, [0]*8))
chk("0x318 TX blocked (Han fake-EPS gone)", not tx(0x318, 2, [0]*8))

print("== fwd hook ==")
chk("car->camera forwards everything", fwd(0, 0x11F) == 2)
chk("camera 0x1E2 BLOCKED while engaged", fwd(2, 0x1E2) == -1)
chk("camera 0x316 BLOCKED while engaged", fwd(2, 0x316) == -1)
tx(0x32E, 0, acc_frame(100, 1))  # OP commanding -> camera-ACC block flag set
chk("camera ACC_CMD blocked while OP commands", fwd(2, 0x32E, bytes(acc_frame(100, 0))) == -1)
chk("camera AEB-grade decel (raw<20) passes the block", fwd(2, 0x32E, bytes(acc_frame(10, 0))) == 0)
rx_brake(True); rx_accstate(0); rx_brake(False)
chk("camera 0x1E2 forwarded while disengaged", fwd(2, 0x1E2) == 0)
chk("camera 0x316 forwarded while disengaged", fwd(2, 0x316) == 0)
chk("camera ACC_CMD forwarded while disengaged (stale flag can't strand ACC)", fwd(2, 0x32E, bytes(acc_frame(100, 0))) == 0)

print("== vehicle_moving ==")
rx(0x121, 0, [0]*8)
chk("speed raw 0 -> not moving", not s.get_vehicle_moving())
rx(0x121, 0, [0x50, 0x01] + [0]*6)
chk("speed raw != 0 -> moving", s.get_vehicle_moving())

print("== M2: gas tip-in vs alternativeExperience ==")
# panda gas_pressed = 0x342 byte0 != 0 (1 count; carstate uses >10 - the M2 asymmetry).
# With alt_exp=0 (the live tree's hardcoded value) the generic gas exit clears CA for
# one frame until the 0x32D level-track re-asserts it. wip sets DISABLE_DISENGAGE_ON_GAS
# via get_alternative_experience(DisengageOnAccelerator=False), which disables the exit.
from opendbc.safety import ALTERNATIVE_EXPERIENCE  # noqa: E402
def rx_gas(count):
    return rx(0x342, 0, [count] + [0]*7)
_lib.set_alternative_experience(0)
rx_gas(0)
latch_engage()
chk("alt_exp=0: engaged with gas released", s.get_controls_allowed())
rx_gas(20)
chk("alt_exp=0: gas tip-in clears CA (the documented 1-frame nuisance)", not s.get_controls_allowed())
rx_accstate(0)
chk("alt_exp=0: level-track re-asserts CA next 0x32D (latch not cleared by gas)", s.get_controls_allowed())
rx_gas(0)
rx_brake(True); rx_accstate(0); rx_brake(False)  # clear the latch (latch_engage toggles)
_lib.set_alternative_experience(ALTERNATIVE_EXPERIENCE.DISABLE_DISENGAGE_ON_GAS)
latch_engage()
rx_gas(20)
chk("DISABLE_DISENGAGE_ON_GAS: gas tip-in does NOT clear CA", s.get_controls_allowed())
rx_accstate(0)
chk("DISABLE_DISENGAGE_ON_GAS: CA stable with gas held", s.get_controls_allowed())
rx_gas(0)

print("== M5: 0x32D staleness clears controls_allowed (safety_tick) ==")
# The level-track only runs on received 0x32D frames; the generic backstop is
# safety_tick's lagging check over the RxCheck list (threshold max(10/freq, 1 s)).
rx_brake(True); rx_accstate(0); rx_brake(False)  # clear the latch (latch_engage toggles)
latch_engage()
chk("engaged before staleness", s.get_controls_allowed())
_lib.set_timer(3_000_000)  # 3 s with no rx on any checked message
_lib.safety_tick_current_safety_config()
chk("stale 0x32D (+all checks) -> controls NOT allowed", not s.get_controls_allowed())
s.set_desired_angle_last(0)
chk("REQ=1 blocked while stale", not tx(0x1E2, 0, steer_frame(1, 1)))
rx_accstate(0)
chk("0x32D resumes -> latch re-asserts CA (latch survives staleness; OP's own state machine governs re-engage)", s.get_controls_allowed())

fails = [n for n, ok in checks if not ok]
print(f"\n{len(checks) - len(fails)}/{len(checks)} passed")
if fails:
    print("FAILED:", *fails, sep="\n  ")
    sys.exit(1)
print("ALL PASS")
