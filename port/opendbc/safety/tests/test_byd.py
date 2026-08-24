#!/usr/bin/env python3
"""BYD ATTO 3 safety tests (this fork's Atto3-only modes/byd.h).

Named test_byd.py so the standard opendbc safety tooling (pytest discovery,
mutation, coverage, MISRA gating) picks it up - the upstream-style
test_byd.py that shipped with the structural port asserted the always-block
check_relay design this port deliberately does NOT use (validated passthrough:
the camera keeps steering while openpilot is not commanding).

Covers every rule in modes/byd.h: the LKAS-button engage latch, the H3 brake
gate, angle rate + error bounds (H4), no-active-steer-while-disengaged, the
0x32E longitudinal envelope, the TX allowlist, the conditional fwd hook
(incl. AEB pass-through and the TX-recency gate), vehicle_moving, gas
behavior and rx-check staleness.
"""
import struct
import unittest

import opendbc.safety.tests.common as common
from opendbc.car.structs import CarParams
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from opendbc.safety.tests.libsafety import libsafety_py

PARAM = 0x10 | 0x1000  # legacy Atto3 selector (ignored) + button-engage latch

STEERING_MODULE_ADAS = 0x1E2
LKAS_HUD_ADAS = 0x316
ACC_HUD_ADAS = 0x32D
ACC_CMD = 0x32E
PCM_BUTTONS = 0x3B0
DRIVE_STATE = 0x242
PEDAL = 0x342
STEER_MODULE_2 = 0x11F
CARSPEED = 0x121
WHEELSPEED_CLEAN = 0x1F0

MAIN_BUS = 0
CAM_BUS = 2

# modes/byd.h BYD_TX_ACTIVE_TIMEOUT_US / BYD_ENGAGE_GRACE_US
TX_ACTIVE_TIMEOUT_US = 200000
ENGAGE_GRACE_US = 500000
# past both windows: openpilot demonstrably owns nothing
NOT_OWNED_US = ENGAGE_GRACE_US + TX_ACTIVE_TIMEOUT_US + 10000


class TestBydSafety(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.byd, PARAM)
    self.safety.init_tests()
    self.t = 1_000_000
    self.safety.set_timer(self.t)

  # ---- helpers ----
  def _tick(self, us=10000):
    self.t += us
    self.safety.set_timer(self.t)

  def _pkt(self, addr, bus, dat):
    return libsafety_py.make_CANPacket(addr, bus, bytes(dat))

  def _rx(self, addr, bus, dat):
    return self.safety.safety_rx_hook(self._pkt(addr, bus, dat))

  def _tx(self, addr, bus, dat):
    return self.safety.safety_tx_hook(self._pkt(addr, bus, dat))

  def _fwd(self, bus, addr):
    return self.safety.safety_fwd_hook(bus, addr)

  def _steer_frame(self, angle_can, req):
    d = bytearray(8)
    if req:
      d[2] |= 0x20  # STEER_REQ bit 21
    d[3:5] = struct.pack("<h", angle_can)
    return d

  def _acc_frame(self, raw_accel, active):
    d = bytearray(8)
    d[0] = raw_accel
    if active:
      d[5] |= 0x10  # AccControlActive byte5 bit4
    return d

  def _rx_btn(self, lkas):
    return self._rx(PCM_BUTTONS, MAIN_BUS, [0x40 if lkas else 0x00] + [0] * 7)

  def _rx_brake(self, pressed):
    d = [0] * 8
    d[4] = 0x20 if pressed else 0
    return self._rx(DRIVE_STATE, MAIN_BUS, d)

  def _rx_accstate(self, state):
    d = [0] * 8
    d[2] = (state & 0x7) << 3
    return self._rx(ACC_HUD_ADAS, CAM_BUS, d)

  def _rx_cam_acc(self, raw_accel):
    return self._rx(ACC_CMD, CAM_BUS, self._acc_frame(raw_accel, 0))

  def _rx_gas(self, count):
    return self._rx(PEDAL, MAIN_BUS, [count] + [0] * 7)

  def _rx_speed(self, ms):
    raw = int(ms * 3.6 / 0.0758)
    return self._rx(WHEELSPEED_CLEAN, MAIN_BUS, list(struct.pack("<H", raw)) + [0] * 6)

  def _rx_angle(self, deg):
    return self._rx(STEER_MODULE_2, MAIN_BUS, list(struct.pack("<h", int(deg * 10))) + [0] * 3)

  def _latch_engage(self):
    self._rx_brake(False)
    self._rx_btn(False)
    self._rx_btn(True)   # rising edge -> latch
    self._rx_btn(False)
    self._rx_accstate(0)  # level-track picks the latch up

  def _latch_clear(self):
    self._rx_brake(True)
    self._rx_accstate(0)
    self._rx_brake(False)

  def _stream_lateral(self):
    """openpilot streaming its own 0x1E2 - what makes the fwd hook block the camera's"""
    self.assertTrue(self._tx(STEERING_MODULE_ADAS, MAIN_BUS, self._steer_frame(0, 0)))

  # ---- engage latch ----
  def test_boot_not_allowed(self):
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx_accstate(0)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_lkas_latch_engage_and_brake_clear(self):
    self._latch_engage()
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx_brake(True)
    self._rx_accstate(0)
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx_brake(False)
    self._rx_accstate(0)
    self.assertFalse(self.safety.get_controls_allowed(), "brake release must not re-engage")

  def test_h3_brake_gates_level_track(self):
    self._rx_brake(True)
    self._rx_accstate(3)   # stock ACC active while brake held
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx_brake(False)
    self._rx_accstate(3)
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx_accstate(0)
    self.assertFalse(self.safety.get_controls_allowed())

  # ---- angle limits (H4) ----
  def _feed_speed_angle(self):
    for _ in range(8):
      self._rx_speed(15.0)
      self._rx_angle(0.0)

  def test_speed_and_angle_observed(self):
    self._feed_speed_angle()
    self.assertTrue(13.5 <= self.safety.get_vehicle_speed_min() <= 15.5)
    self.assertLessEqual(abs(self.safety.get_angle_meas_max()), 1)

  def test_angle_rate_and_error_bounds(self):
    self._feed_speed_angle()
    self._latch_engage()
    self.safety.set_desired_angle_last(0)
    self.assertTrue(self._tx(STEERING_MODULE_ADAS, MAIN_BUS, self._steer_frame(40, 1)), "+4.0 deg/frame allowed")
    self.safety.set_desired_angle_last(0)
    self.assertFalse(self._tx(STEERING_MODULE_ADAS, MAIN_BUS, self._steer_frame(60, 1)), "+6.0 deg/frame blocked")
    self.safety.set_desired_angle_last(495)
    self.assertTrue(self._tx(STEERING_MODULE_ADAS, MAIN_BUS, self._steer_frame(500, 1)), "50 deg from meas allowed")
    self.safety.set_desired_angle_last(495)
    self.assertFalse(self._tx(STEERING_MODULE_ADAS, MAIN_BUS, self._steer_frame(540, 1)), "54 deg from meas blocked")

  def test_reengage_reanchors_rate_limit(self):
    # desired_angle_last updates only on a REQ=1
    # frames, so between engagements it stays frozen at the PREVIOUS engagement's
    # last command. A mid-drive re-engage with the wheel anywhere else then failed
    # the rate check on frame one, and the checker's violation-reset (anchor :=
    # meas) kept it failing every frame after - 37 straight blocked frames, EPS
    # starved into STATE=11, soft disable. The engage edge must re-anchor to meas.
    self._feed_speed_angle()   # 15 m/s, wheel at 0 - first engagement baseline
    self._latch_engage()
    self.assertTrue(self._tx(STEERING_MODULE_ADAS, MAIN_BUS, self._steer_frame(0, 1)))
    self._rx_btn(True)         # second press: latch off -> disengaged
    self._rx_btn(False)
    self._rx_accstate(0)
    self.assertFalse(self.safety.get_controls_allowed())
    for _ in range(8):
      self._rx_angle(20.0)     # driver turns the wheel to 20 deg while disengaged
    self._latch_engage()       # re-engage mid-turn
    self.assertTrue(self._tx(STEERING_MODULE_ADAS, MAIN_BUS, self._steer_frame(200, 1)),
                    "first frame at the measured wheel must pass after re-engage")

  def test_no_active_steer_while_disengaged(self):
    self._feed_speed_angle()
    self._latch_clear()
    self.assertFalse(self.safety.get_controls_allowed())
    self.safety.set_desired_angle_last(0)
    self.assertFalse(self._tx(STEERING_MODULE_ADAS, MAIN_BUS, self._steer_frame(1, 1)))
    self.assertTrue(self._tx(STEERING_MODULE_ADAS, MAIN_BUS, self._steer_frame(300, 0)), "idle keep-alive allowed")
    self.assertTrue(self._tx(STEERING_MODULE_ADAS, MAIN_BUS, self._steer_frame(-3000, 0)), "idle at full lock allowed")

  # ---- longitudinal envelope ----
  def test_acc_cmd_bounds(self):
    self.assertFalse(self._tx(ACC_CMD, MAIN_BUS, self._acc_frame(100, 1)), "commanding while disengaged")
    self.assertTrue(self._tx(ACC_CMD, MAIN_BUS, self._acc_frame(100, 0)), "idle frame always ok")
    self._latch_engage()
    self.assertTrue(self._tx(ACC_CMD, MAIN_BUS, self._acc_frame(100, 1)))
    self.assertTrue(self._tx(ACC_CMD, MAIN_BUS, self._acc_frame(30, 1)), "-3.5 m/s2 edge")
    self.assertTrue(self._tx(ACC_CMD, MAIN_BUS, self._acc_frame(140, 1)), "+2.0 m/s2 edge")
    self.assertFalse(self._tx(ACC_CMD, MAIN_BUS, self._acc_frame(29, 1)))
    self.assertFalse(self._tx(ACC_CMD, MAIN_BUS, self._acc_frame(141, 1)))

  # ---- allowlist ----
  def test_tx_allowlist(self):
    self.assertTrue(self._tx(LKAS_HUD_ADAS, MAIN_BUS, [0] * 8))
    self.assertFalse(self._tx(PCM_BUTTONS, MAIN_BUS, [0] * 8), "no button TX on this port")
    self.assertFalse(self._tx(PCM_BUTTONS, CAM_BUS, [0] * 8))
    self.assertFalse(self._tx(0x318, CAM_BUS, [0] * 8), "Han-era fake EPS feedback is gone")

  # ---- forwarding ----
  def test_fwd_car_to_camera_always(self):
    self.assertEqual(self._fwd(MAIN_BUS, STEER_MODULE_2), CAM_BUS)

  def test_fwd_blocks_camera_lateral_only_while_streaming(self):
    self._latch_engage()
    self._stream_lateral()
    self.assertEqual(self._fwd(CAM_BUS, STEERING_MODULE_ADAS), -1)
    self.assertEqual(self._fwd(CAM_BUS, LKAS_HUD_ADAS), -1)

  def test_fwd_blocks_camera_immediately_at_engage(self):
    """no handover overlap: the camera must be blocked BEFORE our first TX lands,
    or the EPS sees two 0x1E2 sources and latches state 11."""
    self._latch_engage()
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertEqual(self._fwd(CAM_BUS, STEERING_MODULE_ADAS), -1, "engage grace must block")
    self.assertEqual(self._fwd(CAM_BUS, LKAS_HUD_ADAS), -1)

  def test_fwd_latched_but_not_streaming_leaves_camera_in_charge(self):
    """the NO_ENTRY desync class: panda latch set, openpilot never commands"""
    self._latch_engage()
    self.assertTrue(self.safety.get_controls_allowed())
    # the fwd hook samples the controls_allowed edge at bus rate (it runs on every
    # received message on-car); this first call is what starts the grace window
    self.assertEqual(self._fwd(CAM_BUS, STEERING_MODULE_ADAS), -1)
    self._tick(NOT_OWNED_US)   # grace expires with no TX ever seen
    self.assertEqual(self._fwd(CAM_BUS, STEERING_MODULE_ADAS), MAIN_BUS, "never seen our TX")
    self.assertEqual(self._fwd(CAM_BUS, LKAS_HUD_ADAS), MAIN_BUS)

  def test_fwd_camera_lateral_resumes_after_tx_dropout(self):
    self._latch_engage()
    self._stream_lateral()
    self.assertEqual(self._fwd(CAM_BUS, STEERING_MODULE_ADAS), -1)
    self._tick(NOT_OWNED_US)
    self.assertEqual(self._fwd(CAM_BUS, STEERING_MODULE_ADAS), MAIN_BUS, "stream died -> camera takes over")

  def test_fwd_camera_lateral_forwarded_while_disengaged(self):
    self._latch_clear()
    self.assertEqual(self._fwd(CAM_BUS, STEERING_MODULE_ADAS), MAIN_BUS)
    self.assertEqual(self._fwd(CAM_BUS, LKAS_HUD_ADAS), MAIN_BUS)

  def test_fwd_camera_acc_blocked_only_while_commanding(self):
    self._latch_engage()
    self.assertTrue(self._tx(ACC_CMD, MAIN_BUS, self._acc_frame(100, 1)))
    self._rx_cam_acc(100)
    self.assertEqual(self._fwd(CAM_BUS, ACC_CMD), -1)
    self._tick(NOT_OWNED_US)
    self.assertEqual(self._fwd(CAM_BUS, ACC_CMD), MAIN_BUS, "stale block flag must not strand ACC")

  def test_fwd_camera_aeb_passes_the_block(self):
    self._latch_engage()
    self.assertTrue(self._tx(ACC_CMD, MAIN_BUS, self._acc_frame(100, 1)))
    self._rx_cam_acc(10)   # AEB-grade decel seen by rx
    self.assertEqual(self._fwd(CAM_BUS, ACC_CMD), MAIN_BUS)
    self._rx_cam_acc(100)  # comfort frame clears it again
    self.assertEqual(self._fwd(CAM_BUS, ACC_CMD), -1)

  # ---- rx-check staleness ----
  def test_stale_acc_hud_disengages(self):
    self._latch_engage()
    self.assertTrue(self.safety.get_controls_allowed())
    self._tick(3_000_000)
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.get_controls_allowed())
    self.safety.set_desired_angle_last(0)
    self.assertFalse(self._tx(STEERING_MODULE_ADAS, MAIN_BUS, self._steer_frame(1, 1)))
    self._rx_accstate(0)
    self.assertTrue(self.safety.get_controls_allowed(), "latch survives; OP's state machine governs re-engage")

  def test_optional_camera_acc_cmd_absence_does_not_disengage(self):
    """0x32E is observed for AEB only - its absence must never drop controls_allowed"""
    self._feed_speed_angle()
    self._latch_engage()
    self._rx_cam_acc(100)
    self.assertTrue(self.safety.get_controls_allowed())
    # keep every OTHER checked message fresh while 0x32E goes silent
    for _ in range(40):
      self._tick(50000)  # 50 ms
      self._rx_brake(False)
      self._rx_gas(0)
      self._rx_speed(15.0)
      self._rx_angle(0.0)
      self._rx(CARSPEED, MAIN_BUS, [0x50, 0x01] + [0] * 6)
      self._rx_btn(False)
      self._rx_accstate(0)
      self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.get_controls_allowed(), "2 s without camera ACC_CMD must not disengage")

  # ---- misc car state ----
  def test_vehicle_moving(self):
    self._rx(CARSPEED, MAIN_BUS, [0] * 8)
    self.assertFalse(self.safety.get_vehicle_moving())
    self._rx(CARSPEED, MAIN_BUS, [0x50, 0x01] + [0] * 6)
    self.assertTrue(self.safety.get_vehicle_moving())

  def test_gas_does_not_disengage(self):
    self.safety.set_alternative_experience(ALTERNATIVE_EXPERIENCE.DEFAULT)
    self._rx_gas(0)
    self._latch_engage()
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx_gas(20)
    self.assertTrue(self.safety.get_controls_allowed(), "0.11.2 core has no disengage-on-gas")
    self._rx_accstate(0)
    self.assertTrue(self.safety.get_controls_allowed())


class TestBydIgnition(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()

  def _msg(self, gear):
    return common.make_msg(0, DRIVE_STATE, dat=b"\x00" * 5 + bytes([gear]) + b"\x00" * 2)

  def test_ignition_on_for_every_valid_gear(self):
    for gear in (1, 2, 3, 4):
      self.safety.ignition_can_hook(self._msg(gear))
      self.assertTrue(self.safety.get_ignition_can(), f"gear {gear}")

  def test_ignition_off_when_no_gear(self):
    self.safety.ignition_can_hook(self._msg(4))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(0))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
