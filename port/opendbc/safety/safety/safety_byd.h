#pragma once

#include "safety_declarations.h"

// BYD ATTO 3 ONLY (2026-07-17 rewrite). The Han/Tang/Song/Qin/Seal bukapilot lineage
// was removed - this fork drives exactly one car, a 2024 Atto 3 (no-MVS4 mode:
// panda spliced inline camera<->car, openpilot engages via the LKAS-button latch).
// History with the multi-platform file: repo byd_port/ patches up to M9 commit 0d6a81e.

#define BYD_CANADDR_LKAS_HUD          0x316   // camera HUD/arming frame (validated-bit passthrough)
#define BYD_CANADDR_STEERING_MODULE   0x1E2   // steering command (angle + STEER_REQ)
#define BYD_CANADDR_EPS_STATUS        0x1FC   // EPS status (STATE nibble; monitored openpilot-side)
#define BYD_CANADDR_ACC_HUD_ADAS      0x32D   // ACC_STATE level-track source (camera bus)
#define BYD_CANADDR_ACC_CMD           0x32E   // longitudinal accel command
#define BYD_CANADDR_PCM_BUTTONS       0x3B0   // wheel buttons (latch toggle source), 20 Hz
#define BYD_CANADDR_DRIVE_STATE       0x242   // debounced BRAKE_PRESSED bit (byte4 bit5), gear
#define BYD_CANADDR_PEDAL             0x342   // raw pedal positions (gas byte0, brake byte1)
#define BYD_CANADDR_STEER_MODULE_2    0x11F   // measured steering angle (0.1 deg LE16), 100 Hz
#define BYD_CANADDR_CARSPEED          0x121   // vehicle_moving source
#define BYD_CANADDR_WHEELSPEED_CLEAN  0x1F0   // vehicle_speed source: raw16 * 0.0758 = kph

// 0x32E ACCEL_CMD is byte0, scale 0.05, offset -5 => accel = raw*0.05 - 5 (m/s^2).
// Comfort/safety envelope -3.5..+2.0 => raw 30..140 (LONG_PLAN.md LG2).
#define BYD_ACC_ACCEL_RAW_MIN         30U    // -3.5 m/s^2
#define BYD_ACC_ACCEL_RAW_MAX         140U   // +2.0 m/s^2
// AEB pass-through: never let our camera-ACC block mask emergency braking. If the
// camera's own ACC_CMD commands harder decel than any comfort command (raw < 20 =>
// < -4.0 m/s^2), forward it even while we're blocking. The Atto3's AEB decel channel
// is unconfirmed offline (no AEB event on tape) - this is the fail-safe.
#define BYD_ACC_AEB_RAW_THRESH        20U    // -4.0 m/s^2

// WHEELSPEED_CLEAN raw -> m/s: raw * 0.0758 kph (dash-matched scale, byd_atto3.dbc) / 3.6
#define BYD_SPEED_RAW_TO_MS           (0.0758 / 3.6)

#define BYD_CANBUS_ESC  0               // car side
#define BYD_CANBUS_MRR  1               // private FD radar (silent-monitor tap, never TX)
#define BYD_CANBUS_MPC  2               // camera side

static bool byd_block_original_acc = false;  // set while OP TXes commanding ACC frames

// DIAG forwarding toggles (bring-up/bench tools, safety param gated; default off = normal):
static bool byd_diag_fwd_passthrough = false;   // 0x100: transparent bridge, block/inject nothing
static bool byd_diag_unblock_cam_1e2 = false;   // 0x200: do NOT block camera 0x1E2 cam->car
static bool byd_diag_unblock_cam_316 = false;   // 0x400: do NOT block camera 0x316 cam->car

// No-MVS4 button engage (safety param 0x1000): the LKAS button TOGGLES an engage latch
// (stock ACC not required); the debounced brake bit clears it. Gas must NOT clear it
// (driver drives on the accelerator; openpilot doesn't disengage on gas either).
// Bit off (default) => controls_allowed tracks the stock ACC-active level only.
static bool byd_poc_btn_engage = false;
static bool byd_btn_latch = false;
static bool byd_btn_prev = false;

static void byd_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if (bus == BYD_CANBUS_ESC) {
    if (addr == BYD_CANADDR_PEDAL) {
      gas_pressed = (GET_BYTE(to_push, 0) != 0U);
    } else if (addr == BYD_CANADDR_DRIVE_STATE) {
      // Debounced brake bit (byte4 bit5, DBC BRAKE_PRESSED 37|1@0+, 50 Hz; car threshold
      // ~9 pedal counts, validated over 46k v3-log frames). carstate reads THIS SAME bit,
      // so the panda latch and the carstate mirror cannot desync on a light brake graze
      // (the 2026-07-17 "Controls Mismatch" root cause, SESSION §5a).
      brake_pressed = ((GET_BYTE(to_push, 4) >> 5) & 0x1U) != 0U;
      if (brake_pressed) {
        byd_btn_latch = false;
      }
    } else if (addr == BYD_CANADDR_CARSPEED) {
      int speed_raw = ((GET_BYTE(to_push, 1) & 0x0FU) << 8) | GET_BYTE(to_push, 0);
      vehicle_moving = (speed_raw != 0);
    } else if (addr == BYD_CANADDR_WHEELSPEED_CLEAN) {
      // vehicle_speed drives the speed-interpolated angle-rate limits. Before 2026-07-17
      // it was NEVER fed, so steer_angle_cmd_checks interpolated at 0 m/s forever and the
      // rate limit sat at the most permissive tier (SAFETY_REVIEW H4).
      int speed_raw = (GET_BYTE(to_push, 1) << 8) | GET_BYTE(to_push, 0);
      UPDATE_VEHICLE_SPEED(speed_raw * BYD_SPEED_RAW_TO_MS);
    } else if (addr == BYD_CANADDR_STEER_MODULE_2) {
      // Measured steering angle, 0.1 deg/LSB signed LE = the same CAN units as the
      // 0x1E2 command -> feeds the enforce_angle_error bound (command may never run
      // more than max_angle_error from the actual wheel).
      int angle_meas_new = to_signed((GET_BYTE(to_push, 1) << 8) | GET_BYTE(to_push, 0), 16);
      update_sample(&angle_meas, angle_meas_new);
    } else if (addr == BYD_CANADDR_PCM_BUTTONS) {
      // LKAS button (byte0 bit6, DBC LKAS_ON_BTN) toggles the engage latch on each press
      // while not braking. Chosen over SET/RES (the speed rocker on this wheel, byte0
      // bits 3/4) because LKAS doesn't nudge the car's ACC.
      bool btn_lkas = ((GET_BYTE(to_push, 0) >> 6) & 0x1U) != 0U;
      if (byd_poc_btn_engage) {
        if (btn_lkas && !byd_btn_prev && !brake_pressed) {
          byd_btn_latch = !byd_btn_latch;
        }
      }
      byd_btn_prev = btn_lkas;
    } else {
      // no other car-side message feeds safety state
    }

    generic_rx_checks(addr == BYD_CANADDR_LKAS_HUD);

  } else if (bus == BYD_CANBUS_MPC) {
    if (addr == BYD_CANADDR_ACC_HUD_ADAS) {
      // Atto3: controls_allowed LEVEL-tracks ACC-active (re-asserted every 50 Hz 0x32D
      // frame) because the car's ACC is already on when OP engages - pcm_cruise_check's
      // rising edge never fires (RE'd from stock passthrough, route 00000020).
      // The POC button latch ORs in on top (param 0x1000, default off).
      // && !brake_pressed (SAFETY_REVIEW H3): the level-track used to overwrite
      // generic_rx_checks' brake exit within 20 ms, leaving brake-disengage single-path
      // through the latch clear. This restores an independent brake gate at this level.
      unsigned int accstate = ((GET_BYTE(to_push, 2) >> 3) & 0x07U);
      bool acc_active = (accstate == 0x3U) || (accstate == 0x5U);
      controls_allowed = (acc_active || (byd_poc_btn_engage && byd_btn_latch)) && !brake_pressed;
    }
  } else {
    // BYD_CANBUS_MRR: silent-monitor tap, radar frames never touch safety state
  }
}

static bool byd_tx_hook(const CANPacket_t *to_send) {
  // 0x1E2 angle-command limits. Units: STEER_ANGLE is 0.1 deg/LSB (angle_deg_to_can=10);
  // rate lookups are deg per TX frame (50 Hz -> per 20 ms).
  // - rate 4.0 deg/frame flat: openpilot's own cap is 3.0 (values.py MAX_ANGLE_RATE) at
  //   every speed, measured p99.9 command delta over 9 routes is <= 3.0, and the stock
  //   camera's own ceiling is 4.8. 4.0 never blocks a legitimate frame (a blocked active
  //   frame gaps the EPS stream -> STATE=11 dropout, see SESSION §5b) but bounds a
  //   runaway at ~12x tighter than the pre-2026-07-17 effective limit.
  // - enforce_angle_error: command may never run more than 50 deg from the measured
  //   wheel above 3 m/s (legitimate cmd-vs-wheel lag measured 27-40 deg p90 in rt56's
  //   sharpest curves; 50 clears that, yet bounds a runaway command hard).
  // - max_angle 390 deg matches the python-side ANGLE_LIMITS.
  const AngleSteeringLimits BYD_ATTO3_STEERING_LIMITS = {
    .max_angle = 3900,
    .angle_deg_to_can = 10,
    .angle_rate_up_lookup = {
      {0., 5., 15.},
      {4., 4., 4.}
    },
    .angle_rate_down_lookup = {
      {0., 5., 15.},
      {4., 4., 4.}
    },
    .max_angle_error = 500,
    .angle_error_min_speed = 3.0,
    .enforce_angle_error = true,
  };

  bool tx = true;
  int bus = GET_BUS(to_send);
  int addr = GET_ADDR(to_send);

  if (bus == BYD_CANBUS_ESC) {
    if (addr == BYD_CANADDR_ACC_CMD) {
      // AccControlActive = byte5 bit4. While OP commands (=1), the fwd hook blocks the
      // camera's own ACC_CMD (minus AEB pass-through); =0 releases the block.
      int acc_control_active = (GET_BYTE(to_send, 5) >> 4) & 0x1U;
      byd_block_original_acc = (acc_control_active == 1);

      // Longitudinal bounds (LONG_PLAN.md LG2): a commanding frame is only permitted
      // when controls_allowed and within the -3.5..+2.0 m/s^2 envelope; the idle frame
      // (acc_control_active=0) is always fine (it matches the camera's disengaged frame).
      unsigned int accel_raw = GET_BYTE(to_send, 0);
      if (acc_control_active == 1) {
        if (!controls_allowed) { tx = false; }
        if ((accel_raw < BYD_ACC_ACCEL_RAW_MIN) || (accel_raw > BYD_ACC_ACCEL_RAW_MAX)) { tx = false; }
      }
    }

    if (addr == BYD_CANADDR_STEERING_MODULE) {
      // STEER_ANGLE 24|16@1- (bytes 3,4 LE signed), STEER_REQ bit 21. Inactive frames
      // (STEER_REQ=0) carry no steering authority and are always allowed - they are the
      // keep-alive stream the EPS needs at stops (SESSION §5b) and the handshake path.
      bool steer_req = GET_BIT(to_send, 21U);
      if (steer_req) {
        // Comma rule, restored explicitly: no active steering unless controls_allowed.
        // (The shared steer_angle_cmd_checks in this carrot fork hardcodes
        // aol_allowed=true, which neuters its own internal check - and a passing
        // REQ=1 frame while disengaged also collides with the camera 0x1E2 frames the
        // fwd hook forwards when !controls_allowed; both observed in route 5f.)
        if (!controls_allowed) {
          tx = false;
        }
        int desired_angle = to_signed((GET_BYTE(to_send, 4) << 8U) | GET_BYTE(to_send, 3), 16);
        // steer_angle_cmd_checks returns TRUE on VIOLATION -> block. Always call it so
        // desired_angle_last stays fresh (rate limits reference the previous command).
        if (steer_angle_cmd_checks(desired_angle, steer_req, BYD_ATTO3_STEERING_LIMITS)) {
          tx = false;
        }
      }
      // 0x316 (LKAS_HUD) needs no per-bit check here: carcontroller passes the camera's
      // own validated frame through and asserts only the two on-car-proven arming bits.
    }
  }

  return tx;
}

static int byd_fwd_hook(CANPacket_t *to_send) {
  const int bus = GET_BUS(to_send);
  const int addr = GET_ADDR(to_send);
  int bus_fwd = -1;

  // DIAG passthrough: full transparent bridge (emulate stock single bus).
  if (byd_diag_fwd_passthrough) {
    if (bus == BYD_CANBUS_ESC) { bus_fwd = BYD_CANBUS_MPC; }
    else if (bus == BYD_CANBUS_MPC) { bus_fwd = BYD_CANBUS_ESC; }
    return bus_fwd;
  }

  if (bus == BYD_CANBUS_ESC) {
    // car -> camera: forward EVERYTHING (incl. PCM_BUTTONS and real EPS feedback;
    // blocking either faults the MVS4 to ACC_STATE=ERROR - proven via bridge 0x810).
    bus_fwd = BYD_CANBUS_MPC;
  } else if (bus == BYD_CANBUS_MPC) {
    // camera -> car: when openpilot is NOT in control, forward the camera's own steering
    // (0x1E2) + LKAS HUD (0x316) so the factory LKS stays healthy (no "LKS limited").
    // OP takes them over only while controls_allowed - the same gate the TX hook uses,
    // so exactly one source feeds the EPS at any time.
    bool block_mpc_msg = ((addr == BYD_CANADDR_LKAS_HUD) || (addr == BYD_CANADDR_STEERING_MODULE))
                         && controls_allowed;

    // Camera ACC_CMD: blocked only while OP is actually commanding (flag AND
    // controls_allowed - a stale flag must never strand the car without ACC), except
    // AEB-grade decel which always passes.
    if (byd_block_original_acc && controls_allowed && (addr == BYD_CANADDR_ACC_CMD)) {
      block_mpc_msg = true;
      if (GET_BYTE(to_send, 0) < BYD_ACC_AEB_RAW_THRESH) {
        block_mpc_msg = false;  // AEB pass-through
      }
    }

    // DIAG unblocks (bench bisection tools)
    if (byd_diag_unblock_cam_1e2 && (addr == BYD_CANADDR_STEERING_MODULE)) { block_mpc_msg = false; }
    if (byd_diag_unblock_cam_316 && (addr == BYD_CANADDR_LKAS_HUD)) { block_mpc_msg = false; }

    if (!block_mpc_msg) {
      bus_fwd = BYD_CANBUS_ESC;
    }
  }

  return bus_fwd;
}

static safety_config byd_init(uint16_t param) {
  // Every message safety reads is liveness-checked at its measured stock rate
  // (v3 log, 46k+ frames each): pedal/brake/speed/angle sources on the car bus,
  // the ACC_STATE level-track source on the camera bus, buttons at 20 Hz.
  static RxCheck byd_atto3_rx_checks[] = {
    {.msg = {{BYD_CANADDR_PEDAL,             BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_DRIVE_STATE,       BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_CARSPEED,          BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_WHEELSPEED_CLEAN,  BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_STEER_MODULE_2,    BYD_CANBUS_ESC, 5, .ignore_checksum = true, .ignore_counter = true, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_PCM_BUTTONS,       BYD_CANBUS_ESC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 20U}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_ACC_HUD_ADAS,      BYD_CANBUS_MPC, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},
  };

  // openpilot may transmit exactly three messages, all to the car bus.
  static const CanMsg BYD_ATTO3_TX_MSGS[] = {
    {BYD_CANADDR_ACC_CMD,          BYD_CANBUS_ESC, 8},
    {BYD_CANADDR_LKAS_HUD,         BYD_CANBUS_ESC, 8},
    {BYD_CANADDR_STEERING_MODULE,  BYD_CANBUS_ESC, 8},
  };

  // param bit 0x10 was the multi-platform Atto3 selector; accepted and ignored for
  // compatibility (interface.py still sends it). Platform branching is gone.
  const uint32_t FLAG_BYD_DIAG_PASSTHROUGH = 0x100U;
  const uint32_t FLAG_BYD_DIAG_UNBLOCK_1E2 = 0x200U;
  const uint32_t FLAG_BYD_DIAG_UNBLOCK_316 = 0x400U;
  const uint32_t FLAG_BYD_POC_BTN_ENGAGE = 0x1000U;
  byd_diag_fwd_passthrough = GET_FLAG(param, FLAG_BYD_DIAG_PASSTHROUGH);
  byd_diag_unblock_cam_1e2 = GET_FLAG(param, FLAG_BYD_DIAG_UNBLOCK_1E2);
  byd_diag_unblock_cam_316 = GET_FLAG(param, FLAG_BYD_DIAG_UNBLOCK_316);
  byd_poc_btn_engage = GET_FLAG(param, FLAG_BYD_POC_BTN_ENGAGE);
  byd_btn_latch = false;
  byd_btn_prev = false;
  byd_block_original_acc = false;

  return BUILD_SAFETY_CFG(byd_atto3_rx_checks, BYD_ATTO3_TX_MSGS);
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
  .fwd = byd_fwd_hook,
};
