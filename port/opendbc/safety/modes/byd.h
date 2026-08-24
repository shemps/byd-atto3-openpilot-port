#pragma once

#include "opendbc/safety/declarations.h"

// BYD ATTO 3 ONLY. Ported 2026-08-01 from the on-car-validated carrot-tree
// safety_byd.h (M9 wip, Atto3-only rewrite of 2026-07-17 + brake-debounce fix
// 0d6a81e) onto the modern opendbc safety layout. Design = the VALIDATED
// PASSTHROUGH model: the camera's own 0x1E2/0x316/0x32E are FORWARDED to the
// car while openpilot is not in control and blocked only while it is (plus
// AEB pass-through on 0x32E). This pairs with a carcontroller that sends
// regenerated frames ONLY while engaged - do not mix with the always-block
// check_relay design (both sources would collide on the bus while disengaged).
// No-MVS4 mode: panda spliced inline camera<->car, engage via LKAS-button latch.

#define BYD_CANADDR_LKAS_HUD          0x316U   // camera HUD/arming frame (validated-bit passthrough)
#define BYD_CANADDR_STEERING_MODULE   0x1E2U   // steering command (angle + STEER_REQ)
#define BYD_CANADDR_EPS_STATUS        0x1FCU   // EPS status (STATE nibble; monitored openpilot-side)
#define BYD_CANADDR_ACC_HUD_ADAS      0x32DU   // ACC_STATE level-track source (camera bus)
#define BYD_CANADDR_ACC_CMD           0x32EU   // longitudinal accel command
#define BYD_CANADDR_PCM_BUTTONS       0x3B0U   // wheel buttons (latch toggle source), 20 Hz
#define BYD_CANADDR_DRIVE_STATE       0x242U   // debounced BRAKE_PRESSED bit (byte4 bit5), gear
#define BYD_CANADDR_PEDAL             0x342U   // raw pedal positions (gas byte0, brake byte1)
#define BYD_CANADDR_STEER_MODULE_2    0x11FU   // measured steering angle (0.1 deg LE16), 100 Hz
#define BYD_CANADDR_CARSPEED          0x121U   // vehicle_moving source
#define BYD_CANADDR_WHEELSPEED_CLEAN  0x1F0U   // vehicle_speed source: raw16 * 0.0758 = kph

// 0x32E ACCEL_CMD is byte0, scale 0.05, offset -5 => accel = raw*0.05 - 5 (m/s^2).
// Comfort/safety envelope -3.5..+2.0 => raw 30..140 (LONG_PLAN.md LG2).
#define BYD_ACC_ACCEL_RAW_MIN         30U    // -3.5 m/s^2
#define BYD_ACC_ACCEL_RAW_MAX         140U   // +2.0 m/s^2
// AEB pass-through: never let our camera-ACC block mask emergency braking. If the
// camera's own ACC_CMD commands harder decel than any comfort command (raw < 20 =>
// < -4.0 m/s^2), forward it even while we're blocking. The Atto3's AEB decel channel
// is unconfirmed offline (no AEB event on tape) - this is the fail-safe.
// NOTE modern-layout port: the fwd hook no longer sees frame data, and this panda fw
// runs the fwd decision BEFORE the rx hook, so the flag below is set by the PREVIOUS
// 0x32E camera frame - AEB forwarding starts one 33 Hz frame (~30 ms) into the event
// and overlaps one frame past it. Physically negligible for a multi-frame AEB burst.
#define BYD_ACC_AEB_RAW_THRESH        20U    // -4.0 m/s^2

// WHEELSPEED_CLEAN raw -> m/s. Kept byte-identical to the validated wip define (the
// DBC pair moved to 0.0713 for vEgo; safety only gates the >3 m/s angle-error check
// and the flat rate table with it, so the 6% delta is inert - see byd_port notes).
#define BYD_SPEED_RAW_TO_MS           (0.0758 / 3.6)

#define BYD_CANBUS_ESC  0U              // car side
#define BYD_CANBUS_MRR  1U              // private FD radar (silent-monitor tap, never TX)
#define BYD_CANBUS_MPC  2U              // camera side

static bool byd_block_original_acc = false;  // set while OP TXes commanding ACC frames
static bool byd_cam_acc_aeb = false;         // last camera 0x32E was AEB-grade decel (see note above)

// Timestamps of the last openpilot TX we let through, per channel. The fwd hook blocks
// the camera's own frames ONLY while we are actually replacing them - controls_allowed
// alone is not enough: the panda LKAS latch can be set while openpilot refuses to engage
// (any NO_ENTRY: no calibration, DM, sensor, process fault). Blocking on the latch alone
// left the EPS with NO steering source at all -> STATE=11 dropout + "LKS limited" until
// the driver toggled the button back off. Now a latch without a command stream is inert:
// the camera keeps steering the car exactly as if openpilot were not there.
#define BYD_TX_ACTIVE_TIMEOUT_US 200000U   // 200 ms = 10 missed 50 Hz lateral frames
// Grace window after controls_allowed goes true, during which the camera is blocked
// even before our first TX lands. Without it the handover overlaps: openpilot starts
// streaming while the camera's own 0x1E2 is still being forwarded, the EPS sees TWO
// sources and faults into the STATE=11 latch ~250 ms after engage (in-car
// 2026-08-02, route 000000ef: first engage clean for 29 s, every re-engage faulted).
// The wip blocked from the instant controls_allowed went true and never overlapped;
// this keeps that behavior AND still releases the camera if openpilot never streams.
#define BYD_ENGAGE_GRACE_US      500000U   // 500 ms
static uint32_t byd_lat_tx_ts = 0U;        // last accepted 0x1E2/0x316 TX
static uint32_t byd_acc_tx_ts = 0U;        // last accepted 0x32E TX
static bool byd_lat_tx_seen = false;
static bool byd_acc_tx_seen = false;
static bool byd_ca_prev = false;           // controls_allowed edge tracking (fwd hook)
static uint32_t byd_ca_since_ts = 0U;

static bool byd_tx_recent(bool seen, uint32_t ts) {
  return seen && (safety_get_ts_elapsed(microsecond_timer_get(), ts) < BYD_TX_ACTIVE_TIMEOUT_US);
}

// true while openpilot owns the channel: it is streaming, or it just engaged and is
// expected to start within the grace window.
static bool byd_op_owns_channel(bool tx_seen, uint32_t tx_ts) {
  bool in_grace = safety_get_ts_elapsed(microsecond_timer_get(), byd_ca_since_ts) < BYD_ENGAGE_GRACE_US;
  return byd_tx_recent(tx_seen, tx_ts) || in_grace;
}

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

static void byd_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == BYD_CANBUS_ESC) {
    if (msg->addr == BYD_CANADDR_PEDAL) {
      gas_pressed = (msg->data[0] != 0U);
    } else if (msg->addr == BYD_CANADDR_DRIVE_STATE) {
      // Debounced brake bit (byte4 bit5, DBC BRAKE_PRESSED 37|1@0+, 50 Hz; car threshold
      // ~9 pedal counts, validated over 46k v3-log frames). carstate reads THIS SAME bit,
      // so the panda latch and the carstate mirror cannot desync on a light brake graze
      // (the 2026-07-17 "Controls Mismatch" root cause, SESSION §5a).
      brake_pressed = ((msg->data[4] >> 5) & 0x1U) != 0U;
      if (brake_pressed) {
        byd_btn_latch = false;
      }
    } else if (msg->addr == BYD_CANADDR_CARSPEED) {
      int speed_raw = ((msg->data[1] & 0x0FU) << 8) | msg->data[0];
      vehicle_moving = (speed_raw != 0);
    } else if (msg->addr == BYD_CANADDR_WHEELSPEED_CLEAN) {
      // vehicle_speed drives the speed-gated angle-error check. Before 2026-07-17 it
      // was NEVER fed, so the checks interpolated at 0 m/s forever (SAFETY_REVIEW H4).
      int speed_raw = (msg->data[1] << 8) | msg->data[0];
      UPDATE_VEHICLE_SPEED(speed_raw * BYD_SPEED_RAW_TO_MS);
    } else if (msg->addr == BYD_CANADDR_STEER_MODULE_2) {
      // Measured steering angle, 0.1 deg/LSB signed LE = the same CAN units as the
      // 0x1E2 command -> feeds the angle-error bound (command may never run more
      // than max_angle_error from the actual wheel).
      int angle_meas_new = to_signed((msg->data[1] << 8) | msg->data[0], 16);
      update_sample(&angle_meas, angle_meas_new);
    } else if (msg->addr == BYD_CANADDR_PCM_BUTTONS) {
      // LKAS button (byte0 bit6, DBC LKAS_ON_BTN) toggles the engage latch on each press
      // while not braking. Chosen over SET/RES (the speed rocker on this wheel, byte0
      // bits 3/4) because LKAS doesn't nudge the car's ACC.
      bool btn_lkas = ((msg->data[0] >> 6) & 0x1U) != 0U;
      if (byd_poc_btn_engage) {
        if (btn_lkas && !byd_btn_prev && !brake_pressed) {
          byd_btn_latch = !byd_btn_latch;
          if (byd_btn_latch) {
            // Re-anchor the shared rate-limit state to the measured wheel at the
            // engage edge. steer_angle_cmd_checks only runs on our REQ=1 frames,
            // so between engagements desired_angle_last stays frozen at the LAST
            // command of the PREVIOUS engagement. A mid-drive re-engage with the
            // wheel anywhere else then fails the rate check on frame one, and the
            // checker's violation-reset (anchor := measured) turns the rate limit
            // into "stay within 4 deg of the driver's wheel" - a permanent block
            // while controlsd ramps toward the model path (in-car 2026-08-02
            // route 000000f7: 37 consecutive active frames blocked at re-engage,
            // EPS starved into STATE=11, soft disable). Upstream avoids this by
            // running the checker on every frame; our REQ=0 skip (deliberate,
            // see tx hook) means the edge must re-anchor explicitly instead.
            desired_angle_last = SAFETY_CLAMP(angle_meas.values[0], -3900, 3900);
          }
        }
      }
      byd_btn_prev = btn_lkas;
    } else {
      // no other car-side message feeds safety state
    }
  } else if (msg->bus == BYD_CANBUS_MPC) {
    if (msg->addr == BYD_CANADDR_ACC_HUD_ADAS) {
      // Atto3: controls_allowed LEVEL-tracks ACC-active (re-asserted every 50 Hz 0x32D
      // frame) because the car's ACC is already on when OP engages - pcm_cruise_check's
      // rising edge never fires (RE'd from stock passthrough, route 00000020).
      // The POC button latch ORs in on top (param 0x1000, default off).
      // && !brake_pressed (SAFETY_REVIEW H3): the level-track used to overwrite
      // the brake exit within 20 ms, leaving brake-disengage single-path through the
      // latch clear. This keeps an independent brake gate at this level.
      unsigned int accstate = ((msg->data[2] >> 3) & 0x07U);
      bool acc_active = (accstate == 0x3U) || (accstate == 0x5U);
      controls_allowed = (acc_active || (byd_poc_btn_engage && byd_btn_latch)) && !brake_pressed;
    } else if (msg->addr == BYD_CANADDR_ACC_CMD) {
      // AEB pass-through flag for the fwd hook (which cannot see frame data): true
      // while the camera's own ACC_CMD carries AEB-grade decel.
      byd_cam_acc_aeb = (msg->data[0] < BYD_ACC_AEB_RAW_THRESH);
    } else {
      // no other camera-side message feeds safety state
    }
  } else {
    // BYD_CANBUS_MRR: silent-monitor tap, radar frames never touch safety state
  }
}

static bool byd_tx_hook(const CANPacket_t *msg) {
  // 0x1E2 angle-command limits. Units: STEER_ANGLE is 0.1 deg/LSB (angle_deg_to_can=10);
  // rate lookups are deg per TX frame (50 Hz -> per 20 ms).
  // - rate 4.0 deg/frame flat: openpilot's own cap is 3.0 (values.py MAX_ANGLE_RATE) at
  //   every speed, measured p99.9 command delta over 9 routes is <= 3.0, and the stock
  //   camera's own ceiling is 4.8. 4.0 never blocks a legitimate frame (a blocked active
  //   frame gaps the EPS stream -> STATE=11 dropout, see SESSION §5b) but bounds a
  //   runaway at ~12x tighter than the pre-2026-07-17 effective limit.
  // - angle-error bound (inlined below; the modern shared checker dropped the
  //   max_angle_error fields): command may never run more than 50 deg from the measured
  //   wheel above 3 m/s (legitimate cmd-vs-wheel lag measured 27-40 deg p90 in rt56's
  //   sharpest curves; 50 clears that, yet bounds a runaway command hard).
  // - max_angle 390 deg matches the python-side ANGLE_LIMITS.
  static const AngleSteeringLimits BYD_ATTO3_STEERING_LIMITS = {
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
    .frequency = 50U,
  };
  const int BYD_MAX_ANGLE_ERROR = 500;        // 50 deg in CAN units
  const float BYD_ANGLE_ERROR_MIN_SPEED = 3.0;  // m/s

  bool tx = true;

  if (msg->bus == BYD_CANBUS_ESC) {
    if (msg->addr == BYD_CANADDR_ACC_CMD) {
      // AccControlActive = byte5 bit4. While OP commands (=1), the fwd hook blocks the
      // camera's own ACC_CMD (minus AEB pass-through); =0 releases the block.
      int acc_control_active = (msg->data[5] >> 4) & 0x1U;
      byd_block_original_acc = (acc_control_active == 1);

      // Longitudinal bounds (LONG_PLAN.md LG2): a commanding frame is only permitted
      // when controls_allowed and within the -3.5..+2.0 m/s^2 envelope; the idle frame
      // (acc_control_active=0) is always fine (it matches the camera's disengaged frame).
      unsigned int accel_raw = msg->data[0];
      if (acc_control_active == 1) {
        if (!controls_allowed) { tx = false; }
        if ((accel_raw < BYD_ACC_ACCEL_RAW_MIN) || (accel_raw > BYD_ACC_ACCEL_RAW_MAX)) { tx = false; }
      }
      if (tx) {
        byd_acc_tx_ts = microsecond_timer_get();
        byd_acc_tx_seen = true;
      }
    }

    if (msg->addr == BYD_CANADDR_STEERING_MODULE) {
      // STEER_ANGLE 24|16@1- (bytes 3,4 LE signed), STEER_REQ bit 21. Inactive frames
      // (STEER_REQ=0) carry no steering authority and are always allowed - they are the
      // keep-alive stream the EPS needs at stops (SESSION §5b) and the handshake path.
      // Deliberately NOT checked when REQ=0 (matches the validated wip hook): the shared
      // checker's inactive-angle test would block idle frames at manual full lock
      // (wheel past 390 deg), gapping the EPS stream exactly when it must not gap.
      bool steer_req = GET_BIT(msg, 21U);
      if (steer_req) {
        // Comma rule: no active steering unless controls_allowed (also enforced inside
        // the shared checker; kept explicit here as in the validated wip hook).
        if (!controls_allowed) {
          tx = false;
        }
        int desired_angle = to_signed((msg->data[4] << 8U) | msg->data[3], 16);
        // steer_angle_cmd_checks returns TRUE on VIOLATION -> block. Called on every
        // REQ=1 frame so desired_angle_last stays fresh (rate limits reference the
        // previous command).
        if (steer_angle_cmd_checks(desired_angle, steer_req, BYD_ATTO3_STEERING_LIMITS)) {
          tx = false;
        }
        // Angle-error bound, inlined (see limits comment). vehicle_speed and angle_meas
        // are in the sample_t scaled units fed by the rx hook.
        if ((vehicle_speed.min / VEHICLE_SPEED_FACTOR) > BYD_ANGLE_ERROR_MIN_SPEED) {
          const int angle_error_max = angle_meas.max + BYD_MAX_ANGLE_ERROR;
          const int angle_error_min = angle_meas.min - BYD_MAX_ANGLE_ERROR;
          if ((desired_angle > angle_error_max) || (desired_angle < angle_error_min)) {
            tx = false;
          }
        }
      }
      // 0x316 (LKAS_HUD) needs no per-bit check here: carcontroller passes the camera's
      // own validated frame through and asserts only the two on-car-proven arming bits.
    }

    // mark the lateral channel live (both frames stream together from carcontroller);
    // the fwd hook blocks the camera's copies only while this stays fresh
    if (((msg->addr == BYD_CANADDR_STEERING_MODULE) || (msg->addr == BYD_CANADDR_LKAS_HUD)) && tx) {
      byd_lat_tx_ts = microsecond_timer_get();
      byd_lat_tx_seen = true;
    }
  }

  return tx;
}

// Modern fwd hook: return true to BLOCK forwarding of (bus_num, addr) toward the
// default destination (0<->2 bridge; bus 1 radar tap is never forwarded by the core).
static bool byd_fwd_hook(int bus_num, int addr) {
  bool block = false;
  // The framework hands the fwd hook a SIGNED addr while BYD_CANADDR_* are unsigned
  // (they are compared against msg->addr in the rx/tx hooks). Convert once here rather
  // than keeping a second, signed copy of every address that could drift from the first.
  const uint32_t addr_u = (uint32_t)addr;

  // controls_allowed edge -> start the engage grace window (the fwd hook runs on
  // every received message, so this samples at bus rate)
  if (controls_allowed != byd_ca_prev) {
    byd_ca_since_ts = microsecond_timer_get();
    byd_ca_prev = controls_allowed;
  }

  if (byd_diag_fwd_passthrough) {
    // DIAG passthrough: full transparent bridge (emulate stock single bus).
    block = false;
  } else if (bus_num == (int)BYD_CANBUS_MPC) {
    // camera -> car: when openpilot is NOT in control, forward the camera's own steering
    // (0x1E2) + LKAS HUD (0x316) so the factory LKS stays healthy (no "LKS limited").
    // OP takes them over only while controls_allowed - the same gate the TX hook uses,
    // so exactly one source feeds the EPS at any time.
    if ((addr_u == BYD_CANADDR_LKAS_HUD) || (addr_u == BYD_CANADDR_STEERING_MODULE)) {
      // controls_allowed AND we are actually streaming our own frames (see
      // byd_lat_tx_ts): a panda latch without an openpilot command stream must
      // leave the camera in charge, never starve the EPS.
      block = controls_allowed && byd_op_owns_channel(byd_lat_tx_seen, byd_lat_tx_ts);
    }

    // Camera ACC_CMD: blocked only while OP is actually commanding (flag AND
    // controls_allowed AND a fresh TX - a stale flag must never strand the car
    // without ACC), except AEB-grade decel which always passes (byd_cam_acc_aeb,
    // set by the rx hook from the previous camera frame - see the AEB note above).
    if (addr_u == BYD_CANADDR_ACC_CMD) {
      block = byd_block_original_acc && controls_allowed && !byd_cam_acc_aeb &&
              byd_op_owns_channel(byd_acc_tx_seen, byd_acc_tx_ts);
    }

    // DIAG unblocks (bench bisection tools)
    if (byd_diag_unblock_cam_1e2 && (addr_u == BYD_CANADDR_STEERING_MODULE)) { block = false; }
    if (byd_diag_unblock_cam_316 && (addr_u == BYD_CANADDR_LKAS_HUD)) { block = false; }
  } else {
    // car -> camera: forward EVERYTHING (incl. PCM_BUTTONS and real EPS feedback;
    // blocking either faults the MVS4 to ACC_STATE=ERROR - proven via bridge 0x810).
    block = false;
  }

  return block;
}

static safety_config byd_init(uint16_t param) {
  // Every message safety reads is liveness-checked at its measured stock rate
  // (v3 log, 46k+ frames each): pedal/brake/speed/angle sources on the car bus,
  // the ACC_STATE level-track source on the camera bus, buttons at 20 Hz.
  static RxCheck byd_rx_checks[] = {
    {.msg = {{BYD_CANADDR_PEDAL,            BYD_CANBUS_ESC, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_DRIVE_STATE,      BYD_CANBUS_ESC, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_CARSPEED,         BYD_CANBUS_ESC, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_WHEELSPEED_CLEAN, BYD_CANBUS_ESC, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_STEER_MODULE_2,   BYD_CANBUS_ESC, 5, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_PCM_BUTTONS,      BYD_CANBUS_ESC, 8,  20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_ACC_HUD_ADAS,     BYD_CANBUS_MPC, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    // camera's own ACC_CMD: OBSERVED ONLY, for the AEB pass-through flag (the core
    // calls the mode rx hook exclusively for whitelisted addrs). optional=true: this
    // is not a safety-state input, so a camera that stops emitting it must NOT drop
    // controls_allowed - unlike 0x32D above, whose absence legitimately disengages.
    {.msg = {{BYD_CANADDR_ACC_CMD,          BYD_CANBUS_MPC, 8,  33U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .optional = true}, { 0 }, { 0 }}},
  };

  // openpilot may transmit exactly three messages, all to the car bus.
  // check_relay=true arms the stock-ECU/relay-malfunction detection for the two
  // steering-path frames (they must never appear as bus-0 RX); disable_static_blocking
  // opts them out of the core's always-block forwarding so the fwd hook above can
  // implement the validated conditional passthrough. 0x32E stays fully dynamic.
  static const CanMsg BYD_TX_MSGS[] = {
    {BYD_CANADDR_ACC_CMD,          BYD_CANBUS_ESC, 8, .check_relay = false},
    {BYD_CANADDR_LKAS_HUD,         BYD_CANBUS_ESC, 8, .check_relay = true, .disable_static_blocking = true},
    {BYD_CANADDR_STEERING_MODULE,  BYD_CANBUS_ESC, 8, .check_relay = true, .disable_static_blocking = true},
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
  byd_cam_acc_aeb = false;
  byd_lat_tx_seen = false;
  byd_acc_tx_seen = false;
  byd_ca_prev = false;
  byd_ca_since_ts = 0U;
  byd_lat_tx_ts = 0U;
  byd_acc_tx_ts = 0U;

  return BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
  .fwd = byd_fwd_hook,
};
