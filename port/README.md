# The port files

> **No warranty. No claims. Use at your own risk.** This code commands the steering, braking, and
> acceleration of a moving vehicle. It is not claimed to be correct or safe, and it is not claimed to work on
> your vehicle. Read [../DISCLAIMER.md](../DISCLAIMER.md) before you use any of it.

This directory is the delta. It holds the files that do not exist in an openpilot release, plus a patch for
the ten places an existing file has to change. There is no openpilot tree here — copy these into your own.

Read the [top-level README](../README.md) first. It explains the signals and the control messages that these
files implement.

**These files target released openpilot 0.11.2, on a branch named `byd-atto3`.** They call upstream APIs
only. An earlier revision of this repository shipped the same port from a `carrotpilot` fork; that fork is
retired and its API calls are gone. Where 0.11.2 changed an API or dropped a field, the code says so at the
line — `apply_steer_angle_limits_vm`, `AngleSteeringLimitsVM.MAX_ANGLE_RATE`, the removal of
`CarState.brake`, `CarState.gearStep`, `radarTimeStep`, `vEgoStopping`, `vEgoStarting`, `startAccel`,
`startingState` and `longitudinalTuning.kpBP`/`kpV`.

The patch uses the monorepo paths `opendbc_repo/opendbc/...` and `openpilot/selfdrive/...`. In a standalone
opendbc checkout, strip the `opendbc_repo/` prefix.

## New files

| File | Goes to | Role |
|---|---|---|
| `opendbc/car/byd/values.py` | same path | `CAR.BYD_ATTO_3` platform, `CarControllerParams`, `0xF195` FW query, VIN/WMI fuzzy match |
| `opendbc/car/byd/fingerprints.py` | same path | 109-ID bus-0 fingerprint and the six-ECU FW fingerprint |
| `opendbc/car/byd/carstate.py` | same path | Reads the section-3 signals. Driver torque, EPS state, the pedal and brake sources, the button latch |
| `opendbc/car/byd/carcontroller.py` | same path | Builds `0x1E2`, `0x316`, `0x32E`. Angle limiting, the low-speed slew cap, the jerk limiter, stop and go |
| `opendbc/car/byd/bydcan.py` | same path | The three message packers and `byd_checksum` |
| `opendbc/car/byd/interface.py` | same path | Car params. Angle steering, the longitudinal tune, the radar switch |
| `opendbc/car/byd/radar_interface.py` | same path | Veoneer private-bus tracks to `radard`, with the range-origin correction and the cut-in gate |
| `opendbc/car/byd/veoneer_tracks.py` | same path | Reference track-filter logic the bench replay validator exercises. Keeps raw ranges on purpose |
| `opendbc/car/byd/__init__.py` | same path | Empty |
| `opendbc/dbc/byd_atto3.dbc` | same path | Chassis bus |
| `opendbc/dbc/byd_radar_fd.dbc` | same path | Private bus, `RADAR_TRACK_00..09`. Needed only for radar |
| `opendbc/safety/modes/byd.h` | same path | Panda safety. TX bounds, the forward hook, AEB pass-through |

## Changes to existing files

`integration.patch` carries all ten. Apply it, or make the edits by hand — most are one line.

| File | Change |
|---|---|
| `opendbc/car/values.py` | Add `BYD` to the `Platform` union |
| `opendbc/car/car.capnp` | Add `byd @35` to `SafetyModel` |
| `opendbc/can/dbc.py` | Register `byd_checksum` for `byd_` DBCs, and exempt `byd_radar` from it |
| `opendbc/car/torque_data/override.toml` | `BYD_ATTO_3` row. Missing it raises `KeyError` |
| `opendbc/safety/safety.h` | Include `opendbc/safety/modes/byd.h`, add `byd_hooks` to the mode table |
| `opendbc/safety/declarations.h` | Define `SAFETY_BYD 35U` and declare the `byd_hooks` extern |
| `panda/board/drivers/can_common.h` | Ignition from `0x242` gear. Openpilot stays offroad without it |
| `panda/board/main.c` | Put panda bus 1 in bus-monitoring mode so the radar tap cannot ACK |
| `selfdrive/pandad/panda_safety.cc` | Stay SILENT during a `SKIP_FW_QUERY` fingerprint. An inline panda with no relay otherwise starves the camera and sets lost-communication DTCs |
| `selfdrive/pandad/pandad_api_impl.py` | Unrelated to BYD. A pycapnp garbage fix, carried here because the port runs on a PC |

The two `panda/` changes need a panda rebuild and a reflash. The rest is Python and C that builds with the
rest of the tree.

**The mode table entry sits inside `#ifdef ALLOW_DEBUG`.** Build the panda with `ALLOW_DEBUG`, or move the
entry out of that block yourself and understand why it was there.

## Order

1. Copy the new files. Apply `integration.patch`.
2. Rebuild and reflash the panda.
3. Fingerprint the car. Check that it matches as `BYD_ATTO_3`.
4. Run the two tests below.
5. Drive it somewhere empty before you drive it anywhere else.

## Tests

```
python opendbc/car/byd/test_radar_interface_gate.py     # radar cut-in gate, no car needed
pytest opendbc/safety/tests/test_byd.py                 # safety rules against libsafety
```

`test_byd.py` is a standard opendbc safety test — 22 cases — so opendbc's own pytest, mutation, coverage and
MISRA tooling picks it up. Build `libsafety` with opendbc's build first. The earlier standalone acceptance
script and its `carrot_test_prelude.h` are gone; the same coverage lives in `test_byd.py`.

Passing them proves nothing beyond the tests themselves. They are not validation and not a safety case.

## Running on a PC

A comma device supplies an IMU, a camera stack, and a known hardware profile. A PC supplies none of them.
`pc/` covers the gap.

| File | Role |
|---|---|
| `pc/can_imu_bridge.py` | Publishes `gyroscope` and `accelerometer` from the car's own `0x222` / `0x223`. Without it `sensord` is silent and `locationd` / `livePose` degrade. Section 10 of the top-level README explains the decode |
| `pc/pc-host.patch` | `tools/webcam/camera.py`, `tools/webcam/camerad.py`, `system/hardware/pc/hardware.py` |

Run the bridge inside the openpilot venv, after `pandad`, and check the signs before you drive on them:

```
python pc/can_imu_bridge.py --print    # print decoded values, publish nothing
python pc/can_imu_bridge.py            # publish
```

`pc-host.patch` carries three things. The first two are needed for the webcam path to work at all:

- **camerad timestamp.** Stock `camerad.py` stamps frames `frame_id * 50 ms`, which starts near zero. Fed by
  a real-clock IMU, `locationd` then rejects every `cameraOdometry` observation as older than its own filter
  time. The patch stamps `CLOCK_BOOTTIME` instead. This one pairs with the IMU bridge — the bridge alone does
  not help while the camera clock disagrees with it.
- **NV12 device layout.** The model's warp kernels expect row-stride-padded NV12. Publishing tight-packed
  rows shears every frame the model sees while the UI still looks correct.
- **camera.py V4L2 handling**: MJPG forced, capture normalized to the 1928x1208 canvas, and stable device
  paths instead of `/dev/videoN` ordering. Per-module control defaults, with `WEBCAM_*_CTRLS` to carry
  qv4l2-found values into a launcher. The controls apply on every camera open, not only at launch. A USB re-enumeration
  resets the module to its vendor defaults mid-drive, and a launch-time setting does not come back. Read
  section 9 of the top-level README before you use the values in it — they suit two specific modules and do
  not transfer.

Not covered here, because it is rig-specific rather than port-specific: the launcher scripts, process
pinning, and the GPS source. openpilot runs without GPS. A USB NMEA receiver on the `ubloxd` path is one way
to add one.

## Not here

The capture and decode tooling, the drive-review dashboard, the launcher scripts, the tuning work, and the
panda binary. Build the panda from source — do not flash a signed binary you cannot audit.

## Environment switches

The port reads four environment variables. All default to off.

| Variable | Effect |
|---|---|
| `POC_BTN_ENGAGE=1` | Engage on the LKAS button instead of the stock ACC. Also sets safety param `0x1000`, and sets `pcmCruise = False` so the rocker owns the set speed |
| `USE_OP_LONG=1` | openpilot longitudinal. Sends `0x32E` |
| `USE_VEONEER_RADAR=1` | Read the private-bus radar. Needs `byd_radar_fd.dbc` and panda bus 1 |
| `POC_LONG_TEST_ACCEL=<m/s²>` | Bypass the planner with a fixed accel, clamped to ±1.0. Bring-up only |
