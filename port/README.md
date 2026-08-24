# The port files

> **No warranty. No claims. Use at your own risk.** This code commands the steering, braking, and
> acceleration of a moving vehicle. It is not claimed to be correct or safe, and it is not claimed to work on
> your vehicle. Read [../DISCLAIMER.md](../DISCLAIMER.md) before you use any of it.

This directory holds the files that do not exist in an openpilot release, plus a patch for the ten places
an existing file has to change. Copy these into your own tree.

Read the [top-level README](../README.md) first. It explains the signals these files implement.

**These files target released openpilot 0.11.2, branch `byd-atto3`.** On another version, check these APIs
first: `apply_steer_angle_limits_vm`, `AngleSteeringLimitsVM.MAX_ANGLE_RATE`, and the absent
`CarState.brake`, `CarState.gearStep`, `radarTimeStep`, `vEgoStopping`, `vEgoStarting`, `startAccel`,
`startingState`, `longitudinalTuning.kpBP` and `kpV`.

The patch uses the monorepo paths `opendbc_repo/opendbc/...` and `openpilot/selfdrive/...`. Remove the
`opendbc_repo/` prefix in a standalone opendbc checkout.

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

**The mode table entry is inside `#ifdef ALLOW_DEBUG`.** Build the panda with `ALLOW_DEBUG`, or move the
entry out of that block. If you move it, first find out why it is there.

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

Build `libsafety` with the opendbc build first. `test_byd.py` has 22 cases and carries the standard name,
so the opendbc pytest, mutation, coverage and MISRA tools find it.

Passing them proves nothing beyond the tests themselves. They are not validation and not a safety case.

## Running on a PC

A PC has no IMU, no camera stack and no known hardware profile. `pc/` covers the gap.

| File | Role |
|---|---|
| `pc/can_imu_bridge.py` | Publishes `gyroscope` and `accelerometer` from the car's own `0x222` / `0x223`. Without it `sensord` is silent and `locationd` / `livePose` degrade. Section 10 of the top-level README explains the decode |
| `pc/pc-host.patch` | `tools/webcam/camera.py`, `tools/webcam/camerad.py`, `system/hardware/pc/hardware.py` |

Run the bridge inside the openpilot venv, after `pandad`. Check the signs before you drive on them:

```
python pc/can_imu_bridge.py --print    # print decoded values, publish nothing
python pc/can_imu_bridge.py            # publish
```

`pc-host.patch` carries three changes. The webcam path needs the first two to work at all.

- **camerad timestamp.** Stamp frames with `CLOCK_BOOTTIME`. Stock `camerad.py` stamps `frame_id * 50 ms`,
  which starts near zero, and `locationd` then rejects every `cameraOdometry` observation as too old.
- **NV12 device layout.** Publish row-stride-padded NV12. Tight-packed rows shear every frame the model
  sees, while the UI still looks correct.
- **camera.py V4L2 handling.** MJPG forced, capture normalized to the 1928x1208 canvas, stable device paths,
  and per-module control defaults through `WEBCAM_*_CTRLS`. The controls apply on every camera open. Set
  your own values. The ones here suit two specific modules.

## Not here

The capture and decode tooling, the drive-review dashboard, the launcher scripts, process pinning, the
tuning work, the GPS source, and the panda binary. openpilot runs without GPS. Build the panda from source.
Do not flash a signed binary you cannot audit.

## Environment switches

The port reads four environment variables. All default to off.

| Variable | Effect |
|---|---|
| `POC_BTN_ENGAGE=1` | Engage on the LKAS button instead of the stock ACC. Also sets safety param `0x1000`, and sets `pcmCruise = False` so the rocker owns the set speed |
| `USE_OP_LONG=1` | openpilot longitudinal. Sends `0x32E` |
| `USE_VEONEER_RADAR=1` | Read the private-bus radar. Needs `byd_radar_fd.dbc` and panda bus 1 |
| `POC_LONG_TEST_ACCEL=<m/s²>` | Bypass the planner with a fixed accel, clamped to ±1.0. Bring-up only |
