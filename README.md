# BYD Atto 3 — openpilot porting reference

This document gives the data to port openpilot to a BYD Atto 3 (Yuan Plus EV). It covers the wiring, the
bus topology, the signal definitions, the control messages, and the panda requirements.

> **No warranty. No claims. Use at your own risk.** This is a personal notebook from one car, published in
> case it is useful. Nothing here is claimed to be correct, complete, or safe, and nothing here is claimed to
> work on your vehicle. It concerns the steering, braking, and acceleration of a moving vehicle, where an
> error can have serious consequences. Verify everything yourself.
> **Read [DISCLAIMER.md](DISCLAIMER.md) before you use any of it.**

All observations come from one car, a 2024 RHD export build.

https://github.com/user-attachments/assets/0a8011b6-2d72-4bb8-8523-d18c12993d65

The comma 4 does not ship to my location. This port therefore runs on an x86 mini-PC, with a USB webcam as
the road camera. A comma red panda, spliced inline at the camera connector, is the CAN interface. No comma
device is in the car. Two consequences get their own sections. The road camera is a webcam (section 9). The
PC has no IMU, so the port bridges the car's own IMU from CAN (section 10).

**The port runs on released openpilot 0.11.2, on a branch named `byd-atto3`.** The files in
[`port/`](port/) are the 0.11.2 files and use upstream APIs only.

[MIT licensed](LICENSE). Not affiliated with BYD or comma.ai — see [DISCLAIMER.md](DISCLAIMER.md).

| | |
|---|---|
| Vehicle | BYD Atto 3 / Yuan Plus EV, 2024, RHD export build |
| VIN WMI | `LGX` |
| ADAS camera | Veoneer MVS4 — Veoneer P/N `682212300`, BYD P/N `SC2EM-3619100A` |
| Camera FW | HW 0.0.4, SW 4.01.04 |
| Forward radar | Veoneer 77V12FLR |
| EPS | Column-assist (CEPS). Supplier not identified |
| Mass / wheelbase | 1750 kg / 2.72 m |
| Steer ratio | 19.8 |
| openpilot | released 0.11.2, branch `byd-atto3` |

Related upstream work: [opendbc PR #3337](https://github.com/commaai/opendbc/pull/3337) (Atto 3),
[PR #3352](https://github.com/commaai/opendbc/pull/3352) (Sealion 7),
[issue #2065](https://github.com/commaai/opendbc/issues/2065) (harness).

---

## 1. Wiring and bus topology

Interpose at the P13 camera connector. The connector has 8 cavities. 6 cavities are wired.

| Pin | Signal | Wire |
|---:|---|---|
| 1 | Camera GND | black |
| 2 | IG1 (switched 12 V) | red/grey |
| 3 | Private CAN_H | pink |
| 4 | Chassis CAN_H | pink |
| 7 | Private CAN_L | violet |
| 8 | Chassis CAN_L | violet |

Source: the BYD *Definition of Terminals* document.

Both CAN_H wires are pink. Both CAN_L wires are violet. Only the pin position identifies the bus.

**Chassis CAN** (pins 4/8) is classic CAN at 500 kbit/s with 8-byte frames. openpilot reads and writes this
bus. Sections 2 to 7 refer to this bus.

**Private CAN** (pins 3/7) is CAN-FD with a data phase near 2 Mbit/s. It carries the MVS4 camera and the
77V12FLR radar. Use this bus to read radar tracks (section 8). The wire also carries about 72 classic-format
CAN IDs with the FD frames. The source of those IDs is not established. Do not assume that the bus has only
two nodes.

Add termination to the chassis CAN at the panda. The panda breaks the bus at the camera connector, which
separates the camera from the vehicle-side termination.

---

## 2. Car identification

Query firmware versions with `0xF195` (SUPPLIER_SOFTWARE_VERSION). BYD ECUs send a negative response to
`0xF188`, which is the openpilot default. In a diagnostic scan, 24 bus-0 ECUs answered `0xF195`.

UDS request addresses seen answering on this car. These six returned a firmware version, and are the set
this port puts in `FW_VERSIONS`:

| Address | Module |
|---|---|
| `0x704` | MPC camera |
| `0x782` | Brake / IPB |
| `0x783` | EPS |
| `0x7f2` | Forward radar |
| `0x7E0` | Drive unit |
| `0x7F1` | SRS airbag |

**The `0xF195` payload is 6 raw bytes, not ASCII.** It holds date and revision fields. Set
`fw_version_regex=rb"[\x00-\xff]{6}"` in the FW query config, or the match drops every answer.

The MPC camera (`0x704`) does not answer the openpilot firmware query on the car. Make that ECU
non-essential, or the fingerprint match fails.

Take ignition from CAN. `DRIVE_STATE` (`0x242`) carries the gear, and any valid gear means ignition on.
openpilot stays offroad without a CAN ignition hook.

---

## 3. Reading vehicle state

| Signal | Message | Notes |
|---|---|---|
| Steering angle | `STEER_MODULE_2` `0x11F` | `STEER_ANGLE_2`, 0.1 deg/LSB |
| **Driver torque** | **`STEERING_TORQUE` `0x1FC`** | **`DRIVER_TORQUE`, 0.1 Nm/LSB. Use this for override — see below** |
| EPS motor effort | `STEERING_TORQUE` `0x1FC` | `MAIN_TORQUE`. This is the motor, not the driver |
| EPS state | `STEERING_TORQUE` `0x1FC` | See below |
| Vehicle speed | `WHEELSPEED_CLEAN` `0x1F0` | See below |
| Per-wheel speed | `WHEEL_SPEED` `0x122` | `WHEELSPEED_BR` is 14 bits. Bit 62 is a validity flag |
| Gear | `DRIVE_STATE` `0x242` | Also the ignition source |
| Brake pressed | `DRIVE_STATE` `0x242` | Debounce this signal |
| Accelerator | `PEDAL` `0x342` | `GAS_PEDAL`. See below |
| Blinkers | `STALKS` `0x133` | |
| Buttons | `PCM_BUTTONS` `0x3B0` | SET, RES, LKAS, distance, ACC-on |
| Blind spot | `BSD_RADAR` `0x418` | |
| Cruise state | `ACC_HUD_ADAS` `0x32D` | Camera bus. `ACC_STATE`: 2 available, 3 and 5 enabled, 7 error |

### Driver override

**Take the driver torque from `0x1FC STEERING_TORQUE.DRIVER_TORQUE`, at 0.1 Nm/LSB.**

The car masks `STEER_MODULE_2.DRIVER_EPS_TORQUE` (`0x11F`) while the EPS is in state 10, which is the window
openpilot is steering. That signal therefore cannot carry override. Log it if you want it. Do not drive
control off it.

Threshold used here: **18 Nm**. `DRIVER_TORQUE` runs a median of 1.3 Nm while engaged, so 18 Nm clears
hands-on noise and still catches a sustained fight. Fit your own threshold.

Sign convention, measured on manual turns: **positive is left**. Left turns read +17.9 Nm median, right
turns −16.6 Nm.

**Do not set `steeringDisengage` on this car.** It is a NO_ENTRY plus an immediate user disable.
`steeringPressed` is the correct response: openpilot yields lateral to the driver and resumes when the
driver relaxes.

### EPS state

**Read the EPS state from `0x1FC`.** `0x318` is constant `0xFF` and holds no valid state.

The low nibble decodes as bits, not as an enum: `LKS_PREPARED` is bit 0, `CRUISE_ACTIVATED` is bit 1, and
bit 3 is constant 1 in every capture.

| Nibble | Bits | Meaning |
|---|---|---|
| 8 | — | Off |
| 9 | PREPARED | Transitional. Also seen at standstill |
| 10 | ACTIVATED | Actuating |
| 11 | PREPARED + ACTIVATED | Dropout signature |

**Debounce state 11 for about 0.5 s.** State 11 is transitional and is not always a fault. It occurs for 100
to 200 ms when the `0x1E2` source changes, and when the stream has a short gap. This includes the engage
handover. Without the debounce, the `steerTempUnavailable` NO_ENTRY prevents engagement, because state 11
starts before openpilot transmits. A true latched dropout stays set. Section 5 gives the fault itself.

### Pedals and speed

**Take `gasPressed` from `PEDAL.GAS_PEDAL` (`0x342`).** `DRIVE_STATE.RAW_THROTTLE` is a different quantity
and does not follow the pedal. Longitudinal control does not engage if you use `RAW_THROTTLE`.

**Read `brakePressed` from the same signal the panda reads.** Both sides here read
`DRIVE_STATE.BRAKE_PRESSED` (`0x242`). If the two signals differ, a light touch on the brake clears one
engage latch and not the other. openpilot then shows "Controls Mismatch".

**Vehicle speed.** `WHEELSPEED_CLEAN` (`0x1F0`) is a 16-bit raw count. The scale is **0.0713 km/h per
count**. The DBC in `port/` has this scale.

The panda safety define keeps **0.0758**. This value is intentional. Safety uses it only to gate the >3 m/s
angle-error check, and the value stays the same as the validated build. Do not change it to match the DBC
before you run the safety tests again.

**Steer ratio 19.8.** The steer ratio and the speed scale above are a pair. Fit both again if you change
one.

---

## 4. Checksum

Add the BYD checksum to every message you transmit. The checksum is the last byte. Invert the sum of the
first 7 bytes:

```python
data[7] = (~sum(data[0:7])) & 0xFF
```

The camera signs 12 chassis messages with it. openpilot transmits three of them: `0x1E2`, `0x316`, and
`0x32E`.

---

## 5. Lateral control — `0x1E2 STEERING_MODULE_ADAS`

`0x1E2` carries the steering angle command. The panda blocks the camera `0x1E2` on the camera bus, and
openpilot sends its own `0x1E2` on bus 0.

- Send at 50 Hz, which is every second frame at 100 Hz. The stock camera sends at 33 Hz.
- `STEER_ANGLE` is at `24|16@1-`, 0.1 deg/LSB. `STEER_REQ` is at bit 21.
- `ANGLE_RATE_LIMIT_UPPER` (`0|10@1-`) and `ANGLE_RATE_LIMIT_LOWER` (`10|10@1-`) are the rate limits. The
  stock range seen here is 0 to +251 for UPPER and 0 to −252 for LOWER. This port sends the same values.
- `E2E_ALIVE_1` and `E2E_ALIVE_2` are both constant 1.
- Add a rolling `COUNTER`, then the section-4 checksum.

**Send `0x1E2` continuously while openpilot is enabled, including at standstill.** The EPS latches state 11
if the message stops at standstill. `STEER_REQ = 1` alone does not clear this state. Send idle frames with
`STEER_REQ = 0` instead of stopping transmission.

Report `0x1FC` state 11 as `steerFaultTemporary`, **after the ~0.5 s debounce in section 3**.

### EPS limits

The EPS does not hold every angle openpilot commands. In a tight turn the EPS stops to follow the command.
The EPS then leaves lane keeping and returns the wheel to the driver in the middle of the corner. openpilot
shows "Turn Exceeds Steering Limit", then "Steering Assist Temporarily Unavailable". One event causes both
alerts.

**The EPS reports the condition about 0.5 s early, on a 2-bit field at `0x1FC` bits 46–47.** The measured
sequence is the same each time:

1. The field changes from 0 to 1.
2. The EPS holds the `TARGET_ANGLE` it echoes on `0x1FC` at a constant value, and `MAIN_TORQUE` decreases.
   The wheel stops. openpilot continues to increase its command, to as much as 52° past the held target.
3. About 0.5 s later the EPS sets `TARGET_ANGLE` to 0 and latches state 11. Steering assist stops.

Across 14 routes the field showed value 1 **nine times. At each one the wheel angle was 148° to 244° and the
speed was 14 to 24 km/h.** The field showed value 2 thirteen times. Eleven of those were within 15° of
straight ahead, and most had zero motor effort. Value 2 is a different condition with no known effect.

While openpilot steers, `MAIN_TORQUE` stays in the range −252 to +251. This is the same window the port
declares in `0x1E2`. It is not established that the declaration sets the limit.

The boundary is **not linear with speed**. A fit of the nine points is closest to a constant yaw rate:

| Candidate invariant | Value at the limit | Spread |
|---|---|---|
| Wheel angle only | 177° | 16 % |
| **Angle × speed (yaw rate)** | **19.1 deg/s** | **8.2 %** |
| Angle × speed² (lateral acceleration) | 1.94 m/s² | 22 % |

A log-log fit gives angle proportional to v^−0.73. Use this rule: **maximum wheel angle = 3580 / speed in
km/h**. All nine points are between 14 and 24 km/h. Do not apply the exponent outside that range.

Two results apply to a port. First, the openpilot limit is higher than the car limit. The vehicle-model
limiter in `values.py` permits about 3.6 m/s² of lateral acceleration, which is the ISO value plus an
allowance for road bank. The EPS stops near 1.3 to 2.5 m/s². openpilot therefore commands turns that this
EPS does not make. Second, openpilot does not read the field. The command continues past a target that the
EPS refused, and the driver receives that step at the handover.

**Name of the field.** `byd_atto3.dbc` calls the field `TORQUE_TEMP_FAILED`, and labels the values
`1 "TooLarge"` and `2 "TooFast"`. That label comes from opendbc PR #3337, an unmerged draft. No BYD
document, no EPS diagnostic code, and no other fork confirms it. The measurements above do not depend on the
label. The meaning of the two values does. Treat the name as unconfirmed.

---

## 6. Longitudinal control — `0x32E ACC_CMD`

In this setup the stock LKAS stays in the off state, so openpilot is the only sender of longitudinal
commands to the car.

- `ACCEL_CMD` is at `0|8@1+`. The physical value is `raw × 0.05 − 5` m/s². Negative is brake or regen.
- `STANDSTILL_STATE` and `STANDSTILL_RESUME` control stop and hold. Pulse `STANDSTILL_RESUME` at pull-away.
  Without the pulse, the ESC releases the hold on its own timing and the launch is late.
- Pass the camera AEB bits in `0x32E` through. Do not overwrite them. This keeps the camera AEB command on
  the bus.

openpilot engages without the MVS4 in the loop. Use the LKAS button as the engage input with the stock ACC
off. With `pcmCruise = False` openpilot engages only on `buttonEnable`, so map the button latch onto that
event. Do not assign `ret.buttonEnable` inside `update()` — `interfaces.py` overwrites it afterwards, and
the write is dead.

**openpilot 0.11.2 has no `vEgoStopping`, `vEgoStarting`, `startAccel` or `startingState`.** `longcontrol`
does not enter a starting state, so there is no launch kick. Move-off uses `CC.cruiseControl.resume`, the
launch snap-to-zero in the car controller, and a launch jerk limit. 0.11.2 also has no
`longitudinalTuning.kpBP` or `kpV`. Every car builds its PID with kp = 0. The `0x32E` command is a
feedforward request, and the car's IPB closes its own loop with about 0.5 s of lag. Use kf = 1, and let ki
trim the model error.

---

## 7. HUD — `0x316 LKAS_HUD_ADAS`

Send `0x316` so the cluster continues to show the LKAS state while openpilot drives. Copy the stock camera
fields through and override only the LKAS state fields. The frame needs a `COUNTER` and the section-4
checksum.

The ADAS modules check the full bit pattern of this frame. A mismatch puts them in fail-safe and sets
latched DTCs in eight modules. The DBC in `port/` names the unnamed bits `PT1` to `PT4`, so a decode and a
repack give the same bytes. An unnamed bit packs as 0, which is sufficient to fail the check.

---

## 8. Radar

The private CAN carries the radar object tracks on `0x280` to `0x289`. This port decodes them and sends them
to `radard`.

The object tracks are on the private CAN only. The chassis CAN carries a lead flag and the accel command.
Read the private CAN to get tracks. PR #3337 sets `radarUnavailable = True`.

**`LONG_DIST` is measured from the ego rear-axle centre, not from the sensor face.** It is the ISO 8855
vehicle frame. openpilot's `dRel` must be measured from the front bumper, so subtract the car's geometry:

```python
BUMPER_TO_RADAR_ORIGIN_M = 3.60   # front overhang 885 mm + wheelbase 2720 mm
dRel = LONG_DIST - BUMPER_TO_RADAR_ORIGIN_M
```

Without the correction, each radar lead reads about 3.6 m too far. Measurement against four leads of known
body width gave errors of +3.21, +4.11, +2.04 and +3.62 m — a Corolla taxi, a CR-V, a Yaris-class hatch and
a box truck. The vision lead stayed within 0.1 to 0.8 m of the same geometry. The error on the cars is the
same as the error on the truck. This shows the sensor origin, not a reflection point on the target. Over
7863 radar-matched frames the radar-to-vision difference changed from **+4.18 m to +0.43 m** in the 0 to
12 m stopping band.

The correction has two intentional effects. The maximum-range gate also removes negative ranges, because a
target at the bumper reads about 3.6 raw. The cut-in window becomes a true 20 m of road, instead of the
16.4 m it was before.

`veoneer_tracks.py` keeps raw values. It is the decode reference that the bench replay validator scores, so
its numbers must stay comparable with the validated report. The conversion is in `radar_interface.py`.

---

## 9. Road camera — webcam

openpilot has a `USE_WEBCAM` path that takes a V4L2 camera in place of the comma camera stack. The points
below are what a webcam cost here.

- openpilot ships the comma camera intrinsics (`f = 2648` at 1928×1208). Your camera has a different focal
  length. Measure it and set it, or the model projects the road wrong. A wall test is enough: capture a target
  of known width at a known distance and derive the focal in pixels.
- A focal change invalidates the learned calibration. Clear the calibration parameters after you change it.
- Aim the camera mechanically before you rely on calibration. Calibration corrects a small residual, not a
  large mount error.
- Confirm the frame rate is steady at 20 Hz in real driving light, not on the bench in daylight.
- **Read the V4L2 control defaults of your own module and set the ones that matter.** The defaults differ
  per module and a bad one is silent: you get a dark image or a low frame rate, not an error. Auto controls
  that trade frame rate for exposure are the usual offender, because they only engage at night. The values
  that fix one module do not carry to another. Dump your own controls and compare them against a capture
  you know is good.
- **Apply the control values on every camera open, not only at launch.** A USB re-enumeration resets the
  module to its vendor defaults mid-drive, and a launch-time setting does not come back. The `camera.py` in
  `port/pc/pc-host.patch` does this.

---

## 10. IMU — bridged from CAN

A comma device supplies an IMU. This build runs on a PC that has none, so `sensord` publishes nothing and
`locationd` / `livePose` degrade. Bridge the car's own inertial sensors instead. Neither message is in the
#3337 DBC — add both.

| Message | Decode |
|---|---|
| `YAW_RATE` `0x222` | `YawRate` `0\|12@1+` × 0.002133 − 2.094 rad/s. `YawRateOffset` `12\|12@1+` × 0.002133 − 0.13. Subtract the offset from the rate. |
| `AXAY` `0x223` | `Ax` `0\|12@1+` and `Ay` `24\|12@1+`, × 0.027167 and × 0.027127 m/s². The offset fields at `12\|12` and `36\|12` share the scale and the bias, so subtract them raw before you scale. |

Publish these as `gyroscope` and `accelerometer` `SensorEventData`.

- The car frame is ISO. Yaw and `Ay` are positive to the left. The device frame is x forward, y right, z down.
  `Ay` needs a sign flip. Yaw and `Ax` do not.
- The accelerometer must carry gravity. There is no vertical sensor, so send a constant `az = −9.81`.
- Do not report the source as `bmx055`. locationd discards it.
- The rate is 50 Hz, against about 104 Hz from a device IMU.
- Roll and pitch stay weakly observable without vertical accel. Yaw and the planar dynamics are complete,
  which is what lateral control and longitudinal control need.

Validate the three signs against your own drive log before you drive on them. Check yaw against steering
angle, `Ax` against d(speed)/dt, and `Ay` against v × yaw. locationd cross-checks yaw against camera odometry and
rejects a wrong sign, so a flipped yaw shows up as `livePose` never becoming valid.

Take `yawRate` and `aEgo` for carState from the same two messages.

---

## 11. Panda and safety

- TX allowlist: `0x1E2`, `0x316`, `0x32E`.
- The safety mode goes at `opendbc/safety/modes/byd.h`. Put `SAFETY_BYD` and the `byd_hooks` extern in
  `opendbc/safety/declarations.h`, not in `safety.h`.
- The `{SAFETY_BYD, &byd_hooks}` entry is inside `#ifdef ALLOW_DEBUG`. Build the panda with `ALLOW_DEBUG`,
  or move the entry out of that block. If you move it, first find out why it is there.
- Set the angle limits and accel limits from your own measurements. Do not inherit limits from another
  BYD platform, and do not inherit the ones here.

---

## 12. Applicability

All data comes from one car: a 2024 RHD export Atto 3. Check these points before you apply the data to a
different car.

- Chinese-market builds and other model years can differ.
- BYD used two camera suppliers, Veoneer and Bosch, with different connectors. Measure your connector.
- The splice has no relay. openpilot replaces the stock LKS, ACC, and AEB functions while it runs. A harness
  box with a passthrough relay is the correct build.

---

## The port files

[`port/`](port/) holds the code that implements this document: the files that do not exist in an openpilot
release, and a patch for the places an existing file has to change. There is no openpilot tree in it — copy
the files into your own. [`port/README.md`](port/README.md) lists what each file does and where it goes.

The files target released openpilot **0.11.2**. Upstream anything here into opendbc.
