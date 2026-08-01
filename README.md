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

[MIT licensed](LICENSE). Not affiliated with BYD or comma.ai — see [DISCLAIMER.md](DISCLAIMER.md).

| | |
|---|---|
| Vehicle | BYD Atto 3 / Yuan Plus EV, 2024, RHD export build |
| VIN WMI | `LGX` |
| ADAS camera | Veoneer MVS4 — Veoneer P/N `682212300`, BYD P/N `SC2EM-3619100A` |
| Camera FW | HW 0.0.4, SW 4.01.04 |
| Forward radar | Veoneer 77V12FLR |
| Mass / wheelbase | 1750 kg / 2.72 m |
| Steer ratio | 19.5 |

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

**Private CAN** (pins 3/7) is CAN-FD with a data phase near 2 Mbit/s. The bus has two nodes: the MVS4 camera
and the 77V12FLR radar. No other ECU is on this bus. Use this bus to read radar tracks (section 8).

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

The MPC camera (`0x704`) does not answer the openpilot firmware query on the car. Make that ECU
non-essential, or the fingerprint match fails.

Take ignition from CAN. `DRIVE_STATE` (`0x242`) carries the gear, and any valid gear means ignition on.
openpilot stays offroad without a CAN ignition hook.

---

## 3. Reading vehicle state

| Signal | Message | Notes |
|---|---|---|
| Steering angle | `STEER_MODULE_2` `0x11F` | `STEER_ANGLE_2`, 0.1 deg/LSB |
| Driver EPS torque | `STEER_MODULE_2` `0x11F` | `DRIVER_EPS_TORQUE` |
| EPS state | `STEERING_TORQUE` `0x1FC` | See below |
| Vehicle speed | `WHEELSPEED_CLEAN` `0x1F0` | Raw wheel speed, 0.1 km/h per LSB |
| Per-wheel speed | `WHEEL_SPEED` `0x122` | `WHEELSPEED_BR` is 14 bits. Bit 62 is a validity flag |
| Gear | `DRIVE_STATE` `0x242` | Also the ignition source |
| Brake pressed | `DRIVE_STATE` `0x242` | Debounce this signal |
| Accelerator | `PEDAL` `0x342` | `GAS_PEDAL`. See below |
| Blinkers | `STALKS` `0x133` | |
| Buttons | `PCM_BUTTONS` `0x3B0` | SET, RES, LKAS, distance, ACC-on |
| Blind spot | `BSD_RADAR` `0x418` | |
| Cruise state | `ACC_HUD_ADAS` `0x32D` | Camera bus. `ACC_STATE`: 2 available, 3 and 5 enabled, 7 error |

**Read the EPS state from `0x1FC`.** `0x318` is constant `0xFF` and holds no valid state. State 10 is
actuating. State 11 is a latched fault. State 9 is transitional at standstill.

**Take `gasPressed` from `PEDAL.GAS_PEDAL` (`0x342`).** `DRIVE_STATE.RAW_THROTTLE` is a different quantity
and does not follow the pedal. Longitudinal control does not engage if you use `RAW_THROTTLE`.

**Vehicle speed.** `WHEELSPEED_CLEAN` is raw wheel speed at 0.1 km/h per LSB. On this car the cluster
displays `WHEELSPEED_CLEAN × 0.758`. Fit the scale on your own car.

**Steer ratio 19.5** is what this car settled on. It is fitted against the speed scale above, so the two
are a pair — refit both if you change either.

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
  stock range seen here is 0 to +251 for UPPER and 0 to −252 for LOWER.
- `E2E_ALIVE_1` and `E2E_ALIVE_2` are both constant 1.
- Add a rolling `COUNTER`, then the section-4 checksum.

**Send `0x1E2` continuously while openpilot is enabled, including at standstill.** The EPS latches state 11
if the message stops at standstill. `STEER_REQ = 1` alone does not clear this state. Send idle frames with
`STEER_REQ = 0` instead of stopping transmission.

Report `0x1FC` state 11 as `steerFaultTemporary`.

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
off.

---

## 7. HUD — `0x316 LKAS_HUD_ADAS`

Send `0x316` so the cluster continues to show the LKAS state while openpilot drives. Copy the stock camera
fields through and override only the LKAS state fields. The frame needs a `COUNTER` and the section-4
checksum.

---

## 8. Radar

The private CAN carries the radar object tracks on `0x280` to `0x289`. This port decodes them and sends them
to `radard`.

The object tracks are on the private CAN only. The chassis CAN carries a lead flag and the accel command.
Read the private CAN to get tracks. PR #3337 sets `radarUnavailable = True`.

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
- Set the angle limits and accel limits from your own measurements. Do not inherit limits from another
  BYD platform, and do not inherit the ones here.

---

## 12. Applicability

All data comes from one car: a 2024 RHD export Atto 3. Check these points before you apply the data to a
different car.

- Chinese-market builds and other model years can differ.
- BYD used two camera suppliers, Veoneer and Bosch, with different connectors. Measure your connector.
- This port runs on a `carrotpilot` fork.
- The splice has no relay. openpilot replaces the stock LKS, ACC, and AEB functions while it runs. A harness
  box with a passthrough relay is the correct build.

---

## The port files

[`port/`](port/) holds the code that implements this document: the twelve files that do not exist in an
openpilot release, and a patch for the ten places an existing file has to change. There is no openpilot tree
in it — copy the files into your own. [`port/README.md`](port/README.md) lists what each file does and where
it goes.

The files come from a `carrotpilot` fork and call carrot APIs in a few places, so on comma's openpilot they
need porting rather than copying. Upstream anything here into opendbc.
