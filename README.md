# BYD Atto 3 — openpilot porting reference

Data to port openpilot to a BYD Atto 3 (Yuan Plus EV): wiring, bus topology, signals, control messages and
panda requirements.

> **No warranty. No claims. Use at your own risk.** This is a personal notebook from one car. Nothing here is
> claimed to be correct, complete, or safe. It concerns the steering, braking, and acceleration of a moving
> vehicle, where an error can have serious consequences. Verify everything yourself.
> **Read [DISCLAIMER.md](DISCLAIMER.md) before you use any of it.**

All data comes from one car, a 2024 RHD export build.

https://github.com/user-attachments/assets/0a8011b6-2d72-4bb8-8523-d18c12993d65

This port runs on an x86 mini-PC with a USB webcam as the road camera. A comma red panda, spliced inline at
the camera connector, is the CAN interface. There is no comma device in the car. Two consequences get their
own sections: the webcam (section 9) and the IMU bridged from CAN (section 10).

**The port runs on released openpilot 0.11.2, branch `byd-atto3`.** The files in [`port/`](port/) use
upstream APIs only.

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

Interpose at the P13 camera connector. It has 8 cavities. 6 are wired.

| Pin | Signal | Wire |
|---:|---|---|
| 1 | Camera GND | black |
| 2 | IG1 (switched 12 V) | red/grey |
| 3 | Private CAN_H | pink |
| 4 | Chassis CAN_H | pink |
| 7 | Private CAN_L | violet |
| 8 | Chassis CAN_L | violet |

Source: the BYD *Definition of Terminals* document.

Identify the bus by pin position. Both CAN_H wires are pink and both CAN_L wires are violet.

**Chassis CAN** (pins 4/8): classic CAN, 500 kbit/s, 8-byte frames. openpilot reads and writes this bus.
Sections 2 to 7 refer to it.

**Private CAN** (pins 3/7): CAN-FD, data phase near 2 Mbit/s. It carries the MVS4 camera, the 77V12FLR
radar, and about 72 classic-format CAN IDs of unestablished source. Read radar tracks here (section 8).

Add termination to the chassis CAN at the panda. The panda breaks the bus at the camera connector.

---

## 2. Car identification

Query firmware versions with `0xF195` (SUPPLIER_SOFTWARE_VERSION). BYD ECUs reject `0xF188`, the openpilot
default. 24 bus-0 ECUs answered `0xF195`.

These six return a firmware version and are the set this port puts in `FW_VERSIONS`:

| Address | Module |
|---|---|
| `0x704` | MPC camera |
| `0x782` | Brake / IPB |
| `0x783` | EPS |
| `0x7f2` | Forward radar |
| `0x7E0` | Drive unit |
| `0x7F1` | SRS airbag |

Set `fw_version_regex=rb"[\x00-\xff]{6}"`. The `0xF195` payload is 6 raw bytes, not ASCII.

Make the MPC camera (`0x704`) non-essential. It does not answer the openpilot firmware query.

Take ignition from CAN. Any valid gear in `DRIVE_STATE` (`0x242`) means ignition on.

---

## 3. Reading vehicle state

| Signal | Message | Notes |
|---|---|---|
| Steering angle | `STEER_MODULE_2` `0x11F` | `STEER_ANGLE_2`, 0.1 deg/LSB |
| **Driver torque** | **`STEERING_TORQUE` `0x1FC`** | **`DRIVER_TORQUE`, 0.1 Nm/LSB. Use this for override** |
| EPS motor effort | `STEERING_TORQUE` `0x1FC` | `MAIN_TORQUE`. The motor, not the driver |
| EPS state | `STEERING_TORQUE` `0x1FC` | See below |
| Vehicle speed | `WHEELSPEED_CLEAN` `0x1F0` | 16-bit raw count, 0.0713 km/h per count |
| Per-wheel speed | `WHEEL_SPEED` `0x122` | `WHEELSPEED_BR` is 14 bits. Bit 62 is a validity flag |
| Gear | `DRIVE_STATE` `0x242` | Also the ignition source |
| Brake pressed | `DRIVE_STATE` `0x242` | `BRAKE_PRESSED`, debounced |
| Accelerator | `PEDAL` `0x342` | `GAS_PEDAL` |
| Blinkers | `STALKS` `0x133` | |
| Buttons | `PCM_BUTTONS` `0x3B0` | SET, RES, LKAS, distance, ACC-on |
| Blind spot | `BSD_RADAR` `0x418` | |
| Cruise state | `ACC_HUD_ADAS` `0x32D` | Camera bus. `ACC_STATE`: 2 available, 3 and 5 enabled, 7 error |

### Driver override

Take the driver torque from `0x1FC STEERING_TORQUE.DRIVER_TORQUE`, 0.1 Nm/LSB. Threshold: **18 Nm**.
Median while engaged is 1.3 Nm. Fit your own threshold.

Sign: **positive is left**. Manual turns read +17.9 Nm left, −16.6 Nm right.

Do not use `STEER_MODULE_2.DRIVER_EPS_TORQUE` for override. The car masks it while the EPS steers.

Do not set `steeringDisengage`. Use `steeringPressed`.

### EPS state

Read the EPS state from `0x1FC`. `0x318` is constant `0xFF`.

The low nibble is bits, not an enum. `LKS_PREPARED` is bit 0, `CRUISE_ACTIVATED` is bit 1, bit 3 is
constant 1.

| Nibble | Bits | Meaning |
|---|---|---|
| 8 | — | Off |
| 9 | PREPARED | Transitional. Also at standstill |
| 10 | ACTIVATED | Actuating |
| 11 | PREPARED + ACTIVATED | Dropout |

Debounce state 11 for **0.5 s**. It is transitional for 100 to 200 ms at each engage handover. Without the
debounce, `steerTempUnavailable` prevents engagement.

### Pedals and speed

Take `gasPressed` from `PEDAL.GAS_PEDAL` (`0x342`). Do not use `DRIVE_STATE.RAW_THROTTLE`.

Read `brakePressed` from `DRIVE_STATE.BRAKE_PRESSED` (`0x242`), the same signal the panda reads. A
different signal desynchronizes the two engage latches and gives "Controls Mismatch".

Scale `WHEELSPEED_CLEAN` at **0.0713 km/h per count**, as in the DBC in `port/`.

Keep the panda safety define at **0.0758**. Re-run the safety tests before you change it.

Steer ratio **19.8**. It pairs with the speed scale. Fit both again if you change one.

---

## 4. Checksum

Add the BYD checksum to every message you transmit. It is the last byte:

```python
data[7] = (~sum(data[0:7])) & 0xFF
```

The camera signs 12 chassis messages. openpilot transmits three: `0x1E2`, `0x316`, `0x32E`.

---

## 5. Lateral control — `0x1E2 STEERING_MODULE_ADAS`

`0x1E2` carries the steering angle command. The panda blocks the camera `0x1E2`. openpilot sends its own on
bus 0.

- Send at 50 Hz, every second frame at 100 Hz. The stock camera sends at 33 Hz.
- `STEER_ANGLE` is `24|16@1-`, 0.1 deg/LSB. `STEER_REQ` is bit 21.
- `ANGLE_RATE_LIMIT_UPPER` (`0|10@1-`) and `ANGLE_RATE_LIMIT_LOWER` (`10|10@1-`) are the rate limits. Stock
  range is 0 to +251 and 0 to −252. This port sends the same.
- `E2E_ALIVE_1` and `E2E_ALIVE_2` are constant 1.
- Add a rolling `COUNTER`, then the section-4 checksum.

Send `0x1E2` continuously while openpilot is enabled, including at standstill. Send idle `STEER_REQ = 0`
frames instead of stopping. A gap latches EPS state 11, and `STEER_REQ = 1` alone does not clear it.

Report `0x1FC` state 11 as `steerFaultTemporary`, after the 0.5 s debounce in section 3.

### EPS limits

The EPS does not hold every angle openpilot commands. In a tight turn it stops lane keeping and returns the
wheel to the driver mid-corner. openpilot shows "Turn Exceeds Steering Limit", then "Steering Assist
Temporarily Unavailable". One event causes both alerts.

A 2-bit field at `0x1FC` bits 46–47 reports the condition 0.5 s early. The sequence is the same each time:

1. The field changes from 0 to 1.
2. The EPS holds its echoed `TARGET_ANGLE` constant and `MAIN_TORQUE` decreases. The wheel stops. openpilot
   continues to as much as 52° past the held target.
3. After 0.5 s the EPS sets `TARGET_ANGLE` to 0 and latches state 11.

Across 14 routes the field showed value 1 **nine times, each at 148° to 244° of wheel angle and 14 to
24 km/h**. Value 2 occurred thirteen times, eleven within 15° of straight ahead, with no known effect.

`MAIN_TORQUE` stays in the range −252 to +251 while openpilot steers. This is the window the port declares
in `0x1E2`. It is not established that the declaration sets the limit.

The limit is not linear with speed. A fit of the nine points is closest to a constant yaw rate:

| Candidate invariant | Value at the limit | Spread |
|---|---|---|
| Wheel angle only | 177° | 16 % |
| **Angle × speed (yaw rate)** | **19.1 deg/s** | **8.2 %** |
| Angle × speed² (lateral acceleration) | 1.94 m/s² | 22 % |

Use this rule: **maximum wheel angle = 3580 / speed in km/h**. A log-log fit gives angle proportional to
v^−0.73. Do not apply it outside 14 to 24 km/h.

Two results apply to a port. The vehicle-model limiter in `values.py` permits about 3.6 m/s² of lateral
acceleration, but the EPS stops near 1.3 to 2.5 m/s², so openpilot commands turns this EPS does not make.
And openpilot does not read the field, so the command continues past a refused target and the driver
receives that step at the handover.

`byd_atto3.dbc` calls the field `TORQUE_TEMP_FAILED` with values `1 "TooLarge"` and `2 "TooFast"`. That
label comes from opendbc PR #3337, an unmerged draft. No BYD document, no EPS diagnostic code and no other
fork confirms it. Treat the name as unconfirmed. The measurements above do not depend on it.

---

## 6. Longitudinal control — `0x32E ACC_CMD`

The stock LKAS stays off, so openpilot is the only sender of longitudinal commands.

- `ACCEL_CMD` is `0|8@1+`. Physical value is `raw × 0.05 − 5` m/s². Negative is brake or regen.
- `STANDSTILL_STATE` and `STANDSTILL_RESUME` control stop and hold. Pulse `STANDSTILL_RESUME` at pull-away,
  or the launch is late.
- Pass the camera AEB bits in `0x32E` through unchanged.

Use the LKAS button as the engage input with the stock ACC off. Set `pcmCruise = False` and map the button
latch onto `buttonEnable`. Do not assign `ret.buttonEnable` inside `update()`; `interfaces.py` overwrites
it.

openpilot 0.11.2 has no `vEgoStopping`, `vEgoStarting`, `startAccel`, `startingState`,
`longitudinalTuning.kpBP` or `kpV`. There is no launch kick. Move-off uses `CC.cruiseControl.resume`, the
launch snap-to-zero, and a launch jerk limit. `0x32E` is a feedforward request and the IPB closes its own
loop with 0.5 s of lag, so use kf = 1 and let ki trim.

---

## 7. HUD — `0x316 LKAS_HUD_ADAS`

Send `0x316` so the cluster shows the LKAS state while openpilot drives. Copy the stock camera fields
through and override only the LKAS state fields. Add a `COUNTER` and the section-4 checksum.

Name every bit in the frame. The DBC in `port/` adds `PT1` to `PT4` for this. The ADAS modules check the
full bit pattern, and an unnamed bit packs as 0, which sets latched DTCs in eight modules.

---

## 8. Radar

Read the object tracks from the private CAN, `0x280` to `0x289`. The chassis CAN carries only a lead flag
and the accel command. PR #3337 sets `radarUnavailable = True`.

Subtract the car geometry from `LONG_DIST`. It is measured from the ego rear-axle centre, the ISO 8855
vehicle frame, not from the sensor face:

```python
BUMPER_TO_RADAR_ORIGIN_M = 3.60   # front overhang 885 mm + wheelbase 2720 mm
dRel = LONG_DIST - BUMPER_TO_RADAR_ORIGIN_M
```

Without the correction each lead reads 3.6 m too far. Against four leads of known body width the raw range
read +3.21, +4.11, +2.04 and +3.62 m long, while vision stayed within 0.1 to 0.8 m. Cars and the truck read
equally long, which shows the sensor origin and not a reflection point. Over 7863 radar-matched frames the
radar-to-vision difference changed from **+4.18 m to +0.43 m** in the 0 to 12 m stopping band.

The correction also removes negative ranges from the maximum-range gate, and makes the cut-in window a true
20 m instead of 16.4 m.

`veoneer_tracks.py` keeps raw values, because the bench replay validator scores them. The conversion is in
`radar_interface.py`.

---

## 9. Road camera — webcam

openpilot has a `USE_WEBCAM` path that takes a V4L2 camera in place of the comma camera stack.

- Measure your camera focal length in pixels and set it. openpilot ships the comma intrinsics, `f = 2648` at
  1928×1208.
- Clear the calibration parameters after a focal change.
- Aim the camera mechanically before you rely on calibration.
- Confirm the frame rate is steady at 20 Hz in real driving conditions.
- Set V4L2 control defaults for clear image capture in all driving conditions.
- Apply the V4L2 controls on every camera open, not only at launch. A USB re-enumeration resets the module.

---

## 10. IMU — bridged from CAN

Bridge the inertial sensors of the car. A PC has no IMU, so `sensord` publishes nothing and `locationd` degrades.
Neither message is in the #3337 DBC.

| Message | Decode |
|---|---|
| `YAW_RATE` `0x222` | `YawRate` `0\|12@1+` × 0.002133 − 2.094 rad/s. `YawRateOffset` `12\|12@1+` × 0.002133 − 0.13. Subtract the offset from the rate. |
| `AXAY` `0x223` | `Ax` `0\|12@1+` and `Ay` `24\|12@1+`, × 0.027167 and × 0.027127 m/s². The offset fields at `12\|12` and `36\|12` share the scale and the bias, so subtract them raw before you scale. |

Publish these as `gyroscope` and `accelerometer` `SensorEventData`.

- Invert the sign of `Ay`. The car frame is ISO, positive left. The device frame is x forward, y right,
  z down. Yaw and `Ax` need no change.
- Send a constant `az = −9.81`. There is no vertical sensor and the accelerometer must carry gravity.
- Do not report the source as `bmx055`. locationd discards it.
- The rate is 50 Hz, against about 104 Hz from a device IMU.
- Expect weak roll and pitch. Yaw and the planar dynamics are complete, which is what the controls need.
- Take `yawRate` and `aEgo` for carState from the same two messages.

Validate the three signs against your own drive log before you drive on them: yaw against steering angle,
`Ax` against d(speed)/dt, `Ay` against v × yaw. A flipped yaw shows up as `livePose` never becoming valid.

---

## 11. Panda and safety

- TX allowlist: `0x1E2`, `0x316`, `0x32E`.
- Put the safety mode at `opendbc/safety/modes/byd.h`. Put `SAFETY_BYD` and the `byd_hooks` extern in
  `opendbc/safety/declarations.h`, not in `safety.h`.
- Build the panda with `ALLOW_DEBUG`. The `{SAFETY_BYD, &byd_hooks}` entry is inside that block.
- Set the angle limits and accel limits from your own measurements. Do not inherit them from another BYD
  platform, and do not inherit the ones here.

---

## 12. Applicability

All data comes from one car: a 2024 RHD export Atto 3. Check these before you apply it to another car.

- Chinese-market builds and other model years can differ.
- Measure your connector. BYD used two camera suppliers, Veoneer and Bosch, with different connectors.
- The splice has no relay. openpilot replaces the stock LKS, ACC and AEB functions while it runs. A harness
  box with a passthrough relay is the correct build.

---

## The port files

[`port/`](port/) holds the code that implements this document: the files that do not exist in an openpilot
release, and a patch for the places an existing file has to change. Copy the files into your own tree.
[`port/README.md`](port/README.md) lists what each file does and where it goes.

The files target released openpilot **0.11.2**. Upstream anything here into opendbc.
