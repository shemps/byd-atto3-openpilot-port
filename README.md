# BYD Atto 3 — openpilot porting manual

This manual gives the procedure and the data to port openpilot to a BYD Atto 3 (Yuan Plus EV): wiring, bus
topology, signals, control messages, camera, IMU and panda safety.

> **No warranty. No claims. Use at your own risk.** This is a personal notebook from one car. Nothing here is
> claimed to be correct, complete, or safe. It concerns the steering, braking, and acceleration of a moving
> vehicle, where an error can have serious consequences. Verify everything yourself.
> **Read [DISCLAIMER.md](DISCLAIMER.md) before you use any of it.**

All data in this manual comes from one vehicle: a 2024 RHD export build. Read section 12 before you apply it
to another vehicle.

https://github.com/user-attachments/assets/0a8011b6-2d72-4bb8-8523-d18c12993d65

[MIT licensed](LICENSE). Not affiliated with BYD or comma.ai — see [DISCLAIMER.md](DISCLAIMER.md).

## Vehicle and equipment

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

This manual assumes the following equipment. Sections 9 and 10 cover the two items that a comma device would
otherwise supply.

- An x86 mini-PC as the host.
- A USB webcam as the road camera (section 9).
- A comma red panda, spliced inline at the camera connector, as the CAN interface.
- No comma device, therefore no IMU (section 10).

## Procedure

1. Splice the harness at the P13 camera connector (section 1).
2. Fingerprint the vehicle (section 2).
3. Decode the vehicle state (section 3).
4. Add the checksum to every transmitted message (section 4).
5. Add lateral control (section 5).
6. Add longitudinal control (section 6).
7. Add the HUD frame (section 7).
8. Add radar, if you want it (section 8).
9. Set up the road camera (section 9).
10. Bridge the IMU from CAN (section 10).
11. Build and flash the panda safety (section 11).

The code that implements this manual is in [`port/`](port/).

## Credits

This port is derived work. What it takes, and from whom:

| Source | Taken |
|---|---|
| [openpilot](https://github.com/commaai/openpilot) and [opendbc](https://github.com/commaai/opendbc) — comma.ai, MIT | The stack this port plugs into |
| [bukapilot](https://github.com/kommuai/bukapilot) — iXcess / KommuAI | The BYD chassis checksum in section 4. Their `byd_checksum` is a nibble scheme with key `0xAF`. The one-line form in section 4 is an algebraic simplification of it and gives the same byte |
| [opendbc PR #2429](https://github.com/commaai/opendbc/pull/2429) — dotslashofficial | The `opendbc/car/byd/` brand structure and `byd_general_pt.dbc`, which carried the bukapilot work into opendbc |
| [opendbc PR #3337](https://github.com/commaai/opendbc/pull/3337) — lukasloetkolben | `byd_atto3.dbc`, the chassis DBC in `port/`. Section 5 marks the one signal name in it that this manual treats as unconfirmed |
| [tembikaipilot](https://github.com/danielks23/tembikaipilot) — danielks23 | The Veoneer FD radar track layout in `byd_radar_fd.dbc`, section 8. Two fields are corrected here |
| [carrotpilot](https://github.com/ajouatom/openpilot) — ajouatom | The fork this port was developed on before openpilot 0.11.2 |
| [opendbc Subaru support](https://github.com/commaai/opendbc/tree/master/opendbc/car/subaru) — comma.ai | The lead to the private-bus checksum scheme, section 8. EyeSight uses a Veoneer camera |

Related BYD work, not used in this port:

- [opendbc PR #3352](https://github.com/commaai/opendbc/pull/3352) — martinl, BYD Sealion 7 port.
- [opendbc PR #2058](https://github.com/commaai/opendbc/pull/2058) — yysnet, initial BYD brand port.
- [opendbc issue #2065](https://github.com/commaai/opendbc/issues/2065) — qzwf, BYD Atto 3 harness. The
  pinout in section 1 is read from the BYD *Definition of Terminals* document, not from this thread.
- [opendbc issue #2017](https://github.com/commaai/opendbc/issues/2017) — the BYD port bounty.

Copyright in the borrowed portions stays with their authors under their own licences. See
[DISCLAIMER.md](DISCLAIMER.md). If anything here is attributed wrongly, open an issue and it will be
corrected.

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

## 2. Fingerprint the vehicle

Query firmware versions with `0xF195` (SUPPLIER_SOFTWARE_VERSION). BYD ECUs reject `0xF188`, the openpilot
default. 24 bus-0 ECUs answer `0xF195`. These six return a firmware version. Put them in `FW_VERSIONS`:

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

## 3. Decode the vehicle state

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

Read the driver torque from `0x1FC STEERING_TORQUE.DRIVER_TORQUE`, 0.1 Nm/LSB.

Set the override threshold to **18 Nm**. Hands-off torque reads about 1 Nm while engaged. Raise the
threshold if your vehicle reads higher.

The sign is **positive left**. Check it on a manual turn. A left turn gives about +18 Nm.

Do not use `STEER_MODULE_2.DRIVER_EPS_TORQUE` for override. The vehicle masks it while the EPS steers.

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

Debounce state 11 for **0.5 s**. State 11 is transitional for 100 to 200 ms at each engage handover.
Without the debounce, `steerTempUnavailable` prevents engagement.

### Pedals and speed

Read `gasPressed` from `PEDAL.GAS_PEDAL` (`0x342`). Do not use `DRIVE_STATE.RAW_THROTTLE`.

Read `brakePressed` from `DRIVE_STATE.BRAKE_PRESSED` (`0x242`), the same signal the panda reads. A
different signal desynchronizes the two engage latches and gives "Controls Mismatch".

Scale `WHEELSPEED_CLEAN` at **0.0713 km/h per count**.

Keep the panda safety define at **0.0758**. Run the safety tests again before you change it.

Use steer ratio **19.8**. It pairs with the speed scale. Fit both again if you change one.

---

## 4. Add the checksum

Add the BYD checksum to every message you transmit. It is the last byte:

```python
data[7] = (~sum(data[0:7])) & 0xFF
```

The camera signs 12 chassis messages. openpilot transmits three: `0x1E2`, `0x316`, `0x32E`.

---

## 5. Add lateral control — `0x1E2 STEERING_MODULE_ADAS`

`0x1E2` carries the steering angle command. Block the camera `0x1E2` at the panda. Send the openpilot
frame on bus 0.

- Send at 50 Hz, every second frame at 100 Hz. The stock camera sends at 33 Hz.
- `STEER_ANGLE` is `24|16@1-`, 0.1 deg/LSB. `STEER_REQ` is bit 21.
- Send `ANGLE_RATE_LIMIT_UPPER` (`0|10@1-`) and `ANGLE_RATE_LIMIT_LOWER` (`10|10@1-`) at the stock values,
  0 to +251 and 0 to −252.
- Set `E2E_ALIVE_1` and `E2E_ALIVE_2` to 1.
- Add a rolling `COUNTER`, then the section-4 checksum.

Send `0x1E2` continuously while openpilot is enabled, including at standstill. Send idle `STEER_REQ = 0`
frames instead of stopping. A gap latches EPS state 11, and `STEER_REQ = 1` alone does not clear it.

Report `0x1FC` state 11 as `steerFaultTemporary`, after the 0.5 s debounce in section 3.

### EPS limits

The EPS does not hold every angle openpilot commands. In a tight turn it stops lane keeping and returns the
wheel to the driver mid-corner. openpilot shows "Turn Exceeds Steering Limit", then "Steering Assist
Temporarily Unavailable". One event causes both alerts.

Limit the commanded wheel angle to **3580 / speed in km/h**, a constant yaw rate of 19 deg/s. Set this limit
and the EPS holds the commanded angle through a tight turn.

| Speed | Maximum wheel angle |
|---:|---:|
| 15 km/h | 239° |
| 20 km/h | 179° |
| 24 km/h | 149° |

The limit is established between 14 and 24 km/h only. Above 24 km/h, keep it and confirm the behaviour on
your own vehicle.

Do not use a lateral-acceleration limit for this. The vehicle-model limiter defaults to about 3.6 m/s²,
which is above what the EPS delivers, and lateral acceleration does not hold constant at the drop-out
point.

Read `0x1FC` bits 46–47. The field changes from 0 to 1 about 0.5 s before the EPS drops out. Use it to hand
back control early. Without it, the command continues to as much as 52° past the angle the EPS refused, and
the driver receives that step at the handover.

Drop-out sequence, for diagnosis:

1. The field changes from 0 to 1.
2. The EPS holds its echoed `TARGET_ANGLE` constant and `MAIN_TORQUE` decreases. The wheel stops.
3. After 0.5 s the EPS sets `TARGET_ANGLE` to 0 and latches state 11.

Value 2 of the field has no known effect. It occurs near straight ahead.

`byd_atto3.dbc` calls the field `TORQUE_TEMP_FAILED` with values `1 "TooLarge"` and `2 "TooFast"`. That
label comes from opendbc PR #3337, an unmerged draft. No BYD document, no EPS diagnostic code and no other
fork confirms it. Treat the name as unconfirmed. The limits above do not depend on it.

---

## 6. Add longitudinal control — `0x32E ACC_CMD`

With the stock LKAS off, openpilot is the only sender of longitudinal commands.

- `ACCEL_CMD` is `0|8@1+`. Physical value is `raw × 0.05 − 5` m/s². Negative is brake or regen.
- `STANDSTILL_STATE` and `STANDSTILL_RESUME` control stop and hold. Pulse `STANDSTILL_RESUME` at pull-away,
  or the launch is late.
- Pass the camera AEB bits in `0x32E` through unchanged.

Use the LKAS button as the engage input with the stock ACC off. Set `pcmCruise = False` and map the button
latch onto `buttonEnable`. Do not assign `ret.buttonEnable` inside `update()`. `interfaces.py` overwrites
it.

Set kf = 1 and let ki trim. `0x32E` is a feedforward request, and the IPB closes its own loop with 0.5 s of
lag.

openpilot 0.11.2 has no `vEgoStopping`, `vEgoStarting`, `startAccel`, `startingState`,
`longitudinalTuning.kpBP` or `kpV`, and no launch kick. Use `CC.cruiseControl.resume`, a launch
snap-to-zero, and a launch jerk limit for move-off.

---

## 7. Add the HUD frame — `0x316 LKAS_HUD_ADAS`

Send `0x316` so the cluster shows the LKAS state while openpilot drives. Copy the stock camera fields
through and override only the LKAS state fields. Add a `COUNTER` and the section-4 checksum.

Name every bit in the frame. The ADAS modules check the full bit pattern, and an unnamed bit packs as 0,
which sets latched DTCs in eight modules. The DBC in `port/` adds `PT1` to `PT4` for this.

---

## 8. Add radar

Read the object tracks from the private CAN, `0x280` to `0x289`. The chassis CAN carries only a lead flag
and the accel command. PR #3337 sets `radarUnavailable = True`.

The track signal layout in `byd_radar_fd.dbc` comes from
[tembikaipilot](https://github.com/danielks23/tembikaipilot) by danielks23, a bukapilot fork with a full FD
radar decode. Use their layout for `TRACK_ID`, `LONG_DIST`, `LAT_DIST`, `VLEAD`, `ALEAD`, `CONFIDENCE` and
the alternate distance fields.

Two fields in that layout are corrected here. `COUNTER12_1` at `0|12` and `COUNTER12_2` at `256|12` are not
12-bit counters. Byte 0 and byte 32 are frame checksums, and the counter is the low nibble of byte 1 and
byte 33. The corrected DBC in `port/` uses `CHECKSUM 0|8` + `COUNTER 8|4` and `CHECKSUM_2 256|8` +
`COUNTER_2 264|4`. The track payload in bytes 2 and above is unaffected.

Subtract the vehicle geometry from `LONG_DIST`. It is measured from the rear-axle centre, the ISO 8855
vehicle frame, not from the sensor face:

```python
BUMPER_TO_RADAR_ORIGIN_M = 3.60   # front overhang 885 mm + wheelbase 2720 mm
dRel = LONG_DIST - BUMPER_TO_RADAR_ORIGIN_M
```

Without the correction each lead reads 3.6 m too far. With it, the radar and the vision lead agree to within
0.5 m below 12 m. Check that on your own vehicle before you use radar for longitudinal control.

Drop any track with a negative range after the correction. A target at the bumper reads about 3.6 raw. Set
the cut-in window after the correction as well, or it is 3.6 m shorter than the value you set.

Keep raw values in the decode reference that your replay validator scores. Put the conversion in the
openpilot-facing interface.

### Private-bus frame integrity

Every private-bus frame carries a checksum in byte 0. Use it to prove a decode, and to reject a corrupt
frame. Do not register it as an opendbc DBC checksum. It is per-message keyed, not the chassis nibble, so
exempt `byd_radar` when you register `byd_checksum` for the `byd_` DBCs.

```
# 32-byte IDs: 0x250, and 0x2C0 to 0x2E7
byte0  = CRC8(poly=0x1D, init=0x00, data[1:32])  XOR K[id]

# 64-byte IDs: 0x280 to 0x29D. Two independently protected 32-byte halves
byte0  = CRC8(poly=0x1D, init=0x00, data[1:32])  XOR K1[id]
byte32 = CRC8(poly=0x1D, init=0x00, data[33:64]) XOR K2[id]
```

- Poly `0x1D` is CRC-8/SAE-J1850, the AUTOSAR E2E family. MSB first, not reflected.
- The 4-bit free-running counter is the low nibble of byte 1, and of byte 33 for the second half. It sits
  inside the protected payload.
- `K` is a per-message constant that folds `init`, `xorout` and the AUTOSAR DataID into one XOR. It is not a
  function of the CAN ID. Harvest one `K` per ID from a log of your own vehicle.
- `0x100` does not use this scheme. It is a separate 8-byte frame and is still undecoded.

Credit: opendbc's Subaru support pointed at this scheme. EyeSight uses a Veoneer camera, and its
`subaru_checksum` is also a byte-0 scheme. The Subaru formula itself does not fit these frames.

---

## 9. Set up the road camera

openpilot has a `USE_WEBCAM` path that takes a V4L2 camera in place of the comma camera stack.

- Measure your camera focal length in pixels and set it. openpilot ships the comma intrinsics, `f = 2648` at
  1928×1208.
- Clear the calibration parameters after a focal change.
- Aim the camera mechanically before you rely on calibration.
- Confirm the frame rate is steady at 20 Hz in real driving conditions.
- Set V4L2 control defaults for clear image capture in all driving conditions.
- Apply the V4L2 controls on every camera open, not only at launch. A USB re-enumeration resets the module.

---

## 10. Bridge the IMU from CAN

A host without an IMU leaves `sensord` silent and degrades `locationd`. Bridge the vehicle inertial sensors
instead. Neither message is in the #3337 DBC.

| Message | Decode |
|---|---|
| `YAW_RATE` `0x222` | `YawRate` `0\|12@1+` × 0.002133 − 2.094 rad/s. `YawRateOffset` `12\|12@1+` × 0.002133 − 0.13. Subtract the offset from the rate. |
| `AXAY` `0x223` | `Ax` `0\|12@1+` and `Ay` `24\|12@1+`, × 0.027167 and × 0.027127 m/s². The offset fields at `12\|12` and `36\|12` share the scale and the bias, so subtract them raw before you scale. |

Publish these as `gyroscope` and `accelerometer` `SensorEventData`.

- Invert the sign of `Ay`. The vehicle frame is ISO, positive left. The device frame is x forward, y right,
  z down. Yaw and `Ax` need no change.
- Send a constant `az = −9.81`. There is no vertical sensor, and the accelerometer must carry gravity.
- Do not report the source as `bmx055`. locationd discards it.
- Expect 50 Hz, against about 104 Hz from a device IMU.
- Expect weak roll and pitch. Yaw and the planar dynamics are complete, which is what the controls need.
- Take `yawRate` and `aEgo` for carState from the same two messages.

Validate the three signs against a drive log before you drive on them: yaw against steering angle, `Ax`
against d(speed)/dt, `Ay` against v × yaw. A flipped yaw shows up as `livePose` never becoming valid.

---

## 11. Build the panda safety

- Allow three messages to transmit: `0x1E2`, `0x316`, `0x32E`.
- Put the safety mode at `opendbc/safety/modes/byd.h`. Put `SAFETY_BYD` and the `byd_hooks` extern in
  `opendbc/safety/declarations.h`, not in `safety.h`.
- Build the panda with `ALLOW_DEBUG`. The `{SAFETY_BYD, &byd_hooks}` entry is inside that block.
- Set the angle limits and accel limits from your own measurements. Do not inherit them from another BYD
  platform, and do not inherit the ones in this manual.

---

## 12. Applicability

All data in this manual comes from one vehicle: a 2024 RHD export Atto 3. Check these before you apply it to
another vehicle.

- Chinese-market builds and other model years can differ.
- Measure your connector. BYD used two camera suppliers, Veoneer and Bosch, with different connectors.
- Use a harness box with a passthrough relay. A splice without a relay has no hardware fallback: openpilot
  replaces the stock LKS, ACC and AEB functions for as long as it runs.

---

## The port files

[`port/`](port/) holds the code that implements this manual: the files that do not exist in an openpilot
release, and a patch for the places an existing file has to change. Copy the files into your own tree.
[`port/README.md`](port/README.md) lists what each file does and where it goes.

The files target released openpilot **0.11.2**. Upstream anything here into opendbc.
