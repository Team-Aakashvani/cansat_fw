# AAKASHVANI — Telemetry and Command Reference

Everything the flight computer sends and accepts, on USB, Bluetooth LE and the XBee radio.
Team number **001**. Competition TEAM_ID **`2026-IN-SPACeCAN-7USAT-001`**.

---

## 1. Competition frame (guidelines §6)

One CSV line, starting with the TEAM_ID string:

```text
2026-IN-SPACeCAN-7USAT-001,412.3,10308,0.4,100912,31.2,0.00,38521,21.163120,72.784630,14.2,9,0.12,-0.31,9.78,1.2,-0.8,0.3,PAD,182.0,61.4,102,1
```

| # | Field | Unit | Notes |
|---|---|---|---|
| 1 | `TEAM_ID` | — | `2026-IN-SPACeCAN-7USAT-001` |
| 2 | `TIME_STAMPING_S` | s | Time since power-on (0.1 s resolution) |
| 3 | `PACKET_COUNT` | — | Increments with every frame, reset on power-on |
| 4 | `ALTITUDE_M` | m | Altitude above the launch site (vertical Kalman filter) |
| 5 | `PRESSURE_PA` | Pa | BMP585 |
| 6 | `TEMP_C` | °C | BMP585 |
| 7 | `VOLTAGE_V` | V | Battery. `0.00` while the power monitor is not fitted |
| 8 | `GNSS_TIME_S` | s | UTC seconds of day from the GNSS (`hh*3600+mm*60+ss`) |
| 9 | `GNSS_LATITUDE` | ° | Decimal degrees, + = north |
| 10 | `GNSS_LONGITUDE` | ° | Decimal degrees, + = east |
| 11 | `GNSS_ALTITUDE_M` | m | Above mean sea level |
| 12 | `GNSS_SATS` | — | Satellites used in the fix |
| 13–15 | `ACC_X/Y/Z_MPS2` | m/s² | BNO055 body-frame acceleration |
| 16 | `ROLL_DEG` | ° | |
| 17 | `PITCH_DEG` | ° | |
| 18 | `GYRO_SPIN_RATE_DPS` | °/s | Spin rate about the CanSat's long (Z) axis from the IMU gyro. No mechanical gyroscope is fitted, so this field carries the body Z rate. |
| 19 | `FLIGHT_SOFTWARE_STATE` | — | `BOOT`, `PAD`, `ASCENT`, `DESCENT`, `ARMS_DEPLOY`, `STEERING`, `LANDED`; prefixed `LIFT-` during a lift test |
| 20 | `HEADING_DEG` | ° | Magnetic heading + stored north offset |
| 21 | `HUMIDITY_PCT` | % | SHT4x |
| 22 | `VOC_INDEX` | — | SGP41 (1–500, 100 = typical) |
| 23 | `NOX_INDEX` | — | SGP41 (1–500, 1 = typical) |

### Rates and gating

| Path | Rate | When |
|---|---|---|
| USB | 25 Hz | always (paused while a flight log is downloading) |
| Bluetooth LE | 1 Hz | while a client is connected (BLE is off during a real flight) |
| XBee radio | 1 Hz | **only after `CX,ON`**. The guidelines forbid radio transmission until the ground station starts it. `CX,OFF` stops it again. The radio is off at every power-on. |
| SD card | 1 Hz | when an SD card is fitted (not fitted yet) |

### Ground station file

The dock appends every frame it receives to
`Documents\Aakashvani\Flight_2026-IN-SPACeCAN-7USAT-001.csv` with this header row:

```text
TEAM_ID,TIME_STAMPING_S,PACKET_COUNT,ALTITUDE_M,PRESSURE_PA,TEMP_C,VOLTAGE_V,GNSS_TIME_S,GNSS_LATITUDE,GNSS_LONGITUDE,GNSS_ALTITUDE_M,GNSS_SATS,ACC_X_MPS2,ACC_Y_MPS2,ACC_Z_MPS2,ROLL_DEG,PITCH_DEG,GYRO_SPIN_RATE_DPS,FLIGHT_SOFTWARE_STATE,HEADING_DEG,HUMIDITY_PCT,VOC_INDEX,NOX_INDEX
```

Keep one file for the whole flight day. The Flight log page has **Start fresh file**, which
renames the old file with a date stamp.

---

## 2. Internal lines (USB and Bluetooth)

All start with the three-digit team number.

| Line | Rate | Fields |
|---|---|---|
| `001,ATT` | 50 Hz USB, 10 Hz BLE | `mcu_ms, qw, qx, qy, qz, tilt_x, tilt_y, rot_z, mount, reference_count`: attitude for the 3D view |
| `001,MSN` | 5 Hz | `phase, AGL m, vertical speed m/s, peak m, arms, esc, distance to launch m, bearing °, sats, baro_held, tilt_cmd_x, tilt_cmd_y, heading °` |
| `001,HLT` | 1 Hz | `uptime s, IMU Hz, baro ok, sats, battery V, free heap KB, log KB this session, pre-erased KB, temp °C, BIT flags (hex), BLE connected, IMU mount, crash dump present, dropped records` |
| `001,ENV` | 1 Hz | `temperature °C, humidity %, VOC index, NOx index` |
| `001,EVT` | on event | `event name, detail` (launch, release, arms, motors, landed …) |
| `001,ACK` / `001,NAK` | per command | Reply to each command received over Bluetooth (`NAK` gives the reason) |
| `LOGLIST,…` / `LOGBEGIN`, `LOGHDR`, `LOG`, `LOGEV`, `LOGSESSION`, `LOGEND` | on request | Flight-recorder listing and download (USB only) |
| `CRASH,…` | on request | Crash dump summary |

### BIT flags (in `HLT`)

| Bit | Meaning | Blocks flight? |
|---|---|---|
| 0x001 | IMU absent | yes (3 red LED blinks at boot) |
| 0x002 | Barometer absent | yes (3 red LED blinks at boot) |
| 0x004 | Power monitor absent | no; expected until the INA260 is fitted |
| 0x008 | GNSS silent (no NMEA) | no |
| 0x010 | Radio absent | no |
| 0x020 | SD card failed / absent | no; expected until the SD card is fitted |
| 0x040 | NVS failed | no |
| 0x080 | IMU sanity (gravity out of range) | no |
| 0x100 | Barometer sanity (pressure out of range) | no |
| 0x200 | Battery voltage low | no |

With the current hardware, `0x2C` (power monitor, GNSS without a fix at boot, SD) is normal.

---

## 3. Commands

Format: **`CMD,001,<COMMAND>[,<args>]`**, newline-terminated. Team `000` is also accepted
(useful from a phone). Commands can be sent over USB, Bluetooth or the XBee.

| Command | Effect | Allowed |
|---|---|---|
| `CX,ON` / `CX,OFF` | Start / stop radio (XBee) telemetry | always |
| `ST,hh:mm:ss` | Set mission time | always |
| `CAL` | Re-zero the barometer, reset the mission to PAD, re-latch the servos | pad |
| `SIM,ENABLE` / `SIM,DISABLE` | Hardware-in-the-loop mode (injected pressure replaces the barometer) | pad / simulation |
| `SIMP,<Pa>` | Inject one pressure sample | simulation |
| `TARE` | Re-detect IMU mount and re-reference attitude | pad (locked in flight) |
| `NORTH` | Heading calibration: vehicle +X points to true north (stored) | pad |
| `LIFT,<m>` / `LIFT,OFF` | Lift test mode with the arm release at `<m>` (motors inhibited) | pad |
| `LOG,LIST` | List recorder sessions | USB, BLE |
| `LOG,DUMP[,n]` | Download session `n` (1 = newest) | **USB only** |
| `LOG,ERASE` | Erase the recorder | **USB only**, pad |
| `LOG,CRASH` / `LOG,CRASHCLR` | Crash dump report / clear | always |
| `CHUTE` | Manual arm unlatch (backup) | **USB / XBee only** |
| `ABORT` | Motors off for the rest of the flight | always |
| `MTR,<0-3/ALL>,<0-25>` | Spin one motor for 3 s at the given % (**props off**) | **USB only**, pad |
| `PID,START,<pct>` / `PID,STOP` | Bench stabilisation test, 20 s maximum (**props off**) | **USB only**, pad |

Bluetooth refuses `MTR`, `PID`, `CHUTE`, `OTA`, `LOG,DUMP` and `LOG,ERASE` with a `NAK`, because
anyone within radio range could send them.

Over-the-air firmware update is **not** supported. Flash over USB (see `FLASH_GUIDE.md`).
