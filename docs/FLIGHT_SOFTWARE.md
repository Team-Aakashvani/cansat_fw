# AAKASHVANI Flight Software — Mission, Wiring, Bring-up

## 1. Mission profile (what the firmware does)

| # | Phase (telemetry state) | Trigger | Action |
|---|---|---|---|
| 1 | **PAD** (2) | boot | Baro zeroed, launch site averaged from GNSS (≥ 5 sats, 10 s), IMU mount auto-detected |
| 2 | **ASCENT** (3) | rocket: ≥ 3 g for 50 ms **and** ≥ 10 m gain within 4 s · carrier drone: ≥ 25 m gain while climbing ≥ 1 m/s for 2 s | Launch site + IMU reference locked |
| 3 | **DESCENT** (4) | ≥ 5 m below peak, sinking ≥ 2 m/s for 1 s, **plus** a free-fall or shock signature in the last 8 s (drop / ejection). Fallback: ≥ 3 m/s for 3 s and ≥ 20 m below peak | Parachute is passive (opens on ejection) |
| 4 | **ARMS_DEPLOY** (5) | AGL ≤ **600 m** and ≥ 2 s after release (clear of the carrier) | Both linear servos → UNLATCH |
| 5 | **STEERING** (6) | arms open 1.5 s, tilt ≤ 45°, rate ≤ 4 rad/s, AGL > 15 m | ESC armed (DShot), motors spin up, PID tilts the CanSat to vector thrust toward the launch site |
| — | motors cut | AGL ≤ 10 m | Motors stop for good (props / people safety) |
| 6 | **LANDED** (7) | AGL ≤ 5 m, \|v\| ≤ 0.6 m/s for 2 s | Red LED + buzzer beacon |

A carrier that descends slowly with the CanSat still attached never satisfies the release
rule, so the arms cannot open inside the carrier. All transitions are forward-only and the
whole mission state is kept in RTC memory: after a brownout / watchdog / panic reset in the air
the firmware resumes (baro reference restored, servos stay unlatched, motors re-qualify attitude
before restarting). A normal power-on always starts on the pad.

### Failsafes while steering
* Tilt > 65° for 0.3 s or attitude invalid → motors suspended; resume after < 30° for 1 s.
* No fresh GNSS fix (> 3 s) or no launch site → level hold (no tilt command).
* `ABORT` → motors off for the rest of the flight (arms unchanged).
* Control task stalls → no DShot frames → the ESC's own signal-loss failsafe stops the motors.

## 2. Navigation stack (why no IMM)

* **Attitude**: BNO055 fusion quaternion → `AttitudeReference` (mount detection, level trim,
  singularity-free ZXY output). Locked once flight starts.
* **Vertical**: `VerticalKF` — altitude, vertical speed, accel bias, `float`, 100 Hz predict /
  50 Hz baro. Spike protection: median-of-3, physical rate limit, χ² gate, hold-then-resync
  (judged on the baro's own second difference), gate bypass while the accel is clipped
  (BNO055 is locked to ±4 g in fusion mode), phase-scheduled process noise.
* **Horizontal**: GNSS position/velocity directly into `ReturnGuidance` (PI velocity loop that
  learns the wind).

The 5 × 15-state IMM (still in `components/nav`, not used in the flight path) took ~35 ms per
IMU sample in software-emulated `double` on the ESP32-S3, and its regime probabilities had no
effect on any decision (the supervisor floored them out). The phase-scheduled filter reproduces
its only effective feature (per-phase process noise) at a tiny fraction of the cost.

Host tests: `python tests/host/run_tests.py`.

## 3. Pin map (ESP32-S3-DevKitC-1 N16R8) — wire to these

| Function | GPIO | Status |
|---|---|---|
| I²C SDA / SCL (BNO055, BMP585, SHT4x, SGP41) | 38 / 39 | wired |
| GNSS RX / TX (N-GS-01) | 13 / 21 | wired |
| ESC M1 FL / M2 FR / M3 RR / M4 RL (DShot300) | 4 / 5 / 6 / 7 | **to wire** |
| Arm-latch servo A / B (50 Hz) | 15 / 16 | **to wire** |
| XBee TX / RX | 17 / 18 | **to wire** |
| SPI SCK / MOSI / MISO, SD CS, CC1101 CS | 12 / 11 / 10, 9, 14 | **to wire** (set `PINS.spi_devices_fitted = true`) |
| Recovery buzzer / beacon | 42 | to wire |
| Battery divider (ADC) | 1 | optional |

Never use GPIO 19/20 (USB), 26–37 (flash / octal PSRAM), 0/3/45/46 (strapping), 43/44, 48 (RGB LED).
ESC ground must be common with the ESP32 ground; signal wires as short as possible.

## 4. Bench bring-up — PROPS OFF until step 6

1. **Servos**: `CMD,001,CHUTE` unlatches both servos (manual). Re-latch with `CMD,001,CAL` on the pad.
   Adjust `ACT_CFG.servo_lock_us / servo_unlatch_us` to your mechanism.
2. **Motor order**: `CMD,001,MTR,0,8` spins M1 at 8 % for 3 s (then `1`, `2`, `3`). Confirm M1 is
   front-left, M2 front-right, M3 rear-right, M4 rear-left **as seen from above with vehicle +X forward**.
3. **Prop direction**: M1 & M3 clockwise, M2 & M4 counter-clockwise (props-in). If reversed, fix
   in BLHeliSuite (or swap two motor wires), or set `steer.yaw_sign = -1`.
4. **IMU frame vs. motor frame**: vehicle +X is the IMU's forward axis after mount detection.
   If the arms are rotated relative to it, set `steer.frame_yaw_deg` (e.g. 45, 90).
5. **Mixer signs**: `CMD,001,PID,START,15` (20 s max). Tilt the CanSat by hand: the motors on
   the side that dips must speed up. If not, the frame/sign settings above are wrong.
6. **Tethered test** with props, low collective, before any flight.

## 4a. Lift (elevator) test — whole sequence at building scale

`CMD,001,LIFT,<deploy_m>` (pad only) runs the same mission logic with building-sized
thresholds and **motors hard-inhibited** (they cannot spin even with the ESC connected):
launch = sustained 4 m climb, release = sustained 1.5 m descent below the top, arms unlatch at
`<deploy_m>` AGL (default 10), "motor cut-off" at 3 m, landed within 1.5 m of the start floor.
`CMD,001,CAL` resets it to PAD for another ride; `CMD,001,LIFT,OFF` restores flight thresholds.
Persisted like a real flight; a power cycle always returns to normal flight mode.

With the normal flight configuration a lift ride is also a useful *negative* test: a ride of
more than ~25 m latches launch, but riding down must **not** count as a release (a lift is a
carrier descending with the CanSat attached), so the arms must stay latched.

## 5. Pre-flight checklist

* Power on **upright on the pad, still** (mount detection + baro zero happen at boot).
* `MSN` line shows `PAD`, AGL ≈ 0, ≥ 5 satellites; wait ≥ 10 s for the launch site.
* Heading calibration (once per vehicle / after re-mounting the IMU): point vehicle +X at true
  north (phone compass) and send `CMD,001,NORTH` (stored in NVS).
* Check the motors' magnetic effect on heading on the bench (spin at ~35 % and watch the `hdg`
  field in `MSN`). Guidance tolerates ~60° heading error, but keep the IMU away from ESC/battery leads.

## 6. Commands (USB, Bluetooth or XBee: `CMD,001,<CMD>,<args>`)

| Command | Effect | Allowed |
|---|---|---|
| `CX,ON/OFF` | radio (XBee) telemetry; off at every power-on, as the guidelines require | always |
| `ST,hh:mm:ss` | set mission time | always |
| `CAL` | re-zero baro, reset mission to PAD, re-lock servos | pad only |
| `SIM,ENABLE/DISABLE`, `SIMP,<Pa>` | hardware-in-the-loop with injected pressure | pad / sim only |
| `TARE` | re-detect IMU mount and re-reference | pad only (locked in flight) |
| `NORTH` | heading calibration | pad |
| `LIFT,<m>` / `LIFT,OFF` | lift test mode (motors inhibited) | pad |
| `LOG,LIST` / `LOG,DUMP[,n]` / `LOG,ERASE` | flight recorder | USB; erase on pad |
| `LOG,CRASH` / `LOG,CRASHCLR` | crash dump report / clear | always |
| `CHUTE` | manual arm unlatch (backup) | always |
| `ABORT` | motors off for the rest of the flight | always |
| `MTR,<0-3/ALL>,<0-25>` | motor test, 3 s | pad only, props off |
| `PID,START,<pct>` / `PID,STOP` | bench stabilisation test, 20 s | pad only |

## 7. Telemetry lines (USB)

* Frame (25 Hz USB, 1 Hz radio after `CX,ON`): the 23-field competition CSV starting with
  `2026-IN-SPACeCAN-7USAT-001`, with time since power-on in seconds and the state as a name
  (`PAD`, `ASCENT`, ...). Full definition: `docs/TELEMETRY_FORMAT.md`. The *gyro spin rate*
  field carries the IMU's body-Z rate (no mechanical gyroscope is fitted).
* The dock saves every frame it receives to `Documents\Aakashvani\Flight_2026-IN-SPACeCAN-7USAT-001.csv`.
* `ATT` (50 Hz): MCU-timestamped quaternion for the 3D viewer.
* `MSN` (5 Hz): `phase, AGL, v, peak, arms, esc, dist, bearing, sats, baro_held, tilt_cmd_x, tilt_cmd_y, heading`.

## 8. Known limits / open items

* **Thrust authority vs. wind**: return is only possible if the achievable air speed under the
  canopy exceeds the wind. Simulation: ~5 m/s air speed at 0.7 W thrust and 22° tilt; a 3 m/s
  wind at 0.5 W is not overcome (the vehicle still gains ground). Measure real thrust and canopy
  drag early.
* **Heading under motor current**: BNO055 NDOF heading uses the magnetometer; verify on the bench.
* **Accel > 4 g**: the BNO055 clips in fusion mode; the vertical filter handles it (baro-led),
  but a high-g accelerometer would improve boost tracking for rockets.
* Steering gains (`ACT_CFG`, `GUIDE_CFG`) are simulation-tuned starting points — tune on the
  tethered test.

## 9. Storage, crash dumps and the wireless link

### Memory map (ESP32-S3 N16R8: 16 MB flash, 8 MB PSRAM)
| Partition | Offset | Size | Use |
|---|---|---|---|
| nvs | 0x9000 | 24 KB | settings (team id, ground alt, heading offset) |
| factory | 0x20000 | 2 MB | firmware (~720 KB used) |
| config_store | 0x220000 | 64 KB | spare |
| event_log | 0x230000 | 512 KB | discrete flight events (NVS) |
| coredump | 0x2B0000 | 256 KB | crash dump of the last panic |
| (free) | 0x2F0000 | 1 MB | reserved |
| flightlog | 0x400000 | 12 MB | flight data recorder |

PSRAM (8 MB, octal, 80 MHz) is enabled. NimBLE and the flight-recorder queue allocate from it;
task stacks stay in internal RAM. Free internal heap: ~130 KB with Bluetooth running.

### Flight data recorder
* 64-byte records with CRC: one session header per boot, 50 Hz nav records (phase, AGL,
  vertical speed, raw baro, attitude quaternion, accel, gyro, GNSS, arms / ESC state, motor
  throttles, guidance tilt command) and mission events. 12 MB holds about an hour.
* Ring buffer: the oldest data is overwritten. Sectors are erased ahead of the write pointer in
  the background **on the pad only** (a 4 KB erase stalls both cores ~45 ms); keep the CanSat
  powered for ~2 minutes before launch so 3 MB is pre-erased (the Health page shows it).
* Commands (USB): `LOG,LIST`, `LOG,DUMP[,n]` (n = 1 latest), `LOG,ERASE` (pad only).
  The dock's Flight log page saves a session as CSV plus an events CSV.

### Crash dumps
A panic writes a core dump to flash. At the next boot the Health page shows it;
`LOG,CRASH` prints task, PC and backtrace (decode with `xtensa-esp32s3-elf-addr2line -pfiaC -e build/cansat_fw.elf <addresses>`),
`LOG,CRASHCLR` clears it.

### Bluetooth LE link
* Advertises as `AAKASHVANI-001`, Nordic UART Service profile (works with the dock and
  with phone apps such as nRF Connect / Serial Bluetooth Terminal).
* Sends: `HLT` health (1 Hz), `MSN` mission (5 Hz), `ATT` attitude (10 Hz), frame (1 Hz),
  `ENV` (1 Hz), `EVT` events, `ACK` / `NAK` for commands.
* Accepts the normal commands, **except** actuator and bulk-flash ones (`MTR`, `PID`, `CHUTE`,
  `LOG,DUMP`, `LOG,ERASE`, `OTA`): those need USB, because anyone in radio range could send them.
* Off automatically during a real flight (2.4 GHz is shared with the XBee), back on after
  landing for recovery. Stays on during a lift test.
* Its memory comes from PSRAM, so internal RAM stays at ~130 KB free.

### Health line
`HLT,<uptime s>,<IMU Hz>,<baro ok>,<sats>,<battery V>,<free heap KB>,<log KB this session>,<pre-erased KB>,<temp C>,<BIT flags hex>,<BLE connected>,<IMU mount>,<crash dump present>,<dropped records>`

<a id="led"></a>
## 9a. RGB LED

| When | LED |
|---|---|
| Power on | white fade, then blue (buses and sensors starting) |
| IMU aligning | violet breathing: keep the CanSat still |
| Self-test result | 2× green = pass · 2× amber = warnings (optional hardware missing) · 3× red = IMU or barometer missing |
| Reset in flight | one magenta flash, no delay |
| PAD / ASCENT / DESCENT | green / orange / purple |
| ARMS_DEPLOY / STEERING | blue / cyan |
| LANDED | blinking red with the buzzer |

## 10. Ground station (tools/aakashvani_dock.py)

`LAUNCH_DOCK.bat`. Connect over **USB** or **Bluetooth** (header). Pages: Overview (phase rail,
altitude, return-to-launch, events), Attitude (horizon, angles, 3D view, re-reference, north
calibration), Health, Flight log (list / download / erase / crash report), Bench (zero altitude,
lift test, hardware-in-the-loop simulated flights, props-off actuator tests), Console, Firmware
(flashes `build/` using `flasher_args.json`). Fonts: IBM Plex (SIL OFL), icons: Lucide (ISC),
bundled in `tools/assets/`.
