# AAKASHVANI — Software Architecture

How the flight software is built: tasks, data flow, the navigation stack and memory.
For the mission rules and failsafes see `FLIGHT_SOFTWARE.md`.

---

## 1. Platform

* ESP32-S3-DevKitC-1 **N16R8**: dual-core Xtensa LX7 at 240 MHz, 512 KB internal SRAM,
  16 MB quad flash, 8 MB octal PSRAM (80 MHz).
* ESP-IDF 5.5.5, FreeRTOS, C++17.
* Console on the native USB-Serial-JTAG port (no UART bridge).

## 2. Tasks

| Task | Core | Priority | Rate | Job |
|---|---|---|---|---|
| `imu` | 0 | max−1 | 100 Hz | BNO055 read, attitude reference (mount detection, level trim, flight lock) |
| `nav` | 0 | max−2 | 100 Hz | Vertical Kalman filter, mission supervisor, return guidance, flight-recorder feed; the **only** writer of actuator commands |
| `ctrl` | 0 | max−3 | 100 Hz | Arm-latch servos (LEDC) and DShot300 motor frames (RMT), steering controller |
| `sensor` | 1 | max−3 | 50 Hz | BMP585, GNSS parsing, SHT4x, SGP41 |
| `telem` | 1 | 5 | 50 Hz loop | `ATT` (50 Hz), competition frame (25 Hz USB, 1 Hz radio/SD), `MSN`, `HLT`, Bluetooth lines |
| `logging` | 1 | 4 | 1 Hz | XBee state machine, SD flush |
| `power` | 1 | 3 | 1 Hz | Battery monitoring |
| `flightrec` | 1 | 2 | queue | Writes 64-byte records to the flight-log partition, erases ahead on the pad |
| `beacon` | any | 2 | 25 Hz | RGB LED phase colour and recovery buzzer (status only, no actuators) |
| `cli`, `ble_cmd`, `test_mgr` | 1 | 1 | event | USB console, Bluetooth command bridge, bench tests |

Shared state is passed as snapshots under two mutexes (`sensor_mutex`, `fc_mutex`). Tasks
never hold a mutex while doing I/O.

## 3. Data flow

```text
BNO055 ──► imu_task ──► AttitudeReference ──┐
BMP585 ──► sensor_task ─────────────────────┼─► nav_task: VerticalKF ─► MissionSupervisor ─► ReturnGuidance
GNSS   ──► sensor_task ─────────────────────┘                 │                 │
                                                              │                 ▼
                                                              │        ctrl_task: SteerController ─► MotorMixer ─► DShot
                                                              │                  servo latch ◄── arms command
                                                              ▼
                                         FlightRecorder (50 Hz) · telem_task (USB / BLE / XBee / SD)
```

## 4. Navigation stack

* **Attitude**: the BNO055 fusion quaternion passes through `AttitudeReference`
  (`drivers/imu_attitude`). It auto-detects how the IMU is mounted (upright or perpendicular),
  applies a level trim and outputs a singularity-free ZXY attitude. The reference locks when
  the flight starts, so `TARE` cannot change it in the air.
* **Vertical**: `VerticalKF` (`nav/vertical_kf.hpp`) is a 3-state float filter (altitude,
  vertical speed, accelerometer bias): 100 Hz predict, 50 Hz baro update. Spike protection
  works in layers: median-of-3, a physical rate limit, a χ² gate, hold-then-resync judged on the
  baro's own second difference, and a gate bypass while the accelerometer is clipped (BNO055 =
  ±4 g in fusion mode). Process noise changes with the mission phase.
* **Mission**: `MissionSupervisor` (`nav/mission.hpp`) moves forward only through PAD → ASCENT →
  DESCENT → ARMS_DEPLOY → STEERING → LANDED. The state is kept in RTC memory, so a reset in
  flight resumes the mission. It also has a lift-test profile with building-sized thresholds and
  the motors hard-inhibited.
* **Guidance**: `ReturnGuidance` (`nav/guidance.hpp`) is a PI loop on GNSS ground velocity
  toward the launch site. It learns the wind and outputs a tilt command (limited to 25°).
* **Control**: `SteerController` (`control/steer_controller.hpp`) is an angle-PI → rate-PID cascade
  that produces torque, mixed for a quad-X frame in `MotorMixer` and sent as DShot300.

The earlier 5 × 15-state IMM is still in `components/nav` for offline comparison, but it is not
in the flight path. It took ~35 ms per IMU sample in software `double` on the S3, and it had no
effect on any decision.

## 5. Links

* **USB**: console, full telemetry, flight-log download, bench tests.
* **Bluetooth LE**: NimBLE peripheral `AAKASHVANI-001`, Nordic UART Service. Sends health,
  mission, attitude, frame, environment and events. Commands go through `ble_cmd`, which refuses
  actuator and bulk-flash commands. The link is switched off automatically between launch and
  landing, except during a lift test.
* **XBee**: 1 Hz competition frame, gated by `CX,ON`.

## 6. Memory

### Flash (16 MB)

| Partition | Offset | Size | Use |
|---|---|---|---|
| nvs | 0x9000 | 24 KB | Settings: team id, ground altitude, heading offset, cached position |
| otadata / phy_init | 0xF000 / 0x11000 | 8 KB / 4 KB | IDF |
| factory | 0x20000 | 2 MB | Firmware (~730 KB used) |
| config_store | 0x220000 | 64 KB | Spare NVS |
| event_log | 0x230000 | 512 KB | Discrete events (NVS) |
| coredump | 0x2B0000 | 256 KB | ELF core dump of the last panic (CRC32) |
| (free) | 0x2F0000 | 1 MB | Reserved |
| flightlog | 0x400000 | 12 MB | Flight-recorder ring |

### RAM

* PSRAM is enabled (`CONFIG_SPIRAM`, octal, 80 MHz), with `malloc` falling through to PSRAM
  for blocks above 16 KB, and 32 KB of internal RAM reserved for DMA/ISR use.
* NimBLE allocates from PSRAM (`BT_NIMBLE_MEM_ALLOC_MODE_EXTERNAL`). The flight-recorder queue
  is in PSRAM too.
* Result: **~130 KB free internal heap** with Bluetooth running (it was ~84 KB before PSRAM).
* Task stacks stay in internal RAM, because flash writes disable the cache.

### Flight recorder

64-byte records with a CRC-16: one session header per boot, 50 Hz navigation records and event
records. On readout the ring is memory-mapped in 256 KB windows. Listing all 12 MB takes about
2 s. Records whose session header has been overwritten are listed as "start overwritten".

## 7. Boot sequence

1. `system_init`: NVS, team id (synced to `TELEM_CFG.team_id`), boot counter, crash-dump check.
2. RGB LED: white fade (power), then blue (buses). An in-flight resume flashes magenta once and
   skips everything slow.
3. I²C scan, sensor init, built-in test (BIT).
4. IMU alignment: the LED breathes violet until the mount is detected.
5. BIT result on the LED: 2× green = pass, 2× amber = warnings, 3× red = IMU or barometer missing.
6. Tasks start. The LED shows the mission phase from then on.
