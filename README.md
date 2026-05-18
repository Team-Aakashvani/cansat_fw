# AAKASHVANI — CAN-7USAT India 2026
### Flight Software v1.0 | SVNIT Aerospace Team
> ESP32-S3 & ESP32-P4 · ESP-IDF v5.3 · FreeRTOS · C++17

---

## Table of Contents
1. [Project Overview](#1-project-overview)
2. [Quick Start](#2-quick-start)
3. [Hardware Roles & Targets](#3-hardware-roles--targets)
4. [Hardware Bill of Materials](#4-hardware-bill-of-materials)
5. [Mission Profile](#5-mission-profile)
6. [Software Architecture Summary](#6-software-architecture-summary)
7. [Boot Sequence](#7-boot-sequence)
8. [Command Reference](#8-command-reference)
9. [Telemetry Format](#9-telemetry-format)
10. [Partition Layout](#10-partition-layout)
11. [Pre-Flight Checklist](#11-pre-flight-checklist)
12. [Documentation Index](#12-documentation-index)
13. [Known Limitations (v1.0)](#13-known-limitations-v10)
14. [License](#14-license)

---

## 1. Project Overview

**AAKASHVANI** is the aerospace-grade flight software for the CAN-7USAT India 2026 competition. It implements a unified, role-based firmware architecture targeting multiple hardware nodes — Flight Computer (FC), Media Coprocessor (P4), and Ground Station Bridge (GCS) — within a single codebase, selected at compile-time.

### Key Capabilities

| Subsystem | Description |
|-----------|-------------|
| **Navigation (FC)** | 100 Hz Error-State EKF (15-state) inside a 5-regime IMM filter with Bayesian supervisor and SPRT FDIR |
| **Control (FC)** | 100 Hz attitude + descent-rate PID with battery compensation, anti-windup, and motor mixer |
| **Media (P4)** | ESP32-P4 dedicated MIPI-CSI capture, hardware H.264 encoding, high-speed SDMMC storage |
| **RF Mapping** | Independent secondary mission: CC1101 directional RSSI scan synchronized with FC nav state |
| **Comms** | SX1278 LoRa 433 MHz downlink (1 Hz telemetry CSV) + uplink (command parser) |
| **Inter-MCU Link** | XOR-checksum-verified UART at 921,600 baud with heartbeat monitoring and failsafe |
| **Ground Station** | Transparent LoRa-to-USB bridge with promiscuous mode and emergency override |

---

## 2. Quick Start

### Prerequisites
- **ESP-IDF v5.3+** — [Install Guide](docs/BUILD_GUIDE.md)
- **CMake ≥ 3.20** (bundled with ESP-IDF installer)
- **Python 3.8+**

### Build & Flash

```bash
# 1 — Set your team ID before building
#     Edit: components/nav/include/nav/config.hpp → TelemetryConfig::team_id

# 2 — Set the hardware target
idf.py set-target esp32s3   # or esp32p4 for P4 role

# 3 — Select firmware role via menuconfig
idf.py menuconfig
#     → CAN-7USAT Build Target → select ROLE_FC / ROLE_GCS / ROLE_P4

# 4 — Build
idf.py build

# 5 — Flash + monitor
idf.py -p <PORT> flash monitor
```

> **First flash:** Erase NVS before the first flight to clear any stale calibration data.
> ```bash
> python -m esptool --chip esp32s3 -p <PORT> erase_region 0x9000 0x6000
> idf.py -p <PORT> flash
> ```

---

## 3. Hardware Roles & Targets

The firmware role is selected at **compile-time** via `idf.py menuconfig`.

| Role | Chip | Entry Point | Description |
|------|------|-------------|-------------|
| `ROLE_FC` | ESP32-S3 WROOM-1 | `main_fc()` | Primary avionics: navigation, control, logging, comms |
| `ROLE_P4` | ESP32-P4 | `main_p4()` | Dedicated video: MIPI-CSI capture + H.264 + SDMMC |
| `ROLE_GCS` | ESP32-S3 | `main_gcs()` | Ground station: transparent LoRa ↔ USB bridge |

`app_main()` calls `system_init::core_init()` (shared NVS + logging) then dispatches to the appropriate role entry via `#if CONFIG_ROLE_FC / _GCS / _P4`.

---

## 4. Hardware Bill of Materials

### Flight Computer (ESP32-S3 WROOM-1)

| Sensor/IC | Interface | Address/CS | Rate | Purpose |
|-----------|-----------|------------|------|---------|
| BNO085 | I2C-0 (GPIO 8/9) | 0x4A | 100 Hz | IMU — accel + calibrated gyro |
| BMP585 | I2C-0 (GPIO 8/9) | 0x46 | 50 Hz | Barometer — altitude AGL |
| N-GS-01 | UART-1 (GPIO 17/18) | 115200 baud | 1 Hz | NavIC GNSS — position + velocity |
| SX1278 | SPI (GPIO 35/37/36) | CS=GPIO34 | — | LoRa 433 MHz transceiver |
| CC1101 | SPI (shared) | CS=GPIO21 | — | RF scanner (secondary mission) |
| INA260 | I2C-1 (GPIO 10/11) | 0x40 | 1 Hz | Voltage/current monitor |
| MAX17048 | I2C-1 (GPIO 10/11) | 0x36 | 1 Hz | LiPo fuel gauge |
| SDP31 | I2C-1 (GPIO 10/11) | 0x21 | — | Differential pressure / airspeed |
| SHT4x | I2C-1 (GPIO 10/11) | 0x44 | — | Humidity + temperature |
| SGP41 | I2C-1 (GPIO 10/11) | 0x59 | — | VOC / NOx air quality |

### Power System

| Parameter | Value |
|-----------|-------|
| Battery | 2S LiPo (7.4 V nominal, 8.4 V full, 6.4 V cutoff) |
| Capacity | ≥ 1000 mAh recommended |
| MCU Supply | 5 V BEC → ESP32-S3 VIN (3.3 V LDO on-board) |
| Motor Supply | 7.4 V direct to ESCs |
| Hover current | ~2.0 A @ 30% throttle |
| Peak current | ~6.0 A @ 80% throttle |

---

## 5. Mission Profile

```
PRE_FLIGHT → BOOST → BALLISTIC → PARACHUTE → DRONE_HOVER → LANDED
```

| Phase | Code | Trigger | Action |
|-------|------|---------|--------|
| `PRE_FLIGHT` | 0 | On pad | BIT pass, awaiting launch detect |
| `BOOST` | 1 | 2-of-3 vote: accel/alt/vel | Log event, arm P4 recording (`RECORD`) |
| `BALLISTIC` | 2 | Burnout + velocity signature | IMM regime switch (σa=2.5) |
| `PARACHUTE` | 3 | 600 m AGL ±10 m, IMM posterior > 0.75 | Servo release, chute deploy |
| `DRONE_HOVER` | 4 | Deployed + altitude < 30 m | Arm motors, PID attitude + descent (1–3 m/s) |
| `LANDED` | 5 | Velocity ~0 + altitude ~0 for 1 s | Disarm, enable 92 dB beacon, flush P4 video |

### Secondary Mission — RF Mapping
- **Trigger:** `MAP` command from GCS
- **Hardware:** CC1101 scanner + servo-controlled directional antenna (0–180°)
- **Output:** Synchronized log `(Time, Lat, Lon, Alt, Angle, RSSI)` on FC SD card
- **Isolation:** Runs as a low-priority background FreeRTOS task; locks `fc_mutex` only for brief state snapshots

---

## 6. Software Architecture Summary

### FreeRTOS Task Model (FC)

| Task | Core | Rate | Priority | Stack | Function |
|------|------|------|----------|-------|----------|
| `nav_task` | 0 (RT) | 100 Hz | MAX-1 | 8192 W | IMU ingest → IMM EKF propagation |
| `control_task` | 0 (RT) | 100 Hz | MAX-2 | 4096 W | PID attitude + descent → motor mixer |
| `sensor_task` | 1 (SYS) | 50 Hz | MAX-3 | 4096 W | Baro (50 Hz) + GNSS (1 Hz) poll → EKF update |
| `telem_task` | 1 (SYS) | 1 Hz | 5 | 4096 W | Encode CSV → LoRa TX + SD write |
| `logging_task` | 1 (SYS) | 1 Hz / 5 s | 4 | 8192 W | LoRa spin + SD flush |
| `power_task` | 1 (SYS) | 1 Hz | 3 | 2048 W | INA260 + MAX17048 aggregation |
| `p4_link_task` | 1 (SYS) | 10 Hz | 3 | 2048 W | UART heartbeat + command to P4 |
| `rf_map_task` | 1 (SYS) | sweep | 3 | — | CC1101 RSSI scan + SD log |
| `beacon_task` | any | 1 Hz | 2 | 1024 W | GPIO39 recovery beacon (LANDED phase) |

### Navigation Stack

```
EKF State:  x = [p(3), v(3), q(4), ba(3), bg(3)]   (16 elements)
Error-state: δx ∈ ℝ¹⁵  (attitude error as δθ ∈ ℝ³)

IMM Regimes (5):
  BOOST       σa=10.0  — high-thrust launch dynamics
  BALLISTIC   σa=2.5   — free-fall coasting
  PARACHUTE   σa=3.0   — chute-limited descent
  DRONE_HOVER σa=1.0   — active PID stabilization
  LANDED      σa=0.1   — near-stationary

Measurement Updates:
  IMU (100 Hz)  → strapdown INS propagation (2nd-order midpoint)
  Baro (50 Hz)  → altitude AGL position update
  GNSS (1 Hz)   → ENU position + velocity update

Reliability:
  SPRT FDIR          — per-channel fault detection + exclusion
  NIS health monitor — exponential smoothing on innovation statistics
  Mehra 1972 EMA     — adaptive measurement noise covariance
  Joseph-form update — numerically stable covariance propagation
  Cholesky solve     — avoids full matrix inversion
```

### Synchronisation

| Primitive | Protects |
|-----------|----------|
| `fc_mutex` | `FlightComputer` struct (written by nav/sensor, read by control/telem/rf_map) |
| `sensor_mutex` | Latest baro, GNSS, IMU, power snapshots |
| `evt_group` (BIT_PASS bit) | Prevents `control_task` from arming before BIT passes |
| `std::atomic<>` | `packet_count`, `mission_time_s`, `telem_enabled`, `sim_mode`, `sim_pressure_pa` |

---

## 7. Boot Sequence

```
app_main()
  │
  ├─ 1. system_init::core_init()   — NVS init, boot count increment, IDF log level
  │
  └─ main_fc() / main_gcs() / main_p4()   (role dispatch)
       │
       ├─ 1. Create FreeRTOS primitives (mutexes, event groups)
       ├─ 2. HAL init (I2C-0, I2C-1, SPI, UART-1 GNSS, UART-2 P4)
       ├─ 3. Driver init (BNO085, BMP585, N-GS-01, SX1278, INA260, MAX17048, SDP31, SHT4x, SGP41, CC1101)
       ├─ 4. Comms init (LoRa link + command parser callbacks)
       ├─ 5. Logging init (SD card mount + event log; SD failure is non-fatal)
       ├─ 6. Power manager init + low-battery callback
       ├─ 7. BIT — 10 hardware checks; HALT on critical failure (LED rapid blink)
       ├─ 8. FlightComputer init (EKF zero-state, IMM supervisor)
       ├─ 9. Control init (PID configure, motor mixer init, servo home)
       ├─ 10. RF Mapper init
       ├─ 11. Watchdog arm (15 s TWDT, panic on expiry)
       └─ 12. Spawn all FreeRTOS tasks
```

### BIT Failure Codes

| Bit | Flag | Severity |
|-----|------|----------|
| 0 | `BIT_IMU_ABSENT` — BNO085 not responding | **CRITICAL — halts** |
| 1 | `BIT_BARO_ABSENT` — BMP585 not responding | **CRITICAL — halts** |
| 2 | `BIT_POWER_ABSENT` — INA260 not responding | **CRITICAL — halts** |
| 4 | `BIT_LORA_ABSENT` — SX1278 not detected | **CRITICAL — halts** |
| 3 | `BIT_GNSS_NO_NMEA` — No NMEA in 2 s | Warning only |
| 5 | `BIT_SD_FAIL` — SD card not mounted | Warning only |
| 6 | `BIT_NVS_FAIL` — NVS namespace error | Warning only |
| 7 | `BIT_IMU_SANITY` — \|accel\| outside 7.8–11.8 m/s² | Warning only |
| 8 | `BIT_BARO_SANITY` — Pressure outside 70–110 kPa | Warning only |
| 9 | `BIT_VOLTAGE_LOW` — Voltage < 3.0 V | Warning only |

---

## 8. Command Reference

Uplink format: `<TEAM_ID>,<CMD>[,<ARG>]\n` over LoRa 433 MHz.

| Command | Argument | Action |
|---------|----------|--------|
| `CX,ON` | — | Enable telemetry downlink |
| `CX,OFF` | — | Disable telemetry downlink |
| `ST,HH:MM:SS` | Time string | Override mission timer |
| `CAL` | — | Capture current baro as ground reference (saves to NVS) |
| `SIM,ENABLE` | — | Enter simulation mode (baro overridden by SIMP) |
| `SIM,ACTIVATE` | — | Alias for ENABLE |
| `SIM,DISABLE` | — | Exit simulation mode |
| `SIMP,<pa>` | Pressure in Pa | Inject simulated pressure (SIM mode only) |
| `SIMG,<e,n,u,ve,vn,vu>` | Pos/Vel ENU | Inject simulated GNSS state |
| `SIMI,<ax,ay,az,gx,gy,gz>` | Accel/Gyro | Inject simulated IMU state |
| `ABORT` | — | Emergency abort: supervisor override + motor disarm |
| `CHUTE` | — | Manual parachute servo release |
| `RTL` | — | Trigger controlled descent sequence |
| `MAP` | — | Toggle RF mapping mission on/off |
| `OTA,START` | — | Initialize OTA update session |
| `OTA,CHUNK,<hex>` | Hex payload | Write firmware chunk to OTA partition |
| `OTA,FINISH` | — | Finalize OTA and reboot |

---

## 9. USB CLI (Maintenance Mode)

The firmware includes a built-in Command Line Interface accessible via the primary USB-C port (UART0) at **115,200 baud**. This allows for in-field configuration without the need for a LoRa ground station or firmware reflash.

### Basic Commands
- `help` — List all available commands.
- `status` — View current Team ID, Boot Count, and Ground Altitude.
- `reboot` — Soft reset the flight computer.

### Configuration (`get` / `set`)
Change persistent parameters stored in NVS:
- `get <key>` / `set <key> <val>`
- Examples: `set team_id 1234`, `set mag_cal 0.12 -0.05 0.33`
- Keys: `team_id`, `ground_alt`, `baro_offset`, `mag_cal`

### Command Injection (`dispatch`)
Inject any LoRa-style command. **Note:** Use commas for arguments, no spaces.
- `dispatch CAL` — Trigger ground altitude calibration.
- `dispatch SIM,ENABLE` — Enter simulation mode.
- `dispatch SIMG,12.3,45.6,789.0,0.0,0.0,0.0` — Inject GNSS state.
- `dispatch ABORT` — Immediate emergency disarm.

---

## 10. Telemetry Format

One packet per second via LoRa. Also mirrored to SD card (`CANSAT_XXXX.csv`).

```
<TEAM_ID>,<MISSION_TIME>,<PACKET_COUNT>,<ALTITUDE>,<PRESSURE>,<TEMPERATURE>,
<VOLTAGE>,<GNSS_TIME>,<LATITUDE>,<LONGITUDE>,<GNSS_ALT>,<SATS>,
<TILT_X>,<TILT_Y>,<ROT_Z>,<SOFTWARE_STATE>\n
```

**Example:**
```
1234,00:04:32,272,587.34,94312.5,18.3,7.41,07:15:44,12.971600,77.594600,912.30,8,0.12,-0.04,1.23,3
```

### LoRa Link Parameters

| Parameter | Value |
|-----------|-------|
| Frequency | 433 MHz |
| Spreading Factor | SF10 |
| Bandwidth | 125 kHz |
| Coding Rate | CR 4/5 |
| TX Power | 17 dBm (50 mW) |
| CRC | Enabled |
| Approx. Range | ~8 km line-of-sight |

---

## 11. Partition Layout

Flash: 8 MB total (`partitions.csv`)

| Partition | Type | Offset | Size | Purpose |
|-----------|------|--------|------|---------|
| `nvs` | data/nvs | 0x9000 | 24 KB | Team ID, ground alt, calibration |
| `otadata` | data/ota | 0xF000 | 8 KB | OTA slot selector |
| `phy_init` | data/phy | 0x11000 | 4 KB | PHY init data |
| `factory` | app/factory | 0x20000 | 3 MB | Main flight firmware |
| `ota_0` | app/ota_0 | 0x320000 | 3 MB | OTA update slot (active, v1.0) |
| `config_store` | data/nvs | 0x620000 | 64 KB | Extended config NVS |
| `event_log` | data/fat | 0x630000 | 1 MB | Flight event log (FAT, 256-event ring) |
| `coredump` | data/coredump | 0x730000 | 320 KB | Crash coredump capture |

---

## 12. Pre-Flight Checklist

- [ ] Team ID set in `config.hpp` and matches GCS configuration
- [ ] LoRa sync word matches team ID lower byte
- [ ] NVS erased on fresh board
- [ ] BIT passes cleanly (`flags=0x00000000` in serial log)
- [ ] Baro sanity: pressure within ±5 kPa of local sea level
- [ ] IMU sanity: `|accel| ≈ 9.81 m/s²` when flat
- [ ] GNSS fix acquired (≥ 4 satellites, fix_quality ≥ 1)
- [ ] SD card present and `CANSAT_XXXX.csv` file created at boot
- [ ] Telemetry confirmed on GCS (1 Hz packets, correct team ID)
- [ ] `CAL` command sent to zero ground altitude reference
- [ ] SIM mode verified ON and OFF (pressure injection test)
- [ ] Motor spin test: all 4 ESCs respond, servo homes correctly
- [ ] Battery voltage > 7.0 V (nominal 7.4 V)
- [ ] P4 link heartbeat active (`$P4HB,...` frames received by FC)
- [ ] Watchdog alive (no early TWDT panics during 60 s bench soak)

---

## 13. Documentation Index

| Document | Location | Description |
|----------|----------|-------------|
| **README.md** | `/` | This file — project overview and quick start |
| **ARCHITECTURE.md** | `docs/` | Full software architecture, nav stack, data flow |
| **BUILD_GUIDE.md** | `docs/` | Toolchain setup, build commands, error reference |
| **FLASH_GUIDE.md** | `docs/` | Flashing, port detection, first-boot procedure |
| **TESTING_GUIDE.md** | `docs/` | BIT interpretation, bench test, integration test |
| **HARDWARE_SETUP.md** | `docs/` | Wiring diagrams, pinout, PCB recommendations |
| **TELEMETRY_FORMAT.md** | `docs/` | CSV field definitions, LoRa params, GCS example |
| **CHANGELOG.md** | `docs/` | Version history and planned features |

---

## 14. License

© 2026 SVNIT. All Rights Reserved.  
See `LICENSE` for full terms.

---

*AAKASHVANI Flight Software — CAN-7USAT India 2026 | Compliance: §3, §5, §6*
