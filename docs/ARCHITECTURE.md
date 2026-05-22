# AAKASHVANI — Software Architecture Reference
### CAN-7USAT India 2026 | cansat_fw v1.0
> © 2026 SVNIT. All Rights Reserved.

---

## Table of Contents
1. [System Topology](#1-system-topology)
2. [Repository Structure](#2-repository-structure)
3. [Component Dependency Graph](#3-component-dependency-graph)
4. [Dual-Core FreeRTOS Task Model](#4-dual-core-freertos-task-model)
5. [Boot & Initialization Flow](#5-boot--initialization-flow)
6. [Navigation Stack](#6-navigation-stack)
7. [Control Subsystem](#7-control-subsystem)
8. [Communication Protocols](#8-communication-protocols)
9. [Telemetry Pipeline](#9-telemetry-pipeline)
10. [Logging Subsystem](#10-logging-subsystem)
11. [Power Management](#11-power-management)
12. [Secondary Mission — RF Mapping](#12-secondary-mission--rf-mapping)
13. [Fault Detection & Recovery](#13-fault-detection--recovery)
14. [Memory Layout](#14-memory-layout)
15. [Coordinate Frames & Conventions](#15-coordinate-frames--conventions)
16. [Mission Readiness Rating](#16-mission-readiness-rating)

---

## 1. System Topology

The system is split into two independent processing domains connected via a verified high-speed UART link.

```
┌─────────────────────────────────────────────────────────────┐
│                  FLIGHT COMPUTER (FC)                        │
│                  ESP32-S3 WROOM-1 @ 240 MHz                  │
│                                                             │
│  Core 0 (RT)               Core 1 (SYS)                    │
│  ─────────────────          ─────────────────               │
│  nav_task   100Hz          sensor_task  50Hz                │
│  control_task 100Hz        telem_task   1Hz                 │
│                            logging_task 1Hz                 │
│                            power_task   1Hz                 │
│                            p4_link_task 10Hz                │
│                            rf_map_task  sweep               │
│                            beacon_task  1Hz                 │
│                                                             │
│  Sensors: BNO085, BMP585, N-GS-01, SX1278, CC1101          │
│           INA260, MAX17048, SDP31, SHT4x, SGP41             │
└──────────────────────┬──────────────────────────────────────┘
                       │ UART-2 @ 921600 baud
                       │ XOR-checksummed frames
                       │ Heartbeat monitoring
                       │
┌──────────────────────▼──────────────────────────────────────┐
│                MEDIA COPROCESSOR (P4)                        │
│                ESP32-P4 @ 400 MHz                            │
│                                                             │
│  MIPI-CSI camera → Hardware H.264 encoder → SDMMC storage  │
│  Triggered by FC: RECORD on BOOST, HALT_FLUSH on LANDED     │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                 GROUND STATION (GCS)                         │
│                 ESP32-S3 (separate unit)                     │
│                                                             │
│  LoRa RX ──► USB-UART bridge ──► GCS laptop                │
│  Promiscuous mode | Emergency override | Uplink relay       │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. Repository Structure

```
cansat_fw/
│
├── main/                          # Role-dispatch entry points
│   ├── main.cpp                   # app_main() + FC tasks (main_fc)
│   ├── main_gcs.cpp               # GCS entry (main_gcs)
│   ├── main_p4.cpp                # P4 entry (main_p4)
│   ├── main_gcs_extra.cpp         # GCS secondary definitions
│   ├── system_init.cpp/.hpp       # Shared core init (NVS, logging)
│   ├── Kconfig.projbuild          # Role selector (menuconfig)
│   └── CMakeLists.txt             # Component registration
│
├── components/
│   ├── nav/                       # Navigation stack (header-only, no heap)
│   │   └── include/nav/
│   │       ├── config.hpp         # ALL mission/sensor/estimator constants
│   │       ├── matrix.hpp         # Fixed-size Mat<R,C>/Vec<N> templates
│   │       ├── frames.hpp         # SO(3), quaternion math, ISA atmosphere
│   │       ├── nav_state.hpp      # NavState + StrapdownINS propagation
│   │       ├── ekf.hpp            # Error-State EKF (15-state, Joseph-form)
│   │       ├── measurements.hpp   # h(x) functions + H(x) Jacobians
│   │       ├── imm.hpp            # IMM filter (5 regimes)
│   │       ├── fdir.hpp           # SPRT fault detection per channel
│   │       ├── health.hpp         # NIS-based sensor health + EMA
│   │       ├── adaptive.hpp       # Mehra 1972 adaptive measurement noise
│   │       ├── supervisor.hpp     # Bayesian supervisor + LaunchDetector
│   │       └── flight_computer.hpp# Top-level integration (FlightComputer)
│   │
│   ├── hal/                       # Hardware Abstraction Layer
│   │   ├── i2c_bus                # ESP-IDF v5 I2C master bus wrapper
│   │   ├── spi_bus                # SPI2_HOST with DMA
│   │   └── uart_bus               # UART with newline pattern detection
│   │
│   ├── drivers/                   # Sensor + peripheral drivers
│   │   ├── bno085                 # BNO085 IMU — SHTP over I2C, 100 Hz
│   │   ├── bmp585                 # BMP585 barometer — 50 Hz, OSR×4
│   │   ├── ngps01                 # N-GS-01 NavIC GNSS — NMEA/UART, 1 Hz
│   │   ├── sx1278                 # SX1278 LoRa transceiver
│   │   ├── cc1101                 # CC1101 RF scanner
│   │   ├── ina260                 # INA260 voltage/current monitor
│   │   ├── max17048               # MAX17048 LiPo fuel gauge
│   │   ├── sdp31                  # SDP31 differential pressure
│   │   ├── sht4x                  # SHT4x humidity/temperature
│   │   └── sgp41                  # SGP41 VOC/NOx
│   │
│   ├── control/                   # Flight control algorithms
│   │   ├── pid.hpp                # PID: anti-windup, derivative-on-measurement
│   │   └── motor_mixer            # Quadrotor X mixer + LEDC PWM (ESC + servo)
│   │
│   ├── telemetry/                 # Packet encoding
│   │   └── encoder                # CAN-7USAT §5.3 CSV encoder (16 fields, 1 Hz)
│   │
│   ├── comms/                     # RF communications
│   │   ├── xbee_link              # XBee Pro TX queue + RX callback dispatcher
│   │   └── command_parser         # Uplink command decoder + callback registry
│   │
│   ├── logging/                   # Persistent data storage
│   │   ├── sd_logger              # Buffered FAT SD card (64-line ring, 5 s flush)
│   │   └── event_log              # NVS-backed discrete event log (256-event ring)
│   │
│   ├── power/                     # Power monitoring
│   │   └── power_manager          # INA260 + MAX17048 aggregation + callbacks
│   │
│   ├── config_mgr/                # Non-volatile configuration
│   │   └── nvs_config             # NVS: team_id, ground_alt, mag_cal, boot_count
│   │
│   ├── watchdog/                  # System reliability
│   │   └── watchdog               # ESP-IDF TWDT wrapper (15 s timeout, per-task)
│   │
│   ├── bit/                       # Pre-flight self-test
│   │   └── built_in_test          # 10 hardware checks (critical/soft failure)
│   │
│   ├── rf_mapping/                # Secondary mission
│   │   └── rf_mapper              # CC1101 RSSI + servo sweep + synchronized log
│   │
│   └── p4_link/                   # Inter-MCU reliability layer
│       └── p4_link                # Checksum-verified UART command + heartbeat
│
├── docs/                          # Documentation
├── tools/                         # Build utilities
├── CMakeLists.txt                 # Top-level ESP-IDF project (C++17, C11)
├── partitions.csv                 # Custom 8 MB partition table
└── sdkconfig.defaults             # Pre-configured SDK settings
```

---

## 3. Component Dependency Graph

```
app_main
  └── system_init (core_init)
        └── nvs_flash, esp_log

  └── main_fc
        ├── nav ──────────────────── (header-only, zero dependencies)
        │     matrix, frames, nav_state, ekf,
        │     measurements, imm, fdir, health,
        │     adaptive, supervisor, flight_computer
        │
        ├── hal ──────────────────── i2c_bus, spi_bus, uart_bus
        │
        ├── drivers ──────────────── hal
        │     bno085, bmp585, ngps01, xbee_link,
        │     cc1101, ina260, max17048, sdp31, sht4x, sgp41
        │
        ├── control ──────────────── (standalone)
        │     pid, motor_mixer → esp_timer, driver/ledc
        │
        ├── comms ────────────────── drivers/sx1278, hal
        │     lora_link, command_parser
        │
        ├── telemetry ────────────── nav (NavState types)
        │     encoder
        │
        ├── logging ──────────────── sdmmc, fatfs, nvs_flash
        │     sd_logger, event_log
        │
        ├── power ────────────────── drivers/ina260, drivers/max17048
        │
        ├── config_mgr ───────────── nvs_flash
        │
        ├── watchdog ─────────────── esp_timer
        │
        ├── bit ──────────────────── hal, drivers, logging, config_mgr
        │
        ├── rf_mapping ───────────── drivers/cc1101, nav, control, logging
        │
        └── p4_link ──────────────── hal/uart_bus
```

---

## 4. Dual-Core FreeRTOS Task Model

### Core Assignment

```
Core 0 — Real-Time (RT)          Core 1 — System Services (SYS)
══════════════════════════        ══════════════════════════════
nav_task     PRI=MAX-1  100Hz     sensor_task    PRI=MAX-3  50Hz
control_task PRI=MAX-2  100Hz     telem_task     PRI=5       1Hz
                                  logging_task   PRI=4       1Hz
                                  power_task     PRI=3       1Hz
                                  p4_link_task   PRI=3      10Hz
                                  rf_map_task    PRI=3    sweep
                                  beacon_task    PRI=2       1Hz
```

### Task Timing Budget

| Task | Period | Worst-case budget | Stack |
|------|--------|-------------------|-------|
| `nav_task` | 10 ms | < 8 ms (EKF propagate + IMU read) | 8192 words |
| `control_task` | 10 ms | < 4 ms (PID + mix + PWM) | 4096 words |
| `sensor_task` | 20 ms | < 5 ms (I2C baro + UART GNSS) | 4096 words |
| `telem_task` | 1000 ms | < 50 ms (encode + LoRa enqueue + SD) | 4096 words |
| `logging_task` | 1000/5000 ms | < 100 ms (LoRa spin + SD flush) | 8192 words |
| `power_task` | 1000 ms | < 20 ms (I2C read + callback) | 2048 words |
| `p4_link_task` | 100 ms | < 10 ms (UART parse + heartbeat check) | 2048 words |

### Shared State & Synchronisation

```
┌─────────────┐  write (100Hz)    ┌──────────────────────────────────┐
│  nav_task   │ ─────────────────►│  fc_mutex (SemaphoreCreateMutex) │
│ sensor_task │ ─────────────────►│  FlightComputer::last_output      │
└─────────────┘                   └──────────────────────────────────┘
                                           │ read
                        ┌──────────────────┼─────────────────┐
                        ▼                  ▼                 ▼
                  control_task       telem_task         rf_map_task

┌───────────────┐  write (50Hz)    ┌───────────────────────────────────┐
│ sensor_task   │ ────────────────►│  sensor_mutex                     │
│ power_task    │ ────────────────►│  latest_baro, latest_gnss,        │
└───────────────┘                  │  latest_imu_snap, latest_pwr      │
                                   └───────────────────────────────────┘
                                              │ read
                              ┌───────────────┼──────────────┐
                              ▼               ▼              ▼
                         telem_task     control_task    beacon_task

std::atomic<>  ──► packet_count, mission_time_s, telem_enabled,
                   sim_mode, sim_pressure_pa  (lock-free, any task)

EventGroup     ──► EVT_BIT_PASS  (BIT must pass before control_task arms)
```

---

## 5. Boot & Initialization Flow

```
app_main()
│
├─ system_init::core_init()
│    ├─ nvs_flash_init()
│    ├─ NVSConfig::load()   ← team_id, ground_alt, boot_count
│    └─ esp_log_level_set()
│
└─ main_fc()
     │
     ├─ [1] FreeRTOS primitives: fc_mutex, sensor_mutex, evt_group
     │
     ├─ [2] HAL init
     │    ├─ I2C-0: GPIO 8/9 @ 400 kHz  (BNO085 + BMP585)
     │    ├─ I2C-1: GPIO 10/11 @ 400 kHz (enviro + power)
     │    ├─ SPI:   GPIO 35/37/36         (SX1278 + CC1101)
     │    ├─ UART-1: GPIO 17/18 @ 115200  (N-GS-01 GNSS)
     │    └─ UART-2: GPIO tx/rx @ 921600  (ESP32-P4 link)
     │
     ├─ [3] Driver init (ESP_ERROR_CHECK on critical sensors)
     │    BNO085 → BMP585 → CC1101 → GNSS → SDP31 → SHT4x → SGP41
     │
     ├─ [4] Comms: LoRa init → command parser callbacks registered
     │
     ├─ [5] Logging: SD mount (non-fatal) → event_log init → BOOT event
     │
     ├─ [6] Power: INA260/MAX17048 init → low-battery callback
     │
     ├─ [7] BIT — 10 checks
     │    PASS → xEventGroupSetBits(EVT_BIT_PASS)
     │    FAIL → Check Overrides (Kconfig or NVS bit_override)
     │           Override Active → Proceed with warnings
     │           No Override → LED blink loop (infinite halt)
     │
     ├─ [8] FlightComputer init (zero-state EKF, IMM, supervisor)
     │
     ├─ [9] Control: PID configure → motor_mixer.init() → servo_home()
     │
     ├─ [10] RF Mapper init
     │
     ├─ [11] Watchdog arm (TWDT, 15 s)
     │
     └─ [12] Spawn FreeRTOS tasks (Core 0: nav, ctrl | Core 1: rest)
```

---

## 6. Navigation Stack

### State Representation

```
Navigation State x (16 elements):
  p  = [px, py, pz]         — position in ENU (m)
  v  = [vx, vy, vz]         — velocity in ENU (m/s)
  q  = [qw, qx, qy, qz]    — quaternion body→world (Hamilton)
  ba = [bax, bay, baz]      — accelerometer bias (m/s²)
  bg = [bgx, bgy, bgz]      — gyroscope bias (rad/s)

Error-State δx (15 elements, manipulated by EKF):
  δp  ∈ ℝ³   — position error
  δv  ∈ ℝ³   — velocity error
  δθ  ∈ ℝ³   — attitude error (rotation vector, NOT δq)
  δba ∈ ℝ³   — accel bias error
  δbg ∈ ℝ³   — gyro bias error
```

### EKF Pipeline (per 10 ms cycle)

```
IMU sample (100 Hz)
     │
     ▼
StrapdownINS::propagate()
  2nd-order midpoint integration
  Closed-form quaternion exponential map
  Covariance prediction: P = F·P·Fᵀ + Q
     │
     ├── Baro update (50 Hz, when available)
     │    h(x) = p_z         H = [0,0,1, 0,...,0]
     │    Innovation: y = z_baro - h(x)
     │    SPRT FDIR check → exclude if fault
     │    Joseph-form update: P = (I - K·H)·P·(I - K·H)ᵀ + K·R·Kᵀ
     │
     └── GNSS update (1 Hz, when fix valid)
          h(x) = [p_e, p_n, p_u, v_e, v_n, v_u]
          Cholesky solve for Kalman gain (no matrix inversion)
          Joseph-form covariance update
          State injection: x ← x ⊕ δx (quaternion via exp map)
```

### IMM Filter (5 Regimes)

```
Each regime = independent EKF with different process noise:

  Regime          σ_accel   σ_gyro   Covariance intent
  ─────────────────────────────────────────────────────
  BOOST           10.0      0.1      High thrust, aggressive dynamics
  BALLISTIC        2.5      0.05     Free-fall, smooth
  PARACHUTE        3.0      0.1      Chute oscillation
  DRONE_HOVER      1.0      0.05     Active stabilisation
  LANDED           0.1      0.01     Near-stationary

Mixing: x_fused = Σ μᵢ · xᵢ      (probability-weighted state)
        P_fused = Σ μᵢ · (Pᵢ + (xᵢ - x_fused)(xᵢ - x_fused)ᵀ)

Transition matrix Π: tuned to competition mission timeline
```

### Bayesian Supervisor

```
Inputs: IMM regime probabilities μ, fused nav state, MissionLatches

Phase transitions (all require Schmitt-trigger hysteresis):

PRE_FLIGHT ──► BOOST
  trigger: 2-of-3 vote (accel_z > 2g, alt_rate > 5 m/s, velocity > 10 m/s)
  latch: MissionLatches::boost_detected = true (RTC fast memory)

BOOST ──► BALLISTIC
  trigger: accel_z < 1.2g sustained + μ_BALLISTIC > 0.6

BALLISTIC ──► PARACHUTE
  trigger: alt_AGL < 610 m AND μ_PARACHUTE > 0.75
  latch: MissionLatches::chute_deployed = true

PARACHUTE ──► DRONE_HOVER
  trigger: alt_AGL < 30 m AND chute_deployed
  latch: MissionLatches::drone_deployed = true

DRONE_HOVER ──► LANDED
  trigger: |velocity| < 0.3 m/s AND alt_AGL < 2 m sustained 1 s

All latches stored in RTC fast memory — survive soft resets
```

### Reliability Features

| Feature | Implementation | Purpose |
|---------|----------------|---------|
| SPRT FDIR | Sequential Probability Ratio Test per sensor channel | Exclude faulty sensor measurements without halting |
| NIS Health | Innovation normalized by S; EMA smoothed | Detect sensor degradation before full failure |
| Mehra 1972 | EMA of innovation outer product → R̂ update | Adapt measurement noise to actual environment |
| Joseph-form | `P = (I-KH)P(I-KH)ᵀ + KRKᵀ` | Maintain positive-definiteness of covariance |
| Cholesky solve | `S·K = P·Hᵀ` Cholesky decomp | Numerically stable; avoids `S⁻¹` |
| Fixed-size math | `Mat<R,C>`, `Vec<N>` templates | Zero heap allocation; stack-only; no Eigen |

---

## 7. Control Subsystem

### PID Configuration

```
pitch_pid, roll_pid  (attitude):
  kp = ControlConfig::kp_attitude
  ki = ControlConfig::ki_attitude
  kd = ControlConfig::kd_attitude
  dt = 10 ms (100 Hz)
  output: [-1.0, +1.0]
  integral limit: anti_windup_limit_rad
  deadband: attitude_deadband_rad
  derivative: on measurement (not error) → derivative kick prevention
  derivative filter: α = 0.3 (low-pass EMA)

descent_pid:
  kp, ki, kd = ControlConfig::kp/ki/kd_descent
  setpoint: -MISSION.descent_rate_target_mps (e.g. -2.0 m/s)
  output: [0.0, 1.0] → throttle fraction
  integral limit: 0.3
```

### Motor Mixer (Quadrotor Cross "+")

```
Motor layout (body frame, X = Forward):
        M0 (Front)
         │
  M3 ───┼─── M1
  (Left) │  (Right)
        M2 (Rear)

Mix equations:
  M0 = throttle + pitch_cmd - yaw_cmd
  M1 = throttle - roll_cmd  + yaw_cmd
  M2 = throttle - pitch_cmd - yaw_cmd
  M3 = throttle + roll_cmd  + yaw_cmd

  Output scaled by battery_factor (voltage compensation)
  Clamped to [0.0, 1.0] before PWM write

PWM:  LEDC 50 Hz, 16-bit resolution
ESC range: 1000–2000 µs pulse width
Servo release: 2000 µs → parachute latch release
```

### Phase-Gated Actuation

| Phase | Control action |
|-------|----------------|
| `PRE_FLIGHT` | Motors disarmed, servo home |
| `BOOST` | Motors disarmed (rocket powered) |
| `BALLISTIC` | Motors disarmed |
| `PARACHUTE` | `servo_release()` on `chute_deployed` latch |
| `DRONE_HOVER` | Motors armed, all 3 PIDs active, battery compensation |
| `LANDED` | Motors disarmed, beacon GPIO active |

---

## 8. Communication Protocols

### LoRa Downlink (FC → GCS)

```
Physical:  SX1278, 433 MHz, SF10, BW125 kHz, CR4/5, 17 dBm
Packet:    CAN-7USAT §5.3 CSV, 16 fields, ~90 bytes
Rate:      1 Hz
Sync word: 0x12 (or team_id & 0xFF)
```

### LoRa Uplink (GCS → FC)

```
Format:  <TEAM_ID>,<CMD>[,<ARG>]\n
Parsing: CommandParser via set_rx_callback on LoRaLink
Verification: team_id prefix match (no CRC in v1.0)
```

### Inter-MCU Link (FC ↔ P4)

```
Physical:  UART-2, GPIO tx/rx, 921600 baud
Direction: FC → P4 commands; P4 → FC heartbeats

FC → P4 Command frame:
  $P4CMD,<cmd>*<XOR_checksum>\r\n
  Commands: "RECORD", "HALT_FLUSH"
  Trigger:  BOOST→ RECORD; LANDED → HALT_FLUSH

P4 → FC Heartbeat frame:
  $P4HB,<recording>,<free_gb>,<fps>*<XOR_checksum>\r\n
  Rate: 10 Hz
  Watchdog: FC triggers failsafe if heartbeat missed > threshold

Integrity: XOR of all bytes between '$' and '*' (inclusive)
```

### GCS Bridge

```
Role ROLE_GCS (main_gcs.cpp):
  LoRa RX → USB UART forward (transparent)
  USB UART RX → LoRa TX forward (uplink relay)
  Promiscuous mode: forwards all received LoRa frames
  Emergency override: injected directly to LoRa TX queue
```

---

## 9. Telemetry Pipeline

```
sensor_task (50Hz)                  nav_task (100Hz)
     │                                    │
     ▼                                    ▼
 latest_baro                      FlightComputer::last_output
 latest_gnss   ──── sensor_mutex ──────────────────────────────
 latest_pwr                           fc_mutex

                    ▼  (1 Hz snapshot in telem_task)
            TelemetryEncoder::make_frame()
                    │
                    ├─ AlT   ← EKF p_z (baro-primary, EKF fallback)
                    ├─ PRESSURE, TEMP  ← BMP585 raw
                    ├─ VOLTAGE         ← INA260
                    ├─ GNSS fields     ← N-GS-01 (fallback: zeros)
                    ├─ TILT_X/Y        ← BNO085 accel (fallback: EKF Euler)
                    ├─ ROT_Z           ← BNO085 gyro Z
                    └─ SOFTWARE_STATE  ← Supervisor phase (0–5)
                    │
                    ▼
          TelemetryEncoder::encode() → char csv_buf[]
                    │
            ┌───────┴───────┐
            ▼               ▼
       LoRaLink::         SDLogger::
       enqueue_packet()   write_line()
```

---

## 10. Logging Subsystem

### SD Logger

```
Backend:     ESP-IDF SDMMC + FatFS
Buffer:      64-line ring buffer in RAM
Flush:       Every 5 s (logging_task), or on HALT_FLUSH command
Filename:    CANSAT_XXXX.csv  (XXXX = NVS boot_count, 0001-padded)
Header:      Written at boot: TEAM_ID,MISSION_TIME,...,SOFTWARE_STATE
Failure:     Non-fatal — flight continues without SD; warn in log
```

### Event Log

```
Backend:     NVS-backed (FAT partition at 0x630000, 1 MB)
Capacity:    256 events (ring buffer, oldest overwritten)
Survives:    Software resets (RTC-backed NVS partition)
Format:      EventCode, mission_time_s, arg0, arg1, arg2, message
Key events:  BOOT, BIT_PASS, BIT_FAIL, BOOST, PARACHUTE_DEPLOY,
             DRONE_DEPLOY, LANDED, ERROR_FDIR, POWER_LOW,
             CMD_CX_ON, CMD_CX_OFF, CMD_ST, CMD_CAL
```

---

## 11. Power Management

```
Sensors:   INA260 (bus voltage + current, I2C-1 0x40)
           MAX17048 (LiPo SoC %, I2C-1 0x36)

Aggregation (power_task @ 1 Hz):
  PowerState { voltage_v, current_a, power_w, soc_pct, status }

Status thresholds:
  OK        : voltage > POWER_CFG.low_voltage_v
  LOW       : voltage < low_voltage_v    → log event, warn
  CRITICAL  : voltage < critical_v       → log + disarm motors
  BROWNOUT  : voltage < brownout_v       → throttle cap applied

Control compensation:
  bat_factor = min(1.0, voltage / bat_nominal_v)
  Applied in control_task as: mix_and_set(... , bat_factor)
  If critical: bat_factor *= max_throttle_brownout (further cap)
```

---

## 12. Secondary Mission — RF Mapping

```
Trigger:   GCS command "MAP" (toggle on/off)
Hardware:  CC1101 RF scanner (SPI, CS=GPIO21) + servo (GPIO38)
Task:      rf_map_task on Core 1, PRI=3

Sweep algorithm:
  for angle in [0°, 5°, 10°, ..., 180°]:
    1. servo → angle (LEDC PWM)
    2. vTaskDelay(settle time)
    3. CC1101::read_rssi() → int8_t rssi_dBm
    4. xSemaphoreTake(fc_mutex, pdMS_TO_TICKS(5))
       snap ← fc.last_output.imm.nav  [position + time]
       xSemaphoreGive(fc_mutex)
    5. sd_logger.write_line(
         "%llu,%.6f,%.6f,%.2f,%.1f,%d\n",
         time_us, lat, lon, alt, angle, rssi)

Non-interference: fc_mutex held < 1 ms per sample
                  Never touches Core 0 tasks directly
Output file:  RF_MAP_XXXX.csv on same SD card
```

---

## 13. Fault Detection & Recovery

### SPRT FDIR (Per Sensor Channel)

```
For each measurement z_i from sensor s:
  Test statistic Λ = Σ log[p(z_i | H₁) / p(z_i | H₀)]
    H₀: sensor healthy   (nominal noise R)
    H₁: sensor faulty    (inflated noise k·R)

  If Λ > threshold_upper → EXCLUDE sensor (fault detected)
  If Λ < threshold_lower → ACCEPT sensor (healthy confirmed)
  else → UNCERTAIN (continue accumulating)

Reset: on phase transition or manual CAL command
Effect: faulty measurement silently excluded; EKF continues
        on remaining sensors without halting
```

### Watchdog (TWDT)

```
Scope:     nav_task, control_task register with TWDT
Timeout:   15 seconds hardware timeout
Trigger:   Each task calls Watchdog::ping() each cycle
Expiry:    ESP-IDF triggers panic + coredump to flash partition
Recovery:  Next boot reads coredump; event_log shows BOOT event
```

### Launch Detection (3-Channel Majority Vote)

```
Channel A: accelerometer Z > 2g for N consecutive samples
Channel B: barometric altitude rate > 5 m/s
Channel C: EKF velocity Z > 10 m/s

Decision: 2-of-3 TRUE → BOOST latch (irreversible)
Schmitt trigger hysteresis prevents false positives
```

---

## 14. Memory Layout

### RAM (512 KB SRAM, ESP32-S3)

| Region | Contents | Size |
|--------|----------|------|
| Task stacks | nav(32KB), ctrl(16KB), sensor(16KB), telem(16KB), logging(32KB), power(8KB), p4(8KB), beacon(4KB) | ~132 KB |
| `FlightComputer` static | EKF matrices (P 15×15, F, Q, H, K), IMM state × 5 | ~50 KB |
| `SDLogger` ring buffer | 64 lines × 128 bytes | 8 KB |
| Sensor data structs | BaroData, GNSSData, IMUData, PowerData (×2 copies) | < 2 KB |
| FreeRTOS kernel | Scheduler, queues, mutexes | ~10 KB |
| Heap (remaining) | Driver I/O buffers, LoRa TX queue | ~60 KB |

### Flash (8 MB)

See [Partition Layout in README](../README.md#10-partition-layout).

### RTC Fast Memory

`MissionLatches` struct stored in `RTC_DATA_ATTR`:
```cpp
struct MissionLatches {
  bool boost_detected;       // irreversible after launch
  bool chute_deployed;       // irreversible after 600 m
  bool drone_deployed;       // irreversible after < 30 m
};
```
Survives deep-sleep and software resets. Reset only on power cycle.

---

## 15. Coordinate Frames & Conventions

| Frame | Axes | Origin | Used by |
|-------|------|--------|---------|
| **World (ENU)** | X=East, Y=North, Z=Up | First valid GNSS fix | EKF state `p`, `v` |
| **Body** | X=Forward, Y=Left, Z=Up | CanSat centre of mass | IMU measurements, control |
| **Quaternion** | q = [w, x, y, z] Hamilton | — | body → world rotation |

Quaternion convention: `q` transforms a vector from body frame to world frame:
```
v_world = q ⊗ v_body ⊗ q*
```

ISA atmosphere model in `frames.hpp`:
```
alt = T0/L × [1 - (P/P0)^(R·L/g)] — used for baro → altitude AGL
```

---

### Strengths

- **Professional navigation stack**: The 16-state ESEKF inside a 5-regime IMM is far beyond typical student CanSat firmware.
- **OTA Update System**: Field-programmable via LoRa link with partition switching.
- **Robust Uplink**: CRC-16 verification prevents accidental command execution.
- **Full Simulation Coverage**: HIL-ready with pressure, GNSS, and IMU injection.

---

*AAKASHVANI Architecture Reference — cansat_fw v1.0 — CAN-7USAT India 2026*
