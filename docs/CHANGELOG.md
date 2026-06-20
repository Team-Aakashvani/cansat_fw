# Changelog — CAN-7USAT 2026 Flight Software

## v1.1.1 (2026-06-01) — Hardware Synchronization

### Added
- **Bill of Materials (BOM)**: Added `docs/COMPONENTS.md` with specific part numbers for motors, ESCs, servos, and sensors.
- **Battery Specifications**: Standardized on 18650 Molicel 35A Li-ion cells for high-current discharge.

### Changed
- **Documentation Update**: Synchronized `HARDWARE_SETUP.md`, `WIRING.md`, and `README.md` with specific user-provided hardware (HAKRC ESC, BetaFPV motors, Linear Servo).
- **Private Memory**: Established private component tracking in `.cache/tmp` for internal audit.

## v1.1.0 (2026-05-27) — Major Refactor & P4 Deprecation

### Added
- **Cascaded PID Controller**: Dual-loop architecture (angle outer, rate inner) for superior attitude stability.
- **MotorMixerX**: Standalone quadcopter 'X' mixer logic.
- **Altitude-Based Mission Logic**: Automated arming and arm-latching (servo) at 600m ±10m.
- **Descent-Rate Control**: Maintains vertical velocity between 1.0 and 3.0 m/s during drone phase.

### Removed
- **ESP32-P4 Support**: Deprecated and removed all digital video coprocessor components and `p4_link`.
- **Digital Video Tasks**: Removed heartbeat monitoring and recording triggers in favor of standalone analog FPV.
- **Telemetry Fields**: Excised `P4_REC`, `P4_SD`, and `P4_FPS` from downlink format.

### Changed
- **Pin Reclamation**: GPIO 26 and 27 are now unassigned and available for future use.
- **Build System**: Cleaned up dependencies, removing `esp_video` and `esp_h264`.

## v1.0.0 (2026-05-19) — Flight Release

### Added

**Navigation Stack**
- Error-State EKF (16-state: position, velocity, quaternion, accel bias, gyro bias)
- **Magnetometer Fusion**: 3-axis magnetic field integration for heading stability
- IMM filter with 5 dynamic regimes (BOOST, BALLISTIC, PARACHUTE, DRONE_HOVER, LANDED)
- Bayesian Supervisor v2.1 with Schmitt-trigger hysteresis and MissionLatches
- Independent 3-channel launch detector with 2-of-3 majority vote
- SPRT (Sequential Probability Ratio Test) FDIR per sensor channel
- NIS-based sensor health monitor with exponential smoothing
- Mehra 1972 EMA adaptive measurement noise covariance
- Strapdown INS propagation (2nd-order midpoint, closed-form quaternion exp)
- Joseph-form covariance update for numerical stability
- Cholesky innovation solve (no full matrix inversion)
- Fixed-size Mat<R,C>/Vec<N> template library (no Eigen, no heap)

**Hardware Drivers**
- BNO085 IMU (SHTP over I2C, 100 Hz accel + calibrated gyro + magnetometer)
- BMP585 barometer (50 Hz, OSR×4, IIR filter)
- N-GS-01 NavIC GNSS (NMEA-0183 UART, 1 Hz, ENU conversion)
- SX1278 LoRa transceiver (433 MHz, SF10, BW125, CR4/5)
- INA260 precision current/voltage monitor (I2C)
- MAX17048 LiPo fuel gauge (I2C)
- SDP31 differential pressure / airspeed (I2C)
- SHT4x humidity + temperature (I2C)
- SGP41 VOC/NOx sensor (I2C)

**Control**
- PID controller with anti-windup (conditional integration), derivative-on-measurement, deadband
- Quadrotor cross (+) motor mixer with battery factor compensation
- LEDC 50 Hz 16-bit PWM for 4× ESCs and 1× servo
- Servo release (parachute deployment) at 2000 µs

**Communications**
- **Robust LoRa Link**: CCITT CRC-16 integrity verification for all uplink commands
- **OTA Service**: Over-the-air firmware updates via LoRa maintenance commands
- Downlink: 1 Hz CAN-7USAT CSV telemetry
- Uplink: CX / ST / CAL / SIM / SIMP / SIMG / SIMI / OTA / ABORT commands

**CLI (New)**
- **USB/Serial Maintenance Console**: In-field NVS configuration and command injection via UART0

**Telemetry**
- 16-field CAN-7USAT §5.3 compliant CSV encoder
- Fallback values when sensor data is invalid

**Logging**
- Buffered FAT/SDMMC SD card logger (64-line ring buffer, 5 s flush)
- **Coredump Auto-Export**: Automatic binary crash log export to SD card on boot
- NVS-backed discrete flight event log (256-event ring, survives resets)
- CSV header written at boot; unique filename per boot count

**Power Management**
- INA260 + MAX17048 aggregation
- Voltage threshold callbacks (LOW_VOLTAGE, CRITICAL, BROWNOUT)
- Motor throttle cap during low battery

**System**
- Built-In Test (BIT): 10 hardware checks with critical/soft failure classification
- Task-level TWDT watchdog (15 s hardware timeout, panic on expiry)
- NVS-backed config: team ID, ground altitude, magnetometer calibration, boot count
- Dual-core FreeRTOS: Core 0 = RT (nav + control @100 Hz), Core 1 = SYS services
- Recovery audio beacon on GPIO39 (1 Hz, 50% duty after LANDED)
- RTC fast memory reserved for irreversible MissionLatches

**Documentation**
- README.md — project overview and quick start
- docs/BUILD_GUIDE.md — toolchain setup, build commands, error reference
- docs/FLASH_GUIDE.md — flashing, COM port detection, post-flash checklist
- docs/TESTING_GUIDE.md — BIT, bench test, integration test, pre-flight checklist
- docs/HARDWARE_SETUP.md — full wiring guide, pinout, PCB recommendations
- docs/TELEMETRY_FORMAT.md — CSV format, LoRa params, GCS Python example
- docs/ARCHITECTURE.md — task model, nav stack, data flow, memory layout

### Known Limitations (v1.0.0)

- SD card mount failure is non-fatal; flight continues without SD logging
- Single GCS link; no multi-hop or relay support in v1.0
- Magnetometer soft-iron only; manual NVS calibration required for hard-iron offset
- No hardware security; LoRa link lacks encryption (CRC-only integrity)

---

## Planned — Next Release

- Magnetometer hard-iron calibration via `CAL_MAG` uplink command
- Runtime telemetry rate adjustment (0.5–2 Hz)
- Automated event log export to SD card root as `EVENTS_XXXX.csv`
