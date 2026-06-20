# AAKASHVANI — CAN-7USAT India 2026 Flight Software
### Version 1.1.0 (Refactor Release) | SVNIT Aerospace Team
> **State-of-the-Art Avionics Stack for High-Altitude Research & Autonomous Recovery**

---

## 1. Project Mission & Overview

**AAKASHVANI** (Sanskrit: *Voice from the Sky*) is the primary flight software for the SVNIT CanSat, designed for the CAN-7USAT India 2026 competition. The system is engineered to manage high-dynamic launch conditions, transition through multiple flight regimes (Ballistic, Parachute, Drone), and execute a precision autonomous descent using a quadcopter 'X' configuration.

### Mission Profile Summary
The flight follows a strictly defined six-phase lifecycle:
1.  **PRE_FLIGHT:** On-pad diagnostics, sensor calibration, and launch detection arming.
2.  **BOOST:** Ascent under solid rocket motor power (detects >2g acceleration).
3.  **BALLISTIC:** Coasting phase from burnout to apogee (~1000m).
4.  **PARACHUTE:** Parachute deployment at 600m ±10m AGL, initiating passive stabilized descent.
5.  **DRONE_HOVER:** Transition to active motor-stabilized flight (1–3 m/s descent rate).
6.  **LANDED:** Touchdown detection, motor disarming, and activation of recovery beacon.

---

## 2. Key Subsystem Capabilities

### 2.1 Navigation Stack (Aero-Grade EKF)
*   **100 Hz State Estimation:** Uses an Error-State Extended Kalman Filter (ES-EKF) with 16 states (Position, Velocity, Quaternion, Accel/Gyro Biases).
*   **IMM (Interacting Multiple Model):** Runs five parallel filters tuned for specific flight regimes (BOOST vs. LANDED) with Bayesian state switching.
*   **Sensor Fusion:** Integrates BNO085 (High-G Accel/Gyro), BMP585 (Barometric Altitude), and N-GS-01 (NavIC/GPS Position) at their native rates.
*   **SPRT FDIR:** Sequential Probability Ratio Test for Fault Detection, Isolation, and Recovery — automatically excludes "glitched" sensor readings.

### 2.2 Control System (Cascaded PID)
*   **Dual-Loop Architecture:** Inner loop stabilizes angular rates (rad/s), while the outer loop maintains absolute attitude (Euler angles).
*   **Quadcopter 'X' Mixer:** Specialized logic to translate PID torque corrections and collective throttle into microsecond PWM signals (1000–2000µs).
*   **Battery Compensation:** Real-time scaling of motor outputs based on measured battery voltage (7.4V nominal) to maintain consistent thrust.
*   **Altitude-Gated Arming:** Automated arm-latching servo and motor arming triggered exactly at the 600m transition window.

### 2.3 Communications & Telemetry
*   **XBee Pro Link:** 1 Hz telemetry downlink in CSV format (CAN-7USAT §5.3 compliant).
*   **Command Parser:** Robust uplink for in-flight mode changes, simulation injection, and emergency abort.
*   **USB CLI:** A comprehensive maintenance console for field configuration without a ground station.

### 2.4 Reliability & Safety
*   **Task Watchdog:** Hardware-level TWDT monitoring all critical FreeRTOS tasks.
*   **Coredump Support:** Automatic capture of system state during software panics, exported to SD card for post-flight analysis.
*   **BIT (Built-In Test):** Exhaustive 10-point hardware check at boot with rapid-blink LED failure codes.

---

## 3. Hardware Requirements

For a complete list of physical parts and recommended accessories, see the [Bill of Materials (BOM)](docs/COMPONENTS.md).

### 3.1 Primary Avionics (Flight Computer)
| Component | Part Number | Purpose |
|-----------|-------------|---------|
| **MCU** | ESP32-S3 WROOM-1 | Dual-core 240MHz, 8MB Flash, 512KB SRAM |
| **IMU** | BNO085 | 9-DOF fusion-grade IMU (SHTP over I2C) |
| **Baro** | BMP585 | Ultra-precision barometer (±5cm resolution) |
| **GNSS** | N-GS-01 | NavIC-capable GNSS for Indian subcontinent |
| **Radio** | XBee Pro | 2.4GHz / 900MHz Long-range telemetry |
| **Current** | INA260 | Precision voltage/current sensing for power profiling |
| **Fuel Gauge**| MAX17048 | LiPo SoC% and cell voltage monitoring |

### 3.2 Power Specifications
*   **Battery:** 2S Li-ion (18650 Molicel 35A).
*   **Regulation:** 5V BEC for MCU/Servo, 7.4V direct rail for ESCs.
*   **Typical Current:** 200mA (idle) up to 6.0A (peak hover).

---

## 4. Quick Start Guide

### Setup Toolchain
1.  Install **ESP-IDF v5.3.x** (Offline installer recommended).
2.  Set environment variables: `C:\Espressif\frameworks\esp-idf-v5.3\export.bat`.

### Configuration
```bash
# 1. Set Team ID
# Edit components/nav/include/nav/config.hpp -> team_id = 1234;

# 2. Select Build Target
idf.py set-target esp32s3

# 3. Select Role (FC or GCS)
idf.py menuconfig 
# Navigate to: CAN-7USAT Build Target -> select ROLE_FC
```

### Build & Flash
```bash
# Build the binary
idf.py build

# Flash and open the monitor
idf.py -p COMx flash monitor
```

---

## 5. Directory Structure

```text
├── components/
│   ├── nav/           # The EKF/IMM Navigation Engine (Professional Grade)
│   ├── control/       # Cascaded PID and Motor Mixer logic
│   ├── drivers/       # Robust C++ drivers for all I2C/SPI/UART sensors
│   ├── comms/         # XBee link layer and Command Parser
│   ├── logging/       # Buffered SD and discrete Event loggers
│   └── telemetry/     # CAN-7USAT compliant CSV encoder
├── docs/              # Detailed Technical Manuals (Refer to ARCHITECTURE.md)
├── main/              # FreeRTOS Task definitions and System Init
└── tools/             # Python-based Ground Station and Parser utilities
```

---

## 6. Resources, Technical Support & License
*   **Developer:** SVNIT Aerospace Team
*   **Compliance:** CAN-7USAT India 2026 Guidelines (§3, §5, §6)
*   **Design & Simulations:** [Google Drive Simulations Folder](https://drive.google.com/drive/folders/1NzejIpZqC-W4NXzsCNN9vamn6NqNCvO8?usp=sharing) (CAD, CFD, FEA, and flight profile simulations)
*   **License:** Proprietary / SVNIT Internal (See `LICENSE`)
