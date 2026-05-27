# AAKASHVANI — Telemetry & Telecommand Reference
### Standard CSV Format and Robust Uplink Specification
> **The definitive interface guide for Ground Station development.**

---

## 1. Downlink Telemetry (FC → GCS)

The flight computer transmits a single CSV-formatted line every **1000ms** (1 Hz). This format is strictly compliant with the CAN-7USAT India 2026 Guidelines.

### 1.1 The Packet Structure
```text
<TEAM_ID>,<MISSION_TIME>,<PACKET_COUNT>,<ALTITUDE>,<PRESSURE>,<TEMPERATURE>,<VOLTAGE>,<GNSS_TIME>,<LATITUDE>,<LONGITUDE>,<GNSS_ALT>,<SATS>,<TILT_X>,<TILT_Y>,<ROT_Z>,<SOFTWARE_STATE>\n
```

### 1.2 Granular Field Definitions

| Field | Type | Resolution | Range | Description |
|-------|------|------------|-------|-------------|
| **TEAM_ID** | `uint16`| 1 | 0 - 65535 | Unique identifier for your team. |
| **MISSION_TIME**| `string`| 1s | HH:MM:SS | Wall-clock time since the `BOOST` phase was latched. |
| **PACKET_COUNT**| `uint32`| 1 | 0 - 4e9 | Monotonically increasing counter. Reset on power-cycle. |
| **ALTITUDE** | `float` | 0.01m | -100 - 5000m| Fused altitude AGL from the IMM filter. |
| **PRESSURE** | `float` | 0.1Pa | 0 - 110kPa | Raw barometric pressure from BMP585. |
| **TEMPERATURE** | `float` | 0.1°C | -40 - 85°C | Ambient temperature. |
| **VOLTAGE** | `float` | 0.01V | 0 - 10.0V | LiPo bus voltage (after the current shunt). |
| **GNSS_TIME** | `string`| 1s | HH:MM:SS | UTC time from NavIC/GPS satellites. |
| **LATITUDE** | `float` | 1e-6° | ±90.000000| Decimal degrees. Positive = North. |
| **LONGITUDE** | `float` | 1e-6° | ±180.000000| Decimal degrees. Positive = East. |
| **GNSS_ALT** | `float` | 0.1m | 0 - 9000m | MSL Altitude from the GNSS module. |
| **SATS** | `int` | 1 | 0 - 32 | Count of satellites used in the current fix. |
| **TILT_X** | `float` | 0.01° | ±180.00° | Absolute Pitch angle from the EKF Euler state. |
| **TILT_Y** | `float` | 0.01° | ±180.00° | Absolute Roll angle from the EKF Euler state. |
| **ROT_Z** | `float` | 0.01°/s| ±2000.00 | Yaw rate from the BNO085 gyro. |
| **SW_STATE** | `uint8` | 1 | 0 - 5 | Current mission phase (See State Reference). |

### 1.3 State Reference Table
*   **0 (PRE_FLIGHT):** All systems idling. BIT results available.
*   **1 (BOOST):** Ascent detected. Mission timer running.
*   **2 (BALLISTIC):** Coasting phase. EKF $\sigma$ adjusted for freefall.
*   **3 (PARACHUTE):** Deploy confirmed. 600m window passed.
*   **4 (DRONE_HOVER):** PID loop active. Descent rate 1-3m/s.
*   **5 (LANDED):** Ground contact confirmed. Motors off.

---

## 2. Telecommand Specification (GCS → FC)

The CanSat listens for commands on UART2. Commands are ASCII-based and newline-terminated.

### 2.1 Standard Command Set
| Command | Arguments | Effect |
|---------|-----------|--------|
| `CX` | `ON`/`OFF` | Starts/Stops the 1 Hz telemetry radio broadcast. |
| `ST` | `HH:MM:SS` | Manual override for the mission clock. |
| `CAL` | — | Samples the current pressure to set the "0m AGL" reference. |
| `SIM` | `ENABLE`/`DISABLE`| Toggles simulation mode (overrides physical sensors). |
| `SIMP` | `<Pa>` | Injects a raw pressure value for EKF testing. |
| `SIMG` | `<E,N,U,Ve,Vn,Vu>`| Injects ENU position and velocity for GNSS testing. |
| `ABORT` | — | Force-disarm all motors and move to `LANDED` state. |

---

## 3. Data Integrity & Reliability

1.  **Heartbeat Monitoring:** The GCS should alert if `PACKET_COUNT` does not increment for > 3 seconds.
2.  **Gap Detection:** If packets are missed due to RF interference, the `MISSION_TIME` field allows the GCS to correctly re-align the flight curve.
3.  **Range Safety:** All safety-critical commands (like `ABORT`) are protected by a required Team ID match.

---

*This document is the official telemetry interface definition for AAKASHVANI v1.1.0.*
