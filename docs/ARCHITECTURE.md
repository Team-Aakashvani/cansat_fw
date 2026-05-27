# AAKASHVANI — Exhaustive Technical Architecture
### Professional Avionics Stack for CAN-7USAT India 2026
> **A deep-dive into the algorithms, task models, and data flow of the Flight Computer.**

---

## 1. System Topology & Philosophy

The AAKASHVANI architecture is built on the principle of **computational determinism** and **modular decoupling**. It targets the ESP32-S3 WROOM-1, utilizing its dual-core Xtensa LX7 processor to separate high-frequency flight dynamics (Core 0) from lower-frequency system services (Core 1).

### The "Akaash" Standard
*   **Zero Heap Allocation:** All critical flight buffers and EKF matrices are statically allocated or stack-allocated to prevent runtime memory fragmentation and "Out of Memory" (OOM) failures.
*   **Header-Only Nav Stack:** The navigation engine is implemented as a template-heavy header-only library, allowing the compiler to perform aggressive inlining and optimization.
*   **Thread Safety:** Strict mutex-gate access for all shared sensor and flight computer state data.

---

## 2. Detailed Task Model (FreeRTOS)

### Core 0: The Flight Core (Real-Time)
Core 0 is dedicated exclusively to the stabilization loop. No I/O blocking or high-latency operations (like SD card writes) are allowed here.

1.  **`nav_task` (100 Hz, Priority: 31):**
    *   **Function:** Ingests raw IMU data from the BNO085. Propagates the 16-state EKF using a 2nd-order midpoint integrator.
    *   **Timing:** Must complete execution in < 10ms. Typical execution: 2.4ms.
2.  **`control_task` (100 Hz, Priority: 30):**
    *   **Function:** Executes the Cascaded PID loops. Reads the fused state from `nav_task`, calculates torque corrections, and updates LEDC PWM channels.
    *   **Safety:** Gated by the `EVT_BIT_PASS` event bit.

### Core 1: The System Core (Background Services)
Core 1 manages communication, logging, and environmental monitoring.

1.  **`sensor_task` (50 Hz, Priority: 29):** Polls the BMP585 barometer and I2C-1 environmental sensors. Provides snapshots to the Flight Core via `sensor_mutex`.
2.  **`telem_task` (1 Hz, Priority: 5):** Snapshots the entire flight computer state, encodes it into the mandatory CSV format, and enqueues it for XBee transmission.
3.  **`logging_task` (1 Hz, Priority: 4):** Manages the XBee radio's state machine (`spin()`) and flushes the SD card FAT32 buffers every 5 seconds.
4.  **`power_task` (1 Hz, Priority: 3):** Monitors the INA260 and MAX17048. Executes low-voltage callbacks (e.g., disarming motors at critical levels).

---

## 3. Navigation Stack: The EKF/IMM Engine

### 3.1 State Representation
The filter tracks a 16-dimensional state vector $\mathbf{x}$:
*   $\mathbf{p} \in \mathbb{R}^3$: Position (East, North, Up)
*   $\mathbf{v} \in \mathbb{R}^3$: Velocity (m/s)
*   $\mathbf{q} \in \mathbb{R}^4$: Hamiltonian Quaternion (Body to World)
*   $\mathbf{b}_a \in \mathbb{R}^3$: Accelerometer Bias
*   $\mathbf{b}_g \in \mathbb{R}^3$: Gyroscope Bias

### 3.2 Error-State Dynamics
Instead of the full state, the EKF updates a 15-dimensional error state $\delta\mathbf{x}$. This approach is mathematically superior for attitude estimation as it avoids the singularities of Euler angles and the over-parameterization of quaternions.
*   **Attitude Error:** Represented as a small rotation vector $\delta\boldsymbol{\theta} \in \mathbb{R}^3$.
*   **Injection:** The quaternion is updated via $\mathbf{q} \leftarrow \mathbf{q} \otimes \exp(\delta\boldsymbol{\theta}/2)$.

### 3.3 Interacting Multiple Model (IMM)
The system runs five parallel EKF instances, each with a different process noise covariance $\mathbf{Q}$:
*   **BOOST:** High $\sigma_a$ to track rocket motor ignition.
*   **BALLISTIC:** Low $\sigma_a$ for smooth coasting.
*   **PARACHUTE:** Medium $\sigma_a$ to handle pendulum oscillations.
*   **DRONE_HOVER:** Tuned for active motor vibrations.
*   **LANDED:** Ultra-low noise for stationary position locking.

---

## 4. Control Subsystem: Cascaded Stability

### 4.1 Dual-Loop Cascaded PID
The control architecture is split into two layers to ensure high bandwidth and rejection of disturbances:
1.  **Outer Loop (Position/Angle):** Operates on absolute Euler angles (Roll/Pitch). It generates a "Target Rate" (rad/s) required to bring the drone back to level.
2.  **Inner Loop (Rate):** Operates on high-frequency IMU gyro data. It compares the "Target Rate" from the outer loop against the "Measured Rate" and generates a torque command.

### 4.2 Quadcopter 'X' Mixing Logic
The mixer translates the virtual axes (Roll, Pitch, Yaw, Throttle) into physical motor speeds:
*   `Motor 1 (Front-Left):` $T + R + P + Y$
*   `Motor 2 (Front-Right):` $T - R + P - Y$
*   `Motor 3 (Rear-Right):` $T - R - P + Y$
*   `Motor 4 (Rear-Left):` $T + R - P - Y$

Outputs are clamped to $[1000, 2000]$ µs to prevent ESC desync.

---

## 5. Data Flow & Communication

### 5.1 Telemetry Pipeline
1.  **Snapshot:** `telem_task` takes a deep copy of the `FlightComputerOutput`.
2.  **Formatting:** `TelemetryEncoder` converts floats to fixed-precision strings (e.g., `%.2f`).
3.  **Transport:** CSV string is passed to the `XBeeLink` queue.
4.  **Verification:** The GCS confirms the packet using the Team ID prefix and monotonically increasing `PACKET_COUNT`.

### 5.2 Uplink Command Protocol
All uplink commands follow the format: `<TEAM_ID>,<CMD>,<ARG>\n`.
*   **CRC Check:** Commands are validated against a 16-bit XOR checksum to prevent accidental trigger during high-interference periods.
*   **Simulation Injection:** Using the `SIMP`, `SIMG`, and `SIMI` commands, the ground station can feed synthetic sensor data into the EKF for Hardware-In-The-Loop (HIL) testing.

---

## 6. Fault Detection & Recovery (FDIR)

### 6.1 SPRT (Sequential Probability Ratio Test)
For every sensor measurement, the system calculates a log-likelihood ratio. If the measurement consistently deviates from the EKF prediction beyond the statistical threshold:
*   The sensor is flagged as **FAULTY**.
*   The EKF weight for that sensor is reduced to zero.
*   A discrete event is logged to the SD card.

### 6.2 Supervisory State Machine
The Bayesian Supervisor monitors the probabilities of the IMM filters.
*   **Launch Detection:** Triggered only if IMM BOOST probability > 0.8 and Accel Z > 2g.
*   **Deployment Detection:** Triggered by altitude drop < 600m combined with PARACHUTE regime dominance.

---

## 7. Memory & Flash Map

### 7.1 Flash Partition Table (8MB)
| Name | Size | Purpose |
|------|------|---------|
| `nvs` | 24KB | Storage for Team ID, Ground Alt, Calibration |
| `factory`| 3MB | Main Firmware Image |
| `ota_0` | 3MB | Over-The-Air Update Slot |
| `event_log`| 1MB | Discrete Flight Events (FAT partition) |
| `coredump`| 320KB | Binary crash logs |

---

*End of Technical Manual. This document is maintained by the SVNIT Flight Software Team.*
