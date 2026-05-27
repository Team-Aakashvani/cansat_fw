# AAKASHVANI — Exhaustive Testing & Validation Protocol
### Professional Certification for Flight Readiness
> **From Bench to Launch Pad: A 25-Point Verification Guide.**

---

## 1. Testing Philosophy
Flight software validation at SVNIT follows a three-tier hierarchy:
1.  **Deterministic Testing:** Built-In Test (BIT) ensures hardware-level integrity at every power-on.
2.  **Dynamic Simulation:** Command-line injection of synthetic data to verify EKF convergence and state machine transitions.
3.  **Environmental Soak:** Long-duration stress tests to detect memory leaks and thermal drift.

---

## 2. Stage 1: Built-In Test (BIT) Reference

### 2.1 Failure Code Interpretation
If the onboard RGB LED blinks **RED (Rapid)**, a critical BIT failure has occurred. Connect the USB console and check the flags.

| Bit | Flag | Test Procedure | Critical? |
|-----|------|----------------|-----------|
| **0** | `IMU_ABSENT` | BNO085 I2C probe failed. Check Address 0x4A. | **YES** |
| **1** | `BARO_ABSENT`| BMP585 I2C probe failed. Check Address 0x46. | **YES** |
| **2** | `POWER_ABSENT`| INA260 I2C probe failed. Check Address 0x40. | **YES** |
| **4** | `RADIO_ABSENT`| XBee UART response timeout. Check CTS/RTS. | **YES** |
| **7** | `IMU_SANITY` | Accel magnitude check: $|a| \in [7.8, 11.8]$ m/s². | NO |
| **8** | `BARO_SANITY`| Pressure range check: $P \in [70, 110]$ kPa. | NO |

### 2.2 The BIT Override (Dev Mode Only)
To test software logic without a connected CanSat PCB:
1.  Connect via USB CLI (115200 baud).
2.  Enter: `set bit_override 1`.
3.  Enter: `reboot`.
*Warning: This flag is cleared on every NVS-erase. Never fly with bit_override = 1.*

---

## 3. Stage 2: Hardware-In-The-Loop (HIL) Simulation

AAKASHVANI allows the Ground Station to "take over" the sensors via the simulation protocol. This is the only way to verify the transition to the **PARACHUTE** and **DRONE_HOVER** states on the bench.

### 3.1 Verification of 600m Deployment
1.  Enter Simulation Mode: `1234,SIM,ENABLE`
2.  Set Initial Pad Pressure: `1234,SIMP,101325` (Wait for CAL to finish).
3.  Simulate Ascent: Inject decreasing pressure (e.g., `90000` Pa).
4.  Simulate Descent: Inject increasing pressure.
5.  **PASS CRITERIA:** When pressure reaches the equivalent of 600m (approx. 94300 Pa), verify:
    *   `SOFTWARE_STATE` changes from 2 to 3.
    *   Servo (GPIO38) moves to the **RELEASED** (2000µs) position.

### 3.2 Verification of Drone Stabilization
1.  Simulate Altitude < 30m: `1234,SIMP,101000`.
2.  **PASS CRITERIA:**
    *   `SOFTWARE_STATE` changes to 4.
    *   Motor PWM signals transition from 1000µs to active PID values (~1400µs).
    *   Tilt the CanSat: Observe motor PWMs compensating to maintain level (Roll/Pitch).

---

## 4. Stage 3: Professional Pre-Flight Checklist

### 4.1 Physical Integration (The "Shake" Test)
- [ ] All I2C connectors secured with Kapton tape or hot glue.
- [ ] XBee antenna orientation: Vertical, clear of all carbon-fiber or metal struts.
- [ ] Battery voltage check: 7.2V - 8.4V.
- [ ] SD Card: Sandisk Industrial Class 10 (High write endurance).

### 4.2 On-Pad Finalization
- [ ] **BIT PASS:** GCS receives packet with `BIT_FLAGS = 0x00`.
- [ ] **GNSS LOCK:** `SATS >= 7` and `HDOP < 1.5`.
- [ ] **CALIBRATE:** Send `1234,CAL` command. Verify `ALTITUDE` resets to $0.0 \pm 0.5$ m.
- [ ] **TIME SYNC:** Send `1234,ST,00:00:00` to synchronize mission clock.
- [ ] **TELEMETRY:** Confirm RSSI > -90dBm at a distance of 100m.

---

## 5. Post-Flight Crash Forensics

If the flight results in a non-nominal landing or reboot:
1.  **Coredump Retrieval:** Boot the unit with the SD card inserted. Wait for `CoredumpExporter` to finish (LED will blink Blue).
2.  **Event Log Analysis:** Connect via CLI and run `log_dump`. Check for `ERROR_FDIR` or `POWER_LOW` events immediately preceding the crash.
3.  **Trace Analysis:** Use the ESP-IDF GDB tool to map the binary coredump back to the source code line.

---

*This guide is mandatory for all SVNIT Flight Ops personnel.*
