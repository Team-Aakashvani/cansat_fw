# AAKASHVANI — Exhaustive Hardware Specification
### Complete Wiring, Pinout, and Electrical Guide
> **The Blueprint for the CAN-7USAT 2026 Flight Hardware.**

---

## 1. Power Distribution Network (PDN)

The CanSat uses a split-rail architecture to isolate sensitive sensor logic from high-current motor transients.

### 1.1 High-Current Rail (7.4V)
*   **Source:** 2S Li-ion Battery (2x 18650 Molicel 35A in Series).
*   **Path:** Battery → XT30 Connector → INA260 (In-Series) → ESC Power Bus.
*   **Load:** 4x BetaFPV 1103 8500KV Motors.

### 1.2 Logic Rail (5.0V / 3.3V)
*   **Source:** 5V/3A BEC (Battery Elimination Circuit).
*   **Path:** Battery → BEC → ESP32-S3 VIN → Onboard 3.3V LDO.
*   **Load:** ESP32-S3, BNO085, BMP585, GNSS, XBee, Parachute Servo.

**CRITICAL SAFETY:** The INA260 must be placed **before** the BEC and ESC split point to monitor the total system current.

---

## 2. Complete GPIO Map (ESP32-S3 WROOM-1)

| Pin | Symbol | Type | Interface | Device | Logic / Unit |
|-----|--------|------|-----------|--------|--------------|
| **2** | `SD_D0` | I/O | SDMMC | SD Card | Data Line 0 |
| **4** | `SD_D1` | I/O | SDMMC | SD Card | Data Line 1 |
| **5** | `MOT_0` | OUT | PWM | ESC 1 | LEDC_CH0 (Front-Left) |
| **6** | `MOT_1` | OUT | PWM | ESC 2 | LEDC_CH1 (Front-Right) |
| **7** | `MOT_2` | OUT | PWM | ESC 3 | LEDC_CH2 (Rear-Right) |
| **8** | `SDA_0` | I/O | I2C0 | BNO/BMP | Navigation Bus |
| **9** | `SCL_0` | OUT | I2C0 | BNO/BMP | Navigation Bus |
| **10**| `SDA_1` | I/O | I2C1 | INA/MAX/ENV| Power/Environment Bus |
| **11**| `SCL_1` | OUT | I2C1 | INA/MAX/ENV| Power/Environment Bus |
| **12**| `SD_D2` | I/O | SDMMC | SD Card | Data Line 2 |
| **13**| `SD_D3` | I/O | SDMMC | SD Card | Data Line 3 |
| **14**| `SD_CLK`| OUT | SDMMC | SD Card | Clock |
| **15**| `SD_CMD`| OUT | SDMMC | SD Card | Command |
| **16**| `MOT_3` | OUT | PWM | ESC 4 | LEDC_CH3 (Rear-Left) |
| **17**| `GNSS_TX`| OUT | UART1 | N-GS-01 | Transmit to GNSS |
| **18**| `GNSS_RX`| IN | UART1 | N-GS-01 | Receive from GNSS |
| **21**| `CC_CS` | OUT | SPI | CC1101 | Chip Select |
| **26**| `FREE_26`| — | — | — | **UNASSIGNED (Available)** |
| **27**| `FREE_27`| — | — | — | **UNASSIGNED (Available)** |
| **32**| `XBEE_RX`| IN | UART2 | XBee Pro | Telecommand In |
| **34**| `XBEE_TX`| OUT | UART2 | XBee Pro | Telemetry Out |
| **35**| `MOSI` | OUT | SPI | CC1101 | Master Out |
| **36**| `SCK` | OUT | SPI | CC1101 | Serial Clock |
| **37**| `MISO` | IN | SPI | CC1101 | Master In |
| **38**| `SERVO` | OUT | PWM | Servo | LEDC_CH4 (Latching) |
| **39**| `BEACON`| OUT | DIG | Buzzer | Active High |
| **40**| `PWR_SW`| IN | DIG | Switch | Arming Interlock |
| **48**| `LED` | OUT | RGB | WS2812 | Status Indicator |

---

## 3. Communication Protocol Standards

### 3.1 I2C Navigation Bus (I2C0)
*   **Clock Speed:** 400 kHz (Fast Mode).
*   **Pull-ups:** 4.7 kΩ hardware resistors required on SDA/SCL.
*   **Addresses:**
    *   `BNO085:` 0x4A
    *   `BMP585:` 0x46

### 3.2 I2C Power/Env Bus (I2C1)
*   **Clock Speed:** 100 kHz (Standard Mode) for maximum reliability with cable-tethered sensors.
*   **Addresses:**
    *   `INA260:` 0x40
    *   `MAX17048:` 0x36
    *   `SHT4x:` 0x44
    *   `SGP41:` 0x59

### 3.3 High-Speed Serial (UART)
*   **XBee Pro:** 115,200 baud, 8N1, No Flow Control.
*   **GNSS:** 115,200 baud (Configured for NavIC + GPS @ 1Hz).

---

## 4. Actuator Specifications

### 4.1 ESCs (Electronic Speed Controllers)
*   **Model:** 4in1 Blheli_s-ESC 20amp HAKRC.
*   **Protocol:** Standard 50Hz PWM.
*   **Timing:** 
    *   `1000 µs:` Stopped / Disarmed.
    *   `1100 µs:` Minimum Idle.
    *   `2000 µs:` Full Throttle.

### 4.2 Latching Servo
*   **Model:** 1.5G Linear Servo Motor Micro Size Right.
*   **Torque:** ≥ 1.5 kg-cm.
*   **Operation:** 
    *   `1000 µs:` Arms Released (Storage/Ascent).
    *   `2000 µs:` Arms Latched (Drone Mode).

---

## 5. RF Layout & Interference Guidelines

1.  **Antenna Spacing:** The XBee (2.4GHz) and CC1101 (Sub-GHz) antennas must be separated by at least 10cm.
2.  **GNSS Clear Zone:** No copper pours or high-speed traces should be placed within 15mm of the GNSS patch antenna.
3.  **Twisted Pairs:** I2C and UART cables longer than 5cm should be twisted with a Ground wire to minimize EMI from the ESCs.

---

*This specification is the master reference for PCB layout and wiring harness assembly.*
