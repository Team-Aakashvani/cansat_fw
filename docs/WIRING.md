# AAKASHVANI — Exhaustive Wiring Reference
### Point-to-Point Avionics Map for CAN-7USAT 2026
> **The definitive electrical interconnect manual for the SVNIT Flight Computer.**

---

## 1. High-Speed Serial Interconnects (UART)

| Peripheral | Bus | S3 TX | S3 RX | Baud Rate | Wire Color (Std) | Description |
|:---|:---:|:---:|:---:|:---:|:---:|:---|
| **XBee 3 Pro TH** | UART2 | 42 | 41 | 115,200 | Yellow / Orange | Primary Telemetry Link |
| **Edgehax NavIC** | UART1 | 17 | 18 | 115,200 | White / Blue | Edgehax NavIC Module |
| **Native Console**| UART0 | 19 | 20 | 115,200 | USB Internal | System Logs / JTAG |

---

## 2. Navigation & Power Bus (I2C)

The CanSat utilizes two independent I2C buses to maximize reliability and bandwidth.

> [!WARNING]
> **Bridge Architecture Net Naming Rule:** For the H-shaped Bridge PCB variant, any I2C or UART signal passing from the main ESP32 board to the Bridge board must be renamed with a `_BR` suffix on the Bridge side (e.g., `SDA_0` becomes `SDA_0_BR`). This severs the KiCad ratsnest line and forces the copper to route exclusively through the physical castellated edge connectors.

### 2.1 I2C Bus 0 (The Flight Bus)
*   **SDA:** GPIO 8 (Bridge: `SDA_0_BR`)
*   **SCL:** GPIO 9 (Bridge: `SCL_0_BR`)
*   **Pull-up:** 4.7 kΩ to 3.3V
*   **Device 1:** **BNO085 IMU** (Addr: 0x4A). Must be located at the CanSat Geometric Center.
*   **Device 2:** **BMP585 Barometer** (Addr: 0x46). Must have a foam cover to prevent light-induced pressure drift.

### 2.2 I2C Bus 1 (The Power/Env Bus)
*   **SDA:** GPIO 10
*   **SCL:** GPIO 11
*   **Pull-up:** 4.7 kΩ to 3.3V
*   **Device 1:** **INA260** (Addr: 0x40). Monitors Battery Voltage/Current.
*   **Device 2:** **MAX17048** (Addr: 0x36). Monitors LiPo Cell Health.
*   **Device 3:** **SHT4x** (Addr: 0x44). Temp/Humidity.
*   **Device 4:** **SGP41** (Addr: 0x59). VOC Index (Air Quality).

---

## 3. SPI Bus — Secondary Mission

| Function | S3 Pin | Device Pin | Description |
|:---|:---:|:---:|:---|
| **MOSI** | 35 | SI | Master Out, Slave In |
| **MISO** | 37 | SO | Master In, Slave Out |
| **SCK** | 36 | SCLK | Serial Clock |
| **CS** | 21 | CSN | CC1101 Chip Select |

---

## 4. Actuator & Signal Pins (PWM / Digital)

| Device | S3 Pin | Channel | Logic | Range |
|:---|:---:|:---:|:---:|:---|
| **Motor 1 (FL)**| 5 | LEDC_CH0 | PWM | HAKRC ESC 1 |
| **Motor 2 (FR)**| 6 | LEDC_CH1 | PWM | HAKRC ESC 2 |
| **Motor 3 (RR)** | 7 | LEDC_CH2 | PWM | HAKRC ESC 3 |
| **Motor 4 (RL)** | 16 | LEDC_CH3 | PWM | HAKRC ESC 4 |
| **Linear Servo** | 38 | LEDC_CH4 | PWM | 1.5G Micro Servo |
| **Recovery Buzzer** | 39 | — | DIG | Active High (92dB Siren) |
| **Status RGB LED** | 48 | — | RMT | WS2812B Protocol |

---

## 5. Storage (SDMMC 1-Bit Mode)

| Function | S3 Pin | Description |
|:---|:---:|:---|
| **CLK** | 14 | Clock |
| **CMD** | 15 | Command |
| **D0** | 2 | Data Line 0 |
| **D1** | 4 | Data Line 1 (Optional, for 4-bit) |
| **D2** | 12 | Data Line 2 (Optional, for 4-bit) |
| **D3** | 13 | Data Line 3 (Optional, for 4-bit) |

---

## 6. Physical Interconnect Standards

1.  **Wire Gauge:**
    *   **Power Rail (7.4V):** 18 AWG Silicon Wire.
    *   **BEC Rail (5.0V):** 22 AWG.
    *   **Signal (I2C/UART):** 28 AWG or 30 AWG.
2.  **Shielding:** GNSS and XBee wires must be kept away from the ESC power leads to prevent 16kHz PWM interference.
3.  **Strain Relief:** All wires entering the Flight Computer PCB must be secured with zip-ties or strain-relief slots.

---

*This document is the single source of truth for the CanSat electrical harness.*
