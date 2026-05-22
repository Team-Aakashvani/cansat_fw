# Detailed Wiring Mapping — CAN-7USAT 2026 Production

This document provides the definitive pin-to-pin wiring map for all modules in the CAN-7USAT 2026 production configuration. 

**IMPORTANT:** This configuration uses the native **USB-Serial/JTAG Controller** for console output to free up all hardware UARTs for mission peripherals.

---

## 1. Flight Computer (ESP32-S3 WROOM-1)

### 1.1 Primary High-Speed UARTs
| Peripheral | Controller | S3 Pin (TX) | S3 Pin (RX) | Peripheral Pin | Protocol |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **XBee Pro** | UART2 | **GPIO 34** | **GPIO 32** | DIN / DOUT | 115200 Baud |
| **GNSS (NavIC)** | UART1 | **GPIO 17** | **GPIO 18** | RX / TX | 115200 Baud |
| **P4 Media Link**| UART0 | **GPIO 26** | **GPIO 27** | GPIO 26/27 | 921600 Baud |

### 1.2 Shared SPI Bus & RF Scanner
| Peripheral | S3 Pin (MOSI) | S3 Pin (MISO) | S3 Pin (SCK) | S3 Pin (CS) |
| :--- | :--- | :--- | :--- | :--- |
| **CC1101 (Scanner)** | **GPIO 35** | **GPIO 37** | **GPIO 36** | **GPIO 21** |

### 1.3 I2C Bus 0 — Navigation
| Sensor | S3 Pin (SDA) | S3 Pin (SCL) | Address | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **BNO085 (IMU)** | **GPIO 8** | **GPIO 9** | 0x4A | 4.7k Pull-up |
| **BMP585 (Baro)** | **GPIO 8** | **GPIO 9** | 0x46 | 4.7k Pull-up |

### 1.4 I2C Bus 1 — Power & Environment
| Sensor | S3 Pin (SDA) | S3 Pin (SCL) | Address | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **INA260** | **GPIO 10** | **GPIO 11** | 0x40 | 7.4V Rail |
| **MAX17048** | **GPIO 10** | **GPIO 11** | 0x36 | LiPo Gauge |
| **SDP31** | **GPIO 10** | **GPIO 11** | 0x21 | Airspeed |
| **SHT4x** | **GPIO 10** | **GPIO 11** | 0x44 | Humidity |
| **SGP41** | **GPIO 10** | **GPIO 11** | 0x59 | VOC/NOx |

### 1.5 Actuators & Signals
| Device | S3 Pin | Type | Logic |
| :--- | :--- | :--- | :--- |
| **Motor 0 (FL)** | **GPIO 5** | PWM | LEDC_CH0 |
| **Motor 1 (FR)** | **GPIO 6** | PWM | LEDC_CH1 |
| **Motor 2 (RR)** | **GPIO 7** | PWM | LEDC_CH2 |
| **Motor 3 (RL)** | **GPIO 16** | PWM | LEDC_CH3 |
| **Parachute Servo**| **GPIO 38** | PWM | LEDC_CH4 |
| **Audio Beacon** | **GPIO 39** | Digital | Active High |
| **Power Switch** | **GPIO 40** | Digital | Active High |
| **Status LED** | **GPIO 48** | RGB | Built-in |

### 1.6 Storage (SDMMC)
| SD Pin | S3 Pin | Note |
| :--- | :--- | :--- |
| **CLK** | **GPIO 14** | |
| **CMD** | **GPIO 15** | |
| **D0** | **GPIO 2** | |
| **D1** | **GPIO 4** | |
| **D2** | **GPIO 12** | |
| **D3** | **GPIO 13** | |

---

## 2. P4 Media Coprocessor (ESP32-P4)

The P4 connects to the Flight Computer via a dedicated high-speed UART link for command/status.

| P4 Signal | P4 Pin | S3 Pin (FC) | Connection Type |
| :--- | :--- | :--- | :--- |
| **FC RX** | **GPIO 26** | **GPIO 26** | Straight (Matched logic) |
| **FC TX** | **GPIO 27** | **GPIO 27** | Straight (Matched logic) |
| **SD CLK/CMD/D0**| **GPIO 39-41** | N/A | Dedicated P4 SD Slot |

---

## 3. Ground Station (GS) Bridge (ESP32-S3)

The GS Bridge acts as a transparent bridge between the XBee Pro and the laptop.

| GS Signal | S3 Pin (GS) | XBee Pin | Laptop Connection |
| :--- | :--- | :--- | :--- |
| **XBee TX** | **GPIO 34** | **DIN (Pin 3)** | N/A |
| **XBee RX** | **GPIO 32** | **DOUT (Pin 2)**| N/A |
| **Serial Link** | **UART Port**| N/A | USB Cable (Standard UART) |

---

## 4. XBee Pro Configuration (Common for FC and GS)

Configure via XCTU before assembly:

1. **PAN ID (`ATID`)**: Set to **Team ID** (e.g., `1234`)
2. **Baud Rate (`ATBD`)**: Set to **115200** (`ATBD 7`)
3. **Destination Low (`ATDL`)**: Set to `0`
4. **Destination High (`ATDH`)**: Set to `0`
5. **API Mode (`ATAP`)**: Set to **Transparent Mode** (`0`)

---

## 5. Physical Wiring Safety Checklist

- [ ] **Cross UART Lines:** TX (FC) -> RX (Peripheral) except for P4 where software pins are matched.
- [ ] **I2C Pull-ups:** Ensure 4.7kΩ resistors are present on SDA/SCL for both Bus 0 and Bus 1.
- [ ] **Antenna Clearance:** XBee and CC1101 antennas should be at least 50mm apart to avoid interference.
- [ ] **Common Ground:** All modules (S3, P4, Sensors, BEC) must share a common ground plane.
- [ ] **Power Rail:** INA260 must be placed in-series between the Battery and the BEC/ESCs.
