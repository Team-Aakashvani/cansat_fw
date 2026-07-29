# Bill of Materials (BOM) — CAN-7USAT 2026

This document lists the specific hardware components required to build the CAN-7USAT Flight Computer and its accompanying drone module.

## 1. Propulsion System (Drone & Attitude Control)
| Component | Specification | Quantity | Purpose |
| :--- | :--- | :---: | :--- |
| **Motors** | BetaFPV 1103 8500KV | 4 | Main Lift Motors |
| **ESC** | 4in1 BLHeli_S ESC 20A HAKRC | 1 | Motor Speed Control |
| **Propellers** | Gemfan 2030 or 2540 Tri-blades | 4+ | Descent Control Propellers |
| **Attitude Motor** | Flywheel BLDC Motor | 1 | Spin Control & Stabilization |
| **Drone Frame** | 65mm–85mm Carbon Fiber / 3D Printed | 1 | Airframe |

## 2. Flight Computer & Avionics (CanSat)
| Component | Specification | Quantity | Purpose |
| :--- | :--- | :---: | :--- |
| **MCU Breakout** | ESP32-S3-DevKitC-1 (N32R16V) | 1 | Core Processor (32MB Flash, 16MB PSRAM) |
| **IMU Breakout** | CEVA / Hillcrest BNO085 Breakout | 1 | 9-Axis Orientation & Stabilization |
| **Baro Breakout** | Bosch BMP585 Breakout | 1 | High-Precision Altimetry |
| **Airspeed Sensor** | Mateksys AS-DLVR-I2C (Digital) | 1 | Dynamic Airspeed & Pressure (Pitot Tube) |
| **GNSS Breakout** | Edgehax NavIC L5 / GPS Breakout | 1 | Global Positioning (IRNSS/GPS) |
| **Linear Servo** | 1.5G Linear Servo Motor Micro Size Right | 1 | Latching / Payload Deployment Mechanism |
| **Storage** | Adafruit Micro SD Card Breakout | 1 | High-Speed SPI Flight Data Logging |

## 3. Power System
| Component | Specification | Quantity | Purpose |
| :--- | :--- | :---: | :--- |
| **Batteries** | 18650 Molicel 35A Li-Ion Cells | 4 | 2S for FC, 2S for Drone / Actuators |
| **Current Monitor** | Texas Instruments INA260 Breakout (I2C) | 1 | Precision Power & Current Telemetry |
| **Fuel Gauge** | MAX17048 Breakout (I2C) | 1 | LiPo / Li-Ion SOC Health Monitoring |
| **5V Regulator** | Pololu 5V Step-Down BEC | 1 | High-Efficiency 7.4V to 5V Step-down |
| **3.3V Regulator** | AMS1117-3.3V LDO | 1 | Low-Dropout 3.3V Logic Rail |
| **Connectors** | XT30 Connectors (Pack A & B) / JST-PH 2.0 | - | High-Current Power & Signal Interconnects |

## 4. Communication & RF
| Component | Specification | Quantity | Purpose |
| :--- | :--- | :---: | :--- |
| **Primary Link** | Digi XBee 3 Pro TH (2.4 GHz) | 1 | High-Bandwidth Ground Station Telemetry |
| **Secondary Link** | Texas Instruments CC1101 Breakout | 1 | Sub-1 GHz Long-Range Emergency Backup |
| **Antennas** | 2.4 GHz Dipole, 433 MHz Whip, GNSS Patch | 3 | RF Transmission & Satellite Reception |

## 5. Mission Specific & Payload Video (FPV)
| Component | Specification | Quantity | Purpose |
| :--- | :--- | :---: | :--- |
| **FPV Cam & VTX** | AKK KC04 Transmitter System | 1 | 700TVL 2.8mm 120° Camera + 5.8 GHz VTX All-in-One Live Descent Feed |
| **VOC / NOx Sensor**| Sensirion SGP41 Breakout | 1 | Air Quality & Gas Sampling Mission |
| **Temp & Humidity** | Sensirion SHT45 / SHT4x Breakout | 1 | High-Accuracy Environmental Data |

## 6. Recovery & Safety
| Component | Specification | Quantity | Purpose |
| :--- | :--- | :---: | :--- |
| **Buzzer** | Active 5V Buzzer (92 dB) | 1 | Post-Landing Acoustic Recovery |
| **Indicator** | WS2812B RGB LED | 1 | Visual Status Feedback |
| **Parachute** | 30–45cm Diameter Nylon | 1 | Controlled Descent / Recovery |

## 7. PCB Architecture & Structural Connectors
| Component | Specification | Quantity | Purpose |
| :--- | :--- | :---: | :--- |
| **Bridge Connectors**| 10-Pin & 4-Pin Castellated Edge Joints | 2 Sets | 70G-rated structural mating between Sensor Bridge and Main PCBs |
| **Custom Courtyards**| 1:1 Scale Breakout Footprints | All | Embedded in `Aakashwani_Master` library to guarantee zero-collision layout |
