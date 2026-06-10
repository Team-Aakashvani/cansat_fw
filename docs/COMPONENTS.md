# Bill of Materials (BOM) — CAN-7USAT 2026

This document lists the specific hardware components required to build the CAN-7USAT Flight Computer and its accompanying drone module.

## 1. Propulsion System (Drone)
| Component | Specification | Quantity | Purpose |
| :--- | :--- | :---: | :--- |
| **Motors** | BetaFPV 1103 8500KV | 4 | Main Lift Motors |
| **ESC** | 4in1 Blheli_s-ESC 20amp HAKRC | 1 | Motor Speed Control |
| **Propellers** | Gemfan 2030 or 2540 Tri-blades | 4+ | - |
| **Drone Frame** | 65mm-85mm Carbon Fiber / 3D Printed | 1 | Airframe |

## 2. Flight Computer & Avionics (CanSat)
| Component | Specification | Quantity | Purpose |
| :--- | :--- | :---: | :--- |
| **MCU** | ESP32-S3 WROOM-1 | 1 | Core Processor |
| **IMU** | BNO085 (9-Axis) | 1 | Orientation & Stabilization |
| **Barometer** | BMP585 (High Precision) | 1 | Altimetry |
| **GNSS Module** | Edgehax NavIC | 1 | Global Positioning |
| **Linear Servo** | 1.5G Linear Servo Motor Micro Size Right | 1 | Latching Mechanism |
| **Storage** | Micro SD Card (Class 10) | 1 | Flight Data Logging |

## 3. Power System
| Component | Specification | Quantity | Purpose |
| :--- | :--- | :---: | :--- |
| **Batteries** | 18650 Molicel 35A | 4 | 2S for FC, 2S for Drone |
| **Current Monitor** | INA260 (I2C) | 1 | Power Telemetry |
| **Fuel Gauge** | MAX17048 (I2C) | 1 | LiPo Health Monitoring |
| **Voltage Reg** | 5V/3A BEC | 1 | 7.4V to 5V Step-down |
| **Connectors** | XT30 / JST-PH 2.0 | - | Power & Signal Interconnects |

## 4. Communication & RF
| Component | Specification | Quantity | Purpose |
| :--- | :--- | :---: | :--- |
| **Primary Link** | XBee 3 Pro TH (2.4GHz) | 1 | Ground Station Telemetry |
| **Secondary Link** | CC1101 (Sub-GHz) | 1 | Inter-Satellite / Backup |
| **Antennas** | 2.4GHz Dipole, 433MHz Whip, GNSS Patch | 3 | RF Transmission |

## 5. Mission Specific / FPV
| Component | Specification | Quantity | Purpose |
| :--- | :--- | :---: | :--- |
| **Camera** | FPV Camera (Micro Size) | 1 | Visual Navigation (Off-board) |
| **Video TX** | TBS Unify Pro Nano or Happymodel OVX300 | 1 | 5.8GHz Video Feed (Off-board) |
| **VOC Sensor** | SGP41 | 1 | Air Quality Mission |
| **Humid/Temp** | SHT4x | 1 | Environmental Data |

## 6. Recovery & Safety
| Component | Specification | Quantity | Purpose |
| :--- | :--- | :---: | :--- |
| **Buzzer** | Active 5V Buzzer (92dB) | 1 | Post-Landing Recovery |
| **Indicator** | WS2812B RGB LED | 1 | Visual Status Feedback |
| **Parachute** | 30-45cm Diameter Nylon | 1 | Controlled Descent |

## 7. PCB Architecture & Structural Connectors
| Component | Specification | Quantity | Purpose |
| :--- | :--- | :---: | :--- |
| **Bridge Connectors** | 10-Pin & 4-Pin Castellated Edge Joints | 2 Sets | 70G-rated structural mating between Bridge and Main PCBs |
| **Custom Courtyards** | 1:1 Scale Breakout Footprints | All | Embedded in `CanSat_Library` to guarantee physical zero-collision layout |
