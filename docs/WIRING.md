# AAKASHVANI — Wiring Reference (ESP32-S3-DevKitC-1 N16R8)

Authoritative source: `PinConfig` in `components/nav/include/nav/config.hpp`. If this table
and the code ever disagree, the code wins. Then fix this file.

## 1. I²C (one bus, 100 kHz)

| Signal | GPIO | Devices (address) | Status |
|---|---|---|---|
| SDA | 38 | BNO055 (0x28/0x29), BMP585 (0x46/0x47), SHT4x (0x44), SGP41 (0x59) | fitted |
| SCL | 39 | same | fitted |

## 2. UART

| Device | ESP32 TX → device RX | ESP32 RX ← device TX | Baud | Status |
|---|---|---|---|---|
| N-GS-01 GNSS | 21 | 13 | auto (9600 or 115200) | fitted |
| XBee 3 Pro | 17 | 18 | 115200 | to wire |
| Console | native USB (GPIO 19/20) | | | — |

## 3. Actuators

| Device | GPIO | Peripheral | Signal | Status |
|---|---|---|---|---|
| ESC M1 front-left | 4 | RMT | DShot300 | to wire |
| ESC M2 front-right | 5 | RMT | DShot300 | to wire |
| ESC M3 rear-right | 6 | RMT | DShot300 | to wire |
| ESC M4 rear-left | 7 | RMT | DShot300 | to wire |
| Arm-latch servo A | 15 | LEDC | 50 Hz PWM | to wire |
| Arm-latch servo B | 16 | LEDC | 50 Hz PWM | to wire |
| Recovery buzzer | 42 | GPIO | active high | to wire |
| Status RGB LED | 48 | on board | WS2812 | on board |

ESC: connect the four signal pads and the ESC's signal ground. Do **not** connect the ESC's 5 V
output to the ESP32 if the BEC already feeds it.

## 4. SPI (SD card, CC1101)

| Signal | GPIO |
|---|---|
| SCK | 12 |
| MOSI | 11 |
| MISO | 10 |
| SD CS | 9 |
| CC1101 CS | 14 |

These are disabled in firmware until `PINS.spi_devices_fitted = true` (so floating pins do not
stall the boot).

## 5. Analog

| Signal | GPIO | Notes |
|---|---|---|
| Battery divider | 1 | optional; use a divider that gives ≤ 3.1 V at 8.4 V |

## 6. Harness

* Power (battery → ESC): 18 AWG silicone. 5 V: 22 AWG. Signals: 26–30 AWG.
* Twist each signal run longer than 5 cm with a ground wire.
* Keep GNSS and XBee leads away from the ESC and motor wires.
* Strain-relieve every lead at the board.
