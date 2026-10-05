# Bill of Materials — CAN-7USAT 2026

Status: **fitted** = wired and verified with this firmware, **to wire** = supported by the
firmware but not connected yet, **planned** = not supported in firmware yet.

## Flight computer and sensors

| Component | Part | Qty | Status | Purpose |
|---|---|:-:|---|---|
| MCU board | ESP32-S3-DevKitC-1 **N16R8** (16 MB flash, 8 MB PSRAM) | 1 | fitted | Flight computer; on-board WS2812 RGB LED (GPIO 48) |
| IMU | Bosch **BNO055** breakout | 1 | fitted | Attitude (fusion quaternion), accel, gyro, heading |
| Barometer | Bosch BMP585 breakout | 1 | fitted | Altitude, pressure, temperature |
| GNSS | N-GS-01 NavIC/GPS | 1 | fitted | Position, launch site, return guidance |
| Temp / humidity | Sensirion SHT4x | 1 | fitted | Environment |
| Air quality | Sensirion SGP41 | 1 | fitted | VOC / NOx index |
| Radio | Digi XBee 3 Pro (2.4 GHz) | 1 | to wire | Competition telemetry downlink |
| Storage | micro-SD breakout (SPI) | 1 | to wire | 1 Hz copy of the frame (the on-board 12 MB recorder works without it) |
| Sub-GHz radio | TI CC1101 | 1 | to wire | Backup beacon |
| Power monitor | TI INA260 | 1 | planned | Battery voltage / current (frame shows 0.00 V until fitted) |
| Buzzer | Active 5 V buzzer | 1 | to wire | Recovery beacon after landing |

## Propulsion and actuators

| Component | Part | Qty | Status | Purpose |
|---|---|:-:|---|---|
| Motors | **EMAX ECO 1404 3700KV** | 4 | to wire | Thrust for steering under the canopy |
| ESC | **HAKRC 35A 4-in-1 BLHeli_S** | 1 | to wire | DShot300 from the flight computer |
| Arm latches | 1.5 g linear micro servo | 2 | to wire | Unlatch the drone arms at 600 m |
| Propellers | 3-inch class, matched to the 1404 | 4+ | — | |
| Parachute | Passive, opens on ejection | 1 | — | Main descent; the CanSat stays attached while steering |

## Power

| Component | Part | Qty | Purpose |
|---|---|:-:|---|
| Battery | 2S Li-ion 18650 (Molicel P28A/35A class) | 1 pack | Motors and logic |
| 5 V regulator | 5 V step-down BEC | 1 | ESP32 board and servos |
| Connectors | XT30 (power), JST-PH 2.0 (signals) | — | |

## Ground station

| Item | Notes |
|---|---|
| Windows laptop | Runs the dock (`LAUNCH_DOCK.bat`); Bluetooth LE built in or a USB dongle |
| USB-C data cable | Flashing, flight-log download, bench tests |
| XBee 3 Pro + USB adapter | Ground side of the radio link (when the flight XBee is wired) |
