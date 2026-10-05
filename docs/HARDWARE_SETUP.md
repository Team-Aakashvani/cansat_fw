# AAKASHVANI — Hardware Setup

Power, electrical rules and mechanical notes. Pin-by-pin connections are in `WIRING.md`.
The authoritative pin map is `PinConfig` in `components/nav/include/nav/config.hpp`.

## 1. Power

* **Battery**: 2S Li-ion (7.4 V nominal, 8.4 V full) → XT30 → ESC power pads (motors) and a
  5 V BEC.
* **5 V BEC** → ESP32-S3 board `5V` pin and the two servos. The board's own regulator makes 3.3 V
  for the sensors.
* All grounds common: battery, ESC signal ground, BEC, ESP32, servos.
* When the INA260 is fitted, put it in series **before** the ESC/BEC split, so it measures the
  whole system.
* On the bench, USB alone powers the flight computer and sensors. The motors and servos need the
  battery.

## 2. Electrical rules

* **3.3 V logic only** on every ESP32 pin. The BNO055, BMP585, SHT4x and SGP41 breakouts have
  their own pull-ups. Keep the I²C bus short (< 20 cm total).
* Never use GPIO 19/20 (USB), 26–37 (flash and octal PSRAM), 0/3/45/46 (strapping) or 43/44.
  GPIO 48 is the on-board RGB LED.
* DShot signal wires: short, twisted with a ground wire, routed away from the GNSS antenna.
* Keep the BNO055 away from battery leads, ESC and motors. Its heading uses the magnetometer.
* The barometer needs a foam cover (light and airflow cause pressure noise) and a vent hole in the
  CanSat shell.

## 3. Mechanical

* The IMU can be mounted upright or on its side. The firmware detects which at boot. Power on with
  the CanSat **upright and still**.
* Vehicle +X is the IMU's forward axis after mount detection. Motor M1 is front-left as seen
  from above. If the arms are rotated relative to the IMU, set `steer.frame_yaw_deg`.
* Props-in rotation: M1 and M3 clockwise, M2 and M4 counter-clockwise.

## 4. PCB tracks

The avionics PCB is developed on two tracks: the H-shaped *bridge* board (three sections joined by
soldered castellated edges, nets crossing the bridge named `_BR`) as the flight candidate, and a
single 100 × 200 mm board to check component fit. Both use the 1:1 courtyards in the
`CanSat_Library` KiCad library.
