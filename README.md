# AAKASHVANI — CAN-7USAT India 2026 Flight Software
### Team 001 · SVNIT Aerospace · v2.0.0

**AAKASHVANI** (Sanskrit: *voice from the sky*) is the flight software and ground station for
the SVNIT CanSat. A carrier drone or rocket takes the CanSat up. After release, a passive
parachute opens. At **600 m above the launch site**, two linear servos unlatch the drone arms. The
four motors then tilt the CanSat under its canopy, steering it back toward the launch site.

| You want to… | Read |
|---|---|
| **Use** the CanSat and dock (no prior knowledge needed) | [`docs/USER_MANUAL.pdf`](docs/USER_MANUAL.pdf) |
| Understand the mission logic, failsafes, bench bring-up | [`docs/FLIGHT_SOFTWARE.md`](docs/FLIGHT_SOFTWARE.md) |
| See the telemetry frame and the command list | [`docs/TELEMETRY_FORMAT.md`](docs/TELEMETRY_FORMAT.md) |
| Wire the hardware | [`docs/WIRING.md`](docs/WIRING.md), [`docs/HARDWARE_SETUP.md`](docs/HARDWARE_SETUP.md) |
| Build and flash the firmware | [`docs/BUILD_GUIDE.md`](docs/BUILD_GUIDE.md), [`docs/FLASH_GUIDE.md`](docs/FLASH_GUIDE.md) |
| Test it (host tests, simulated flight, lift test) | [`docs/TESTING_GUIDE.md`](docs/TESTING_GUIDE.md) |
| Look at the software structure | [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) |
| See the parts list | [`docs/COMPONENTS.md`](docs/COMPONENTS.md) |

---

## Mission at a glance

| Phase (telemetry state) | What happens |
|---|---|
| `PAD` | Barometer zeroed, launch site averaged from GNSS, IMU mount detected, flight log pre-erased |
| `ASCENT` | Carrier climb detected (rocket boost or drone climb); attitude reference and launch site locked |
| `DESCENT` | Release detected (free-fall / ejection signature); the parachute opens by itself |
| `ARMS_DEPLOY` | At 600 m AGL both servos unlatch the arms |
| `STEERING` | DShot motors tilt the CanSat toward the launch site (PI guidance + attitude PID); cut at 10 m |
| `LANDED` | Motors off, red LED and buzzer for recovery, Bluetooth back on |

Mission state survives a reset in the air: after a brownout or watchdog restart, the firmware
resumes in the phase it was in.

## What is in the box

* **Flight computer**: ESP32-S3-DevKitC-1 **N16R8** (16 MB flash, 8 MB PSRAM), ESP-IDF 5.5.5.
* **Sensors fitted**: BNO055 (attitude), BMP585 (barometer), N-GS-01 NavIC/GPS, SHT4x
  (temperature/humidity), SGP41 (VOC/NOx).
* **Actuators (to be wired)**: EMAX ECO 1404 3700KV motors on a HAKRC 35A 4-in-1 BLHeli_S ESC
  (DShot300), two 1.5 g linear servos for the arm latches.
* **Links**: USB console, Bluetooth LE health link (`AAKASHVANI-001`), XBee 3 Pro downlink
  (silent until the ground station sends `CX,ON`, as the guidelines require).
* **On-board flight recorder**: a 12 MB ring in flash (about one hour at 50 Hz). It survives
  power loss and is downloaded with the dock.
* **Crash dumps**: if the firmware panics, the dump goes to flash and the dock shows it at the next boot.
* **Boot light show**: the on-board RGB LED shows a boot sequence and the BIT result. See the
  manual or [`docs/FLIGHT_SOFTWARE.md`](docs/FLIGHT_SOFTWARE.md#led).

## Ground station (dock)

Double-click **`LAUNCH_DOCK.bat`**. It connects over **USB** or **Bluetooth** and has these
pages: Overview, Attitude (with 3D view), Health, Flight log, Bench, Console and Firmware. Every
competition frame it receives is saved to
`Documents\Aakashvani\Flight_2026-IN-SPACeCAN-7USAT-001.csv` with a header row. This is the file
for the judges.

`LAUNCH_3D_VISUALIZER.bat` opens the stand-alone 3D attitude viewer in the browser.

## Quick build

```powershell
# ESP-IDF 5.5.5 environment, from the project root
idf.py set-target esp32s3      # first time only
idf.py build
idf.py -p COM15 flash          # or: python tools/flash_cansat.py COM15, or the dock's Firmware page
python tests/host/run_tests.py # host unit tests (attitude, vertical filter, mission)
```

## Directory layout

```text
components/
  nav/        attitude reference, vertical Kalman filter, mission supervisor, return guidance, config
  control/    steering controller, quad-X motor mixer
  drivers/    BNO055/BMP585/GNSS/SHT4x/SGP41 drivers, DShot (RMT), servos (LEDC)
  comms/      XBee link, BLE (NimBLE, Nordic UART) link, command parser
  logging/    flight recorder (12 MB flash ring), event log, core-dump export
  telemetry/  competition CSV frame encoder
  cli/        USB console
main/         FreeRTOS tasks and system init
tests/host/   host unit tests (g++)
tools/        dock, 3D viewer server, flash/build helpers, simulation scripts
docs/         manuals
```

## Resources

* Competition rules: `guidelines.txt` (CAN-7USAT India 2026)
* Design and simulations: [Google Drive folder](https://drive.google.com/drive/folders/1NzejIpZqC-W4NXzsCNN9vamn6NqNCvO8?usp=sharing)
* Bundled assets: IBM Plex fonts (SIL OFL 1.1), Lucide icons (ISC). See `tools/assets/ASSETS_LICENSES.txt`.
* License: proprietary, SVNIT internal.
