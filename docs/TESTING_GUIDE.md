# AAKASHVANI — Testing Guide

Test in this order. Each level catches problems that are cheaper to fix than at the next one.

## 1. Host unit tests (PC, no hardware)

```powershell
python tests/host/run_tests.py
```

They cover the attitude reference (mount detection, gimbal-lock-free output), the vertical
filter (pressure spikes, accelerometer clipping) and the mission supervisor (rocket, drone drop,
slow carrier that must **not** count as a release, in-flight resume, lift test). All must pass.

## 2. Power-on self test (every boot)

Watch the RGB LED:

| LED | Meaning |
|---|---|
| white fade → blue | power on, buses starting |
| violet breathing | IMU aligning: keep the CanSat still |
| 2× green | all checks passed |
| 2× amber | warnings: optional hardware missing (normal for now: power monitor, SD) |
| 3× red | IMU or barometer missing. Do not fly. |
| 1× magenta | resumed after a reset in flight |

Then the phase colour: green PAD, orange ASCENT, purple DESCENT, blue ARMS_DEPLOY, cyan STEERING,
blinking red LANDED (with the buzzer). The BIT flags are on the dock's Health page (see
`TELEMETRY_FORMAT.md` §2).

## 3. Hardware-in-the-loop flight (bench)

Dock → **Bench** → *Simulated flight*. Choose rocket or drone drop. The dock switches the board
to `SIM,ENABLE` and plays a pressure profile with `SIMP`. Then check, on the Overview page:

* PAD → ASCENT → DESCENT → ARMS_DEPLOY at 600 m → STEERING → LANDED;
* the servos unlatch at ARMS_DEPLOY (if they are wired);
* the event list shows every transition.

By hand (USB console): `CMD,001,SIM,ENABLE`, `CMD,001,CAL`, then `CMD,001,SIMP,<Pa>` at
5–10 Hz. `CMD,001,SIM,DISABLE` ends it.

## 4. Lift (elevator) test, real sensors

`CMD,001,LIFT,10` (or Bench → *Lift test*) on the ground floor, then ride up at least 4 floors and
come back down. Expected: ASCENT on the way up, DESCENT shortly after starting down, arms unlatch
10 m above the start floor, LANDED at the bottom. **The motors are hard-inhibited** in this mode.
`CMD,001,CAL` resets for another ride. `CMD,001,LIFT,OFF` (or a power cycle) restores flight
mode.

Negative test: in normal flight mode, a lift ride down must **not** release the arms (a carrier
descending slowly with the CanSat still attached).

## 5. Actuator bench tests: PROPS OFF

USB only. See `FLIGHT_SOFTWARE.md` §4: servo unlatch with `CHUTE`, motor order with `MTR`,
spin direction, mixer signs with `PID,START`.

## 6. Pre-flight checklist

- [ ] Flight log downloaded or no longer needed. Recorder pre-erased ≥ 3 MB (Health page:
      power on ≥ 2 minutes before launch).
- [ ] Boot ended with 2× green or the expected amber; phase `PAD`; altitude ≈ 0 m.
- [ ] GNSS ≥ 5 satellites, launch site set (Overview).
- [ ] Heading calibrated (`NORTH`) after any change to the IMU mounting.
- [ ] Ground station recording `Flight_2026-IN-SPACeCAN-7USAT-001.csv`.
- [ ] `CX,ON` sent when the judges allow radio transmission.
- [ ] Battery charged, props tight, arms latched.

## 7. After the flight

1. Connect USB → **Flight log** → *Refresh list* → select the flight → *Download selected*. You get a CSV
   of the 50 Hz records plus an events CSV.
2. If the Health page shows a crash dump: *Crash report*, then decode the addresses with
   `xtensa-esp32s3-elf-addr2line -pfiaC -e build/cansat_fw.elf <addresses>`.
3. Hand in `Flight_2026-IN-SPACeCAN-7USAT-001.csv`.
