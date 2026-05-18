# Testing Guide — CAN-7USAT 2026 Flight Software

## Overview

Testing is split into four stages:
1. **BIT (Built-In Test)** — automatic power-on self-test
2. **Bench Test** — sensor and telemetry verification on the desk
3. **Integration Test** — full system with flight battery and GCS
4. **Pre-Flight Checklist** — final go/no-go before launch

---

## Stage 1 — BIT (Built-In Test)

BIT runs automatically at every boot. Check the serial monitor output.

### Pass criteria

```
I (xxx) BIT: === BIT PASS (flags=0x00000000) ===
```

### Interpreting BIT flags

| Bit | Flag | Meaning | Critical? |
|-----|------|---------|-----------|
| 0 | `BIT_IMU_ABSENT` | BNO085 not responding | YES — halts |
| 1 | `BIT_BARO_ABSENT` | BMP585 not responding | YES — halts |
| 2 | `BIT_POWER_ABSENT` | INA260 not responding | YES — halts |
| 3 | `BIT_GNSS_NO_NMEA` | No NMEA from N-GS-01 in 2 s | No — logs warning |
| 4 | `BIT_LORA_ABSENT` | SX1278 not detected | YES — halts |
| 5 | `BIT_SD_FAIL` | SD card not mounted | No — logs warning |
| 6 | `BIT_NVS_FAIL` | NVS namespace error | No — uses defaults |
| 7 | `BIT_IMU_SANITY` | \|accel\| outside 7.8–11.8 m/s² | No — warns |
| 8 | `BIT_BARO_SANITY` | Pressure outside 70–110 kPa | No — warns |
| 9 | `BIT_VOLTAGE_LOW` | Voltage < 3.0 V | No — warns |

### If BIT halts (critical failure)

The status LED blinks rapidly (100 ms on/off) and the system stops. Check:
- I2C wiring continuity (SDA/SCL correct, 4.7 kΩ pull-ups present)
- SPI CS/MOSI/MISO/CLK wiring
- Sensor power supply (3.3 V rail must be stable)

To identify which sensor failed, read the serial output lines before the halt.

---

## Stage 2 — Bench Test

### 2.0 USB CLI & Maintenance Mode

The CanSat provides a direct configuration interface over the USB-C port.

**Test Procedure:**
1. Connect CanSat to PC.
2. Open a serial terminal (e.g., PuTTY, Minicom, or `idf.py monitor`) at **115200 baud**.
3. Press `Enter` to see the `> ` prompt.
4. Type `help` to list commands.

**Checks:**
- [ ] `status` command returns current Team ID and Ground Altitude.
- [ ] `get team_id` returns the value set in `config.hpp`.
- [ ] `set team_id <new_id>` persists the value (verify with `get` after `reboot`).
- [ ] `dispatch CAL` triggers a calibration event (check serial log).

---

### 2.1 IMU Test

**Expected output (100 Hz log rate — use DEBUG level):**
```
D (xxx) nav_task: IMU acc=[0.12, 0.04, 9.81] m/s² gyr=[0.00, 0.00, 0.00] rad/s
```

**Checks:**
- [ ] `acc_z ≈ +9.81 m/s²` when board is flat and face-up
- [ ] No `IMU: read failed` messages
- [ ] Rotate board — accel components shift correctly between axes
- [ ] Gyro reads ~0 when stationary (bias < 0.05 rad/s)

### 2.2 Barometer Test

**Expected output:**
```
I (xxx) BMP585: ready, CHIP_ID=0x51
D (xxx) sensor_task: BARO P=101234.5Pa T=28.3°C alt=-1.2m
```

**Checks:**
- [ ] Pressure within ±5 kPa of your local sea-level pressure
- [ ] Temperature within ±5 °C of ambient
- [ ] Altitude changes when you carry the board upstairs (+1 m per floor ~= −12 Pa)

### 2.3 GNSS Test

GNSS requires a clear sky view. Test outdoors or near a window.

**Expected output after fix (up to 60 s first cold start):**
```
I (xxx) NGPS01: GGA: lat=12.971600 lon=77.594600 alt=912.3m sats=7 fix=1
```

**Checks:**
- [ ] `fix_quality >= 1` (1=GPS, 4=NavIC)
- [ ] `satellites >= 4`
- [ ] Coordinates match your actual location (±5 m)
- [ ] `time_str` matches UTC time

### 2.4 LoRa Self-Test

**Using two boards (loopback):**

On the transmitting board, send a CX command from the GCS:
```
1234,CX,ON
```
Expected:
```
I (xxx) LoRaLink: RX cmd type=1 arg='ON' RSSI=-45
I (xxx) CmdParser: CX → telemetry ON
```

**Without a second board:**
Check SX1278 version register:
```
I (xxx) SX1278: version=0x12 — OK
```

**Checks:**
- [ ] `LoRa link ready` message in boot log
- [ ] No `SX1278 init failed` error
- [ ] Telemetry packets transmitted every 1 s (`TX[1] 89 bytes`)

### 2.5 Telemetry Format Verification

Enable telemetry with a CX ON command or verify it is enabled by default.
Watch for 1 Hz output on the serial monitor (copy from DEBUG log or SD card):

```
1234,00:00:05,5,23.4,101234.5,28.3,7.41,12:30:00,12.971600,77.594600,912.30,7,0.12,0.04,0.02,0
```

Verify against [TELEMETRY_FORMAT.md](TELEMETRY_FORMAT.md):
- [ ] 16 comma-separated fields
- [ ] Team ID matches config
- [ ] Packet count increments each second
- [ ] All floating-point fields have correct decimal places
- [ ] `SOFTWARE_STATE` = 0 (PRE_FLIGHT) before launch

### 2.6 SD Card Test

Insert a formatted (FAT32) micro-SD card.
```
I (xxx) SDLogger: Logging to /sdcard/CANSAT_0001.csv
```

After 10 seconds, remove the SD card and read it on a PC:
- [ ] `CANSAT_0001.csv` exists
- [ ] First line is the CSV header
- [ ] Subsequent lines are valid telemetry rows

### 2.7 Power Monitor Test

```
I (xxx) PowerMgr: Power status: 0 → 0 (V=7.42V SoC=87.5%)
```

**Checks:**
- [ ] Voltage matches multimeter reading ±0.1 V
- [ ] SoC% is plausible for your battery charge state
- [ ] No `LOW_VOLTAGE` warning at full charge

---

## Stage 3 — Integration Test

Assemble the full CanSat (sensors, battery, LoRa antenna, SD card). Run the ground station.

### 3.1 Ground Station Setup

Minimum GCS setup: a second ESP32 or an SX1278 module connected to a PC running a serial monitor that prints raw LoRa packets.

**Recommended:** Use the [CAN-7USAT GCS software](https://cansat.in) or any LoRa receive tool at 433 MHz, SF10, BW125, CR4/5.

### 3.2 Range Test

- [ ] LoRa packets received at 50 m range (RSSI > −100 dBm)
- [ ] Packet loss rate < 5% over 60 s at 50 m (= < 3 missed packets in 60)
- [ ] GCS software plots telemetry in real time

### 3.3 Command Uplink Test

From the GCS, send each command and verify response:

| Command | Expected Behaviour |
|---------|-------------------|
| `1234,CX,ON` | Telemetry starts (if previously off) |
| `1234,CX,OFF` | Telemetry pauses |
| `1234,ST,00:00:00` | Mission time reset to 0 |
| `1234,CAL` | Ground altitude set to current baro reading |
| `1234,SIM,ENABLE` | Simulation mode activated |
| `1234,SIMP,101325` | Pressure set to 101325 Pa in sim mode |
| `1234,SIM,DISABLE` | Simulation mode off |

### 3.4 Vibration Test (drop test)

With the CanSat fully assembled:
1. Place on a vibrating surface (or tap firmly with finger)
2. Verify IMU does not freeze or report all-zero accel
3. Verify SD file is not corrupted after vibration
4. Verify LoRa TX continues without gaps

### 3.5 Thermal Test (optional)

- [ ] Operates at ambient -10 °C to +60 °C (typical competition range)
- [ ] BMP585 temperature reading tracks a reference thermometer
- [ ] No reset or watchdog trigger during 30-minute soak

---

## Stage 4 — Pre-Flight Checklist

Complete immediately before launch. Print this checklist.

### Hardware
- [ ] Flight battery fully charged (measured voltage ≥ 7.2 V for 2S LiPo)
- [ ] SD card formatted FAT32 and inserted
- [ ] LoRa antenna connected (TX into open air without antenna WILL damage SX1278)
- [ ] Parachute packed and tether attached
- [ ] Servo mechanism moves freely (servo_release → servo_home)
- [ ] All I2C/SPI/UART wiring secured and strain-relieved
- [ ] Buzzer/beacon connected to GPIO39

### Software State
- [ ] Power the CanSat; wait for BIT PASS in GCS telemetry
- [ ] Team ID in `SOFTWARE_STATE = 0` (PRE_FLIGHT)
- [ ] `PACKET_COUNT` incrementing at 1 Hz
- [ ] `ALTITUDE` within ±20 m of launch site known elevation
- [ ] `PRESSURE` within ±500 Pa of local met station
- [ ] `GNSS_TIME` shows correct UTC ±5 min
- [ ] `SATS ≥ 4` (wait for fix if needed)
- [ ] `VOLTAGE` ≥ 7.1 V
- [ ] Mission time = `00:00:00` (or set via ST command)

### Ground Station
- [ ] GCS receiving packets at 1 Hz with RSSI > −100 dBm
- [ ] `CX,ON` command acknowledged
- [ ] `CAL` command sent to calibrate ground altitude (send once on-pad)
- [ ] Packet log file started on GCS PC

### Final
- [ ] Remove USB cable (board runs from battery)
- [ ] Confirm LED status light behaviour (green = nominal in PRE_FLIGHT)
- [ ] Hand CanSat to launch team. Do NOT send `ST` after this point.

---

## Software State Reference

| `SOFTWARE_STATE` | Phase | Description |
|-----------------|-------|-------------|
| 0 | PRE_FLIGHT | Sitting on pad, waiting for launch |
| 1 | BOOST | Rocket motor burning |
| 2 | BALLISTIC | Coasting upward after burnout |
| 3 | PARACHUTE | Parachute deployed, descending |
| 4 | DRONE_HOVER | Active stabilised descent |
| 5 | LANDED | On the ground, beacon active |

---

## Post-Flight Analysis

### 5.1 Event Log Reading
The event log survives software resets and can be read via the USB CLI after recovery:
1. Connect via USB.
2. Run `status` to see general flight stats (boot count, final ground alt).
3. (Future) Use `log_dump` to print the NVS event ring buffer.

### 5.2 Crash Analysis (Coredump)
If the CanSat crashed or reset during flight due to a software panic:
1. Insert the SD card and boot the CanSat.
2. The firmware automatically detects the crash dump in flash.
3. Look for the message: `I (xxx) CoredumpExporter: SUCCESS: Coredump saved to /sdcard/CRASH_XXXX.bin`.
4. Copy the `.bin` file to your PC and use `espcoredump.py` to analyze:
   ```bash
   python -m esp_coredump info_corefile -t b -c CRASH_XXXX.bin build/factory.elf
   ```

---

## Known Test Limitations

| Limitation | Notes |
|------------|-------|
| GNSS cold start 30–90 s | Power on 5 min before launch for warm start |
| SD write lag | Flush every 5 s; last 5 s may be lost on hard impact landing |
| LoRa range ≈ 5–10 km at 433 MHz SF10 | Sufficient for 1 km apogee |
