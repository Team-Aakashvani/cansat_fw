# Telemetry Format — CAN-7USAT 2026

## Downlink CSV Format (CAN-7USAT §5.3)

One packet transmitted per second over LoRa at 433 MHz.

### Format

```
<TEAM_ID>,<MISSION_TIME>,<PACKET_COUNT>,<ALTITUDE>,<PRESSURE>,<TEMPERATURE>,<VOLTAGE>,<GNSS_TIME>,<LATITUDE>,<LONGITUDE>,<GNSS_ALT>,<SATS>,<TILT_X>,<TILT_Y>,<ROT_Z>,<SOFTWARE_STATE>\n
```

### Field Definitions

| # | Field | Type | Format | Unit | Source | Example |
|---|-------|------|--------|------|--------|---------|
| 1 | `TEAM_ID` | uint16 | `%u` | — | Config | `1234` |
| 2 | `MISSION_TIME` | string | `HH:MM:SS` | s | Elapsed since BOOST latch | `00:04:32` |
| 3 | `PACKET_COUNT` | uint32 | `%u` | — | Increments 1/s | `272` |
| 4 | `ALTITUDE` | float | `%.2f` | m AGL | BMP585 → EKF | `587.34` |
| 5 | `PRESSURE` | float | `%.1f` | Pa | BMP585 | `94312.5` |
| 6 | `TEMPERATURE` | float | `%.1f` | °C | BMP585 | `18.3` |
| 7 | `VOLTAGE` | float | `%.2f` | V | INA260 | `7.41` |
| 8 | `GNSS_TIME` | string | `HH:MM:SS` | UTC | N-GS-01 | `07:15:44` |
| 9 | `LATITUDE` | float | `%.6f` | ° (decimal) | N-GS-01 | `12.971600` |
| 10 | `LONGITUDE` | float | `%.6f` | ° (decimal) | N-GS-01 | `77.594600` |
| 11 | `GNSS_ALT` | float | `%.2f` | m MSL | N-GS-01 | `912.30` |
| 12 | `SATS` | int | `%d` | — | N-GS-01 | `8` |
| 13 | `TILT_X` | float | `%.2f` | m/s² or ° | BNO085 accel X (or EKF pitch°) | `0.12` |
| 14 | `TILT_Y` | float | `%.2f` | m/s² or ° | BNO085 accel Y (or EKF roll°) | `-0.04` |
| 15 | `ROT_Z` | float | `%.2f` | °/s | BNO085 gyro Z | `1.23` |
| 16 | `SOFTWARE_STATE` | uint8 | `%u` | — | Phase code (0–5) | `3` |

> **Fallback values when a sensor is invalid:**
> - Altitude: EKF position Z
> - Pressure: 101325.0 Pa
> - Temperature: 25.0 °C
> - Voltage: 0.0 V
> - GNSS fields: latitude=0, longitude=0, gnss_alt=0, sats=0, gnss_time=`00:00:00`
> - TILT_X/Y: EKF Euler pitch/roll (°) instead of raw accel
> - ROT_Z: 0.0

---

### Example Packet

```
1234,00:04:32,272,587.34,94312.5,18.3,7.41,07:15:44,12.971600,77.594600,912.30,8,0.12,-0.04,1.23,3
```

Breaking down:
- Team ID: **1234**
- Mission time: **4 min 32 s** after launch
- Packet number: **272**
- Altitude AGL: **587.34 m** (near 600 m deploy window)
- Pressure: **94312.5 Pa**
- Temperature: **18.3 °C**
- Battery: **7.41 V**
- UTC time: **07:15:44**
- Position: **12.9716°N, 77.5946°E** (Bengaluru approx.)
- GNSS altitude MSL: **912.30 m**
- Satellites: **8**
- Accel X: **0.12 m/s²**
- Accel Y: **-0.04 m/s²**
- Gyro Z: **1.23 °/s**
- State: **3** (PARACHUTE)

---

## SOFTWARE_STATE Values

| Code | Phase | Description |
|------|-------|-------------|
| 0 | PRE_FLIGHT | On pad, awaiting launch |
| 1 | BOOST | Rocket motor burning |
| 2 | BALLISTIC | Coasting after burnout |
| 3 | PARACHUTE | Chute deployed, passive descent |
| 4 | DRONE_HOVER | Active PID descent |
| 5 | LANDED | On ground, beacon active |

---

## LoRa Link Parameters (CAN-7USAT §6.2)

| Parameter | Value |
|-----------|-------|
| Frequency | 433 MHz |
| Spreading Factor | SF10 |
| Bandwidth | 125 kHz |
| Coding Rate | CR 4/5 |
| TX Power | 17 dBm (50 mW) |
| Preamble | 8 symbols |
| Sync Word | `0x12` (or lower byte of team ID) |
| Modulation | LoRa (CSS) |
| CRC | Enabled |

### Approximate Range & Data Rate

| SF | Symbol Rate | Packet Time (90 bytes) | Range (LoS) |
|----|-------------|----------------------|-------------|
| SF7 | fast | ~56 ms | ~2 km |
| SF10 | medium | ~330 ms | ~8 km |
| SF12 | slow | ~1300 ms | ~15 km |

SF10 leaves 670 ms of receive window per second — sufficient for uplink command reception.

---

## Uplink Command Format (Ground → CanSat)

```
<TEAM_ID>,<CMD>[,<ARG>]\n
```

### Supported Commands

| Command | Argument | Description |
|---------|---------|-------------|
| `CX,ON` | — | Enable telemetry downlink |
| `CX,OFF` | — | Disable telemetry downlink |
| `ST,HH:MM:SS` | Time string | Override mission timer |
| `CAL` | — | Calibrate ground altitude (uses current baro) |
| `SIM,ENABLE` | — | Enter simulation mode |
| `SIM,DISABLE` | — | Exit simulation mode |
| `SIM,ACTIVATE` | — | Same as ENABLE (alias) |
| `SIMP,<pa>` | Pressure Pa | Feed simulated pressure in SIM mode |

**Examples:**

```
1234,CX,ON
1234,ST,00:00:00
1234,CAL
1234,SIM,ENABLE
1234,SIMP,95000
```

---

## SD Card Log Format

The SD card file `CANSAT_XXXX.csv` has the same format as the telemetry downlink:

**Line 1 — Header:**
```
TEAM_ID,MISSION_TIME,PACKET_COUNT,ALTITUDE,PRESSURE,TEMPERATURE,VOLTAGE,GNSS_TIME,LATITUDE,LONGITUDE,GNSS_ALT,SATS,TILT_X,TILT_Y,ROT_Z,SOFTWARE_STATE
```

**Subsequent lines — Data (one per second):**
```
1234,00:00:01,1,0.12,101234.5,28.3,7.41,12:30:01,12.971600,77.594600,912.30,7,0.12,-0.04,0.02,0
1234,00:00:02,2,0.15,101233.2,28.3,7.41,12:30:02,12.971600,77.594600,912.31,7,0.11,-0.03,0.01,0
...
```

File flush occurs every **5 seconds**. Last 5 s of data may be lost if power is cut abruptly.

---

## Ground Station Setup

### Minimum GCS Hardware

Any of the following:
- Second ESP32-S3 with SX1278 running a simple receive-print sketch
- TTGO LoRa32 (ESP32 + SX1278 integrated)
- Raspberry Pi + Ra-02 SX1278 module via SPI

### GCS Receive Parameters

Configure your GCS radio to:
```
Frequency:   433.0 MHz
SF:          10
BW:          125 kHz
CR:          4/5
Sync Word:   0x12  (or your team ID lower byte)
CRC:         enabled
```

### Parsing the CSV

Python example:

```python
import serial

FIELDS = [
    "team_id", "mission_time", "packet_count",
    "altitude", "pressure", "temperature", "voltage",
    "gnss_time", "latitude", "longitude", "gnss_alt", "sats",
    "tilt_x", "tilt_y", "rot_z", "software_state"
]

port = serial.Serial("COM5", 115200)  # Your GCS LoRa serial port

while True:
    line = port.readline().decode().strip()
    if not line or ',' not in line:
        continue
    parts = line.split(',')
    if len(parts) != len(FIELDS):
        continue
    data = dict(zip(FIELDS, parts))
    print(f"ALT={data['altitude']}m  STATE={data['software_state']}  SATS={data['sats']}")
```

### Sending Uplink Commands

```python
def send_command(port, team_id, cmd, arg=""):
    pkt = f"{team_id},{cmd}"
    if arg:
        pkt += f",{arg}"
    pkt += "\n"
    port.write(pkt.encode())
    print(f"Sent: {pkt.strip()}")

send_command(port, 1234, "CX", "ON")
send_command(port, 1234, "CAL")
send_command(port, 1234, "ST", "00:00:00")
```

---

## Telemetry Validation Rules

Apply these checks in your GCS to flag anomalous packets:

| Field | Valid Range | Anomaly Action |
|-------|------------|----------------|
| ALTITUDE | −100 to 2000 m | Flag, do not plot |
| PRESSURE | 60000 to 110000 Pa | Flag |
| TEMPERATURE | −40 to 85 °C | Flag |
| VOLTAGE | 3.0 to 8.5 V | Alert if < 6.8 V |
| LATITUDE | 8 to 37 °N (India) | Flag if 0.0 |
| LONGITUDE | 68 to 97 °E (India) | Flag if 0.0 |
| SATS | 0 to 30 | Alert if < 4 during flight |
| SOFTWARE_STATE | 0 to 5 | Flag if > 5 |
| PACKET_COUNT | Monotonically increasing | Flag if gaps > 3 |
