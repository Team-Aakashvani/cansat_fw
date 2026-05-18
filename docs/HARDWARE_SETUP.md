# Hardware Setup Guide — CAN-7USAT 2026

## CanSat Physical Specification

| Parameter | Value |
|-----------|-------|
| Form factor | Cylindrical, 150 mm diameter × 400 mm height |
| Max mass | < 1 kg (target ~0.5 kg) |
| Apogee | ~1000 m AGL |
| Deployment altitude | 600 m ±10 m AGL |
| Target descent rate | 1–3 m/s (drone-stabilised) |

---

## Power System

### Flight Battery
- **Type:** 2S LiPo (7.4 V nominal, 8.4 V fully charged, 6.4 V cutoff)
- **Capacity:** ≥ 1000 mAh recommended (flight is ~5 min)
- **Connector:** XT30 or JST-PH2.0 as per your PCB footprint

### Power Distribution

```
LiPo 2S (7.4V)
    │
    ├── INA260 (current/voltage monitor, I2C)
    │       │
    │       └── 5V BEC ──► ESP32-S3 VIN (3.3V LDO on-board)
    │                  └── Logic sensors (3.3V)
    │
    ├── MAX17048 (fuel gauge, I2C, monitors raw LiPo voltage)
    │
    └── ESC ×4 (7.4V direct)  ──► BLDC motors ×4
```

**IMPORTANT:** Never power ESP32-S3 directly from 7.4 V. Use a 5 V BEC → ESP32-S3 5V pin.

### Power Consumption (typical)

| State | Current (7.4 V) | Power |
|-------|-----------------|-------|
| Idle (PRE_FLIGHT) | ~200 mA | 1.5 W |
| Active (BALLISTIC + comms) | ~350 mA | 2.6 W |
| Drone hover (motors @ 30%) | ~2.0 A | 14.8 W |
| Peak (motors @ 80%) | ~6.0 A | 44.4 W |

---

## GPIO Pin Assignment

### ESP32-S3 WROOM-1 Pinout (as configured in `config.hpp`)

```
                    ESP32-S3 WROOM-1
                 ┌──────────────────────┐
            3V3 ─┤ 3V3          GPIO1   ├─ (not used)
             GND─┤ GND          GPIO2   ├─ SD_D0
           GPIO3─┤ (not used)   GPIO3   ├─ (not used)
           GPIO4─┤ SD_D1        GPIO5   ├─ MOTOR_0
           GPIO6─┤ MOTOR_1      GPIO7   ├─ MOTOR_2
           GPIO8─┤ I2C0_SDA     GPIO9   ├─ I2C0_SCL
          GPIO10─┤ I2C1_SDA     GPIO11  ├─ I2C1_SCL
          GPIO12─┤ SD_D2        GPIO13  ├─ SD_D3
          GPIO14─┤ SD_CLK       GPIO15  ├─ SD_CMD
          GPIO16─┤ MOTOR_3      GPIO17  ├─ GNSS_TX
          GPIO18─┤ GNSS_RX      GPIO19  ├─ (USB D-)
          GPIO20─┤ (USB D+)     GPIO21  ├─ (not used)
          GPIO33─┤ LORA_RST     GPIO34  ├─ LORA_CS
          GPIO35─┤ SPI_MOSI     GPIO36  ├─ SPI_SCK
          GPIO37─┤ SPI_MISO     GPIO38  ├─ SERVO
          GPIO39─┤ BEACON       GPIO40  ├─ PWR_SWITCH
          GPIO48─┤ LED_STATUS           │
                 └──────────────────────┘
```

---

## I2C Bus 0 — Navigation Sensors (GPIO 8/9)

**Pull-up resistors:** 4.7 kΩ to 3.3 V on SDA and SCL. Do not omit.

### BNO085 (IMU)

| BNO085 Pin | ESP32-S3 Pin | Notes |
|------------|-------------|-------|
| VDD | 3.3V | |
| GND | GND | |
| SDA | GPIO8 | 4.7 kΩ pull-up |
| SCL | GPIO9 | 4.7 kΩ pull-up |
| PS0 | GND | Selects I2C mode (PS0=0, PS1=0) |
| PS1 | GND | |
| NRST | 3.3V via 10kΩ | Optional reset; tie high if unused |
| INT | (not connected) | Interrupt — unused in current firmware |

Default I2C address: **0x4A** (SA0 pin = GND)
To use address 0x4B: connect SA0 to 3.3V and update `I2C_ADDR` in `bno085.hpp`.

**Orientation:** Mount BNO085 with X-axis pointing toward the nose of the CanSat (upward on launch rail). The firmware assumes body-z aligned with launch rail vertical.

### BMP585 (Barometer)

| BMP585 Pin | ESP32-S3 Pin | Notes |
|------------|-------------|-------|
| VDDIO | 3.3V | |
| GND | GND | |
| SDA | GPIO8 | Shared with BNO085 |
| SCL | GPIO9 | Shared with BNO085 |
| SDO | GND | Sets I2C address 0x46 |
| CSB | 3.3V | Selects I2C mode (not SPI) |

Default I2C address: **0x46** (SDO = GND). For 0x47: SDO = 3.3V.

**Placement:** Mount BMP585 with a clear port to ambient air. Shield from direct solar radiation and motor downdraft. A small labyrinth vent is recommended.

---

## I2C Bus 1 — Environmental + Power (GPIO 10/11)

Same pull-up requirement: 4.7 kΩ to 3.3 V.

### INA260 (Voltage/Current Monitor)

| INA260 Pin | Connection | Notes |
|------------|-----------|-------|
| V+ | Battery+ (7.4V side) | Input rail |
| V- | Load+ (to BEC) | Output rail |
| ALERT | (not connected) | |
| SDA | GPIO10 | |
| SCL | GPIO11 | |
| GND | GND | |
| A0 | GND | I2C address 0x40 |
| A1 | GND | |

The INA260 measures bus voltage and bidirectional current on the main supply rail. Connect the bus voltage pins **in-line** with your main power path.

### MAX17048 (Fuel Gauge)

| MAX17048 Pin | Connection |
|-------------|-----------|
| VCELL | LiPo+ (direct, no load) |
| VSS | LiPo− |
| SDA | GPIO10 |
| SCL | GPIO11 |
| ALERT | (not connected) |

Fixed I2C address: **0x36**

### SDP31 (Differential Pressure — Airspeed)

| Pin | Connection |
|-----|-----------|
| VDD | 3.3V |
| GND | GND |
| SDA | GPIO10 |
| SCL | GPIO11 |
| Port 1 | Pitot tube facing forward |
| Port 2 | Static pressure port |

I2C address: **0x21**

### SHT4x (Humidity/Temperature)

| Pin | Connection |
|-----|-----------|
| VDD | 3.3V |
| GND | GND |
| SDA | GPIO10 |
| SCL | GPIO11 |

I2C address: **0x44**. Shield from direct sunlight and motor heat.

### SGP41 (VOC/NOx)

| Pin | Connection |
|-----|-----------|
| VDD | 3.3V |
| GND | GND |
| SDA | GPIO10 |
| SCL | GPIO11 |

I2C address: **0x59**

---

## SPI Bus — LoRa Radio (GPIO 33–37)

### SX1278 LoRa Module (e.g. Ra-02 or custom)

| SX1278 Pin | ESP32-S3 Pin | Notes |
|------------|-------------|-------|
| VCC | 3.3V | Max 200 mA peak during TX |
| GND | GND | |
| MOSI | GPIO35 | |
| MISO | GPIO37 | |
| SCK | GPIO36 | |
| NSS (CS) | GPIO34 | Active low |
| RESET | GPIO33 | Active low |
| DIO0 | GPIO32 | TxDone/RxDone IRQ |
| DIO1 | (not connected) | |
| ANT | Whip antenna | 17.3 cm for 433 MHz quarter-wave |

**Antenna:** A simple 17.3 cm wire monopole works. For better gain use a 3 dBi helical. **Never transmit without antenna connected.**

---

## UART Bus — N-GS-01 GNSS (GPIO 17/18)

| N-GS-01 Pin | ESP32-S3 Pin | Notes |
|-------------|-------------|-------|
| VCC | 3.3V or 5V (check module spec) | |
| GND | GND | |
| TX | GPIO18 (ESP RX) | GNSS transmits NMEA to ESP |
| RX | GPIO17 (ESP TX) | ESP sends config to GNSS |

Baud rate: **115200** (default N-GS-01 factory setting).
Protocol: NMEA-0183 sentences GPGGA, GPRMC, GPVTG.

**Antenna:** The N-GS-01 has an internal patch antenna. Position the CanSat with the GNSS module facing upward (antenna looking at sky). Avoid metal structures within 5 cm of the patch.

---

## Motor + Servo (LEDC PWM)

### ESC Connections

| Channel | GPIO | Motor Position | Rotation |
|---------|------|---------------|----------|
| LEDC_CH0 | GPIO5 | Front-Left | CW |
| LEDC_CH1 | GPIO6 | Front-Right | CCW |
| LEDC_CH2 | GPIO7 | Rear-Right | CW |
| LEDC_CH3 | GPIO16 | Rear-Left | CCW |
| LEDC_CH4 | GPIO38 | **Servo** | N/A |

All ESCs use standard 50 Hz PWM: 1000 µs = disarmed, 1050 µs = idle, 2000 µs = full throttle.

**ESC Calibration:** Before first flight, calibrate all four ESCs:
1. Power on with throttle at 2000 µs (or use calibration procedure in ESC manual)
2. Wait for beeps
3. Move to 1000 µs
4. Wait for confirming beeps

### Servo (Parachute Release)

- 1000 µs → Home (closed, tether retained)
- 2000 µs → Released (tether deployed)

Connect servo signal to GPIO38, servo power (5V) from BEC.

---

## Recovery Beacon

Connect a piezoelectric buzzer or transistor-driven siren to **GPIO39**.

| GPIO39 State | Beacon |
|-------------|--------|
| 0 | OFF |
| 1 | ON (50% duty 1 Hz after LANDED) |

Circuit: GPIO39 → 1 kΩ → NPN transistor (BC547) base. Collector to buzzer (5V). Emitter to GND.

---

## SD Card

| SDMMC Pin | ESP32-S3 Pin |
|-----------|-------------|
| CLK | GPIO14 |
| CMD | GPIO15 |
| D0 | GPIO2 |
| D1 | GPIO4 |
| D2 | GPIO12 |
| D3 | GPIO13 |
| VCC | 3.3V |
| GND | GND |

**Format:** FAT32. Cards up to 32 GB supported by ESP-IDF FATFS. Use **Class 10 / U1** minimum for reliable 1 Hz write performance.

---

## LED Status Indicator

The ESP32-S3 WROOM-1 includes an onboard RGB LED on **GPIO48**.

| Pattern | Meaning |
|---------|---------|
| Rapid blink (100 ms) | BIT FAILURE — halt |
| Solid | Normal operation (future: add phase-colour mapping) |

---

## PCB Design Recommendations

1. **Ground plane:** Full ground pour on bottom layer. No gaps under RF section.
2. **Decoupling caps:** 100 nF + 10 µF ceramic at each sensor VDD pin.
3. **I2C pull-ups:** Single 4.7 kΩ per line. Do not add multiple sets.
4. **GNSS keep-out:** 10 mm no-copper zone around GNSS patch antenna.
5. **LoRa keep-out:** 15 mm no-copper around LoRa module RF section.
6. **ESP32-S3 antenna:** Leave 10 mm clearance around the PCB trace antenna on the WROOM-1 module edge.
7. **SDMMC:** Keep SD traces short (< 30 mm), matched length, away from RF.
8. **Motor driver isolation:** ESC power ground and signal ground joined at single star point.
