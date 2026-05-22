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
          GPIO18─┤ GNSS_RX      GPIO19  ├─ (USB-JTAG D-)
          GPIO20─┤ (USB-JTAG D+)GPIO21  ├─ CC1101_CS
          GPIO26─┤ P4_LINK_TX   GPIO27  ├─ P4_LINK_RX
          GPIO32─┤ XBEE_RX      GPIO34  ├─ XBEE_TX
          GPIO35─┤ SPI_MOSI     GPIO36  ├─ SPI_SCK
          GPIO37─┤ SPI_MISO     GPIO38  ├─ SERVO
          GPIO39─┤ BEACON       GPIO40  ├─ PWR_SWITCH
          GPIO48─┤ LED_STATUS           │
                 └──────────────────────┘
```

**NOTE:** For a detailed point-to-point mapping of all wires, see [WIRING_DIAGRAM.md](./WIRING_DIAGRAM.md).

---

## High-Speed Serial Links

### XBee Pro (Primary Telemetry)
The XBee Pro operates in **Transparent Mode** on UART2. It carries the primary 1 Hz telemetry CSV and receives uplink commands.

| XBee Pro Pin | ESP32-S3 Pin | Notes |
|------------|-------------|-------|
| DIN (Pin 3) | GPIO34 | TX from ESP |
| DOUT (Pin 2)| GPIO32 | RX to ESP |
| VCC | 3.3V | 215mA peak TX |
| GND | GND | |

### P4 Media Link (Video Control)
Dedicated link to the ESP32-P4 coprocessor for video state management.

| P4 Signal | ESP32-S3 Pin | Notes |
|------------|-------------|-------|
| P4_RX | GPIO27 | Commands to P4 |
| P4_TX | GPIO26 | Heartbeat from P4 |

### GNSS (NavIC)
Standard UART1 connection to the N-GS-01 module.

| N-GS-01 Pin | ESP32-S3 Pin | Notes |
|-------------|-------------|-------|
| TX | GPIO18 | NMEA Data |
| RX | GPIO17 | Configuration |

---

## SPI Bus — CC1101 RF Scanner (GPIO 35–37)

The SPI bus is used exclusively by the **CC1101 RF scanner** for the secondary mission objective.

| CC1101 Pin | ESP32-S3 Pin |
|------------|-------------|
| MOSI | GPIO35 |
| MISO | GPIO37 |
| SCK | GPIO36 |
| CS | GPIO21 |
| VCC | 3.3V |
| GND | GND |

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
