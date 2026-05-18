# Flash Guide — CAN-7USAT 2026 Flight Software

## Step 1 — Connect the ESP32-S3

Connect the ESP32-S3 WROOM-1 to your PC via USB. The board uses either:
- **USB-UART bridge** (CP2102 / CH340 chip) → appears as COM port
- **Native USB** (ESP32-S3 USB OTG) → appears as COM port with JTAG

### Find your COM port (Windows)

Open **Device Manager** (`Win + X` → Device Manager) → **Ports (COM & LPT)**.
Look for:
- `Silicon Labs CP210x USB to UART Bridge (COM3)` — if using CP2102 bridge
- `USB Serial Device (COM4)` — if using native USB

If no port appears:
- Install CP210x driver: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
- Install CH340 driver: https://www.wch.cn/downloads/CH341SER_ZIP.html

---

## Step 2 — Enter Download Mode (if needed)

Most ESP32-S3 development boards auto-reset into download mode during flash. If not:

1. Hold the **BOOT** button (GPIO0)
2. Press and release the **RESET** (EN) button
3. Release **BOOT**

The board is now in download mode (no LED activity).

---

## Step 3 — First-Time Full Flash

Open **ESP-IDF Command Prompt** and navigate to the project:

```bash
cd "C:\Users\Lenovo\Desktop\Dev_Coding\Environments\Arduino_Projects\CANSAT\cansat_fw"
```

### Flash everything (bootloader + partition table + firmware)

```bash
idf.py -p COM3 flash
```
Replace `COM3` with your actual port.

This flashes:
- Bootloader (`build/bootloader/bootloader.bin` at 0x0000)
- Partition table (`build/partition_table/partition-table.bin` at 0x8000)
- Factory application (`build/cansat_fw.bin` at 0x10000)

### Flash + open monitor (recommended)

```bash
idf.py -p COM3 flash monitor
```

Press `Ctrl + ]` to exit the monitor.

---

## Step 4 — Erase NVS Before First Flight

The NVS partition stores team ID, ground altitude, and calibration. Erase it once on a fresh board:

```bash
idf.py -p COM3 erase-flash
idf.py -p COM3 flash
```

Or erase only the NVS partition (preserves firmware):

```bash
# Erase NVS partition (offset 0x9000, size 0x6000 = 24 KB from partitions.csv)
python -m esptool --chip esp32s3 -p COM3 erase_region 0x9000 0x6000
```

---

## Step 5 — Monitor Serial Output

```bash
idf.py -p COM3 monitor
```

Expected boot sequence output:
```
I (xxx) main: === CAN-7USAT 2026 Flight Software v1.0 ===
I (xxx) main: ESP-IDF v5.3 | CPU @ 240MHz
I (xxx) NVSConfig: NVS ready (team_id=1234 ground_alt=0.0m boot=1)
I (xxx) main: Initialising HAL...
I (xxx) I2CBus: I2C-0 ready (SDA=8 SCL=9 400kHz)
I (xxx) I2CBus: I2C-1 ready (SDA=10 SCL=11 400kHz)
I (xxx) main: Initialising sensors...
I (xxx) BNO085: SHTP ready, version 4.x.x
I (xxx) BMP585: ready, CHIP_ID=0x51
I (xxx) main: Initialising LoRa link...
I (xxx) LoRaLink: LoRa link ready (433MHz SF10 BW125 CR4/5)
I (xxx) main: Running BIT...
I (xxx) BIT: [BIT] IMU: PASS (|a|=9.82 m/s²)
I (xxx) BIT: [BIT] BARO: PASS (P=101234.5 Pa T=28.3°C alt=-2.1m)
I (xxx) BIT: [BIT] Power: PASS (V=7.41V I=0.120A P=0.89W)
I (xxx) BIT: === BIT PASS (flags=0x00000000) ===
I (xxx) main: All tasks spawned. Flight software running.
```

---

## Flashing Without idf.py (esptool directly)

If you only have pre-built binaries:

```bash
python -m esptool --chip esp32s3 -p COM3 -b 460800 \
  --before default_reset --after hard_reset write_flash \
  0x0     build/bootloader/bootloader.bin \
  0x8000  build/partition_table/partition-table.bin \
  0x10000 build/cansat_fw.bin
```

---

## Flash Speed

| Baud Rate | Flash Time (1.5 MB binary) |
|-----------|---------------------------|
| 115200 | ~2 min |
| 460800 | ~30 s |
| 921600 | ~15 s |

Set baud rate:
```bash
idf.py -p COM3 -b 921600 flash
```

---

## OTA Update (Over-The-Air)

The partition table includes `ota_0` (3 MB). OTA is not yet implemented in v1.0 but the partition is reserved. To re-flash mid-campaign without USB access, implement `esp_ota_*` API calls in a future update.

---

## Troubleshooting

### `A fatal error occurred: Failed to connect to ESP32-S3`

**Cause:** Board not in download mode, or wrong COM port.
**Fix:**
1. Hold BOOT, tap RESET, release BOOT
2. Try `idf.py -p COM3 --before no_reset flash`
3. Try lower baud: `idf.py -p COM3 -b 115200 flash`

---

### `esptool.py: error: Failed to write ... target flash`

**Cause:** Flash write error, possibly due to flash chip mismatch.
**Fix:** Erase flash completely first:
```bash
idf.py -p COM3 erase-flash
idf.py -p COM3 flash
```

---

### Monitor garbled / no output

**Cause:** Wrong baud rate selected in terminal, or UART pins misconfigured.
**Fix:** `idf.py monitor` auto-detects. If using a manual terminal, set 115200 8N1.

---

### Board resets immediately after flashing (brownout)

**Cause:** USB power insufficient (voltage < 2.76 V threshold set in sdkconfig).
**Fix:** Use a powered USB hub, or power the ESP32-S3 from the flight battery via the 3.3V regulator.

---

## Post-Flash Verification Checklist

- [ ] BIT PASS logged in serial monitor
- [ ] Team ID shown correctly: `team_id=XXXX`
- [ ] Barometric pressure reads ~101325 Pa (sea level) ±5000 Pa
- [ ] IMU specific-force ~9.81 m/s² ±2
- [ ] Voltage reads > 3.0 V (USB) or > 6.4 V (flight battery)
- [ ] LoRa: `LoRa link ready` message appears
- [ ] SD card: `Logging to /sdcard/CANSAT_0001.csv` (if SD inserted)
