# AAKASHVANI — Advanced Flashing & Deployment Manual
### Production Procedures for SVNIT Flight Avionics
> **The official guide for firmware deployment and partition management.**

---

## 1. Hardware Interface Identification

The ESP32-S3 WROOM-1 supports two methods of flashing. SVNIT standards prioritize the **Native USB** interface for its built-in JTAG debugging capabilities.

### 1.1 Native USB (Recommended)
*   **Port:** Connected directly to GPIO 19 (D-) and 20 (D+).
*   **Device Name:** `ESP32-S3 USB Serial/JTAG Controller`.
*   **Advantages:** Faster flashing (up to 2 Mbps), zero-config debugging.

### 1.2 UART Bridge
*   **Port:** Connected to GPIO 43 (TX) and 44 (RX).
*   **Device Name:** `Silicon Labs CP210x` or `CH340`.
*   **Note:** Use this only if the native USB port is physically damaged.

---

## 2. Deployment Workflow

### 2.1 The "Golden" Flash Command
To deploy a flight-ready image, use the high-speed baud rate:
```bash
idf.py -p COMx -b 921600 flash monitor
```

### 2.2 First-Time Board Preparation
When deploying to a brand-new ESP32-S3, you must wipe the NVS partition to clear factory-default junk data.
```bash
# Full Flash Erase (Destructive)
idf.py -p COMx erase-flash

# Re-flash everything
idf.py -p COMx flash
```

---

## 3. Partition Table Management

The flight computer uses a custom 8MB partition map (`partitions.csv`). Understanding this is critical for post-flight analysis.

| Name | Offset | Size | Criticality |
|------|--------|------|-------------|
| **nvs** | 0x9000 | 24KB | **HIGH** (Contains Team ID & Calib) |
| **factory** | 0x20000 | 3MB | **HIGH** (The Flight Firmware) |
| **ota_0** | 0x320000 | 3MB | LOW (Used for Field Updates) |
| **event_log**| 0x630000 | 1MB | **MED** (Your flight history) |
| **coredump** | 0x730000 | 320KB| **MED** (Crash diagnostics) |

---

## 4. Field Updates (OTA)

AAKASHVANI supports wireless firmware updates via the XBee link. This is used when the CanSat is already inside the rocket fairing.

1.  **Trigger:** Send `1234,OTA,START` via the GCS.
2.  **Transfer:** The GCS sends 32-byte hex chunks using `1234,OTA,CHUNK,<HEX>`.
3.  **Finalize:** Send `1234,OTA,FINISH`. The CanSat will swap partitions and reboot.

---

## 5. Post-Flash Verification

After flashing, the unit MUST be verified against these criteria:
1.  **Boot Count:** Verify the console logs `Boot #X`. If this does not increment, the NVS is read-only or corrupted.
2.  **Team ID:** Run `status` in the CLI. Ensure it matches your assigned ID.
3.  **Task Soak:** Leave the unit powered for 5 minutes. Verify no watchdog resets (`TWDT`) occur.

---

*Firmware deployment is a safety-critical operation. Follow this manual strictly.*
