# Build Guide — CAN-7USAT 2026 Flight Software

## Prerequisites

### 1. ESP-IDF v5.3 or later

**Windows (Recommended — Official Installer)**

1. Download the ESP-IDF Windows Installer from:
   `https://dl.espressif.com/dl/esp-idf/`
   Choose the **Offline installer** for ESP-IDF v5.3.x (includes all tools).

2. Run the installer. Accept defaults. It installs to:
   `C:\Espressif\`

3. After install, open **ESP-IDF Command Prompt** (created in Start Menu).
   All `idf.py` commands in this guide must be run inside that terminal.

**Alternative — VS Code + ESP-IDF Extension (Recommended for development)**

1. Install [VS Code](https://code.visualstudio.com/)
2. Install the **Espressif IDF** extension (ID: `espressif.esp-idf-extension`)
3. In VS Code: `Ctrl+Shift+P` → **ESP-IDF: Configure ESP-IDF extension**
4. Select **Express** install, choose IDF v5.3

**Manual Install (Advanced)**

```powershell
# Clone IDF
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.3

# Install tools
./install.ps1 esp32s3

# Add to environment (run each session, or add to profile)
./export.ps1
```

---

### 2. Verify Installation

Open ESP-IDF Command Prompt and run:

```bash
idf.py --version
# Expected output: ESP-IDF v5.3.x
python --version
# Expected: Python 3.8+
```

---

## Project Setup

### 1. Navigate to the firmware directory

```bash
cd "C:\Users\Lenovo\Desktop\Dev_Coding\Environments\Arduino_Projects\CANSAT\cansat_fw"
```

### 2. Set your Team ID

Before building, edit:
```
components/nav/include/nav/config.hpp
```

Find `TelemetryConfig` and set your team ID:

```cpp
struct TelemetryConfig {
    uint16_t team_id = 1234;   // ← REPLACE with your assigned CAN-7USAT team ID
    ...
};
```

Also verify the LoRa sync word matches your team ID (lower byte):

```cpp
uint8_t lora_sync_word = 0x12;  // Set to (team_id & 0xFF) if required by rules
```

### 3. Set target chip

```bash
idf.py set-target esp32s3
```

This creates `build/` and generates `sdkconfig` from `sdkconfig.defaults`.

---

## Build

### Standard build

```bash
idf.py build
```

Expected output on success:
```
[100%] Linking CXX executable cansat_fw.elf
esptool.py v4.x.x
...
Project build complete. To flash, run:
 idf.py flash
or
 idf.py -p PORT flash
```

Build artifacts are in `build/`:
- `cansat_fw.elf` — ELF image (for debugging)
- `cansat_fw.bin` — Factory binary
- `partition_table/partition-table.bin`

### Build with verbose output (troubleshooting)

```bash
idf.py build -v 2>&1 | tee build.log
```

### Clean build (if headers changed)

```bash
idf.py fullclean
idf.py build
```

---

## Configuration (menuconfig)

The project ships with `sdkconfig.defaults` pre-configured for ESP32-S3 WROOM-1. Only change if needed:

```bash
idf.py menuconfig
```

Key settings are under:
- `Component config → FreeRTOS` — tick rate, SMP, stack checks
- `Component config → ESP32S3-Specific` — CPU frequency (should be 240 MHz)
- `Serial flasher config` — flash size (must be 8 MB)
- `Partition Table` — custom, pointing to `partitions.csv`

**Do not change:**
- FreeRTOS tick rate (must stay 1000 Hz for 1 ms task periods)
- C++ exceptions (must stay disabled)
- Dual-core SMP mode (FREERTOS_UNICORE must stay `n`)

---

## Common Build Errors and Fixes

### Error: `fatal error: 'freertos/FreeRTOS.h' not found`
**Cause:** IDF environment not activated.
**Fix:** Open ESP-IDF Command Prompt (not a regular terminal).

---

### Error: `cmake: command not found` or `CMake Error`
**Cause:** CMake version too old (< 3.20).
**Fix:** The ESP-IDF installer bundles the correct CMake. Use the IDF terminal.

---

### Error: `idf_component_register: required component XXX not found`
**Cause:** A component REQUIRES entry references a non-existent component.
**Fix:** Check all `components/*/CMakeLists.txt` REQUIRES lists.

---

### Error: `undefined reference to 'nav::...'`
**Cause:** Template functions in nav headers are not being instantiated.
**Fix:** The nav `.cpp` stubs must include their headers. Check `components/nav/src/*.cpp`.

---

### Error: `redefinition of 'static constexpr ...'`
**Cause:** A `constexpr` variable in a header is included in multiple TUs without `inline`.
**Fix:** Add `inline` keyword to the constexpr declaration in the header, or move to a `.cpp`.

---

### Error: Flash size too small / partition table overflow
**Cause:** Binary exceeds factory partition (3 MB).
**Fix:**
```bash
idf.py size
```
Check section sizes. If `.text` > 2.5 MB, optimize:
- Set `CONFIG_LOG_DEFAULT_LEVEL_WARN=y` in sdkconfig.defaults
- Enable link-time optimization: `CONFIG_COMPILER_OPTIMIZATION_PERF=y`

---

### Warning: `double precision not supported, using float`
**Cause:** ESP32-S3 FPU is single-precision only; `double` uses software emulation.
**Expected behaviour:** The nav stack intentionally uses `double` for EKF stability. This warning is safe to ignore.

---

## Build Size Reference (approx.)

| Section | Target Size |
|---------|-------------|
| .text (code) | ~600–900 KB |
| .rodata | ~50–100 KB |
| .data + .bss | ~100–200 KB |
| Total flash | < 1.5 MB (well within 3 MB factory partition) |

---

## Component Dependency Tree

```
main
├── nav             (header-only EKF/IMM/supervisor)
├── hal             (I2C, SPI, UART bus wrappers)
├── drivers         (BNO085, BMP585, N-GS-01, SX1278, INA260, MAX17048, SDP31, SHT4x, SGP41)
│   └── hal
├── control         (PID, MotorMixer)
│   └── nav
├── telemetry       (TelemetryEncoder)
│   └── nav, drivers
├── comms           (LoRaLink, CommandParser)
│   ├── hal, drivers, nav
├── logging         (SDLogger, EventLog)
│   └── nav, nvs_flash
├── power           (PowerManager)
│   └── drivers
├── config_mgr      (NVSConfig)
│   └── nav, nvs_flash
├── watchdog        (Watchdog)
└── bit             (BuiltInTest)
    └── all above
```
