# AAKASHVANI — Professional Build & Toolchain Guide
### For SVNIT Flight Software v1.1.0
> **Step-by-step instructions for reproducing the flight-certified binaries.**

---

## 1. Prerequisites (Mandatory)

### 1.1 The Toolchain
*   **ESP-IDF v5.3.x:** You MUST use version 5.3 or later. Earlier versions lack support for the ESP32-S3's native USB-JTAG controller features used in this project.
*   **Python 3.10+:** Required for the EKF matrix generation and telemetry parsing scripts.
*   **CMake 3.24+:** Required for modular component registration.

---

## 2. Environment Setup

### 2.1 Windows Installation
1.  Download the **ESP-IDF Windows Offline Installer** (v5.3).
2.  During install, ensure the following are checked:
    *   `ESP-IDF Build Tools`
    *   `Xtensa ESP32-S3 Toolchain`
3.  Launch the **ESP-IDF PowerShell** from the Start Menu.

### 2.2 VS Code Integration (Recommended)
1.  Install the **Espressif IDF Extension**.
2.  Run `ESP-IDF: Configure ESP-IDF Extension` and point it to your installation path.
3.  Ensure `idf.py` is available in the VS Code integrated terminal.

---

## 3. Build Procedure

### 3.1 Step 1: Target Definition
The project must be told which chip it is running on before it can compile the HAL.
```powershell
idf.py set-target esp32s3
```

### 3.2 Step 2: Role Selection
The AAKASHVANI codebase can compile into two different roles.
1.  Run `idf.py menuconfig`.
2.  Navigate to: `CAN-7USAT Build Target`.
3.  Choose: `ROLE_FC` (for the Flight Computer) or `ROLE_GCS` (for the Ground Station).
4.  Press `S` to save and `Q` to quit.

### 3.3 Step 3: Compilation
Execute the multicore build command.
```powershell
idf.py build
```
The build process typically takes 45–90 seconds on a modern quad-core CPU. On success, your binaries will be in the `build/` directory.

---

## 4. Flash & Boot Verification

### 4.1 Initial Flash
Connect the board via the **USB-C** port.
```powershell
idf.py -p COMx flash monitor
```

### 4.2 Verifying the "Silver Bullet" Boot
Observe the serial output. A successful build will show:
1.  `I (xxx) main: Flight Software v1.1.0 Init`
2.  `I (xxx) BIT: [BIT] IMU: PASS`
3.  `I (xxx) BIT: [BIT] BARO: PASS`
4.  `I (xxx) main: All tasks spawned. System Running.`

---

## 5. Troubleshooting the Build

| Error Message | Likely Cause | Resolution |
|---------------|--------------|------------|
| `ninja: error: build.ninja` | Dirty build directory. | `idf.py fullclean` and retry. |
| `nav/config.hpp: No such file` | Missing submodule. | `git submodule update --init --recursive` |
| `Region 'factory' overflow` | Binary too large. | Reduce `LOG_DEFAULT_LEVEL` in `menuconfig`. |
| `xtensa-esp32s3-elf-gcc not found` | Environment not exported.| Run `export.bat` or `export.ps1`. |

---

*This guide ensures binary parity across all team development machines.*
