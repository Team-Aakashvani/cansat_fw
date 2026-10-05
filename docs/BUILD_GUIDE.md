# AAKASHVANI — Build Guide

## 1. Toolchain

* **ESP-IDF 5.5.5** (Windows offline installer, default path `C:\Espressif`).
* Python 3.11 (comes with the IDF environment).
* For the dock: `pip install customtkinter pyserial bleak matplotlib pillow websockets numpy`.

## 2. First build

Open the **ESP-IDF 5.5 PowerShell** from the Start menu, then:

```powershell
cd <path>\cansat_fw
idf.py set-target esp32s3     # only once; this regenerates sdkconfig from sdkconfig.defaults
idf.py build
```

The binary is `build/cansat_fw.bin` (about 730 KB, out of a 2 MB factory partition).

After the first configure, `python tools/build_cansat.py` rebuilds with ninja without opening
an IDF shell. It uses the paths written at the top of the script, so edit them if your IDF
lives somewhere else.

## 3. Configuration

* `sdkconfig.defaults` holds every non-default setting: 16 MB flash, octal PSRAM, NimBLE in
  PSRAM, core dump to flash, USB-Serial-JTAG console, partition table. If you change a setting in
  `menuconfig`, copy it into `sdkconfig.defaults` too. A fresh clone (or a deleted `sdkconfig`)
  is built from the defaults only.
* Mission and hardware constants live in `components/nav/include/nav/config.hpp`: pins
  (`PINS`), team number (`TELEM_CFG.team_id = 1`), flight profile (`PROFILE_CFG`), vertical
  filter (`VERT_CFG`), guidance (`GUIDE_CFG`) and steering gains (`ACT_CFG`).
* The team number stored in NVS overrides the compiled one at boot.

## 4. Host tests

```powershell
python tests/host/run_tests.py
```

This builds `tests/host/test_attitude.cpp` and `test_mission.cpp` with the host `g++` (MinGW or
MSYS2 on Windows), using small stubs for the IDF headers. It covers attitude maths, the vertical
filter's spike handling and full mission scenarios (rocket, drone drop, slow carrier, lift).

## 5. Troubleshooting

| Symptom | Fix |
|---|---|
| `xtensa-esp32s3-elf-gcc not found` | Use the ESP-IDF PowerShell, or run `export.ps1` first |
| PSRAM / flash size settings ignored | Delete `sdkconfig` and run `idf.py set-target esp32s3` again |
| `ninja: error` after moving the folder | `idf.py fullclean`, then `idf.py build` |
| Partition overflow | The app has 2 MB. Check for debug builds with `-O0` |
