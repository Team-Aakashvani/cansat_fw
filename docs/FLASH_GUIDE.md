# AAKASHVANI — Flashing Guide

The board is flashed over its **USB** port, the native USB-Serial-JTAG port (the one the
dock uses). It shows up as `USB Serial Device (COMx)`. On the development PC it is COM15.

## Option A: the dock (easiest)

1. Build the firmware (see `BUILD_GUIDE.md`) so that `build/` contains the binaries.
2. Open the dock (`LAUNCH_DOCK.bat`) and go to the **Firmware** page.
3. Pick the port and press **Flash firmware**. The dock disconnects and flashes everything listed in
   `build/flasher_args.json`. Press **Connect** again when it has finished.

## Option B: command line

```powershell
python tools/flash_cansat.py COM15     # finds the port itself if you leave it out
# or, inside the ESP-IDF shell:
idf.py -p COM15 flash
```

## What flashing keeps

Normal flashing writes the bootloader, partition table and app. It does **not** touch:

* **NVS**: team number, ground altitude, heading offset;
* **the flight recorder** (12 MB) and the **crash dump**.

`idf.py erase-flash` wipes all of these. Only use it on a brand-new board or after changing
the partition table, and download your flight logs first.

## Partition table (16 MB)

See `ARCHITECTURE.md` §6. The offsets of `nvs`, `event_log` and `coredump` match the old 4 MB
layout, so settings survived the move to 16 MB.

## Over-the-air updates

Not supported. The board has a single factory app slot, and the 2 MB freed this way is used for
the larger app partition. Always flash over USB.

## After flashing

1. The RGB LED plays the boot sequence and ends with 2× green (or 2× amber if optional hardware is
   missing).
2. In the dock, the Health page shows the uptime counting, IMU ~100 Hz and free heap ~130 KB.
3. `CMD,001,LOG,LIST` in the Console lists the recorder sessions, which shows the flight log
   survived.
