import sys
import os
import time
import subprocess

import serial.tools.list_ports

def find_port():
    if len(sys.argv) > 1:
        return sys.argv[1]
    ports = list(serial.tools.list_ports.comports())
    # Priority: CH343, CP210x, USB-Serial, ESP32
    for p in ports:
        desc = (p.description or "").lower()
        if "ch34" in desc or "cp210" in desc or "usb-serial" in desc or "jtag" in desc or "esp" in desc:
            return p.device
    for p in ports:
        if "bluetooth" not in (p.description or "").lower():
            return p.device
    return "COM14"

PORT = find_port()
BAUD = '460800'

def main():
    workspace = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    build_dir = os.path.join(workspace, "build")
    bootloader = os.path.join(build_dir, "bootloader", "bootloader.bin")
    app = os.path.join(build_dir, "cansat_fw.bin")
    partitions = os.path.join(build_dir, "partition_table", "partition-table.bin")
    ota_data = os.path.join(build_dir, "ota_data_initial.bin")

    for f in [bootloader, app, partitions, ota_data]:
        if not os.path.exists(f):
            print(f"[!] Missing binary: {f}")
            return 1

    idf_py = r"C:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe"
    py_exec = idf_py if os.path.exists(idf_py) else sys.executable

    cmd = [
        py_exec, "-m", "esptool",
        "--chip", "esp32s3",
        "--port", PORT,
        "--baud", BAUD,
        "--before", "default_reset",
        "--after", "hard_reset",
        "write_flash",
        "--flash_mode", "dio",
        "--flash_freq", "80m",
        "--flash_size", "16MB",
        "0x0", bootloader,
        "0x8000", partitions,
        "0xf000", ota_data,
        "0x20000", app
    ]

    print("=" * 70)
    print(f" FLASHING CANSAT FIRMWARE TO {PORT} @ {BAUD} BAUD")
    print(" >>> IF 'Connecting...' APPEARS, PRESS AND HOLD THE 'BOOT' BUTTON <<<")
    print("=" * 70)

    bauds = ['460800', '115200', '460800', '115200']
    for attempt, baud in enumerate(bauds, 1):
        print(f"\n[Attempt {attempt}/{len(bauds)}] Connecting to ESP32 on {PORT} @ {baud} baud...")
        print(" >>> HOLD DOWN THE 'BOOT' (IO0) BUTTON NOW <<<")
        current_cmd = list(cmd)
        current_cmd[current_cmd.index(BAUD)] = baud
        res = subprocess.run(current_cmd)
        if res.returncode == 0:
            print("\n" + "=" * 70)
            print(" [OK] SUCCESS: CANSAT FIRMWARE FLASHED & BOOTED!")
            print("=" * 70)
            return 0
        print(f"[!] Attempt {attempt} failed. Retrying in 2 seconds...")
        time.sleep(2)

    print("\n[X] Could not connect. Please check:")
    print("  1. Is the board power switch ON?")
    print("  2. Hold down the 'BOOT' (IO0) button and press 'EN' (RST) once, then retry.")
    return 1

if __name__ == '__main__':
    sys.exit(main())
