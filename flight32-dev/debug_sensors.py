import serial
import time
import struct
import sys

PORT = "COM13"
BAUD = 115200

def read_msp_debug(ser, cmd_id):
    pkt = b"$M<" + struct.pack("<BB", 0, cmd_id)
    chk = 0 ^ cmd_id
    pkt += struct.pack("<B", chk)
    ser.reset_input_buffer()
    ser.write(pkt)
    
    start = time.time()
    buf = b""
    while time.time() - start < 0.5:
        if ser.in_waiting > 0:
            buf += ser.read(ser.in_waiting)
        time.sleep(0.01)
    return buf

def print_boot_logs(ser):
    print("\n--- Resetting ESP32 into RUN mode to capture boot logs ---")
    ser.dtr = False
    ser.rts = True  # Assert Reset (EN low)
    time.sleep(0.1)
    ser.rts = False # Release Reset (EN high) while GPIO0 is high
    ser.dtr = False
    time.sleep(3.0)
    logs = ser.read_all()
    print("=== ESP32 BOOT LOGS ===")
    print(logs.decode('utf-8', errors='replace'))
    print("=======================\n")

def run_terminal_cmd(ser, cmd):
    print(f"\n--- Running terminal command: '{cmd}' ---")
    ser.write((cmd + "\r\n").encode('utf-8'))
    start = time.time()
    res = b""
    while time.time() - start < 1.0:
        if ser.in_waiting > 0:
            res += ser.read(ser.in_waiting)
        time.sleep(0.01)
    print(res.decode('utf-8', errors='replace'))

def main():
    print(f"--- SENSOR RAW DEBUG SCRIPT: Connecting to {PORT} ---")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"Error opening {PORT}: {e}")
        return

    print_boot_logs(ser)
    ser.reset_input_buffer()
    run_terminal_cmd(ser, "i2c_scan")
    run_terminal_cmd(ser, "status")
    run_terminal_cmd(ser, "get imu.data")

    for name, cid, exp_len in [
        ("MSP_ATTITUDE", 108, 6),
        ("MSP_STATUS_EX", 150, 14),
        ("MSP_RAW_IMU", 106, 18),
        ("MSP_FLIGHT32_ENV", 240, 12),
        ("MSP_ALTITUDE", 109, 6),
    ]:
        buf = read_msp_debug(ser, cid)
        print(f"\n[{name} (ID={cid})] Raw response len={len(buf)} bytes:")
        print(f"  HEX  : {buf.hex()}")
        print(f"  ASCII: {repr(buf)}")
        if len(buf) >= 6 and buf[:3] in (b"$M>", b"$M!"):
            size = buf[3]
            data = buf[5:5+size]
            print(f"  Parsed Payload ({size} bytes): {data.hex()}")
            if cid == 106 and len(data) >= 18:
                ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack("<9h", data[:18])
                print(f"  -> ACC: ({ax}, {ay}, {az}) | GYRO: ({gx}, {gy}, {gz}) | MAG: ({mx}, {my}, {mz})")
            elif cid == 240 and len(data) >= 12:
                temp_c, pressure_hpa, alt_m = struct.unpack("<3f", data[:12])
                print(f"  -> TEMP: {temp_c:.2f} C | PRESSURE: {pressure_hpa:.2f} hPa | ALT: {alt_m:.2f} m")

    ser.close()
    print("\n--- RAW DEBUG COMPLETED ---")

if __name__ == "__main__":
    main()
