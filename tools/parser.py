#!/usr/bin/env python3
"""
CAN-7USAT 2026 — Ground Station Serial → WebSocket Parser
==========================================================

FILE:  tools/parser.py

WHAT THIS SCRIPT DOES (plain English):
--------------------------------------
1. Plugs into the GCS bridge box via a USB cable (serial port).
2. Reads lines of text coming from the CanSat (via LoRa radio → GCS bridge → USB).
3. Each line has 16 comma-separated values (altitude, temperature, GPS, etc.).
4. Checks that each line is valid (correct number of fields, correct types).
5. Converts valid lines into labeled JSON and broadcasts them over a WebSocket
   so the ground station GUI can display the data.
6. Also listens for commands FROM the GUI (like "CX,ON")
   and sends them back to the CanSat through the same USB cable.

HOW TO RUN:
-----------
    pip install pyserial websockets
    python parser.py --port /dev/ttyUSB0

    On Windows, replace /dev/ttyUSB0 with something like COM3.
"""

import asyncio
import argparse
import json
import logging
import re
import sys
import threading
import serial
import websockets

# =============================================================================
# CONFIGURATION — things you might want to change
# =============================================================================

DEFAULT_PORT = "/dev/ttyUSB0"   # Default serial port (Linux). Windows would be "COM3" etc.
DEFAULT_BAUD = 921600           # Speed of the serial connection (must match the GCS bridge)
WS_HOST = "127.0.0.1"          # WebSocket listens on localhost only (same machine)
WS_PORT = 5555                  # WebSocket port number — GUI connects to ws://127.0.0.1:5555

# Set up logging so we can see what's happening
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
log = logging.getLogger("gcs_parser")


# =============================================================================
# FIELD DEFINITIONS — the 21 data fields in every telemetry line
# =============================================================================
#
# Following CAN-7USAT 2026 §5.3 (Telemetry Data Format) + Extended Mission
#
# Each telemetry line from the CanSat looks like:
#   1234,00:04:32,272,587.34,94312.5,18.3,7.41,07:15:44,12.971600,77.594600,912.30,8,0.12,-0.04,1.23,3,433.5,-85,1,3.5,25
#

FIELDS = [
    ("team_id",         int),     #  0 — Team identification number
    ("mission_time",    str),     #  1 — Clock time "HH:MM:SS"
    ("packet_count",    int),     #  2 — How many packets sent so far
    ("altitude_m",      float),   #  3 — Height above ground in meters
    ("pressure_pa",     float),   #  4 — Air pressure in Pascals
    ("temperature_c",   float),   #  5 — Temperature in Celsius
    ("voltage_v",       float),   #  6 — Battery voltage
    ("gnss_time",       str),     #  7 — GPS clock time
    ("latitude_deg",    float),   #  8 — GPS latitude (north/south position)
    ("longitude_deg",   float),   #  9 — GPS longitude (east/west position)
    ("gnss_alt_m",      float),   # 10 — GPS altitude
    ("satellites",      int),     # 11 — Number of GPS satellites visible
    ("tilt_x",          float),   # 12 — Tilt angle X axis
    ("tilt_y",          float),   # 13 — Tilt angle Y axis
    ("rot_z",           float),   # 14 — Rotation around Z axis (spin rate)
    ("software_state",  int),     # 15 — Which flight phase (0=preflight, 3=parachute, etc.)
    ("cc1101_freq_mhz", float),   # 16 — RF scanner current frequency in MHz
    ("cc1101_rssi_dbm", int),     # 17 — RF scanner signal strength (negative = weaker)
    ("p4_recording",    int),     # 18 — Is the camera recording? (0=no, 1=yes)
    ("p4_sd_gb",        float),   # 19 — Free space on camera SD card in GB
    ("p4_fps",          int),     # 20 — Camera frames per second
]

EXPECTED_FIELD_COUNT = len(FIELDS)


# =============================================================================
# PARSING FUNCTION — takes a raw text line and converts it to a dictionary
# =============================================================================

def parse_line(raw_line: str) -> dict | None:
    """
    Takes one raw CSV line and returns a dictionary.
    Handles potential extra fields (like double Team ID or CRC) by finding
    the sequence of fields that best matches our spec.
    """
    line = raw_line.strip()
    if not line:
        return None

    # Split and clean parts
    parts = [p.strip() for p in line.split(",")]

    # Basic heuristic: if we have more parts than expected, try to find a sub-sequence
    # that looks like our data.
    
    if len(parts) > EXPECTED_FIELD_COUNT:
        # Check for double Team ID (common firmware bug where it's prepended twice)
        if parts[0] == parts[1] and parts[0].isdigit():
            parts = parts[1:]
        
        # Strip trailing CRC if present (usually 4 hex chars at the very end)
        if len(parts) > EXPECTED_FIELD_COUNT:
            if re.match(r"^[0-9A-F]{4}$", parts[-1]):
                parts = parts[:-1]
    
    if len(parts) < EXPECTED_FIELD_COUNT:
        return None
    
    # Trim to exactly what we expect if still too long
    parts = parts[:EXPECTED_FIELD_COUNT]

    result = {}
    for i, (field_name, field_type) in enumerate(FIELDS):
        try:
            if field_type == str:
                result[field_name] = parts[i]
            else:
                result[field_name] = field_type(parts[i])
        except (ValueError, TypeError, IndexError):
            return None

    return result


# =============================================================================
# UPLINK SANITIZER — makes sure commands from the GUI are safe to send
# =============================================================================

# Allows alphanumeric, underscore, comma, colon, and period (for float args)
SAFE_PAYLOAD_PATTERN = re.compile(r"^[A-Za-z0-9_,\.:\-]+$")


def sanitize_uplink(payload: str) -> str | None:
    """
    Checks that an uplink command payload is safe.
    """
    payload = payload.strip().upper()
    if SAFE_PAYLOAD_PATTERN.match(payload):
        return payload
    return None


# =============================================================================
# SERIAL READER THREAD — reads lines from the USB cable in the background
# =============================================================================

def serial_reader_thread(
    ser: serial.Serial,
    rx_queue: asyncio.Queue,
    loop: asyncio.AbstractEventLoop
):
    """
    Reads lines from the serial port and puts them into the rx_queue.
    """
    log.info("Serial reader thread started.")
    while ser.is_open:
        try:
            raw = ser.readline()
            if raw:
                line = raw.decode("utf-8", errors="replace")
                loop.call_soon_threadsafe(rx_queue.put_nowait, line)
        except serial.SerialException as e:
            log.error(f"Serial read error: {e}")
            break
        except Exception as e:
            log.error(f"Unexpected error in serial reader: {e}")
            break

    log.warning("Serial reader thread has stopped.")


# =============================================================================
# MAIN ASYNC SERVER — the heart of the program
# =============================================================================

async def main(port: str, baud: int):
    """
    Main async loop.
    """
    rx_queue = asyncio.Queue()
    tx_queue = asyncio.Queue()
    connected_clients = set()

    # --- Open serial port ONCE ---
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        log.info(f"Serial port opened: {port} at {baud} baud")
    except serial.SerialException as e:
        log.critical(f"Cannot open serial port {port}: {e}")
        return

    # --- Start reader thread ---
    loop = asyncio.get_event_loop()
    reader_thread = threading.Thread(
        target=serial_reader_thread,
        args=(ser, rx_queue, loop),
        daemon=True
    )
    reader_thread.start()

    async def handle_client(websocket):
        connected_clients.add(websocket)
        log.info(f"GUI client connected: {websocket.remote_address}")
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    if isinstance(data, dict) and data.get("type") == "cmd":
                        payload = sanitize_uplink(str(data.get("payload", "")))
                        if payload:
                            # GCS bridge expects "$UPLINK,<CMD>\n"
                            await tx_queue.put(f"$UPLINK,{payload}\r\n")
                            log.info(f"Uplink queued: {payload}")
                        else:
                            log.warning(f"Rejected unsafe uplink payload: {data.get('payload')}")
                except json.JSONDecodeError:
                    log.warning("Received non-JSON message from client")
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            connected_clients.discard(websocket)
            log.info("GUI client disconnected")

    # Start WebSocket server
    await websockets.serve(handle_client, WS_HOST, WS_PORT)
    log.info(f"WebSocket server on ws://{WS_HOST}:{WS_PORT}")

    # --- Background Tasks ---
    async def process_telemetry():
        while True:
            raw_line = await rx_queue.get()
            parsed = parse_line(raw_line)
            if parsed:
                msg = json.dumps(parsed)
                if connected_clients:
                    websockets.broadcast(connected_clients, msg)
            rx_queue.task_done()

    async def process_uplink():
        while True:
            msg = await tx_queue.get()
            try:
                ser.write(msg.encode("utf-8"))
                ser.flush()
                log.info(f"Serial TX: {msg.strip()}")
            except serial.SerialException as e:
                log.error(f"Serial write error: {e}")
            tx_queue.task_done()

    # Run everything concurrently
    log.info("Parser engine live. Press Ctrl+C to stop.")
    try:
        await asyncio.gather(
            process_telemetry(),
            process_uplink()
        )
    except asyncio.CancelledError:
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CAN-7USAT GCS Parser")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate")
    args = parser.parse_args()

    try:
        asyncio.run(main(args.port, args.baud))
    except KeyboardInterrupt:
        log.info("Stopped by user.")
        sys.exit(0)
