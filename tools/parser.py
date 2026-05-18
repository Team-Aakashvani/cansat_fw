#!/usr/bin/env python3
"""
CAN-7USAT 2026 — Ground Station Serial → WebSocket Parser
==========================================================

FILE:  gcs/parser.py

WHAT THIS SCRIPT DOES (plain English):
--------------------------------------
1. Plugs into the GCS bridge box via a USB cable (serial port).
2. Reads lines of text coming from the CanSat (via LoRa radio → GCS bridge → USB).
3. Each line has 21 comma-separated values (altitude, temperature, GPS, etc.).
4. Checks that each line is valid (correct number of fields, correct types).
5. Converts valid lines into labeled JSON and broadcasts them over a WebSocket
   so the ground station GUI can display the data.
6. Also listens for commands FROM the GUI (like "start recording video")
   and sends them back to the CanSat through the same USB cable.

HOW TO RUN:
-----------
    pip install pyserial websockets
    python parser.py --port /dev/ttyUSB0

    On Windows, replace /dev/ttyUSB0 with something like COM3.

HOW TO TEST WITHOUT HARDWARE:
-----------------------------
    You can use a virtual serial port or just read the code to understand
    the logic. The script will print errors if no device is connected.

ARCHITECTURE (how the pieces fit together):
-------------------------------------------
    ┌─────────────┐    USB cable     ┌──────────────┐   WebSocket    ┌─────────┐
    │  CanSat     │ ──── LoRa ────▶ │  GCS Bridge  │ ──── USB ────▶ │  THIS   │ ──────▶ │  GUI    │
    │  (flying)   │                  │  (small box) │                │  SCRIPT │ ◀────── │  (app)  │
    └─────────────┘                  └──────────────┘                └─────────┘         └─────────┘
         ▲                                                               │
         └───────────────── uplink commands ($UPLINK,...) ◀──────────────┘
"""

# =============================================================================
# IMPORTS — these are the libraries (tools) we need
# =============================================================================

import asyncio          # Lets us do multiple things at once (read serial + serve WebSocket)
import argparse         # Lets the user pass options like --port COM3 when running the script
import json             # Converts Python dictionaries to JSON text and back
import logging          # Prints helpful debug/error messages with timestamps
import re               # Regular expressions — for checking that uplink commands are safe
import sys              # System-level stuff (like exiting the program)
import threading        # Lets us run the serial-read loop in a separate thread

import serial           # "pyserial" — talks to USB/serial ports
import websockets       # Runs a WebSocket server that the GUI connects to


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
# Each telemetry line from the CanSat looks like:
#   1234,12:30:45,100,350.5,101325.0,25.3,7.4,12:30:45,28.61,77.20,350.0,8,0.1,0.2,15.3,3,433.5,-85,1,3.50,25
#
# That's 21 values separated by commas. Each value has a name and a type.
# "int" means whole number, "float" means decimal number, "str" means text.
#
# This list defines the name and expected type for each position:

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

EXPECTED_FIELD_COUNT = len(FIELDS)  # = 21


# =============================================================================
# PARSING FUNCTION — takes a raw text line and converts it to a dictionary
# =============================================================================

def parse_line(raw_line: str) -> dict | None:
    """
    Takes one raw CSV line like "1234,12:30:45,100,350.5,..."
    and returns a dictionary like {"team_id": 1234, "mission_time": "12:30:45", ...}

    Returns None if the line is malformed (wrong number of fields, wrong types, etc.)
    We silently drop bad lines — the spec says to discard them without error messages.
    """

    # Step 1: Remove whitespace from both ends (spaces, newlines, carriage returns)
    line = raw_line.strip()

    # Step 2: Skip empty lines
    if not line:
        return None

    # Step 3: Split the line by commas into a list of strings
    #   "1234,12:30:45,100" → ["1234", "12:30:45", "100"]
    parts = line.split(",")

    # Step 4: Check that we got exactly 21 fields
    if len(parts) != EXPECTED_FIELD_COUNT:
        return None

    # Step 5: Try to convert each field to its expected type
    result = {}
    for i, (field_name, field_type) in enumerate(FIELDS):
        raw_value = parts[i].strip()
        try:
            if field_type == str:
                # Strings are kept as-is (like "12:30:45")
                result[field_name] = raw_value
            else:
                # Convert to int or float — this will fail if the value is garbage
                result[field_name] = field_type(raw_value)
        except (ValueError, TypeError):
            # If any single field fails to convert, the whole line is bad
            return None

    return result


# =============================================================================
# UPLINK SANITIZER — makes sure commands from the GUI are safe to send
# =============================================================================

# This pattern allows only letters, numbers, and underscores.
# Example: "RECORD_TOGGLE" is OK, but "DROP TABLE;" is NOT.
SAFE_PAYLOAD_PATTERN = re.compile(r"^[A-Za-z0-9_]+$")


def sanitize_uplink(payload: str) -> str | None:
    """
    Checks that an uplink command payload is safe (only letters, numbers, underscores).
    Returns the cleaned payload, or None if it's unsafe.
    """
    payload = payload.strip()
    if SAFE_PAYLOAD_PATTERN.match(payload):
        return payload
    return None


# =============================================================================
# SERIAL READER THREAD — reads lines from the USB cable in the background
# =============================================================================

def serial_reader_thread(
    port: str,
    baud: int,
    rx_queue: asyncio.Queue,
    loop: asyncio.AbstractEventLoop
):
    """
    This function runs in a separate thread (background worker).

    WHY A SEPARATE THREAD?
    Reading from a serial port is "blocking" — the program sits and waits
    until a new line arrives. If we did this in the main thread, the WebSocket
    server would freeze while waiting for serial data. By putting the serial
    read in its own thread, both can run at the same time.

    It reads lines from the serial port and puts them into a queue (a to-do list)
    that the main async loop picks up and processes.
    """
    try:
        ser = serial.Serial(port, baud, timeout=1)
        log.info(f"Serial port opened: {port} at {baud} baud")
    except serial.SerialException as e:
        log.error(f"Cannot open serial port {port}: {e}")
        log.error("Check that the GCS bridge is plugged in and the port name is correct.")
        return

    while True:
        try:
            # Read one line (waits until '\n' is received, or times out after 1 second)
            raw = ser.readline()
            if raw:
                # Decode bytes to text string (the serial port gives us raw bytes)
                line = raw.decode("utf-8", errors="replace")
                # Put the line into the queue for the async loop to process
                # loop.call_soon_threadsafe safely passes data between threads
                loop.call_soon_threadsafe(rx_queue.put_nowait, line)
        except serial.SerialException as e:
            log.error(f"Serial read error: {e}")
            break
        except Exception as e:
            log.error(f"Unexpected error in serial reader: {e}")
            break

    log.warning("Serial reader thread has stopped.")


def serial_write(ser_port: serial.Serial | None, message: str):
    """
    Writes a string to the serial port (for sending uplink commands to the CanSat).
    """
    if ser_port and ser_port.is_open:
        try:
            ser_port.write(message.encode("utf-8"))
            ser_port.flush()
        except serial.SerialException as e:
            log.error(f"Serial write error: {e}")


# =============================================================================
# MAIN ASYNC SERVER — the heart of the program
# =============================================================================

async def main(port: str, baud: int):
    """
    The main function that ties everything together:
    1. Opens the serial port in a background thread
    2. Starts a WebSocket server
    3. Reads parsed telemetry from the queue and broadcasts to all GUI clients
    4. Receives uplink commands from GUI clients and sends them to serial
    """

    # --- Queues ---
    # Think of queues like mailboxes:
    #   rx_queue: serial thread drops incoming lines here → main loop picks them up
    #   tx_queue: WebSocket clients drop uplink commands here → main loop sends to serial
    rx_queue: asyncio.Queue = asyncio.Queue()
    tx_queue: asyncio.Queue = asyncio.Queue()

    # --- Keep track of all connected GUI clients ---
    connected_clients: set = set()

    # --- Open serial port for uplink writes ---
    ser_port = None
    try:
        ser_port = serial.Serial(port, baud, timeout=1)
    except serial.SerialException:
        log.warning(f"Could not open serial port {port} for uplink writes. "
                    "Uplink commands will not work.")

    # --- Start the serial reader in a background thread ---
    loop = asyncio.get_event_loop()
    reader_thread = threading.Thread(
        target=serial_reader_thread,
        args=(port, baud, rx_queue, loop),
        daemon=True  # "daemon" means this thread dies when the main program exits
    )
    reader_thread.start()

    # --- WebSocket handler ---
    # This function is called once for EACH GUI client that connects.
    async def handle_client(websocket):
        """
        Called when a new GUI client connects to ws://127.0.0.1:5555.
        Listens for uplink commands from this client.
        """
        connected_clients.add(websocket)
        client_addr = websocket.remote_address
        log.info(f"GUI client connected: {client_addr}")

        try:
            # Listen for messages from this client (uplink commands)
            async for message in websocket:
                try:
                    data = json.loads(message)

                    # The GUI sends commands like: {"type": "cmd", "payload": "RECORD_TOGGLE"}
                    if (isinstance(data, dict)
                            and data.get("type") == "cmd"
                            and "payload" in data):

                        payload = sanitize_uplink(str(data["payload"]))

                        if payload:
                            # Format it as "$UPLINK,RECORD_TOGGLE\r\n" and queue for sending
                            uplink_msg = f"$UPLINK,{payload}\r\n"
                            await tx_queue.put(uplink_msg)
                            log.info(f"Uplink command queued: {payload}")
                        else:
                            log.warning(f"Rejected unsafe uplink payload: {data.get('payload')}")

                except json.JSONDecodeError:
                    log.warning(f"Received non-JSON message from client: {message[:50]}")

        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            connected_clients.discard(websocket)
            log.info(f"GUI client disconnected: {client_addr}")

    # --- Start the WebSocket server ---
    server = await websockets.serve(handle_client, WS_HOST, WS_PORT)
    log.info(f"WebSocket server running on ws://{WS_HOST}:{WS_PORT}")
    log.info("Waiting for GUI clients to connect...")

    # --- Main processing loop ---
    # This runs forever, doing two things:
    #   1. Drain the rx_queue (incoming telemetry) → parse → broadcast to GUI clients
    #   2. Drain the tx_queue (uplink commands) → write to serial port
    while True:
        # --- Process incoming telemetry ---
        while not rx_queue.empty():
            try:
                raw_line = rx_queue.get_nowait()
                parsed = parse_line(raw_line)

                if parsed is not None:
                    # Convert the dictionary to a JSON string
                    json_msg = json.dumps(parsed)

                    # Send to ALL connected GUI clients
                    if connected_clients:
                        # websockets.broadcast sends to everyone at once
                        websockets.broadcast(connected_clients, json_msg)

            except asyncio.QueueEmpty:
                break

        # --- Process outgoing uplink commands ---
        while not tx_queue.empty():
            try:
                uplink_msg = tx_queue.get_nowait()
                serial_write(ser_port, uplink_msg)
                log.info(f"Sent to serial: {uplink_msg.strip()}")
            except asyncio.QueueEmpty:
                break

        # --- Small sleep to avoid hogging the CPU ---
        # 50ms means we check ~20 times per second — fast enough for 1 Hz telemetry
        await asyncio.sleep(0.05)


# =============================================================================
# ENTRY POINT — what runs when you type "python parser.py"
# =============================================================================

if __name__ == "__main__":
    # --- Parse command-line arguments ---
    # This lets you run: python parser.py --port COM3 --baud 921600
    arg_parser = argparse.ArgumentParser(
        description="CAN-7USAT GCS Parser — Serial to WebSocket bridge"
    )
    arg_parser.add_argument(
        "--port", default=DEFAULT_PORT,
        help=f"Serial port (default: {DEFAULT_PORT}). Example: COM3, /dev/ttyUSB0"
    )
    arg_parser.add_argument(
        "--baud", type=int, default=DEFAULT_BAUD,
        help=f"Baud rate (default: {DEFAULT_BAUD})"
    )
    args = arg_parser.parse_args()

    log.info("=" * 60)
    log.info("CAN-7USAT 2026 — Ground Station Parser")
    log.info(f"Serial port : {args.port}")
    log.info(f"Baud rate   : {args.baud}")
    log.info(f"WebSocket   : ws://{WS_HOST}:{WS_PORT}")
    log.info("=" * 60)

    try:
        asyncio.run(main(args.port, args.baud))
    except KeyboardInterrupt:
        log.info("Parser stopped by user (Ctrl+C).")
        sys.exit(0)
