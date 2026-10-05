"""
AAKASHVANI — CanSat 3D Telemetry and WebSocket Server
=====================================================
Provides HTTP web server and real-time WebSocket streaming
for the 3D CanSat body attitude visualizer.
"""

import os
import sys
import time
import json
import asyncio
import threading
import http.server
import socketserver
from urllib.parse import urlparse

# Default Ports
HTTP_PORT = 8055
WS_PORT = 8765

# Base directories
TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
WEB_DIR = os.path.join(TOOLS_DIR, "web")
ASSETS_DIR = os.path.join(TOOLS_DIR, "assets")
FALLBACK_CAD_DIR = r"C:\Users\Lenovo\Downloads\Final_Body_Cansat"

# Global state
_telemetry_lock = threading.Lock()
_latest_telemetry = {
    "pitch": 0.0,
    "roll": 0.0,
    "yaw": 0.0,
    "alt": 0.0,
    "pres": 101325.0,
    "temp": 25.0,
    "volt": 7.4,
    "state": 0,
    "state_name": "IDLE",
    "pcount": 0,
    "timestamp": time.time()
}

_connected_ws_clients = set()
_ws_loop = None
_http_server = None
_server_threads = []
_is_running = False


class CanSatHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEB_DIR, **kwargs)

    def log_message(self, format, *args):
        # Suppress routine GET logging to keep terminal clean
        pass

    def do_GET(self):
        try:
            parsed = urlparse(self.path)
            path = parsed.path

            if path in ("", "/", "/index.html"):
                viewer_file = os.path.join(WEB_DIR, "cansat_imu_viewer.html")
                if not os.path.exists(viewer_file):
                    viewer_file = os.path.join(FALLBACK_CAD_DIR, "cansat_imu_viewer.html")
                if not os.path.exists(viewer_file):
                    viewer_file = os.path.join(FALLBACK_CAD_DIR, "cansat_viewer.html")

                if os.path.exists(viewer_file):
                    try:
                        self.send_response(200)
                        self.send_header("Content-Type", "text/html; charset=utf-8")
                        self.send_header("Access-Control-Allow-Origin", "*")
                        self.send_header("Connection", "close")
                        self.send_header("Content-Length", str(os.path.getsize(viewer_file)))
                        self.end_headers()
                        with open(viewer_file, "rb") as f:
                            while chunk := f.read(65536):
                                self.wfile.write(chunk)
                    except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError):
                        pass
                    return

            elif path == "/telemetry":
                try:
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.send_header("Connection", "close")
                    with _telemetry_lock:
                        data = json.dumps(_latest_telemetry).encode("utf-8")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError):
                    pass
                return

            elif path.endswith(".glb") or "cansat_assembly.glb" in path:
                glb_path = os.path.join(WEB_DIR, "cansat_assembly.glb")
                if not os.path.exists(glb_path):
                    glb_path = os.path.join(ASSETS_DIR, "cansat_assembly.glb")
                if not os.path.exists(glb_path):
                    glb_path = os.path.join(FALLBACK_CAD_DIR, "cansat_assembly.glb")

                if os.path.exists(glb_path):
                    try:
                        self.send_response(200)
                        self.send_header("Content-Type", "model/gltf-binary")
                        self.send_header("Access-Control-Allow-Origin", "*")
                        self.send_header("Connection", "close")
                        self.send_header("Content-Length", str(os.path.getsize(glb_path)))
                        self.end_headers()
                        with open(glb_path, "rb") as f:
                            while chunk := f.read(65536):
                                self.wfile.write(chunk)
                    except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError):
                        pass
                    return

            # Default handler
            try:
                super().do_GET()
            except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError):
                pass
        except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError):
            pass

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == "/telemetry":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length)
            try:
                data = json.loads(body.decode("utf-8"))
                update_telemetry(
                    pitch=data.get("pitch", 0.0),
                    roll=data.get("roll", 0.0),
                    yaw=data.get("yaw", 0.0),
                    alt=data.get("alt", 0.0),
                    pres=data.get("pres", 101325.0),
                    temp=data.get("temp", 25.0),
                    volt=data.get("volt", 7.4),
                    state=data.get("state", 0),
                    state_name=data.get("state_name", "IDLE"),
                    pcount=data.get("pcount", 0)
                )
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Connection", "close")
                self.end_headers()
                self.wfile.write(b'{"status":"ok"}')
                return
            except Exception as e:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(str(e).encode("utf-8"))
                return
        self.send_response(404)
        self.end_headers()


class ReusableTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = False
    daemon_threads = True


def _run_http_server(port=HTTP_PORT):
    global _http_server
    try:
        handler = CanSatHTTPRequestHandler
        _http_server = ReusableTCPServer(("0.0.0.0", port), handler)
        print(f"[3D Server] HTTP Web Server running at http://localhost:{port}/", flush=True)
        _http_server.serve_forever()
    except Exception as e:
        print(f"[3D Server] HTTP Server error on port {port}: {e}", flush=True)


async def _ws_handler(websocket):
    global _connected_ws_clients
    _connected_ws_clients.add(websocket)
    try:
        # Send initial snapshot immediately on connect
        with _telemetry_lock:
            init_msg = json.dumps(_latest_telemetry)
        await websocket.send(init_msg)

        async for message in websocket:
            try:
                data = json.loads(message)
                if "pitch" in data or "roll" in data or "yaw" in data:
                    update_telemetry(
                        pitch=data.get("pitch", _latest_telemetry["pitch"]),
                        roll=data.get("roll", _latest_telemetry["roll"]),
                        yaw=data.get("yaw", _latest_telemetry["yaw"]),
                        alt=data.get("alt", _latest_telemetry["alt"]),
                        state=data.get("state", _latest_telemetry["state"])
                    )
            except Exception:
                pass
    except Exception:
        pass
    finally:
        _connected_ws_clients.discard(websocket)


def _run_ws_server(port=WS_PORT):
    global _ws_loop

    async def _main():
        global _ws_loop
        _ws_loop = asyncio.get_running_loop()
        import websockets
        async with websockets.serve(_ws_handler, "0.0.0.0", port):
            print(f"[3D Server] WebSocket Telemetry Stream running on ws://localhost:{port}/", flush=True)
            await asyncio.get_running_loop().create_future()

    try:
        asyncio.run(_main())
    except ImportError:
        print("[3D Server] Note: 'websockets' not installed; 3D viewer will use HTTP polling.", flush=True)
    except Exception as e:
        print(f"[3D Server] WebSocket Server on port {port}: {e}", flush=True)


def update_telemetry(pitch=None, roll=None, yaw=None, alt=None, pres=None, temp=None, volt=None,
                     state=None, state_name=None, pcount=None, q=None, t_mcu=None, mount_name=None):
    """
    Partial update: only the fields that are given are changed and broadcast.

    q      : vehicle quaternion [w, x, y, z] from the firmware ATT line
    t_mcu  : MCU clock (ms) at the IMU sample instant; the viewer interpolates on this
             time base, so websocket/serial jitter never distorts the motion.
    Attitude (fast, ~50 Hz) and frame data (25 Hz) therefore never overwrite each other.
    """
    global _latest_telemetry
    now = time.time()
    packet = {"timestamp": now}
    if pitch is not None: packet["pitch"] = round(float(pitch), 2)
    if roll is not None: packet["roll"] = round(float(roll), 2)
    if yaw is not None: packet["yaw"] = round(float(yaw), 2)
    if alt is not None: packet["alt"] = round(float(alt), 2)
    if pres is not None: packet["pres"] = round(float(pres), 1)
    if temp is not None: packet["temp"] = round(float(temp), 1)
    if volt is not None: packet["volt"] = round(float(volt), 2)
    if state is not None: packet["state"] = int(state)
    if state_name is not None: packet["state_name"] = str(state_name)
    if pcount is not None: packet["pcount"] = int(pcount)
    if q is not None: packet["q"] = [round(float(c), 6) for c in q]
    if t_mcu is not None: packet["t_mcu"] = int(t_mcu)
    if mount_name is not None: packet["mount_name"] = str(mount_name)

    with _telemetry_lock:
        _latest_telemetry.update(packet)
        if q is None and pitch is not None:
            # Euler-only source took over: stale quaternion must not override it
            _latest_telemetry.pop("q", None)
            _latest_telemetry.pop("t_mcu", None)

    # Broadcast via WebSocket if clients connected
    if _ws_loop and _connected_ws_clients:
        msg = json.dumps(packet)
        async def _broadcast():
            dead = []
            for ws in list(_connected_ws_clients):
                try:
                    await ws.send(msg)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                _connected_ws_clients.discard(ws)
        try:
            asyncio.run_coroutine_threadsafe(_broadcast(), _ws_loop)
        except Exception:
            pass

    last_disk = getattr(update_telemetry, "_last_disk", 0.0)
    if now - last_disk >= 0.5:
        update_telemetry._last_disk = now
        try:
            cache_path = os.path.join(TOOLS_DIR, "latest_telemetry.json")
            with _telemetry_lock:
                snapshot = dict(_latest_telemetry)
            with open(cache_path, "w", encoding="utf-8") as f:
                json.dump(snapshot, f)
        except Exception:
            pass


def is_port_in_use(port):
    import socket
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.3)
            return s.connect_ex(('127.0.0.1', port)) == 0
    except Exception:
        return False


def get_telemetry():
    with _telemetry_lock:
        return dict(_latest_telemetry)


def start_server(http_port=HTTP_PORT, ws_port=WS_PORT):
    global _is_running, _server_threads
    if _is_running:
        return

    if is_port_in_use(http_port):
        print(f"[3D Server] Port {http_port} already in use; attaching to existing server instance.")
        _is_running = True
        return

    _is_running = True
    os.makedirs(WEB_DIR, exist_ok=True)
    os.makedirs(ASSETS_DIR, exist_ok=True)

    t_http = threading.Thread(target=_run_http_server, args=(http_port,), daemon=True, name="3D-HTTP-Server")
    t_ws = threading.Thread(target=_run_ws_server, args=(ws_port,), daemon=True, name="3D-WS-Server")

    _server_threads = [t_http, t_ws]
    t_http.start()
    t_ws.start()
    time.sleep(0.3)


def stop_server():
    global _is_running, _http_server, _ws_loop
    _is_running = False
    if _http_server:
        try:
            _http_server.shutdown()
            _http_server.server_close()
        except Exception:
            pass
