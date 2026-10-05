"""
Aakashvani Ground Station
=========================

Ground software for the CAN-7USAT flight computer.

  Links      USB (native USB-Serial-JTAG) or Bluetooth LE (pad / recovery health link)
  Overview   mission phase, altitude, return-to-launch status, events
  Attitude   live orientation, 3D viewer, re-reference / north calibration
  Health     sensors, power, flight-log storage, self-test, crash dumps
  Flight log download / erase the on-board 12 MB flight recorder
  Bench      ground calibration, lift test, simulated flight, actuator tests
  Console    raw line console
  Firmware   flash builds from build/ with esptool

Fonts: IBM Plex Sans / Plex Mono (SIL OFL), icons: Lucide (ISC) - see tools/assets.
"""

import csv
import ctypes
import json
import math
import os
import queue
import re
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime

import serial
import serial.tools.list_ports
import customtkinter as ctk
import tkinter as tk
from tkinter import filedialog, messagebox
from PIL import Image, ImageDraw, ImageFont

try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    HAVE_MPL = True
except Exception:
    HAVE_MPL = False

try:
    import telemetry_3d_server
except Exception:
    telemetry_3d_server = None

TOOLS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TOOLS)
ASSETS = os.path.join(TOOLS, "assets")
SETTINGS_FILE = os.path.join(os.path.expanduser("~"), ".aakashvani_dock.json")
TEAM = "001"
TEAM_ID = "2026-IN-SPACeCAN-7USAT-001"          # TEAM_ID field of the competition frame
FRAME_HEADER = ("TEAM_ID,TIME_STAMPING_S,PACKET_COUNT,ALTITUDE_M,PRESSURE_PA,TEMP_C,VOLTAGE_V,"
                "GNSS_TIME_S,GNSS_LATITUDE,GNSS_LONGITUDE,GNSS_ALTITUDE_M,GNSS_SATS,"
                "ACC_X_MPS2,ACC_Y_MPS2,ACC_Z_MPS2,ROLL_DEG,PITCH_DEG,GYRO_SPIN_RATE_DPS,"
                "FLIGHT_SOFTWARE_STATE,HEADING_DEG,HUMIDITY_PCT,VOC_INDEX,NOX_INDEX")
DEFAULT_TM_DIR = os.path.join(os.path.expanduser("~"), "Documents", "Aakashvani")

# =============================================================================
# Look & feel
# =============================================================================
C = {
    "bg":      "#0E1116",
    "panel":   "#151920",
    "raised":  "#1B2028",
    "hover":   "#232933",
    "line":    "#262C35",
    "text":    "#E4E7EB",
    "muted":   "#8B95A3",
    "faint":   "#5A6370",
    "accent":  "#E9A23B",     # one accent: amber, used sparingly
    "ok":      "#3FB27F",
    "warn":    "#E9A23B",
    "bad":     "#E5484D",
    "info":    "#5B9DF0",
}

SANS, SANS_MED, SANS_SEMI, MONO, MONO_MED = "Segoe UI", "Segoe UI", "Segoe UI Semibold", "Consolas", "Consolas"


def load_fonts():
    """Register the bundled fonts for this process only (no system install)."""
    global SANS, SANS_MED, SANS_SEMI, MONO, MONO_MED
    if os.name != "nt":
        return
    try:
        for sub in ("fonts", "icons"):
            d = os.path.join(ASSETS, sub)
            for f in os.listdir(d):
                if f.endswith(".ttf"):
                    ctypes.windll.gdi32.AddFontResourceExW(os.path.join(d, f), 0x10, 0)
        SANS, SANS_MED, SANS_SEMI = "IBM Plex Sans", "IBM Plex Sans Medm", "IBM Plex Sans SmBld"
        MONO, MONO_MED = "IBM Plex Mono", "IBM Plex Mono Medium"
    except Exception:
        pass
    if HAVE_MPL:                                   # matplotlib has its own font registry
        try:
            from matplotlib import font_manager
            for f in os.listdir(os.path.join(ASSETS, "fonts")):
                font_manager.fontManager.addfont(os.path.join(ASSETS, "fonts", f))
            matplotlib.rcParams["font.family"] = "IBM Plex Sans"
        except Exception:
            pass


def font(size=12, kind="sans"):
    family = {"sans": SANS, "med": SANS_MED, "semi": SANS_SEMI, "mono": MONO, "monomed": MONO_MED}[kind]
    return ctk.CTkFont(family=family, size=size)


# Lucide codepoints (tools/assets/icons/info.json)
ICONS = {
    "overview": 0xe0f8, "attitude": 0xe2ea, "health": 0xe372, "log": 0xe0b1, "bench": 0xe158,
    "console": 0xe185, "firmware": 0xe0ad, "usb": 0xe35a, "ble": 0xe060, "ble_on": 0xe1b8,
    "plug": 0xe383, "refresh": 0xe149, "download": 0xe0b6, "upload": 0xe19e, "trash": 0xe18e,
    "check": 0xe226, "alert": 0xe193, "x": 0xe088, "play": 0xe140, "stop": 0xe16b, "send": 0xe156,
    "compass": 0xe09f, "crosshair": 0xe0b0, "box": 0xe065, "mountain": 0xe231, "rocket": 0xe286,
    "satellite": 0xe44c, "battery": 0xe057, "thermo": 0xe186, "cpu": 0xe0ad, "drive": 0xe0f0,
    "activity": 0xe038, "gauge": 0xe1bf, "lock_open": 0xe110, "zap": 0xe1b4, "wind": 0xe1b0,
    "nav": 0xe127, "list": 0xe10c, "radio": 0xe146, "flag": 0xe0d6, "layers": 0xe104,
}
_icon_cache = {}


def icon(name, size=16, color=None):
    color = color or C["muted"]
    key = (name, size, color)
    if key not in _icon_cache:
        path = os.path.join(ASSETS, "icons", "lucide.ttf")
        img = Image.new("RGBA", (size * 2, size * 2), (0, 0, 0, 0))
        try:
            f = ImageFont.truetype(path, int(size * 1.7))
            d = ImageDraw.Draw(img)
            ch = chr(ICONS[name])
            b = d.textbbox((0, 0), ch, font=f)
            d.text(((size * 2 - (b[2] - b[0])) / 2 - b[0], (size * 2 - (b[3] - b[1])) / 2 - b[1]), ch, font=f, fill=color)
        except Exception:
            pass
        _icon_cache[key] = ctk.CTkImage(light_image=img, dark_image=img, size=(size, size))
    return _icon_cache[key]


# =============================================================================
# Protocol
# =============================================================================
PHASES = ["PAD", "ASCENT", "DESCENT", "ARMS_DEPLOY", "STEERING", "LANDED"]
PHASE_LABEL = {"PAD": "Pad", "ASCENT": "Ascent", "DESCENT": "Descent", "ARMS_DEPLOY": "Arms out",
               "STEERING": "Steering", "LANDED": "Landed"}
STATE_CODE = {2: "PAD", 3: "ASCENT", 4: "DESCENT", 5: "ARMS_DEPLOY", 6: "STEERING", 7: "LANDED"}
ESC_STATE = {0: "silent", 1: "arming", 2: "spin-up", 3: "running", 4: "stopped"}
MOUNTS = {0: "not referenced", 1: "parallel, Z up", 2: "parallel, Z down", 3: "perpendicular, X up",
          4: "perpendicular, X down", 5: "perpendicular, Y up", 6: "perpendicular, Y down"}
BIT_NAMES = {0: "IMU absent", 1: "baro absent", 2: "power monitor absent", 3: "GNSS silent",
             4: "radio absent", 5: "SD card", 6: "NVS", 7: "IMU implausible", 8: "baro implausible",
             9: "battery low"}
BLE_BLOCKED = ("MTR", "MOTOR", "PID", "CHUTE", "OTA", "LOG,DUMP", "LOG,ERASE")


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def euler_zxy(q):
    w, x, y, z = q
    m = max(-1.0, min(1.0, 2.0 * (y * z + w * x)))
    p = math.degrees(math.asin(m))
    if abs(m) < 0.999999999:
        r = math.degrees(math.atan2(-2.0 * (x * z - w * y), 1.0 - 2.0 * (x * x + y * y)))
        yw = math.degrees(math.atan2(-2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z)))
    else:
        r = 0.0
        yw = math.degrees(math.atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z)))
    yw %= 360.0
    return p, r, (0.0 if yw >= 359.995 else yw)


# =============================================================================
# Links
# =============================================================================
class SerialLink:
    kind = "usb"

    def __init__(self, port, on_line, on_status):
        self.port, self.on_line, self.on_status = port, on_line, on_status
        self.ser = None
        self._stop = threading.Event()

    def start(self):
        s = serial.Serial()
        s.port, s.baudrate, s.timeout, s.write_timeout = self.port, 115200, 0.1, 1.0
        s.dtr = False
        s.rts = False          # never reset the board when connecting
        s.open()
        self.ser = s
        threading.Thread(target=self._run, daemon=True).start()
        self.on_status("connected", f"USB {self.port}")

    def _run(self):
        buf = b""
        while not self._stop.is_set():
            try:
                buf += self.ser.read(4096)
            except Exception:
                self.on_status("lost", "USB disconnected")
                return
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.on_line(line.decode("utf-8", "replace").strip())

    def send(self, text):
        try:
            self.ser.write((text.strip() + "\r\n").encode())
            return True
        except Exception:
            return False

    def stop(self):
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass


class BleLink:
    kind = "ble"
    RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
    TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

    def __init__(self, on_line, on_status, prefix="AAKASHVANI"):
        self.on_line, self.on_status, self.prefix = on_line, on_status, prefix
        self.loop = None
        self.client = None
        self._stop = threading.Event()
        self._buf = bytearray()

    def start(self):
        threading.Thread(target=self._thread, daemon=True).start()

    def _thread(self):
        import asyncio
        self.loop = asyncio.new_event_loop()
        try:
            self.loop.run_until_complete(self._main())
        except Exception as e:
            self.on_status("lost", f"Bluetooth: {e}")

    async def _main(self):
        import asyncio
        from bleak import BleakScanner, BleakClient
        self.on_status("busy", "Searching for the CanSat...")
        dev = await BleakScanner.find_device_by_filter(
            lambda d, ad: (d.name or ad.local_name or "").startswith(self.prefix), timeout=12)
        if dev is None:
            self.on_status("lost", "No CanSat found over Bluetooth")
            return
        self.on_status("busy", f"Connecting to {dev.name}...")
        async with BleakClient(dev, disconnected_callback=lambda _: self.on_status("lost", "Bluetooth link lost")) as c:
            self.client = c
            await c.start_notify(self.TX, self._notify)
            self.on_status("connected", f"Bluetooth {dev.name}")
            while not self._stop.is_set() and c.is_connected:
                await asyncio.sleep(0.2)

    def _notify(self, _, data):
        self._buf.extend(data)
        while b"\n" in self._buf:
            i = self._buf.index(b"\n")
            self.on_line(self._buf[:i].decode("utf-8", "replace").strip())
            del self._buf[:i + 1]

    def send(self, text):
        import asyncio
        if not (self.loop and self.client):
            return False

        async def write():
            data = (text.strip() + "\n").encode()
            for i in range(0, len(data), 180):
                await self.client.write_gatt_char(self.RX, data[i:i + 180], response=False)
        asyncio.run_coroutine_threadsafe(write(), self.loop)
        return True

    def stop(self):
        self._stop.set()


# =============================================================================
# Widgets
# =============================================================================
class Card(ctk.CTkFrame):
    def __init__(self, master, title=None, **kw):
        super().__init__(master, fg_color=C["panel"], corner_radius=10, border_width=1, border_color=C["line"], **kw)
        if title:
            ctk.CTkLabel(self, text=title.upper(), font=font(10, "semi"), text_color=C["faint"]).pack(
                anchor="w", padx=16, pady=(12, 0))


class Metric(ctk.CTkFrame):
    """Label over a large tabular number with a unit."""

    def __init__(self, master, label, unit="", size=26):
        super().__init__(master, fg_color="transparent")
        ctk.CTkLabel(self, text=label, font=font(11), text_color=C["muted"]).pack(anchor="w")
        row = ctk.CTkFrame(self, fg_color="transparent")
        row.pack(anchor="w")
        self.value = ctk.CTkLabel(row, text="--", font=font(size, "monomed"), text_color=C["text"])
        self.value.pack(side="left")
        if unit:
            ctk.CTkLabel(row, text=unit, font=font(12), text_color=C["faint"]).pack(side="left", padx=(5, 0), pady=(6, 0))

    def set(self, text, color=None):
        self.value.configure(text=text, text_color=color or C["text"])


class Dot(tk.Canvas):
    """Small round status light."""

    def __init__(self, master, size=9, bg=None):
        super().__init__(master, width=size, height=size, bg=bg or C["panel"], highlightthickness=0)
        self.s = size
        self.set(C["faint"])

    def set(self, color):
        self.delete("all")
        self.create_oval(1, 1, self.s - 1, self.s - 1, fill=color, outline="")


class Row(ctk.CTkFrame):
    """Label ... value line for detail lists."""

    def __init__(self, master, label):
        super().__init__(master, fg_color="transparent")
        ctk.CTkLabel(self, text=label, font=font(12), text_color=C["muted"]).pack(side="left")
        self.value = ctk.CTkLabel(self, text="--", font=font(12, "mono"), text_color=C["text"])
        self.value.pack(side="right")

    def set(self, text, color=None):
        self.value.configure(text=text, text_color=color or C["text"])


def button(master, text, command, kind="normal", ic=None, width=None):
    styles = {
        "normal":  dict(fg_color=C["raised"], hover_color=C["hover"], text_color=C["text"], border_width=1, border_color=C["line"]),
        "primary": dict(fg_color=C["accent"], hover_color="#D18E2C", text_color="#1A1205", border_width=0),
        "danger":  dict(fg_color="#3A1A1C", hover_color="#4A2023", text_color="#F3B3B5", border_width=1, border_color="#5A2629"),
        "quiet":   dict(fg_color="transparent", hover_color=C["hover"], text_color=C["muted"], border_width=0),
    }[kind]
    color = styles["text_color"]
    b = ctk.CTkButton(master, text=text, command=command, height=34, corner_radius=7, font=font(12, "med"),
                      image=icon(ic, 15, color) if ic else None, compound="left", **styles)
    if width:
        b.configure(width=width)
    return b


class PhaseRail(tk.Canvas):
    def __init__(self, master):
        super().__init__(master, height=74, bg=C["panel"], highlightthickness=0)
        self.current, self.lift = "PAD", False
        self.bind("<Configure>", lambda e: self.draw())

    def set(self, phase, lift=False):
        if phase != self.current or lift != self.lift:
            self.current, self.lift = phase, lift
            self.draw()

    def draw(self):
        self.delete("all")
        w = self.winfo_width()
        n = len(PHASES)
        x0, x1, y = 40, max(80, w - 40), 26
        idx = PHASES.index(self.current) if self.current in PHASES else 0
        step = (x1 - x0) / (n - 1)
        self.create_line(x0, y, x1, y, fill=C["line"], width=2)
        if idx > 0:
            self.create_line(x0, y, x0 + step * idx, y, fill=C["accent"], width=2)
        for i, ph in enumerate(PHASES):
            x = x0 + step * i
            if i < idx:
                self.create_oval(x - 6, y - 6, x + 6, y + 6, fill=C["accent"], outline="")
            elif i == idx:
                self.create_oval(x - 11, y - 11, x + 11, y + 11, outline=C["accent"], width=2)
                self.create_oval(x - 6, y - 6, x + 6, y + 6, fill=C["accent"], outline="")
            else:
                self.create_oval(x - 6, y - 6, x + 6, y + 6, fill=C["panel"], outline=C["faint"], width=2)
            self.create_text(x, y + 27, text=PHASE_LABEL[ph], fill=C["text"] if i == idx else C["muted"],
                             font=(SANS_SEMI if i == idx else SANS, 10))
        if self.lift:
            self.create_text(x1, 6, text="LIFT TEST", anchor="ne", fill=C["accent"], font=(SANS_SEMI, 9))


class Horizon(tk.Canvas):
    """Small attitude indicator driven by pitch / roll."""

    def __init__(self, master, size=210):
        super().__init__(master, width=size, height=size, bg=C["panel"], highlightthickness=0)
        self.s = size

    def set(self, pitch, roll):
        s, c = self.s, self.s / 2
        k = 2.2                                               # pixels per degree of pitch
        self.delete("all")
        a = math.radians(-roll)                               # horizon turns opposite to the roll
        dx, dy = math.cos(a), -math.sin(a)                    # along the horizon (screen y points down)
        nx, ny = -dy, dx                                      # normal, pointing to the ground
        off = max(-c, min(c, pitch * k))                      # nose up -> horizon moves down
        cx, cy = c + nx * off, c + ny * off
        L = s * 2
        p1 = (cx - dx * L, cy - dy * L)
        p2 = (cx + dx * L, cy + dy * L)
        self.create_rectangle(0, 0, s, s, fill="#1E3A57", outline="")
        self.create_polygon(p1[0], p1[1], p2[0], p2[1], p2[0] + nx * L, p2[1] + ny * L,
                            p1[0] + nx * L, p1[1] + ny * L, fill="#4A3524", outline="")
        self.create_line(p1[0], p1[1], p2[0], p2[1], fill="#D9DEE5", width=2)
        for deg in (-20, -10, 10, 20):                        # pitch ladder
            mx, my = cx - nx * deg * k, cy - ny * deg * k
            hl = 30 if deg % 20 == 0 else 18
            self.create_line(mx - dx * hl, my - dy * hl, mx + dx * hl, my + dy * hl, fill="#C9CFD8", width=1)
        # fixed aircraft symbol + ring mask
        self.create_line(c - 46, c, c - 14, c, fill=C["accent"], width=3)
        self.create_line(c + 14, c, c + 46, c, fill=C["accent"], width=3)
        self.create_oval(c - 3, c - 3, c + 3, c + 3, fill=C["accent"], outline="")
        self.create_oval(-s * 0.2, -s * 0.2, s * 1.2, s * 1.2, outline=C["panel"], width=s * 0.4)
        self.create_oval(4, 4, s - 4, s - 4, outline=C["line"], width=2)


# =============================================================================
# Application
# =============================================================================
class Dock(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Aakashvani Ground Station")
        self.geometry("1280x760+40+20")
        self.minsize(1080, 680)
        self.configure(fg_color=C["bg"])

        self.settings = self._load_settings()
        self.link = None
        self.rx = queue.Queue()
        self.capture = None            # (kind, end_prefix, lines) while a log list / dump is streaming
        self.tm = dict(phase="PAD", lift=False, agl=0.0, v=0.0, peak=0.0, arms=0, esc=0, dist=0.0,
                          brg=0.0, sats=0, held=0, sp=(0.0, 0.0), hdg=0.0, mtime="--:--:--", pkts=0,
                          pressure=0.0, temp=0.0, lat=0.0, lon=0.0, q=(1, 0, 0, 0), mount=0, att_rate=0.0)
        self.health = {}
        self.env = {}
        self.alt_hist = deque(maxlen=600)
        self.events = []
        self._att_count, self._att_t0, self._last_att_ui = 0, time.time(), 0.0
        self._rx_count, self._rx_t0, self._rx_rate = 0, time.time(), 0.0
        self._last_line_t = 0.0
        self.sim_running = False
        self.tm_file = None            # Flight_<TEAM_ID>.csv: every received frame (graded by the judges)
        self.tm_rows = 0

        self._build_header()
        self._build_body()
        self._build_statusbar()
        self.show("overview")

        if telemetry_3d_server:
            try:
                telemetry_3d_server.start_server(http_port=8055, ws_port=8765)
            except Exception:
                pass
        self.after(25, self._pump)
        self.after(500, self._tick)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------ settings
    def _load_settings(self):
        try:
            with open(SETTINGS_FILE, encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return {"transport": "USB", "port": "", "log_dir": os.path.expanduser("~")}

    def _save_settings(self):
        try:
            with open(SETTINGS_FILE, "w", encoding="utf-8") as f:
                json.dump(self.settings, f)
        except Exception:
            pass

    # ------------------------------------------------------------------ layout
    def _build_header(self):
        h = ctk.CTkFrame(self, fg_color=C["panel"], corner_radius=0, height=56, border_width=0)
        h.pack(fill="x", side="top")
        h.pack_propagate(False)
        ctk.CTkFrame(self, fg_color=C["line"], height=1, corner_radius=0).pack(fill="x", side="top")

        brand = ctk.CTkFrame(h, fg_color="transparent")
        brand.pack(side="left", padx=(20, 0))
        ctk.CTkLabel(brand, text="Aakashvani", font=font(17, "semi"), text_color=C["text"]).pack(side="left")
        ctk.CTkLabel(brand, text="Ground Station", font=font(13), text_color=C["muted"]).pack(side="left", padx=(8, 0), pady=(3, 0))

        right = ctk.CTkFrame(h, fg_color="transparent")
        right.pack(side="right", padx=16)
        self.link_dot = Dot(right, 10)
        self.link_text = ctk.CTkLabel(right, text="Not connected", font=font(12), text_color=C["muted"])
        self.transport = ctk.CTkSegmentedButton(right, values=["USB", "Bluetooth"], font=font(12, "med"),
                                                selected_color="#343C48", selected_hover_color="#3B4451",
                                                unselected_color=C["panel"], unselected_hover_color=C["hover"],
                                                fg_color=C["panel"], text_color=C["text"], height=32,
                                                command=lambda v: self._on_transport(v))
        self.transport.set(self.settings.get("transport", "USB"))
        self.port_menu = ctk.CTkOptionMenu(right, values=self._ports(), width=150, height=32, font=font(12),
                                           fg_color=C["raised"], button_color=C["raised"], button_hover_color=C["hover"],
                                           dropdown_fg_color=C["raised"], dropdown_font=font(12))
        self.refresh_btn = ctk.CTkButton(right, text="", image=icon("refresh", 15), width=32, height=32,
                                         fg_color=C["raised"], hover_color=C["hover"], border_width=1,
                                         border_color=C["line"], command=self._refresh_ports)
        self.connect_btn = button(right, "Connect", self._toggle_link, "primary", "plug", width=120)
        for w in (self.connect_btn, self.refresh_btn, self.port_menu, self.transport):
            w.pack(side="right", padx=(8, 0))
        self.link_text.pack(side="right", padx=(0, 14))
        self.link_dot.pack(side="right", padx=(0, 8))
        self._refresh_ports()
        self._on_transport(self.transport.get())

    def _build_body(self):
        body = ctk.CTkFrame(self, fg_color=C["bg"], corner_radius=0)
        body.pack(fill="both", expand=True)

        nav = ctk.CTkFrame(body, fg_color=C["panel"], corner_radius=0, width=196)
        nav.pack(side="left", fill="y")
        nav.pack_propagate(False)
        ctk.CTkFrame(body, fg_color=C["line"], width=1, corner_radius=0).pack(side="left", fill="y")

        self.pages, self.nav_buttons = {}, {}
        items = [("overview", "Overview"), ("attitude", "Attitude"), ("health", "Health"),
                 ("log", "Flight log"), ("bench", "Bench"), ("console", "Console"), ("firmware", "Firmware")]
        ctk.CTkLabel(nav, text="", height=8).pack()
        for key, label in items:
            b = ctk.CTkButton(nav, text=label, anchor="w", height=38, corner_radius=7, font=font(13, "med"),
                              fg_color="transparent", hover_color=C["hover"], text_color=C["muted"],
                              image=icon(key, 17), compound="left", command=lambda k=key: self.show(k))
            b.pack(fill="x", padx=10, pady=1)
            self.nav_buttons[key] = b

        self.content = ctk.CTkFrame(body, fg_color=C["bg"], corner_radius=0)
        self.content.pack(side="left", fill="both", expand=True)
        for key, _ in items:
            page = ctk.CTkFrame(self.content, fg_color=C["bg"], corner_radius=0)
            self.pages[key] = page
            getattr(self, f"_page_{key}")(page)

    def _build_statusbar(self):
        ctk.CTkFrame(self, fg_color=C["line"], height=1, corner_radius=0).pack(fill="x", side="bottom")
        sb = ctk.CTkFrame(self, fg_color=C["panel"], corner_radius=0, height=28)
        sb.pack(fill="x", side="bottom")
        sb.pack_propagate(False)
        self.sb_items = {}
        for key in ("mtime", "pkts", "rate", "log", "note"):
            lbl = ctk.CTkLabel(sb, text="", font=font(11, "mono"), text_color=C["muted"])
            lbl.pack(side="left", padx=(16, 8))
            self.sb_items[key] = lbl

    def show(self, key):
        for k, p in self.pages.items():
            p.pack_forget()
        self.pages[key].pack(fill="both", expand=True, padx=22, pady=18)
        for k, b in self.nav_buttons.items():
            active = (k == key)
            b.configure(fg_color=C["raised"] if active else "transparent",
                        text_color=C["text"] if active else C["muted"],
                        image=icon(k, 17, C["accent"] if active else C["muted"]))

    def _page_title(self, page, title, subtitle):
        ctk.CTkLabel(page, text=title, font=font(20, "semi"), text_color=C["text"]).pack(anchor="w")
        ctk.CTkLabel(page, text=subtitle, font=font(12), text_color=C["muted"]).pack(anchor="w", pady=(0, 14))

    # ------------------------------------------------------------------ pages
    def _page_overview(self, page):
        self._page_title(page, "Mission", "Live phase, altitude and return-to-launch status")
        rail_card = Card(page)
        rail_card.pack(fill="x")
        self.rail = PhaseRail(rail_card)
        self.rail.pack(fill="x", padx=12, pady=10)

        row = ctk.CTkFrame(page, fg_color="transparent")
        row.pack(fill="x", pady=12)
        self.ov = {}
        for i, (key, label, unit) in enumerate([("agl", "Altitude above pad", "m"), ("v", "Vertical speed", "m/s"),
                                                 ("peak", "Peak", "m"), ("mtime", "Time since power-on", "")]):
            card = Card(row)
            card.grid(row=0, column=i, sticky="nsew", padx=(0 if i == 0 else 6, 0))
            row.grid_columnconfigure(i, weight=1)
            m = Metric(card, label, unit, 28)
            m.pack(anchor="w", padx=16, pady=14)
            self.ov[key] = m

        lower = ctk.CTkFrame(page, fg_color="transparent")
        lower.pack(fill="both", expand=True)
        lower.grid_columnconfigure(0, weight=3)
        lower.grid_columnconfigure(1, weight=2)
        lower.grid_rowconfigure(0, weight=1)

        chart = Card(lower, "Altitude, last 2 minutes")
        chart.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        if HAVE_MPL:
            fig = Figure(figsize=(6, 3), dpi=100, facecolor=C["panel"])
            self.ax = fig.add_subplot(111)
            self._style_axes(self.ax)
            self.alt_line, = self.ax.plot([], [], color=C["accent"], linewidth=1.6)
            fig.subplots_adjust(left=0.09, right=0.98, top=0.95, bottom=0.14)
            self.alt_canvas = FigureCanvasTkAgg(fig, master=chart)
            self.alt_canvas.get_tk_widget().configure(bg=C["panel"], highlightthickness=0)
            self.alt_canvas.get_tk_widget().pack(fill="both", expand=True, padx=8, pady=8)

        side = ctk.CTkFrame(lower, fg_color="transparent")
        side.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        ret = Card(side, "Return to launch")
        ret.pack(fill="x")
        self.ret = {k: Row(ret, l) for k, l in [("dist", "Distance to site"), ("brg", "Bearing"), ("sats", "Satellites"),
                                                  ("tilt", "Tilt command"), ("arms", "Drone arms"), ("esc", "Motors")]}
        for r in self.ret.values():
            r.pack(fill="x", padx=16, pady=3)
        ctk.CTkLabel(ret, text="", height=6).pack()
        ev = Card(side, "Events")
        ev.pack(fill="both", expand=True, pady=(12, 0))
        self.event_box = ctk.CTkTextbox(ev, fg_color=C["panel"], text_color=C["text"], font=font(11, "mono"),
                                        border_width=0, wrap="word", activate_scrollbars=True)
        self.event_box.pack(fill="both", expand=True, padx=10, pady=(4, 10))
        self.event_box.insert("end", "Waiting for the flight computer.\n")
        self.event_box.configure(state="disabled")

    def _page_attitude(self, page):
        self._page_title(page, "Attitude", "Orientation from the BNO055 quaternion (gimbal-lock free)")
        top = ctk.CTkFrame(page, fg_color="transparent")
        top.pack(fill="x")
        hc = Card(top, "Horizon")
        hc.pack(side="left", padx=(0, 12))
        self.horizon = Horizon(hc, 230)
        self.horizon.pack(padx=16, pady=14)
        self.horizon.set(0, 0)

        nums = Card(top, "Angles")
        nums.pack(side="left", fill="both", expand=True)
        grid = ctk.CTkFrame(nums, fg_color="transparent")
        grid.pack(fill="x", padx=16, pady=14)
        self.att = {}
        for i, (k, l) in enumerate([("p", "Tilt X (pitch)"), ("r", "Tilt Y (roll)"), ("y", "Heading")]):
            m = Metric(grid, l, "deg", 30)
            m.grid(row=0, column=i, sticky="w", padx=(0, 36))
            self.att[k] = m
        self.att_rows = {k: Row(nums, l) for k, l in [("mount", "Sensor mount (auto-detected)"), ("rate", "Attitude stream"),
                                                        ("mcu", "Flight computer clock")]}
        for r in self.att_rows.values():
            r.pack(fill="x", padx=16, pady=3)

        act = Card(page, "Tools")
        act.pack(fill="x", pady=12)
        bar = ctk.CTkFrame(act, fg_color="transparent")
        bar.pack(fill="x", padx=16, pady=(8, 14))
        button(bar, "3D view", self._open_3d, "normal", "box").pack(side="left")
        button(bar, "3D view in browser", lambda: __import__("webbrowser").open("http://localhost:8055/"), "quiet", "layers").pack(side="left", padx=8)
        button(bar, "Re-reference", self._tare, "normal", "crosshair").pack(side="left", padx=(24, 0))
        button(bar, "Calibrate north", self._north, "normal", "compass").pack(side="left", padx=8)
        ctk.CTkLabel(act, text="Re-reference and north calibration only work on the pad; the reference is locked once a flight starts.",
                     font=font(11), text_color=C["faint"]).pack(anchor="w", padx=16, pady=(0, 12))

    def _page_health(self, page):
        self._page_title(page, "Health", "One glance before flight. Works over USB or Bluetooth")
        grid = ctk.CTkFrame(page, fg_color="transparent")
        grid.pack(fill="x")
        self.tiles = {}
        specs = [("imu", "IMU", "activity"), ("baro", "Barometer", "gauge"), ("gnss", "GNSS", "satellite"),
                 ("bat", "Battery", "battery"), ("mem", "Free memory", "cpu"), ("log", "Flight log", "drive"),
                 ("temp", "Board temperature", "thermo"), ("link", "Bluetooth", "ble")]
        for i, (k, label, ic) in enumerate(specs):
            card = Card(grid)
            card.grid(row=i // 4, column=i % 4, sticky="nsew", padx=(0 if i % 4 == 0 else 6, 0), pady=(0, 12))
            grid.grid_columnconfigure(i % 4, weight=1)
            head = ctk.CTkFrame(card, fg_color="transparent")
            head.pack(fill="x", padx=16, pady=(14, 0))
            ctk.CTkLabel(head, text="", image=icon(ic, 16)).pack(side="left")
            ctk.CTkLabel(head, text=label, font=font(12), text_color=C["muted"]).pack(side="left", padx=(8, 0))
            dot = Dot(head, 9)
            dot.pack(side="right")
            val = ctk.CTkLabel(card, text="--", font=font(20, "monomed"), text_color=C["text"])
            val.pack(anchor="w", padx=16, pady=(6, 0))
            sub = ctk.CTkLabel(card, text="", font=font(11), text_color=C["faint"])
            sub.pack(anchor="w", padx=16, pady=(0, 14))
            self.tiles[k] = (dot, val, sub)

        lower = ctk.CTkFrame(page, fg_color="transparent")
        lower.pack(fill="both", expand=True)
        lower.grid_columnconfigure(0, weight=1)
        lower.grid_columnconfigure(1, weight=1)
        selft = Card(lower, "Self test")
        selft.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self.bit_label = ctk.CTkLabel(selft, text="No data yet", font=font(12), text_color=C["muted"], justify="left")
        self.bit_label.pack(anchor="w", padx=16, pady=(8, 6))
        self.crash_label = ctk.CTkLabel(selft, text="", font=font(12), text_color=C["muted"], justify="left")
        self.crash_label.pack(anchor="w", padx=16, pady=(0, 14))
        envc = Card(lower, "Environment")
        envc.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        self.env_rows = {k: Row(envc, l) for k, l in [("t", "Air temperature"), ("rh", "Relative humidity"),
                                                        ("voc", "VOC index"), ("nox", "NOx index"), ("p", "Pressure")]}
        for r in self.env_rows.values():
            r.pack(fill="x", padx=16, pady=3)
        ctk.CTkLabel(envc, text="", height=6).pack()

    def _page_log(self, page):
        self._page_title(page, "Flight log", "The on-board recorder keeps 50 Hz flight data in 12 MB of internal flash")
        bar = ctk.CTkFrame(page, fg_color="transparent")
        bar.pack(fill="x", pady=(0, 10))
        button(bar, "Refresh list", self._log_list, "normal", "refresh").pack(side="left")
        button(bar, "Download selected", self._log_download, "primary", "download").pack(side="left", padx=8)
        button(bar, "Crash report", self._crash_report, "quiet", "alert").pack(side="left", padx=(16, 0))
        button(bar, "Erase log", self._log_erase, "danger", "trash").pack(side="right")
        gc = Card(page, "Ground telemetry file (for the judges)")
        gc.pack(fill="x", pady=(0, 12))
        gr = ctk.CTkFrame(gc, fg_color="transparent")
        gr.pack(fill="x", padx=16, pady=(6, 14))
        self.tm_label = ctk.CTkLabel(gr, text="", font=font(12, "mono"), text_color=C["text"], anchor="w", justify="left")
        self.tm_label.pack(side="left", fill="x", expand=True)
        button(gr, "Open folder", self._tm_open_folder, "quiet", "drive").pack(side="right")
        button(gr, "Start fresh file", self._tm_archive, "quiet", "refresh").pack(side="right", padx=8)
        lc = Card(page, "On-board recorder sessions (newest first)")
        lc.pack(fill="both", expand=True)
        self.log_list = tk.Listbox(lc, bg=C["panel"], fg=C["text"], selectbackground=C["raised"],
                                   selectforeground=C["accent"], highlightthickness=0, borderwidth=0,
                                   font=(MONO, 10), activestyle="none")
        self.log_list.pack(fill="both", expand=True, padx=14, pady=10)
        self.log_status = ctk.CTkLabel(page, text="Connect over USB, then refresh.", font=font(11), text_color=C["muted"])
        self.log_status.pack(anchor="w", pady=(8, 0))

    def _page_bench(self, page):
        self._page_title(page, "Bench", "Calibration and tests on the ground. Actuator tests need USB and props off")
        cols = ctk.CTkFrame(page, fg_color="transparent")
        cols.pack(fill="both", expand=True)
        cols.grid_columnconfigure(0, weight=1)
        cols.grid_columnconfigure(1, weight=1)

        g = Card(cols, "Ground")
        g.grid(row=0, column=0, sticky="nsew", padx=(0, 6), pady=(0, 12))
        b = ctk.CTkFrame(g, fg_color="transparent")
        b.pack(fill="x", padx=16, pady=(8, 14))
        button(b, "Zero altitude", lambda: self._send("CAL"), "normal", "mountain").pack(side="left")
        button(b, "Reset mission clock", lambda: self._send("ST,00:00:00"), "quiet").pack(side="left", padx=8)
        self.stream_switch = ctk.CTkSwitch(b, text="Radio telemetry (CX)", font=font(12), progress_color=C["accent"],
                                           command=lambda: self._send("CX," + ("ON" if self.stream_switch.get() else "OFF")))
        self.stream_switch.pack(side="right")

        lt = Card(cols, "Lift test")
        lt.grid(row=0, column=1, sticky="nsew", padx=(6, 0), pady=(0, 12))
        ctk.CTkLabel(lt, text="Runs the whole mission at building scale. Motors stay inhibited.",
                     font=font(11), text_color=C["faint"]).pack(anchor="w", padx=16, pady=(6, 0))
        r = ctk.CTkFrame(lt, fg_color="transparent")
        r.pack(fill="x", padx=16, pady=(8, 14))
        ctk.CTkLabel(r, text="Open arms at", font=font(12), text_color=C["muted"]).pack(side="left")
        self.lift_entry = ctk.CTkEntry(r, width=56, height=32, font=font(12, "mono"), fg_color=C["raised"], border_color=C["line"])
        self.lift_entry.insert(0, "10")
        self.lift_entry.pack(side="left", padx=6)
        ctk.CTkLabel(r, text="m", font=font(12), text_color=C["muted"]).pack(side="left")
        button(r, "Stop", lambda: self._send("LIFT,OFF"), "quiet").pack(side="right")
        button(r, "Start", lambda: self._send(f"LIFT,{self.lift_entry.get().strip() or '10'}"), "normal", "play").pack(side="right", padx=6)

        sm = Card(cols, "Simulated flight")
        sm.grid(row=1, column=0, sticky="nsew", padx=(0, 6), pady=(0, 12))
        ctk.CTkLabel(sm, text="Feeds a pressure profile to the real firmware (hardware in the loop).",
                     font=font(11), text_color=C["faint"]).pack(anchor="w", padx=16, pady=(6, 0))
        r2 = ctk.CTkFrame(sm, fg_color="transparent")
        r2.pack(fill="x", padx=16, pady=(8, 14))
        self.sim_profile = ctk.CTkOptionMenu(r2, values=["Carrier drone to 750 m", "Lift, 10 floors"], width=200, height=32,
                                             font=font(12), fg_color=C["raised"], button_color=C["raised"],
                                             button_hover_color=C["hover"], dropdown_fg_color=C["raised"])
        self.sim_profile.pack(side="left")
        self.sim_btn = button(r2, "Run", self._sim_toggle, "normal", "play", width=90)
        self.sim_btn.pack(side="left", padx=8)

        ac = Card(cols, "Actuators (props off)")
        ac.grid(row=1, column=1, sticky="nsew", padx=(6, 0), pady=(0, 12))
        r3 = ctk.CTkFrame(ac, fg_color="transparent")
        r3.pack(fill="x", padx=16, pady=(8, 4))
        ctk.CTkLabel(r3, text="Throttle", font=font(12), text_color=C["muted"]).pack(side="left")
        self.mtr_slider = ctk.CTkSlider(r3, from_=0, to=25, number_of_steps=25, width=150, progress_color=C["accent"],
                                        button_color=C["text"], button_hover_color=C["accent"])
        self.mtr_slider.set(8)
        self.mtr_slider.pack(side="left", padx=8)
        self.mtr_value = ctk.CTkLabel(r3, text="8 %", font=font(12, "mono"), text_color=C["text"], width=40)
        self.mtr_value.pack(side="left")
        self.mtr_slider.configure(command=lambda v: self.mtr_value.configure(text=f"{int(v)} %"))
        r4 = ctk.CTkFrame(ac, fg_color="transparent")
        r4.pack(fill="x", padx=16, pady=(4, 6))
        for i, name in enumerate(["M1", "M2", "M3", "M4", "All"]):
            button(r4, name, lambda i=i: self._motor_test(i if i < 4 else -1), "normal", width=52).pack(side="left", padx=(0, 6))
        r5 = ctk.CTkFrame(ac, fg_color="transparent")
        r5.pack(fill="x", padx=16, pady=(4, 14))
        button(r5, "Open arm latches", self._unlatch, "normal", "lock_open").pack(side="left")
        button(r5, "Abort motors", lambda: self._send("ABORT"), "danger", "x").pack(side="right")

    def _page_console(self, page):
        self._page_title(page, "Console", "Everything the flight computer prints")
        bar = ctk.CTkFrame(page, fg_color="transparent")
        bar.pack(fill="x", pady=(0, 8))
        self.show_telem = ctk.CTkSwitch(bar, text="Show telemetry lines", font=font(12), progress_color=C["accent"])
        self.show_telem.pack(side="left")
        button(bar, "Clear", lambda: self._console_clear(), "quiet").pack(side="right")
        self.console = ctk.CTkTextbox(page, fg_color=C["panel"], text_color=C["text"], font=font(11, "mono"),
                                      border_width=1, border_color=C["line"], corner_radius=10, wrap="none")
        self.console.pack(fill="both", expand=True)
        entry_row = ctk.CTkFrame(page, fg_color="transparent")
        entry_row.pack(fill="x", pady=(8, 0))
        self.cmd_entry = ctk.CTkEntry(entry_row, height=36, font=font(12, "mono"), fg_color=C["panel"], border_color=C["line"],
                                      placeholder_text="CMD,001,...  or a console command (status, help)")
        self.cmd_entry.pack(side="left", fill="x", expand=True)
        self.cmd_entry.bind("<Return>", lambda e: self._console_send())
        self.cmd_entry.bind("<Up>", lambda e: self._history(-1))
        self.cmd_entry.bind("<Down>", lambda e: self._history(1))
        button(entry_row, "Send", self._console_send, "normal", "send", width=90).pack(side="left", padx=(8, 0))
        self.history, self.hist_i = [], 0

    def _page_firmware(self, page):
        self._page_title(page, "Firmware", "Flashes build/ over USB with esptool (disconnects the link first)")
        bar = ctk.CTkFrame(page, fg_color="transparent")
        bar.pack(fill="x", pady=(0, 10))
        button(bar, "Flash firmware", self._flash, "primary", "upload").pack(side="left")
        button(bar, "Identify chip", self._chip_id, "normal", "cpu").pack(side="left", padx=8)
        button(bar, "Erase entire flash", self._erase_flash, "danger", "trash").pack(side="right")
        self.flash_box = ctk.CTkTextbox(page, fg_color=C["panel"], text_color=C["muted"], font=font(11, "mono"),
                                        border_width=1, border_color=C["line"], corner_radius=10)
        self.flash_box.pack(fill="both", expand=True)

    # ------------------------------------------------------------------ links
    def _ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports() if "bluetooth" not in (p.description or "").lower()]
        return ports or ["No USB port"]

    def _refresh_ports(self):
        ports = self._ports()
        self.port_menu.configure(values=ports)
        want = self.settings.get("port")
        self.port_menu.set(want if want in ports else ports[0])

    def _on_transport(self, value):
        self.settings["transport"] = value
        if value == "USB":
            self.port_menu.configure(state="normal")
            self.refresh_btn.configure(state="normal")
        else:
            self.port_menu.configure(state="disabled")
            self.refresh_btn.configure(state="disabled")

    def _toggle_link(self):
        if self.link:
            self.link.stop()
            self.link = None
            self._set_link_status("idle", "Not connected")
            return
        try:
            if self.transport.get() == "USB":
                port = self.port_menu.get()
                if port.startswith("No "):
                    return
                self.settings["port"] = port
                self.link = SerialLink(port, self._on_line, self._link_status_threadsafe)
            else:
                self.link = BleLink(self._on_line, self._link_status_threadsafe)
            self.link.start()
            self._save_settings()
            self.connect_btn.configure(text="Disconnect", image=icon("x", 15, C["text"]), fg_color=C["raised"],
                                       hover_color=C["hover"], text_color=C["text"], border_width=1, border_color=C["line"])
        except Exception as e:
            self.link = None
            messagebox.showerror("Connection failed", str(e))

    def _link_status_threadsafe(self, state, text):
        self.rx.put(("status", state, text))

    def _set_link_status(self, state, text):
        color = {"connected": C["ok"], "busy": C["warn"], "lost": C["bad"]}.get(state, C["faint"])
        self.link_dot.set(color)
        self.link_text.configure(text=text, text_color=C["text"] if state == "connected" else C["muted"])
        if state in ("idle", "lost"):
            if state == "lost" and self.link:
                self.link.stop()
                self.link = None
            self.connect_btn.configure(text="Connect", image=icon("plug", 15, "#1A1205"), fg_color=C["accent"],
                                       hover_color="#D18E2C", text_color="#1A1205", border_width=0)

    def _send(self, cmd, raw=False):
        if not self.link:
            self._note("Not connected")
            return False
        line = cmd if raw or cmd.upper().startswith("CMD,") or " " in cmd or cmd.lower() in ("status", "help") \
            else f"CMD,{TEAM},{cmd}"
        if self.link.kind == "ble" and any(f",{b}" in line.upper() or line.upper().startswith(b) for b in BLE_BLOCKED):
            self._note("That command needs the USB link")
            return False
        self._console_add(f"> {line}\n", C["accent"])
        return self.link.send(line)

    # ------------------------------------------------------------------ RX path
    def _on_line(self, line):
        """Called from the link thread."""
        if not line:
            return
        self._rx_count += 1
        cap = self.capture
        if cap is not None:
            kind, end, lines = cap
            if line.startswith(("LOG", "CRASH")):
                lines.append(line)
                if line.startswith(end):
                    self.capture = None
                    self.rx.put(("capture", kind, lines))
                return
        if ",ATT," in line[:12]:
            self._att_fast(line)
            return
        self.rx.put(("line", line))

    def _att_fast(self, line):
        p = line.split(",")
        if len(p) < 11:
            return
        try:
            q = (float(p[3]), float(p[4]), float(p[5]), float(p[6]))
            mcu, mount = int(p[2]), int(p[10])
        except ValueError:
            return
        self._att_count += 1
        if telemetry_3d_server:
            telemetry_3d_server.update_telemetry(q=q, t_mcu=mcu, mount_name=MOUNTS.get(mount, "?"))
        now = time.time()
        if now - self._last_att_ui > 0.05:
            self._last_att_ui = now
            self.rx.put(("att", q, mcu, mount))

    def _pump(self):
        n = 0
        while n < 400:
            try:
                msg = self.rx.get_nowait()
            except queue.Empty:
                break
            n += 1
            kind = msg[0]
            if kind == "line":
                self._handle_line(msg[1])
            elif kind == "att":
                self._handle_att(*msg[1:])
            elif kind == "status":
                self._set_link_status(msg[1], msg[2])
            elif kind == "capture":
                self._handle_capture(msg[1], msg[2])
        self.after(25, self._pump)

    def _handle_line(self, line):
        self._last_line_t = time.time()
        p = line.split(",")
        telem = False
        if len(p) > 2 and p[0] == TEAM:
            tag = p[1]
            telem = True
            if tag == "MSN" and len(p) >= 15:
                self._on_msn(p)
            elif tag == "HLT" and len(p) >= 16:
                self._on_hlt(p)
            elif tag == "ENV" and len(p) >= 6:
                self._on_env(p)
            elif tag == "EVT":
                self._add_event(",".join(p[2:]))
                telem = False
            elif tag in ("ACK", "NAK"):
                self._note(("Sent: " if tag == "ACK" else "Refused: ") + ",".join(p[2:]))
                telem = False
        elif p[0] == TEAM_ID and len(p) >= 19:
            telem = True
            self._on_frame(p, line)
        elif "MISSION:" in line and "]" in line:
            self._add_event(line.split("MISSION:", 1)[1].strip())
        if not telem or self.show_telem.get():
            self._console_add(line + "\n")

    def _on_msn(self, p):
        try:
            ph = p[2]
            s = self.tm
            s["lift"] = ph.startswith("LIFT-")
            s["phase"] = ph[5:] if s["lift"] else ph
            s["agl"], s["v"], s["peak"] = float(p[3]), float(p[4]), float(p[5])
            s["arms"], s["esc"] = int(p[6]), int(p[7])
            s["dist"], s["brg"], s["sats"], s["held"] = float(p[8]), float(p[9]), int(p[10]), int(p[11])
            s["sp"], s["hdg"] = (float(p[12]), float(p[13])), float(p[14])
            self.alt_hist.append((time.time(), s["agl"]))
            if telemetry_3d_server:
                telemetry_3d_server.update_telemetry(alt=s["agl"], state_name=PHASE_LABEL.get(s["phase"], s["phase"]))
        except (ValueError, IndexError):
            pass

    def _on_frame(self, p, line):
        self._record_frame(line)
        try:
            s = self.tm
            t = float(p[1])
            s["mtime"] = f"{int(t // 3600):02d}:{int(t % 3600 // 60):02d}:{int(t % 60):02d}"
            s["pkts"] = int(p[2])
            s["pressure"], s["temp"] = float(p[4]), float(p[5])
            s["lat"], s["lon"] = float(p[8]), float(p[9])
            if telemetry_3d_server:
                telemetry_3d_server.update_telemetry(pres=s["pressure"], temp=s["temp"], pcount=s["pkts"])
        except (ValueError, IndexError):
            pass

    def _on_hlt(self, p):
        keys = ["uptime", "imu_hz", "baro", "sats", "vbat", "heap", "log_kb", "erased_kb", "temp", "bit",
                "ble", "mount", "crash", "dropped"]
        h = {}
        for k, v in zip(keys, p[2:16]):
            h[k] = v
        self.health = h
        if telemetry_3d_server:
            try:
                telemetry_3d_server.update_telemetry(volt=float(h.get("vbat", 0)))
            except ValueError:
                pass

    def _on_env(self, p):
        try:
            self.env = dict(t=float(p[2]), rh=float(p[3]), voc=int(p[4]), nox=int(p[5]))
        except ValueError:
            pass

    def _handle_att(self, q, mcu, mount):
        s = self.tm
        s["q"], s["mcu"], s["mount"] = q, mcu, mount

    def _add_event(self, text):
        stamp = datetime.now().strftime("%H:%M:%S")
        if self.events and self.events[-1][1] == text:
            return
        self.events.append((stamp, text))
        self.event_box.configure(state="normal")
        if len(self.events) == 1:
            self.event_box.delete("1.0", "end")
        self.event_box.insert("end", f"{stamp}  {text}\n")
        self.event_box.see("end")
        self.event_box.configure(state="disabled")

    # ------------------------------------------------------------------ periodic UI refresh (2 Hz/10 Hz)
    def _tick(self):
        s = self.tm
        now = time.time()
        if now - self._rx_t0 >= 1.0:
            self._rx_rate = self._rx_count / (now - self._rx_t0)
            s["att_rate"] = self._att_count / (now - self._rx_t0)
            self._rx_count = self._att_count = 0
            self._rx_t0 = now

        self.rail.set(s["phase"], s["lift"])
        self.ov["agl"].set(f"{s['agl']:.1f}")
        self.ov["v"].set(f"{s['v']:+.1f}")
        self.ov["peak"].set(f"{s['peak']:.0f}")
        self.ov["mtime"].set(s["mtime"])
        self.ret["dist"].set(f"{s['dist']:.0f} m" if s["dist"] > 0 else "no site fix", None if s["dist"] > 0 else C["faint"])
        self.ret["brg"].set(f"{s['brg']:.0f} deg")
        self.ret["sats"].set(str(s["sats"]), C["ok"] if s["sats"] >= 5 else C["warn"])
        self.ret["tilt"].set(f"{s['sp'][0]:+.0f}, {s['sp'][1]:+.0f} deg")
        self.ret["arms"].set("open" if s["arms"] else "latched", C["accent"] if s["arms"] else None)
        esc = ESC_STATE.get(s["esc"], "?")
        self.ret["esc"].set(esc, C["ok"] if esc == "running" else None)

        pitch, roll, yaw = euler_zxy(s["q"])
        self.horizon.set(pitch, roll)
        self.att["p"].set(f"{pitch:+.1f}")
        self.att["r"].set(f"{roll:+.1f}")
        self.att["y"].set(f"{yaw:.1f}")
        self.att_rows["mount"].set(MOUNTS.get(s["mount"], "?"))
        self.att_rows["rate"].set(f"{s['att_rate']:.0f} Hz")
        self.att_rows["mcu"].set(f"{s.get('mcu', 0) / 1000:.2f} s")

        self._refresh_health()
        if HAVE_MPL and now - getattr(self, "_last_plot", 0) > 0.5:
            self._last_plot = now
            self._refresh_plot()

        live = self.link is not None and now - self._last_line_t < 3
        self.sb_items["mtime"].configure(text=f"T+ {s['mtime']}")
        self.sb_items["pkts"].configure(text=f"packets {s['pkts']}")
        self.sb_items["rate"].configure(text=f"{self._rx_rate:.0f} lines/s" if live else "no data")
        self.sb_items["log"].configure(text=f"recording Flight_{TEAM_ID}.csv  {self.tm_rows} rows" if self.tm_rows else "")
        if hasattr(self, "tm_label"):
            self.tm_label.configure(text=f"{self._tm_path()}\n{self.tm_rows} frames recorded this session")
        self.after(100, self._tick)

    def _refresh_health(self):
        h, t = self.health, self.tiles
        if not h:
            return

        def tile(k, value, sub, status):
            dot, val, sl = t[k]
            dot.set({"ok": C["ok"], "warn": C["warn"], "bad": C["bad"]}.get(status, C["faint"]))
            val.configure(text=value)
            sl.configure(text=sub)
        try:
            imu = float(h["imu_hz"])
            tile("imu", f"{imu:.0f} Hz", MOUNTS.get(int(h["mount"]), ""), "ok" if imu >= 90 else "warn" if imu > 0 else "bad")
            tile("baro", "OK" if h["baro"] == "1" else "No data", f"{self.tm['pressure']:.0f} Pa" if self.tm["pressure"] else "",
                 "ok" if h["baro"] == "1" else "bad")
            sats = int(h["sats"])
            tile("gnss", f"{sats} sats", "launch site needs 5+", "ok" if sats >= 5 else "warn")
            vb = float(h["vbat"])
            tile("bat", f"{vb:.2f} V" if vb > 0 else "not fitted", "power monitor", "ok" if vb > 7.0 else "warn" if vb > 0 else None)
            heap = int(h["heap"])
            tile("mem", f"{heap} KB", "internal RAM free", "ok" if heap > 40 else "warn")
            erased = int(h["erased_kb"]) / 1024
            tile("log", f"{int(h['log_kb'])} KB", f"{erased:.1f} MB pre-erased ahead",
                 "ok" if erased >= 2.5 else "warn")
            tile("temp", f"{float(h['temp']):.1f} C", "baro sensor", "ok")
            tile("link", "Connected" if h["ble"] == "1" else "Advertising", "off automatically in flight", "ok")
            bits = int(h["bit"], 16)
            if bits == 0:
                self.bit_label.configure(text="All checks passed", text_color=C["ok"])
            else:
                names = [BIT_NAMES[i] for i in BIT_NAMES if bits & (1 << i)]
                self.bit_label.configure(text="Attention: " + ", ".join(names), text_color=C["warn"])
            self.crash_label.configure(
                text="A crash dump from a previous session is stored. See Flight log, Crash report." if h["crash"] == "1"
                else "No crash dump stored", text_color=C["bad"] if h["crash"] == "1" else C["muted"])
        except (KeyError, ValueError):
            pass
        e = self.env
        if e:
            self.env_rows["t"].set(f"{e['t']:.1f} C")
            self.env_rows["rh"].set(f"{e['rh']:.1f} %")
            self.env_rows["voc"].set(str(e["voc"]))
            self.env_rows["nox"].set(str(e["nox"]))
        if self.tm["pressure"]:
            self.env_rows["p"].set(f"{self.tm['pressure'] / 100:.1f} hPa")

    def _style_axes(self, ax):
        ax.set_facecolor(C["panel"])
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(C["line"])
        ax.tick_params(colors=C["muted"], labelsize=8, length=0)
        ax.grid(True, color=C["line"], linewidth=0.6)
        for lbl in ax.get_xticklabels() + ax.get_yticklabels():
            lbl.set_fontfamily("IBM Plex Sans" if SANS.startswith("IBM") else "sans-serif")

    def _refresh_plot(self):
        if not self.alt_hist:
            return
        now = time.time()
        pts = [(t - now, a) for t, a in self.alt_hist if now - t <= 120]
        if not pts:
            return
        xs, ys = zip(*pts)
        self.alt_line.set_data(xs, ys)
        self.ax.set_xlim(-120, 0)
        lo, hi = min(ys), max(ys)
        pad = max(2.0, (hi - lo) * 0.15)
        self.ax.set_ylim(lo - pad, hi + pad)
        self.alt_canvas.draw_idle()

    # ------------------------------------------------------------------ ground telemetry file
    def _tm_path(self):
        d = self.settings.get("tm_dir") or DEFAULT_TM_DIR
        os.makedirs(d, exist_ok=True)
        return os.path.join(d, f"Flight_{TEAM_ID}.csv")

    def _record_frame(self, line):
        """Append a received frame to Flight_<TEAM_ID>.csv (header written once)."""
        try:
            if self.tm_file is None:
                path = self._tm_path()
                new = not os.path.exists(path) or os.path.getsize(path) == 0
                self.tm_file = open(path, "a", encoding="utf-8", newline="")
                if new:
                    self.tm_file.write(FRAME_HEADER + "\n")
            self.tm_file.write(line.strip() + "\n")
            self.tm_rows += 1
            if self.tm_rows % 25 == 0:
                self.tm_file.flush()
        except OSError as e:
            self._note(f"Cannot write telemetry file: {e}")

    def _tm_archive(self):
        path = self._tm_path()
        if self.tm_file:
            self.tm_file.close()
            self.tm_file = None
        if os.path.exists(path) and os.path.getsize(path) > 0:
            if not messagebox.askyesno("Start a fresh file",
                                       f"Rename the current file to Flight_{TEAM_ID}_<date>.csv and start a new one?\n\n"
                                       "On flight day keep one file for the whole mission."):
                return
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            os.replace(path, path.replace(".csv", f"_{stamp}.csv"))
        self.tm_rows = 0
        self._note("Started a fresh telemetry file")

    def _tm_open_folder(self):
        folder = os.path.dirname(self._tm_path())
        try:
            os.startfile(folder)
        except Exception:
            messagebox.showinfo("Telemetry folder", folder)

    # ------------------------------------------------------------------ console
    def _console_add(self, text, color=None):
        self.console.insert("end", text)
        if int(self.console.index("end-1c").split(".")[0]) > 3000:
            self.console.delete("1.0", "500.0")
        self.console.see("end")

    def _console_clear(self):
        self.console.delete("1.0", "end")

    def _console_send(self):
        text = self.cmd_entry.get().strip()
        if not text:
            return
        self.history.append(text)
        self.hist_i = len(self.history)
        self.cmd_entry.delete(0, "end")
        self._send(text, raw=True)

    def _history(self, step):
        if not self.history:
            return
        self.hist_i = max(0, min(len(self.history) - 1, self.hist_i + step))
        self.cmd_entry.delete(0, "end")
        self.cmd_entry.insert(0, self.history[self.hist_i])

    def _note(self, text):
        self.sb_items["note"].configure(text=text, text_color=C["text"])
        self.after(5000, lambda: self.sb_items["note"].configure(text=""))

    # ------------------------------------------------------------------ attitude tools
    def _open_3d(self):
        script = os.path.join(TOOLS, "launch_3d_visualizer.py")
        threading.Thread(target=lambda: subprocess.run([sys.executable, script]), daemon=True).start()

    def _tare(self):
        self._send("TARE")

    def _north(self):
        if messagebox.askokcancel("Calibrate north",
                                  "Point the CanSat's +X axis at true north (use a phone compass), keep it still, "
                                  "then press OK. The offset is stored on the flight computer."):
            self._send("NORTH")

    # ------------------------------------------------------------------ flight log
    def _need_usb(self):
        if not self.link or self.link.kind != "usb":
            messagebox.showinfo("USB needed", "Downloading or erasing the flight log needs the USB link.")
            return False
        return True

    def _log_list(self):
        if not self.link:
            self._note("Not connected")
            return
        self.capture = ("list", "LOGLIST,END", [])
        self.log_status.configure(text="Reading session list...")
        self._send("LOG,LIST")

    def _log_download(self):
        if not self._need_usb():
            return
        sel = self.log_list.curselection()
        idx = (sel[0] + 1) if sel else 1
        path = filedialog.asksaveasfilename(
            title="Save flight log", defaultextension=".csv", initialdir=self.settings.get("log_dir"),
            initialfile=f"flight_{datetime.now():%Y%m%d_%H%M}_session{idx}.csv", filetypes=[("CSV", "*.csv")])
        if not path:
            return
        self.settings["log_dir"] = os.path.dirname(path)
        self._save_settings()
        self._dump_path = path
        self.capture = ("dump", "LOGEND", [])
        self.log_status.configure(text=f"Downloading session {idx}... (telemetry pauses meanwhile)")
        self._send(f"LOG,DUMP,{idx}")

    def _log_erase(self):
        if not self._need_usb():
            return
        if messagebox.askyesno("Erase flight log", "Erase all recorded sessions? This cannot be undone. "
                                                   "It takes about a minute and only works on the pad."):
            self._send("LOG,ERASE")
            self.log_status.configure(text="Erasing... refresh the list in a minute.")

    def _crash_report(self):
        if not self.link:
            self._note("Not connected")
            return
        self.capture = ("crash", "CRASH", [])
        self._send("LOG,CRASH")

    def _handle_capture(self, kind, lines):
        if kind == "list":
            self.log_list.delete(0, "end")
            sessions = [l for l in lines if re.match(r"LOGLIST,\d+,", l)]
            for l in sessions:
                f = l.split(",")
                self.log_list.insert("end", f"  #{f[1]:<3} {f[2]:<10} {f[3]:<16} {f[4]:<11} {f[5]:<8} {f[6]}  {','.join(f[7:])}")
            head = next((l for l in lines if l.startswith("LOGLIST,BEGIN")), "")
            hf = head.split(",")
            extra = f"   capacity {int(hf[3]) // 1024} MB, {int(hf[4]) / 1024:.1f} MB pre-erased" if len(hf) >= 5 else ""
            self.log_status.configure(text=f"{len(sessions)} session(s).{extra}")
        elif kind == "dump":
            hdr = next((l for l in lines if l.startswith("LOGHDR,")), None)
            rows = [l for l in lines if l.startswith("LOG,")]
            events = [l for l in lines if l.startswith("LOGEV,")]
            try:
                with open(self._dump_path, "w", newline="", encoding="utf-8") as f:
                    w = csv.writer(f)
                    w.writerow(hdr.split(",")[1:] if hdr else [])
                    for r in rows:
                        w.writerow(r.split(",")[1:])
                ev_path = os.path.splitext(self._dump_path)[0] + "_events.csv"
                with open(ev_path, "w", newline="", encoding="utf-8") as f:
                    w = csv.writer(f)
                    w.writerow(["t_ms", "phase", "event"])
                    for e in events:
                        parts = e.split(",", 3)
                        w.writerow(parts[1:])
                self.log_status.configure(text=f"Saved {len(rows)} records and {len(events)} events to {self._dump_path}")
            except Exception as e:
                self.log_status.configure(text=f"Could not save: {e}")
        elif kind == "crash":
            messagebox.showinfo("Crash report", "\n".join(l.replace("CRASH,", "") for l in lines) or "No crash dump")

    # ------------------------------------------------------------------ bench
    def _unlatch(self):
        if self.link and self.link.kind != "usb":
            self._note("Actuator commands need the USB link")
            return
        if messagebox.askyesno("Open arm latches", "Drive both linear servos to the unlatched position?"):
            self._send("CHUTE")

    def _motor_test(self, motor):
        if self.link and self.link.kind != "usb":
            self._note("Actuator commands need the USB link")
            return
        pct = int(self.mtr_slider.get())
        name = "all motors" if motor < 0 else f"motor {motor + 1}"
        if messagebox.askyesno("Motor test", f"Spin {name} at {pct} % for 3 seconds?\n\nProps must be OFF."):
            self._send(f"MTR,{'ALL' if motor < 0 else motor},{pct}")

    def _sim_toggle(self):
        if self.sim_running:
            self.sim_running = False
            return
        if not self.link:
            self._note("Not connected")
            return
        self.sim_running = True
        self.sim_btn.configure(text="Stop", image=icon("stop", 15, C["text"]))
        threading.Thread(target=self._sim_run, args=(self.sim_profile.get(),), daemon=True).start()

    def _sim_run(self, profile):
        p0 = self.tm["pressure"] or 101325.0
        if profile.startswith("Lift"):
            segs = [(5, 0, 0), (20, 0, 30), (15, 30, 30), (20, 30, 0), (10, 0, 0)]
            self._send("LIFT,10")
        else:
            segs = [(5, 0, 0), (30, 0, 750), (5, 750, 750), (62.5, 750, 0), (12, 0, 0)]
        self._send("SIM,ENABLE")
        time.sleep(0.5)
        for dur, h0, h1 in segs:
            n = int(dur * 20)
            for k in range(n):
                if not self.sim_running or not self.link:
                    break
                h = h0 + (h1 - h0) * k / n
                self.link.send(f"CMD,{TEAM},SIMP,{p0 * (1 - 2.25577e-5 * h) ** 5.25588:.1f}")
                time.sleep(0.05)
        if profile.startswith("Lift"):
            self._send("LIFT,OFF")
        self._send("SIM,DISABLE")
        self.sim_running = False
        self.after(0, lambda: self.sim_btn.configure(text="Run", image=icon("play", 15, C["text"])))

    # ------------------------------------------------------------------ firmware
    def _flash_log(self, text):
        self.after(0, lambda: (self.flash_box.insert("end", text), self.flash_box.see("end")))

    def _esptool(self, args, title):
        port = self.port_menu.get()
        if self.link:
            self._toggle_link()
            time.sleep(0.3)
        idf_py = r"C:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe"
        py = idf_py if os.path.exists(idf_py) else sys.executable
        cmd = [py, "-m", "esptool", "--chip", "esp32s3", "--port", port, "--baud", "460800"] + args

        def run():
            self._flash_log(f"\n{title}\n")
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=ROOT)
            for line in proc.stdout:
                self._flash_log(line)
            proc.wait()
            self._flash_log("Done.\n" if proc.returncode == 0 else f"esptool exited with {proc.returncode}\n")
        threading.Thread(target=run, daemon=True).start()

    def _flash(self):
        fa = os.path.join(ROOT, "build", "flasher_args.json")
        if not os.path.exists(fa):
            messagebox.showerror("No build", "build/flasher_args.json not found. Build the firmware first.")
            return
        with open(fa, encoding="utf-8") as f:
            args = json.load(f)
        fs = args.get("flash_settings", {})
        files = []
        for addr, rel in sorted(args.get("flash_files", {}).items(), key=lambda kv: int(kv[0], 16)):
            files += [addr, os.path.join(ROOT, "build", rel)]
        self._esptool(["--before", "default_reset", "--after", "hard_reset", "write_flash",
                       "--flash_mode", fs.get("flash_mode", "dio"), "--flash_freq", fs.get("flash_freq", "80m"),
                       "--flash_size", fs.get("flash_size", "16MB")] + files, "Flashing firmware from build/")

    def _chip_id(self):
        self._esptool(["flash_id"], "Reading chip information")

    def _erase_flash(self):
        if messagebox.askyesno("Erase entire flash", "Erase everything, including settings, the flight log and the "
                                                     "firmware? You will need to flash again afterwards."):
            self._esptool(["erase_flash"], "Erasing the entire flash")

    # ------------------------------------------------------------------ shutdown
    def _on_close(self):
        self.sim_running = False
        if self.tm_file:
            self.tm_file.close()
        if self.link:
            self.link.stop()
        self._save_settings()
        self.destroy()


def main():
    load_fonts()
    ctk.set_appearance_mode("dark")
    app = Dock()
    app.mainloop()


if __name__ == "__main__":
    main()
