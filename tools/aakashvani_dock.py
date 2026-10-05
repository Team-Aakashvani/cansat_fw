"""
AAKASHVANI - Mission Control & Flasher Dock
============================================
All-in-One GUI for:
1. ESP32 Flashing & Chip Identification (esptool)
2. Ground Station (GCS) Telemetry Live Visualizer & Telecommand Uplink
3. Flight Computer (FC) Diagnostics & Sensor Reaction Monitor
4. Sensorless Bench Testing & Hardware-In-The-Loop (HIL) Simulation
5. Universal Interactive Serial Terminal
"""

import sys
import os
import time
import json
import threading
import queue
import re
from datetime import datetime
from collections import deque
import math

import serial
import serial.tools.list_ports
import subprocess

try:
    import telemetry_3d_server
except ImportError:
    telemetry_3d_server = None

try:
    import customtkinter as ctk
    ctk.set_appearance_mode("Dark")
    ctk.set_default_color_theme("blue")
except ImportError:
    import tkinter as ctk

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

# =============================================================================

# CONSTANTS & PROTOCOL SPECS

# =============================================================================

STATE_NAMES = {
0: ("IDLE", "#7f8c8d"),
1: ("STANDBY", "#3498db"),
2: ("LAUNCH_PAD", "#3498db"),
3: ("ASCENT", "#e67e22"),
4: ("PARACHUTE", "#2ecc71"),
5: ("BURNOUT", "#9b59b6"),
6: ("DRONE_HOVER", "#1abc9c"),
7: ("LANDED", "#e74c3c")
}

TELEMETRY_FIELDS = [
"team_id", "mission_time", "packet_count", "altitude", "pressure",
"temperature", "voltage", "gnss_time", "latitude", "longitude",
"gnss_alt", "sats", "pitch", "roll", "yaw", "software_state", "freq", "rssi"
]

def wrap_diff_180(val, base):
    """Calculates shortest angular difference (val - base) mapped to [-180, +180]."""
    return ((val - base + 180.0) % 360.0) - 180.0

MOUNT_NAMES = {
    0: "Not referenced",
    1: "PARALLEL (Z up)",
    2: "PARALLEL (Z down)",
    3: "PERPENDICULAR (X up)",
    4: "PERPENDICULAR (X down)",
    5: "PERPENDICULAR (Y up)",
    6: "PERPENDICULAR (Y down)",
}

Q_IDENTITY = (1.0, 0.0, 0.0, 0.0)


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def quat_from_zxy(p_deg, r_deg, y_deg):
    """Three.js 'ZXY' Euler (tilt_x, tilt_y, heading) -> quaternion: Rz(y) Rx(p) Ry(r)."""
    hp, hr, hy = math.radians(p_deg) / 2, math.radians(r_deg) / 2, math.radians(y_deg) / 2
    qz = (math.cos(hy), 0.0, 0.0, math.sin(hy))
    qx = (math.cos(hp), math.sin(hp), 0.0, 0.0)
    qy = (math.cos(hr), 0.0, math.sin(hr), 0.0)
    return quat_mul(quat_mul(qz, qx), qy)


def euler_zxy(q):
    """Quaternion -> ZXY Euler degrees (tilt_x [-90,90], tilt_y (-180,180], heading [0,360))."""
    w, x, y, z = q
    m32 = max(-1.0, min(1.0, 2.0 * (y * z + w * x)))
    p = math.degrees(math.asin(m32))
    if abs(m32) < 0.999999999:
        r = math.degrees(math.atan2(-2.0 * (x * z - w * y), 1.0 - 2.0 * (x * x + y * y)))
        yaw = math.degrees(math.atan2(-2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z)))
    else:  # gimbal lock: only yaw +/- roll observable; report it all as heading
        r = 0.0
        yaw = math.degrees(math.atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z)))
    yaw %= 360.0
    if yaw >= 359.995:
        yaw = 0.0
    return p, r, yaw


def apply_tare(q_tare, q):
    """Operator zero as a quaternion (world-frame left multiply); never Euler subtraction,
    which is wrong for any combined rotation and breaks near +/-90 deg."""
    return quat_mul(q_tare, q)


    # =============================================================================

    # MAIN DOCK APPLICATION CLASS

    # =============================================================================

class AakashvaniDock(ctk.CTk if hasattr(ctk, 'CTk') else tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("AAKASHVANI — Mission Control & Flasher Dock (CAN-7USAT 2026)")
        self.geometry("1240x820")
        self.minsize(1050, 720)

        # Serial Connection
        self.ser = None
        self.serial_thread = None
        self.is_connected = False
        self.stop_serial = threading.Event()
        self.serial_rx_queue = queue.Queue()

        # Telemetry Data Buffers (for live plotting)
        self.data_history_len = 120
        self.time_buffer = deque(maxlen=self.data_history_len)
        self.alt_buffer = deque(maxlen=self.data_history_len)
        self.pres_buffer = deque(maxlen=self.data_history_len)
        self.temp_buffer = deque(maxlen=self.data_history_len)
        self.volt_buffer = deque(maxlen=self.data_history_len)
        self.state_buffer = deque(maxlen=self.data_history_len)

        # Environmental Telemetry Buffers (SHT4x & SGP41)
        self.hum_buffer = deque(maxlen=self.data_history_len)
        self.voc_buffer = deque(maxlen=self.data_history_len)
        self.nox_buffer = deque(maxlen=self.data_history_len)
        self.env_time_buffer = deque(maxlen=self.data_history_len)
        self.latest_temp = 32.5
        self.latest_hum = 52.4
        self.latest_voc = 100
        self.latest_nox = 1

        # Telemetry logging
        self.log_file = None
        self.is_logging = False

        # Live Telemetry Rate Meter
        self._rx_pkt_count = 0
        self._last_rate_time = time.time()

        # Attitude state. The firmware identifies the mount (parallel / perpendicular) and
        # references the attitude itself; q_tare is only an optional viewer-side zero.
        self.q_raw = Q_IDENTITY          # vehicle quaternion as received
        self.q_tare = Q_IDENTITY         # operator zero (quaternion, never Euler offsets)
        self.att_last_rx = 0.0           # host time of the last ATT line (quaternion stream)
        self.att_count = 0
        self.att_rate_hz = 0.0
        self._att_rate_t0 = time.time()
        self.att_mcu_ms = 0
        self.mount_name = MOUNT_NAMES[0]

        # Build UI Layout
        self._create_header()
        self._create_tabview()
        self._create_statusbar()

        # Polling Timer for Serial Queue and UI updates
        self.after(20, self._process_serial_queue)
        self.after(500, self._update_plots)

        # 3D Telemetry and WebSocket Server
        if telemetry_3d_server:
            try:
                telemetry_3d_server.start_server(http_port=8055, ws_port=8765)
            except Exception as e:
                print(f"[Dock] Note: 3D Server start: {e}")

    # -------------------------------------------------------------------------
    # Header & Navigation
    # -------------------------------------------------------------------------
    def _create_header(self):
        header_frame = ctk.CTkFrame(self, height=60, corner_radius=0)
        header_frame.pack(fill="x", side="top", padx=0, pady=0)

        title_lbl = ctk.CTkLabel(
            header_frame,
            text="AAKASHVANI MISSION CONTROL & DOCK",
            font=ctk.CTkFont(size=20, weight="bold")
        )
        title_lbl.pack(side="left", padx=20, pady=12)

        subtitle_lbl = ctk.CTkLabel(
            header_frame,
            text="SVNIT Aerospace | CAN-7USAT 2026",
            font=ctk.CTkFont(size=12, slant="italic"),
            text_color="gray"
        )
        subtitle_lbl.pack(side="left", padx=5, pady=12)

        # Connection Bar in Header
        conn_frame = ctk.CTkFrame(header_frame, fg_color="transparent")
        conn_frame.pack(side="right", padx=15, pady=8)

        self.port_combo = ctk.CTkComboBox(conn_frame, width=120, values=self._get_available_ports())
        self.port_combo.pack(side="left", padx=5)

        self.refresh_btn = ctk.CTkButton(conn_frame, text="↻", width=32, command=self._refresh_ports)
        self.refresh_btn.pack(side="left", padx=2)

        self.baud_combo = ctk.CTkComboBox(conn_frame, width=105, values=["115200", "921600", "57600", "9600"])
        self.baud_combo.set("115200")
        self.baud_combo.pack(side="left", padx=5)

        self.connect_btn = ctk.CTkButton(conn_frame, text="Connect", width=90, fg_color="#2ecc71", hover_color="#27ae60", command=self._toggle_connection)
        self.connect_btn.pack(side="left", padx=5)

    def _create_tabview(self):
        self.tabview = ctk.CTkTabview(self)
        self.tabview.pack(fill="both", expand=True, padx=15, pady=(5, 10))

        # Define Tabs
        self.tab_flasher = self.tabview.add("⚡ Flasher & Chip Manager")
        self.tab_gcs = self.tabview.add("📡 Ground Station (GCS)")
        self.tab_3d = self.tabview.add("🛰️ 3D IMU Visualizer")
        self.tab_env = self.tabview.add("🌿 Environmental & Calibration")
        self.tab_fc = self.tabview.add("🛰️️ Flight Computer (FC)")
        self.tab_hil = self.tabview.add("🧪 Sensorless Test & HIL")
        self.tab_terminal = self.tabview.add("💻 Serial Terminal")

        # Build each Tab
        self._build_flasher_tab()
        self._build_gcs_tab()
        self._build_3d_tab()
        self._build_env_tab()
        self._build_fc_tab()
        self._build_hil_tab()
        self._build_terminal_tab()

    def _create_statusbar(self):
        self.status_frame = ctk.CTkFrame(self, height=28, corner_radius=0)
        self.status_frame.pack(fill="x", side="bottom")

        self.status_lbl = ctk.CTkLabel(self.status_frame, text="Status: Ready | Port: Disconnected", font=ctk.CTkFont(size=11), text_color="gray")
        self.status_lbl.pack(side="left", padx=15, pady=2)

        self.packet_rate_lbl = ctk.CTkLabel(self.status_frame, text="Packets: 0 | 0.0 Hz", font=ctk.CTkFont(size=11), text_color="gray")
        self.packet_rate_lbl.pack(side="right", padx=15, pady=2)

    # -------------------------------------------------------------------------
    # TAB 1: FLASHER & CHIP MANAGER
    # -------------------------------------------------------------------------
    def _build_flasher_tab(self):
        parent = self.tab_flasher

        # Left Column: Configuration & Options
        left_box = ctk.CTkFrame(parent, width=420)
        left_box.pack(side="left", fill="both", padx=10, pady=10, expand=False)

        ctk.CTkLabel(left_box, text="Firmware Flasher", font=ctk.CTkFont(size=16, weight="bold")).pack(anchor="w", padx=15, pady=(15, 5))
        ctk.CTkLabel(left_box, text="Target Firmware Role:", font=ctk.CTkFont(size=13, weight="bold")).pack(anchor="w", padx=15, pady=(10, 2))

        self.role_var = tk.StringVar(value="FC")
        r1 = ctk.CTkRadioButton(left_box, text="Flight Computer / Transmitter (ROLE_FC)", variable=self.role_var, value="FC")
        r1.pack(anchor="w", padx=20, pady=4)
        r2 = ctk.CTkRadioButton(left_box, text="Ground Station Receiver Bridge (ROLE_GCS)", variable=self.role_var, value="GCS")
        r2.pack(anchor="w", padx=20, pady=4)

        # Options Checkboxes
        ctk.CTkLabel(left_box, text="Build & Flash Options:", font=ctk.CTkFont(size=13, weight="bold")).pack(anchor="w", padx=15, pady=(15, 2))

        self.bit_bypass_var = tk.BooleanVar(value=True)
        cb_bit = ctk.CTkCheckBox(left_box, text="Sensorless Bench Mode (BIT Bypass)", variable=self.bit_bypass_var)
        cb_bit.pack(anchor="w", padx=20, pady=4)

        self.erase_flash_var = tk.BooleanVar(value=False)
        cb_erase = ctk.CTkCheckBox(left_box, text="Erase Entire Flash Before Flashing", variable=self.erase_flash_var)
        cb_erase.pack(anchor="w", padx=20, pady=4)

        # Chip Operations Buttons
        ctk.CTkLabel(left_box, text="Hardware Actions:", font=ctk.CTkFont(size=13, weight="bold")).pack(anchor="w", padx=15, pady=(15, 2))

        btn_chip_info = ctk.CTkButton(left_box, text="🔍 Detect Chip & Flash Info", fg_color="#34495e", hover_color="#2c3e50", command=self._action_detect_chip)
        btn_chip_info.pack(fill="x", padx=20, pady=6)

        btn_flash = ctk.CTkButton(left_box, text="⚡ FLASH TARGET FIRMWARE", fg_color="#2980b9", hover_color="#1f618d", font=ctk.CTkFont(size=14, weight="bold"), height=40, command=self._action_flash_firmware)
        btn_flash.pack(fill="x", padx=20, pady=12)

        btn_erase = ctk.CTkButton(left_box, text="⚠️ Erase Entire Flash", fg_color="#c0392b", hover_color="#962d22", command=self._action_erase_flash)
        btn_erase.pack(fill="x", padx=20, pady=4)

        # Right Column: Live Output Console
        right_box = ctk.CTkFrame(parent)
        right_box.pack(side="right", fill="both", padx=10, pady=10, expand=True)

        ctk.CTkLabel(right_box, text="Flasher & Hardware Diagnostics Log", font=ctk.CTkFont(size=14, weight="bold")).pack(anchor="w", padx=15, pady=(15, 5))

        self.flash_log_txt = ctk.CTkTextbox(right_box, font=("Consolas", 11), wrap="char")
        self.flash_log_txt.pack(fill="both", expand=True, padx=15, pady=(5, 15))

    # -------------------------------------------------------------------------
    # TAB 2: GROUND STATION (GCS) TELEMETRY DOCK
    # -------------------------------------------------------------------------
    def _build_gcs_tab(self):
        parent = self.tab_gcs

        # Left Column: Metrics Grid & Flight State Indicator
        left_col = ctk.CTkFrame(parent, width=380)
        left_col.pack(side="left", fill="both", padx=10, pady=10, expand=False)

        # Flight State Card
        state_card = ctk.CTkFrame(left_col, fg_color="#1c2833")
        state_card.pack(fill="x", padx=10, pady=8)

        ctk.CTkLabel(state_card, text="CURRENT FLIGHT PHASE", font=ctk.CTkFont(size=11, weight="bold"), text_color="gray").pack(pady=(8, 0))
        self.state_badge = ctk.CTkLabel(state_card, text="PRE_FLIGHT (0)", font=ctk.CTkFont(size=18, weight="bold"), text_color="#3498db")
        self.state_badge.pack(pady=(2, 8))

        # Real-time Metrics Grid
        metrics_box = ctk.CTkFrame(left_col)
        metrics_box.pack(fill="both", expand=True, padx=10, pady=5)

        self.metric_labels = {}
        fields_display = [
            ("Altitude (AGL)", "0.00 m", "alt"),
            ("Pressure", "101325 Pa", "pres"),
            ("Temperature", "32.5 °C", "temp"),
            ("Relative Humidity", "52.4 %", "hum"),
            ("VOC Air Quality", "100 (Clean)", "voc"),
            ("NOx Gas Index", "1 (Normal)", "nox"),
            ("Battery Voltage", "7.40 V", "volt"),
            ("Mission Time", "00:00:00", "mtime"),
            ("Packet Count", "0", "pcount"),
            ("GNSS Satellites", "0", "sats"),
            ("Latitude / Long", "0.0000, 0.0000", "gps"),
            ("Tilt (Pitch/Roll)", "0.0°, 0.0°", "tilt"),
            ("Rotation Z (Heading)", "0.0°", "rotz"),
            ("Sensor Constellation", "4/4 Active", "sensors")
        ]

        for idx, (title, default_val, key) in enumerate(fields_display):
            row = idx // 2
            col = idx % 2
            card = ctk.CTkFrame(metrics_box, fg_color="#212f3d", corner_radius=6)
            card.grid(row=row, column=col, padx=5, pady=5, sticky="nsew")
            metrics_box.grid_columnconfigure(col, weight=1)

            ctk.CTkLabel(card, text=title, font=ctk.CTkFont(size=10), text_color="gray").pack(anchor="w", padx=8, pady=(4, 0))
            lbl = ctk.CTkLabel(card, text=default_val, font=ctk.CTkFont(size=14, weight="bold"))
            lbl.pack(anchor="w", padx=8, pady=(0, 4))
            self.metric_labels[key] = lbl

        # Telecommand Uplink Box
        cmd_box = ctk.CTkFrame(left_col)
        cmd_box.pack(fill="x", padx=10, pady=8)

        ctk.CTkLabel(cmd_box, text="Telecommand Uplink (GCS ➔ FC)", font=ctk.CTkFont(size=12, weight="bold")).pack(anchor="w", padx=10, pady=(6, 4))

        quick_cmds_frame = ctk.CTkFrame(cmd_box, fg_color="transparent")
        quick_cmds_frame.pack(fill="x", padx=5, pady=2)

        ctk.CTkButton(quick_cmds_frame, text="CAL (Zero Alt)", width=95, command=lambda: self._send_command("1234,CAL\n")).pack(side="left", padx=3)
        ctk.CTkButton(quick_cmds_frame, text="CX ON", width=70, command=lambda: self._send_command("1234,CX,ON\n")).pack(side="left", padx=3)
        ctk.CTkButton(quick_cmds_frame, text="CX OFF", width=70, command=lambda: self._send_command("1234,CX,OFF\n")).pack(side="left", padx=3)
        ctk.CTkButton(quick_cmds_frame, text="ABORT", width=75, fg_color="#c0392b", hover_color="#962d22", command=lambda: self._send_command("1234,ABORT\n")).pack(side="left", padx=3)
        ctk.CTkButton(quick_cmds_frame, text="🛰️ 3D Body", width=80, fg_color="#8e44ad", hover_color="#732d91", command=self._open_3d_viewer).pack(side="left", padx=3)

        # Right Column: Real-Time Matplotlib Telemetry Plots
        right_col = ctk.CTkFrame(parent)
        right_col.pack(side="right", fill="both", padx=10, pady=10, expand=True)

        if MATPLOTLIB_AVAILABLE:
            self.fig = Figure(figsize=(6, 5), dpi=100, facecolor='#2b2b2b')
            self.ax1 = self.fig.add_subplot(211)
            self.ax2 = self.fig.add_subplot(212)

            # --- TOP PLOT: Altitude with Phase Thresholds ---
            self.line_alt, = self.ax1.plot([], [], color="#00d2d3", linewidth=2.0, label="Altitude (m AGL)")
            self.ax1.axhline(600.0, color="#2ecc71", linestyle=":", alpha=0.7, label="Chute (600m)")
            self.ax1.axhline(350.0, color="#1abc9c", linestyle="--", alpha=0.7, label="PID Hover (350m)")
            self.ax1.axhline(0.0, color="#7f8c8d", linestyle="-", alpha=0.5)
            self.ax1.legend(loc="upper right", fontsize=7)
            self._style_axis(self.ax1, "Altitude AGL (m)", "#00d2d3")

            # --- BOTTOM PLOT: Flight Computer Mission State Ladder ---
            self.line_state, = self.ax2.step([], [], where="post", color="#f39c12", linewidth=2.0, label="Flight State Code")
            self.ax2.set_yticks([2, 3, 4, 6, 7])
            self.ax2.set_yticklabels(["2: PAD", "3: ASCENT", "4: CHUTE", "6: HOVER", "7: LAND"])
            self.ax2.set_ylim(1.5, 7.5)
            self.ax2.legend(loc="upper right", fontsize=7)
            self._style_axis(self.ax2, "Mission State", "#f39c12")

            self.fig.tight_layout()

            self.canvas = FigureCanvasTkAgg(self.fig, master=right_col)
            self.canvas.draw()
            self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=5, pady=5)
        else:
            ctk.CTkLabel(right_col, text="Matplotlib not installed. Telemetry graphs unavailable.").pack(pady=50)

    def _style_axis(self, ax, ylabel, color):
        ax.set_facecolor('#1f1f1f')
        ax.tick_params(colors='white', labelsize=8)
        ax.set_ylabel(ylabel, color=color, fontsize=9, weight='bold')
        ax.grid(True, linestyle='--', alpha=0.3, color='gray')
        for spine in ax.spines.values():
            spine.set_color('#444444')

    # -------------------------------------------------------------------------
    # TAB: 3D ASSEMBLED BODY & IMU ATTITUDE VISUALIZER
    # -------------------------------------------------------------------------
    def _build_3d_tab(self):
        parent = self.tab_3d

        # Left Column: Real-time Attitude Readouts & Launcher Controls
        left_box = ctk.CTkFrame(parent, width=410)
        left_box.pack(side="left", fill="both", padx=10, pady=10, expand=False)

        ctk.CTkLabel(left_box, text="3D CanSat Body & IMU Attitude", font=ctk.CTkFont(size=16, weight="bold")).pack(anchor="w", padx=15, pady=(15, 3))
        ctk.CTkLabel(left_box, text="Live orientation tracking with assembled REV_H CAD body", font=ctk.CTkFont(size=11), text_color="gray").pack(anchor="w", padx=15, pady=(0, 8))

        # Attitude Angle Badges Card
        angles_frame = ctk.CTkFrame(left_box, fg_color="#1c2833")
        angles_frame.pack(fill="x", padx=10, pady=6)

        grid_angles = ctk.CTkFrame(angles_frame, fg_color="transparent")
        grid_angles.pack(fill="x", padx=8, pady=8)

        # Pitch
        c_pitch = ctk.CTkFrame(grid_angles, fg_color="#212f3d", corner_radius=6)
        c_pitch.grid(row=0, column=0, padx=4, pady=4, sticky="nsew")
        ctk.CTkLabel(c_pitch, text="PITCH (X)", font=ctk.CTkFont(size=10, weight="bold"), text_color="#38bdf8").pack(pady=(4,0))
        self.lbl_3d_pitch = ctk.CTkLabel(c_pitch, text="+0.0°", font=ctk.CTkFont(size=18, weight="bold"), text_color="#38bdf8")
        self.lbl_3d_pitch.pack(pady=(0,4))

        # Roll
        c_roll = ctk.CTkFrame(grid_angles, fg_color="#212f3d", corner_radius=6)
        c_roll.grid(row=0, column=1, padx=4, pady=4, sticky="nsew")
        ctk.CTkLabel(c_roll, text="ROLL (Y)", font=ctk.CTkFont(size=10, weight="bold"), text_color="#2ecc71").pack(pady=(4,0))
        self.lbl_3d_roll = ctk.CTkLabel(c_roll, text="+0.0°", font=ctk.CTkFont(size=18, weight="bold"), text_color="#2ecc71")
        self.lbl_3d_roll.pack(pady=(0,4))

        # Yaw
        c_yaw = ctk.CTkFrame(grid_angles, fg_color="#212f3d", corner_radius=6)
        c_yaw.grid(row=0, column=2, padx=4, pady=4, sticky="nsew")
        ctk.CTkLabel(c_yaw, text="YAW (Z)", font=ctk.CTkFont(size=10, weight="bold"), text_color="#f39c12").pack(pady=(4,0))
        self.lbl_3d_yaw = ctk.CTkLabel(c_yaw, text="0.0°", font=ctk.CTkFont(size=18, weight="bold"), text_color="#f39c12")
        self.lbl_3d_yaw.pack(pady=(0,4))

        grid_angles.grid_columnconfigure(0, weight=1)
        grid_angles.grid_columnconfigure(1, weight=1)
        grid_angles.grid_columnconfigure(2, weight=1)

        # IMU mount: auto-detected by the firmware from gravity at boot / on Zero Tare
        orient_frame = ctk.CTkFrame(left_box)
        orient_frame.pack(fill="x", padx=10, pady=6)
        ctk.CTkLabel(orient_frame, text="IMU Mount (auto-detected at boot):", font=ctk.CTkFont(size=12, weight="bold")).pack(anchor="w", padx=10, pady=(6, 2))
        self.lbl_mount = ctk.CTkLabel(orient_frame, text=self.mount_name, font=ctk.CTkFont(size=13, weight="bold"), text_color="#f39c12")
        self.lbl_mount.pack(anchor="w", padx=12, pady=(0, 2))
        self.lbl_att_link = ctk.CTkLabel(orient_frame, text="Attitude stream: waiting...", font=ctk.CTkFont(size=11), text_color="gray")
        self.lbl_att_link.pack(anchor="w", padx=12, pady=(0, 6))

        # Launch 3D Viewer Actions
        action_frame = ctk.CTkFrame(left_box)
        action_frame.pack(fill="x", padx=10, pady=8)

        ctk.CTkLabel(action_frame, text="Interactive 3D Visualizer Window:", font=ctk.CTkFont(size=12, weight="bold")).pack(anchor="w", padx=10, pady=(8, 4))

        btn_pyqt = ctk.CTkButton(action_frame, text="🚀 Launch 3D Desktop Window", fg_color="#2980b9", hover_color="#1f618d", font=ctk.CTkFont(size=13, weight="bold"), height=36, command=self._open_3d_viewer)
        btn_pyqt.pack(fill="x", padx=10, pady=5)

        btn_browser = ctk.CTkButton(action_frame, text="🌐 Open in Web Browser (Chrome/Edge)", fg_color="#34495e", hover_color="#2c3e50", command=self._open_3d_browser)
        btn_browser.pack(fill="x", padx=10, pady=3)

        self.lbl_3d_server_status = ctk.CTkLabel(action_frame, text="● 3D Stream: ws://127.0.0.1:8765", font=ctk.CTkFont(size=11), text_color="#2ecc71")
        self.lbl_3d_server_status.pack(pady=(4, 6))

        # Baseline Calibration & Zero Tare Controls
        tare_frame = ctk.CTkFrame(left_box)
        tare_frame.pack(fill="x", padx=10, pady=6)

        ctk.CTkLabel(tare_frame, text="Attitude Baseline Calibration:", font=ctk.CTkFont(size=12, weight="bold")).pack(anchor="w", padx=10, pady=(6, 4))

        tare_btns = ctk.CTkFrame(tare_frame, fg_color="transparent")
        tare_btns.pack(fill="x", padx=5, pady=2)

        ctk.CTkButton(tare_btns, text="⚖️ Zero Tare (Set 0°)", fg_color="#27ae60", hover_color="#1e8449", command=self._zero_tare_imu).pack(side="left", padx=3, fill="x", expand=True)
        ctk.CTkButton(tare_btns, text="Reset Tare", fg_color="#7f8c8d", hover_color="#626567", command=self._reset_tare_imu).pack(side="left", padx=3, fill="x", expand=True)

        # Bench Simulation & Testing Controls
        test_frame = ctk.CTkFrame(left_box)
        test_frame.pack(fill="x", padx=10, pady=6)

        ctk.CTkLabel(test_frame, text="Sensorless Bench IMU Motion Tests:", font=ctk.CTkFont(size=12, weight="bold")).pack(anchor="w", padx=10, pady=(6, 4))

        test_btns = ctk.CTkFrame(test_frame, fg_color="transparent")
        test_btns.pack(fill="x", padx=5, pady=2)

        ctk.CTkButton(test_btns, text="Level (0,0)", width=80, command=lambda: self._test_inject_angles(0, 0, 0)).pack(side="left", padx=2)
        ctk.CTkButton(test_btns, text="Pitch +30°", width=80, command=lambda: self._test_inject_angles(30, 0, 0)).pack(side="left", padx=2)
        ctk.CTkButton(test_btns, text="Roll +45°", width=80, command=lambda: self._test_inject_angles(0, 45, 0)).pack(side="left", padx=2)
        ctk.CTkButton(test_btns, text="Nose Down", width=80, command=lambda: self._test_inject_angles(-45, 0, 0)).pack(side="left", padx=2)

        self.btn_sine_test = ctk.CTkButton(test_frame, text="▶ Start Automated Sine Motion Wave", fg_color="#8e44ad", hover_color="#732d91", command=self._toggle_sine_test)
        self.btn_sine_test.pack(fill="x", padx=10, pady=(6, 8))

        # Right Column: Model Info & Subsystem Hierarchy
        right_box = ctk.CTkFrame(parent)
        right_box.pack(side="right", fill="both", padx=10, pady=10, expand=True)

        ctk.CTkLabel(right_box, text="3D CAD Vehicle Integration & Subsystem Hierarchy", font=ctk.CTkFont(size=14, weight="bold")).pack(anchor="w", padx=15, pady=(15, 5))

        info_txt = (
            "CAN-7USAT REV_H Mechanical Architecture Details:\n\n"
            "• Assembled 3D CAD Body: Final_Body_Cansat / cansat_assembly.glb (406mm H x 154mm Ø)\n"
            "• Subsystems Integrated in 3D Visualization:\n"
            "    - Module 01: Outer NACA Fin Structure & Aerodynamic Hull\n"
            "    - Module 02: Quadcopter Drone Deployment Mechanism (Arms, Actuator Rod, Pins)\n"
            "    - Module 03: Turbine Generator System (Dual Counter-Rotating Rotors, Bushings)\n"
            "    - Module 04: Parachute Recovery System (Parachute Bay & Ejection Lid)\n"
            "    - Module 05: Electronics Bay (Monolithic Cage, Battery Rack, Pitot Mount, Camera)\n\n"
            "Real-Time Dynamic Attitude Tracking:\n"
            "• Center-of-Mass Pivot: Exact geometric centroid offset at Z=197mm for realistic physical rotation\n"
            "• Attitude: firmware quaternion (ATT, 50 Hz, MCU-timestamped); Euler ZXY for readouts only\n"
            "• Render Engine: Three.js WebGL, quaternion SLERP on the MCU time base (no gimbal-lock spins)\n"
            "• Broadcast Server: Zero-latency local WebSocket pipeline at ws://127.0.0.1:8765\n"
        )
        lbl_info = ctk.CTkLabel(right_box, text=info_txt, font=ctk.CTkFont(size=12), justify="left", text_color="#bdc3c7")
        lbl_info.pack(anchor="nw", padx=15, pady=10)

    def _open_3d_viewer(self):
        script = os.path.join(os.path.dirname(__file__), "launch_3d_visualizer.py")
        threading.Thread(target=lambda: subprocess.run([sys.executable, script]), daemon=True).start()

    def _open_3d_browser(self):
        import webbrowser
        webbrowser.open("http://localhost:8055/")

    def _zero_tare_imu(self):
        if self.is_connected and (time.time() - self.att_last_rx) < 1.0:
            # Firmware re-identifies the mount, re-levels and zeroes heading on the IMU itself
            self.q_tare = Q_IDENTITY
            self._send_command("CMD,1234,TARE\r\n")
        else:
            self.q_tare = quat_conj(self.q_raw)
        self._update_3d_tab_readouts(0.0, 0.0, 0.0)

    def _reset_tare_imu(self):
        self.q_tare = Q_IDENTITY
        p, r, y = euler_zxy(self.q_raw)
        self._update_3d_tab_readouts(p, r, y)

    def _test_inject_angles(self, p, r, y):
        self.att_last_rx = 0.0
        self.q_raw = quat_from_zxy(float(p), float(r), float(y))
        eff_pitch, eff_roll, eff_yaw = euler_zxy(apply_tare(self.q_tare, self.q_raw))
        if telemetry_3d_server:
            telemetry_3d_server.update_telemetry(pitch=eff_pitch, roll=eff_roll, yaw=eff_yaw, alt=25.0, state=2, state_name="ON_PAD", pcount=1)
        self._update_3d_tab_readouts(eff_pitch, eff_roll, eff_yaw, alt=25.0, state=2, state_str="ON_PAD")

    def _toggle_sine_test(self):
        if not hasattr(self, '_sine_test_active'):
            self._sine_test_active = False

        self._sine_test_active = not self._sine_test_active
        if self._sine_test_active:
            self.btn_sine_test.configure(text="⏹ Stop Automated Sine Motion Wave", fg_color="#c0392b", hover_color="#962d22")
            threading.Thread(target=self._run_sine_test_loop, daemon=True).start()
        else:
            self.btn_sine_test.configure(text="▶ Start Automated Sine Motion Wave", fg_color="#8e44ad", hover_color="#732d91")

    def _run_sine_test_loop(self):
        t = 0.0
        pcount = 0
        while getattr(self, '_sine_test_active', False):
            t += 0.05
            pcount += 1
            p = 25.0 * math.sin(t)
            r = 40.0 * math.cos(t * 0.8)
            y = (t * 20.0) % 360.0
            alt = 150.0 + 30.0 * math.sin(t * 0.5)
            pres = 101325.0 * ((1.0 - 2.25577e-5 * alt) ** 5.25588)
            if telemetry_3d_server:
                telemetry_3d_server.update_telemetry(pitch=p, roll=r, yaw=y, alt=alt, pres=pres, state=3, state_name="ASCENT", pcount=pcount)
            self.after(0, lambda p=p, r=r, y=y, alt=alt, pres=pres: self._update_3d_tab_readouts(p, r, y, alt=alt, pres=pres, state=3, state_str="ASCENT"))
            time.sleep(0.04)

    def _update_3d_tab_readouts(self, p, r, y, alt=None, pres=None, state=None, state_str=None):
        if hasattr(self, 'lbl_3d_pitch'):
            sign_p = "+" if p >= 0 else ""
            sign_r = "+" if r >= 0 else ""
            self.lbl_3d_pitch.configure(text=f"{sign_p}{p:.1f}°")
            self.lbl_3d_roll.configure(text=f"{sign_r}{r:.1f}°")
            self.lbl_3d_yaw.configure(text=f"{y:.1f}°")
        if hasattr(self, 'metric_labels'):
            if "tilt" in self.metric_labels:
                self.metric_labels["tilt"].configure(text=f"{p:.1f}°, {r:.1f}°")
            if "rotz" in self.metric_labels:
                self.metric_labels["rotz"].configure(text=f"{y:.1f}°")
            if alt is not None and "alt" in self.metric_labels:
                self.metric_labels["alt"].configure(text=f"{alt:.2f} m")
            if pres is not None and "pres" in self.metric_labels:
                self.metric_labels["pres"].configure(text=f"{pres:.1f} Pa")
        if state is not None and hasattr(self, 'state_badge'):
            state_info = STATE_NAMES.get(state, (state_str or f"STATE_{state}", "#3498db"))
            self.state_badge.configure(text=f"{state_info[0]} ({state})", text_color=state_info[1])

    # -------------------------------------------------------------------------
    # TAB: ENVIRONMENTAL TELEMETRY & GAS SENSOR CALIBRATION CURVES
    # -------------------------------------------------------------------------
    def _build_env_tab(self):
        parent = self.tab_env

        # Left Column: KPI Cards & Physical Models
        left_box = ctk.CTkFrame(parent, width=380)
        left_box.pack(side="left", fill="both", padx=10, pady=10, expand=False)

        ctk.CTkLabel(left_box, text="🌿 Environmental Suite & Gas Index", font=ctk.CTkFont(size=16, weight="bold")).pack(anchor="w", padx=15, pady=(15, 3))
        ctk.CTkLabel(left_box, text="Sensirion SHT4x (RHT) & SGP41 (VOC/NOx) Sensors", font=ctk.CTkFont(size=11), text_color="gray").pack(anchor="w", padx=15, pady=(0, 10))

        # 4 Environmental Metric Cards in Left Column
        cards_frame = ctk.CTkFrame(left_box, fg_color="transparent")
        cards_frame.pack(fill="x", padx=10, pady=5)

        # 1. Temperature Card
        c_temp = ctk.CTkFrame(cards_frame, fg_color="#212f3d", corner_radius=6)
        c_temp.pack(fill="x", pady=4)
        ctk.CTkLabel(c_temp, text="AMBIENT TEMPERATURE (SHT4x)", font=ctk.CTkFont(size=10, weight="bold"), text_color="#ff7675").pack(anchor="w", padx=10, pady=(6, 0))
        self.env_lbl_temp = ctk.CTkLabel(c_temp, text="32.5 °C", font=ctk.CTkFont(size=20, weight="bold"), text_color="#ff7675")
        self.env_lbl_temp.pack(anchor="w", padx=10, pady=(0, 2))
        ctk.CTkLabel(c_temp, text="Range: -40°C to +125°C | Precision: ±0.2°C", font=ctk.CTkFont(size=9), text_color="gray").pack(anchor="w", padx=10, pady=(0, 6))

        # 2. Relative Humidity Card
        c_hum = ctk.CTkFrame(cards_frame, fg_color="#212f3d", corner_radius=6)
        c_hum.pack(fill="x", pady=4)
        ctk.CTkLabel(c_hum, text="RELATIVE HUMIDITY (SHT4x)", font=ctk.CTkFont(size=10, weight="bold"), text_color="#00d2d3").pack(anchor="w", padx=10, pady=(6, 0))
        self.env_lbl_hum = ctk.CTkLabel(c_hum, text="52.4 %", font=ctk.CTkFont(size=20, weight="bold"), text_color="#00d2d3")
        self.env_lbl_hum.pack(anchor="w", padx=10, pady=(0, 2))
        ctk.CTkLabel(c_hum, text="Range: 0% to 100% RH | Precision: ±1.8% RH", font=ctk.CTkFont(size=9), text_color="gray").pack(anchor="w", padx=10, pady=(0, 6))

        # 3. VOC Index Card
        c_voc = ctk.CTkFrame(cards_frame, fg_color="#212f3d", corner_radius=6)
        c_voc.pack(fill="x", pady=4)
        ctk.CTkLabel(c_voc, text="VOC AIR QUALITY INDEX (SGP41)", font=ctk.CTkFont(size=10, weight="bold"), text_color="#2ecc71").pack(anchor="w", padx=10, pady=(6, 0))
        self.env_lbl_voc = ctk.CTkLabel(c_voc, text="100 (Clean Air)", font=ctk.CTkFont(size=20, weight="bold"), text_color="#2ecc71")
        self.env_lbl_voc.pack(anchor="w", padx=10, pady=(0, 2))
        ctk.CTkLabel(c_voc, text="Log Scale: 1-500 (100 = Clean Air Baseline)", font=ctk.CTkFont(size=9), text_color="gray").pack(anchor="w", padx=10, pady=(0, 6))

        # 4. NOx Index Card
        c_nox = ctk.CTkFrame(cards_frame, fg_color="#212f3d", corner_radius=6)
        c_nox.pack(fill="x", pady=4)
        ctk.CTkLabel(c_nox, text="NOx GAS INDEX (SGP41)", font=ctk.CTkFont(size=10, weight="bold"), text_color="#f39c12").pack(anchor="w", padx=10, pady=(6, 0))
        self.env_lbl_nox = ctk.CTkLabel(c_nox, text="1 (Normal Ambient)", font=ctk.CTkFont(size=20, weight="bold"), text_color="#f39c12")
        self.env_lbl_nox.pack(anchor="w", padx=10, pady=(0, 2))
        ctk.CTkLabel(c_nox, text="Scale: 1-500 (1 = Negligible / Normal Air)", font=ctk.CTkFont(size=9), text_color="gray").pack(anchor="w", padx=10, pady=(0, 6))

        # Mathematical Calibration Equations Box
        math_box = ctk.CTkFrame(left_box)
        math_box.pack(fill="both", expand=True, padx=10, pady=8)
        ctk.CTkLabel(math_box, text="Sensirion Gas Calibration Model:", font=ctk.CTkFont(size=12, weight="bold")).pack(anchor="w", padx=10, pady=(8, 4))

        formula_text = (
            "• SGP41 VOC Transfer Function:\n"
            "  Index_VOC = 100 · (S_raw / S_base)^(-2.5)\n"
            "  [Normalized to Baseline: Index=100]\n\n"
            "• SGP41 NOx Response Model:\n"
            "  Index_NOx = 1 + 499 · (1 - exp(-5.0·Δs))\n"
            "  [Normalized to Clean Air: Index=1]\n\n"
            "• SHT4x Temperature & Humidity Linearity:\n"
            "  T(°C) = -45 + 175 · (S_T / 65535)\n"
            "  RH(%) = -6 + 125 · (S_RH / 65535)\n\n"
            "• Zero ADC Artifacts: Calibrated in physical units."
        )
        ctk.CTkLabel(math_box, text=formula_text, font=ctk.CTkFont(size=10), text_color="#bdc3c7", justify="left").pack(anchor="w", padx=10, pady=(0, 8))

        # Right Column: Matplotlib Real-time Trends & Calibration Curves
        right_box = ctk.CTkFrame(parent)
        right_box.pack(side="right", fill="both", padx=10, pady=10, expand=True)

        if MATPLOTLIB_AVAILABLE:
            self.env_fig = Figure(figsize=(7, 6), dpi=100, facecolor='#2b2b2b')
            # 2 Rows: Top = Live Time Trends, Bottom = 2 Calibration Curves (VOC & NOx)
            self.ax_env_trend = self.env_fig.add_subplot(211)
            self.ax_voc_curve = self.env_fig.add_subplot(223)
            self.ax_nox_curve = self.env_fig.add_subplot(224)

            # Top: Live Environmental Trend
            self.line_env_hum, = self.ax_env_trend.plot([], [], color="#00d2d3", linewidth=2.0, label="Humidity (% RH)")
            self.line_env_temp, = self.ax_env_trend.plot([], [], color="#ff7675", linewidth=1.8, label="Temperature (°C)")
            self.line_env_voc, = self.ax_env_trend.plot([], [], color="#2ecc71", linewidth=1.8, linestyle="--", label="VOC Index (1-500)")
            self.line_env_nox, = self.ax_env_trend.plot([], [], color="#f39c12", linewidth=1.5, linestyle=":", label="NOx Index (1-500)")
            self.ax_env_trend.legend(loc="upper right", fontsize=8)
            self._style_axis(self.ax_env_trend, "Live Trend Readings", "#00d2d3")

            # Bottom Left: VOC Calibration Curve
            x_voc_vals = [0.4 + i * (0.9 / 80) for i in range(81)]
            y_voc_vals = [min(500.0, max(1.0, 100.0 * (x ** -2.5))) for x in x_voc_vals]
            self.ax_voc_curve.plot(x_voc_vals, y_voc_vals, color="#2ecc71", linewidth=2.0, label="VOC Characteristic")
            self.ax_voc_curve.axvline(1.0, color="gray", linestyle=":", alpha=0.6, label="Base (Index=100)")
            self.ax_voc_curve.axhline(100.0, color="gray", linestyle=":", alpha=0.6)
            self.dot_voc, = self.ax_voc_curve.plot([1.0], [100.0], 'ro', markersize=8, markeredgecolor='white', label="Operating Point")
            self.ax_voc_curve.set_xlabel("Signal Ratio (S_raw / S_base)", color="white", fontsize=8)
            self.ax_voc_curve.set_ylim(0, 520)
            self.ax_voc_curve.legend(loc="upper right", fontsize=7)
            self._style_axis(self.ax_voc_curve, "VOC Index (1-500)", "#2ecc71")

            # Bottom Right: NOx Calibration Curve
            x_nox_vals = [1.0 + i * (0.6 / 80) for i in range(81)]
            y_nox_vals = [min(500.0, max(1.0, 1.0 + 499.0 * (1.0 - math.exp(-5.0 * (x - 1.0))))) for x in x_nox_vals]
            self.ax_nox_curve.plot(x_nox_vals, y_nox_vals, color="#f39c12", linewidth=2.0, label="NOx Characteristic")
            self.ax_nox_curve.axvline(1.0, color="gray", linestyle=":", alpha=0.6, label="Clean (Index=1)")
            self.dot_nox, = self.ax_nox_curve.plot([1.0], [1.0], 'yo', markersize=8, markeredgecolor='white', label="Operating Point")
            self.ax_nox_curve.set_xlabel("Signal Ratio (S_raw / S_base)", color="white", fontsize=8)
            self.ax_nox_curve.set_ylim(0, 520)
            self.ax_nox_curve.legend(loc="lower right", fontsize=7)
            self._style_axis(self.ax_nox_curve, "NOx Index (1-500)", "#f39c12")

            self.env_fig.tight_layout()
            self.env_canvas = FigureCanvasTkAgg(self.env_fig, master=right_box)
            self.env_canvas.draw()
            self.env_canvas.get_tk_widget().pack(fill="both", expand=True, padx=5, pady=5)
        else:
            ctk.CTkLabel(right_box, text="Matplotlib not installed. Graphs unavailable.").pack(pady=50)

    # -------------------------------------------------------------------------
    # TAB: FLIGHT COMPUTER (FC) DIAGNOSTICS & MOTOR REACTIONS
    # -------------------------------------------------------------------------
    def _build_fc_tab(self):
        parent = self.tab_fc

        # Grid of Subsystem Health Checks
        health_frame = ctk.CTkFrame(parent)
        health_frame.pack(fill="x", padx=15, pady=10)

        ctk.CTkLabel(health_frame, text="Onboard Built-In Test (BIT) Subsystem Status", font=ctk.CTkFont(size=14, weight="bold")).pack(anchor="w", padx=15, pady=(10, 5))

        chips = [
            ("IMU (BNO085 / BNO055)", "imu_status"),
            ("Barometer (BMP585)", "baro_status"),
            ("GNSS / NavIC (N-GS-01)", "gnss_status"),
            ("Humidity / Temp (SHT4x)", "sht_status"),
            ("Air Quality VOC/NOx (SGP41)", "sgp_status"),
            ("Power Monitor (INA260)", "power_status"),
            ("Blackbox Storage (SD Card)", "sd_status"),
            ("Telemetry Radio (XBee / LoRa)", "radio_status"),
            ("Shared I2C Bus (GPIO 38/39)", "i2c_status")
        ]

        grid_frame = ctk.CTkFrame(health_frame, fg_color="transparent")
        grid_frame.pack(fill="x", padx=10, pady=5)

        self.sensor_status_badges = {}
        for idx, (sensor_name, key) in enumerate(chips):
            col = idx % 3
            row = idx // 3
            card = ctk.CTkFrame(grid_frame, fg_color="#212f3d", corner_radius=6)
            card.grid(row=row, column=col, padx=8, pady=6, sticky="nsew")
            grid_frame.grid_columnconfigure(col, weight=1)

            ctk.CTkLabel(card, text=sensor_name, font=ctk.CTkFont(size=11, weight="bold")).pack(anchor="w", padx=10, pady=(6, 0))
            badge = ctk.CTkLabel(card, text="● READY", font=ctk.CTkFont(size=12, weight="bold"), text_color="#2ecc71")
            badge.pack(anchor="w", padx=10, pady=(0, 6))
            self.sensor_status_badges[key] = badge

        # Quadcopter Motor Mixer & Parachute Reaction Display
        motor_frame = ctk.CTkFrame(parent)
        motor_frame.pack(fill="both", expand=True, padx=15, pady=10)

        ctk.CTkLabel(motor_frame, text="Quadcopter 'X' Mixer & Parachute Servo Outputs", font=ctk.CTkFont(size=14, weight="bold")).pack(anchor="w", padx=15, pady=(10, 5))

        mixer_grid = ctk.CTkFrame(motor_frame, fg_color="transparent")
        mixer_grid.pack(fill="both", expand=True, padx=15, pady=10)

        self.motor_bars = {}
        motors = [
            ("Motor 1 (Front-Left)", "m1"),
            ("Motor 2 (Front-Right)", "m2"),
            ("Motor 3 (Rear-Right)", "m3"),
            ("Motor 4 (Rear-Left)", "m4"),
            ("Parachute Release Servo", "servo")
        ]

        for idx, (label, key) in enumerate(motors):
            m_card = ctk.CTkFrame(mixer_grid, fg_color="#212f3d", corner_radius=6)
            m_card.pack(fill="x", pady=6)

            lbl = ctk.CTkLabel(m_card, text=label, width=180, anchor="w", font=ctk.CTkFont(size=12, weight="bold"))
            lbl.pack(side="left", padx=15, pady=8)

            prog = ctk.CTkProgressBar(m_card, width=400, height=18)
            prog.set(0.0)
            prog.pack(side="left", padx=15, fill="x", expand=True)

            val_lbl = ctk.CTkLabel(m_card, text="1000 µs (OFF)", width=120, font=ctk.CTkFont(size=12))
            val_lbl.pack(side="right", padx=15)

            self.motor_bars[key] = (prog, val_lbl)

    # -------------------------------------------------------------------------
    # TAB 4: SENSORLESS BENCH TEST & HIL SIMULATION
    # -------------------------------------------------------------------------
    def _build_hil_tab(self):
        parent = self.tab_hil

        desc_frame = ctk.CTkFrame(parent)
        desc_frame.pack(fill="x", padx=15, pady=10)

        ctk.CTkLabel(desc_frame, text="Hardware-In-The-Loop (HIL) Sensor Simulation & Bench Test", font=ctk.CTkFont(size=15, weight="bold")).pack(anchor="w", padx=15, pady=(10, 2))
        ctk.CTkLabel(desc_frame, text="Inject synthetic flight dynamics to verify state machine transitions, apogee detection, 600m parachute deployment, and motor PID activation on the desk.", font=ctk.CTkFont(size=11), text_color="gray").pack(anchor="w", padx=15, pady=(0, 10))

        # Simulation Mode Activation & 1-Click Bench Mode
        ctrl_frame = ctk.CTkFrame(parent)
        ctrl_frame.pack(fill="x", padx=15, pady=5)

        btn_enable_sim = ctk.CTkButton(ctrl_frame, text="1. Enable HIL Simulation", fg_color="#2980b9", hover_color="#1f618d", command=lambda: self._send_command("CMD,1234,SIM,ENABLE\n"))
        btn_enable_sim.pack(side="left", padx=10, pady=10)

        btn_cal = ctk.CTkButton(ctrl_frame, text="2. Tare Ground Zero (CAL)", fg_color="#27ae60", hover_color="#1e8449", command=lambda: self._send_command("CMD,1234,CAL\n"))
        btn_cal.pack(side="left", padx=5, pady=10)

        btn_disable_sim = ctk.CTkButton(ctrl_frame, text="Disable Simulation", fg_color="#7f8c8d", hover_color="#626567", command=lambda: self._send_command("CMD,1234,SIM,DISABLE\n"))
        btn_disable_sim.pack(side="left", padx=5, pady=10)

        btn_abort = ctk.CTkButton(ctrl_frame, text="🛑 EMERGENCY ABORT", fg_color="#e74c3c", hover_color="#c0392b", font=ctk.CTkFont(weight="bold"), command=lambda: self._send_command("CMD,1234,ABORT\n"))
        btn_abort.pack(side="right", padx=10, pady=10)

        # Preset Flight Profiles
        presets_frame = ctk.CTkFrame(parent)
        presets_frame.pack(fill="x", padx=15, pady=10)

        ctk.CTkLabel(presets_frame, text="Automated Flight Profile Injections:", font=ctk.CTkFont(size=13, weight="bold")).pack(anchor="w", padx=15, pady=(10, 5))

        btn_box = ctk.CTkFrame(presets_frame, fg_color="transparent")
        btn_box.pack(fill="x", padx=10, pady=5)

        ctk.CTkButton(btn_box, text="🚀 Run Full Mission Simulation (0 ➔ 670m ➔ 0m)", fg_color="#8e44ad", hover_color="#732d91", font=ctk.CTkFont(weight="bold"), command=self._sim_launch_profile).pack(side="left", padx=5, pady=5)
        ctk.CTkButton(btn_box, text="🎯 Test Live PID Stabilization (15%)", fg_color="#d35400", hover_color="#ba4a00", command=lambda: self._send_command("CMD,1234,PID,START,15\n")).pack(side="left", padx=5, pady=5)
        ctk.CTkButton(btn_box, text="🪂 Trigger Parachute (State 4)", command=lambda: self._send_command("CMD,1234,SIMP,93500\n")).pack(side="left", padx=5, pady=5)
        ctk.CTkButton(btn_box, text="🚁 Trigger Drone Hover (State 6)", command=lambda: self._send_command("CMD,1234,SIMP,98000\n")).pack(side="left", padx=5, pady=5)
        ctk.CTkButton(btn_box, text="🏁 Trigger Touchdown (State 7)", command=lambda: self._send_command("CMD,1234,SIMP,101325\n")).pack(side="left", padx=5, pady=5)

        # Manual Pressure & Altitude Slider
        slider_frame = ctk.CTkFrame(parent)
        slider_frame.pack(fill="both", expand=True, padx=15, pady=10)

        ctk.CTkLabel(slider_frame, text="Manual Altitude / Pressure Injection:", font=ctk.CTkFont(size=13, weight="bold")).pack(anchor="w", padx=15, pady=(10, 2))

        self.alt_slider_val_lbl = ctk.CTkLabel(slider_frame, text="Injected Altitude: 0 m | Pressure: 101325 Pa", font=ctk.CTkFont(size=13, weight="bold"))
        self.alt_slider_val_lbl.pack(pady=5)

        self.alt_slider = ctk.CTkSlider(slider_frame, from_=0, to=1200, number_of_steps=120, command=self._on_alt_slider_change)
        self.alt_slider.set(0)
        self.alt_slider.pack(fill="x", padx=30, pady=10)

        ctk.CTkButton(slider_frame, text="Inject Selected Pressure", width=200, command=self._inject_slider_pressure).pack(pady=5)

    def _on_alt_slider_change(self, value):
        alt = float(value)
        pres = 101325.0 * ((1.0 - 2.25577e-5 * alt) ** 5.25588)
        self.alt_slider_val_lbl.configure(text=f"Injected Altitude: {alt:.1f} m | Calculated Pressure: {pres:.1f} Pa")

    def _inject_slider_pressure(self):
        alt = self.alt_slider.get()
        pres = 101325.0 * ((1.0 - 2.25577e-5 * alt) ** 5.25588)
        self._send_command(f"CMD,1234,SIMP,{pres:.1f}\n")

    def _action_enable_bench_mode(self):
        if not self.is_connected:
            messagebox.showwarning("Not Connected", "Please connect to the ESP32 serial port first.")
            return

        def run_bench():
            self._send_command("set bit_override 1\n")
            time.sleep(0.3)
            self._send_command("reboot\n")

        threading.Thread(target=run_bench, daemon=True).start()
        messagebox.showinfo("Desk Test Mode", "Sensor check bypass command sent ('set bit_override 1').\nESP32 is rebooting into Desk Test Mode!")

    def _action_restore_safety_mode(self):
        if not self.is_connected:
            messagebox.showwarning("Not Connected", "Please connect to the ESP32 serial port first.")
            return

        def run_safety():
            self._send_command("set bit_override 0\n")
            time.sleep(0.3)
            self._send_command("reboot\n")

        threading.Thread(target=run_safety, daemon=True).start()
        messagebox.showinfo("Flight Safety Restored", "Safety check restored ('set bit_override 0').\nESP32 is rebooting in Flight Safe Mode!")

    def _sim_launch_profile(self):
        def run_sim():
            self._send_command("CMD,1234,SIM,ENABLE\n")
            time.sleep(0.3)
            self._send_command("CMD,1234,CAL\n")
            time.sleep(0.5)

            p0 = 101325.0

            # Phase 1: Ascent 0 -> 670m (State 3 ASCENT)
            pcount = 0
            for step in range(40):
                frac = step / 40.0
                pcount += 1
                alt = 670.0 * (1.0 - math.cos(frac * math.pi / 2.0))
                pres = p0 * ((1.0 - 2.25577e-5 * alt) ** 5.25588)
                pitch = 12.0 * math.sin(step * 0.4)
                roll = 8.0 * math.cos(step * 0.3)
                yaw = (step * 8.0) % 360.0

                self._send_command(f"CMD,1234,SIMP,{pres:.1f}\n")
                if telemetry_3d_server:
                    telemetry_3d_server.update_telemetry(pitch=pitch, roll=roll, yaw=yaw, alt=alt, pres=pres, state=3, state_name="ASCENT", pcount=pcount)
                self.after(0, lambda p=pitch, r=roll, y=yaw, a=alt, pr=pres: self._update_3d_tab_readouts(p, r, y, alt=a, pres=pr, state=3, state_str="ASCENT"))
                time.sleep(0.12)

            # Phase 2: Parachute Descent 670m -> 320m (State 4 PARACHUTE)
            for step in range(25):
                frac = step / 25.0
                pcount += 1
                alt = 670.0 - (670.0 - 320.0) * frac
                pres = p0 * ((1.0 - 2.25577e-5 * alt) ** 5.25588)
                pitch = 18.0 * math.sin(step * 0.5)
                roll = 22.0 * math.cos(step * 0.4)
                yaw = (320.0 + step * 4.0) % 360.0

                self._send_command(f"CMD,1234,SIMP,{pres:.1f}\n")
                if telemetry_3d_server:
                    telemetry_3d_server.update_telemetry(pitch=pitch, roll=roll, yaw=yaw, alt=alt, pres=pres, state=4, state_name="PARACHUTE", pcount=pcount)
                self.after(0, lambda p=pitch, r=roll, y=yaw, a=alt, pr=pres: self._update_3d_tab_readouts(p, r, y, alt=a, pres=pr, state=4, state_str="PARACHUTE"))
                time.sleep(0.12)

            # Phase 3: Drone Hover & Final Descent 320m -> 0m (State 6 DRONE_HOVER)
            for step in range(40):
                frac = step / 40.0
                pcount += 1
                alt = 320.0 - (320.0 - 0.0) * frac
                pres = p0 * ((1.0 - 2.25577e-5 * alt) ** 5.25588)
                pitch = 5.0 * math.sin(step * 0.8)
                roll = 6.0 * math.cos(step * 0.7)
                yaw = (step * 3.0) % 360.0

                self._send_command(f"CMD,1234,SIMP,{pres:.1f}\n")
                if telemetry_3d_server:
                    telemetry_3d_server.update_telemetry(pitch=pitch, roll=roll, yaw=yaw, alt=alt, pres=pres, state=6, state_name="DRONE_HOVER", pcount=pcount)
                self.after(0, lambda p=pitch, r=roll, y=yaw, a=alt, pr=pres: self._update_3d_tab_readouts(p, r, y, alt=a, pres=pr, state=6, state_str="DRONE_HOVER"))
                time.sleep(0.12)

            # Phase 4: Landed (State 7 LANDED)
            for _ in range(5):
                pcount += 1
                self._send_command(f"CMD,1234,SIMP,{p0:.1f}\n")
                if telemetry_3d_server:
                    telemetry_3d_server.update_telemetry(pitch=0.0, roll=0.0, yaw=0.0, alt=0.0, pres=p0, state=7, state_name="LANDED", pcount=pcount)
                self.after(0, lambda: self._update_3d_tab_readouts(0, 0, 0, alt=0.0, pres=p0, state=7, state_str="LANDED"))
                time.sleep(0.2)

        threading.Thread(target=run_sim, daemon=True).start()

    # -------------------------------------------------------------------------
    # TAB 5: SERIAL TERMINAL
    # -------------------------------------------------------------------------
    def _build_terminal_tab(self):
        parent = self.tab_terminal

        self.term_txt = ctk.CTkTextbox(parent, font=("Consolas", 11), wrap="char")
        self.term_txt.pack(fill="both", expand=True, padx=15, pady=(10, 5))

        cmd_frame = ctk.CTkFrame(parent, fg_color="transparent")
        cmd_frame.pack(fill="x", padx=15, pady=(0, 10))

        self.term_input = ctk.CTkEntry(cmd_frame, placeholder_text="Enter CLI command (e.g. tasks, bit, cal, help)...")
        self.term_input.pack(side="left", fill="x", expand=True, padx=(0, 10))
        self.term_input.bind("<Return>", lambda e: self._send_terminal_input())
        if hasattr(self.term_input, "_entry"):
            self.term_input._entry.bind("<Return>", lambda e: self._send_terminal_input())

        btn_send = ctk.CTkButton(cmd_frame, text="Send", width=90, command=self._send_terminal_input)
        btn_send.pack(side="right")

        btn_clear = ctk.CTkButton(cmd_frame, text="Clear", width=70, fg_color="#7f8c8d", hover_color="#626567", command=lambda: self.term_txt.delete("1.0", "end"))
        btn_clear.pack(side="right", padx=5)

    def _append_term_text(self, text):
        try:
            self.term_txt.insert("end", text)
            try:
                line_count = int(self.term_txt.index("end-1c").split('.')[0])
                if line_count > 1500:
                    self.term_txt.delete("1.0", "300.0")
            except Exception:
                pass
            self.term_txt.see("end")
        except Exception:
            pass

    def _send_terminal_input(self):
        text = self.term_input.get()
        if text:
            self._send_command(text + "\n")
            self.term_input.delete(0, "end")

    # -------------------------------------------------------------------------
    # SERIAL & PROTOCOL HANDLING
    # -------------------------------------------------------------------------
    def _get_available_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        return ports if ports else ["No Ports Found"]

    def _refresh_ports(self):
        ports = self._get_available_ports()
        self.port_combo.configure(values=ports)
        if "COM15" in ports:
            self.port_combo.set("COM15")
        elif ports and ports[0] != "No Ports Found":
            self.port_combo.set(ports[0])

    def _toggle_connection(self):
        if not self.is_connected:
            port = self.port_combo.get()
            baud = int(self.baud_combo.get())
            if not port or port == "No Ports Found":
                messagebox.showerror("Error", "No valid serial COM port selected.")
                return

            try:
                self.ser = serial.Serial()
                self.ser.port = port
                self.ser.baudrate = baud
                self.ser.timeout = 0.1
                self.ser.write_timeout = 0.5
                self.ser.dtr = False
                self.ser.rts = False
                self.ser.open()
                self.is_connected = True
                self.att_last_rx = 0.0
                self.stop_serial.clear()
                self.serial_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
                self.serial_thread.start()

                self.connect_btn.configure(text="Disconnect", fg_color="#e74c3c", hover_color="#c0392b")
                self.status_lbl.configure(text=f"Status: Connected to {port} @ {baud} baud", text_color="#2ecc71")
            except Exception as e:
                messagebox.showerror("Connection Failed", f"Could not open {port}:\n{str(e)}")
        else:
            self._disconnect_serial()

    def _disconnect_serial(self):
        self.stop_serial.set()
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.is_connected = False
        self.connect_btn.configure(text="Connect", fg_color="#2ecc71", hover_color="#27ae60")
        self.status_lbl.configure(text="Status: Disconnected", text_color="gray")

    def _serial_read_loop(self):
        while not self.stop_serial.is_set():
            if self.ser and self.ser.is_open:
                try:
                    line = self.ser.readline().decode("utf-8", errors="replace")
                    if line:
                        if ",ATT," in line[:12]:
                            # Attitude goes straight to the 3D stream from this thread so the
                            # Tk event loop can never add latency or jitter to the motion.
                            self._handle_att_line(line)
                        else:
                            self.serial_rx_queue.put(line)
                except Exception:
                    time.sleep(0.02)
            else:
                time.sleep(0.05)

    def _handle_att_line(self, line):
        """1234,ATT,<mcu_ms>,<qw>,<qx>,<qy>,<qz>,<tilt_x>,<tilt_y>,<rot_z>,<mount>,<ref_count>"""
        parts = line.strip().split(",")
        if len(parts) < 11:
            return
        try:
            mcu_ms = int(parts[2])
            q = (float(parts[3]), float(parts[4]), float(parts[5]), float(parts[6]))
            mount = int(parts[10])
        except ValueError:
            return
        if abs(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] - 1.0) > 0.05:
            return
        now = time.time()
        self.q_raw = q
        self.att_last_rx = now
        self.att_mcu_ms = mcu_ms
        self.att_count += 1
        self.mount_name = MOUNT_NAMES.get(mount, f"code {mount}")
        q_disp = apply_tare(self.q_tare, q)
        if telemetry_3d_server:
            telemetry_3d_server.update_telemetry(q=q_disp, t_mcu=mcu_ms, mount_name=self.mount_name)
        # UI readouts at most 20 Hz (Tk thread)
        if now - getattr(self, "_last_att_ui", 0.0) >= 0.05:
            self._last_att_ui = now
            self.serial_rx_queue.put(("ATT", q_disp))

    def _send_command(self, cmd_str):
        if not self.is_connected or not self.ser or not self.ser.is_open:
            messagebox.showwarning("Not Connected", "Please connect to the serial port first.")
            return

        # Echo command to terminal immediately on the main GUI thread
        self._append_term_text(f">>> {cmd_str}")

        def do_write():
            try:
                if self.ser and self.ser.is_open:
                    self.ser.write(cmd_str.encode("utf-8", errors="replace"))
            except Exception as e:
                self.serial_rx_queue.put(f"ERR: Failed to send command: {e}\n")

        threading.Thread(target=do_write, daemon=True).start()

    def _process_serial_queue(self):
        while not self.serial_rx_queue.empty():
            item = self.serial_rx_queue.get_nowait()
            if isinstance(item, tuple):
                self._apply_att_ui(item[1])
            else:
                self._handle_incoming_line(item)
        self.after(10, self._process_serial_queue)

    def _apply_att_ui(self, q_disp):
        p, r, y = euler_zxy(q_disp)
        self._update_3d_tab_readouts(p, r, y)
        if hasattr(self, "metric_labels"):
            self.metric_labels["tilt"].configure(text=f"{p:.1f}°, {r:.1f}°")
            self.metric_labels["rotz"].configure(text=f"{y:.1f}°")
        now = time.time()
        if now - self._att_rate_t0 >= 1.0:
            self.att_rate_hz = self.att_count / (now - self._att_rate_t0)
            self.att_count = 0
            self._att_rate_t0 = now
            if hasattr(self, "lbl_mount"):
                self.lbl_mount.configure(text=self.mount_name)
                self.lbl_att_link.configure(
                    text=f"Attitude stream: {self.att_rate_hz:.0f} Hz | MCU t = {self.att_mcu_ms / 1000.0:.2f} s")

    def _att_stream_active(self):
        return (time.time() - self.att_last_rx) < 0.5

    def _handle_env_line(self, line):
        try:
            parts = [p.strip() for p in line.split(",")]
            temp = None
            hum = None
            voc = None
            nox = None

            if len(parts) >= 6 and parts[1] == "ENV":
                temp = float(parts[2])
                hum = float(parts[3])
                voc = int(parts[4])
                nox = int(parts[5])
            elif len(parts) >= 5 and parts[0] == "ENV":
                temp = float(parts[1])
                hum = float(parts[2])
                voc = int(parts[3])
                nox = int(parts[4])
            else:
                m_t = re.search(r"T=([0-9.-]+)", line)
                m_h = re.search(r"RH=([0-9.-]+)", line)
                m_v = re.search(r"VOC_idx=([0-9]+)", line)
                m_n = re.search(r"NOx_idx=([0-9]+)", line)
                if m_t: temp = float(m_t.group(1))
                if m_h: hum = float(m_h.group(1))
                if m_v: voc = int(m_v.group(1))
                if m_n: nox = int(m_n.group(1))

            if temp is not None and hum is not None and voc is not None and nox is not None:
                self.latest_temp = temp
                self.latest_hum = hum
                self.latest_voc = voc
                self.latest_nox = nox

                self.hum_buffer.append(hum)
                self.voc_buffer.append(voc)
                self.nox_buffer.append(nox)
                self.temp_buffer.append(temp)
                self.env_time_buffer.append(datetime.now().strftime("%H:%M:%S"))

                # Quality & Color Mapping for VOC (Sensirion UBA Standard):
                if voc <= 100:
                    voc_desc = "Clean Baseline"
                    voc_color = "#2ecc71"
                elif voc <= 150:
                    voc_desc = "VOC Detected"
                    voc_color = "#f1c40f"
                elif voc <= 250:
                    voc_desc = "Elevated VOC"
                    voc_color = "#e67e22"
                elif voc <= 380:
                    voc_desc = "High VOC Alert"
                    voc_color = "#e74c3c"
                else:
                    voc_desc = "Hazardous Spike"
                    voc_color = "#9b59b6"

                # Quality & Color Mapping for NOx:
                if nox <= 5:
                    nox_desc = "Normal Ambient"
                    nox_color = "#2ecc71"
                elif nox <= 25:
                    nox_desc = "Low NOx"
                    nox_color = "#f1c40f"
                elif nox <= 100:
                    nox_desc = "Elevated NOx"
                    nox_color = "#e67e22"
                else:
                    nox_desc = "High NOx Alert"
                    nox_color = "#e74c3c"

                # Update GCS Tab Metrics
                if hasattr(self, 'metric_labels'):
                    if "hum" in self.metric_labels:
                        self.metric_labels["hum"].configure(text=f"{hum:.1f} %")
                    if "temp" in self.metric_labels:
                        self.metric_labels["temp"].configure(text=f"{temp:.1f} °C")
                    if "voc" in self.metric_labels:
                        self.metric_labels["voc"].configure(text=f"{voc} ({voc_desc})", text_color=voc_color)
                    if "nox" in self.metric_labels:
                        self.metric_labels["nox"].configure(text=f"{nox} ({nox_desc})", text_color=nox_color)

                # Update Environmental Tab Cards
                if hasattr(self, 'env_lbl_temp'):
                    self.env_lbl_temp.configure(text=f"{temp:.1f} °C")
                if hasattr(self, 'env_lbl_hum'):
                    self.env_lbl_hum.configure(text=f"{hum:.1f} %")
                if hasattr(self, 'env_lbl_voc'):
                    self.env_lbl_voc.configure(text=f"{voc} ({voc_desc})", text_color=voc_color)
                if hasattr(self, 'env_lbl_nox'):
                    self.env_lbl_nox.configure(text=f"{nox} ({nox_desc})", text_color=nox_color)

                # Update FC Subsystem BIT Badges
                if hasattr(self, 'sensor_status_badges'):
                    if "sht_status" in self.sensor_status_badges:
                        self.sensor_status_badges["sht_status"].configure(text="● READY", text_color="#2ecc71")
                    if "sgp_status" in self.sensor_status_badges:
                        self.sensor_status_badges["sgp_status"].configure(text="● READY", text_color="#2ecc71")
        except Exception:
            pass

    def _handle_incoming_line(self, line):
        # Terminal: show logs/commands always; telemetry frames at most 2 per second
        if line.startswith("1234,0") or line.startswith("1234,1") or line.startswith("1234,2"):
            now_t = time.time()
            if now_t - getattr(self, "_last_term_frame_t", 0.0) >= 0.5:
                self._last_term_frame_t = now_t
                self._append_term_text(line)
        else:
            self._append_term_text(line)

        clean_line = line.strip()
        if not clean_line:
            return

        # Check for Environmental packet (1234,ENV,... or ENV:...):
        if clean_line.startswith("1234,ENV,") or clean_line.startswith("ENV,") or "ENV: SHT4x:" in clean_line:
            self._handle_env_line(clean_line)
            return

        # Check for TX > format
        if clean_line.startswith("TX >"):
            clean_line = clean_line[4:].strip()

        parts = [p.strip() for p in clean_line.split(",")]

        # 16-field CAN-7USAT standard
        if len(parts) >= 16:
            try:
                team_id = parts[0]
                m_time = parts[1]
                p_count = int(parts[2])
                alt = float(parts[3])
                pres = float(parts[4])
                temp = float(parts[5])
                volt = float(parts[6])
                gnss_time = parts[7]
                lat = float(parts[8])
                lon = float(parts[9])
                sats = int(parts[11])
                tilt_x = float(parts[12])
                tilt_y = float(parts[13])
                rot_z = float(parts[14])
                state = int(parts[15])
                state_info = STATE_NAMES.get(state, (f"STATE_{state}", "#95a5a6"))

                att_live = self._att_stream_active()
                if not att_live:
                    # Older firmware without the ATT quaternion stream: use the frame's Euler
                    self.q_raw = quat_from_zxy(tilt_x, tilt_y, rot_z)
                eff_pitch, eff_roll, eff_yaw = euler_zxy(apply_tare(self.q_tare, self.q_raw))

                self.time_buffer.append(m_time)
                self.alt_buffer.append(alt)
                self.pres_buffer.append(pres)
                self.temp_buffer.append(temp)
                self.volt_buffer.append(volt)
                self.state_buffer.append(state)

                if telemetry_3d_server:
                    if att_live:
                        # Attitude is streamed separately (quaternion); send only frame data
                        telemetry_3d_server.update_telemetry(
                            alt=alt, pres=pres, temp=temp, volt=volt,
                            state=state, state_name=state_info[0], pcount=p_count
                        )
                    else:
                        telemetry_3d_server.update_telemetry(
                            pitch=eff_pitch, roll=eff_roll, yaw=eff_yaw,
                            alt=alt, pres=pres, temp=temp, volt=volt,
                            state=state, state_name=state_info[0], pcount=p_count
                        )

                # Throttle Tkinter UI widget configurations to 20 Hz (50ms)
                now_ui = time.time()
                if not hasattr(self, '_last_ui_draw_t') or (now_ui - self._last_ui_draw_t >= 0.05):
                    self._last_ui_draw_t = now_ui
                    self.metric_labels["alt"].configure(text=f"{alt:.2f} m")
                    self.metric_labels["pres"].configure(text=f"{pres:.1f} Pa")
                    self.metric_labels["temp"].configure(text=f"{temp:.1f} °C")
                    self.metric_labels["volt"].configure(text=f"{volt:.2f} V")
                    self.metric_labels["mtime"].configure(text=m_time)
                    self.metric_labels["pcount"].configure(text=str(p_count))
                    self.metric_labels["sats"].configure(text=str(sats))
                    self.metric_labels["gps"].configure(text=f"{lat:.4f}, {lon:.4f}")
                    self.state_badge.configure(text=f"{state_info[0]} ({state})", text_color=state_info[1])
                    self._update_fc_reactions(state, alt, eff_pitch, eff_roll)
                    if not att_live:
                        self.metric_labels["tilt"].configure(text=f"{eff_pitch:.1f}°, {eff_roll:.1f}°")
                        self.metric_labels["rotz"].configure(text=f"{eff_yaw:.1f}°")
                        self._update_3d_tab_readouts(eff_pitch, eff_roll, eff_yaw)
            except Exception:
                pass

        # 11-field Transmitter format
        elif len(parts) >= 11:
            try:
                p_count = int(parts[0])
                state_str = parts[1]
                alt = float(parts[2])
                temp_bmp = float(parts[3])
                hum = float(parts[5])
                pres_hpa = float(parts[6])
                pitch = float(parts[7])
                roll = float(parts[8])
                az_world = float(parts[9])
                vel_z = float(parts[10])

                # This format carries pitch/roll only (field 9 is vertical acceleration, not yaw)
                self.q_raw = quat_from_zxy(pitch, roll, 0.0)
                eff_pitch, eff_roll, eff_yaw = euler_zxy(apply_tare(self.q_tare, self.q_raw))

                state_map = {"ON_PAD": 2, "ASCENT": 3, "DESCENT": 4, "DEPLOYED": 4, "HOVER": 6, "LANDED": 7}
                state = state_map.get(state_str, 2)
                state_info = STATE_NAMES.get(state, (state_str, "#95a5a6"))

                self.time_buffer.append(datetime.now().strftime("%H:%M:%S"))
                self.alt_buffer.append(alt)
                self.pres_buffer.append(pres_hpa * 100.0)
                self.temp_buffer.append(temp_bmp)
                self.volt_buffer.append(7.40)
                self.state_buffer.append(state)

                if telemetry_3d_server:
                    telemetry_3d_server.update_telemetry(
                        pitch=eff_pitch, roll=eff_roll, yaw=eff_yaw,
                        alt=alt, pres=pres_hpa * 100.0, temp=temp_bmp, volt=7.4,
                        state=state, state_name=state_info[0], pcount=p_count
                    )

                # Throttle Tkinter UI widget configurations to 20 Hz (50ms)
                now_ui = time.time()
                if not hasattr(self, '_last_ui_draw_t') or (now_ui - self._last_ui_draw_t >= 0.05):
                    self._last_ui_draw_t = now_ui
                    self.metric_labels["alt"].configure(text=f"{alt:.2f} m")
                    self.metric_labels["pres"].configure(text=f"{pres_hpa * 100.0:.1f} Pa")
                    self.metric_labels["temp"].configure(text=f"{temp_bmp:.1f} °C")
                    self.metric_labels["volt"].configure(text="7.40 V")
                    self.metric_labels["pcount"].configure(text=str(p_count))
                    self.metric_labels["tilt"].configure(text=f"{eff_pitch:.1f}°, {eff_roll:.1f}°")
                    self.metric_labels["rotz"].configure(text=f"{az_world:.1f} m/s²")
                    self.state_badge.configure(text=f"{state_info[0]} ({state})", text_color=state_info[1])
                    self._update_fc_reactions(state, alt, eff_pitch, eff_roll)
                    self._update_3d_tab_readouts(eff_pitch, eff_roll, eff_yaw)
            except Exception:
                pass

    def _update_fc_reactions(self, state, alt, pitch, roll):
        if state >= 4:
            self.motor_bars["servo"][0].set(1.0)
            self.motor_bars["servo"][1].configure(text="2000 µs (DEPLOYED)")
        else:
            self.motor_bars["servo"][0].set(0.0)
            self.motor_bars["servo"][1].configure(text="1000 µs (LOCKED)")

        if state == 6:
            base_pwm = 1250
            m1_val = max(1000, min(1450, int(base_pwm + pitch * 2 + roll * 2)))
            m2_val = max(1000, min(1450, int(base_pwm + pitch * 2 - roll * 2)))
            m3_val = max(1000, min(1450, int(base_pwm - pitch * 2 - roll * 2)))
            m4_val = max(1000, min(1450, int(base_pwm - pitch * 2 + roll * 2)))

            for k, val in [("m1", m1_val), ("m2", m2_val), ("m3", m3_val), ("m4", m4_val)]:
                self.motor_bars[k][0].set((val - 1000) / 1000.0)
                self.motor_bars[k][1].configure(text=f"{val} µs (PID ACTIVE)")
        elif state == 7:
            for k in ["m1", "m2", "m3", "m4"]:
                self.motor_bars[k][0].set(0.0)
                self.motor_bars[k][1].configure(text="1000 µs (LANDED CUTOFF)")
        else:
            for k in ["m1", "m2", "m3", "m4"]:
                self.motor_bars[k][0].set(0.0)
                self.motor_bars[k][1].configure(text="1000 µs (DISARMED)")

    def _update_plots(self):
        if MATPLOTLIB_AVAILABLE and hasattr(self, 'line_alt') and len(self.alt_buffer) > 1:
            try:
                x_pts = list(range(len(self.alt_buffer)))
                alts = list(self.alt_buffer)
                states = list(self.state_buffer) if len(self.state_buffer) == len(self.alt_buffer) else [2] * len(x_pts)

                self.line_alt.set_data(x_pts, alts)
                self.ax1.set_xlim(0, max(20, len(x_pts)))
                min_a = min(alts)
                max_a = max(alts)
                self.ax1.set_ylim(min(0.0, min_a - 10.0), max(50.0, max_a + 20.0))

                self.line_state.set_data(x_pts, states)
                self.ax2.set_xlim(0, max(20, len(x_pts)))

                self.canvas.draw_idle()
            except Exception:
                pass

        if MATPLOTLIB_AVAILABLE and hasattr(self, 'line_env_hum') and len(self.hum_buffer) > 1:
            try:
                x_env = list(range(len(self.hum_buffer)))
                hums = list(self.hum_buffer)
                temps = list(self.temp_buffer)[-len(x_env):] if len(self.temp_buffer) >= len(x_env) else [self.latest_temp] * len(x_env)
                vocs = list(self.voc_buffer)
                noxs = list(self.nox_buffer)

                self.line_env_hum.set_data(x_env, hums)
                self.line_env_temp.set_data(x_env, temps)
                self.line_env_voc.set_data(x_env, vocs)
                self.line_env_nox.set_data(x_env, noxs)

                self.ax_env_trend.set_xlim(0, max(20, len(x_env)))
                max_val = max(110.0, max(vocs) + 15.0 if vocs else 110.0)
                self.ax_env_trend.set_ylim(0, max_val)

                curr_voc = max(1, self.latest_voc)
                curr_nox = max(1, self.latest_nox)

                r_voc = (curr_voc / 100.0) ** (-0.4)
                self.dot_voc.set_data([r_voc], [curr_voc])

                if curr_nox > 1:
                    arg = max(0.001, 1.0 - (curr_nox - 1.0) / 499.0)
                    r_nox = 1.0 - math.log(arg) / 5.0
                else:
                    r_nox = 1.0
                self.dot_nox.set_data([r_nox], [curr_nox])

                self.env_canvas.draw_idle()
            except Exception:
                pass

        self.after(350, self._update_plots)

    # -------------------------------------------------------------------------
    # FLASHER ACTIONS (esptool)
    # -------------------------------------------------------------------------
    def _log_flash(self, msg):
        self.flash_log_txt.insert("end", msg)
        self.flash_log_txt.see("end")

    def _action_detect_chip(self):
        port = self.port_combo.get()
        if not port or port == "No Ports Found":
            messagebox.showerror("Error", "No valid COM port selected.")
            return

        def run_detect():
            self._log_flash(f"\n--- Probing ESP32 on {port} ---\n")
            if self.is_connected:
                self._disconnect_serial()

            cmd = [sys.executable, "-m", "esptool", "--port", port, "chip-id"]
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            for line in proc.stdout:
                self._log_flash(line)
            proc.wait()

            cmd_flash = [sys.executable, "-m", "esptool", "--port", port, "flash-id"]
            proc_f = subprocess.Popen(cmd_flash, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            for line in proc_f.stdout:
                self._log_flash(line)
            proc_f.wait()

        threading.Thread(target=run_detect, daemon=True).start()

    def _action_erase_flash(self):
        port = self.port_combo.get()
        if not port or port == "No Ports Found":
            messagebox.showerror("Error", "No valid COM port selected.")
            return

        if not messagebox.askyesno("Confirm Erase", f"Are you sure you want to completely erase the flash memory on {port}?"):
            return

        def run_erase():
            self._log_flash(f"\n--- Erasing Flash on {port} ---\n")
            if self.is_connected:
                self._disconnect_serial()

            cmd = [sys.executable, "-m", "esptool", "--port", port, "erase-flash"]
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            for line in proc.stdout:
                self._log_flash(line)
            proc.wait()
            self._log_flash("Flash erase completed.\n")

        threading.Thread(target=run_erase, daemon=True).start()

    def _action_flash_firmware(self):
        port = self.port_combo.get()
        role = self.role_var.get()
        bit_bypass = self.bit_bypass_var.get()

        if not port or port == "No Ports Found":
            messagebox.showerror("Error", "No valid COM port selected.")
            return

        def run_flash_task():
            self._log_flash(f"\n========================================\n")
            self._log_flash(f"Flashing Target: ROLE_{role}\n")
            self._log_flash(f"Sensorless BIT Bypass: {bit_bypass}\n")
            self._log_flash(f"Target Port: {port}\n")
            self._log_flash(f"========================================\n")

            if self.is_connected:
                self._disconnect_serial()

            workspace_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            build_dir = os.path.join(workspace_dir, "build")

            flasher_args_file = os.path.join(build_dir, "flasher_args.json")
            if os.path.exists(flasher_args_file):
                try:
                    with open(flasher_args_file, "r") as f:
                        fargs = json.load(f)
                    chip = fargs.get("extra_esptool_args", {}).get("chip", "esp32s3")
                    flash_files = fargs.get("flash_files", {})
                    self._log_flash(f"Flashing {chip} binaries from build/...\n")
                    cmd = [
                        sys.executable, "-m", "esptool",
                        "--chip", chip,
                        "--port", port,
                        "--baud", "460800",
                        "--before", "default_reset",
                        "--after", "hard_reset",
                        "write_flash", "-z"
                    ]
                    for addr, rel_path in flash_files.items():
                        cmd.extend([addr, os.path.join(build_dir, rel_path)])

                    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                    for line in proc.stdout:
                        self._log_flash(line)
                    proc.wait()
                    if proc.returncode == 0:
                        self._log_flash("\n✔ CANSAT_FW FLASH SUCCESSFUL!\n")
                    else:
                        self._log_flash(f"\n✖ Flash failed with code {proc.returncode}\n")
                except Exception as ex:
                    self._log_flash(f"Error parsing flasher_args.json: {ex}\n")
            else:
                self._log_flash(f"Workspace: {workspace_dir}\n")
                self._log_flash(f"Building cansat_fw with ESP-IDF...\n")
                cmd = ["idf.py", "-C", workspace_dir, "build", "flash", "-p", port]
                try:
                    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                    for line in proc.stdout:
                        self._log_flash(line)
                    proc.wait()
                except FileNotFoundError:
                    self._log_flash("ESP-IDF (idf.py) is currently being installed.\n")

        threading.Thread(target=run_flash_task, daemon=True).start()


    # =============================================================================

    # ENTRY POINT

    # =============================================================================

if __name__ == "__main__":
    app = AakashvaniDock()
    app.mainloop()