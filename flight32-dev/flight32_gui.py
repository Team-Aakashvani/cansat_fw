#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Flight32 Ground Station & MSP Test Dashboard GUI
A cool, cyberpunk-themed aerospace GUI for ESP32 Flight Controller (Flight32).
Built with ttkbootstrap (Bootstrap theming for Tkinter).

Author: Antigravity AI & Team-Aakashvani
License: MIT
"""

import sys
import os
import time
import struct
import threading
import queue
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import ttkbootstrap as ttkb
    from ttkbootstrap.constants import *
    try:
        from ttkbootstrap.widgets.scrolled import ScrolledText
    except ImportError:
        from ttkbootstrap.scrolled import ScrolledText
except ImportError:
    print("Error: ttkbootstrap is required. Install with: pip install ttkbootstrap")
    sys.exit(1)

import serial
import serial.tools.list_ports
import socket
import select

# Import the existing MSP tester logic from the local directory
import flight32_msp_tester as msp_test


class NetworkSerial:
    """Wrapper that emulates serial.Serial over a Wi-Fi TCP socket connection."""
    def __init__(self, ip, port=2323, timeout=1.0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect((ip, port))
        self.sock.setblocking(False)
        self.is_open = True
        self.dtr = False
        self.rts = False

    def write(self, data):
        if not self.is_open:
            return 0
        try:
            return self.sock.send(data)
        except (BlockingIOError, socket.error):
            return 0

    @property
    def in_waiting(self):
        r, _, _ = select.select([self.sock], [], [], 0)
        return 1024 if r else 0

    def read(self, size=1):
        if not self.is_open:
            return b""
        try:
            return self.sock.recv(size)
        except (BlockingIOError, socket.error):
            return b""

    def reset_input_buffer(self):
        try:
            while True:
                data = self.sock.recv(4096)
                if not data:
                    break
        except (BlockingIOError, socket.error):
            pass

    def close(self):
        self.is_open = False
        try:
            self.sock.close()
        except Exception:
            pass


class Flight32GroundStation(ttkb.Window):
    def __init__(self):
        super().__init__(title="⚡ Flight32 Cyberpunk Ground Station & MSP Dashboard", themename="cyborg")
        self.geometry("1180x780")
        self.minsize(980, 650)

        # Serial Connection State
        self.ser = None
        self.is_connected = False
        self.telemetry_running = False
        self.cli_mode = False

        # Build State
        self.build_process = None
        self.reconnect_after_upload = False

        # Queues for thread-safe UI updates
        self.ui_queue = queue.Queue()

        # Build UI
        self._build_header_toolbar()
        self._build_main_notebook()
        self._build_status_bar()

        # Start UI event pump
        self.after(50, self._process_ui_queue)

        # Auto-discover ports on launch
        self._refresh_ports()

    # =========================================================================
    # HEADER / TOOLBAR
    # =========================================================================
    def _build_header_toolbar(self):
        toolbar = ttk.Frame(self, padding=(15, 12))
        toolbar.pack(fill=X, side=TOP)

        # Title / Brand Logo
        title_frame = ttk.Frame(toolbar)
        title_frame.pack(side=LEFT, padx=(0, 25))
        
        lbl_logo = ttk.Label(
            title_frame,
            text="⚡ FLIGHT32",
            font=("Outfit", 18, "bold"),
            bootstyle="info"
        )
        lbl_logo.pack(side=LEFT)

        lbl_sub = ttk.Label(
            title_frame,
            text=" GROUND STATION v0.4",
            font=("Outfit", 10),
            bootstyle="secondary"
        )
        lbl_sub.pack(side=LEFT, padx=(6, 0), pady=(6, 0))

        # Port Selection
        lbl_port = ttk.Label(toolbar, text="PORT:", font=("Outfit", 9, "bold"))
        lbl_port.pack(side=LEFT, padx=(10, 4))

        self.cb_ports = ttk.Combobox(toolbar, width=12, state="readonly")
        self.cb_ports.pack(side=LEFT, padx=(0, 6))

        btn_refresh = ttk.Button(
            toolbar,
            text="🔄",
            width=3,
            bootstyle="secondary-outline",
            command=self._refresh_ports
        )
        btn_refresh.pack(side=LEFT, padx=(0, 15))

        # Baud Rate
        lbl_baud = ttk.Label(toolbar, text="BAUD:", font=("Outfit", 9, "bold"))
        lbl_baud.pack(side=LEFT, padx=(5, 4))

        self.cb_baud = ttk.Combobox(toolbar, width=8, state="readonly", values=["9600", "57600", "115200", "460800", "921600"])
        self.cb_baud.set("115200")
        self.cb_baud.pack(side=LEFT, padx=(0, 15))

        # Connect Button
        self.btn_connect = ttk.Button(
            toolbar,
            text="⚡ CONNECT",
            width=14,
            bootstyle="success",
            command=self._toggle_connection
        )
        self.btn_connect.pack(side=LEFT, padx=(5, 10))

        # Status Indicator Badge
        self.lbl_conn_status = ttk.Label(
            toolbar,
            text="● DISCONNECTED",
            font=("Outfit", 10, "bold"),
            bootstyle="danger"
        )
        self.lbl_conn_status.pack(side=LEFT, padx=10)

        # Theme Selector on Right
        theme_frame = ttk.Frame(toolbar)
        theme_frame.pack(side=RIGHT)
        lbl_theme = ttk.Label(theme_frame, text="THEME:", font=("Outfit", 9))
        lbl_theme.pack(side=LEFT, padx=(0, 4))
        self.cb_theme = ttk.Combobox(
            theme_frame,
            width=10,
            state="readonly",
            values=["cyborg", "darkly", "solar", "vapor", "superhero"]
        )
        self.cb_theme.set("cyborg")
        self.cb_theme.bind("<<ComboboxSelected>>", self._change_theme)
        self.cb_theme.pack(side=LEFT)

    # =========================================================================
    # MAIN TABS (NOTEBOOK)
    # =========================================================================
    def _build_main_notebook(self):
        self.notebook = ttk.Notebook(self, bootstyle="info")
        self.notebook.pack(fill=BOTH, expand=True, padx=15, pady=(0, 10))

        # Tab 1: Live Dashboard & Telemetry
        self.tab_dashboard = ttk.Frame(self.notebook, padding=15)
        self.notebook.add(self.tab_dashboard, text="  🛸 LIVE TELEMETRY DASHBOARD  ")
        self._build_dashboard_tab(self.tab_dashboard)

        # Tab 2: Automated 31-Command MSP Test Suite
        self.tab_msp_suite = ttk.Frame(self.notebook, padding=15)
        self.notebook.add(self.tab_msp_suite, text="  🧪 MSP AUTOMATED TEST SUITE (31/31)  ")
        self._build_msp_suite_tab(self.tab_msp_suite)

        # Tab 3: Interactive CLI Terminal
        self.tab_cli = ttk.Frame(self.notebook, padding=15)
        self.notebook.add(self.tab_cli, text="  💻 INTERACTIVE FC TERMINAL  ")
        self._build_cli_tab(self.tab_cli)

        # Tab 4: Firmware Builder & Flasher
        self.tab_builder = ttk.Frame(self.notebook, padding=15)
        self.notebook.add(self.tab_builder, text="  🛠️ FIRMWARE BUILDER & FLASHER  ")
        self._build_builder_tab(self.tab_builder)

    # =========================================================================
    # TAB 1: DASHBOARD & TELEMETRY
    # =========================================================================
    def _build_dashboard_tab(self, parent):
        top_ctrl = ttk.Frame(parent)
        top_ctrl.pack(fill=X, pady=(0, 15))

        self.btn_toggle_telemetry = ttk.Button(
            top_ctrl,
            text="▶ START LIVE TELEMETRY (200ms)",
            bootstyle="info",
            command=self._toggle_telemetry
        )
        self.btn_toggle_telemetry.pack(side=LEFT)

        self.lbl_telem_hint = ttk.Label(
            top_ctrl,
            text="Connect to COM port first to stream live Attitude & Motor DShot data.",
            font=("Outfit", 9, "italic"),
            bootstyle="secondary"
        )
        self.lbl_telem_hint.pack(side=LEFT, padx=15)

        # Onboard Sensor Hardware Status Bar & Bench Debug Toolbar
        sensor_bar = ttk.Labelframe(parent, text="  🛰️ ONBOARD SENSOR DETECTION & OS HEALTH MONITOR  ", padding=10)
        sensor_bar.pack(fill=X, pady=(0, 15))

        badges_frame = ttk.Frame(sensor_bar)
        badges_frame.pack(side=LEFT, fill=X, expand=True)

        self.badge_acc = ttk.Label(badges_frame, text=" ACCEL: --- ", font=("Outfit", 9, "bold"), bootstyle="inverse-secondary", padding=(8, 4))
        self.badge_acc.pack(side=LEFT, padx=(0, 6))

        self.badge_gyro = ttk.Label(badges_frame, text=" GYRO: --- ", font=("Outfit", 9, "bold"), bootstyle="inverse-secondary", padding=(8, 4))
        self.badge_gyro.pack(side=LEFT, padx=(0, 6))

        self.badge_baro = ttk.Label(badges_frame, text=" BARO: --- ", font=("Outfit", 9, "bold"), bootstyle="inverse-secondary", padding=(8, 4))
        self.badge_baro.pack(side=LEFT, padx=(0, 6))

        self.badge_mag = ttk.Label(badges_frame, text=" MAG: --- ", font=("Outfit", 9, "bold"), bootstyle="inverse-secondary", padding=(8, 4))
        self.badge_mag.pack(side=LEFT, padx=(0, 6))

        self.badge_gps = ttk.Label(badges_frame, text=" GPS: --- ", font=("Outfit", 9, "bold"), bootstyle="inverse-secondary", padding=(8, 4))
        self.badge_gps.pack(side=LEFT, padx=(0, 15))

        btn_scan_i2c = ttk.Button(
            sensor_bar,
            text="🔍 SCAN I2C CHIPS",
            bootstyle="warning-outline",
            command=self._run_i2c_scanner_dialog
        )
        btn_scan_i2c.pack(side=RIGHT, padx=(6, 0))

        btn_air_reload = ttk.Button(
            sensor_bar,
            text="⚡ AIR-COMPILE & OTA RELOAD",
            bootstyle="success",
            command=self._trigger_bench_ota_reload
        )
        btn_air_reload.pack(side=RIGHT, padx=(6, 0))

        # Top Grid: System Cards
        cards_frame = ttk.Frame(parent)
        cards_frame.pack(fill=X, pady=(0, 20))
        for i in range(6):
            cards_frame.columnconfigure(i, weight=1, uniform="card")

        self.card_cycletime   = self._create_metric_card(cards_frame, 0, 0, "CYCLE TIME", "--- us", "info")
        self.card_i2c_err     = self._create_metric_card(cards_frame, 0, 1, "I2C ERRORS", "---", "warning")
        self.card_flt_mode    = self._create_metric_card(cards_frame, 0, 2, "FLIGHT MODE", "ACRO", "primary")
        self.card_arm_state   = self._create_metric_card(cards_frame, 0, 3, "ARMING STATE", "DISARMED", "success")
        self.card_temperature = self._create_metric_card(cards_frame, 0, 4, "TEMPERATURE", "--- °C", "danger")
        self.card_pressure    = self._create_metric_card(cards_frame, 0, 5, "BARO PRESSURE", "--- hPa", "info")

        # Main Telemetry Display Split (Left: Attitude Gauges, Right: Motor Outputs)
        split_frame = ttk.Frame(parent)
        split_frame.pack(fill=BOTH, expand=True)
        split_frame.columnconfigure(0, weight=5)
        split_frame.columnconfigure(1, weight=4)

        # Left: Attitude Progress Bars
        att_frame = ttk.Labelframe(split_frame, text="  🧭 ATTITUDE ORIENTATION (EULER ANGLES)  ", padding=15)
        att_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        # Roll
        self.bar_roll, self.lbl_roll_val = self._create_gauge_row(att_frame, "ROLL", "-180°", "180°", -180, 180, "info-striped")
        # Pitch
        self.bar_pitch, self.lbl_pitch_val = self._create_gauge_row(att_frame, "PITCH", "-90°", "90°", -90, 90, "success-striped")
        # Yaw
        self.bar_yaw, self.lbl_yaw_val = self._create_gauge_row(att_frame, "YAW", "0°", "360°", 0, 360, "warning-striped")

        # Right: 4x Motor Output DShot
        motor_frame = ttk.Labelframe(split_frame, text="  ⚙️ MOTOR DSHOT300 OUTPUTS (1000 - 2000)  ", padding=15)
        motor_frame.grid(row=0, column=1, sticky="nsew", padx=(10, 0))

        self.motor_bars = []
        self.motor_labels = []
        for i in range(4):
            bar, lbl = self._create_gauge_row(motor_frame, f"MOTOR {i+1}", "1000", "2000", 1000, 2000, "danger")
            self.motor_bars.append(bar)
            self.motor_labels.append(lbl)

        # Bottom: 9-DOF Raw Inertial Measurement & Environmental Matrix
        dof_frame = ttk.Labelframe(parent, text="  📐 9-DOF INERTIAL MEASUREMENT UNIT (ACCEL / GYRO / MAG & ENV)  ", padding=15)
        dof_frame.pack(fill=X, pady=(15, 0))
        for c in range(3):
            dof_frame.columnconfigure(c, weight=1, uniform="dof")

        # Column 0: Accelerometer (G)
        acc_col = ttk.Frame(dof_frame)
        acc_col.grid(row=0, column=0, sticky="nsew", padx=10)
        ttk.Label(acc_col, text="🚀 ACCELEROMETER (G)", font=("Outfit", 10, "bold"), bootstyle="info").pack(anchor=W, pady=(0, 6))
        self.lbl_acc_x = self._create_dof_readout(acc_col, "ACCEL X", "0.00 G", "info")
        self.lbl_acc_y = self._create_dof_readout(acc_col, "ACCEL Y", "0.00 G", "info")
        self.lbl_acc_z = self._create_dof_readout(acc_col, "ACCEL Z", "1.00 G", "info")

        # Column 1: Gyroscope (DEG/S)
        gyro_col = ttk.Frame(dof_frame)
        gyro_col.grid(row=0, column=1, sticky="nsew", padx=10)
        ttk.Label(gyro_col, text="🌀 GYROSCOPE (DEG/S)", font=("Outfit", 10, "bold"), bootstyle="success").pack(anchor=W, pady=(0, 6))
        self.lbl_gyro_x = self._create_dof_readout(gyro_col, "GYRO X", "0.0 °/s", "success")
        self.lbl_gyro_y = self._create_dof_readout(gyro_col, "GYRO Y", "0.0 °/s", "success")
        self.lbl_gyro_z = self._create_dof_readout(gyro_col, "GYRO Z", "0.0 °/s", "success")

        # Column 2: Magnetometer & Altitude
        mag_col = ttk.Frame(dof_frame)
        mag_col.grid(row=0, column=2, sticky="nsew", padx=10)
        ttk.Label(mag_col, text="🧭 MAGNETOMETER & ALTITUDE", font=("Outfit", 10, "bold"), bootstyle="warning").pack(anchor=W, pady=(0, 6))
        self.lbl_mag_x = self._create_dof_readout(mag_col, "MAG X (uT)", "0.0 uT", "warning")
        self.lbl_mag_y = self._create_dof_readout(mag_col, "MAG Y (uT)", "0.0 uT", "warning")
        self.lbl_mag_z = self._create_dof_readout(mag_col, "MAG Z (uT)", "0.0 uT", "warning")
        self.lbl_altitude = self._create_dof_readout(mag_col, "EST ALTITUDE", "0.00 m", "info")

    def _create_metric_card(self, parent, row, col, title, initial_val, style):
        card = ttk.Frame(parent, bootstyle=f"{style}", padding=2)
        card.grid(row=row, column=col, sticky="nsew", padx=6)
        
        inner = ttk.Frame(card, padding=12)
        inner.pack(fill=BOTH, expand=True)

        lbl_title = ttk.Label(inner, text=title, font=("Outfit", 9, "bold"), bootstyle="secondary")
        lbl_title.pack(anchor=W)

        lbl_val = ttk.Label(inner, text=initial_val, font=("Outfit", 20, "bold"), bootstyle=style)
        lbl_val.pack(anchor=W, pady=(5, 0))
        return lbl_val

    def _create_gauge_row(self, parent, name, min_txt, max_txt, min_val, max_val, style):
        row = ttk.Frame(parent)
        row.pack(fill=X, pady=10)

        header = ttk.Frame(row)
        header.pack(fill=X, pady=(0, 4))
        lbl_name = ttk.Label(header, text=name, font=("Outfit", 10, "bold"))
        lbl_name.pack(side=LEFT)

        lbl_val = ttk.Label(header, text="0.0", font=("Outfit", 11, "bold"), bootstyle="info")
        lbl_val.pack(side=RIGHT)

        bar = ttk.Progressbar(
            row,
            bootstyle=style,
            value=0,
            maximum=max_val - min_val
        )
        bar.pack(fill=X, ipady=4)
        return bar, lbl_val

    def _create_dof_readout(self, parent, label_text, init_val, style):
        box = ttk.Frame(parent, padding=4)
        box.pack(fill=X, pady=3)
        lbl_n = ttk.Label(box, text=label_text, font=("Outfit", 9, "bold"), bootstyle="secondary")
        lbl_n.pack(side=LEFT)
        lbl_v = ttk.Label(box, text=init_val, font=("Consolas", 11, "bold"), bootstyle=style)
        lbl_v.pack(side=RIGHT)
        return lbl_v

    # =========================================================================
    # TAB 2: MSP AUTOMATED TEST SUITE (31/31)
    # =========================================================================
    def _build_msp_suite_tab(self, parent):
        top_ctrl = ttk.Frame(parent)
        top_ctrl.pack(fill=X, pady=(0, 15))

        self.btn_run_suite = ttk.Button(
            top_ctrl,
            text="⚡ RUN 31-COMMAND MSP TEST SUITE",
            bootstyle="success",
            command=self._run_msp_test_suite_thread
        )
        self.btn_run_suite.pack(side=LEFT)

        # Progress bar
        self.msp_progress = ttk.Progressbar(
            top_ctrl,
            bootstyle="success-striped",
            value=0,
            maximum=31
        )
        self.msp_progress.pack(side=LEFT, fill=X, expand=True, padx=20, ipady=4)

        # Counters
        self.lbl_msp_stats = ttk.Label(
            top_ctrl,
            text="Total: 31 | ✅ Pass: 0 | ❌ Fail: 0",
            font=("Outfit", 11, "bold"),
            bootstyle="info"
        )
        self.lbl_msp_stats.pack(side=RIGHT)

        # Table Grid (Treeview)
        cols = ("name", "code", "status", "decoded", "raw")
        self.tree_msp = ttk.Treeview(parent, columns=cols, show="headings", height=16, bootstyle="info")
        self.tree_msp.heading("name", text="MSP Command")
        self.tree_msp.heading("code", text="ID Code")
        self.tree_msp.heading("status", text="Status")
        self.tree_msp.heading("decoded", text="Decoded Result / Payload Summary")
        self.tree_msp.heading("raw", text="Raw Hex Payload")

        self.tree_msp.column("name", width=190, anchor=W)
        self.tree_msp.column("code", width=70, anchor=CENTER)
        self.tree_msp.column("status", width=90, anchor=CENTER)
        self.tree_msp.column("decoded", width=360, anchor=W)
        self.tree_msp.column("raw", width=220, anchor=W)

        scroll_y = ttk.Scrollbar(parent, orient=VERTICAL, command=self.tree_msp.yview)
        self.tree_msp.configure(yscroll=scroll_y.set)

        self.tree_msp.pack(side=LEFT, fill=BOTH, expand=True)
        scroll_y.pack(side=RIGHT, fill=Y)

        # Populate initial rows
        self._populate_initial_msp_tree()

    def _populate_initial_msp_tree(self):
        self.tree_msp.delete(*self.tree_msp.get_children())
        for cmd_name, cmd_code in msp_test.MSP_COMMANDS.items():
            self.tree_msp.insert("", END, iid=cmd_name, values=(cmd_name, str(cmd_code), "READY", "---", "---"))

    # =========================================================================
    # TAB 3: INTERACTIVE CLI TERMINAL
    # =========================================================================
    def _build_cli_tab(self, parent):
        btn_frame = ttk.Frame(parent)
        btn_frame.pack(fill=X, pady=(0, 10))

        quick_cmds = ["help", "status", "tasks", "mem", "settings", "dump", "save"]
        for qcmd in quick_cmds:
            btn = ttk.Button(
                btn_frame,
                text=qcmd.upper(),
                bootstyle="secondary-outline",
                command=lambda c=qcmd: self._send_cli_command(c)
            )
            btn.pack(side=LEFT, padx=(0, 8))

        btn_clear = ttk.Button(
            btn_frame,
            text="🧹 CLEAR",
            bootstyle="warning-outline",
            command=lambda: self.txt_cli.delete("1.0", END)
        )
        btn_clear.pack(side=RIGHT)

        # Terminal Output window
        self.txt_cli = ScrolledText(
            parent,
            font=("Consolas", 10),
            height=20,
            wrap=WORD,
            bootstyle="info"
        )
        self.txt_cli.pack(fill=BOTH, expand=True, pady=(0, 12))

        # Command Entry Box
        entry_frame = ttk.Frame(parent)
        entry_frame.pack(fill=X)

        lbl_prompt = ttk.Label(entry_frame, text="[Flight32 ~] >", font=("Consolas", 11, "bold"), bootstyle="info")
        lbl_prompt.pack(side=LEFT, padx=(0, 8))

        self.ent_cli = ttk.Entry(entry_frame, font=("Consolas", 11))
        self.ent_cli.pack(side=LEFT, fill=X, expand=True)
        self.ent_cli.bind("<Return>", lambda e: self._send_cli_command(self.ent_cli.get()))

        btn_send = ttk.Button(
            entry_frame,
            text="SEND ↵",
            bootstyle="info",
            command=lambda: self._send_cli_command(self.ent_cli.get())
        )
        btn_send.pack(side=RIGHT, padx=(8, 0))

    # =========================================================================
    # TAB 4: FIRMWARE BUILDER & FLASHER
    # =========================================================================
    def _build_builder_tab(self, parent):
        config_frame = ttk.Labelframe(parent, text="  🔧 BUILD & UPLOAD TARGET CONFIGURATION  ", padding=15)
        config_frame.pack(fill=X, pady=(0, 15))

        # Row 1: Target Selector & Options
        r1 = ttk.Frame(config_frame)
        r1.pack(fill=X, pady=(0, 10))

        lbl_target = ttk.Label(r1, text="BOARD TARGET (FQBN):", font=("Outfit", 9, "bold"))
        lbl_target.pack(side=LEFT, padx=(0, 8))

        self.cb_fqbn = ttk.Combobox(
            r1,
            width=30,
            state="readonly",
            values=[
                "esp32:esp32:esp32 (ESP32 Regular - Bench/Dev)",
                "esp32:esp32:esp32s3 (ESP32-S3 WROOM - Flight HW)"
            ]
        )
        self.cb_fqbn.set("esp32:esp32:esp32 (ESP32 Regular - Bench/Dev)")
        self.cb_fqbn.pack(side=LEFT, padx=(0, 20))

        self.chk_bench_test = tk.BooleanVar(value=True)
        chk = ttk.Checkbutton(
            r1,
            text="Bench Test Mode (Log warning on IMU error, don't halt)",
            variable=self.chk_bench_test,
            bootstyle="info-round-toggle"
        )
        chk.pack(side=LEFT)

        # Row 2: Upload Method & OTA IP / Hostname
        r2_ota = ttk.Frame(config_frame)
        r2_ota.pack(fill=X, pady=(0, 10))

        lbl_method = ttk.Label(r2_ota, text="UPLOAD MODE:", font=("Outfit", 9, "bold"))
        lbl_method.pack(side=LEFT, padx=(0, 8))

        self.cb_upload_method = ttk.Combobox(
            r2_ota,
            width=28,
            state="readonly",
            values=[
                "Serial COM Port (USB UART)",
                "Network OTA IP Address (WiFi)"
            ]
        )
        self.cb_upload_method.set("Serial COM Port (USB UART)")
        self.cb_upload_method.pack(side=LEFT, padx=(0, 20))

        lbl_ota_ip = ttk.Label(r2_ota, text="OTA TARGET IP / HOST:", font=("Outfit", 9, "bold"))
        lbl_ota_ip.pack(side=LEFT, padx=(0, 8))

        self.ent_ota_ip = ttk.Entry(r2_ota, width=18, font=("Outfit", 9))
        self.ent_ota_ip.insert(0, "192.168.4.1")
        self.ent_ota_ip.pack(side=LEFT, padx=(0, 15))

        # Row 3: Action Buttons
        r2 = ttk.Frame(config_frame)
        r2.pack(fill=X, pady=(5, 0))

        self.btn_compile_only = ttk.Button(
            r2,
            text="🔨 COMPILE FIRMWARE ONLY",
            bootstyle="info",
            command=lambda: self._start_build_thread(upload=False)
        )
        self.btn_compile_only.pack(side=LEFT, padx=(0, 10))

        self.btn_compile_flash = ttk.Button(
            r2,
            text="⚡ COMPILE & FLASH TO ESP32",
            bootstyle="success",
            command=lambda: self._start_build_thread(upload=True)
        )
        self.btn_compile_flash.pack(side=LEFT, padx=(0, 10))

        self.btn_abort_build = ttk.Button(
            r2,
            text="🛑 ABORT BUILD",
            bootstyle="danger",
            state=DISABLED,
            command=self._abort_build
        )
        self.btn_abort_build.pack(side=LEFT)

        self.lbl_build_badge = ttk.Label(
            r2,
            text="● READY TO BUILD",
            font=("Outfit", 10, "bold"),
            bootstyle="secondary"
        )
        self.lbl_build_badge.pack(side=RIGHT, padx=10)

        # Build Console Log
        lbl_console = ttk.Label(parent, text="COMPILER & FLASH UPLOAD CONSOLE LOGS:", font=("Outfit", 10, "bold"))
        lbl_console.pack(anchor=W, pady=(5, 6))

        self.txt_build = ScrolledText(
            parent,
            font=("Consolas", 9),
            height=18,
            wrap=WORD,
            bootstyle="info"
        )
        self.txt_build.pack(fill=BOTH, expand=True)

    def _get_arduino_cli_path(self):
        candidate_paths = [
            r"C:\Users\Lenovo\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe",
            r"C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe",
            r"C:\Program Files (x86)\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe",
            r"C:\Users\Lenovo\Downloads\arduino-ide_nightly-20240312_Windows_64bit\resources\app\lib\backend\resources\arduino-cli.exe",
        ]
        for path in candidate_paths:
            if os.path.exists(path):
                return path
        return "arduino-cli"

    def _start_build_thread(self, upload=False):
        if self.build_process is not None:
            messagebox.showwarning("Busy", "A build or upload process is already running.")
            return

        is_ota = False
        if hasattr(self, "cb_upload_method") and "Network OTA" in self.cb_upload_method.get():
            port = self.ent_ota_ip.get().strip()
            is_ota = True
            if upload and not port:
                messagebox.showwarning("OTA IP Missing", "Please enter a valid ESP32 IP address or hostname for OTA upload.")
                return
        else:
            port = self.cb_ports.get()
            if upload and not port:
                messagebox.showwarning("Port Missing", "Please select a COM port before uploading.")
                return

        # If uploading via USB UART and currently connected, disconnect temporarily so COM port is free
        if upload and self.is_connected and not is_ota:
            self._log_status("Temporarily disconnecting serial monitor for flash upload...")
            self._disconnect_port()
            self.reconnect_after_upload = True
        else:
            self.reconnect_after_upload = False

        self.btn_compile_only.configure(state=DISABLED)
        self.btn_compile_flash.configure(state=DISABLED)
        self.btn_abort_build.configure(state=NORMAL)
        self.lbl_build_badge.configure(text="🟡 BUILDING FIRMWARE...", bootstyle="warning")

        self.txt_build.delete("1.0", END)
        self._build_print(f"=== FLIGHT32 FIRMWARE BUILD & UPLOAD SESSION ({'OTA WIRELESS' if is_ota else 'USB UART'}) ===\n", "header")

        threading.Thread(target=self._build_worker, args=(upload, port), daemon=True).start()

    def _build_worker(self, upload, port):
        try:
            fqbn_sel = self.cb_fqbn.get()
            fqbn = "esp32:esp32:esp32s3" if "esp32s3" in fqbn_sel else "esp32:esp32:esp32"

            cli_exe = self._get_arduino_cli_path()
            cmd = [cli_exe, "compile", "--fqbn", fqbn, "."]
            if upload:
                cmd.extend(["--upload", "-p", port])

            self.ui_queue.put(("build_log", f"[CMD] {' '.join(cmd)}\n\n"))

            custom_env = os.environ.copy()
            python312_dir = r"C:\Users\Lenovo\AppData\Local\Programs\Python\Python312"
            if os.path.exists(python312_dir):
                custom_env["PATH"] = python312_dir + ";" + custom_env.get("PATH", "")

            work_dir = os.path.abspath(os.path.dirname(__file__))

            self.build_process = subprocess.Popen(
                cmd,
                cwd=work_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=custom_env,
                text=True,
                bufsize=1,
                encoding="utf-8",
                errors="replace"
            )

            for line in self.build_process.stdout:
                self.ui_queue.put(("build_log", line))

            return_code = self.build_process.wait()
            self.build_process = None

            if return_code == 0:
                msg = "🟢 UPLOAD SUCCESSFUL!" if upload else "🟢 COMPILE SUCCESSFUL!"
                self.ui_queue.put(("build_done", (True, msg, upload)))
            else:
                self.ui_queue.put(("build_done", (False, f"🔴 BUILD FAILED (Exit code {return_code})", upload)))

        except Exception as e:
            self.build_process = None
            self.ui_queue.put(("build_done", (False, f"🔴 BUILD ERROR: {e}", upload)))

    def _abort_build(self):
        if self.build_process:
            try:
                self.build_process.terminate()
                self._build_print("\n[ABORTED] Build process terminated by user.\n")
            except Exception:
                pass

    def _build_print(self, text, tag=None):
        self.txt_build.insert(END, text)
        self.txt_build.see(END)

    # =========================================================================
    # STATUS BAR
    # =========================================================================
    def _build_status_bar(self):
        sbar = ttk.Frame(self, padding=(15, 6))
        sbar.pack(fill=X, side=BOTTOM)

        self.lbl_status = ttk.Label(
            sbar,
            text="Ready. Select COM port and Connect to begin.",
            font=("Outfit", 9),
            bootstyle="secondary"
        )
        self.lbl_status.pack(side=LEFT)

        lbl_author = ttk.Label(
            sbar,
            text="Team-Aakashvani | CANSAT FW",
            font=("Outfit", 9, "bold"),
            bootstyle="secondary"
        )
        lbl_author.pack(side=RIGHT)

    # =========================================================================
    # PORT MANAGEMENT & CONNECTION
    # =========================================================================
    def _refresh_ports(self):
        coms = [p.device for p in serial.tools.list_ports.comports()]
        ports = ["WIFI: 192.168.4.1:2323"] + coms
        self.cb_ports["values"] = ports
        if coms:
            if "COM13" in coms:
                self.cb_ports.set("COM13")
            else:
                self.cb_ports.set(coms[0])
        else:
            self.cb_ports.set("WIFI: 192.168.4.1:2323")
        self._log_status(f"Discovered {len(coms)} serial ports + Wi-Fi target.")

    def _toggle_connection(self):
        if self.is_connected:
            self._disconnect_port()
        else:
            self._connect_port()

    def _connect_port(self):
        port = self.cb_ports.get().strip()
        baud = int(self.cb_baud.get())
        if not port:
            messagebox.showwarning("Port Missing", "Please select a valid COM port or WIFI target first.")
            return

        try:
            if port.upper().startswith("WIFI:"):
                target = port.split(":", 1)[1].strip()
                if ":" in target:
                    ip, p = target.split(":", 1)
                    p = int(p)
                else:
                    ip = target
                    p = 2323
                self.ser = NetworkSerial(ip, port=p, timeout=1.0)
                self.is_connected = True

                self.btn_connect.configure(text="✕ DISCONNECT", bootstyle="danger")
                self.lbl_conn_status.configure(text="● CONNECTED (WIFI)", bootstyle="success")
                self._log_status(f"Connected wirelessly to Wi-Fi target {ip}:{p}.")
                self._cli_print(f"\n[SYSTEM] Connected wirelessly to Wi-Fi target {ip}:{p}.\n")
            else:
                self.ser = serial.Serial(port, baud, timeout=0.5)
                # Ensure DTR/RTS do not trigger unexpected bootloader loops
                self.ser.dtr = False
                self.ser.rts = False
                self.is_connected = True

                self.btn_connect.configure(text="✕ DISCONNECT", bootstyle="danger")
                self.lbl_conn_status.configure(text="● CONNECTED", bootstyle="success")
                self._log_status(f"Connected to {port} at {baud} bps.")
                self._cli_print(f"\n[SYSTEM] Connected to {port} at {baud} bps.\n")
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to open {port}:\n{e}")

    def _disconnect_port(self):
        self.telemetry_running = False
        self.btn_toggle_telemetry.configure(text="▶ START LIVE TELEMETRY (200ms)", bootstyle="info")
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.is_connected = False
        self.btn_connect.configure(text="⚡ CONNECT", bootstyle="success")
        self.lbl_conn_status.configure(text="● DISCONNECTED", bootstyle="danger")
        self._log_status("Serial port disconnected.")

    def _change_theme(self, event=None):
        theme = self.cb_theme.get()
        self.style.theme_use(theme)

    # =========================================================================
    # MSP AUTOMATED TEST SUITE (THREADED)
    # =========================================================================
    def _run_msp_test_suite_thread(self):
        if not self.is_connected:
            messagebox.showwarning("Not Connected", "Please connect to a COM port before running the test suite.")
            return

        if self.telemetry_running:
            self._toggle_telemetry()  # Pause telemetry during suite test

        self.btn_run_suite.configure(state=DISABLED, text="⏳ TESTING IN PROGRESS...")
        self.msp_progress["value"] = 0
        self._populate_initial_msp_tree()

        threading.Thread(target=self._msp_test_worker, daemon=True).start()

    def _msp_test_worker(self):
        passed = 0
        failed = 0
        total = len(msp_test.MSP_COMMANDS)
        index = 0

        for cmd_name, cmd_code in msp_test.MSP_COMMANDS.items():
            index += 1
            # Update row to testing state
            self.ui_queue.put(("msp_row", (cmd_name, "TESTING...", "---", "---", "info")))

            # Ensure buffer is flushed before writing command (our fix!)
            try:
                self.ser.reset_input_buffer()
                packet = msp_test.serialize_msp_command(cmd_code, b"", 1)
                self.ser.write(packet)

                # Read response
                response_buffer = b""
                start_time = time.time()
                while time.time() - start_time < 0.8:
                    if self.ser.in_waiting > 0:
                        byte = self.ser.read(1)
                        response_buffer += byte
                        # Check for valid packet completion
                        if len(response_buffer) >= 6:
                            if response_buffer[0:3] == b"$M>" or response_buffer[0:3] == b"$M!":
                                size = response_buffer[3]
                                if len(response_buffer) >= 6 + size:
                                    break
                    time.sleep(0.002)

                if not response_buffer:
                    failed += 1
                    self.ui_queue.put(("msp_row", (cmd_name, "FAIL ❌", "No response / Timeout", "", "danger")))
                else:
                    rx_code, rx_data, is_err, rx_ver = msp_test.deserialize_msp_response(response_buffer)
                    if rx_code is not None and not is_err:
                        passed += 1
                        raw_hex = rx_data.hex() if rx_data else "00"
                        # Parse decoded summary
                        summary = self._decode_summary(cmd_name, rx_data)
                        self.ui_queue.put(("msp_row", (cmd_name, "PASS ✅", summary, raw_hex, "success")))
                    else:
                        failed += 1
                        self.ui_queue.put(("msp_row", (cmd_name, "FAIL ❌", "CRC Error or FC NACK", "", "danger")))
            except Exception as e:
                failed += 1
                self.ui_queue.put(("msp_row", (cmd_name, "FAIL ❌", str(e), "", "danger")))

            # Update summary & progress bar
            self.ui_queue.put(("msp_progress", (index, total, passed, failed)))
            time.sleep(0.03)

        self.ui_queue.put(("msp_done", (passed, failed)))

    def _decode_summary(self, cmd_name, raw_bytes):
        """Generates a human-friendly summary string for common MSP commands."""
        try:
            if cmd_name == "MSP_API_VERSION" and len(raw_bytes) >= 3:
                return f"MSP Protocol v{raw_bytes[0]}, API v{raw_bytes[1]}.{raw_bytes[2]}"
            elif cmd_name == "MSP_FC_VARIANT" and len(raw_bytes) >= 4:
                return f"Variant: {raw_bytes.decode('utf-8', errors='ignore')}"
            elif cmd_name == "MSP_FC_VERSION" and len(raw_bytes) >= 3:
                return f"Version {raw_bytes[0]}.{raw_bytes[1]}.{raw_bytes[2]}"
            elif cmd_name == "MSP_NAME":
                return f"FC Name: {raw_bytes.decode('utf-8', errors='ignore')}"
            elif cmd_name == "MSP_BUILD_INFO":
                return f"Build: {raw_bytes.decode('utf-8', errors='ignore')}"
            elif cmd_name == "MSP_ATTITUDE" and len(raw_bytes) >= 6:
                roll, pitch, yaw = struct.unpack("<hhh", raw_bytes[:6])
                return f"Roll: {roll/10.0:.1f}°, Pitch: {pitch/10.0:.1f}°, Yaw: {yaw}.0°"
            elif cmd_name == "MSP_STATUS_EX" and len(raw_bytes) >= 11:
                cyc, i2c_err, sens, flt = struct.unpack("<HHHH", raw_bytes[:8])
                return f"CycleTime: {cyc} us | I2C Err: {i2c_err} | Sensors: {bin(sens)}"
            elif cmd_name == "MSP_MOTOR" and len(raw_bytes) >= 8:
                motors = struct.unpack("<4H", raw_bytes[:8])
                return f"M1:{motors[0]}  M2:{motors[1]}  M3:{motors[2]}  M4:{motors[3]}"
            elif cmd_name == "MSP_PIDNAMES":
                names = [n for n in raw_bytes.decode('utf-8', errors='ignore').split(';') if n]
                return f"PID Channels: {', '.join(names[:6])}..."
            elif cmd_name == "MSP_BOXNAMES":
                names = [n for n in raw_bytes.decode('utf-8', errors='ignore').split(';') if n]
                return f"Modes: {', '.join([x for x in names if not x.isdigit()][:6])}..."
            elif cmd_name == "MSP_MOTOR_CONFIG" and len(raw_bytes) >= 6:
                min_t, max_t, min_c = struct.unpack("<HHH", raw_bytes[:6])
                return f"MinThr:{min_t} MaxThr:{max_t} MinCmd:{min_c}"
            return "OK (Received valid MSP packet)"
        except Exception:
            return "OK (Raw MSP payload received)"

    # =========================================================================
    # ONBOARD SENSOR HEALTH DETECTION & BENCH OTA TOOL METHODS
    # =========================================================================
    def _update_sensor_badges(self, sens_bitmask):
        # bit 0: ACCEL (1), bit 5: GYRO (32)
        if (sens_bitmask & 1) or (sens_bitmask & 32):
            self.badge_acc.configure(text=" ACCEL: ONLINE ", bootstyle="inverse-success")
            self.badge_gyro.configure(text=" GYRO: ONLINE ", bootstyle="inverse-success")
        else:
            self.badge_acc.configure(text=" ACCEL: OFFLINE ", bootstyle="inverse-danger")
            self.badge_gyro.configure(text=" GYRO: OFFLINE ", bootstyle="inverse-danger")

        # bit 1: BARO (2)
        if sens_bitmask & 2:
            self.badge_baro.configure(text=" BARO: ONLINE ", bootstyle="inverse-success")
        else:
            self.badge_baro.configure(text=" BARO: OFFLINE ", bootstyle="inverse-danger")

        # bit 2: MAG (4)
        if sens_bitmask & 4:
            self.badge_mag.configure(text=" MAG: ONLINE ", bootstyle="inverse-success")
        else:
            self.badge_mag.configure(text=" MAG: OFFLINE ", bootstyle="inverse-secondary")

        # bit 3: GPS (8)
        if sens_bitmask & 8:
            self.badge_gps.configure(text=" GPS: ONLINE ", bootstyle="inverse-success")
        else:
            self.badge_gps.configure(text=" GPS: OFFLINE ", bootstyle="inverse-secondary")

    def _run_i2c_scanner_dialog(self):
        if not self.is_connected or not self.ser:
            messagebox.showwarning("Not Connected", "Please connect to the ESP32 serial port first to scan the I2C bus.")
            return

        dlg = ttkb.Toplevel(self)
        dlg.title("Flight32 Live I2C Hardware Bus Scanner")
        dlg.geometry("540x440")
        dlg.transient(self)

        ttk.Label(dlg, text="🛰️ LIVE I2C HARDWARE BUS SCANNER (GPIO 21/22)", font=("Outfit", 12, "bold"), bootstyle="warning").pack(pady=15)
        ttk.Label(dlg, text="Scanning for onboard & external I2C sensor chips...", font=("Outfit", 10)).pack(pady=(0, 10))

        txt_scan = ScrolledText(dlg, font=("Consolas", 10), height=13, wrap=WORD, bootstyle="info")
        txt_scan.pack(fill=BOTH, expand=True, padx=15, pady=5)

        was_running = self.telemetry_running
        if was_running:
            self._toggle_telemetry()

        def scan_worker():
            try:
                time.sleep(0.1)
                self.ser.reset_input_buffer()
                self.ser.write(b"i2c_scan\n")
                start_t = time.time()
                output_lines = []
                while time.time() - start_t < 3.0:
                    if self.ser.in_waiting > 0:
                        raw = self.ser.read(self.ser.in_waiting).decode("utf-8", errors="replace")
                        for l in raw.splitlines():
                            if "Found I2C device" in l or "Scan complete" in l or "Scanning I2C" in l or "reg[" in l:
                                output_lines.append(l)
                    time.sleep(0.05)

                txt_scan.insert(END, "\n".join(output_lines) if output_lines else "No devices found or scan timed out.\n")
                txt_scan.insert(END, "\n\n=== Known Addresses ===\n0x68 : MPU6050/6500 IMU\n0x76/0x77 : BMP280 Barometer\n0x0C : AK8963 Magnetometer\n0x0D : QMC5883L Compass\n")
            except Exception as e:
                txt_scan.insert(END, f"Error scanning bus: {e}")
            finally:
                if was_running:
                    self._toggle_telemetry()

        threading.Thread(target=scan_worker, daemon=True).start()
        ttk.Button(dlg, text="CLOSE", bootstyle="secondary", command=dlg.destroy).pack(pady=12)

    def _trigger_bench_ota_reload(self):
        # Select Builder Tab and initiate compile & upload immediately
        self.notebook.select(self.tab_builder)
        self._start_build_thread(upload=True)

    # =========================================================================
    # LIVE TELEMETRY POLLING (THREADED)
    # =========================================================================
    def _toggle_telemetry(self):
        if not self.is_connected:
            messagebox.showwarning("Not Connected", "Please connect to a COM port first.")
            return

        if not self.telemetry_running:
            self.telemetry_running = True
            self.btn_toggle_telemetry.configure(text="✕ STOP LIVE TELEMETRY", bootstyle="danger")
            threading.Thread(target=self._telemetry_worker, daemon=True).start()
        else:
            self.telemetry_running = False
            self.btn_toggle_telemetry.configure(text="▶ START LIVE TELEMETRY (FAST REAL-TIME)", bootstyle="info")

    def _telemetry_worker(self):
        loop_counter = 0
        while self.telemetry_running and self.is_connected:
            try:
                loop_counter += 1
                # 1. Query Attitude (MSP_ATTITUDE = 108) - High Speed Real-Time (~25 Hz)
                self.ser.reset_input_buffer()
                pkt_att = msp_test.serialize_msp_command(108, b"", 1)
                self.ser.write(pkt_att)
                rx_att = self._read_packet_simple()
                if rx_att and len(rx_att) >= 6:
                    roll, pitch, yaw = struct.unpack("<hhh", rx_att[:6])
                    self.ui_queue.put(("telem_att", (roll / 10.0, pitch / 10.0, float(yaw))))

                time.sleep(0.005)

                # 2. Query Raw IMU 9-DOF (MSP_RAW_IMU = 106) - High Speed Real-Time (~25 Hz)
                self.ser.reset_input_buffer()
                pkt_imu = msp_test.serialize_msp_command(106, b"", 1)
                self.ser.write(pkt_imu)
                rx_imu = self._read_packet_simple()
                if rx_imu and len(rx_imu) >= 18:
                    ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack("<9h", rx_imu[:18])
                    self.ui_queue.put(("telem_raw_imu", (ax, ay, az, gx, gy, gz, mx, my, mz)))

                time.sleep(0.005)

                # 3. Query Motor Outputs (MSP_MOTOR = 104) - Medium Speed (~8 Hz)
                if loop_counter % 3 == 0:
                    self.ser.reset_input_buffer()
                    pkt_mot = msp_test.serialize_msp_command(104, b"", 1)
                    self.ser.write(pkt_mot)
                    rx_mot = self._read_packet_simple()
                    if rx_mot and len(rx_mot) >= 8:
                        m = struct.unpack("<4H", rx_mot[:8])
                        self.ui_queue.put(("telem_motors", m))
                    time.sleep(0.005)

                # 4. Query Status Ex & Environment - Periodic (~2.5 Hz)
                if loop_counter % 10 == 0:
                    self.ser.reset_input_buffer()
                    pkt_status = msp_test.serialize_msp_command(150, b"", 1)
                    self.ser.write(pkt_status)
                    rx_status = self._read_packet_simple()
                    if rx_status and len(rx_status) >= 11:
                        cyc, i2c_err, sens, flt_mode_flags = struct.unpack("<HHHH", rx_status[:8])
                        self.ui_queue.put(("telem_status", (cyc, i2c_err, sens, flt_mode_flags)))
                    time.sleep(0.005)

                    self.ser.reset_input_buffer()
                    pkt_env = msp_test.serialize_msp_command(240, b"", 1)
                    self.ser.write(pkt_env)
                    rx_env = self._read_packet_simple()
                    if rx_env and len(rx_env) >= 12:
                        temp_c, pressure_hpa, alt_m = struct.unpack("<3f", rx_env[:12])
                        self.ui_queue.put(("telem_env", (temp_c, pressure_hpa, alt_m)))

            except Exception:
                pass
            time.sleep(0.01)

    def _read_packet_simple(self):
        buf = b""
        start = time.time()
        while time.time() - start < 0.04:
            if self.ser.in_waiting > 0:
                buf += self.ser.read(self.ser.in_waiting)
                if len(buf) >= 6 and buf[:3] in (b"$M>", b"$M!"):
                    size = buf[3]
                    if len(buf) >= 6 + size:
                        return buf[5:5+size]
            time.sleep(0.001)
        return None

    # =========================================================================
    # INTERACTIVE CLI TERMINAL
    # =========================================================================
    def _send_cli_command(self, cmd_text):
        if not self.is_connected:
            messagebox.showwarning("Not Connected", "Please connect to a COM port first.")
            return

        cmd_text = cmd_text.strip()
        if not cmd_text:
            return

        if self.telemetry_running:
            self._toggle_telemetry()  # Pause telemetry while using CLI

        self.ent_cli.delete(0, END)
        self._cli_print(f"[Flight32 ~] > {cmd_text}\n", "command")

        threading.Thread(target=self._cli_worker, args=(cmd_text,), daemon=True).start()

    def _cli_worker(self, cmd_text):
        try:
            self.ser.reset_input_buffer()
            # Send command + LF line ending
            self.ser.write((cmd_text + "\r\n").encode("utf-8"))

            time.sleep(0.15)
            # Read output lines
            output = ""
            start = time.time()
            while time.time() - start < 0.6:
                if self.ser.in_waiting > 0:
                    output += self.ser.read(self.ser.in_waiting).decode("utf-8", errors="replace")
                time.sleep(0.05)

            self.ui_queue.put(("cli_output", output))
        except Exception as e:
            self.ui_queue.put(("cli_output", f"[ERROR] {e}\n"))

    def _cli_print(self, text, tag=None):
        self.txt_cli.insert(END, text)
        self.txt_cli.see(END)

    # =========================================================================
    # THREAD-SAFE UI QUEUE PUMP
    # =========================================================================
    def _process_ui_queue(self):
        while not self.ui_queue.empty():
            try:
                msg_type, data = self.ui_queue.get_nowait()

                if msg_type == "msp_row":
                    cmd_name, status_txt, decoded_txt, raw_txt, style = data
                    item = self.tree_msp.exists(cmd_name)
                    if item:
                        code_str = str(msp_test.MSP_COMMANDS.get(cmd_name, ""))
                        self.tree_msp.item(
                            cmd_name,
                            values=(cmd_name, code_str, status_txt, decoded_txt, raw_txt),
                            tags=(style,)
                        )
                        self.tree_msp.tag_configure("success", foreground="#00e676")
                        self.tree_msp.tag_configure("danger", foreground="#ff1744")
                        self.tree_msp.tag_configure("info", foreground="#00b0ff")
                        self.tree_msp.see(cmd_name)

                elif msg_type == "msp_progress":
                    idx, total, passed, failed = data
                    self.msp_progress["value"] = idx
                    self.lbl_msp_stats.configure(
                        text=f"Total: {total} | ✅ Pass: {passed} | ❌ Fail: {failed}"
                    )

                elif msg_type == "msp_done":
                    passed, failed = data
                    self.btn_run_suite.configure(
                        state=NORMAL,
                        text="⚡ RUN 31-COMMAND MSP TEST SUITE"
                    )
                    self._log_status(f"MSP Test Suite Finished! {passed} Passed, {failed} Failed.")
                    if failed == 0:
                        messagebox.showinfo("Suite Passed! 🎉", f"All {passed}/{passed} MSP Commands verified successfully!")

                elif msg_type == "telem_att":
                    roll, pitch, yaw = data
                    yaw_circ = (yaw % 360.0 + 360.0) % 360.0  # Circular 0..360 compass angle
                    self.lbl_roll_val.configure(text=f"{roll:.1f}°")
                    self.lbl_pitch_val.configure(text=f"{pitch:.1f}°")
                    self.lbl_yaw_val.configure(text=f"{yaw_circ:.0f}°")
                    # Map progress bars safely
                    self.bar_roll["value"] = max(0, min(360, roll + 180))
                    self.bar_pitch["value"] = max(0, min(180, pitch + 90))
                    self.bar_yaw["value"] = max(0, min(360, yaw_circ))

                elif msg_type == "telem_status":
                    cyc, i2c_err, sens, flt_flags = data
                    self.card_cycletime.configure(text=f"{cyc} us")
                    self.card_i2c_err.configure(text=f"{i2c_err}")
                    self._update_sensor_badges(sens)

                elif msg_type == "telem_motors":
                    motors = data
                    for i in range(4):
                        val = motors[i]
                        self.motor_labels[i].configure(text=str(val))
                        self.motor_bars[i]["value"] = max(0, min(1000, val - 1000))

                elif msg_type == "telem_raw_imu":
                    ax, ay, az, gx, gy, gz, mx, my, mz = data
                    self.lbl_acc_x.configure(text=f"{ax/1000.0:+.2f} G")
                    self.lbl_acc_y.configure(text=f"{ay/1000.0:+.2f} G")
                    self.lbl_acc_z.configure(text=f"{az/1000.0:+.2f} G")
                    self.lbl_gyro_x.configure(text=f"{gx/1000.0:+.1f} °/s")
                    self.lbl_gyro_y.configure(text=f"{gy/1000.0:+.1f} °/s")
                    self.lbl_gyro_z.configure(text=f"{gz/1000.0:+.1f} °/s")
                    self.lbl_mag_x.configure(text=f"{mx:.1f} uT")
                    self.lbl_mag_y.configure(text=f"{my:.1f} uT")
                    self.lbl_mag_z.configure(text=f"{mz:.1f} uT")

                elif msg_type == "telem_env":
                    temp_c, pressure_hpa, alt_m = data
                    self.card_temperature.configure(text=f"{temp_c:.1f} °C")
                    self.card_pressure.configure(text=f"{pressure_hpa:.1f}")
                    self.lbl_altitude.configure(text=f"{alt_m:.2f} m")

                elif msg_type == "cli_output":
                    self._cli_print(data)

                elif msg_type == "build_log":
                    self._build_print(data)

                elif msg_type == "build_done":
                    success, msg, was_upload = data
                    self.btn_compile_only.configure(state=NORMAL)
                    self.btn_compile_flash.configure(state=NORMAL)
                    self.btn_abort_build.configure(state=DISABLED)
                    style = "success" if success else "danger"
                    self.lbl_build_badge.configure(text=msg, bootstyle=style)
                    self._log_status(msg)
                    self._build_print(f"\n[DONE] {msg}\n")

                    if was_upload and success and self.reconnect_after_upload:
                        self._log_status("Reconnecting serial port in 3.0s...")
                        self.after(3000, self._connect_port)

            except queue.Empty:
                break
            except Exception as e:
                print(f"UI Queue Error: {e}")

        self.after(50, self._process_ui_queue)

    def _log_status(self, msg):
        self.lbl_status.configure(text=f"⚡ {msg}")


if __name__ == "__main__":
    app = Flight32GroundStation()
    app.mainloop()
