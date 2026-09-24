#!/usr/bin/env python3
"""
DA WINNA — MT6816 / MT6818 SPI Encoder & Closed-Loop KV Test Suite GUI (GM3506 @ 12V)
Features:
  - High-Speed 921600 Baud Telemetry & Command Engine
  - Real-time 14-Bit Angular Position Visualizer & Compass Dial
  - Live Real-Time Motion Parameter Tuning Panel (⚡ Live Tunable over Serial):
      * Output Voltage Ramp Rate (V/s, e.g. 1000 V/s)
      * Voltage Limit (V) & Maximum Velocity Limit (RPM)
      * Live Velocity PID (P, I, D) & LPF Filter Tf (s)
      * Back-EMF Feedforward KV Rating (RPM/V)
      * Dedicated "Reset to GM3506 Defaults (12V)" Action
  - Hardware & Inverter Specifications Inspector (Compile-Time Parameters):
      * PWM Carrier Frequency (25 kHz)
      * Motor Pole Pairs (11 PP / 24N22P)
      * SPI Master Clock (4.0 MHz) & Alignment Voltage (3.0 V)
  - 2-Minute Automated PID Sweep & Speed Variation Benchmark (Auto-Tuner)
  - Ranked Leaderboard Modal with One-Click Winner Preset Application
  - ⚡ High-Precision Closed-Loop KV Analyzer Lab:
      * Real-Time Scatter Plot + Linear Regression Best-Fit Line (RPM vs Uq)
      * Forward (CW) vs Reverse (CCW) Directional Symmetry Analysis
      * Physical Constants Extraction: True KV, R^2 Linearity, V0 Friction Offset, Ke, Kt
      * Real-Time Data Point Table with Residual Errors and Ripple
      * Export Test Report to CSV and Formatted Text
  - Speed Presets (0, 50, 100, 200, 400 RPM) & Dual-Trace Oscilloscope
  - Stall & Overload Detection Alarm Badge & Guard Toggle
  - Interactive Serial Console with Direct Command Entry & Auto-Scroll Pause
  - Full support for 4-Wire Hardware SPI / 3-Wire Half-Duplex
"""

import sys
import os
import re
import time
import math
import queue
import threading
import subprocess
import shutil
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports

def find_pio_executable():
    for name in ["pio", "platformio", "pio.exe", "platformio.exe"]:
        p = shutil.which(name)
        if p:
            return p
    user_home = os.path.expanduser("~")
    candidates = [
        os.path.join(user_home, ".platformio", "penv", "Scripts", "platformio.exe"),
        os.path.join(user_home, ".platformio", "penv", "Scripts", "pio.exe"),
        os.path.join(user_home, ".platformio", "penv", "bin", "platformio"),
        os.path.join(user_home, ".platformio", "penv", "bin", "pio"),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    return "platformio"

def find_esp32_port():
    for p in serial.tools.list_ports.comports():
        hw = (p.hwid or "").upper()
        desc = (p.description or "").upper()
        if "303A" in hw or "1A86" in hw or "10C4" in hw or "ESP" in desc or "CH34" in desc or "CP210" in desc or "USB" in desc or "SERIAL" in desc:
            return p.device
    return "COM5"

class ESP32SerialReader:
    def __init__(self, port=None, baud=921600):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = False
        self.connected = False
        self.thread = None
        self.raw_log_queue = queue.Queue(maxsize=3000)
        self.lock = threading.Lock()

        # Telemetry State
        self.last_angle_deg = 0.0
        self.last_angle_rad = 0.0
        self.last_raw_14bit = 0
        self.last_target_rpm = 0.0
        self.last_actual_rpm = 0.0
        self.last_uq = 0.0
        self.last_ud = 0.0
        self.last_pwm_deg = 0.0
        self.last_pwm_duty = 0.0
        self.last_pwm_freq = 0
        self.last_control_mode = "DISABLED"
        self.last_bus_mode = "4-Wire"
        self.last_lut_active = False
        self.last_mag_ok = True
        self.last_parity_ok = True
        self.last_stall_alarm = False
        self.last_stall_guard = False
        self.calibrating = False
        self.last_calib_zero_angle = None
        self.msg_count = 0

        # KV Test Streaming & Results State
        self.kv_test_active = False
        self.kv_points = []
        self.kv_results = {}

        self.history_time = []
        self.history_target_rpm = []
        self.history_actual_rpm = []
        self.history_angle = []
        self.start_time = time.time()

    def connect(self):
        if not self.port:
            self.port = find_esp32_port()
        if not self.port:
            raise RuntimeError("No ESP32-S3 USB Serial port found! Ensure ESP32-S3 is plugged in.")
        try:
            self.disconnect()
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=0.2)
            self.ser.dtr = True
            self.ser.rts = True
            self.running = True
            self.connected = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            self.running = False
            self.connected = False
            raise e

    def disconnect(self):
        self.running = False
        self.connected = False
        if self.ser:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
        self.ser = None

    def send_cmd(self, cmd_str):
        if self.ser and self.ser.is_open and self.connected:
            try:
                if not cmd_str.endswith("\n"):
                    cmd_str += "\n"
                self.ser.write(cmd_str.encode("utf-8"))
            except Exception:
                self.connected = False

    def _read_loop(self):
        buffer = ""
        while self.running:
            if not self.ser or not self.ser.is_open or not self.connected:
                time.sleep(0.5)
                try:
                    port = self.port or find_esp32_port()
                    if port:
                        self.port = port
                        self.ser = serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=0.2)
                        self.ser.dtr = True
                        self.ser.rts = True
                        self.connected = True
                        buffer = ""
                except Exception:
                    continue

            try:
                data = self.ser.read(256).decode("utf-8", errors="ignore")
                if not data:
                    time.sleep(0.005)
                    continue
                buffer += data
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip("\r")
                    if line:
                        try:
                            self.raw_log_queue.put_nowait(line)
                        except queue.Full:
                            try:
                                self.raw_log_queue.get_nowait()
                                self.raw_log_queue.put_nowait(line)
                            except Exception:
                                pass
                        self._parse_line(line)
            except Exception:
                self.connected = False
                try:
                    if self.ser and self.ser.is_open:
                        self.ser.close()
                except Exception:
                    pass
                time.sleep(0.2)

    def _parse_line(self, line):
        if "SAFETY SHUTDOWN: Motor Stall" in line or "Motor stall/overload" in line or "Motor Stall / Overload" in line:
            with self.lock:
                self.last_stall_alarm = True
                self.last_control_mode = "DISABLED"
            return

        if "[KV_START]" in line:
            with self.lock:
                self.kv_test_active = True
                self.kv_points = []
                self.kv_results = {}
            return

        if "[KV_POINT]:" in line:
            try:
                m_stg = re.search(r'Stage=(\d+)/(\d+)', line)
                m_v = re.search(r'V=([+-]?\d+\.?\d*)', line)
                m_rpm = re.search(r'RPM=([+-]?\d+\.?\d*)', line)
                m_rip = re.search(r'Ripple=([+-]?\d+\.?\d*)', line)
                m_sd = re.search(r'StdDev=([+-]?\d+\.?\d*)', line)
                if m_v and m_rpm:
                    pt = {
                        "stage": int(m_stg.group(1)) if m_stg else len(self.kv_points) + 1,
                        "total_stages": int(m_stg.group(2)) if m_stg else 18,
                        "voltage": float(m_v.group(1)),
                        "rpm": float(m_rpm.group(1)),
                        "ripple": float(m_rip.group(1)) if m_rip else 0.0,
                        "std_dev": float(m_sd.group(1)) if m_sd else 0.0
                    }
                    with self.lock:
                        self.kv_points.append(pt)
                        self.kv_test_active = True
            except Exception:
                pass
            return

        if "[KV_RESULT]:" in line:
            try:
                res = {}
                for item in line.split("[KV_RESULT]:")[1].split(","):
                    if "=" in item:
                        k, v = item.strip().split("=", 1)
                        res[k.strip()] = float(v.strip())
                with self.lock:
                    self.kv_results = res
                    self.kv_test_active = False
            except Exception:
                pass
            return

        if "KV TEST ABORTED" in line:
            with self.lock:
                self.kv_test_active = False
            return

        if "STARTING COMPREHENSIVE MULTI-POINT ELECTRICAL ZERO CALIBRATION" in line:
            with self.lock:
                self.calibrating = True
                self.last_calib_zero_angle = None
            return

        if "motor.zero_electric_angle =" in line:
            m = re.search(r'motor\.zero_electric_angle\s*=\s*([0-9]+\.[0-9]+)', line)
            if m:
                with self.lock:
                    self.last_calib_zero_angle = float(m.group(1))
                    self.last_zero_electric_angle = float(m.group(1))
                    self.calibrating = False

        if "Updated Base Electrical Zero Angle:" in line:
            m = re.search(r'([0-9]+\.[0-9]+)\s*rad', line)
            if m:
                with self.lock:
                    self.last_zero_electric_angle = float(m.group(1))

        if "Driver ENABLED" in line:
            with self.lock:
                self.last_control_mode = "ENABLED"
        elif "Driver DISABLED" in line or "MOTOR STOPPED" in line:
            with self.lock:
                self.last_control_mode = "DISABLED"
        elif "FOC Calibration Complete" in line or "CERTIFIED ELECTRICAL ALIGNMENT RESULTS" in line:
            with self.lock:
                self.last_control_mode = "CLOSED-LOOP FOC"
                self.calibrating = False
        elif "OPEN-LOOP Velocity Mode" in line:
            with self.lock:
                self.last_control_mode = "OPEN-LOOP"
        elif "Stall Watchdog: ENABLED" in line:
            with self.lock:
                self.last_stall_guard = True
        elif "Stall Watchdog: DISABLED" in line:
            with self.lock:
                self.last_stall_guard = False
        elif "3-Wire Half-Duplex" in line:
            with self.lock:
                self.last_bus_mode = "3-Wire"
        elif "4-Wire Full Hardware" in line:
            with self.lock:
                self.last_bus_mode = "4-Wire"

        if "Ctrl:" in line and "Angle:" in line:
            try:
                m_ang = re.search(r'Angle:\s*([+-]?\d+\.?\d*)\s*deg', line)
                angle_deg = float(m_ang.group(1)) if m_ang else self.last_angle_deg

                m_tgt = re.search(r'Target:\s*([+-]?\d+\.?\d*)\s*(RPM|V)', line)
                target_val = float(m_tgt.group(1)) if m_tgt else self.last_target_rpm

                m_act = re.search(r'Actual:\s*([+-]?\d+\.?\d*)\s*RPM', line)
                actual_rpm = float(m_act.group(1)) if m_act else self.last_actual_rpm

                m_raw = re.search(r'Raw:\s*(\d+)/16383', line)
                raw_count = int(m_raw.group(1)) if m_raw else int((angle_deg / 360.0) * 16384)

                m_uq = re.search(r'Uq:\s*([+-]?\d+\.?\d*)V', line)
                uq = float(m_uq.group(1)) if m_uq else self.last_uq

                m_ud = re.search(r'Ud:\s*([+-]?\d+\.?\d*)V', line)
                ud = float(m_ud.group(1)) if m_ud else self.last_ud

                ctrl_mode = "CLOSED-LOOP FOC"
                if "Ctrl: DISBL" in line:
                    ctrl_mode = "DISABLED"
                elif "Ctrl: KV-TEST" in line:
                    ctrl_mode = "KV-TEST (VOLT)"
                elif "Ctrl: OPEN" in line:
                    ctrl_mode = "OPEN-LOOP"
                elif "Ctrl: VOLT" in line:
                    ctrl_mode = "CLOSED-LOOP VOLTAGE"
                elif "Ctrl: CLOSE" in line:
                    ctrl_mode = "CLOSED-LOOP VELOCITY"

                bus_mode = "3-Wire" if "Mode: 3-Wire" in line else "4-Wire"
                lut_active = "(FOURIER:128)" in line or "128-Point LUT" in line or "128-PT LUT" in line
                mag_ok = "[ OK ]" in line or "[MAGNET OK]" in line
                parity_ok = "[PASS]" in line or "Parity: [ PASS ]" in line

                if "DISBL" in ctrl_mode and self.last_stall_alarm and target_val == 0.0:
                    pass
                else:
                    self.last_stall_alarm = False

                t_now = time.time() - self.start_time
                with self.lock:
                    self.last_angle_deg = angle_deg
                    self.last_angle_rad = (angle_deg / 180.0) * math.pi
                    self.last_raw_14bit = raw_count
                    self.last_target_rpm = target_val
                    self.last_actual_rpm = actual_rpm
                    self.last_uq = uq
                    self.last_ud = ud
                    self.last_control_mode = ctrl_mode
                    self.last_bus_mode = bus_mode
                    self.last_lut_active = lut_active
                    self.last_mag_ok = mag_ok
                    self.last_parity_ok = parity_ok
                    self.msg_count += 1

                    self.history_time.append(t_now)
                    self.history_target_rpm.append(target_val)
                    self.history_actual_rpm.append(actual_rpm)
                    self.history_angle.append(angle_deg)
                    if len(self.history_time) > 150:
                        self.history_time.pop(0)
                        self.history_target_rpm.pop(0)
                        self.history_actual_rpm.pop(0)
                        self.history_angle.pop(0)
            except Exception:
                pass


class MT6818DashboardGUI:
    def __init__(self, root, reader):
        self.root = root
        self.reader = reader
        self.root.title("DA WINNA — MT6816 / MT6818 SimpleFOC & Closed-Loop KV Test Suite (GM3506 @ 12V)")
        self.root.geometry("1280x920")
        self.root.minsize(1140, 780)
        self.root.configure(bg="#12161E")

        self.autoscroll_enabled = True

        # PID Benchmark Runner State
        self.benchmark_running = False
        self.benchmark_thread = None
        self.benchmark_results = []
        self.benchmark_progress_var = tk.DoubleVar(value=0.0)
        self.benchmark_status_var = tk.StringVar(value="Ready for 2-Min PID Benchmark")

        # Symmetry & Phase Angle Equalizer State
        self.current_zero_angle_rad = 0.11275
        self.sym_running = False
        self.sym_thread = None
        self.sym_fwd_rpm = 0.0
        self.sym_rev_rpm = 0.0
        self.sym_status_var = tk.StringVar(value="Ready to test KV Symmetry & Phase Angle Balance.")
        self.sym_progress_var = tk.DoubleVar(value=0.0)

        # KV Test Engine State
        self.kv_progress_var = tk.DoubleVar(value=0.0)
        self.kv_status_var = tk.StringVar(value="Ready for Closed-Loop KV Sweep Test.")

        self._setup_styles()
        self._build_ui()
        self._schedule_update()

    def _setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#12161E")
        style.configure("Card.TFrame", background="#1A2230", relief="flat")
        style.configure("Header.TLabel", background="#12161E", foreground="#00E5FF", font=("Segoe UI", 15, "bold"))
        style.configure("SubHeader.TLabel", background="#12161E", foreground="#8A99AD", font=("Segoe UI", 9))
        style.configure("CardTitle.TLabel", background="#1A2230", foreground="#8A99AD", font=("Segoe UI", 9, "bold"))
        style.configure("CardVal.TLabel", background="#1A2230", foreground="#FFFFFF", font=("Segoe UI", 18, "bold"))

        style.configure("TNotebook", background="#12161E", borderwidth=0)
        style.configure("TNotebook.Tab", background="#1A2230", foreground="#8A99AD", font=("Segoe UI", 10, "bold"), padding="18 8")
        style.map("TNotebook.Tab",
                  background=[("selected", "#00E5FF"), ("active", "#263238")],
                  foreground=[("selected", "#000000"), ("active", "#FFFFFF")])

    def _build_ui(self):
        # 1. Top Header Bar
        header_frame = ttk.Frame(self.root, padding="15 10 15 8")
        header_frame.pack(fill="x")

        ttk.Label(header_frame, text="🏎️ DA WINNA — MT6816 SPI ENCODER & MOTOR 1 BENCH (GM3506 @ 12V)", style="Header.TLabel").pack(side="left")

        self.conn_label = tk.Label(header_frame, text="DISCONNECTED", font=("Segoe UI", 10, "bold"), fg="#FF5252", bg="#12161E")
        self.conn_label.pack(side="right", padx=8)

        self.port_entry = tk.Entry(header_frame, width=10, font=("Segoe UI", 10), bg="#222D3E", fg="#FFFFFF", insertbackground="#FFF")
        self.port_entry.insert(0, self.reader.port or "COM3")
        self.port_entry.pack(side="right", padx=4)

        self.btn_connect = tk.Button(header_frame, text="Connect", font=("Segoe UI", 9, "bold"), bg="#00E5FF", fg="#000",
                                     activebackground="#00B4D8", command=self._toggle_connect, relief="flat", padx=8)
        self.btn_connect.pack(side="right", padx=4)

        # 2. Main Notebook Tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=12, pady=(0, 8))

        self.tab_dash = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_dash, text="  🏎️ Live Dashboard & Motion Tuning  ")

        self.tab_kv = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_kv, text="  ⚡ Closed-Loop KV Analyzer  ")

        self.tab_sym = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_sym, text="  ⚖️ Phase Angle & KV Equalizer  ")

        self._build_dashboard_tab(self.tab_dash)
        self._build_kv_analyzer_tab(self.tab_kv)
        self._build_symmetry_tab(self.tab_sym)

    def _build_dashboard_tab(self, parent):
        main_paned = tk.PanedWindow(parent, orient="horizontal", bg="#12161E", bd=0, sashwidth=4)
        main_paned.pack(fill="both", expand=True, padx=0, pady=4)

        # LEFT COLUMN: Gauges, Real-Time Tuning & Fixed Hardware Config
        left_col = ttk.Frame(main_paned)
        main_paned.add(left_col, minsize=560)

        # Dial Gauge Card
        gauge_card = ttk.Frame(left_col, style="Card.TFrame", padding=8)
        gauge_card.pack(fill="x", pady=4)

        ttk.Label(gauge_card, text="14-BIT ABSOLUTE ANGLE COMPASS & COMMUTATION POSITION", style="CardTitle.TLabel").pack(anchor="w")
        self.gauge_canvas = tk.Canvas(gauge_card, bg="#151C28", height=170, highlightthickness=0)
        self.gauge_canvas.pack(fill="x", expand=True, pady=4)

        # Telemetry Value Grid
        grid_frame = ttk.Frame(left_col)
        grid_frame.pack(fill="x", pady=4)

        self.card_angle = self._create_card(grid_frame, "MEASURED ANGLE", "0.00°", "#00E5FF", 0, 0)
        self.card_target_rpm = self._create_card(grid_frame, "TARGET VELOCITY", "0.0 RPM", "#FFD600", 0, 1)
        self.card_actual_rpm = self._create_card(grid_frame, "ACTUAL VELOCITY", "0.0 RPM", "#69F0AE", 1, 0)
        self.card_raw = self._create_card(grid_frame, "RAW 14-BIT / CPR", "0 / 16383", "#B388FF", 1, 1)

        # Health Badges Bar
        badge_bar = ttk.Frame(left_col, style="Card.TFrame", padding="8 6 8 6")
        badge_bar.pack(fill="x", pady=4)

        self.lbl_mode = tk.Label(badge_bar, text="CTRL: IDLE", font=("Segoe UI", 9, "bold"), bg="#455A64", fg="#FFF", padx=6, pady=2)
        self.lbl_mode.pack(side="left", padx=3)

        self.lbl_bus = tk.Label(badge_bar, text="BUS: 4-WIRE", font=("Segoe UI", 9, "bold"), bg="#0288D1", fg="#FFF", padx=6, pady=2)
        self.lbl_bus.pack(side="left", padx=3)

        self.lbl_lut = tk.Label(badge_bar, text="LUT: RAW", font=("Segoe UI", 9, "bold"), bg="#455A64", fg="#B0BEC5", padx=6, pady=2)
        self.lbl_lut.pack(side="left", padx=3)

        self.lbl_mag = tk.Label(badge_bar, text="MAGNET: OK", font=("Segoe UI", 9, "bold"), bg="#69F0AE", fg="#1A2230", padx=6, pady=2)
        self.lbl_mag.pack(side="left", padx=3)

        self.lbl_parity = tk.Label(badge_bar, text="PARITY: PASS", font=("Segoe UI", 9, "bold"), bg="#69F0AE", fg="#1A2230", padx=6, pady=2)
        self.lbl_parity.pack(side="left", padx=3)

        self.lbl_stall = tk.Label(badge_bar, text="STALL: OK", font=("Segoe UI", 9, "bold"), bg="#455A64", fg="#B0BEC5", padx=6, pady=2)
        self.lbl_stall.pack(side="left", padx=3)

        # -------------------------------------------------------------
        # ⚙️ LIVE REAL-TIME PARAMETER TUNING PANEL (REAL-TIME OVER SERIAL)
        # -------------------------------------------------------------
        pid_card = ttk.Frame(left_col, style="Card.TFrame", padding=10)
        pid_card.pack(fill="x", pady=4)

        hdr_row = ttk.Frame(pid_card)
        hdr_row.grid(row=0, column=0, columnspan=6, sticky="ew", pady=(0, 6))

        ttk.Label(hdr_row, text="⚡ REAL-TIME LIVE TUNABLE PARAMETERS (UPDATES INSTANTLY)", style="CardTitle.TLabel").pack(side="left")
        
        tk.Button(hdr_row, text="🔄 Reset 12V Defaults", font=("Segoe UI", 8, "bold"), bg="#FF6D00", fg="#FFF",
                  relief="flat", padx=6, pady=1, command=self._reset_gm3506_defaults).pack(side="right")

        # Row 1: Output Voltage Ramp Rate & Voltage Limit
        ttk.Label(pid_card, text="Volt Ramp:", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#64FFDA").grid(row=1, column=0, sticky="w", padx=2, pady=2)
        self.entry_ramp = tk.Entry(pid_card, width=7, font=("Segoe UI", 9), bg="#222D3E", fg="#64FFDA", insertbackground="#FFF")
        self.entry_ramp.insert(0, "200.0")
        self.entry_ramp.grid(row=1, column=1, padx=2, pady=2)
        tk.Button(pid_card, text="Set V/s", font=("Segoe UI", 8, "bold"), bg="#64FFDA", fg="#000", relief="flat", command=lambda: self._set_param("a", self.entry_ramp.get())).grid(row=1, column=2, padx=2, pady=2)

        ttk.Label(pid_card, text="Volt Limit:", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#B0BEC5").grid(row=1, column=3, sticky="w", padx=2, pady=2)
        self.entry_v = tk.Entry(pid_card, width=7, font=("Segoe UI", 9), bg="#222D3E", fg="#FFF", insertbackground="#FFF")
        self.entry_v.insert(0, "12.0")
        self.entry_v.grid(row=1, column=4, padx=2, pady=2)
        tk.Button(pid_card, text="Set V", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000", relief="flat", command=lambda: self._set_param("l", self.entry_v.get())).grid(row=1, column=5, padx=2, pady=2)

        # Row 2: P Gain, I Gain
        ttk.Label(pid_card, text="P Gain:", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#B0BEC5").grid(row=2, column=0, sticky="w", padx=2, pady=2)
        self.entry_p = tk.Entry(pid_card, width=7, font=("Segoe UI", 9), bg="#222D3E", fg="#FFF", insertbackground="#FFF")
        self.entry_p.insert(0, "0.12")
        self.entry_p.grid(row=2, column=1, padx=2, pady=2)
        tk.Button(pid_card, text="Set P", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000", relief="flat", command=lambda: self._set_param("p", self.entry_p.get())).grid(row=2, column=2, padx=2, pady=2)

        ttk.Label(pid_card, text="I Gain:", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#B0BEC5").grid(row=2, column=3, sticky="w", padx=2, pady=2)
        self.entry_i = tk.Entry(pid_card, width=7, font=("Segoe UI", 9), bg="#222D3E", fg="#FFF", insertbackground="#FFF")
        self.entry_i.insert(0, "1.20")
        self.entry_i.grid(row=2, column=4, padx=2, pady=2)
        tk.Button(pid_card, text="Set I", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000", relief="flat", command=lambda: self._set_param("i", self.entry_i.get())).grid(row=2, column=5, padx=2, pady=2)

        # Row 3: D Gain, LPF Tf
        ttk.Label(pid_card, text="D Gain:", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#B0BEC5").grid(row=3, column=0, sticky="w", padx=2, pady=2)
        self.entry_d = tk.Entry(pid_card, width=7, font=("Segoe UI", 9), bg="#222D3E", fg="#FFF", insertbackground="#FFF")
        self.entry_d.insert(0, "0.000")
        self.entry_d.grid(row=3, column=1, padx=2, pady=2)
        tk.Button(pid_card, text="Set D", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000", relief="flat", command=lambda: self._set_param("d", self.entry_d.get())).grid(row=3, column=2, padx=2, pady=2)

        ttk.Label(pid_card, text="LPF Tf (s):", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#B0BEC5").grid(row=3, column=3, sticky="w", padx=2, pady=2)
        self.entry_f = tk.Entry(pid_card, width=7, font=("Segoe UI", 9), bg="#222D3E", fg="#FFF", insertbackground="#FFF")
        self.entry_f.insert(0, "0.005")
        self.entry_f.grid(row=3, column=4, padx=2, pady=2)
        tk.Button(pid_card, text="Set Tf", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000", relief="flat", command=lambda: self._set_param("f", self.entry_f.get())).grid(row=3, column=5, padx=2, pady=2)

        # Row 4: Max RPM Limit & Target Speed
        ttk.Label(pid_card, text="Max RPM:", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#FF80AB").grid(row=4, column=0, sticky="w", padx=2, pady=2)
        self.entry_max_rpm = tk.Entry(pid_card, width=7, font=("Segoe UI", 9), bg="#222D3E", fg="#FF80AB", insertbackground="#FFF")
        self.entry_max_rpm.insert(0, "1432.0")
        self.entry_max_rpm.grid(row=4, column=1, padx=2, pady=2)
        tk.Button(pid_card, text="Set Max", font=("Segoe UI", 8, "bold"), bg="#FF80AB", fg="#000", relief="flat", command=lambda: self._set_param("m", self.entry_max_rpm.get())).grid(row=4, column=2, padx=2, pady=2)

        ttk.Label(pid_card, text="Target RPM:", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#FFD600").grid(row=4, column=3, sticky="w", padx=2, pady=2)
        self.entry_target = tk.Entry(pid_card, width=7, font=("Segoe UI", 9, "bold"), bg="#222D3E", fg="#FFD600", insertbackground="#FFF")
        self.entry_target.insert(0, "0.0")
        self.entry_target.grid(row=4, column=4, padx=2, pady=2)
        tk.Button(pid_card, text="Set RPM", font=("Segoe UI", 8, "bold"), bg="#FFD600", fg="#000", relief="flat", command=lambda: self._set_param("t", self.entry_target.get())).grid(row=4, column=5, padx=2, pady=2)

        # Row 5: Dynamic Sensor Lag Compensation & Motor KV Rating
        ttk.Label(pid_card, text="Lag (µs):", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#FFD54F").grid(row=5, column=0, sticky="w", padx=2, pady=2)
        self.entry_lag = tk.Entry(pid_card, width=7, font=("Segoe UI", 9), bg="#222D3E", fg="#FFD54F", insertbackground="#FFF")
        self.entry_lag.insert(0, "35.0")
        self.entry_lag.grid(row=5, column=1, padx=2, pady=2)
        tk.Button(pid_card, text="Set µs", font=("Segoe UI", 8, "bold"), bg="#FFD54F", fg="#000", relief="flat", command=lambda: self._set_param("s", self.entry_lag.get())).grid(row=5, column=2, padx=2, pady=2)

        ttk.Label(pid_card, text="Motor KV:", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#B388FF").grid(row=5, column=3, sticky="w", padx=2, pady=2)
        self.entry_kv = tk.Entry(pid_card, width=7, font=("Segoe UI", 9, "bold"), bg="#222D3E", fg="#B388FF", insertbackground="#FFF")
        self.entry_kv.insert(0, "171.9")
        self.entry_kv.grid(row=5, column=4, padx=2, pady=2)
        tk.Button(pid_card, text="Set KV", font=("Segoe UI", 8, "bold"), bg="#B388FF", fg="#000", relief="flat", command=lambda: self._set_param("q", self.entry_kv.get())).grid(row=5, column=5, padx=2, pady=2)

        # Speed Presets Bar
        preset_bar = ttk.Frame(pid_card)
        preset_bar.grid(row=6, column=0, columnspan=6, sticky="ew", pady=(6, 0))
        for spd in [0, 50, 100, 200, 400]:
            btn_col = "#D50000" if spd == 0 else "#2979FF"
            tk.Button(preset_bar, text=f"{spd} RPM", font=("Segoe UI", 8, "bold"), bg=btn_col, fg="#FFF", relief="flat",
                      command=lambda s=spd: self._set_param("t", str(s))).pack(side="left", fill="x", expand=True, padx=2)

        # -------------------------------------------------------------
        # 🔒 COMPILE-TIME HARDWARE & INVERTER CONFIGURATION INFO
        # -------------------------------------------------------------
        hw_info_card = ttk.Frame(left_col, style="Card.TFrame", padding="8 6 8 6")
        hw_info_card.pack(fill="x", pady=4)

        ttk.Label(hw_info_card, text="🔒 COMPILE-TIME HARDWARE CONSTANTS (EDIT IN MAIN.CPP)", style="CardTitle.TLabel").pack(anchor="w", pady=(0, 2))
        info_txt = "PWM Carrier: 25.0 kHz | Pole Pairs: 11 (GM3506) | Align Volt: 3.0V | SPI Baud: 4.0 MHz | Pins: PA8,PA9,PA10,PB3 EN"
        tk.Label(hw_info_card, text=info_txt, font=("Segoe UI", 8), bg="#1A2230", fg="#8A99AD", anchor="w").pack(fill="x")

        # 2-Minute PID Benchmark Card
        bench_card = ttk.Frame(left_col, style="Card.TFrame", padding=10)
        bench_card.pack(fill="x", pady=4)

        ttk.Label(bench_card, text="🧪 2-MINUTE AUTOMATED PID BENCHMARK & AUTO-TUNER", style="CardTitle.TLabel").pack(anchor="w")

        bench_ctrl_row = ttk.Frame(bench_card)
        bench_ctrl_row.pack(fill="x", pady=4)

        self.btn_benchmark = tk.Button(bench_ctrl_row, text="▶️ Start 2-Min PID Benchmark", font=("Segoe UI", 9, "bold"),
                                       bg="#00E676", fg="#000", activebackground="#69F0AE", relief="flat", padx=10, pady=4,
                                       command=self._toggle_benchmark)
        self.btn_benchmark.pack(side="left", padx=4)

        self.btn_show_results = tk.Button(bench_ctrl_row, text="📊 View Leaderboard", font=("Segoe UI", 9, "bold"),
                                          bg="#7C4DFF", fg="#FFF", relief="flat", padx=8, pady=4,
                                          state="disabled", command=self._show_results_modal)
        self.btn_show_results.pack(side="left", padx=4)

        self.bench_progress = ttk.Progressbar(bench_card, variable=self.benchmark_progress_var, maximum=100.0)
        self.bench_progress.pack(fill="x", pady=4)

        self.lbl_bench_status = tk.Label(bench_card, textvariable=self.benchmark_status_var, font=("Segoe UI", 9),
                                         bg="#1A2230", fg="#8A99AD", anchor="w")
        self.lbl_bench_status.pack(fill="x")

        # RIGHT COLUMN: Motor Controls, Oscilloscope Chart, & Interactive Serial Console
        right_col = ttk.Frame(main_paned)
        main_paned.add(right_col, minsize=520)

        # Control Action Buttons Grid
        btn_frame = ttk.Frame(right_col, style="Card.TFrame", padding=10)
        btn_frame.pack(fill="x", pady=4)

        ttk.Label(btn_frame, text="⚡ MOTOR CONTROL & BENCH ACTIONS", style="CardTitle.TLabel").grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 6))

        self._create_btn(btn_frame, "🔬 128-PT LUT (w)", "w", "#E040FB", 1, 0)
        self._create_btn(btn_frame, "⚡ 11-Pole Align (c)", "c", "#7C4DFF", 1, 1)
        self._create_btn(btn_frame, "🟢 Enable (e)", "e", "#00E676", 1, 2)
        self._create_btn(btn_frame, "🛑 STOP / Dis (x)", "x", "#D50000", 1, 3)

        self._create_btn(btn_frame, "🏎️ RPM +10 (+)", "+", "#00E676", 2, 0)
        self._create_btn(btn_frame, "🏎️ RPM -10 (-)", "-", "#FF5252", 2, 1)
        self._create_btn(btn_frame, "🎯 Zero / Tare (z)", "z", "#FFD600", 2, 2)
        self._create_btn(btn_frame, "🔬 Full Calib (w)", "w", "#00B0FF", 2, 3)

        self._create_btn(btn_frame, "🎲 Re-Roll Zero (c)", "c", "#FF6D00", 3, 0)
        self._create_btn(btn_frame, "⚡ Trim Lag (n1000)", "n1000", "#FFD600", 3, 1)
        self._create_btn(btn_frame, "🏎️ Test 1000 RPM", "t1000", "#00E676", 3, 2)
        self._create_btn(btn_frame, "📋 Dump LUT (u)", "u", "#00E5FF", 3, 3)

        self._create_btn(btn_frame, "💾 Save Flash (NVS)", "save", "#00E5FF", 4, 0)
        self._create_btn(btn_frame, "🔄 Load Flash (NVS)", "load", "#2979FF", 4, 1)
        self._create_btn(btn_frame, "💾 Save JSON File", "SAVE_JSON", "#7C4DFF", 4, 2)
        self._create_btn(btn_frame, "📂 Load JSON File", "LOAD_JSON", "#E040FB", 4, 3)

        # Oscilloscope Chart Canvas
        chart_card = ttk.Frame(right_col, style="Card.TFrame", padding=10)
        chart_card.pack(fill="x", pady=4)

        ttk.Label(chart_card, text="REAL-TIME VELOCITY TRACKING (Yellow: Target | Green: Actual)", style="CardTitle.TLabel").pack(anchor="w")
        self.chart_canvas = tk.Canvas(chart_card, bg="#10151E", height=120, highlightthickness=0)
        self.chart_canvas.pack(fill="x", expand=True, pady=4)

        # Raw Serial Console Log with Interactive Input Bar
        log_card = ttk.Frame(right_col, style="Card.TFrame", padding=10)
        log_card.pack(fill="both", expand=True, pady=4)

        log_hdr = ttk.Frame(log_card)
        log_hdr.pack(fill="x", pady=(0, 4))
        ttk.Label(log_hdr, text="ESP32-S3 SERIAL CONSOLE OUTPUT & DIAGNOSTICS", style="CardTitle.TLabel").pack(side="left")

        self.btn_autoscroll = tk.Button(log_hdr, text="Pause Scroll", font=("Segoe UI", 7, "bold"), bg="#37474F", fg="#FFF",
                                        relief="flat", padx=5, pady=0, command=self._toggle_autoscroll)
        self.btn_autoscroll.pack(side="right", padx=2)

        tk.Button(log_hdr, text="Clear", font=("Segoe UI", 7), bg="#263238", fg="#B0BEC5",
                  relief="flat", padx=5, pady=0, command=self._clear_log).pack(side="right", padx=2)

        self.log_text = tk.Text(log_card, bg="#0E131B", fg="#B0BEC5", font=("Consolas", 8), height=10, relief="flat", wrap="none")
        self.log_text.pack(fill="both", expand=True, pady=2)

        cmd_bar = ttk.Frame(log_card)
        cmd_bar.pack(fill="x", pady=(4, 0))

        ttk.Label(cmd_bar, text="CMD:", font=("Consolas", 9, "bold"), foreground="#00E5FF", background="#1A2230").pack(side="left", padx=2)
        self.cmd_entry = tk.Entry(cmd_bar, font=("Consolas", 9), bg="#222D3E", fg="#FFFFFF", insertbackground="#FFF")
        self.cmd_entry.pack(side="left", fill="x", expand=True, padx=4)
        self.cmd_entry.bind("<Return>", lambda e: self._send_custom_cmd())

        tk.Button(cmd_bar, text="Send", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000",
                  relief="flat", padx=8, pady=1, command=self._send_custom_cmd).pack(side="right", padx=2)

    # -------------------------------------------------------------------------
    # TAB 2: ⚡ High-Precision Closed-Loop KV Analyzer Lab
    # -------------------------------------------------------------------------
    def _build_kv_analyzer_tab(self, parent):
        kv_paned = tk.PanedWindow(parent, orient="horizontal", bg="#12161E", bd=0, sashwidth=4)
        kv_paned.pack(fill="both", expand=True, padx=0, pady=4)

        # LEFT KV COLUMN: Real-Time Scatter & Linear Regression Graph + Analytical Cards
        left_kv = ttk.Frame(kv_paned)
        kv_paned.add(left_kv, minsize=640)

        plot_card = ttk.Frame(left_kv, style="Card.TFrame", padding=10)
        plot_card.pack(fill="both", expand=True, pady=4)

        plot_hdr = ttk.Frame(plot_card)
        plot_hdr.pack(fill="x", pady=(0, 4))
        ttk.Label(plot_hdr, text="📈 VOLTAGE (Uq) vs VELOCITY (RPM) LINEAR REGRESSION", style="CardTitle.TLabel").pack(side="left")

        self.lbl_kv_r2_tag = tk.Label(plot_hdr, text="R² Fit: --", font=("Segoe UI", 9, "bold"), bg="#1A2230", fg="#69F0AE")
        self.lbl_kv_r2_tag.pack(side="right", padx=4)

        self.kv_canvas = tk.Canvas(plot_card, bg="#0E131B", height=320, highlightthickness=0)
        self.kv_canvas.pack(fill="both", expand=True, pady=4)

        # Analytical Results Metric Cards Grid
        metrics_grid = ttk.Frame(left_kv)
        metrics_grid.pack(fill="x", pady=4)

        self.card_kv_true = self._create_card(metrics_grid, "⭐ TRUE MOTOR KV", "-- RPM/V", "#00E5FF", 0, 0)
        self.card_kv_r2 = self._create_card(metrics_grid, "📈 LINEARITY (R²)", "--", "#69F0AE", 0, 1)
        self.card_kv_v0 = self._create_card(metrics_grid, "🚪 FRICTION OFFSET (V0)", "-- V", "#FFD600", 0, 2)
        self.card_kv_ke = self._create_card(metrics_grid, "⚡ BACK-EMF (Ke)", "-- V·s/rad", "#B388FF", 0, 3)

        self.card_kv_kt = self._create_card(metrics_grid, "🦾 TORQUE CONST (Kt)", "-- N·m/A", "#FF80AB", 1, 0)
        self.card_kv_sym = self._create_card(metrics_grid, "⚖️ CW/CCW SYMMETRY", "-- %", "#64FFDA", 1, 1)
        self.card_kv_max_rpm = self._create_card(metrics_grid, "🚀 MAX 12V SPEED", "-- RPM", "#FFAB40", 1, 2)
        self.card_kv_code = self._create_card(metrics_grid, "📋 SIMPLEFOC SNIPPET", "motor.KV_rating = --;", "#FFFFFF", 1, 3)

        # RIGHT KV COLUMN: Test Controller, Data Points Table, & Action Buttons
        right_kv = ttk.Frame(kv_paned)
        kv_paned.add(right_kv, minsize=460)

        # KV Test Controller Card
        ctrl_card = ttk.Frame(right_kv, style="Card.TFrame", padding=10)
        ctrl_card.pack(fill="x", pady=4)

        ttk.Label(ctrl_card, text="🎮 CLOSED-LOOP KV TEST CONTROLLER", style="CardTitle.TLabel").pack(anchor="w", pady=(0, 6))

        btn_row1 = ttk.Frame(ctrl_card)
        btn_row1.pack(fill="x", pady=3)

        self.btn_run_kv = tk.Button(btn_row1, text="🚀 START 18-Stage KV Sweep (v)", font=("Segoe UI", 9, "bold"),
                                    bg="#00E676", fg="#000", activebackground="#69F0AE", relief="flat", padx=10, pady=5,
                                    command=self._start_firmware_kv_test)
        self.btn_run_kv.pack(side="left", fill="x", expand=True, padx=2)

        self.btn_stop_kv = tk.Button(btn_row1, text="⏹️ Abort (x)", font=("Segoe UI", 9, "bold"),
                                     bg="#D50000", fg="#FFF", relief="flat", padx=8, pady=5,
                                     command=self._abort_kv_test)
        self.btn_stop_kv.pack(side="left", padx=2)

        cfg_frame = ttk.Frame(ctrl_card)
        cfg_frame.pack(fill="x", pady=4)

        ttk.Label(cfg_frame, text="Voltage Range: ±1.0V to ±11.5V (18 Points) | Settle: 1.2s | Samples: 0.6s @ 200Hz",
                  font=("Segoe UI", 8), foreground="#8A99AD", background="#1A2230").pack(anchor="w")

        self.kv_prog_bar = ttk.Progressbar(ctrl_card, variable=self.kv_progress_var, maximum=100.0)
        self.kv_prog_bar.pack(fill="x", pady=(6, 2))

        self.lbl_kv_status = tk.Label(ctrl_card, textvariable=self.kv_status_var, font=("Segoe UI", 9),
                                      bg="#1A2230", fg="#8A99AD", anchor="w")
        self.lbl_kv_status.pack(fill="x")

        # Measured Data Points Table Card
        table_card = ttk.Frame(right_kv, style="Card.TFrame", padding=10)
        table_card.pack(fill="both", expand=True, pady=4)

        table_hdr = ttk.Frame(table_card)
        table_hdr.pack(fill="x", pady=(0, 4))
        ttk.Label(table_hdr, text="📊 MEASURED STAGE DATA & RESIDUALS", style="CardTitle.TLabel").pack(side="left")

        tk.Button(table_hdr, text="💾 Export CSV / Report", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000",
                  relief="flat", padx=6, pady=1, command=self._export_kv_report).pack(side="right", padx=2)

        tk.Button(table_hdr, text="⚡ Apply to Motor", font=("Segoe UI", 8, "bold"), bg="#B388FF", fg="#000",
                  relief="flat", padx=6, pady=1, command=self._apply_kv_to_motor).pack(side="right", padx=2)

        tk.Button(table_hdr, text="📋 Copy Snippet", font=("Segoe UI", 8), bg="#263238", fg="#B0BEC5",
                  relief="flat", padx=5, pady=1, command=self._copy_simplefoc_snippet).pack(side="right", padx=2)

        cols = ("Stage", "Voltage", "RPM", "Fit RPM", "Residual", "Ripple", "StdDev")
        self.kv_tree = ttk.Treeview(table_card, columns=cols, show="headings", height=12)

        widths = [45, 65, 75, 75, 65, 65, 65]
        for c, w in zip(cols, widths):
            self.kv_tree.heading(c, text=c)
            self.kv_tree.column(c, width=w, anchor="center")

        self.kv_tree.pack(fill="both", expand=True, pady=2)

    def _build_symmetry_tab(self, parent):
        paned = tk.PanedWindow(parent, orient="horizontal", bg="#12161E", bd=0, sashwidth=4)
        paned.pack(fill="both", expand=True, padx=0, pady=4)

        # LEFT COLUMN: Phase Angle Nudging, Alignment Volt & Auto Equalizer Controls
        left_col = ttk.Frame(paned)
        paned.add(left_col, minsize=520)

        # 1. Info / Strategy Header Card
        info_card = ttk.Frame(left_col, style="Card.TFrame", padding=10)
        info_card.pack(fill="x", pady=4)
        ttk.Label(info_card, text="⚖️ HOW TO EQUALIZE KV SYMMETRY (FIND TRUE NEUTRAL ZERO)", style="CardTitle.TLabel").pack(anchor="w")
        desc = (
            "1. Command fixed ±Voltage in closed loop (e.g. ±4.0V or ±5.0V).\n"
            "2. Nudge electrical zero angle (±0.002 to ±0.05 rad) until Forward & Reverse RPM are equal.\n"
            "3. Once balanced, True Motor KV settles cleanly at the symmetric geometric mean (~138–142 RPM/V)."
        )
        tk.Label(info_card, text=desc, font=("Segoe UI", 9), bg="#1A2230", fg="#8A99AD", justify="left").pack(anchor="w", pady=(2, 0))

        # 2. Calibration Voltage Card (Lower current alignment)
        cal_card = ttk.Frame(left_col, style="Card.TFrame", padding=10)
        cal_card.pack(fill="x", pady=4)
        ttk.Label(cal_card, text="🔬 CALIBRATION ALIGNMENT VOLTAGE & MULTI-POLE ZERO", style="CardTitle.TLabel").pack(anchor="w")
        cal_row = ttk.Frame(cal_card)
        cal_row.pack(fill="x", pady=(4, 0))

        ttk.Label(cal_row, text="Align Volt (V):", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#00E5FF").pack(side="left", padx=2)
        self.entry_align_v = tk.Entry(cal_row, width=6, font=("Segoe UI", 9), bg="#222D3E", fg="#00E5FF", insertbackground="#FFF")
        self.entry_align_v.insert(0, "3.5")
        self.entry_align_v.pack(side="left", padx=4)
        tk.Button(cal_row, text="Set Volt", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000", relief="flat",
                  command=lambda: self._set_param("y", self.entry_align_v.get())).pack(side="left", padx=2)

        tk.Label(cal_row, text="(GM3506 draws ~0.50A @ 3.5V vs 0.8A @ 4.5V)", font=("Segoe UI", 8), bg="#1A2230", fg="#8A99AD").pack(side="left", padx=6)
        tk.Button(cal_row, text="🔬 Run Alignment (c)", font=("Segoe UI", 8, "bold"), bg="#FFD600", fg="#000", relief="flat",
                  command=lambda: self.reader.send_cmd("c")).pack(side="right", padx=2)

        # 3. Live Electrical Zero Angle & Direct Nudge Controls
        nudge_card = ttk.Frame(left_col, style="Card.TFrame", padding=10)
        nudge_card.pack(fill="x", pady=4)
        ttk.Label(nudge_card, text="🎯 ELECTRICAL ZERO ANGLE TUNING & NUDGE (j command)", style="CardTitle.TLabel").pack(anchor="w")

        angle_display_row = ttk.Frame(nudge_card)
        angle_display_row.pack(fill="x", pady=(4, 6))

        ttk.Label(angle_display_row, text="Zero Angle (rad):", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#B388FF").pack(side="left", padx=2)
        self.entry_zero_angle = tk.Entry(angle_display_row, width=10, font=("Segoe UI", 9, "bold"), bg="#222D3E", fg="#B388FF", insertbackground="#FFF")
        self.entry_zero_angle.insert(0, f"{self.current_zero_angle_rad:.5f}")
        self.entry_zero_angle.pack(side="left", padx=4)

        tk.Button(angle_display_row, text="Apply Angle", font=("Segoe UI", 8, "bold"), bg="#B388FF", fg="#000", relief="flat",
                  command=self._apply_manual_zero_angle).pack(side="left", padx=2)

        self.lbl_angle_deg = tk.Label(angle_display_row, text=f"({self.current_zero_angle_rad*57.29578:.2f}°)", font=("Segoe UI", 9), bg="#1A2230", fg="#8A99AD")
        self.lbl_angle_deg.pack(side="left", padx=6)

        # Nudge Buttons Grid
        nudge_btn_row = ttk.Frame(nudge_card)
        nudge_btn_row.pack(fill="x", pady=2)
        for delta, txt, col in [
            (-0.050, "-0.05 rad (-2.9°)", "#FF5252"),
            (-0.010, "-0.01 rad (-0.6°)", "#FF7043"),
            (-0.002, "-0.002 rad", "#FFA726"),
            (+0.002, "+0.002 rad", "#66BB6A"),
            (+0.010, "+0.01 rad (+0.6°)", "#42A5F5"),
            (+0.050, "+0.05 rad (+2.9°)", "#2979FF")
        ]:
            tk.Button(nudge_btn_row, text=txt, font=("Segoe UI", 8, "bold"), bg=col, fg="#FFF", relief="flat",
                      command=lambda d=delta: self._nudge_zero_angle(d)).pack(side="left", fill="x", expand=True, padx=1)

        # 4. Interactive Test & Automated 1-Click Equalizer Card
        auto_card = ttk.Frame(left_col, style="Card.TFrame", padding=10)
        auto_card.pack(fill="x", pady=4)
        ttk.Label(auto_card, text="⚡ INTERACTIVE VOLTAGE TEST & AUTO-EQUALIZER", style="CardTitle.TLabel").pack(anchor="w")

        vtest_row = ttk.Frame(auto_card)
        vtest_row.pack(fill="x", pady=(4, 4))
        ttk.Label(vtest_row, text="Test Voltage (V):", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#FFD54F").pack(side="left", padx=2)
        self.entry_sym_v = tk.Entry(vtest_row, width=6, font=("Segoe UI", 9), bg="#222D3E", fg="#FFD54F", insertbackground="#FFF")
        self.entry_sym_v.insert(0, "4.0")
        self.entry_sym_v.pack(side="left", padx=4)

        tk.Button(vtest_row, text="▶️ Forward (+V)", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000", relief="flat",
                  command=self._spin_forward_test).pack(side="left", padx=3)
        tk.Button(vtest_row, text="◀️ Reverse (-V)", font=("Segoe UI", 8, "bold"), bg="#FF4081", fg="#FFF", relief="flat",
                  command=self._spin_reverse_test).pack(side="left", padx=3)
        tk.Button(vtest_row, text="⏹️ Stop (0V)", font=("Segoe UI", 8, "bold"), bg="#D50000", fg="#FFF", relief="flat",
                  command=lambda: self.reader.send_cmd("u0.0")).pack(side="left", padx=3)

        # Calibration Options Checkbox
        self.chk_recalib_var = tk.BooleanVar(value=False)
        chk_cal = ttk.Checkbutton(auto_card, text="🔄 Re-run 11-Pole Alignment Sweep before Equalizing",
                                  variable=self.chk_recalib_var)
        chk_cal.pack(fill="x", pady=(4, 2))

        # 1-Click Auto Balancer Button
        auto_btn_row = ttk.Frame(auto_card)
        auto_btn_row.pack(fill="x", pady=(4, 2))
        self.btn_auto_sym = tk.Button(auto_btn_row, text="🚀 1-CLICK AUTO-EQUALIZE SYMMETRY", font=("Segoe UI", 10, "bold"),
                                      bg="#00E676", fg="#000", activebackground="#69F0AE", relief="flat", pady=6,
                                      command=self._toggle_auto_symmetry)
        self.btn_auto_sym.pack(fill="x", expand=True)

        self.sym_pbar = ttk.Progressbar(auto_card, variable=self.sym_progress_var, maximum=100)
        self.sym_pbar.pack(fill="x", pady=(6, 2))
        self.lbl_sym_status = tk.Label(auto_card, textvariable=self.sym_status_var, font=("Segoe UI", 8), bg="#1A2230", fg="#8A99AD", anchor="w")
        self.lbl_sym_status.pack(fill="x")

        # RIGHT COLUMN: Visual Symmetry Balance Meter & Live Results Cards
        right_col = ttk.Frame(paned)
        paned.add(right_col, minsize=540)

        # Metrics Card Grid
        m_grid = ttk.Frame(right_col)
        m_grid.pack(fill="x", pady=4)

        self.card_sym_live_rpm = self._create_card(m_grid, "LIVE SHAFT VELOCITY", "0.0 RPM", "#69F0AE", 0, 0)
        self.card_sym_live_uq  = self._create_card(m_grid, "COMMANDED VOLTAGE (Uq)", "0.00 V", "#00E5FF", 0, 1)
        self.card_sym_fwd      = self._create_card(m_grid, "FORWARD SPEED (+V)", "+0.0 RPM", "#00E5FF", 1, 0)
        self.card_sym_rev      = self._create_card(m_grid, "REVERSE SPEED (-V)", "-0.0 RPM", "#FF4081", 1, 1)
        self.card_sym_delta    = self._create_card(m_grid, "SPEED DELTA (Δ RPM)", "0.0 RPM", "#FFD600", 2, 0)
        self.card_sym_pct      = self._create_card(m_grid, "DIRECTIONAL SYMMETRY", "0.0 %", "#69F0AE", 2, 1)
        self.card_sym_kv       = self._create_card(m_grid, "TRUE EQUALIZED KV", "0.0 RPM/V", "#B388FF", 3, 0)
        self.card_sym_ke       = self._create_card(m_grid, "BACK-EMF CONSTANT (Ke)", "0.0000 V·s/rad", "#00E5FF", 3, 1)

        # Visual Balance Needle Meter Card
        needle_card = ttk.Frame(right_col, style="Card.TFrame", padding=8)
        needle_card.pack(fill="x", pady=2)
        ttk.Label(needle_card, text="⚖️ LIVE ROTATIONAL SYMMETRY BALANCE METER", style="CardTitle.TLabel").pack(anchor="w")

        self.sym_canvas = tk.Canvas(needle_card, bg="#151C28", height=130, highlightthickness=0)
        self.sym_canvas.pack(fill="x", expand=True, pady=2)

        # Stage Report Table Card
        sym_table_card = ttk.Frame(right_col, style="Card.TFrame", padding=8)
        sym_table_card.pack(fill="both", expand=True, pady=2)

        sym_tbl_hdr = ttk.Frame(sym_table_card)
        sym_tbl_hdr.pack(fill="x", pady=(0, 2))
        ttk.Label(sym_tbl_hdr, text="📊 STAGE-BY-STAGE OPTIMIZATION FEEDBACK REPORT", style="CardTitle.TLabel").pack(side="left")
        tk.Button(sym_tbl_hdr, text="📋 Copy Report", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000", relief="flat",
                  command=self._copy_symmetry_report).pack(side="right", padx=2)

        sym_cols = ("Stage", "Angle_rad", "Angle_deg", "Nudge", "Fwd_RPM", "Rev_RPM", "Delta_RPM", "Symmetry", "KV_rating", "Status")
        self.sym_tree = ttk.Treeview(sym_table_card, columns=sym_cols, show="headings", height=7)
        sym_widths = [85, 75, 75, 75, 75, 75, 75, 70, 75, 95]
        for c, w in zip(sym_cols, sym_widths):
            self.sym_tree.heading(c, text=c)
            self.sym_tree.column(c, width=w, anchor="center")
        self.sym_tree.pack(fill="both", expand=True, pady=2)

    def _copy_symmetry_report(self):
        items = self.sym_tree.get_children()
        if not items:
            messagebox.showinfo("Report Empty", "No optimization data to copy yet. Run the auto-equalizer or manual test first.")
            return

        lines = []
        lines.append("=== HIERARCHICAL MULTI-RESOLUTION BRUTE-FORCE OPTIMIZATION REPORT ===")
        lines.append(f"Motor: GM3506 (11 Pole Pairs) | Test Voltage: {self.entry_sym_v.get()}V")
        lines.append(f"Baseline Zero: {self.entry_zero_angle.get()} rad")
        lines.append("-------------------------------------------------------------------------------------------------------------------------")
        lines.append(f"{'Stage':<12} | {'Angle (rad)':<12} | {'Angle (deg)':<12} | {'Nudge (rad)':<12} | {'Fwd RPM':<10} | {'Rev RPM':<10} | {'Δ RPM':<10} | {'Symmetry':<10} | {'KV (RPM/V)':<10} | {'Status':<16}")
        lines.append("-------------+--------------+--------------+--------------+------------+------------+------------+------------+------------+-----------------")

        for item in items:
            v = self.sym_tree.item(item, "values")
            lines.append(f"{v[0]:<12} | {v[1]:<12} | {v[2]:<12} | {v[3]:<12} | {v[4]:<10} | {v[5]:<10} | {v[6]:<10} | {v[7]:<10} | {v[8]:<10} | {v[9]:<16}")

        lines.append("-------------------------------------------------------------------------------------------------------------------------")
        lines.append(f"Final Neutral Zero Angle : {self.current_zero_angle_rad:.5f} rad ({self.current_zero_angle_rad * 57.29578:.2f}°)")
        lines.append(f"Final Equalized Motor KV : {self.entry_kv.get()} RPM/V")
        lines.append(f"Status                   : {self.sym_status_var.get()}")
        lines.append("=========================================================================================================================\n")

        full_text = "\n".join(lines)
        self.root.clipboard_clear()
        self.root.clipboard_append(full_text)
        messagebox.showinfo("Report Copied", "Optimization feedback report successfully copied to clipboard!")

    def _nudge_zero_angle(self, delta_rad):
        self.current_zero_angle_rad = (self.current_zero_angle_rad + delta_rad) % (2.0 * math.pi)
        self.entry_zero_angle.delete(0, "end")
        self.entry_zero_angle.insert(0, f"{self.current_zero_angle_rad:.5f}")
        self.lbl_angle_deg.config(text=f"({self.current_zero_angle_rad*57.29578:.2f}°)")
        self.reader.send_cmd(f"j{self.current_zero_angle_rad:.5f}")

    def _apply_manual_zero_angle(self):
        try:
            val = float(self.entry_zero_angle.get().strip())
            self.current_zero_angle_rad = val % (2.0 * math.pi)
            self.entry_zero_angle.delete(0, "end")
            self.entry_zero_angle.insert(0, f"{self.current_zero_angle_rad:.5f}")
            self.lbl_angle_deg.config(text=f"({self.current_zero_angle_rad*57.29578:.2f}°)")
            self.reader.send_cmd(f"j{self.current_zero_angle_rad:.5f}")
        except ValueError:
            messagebox.showwarning("Input Error", "Invalid numeric angle in radians.")

    def _spin_forward_test(self):
        try:
            v = abs(float(self.entry_sym_v.get().strip()))
            self.reader.send_cmd(f"u{v:.2f}")
        except ValueError:
            pass

    def _spin_reverse_test(self):
        try:
            v = abs(float(self.entry_sym_v.get().strip()))
            self.reader.send_cmd(f"u-{v:.2f}")
        except ValueError:
            pass

    def _toggle_auto_symmetry(self):
        if self.sym_running:
            self.sym_running = False
            self.btn_auto_sym.config(text="🚀 1-CLICK AUTO-EQUALIZE SYMMETRY", bg="#00E676", fg="#000")
            self.sym_status_var.set("Auto Equalizer Aborted by User.")
            self.reader.send_cmd("u0.0")
        else:
            if not self.reader.connected:
                messagebox.showerror("Connection Error", "Please connect to ESP32-S3 first!")
                return
            self.sym_running = True
            self.btn_auto_sym.config(text="⏹️ Stop Auto Equalizer", bg="#D50000", fg="#FFF")
            self.sym_thread = threading.Thread(target=self._run_auto_symmetry_worker, daemon=True)
            self.sym_thread.start()

    def _measure_candidate_angle(self, candidate_angle, stage_name, v_test, base_zero):
        if not self.sym_running:
            return None

        # 1. Update Zero Angle on ESP32-S3
        self.reader.send_cmd(f"j{candidate_angle:.5f}")
        self.current_zero_angle_rad = candidate_angle
        self.root.after(0, lambda a=candidate_angle: [
            self.entry_zero_angle.delete(0, "end"),
            self.entry_zero_angle.insert(0, f"{a:.5f}"),
            self.lbl_angle_deg.config(text=f"({a*57.29578:.2f}°)")
        ])
        time.sleep(0.05)

        # 2. Forward Spin (+V): Smooth settle with 0-RPM duration stall detection (400ms)
        self.reader.send_cmd(f"v{v_test:.2f}")
        t_spin = time.time()
        stall_time_s = 0.0
        last_t = time.time()
        while time.time() - t_spin < 1.1:
            if not self.sym_running:
                self.reader.send_cmd("v0.0")
                self.reader.send_cmd("x")
                return None
            
            now = time.time()
            dt = now - last_t
            last_t = now

            with self.reader.lock:
                live_spd = abs(self.reader.last_actual_rpm)
            
            if (now - t_spin) > 0.15:
                if live_spd < 3.0:
                    stall_time_s += dt
                    if stall_time_s >= 0.40: # Locked at 0 RPM for >= 400ms
                        self.reader.send_cmd("v0.0")
                        self.reader.send_cmd("x")
                        self._record_sym_row(stage_name, candidate_angle, base_zero, 0.0, 0.0, "[STALL - 0 RPM]")
                        return {"angle": candidate_angle, "fwd": 0, "rev": 0, "delta": 0, "sym": 0, "kv": 0, "score": 0, "status": "[STALL]", "is_valid": False}
                else:
                    stall_time_s = 0.0
            time.sleep(0.02)

        fwd_samples = []
        for _ in range(15):
            if not self.sym_running:
                self.reader.send_cmd("v0.0")
                return None
            with self.reader.lock:
                fwd_samples.append(abs(self.reader.last_actual_rpm))
            time.sleep(0.03)
        fwd_rpm = sum(fwd_samples) / max(1, len(fwd_samples))
        self.sym_fwd_rpm = fwd_rpm

        # Soft stop
        self.reader.send_cmd("v0.0")
        t_stop = time.time()
        while time.time() - t_stop < 0.40:
            if not self.sym_running: return None
            time.sleep(0.02)

        if not self.sym_running: return None

        # 3. Reverse Spin (-V): Smooth settle with 0-RPM duration stall detection (400ms)
        self.reader.send_cmd(f"v-{v_test:.2f}")
        t_rev = time.time()
        stall_time_s = 0.0
        last_t = time.time()
        while time.time() - t_rev < 1.1:
            if not self.sym_running:
                self.reader.send_cmd("v0.0")
                self.reader.send_cmd("x")
                return None
            
            now = time.time()
            dt = now - last_t
            last_t = now

            with self.reader.lock:
                live_spd = abs(self.reader.last_actual_rpm)
            
            if (now - t_rev) > 0.15:
                if live_spd < 3.0:
                    stall_time_s += dt
                    if stall_time_s >= 0.40: # Locked at 0 RPM for >= 400ms
                        self.reader.send_cmd("v0.0")
                        self.reader.send_cmd("x")
                        self._record_sym_row(stage_name, candidate_angle, base_zero, fwd_rpm, 0.0, "[STALL - 0 RPM]")
                        return {"angle": candidate_angle, "fwd": fwd_rpm, "rev": 0, "delta": 0, "sym": 0, "kv": 0, "score": 0, "status": "[STALL]", "is_valid": False}
                else:
                    stall_time_s = 0.0
            time.sleep(0.02)

        rev_samples = []
        for _ in range(15):
            if not self.sym_running:
                self.reader.send_cmd("v0.0")
                return None
            with self.reader.lock:
                rev_samples.append(abs(self.reader.last_actual_rpm))
            time.sleep(0.03)
        rev_rpm = sum(rev_samples) / max(1, len(rev_samples))
        self.sym_rev_rpm = rev_rpm

        # Soft stop
        self.reader.send_cmd("v0.0")
        t_stop2 = time.time()
        while time.time() - t_stop2 < 0.40:
            if not self.sym_running: return None
            time.sleep(0.02)

        # 4. Evaluation & Score
        delta_rpm = fwd_rpm - rev_rpm
        is_stalled = (fwd_rpm < 5.0 or rev_rpm < 5.0)
        mean_rpm = (fwd_rpm + rev_rpm) * 0.5
        stage_kv = mean_rpm / max(0.1, v_test)
        max_rpm = max(fwd_rpm, rev_rpm, 1.0)
        sym_pct = (min(fwd_rpm, rev_rpm) / max_rpm) * 100.0 if not is_stalled else 0.0

        if is_stalled:
            score = 0.0
            status = "[STALL - 0 RPM]"
            is_valid = False
        else:
            score = sym_pct
            status = f"OK ({sym_pct:.1f}%)"
            is_valid = True

        self._record_sym_row(stage_name, candidate_angle, base_zero, fwd_rpm, rev_rpm, status)

        return {
            "angle": candidate_angle,
            "fwd": fwd_rpm,
            "rev": rev_rpm,
            "delta": delta_rpm,
            "sym": sym_pct,
            "kv": stage_kv,
            "score": score,
            "status": status,
            "is_valid": is_valid
        }

    def _record_sym_row(self, stage_name, candidate_angle, base_zero, fwd_rpm, rev_rpm, status):
        nudge_val = candidate_angle - base_zero
        delta_rpm = fwd_rpm - rev_rpm
        max_rpm = max(fwd_rpm, rev_rpm, 1.0)
        sym_pct = (min(fwd_rpm, rev_rpm) / max_rpm) * 100.0 if (fwd_rpm > 5.0 and rev_rpm > 5.0) else 0.0
        stage_kv = ((fwd_rpm + rev_rpm) * 0.5) / 4.0

        row_data = (
            stage_name,
            f"{candidate_angle:.5f}",
            f"{candidate_angle*57.29578:.2f}°",
            f"{nudge_val:+.5f} rad",
            f"{fwd_rpm:+.1f} RPM",
            f"-{rev_rpm:.1f} RPM",
            f"{delta_rpm:+.1f} RPM",
            f"{sym_pct:.1f} %",
            f"{stage_kv:.1f} RPM/V",
            status
        )
        self.root.after(0, lambda r=row_data: [
            self.sym_tree.insert("", "end", values=r),
            self.sym_tree.see(self.sym_tree.get_children()[-1])
        ])

    def _run_auto_symmetry_worker(self):
        try:
            v_test = abs(float(self.entry_sym_v.get().strip()))
        except Exception:
            v_test = 4.0

        # Clear report table in UI
        self.root.after(0, lambda: [
            self.sym_tree.delete(item) for item in self.sym_tree.get_children()
        ])

        # Step 1: Baseline Zero
        try:
            base_zero = float(self.entry_zero_angle.get().strip())
        except ValueError:
            with self.reader.lock:
                base_zero = self.reader.last_zero_electric_angle or self.current_zero_angle_rad

        self.current_zero_angle_rad = base_zero
        all_results = []

        # =============================================================
        # PASS 1: MEDIUM GRID SWEEP (7 Candidate Points across ±0.0030 rad)
        # =============================================================
        self.sym_status_var.set("Pass 1/2: Sweeping 7 Candidate Angles across Baseline ±0.0030 rad...")
        med_offsets = [0.0000, +0.0010, -0.0010, +0.0020, -0.0020, +0.0030, -0.0030]
        pass1_results = []

        for i, off in enumerate(med_offsets, 1):
            if not self.sym_running: break
            self.sym_progress_var.set(5.0 + (i / len(med_offsets)) * 45.0)
            ang = base_zero + off
            label = f"Medium #{i} ({off:+.4f})"
            res = self._measure_candidate_angle(ang, label, v_test, base_zero)
            if res:
                pass1_results.append(res)
                all_results.append(res)

        valid_pass1 = [r for r in pass1_results if r["is_valid"]]
        if not valid_pass1:
            self.sym_status_var.set("❌ Motor stalled or no valid movements recorded during Pass 1.")
            self._cleanup_sym_ui()
            return

        # Best from Pass 1 (highest symmetry score)
        best_medium = max(valid_pass1, key=lambda r: r["score"])
        best_angle = best_medium["angle"]

        # =============================================================
        # PASS 2: FINE GRID SWEEP (5 Candidate Points around Winner ±0.0006 rad)
        # =============================================================
        pass2_results = [best_medium]
        if self.sym_running:
            self.sym_status_var.set(f"Pass 2/2: Fine-Tuning 5 Micro-Angles around Winner ({best_angle*57.3:.2f}°)...")
            fine_offsets = [+0.0003, -0.0003, +0.0006, -0.0006]

            for i, off in enumerate(fine_offsets, 1):
                if not self.sym_running: break
                self.sym_progress_var.set(50.0 + (i / len(fine_offsets)) * 45.0)
                ang = best_angle + off
                label = f"Fine #{i} ({off:+.4f})"
                res = self._measure_candidate_angle(ang, label, v_test, base_zero)
                if res:
                    pass2_results.append(res)
                    all_results.append(res)

        valid_pass2 = [r for r in pass2_results if r["is_valid"]]
        winner = max(valid_pass2, key=lambda r: r["score"]) if valid_pass2 else best_medium

        if not self.sym_running:
            self.sym_status_var.set("Auto Equalizer Aborted by User.")
            self._cleanup_sym_ui()
            self.reader.send_cmd("v0.0")
            return

        # =============================================================
        # FINAL APPLY & RESULTS
        # =============================================================
        final_ang = winner["angle"]
        final_kv = max(100.0, min(180.0, winner.get("kv", 130.0)))
        final_sym = winner.get("sym", 95.0)

        self.current_zero_angle_rad = final_ang
        self.reader.send_cmd(f"j{final_ang:.5f}")
        self.reader.send_cmd("v0.0")

        self.sym_progress_var.set(100.0)
        self.sym_status_var.set(
            f"🎉 Optimization Complete! Neutral Zero = {final_ang:.5f} rad ({final_ang*57.29578:.2f}°) | Sym = {final_sym:.1f}% | Δ = {winner['delta']:+.1f} RPM"
        )
        self._cleanup_sym_ui()

        # Update UI Entries
        self.root.after(0, lambda k=final_kv, a=final_ang: [
            self.entry_kv.delete(0, "end"),
            self.entry_kv.insert(0, f"{k:.1f}"),
            self.entry_zero_angle.delete(0, "end"),
            self.entry_zero_angle.insert(0, f"{a:.5f}"),
            self.lbl_angle_deg.config(text=f"({a*57.29578:.2f}°)")
        ])

    def _cleanup_sym_ui(self):
        self.sym_running = False
        self.btn_auto_sym.config(text="🚀 1-CLICK AUTO-EQUALIZE SYMMETRY", bg="#00E676", fg="#000")

    def _create_card(self, parent, title, init_val, color, row, col):
        card = ttk.Frame(parent, style="Card.TFrame", padding="8 6 8 6")
        card.grid(row=row, column=col, sticky="nsew", padx=3, pady=3)
        parent.columnconfigure(col, weight=1)

        ttk.Label(card, text=title, style="CardTitle.TLabel").pack(anchor="w")
        val_lbl = tk.Label(card, text=init_val, font=("Segoe UI", 13, "bold"), fg=color, bg="#1A2230")
        val_lbl.pack(anchor="w", pady=(2, 0))
        return val_lbl

    def _create_btn(self, parent, text, cmd, color, row, col):
        fg_col = "#000000" if color in ["#00E676", "#00E5FF", "#FFD600", "#FF9100", "#64FFDA"] else "#FFFFFF"
        b = tk.Button(parent, text=text, font=("Segoe UI", 8, "bold"), bg=color, fg=fg_col,
                      activebackground="#ECEFF1", relief="flat", padx=4, pady=4,
                      command=lambda: self._handle_btn(cmd))
        b.grid(row=row, column=col, sticky="nsew", padx=2, pady=2)
        parent.columnconfigure(col, weight=1)
        return b

    def _toggle_autoscroll(self):
        self.autoscroll_enabled = not self.autoscroll_enabled
        self.btn_autoscroll.config(
            text="Resume Scroll" if not self.autoscroll_enabled else "Pause Scroll",
            bg="#FF9100" if not self.autoscroll_enabled else "#37474F"
        )

    def _clear_log(self):
        self.log_text.delete("1.0", "end")

    def _send_custom_cmd(self):
        cmd = self.cmd_entry.get().strip()
        if cmd:
            self.reader.send_cmd(cmd)
            self.cmd_entry.delete(0, "end")

    def _reset_gm3506_defaults(self):
        self.entry_ramp.delete(0, "end"); self.entry_ramp.insert(0, "200.0")
        self.entry_v.delete(0, "end"); self.entry_v.insert(0, "12.0")
        self.entry_max_rpm.delete(0, "end"); self.entry_max_rpm.insert(0, "1432.0")
        self.entry_p.delete(0, "end"); self.entry_p.insert(0, "0.12")
        self.entry_i.delete(0, "end"); self.entry_i.insert(0, "1.20")
        self.entry_d.delete(0, "end"); self.entry_d.insert(0, "0.000")
        self.entry_f.delete(0, "end"); self.entry_f.insert(0, "0.005")
        self.entry_lag.delete(0, "end"); self.entry_lag.insert(0, "35.0")
        self.entry_kv.delete(0, "end"); self.entry_kv.insert(0, "171.9")
        self.entry_target.delete(0, "end"); self.entry_target.insert(0, "0.0")
        self.reader.send_cmd("r")

    def _apply_kv_to_motor(self):
        with self.reader.lock:
            res = dict(self.reader.kv_results)
        kv = res.get('KV', 171.9)
        self.entry_kv.delete(0, "end")
        self.entry_kv.insert(0, f"{kv:.1f}")
        self._set_param("q", f"{kv:.1f}")
        messagebox.showinfo("KV Applied", f"Motor KV rating set to {kv:.1f} RPM/V on ESP32-S3 controller and updated in Live Tuning Panel.")

    def _set_param(self, prefix, val_str):
        try:
            val = float(val_str.strip())
            cmd = f"{prefix}{val}"
            self.reader.send_cmd(cmd)
        except ValueError:
            messagebox.showwarning("Input Error", f"Invalid numeric value: '{val_str}'")

    def _handle_btn(self, cmd):
        if cmd == "FLASH":
            self._flash_firmware()
        elif cmd == "SAVE_JSON":
            self._export_config_dialog()
        elif cmd == "LOAD_JSON":
            self._upload_config_dialog()
        else:
            self.reader.send_cmd(cmd)

    def _export_config_dialog(self):
        filename = filedialog.asksaveasfilename(
            title="Save Calibration Profile JSON",
            defaultextension=".json",
            filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")],
            initialfile="motor_config_gm3506.json"
        )
        if not filename:
            return
        
        cfg = {
            "zero_electric_angle": self.current_zero_angle_rad,
            "sensor_direction": 1,
            "sensor_lag_comp_us": 35.0,
            "motor_kv_rating": float(self.entry_kv.get().strip() or "130.0"),
            "pid_p": float(self.entry_p.get().strip() or "0.12"),
            "pid_i": float(self.entry_i.get().strip() or "1.20"),
            "pid_d": float(self.entry_d.get().strip() or "0.00"),
            "lpf_tf": float(self.entry_f.get().strip() or "0.005"),
            "date": time.strftime("%Y-%m-%d %H:%M:%S")
        }
        try:
            import json
            with open(filename, "w", encoding="utf-8") as f:
                json.dump(cfg, f, indent=2)
            messagebox.showinfo("Config Saved", f"Profile successfully saved to:\n{filename}")
        except Exception as e:
            messagebox.showerror("Save Error", f"Failed to save file:\n{str(e)}")

    def _upload_config_dialog(self):
        filename = filedialog.askopenfilename(
            title="Load Calibration Profile JSON",
            filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")]
        )
        if not filename:
            return
        try:
            import json
            with open(filename, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            
            if "zero_electric_angle" in cfg:
                ang = float(cfg["zero_electric_angle"])
                self.current_zero_angle_rad = ang
                self.entry_zero_angle.delete(0, "end")
                self.entry_zero_angle.insert(0, f"{ang:.5f}")
                self.lbl_angle_deg.config(text=f"({ang*57.29578:.2f}°)")
                self.reader.send_cmd(f"j{ang:.5f}")
                time.sleep(0.05)
            
            if "pid_p" in cfg:
                self.entry_p.delete(0, "end"); self.entry_p.insert(0, str(cfg["pid_p"]))
                self.reader.send_cmd(f"p{cfg['pid_p']}")
                time.sleep(0.05)
            if "pid_i" in cfg:
                self.entry_i.delete(0, "end"); self.entry_i.insert(0, str(cfg["pid_i"]))
                self.reader.send_cmd(f"i{cfg['pid_i']}")
                time.sleep(0.05)
            if "pid_d" in cfg:
                self.entry_d.delete(0, "end"); self.entry_d.insert(0, str(cfg["pid_d"]))
                self.reader.send_cmd(f"d{cfg['pid_d']}")
                time.sleep(0.05)
            if "lpf_tf" in cfg:
                self.entry_f.delete(0, "end"); self.entry_f.insert(0, str(cfg["lpf_tf"]))
                self.reader.send_cmd(f"f{cfg['lpf_tf']}")
                time.sleep(0.05)
            if "sensor_lag_comp_us" in cfg:
                self.reader.send_cmd(f"s{cfg['sensor_lag_comp_us']}")
                time.sleep(0.05)
            if "motor_kv_rating" in cfg:
                self.entry_kv.delete(0, "end"); self.entry_kv.insert(0, str(cfg["motor_kv_rating"]))
                self.reader.send_cmd(f"q{cfg['motor_kv_rating']}")
            
            messagebox.showinfo("Config Uploaded", f"Profile loaded and applied to motor controller:\n{filename}")
        except Exception as e:
            messagebox.showerror("Upload Error", f"Failed to load profile:\n{str(e)}")

    def _flash_firmware(self):
        ans = messagebox.askyesno("Flash ESP32-S3", "Upload firmware to ESP32-S3 (ESP32-S3_Motor_24_08)?\n(ESP32-S3 should be connected in DFU / Boot mode)")
        if not ans:
            return
        self.reader.disconnect()
        self.conn_label.config(text="FLASHING...", fg="#FFD600")
        self.root.update()

        def do_flash():
            pio_exe = find_pio_executable()
            workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
            cmd = [pio_exe, "run", "-e", "ESP32-S3_Motor_24_08", "-t", "upload"]
            try:
                res = subprocess.run(cmd, capture_output=True, text=True, cwd=workspace_root)
                if res.returncode == 0:
                    self.root.after(0, lambda: messagebox.showinfo("Flash Success", "ESP32-S3 firmware (ESP32-S3_Motor_24_08) flashed successfully!"))
                else:
                    err_msg = res.stderr.strip() or res.stdout.strip()
                    self.root.after(0, lambda: messagebox.showerror("Flash Error", f"Upload failed:\n{err_msg[-500:]}"))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("Flash Error", f"Execution error: {str(e)}"))
            self.root.after(0, lambda: self._toggle_connect(force_connect=True))

        threading.Thread(target=do_flash, daemon=True).start()

    def _toggle_connect(self, force_connect=False):
        if self.reader.running and not force_connect:
            self.reader.disconnect()
            self.conn_label.config(text="DISCONNECTED", fg="#FF5252")
            self.btn_connect.config(text="Connect", bg="#00E5FF")
        else:
            self.reader.port = self.port_entry.get().strip()
            try:
                self.reader.connect()
                self.conn_label.config(text=f"ONLINE ({self.reader.port}) @ {self.reader.baud}", fg="#69F0AE")
                self.btn_connect.config(text="Disconnect", bg="#FF5252")
            except Exception as e:
                messagebox.showerror("Connection Error", f"Failed to open {self.reader.port}:\n{str(e)}")

    def _start_firmware_kv_test(self):
        if not self.reader.connected:
            messagebox.showerror("Connection Error", "Please connect to ESP32-S3 first!")
            return

        with self.reader.lock:
            self.reader.kv_points = []
            self.reader.kv_results = {}
            self.reader.kv_test_active = True

        for item in self.kv_tree.get_children():
            self.kv_tree.delete(item)

        self.btn_run_kv.config(text="🔄 Running KV Sweep...", bg="#FF9100", fg="#000")
        self.kv_progress_var.set(5.0)
        self.kv_status_var.set("Initiating FOC Alignment & Voltage Sweep...")
        self.reader.send_cmd("v")

    def _abort_kv_test(self):
        self.reader.send_cmd("x")
        self.btn_run_kv.config(text="🚀 START 18-Stage KV Sweep (v)", bg="#00E676", fg="#000")
        self.kv_status_var.set("KV Test Aborted by User.")
        with self.reader.lock:
            self.reader.kv_test_active = False

    def _export_kv_report(self):
        with self.reader.lock:
            pts = list(self.reader.kv_points)
            res = dict(self.reader.kv_results)

        if not pts and not res:
            messagebox.showinfo("No Data", "Please run the KV test first before exporting!")
            return

        file_path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV File", "*.csv"), ("Text Report", "*.txt"), ("All Files", "*.*")],
            title="Export Closed-Loop KV Test Report"
        )
        if not file_path:
            return

        try:
            with open(file_path, "w", encoding="utf-8") as f:
                f.write("# DA WINNA — CLOSED-LOOP MOTOR KV TEST REPORT\n")
                f.write(f"# Date: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# Motor: GM3506 (11 Pole Pairs) @ 12V SpaceVectorPWM\n")
                f.write(f"# Sensor: MT6816/MT6818 14-Bit SPI Magnetic Encoder\n")
                f.write("# ====================================================================\n")
                if res:
                    f.write(f"# True Motor KV: {res.get('KV', 0):.2f} RPM/V\n")
                    f.write(f"# Forward KV (CW): {res.get('FwdKV', 0):.2f} RPM/V\n")
                    f.write(f"# Reverse KV (CCW): {res.get('RevKV', 0):.2f} RPM/V\n")
                    f.write(f"# Linearity (R^2): {res.get('R2', 0):.5f}\n")
                    f.write(f"# Friction Voltage (V0): {res.get('V0', 0):.3f} V\n")
                    f.write(f"# Back-EMF Constant (Ke): {res.get('Ke', 0):.5f} V*s/rad\n")
                    f.write(f"# Torque Constant (Kt): {res.get('Kt', 0):.5f} N*m/A\n")
                    f.write(f"# Max No-Load Speed @ 12V: {res.get('MaxRPM', 0):.1f} RPM\n")
                    f.write(f"# CW/CCW Symmetry: {res.get('Symmetry', 0):.1f} %\n")
                    f.write("# ====================================================================\n\n")

                f.write("Stage,Voltage_V,Measured_RPM,Speed_Ripple_RPM,StdDev_RPM\n")
                for p in pts:
                    f.write(f"{p['stage']},{p['voltage']:.2f},{p['rpm']:.2f},{p['ripple']:.2f},{p['std_dev']:.3f}\n")

            messagebox.showinfo("Export Success", f"KV test report successfully saved to:\n{file_path}")
        except Exception as e:
            messagebox.showerror("Export Error", f"Failed to save report:\n{str(e)}")

    def _copy_simplefoc_snippet(self):
        with self.reader.lock:
            res = dict(self.reader.kv_results)
        kv = res.get('KV', 130.0)
        snippet = f"motor.KV_rating = {kv:.1f}f; // GM3506 Measured Closed-Loop KV"
        self.root.clipboard_clear()
        self.root.clipboard_append(snippet)
        messagebox.showinfo("Copied", f"Copied SimpleFOC configuration to clipboard:\n\n{snippet}")

    def _toggle_benchmark(self):
        if self.benchmark_running:
            self.benchmark_running = False
            self.btn_benchmark.config(text="▶️ Start 2-Min PID Benchmark", bg="#00E676", fg="#000")
            self.benchmark_status_var.set("Benchmark Aborted by User.")
            self.reader.send_cmd("x")
        else:
            if not self.reader.connected:
                messagebox.showerror("Connection Error", "Please connect to ESP32-S3 first!")
                return
            self.benchmark_running = True
            self.btn_benchmark.config(text="⏹️ Stop Benchmark", bg="#D50000", fg="#FFF")
            self.btn_show_results.config(state="disabled")
            self.benchmark_results = []
            self.benchmark_thread = threading.Thread(target=self._run_benchmark_worker, daemon=True)
            self.benchmark_thread.start()

    def _run_benchmark_worker(self):
        presets = [
            {"name": "GM3506 Baseline Smooth", "p": 0.08, "i": 0.80, "d": 0.000, "tf": 0.035, "target": 80.0},
            {"name": "GM3506 Optimal Balanced","p": 0.12, "i": 1.20, "d": 0.001, "tf": 0.015, "target": 80.0},
            {"name": "Crisp Direct Tracking", "p": 0.15, "i": 1.50, "d": 0.001, "tf": 0.020, "target": 80.0},
            {"name": "High Integral Authority","p": 0.10, "i": 2.00, "d": 0.001, "tf": 0.025, "target": 80.0},
            {"name": "Ultra-Fast LPF Response", "p": 0.12, "i": 1.20, "d": 0.000, "tf": 0.010, "target": 80.0},
            {"name": "Heavy Damped Smooth",   "p": 0.10, "i": 1.00, "d": 0.002, "tf": 0.030, "target": 80.0},
            {"name": "High Speed 150 RPM",    "p": 0.12, "i": 1.20, "d": 0.001, "tf": 0.015, "target": 150.0},
            {"name": "High Speed 250 RPM",    "p": 0.14, "i": 1.50, "d": 0.001, "tf": 0.015, "target": 250.0},
            {"name": "Aggressive High-P",     "p": 0.18, "i": 1.80, "d": 0.001, "tf": 0.015, "target": 80.0},
            {"name": "Gentle Low-Current",    "p": 0.06, "i": 0.60, "d": 0.000, "tf": 0.025, "target": 80.0},
            {"name": "Precise D-Damped",      "p": 0.12, "i": 1.00, "d": 0.003, "tf": 0.020, "target": 80.0},
            {"name": "Maximum Velocity 400",  "p": 0.15, "i": 1.50, "d": 0.001, "tf": 0.015, "target": 400.0},
        ]

        total_stages = len(presets)
        stage_duration = 10.0
        settle_time = 2.5

        self.root.after(0, lambda: self.benchmark_status_var.set("Ensuring SimpleFOC Calibration..."))
        self.reader.send_cmd("c")
        time.sleep(1.5)

        start_bench_time = time.time()

        for idx, p in enumerate(presets):
            if not self.benchmark_running:
                break

            self.reader.send_cmd(f"p{p['p']}")
            time.sleep(0.05)
            self.reader.send_cmd(f"i{p['i']}")
            time.sleep(0.05)
            self.reader.send_cmd(f"d{p['d']}")
            time.sleep(0.05)
            self.reader.send_cmd(f"f{p['tf']}")
            time.sleep(0.05)
            self.reader.send_cmd(f"t{p['target']}")

            def _update_ui_entries(preset=p):
                self.entry_p.delete(0, "end"); self.entry_p.insert(0, str(preset['p']))
                self.entry_i.delete(0, "end"); self.entry_i.insert(0, str(preset['i']))
                self.entry_d.delete(0, "end"); self.entry_d.insert(0, str(preset['d']))
                self.entry_f.delete(0, "end"); self.entry_f.insert(0, str(preset['tf']))
                self.entry_target.delete(0, "end"); self.entry_target.insert(0, str(preset['target']))
            self.root.after(0, _update_ui_entries)

            stage_start = time.time()
            samples = []

            while time.time() - stage_start < stage_duration:
                if not self.benchmark_running:
                    break

                elapsed_in_stage = time.time() - stage_start
                total_elapsed = time.time() - start_bench_time
                total_progress = min(100.0, (total_elapsed / (total_stages * stage_duration)) * 100.0)

                remaining_sec = max(0, int((total_stages * stage_duration) - total_elapsed))
                def _update_status(i=idx, pr=p, prog=total_progress, rem=remaining_sec):
                    self.benchmark_progress_var.set(prog)
                    self.benchmark_status_var.set(
                        f"Testing [{i+1}/{total_stages}] {pr['name']} (P={pr['p']}, I={pr['i']}, Tf={pr['tf']}) | "
                        f"Target: {pr['target']} RPM | {rem}s remaining"
                    )
                self.root.after(0, _update_status)

                if elapsed_in_stage >= settle_time:
                    with self.reader.lock:
                        act = self.reader.last_actual_rpm
                        uq = self.reader.last_uq
                    samples.append((act, uq))

                time.sleep(0.05)

            if samples:
                speeds = [s[0] for s in samples]
                uqs = [s[1] for s in samples]
                target = p['target']

                mean_speed = sum(speeds) / len(speeds)
                errors = [abs(s - target) for s in speeds]
                mae = sum(errors) / len(errors)

                variance = sum((s - mean_speed) ** 2 for s in speeds) / len(speeds)
                std_dev = math.sqrt(variance)

                ripple_pk_pk = max(speeds) - min(speeds)
                mean_uq = sum(uqs) / len(uqs)

                score = std_dev + (0.5 * mae)

                result_entry = {
                    "rank": 0,
                    "name": p['name'],
                    "p": p['p'],
                    "i": p['i'],
                    "d": p['d'],
                    "tf": p['tf'],
                    "target": target,
                    "mean_speed": mean_speed,
                    "mae": mae,
                    "std_dev": std_dev,
                    "ripple_pk_pk": ripple_pk_pk,
                    "mean_uq": mean_uq,
                    "score": score
                }
                self.benchmark_results.append(result_entry)

        self.benchmark_running = False
        self.reader.send_cmd("t0")

        def _finish():
            self.btn_benchmark.config(text="▶️ Start 2-Min PID Benchmark", bg="#00E676", fg="#000")
            if self.benchmark_results:
                self.benchmark_results.sort(key=lambda x: x['score'])
                for rank_idx, r in enumerate(self.benchmark_results):
                    r['rank'] = rank_idx + 1

                best = self.benchmark_results[0]
                self.benchmark_progress_var.set(100.0)
                self.benchmark_status_var.set(
                    f"✅ Benchmark Complete! Best Preset: #{best['rank']} '{best['name']}' "
                    f"(P={best['p']}, I={best['i']}, Tf={best['tf']} | Jitter: ±{best['std_dev']:.2f} RPM)"
                )
                self.btn_show_results.config(state="normal")
                self.root.after(200, self._show_results_modal)
        self.root.after(0, _finish)

    def _show_results_modal(self):
        if not self.benchmark_results:
            messagebox.showinfo("No Results", "Please run the 2-minute benchmark first!")
            return

        modal = tk.Toplevel(self.root)
        modal.title("🏆 PID Benchmark & Speed Variation Leaderboard")
        modal.geometry("920x530")
        modal.configure(bg="#12161E")
        modal.transient(self.root)

        header = tk.Frame(modal, bg="#1A2230", padx=15, pady=12)
        header.pack(fill="x")

        best = self.benchmark_results[0]
        tk.Label(header, text="🏆 PID TUNING LEADERBOARD & VARIATION ANALYSIS", font=("Segoe UI", 14, "bold"), fg="#00E5FF", bg="#1A2230").pack(anchor="w")
        tk.Label(header, text=f"Top Performer: #{best['rank']} {best['name']} | P={best['p']}, I={best['i']}, D={best['d']}, Tf={best['tf']} | Jitter: ±{best['std_dev']:.2f} RPM | Ripple: {best['ripple_pk_pk']:.1f} RPM",
                 font=("Segoe UI", 10, "bold"), fg="#69F0AE", bg="#1A2230").pack(anchor="w", pady=(4, 0))

        table_frame = tk.Frame(modal, bg="#12161E", padx=10, pady=10)
        table_frame.pack(fill="both", expand=True)

        cols = ("Rank", "Preset Name", "P Gain", "I Gain", "D Gain", "LPF Tf", "Target", "Actual", "MAE Err", "StdDev (Jitter)", "Ripple Pk-Pk")
        tree = ttk.Treeview(table_frame, columns=cols, show="headings", height=12)

        col_widths = [50, 160, 60, 60, 60, 60, 65, 65, 75, 100, 90]
        for c, w in zip(cols, col_widths):
            tree.heading(c, text=c)
            tree.column(c, width=w, anchor="center")

        for r in self.benchmark_results:
            values = (
                f"#{r['rank']}",
                r['name'],
                f"{r['p']:.2f}",
                f"{r['i']:.2f}",
                f"{r['d']:.3f}",
                f"{r['tf']:.3f}",
                f"{r['target']:.1f}",
                f"{r['mean_speed']:.1f}",
                f"±{r['mae']:.2f}",
                f"±{r['std_dev']:.2f} RPM",
                f"{r['ripple_pk_pk']:.1f} RPM"
            )
            tree.insert("", "end", values=values)

        tree.pack(fill="both", expand=True)

        btn_bar = tk.Frame(modal, bg="#12161E", padx=15, pady=10)
        btn_bar.pack(fill="x")

        def apply_selected():
            sel = tree.selection()
            if not sel:
                target_res = self.benchmark_results[0]
            else:
                idx = tree.index(sel[0])
                target_res = self.benchmark_results[idx]

            self.entry_p.delete(0, "end"); self.entry_p.insert(0, str(target_res['p']))
            self.entry_i.delete(0, "end"); self.entry_i.insert(0, str(target_res['i']))
            self.entry_d.delete(0, "end"); self.entry_d.insert(0, str(target_res['d']))
            self.entry_f.delete(0, "end"); self.entry_f.insert(0, str(target_res['tf']))

            self._set_param("p", str(target_res['p']))
            self._set_param("i", str(target_res['i']))
            self._set_param("d", str(target_res['d']))
            self._set_param("f", str(target_res['tf']))

            messagebox.showinfo("Preset Applied", f"Applied Preset #{target_res['rank']} '{target_res['name']}' to active motor controller!")
            modal.destroy()

        tk.Button(btn_bar, text="🚀 Apply Selected / #1 Best Preset", font=("Segoe UI", 10, "bold"),
                  bg="#00E676", fg="#000", padx=12, pady=6, relief="flat", command=apply_selected).pack(side="left", padx=5)

        tk.Button(btn_bar, text="Close", font=("Segoe UI", 9), bg="#455A64", fg="#FFF", padx=10, pady=6, relief="flat",
                  command=modal.destroy).pack(side="right", padx=5)

    def _schedule_update(self):
        try:
            self._update_telemetry()
        except Exception:
            pass
        self.root.after(30, self._schedule_update)

    def _update_telemetry(self):
        lines_to_add = []
        while not self.reader.raw_log_queue.empty():
            try:
                line = self.reader.raw_log_queue.get_nowait()
                lines_to_add.append(line)
            except Exception:
                break

        if lines_to_add:
            text_block = "\n".join(lines_to_add) + "\n"
            self.log_text.insert("end", text_block)
            line_count = int(self.log_text.index("end-1c").split(".")[0])
            if line_count > 300:
                self.log_text.delete("1.0", f"{line_count - 250}.0")
            if self.autoscroll_enabled:
                self.log_text.see("end")

        with self.reader.lock:
            deg = self.reader.last_angle_deg
            rad = self.reader.last_angle_rad
            raw = self.reader.last_raw_14bit
            tgt = self.reader.last_target_rpm
            act = self.reader.last_actual_rpm
            mode = self.reader.last_control_mode
            bus = self.reader.last_bus_mode
            mag_ok = self.reader.last_mag_ok
            parity_ok = self.reader.last_parity_ok
            stall = self.reader.last_stall_alarm
            kv_active = self.reader.kv_test_active
            kv_pts = list(self.reader.kv_points)
            kv_res = dict(self.reader.kv_results)

        self.card_angle.config(text=f"{deg:.2f}°  ({rad:.3f} rad)")
        self.card_target_rpm.config(text=f"{tgt:+.1f} RPM")
        self.card_actual_rpm.config(text=f"{act:+.1f} RPM")
        self.card_raw.config(text=f"{raw} / 16383  ({(raw / 16384.0)*100:.1f}%)")

        if "KV-TEST" in mode or kv_active:
            self.lbl_mode.config(text="CTRL: KV SWEEP TEST", bg="#FF6D00", fg="#FFF")
        elif "CLOSE" in mode or "FOC" in mode:
            self.lbl_mode.config(text="CTRL: CLOSED-LOOP FOC", bg="#7C4DFF", fg="#FFF")
        elif "OPEN" in mode:
            self.lbl_mode.config(text="CTRL: OPEN-LOOP", bg="#00E5FF", fg="#000")
        elif "VOLT" in mode:
            self.lbl_mode.config(text="CTRL: CLOSED-LOOP VOLT", bg="#FFAB00", fg="#000")
        elif "ENABLE" in mode:
            self.lbl_mode.config(text="CTRL: ENABLED", bg="#00E676", fg="#000")
        else:
            self.lbl_mode.config(text="CTRL: DISABLED / STOP", bg="#455A64", fg="#FFF")

        self.lbl_bus.config(text=f"BUS: {bus.upper()}", bg="#0288D1" if bus == "4-Wire" else "#E65100", fg="#FFF")

        with self.reader.lock:
            lut_is_active = self.reader.last_lut_active
        if lut_is_active:
            self.lbl_lut.config(text="LUT: 128-PT [ACTIVE]", bg="#7C4DFF", fg="#FFFFFF")
        else:
            self.lbl_lut.config(text="LUT: RAW", bg="#455A64", fg="#B0BEC5")

        if mag_ok:
            self.lbl_mag.config(text="MAGNET: OK", bg="#69F0AE", fg="#1A2230")
        else:
            self.lbl_mag.config(text="MAGNET: FAULT", bg="#FF5252", fg="#FFFFFF")

        if parity_ok:
            self.lbl_parity.config(text="PARITY: PASS", bg="#69F0AE", fg="#1A2230")
        else:
            self.lbl_parity.config(text="PARITY: NOISE", bg="#FF5252", fg="#FFFFFF")

        if stall:
            self.lbl_stall.config(text="🚨 STALL SHUTDOWN!", bg="#D50000", fg="#FFFFFF")
        else:
            self.lbl_stall.config(text="STALL GUARD: OK", bg="#455A64", fg="#B0BEC5")

        if self.reader.connected:
            self.conn_label.config(text=f"ONLINE ({self.reader.port})", fg="#69F0AE")
            self.btn_connect.config(text="Disconnect", bg="#FF5252")
        else:
            self.conn_label.config(text="RECONNECTING...", fg="#FFD600")

        self._update_kv_tab_data(kv_pts, kv_res, kv_active)
        self._update_symmetry_tab()
        self._draw_gauge(deg)
        self._draw_chart()
        self._draw_kv_scatter_plot(kv_pts, kv_res)
        self._draw_symmetry_needle()

    def _update_kv_tab_data(self, kv_pts, kv_res, kv_active):
        if kv_active:
            self.btn_run_kv.config(text="🔄 Running KV Sweep...", bg="#FF9100", fg="#000")
            if kv_pts:
                last_pt = kv_pts[-1]
                pct = (last_pt['stage'] / last_pt['total_stages']) * 100.0
                self.kv_progress_var.set(pct)
                self.kv_status_var.set(
                    f"Stage [{last_pt['stage']}/{last_pt['total_stages']}]: Commanded Uq = {last_pt['voltage']:+.1f}V | "
                    f"Measured Speed = {last_pt['rpm']:+.1f} RPM | Ripple = ±{last_pt['ripple']*0.5:.1f} RPM"
                )
        else:
            if kv_res:
                self.btn_run_kv.config(text="🚀 START 18-Stage KV Sweep (v)", bg="#00E676", fg="#000")
                self.kv_progress_var.set(100.0)
                self.kv_status_var.set(
                    f"✅ KV Sweep Complete! True Motor KV = {kv_res.get('KV', 0):.1f} RPM/V (R² = {kv_res.get('R2', 0):.5f})"
                )

        if kv_res:
            kv_val = kv_res.get('KV', 0.0)
            r2_val = kv_res.get('R2', 0.0)
            v0_val = kv_res.get('V0', 0.0)
            ke_val = kv_res.get('Ke', 0.0)
            kt_val = kv_res.get('Kt', 0.0)
            sym_val = kv_res.get('Symmetry', 0.0)
            max_rpm = kv_res.get('MaxRPM', 0.0)

            self.card_kv_true.config(text=f"{kv_val:.1f} RPM/V")
            self.card_kv_r2.config(text=f"{r2_val:.5f}")
            self.card_kv_v0.config(text=f"{v0_val:.2f} V")
            self.card_kv_ke.config(text=f"{ke_val:.4f} V·s/rad")
            self.card_kv_kt.config(text=f"{kt_val:.4f} N·m/A")
            self.card_kv_sym.config(text=f"{sym_val:.1f} %")
            self.card_kv_max_rpm.config(text=f"{max_rpm:.1f} RPM")
            self.card_kv_code.config(text=f"motor.KV_rating = {kv_val:.1f}f;")
            self.lbl_kv_r2_tag.config(text=f"R² Fit: {r2_val:.5f} | KV: {kv_val:.1f} RPM/V")

        current_rows = len(self.kv_tree.get_children())
        if len(kv_pts) > current_rows:
            slope = kv_res.get('KV', 130.0) if kv_res else 130.0
            v0 = kv_res.get('V0', 0.0) if kv_res else 0.0

            for i in range(current_rows, len(kv_pts)):
                p = kv_pts[i]
                v = p['voltage']
                rpm = p['rpm']
                if v >= 0:
                    fit_rpm = max(0.0, (v - v0) * slope)
                else:
                    fit_rpm = min(0.0, (v + v0) * slope)
                residual = rpm - fit_rpm

                row_vals = (
                    f"#{p['stage']}",
                    f"{v:+.1f} V",
                    f"{rpm:+.1f} RPM",
                    f"{fit_rpm:+.1f} RPM",
                    f"{residual:+.1f}",
                    f"±{p['ripple']*0.5:.1f}",
                    f"±{p['std_dev']:.2f}"
                )
                self.kv_tree.insert("", "end", values=row_vals)
            children = self.kv_tree.get_children()
            if children:
                self.kv_tree.see(children[-1])

    def _draw_gauge(self, deg):
        try:
            canvas = self.gauge_canvas
            canvas.delete("all")
            w = canvas.winfo_width()
            h = canvas.winfo_height()
            if w < 10 or h < 10:
                return

            cx, cy = w / 2, h / 2
            r = min(cx, cy) - 18

            canvas.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#2A374A", width=3, fill="#121824")
            canvas.create_oval(cx - (r - 10), cy - (r - 10), cx + (r - 10), cy + (r - 10), outline="#1F2A38", width=1)

            for a in range(0, 360, 30):
                rad_a = math.radians(a - 90)
                inner_r = r - 14 if a % 90 == 0 else r - 7
                x1 = cx + (r - 2) * math.cos(rad_a)
                y1 = cy + (r - 2) * math.sin(rad_a)
                x2 = cx + inner_r * math.cos(rad_a)
                y2 = cy + inner_r * math.sin(rad_a)
                color = "#00E5FF" if a % 90 == 0 else "#4A5D78"
                canvas.create_line(x1, y1, x2, y2, fill=color, width=2 if a % 90 == 0 else 1)

                if a % 90 == 0:
                    lx = cx + (r - 24) * math.cos(rad_a)
                    ly = cy + (r - 24) * math.sin(rad_a)
                    canvas.create_text(lx, ly, text=f"{a}°", fill="#8A99AD", font=("Segoe UI", 8, "bold"))

            canvas.create_oval(cx - 8, cy - 8, cx + 8, cy + 8, fill="#00E5FF", outline="#FFFFFF", width=2)

            needle_rad = math.radians(deg - 90)
            nx = cx + (r - 15) * math.cos(needle_rad)
            ny = cy + (r - 15) * math.sin(needle_rad)

            perp_rad = needle_rad + math.pi / 2
            bx1 = cx + 6 * math.cos(perp_rad)
            by1 = cy + 6 * math.sin(perp_rad)
            bx2 = cx - 6 * math.cos(perp_rad)
            by2 = cy - 6 * math.sin(perp_rad)

            canvas.create_polygon(bx1, by1, nx, ny, bx2, by2, fill="#FF5252", outline="#FF1744", width=1)
            canvas.create_line(cx, cy, nx, ny, fill="#FFFFFF", width=2)
            canvas.create_text(cx, cy + (r / 2) + 6, text=f"{deg:.1f}°", fill="#00E5FF", font=("Segoe UI", 14, "bold"))
        except Exception:
            pass

    def _draw_chart(self):
        try:
            with self.reader.lock:
                hist_act = list(self.reader.history_actual_rpm)
                hist_tgt = list(self.reader.history_target_rpm)

            canvas = self.chart_canvas
            canvas.delete("all")
            w = canvas.winfo_width()
            h = canvas.winfo_height()
            if w < 20 or h < 20 or len(hist_act) < 2:
                return

            cy = h / 2
            canvas.create_line(0, cy, w, cy, fill="#2A374A", width=1, dash=(4, 4))
            canvas.create_text(30, cy - 8, text="0 RPM", fill="#546E7A", font=("Segoe UI", 8))

            max_val = max(100.0, max([abs(x) for x in hist_act + hist_tgt] + [100.0]))
            scale_y = (h / 2 - 10) / max_val

            n = len(hist_act)
            dx = w / max(n - 1, 1)

            pts_tgt = []
            for i, val in enumerate(hist_tgt):
                x = i * dx
                y = cy - (val * scale_y)
                pts_tgt.append((x, y))
            for i in range(len(pts_tgt) - 1):
                canvas.create_line(pts_tgt[i][0], pts_tgt[i][1], pts_tgt[i+1][0], pts_tgt[i+1][1], fill="#FFD600", width=2)

            pts_act = []
            for i, val in enumerate(hist_act):
                x = i * dx
                y = cy - (val * scale_y)
                pts_act.append((x, y))
            for i in range(len(pts_act) - 1):
                canvas.create_line(pts_act[i][0], pts_act[i][1], pts_act[i+1][0], pts_act[i+1][1], fill="#69F0AE", width=2)
        except Exception:
            pass

    def _draw_kv_scatter_plot(self, kv_pts, kv_res):
        try:
            canvas = self.kv_canvas
            canvas.delete("all")
            w = canvas.winfo_width()
            h = canvas.winfo_height()
            if w < 40 or h < 40:
                return

            pad_left = 60
            pad_right = 30
            pad_top = 30
            pad_bottom = 40

            pw = w - pad_left - pad_right
            ph = h - pad_top - pad_bottom

            cx = pad_left + pw / 2
            cy = pad_top + ph / 2

            canvas.create_rectangle(pad_left, pad_top, pad_left + pw, pad_top + ph, fill="#121722", outline="#222D3E", width=1)

            v_max = 12.0
            rpm_max = 1600.0

            for rpm_val in [-1500, -1000, -500, 0, 500, 1000, 1500]:
                y = cy - (rpm_val / rpm_max) * (ph / 2)
                color = "#37474F" if rpm_val == 0 else "#1F2837"
                width = 2 if rpm_val == 0 else 1
                dash = () if rpm_val == 0 else (2, 4)
                canvas.create_line(pad_left, y, pad_left + pw, y, fill=color, width=width, dash=dash)
                canvas.create_text(pad_left - 8, y, text=f"{rpm_val:+} RPM", fill="#8A99AD", font=("Segoe UI", 8), anchor="e")

            for v_val in [-12, -8, -4, 0, 4, 8, 12]:
                x = cx + (v_val / v_max) * (pw / 2)
                color = "#37474F" if v_val == 0 else "#1F2837"
                width = 2 if v_val == 0 else 1
                dash = () if v_val == 0 else (2, 4)
                canvas.create_line(x, pad_top, x, pad_top + ph, fill=color, width=width, dash=dash)
                canvas.create_text(x, pad_top + ph + 14, text=f"{v_val:+} V", fill="#8A99AD", font=("Segoe UI", 8), anchor="center")

            canvas.create_text(pad_left + pw / 2, pad_top + ph + 30, text="Commanded Q-Axis Voltage Uq (V)", fill="#00E5FF", font=("Segoe UI", 9, "bold"))
            canvas.create_text(16, pad_top + ph / 2, text="Actual Speed (RPM)", fill="#69F0AE", font=("Segoe UI", 9, "bold"), angle=90)

            if kv_res and kv_res.get('KV', 0) > 1.0:
                kv = kv_res['KV']
                v0 = kv_res.get('V0', 0.0)

                x_start_pos = cx + (v0 / v_max) * (pw / 2)
                y_start_pos = cy
                x_end_pos = cx + (12.0 / v_max) * (pw / 2)
                y_end_pos = cy - ((12.0 - v0) * kv / rpm_max) * (ph / 2)
                canvas.create_line(x_start_pos, y_start_pos, x_end_pos, y_end_pos, fill="#69F0AE", width=2)

                x_start_neg = cx - (v0 / v_max) * (pw / 2)
                y_start_neg = cy
                x_end_neg = cx - (12.0 / v_max) * (pw / 2)
                y_end_neg = cy + ((12.0 - v0) * kv / rpm_max) * (ph / 2)
                canvas.create_line(x_start_neg, y_start_neg, x_end_neg, y_end_neg, fill="#69F0AE", width=2)

                eq_text = f"y = {kv:.1f} · (Uq - {v0:.2f}V)  [R² = {kv_res.get('R2', 0):.5f}]"
                canvas.create_text(pad_left + 15, pad_top + 15, text=eq_text, fill="#69F0AE", font=("Segoe UI", 9, "bold"), anchor="w")

            for pt in kv_pts:
                v = pt['voltage']
                rpm = pt['rpm']
                ripple = pt.get('ripple', 0.0)

                px = cx + (v / v_max) * (pw / 2)
                py = cy - (rpm / rpm_max) * (ph / 2)

                if ripple > 0.5:
                    r_half = (ripple * 0.5 / rpm_max) * (ph / 2)
                    canvas.create_line(px, py - r_half, px, py + r_half, fill="#546E7A", width=1)
                    canvas.create_line(px - 3, py - r_half, px + 3, py - r_half, fill="#546E7A", width=1)
                    canvas.create_line(px - 3, py + r_half, px + 3, py + r_half, fill="#546E7A", width=1)

                pt_col = "#00E5FF" if v >= 0 else "#FF4081"
                canvas.create_oval(px - 4, py - 4, px + 4, py + 4, fill=pt_col, outline="#FFFFFF", width=1)
        except Exception:
            pass

    def _update_symmetry_tab(self):
        with self.reader.lock:
            act_rpm = self.reader.last_actual_rpm
            target_uq = self.reader.last_uq

        self.card_sym_live_rpm.config(text=f"{act_rpm:+.1f} RPM")
        self.card_sym_live_uq.config(text=f"{target_uq:+.2f} V")

        fwd = self.sym_fwd_rpm
        rev = self.sym_rev_rpm
        delta = fwd - rev
        max_v = max(fwd, rev, 1.0)
        sym_pct = (min(fwd, rev) / max_v) * 100.0 if (fwd > 10 and rev > 10) else (100.0 if fwd == 0 and rev == 0 else 0.0)

        try:
            v_test = abs(float(self.entry_sym_v.get().strip()))
        except Exception:
            v_test = 4.0

        mean_rpm = (fwd + rev) * 0.5
        equalized_kv = mean_rpm / max(0.1, v_test) if mean_rpm > 10 else 0.0
        ke = 1.0 / (equalized_kv * (2.0 * math.pi / 60.0)) if equalized_kv > 1.0 else 0.0

        self.card_sym_fwd.config(text=f"{fwd:+.1f} RPM")
        self.card_sym_rev.config(text=f"-{rev:.1f} RPM")
        self.card_sym_delta.config(text=f"{delta:+.1f} RPM")
        self.card_sym_pct.config(text=f"{sym_pct:.1f} %")
        self.card_sym_kv.config(text=f"{equalized_kv:.1f} RPM/V")
        self.card_sym_ke.config(text=f"{ke:.4f} V·s/rad")

    def _draw_symmetry_needle(self):
        try:
            canvas = self.sym_canvas
            canvas.delete("all")
            w = canvas.winfo_width()
            h = canvas.winfo_height()
            if w < 20 or h < 20:
                return

            cx = w / 2
            cy = h - 25
            r = min(w / 2 - 40, h - 45)

            # Arc background
            canvas.create_arc(cx - r, cy - r, cx + r, cy + r, start=30, extent=120, style="arc", outline="#2A374A", width=4)
            # Center target balanced zone (±10 deg arc)
            canvas.create_arc(cx - r, cy - r, cx + r, cy + r, start=82, extent=16, style="arc", outline="#69F0AE", width=6)

            # Center mark
            canvas.create_line(cx, cy - r - 8, cx, cy - r + 8, fill="#69F0AE", width=2)
            canvas.create_text(cx, cy - r - 16, text="PERFECT 1:1 SYMMETRY", fill="#69F0AE", font=("Segoe UI", 8, "bold"))

            # Reverse (left) and Forward (right) markers
            canvas.create_text(cx - r + 15, cy - 10, text="◀ CCW Faster", fill="#FF4081", font=("Segoe UI", 8, "bold"), anchor="w")
            canvas.create_text(cx + r - 15, cy - 10, text="CW Faster ▶", fill="#00E5FF", font=("Segoe UI", 8, "bold"), anchor="e")

            # Balance deflection angle: -45 deg to +45 deg
            fwd = self.sym_fwd_rpm
            rev = self.sym_rev_rpm
            delta = fwd - rev
            denom = max(fwd + rev, 100.0)
            deflection = (delta / denom) * 45.0
            deflection = max(-50.0, min(50.0, deflection))

            needle_angle_rad = math.radians(90.0 - deflection)
            nx = cx + (r - 10) * math.cos(needle_angle_rad)
            ny = cy - (r - 10) * math.sin(needle_angle_rad)

            needle_col = "#69F0AE" if abs(delta) < 15.0 and fwd > 10 and rev > 10 else ("#00E5FF" if delta > 0 else "#FF4081")
            canvas.create_line(cx, cy, nx, ny, fill=needle_col, width=3, arrow="last", arrowshape=(10, 12, 4))
            canvas.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill=needle_col, outline="#FFF", width=2)

            diff_str = f"Speed Difference: {delta:+.1f} RPM"
            canvas.create_text(cx, cy + 12, text=diff_str, fill="#FFFFFF", font=("Segoe UI", 9, "bold"))
        except Exception:
            pass


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="MT6816 SPI Encoder & Motor 1 Test Bench with Closed-Loop KV Analyzer")
    parser.add_argument("--port", type=str, default=None, help="Serial COM port (e.g. COM3)")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate (default: 921600)")
    args = parser.parse_args()

    reader = ESP32SerialReader(port=args.port, baud=args.baud)
    try:
        reader.connect()
    except Exception:
        pass

    root = tk.Tk()
    app = MT6818DashboardGUI(root, reader)

    def on_close():
        reader.disconnect()
        try:
            root.destroy()
        except Exception:
            pass
        os._exit(0)

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
    on_close()
