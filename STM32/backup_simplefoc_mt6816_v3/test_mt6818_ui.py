#!/usr/bin/env python3
"""
DA WINNA — MT6816 / MT6818 SPI Encoder & SimpleFOC Motor 1 Test Bench GUI
Features:
  - High-Speed 921600 Baud Telemetry & Command Engine
  - Real-time 14-Bit Angular Position Visualizer & Compass Dial
  - Live PID Tuning Panel (P, I, D, LPF Tf, Voltage Limit, Target RPM)
  - Dedicated "Reset to GM3506 Defaults (12V)" Action
  - 2-Minute Automated PID Sweep & Speed Variation Benchmark (Auto-Tuner)
  - Ranked Leaderboard Modal with One-Click Winner Preset Application
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
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports

def find_stm32_port():
    for p in serial.tools.list_ports.comports():
        hw = (p.hwid or "").upper()
        desc = (p.description or "").upper()
        if "0483" in hw or "STLINK" in desc or "SERIAL" in desc or "STM" in desc:
            return p.device
    return "COM3"

class STM32SerialReader:
    def __init__(self, port=None, baud=921600):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = False
        self.connected = False
        self.thread = None
        self.raw_log_queue = queue.Queue(maxsize=2000)
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
        self.last_mag_ok = True
        self.last_parity_ok = True
        self.last_stall_alarm = False
        self.last_stall_guard = False
        self.msg_count = 0

        self.history_time = []
        self.history_target_rpm = []
        self.history_actual_rpm = []
        self.history_angle = []
        self.start_time = time.time()

    def connect(self):
        if not self.port:
            self.port = find_stm32_port()
        if not self.port:
            raise RuntimeError("No STM32 USB Serial port found! Ensure STM32 is plugged in.")
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
                    port = self.port or find_stm32_port()
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
        # 1. Stall alarm detection
        if "SAFETY SHUTDOWN: Motor Stall" in line or "Motor stall/overload" in line or "Motor Stall / Overload" in line:
            with self.lock:
                self.last_stall_alarm = True
                self.last_control_mode = "DISABLED"
            return

        # 2. Discrete command acknowledgments / state changes
        if "Driver ENABLED" in line:
            with self.lock:
                self.last_control_mode = "ENABLED"
        elif "Driver DISABLED" in line or "MOTOR STOPPED" in line:
            with self.lock:
                self.last_control_mode = "DISABLED"
        elif "FOC Calibration Complete" in line:
            with self.lock:
                self.last_control_mode = "CLOSED-LOOP FOC"
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

        # 3. Rich Streaming Telemetry Parsing
        # Format: [PWM@PA0]: 45.2 deg (12.5% @ 995Hz) [OK] || Ctrl: CLOSE | Target: +50.0 RPM | Actual: +49.8 RPM | Angle: 120.50 deg | Uq: +2.10V | Ud: +0.05V | Raw: 5000/16383 | Mode: 4-Wire | Mag: [ OK ] | Parity: [PASS]
        if "Ctrl:" in line and "Angle:" in line:
            try:
                # Angle (deg)
                m_ang = re.search(r'Angle:\s*([+-]?\d+\.?\d*)\s*deg', line)
                angle_deg = float(m_ang.group(1)) if m_ang else self.last_angle_deg

                # Target RPM
                m_tgt = re.search(r'Target:\s*([+-]?\d+\.?\d*)\s*RPM', line)
                target_rpm = float(m_tgt.group(1)) if m_tgt else self.last_target_rpm

                # Actual RPM
                m_act = re.search(r'Actual:\s*([+-]?\d+\.?\d*)\s*RPM', line)
                actual_rpm = float(m_act.group(1)) if m_act else self.last_actual_rpm

                # Raw CPR count
                m_raw = re.search(r'Raw:\s*(\d+)/16383', line)
                raw_count = int(m_raw.group(1)) if m_raw else int((angle_deg / 360.0) * 16384)

                # Voltages
                m_uq = re.search(r'Uq:\s*([+-]?\d+\.?\d*)V', line)
                uq = float(m_uq.group(1)) if m_uq else self.last_uq

                m_ud = re.search(r'Ud:\s*([+-]?\d+\.?\d*)V', line)
                ud = float(m_ud.group(1)) if m_ud else self.last_ud

                # Control mode
                ctrl_mode = "CLOSED-LOOP FOC"
                if "Ctrl: DISBL" in line:
                    ctrl_mode = "DISABLED"
                elif "Ctrl: OPEN" in line:
                    ctrl_mode = "OPEN-LOOP"
                elif "Ctrl: CLOSE" in line:
                    ctrl_mode = "CLOSED-LOOP FOC"

                # Bus mode
                bus_mode = "3-Wire" if "Mode: 3-Wire" in line else "4-Wire"

                # Health flags
                mag_ok = "[ OK ]" in line or "[MAGNET OK]" in line
                parity_ok = "[PASS]" in line or "Parity: [ PASS ]" in line

                if "DISBL" in ctrl_mode and self.last_stall_alarm and target_rpm == 0.0:
                    pass
                else:
                    self.last_stall_alarm = False

                t_now = time.time() - self.start_time
                with self.lock:
                    self.last_angle_deg = angle_deg
                    self.last_angle_rad = (angle_deg / 180.0) * math.pi
                    self.last_raw_14bit = raw_count
                    self.last_target_rpm = target_rpm
                    self.last_actual_rpm = actual_rpm
                    self.last_uq = uq
                    self.last_ud = ud
                    self.last_control_mode = ctrl_mode
                    self.last_bus_mode = bus_mode
                    self.last_mag_ok = mag_ok
                    self.last_parity_ok = parity_ok
                    self.msg_count += 1

                    self.history_time.append(t_now)
                    self.history_target_rpm.append(target_rpm)
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
        self.root.title("DA WINNA — MT6816 / MT6818 SimpleFOC Motor 1 Test Bench (GM3506)")
        self.root.geometry("1200x860")
        self.root.minsize(1080, 740)
        self.root.configure(bg="#12161E")

        self.autoscroll_enabled = True

        # Benchmark Runner State
        self.benchmark_running = False
        self.benchmark_thread = None
        self.benchmark_results = []
        self.benchmark_progress_var = tk.DoubleVar(value=0.0)
        self.benchmark_status_var = tk.StringVar(value="Ready for 2-Min PID Benchmark")

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

    def _build_ui(self):
        # 1. Header Bar
        header_frame = ttk.Frame(self.root, padding="15 10 15 8")
        header_frame.pack(fill="x")

        ttk.Label(header_frame, text="🏎️ DA WINNA — MT6816 SPI ENCODER & SIMPLEFOC BENCH (GM3506)", style="Header.TLabel").pack(side="left")

        self.conn_label = tk.Label(header_frame, text="DISCONNECTED", font=("Segoe UI", 10, "bold"), fg="#FF5252", bg="#12161E")
        self.conn_label.pack(side="right", padx=8)

        self.port_entry = tk.Entry(header_frame, width=10, font=("Segoe UI", 10), bg="#222D3E", fg="#FFFFFF", insertbackground="#FFF")
        self.port_entry.insert(0, self.reader.port or "COM3")
        self.port_entry.pack(side="right", padx=4)

        self.btn_connect = tk.Button(header_frame, text="Connect", font=("Segoe UI", 9, "bold"), bg="#00E5FF", fg="#000",
                                     activebackground="#00B4D8", command=self._toggle_connect, relief="flat", padx=8)
        self.btn_connect.pack(side="right", padx=4)

        # 2. Main Content Paned Window
        main_paned = tk.PanedWindow(self.root, orient="horizontal", bg="#12161E", bd=0, sashwidth=4)
        main_paned.pack(fill="both", expand=True, padx=12, pady=4)

        # LEFT COLUMN: Gauges, Readouts & PID Tuning Panel
        left_col = ttk.Frame(main_paned)
        main_paned.add(left_col, minsize=530)

        # Dial Gauge Card
        gauge_card = ttk.Frame(left_col, style="Card.TFrame", padding=8)
        gauge_card.pack(fill="x", pady=4)

        ttk.Label(gauge_card, text="14-BIT ABSOLUTE ANGLE COMPASS & COMMUTATION POSITION", style="CardTitle.TLabel").pack(anchor="w")
        self.gauge_canvas = tk.Canvas(gauge_card, bg="#151C28", height=180, highlightthickness=0)
        self.gauge_canvas.pack(fill="x", expand=True, pady=4)

        # Telemetry Value Grid
        grid_frame = ttk.Frame(left_col)
        grid_frame.pack(fill="x", pady=4)

        self.card_angle = self._create_card(grid_frame, "MEASURED ANGLE", "0.00°", "#00E5FF", 0, 0)
        self.card_target_rpm = self._create_card(grid_frame, "TARGET VELOCITY", "0.0 RPM", "#FFD600", 0, 1)
        self.card_actual_rpm = self._create_card(grid_frame, "ACTUAL VELOCITY", "0.0 RPM", "#69F0AE", 1, 0)
        self.card_raw = self._create_card(grid_frame, "RAW 14-BIT / CPR", "0 / 16383", "#B388FF", 1, 1)

        # Health & Mode Badges Bar
        badge_bar = ttk.Frame(left_col, style="Card.TFrame", padding="8 6 8 6")
        badge_bar.pack(fill="x", pady=4)

        self.lbl_mode = tk.Label(badge_bar, text="CTRL: IDLE", font=("Segoe UI", 9, "bold"), bg="#455A64", fg="#FFF", padx=6, pady=2)
        self.lbl_mode.pack(side="left", padx=3)

        self.lbl_bus = tk.Label(badge_bar, text="BUS: 4-WIRE", font=("Segoe UI", 9, "bold"), bg="#0288D1", fg="#FFF", padx=6, pady=2)
        self.lbl_bus.pack(side="left", padx=3)

        self.lbl_mag = tk.Label(badge_bar, text="MAGNET: OK", font=("Segoe UI", 9, "bold"), bg="#69F0AE", fg="#1A2230", padx=6, pady=2)
        self.lbl_mag.pack(side="left", padx=3)

        self.lbl_parity = tk.Label(badge_bar, text="PARITY: PASS", font=("Segoe UI", 9, "bold"), bg="#69F0AE", fg="#1A2230", padx=6, pady=2)
        self.lbl_parity.pack(side="left", padx=3)

        self.lbl_stall = tk.Label(badge_bar, text="STALL: OK", font=("Segoe UI", 9, "bold"), bg="#455A64", fg="#B0BEC5", padx=6, pady=2)
        self.lbl_stall.pack(side="left", padx=3)

        # -------------------------------------------------------------
        # Live PID & Motion Parameter Tuning Panel (GM3506 Optimized)
        # -------------------------------------------------------------
        pid_card = ttk.Frame(left_col, style="Card.TFrame", padding=10)
        pid_card.pack(fill="x", pady=4)

        hdr_row = ttk.Frame(pid_card)
        hdr_row.grid(row=0, column=0, columnspan=6, sticky="ew", pady=(0, 6))

        ttk.Label(hdr_row, text="⚙️ GM3506 SIMPLEFOC PID & PARAMETERS (12V)", style="CardTitle.TLabel").pack(side="left")
        
        tk.Button(hdr_row, text="🔄 Reset GM3506 Defaults", font=("Segoe UI", 8, "bold"), bg="#FF6D00", fg="#FFF",
                  relief="flat", padx=6, pady=1, command=self._reset_gm3506_defaults).pack(side="right")

        # Row 1: P Gain, I Gain
        ttk.Label(pid_card, text="P Gain:", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#B0BEC5").grid(row=1, column=0, sticky="w", padx=2, pady=2)
        self.entry_p = tk.Entry(pid_card, width=7, font=("Segoe UI", 9), bg="#222D3E", fg="#FFF", insertbackground="#FFF")
        self.entry_p.insert(0, "0.12")
        self.entry_p.grid(row=1, column=1, padx=2, pady=2)
        tk.Button(pid_card, text="Set P", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000", relief="flat", command=lambda: self._set_param("p", self.entry_p.get())).grid(row=1, column=2, padx=2, pady=2)

        ttk.Label(pid_card, text="I Gain:", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#B0BEC5").grid(row=1, column=3, sticky="w", padx=2, pady=2)
        self.entry_i = tk.Entry(pid_card, width=7, font=("Segoe UI", 9), bg="#222D3E", fg="#FFF", insertbackground="#FFF")
        self.entry_i.insert(0, "1.20")
        self.entry_i.grid(row=1, column=4, padx=2, pady=2)
        tk.Button(pid_card, text="Set I", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000", relief="flat", command=lambda: self._set_param("i", self.entry_i.get())).grid(row=1, column=5, padx=2, pady=2)

        # Row 2: D Gain, LPF Tf
        ttk.Label(pid_card, text="D Gain:", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#B0BEC5").grid(row=2, column=0, sticky="w", padx=2, pady=2)
        self.entry_d = tk.Entry(pid_card, width=7, font=("Segoe UI", 9), bg="#222D3E", fg="#FFF", insertbackground="#FFF")
        self.entry_d.insert(0, "0.001")
        self.entry_d.grid(row=2, column=1, padx=2, pady=2)
        tk.Button(pid_card, text="Set D", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000", relief="flat", command=lambda: self._set_param("d", self.entry_d.get())).grid(row=2, column=2, padx=2, pady=2)

        ttk.Label(pid_card, text="LPF Tf (s):", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#B0BEC5").grid(row=2, column=3, sticky="w", padx=2, pady=2)
        self.entry_f = tk.Entry(pid_card, width=7, font=("Segoe UI", 9), bg="#222D3E", fg="#FFF", insertbackground="#FFF")
        self.entry_f.insert(0, "0.015")
        self.entry_f.grid(row=2, column=4, padx=2, pady=2)
        tk.Button(pid_card, text="Set Tf", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000", relief="flat", command=lambda: self._set_param("f", self.entry_f.get())).grid(row=2, column=5, padx=2, pady=2)

        # Row 3: Voltage Limit, Speed Target
        ttk.Label(pid_card, text="Volt Limit:", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#B0BEC5").grid(row=3, column=0, sticky="w", padx=2, pady=2)
        self.entry_v = tk.Entry(pid_card, width=7, font=("Segoe UI", 9), bg="#222D3E", fg="#FFF", insertbackground="#FFF")
        self.entry_v.insert(0, "12.0")
        self.entry_v.grid(row=3, column=1, padx=2, pady=2)
        tk.Button(pid_card, text="Set V", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000", relief="flat", command=lambda: self._set_param("v", self.entry_v.get())).grid(row=3, column=2, padx=2, pady=2)

        ttk.Label(pid_card, text="Target RPM:", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#FFD600").grid(row=3, column=3, sticky="w", padx=2, pady=2)
        self.entry_target = tk.Entry(pid_card, width=7, font=("Segoe UI", 9, "bold"), bg="#222D3E", fg="#FFD600", insertbackground="#FFF")
        self.entry_target.insert(0, "0.0")
        self.entry_target.grid(row=3, column=4, padx=2, pady=2)
        tk.Button(pid_card, text="Set RPM", font=("Segoe UI", 8, "bold"), bg="#FFD600", fg="#000", relief="flat", command=lambda: self._set_param("t", self.entry_target.get())).grid(row=3, column=5, padx=2, pady=2)

        # Speed Presets Bar
        preset_bar = ttk.Frame(pid_card)
        preset_bar.grid(row=4, column=0, columnspan=6, sticky="ew", pady=(6, 0))
        for spd in [0, 50, 100, 200, 400]:
            btn_col = "#D50000" if spd == 0 else "#2979FF"
            tk.Button(preset_bar, text=f"{spd} RPM", font=("Segoe UI", 8, "bold"), bg=btn_col, fg="#FFF", relief="flat",
                      command=lambda s=spd: self._set_param("t", str(s))).pack(side="left", fill="x", expand=True, padx=2)

        # -------------------------------------------------------------
        # 2-Minute PID Sweep Benchmark Card
        # -------------------------------------------------------------
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

        ttk.Label(btn_frame, text="⚡ MOTOR CONTROL & TEST COMMANDS", style="CardTitle.TLabel").grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 6))

        self._create_btn(btn_frame, "⚡ FOC Align & Run (c)", "c", "#7C4DFF", 1, 0)
        self._create_btn(btn_frame, "🔄 Open-Loop Mode (o)", "o", "#00E5FF", 1, 1)
        self._create_btn(btn_frame, "🟢 Enable Driver (e)", "e", "#00E676", 1, 2)
        self._create_btn(btn_frame, "🛑 STOP / Disable (x)", "x", "#D50000", 1, 3)

        self._create_btn(btn_frame, "🏎️ RPM +10 (+)", "+", "#00E676", 2, 0)
        self._create_btn(btn_frame, "🏎️ RPM -10 (-)", "-", "#FF5252", 2, 1)
        self._create_btn(btn_frame, "🎯 Zero / Tare (z)", "z", "#FFD600", 2, 2)
        self._create_btn(btn_frame, "🔌 3-Wire / 4-Wire (w)", "w", "#00B0FF", 2, 3)

        self._create_btn(btn_frame, "🛡️ Stall Guard (k)", "k", "#FF9100", 3, 0)
        self._create_btn(btn_frame, "📡 Stream ON/OFF (g)", "g", "#2979FF", 3, 1)
        self._create_btn(btn_frame, "📖 Help Menu (h)", "h", "#455A64", 3, 2)
        self._create_btn(btn_frame, "⚡ Flash Firmware", "FLASH", "#E91E63", 3, 3)

        # Strip-Chart Canvas (Dual-trace Oscilloscope)
        chart_card = ttk.Frame(right_col, style="Card.TFrame", padding=10)
        chart_card.pack(fill="x", pady=4)

        ttk.Label(chart_card, text="REAL-TIME VELOCITY TRACKING (Yellow: Target | Green: Actual)", style="CardTitle.TLabel").pack(anchor="w")
        self.chart_canvas = tk.Canvas(chart_card, bg="#10151E", height=120, highlightthickness=0)
        self.chart_canvas.pack(fill="x", expand=True, pady=4)

        # Raw Serial Console Log with Interactive Input Bar & Controls
        log_card = ttk.Frame(right_col, style="Card.TFrame", padding=10)
        log_card.pack(fill="both", expand=True, pady=4)

        log_hdr = ttk.Frame(log_card)
        log_hdr.pack(fill="x", pady=(0, 4))
        ttk.Label(log_hdr, text="STM32 SERIAL CONSOLE OUTPUT & DIAGNOSTICS", style="CardTitle.TLabel").pack(side="left")

        self.btn_autoscroll = tk.Button(log_hdr, text="Pause Scroll", font=("Segoe UI", 7, "bold"), bg="#37474F", fg="#FFF",
                                        relief="flat", padx=5, pady=0, command=self._toggle_autoscroll)
        self.btn_autoscroll.pack(side="right", padx=2)

        tk.Button(log_hdr, text="Clear", font=("Segoe UI", 7), bg="#263238", fg="#B0BEC5",
                  relief="flat", padx=5, pady=0, command=self._clear_log).pack(side="right", padx=2)

        self.log_text = tk.Text(log_card, bg="#0E131B", fg="#B0BEC5", font=("Consolas", 8), height=10, relief="flat", wrap="none")
        self.log_text.pack(fill="both", expand=True, pady=2)

        # Interactive Serial Input Bar
        cmd_bar = ttk.Frame(log_card)
        cmd_bar.pack(fill="x", pady=(4, 0))

        ttk.Label(cmd_bar, text="CMD:", font=("Consolas", 9, "bold"), foreground="#00E5FF", background="#1A2230").pack(side="left", padx=2)
        self.cmd_entry = tk.Entry(cmd_bar, font=("Consolas", 9), bg="#222D3E", fg="#FFFFFF", insertbackground="#FFF")
        self.cmd_entry.pack(side="left", fill="x", expand=True, padx=4)
        self.cmd_entry.bind("<Return>", lambda e: self._send_custom_cmd())

        tk.Button(cmd_bar, text="Send", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000",
                  relief="flat", padx=8, pady=1, command=self._send_custom_cmd).pack(side="right", padx=2)

    def _create_card(self, parent, title, init_val, color, row, col):
        card = ttk.Frame(parent, style="Card.TFrame", padding="8 6 8 6")
        card.grid(row=row, column=col, sticky="nsew", padx=3, pady=3)
        parent.columnconfigure(col, weight=1)

        ttk.Label(card, text=title, style="CardTitle.TLabel").pack(anchor="w")
        val_lbl = tk.Label(card, text=init_val, font=("Segoe UI", 15, "bold"), fg=color, bg="#1A2230")
        val_lbl.pack(anchor="w", pady=(2, 0))
        return val_lbl

    def _create_btn(self, parent, text, cmd, color, row, col):
        fg_col = "#000000" if color in ["#00E676", "#00E5FF", "#FFD600", "#FF9100"] else "#FFFFFF"
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
        self.entry_p.delete(0, "end"); self.entry_p.insert(0, "0.12")
        self.entry_i.delete(0, "end"); self.entry_i.insert(0, "1.20")
        self.entry_d.delete(0, "end"); self.entry_d.insert(0, "0.001")
        self.entry_f.delete(0, "end"); self.entry_f.insert(0, "0.025")
        self.entry_v.delete(0, "end"); self.entry_v.insert(0, "12.0")
        self.entry_target.delete(0, "end"); self.entry_target.insert(0, "0.0")
        self.reader.send_cmd("r")

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
        else:
            self.reader.send_cmd(cmd)

    def _flash_firmware(self):
        ans = messagebox.askyesno("Flash STM32", "Upload firmware to STM32?\n(STM32 should be in DFU / Boot mode)")
        if not ans:
            return
        self.reader.disconnect()
        self.conn_label.config(text="FLASHING...", fg="#FFD600")
        self.root.update()

        def do_flash():
            cmd = [os.path.expandvars(r"%USERPROFILE%\.platformio\penv\Scripts\platformio.exe"), "run", "-e", "encoder_test", "-t", "upload"]
            res = subprocess.run(cmd, capture_output=True, text=True, cwd=os.path.dirname(os.path.abspath(__file__)) + "/..")
            if res.returncode == 0:
                self.root.after(0, lambda: messagebox.showinfo("Flash Success", "STM32 firmware flashed successfully!"))
            else:
                self.root.after(0, lambda: messagebox.showerror("Flash Error", f"Upload failed:\n{res.stdout[-400:]}"))
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

    # -------------------------------------------------------------------------
    # 2-Minute Automated PID Benchmark Engine
    # -------------------------------------------------------------------------
    def _toggle_benchmark(self):
        if self.benchmark_running:
            self.benchmark_running = False
            self.btn_benchmark.config(text="▶️ Start 2-Min PID Benchmark", bg="#00E676", fg="#000")
            self.benchmark_status_var.set("Benchmark Aborted by User.")
            self.reader.send_cmd("x")
        else:
            if not self.reader.connected:
                messagebox.showerror("Connection Error", "Please connect to STM32 first!")
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
            {"name": "GM3506 Optimal Balanced","p": 0.12, "i": 1.20, "d": 0.001, "tf": 0.025, "target": 80.0},
            {"name": "Crisp Direct Tracking", "p": 0.15, "i": 1.50, "d": 0.001, "tf": 0.020, "target": 80.0},
            {"name": "High Integral Authority","p": 0.10, "i": 2.00, "d": 0.001, "tf": 0.025, "target": 80.0},
            {"name": "Ultra-Fast LPF Response", "p": 0.12, "i": 1.20, "d": 0.000, "tf": 0.015, "target": 80.0},
            {"name": "Heavy Damped Smooth",   "p": 0.10, "i": 1.00, "d": 0.002, "tf": 0.040, "target": 80.0},
            {"name": "High Speed 150 RPM",    "p": 0.12, "i": 1.20, "d": 0.001, "tf": 0.025, "target": 150.0},
            {"name": "High Speed 250 RPM",    "p": 0.14, "i": 1.50, "d": 0.001, "tf": 0.020, "target": 250.0},
            {"name": "Aggressive High-P",     "p": 0.18, "i": 1.80, "d": 0.001, "tf": 0.020, "target": 80.0},
            {"name": "Gentle Low-Current",    "p": 0.06, "i": 0.60, "d": 0.000, "tf": 0.030, "target": 80.0},
            {"name": "Precise D-Damped",      "p": 0.12, "i": 1.00, "d": 0.003, "tf": 0.025, "target": 80.0},
            {"name": "Maximum Velocity 400",  "p": 0.15, "i": 1.50, "d": 0.001, "tf": 0.020, "target": 400.0},
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

            # 1. Apply Preset Parameters
            self.reader.send_cmd(f"p{p['p']}")
            time.sleep(0.05)
            self.reader.send_cmd(f"i{p['i']}")
            time.sleep(0.05)
            self.reader.send_cmd(f"d{p['d']}")
            time.sleep(0.05)
            self.reader.send_cmd(f"f{p['tf']}")
            time.sleep(0.05)
            self.reader.send_cmd(f"t{p['target']}")

            # Update entries in GUI (thread-safe)
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

        # Finish Benchmark
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
        modal.geometry("900x520")
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
        # 1. Update text logs (batch insert for maximum GUI performance)
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
            
            # Prune old text if beyond 300 lines
            line_count = int(self.log_text.index("end-1c").split(".")[0])
            if line_count > 300:
                self.log_text.delete("1.0", f"{line_count - 250}.0")

            if self.autoscroll_enabled:
                self.log_text.see("end")

        # 2. Snapshot state safely under lock
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

        self.card_angle.config(text=f"{deg:.2f}°  ({rad:.3f} rad)")
        self.card_target_rpm.config(text=f"{tgt:+.1f} RPM")
        self.card_actual_rpm.config(text=f"{act:+.1f} RPM")
        self.card_raw.config(text=f"{raw} / 16383  ({(raw / 16384.0)*100:.1f}%)")

        # Mode Badge
        if "CLOSE" in mode or "FOC" in mode:
            self.lbl_mode.config(text="CTRL: CLOSED-LOOP FOC", bg="#7C4DFF", fg="#FFF")
        elif "OPEN" in mode:
            self.lbl_mode.config(text="CTRL: OPEN-LOOP", bg="#00E5FF", fg="#000")
        elif "ENABLE" in mode:
            self.lbl_mode.config(text="CTRL: ENABLED", bg="#00E676", fg="#000")
        else:
            self.lbl_mode.config(text="CTRL: DISABLED / STOP", bg="#455A64", fg="#FFF")

        # Bus Mode Badge
        self.lbl_bus.config(text=f"BUS: {bus.upper()}", bg="#0288D1" if bus == "4-Wire" else "#E65100", fg="#FFF")

        # Health Badges
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

        # Connection status sync
        if self.reader.connected:
            self.conn_label.config(text=f"ONLINE ({self.reader.port})", fg="#69F0AE")
            self.btn_connect.config(text="Disconnect", bg="#FF5252")
        else:
            self.conn_label.config(text="RECONNECTING...", fg="#FFD600")

        # Draw Gauge & Chart
        self._draw_gauge(deg)
        self._draw_chart()

    def _draw_gauge(self, deg):
        try:
            canvas = self.gauge_canvas
            canvas.delete("all")
            w = canvas.winfo_width()
            h = canvas.winfo_height()
            if w < 10 or h < 10:
                return

            cx, cy = w / 2, h / 2
            r = min(cx, cy) - 20

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
            canvas.create_text(cx, cy + (r / 2) + 6, text=f"{deg:.1f}°", fill="#00E5FF", font=("Segoe UI", 15, "bold"))
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

            # Draw Target RPM Line (Yellow)
            pts_tgt = []
            for i, val in enumerate(hist_tgt):
                x = i * dx
                y = cy - (val * scale_y)
                pts_tgt.append((x, y))
            for i in range(len(pts_tgt) - 1):
                canvas.create_line(pts_tgt[i][0], pts_tgt[i][1], pts_tgt[i+1][0], pts_tgt[i+1][1], fill="#FFD600", width=2)

            # Draw Actual RPM Line (Green)
            pts_act = []
            for i, val in enumerate(hist_act):
                x = i * dx
                y = cy - (val * scale_y)
                pts_act.append((x, y))
            for i in range(len(pts_act) - 1):
                canvas.create_line(pts_act[i][0], pts_act[i][1], pts_act[i+1][0], pts_act[i+1][1], fill="#69F0AE", width=2)
        except Exception:
            pass


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="MT6816 SPI Encoder & Motor 1 Test Bench")
    parser.add_argument("--port", type=str, default=None, help="Serial COM port (e.g. COM3)")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate (default: 921600)")
    args = parser.parse_args()

    reader = STM32SerialReader(port=args.port, baud=args.baud)
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
