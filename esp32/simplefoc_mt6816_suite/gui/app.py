"""
Main Tkinter Dashboard Application Window for MT6816/MT6818 and SimpleFOC test suite.
Composes UI tabs, controls, submodules, telemetry loops, and flash operations.
"""

import json
import math
import os
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

from .utils import find_pio_executable
from .benchmarker import BenchmarkRunner
from .calibrator import Calibrator
from .symmetry_equalizer import SymmetryEqualizer
from .kv_analyzer import KVAnalyzer
from .plots import draw_gauge, draw_chart, draw_kv_scatter_plot, draw_symmetry_needle, draw_harmonic_overlay_plot


class MT6818DashboardGUI:
    def __init__(self, root, reader):
        self.root = root
        self.reader = reader
        self.root.title("DA WINNA — MT6816 / MT6818 SimpleFOC & Closed-Loop KV Test Suite (GM3506 @ 12V)")
        self.root.geometry("1280x920")
        self.root.minsize(1140, 780)
        self.root.configure(bg="#12161E")

        self.autoscroll_enabled = True

        # State Variables
        self.sensor_mode_var = tk.IntVar(value=0)

        self.benchmark_progress_var = tk.DoubleVar(value=0.0)
        self.benchmark_status_var = tk.StringVar(value="Ready for Multi-Speed GA Auto-Tuner (400 RPM Base + 50/400/1000 RPM Gauntlet)")

        self.sym_status_var = tk.StringVar(value="Ready to test KV Symmetry & Phase Angle Balance.")
        self.sym_progress_var = tk.DoubleVar(value=0.0)

        self.kv_progress_var = tk.DoubleVar(value=0.0)
        self.kv_status_var = tk.StringVar(value="Ready for Closed-Loop KV Sweep Test.")

        self.calib_progress_var = tk.DoubleVar(value=0.0)
        self.calib_status_var = tk.StringVar(value="Ready for High-Precision Auto-Calibration.")
        self.calib_grade_var = tk.StringVar(value="NOT CALIBRATED")
        self.calib_align_volt_var = tk.StringVar(value="4.0")
        self.calib_boost_retry_var = tk.BooleanVar(value=True)
        self.calib_verify_var = tk.BooleanVar(value=True)

        # Submodules
        self.benchmarker = BenchmarkRunner(
            self.reader, self.root, self.benchmark_progress_var, self.benchmark_status_var,
            self._set_param, on_benchmark_finish_cb=lambda: self.benchmarker.show_results_modal(self._apply_benchmark_preset)
        )
        self.calibrator = Calibrator(
            self.reader, self.root, self.calib_progress_var, self.calib_status_var,
            self.calib_grade_var, self.calib_align_volt_var, self.calib_boost_retry_var,
            self.calib_verify_var, on_calib_finish_cb=self._on_zero_angle_updated
        )
        self.symmetry_equalizer = SymmetryEqualizer(
            self.reader, self.root, self.sym_progress_var, self.sym_status_var,
            on_zero_angle_updated=self._on_zero_angle_updated
        )
        self.kv_analyzer = KVAnalyzer(
            self.reader, self.root, self.kv_progress_var, self.kv_status_var
        )

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

        # Big Red Global Emergency Stop in Top Bar
        self.btn_top_estop = tk.Button(header_frame, text="🛑 E-STOP / DISABLE (Esc)", font=("Segoe UI", 10, "bold"),
                                       bg="#D50000", fg="#FFFFFF", activebackground="#FF1744", activeforeground="#FFFFFF",
                                       relief="flat", padx=12, pady=2, command=self.emergency_stop)
        self.btn_top_estop.pack(side="right", padx=10)

        # Keyboard shortcuts: Escape or Space (when not typing in an entry) always triggers E-Stop
        self.root.bind_all("<Escape>", lambda e: self.emergency_stop())
        self.root.bind_all("<Key-space>", self._on_space_pressed)

        # 2. Main Notebook Tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=12, pady=(0, 8))

        self.tab_dash = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_dash, text="  🏎️ Live Dashboard & Motion Tuning  ")

        self.tab_kv = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_kv, text="  ⚡ Closed-Loop KV Analyzer  ")

        self.tab_sym = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_sym, text="  ⚖️ Phase Angle & KV Equalizer  ")

        self.tab_harm = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_harm, text="  🔬 Harmonic Calibration & Benchmark  ")

        self._build_dashboard_tab(self.tab_dash)
        self._build_kv_analyzer_tab(self.tab_kv)
        self._build_symmetry_tab(self.tab_sym)
        self._build_harmonic_tab(self.tab_harm)

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

        self.lbl_foc = tk.Label(badge_bar, text="FOC: -- kHz", font=("Segoe UI", 9, "bold"), bg="#1B5E20", fg="#69F0AE", padx=6, pady=2)
        self.lbl_foc.pack(side="left", padx=3)

        self.lbl_lat = tk.Label(badge_bar, text="LAT: -- µs", font=("Segoe UI", 9, "bold"), bg="#263238", fg="#B0BEC5", padx=6, pady=2)
        self.lbl_lat.pack(side="left", padx=3)

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
        self.entry_v.insert(0, "6.93")
        self.entry_v.grid(row=1, column=4, padx=2, pady=2)
        tk.Button(pid_card, text="Set V", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000", relief="flat", command=lambda: self._set_param("l", self.entry_v.get())).grid(row=1, column=5, padx=2, pady=2)

        # Row 2: P Gain, I Gain
        ttk.Label(pid_card, text="P Gain:", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#B0BEC5").grid(row=2, column=0, sticky="w", padx=2, pady=2)
        self.entry_p = tk.Entry(pid_card, width=7, font=("Segoe UI", 9), bg="#222D3E", fg="#FFF", insertbackground="#FFF")
        self.entry_p.insert(0, "0.15")
        self.entry_p.grid(row=2, column=1, padx=2, pady=2)
        tk.Button(pid_card, text="Set P", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000", relief="flat", command=lambda: self._set_param("p", self.entry_p.get())).grid(row=2, column=2, padx=2, pady=2)

        ttk.Label(pid_card, text="I Gain:", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#B0BEC5").grid(row=2, column=3, sticky="w", padx=2, pady=2)
        self.entry_i = tk.Entry(pid_card, width=7, font=("Segoe UI", 9), bg="#222D3E", fg="#FFF", insertbackground="#FFF")
        self.entry_i.insert(0, "1.50")
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
        self.entry_f.insert(0, "0.002")
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
        self.entry_kv.insert(0, "123.0")
        self.entry_kv.grid(row=5, column=4, padx=2, pady=2)
        tk.Button(pid_card, text="Set KV", font=("Segoe UI", 8, "bold"), bg="#B388FF", fg="#000", relief="flat", command=lambda: self._set_param("q", self.entry_kv.get())).grid(row=5, column=5, padx=2, pady=2)

        # Speed Presets Bar
        preset_bar = ttk.Frame(pid_card)
        preset_bar.grid(row=6, column=0, columnspan=6, sticky="ew", pady=(6, 0))
        for spd in [0, 50, 100, 200, 400]:
            btn_col = "#D50000" if spd == 0 else "#2979FF"
            tk.Button(preset_bar, text=f"{spd} RPM", font=("Segoe UI", 8, "bold"), bg=btn_col, fg="#FFF", relief="flat",
                      command=lambda s=spd: self._set_speed_preset(s)).pack(side="left", fill="x", expand=True, padx=2)

        # -------------------------------------------------------------
        # 🔒 COMPILE-TIME HARDWARE & INVERTER CONFIGURATION INFO
        # -------------------------------------------------------------
        hw_info_card = ttk.Frame(left_col, style="Card.TFrame", padding="8 6 8 6")
        hw_info_card.pack(fill="x", pady=4)

        ttk.Label(hw_info_card, text="🔒 COMPILE-TIME HARDWARE CONSTANTS (EDIT IN MAIN.CPP)", style="CardTitle.TLabel").pack(anchor="w", pady=(0, 2))
        info_txt = "PWM Carrier: 25.0 kHz | Pole Pairs: 11 (GM3506) | Align Volt: 4.0V (4.2V Boost) | Hold: 2.4V | SPI: 4.0 MHz | Driver: GPIO 15,16,17,2 EN"
        tk.Label(hw_info_card, text=info_txt, font=("Segoe UI", 8), bg="#1A2230", fg="#8A99AD", anchor="w").pack(fill="x")

        # 5-Minute GA Auto-Tuner & PID Benchmark Card
        bench_card = ttk.Frame(left_col, style="Card.TFrame", padding=10)
        bench_card.pack(fill="x", pady=4)

        ttk.Label(bench_card, text="🧬 5-MIN GA AUTO-TUNER & PID BENCHMARK (D=0, LPF 0.8-3.0ms)", style="CardTitle.TLabel").pack(anchor="w")

        bench_ctrl_row = ttk.Frame(bench_card)
        bench_ctrl_row.pack(fill="x", pady=4)

        self.btn_benchmark = tk.Button(bench_ctrl_row, text="🧬 5-Min GA Auto-Tuner", font=("Segoe UI", 9, "bold"),
                                       bg="#00E676", fg="#000", activebackground="#69F0AE", relief="flat", padx=8, pady=4,
                                       command=lambda: self.benchmarker.toggle_ga(self.btn_benchmark, self.btn_sweep, self.btn_show_results))
        self.btn_benchmark.pack(side="left", padx=3)

        self.btn_sweep = tk.Button(bench_ctrl_row, text="🧪 18-Stage Sweep", font=("Segoe UI", 9, "bold"),
                                   bg="#00E5FF", fg="#000", activebackground="#84FFFF", relief="flat", padx=8, pady=4,
                                   command=lambda: self.benchmarker.toggle_sweep(self.btn_sweep, self.btn_benchmark, self.btn_show_results))
        self.btn_sweep.pack(side="left", padx=3)

        self.btn_show_results = tk.Button(bench_ctrl_row, text="📊 View Leaderboard", font=("Segoe UI", 9, "bold"),
                                          bg="#7C4DFF", fg="#FFF", relief="flat", padx=8, pady=4,
                                          state="disabled", command=lambda: self.benchmarker.show_results_modal(self._apply_benchmark_preset))
        self.btn_show_results.pack(side="left", padx=3)

        self.bench_progress = ttk.Progressbar(bench_card, variable=self.benchmark_progress_var, maximum=100.0)
        self.bench_progress.pack(fill="x", pady=4)

        self.lbl_bench_status = tk.Label(bench_card, textvariable=self.benchmark_status_var, font=("Segoe UI", 9),
                                         bg="#1A2230", fg="#8A99AD", anchor="w")
        self.lbl_bench_status.pack(fill="x")

        # RIGHT COLUMN: Motor Controls, Oscilloscope Chart, & Interactive Serial Console
        right_col = ttk.Frame(main_paned)
        main_paned.add(right_col, minsize=520)

        # -------------------------------------------------------------
        # 🎯 HIGH-PRECISION AUTO-CALIBRATION HUB (ACCURACY FIRST)
        # -------------------------------------------------------------
        cal_hub = ttk.Frame(right_col, style="Card.TFrame", padding=10)
        cal_hub.pack(fill="x", pady=4)

        cal_hdr = ttk.Frame(cal_hub)
        cal_hdr.pack(fill="x", pady=(0, 4))
        ttk.Label(cal_hdr, text="🎯 HIGH-PRECISION AUTO-CALIBRATION HUB", style="CardTitle.TLabel").pack(side="left")

        self.lbl_calib_grade_badge = tk.Label(cal_hdr, textvariable=self.calib_grade_var, font=("Segoe UI", 8, "bold"),
                                              bg="#263238", fg="#B0BEC5", padx=6, pady=1)
        self.lbl_calib_grade_badge.pack(side="right")

        # Config Row
        cal_cfg_row = ttk.Frame(cal_hub)
        cal_cfg_row.pack(fill="x", pady=(2, 4))
        ttk.Label(cal_cfg_row, text="Align Volt (V):", font=("Segoe UI", 9, "bold"), background="#1A2230", foreground="#00E5FF").pack(side="left", padx=2)
        self.entry_cal_v = tk.Entry(cal_cfg_row, width=5, font=("Segoe UI", 9), textvariable=self.calib_align_volt_var, bg="#222D3E", fg="#00E5FF", insertbackground="#FFF")
        self.entry_cal_v.pack(side="left", padx=4)

        ttk.Checkbutton(cal_cfg_row, text="Boost V on Stall", variable=self.calib_boost_retry_var).pack(side="left", padx=4)
        ttk.Checkbutton(cal_cfg_row, text="Verify Closed-Loop", variable=self.calib_verify_var).pack(side="left", padx=4)

        # Master Action Button Row
        cal_act_row = ttk.Frame(cal_hub)
        cal_act_row.pack(fill="x", pady=(4, 4))

        self.btn_master_calib = tk.Button(cal_act_row, text="🚀 1-CLICK MASTER AUTO-CALIBRATE (MULTI-METHOD)", font=("Segoe UI", 10, "bold"),
                                          bg="#00E676", fg="#000", activebackground="#69F0AE", relief="flat", pady=6,
                                          command=lambda: self.calibrator.toggle(self.btn_master_calib))
        self.btn_master_calib.pack(side="left", fill="x", expand=True, padx=2)

        self.cal_prog_bar = ttk.Progressbar(cal_hub, variable=self.calib_progress_var, maximum=100.0)
        self.cal_prog_bar.pack(fill="x", pady=(4, 2))

        self.lbl_cal_status = tk.Label(cal_hub, textvariable=self.calib_status_var, font=("Segoe UI", 8),
                                       bg="#1A2230", fg="#8A99AD", anchor="w")
        self.lbl_cal_status.pack(fill="x")

        # Control Action Buttons Grid
        btn_frame = ttk.Frame(right_col, style="Card.TFrame", padding=10)
        btn_frame.pack(fill="x", pady=4)

        ttk.Label(btn_frame, text="⚡ MOTOR CONTROL & BENCH ACTIONS", style="CardTitle.TLabel").grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 6))

        # Mode Selector Radio Buttons Row
        mode_radio_row = ttk.Frame(btn_frame)
        mode_radio_row.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(0, 6))
        ttk.Label(mode_radio_row, text="Active Mode:", font=("Segoe UI", 8, "bold"),
                  background="#1A2230", foreground="#00E5FF").pack(side="left", padx=2)
        for val, label in [(0, "Raw (0)"), (1, "128-PT LUT (1)"), (2, "Harmonics (2)")]:
            tk.Radiobutton(mode_radio_row, text=label, variable=self.sensor_mode_var, value=val,
                           font=("Segoe UI", 8, "bold"), bg="#1A2230", fg="#FFFFFF", selectcolor="#0E131B",
                           activebackground="#1A2230", activeforeground="#00E5FF",
                           command=self._on_mode_change).pack(side="left", padx=4)

        self._create_btn(btn_frame, "🚀 Harmonic Calib", "CALIB_DYN", "#00E676", 2, 0)
        self._create_btn(btn_frame, "🔬 Stepped LUT", "CALIB_LUT", "#E040FB", 2, 1)
        self._create_btn(btn_frame, "🏁 3-Way Bench", "BENCH_COMPARE", "#00E5FF", 2, 2)
        self._create_btn(btn_frame, "🛑 STOP / Dis (x)", "x", "#D50000", 2, 3)

        self._create_btn(btn_frame, "🟢 Enable (e)", "e", "#00E676", 3, 0)
        self._create_btn(btn_frame, "⚡ 11-Pole Align (c)", "c", "#7C4DFF", 3, 1)
        self._create_btn(btn_frame, "🎯 Zero / Tare (z)", "z", "#FFD600", 3, 2)
        self._create_btn(btn_frame, "🏎️ Test 1000 RPM", "t1000", "#00E676", 3, 3)

        self._create_btn(btn_frame, "🏎️ RPM +10 (+)", "+", "#00E676", 4, 0)
        self._create_btn(btn_frame, "🏎️ RPM -10 (-)", "-", "#FF5252", 4, 1)
        self._create_btn(btn_frame, "⚡ Trim Lag (n1000)", "n1000", "#FFD600", 4, 2)
        self._create_btn(btn_frame, "📋 Dump LUT (u)", "u", "#00E5FF", 4, 3)

        self._create_btn(btn_frame, "💾 Save Flash (NVS)", "save", "#00E5FF", 5, 0)
        self._create_btn(btn_frame, "🔄 Load Flash (NVS)", "load", "#2979FF", 5, 1)
        self._create_btn(btn_frame, "💾 Save JSON File", "SAVE_JSON", "#7C4DFF", 5, 2)
        self._create_btn(btn_frame, "📂 Load JSON File", "LOAD_JSON", "#E040FB", 5, 3)

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

        self.btn_run_kv = tk.Button(btn_row1, text="🚀 START 18-Stage KV Sweep (k)", font=("Segoe UI", 9, "bold"),
                                    bg="#00E676", fg="#000", activebackground="#69F0AE", relief="flat", padx=10, pady=5,
                                    command=lambda: self.kv_analyzer.start_test(self.btn_run_kv, self.kv_tree))
        self.btn_run_kv.pack(side="left", fill="x", expand=True, padx=2)

        self.btn_stop_kv = tk.Button(btn_row1, text="⏹️ Abort (x)", font=("Segoe UI", 9, "bold"),
                                     bg="#D50000", fg="#FFF", relief="flat", padx=8, pady=5,
                                     command=lambda: self.kv_analyzer.abort_test(self.btn_run_kv))
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
                  relief="flat", padx=6, pady=1, command=self.kv_analyzer.export_report).pack(side="right", padx=2)

        tk.Button(table_hdr, text="⚡ Apply to Motor", font=("Segoe UI", 8, "bold"), bg="#B388FF", fg="#000",
                  relief="flat", padx=6, pady=1, command=lambda: self.kv_analyzer.apply_kv_to_motor(self.entry_kv, self._set_param)).pack(side="right", padx=2)

        tk.Button(table_hdr, text="📋 Copy Snippet", font=("Segoe UI", 8), bg="#263238", fg="#B0BEC5",
                  relief="flat", padx=5, pady=1, command=self.kv_analyzer.copy_simplefoc_snippet).pack(side="right", padx=2)

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
        self.entry_align_v.insert(0, "4.0")
        self.entry_align_v.pack(side="left", padx=4)
        tk.Button(cal_row, text="Set Volt", font=("Segoe UI", 8, "bold"), bg="#00E5FF", fg="#000", relief="flat",
                  command=lambda: self._set_param("y", self.entry_align_v.get())).pack(side="left", padx=2)

        tk.Label(cal_row, text="(GM3506 draws ~0.65A @ 4.0V vs 0.75A @ 4.2V boost | Hold 2.4V)", font=("Segoe UI", 8), bg="#1A2230", fg="#8A99AD").pack(side="left", padx=6)
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
        self.entry_zero_angle.insert(0, f"{self.symmetry_equalizer.current_zero_angle_rad:.5f}")
        self.entry_zero_angle.pack(side="left", padx=4)

        tk.Button(angle_display_row, text="Apply Angle", font=("Segoe UI", 8, "bold"), bg="#B388FF", fg="#000", relief="flat",
                  command=self._apply_manual_zero_angle).pack(side="left", padx=2)

        self.lbl_angle_deg = tk.Label(angle_display_row, text=f"({self.symmetry_equalizer.current_zero_angle_rad*57.29578:.2f}°)", font=("Segoe UI", 9), bg="#1A2230", fg="#8A99AD")
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
                  command=lambda: self.reader.send_cmd("v0.0")).pack(side="left", padx=3)

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
                                      command=lambda: self.symmetry_equalizer.toggle(
                                          self.btn_auto_sym, self.sym_tree, self.entry_sym_v,
                                          self.entry_zero_angle, self.entry_align_v, self.entry_kv, self.chk_recalib_var
                                      ))
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
        self.card_sym_live_uq = self._create_card(m_grid, "COMMANDED VOLTAGE (Uq)", "0.00 V", "#00E5FF", 0, 1)
        self.card_sym_fwd = self._create_card(m_grid, "FORWARD SPEED (+V)", "+0.0 RPM", "#00E5FF", 1, 0)
        self.card_sym_rev = self._create_card(m_grid, "REVERSE SPEED (-V)", "-0.0 RPM", "#FF4081", 1, 1)
        self.card_sym_delta = self._create_card(m_grid, "SPEED DELTA (Δ RPM)", "0.0 RPM", "#FFD600", 2, 0)
        self.card_sym_pct = self._create_card(m_grid, "DIRECTIONAL SYMMETRY", "0.0 %", "#69F0AE", 2, 1)
        self.card_sym_kv = self._create_card(m_grid, "TRUE EQUALIZED KV", "0.0 RPM/V", "#B388FF", 3, 0)
        self.card_sym_ke = self._create_card(m_grid, "BACK-EMF CONSTANT (Ke)", "0.0000 V·s/rad", "#00E5FF", 3, 1)

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

        curr_ang = self.symmetry_equalizer.current_zero_angle_rad
        lines.append("-------------------------------------------------------------------------------------------------------------------------")
        lines.append(f"Final Neutral Zero Angle : {curr_ang:.5f} rad ({curr_ang * 57.29578:.2f}°)")
        lines.append(f"Final Equalized Motor KV : {self.entry_kv.get()} RPM/V")
        lines.append(f"Status                   : {self.sym_status_var.get()}")
        lines.append("=========================================================================================================================\n")

        full_text = "\n".join(lines)
        self.root.clipboard_clear()
        self.root.clipboard_append(full_text)
        messagebox.showinfo("Report Copied", "Optimization feedback report successfully copied to clipboard!")

    def _on_zero_angle_updated(self, angle_rad):
        self.symmetry_equalizer.current_zero_angle_rad = angle_rad
        self.entry_zero_angle.delete(0, "end")
        self.entry_zero_angle.insert(0, f"{angle_rad:.5f}")
        self.lbl_angle_deg.config(text=f"({angle_rad*57.29578:.2f}°)")

    def _nudge_zero_angle(self, delta_rad):
        curr_ang = (self.symmetry_equalizer.current_zero_angle_rad + delta_rad) % (2.0 * math.pi)
        self._on_zero_angle_updated(curr_ang)
        self.reader.send_cmd(f"j{curr_ang:.5f}")

    def _apply_manual_zero_angle(self):
        try:
            val = float(self.entry_zero_angle.get().strip())
            curr_ang = val % (2.0 * math.pi)
            self._on_zero_angle_updated(curr_ang)
            self.reader.send_cmd(f"j{curr_ang:.5f}")
        except ValueError:
            messagebox.showwarning("Input Error", "Invalid numeric angle in radians.")

    def _spin_forward_test(self):
        try:
            v = abs(float(self.entry_sym_v.get().strip()))
            self.reader.send_cmd(f"v{v:.2f}")
        except ValueError:
            pass

    def _spin_reverse_test(self):
        try:
            v = abs(float(self.entry_sym_v.get().strip()))
            self.reader.send_cmd(f"v-{v:.2f}")
        except ValueError:
            pass

    def _copy_harmonic_report(self):
        with self.reader.lock:
            coeffs = dict(getattr(self.reader, 'harmonic_coeffs', {}))
        if not coeffs:
            messagebox.showinfo("Report Empty", "No harmonic calibration data available yet. Run 'CALIB_DYN' first.")
            return

        a1 = coeffs.get("A1", 0.0); b1 = coeffs.get("B1", 0.0); h1 = coeffs.get("H1", math.hypot(a1, b1) * 57.2957795)
        a2 = coeffs.get("A2", 0.0); b2 = coeffs.get("B2", 0.0); h2 = coeffs.get("H2", math.hypot(a2, b2) * 57.2957795)
        a3 = coeffs.get("A3", 0.0); b3 = coeffs.get("B3", 0.0); h3 = coeffs.get("H3", math.hypot(a3, b3) * 57.2957795)
        a4 = coeffs.get("A4", 0.0); b4 = coeffs.get("B4", 0.0); h4 = coeffs.get("H4", math.hypot(a4, b4) * 57.2957795)

        phi1 = coeffs.get("Phi1", math.degrees(math.atan2(b1, a1)) % 360.0)
        phi2 = coeffs.get("Phi2", math.degrees(math.atan2(b2, a2)) % 360.0)
        phi3 = coeffs.get("Phi3", math.degrees(math.atan2(b3, a3)) % 360.0)
        phi4 = coeffs.get("Phi4", math.degrees(math.atan2(b4, a4)) % 360.0)

        r2 = coeffs.get("R2", 0.0)
        pk_pk = coeffs.get("PkPk", 0.0)
        rms = coeffs.get("RMS", 0.0)
        res_pk = coeffs.get("ResPkPk", 0.0)
        res_rms = coeffs.get("ResRMS", 0.0)
        lag_m = coeffs.get("LagMean", 0.0)
        lag_rip = coeffs.get("LagRipple", 0.0)
        lag_ang = coeffs.get("LagPeakAng", 0.0)

        apex1 = phi1
        apex2 = (phi2 * 0.5) % 360.0
        apex3 = (phi3 / 3.0) % 360.0
        apex4 = (phi4 * 0.25) % 360.0

        lines = [
            "================================================================================",
            "🔬 MT6816 CONTINUOUS HARMONIC CALIBRATION & PHYSICAL DIAGNOSTIC REPORT",
            "================================================================================",
            f"Active Mode: Mode {self.sensor_mode_var.get()}",
            "--------------------------------------------------------------------------------",
            "PHYSICAL HARMONIC DECOMPOSITION & APEX LOCALIZATION:",
            f"  • Harmonic 1 (Runout/Eccentricity) : ±{h1:.4f}°  [A1={a1:+.6f}, B1={b1:+.6f}]",
            f"    Phase φ1: {phi1:5.1f}°  ==> Rotor Eccentricity Apex at: {apex1:5.1f}° mech",
            f"  • Harmonic 2 (Magnet Tilt/Ovality) : ±{h2:.4f}°  [A2={a2:+.6f}, B2={b2:+.6f}]",
            f"    Phase φ2: {phi2:5.1f}°  ==> Magnet Tilt Axis Apex at   : {apex2:5.1f}° & {(apex2+180)%360:5.1f}° mech",
            f"  • Harmonic 3 (3-Point Chassis Pinch: ±{h3:.4f}°  [A3={a3:+.6f}, B3={b3:+.6f}]",
            f"    Phase φ3: {phi3:5.1f}°  ==> Primary Clamp Pinch Apex at: {apex3:5.1f}° mech (every 120°)",
            f"  • Harmonic 4 (Bridge/Saturation)   : ±{h4:.4f}°  [A4={a4:+.6f}, B4={b4:+.6f}]",
            f"    Phase φ4: {phi4:5.1f}°  ==> Quad-Bridge Mismatch Axis  : {apex4:5.1f}° mech (every 90°)",
            "--------------------------------------------------------------------------------",
            "CALIBRATION QUALITY, RIPPLE REDUCTION & FIT:",
            f"  • Raw Uncalibrated Error Span      : {pk_pk:.4f}° mech (±{pk_pk*0.5:.4f}°) | RMS: {rms:.4f}°",
            f"  • Reconstructed Residual Ripple    : {res_pk:.4f}° mech (±{res_pk*0.5:.4f}°) | RMS: {res_rms:.4f}°",
            f"  • Non-Linearity Ripple Reduction   : {(1.0 - (res_pk/pk_pk))*100.0 if pk_pk > 1e-4 else 0.0:.1f}%",
            f"  • Harmonic Model Fit Quality (R²)  : {r2*100.0:.2f}%",
            "--------------------------------------------------------------------------------",
            "DYNAMIC MECHANICAL DRAG & BEARING UNIFORMITY (38 RPM):",
            f"  • Mean Directional Rotor Lag       : {lag_m:.4f}° mech",
            f"  • Bearing Drag Non-Uniformity (Pk) : {lag_rip:.4f}° mech",
            f"  • Peak Friction / Binding Angle    : {lag_ang:.1f}° mech",
            "--------------------------------------------------------------------------------",
            "PHYSICAL FAULT DIAGNOSTICS & ACTIONS:",
            f"  • Chassis Clamping Distortion      : {'⚠️ WARNING: H3 exceeds 20% H1 (Pinch at ' + f'{apex3:.1f}°)' if (h1 > 0.02 and h3/h1 > 0.20) else '✅ PASS (<20% of H1)'}",
            f"  • Hall Quad-Bridge / Saturation    : {'⚠️ WARNING: H4 exceeds 25% H1 (Airgap < 0.8mm)' if (h1 > 0.02 and h4/h1 > 0.25) else '✅ PASS (<25% of H1)'}",
            "================================================================================\n"
        ]
        text_report = "\n".join(lines)
        self.root.clipboard_clear()
        self.root.clipboard_append(text_report)
        messagebox.showinfo("Report Copied", "Complete physical harmonic diagnostic report copied to clipboard!")

    def _on_mode_change(self):
        m = self.sensor_mode_var.get()
        self.reader.send_cmd(f"SET_MODE {m}")

    def _build_harmonic_tab(self, parent):
        scroll_container = ttk.Frame(parent)
        scroll_container.pack(fill="both", expand=True, padx=8, pady=8)

        # Top Control & Mode Card
        ctrl_card = ttk.Frame(scroll_container, style="Card.TFrame", padding=10)
        ctrl_card.pack(fill="x", pady=(0, 6))

        top_row = ttk.Frame(ctrl_card)
        top_row.pack(fill="x", pady=(0, 6))

        ttk.Label(top_row, text="🔬 CONTINUOUS SPATIAL-BINNED HARMONIC CALIBRATION & ANALYSIS", style="CardTitle.TLabel").pack(side="left")

        # Action Buttons Row
        btn_bar = ttk.Frame(ctrl_card)
        btn_bar.pack(fill="x", pady=(2, 6))

        tk.Button(btn_bar, text="🚀 Continuous Harmonic Calib (CALIB_DYN)", font=("Segoe UI", 9, "bold"),
                  bg="#00E676", fg="#000", activebackground="#69F0AE", relief="flat", padx=10, pady=5,
                  command=lambda: self.reader.send_cmd("CALIB_DYN")).pack(side="left", padx=3)

        tk.Button(btn_bar, text="🔬 Stepped LUT Calib (CALIB_LUT)", font=("Segoe UI", 9, "bold"),
                  bg="#E040FB", fg="#000", activebackground="#EA80FC", relief="flat", padx=10, pady=5,
                  command=lambda: self.reader.send_cmd("CALIB_LUT")).pack(side="left", padx=3)

        tk.Button(btn_bar, text="🏁 3-Way Mode Benchmark (BENCH_COMPARE)", font=("Segoe UI", 9, "bold"),
                  bg="#00E5FF", fg="#000", activebackground="#84FFFF", relief="flat", padx=10, pady=5,
                  command=lambda: self.reader.send_cmd("BENCH_COMPARE")).pack(side="left", padx=3)

        tk.Button(btn_bar, text="📋 Copy Diagnostic Report", font=("Segoe UI", 9, "bold"),
                  bg="#FFD600", fg="#000", activebackground="#FFE57F", relief="flat", padx=10, pady=5,
                  command=self._copy_harmonic_report).pack(side="left", padx=3)

        tk.Button(btn_bar, text="⏱️ FOC Profiler (perf)", font=("Segoe UI", 9, "bold"),
                  bg="#FF9100", fg="#000", activebackground="#FFB74D", relief="flat", padx=10, pady=5,
                  command=lambda: self.reader.send_cmd("perf")).pack(side="left", padx=3)

        tk.Button(btn_bar, text="💾 Save NVS", font=("Segoe UI", 8, "bold"), bg="#2979FF", fg="#FFF",
                  relief="flat", padx=8, pady=5, command=lambda: self.reader.send_cmd("save")).pack(side="right", padx=2)
        tk.Button(btn_bar, text="🔄 Load NVS", font=("Segoe UI", 8, "bold"), bg="#455A64", fg="#FFF",
                  relief="flat", padx=8, pady=5, command=lambda: self.reader.send_cmd("load")).pack(side="right", padx=2)

        # Radio Buttons Row
        mode_row = ttk.Frame(ctrl_card)
        mode_row.pack(fill="x", pady=(4, 2))
        ttk.Label(mode_row, text="Active Runtime Correction Mode:", font=("Segoe UI", 9, "bold"),
                  background="#1A2230", foreground="#00E5FF").pack(side="left", padx=(2, 10))

        modes = [
            (0, "Mode 0: Raw (Uncalibrated)"),
            (1, "Mode 1: 128-Point Stepped LUT"),
            (2, "Mode 2: Continuous Harmonics (Double-Angle Identities)")
        ]
        for val, label in modes:
            tk.Radiobutton(mode_row, text=label, variable=self.sensor_mode_var, value=val,
                           font=("Segoe UI", 9, "bold"), bg="#1A2230", fg="#FFFFFF", selectcolor="#0E131B",
                           activebackground="#1A2230", activeforeground="#00E5FF",
                           command=self._on_mode_change).pack(side="left", padx=8)

        # Harmonic Overlay Graph Canvas Card
        plot_card = ttk.Frame(scroll_container, style="Card.TFrame", padding=10)
        plot_card.pack(fill="x", pady=4)

        ttk.Label(plot_card, text="SPATIAL-BINNED DRAG-FREE SAMPLES & ANALYTICAL HARMONIC CURVE (128 BINS ACROSS 360° MECH)",
                  style="CardTitle.TLabel").pack(anchor="w")

        self.harm_canvas = tk.Canvas(plot_card, bg="#10151E", height=240, highlightthickness=0)
        self.harm_canvas.pack(fill="x", expand=True, pady=4)

        # Primary Diagnostic Cards Grid (H1, H2, H3, H4)
        diag_grid = ttk.Frame(scroll_container)
        diag_grid.pack(fill="x", pady=4)
        diag_grid.columnconfigure((0, 1, 2, 3), weight=1)

        # Card 1: H1
        c1 = ttk.Frame(diag_grid, style="Card.TFrame", padding=8)
        c1.grid(row=0, column=0, sticky="nsew", padx=2)
        ttk.Label(c1, text="HARMONIC 1: RUNOUT / ECCENTRICITY", style="CardTitle.TLabel").pack(anchor="w")
        self.lbl_h1_val = tk.Label(c1, text="±0.000° mech", font=("Segoe UI", 12, "bold"), bg="#1A2230", fg="#69F0AE")
        self.lbl_h1_val.pack(anchor="w", pady=1)
        self.lbl_h1_desc = tk.Label(c1, text="Apex at --° mech | A1=0.00, B1=0.00", font=("Segoe UI", 8), bg="#1A2230", fg="#8A99AD")
        self.lbl_h1_desc.pack(anchor="w")
        self.lbl_h1_status = tk.Label(c1, text="Status: Ready", font=("Segoe UI", 8, "bold"), bg="#1A2230", fg="#00E5FF")
        self.lbl_h1_status.pack(anchor="w")

        # Card 2: H2
        c2 = ttk.Frame(diag_grid, style="Card.TFrame", padding=8)
        c2.grid(row=0, column=1, sticky="nsew", padx=2)
        ttk.Label(c2, text="HARMONIC 2: MAGNET TILT / ELLIPTICITY", style="CardTitle.TLabel").pack(anchor="w")
        self.lbl_h2_val = tk.Label(c2, text="±0.000° mech", font=("Segoe UI", 12, "bold"), bg="#1A2230", fg="#69F0AE")
        self.lbl_h2_val.pack(anchor="w", pady=1)
        self.lbl_h2_desc = tk.Label(c2, text="Tilt Axis at --° & --° mech", font=("Segoe UI", 8), bg="#1A2230", fg="#8A99AD")
        self.lbl_h2_desc.pack(anchor="w")
        self.lbl_h2_status = tk.Label(c2, text="Status: Ready", font=("Segoe UI", 8, "bold"), bg="#1A2230", fg="#00E5FF")
        self.lbl_h2_status.pack(anchor="w")

        # Card 3: H3
        c3 = ttk.Frame(diag_grid, style="Card.TFrame", padding=8)
        c3.grid(row=0, column=2, sticky="nsew", padx=2)
        ttk.Label(c3, text="HARMONIC 3: 3-POINT CLAMPING", style="CardTitle.TLabel").pack(anchor="w")
        self.lbl_h3_val = tk.Label(c3, text="±0.000° mech", font=("Segoe UI", 12, "bold"), bg="#1A2230", fg="#69F0AE")
        self.lbl_h3_val.pack(anchor="w", pady=1)
        self.lbl_h3_desc = tk.Label(c3, text="Pinch Apex at --° mech (every 120°)", font=("Segoe UI", 8), bg="#1A2230", fg="#8A99AD")
        self.lbl_h3_desc.pack(anchor="w")
        self.lbl_h3_status = tk.Label(c3, text="Status: Ready (<20% H1)", font=("Segoe UI", 8, "bold"), bg="#1A2230", fg="#00E5FF")
        self.lbl_h3_status.pack(anchor="w")

        # Card 4: H4
        c4 = ttk.Frame(diag_grid, style="Card.TFrame", padding=8)
        c4.grid(row=0, column=3, sticky="nsew", padx=2)
        ttk.Label(c4, text="HARMONIC 4: FLUX SATURATION / BRIDGE", style="CardTitle.TLabel").pack(anchor="w")
        self.lbl_h4_val = tk.Label(c4, text="±0.000° mech", font=("Segoe UI", 12, "bold"), bg="#1A2230", fg="#69F0AE")
        self.lbl_h4_val.pack(anchor="w", pady=1)
        self.lbl_h4_desc = tk.Label(c4, text="Mismatch Axis at --° mech (every 90°)", font=("Segoe UI", 8), bg="#1A2230", fg="#8A99AD")
        self.lbl_h4_desc.pack(anchor="w")
        self.lbl_h4_status = tk.Label(c4, text="Status: Ready (<25% H1)", font=("Segoe UI", 8, "bold"), bg="#1A2230", fg="#00E5FF")
        self.lbl_h4_status.pack(anchor="w")

        # Secondary Quality & Dynamic Metrics Grid
        diag_grid2 = ttk.Frame(scroll_container)
        diag_grid2.pack(fill="x", pady=(2, 4))
        diag_grid2.columnconfigure((0, 1, 2, 3), weight=1)

        # Card 5: Fit Quality R^2
        c5 = ttk.Frame(diag_grid2, style="Card.TFrame", padding=8)
        c5.grid(row=0, column=0, sticky="nsew", padx=2)
        ttk.Label(c5, text="HARMONIC FIT QUALITY (R²)", style="CardTitle.TLabel").pack(anchor="w")
        self.lbl_r2_val = tk.Label(c5, text="0.0 %", font=("Segoe UI", 12, "bold"), bg="#1A2230", fg="#69F0AE")
        self.lbl_r2_val.pack(anchor="w", pady=1)
        self.lbl_r2_desc = tk.Label(c5, text="Variance Explained (H1+H2+H4)", font=("Segoe UI", 8), bg="#1A2230", fg="#8A99AD")
        self.lbl_r2_desc.pack(anchor="w")
        self.lbl_r2_status = tk.Label(c5, text="Status: Ready", font=("Segoe UI", 8, "bold"), bg="#1A2230", fg="#00E5FF")
        self.lbl_r2_status.pack(anchor="w")

        # Card 6: Residual Non-Harmonic Ripple
        c6 = ttk.Frame(diag_grid2, style="Card.TFrame", padding=8)
        c6.grid(row=0, column=1, sticky="nsew", padx=2)
        ttk.Label(c6, text="RESIDUAL NON-HARMONIC RIPPLE", style="CardTitle.TLabel").pack(anchor="w")
        self.lbl_res_val = tk.Label(c6, text="±0.000° mech", font=("Segoe UI", 12, "bold"), bg="#1A2230", fg="#69F0AE")
        self.lbl_res_val.pack(anchor="w", pady=1)
        self.lbl_res_desc = tk.Label(c6, text="RMS Residual: 0.000°", font=("Segoe UI", 8), bg="#1A2230", fg="#8A99AD")
        self.lbl_res_desc.pack(anchor="w")
        self.lbl_res_status = tk.Label(c6, text="Ripple Reduction: -- %", font=("Segoe UI", 8, "bold"), bg="#1A2230", fg="#00E5FF")
        self.lbl_res_status.pack(anchor="w")

        # Card 7: Dynamic Rotor Lag
        c7 = ttk.Frame(diag_grid2, style="Card.TFrame", padding=8)
        c7.grid(row=0, column=2, sticky="nsew", padx=2)
        ttk.Label(c7, text="DYNAMIC ROTOR LAG (38 RPM)", style="CardTitle.TLabel").pack(anchor="w")
        self.lbl_lag_val = tk.Label(c7, text="0.000° mech", font=("Segoe UI", 12, "bold"), bg="#1A2230", fg="#00E5FF")
        self.lbl_lag_val.pack(anchor="w", pady=1)
        self.lbl_lag_desc = tk.Label(c7, text="Phase Lag under Dynamic Load", font=("Segoe UI", 8), bg="#1A2230", fg="#8A99AD")
        self.lbl_lag_desc.pack(anchor="w")
        self.lbl_lag_status = tk.Label(c7, text="Status: Ready", font=("Segoe UI", 8, "bold"), bg="#1A2230", fg="#69F0AE")
        self.lbl_lag_status.pack(anchor="w")

        # Card 8: Bearing Friction Uniformity
        c8 = ttk.Frame(diag_grid2, style="Card.TFrame", padding=8)
        c8.grid(row=0, column=3, sticky="nsew", padx=2)
        ttk.Label(c8, text="BEARING FRICTION UNIFORMITY", style="CardTitle.TLabel").pack(anchor="w")
        self.lbl_fric_val = tk.Label(c8, text="Δ0.000° mech", font=("Segoe UI", 12, "bold"), bg="#1A2230", fg="#69F0AE")
        self.lbl_fric_val.pack(anchor="w", pady=1)
        self.lbl_fric_desc = tk.Label(c8, text="Peak Drag Angle: --° mech", font=("Segoe UI", 8), bg="#1A2230", fg="#8A99AD")
        self.lbl_fric_desc.pack(anchor="w")
        self.lbl_fric_status = tk.Label(c8, text="Status: Ready", font=("Segoe UI", 8, "bold"), bg="#1A2230", fg="#00E5FF")
        self.lbl_fric_status.pack(anchor="w")

        # Benchmark Comparison Leaderboard Card
        bench_card = ttk.Frame(scroll_container, style="Card.TFrame", padding=10)
        bench_card.pack(fill="x", pady=4)

        b_hdr = ttk.Frame(bench_card)
        b_hdr.pack(fill="x", pady=(0, 4))
        ttk.Label(b_hdr, text="🏆 HEAD-TO-HEAD BENCHMARK COMPARISON LEADERBOARD (GM3506 @ 30 rad/s)", style="CardTitle.TLabel").pack(side="left")

        # Treeview table
        cols = ("mode", "name", "latency", "jitter", "mae", "temp", "zero")
        self.bench_tree = ttk.Treeview(bench_card, columns=cols, show="headings", height=3)
        self.bench_tree.heading("mode", text="Mode")
        self.bench_tree.heading("name", text="Description")
        self.bench_tree.heading("latency", text="Read Latency (µs)")
        self.bench_tree.heading("jitter", text="Velocity Jitter σ (rad/s)")
        self.bench_tree.heading("mae", text="Tracking MAE (rad/s)")
        self.bench_tree.heading("temp", text="ΔTemp (°C)")
        self.bench_tree.heading("zero", text="Zero Angle (rad)")

        self.bench_tree.column("mode", width=50, anchor="center")
        self.bench_tree.column("name", width=200, anchor="w")
        self.bench_tree.column("latency", width=120, anchor="center")
        self.bench_tree.column("jitter", width=140, anchor="center")
        self.bench_tree.column("mae", width=130, anchor="center")
        self.bench_tree.column("temp", width=90, anchor="center")
        self.bench_tree.column("zero", width=130, anchor="center")
        self.bench_tree.pack(fill="x", pady=2)

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

    def _set_speed_preset(self, speed_rpm):
        self.entry_target.delete(0, "end")
        self.entry_target.insert(0, str(float(speed_rpm)))
        self._set_param("t", str(float(speed_rpm)))

    def _nudge_target_rpm(self, delta):
        try:
            curr = float(self.entry_target.get().strip())
        except Exception:
            with self.reader.lock:
                curr = self.reader.last_target_rpm
        new_rpm = max(-1432.0, min(1432.0, curr + delta))
        self.entry_target.delete(0, "end")
        self.entry_target.insert(0, f"{new_rpm:.1f}")
        self._set_param("t", f"{new_rpm:.1f}")

    def _reset_gm3506_defaults(self):
        self.entry_ramp.delete(0, "end"); self.entry_ramp.insert(0, "200.0")
        self.entry_v.delete(0, "end"); self.entry_v.insert(0, "6.93")
        self.entry_max_rpm.delete(0, "end"); self.entry_max_rpm.insert(0, "1432.0")
        self.entry_p.delete(0, "end"); self.entry_p.insert(0, "0.12")
        self.entry_i.delete(0, "end"); self.entry_i.insert(0, "1.20")
        self.entry_d.delete(0, "end"); self.entry_d.insert(0, "0.000")
        self.entry_f.delete(0, "end"); self.entry_f.insert(0, "0.005")
        self.entry_lag.delete(0, "end"); self.entry_lag.insert(0, "35.0")
        self.entry_kv.delete(0, "end"); self.entry_kv.insert(0, "122.1")
        self.entry_target.delete(0, "end"); self.entry_target.insert(0, "0.0")

        self.reader.send_cmd("a200.0")
        self.reader.send_cmd("l6.93")
        self.reader.send_cmd("m1432.0")
        self.reader.send_cmd("p0.12")
        self.reader.send_cmd("i1.20")
        self.reader.send_cmd("d0.000")
        self.reader.send_cmd("f0.005")
        self.reader.send_cmd("s35.0")
        self.reader.send_cmd("q171.9")
        self.reader.send_cmd("t0.0")

    def _apply_benchmark_preset(self, target_res):
        self.entry_p.delete(0, "end"); self.entry_p.insert(0, str(target_res['p']))
        self.entry_i.delete(0, "end"); self.entry_i.insert(0, str(target_res['i']))
        self.entry_d.delete(0, "end"); self.entry_d.insert(0, str(target_res['d']))
        self.entry_f.delete(0, "end"); self.entry_f.insert(0, str(target_res['tf']))

        self._set_param("p", str(target_res['p']))
        self._set_param("i", str(target_res['i']))
        self._set_param("d", str(target_res['d']))
        self._set_param("f", str(target_res['tf']))

    def _set_param(self, prefix, val_str):
        try:
            val = float(val_str.strip())
            cmd = f"{prefix}{val}"
            self.reader.send_cmd(cmd)
        except ValueError:
            messagebox.showwarning("Input Error", f"Invalid numeric value: '{val_str}'")

    def emergency_stop(self):
        """Universal Emergency Stop: Immediately terminates all worker threads, aborts sweeps, and disables inverter."""
        self.calibrator.abort()
        self.symmetry_equalizer.abort()
        self.benchmarker.abort()

        try:
            self.btn_master_calib.config(text="🚀 1-CLICK MASTER AUTO-CALIBRATE (MULTI-METHOD)", bg="#00E676", fg="#000")
            self.btn_auto_sym.config(text="🚀 1-CLICK AUTO-EQUALIZE SYMMETRY", bg="#00E676", fg="#000")
            self.btn_benchmark.config(text="🧬 5-Min GA Auto-Tuner", bg="#00E676", fg="#000", state="normal")
            if hasattr(self, 'btn_sweep'):
                self.btn_sweep.config(text="🧪 18-Stage Sweep", bg="#00E5FF", fg="#000", state="normal")
            self.btn_run_kv.config(text="🚀 START 18-Stage KV Sweep (k)", bg="#00E676", fg="#000")
        except Exception:
            pass

        self.kv_status_var.set("🛑 EMERGENCY STOP: KV test stopped.")
        self.reader.send_emergency_stop()

    def _on_space_pressed(self, event):
        widget = event.widget
        if isinstance(widget, (tk.Entry, ttk.Entry, tk.Text)):
            return
        self.emergency_stop()

    def _handle_btn(self, cmd):
        if cmd == "x" or cmd == "X":
            self.emergency_stop()
        elif cmd == "+":
            self._nudge_target_rpm(+10.0)
        elif cmd == "-":
            self._nudge_target_rpm(-10.0)
        elif cmd == "FLASH":
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

        with self.reader.lock:
            detected_dir = getattr(self.reader, 'last_sensor_direction', -1)
        try:
            lag_val = float(self.entry_lag.get().strip() or "35.0")
        except ValueError:
            lag_val = 35.0

        cfg = {
            "zero_electric_angle": self.symmetry_equalizer.current_zero_angle_rad,
            "sensor_direction": detected_dir,
            "sensor_lag_comp_us": lag_val,
            "motor_kv_rating": float(self.entry_kv.get().strip() or "123.0"),
            "pid_p": float(self.entry_p.get().strip() or "0.15"),
            "pid_i": float(self.entry_i.get().strip() or "1.50"),
            "pid_d": float(self.entry_d.get().strip() or "0.00"),
            "lpf_tf": float(self.entry_f.get().strip() or "0.002"),
            "date": time.strftime("%Y-%m-%d %H:%M:%S")
        }
        try:
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
            with open(filename, "r", encoding="utf-8") as f:
                cfg = json.load(f)

            if "zero_electric_angle" in cfg:
                ang = float(cfg["zero_electric_angle"])
                self._on_zero_angle_updated(ang)
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
                self.entry_lag.delete(0, "end"); self.entry_lag.insert(0, str(cfg["sensor_lag_comp_us"]))
                self.reader.send_cmd(f"s{cfg['sensor_lag_comp_us']}")
                time.sleep(0.05)
            if "motor_kv_rating" in cfg:
                self.entry_kv.delete(0, "end"); self.entry_kv.insert(0, str(cfg["motor_kv_rating"]))
                self.reader.send_cmd(f"q{cfg['motor_kv_rating']}")

            messagebox.showinfo("Config Uploaded", f"Profile loaded and applied to motor controller:\n{filename}")
        except Exception as e:
            messagebox.showerror("Upload Error", f"Failed to load profile:\n{str(e)}")

    def _flash_firmware(self):
        ans = messagebox.askyesno("Flash ESP32-S3", "Upload firmware to ESP32-S3 (esp32s3_suite)?\n(ESP32-S3 should be connected in DFU / Boot mode)")
        if not ans:
            return
        self.reader.disconnect()
        self.conn_label.config(text="FLASHING...", fg="#FFD600")
        self.root.update()

        def do_flash():
            pio_exe = find_pio_executable()
            workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
            cmd = [pio_exe, "run", "-e", "esp32s3_suite", "-t", "upload"]
            try:
                res = subprocess.run(cmd, capture_output=True, text=True, cwd=workspace_root)
                if res.returncode == 0:
                    self.root.after(0, lambda: messagebox.showinfo("Flash Success", "ESP32-S3 firmware (esp32s3_suite) flashed successfully!"))
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

        with self.reader.lock:
            foc_khz = getattr(self.reader, "last_foc_khz", 0.0)
            lat_avg = getattr(self.reader, "last_lat_avg_us", 0.0)
            lat_max = getattr(self.reader, "last_lat_max_us", 0.0)
            spk_cnt = getattr(self.reader, "last_spikes_count", 0)

        if foc_khz > 0.0:
            self.lbl_foc.config(text=f"FOC: {foc_khz:.1f} kHz", bg="#00E676" if foc_khz >= 5.0 else "#FFD600", fg="#000000")
        else:
            self.lbl_foc.config(text="FOC: -- kHz", bg="#1B5E20", fg="#69F0AE")

        if lat_avg > 0.0:
            if spk_cnt == 0:
                self.lbl_lat.config(text=f"LAT: {lat_avg:.0f}/{lat_max:.0f} µs", bg="#00897B", fg="#FFFFFF")
            else:
                self.lbl_lat.config(text=f"LAT: {lat_avg:.0f}/{lat_max:.0f} µs | 🚨 {spk_cnt} SPK", bg="#D50000", fg="#FFFFFF")
        else:
            self.lbl_lat.config(text="LAT: -- µs", bg="#263238", fg="#B0BEC5")

        if self.reader.connected:
            self.conn_label.config(text=f"ONLINE ({self.reader.port})", fg="#69F0AE")
            self.btn_connect.config(text="Disconnect", bg="#FF5252")
        else:
            self.conn_label.config(text="RECONNECTING...", fg="#FFD600")

        # Sync Auto-Calibration UI elements
        with self.reader.lock:
            c_prog = self.reader.calib_progress_pct
            c_stage = self.reader.calib_stage_name
            c_active = self.reader.calibrating
            c_grade = self.reader.last_calib_grade

        if not self.calibrator.running and c_active:
            if c_prog > 0:
                self.calib_progress_var.set(c_prog)
            if c_stage:
                self.calib_status_var.set(c_stage)

        if c_grade:
            self.calib_grade_var.set(c_grade)
            if "GRADE A" in c_grade:
                grade_col = "#00E676"
            elif "GRADE B" in c_grade:
                grade_col = "#69F0AE"
            elif "GRADE C" in c_grade or "CERTIFIED" in c_grade:
                grade_col = "#00E5FF"
            elif "GRADE D" in c_grade:
                grade_col = "#FFD600"
            elif "GRADE E" in c_grade:
                grade_col = "#FF9100"
            else:
                grade_col = "#FF5252"
            self.lbl_calib_grade_badge.config(fg=grade_col)
        elif lut_is_active:
            self.calib_grade_var.set("128-PT LUT ACTIVE")
            self.lbl_calib_grade_badge.config(fg="#69F0AE")

        cards = {
            "kv_true": self.card_kv_true,
            "kv_r2": self.card_kv_r2,
            "kv_v0": self.card_kv_v0,
            "kv_ke": self.card_kv_ke,
            "kv_kt": self.card_kv_kt,
            "kv_sym": self.card_kv_sym,
            "kv_max_rpm": self.card_kv_max_rpm,
            "kv_code": self.card_kv_code,
            "lbl_r2_tag": self.lbl_kv_r2_tag
        }
        self.kv_analyzer.update_table_and_metrics(self.kv_tree, kv_pts, kv_res, kv_active, self.btn_run_kv, cards)
        self._update_symmetry_tab()
        self._update_harmonic_tab()
        draw_gauge(self.gauge_canvas, deg)
        draw_chart(self.chart_canvas, self.reader)
        draw_kv_scatter_plot(self.kv_canvas, kv_pts, kv_res)
        draw_symmetry_needle(self.sym_canvas, self.symmetry_equalizer.fwd_rpm, self.symmetry_equalizer.rev_rpm)

    def _update_symmetry_tab(self):
        with self.reader.lock:
            act_rpm = self.reader.last_actual_rpm
            target_uq = self.reader.last_uq

        self.card_sym_live_rpm.config(text=f"{act_rpm:+.1f} RPM")
        self.card_sym_live_uq.config(text=f"{target_uq:+.2f} V")

        fwd = self.symmetry_equalizer.fwd_rpm
        rev = self.symmetry_equalizer.rev_rpm
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

    def _update_harmonic_tab(self):
        with self.reader.lock:
            coeffs = dict(getattr(self.reader, 'harmonic_coeffs', {}))
            pts = list(getattr(self.reader, 'harmonic_bin_points', []))
            bench_res = dict(getattr(self.reader, 'bench_compare_results', {}))
            lut_mode = getattr(self.reader, 'last_lut_mode', -1)

        # Sync radio button if firmware reported mode
        if lut_mode in [0, 1, 2] and self.sensor_mode_var.get() != lut_mode:
            self.sensor_mode_var.set(lut_mode)

        # Update diagnostic cards
        if coeffs:
            a1 = coeffs.get("A1", 0.0); b1 = coeffs.get("B1", 0.0); h1 = coeffs.get("H1", math.hypot(a1, b1) * 57.2957795)
            a2 = coeffs.get("A2", 0.0); b2 = coeffs.get("B2", 0.0); h2 = coeffs.get("H2", math.hypot(a2, b2) * 57.2957795)
            a3 = coeffs.get("A3", 0.0); b3 = coeffs.get("B3", 0.0); h3 = coeffs.get("H3", math.hypot(a3, b3) * 57.2957795)
            a4 = coeffs.get("A4", 0.0); b4 = coeffs.get("B4", 0.0); h4 = coeffs.get("H4", math.hypot(a4, b4) * 57.2957795)

            phi1 = coeffs.get("Phi1", math.degrees(math.atan2(b1, a1)) % 360.0)
            phi2 = coeffs.get("Phi2", math.degrees(math.atan2(b2, a2)) % 360.0)
            phi3 = coeffs.get("Phi3", math.degrees(math.atan2(b3, a3)) % 360.0)
            phi4 = coeffs.get("Phi4", math.degrees(math.atan2(b4, a4)) % 360.0)

            r2 = coeffs.get("R2", 0.0)
            pk_pk = coeffs.get("PkPk", 0.0)
            rms = coeffs.get("RMS", 0.0)
            res_pk = coeffs.get("ResPkPk", 0.0)
            res_rms = coeffs.get("ResRMS", 0.0)
            lag_m = coeffs.get("LagMean", 0.0)
            lag_rip = coeffs.get("LagRipple", 0.0)
            lag_ang = coeffs.get("LagPeakAng", 0.0)

            apex1 = phi1
            apex2 = (phi2 * 0.5) % 360.0
            apex3 = (phi3 / 3.0) % 360.0
            apex4 = (phi4 * 0.25) % 360.0

            self.lbl_h1_val.config(text=f"±{h1:.4f}° mech")
            self.lbl_h1_desc.config(text=f"Apex at {apex1:.1f}° | A1={a1:+.4f}, B1={b1:+.4f}")
            self.lbl_h1_status.config(text=f"Runout: ±{(h1*1000):.1f} mdeg (φ={phi1:.1f}°)", fg="#69F0AE")

            self.lbl_h2_val.config(text=f"±{h2:.4f}° mech")
            self.lbl_h2_desc.config(text=f"Tilt Axis at {apex2:.1f}° & {(apex2+180)%360:.1f}°")
            self.lbl_h2_status.config(text=f"Tilt: ±{(h2*1000):.1f} mdeg (φ={phi2:.1f}°)", fg="#69F0AE")

            self.lbl_h3_val.config(text=f"±{h3:.4f}° mech")
            self.lbl_h3_desc.config(text=f"Pinch Apex at {apex3:.1f}° mech (every 120°)")
            if h1 > 0.02 and (h3 / h1) > 0.20:
                self.lbl_h3_status.config(text=f"⚠️ PINCH ALERT! ({h3/h1*100:.1f}% H1 at {apex3:.1f}°)", fg="#FF5252")
            else:
                pct = (h3 / h1 * 100.0) if h1 > 0.001 else 0.0
                self.lbl_h3_status.config(text=f"✅ CHASSIS PASS ({pct:.1f}% H1)", fg="#69F0AE")

            self.lbl_h4_val.config(text=f"±{h4:.4f}° mech")
            self.lbl_h4_desc.config(text=f"Mismatch Axis at {apex4:.1f}° (every 90°)")
            if h1 > 0.02 and (h4 / h1) > 0.25:
                self.lbl_h4_status.config(text=f"⚠️ FLUX SAT / BRIDGE! ({h4/h1*100:.1f}% H1)", fg="#FF5252")
            else:
                pct = (h4 / h1 * 100.0) if h1 > 0.001 else 0.0
                self.lbl_h4_status.config(text=f"✅ FLUX PASS ({pct:.1f}% H1)", fg="#69F0AE")

            # Secondary Quality & Dynamic Cards Update
            self.lbl_r2_val.config(text=f"{r2*100.0:.2f} %")
            if r2 >= 0.90:
                self.lbl_r2_status.config(text="✅ EXCELLENT (>90% fit)", fg="#69F0AE")
            elif r2 >= 0.75:
                self.lbl_r2_status.config(text="ℹ️ MODERATE (75-90%)", fg="#FFD600")
            else:
                self.lbl_r2_status.config(text="⚠️ LOW (<75% explained)", fg="#FF5252")

            reduction_pct = (1.0 - (res_pk / pk_pk)) * 100.0 if pk_pk > 1e-4 else 0.0
            self.lbl_res_val.config(text=f"±{res_pk*0.5:.4f}° mech")
            self.lbl_res_desc.config(text=f"RMS Residual: {res_rms:.4f}° mech")
            self.lbl_res_status.config(text=f"Ripple Reduction: {reduction_pct:.1f}%", fg="#69F0AE")

            self.lbl_lag_val.config(text=f"{lag_m:.4f}° mech")
            self.lbl_lag_status.config(text=f"Rotor Lag @ 38 RPM", fg="#00E5FF")

            self.lbl_fric_val.config(text=f"Δ{lag_rip:.4f}° mech")
            self.lbl_fric_desc.config(text=f"Peak Drag at {lag_ang:.1f}° mech")
            if lag_rip > 0.08:
                self.lbl_fric_status.config(text=f"⚠️ BINDING DETECTED ({lag_ang:.1f}°)", fg="#FF5252")
            else:
                self.lbl_fric_status.config(text="✅ UNIFORM ROTATION", fg="#69F0AE")

        # Redraw harmonic overlay canvas
        draw_harmonic_overlay_plot(self.harm_canvas, pts, coeffs)

        # Update Benchmark leaderboard table
        if bench_res:
            current_keys = set(bench_res.keys())
            existing_items = self.bench_tree.get_children()
            if len(existing_items) != len(current_keys):
                self.bench_tree.delete(*existing_items)
                mode_names = {
                    0: "Mode 0: Raw (Uncalibrated)",
                    1: "Mode 1: 128-PT Stepped LUT",
                    2: "Mode 2: Continuous Harmonics (Double-Angle)"
                }
                for m_id in sorted(current_keys):
                    row = bench_res[m_id]
                    self.bench_tree.insert("", "end", values=(
                        f"Mode {m_id}",
                        mode_names.get(m_id, f"Mode {m_id}"),
                        f"{row.get('latency_us', 0.0):.2f}",
                        f"{row.get('jitter_rad_s', 0.0):.3f}",
                        f"{row.get('mae_rad_s', 0.0):.3f}",
                        f"{row.get('delta_temp_c', 0.0):+.2f}",
                        f"{row.get('zero_angle_rad', 0.0):.5f}"
                    ))

