#!/usr/bin/env python3
"""
app.py
======
FOLKRACE — STM32G474 Dual Motor MT6816 20.0 kHz SimpleFOC Live GUI
Unified Dual-Motor Suite with 1-Button View Switching & Independent Calibration Persistence:
  • 1-Button Instant View Switching between Motor 1 & Motor 2
  • Separate, Non-Conflicting Calibration Profiles for Encoders 1 & 2:
    - Motor 1 / Enc 1: motor_config_m1.json & lut_calibration_m1.json
    - Motor 2 / Enc 2: motor_config_m2.json & lut_calibration_m2.json
    - Unified persistence in lut_calibrations.json
  • 1-Click Master Auto-Calibrator (Stepped 128-Point Dual-Pass LUT + 11-Pole Electrical Zero Alignment)
  • Closed-Loop Speed Control with DWT Velocity Window, Zero Interpolation & Sensor Lag Compensation
  • Live Real-Time PID / LPF / Voltage Ramp / Lag / Zero / KV Tuning + Flash & JSON Persistence
  • Dual-Stream Velocity Tracking Oscilloscope & Dual 14-Bit Angle Compass Dials
"""

import sys
import os
import json
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

# Add current directory to path
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from serial_comm import STM32DualSerial
from plots import MotorAngleDial, VelocityOscilloscopeChart

COLOR_BG = "#0E121A"
COLOR_CARD = "#151B27"
COLOR_CARD_SECONDARY = "#1B2332"
COLOR_ACCENT_CYAN = "#00E5FF"
COLOR_ACCENT_GREEN = "#00E676"
COLOR_ACCENT_AMBER = "#FFB300"
COLOR_ACCENT_RED = "#FF334B"
COLOR_TEXT = "#E2E8F0"
COLOR_TEXT_MUTED = "#8EA1C0"
COLOR_BORDER = "#253043"

# Non-conflicting calibration and config file paths
LUT_JSON_PATH = os.path.join(CURRENT_DIR, "lut_calibrations.json")
LUT_M1_JSON_PATH = os.path.join(CURRENT_DIR, "lut_calibration_m1.json")
LUT_M2_JSON_PATH = os.path.join(CURRENT_DIR, "lut_calibration_m2.json")

MOTOR_CONFIG_M1_JSON_PATH = os.path.abspath(os.path.join(CURRENT_DIR, "..", "motor_config_m1.json"))
MOTOR_CONFIG_M2_JSON_PATH = os.path.abspath(os.path.join(CURRENT_DIR, "..", "motor_config_m2.json"))
MOTOR_CONFIG_JSON_PATH = os.path.abspath(os.path.join(CURRENT_DIR, "..", "motor_config_gm3506.json"))

DEFAULT_MOTOR_CONFIG = {
    "pid_p": 0.150,
    "pid_i": 1.500,
    "pid_d": 0.0000,
    "lpf_tf": 0.5,
    "vramp": 100.0,
    "vlim": 12.00,
    "maxrpm": 2000.0,
    "lag": 35.0,
    "kv": 130.0,
    "dwt": 2.0,
    "zero": 0.0000,
    "align_v": 4.0,
    "corr_mode": 1,
    "sensor_direction": 1
}


class DualMotorDashboardApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("🏎️ FOLKRACE — STM32G474 MASTER LUT CALIBRATION & SPEED CONTROL SUITE (20.0 kHz FOC)")
        self.geometry("1420x930")
        self.minsize(1240, 840)
        self.configure(bg=COLOR_BG)

        # Active view: 1 = Motor 1 (TIM2), 2 = Motor 2 (TIM3)
        self.active_motor = 1

        # In-memory cached configs for Motor 1 and Motor 2
        self.motor_configs = {
            1: dict(DEFAULT_MOTOR_CONFIG),
            2: dict(DEFAULT_MOTOR_CONFIG)
        }

        # Serial Communication Engine
        self.serial_engine = STM32DualSerial(
            port="COM3",
            baud=921600,
            on_telemetry=self._on_telemetry,
            on_log=self._on_serial_log,
            on_config_json=self._on_config_json_received
        )

        self.cmd_history = []
        self.cmd_history_idx = -1
        self.log_paused = False
        self.is_master_calibrating = False
        self._pending_json_save_path = None
        self._pending_export_motor_id = 1
        self._calib_result_event = threading.Event()
        self._calib_last_success = False
        self._calib_last_reason = ""

        self._init_styles()
        self._build_ui()

        # Load initial calibrations from disk
        self._load_initial_calibrations_from_disk()

        # Keyboard Shortcuts
        self.bind("<Escape>", lambda e: self.global_estop())
        self.bind("<F2>", lambda e: self.toggle_motor_view())

        # Periodic UI update timer (25 Hz)
        self.after(40, self._ui_poll_loop)

    def _init_styles(self):
        style = ttk.Style(self)
        style.theme_use("clam")

        style.configure(".", background=COLOR_BG, foreground=COLOR_TEXT)
        style.configure("TLabel", background=COLOR_BG, foreground=COLOR_TEXT, font=("Segoe UI", 10))
        style.configure("Card.TFrame", background=COLOR_CARD)
        style.configure("CardSec.TFrame", background=COLOR_CARD_SECONDARY)

        style.configure("TCombobox", fieldbackground=COLOR_CARD_SECONDARY, background=COLOR_CARD_SECONDARY, foreground=COLOR_TEXT)
        style.map("TCombobox", fieldbackground=[("readonly", COLOR_CARD_SECONDARY)], foreground=[("readonly", COLOR_TEXT)])

        style.configure("TProgressbar", thickness=10, background=COLOR_ACCENT_CYAN, troughcolor=COLOR_CARD_SECONDARY)

    def _build_ui(self):
        # =====================================================================
        # 1. TOP HEADER & CONNECTION BAR
        # =====================================================================
        top_bar = tk.Frame(self, bg=COLOR_CARD, height=50, bd=1, relief=tk.SOLID, highlightbackground=COLOR_BORDER)
        top_bar.pack(fill=tk.X, side=tk.TOP, padx=8, pady=(6, 4))
        top_bar.pack_propagate(False)

        title_lbl = tk.Label(
            top_bar,
            text="🏎️ FOLKRACE | DUAL FOC",
            font=("Segoe UI", 11, "bold"),
            bg=COLOR_CARD,
            fg=COLOR_ACCENT_CYAN
        )
        title_lbl.pack(side=tk.LEFT, padx=(10, 6))

        conn_frame = tk.Frame(top_bar, bg=COLOR_CARD)
        conn_frame.pack(side=tk.LEFT, padx=6)

        tk.Label(conn_frame, text="Port:", bg=COLOR_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=(0, 3))
        self.port_combo = ttk.Combobox(conn_frame, width=10, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=2)
        self.refresh_ports()

        tk.Button(
            conn_frame,
            text="🔄",
            command=self.refresh_ports,
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_TEXT,
            activebackground=COLOR_BORDER,
            bd=0,
            padx=5,
            pady=2
        ).pack(side=tk.LEFT, padx=2)

        tk.Label(conn_frame, text="921.6k", bg=COLOR_CARD, fg=COLOR_ACCENT_GREEN, font=("Consolas", 9, "bold")).pack(side=tk.LEFT, padx=4)

        self.connect_btn = tk.Button(
            conn_frame,
            text="CONNECT",
            command=self.toggle_connection,
            bg="#00838F",
            fg="#FFFFFF",
            activebackground=COLOR_ACCENT_CYAN,
            font=("Segoe UI", 9, "bold"),
            bd=0,
            padx=10,
            pady=3
        )
        self.connect_btn.pack(side=tk.LEFT, padx=4)

        tk.Button(
            conn_frame,
            text="⚡ DFU",
            command=lambda: self.serial_engine.send_command("DFU"),
            bg="#37474F",
            fg="#ECEFF1",
            activebackground="#455A64",
            font=("Segoe UI", 8),
            bd=0,
            padx=6,
            pady=3
        ).pack(side=tk.LEFT, padx=3)

        # ---------------------------------------------------------------------
        # THE 1 VIEW SWITCH BUTTON (Motor 1 <-> Motor 2)
        # ---------------------------------------------------------------------
        self.view_btn = tk.Button(
            top_bar,
            text="🔁 ACTIVE VIEW: MOTOR 1  |  CLICK TO SWITCH ➔ MOTOR 2",
            command=self.toggle_motor_view,
            bg="#0284C7",
            fg="#FFFFFF",
            activebackground="#0369A1",
            font=("Segoe UI", 9, "bold"),
            bd=0,
            padx=14,
            pady=4,
            cursor="hand2"
        )
        self.view_btn.pack(side=tk.LEFT, padx=10)

        self.sys_status_lbl = tk.Label(
            top_bar,
            text="● STANDBY (DISCONNECTED)",
            font=("Segoe UI", 9, "bold"),
            bg=COLOR_CARD,
            fg=COLOR_TEXT_MUTED
        )
        self.sys_status_lbl.pack(side=tk.LEFT, padx=6)

        self.estop_btn = tk.Button(
            top_bar,
            text="🛑 GLOBAL E-STOP (Esc)",
            command=self.global_estop,
            bg=COLOR_ACCENT_RED,
            fg="#FFFFFF",
            activebackground="#D50000",
            font=("Segoe UI", 10, "bold"),
            bd=0,
            padx=16,
            pady=4,
            cursor="hand2"
        )
        self.estop_btn.pack(side=tk.RIGHT, padx=10)

        # =====================================================================
        # 2. MAIN WORKSPACE (Left: Active Master Calib & Speed | Right: Scope & Diag)
        # =====================================================================
        main_panes = tk.Frame(self, bg=COLOR_BG)
        main_panes.pack(fill=tk.BOTH, expand=True, padx=8, pady=2)

        self.primary_master_frame = self._build_primary_master_column(main_panes)
        self.primary_master_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 4))

        self.right_frame = self._build_right_telemetry_column(main_panes)
        self.right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False, padx=(4, 0))

        # =====================================================================
        # 3. BOTTOM INTERACTIVE SERIAL CONSOLE
        # =====================================================================
        bottom_frame = tk.Frame(self, bg=COLOR_CARD, height=140, bd=1, relief=tk.SOLID, highlightbackground=COLOR_BORDER)
        bottom_frame.pack(fill=tk.X, side=tk.BOTTOM, padx=8, pady=(2, 6))
        bottom_frame.pack_propagate(False)

        console_header = tk.Frame(bottom_frame, bg=COLOR_CARD)
        console_header.pack(fill=tk.X, padx=8, pady=(2, 1))

        tk.Label(
            console_header,
            text="INTERACTIVE SERIAL CONSOLE (26-Byte Binary Telemetry + UTF-8 Command Log)",
            font=("Segoe UI", 8, "bold"),
            bg=COLOR_CARD,
            fg=COLOR_TEXT_MUTED
        ).pack(side=tk.LEFT)

        self.pause_btn = tk.Button(console_header, text="Pause Log", command=self._toggle_log_pause, bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=0, padx=8, pady=1, font=("Segoe UI", 8))
        self.pause_btn.pack(side=tk.RIGHT, padx=4)

        tk.Button(console_header, text="Clear", command=self._clear_console, bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=0, padx=8, pady=1, font=("Segoe UI", 8)).pack(side=tk.RIGHT, padx=4)
        tk.Button(console_header, text="Print Status (s)", command=lambda: self.serial_engine.send_command("s"), bg=COLOR_CARD_SECONDARY, fg=COLOR_ACCENT_CYAN, bd=0, padx=8, pady=1, font=("Segoe UI", 8)).pack(side=tk.RIGHT, padx=4)

        self.console_text = tk.Text(
            bottom_frame,
            bg="#0B0E14",
            fg="#A0AEC0",
            font=("Consolas", 9),
            bd=0,
            highlightthickness=0,
            height=4
        )
        self.console_text.pack(fill=tk.BOTH, expand=True, padx=8, pady=2)

        cmd_bar = tk.Frame(bottom_frame, bg=COLOR_CARD)
        cmd_bar.pack(fill=tk.X, padx=8, pady=(1, 4))

        tk.Label(cmd_bar, text=">>>", bg=COLOR_CARD, fg=COLOR_ACCENT_CYAN, font=("Consolas", 10, "bold")).pack(side=tk.LEFT, padx=(0, 4))
        self.cmd_entry = tk.Entry(cmd_bar, bg="#0B0E14", fg=COLOR_TEXT, font=("Consolas", 10), insertbackground=COLOR_ACCENT_CYAN, bd=1, relief=tk.SOLID)
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.cmd_entry.bind("<Return>", self._send_console_cmd)
        self.cmd_entry.bind("<Up>", self._prev_cmd)
        self.cmd_entry.bind("<Down>", self._next_cmd)

        tk.Button(cmd_bar, text="Send", command=self._send_console_cmd, bg="#00838F", fg="#FFFFFF", bd=0, padx=12, pady=1, font=("Segoe UI", 9, "bold")).pack(side=tk.RIGHT, padx=2)

    # =========================================================================
    # LEFT COLUMN: ACTIVE MOTOR MASTER CALIBRATION & SPEED CONTROL
    # =========================================================================
    def _build_primary_master_column(self, parent):
        frame = tk.Frame(parent, bg=COLOR_CARD, bd=1, relief=tk.SOLID, highlightbackground=COLOR_BORDER)

        # 1. Column Header (Switches dynamically between Motor 1 and Motor 2)
        hdr = tk.Frame(frame, bg=COLOR_CARD_SECONDARY, height=34)
        hdr.pack(fill=tk.X, side=tk.TOP)
        hdr.pack_propagate(False)

        self.primary_col_hdr_lbl = tk.Label(
            hdr,
            text="MOTOR 1 — GM3506 FOC CONTROLLER (TIM2: PA5/PA1/PA2 | EN: PA3 | CS1: PA15)",
            font=("Segoe UI", 10, "bold"),
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_ACCENT_CYAN
        )
        self.primary_col_hdr_lbl.pack(side=tk.LEFT, padx=10)

        self.primary_status_badge = tk.Label(hdr, text="DISABLED", font=("Consolas", 9, "bold"), bg="#37474F", fg="#FFFFFF", padx=8, pady=2)
        self.primary_status_badge.pack(side=tk.RIGHT, padx=8)

        # 2. Top Dial & Real-Time Primary Telemetry Row
        dial_panel = tk.Frame(frame, bg=COLOR_CARD)
        dial_panel.pack(fill=tk.X, padx=8, pady=4)

        self.m1_dial = MotorAngleDial(dial_panel, size=185, title="M1", accent_color=COLOR_ACCENT_CYAN)
        self.m1_dial.pack(side=tk.LEFT, padx=4)

        metrics_box = tk.Frame(dial_panel, bg=COLOR_CARD_SECONDARY, bd=1, relief=tk.SOLID, highlightbackground=COLOR_BORDER)
        metrics_box.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=4, pady=2)

        spd_row = tk.Frame(metrics_box, bg=COLOR_CARD_SECONDARY)
        spd_row.pack(fill=tk.X, padx=10, pady=(6, 2))

        spd_left = tk.Frame(spd_row, bg=COLOR_CARD_SECONDARY)
        spd_left.pack(side=tk.LEFT)
        tk.Label(spd_left, text="ACTUAL VELOCITY (DWT WINDOW):", font=("Segoe UI", 8, "bold"), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT_MUTED).pack(anchor="w")
        self.m1_act_rpm_lbl = tk.Label(spd_left, text="+0.0 RPM", font=("Consolas", 20, "bold"), bg=COLOR_CARD_SECONDARY, fg=COLOR_ACCENT_GREEN)
        self.m1_act_rpm_lbl.pack(anchor="w")
        self.m1_tgt_rpm_lbl = tk.Label(spd_left, text="Target: +0.0 RPM", font=("Consolas", 9, "bold"), bg=COLOR_CARD_SECONDARY, fg=COLOR_ACCENT_CYAN)
        self.m1_tgt_rpm_lbl.pack(anchor="w")

        # Quick Enable / Stop / Disable Pillar
        quick_act = tk.Frame(spd_row, bg=COLOR_CARD_SECONDARY)
        quick_act.pack(side=tk.RIGHT, padx=4)
        self.btn_quick_enable = tk.Button(quick_act, text="⚡ ENABLE M1 (E1)", command=self._quick_enable_motor, bg="#1B5E20", fg="#FFFFFF", bd=0, padx=10, pady=3, font=("Segoe UI", 8, "bold"))
        self.btn_quick_enable.pack(fill=tk.X, pady=1)
        self.btn_quick_stop = tk.Button(quick_act, text="🛑 STOP M1 0 RPM (x)", command=self._quick_stop_motor, bg="#B71C1C", fg="#FFFFFF", bd=0, padx=10, pady=3, font=("Segoe UI", 8, "bold"))
        self.btn_quick_stop.pack(fill=tk.X, pady=1)
        self.btn_quick_disable = tk.Button(quick_act, text="DISABLE HI-Z (D1)", command=self._quick_disable_motor, bg="#37474F", fg="#FFFFFF", bd=0, padx=10, pady=2, font=("Segoe UI", 8))
        self.btn_quick_disable.pack(fill=tk.X, pady=1)

        volt_row = tk.Frame(metrics_box, bg=COLOR_CARD_SECONDARY)
        volt_row.pack(fill=tk.X, padx=10, pady=2)
        self.m1_uq_lbl = tk.Label(volt_row, text="Uq: +0.00 V", font=("Consolas", 9, "bold"), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT)
        self.m1_uq_lbl.pack(side=tk.LEFT, padx=(0, 12))
        self.m1_raw_lbl = tk.Label(volt_row, text="Raw14: 0", font=("Consolas", 9), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT_MUTED)
        self.m1_raw_lbl.pack(side=tk.LEFT, padx=(0, 12))
        self.m1_zero_lbl = tk.Label(volt_row, text="Zero: 0.0000 rad (CW)", font=("Consolas", 9), bg=COLOR_CARD_SECONDARY, fg=COLOR_ACCENT_AMBER)
        self.m1_zero_lbl.pack(side=tk.LEFT)

        badge_row = tk.Frame(metrics_box, bg=COLOR_CARD_SECONDARY)
        badge_row.pack(fill=tk.X, padx=10, pady=(4, 6))
        self.m1_badge_parity = tk.Label(badge_row, text="PARITY: OK", font=("Consolas", 8, "bold"), bg="#1B5E20", fg="#FFFFFF", padx=5)
        self.m1_badge_parity.pack(side=tk.LEFT, padx=2)
        self.m1_badge_mag = tk.Label(badge_row, text="MAG: OK", font=("Consolas", 8, "bold"), bg="#1B5E20", fg="#FFFFFF", padx=5)
        self.m1_badge_mag.pack(side=tk.LEFT, padx=2)
        self.m1_badge_lut = tk.Label(badge_row, text="LUT: OFF (RAW)", font=("Consolas", 8, "bold"), bg="#37474F", fg="#B0BEC5", padx=5)
        self.m1_badge_lut.pack(side=tk.LEFT, padx=2)
        self.m1_badge_foc = tk.Label(badge_row, text="20.0 kHz", font=("Consolas", 8, "bold"), bg="#004D40", fg=COLOR_ACCENT_CYAN, padx=5)
        self.m1_badge_foc.pack(side=tk.LEFT, padx=2)
        self.m1_badge_lat = tk.Label(badge_row, text="LAT: 12 µs", font=("Consolas", 8, "bold"), bg="#263238", fg="#ECEFF1", padx=5)
        self.m1_badge_lat.pack(side=tk.LEFT, padx=2)

        # =====================================================================
        # 3. HIGH-PRECISION MASTER AUTO-CALIBRATION HUB (STEPPED LUT + ZERO)
        # =====================================================================
        self.cal_sec = tk.LabelFrame(
            frame,
            text="🎯 HIGH-PRECISION MASTER AUTO-CALIBRATION: MOTOR 1 (128-PT STEPPED LUT + 11-POLE ZERO ALIGNMENT)",
            bg=COLOR_CARD,
            fg=COLOR_ACCENT_AMBER,
            font=("Segoe UI", 8, "bold")
        )
        self.cal_sec.pack(fill=tk.X, padx=8, pady=4)

        cal_top = tk.Frame(self.cal_sec, bg=COLOR_CARD)
        cal_top.pack(fill=tk.X, padx=6, pady=(3, 2))

        self.calib_status_lbl = tk.Label(
            cal_top,
            text="Status: Ready for Stepped 128-Point LUT + 11-Pole Zero Calibration",
            font=("Segoe UI", 8, "bold"),
            bg=COLOR_CARD,
            fg=COLOR_TEXT
        )
        self.calib_status_lbl.pack(side=tk.LEFT)

        self.calib_grade_badge = tk.Label(
            cal_top,
            text="NOT CALIBRATED",
            font=("Consolas", 8, "bold"),
            bg="#37474F",
            fg="#FFFFFF",
            padx=8,
            pady=1
        )
        self.calib_grade_badge.pack(side=tk.RIGHT)

        self.m1_lut_progress = ttk.Progressbar(self.cal_sec, orient=tk.HORIZONTAL, length=100, mode="determinate")
        self.m1_lut_progress.pack(fill=tk.X, padx=6, pady=2)

        # Calibration Options Row
        cal_opt_row = tk.Frame(self.cal_sec, bg=COLOR_CARD)
        cal_opt_row.pack(fill=tk.X, padx=6, pady=2)

        tk.Label(cal_opt_row, text="Align V:", bg=COLOR_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8)).pack(side=tk.LEFT)
        self.align_v_ent = tk.Entry(cal_opt_row, width=5, font=("Consolas", 8), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=1, relief=tk.SOLID)
        self.align_v_ent.insert(0, "4.0")
        self.align_v_ent.pack(side=tk.LEFT, padx=3)
        tk.Button(
            cal_opt_row,
            text="Set",
            command=self._set_align_voltage,
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_ACCENT_CYAN,
            bd=0,
            padx=5,
            font=("Segoe UI", 8)
        ).pack(side=tk.LEFT, padx=(0, 8))

        self.boost_var = tk.BooleanVar(value=True)
        tk.Checkbutton(
            cal_opt_row,
            text="Boost 4.2V on Stall",
            variable=self.boost_var,
            bg=COLOR_CARD,
            fg=COLOR_TEXT,
            selectcolor=COLOR_CARD_SECONDARY,
            activebackground=COLOR_CARD,
            font=("Segoe UI", 8)
        ).pack(side=tk.LEFT, padx=4)

        self.verify_spin_var = tk.BooleanVar(value=True)
        tk.Checkbutton(
            cal_opt_row,
            text="Verify Closed-Loop Spin",
            variable=self.verify_spin_var,
            bg=COLOR_CARD,
            fg=COLOR_TEXT,
            selectcolor=COLOR_CARD_SECONDARY,
            activebackground=COLOR_CARD,
            font=("Segoe UI", 8)
        ).pack(side=tk.LEFT, padx=4)

        # Correction Mode Selector (0: Raw vs 1: 128-PT LUT)
        self.corr_mode_var = tk.IntVar(value=1)
        mode_box = tk.Frame(cal_opt_row, bg=COLOR_CARD)
        mode_box.pack(side=tk.RIGHT)
        tk.Label(mode_box, text="Sensor Mode:", bg=COLOR_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=2)
        tk.Radiobutton(
            mode_box, text="0: Raw", variable=self.corr_mode_var, value=0,
            command=lambda: self._set_sensor_mode(0),
            bg=COLOR_CARD, fg=COLOR_TEXT, selectcolor=COLOR_CARD_SECONDARY, activebackground=COLOR_CARD, font=("Segoe UI", 8)
        ).pack(side=tk.LEFT)
        tk.Radiobutton(
            mode_box, text="1: 128-PT LUT", variable=self.corr_mode_var, value=1,
            command=lambda: self._set_sensor_mode(1),
            bg=COLOR_CARD, fg=COLOR_ACCENT_CYAN, selectcolor=COLOR_CARD_SECONDARY, activebackground=COLOR_CARD, font=("Segoe UI", 8, "bold")
        ).pack(side=tk.LEFT)

        # Prominent 1-Click Master Auto-Calibrate Button
        self.master_cal_btn = tk.Button(
            self.cal_sec,
            text="🚀 1-CLICK MASTER AUTO-CALIBRATE MOTOR 1 (128-PT STEPPED LUT + 11-POLE ZERO + AUTO-SAVE)",
            command=self.start_master_auto_calibration,
            bg="#D97706",
            fg="#FFFFFF",
            activebackground="#B45309",
            font=("Segoe UI", 9, "bold"),
            bd=0,
            pady=5,
            cursor="hand2"
        )
        self.master_cal_btn.pack(fill=tk.X, padx=6, pady=(3, 3))

        # Individual Stepped LUT & Alignment Sub-Action Buttons
        cal_sub_row = tk.Frame(self.cal_sec, bg=COLOR_CARD)
        cal_sub_row.pack(fill=tk.X, padx=6, pady=(1, 5))

        self.btn_sweep_lut = tk.Button(
            cal_sub_row,
            text="🔬 Stepped 128-Pt LUT + Align (w1)",
            command=self._calib_sweep_clicked,
            bg="#00838F",
            fg="#FFFFFF",
            bd=0,
            padx=6,
            pady=2,
            font=("Segoe UI", 8, "bold")
        )
        self.btn_sweep_lut.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.btn_zero_align = tk.Button(
            cal_sub_row,
            text="⚡ 11-Pole Zero Align Only (C1)",
            command=self._calib_zero_align_clicked,
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_TEXT,
            bd=0,
            padx=6,
            pady=2,
            font=("Segoe UI", 8)
        )
        self.btn_zero_align.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.btn_trim_lag = tk.Button(
            cal_sub_row,
            text="⚡ Trim Lag @ 1000 RPM (N1)",
            command=self._calib_trim_lag_clicked,
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_ACCENT_GREEN,
            bd=0,
            padx=6,
            pady=2,
            font=("Segoe UI", 8, "bold")
        )
        self.btn_trim_lag.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.btn_dump_lut = tk.Button(
            cal_sub_row,
            text="📋 Dump LUT Array (u1)",
            command=self._calib_dump_lut_clicked,
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_TEXT_MUTED,
            bd=0,
            padx=6,
            pady=2,
            font=("Segoe UI", 8)
        )
        self.btn_dump_lut.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # =====================================================================
        # 4. OVERALL SPEED CONTROL & DIRECT VOLTAGE TEST
        # =====================================================================
        self.spd_sec = tk.LabelFrame(
            frame,
            text="⚡ OVERALL SPEED CONTROL: MOTOR 1 (CLOSED-LOOP RPM) & DIRECT VOLTAGE TEST",
            bg=COLOR_CARD,
            fg=COLOR_ACCENT_CYAN,
            font=("Segoe UI", 8, "bold")
        )
        self.spd_sec.pack(fill=tk.X, padx=8, pady=4)

        slider_row = tk.Frame(self.spd_sec, bg=COLOR_CARD)
        slider_row.pack(fill=tk.X, padx=6, pady=2)

        tk.Label(slider_row, text="Speed Slider (-1800..+1800 RPM):", bg=COLOR_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT)
        self.coupled_slider = tk.Scale(
            slider_row,
            from_=-1800,
            to=1800,
            orient=tk.HORIZONTAL,
            bg=COLOR_CARD,
            fg=COLOR_TEXT,
            troughcolor=COLOR_CARD_SECONDARY,
            highlightthickness=0,
            command=self._on_coupled_slider
        )
        self.coupled_slider.set(0)
        self.coupled_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)

        # Speed Presets Row 1 (Forward + Stop + Reverse)
        preset_row = tk.Frame(self.spd_sec, bg=COLOR_CARD)
        preset_row.pack(fill=tk.X, padx=4, pady=2)
        for spd in [-1800, -1200, -800, -400, -200, -100, 0, 100, 200, 400, 800, 1200, 1800]:
            bg_c = "#B71C1C" if spd == 0 else COLOR_CARD_SECONDARY
            fg_c = "#FFFFFF" if spd == 0 else (COLOR_ACCENT_CYAN if spd > 0 else COLOR_TEXT_MUTED)
            tk.Button(
                preset_row,
                text=f"{spd}" if spd != 0 else "0 STOP",
                command=lambda s=spd: self._set_active_motor_speed(s),
                bg=bg_c,
                fg=fg_c,
                bd=0,
                padx=4,
                pady=2,
                font=("Consolas", 8, "bold" if spd == 0 else "normal")
            ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=1)

        # Fine Nudge & Direct Voltage Test Row
        nudge_row = tk.Frame(self.spd_sec, bg=COLOR_CARD)
        nudge_row.pack(fill=tk.X, padx=4, pady=(2, 5))

        tk.Label(nudge_row, text="Nudge RPM:", bg=COLOR_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=4)
        for delta in [-100, -25, -10, +10, +25, +100]:
            tk.Button(
                nudge_row,
                text=f"{delta:+d}",
                command=lambda d=delta: self._nudge_motor_speed(d),
                bg=COLOR_CARD_SECONDARY,
                fg=COLOR_TEXT,
                bd=0,
                padx=6,
                pady=1,
                font=("Consolas", 8)
            ).pack(side=tk.LEFT, padx=2)

        tk.Label(nudge_row, text="  |  Direct Torque Uq Test:", bg=COLOR_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=6)
        self.btn_torque_cw = tk.Button(nudge_row, text="+2.0V CW (v1)", command=lambda: self._send_direct_torque(+2.0), bg="#065F46", fg="#FFFFFF", bd=0, padx=8, pady=1, font=("Segoe UI", 8, "bold"))
        self.btn_torque_cw.pack(side=tk.LEFT, padx=2)
        self.btn_torque_ccw = tk.Button(nudge_row, text="-2.0V CCW (v1)", command=lambda: self._send_direct_torque(-2.0), bg="#065F46", fg="#FFFFFF", bd=0, padx=8, pady=1, font=("Segoe UI", 8, "bold"))
        self.btn_torque_ccw.pack(side=tk.LEFT, padx=2)
        self.btn_tare = tk.Button(nudge_row, text="Tare (Z1)", command=self._tare_active_motor, bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=0, padx=8, pady=1, font=("Segoe UI", 8))
        self.btn_tare.pack(side=tk.RIGHT, padx=4)

        # =====================================================================
        # 5. LIVE PARAMETER TUNING & FLASH / JSON PERSISTENCE
        # =====================================================================
        self.tune_sec = tk.LabelFrame(
            frame,
            text="🎛️ LIVE FOC PARAMETER TUNING: MOTOR 1 & FLASH / JSON PERSISTENCE (PRESS ENTER OR APPLY)",
            bg=COLOR_CARD,
            fg=COLOR_TEXT_MUTED,
            font=("Segoe UI", 8, "bold")
        )
        self.tune_sec.pack(fill=tk.X, padx=8, pady=4)

        t_row1 = tk.Frame(self.tune_sec, bg=COLOR_CARD)
        t_row1.pack(fill=tk.X, padx=4, pady=2)
        self.p_ent = self._make_tune_field(t_row1, "P:", "0.150", width=6, param_key="pid_p")
        self.i_ent = self._make_tune_field(t_row1, "I:", "1.500", width=6, param_key="pid_i")
        self.d_ent = self._make_tune_field(t_row1, "D:", "0.0000", width=6, param_key="pid_d")
        self.tf_ent = self._make_tune_field(t_row1, "LPF(ms):", "0.5", width=6, param_key="lpf_tf")
        self.vramp_ent = self._make_tune_field(t_row1, "V-Ramp(V/s):", "100.0", width=6, param_key="vramp")
        self.vlim_ent = self._make_tune_field(t_row1, "V-Lim(V):", "12.00", width=5, param_key="vlim")

        t_row2 = tk.Frame(self.tune_sec, bg=COLOR_CARD)
        t_row2.pack(fill=tk.X, padx=4, pady=2)
        self.maxrpm_ent = self._make_tune_field(t_row2, "Max RPM:", "2000", width=6, param_key="maxrpm")
        self.lag_ent = self._make_tune_field(t_row2, "Lag(µs):", "35.0", width=5, param_key="lag")
        self.kv_ent = self._make_tune_field(t_row2, "KV:", "130.0", width=5, param_key="kv")
        self.dwt_ent = self._make_tune_field(t_row2, "DWT(ms):", "2.0", width=5, param_key="dwt")
        self.zero_ent = self._make_tune_field(t_row2, "Zero(rad):", "0.0000", width=7, param_key="zero")

        tk.Button(t_row2, text="-0.002", command=lambda: self._nudge_zero(-0.002), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=0, padx=5, font=("Consolas", 8)).pack(side=tk.LEFT, padx=1)
        tk.Button(t_row2, text="+0.002", command=lambda: self._nudge_zero(+0.002), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=0, padx=5, font=("Consolas", 8)).pack(side=tk.LEFT, padx=1)

        tk.Button(
            t_row2,
            text="Apply All",
            command=self._apply_all_tuning,
            bg="#00838F",
            fg="#FFFFFF",
            bd=0,
            padx=10,
            pady=1,
            font=("Segoe UI", 8, "bold")
        ).pack(side=tk.RIGHT, padx=3)

        self.btn_defaults = tk.Button(
            t_row2,
            text="🔄 12V Defaults (R1)",
            command=self._reset_12v_defaults,
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_TEXT_MUTED,
            bd=0,
            padx=8,
            pady=1,
            font=("Segoe UI", 8)
        )
        self.btn_defaults.pack(side=tk.RIGHT, padx=3)

        # Flash & JSON Persistence Row
        persist_row = tk.Frame(self.tune_sec, bg=COLOR_CARD)
        persist_row.pack(fill=tk.X, padx=4, pady=(3, 5))

        tk.Button(
            persist_row,
            text="💾 Save STM32 Flash (SAVE)",
            command=lambda: self.serial_engine.send_command("SAVE"),
            bg="#065F46",
            fg="#FFFFFF",
            bd=0,
            padx=8,
            pady=2,
            font=("Segoe UI", 8, "bold")
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        tk.Button(
            persist_row,
            text="🔄 Load STM32 Flash (LOAD)",
            command=lambda: self.serial_engine.send_command("LOAD"),
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_ACCENT_CYAN,
            bd=0,
            padx=8,
            pady=2,
            font=("Segoe UI", 8)
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.btn_export_json = tk.Button(
            persist_row,
            text="💾 Export Motor 1 Config & LUT",
            command=self._export_config_to_json,
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_ACCENT_AMBER,
            bd=0,
            padx=8,
            pady=2,
            font=("Segoe UI", 8, "bold")
        )
        self.btn_export_json.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.btn_upload_json = tk.Button(
            persist_row,
            text="📂 Upload Motor 1 Config / LUT",
            command=self._upload_config_from_json,
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_TEXT,
            bd=0,
            padx=8,
            pady=2,
            font=("Segoe UI", 8)
        )
        self.btn_upload_json.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        return frame

    # =========================================================================
    # RIGHT COLUMN: VELOCITY OSCILLOSCOPE, SECONDARY ENCODER & HARDWARE PINOUT
    # =========================================================================
    def _build_right_telemetry_column(self, parent):
        frame = tk.Frame(parent, bg=COLOR_CARD, width=500, bd=1, relief=tk.SOLID, highlightbackground=COLOR_BORDER)
        frame.pack_propagate(False)

        # 1. Real-Time Velocity Tracking Oscilloscope Card
        scope_hdr = tk.Frame(frame, bg=COLOR_CARD_SECONDARY, height=32)
        scope_hdr.pack(fill=tk.X, side=tk.TOP)
        scope_hdr.pack_propagate(False)
        self.scope_hdr_lbl = tk.Label(
            scope_hdr,
            text="📈 REAL-TIME VELOCITY TRACKING OSCILLOSCOPE (MOTOR 1)",
            font=("Segoe UI", 9, "bold"),
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_ACCENT_CYAN
        )
        self.scope_hdr_lbl.pack(side=tk.LEFT, padx=10)

        self.scope_chart = VelocityOscilloscopeChart(frame, height=150)
        self.scope_chart.pack(fill=tk.X, padx=8, pady=6)

        # 2. Secondary Encoder Diagnostics Card
        hdr2 = tk.Frame(frame, bg=COLOR_CARD_SECONDARY, height=30)
        hdr2.pack(fill=tk.X)
        hdr2.pack_propagate(False)

        self.sec_hdr_lbl = tk.Label(
            hdr2,
            text="🧭 ENCODER 2 DIAGNOSTICS (TIM3 | SPI2: PB13/PB14/PB15 | CS2: PB11)",
            font=("Segoe UI", 9, "bold"),
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_ACCENT_GREEN
        )
        self.sec_hdr_lbl.pack(side=tk.LEFT, padx=10)

        self.m2_status_badge = tk.Label(hdr2, text="STANDBY", font=("Consolas", 8, "bold"), bg="#37474F", fg="#FFFFFF", padx=6, pady=1)
        self.m2_status_badge.pack(side=tk.RIGHT, padx=8)

        dial_panel = tk.Frame(frame, bg=COLOR_CARD)
        dial_panel.pack(fill=tk.X, padx=8, pady=4)

        self.m2_dial = MotorAngleDial(dial_panel, size=165, title="ENC2", accent_color=COLOR_ACCENT_GREEN)
        self.m2_dial.pack(side=tk.LEFT, padx=4)

        metrics_box = tk.Frame(dial_panel, bg=COLOR_CARD_SECONDARY, bd=1, relief=tk.SOLID, highlightbackground=COLOR_BORDER)
        metrics_box.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=4, pady=2)

        spd_row = tk.Frame(metrics_box, bg=COLOR_CARD_SECONDARY)
        spd_row.pack(fill=tk.X, padx=8, pady=(6, 2))
        tk.Label(spd_row, text="ALTERNATE SHAFT SPEED:", font=("Segoe UI", 8, "bold"), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT_MUTED).pack(anchor="w")
        self.m2_act_rpm_lbl = tk.Label(spd_row, text="+0.0 RPM", font=("Consolas", 16, "bold"), bg=COLOR_CARD_SECONDARY, fg=COLOR_ACCENT_GREEN)
        self.m2_act_rpm_lbl.pack(anchor="w")
        self.m2_tgt_rpm_lbl = tk.Label(spd_row, text="Target: +0.0 RPM", font=("Consolas", 8), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT_MUTED)
        self.m2_tgt_rpm_lbl.pack(anchor="w")

        volt_row = tk.Frame(metrics_box, bg=COLOR_CARD_SECONDARY)
        volt_row.pack(fill=tk.X, padx=8, pady=2)
        self.m2_uq_lbl = tk.Label(volt_row, text="Uq: +0.00 V", font=("Consolas", 8), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT)
        self.m2_uq_lbl.pack(side=tk.LEFT, padx=(0, 8))
        self.m2_raw_lbl = tk.Label(volt_row, text="Raw: 0", font=("Consolas", 8), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT_MUTED)
        self.m2_raw_lbl.pack(side=tk.LEFT)

        badge_row1 = tk.Frame(metrics_box, bg=COLOR_CARD_SECONDARY)
        badge_row1.pack(fill=tk.X, padx=8, pady=(4, 2))
        self.m2_badge_parity = tk.Label(badge_row1, text="PARITY: OK", font=("Consolas", 8, "bold"), bg="#1B5E20", fg="#FFFFFF", padx=4)
        self.m2_badge_parity.pack(side=tk.LEFT, padx=2)
        self.m2_badge_mag = tk.Label(badge_row1, text="MAG: OK", font=("Consolas", 8, "bold"), bg="#1B5E20", fg="#FFFFFF", padx=4)
        self.m2_badge_mag.pack(side=tk.LEFT, padx=2)
        self.m2_badge_lut = tk.Label(badge_row1, text="LUT: OFF", font=("Consolas", 8, "bold"), bg="#37474F", fg="#B0BEC5", padx=4)
        self.m2_badge_lut.pack(side=tk.LEFT, padx=2)

        badge_row2 = tk.Frame(metrics_box, bg=COLOR_CARD_SECONDARY)
        badge_row2.pack(fill=tk.X, padx=8, pady=(2, 6))
        self.m2_badge_foc = tk.Label(badge_row2, text="SPI2 MODE3", font=("Consolas", 8, "bold"), bg="#004D40", fg=COLOR_ACCENT_GREEN, padx=4)
        self.m2_badge_foc.pack(side=tk.LEFT, padx=2)
        self.m2_badge_lat = tk.Label(badge_row2, text="LAT: 12 µs", font=("Consolas", 8, "bold"), bg="#263238", fg="#ECEFF1", padx=4)
        self.m2_badge_lat.pack(side=tk.LEFT, padx=2)

        e_btn_row = tk.Frame(frame, bg=COLOR_CARD)
        e_btn_row.pack(fill=tk.X, padx=8, pady=2)
        self.sec_tare_btn = tk.Button(e_btn_row, text="Tare Enc 2 (Z2)", command=self._tare_secondary_motor, bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=0, padx=6, pady=2, font=("Segoe UI", 8))
        self.sec_tare_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(e_btn_row, text="SPI Bus Diag (ENC)", command=lambda: self.serial_engine.send_command("ENC"), bg=COLOR_CARD_SECONDARY, fg=COLOR_ACCENT_AMBER, bd=0, padx=6, pady=2, font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(e_btn_row, text="Toggle Watchdog (K)", command=lambda: self.serial_engine.send_command("K"), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=0, padx=6, pady=2, font=("Segoe UI", 8)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # 3. STM32 Active Pinout Documentation Card
        pin_sec = tk.LabelFrame(frame, text="HARDWARE ACTIVE PINOUT & ARCHITECTURE", bg=COLOR_CARD, fg=COLOR_ACCENT_AMBER, font=("Segoe UI", 8, "bold"))
        pin_sec.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        pin_text = tk.Text(
            pin_sec,
            bg="#0B0E14",
            fg="#CBD5E1",
            font=("Consolas", 8),
            bd=0,
            padx=8,
            pady=6,
            highlightthickness=0
        )
        pin_text.pack(fill=tk.BOTH, expand=True)

        pinout_info = (
            "STM32G474 @ 170MHz DUAL-MOTOR ACTIVE CONFIGURATION\n"
            "======================================================\n"
            "Motor 1 Channel (TIM2 Center-Aligned PWM @ 25 kHz):\n"
            "  • Phase A / B / C : PA5 / PA1 / PA2\n"
            "  • Driver Enable   : PA3 (Active-High Gate Driver)\n"
            "  • MT6816 Enc 1    : SPI1 (PB3 SCK, PB4 MISO, PB5 MOSI)\n"
            "  • Enc 1 CS        : PA15 (5.31 MHz SPI Mode 3)\n\n"
            "Motor 2 Channel (TIM3 Center-Aligned PWM @ 25 kHz):\n"
            "  • Phase A / B / C : PA6 / PA4 / PB0\n"
            "  • Driver Enable   : PA7 (Active-High Gate Driver)\n"
            "  • MT6816 Enc 2    : SPI2 (PB13 SCK, PB14 MISO, PB15 MOSI)\n"
            "  • Enc 2 CS        : PB11 (5.31 MHz SPI Mode 3)\n\n"
            "Hard Real-Time SimpleFOC Core:\n"
            "  • Hard ISR        : TIM6 @ 20.0 kHz (50 µs tick)\n"
            "  • Velocity Window : DWT 2.0 ms Cycle Integration\n"
            "  • Commutation     : 128-Pt LUT + 11-Pole Zero Interp\n"
            "  • Speed Comp      : Dynamic Sensor Lag (35 µs)\n"
        )
        pin_text.insert(tk.END, pinout_info)
        pin_text.configure(state="disabled")

        return frame

    def _make_tune_field(self, parent, label_text, default_val, width=6, param_key=None):
        box = tk.Frame(parent, bg=COLOR_CARD)
        box.pack(side=tk.LEFT, padx=3)
        tk.Label(box, text=label_text, font=("Segoe UI", 8), bg=COLOR_CARD, fg=COLOR_TEXT_MUTED).pack(side=tk.LEFT)
        ent = tk.Entry(box, width=width, font=("Consolas", 8), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=1, relief=tk.SOLID, insertbackground=COLOR_ACCENT_CYAN)
        ent.insert(0, default_val)
        ent.pack(side=tk.LEFT, padx=1)
        if param_key:
            ent.bind("<Return>", lambda e, k=param_key, w=ent: self._send_param_tune(k, w.get()))
        return ent

    # =========================================================================
    # VIEW SWITCHING ENGINE (1-BUTTON TOGGLE)
    # =========================================================================
    def toggle_motor_view(self):
        """Switches the active view between Motor 1 and Motor 2 with 1 button."""
        if self.is_master_calibrating:
            messagebox.showinfo("Busy", "Cannot switch views while calibration is actively in progress.")
            return

        # 1. Save current GUI entry values to in-memory config for currently active motor
        self._save_gui_to_motor_config(self.active_motor)

        # 2. Toggle active motor: 1 <-> 2
        self.active_motor = 2 if self.active_motor == 1 else 1
        m = self.active_motor
        other_m = 2 if m == 1 else 1

        accent = COLOR_ACCENT_CYAN if m == 1 else COLOR_ACCENT_GREEN
        other_accent = COLOR_ACCENT_GREEN if m == 1 else COLOR_ACCENT_CYAN

        # 3. Update view switch button
        if m == 1:
            self.view_btn.configure(
                text="🔁 ACTIVE VIEW: MOTOR 1  |  CLICK TO SWITCH ➔ MOTOR 2",
                bg="#0284C7",
                activebackground="#0369A1"
            )
        else:
            self.view_btn.configure(
                text="🔁 ACTIVE VIEW: MOTOR 2  |  CLICK TO SWITCH ➔ MOTOR 1",
                bg="#059669",
                activebackground="#047857"
            )

        # 4. Update Left Column (Primary Master Control)
        if m == 1:
            self.primary_col_hdr_lbl.configure(
                text="MOTOR 1 — GM3506 FOC CONTROLLER (TIM2: PA5/PA1/PA2 | EN: PA3 | CS1: PA15)",
                fg=COLOR_ACCENT_CYAN
            )
        else:
            self.primary_col_hdr_lbl.configure(
                text="MOTOR 2 — GM3506 FOC CONTROLLER (TIM3: PA6/PA4/PB0 | EN: PA7 | CS2: PB11)",
                fg=COLOR_ACCENT_GREEN
            )

        self.m1_dial.title = f"M{m}"
        self.m1_dial.accent_color = accent
        self.m1_dial.redraw()

        self.btn_quick_enable.configure(text=f"⚡ ENABLE M{m} (E{m})")
        self.btn_quick_stop.configure(text=f"🛑 STOP M{m} 0 RPM (x)")
        self.btn_quick_disable.configure(text=f"DISABLE HI-Z (D{m})")

        # Update Calibration Hub
        self.cal_sec.configure(
            text=f"🎯 MASTER AUTO-CALIBRATION: MOTOR {m} (128-PT STEPPED LUT + 11-POLE ZERO ALIGNMENT)",
            fg=COLOR_ACCENT_AMBER
        )
        self.master_cal_btn.configure(
            text=f"🚀 1-CLICK MASTER AUTO-CALIBRATE MOTOR {m} (128-PT STEPPED LUT + 11-POLE ZERO + AUTO-SAVE)",
            bg="#D97706" if m == 1 else "#0D9488"
        )
        self.btn_sweep_lut.configure(text=f"🔬 Stepped 128-Pt LUT + Align (w{m})")
        self.btn_zero_align.configure(text=f"⚡ 11-Pole Zero Align Only (C{m})")
        self.btn_trim_lag.configure(text=f"⚡ Trim Lag @ 1000 RPM (N{m})")
        self.btn_dump_lut.configure(text=f"📋 Dump LUT Array (u{m})")

        # Update Speed Control & Direct Torque
        self.spd_sec.configure(
            text=f"⚡ OVERALL SPEED CONTROL: MOTOR {m} (CLOSED-LOOP RPM) & DIRECT VOLTAGE TEST",
            fg=accent
        )
        self.btn_torque_cw.configure(text=f"+2.0V CW (v{m})")
        self.btn_torque_ccw.configure(text=f"-2.0V CCW (v{m})")
        self.btn_tare.configure(text=f"Tare (Z{m})")

        cur_tgt = self.serial_engine.state.get(f"m{m}_target_rpm", 0.0)
        self.coupled_slider.set(int(round(cur_tgt)))

        # Update Live Tuning & Persistence
        self.tune_sec.configure(
            text=f"🎛️ LIVE FOC PARAMETER TUNING: MOTOR {m} & FLASH / JSON PERSISTENCE (PRESS ENTER OR APPLY)"
        )
        self.btn_defaults.configure(text=f"🔄 12V Defaults (R{m})")
        self.btn_export_json.configure(text=f"💾 Export Motor {m} Config & LUT")
        self.btn_upload_json.configure(text=f"📂 Upload Motor {m} Config / LUT")

        # 5. Update Right Column (Oscilloscope & Alternate Diagnostics)
        self.scope_hdr_lbl.configure(
            text=f"📈 REAL-TIME VELOCITY TRACKING OSCILLOSCOPE (MOTOR {m})",
            fg=accent
        )

        if other_m == 2:
            self.sec_hdr_lbl.configure(
                text="🧭 ENCODER 2 DIAGNOSTICS (TIM3 | SPI2: PB13/PB14/PB15 | CS2: PB11)",
                fg=COLOR_ACCENT_GREEN
            )
            self.m2_badge_foc.configure(text="SPI2 MODE3", fg=COLOR_ACCENT_GREEN)
        else:
            self.sec_hdr_lbl.configure(
                text="🧭 ENCODER 1 DIAGNOSTICS (TIM2 | SPI1: PB3/PB4/PB5 | CS1: PA15)",
                fg=COLOR_ACCENT_CYAN
            )
            self.m2_badge_foc.configure(text="SPI1 MODE3", fg=COLOR_ACCENT_CYAN)

        self.m2_dial.title = f"ENC{other_m}"
        self.m2_dial.accent_color = other_accent
        self.m2_dial.redraw()

        self.sec_tare_btn.configure(text=f"Tare Enc {other_m} (Z{other_m})")

        # 6. Load target motor config into GUI entries
        self._load_gui_from_motor_config(m)

        self._log_console(f"[GUI] Active View switched to MOTOR {m} (Encoder {m}). Diagnostics showing Motor {other_m}.\n")

    def _save_gui_to_motor_config(self, motor_id):
        """Caches GUI values into memory for specified motor."""
        cfg = self.motor_configs.get(motor_id, {})
        try:
            if hasattr(self, "p_ent"): cfg['pid_p'] = float(self.p_ent.get())
            if hasattr(self, "i_ent"): cfg['pid_i'] = float(self.i_ent.get())
            if hasattr(self, "d_ent"): cfg['pid_d'] = float(self.d_ent.get())
            if hasattr(self, "tf_ent"):
                tf_v = float(self.tf_ent.get())
                cfg['lpf_tf'] = tf_v * 0.001 if tf_v > 0.02 else tf_v
            if hasattr(self, "vramp_ent"): cfg['vramp'] = float(self.vramp_ent.get())
            if hasattr(self, "vlim_ent"): cfg['vlim'] = float(self.vlim_ent.get())
            if hasattr(self, "maxrpm_ent"): cfg['maxrpm'] = float(self.maxrpm_ent.get())
            if hasattr(self, "lag_ent"): cfg['lag'] = float(self.lag_ent.get())
            if hasattr(self, "kv_ent"): cfg['kv'] = float(self.kv_ent.get())
            if hasattr(self, "dwt_ent"): cfg['dwt'] = float(self.dwt_ent.get())
            if hasattr(self, "zero_ent"): cfg['zero'] = float(self.zero_ent.get())
            if hasattr(self, "align_v_ent"): cfg['align_v'] = float(self.align_v_ent.get())
            if hasattr(self, "corr_mode_var"): cfg['corr_mode'] = int(self.corr_mode_var.get())
        except Exception:
            pass

    def _load_gui_from_motor_config(self, motor_id):
        """Populates GUI entries from memory for specified motor."""
        cfg = self.motor_configs.get(motor_id, {})

        def _set(ent_name, val_str):
            if hasattr(self, ent_name):
                w = getattr(self, ent_name)
                w.delete(0, tk.END)
                w.insert(0, val_str)

        _set("p_ent", f"{float(cfg.get('pid_p', 0.15)):.4f}")
        _set("i_ent", f"{float(cfg.get('pid_i', 1.5)):.4f}")
        _set("d_ent", f"{float(cfg.get('pid_d', 0.0)):.4f}")

        raw_tf = float(cfg.get('lpf_tf', 0.5))
        if raw_tf >= 20.0:
            tf_ms = raw_tf * 0.001
        elif raw_tf < 0.02:
            tf_ms = raw_tf * 1000.0
        else:
            tf_ms = raw_tf
        _set("tf_ent", f"{tf_ms:.3f}".rstrip('0').rstrip('.'))

        _set("vramp_ent", f"{float(cfg.get('vramp', 100.0)):.1f}")
        _set("vlim_ent", f"{float(cfg.get('vlim', 12.00)):.2f}")
        _set("maxrpm_ent", f"{int(round(float(cfg.get('maxrpm', 2000))))}")
        _set("lag_ent", f"{float(cfg.get('lag', 35.0)):.1f}")
        _set("kv_ent", f"{float(cfg.get('kv', 130.0)):.1f}")
        _set("dwt_ent", f"{float(cfg.get('dwt', 2.0)):.1f}")
        _set("zero_ent", f"{float(cfg.get('zero', 0.0)):.4f}")
        _set("align_v_ent", f"{float(cfg.get('align_v', 4.0)):.1f}")

        if hasattr(self, "corr_mode_var"):
            self.corr_mode_var.set(int(cfg.get('corr_mode', 1)))

    def _send_param_tune(self, param_key, val_str):
        try:
            v = float(val_str)
            m = self.active_motor
            cmd_map = {
                "pid_p": f"P{m} {v}",
                "pid_i": f"I{m} {v}",
                "pid_d": f"D{m} {v}",
                "lpf_tf": f"F{m} {v}",
                "vramp": f"RAMP{m} {v}",
                "vlim": f"V{m} {v}",
                "maxrpm": f"MAXRPM{m} {v}",
                "lag": f"LAG{m} {v}",
                "kv": f"KV{m} {v}",
                "dwt": f"DWT{m} {v}",
                "zero": f"O{m} {v:.4f}"
            }
            if param_key in cmd_map:
                self.serial_engine.send_command(cmd_map[param_key])
                self.motor_configs[m][param_key] = v
        except ValueError:
            pass

    # =========================================================================
    # QUICK ACTION & CONTROL HANDLERS
    # =========================================================================
    def _quick_enable_motor(self):
        self.serial_engine.send_command(f"E{self.active_motor}")

    def _quick_stop_motor(self):
        self.coupled_slider.set(0)
        self.serial_engine.send_command(f"T{self.active_motor} 0")

    def _quick_disable_motor(self):
        self.serial_engine.send_command(f"D{self.active_motor}")

    def _tare_active_motor(self):
        self.serial_engine.send_command(f"Z{self.active_motor}")

    def _tare_secondary_motor(self):
        other_m = 2 if self.active_motor == 1 else 1
        self.serial_engine.send_command(f"Z{other_m}")

    def _send_direct_torque(self, v):
        self.serial_engine.send_command(f"v{self.active_motor} {v:.1f}")

    def _set_active_motor_speed(self, spd):
        self.coupled_slider.set(spd)
        self.serial_engine.send_command(f"T{self.active_motor} {spd}")

    def _nudge_motor_speed(self, delta_rpm):
        cur = self.serial_engine.state.get(f"m{self.active_motor}_target_rpm", 0.0)
        new_spd = max(-1800.0, min(1800.0, cur + delta_rpm))
        self.coupled_slider.set(int(round(new_spd)))
        self.serial_engine.send_command(f"T{self.active_motor} {new_spd:.1f}")

    def _on_coupled_slider(self, val):
        self.serial_engine.send_command(f"T{self.active_motor} {val}")

    def _set_align_voltage(self):
        try:
            v = float(self.align_v_ent.get())
            self.motor_configs[self.active_motor]["align_v"] = v
            self.serial_engine.send_command(f"A{self.active_motor} {v:.2f}")
        except ValueError:
            messagebox.showerror("Format Error", "Align voltage must be a valid number.")

    def _set_sensor_mode(self, mode):
        self.corr_mode_var.set(mode)
        self.motor_configs[self.active_motor]["corr_mode"] = mode
        self.serial_engine.send_command(f"LUT_MODE {self.active_motor} {mode}")

    def _nudge_zero(self, delta_rad):
        try:
            cur = float(self.zero_ent.get())
        except ValueError:
            cur = self.serial_engine.state.get(f"m{self.active_motor}_zero_angle", 0.0)

        new_z = (cur + delta_rad) % 6.2831853
        if new_z < 0:
            new_z += 6.2831853
        self.zero_ent.delete(0, tk.END)
        self.zero_ent.insert(0, f"{new_z:.4f}")
        self.motor_configs[self.active_motor]["zero"] = new_z
        self.serial_engine.send_command(f"O{self.active_motor} {new_z:.4f}")

    def _apply_all_tuning(self):
        try:
            m = self.active_motor
            p = float(self.p_ent.get())
            i = float(self.i_ent.get())
            d = float(self.d_ent.get())
            tf = float(self.tf_ent.get())
            vramp = float(self.vramp_ent.get())
            vlim = float(self.vlim_ent.get())
            maxrpm = float(self.maxrpm_ent.get())
            lag = float(self.lag_ent.get())
            kv = float(self.kv_ent.get())
            dwt = float(self.dwt_ent.get())

            self.serial_engine.send_command(f"P{m} {p}")
            self.serial_engine.send_command(f"I{m} {i}")
            self.serial_engine.send_command(f"D{m} {d}")
            self.serial_engine.send_command(f"F{m} {tf}")
            self.serial_engine.send_command(f"RAMP{m} {vramp}")
            self.serial_engine.send_command(f"V{m} {vlim}")
            self.serial_engine.send_command(f"MAXRPM{m} {maxrpm}")
            self.serial_engine.send_command(f"LAG{m} {lag}")
            self.serial_engine.send_command(f"KV{m} {kv}")
            self.serial_engine.send_command(f"DWT{m} {dwt}")

            self._save_gui_to_motor_config(m)
            self._log_console(f"[SYS] Applied all tuning parameters for Motor {m}.\n")
        except ValueError:
            messagebox.showerror("Format Error", "All tuning parameters must be valid numbers.")

    def _reset_12v_defaults(self):
        m = self.active_motor
        self.serial_engine.send_command(f"R{m}")
        self.motor_configs[m] = dict(DEFAULT_MOTOR_CONFIG)
        self._load_gui_from_motor_config(m)
        self._log_console(f"[SYS] Reset Motor {m} to default 12V parameters.\n")

    # =========================================================================
    # CALIBRATION SUB-ACTIONS
    # =========================================================================
    def _calib_sweep_clicked(self):
        self.serial_engine.send_command(f"CALIB {self.active_motor}")

    def _calib_zero_align_clicked(self):
        self.serial_engine.send_command(f"C{self.active_motor}")

    def _calib_trim_lag_clicked(self):
        self.serial_engine.send_command(f"N{self.active_motor}")

    def _calib_dump_lut_clicked(self):
        self.serial_engine.send_command(f"DUMP_LUT {self.active_motor}")

    # =========================================================================
    # 1-CLICK MASTER AUTO-CALIBRATION ENGINE
    # =========================================================================
    def start_master_auto_calibration(self):
        if not self.serial_engine.state["connected"]:
            messagebox.showwarning("Not Connected", "Please connect to the STM32 COM port first.")
            return
        if self.is_master_calibrating or self.serial_engine.state["m1_calib"] or self.serial_engine.state["m2_calib"]:
            messagebox.showinfo("Busy", "Calibration is already running.")
            return

        self.is_master_calibrating = True
        self.view_btn.configure(state="disabled")
        self.master_cal_btn.configure(
            text=f"⏳ MASTER AUTO-CALIBRATION IN PROGRESS FOR MOTOR {self.active_motor}...",
            bg="#B45309",
            state="disabled"
        )
        threading.Thread(target=self._master_calibrate_worker, args=(self.active_motor,), daemon=True).start()

    def _master_calibrate_worker(self, motor_id):
        try:
            st = self.serial_engine.state
            m = motor_id

            # Step 0: Sensor Health Verification
            self._set_calib_ui(f"Step 0/4: Checking MT6816 SPI Sensor {m} Health...", "CHECKING SENSOR", "#0284C7")
            self.serial_engine.send_command("ENC")
            time.sleep(0.4)
            if not st[f"m{m}_parity_ok"] or not st[f"m{m}_mag_ok"]:
                self._set_calib_ui(f"❌ Aborted: Encoder {m} SPI Parity or Magnet error!", "SENSOR FAULT", COLOR_ACCENT_RED)
                return

            # Step 1: Configure Align Voltage & Run Stepped 128-Point Sweep
            try:
                align_v = float(self.align_v_ent.get())
            except ValueError:
                align_v = 4.0

            self.serial_engine.send_command(f"A{m} {align_v:.2f}")
            time.sleep(0.15)

            self._calib_result_event.clear()
            self._calib_last_success = False
            self._calib_last_reason = ""

            self._set_calib_ui(
                f"Step 1/4: Running 128-Pt Stepped LUT + Zero Sweep for Motor {m} @ {align_v:.1f}V...",
                f"SWEEPING M{m}...",
                "#D97706"
            )
            self.serial_engine.send_command(f"CALIB {m}")

            # Wait up to 95 seconds for sweep + cooldown + zero alignment
            t0 = time.time()
            while time.time() - t0 < 95.0:
                if not st["connected"]:
                    self._set_calib_ui("❌ Disconnected during calibration.", "DISCONNECTED", COLOR_ACCENT_RED)
                    return
                if self._calib_result_event.wait(timeout=0.25):
                    break

            # Boost Retry if stalled
            if not self._calib_last_success and self.boost_var.get() and align_v < 4.2:
                boost_v = 4.2
                self._set_calib_ui(
                    f"⚠️ Stall detected ({self._calib_last_reason}) — Retrying Motor {m} at Boost {boost_v:.1f}V...",
                    "BOOST RETRY",
                    "#D97706"
                )
                self.serial_engine.send_command(f"A{m} {boost_v:.2f}")
                time.sleep(0.2)
                self._calib_result_event.clear()
                self.serial_engine.send_command(f"CALIB {m}")

                t1 = time.time()
                while time.time() - t1 < 95.0:
                    if not st["connected"]:
                        return
                    if self._calib_result_event.wait(timeout=0.25):
                        break

            if not self._calib_last_success:
                self._set_calib_ui(
                    f"❌ Calibration Failed: {self._calib_last_reason or 'Timeout'}",
                    "CALIB FAILED",
                    COLOR_ACCENT_RED
                )
                return

            # Step 2: Ensure 128-Point LUT mode is active
            self.corr_mode_var.set(1)
            self.motor_configs[m]["corr_mode"] = 1
            self.serial_engine.send_command(f"LUT_MODE {m} 1")
            self.serial_engine.send_command(f"M{m} 1")
            time.sleep(0.2)

            # Step 3: Optional Closed-Loop Verification Spin
            if self.verify_spin_var.get():
                self._set_calib_ui(f"Step 3/4: Verifying Closed-Loop Spin for Motor {m} (±120 RPM)...", "VERIFYING SPIN", "#0284C7")
                self.serial_engine.send_command(f"E{m}")
                time.sleep(0.2)
                self.serial_engine.send_command(f"T{m} 120")
                time.sleep(1.6)
                cw_rpm = abs(st[f"m{m}_actual_rpm"])

                self.serial_engine.send_command(f"T{m} -120")
                time.sleep(1.8)
                ccw_rpm = abs(st[f"m{m}_actual_rpm"])

                self.serial_engine.send_command(f"T{m} 0")
                time.sleep(0.4)
                self._log_console(f"[VERIFY M{m}] Spin check: CW={cw_rpm:.1f} RPM, CCW={ccw_rpm:.1f} RPM\n")

                if cw_rpm < 40.0 or ccw_rpm < 40.0:
                    self._set_calib_ui(
                        f"❌ Spin Check Failed: CW={cw_rpm:.1f}, CCW={ccw_rpm:.1f} RPM. Stalled!",
                        "SPIN FAILED",
                        COLOR_ACCENT_RED
                    )
                    return

            # Step 4: Save to STM32 Flash & Persist to Non-Conflicting JSON
            self._set_calib_ui(f"Step 4/4: Saving Motor {m} LUT & Zeros to STM32 Flash & JSON...", "SAVING...", COLOR_ACCENT_GREEN)
            self.serial_engine.send_command("SAVE")
            time.sleep(0.25)
            self._export_config_to_json(motor_id=m, silent=True)
            time.sleep(0.4)

            grade = st.get(f"m{m}_calib_grade", "128-PT LUT ACTIVE")
            self._set_calib_ui(
                f"✅ Motor {m} Calibration Complete! Zero={st[f'm{m}_zero_angle']:.4f} rad ({st[f'm{m}_direction']}) | 128-Pt LUT Saved.",
                grade,
                "#059669"
            )

        finally:
            self.is_master_calibrating = False
            self.after(0, lambda: self.view_btn.configure(state="normal"))
            self.after(0, lambda: self.master_cal_btn.configure(
                text=f"🚀 1-CLICK MASTER AUTO-CALIBRATE MOTOR {self.active_motor} (128-PT STEPPED LUT + 11-POLE ZERO + AUTO-SAVE)",
                bg="#D97706" if self.active_motor == 1 else "#0D9488",
                state="normal"
            ))

    def _set_calib_ui(self, status_text, badge_text, badge_color):
        def _upd():
            self.calib_status_lbl.configure(text=status_text)
            self.calib_grade_badge.configure(text=badge_text, bg=badge_color)
        self.after(0, _upd)

    # =========================================================================
    # DUAL-ENCODER NON-CONFLICTING CALIBRATION FILE MANAGEMENT
    # =========================================================================
    def _load_initial_calibrations_from_disk(self):
        """Loads non-conflicting saved calibration files from disk at startup."""
        # 1. Check Motor 1 config file
        m1_file = MOTOR_CONFIG_M1_JSON_PATH if os.path.exists(MOTOR_CONFIG_M1_JSON_PATH) else MOTOR_CONFIG_JSON_PATH
        if os.path.exists(m1_file):
            try:
                with open(m1_file, "r") as f:
                    data = json.load(f)
                m1_data = data.get("motor1", data) if isinstance(data.get("motor1"), dict) else data
                self._update_motor_config_cache(1, m1_data)
                # If legacy file has m2 data
                if "m2_zero_electric_angle" in data:
                    self._update_motor_config_cache(2, {
                        "zero_electric_angle": data["m2_zero_electric_angle"],
                        "sensor_direction": data.get("m2_sensor_direction", 1)
                    })
            except Exception:
                pass

        # 2. Check Motor 2 dedicated config file
        if os.path.exists(MOTOR_CONFIG_M2_JSON_PATH):
            try:
                with open(MOTOR_CONFIG_M2_JSON_PATH, "r") as f:
                    data = json.load(f)
                m2_data = data.get("motor2", data) if isinstance(data.get("motor2"), dict) else data
                self._update_motor_config_cache(2, m2_data)
            except Exception:
                pass

        # 3. Check LUT files
        if os.path.exists(LUT_JSON_PATH):
            try:
                with open(LUT_JSON_PATH, "r") as f:
                    lut_all = json.load(f)
                if "motor1" in lut_all:
                    self._update_motor_config_cache(1, lut_all["motor1"])
                if "motor2" in lut_all:
                    self._update_motor_config_cache(2, lut_all["motor2"])
            except Exception:
                pass

        # Populate GUI entries with active motor's values
        self._load_gui_from_motor_config(self.active_motor)

    def _update_motor_config_cache(self, motor_id, cfg):
        target = self.motor_configs[motor_id]
        if "pid_p" in cfg: target["pid_p"] = float(cfg["pid_p"])
        if "pid_i" in cfg: target["pid_i"] = float(cfg["pid_i"])
        if "pid_d" in cfg: target["pid_d"] = float(cfg["pid_d"])
        if "lpf_tf" in cfg: target["lpf_tf"] = float(cfg["lpf_tf"])
        if "motor_kv_rating" in cfg: target["kv"] = float(cfg["motor_kv_rating"])
        if "dwt_window_ms" in cfg: target["dwt"] = float(cfg["dwt_window_ms"])
        if "sensor_lag_comp_us" in cfg: target["lag"] = float(cfg["sensor_lag_comp_us"])
        if "zero_electric_angle" in cfg: target["zero"] = float(cfg["zero_electric_angle"])
        if "sensor_direction" in cfg: target["sensor_direction"] = int(cfg["sensor_direction"])
        if "align_v" in cfg: target["align_v"] = float(cfg["align_v"])

    def _export_config_to_json(self, motor_id=None, silent=False):
        """Exports calibration without conflict. Motor 1 -> motor_config_m1.json, Motor 2 -> motor_config_m2.json"""
        if motor_id is None:
            motor_id = self.active_motor

        target_file = MOTOR_CONFIG_M1_JSON_PATH if motor_id == 1 else MOTOR_CONFIG_M2_JSON_PATH

        if not silent:
            chosen = filedialog.asksaveasfilename(
                title=f"Export Motor {motor_id} Calibration & Tuning Profile",
                initialdir=os.path.dirname(target_file),
                initialfile=os.path.basename(target_file),
                filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")]
            )
            if not chosen:
                return
            target_file = chosen

        self._pending_json_save_path = target_file
        self._pending_export_motor_id = motor_id
        self._silent_json_export = silent
        self._explicit_export_requested = True
        self.serial_engine.send_command("export")

    def _on_config_json_received(self, json_str):
        """Processes dual-motor config JSON from STM32 and writes separated calibration files."""
        try:
            cfg = json.loads(json_str)
            target_motor = getattr(self, "_pending_export_motor_id", self.active_motor)
            is_valid = getattr(self, "_explicit_export_requested", False)
            self._explicit_export_requested = False

            # Extract Motor 1 data
            m1_lut = cfg.get("lut_128", [0.0] * 128)
            m1_cfg = {
                "motor_id": 1,
                "zero_electric_angle": float(cfg.get("zero_electric_angle", 0.0)),
                "sensor_direction": int(cfg.get("sensor_direction", 1)),
                "sensor_lag_comp_us": float(cfg.get("sensor_lag_comp_us", 35.0)),
                "motor_kv_rating": float(cfg.get("motor_kv_rating", 130.0)),
                "is_lut_calibrated": bool(cfg.get("is_lut_calibrated", False)),
                "pid_p": float(cfg.get("pid_p", 0.15)),
                "pid_i": float(cfg.get("pid_i", 1.5)),
                "pid_d": float(cfg.get("pid_d", 0.0)),
                "lpf_tf": float(cfg.get("lpf_tf", 0.0005)),
                "dwt_window_ms": float(cfg.get("dwt_window_ms", 2.0)),
                "lut_128": m1_lut
            }

            # Extract Motor 2 data
            m2_zero = float(cfg.get("m2_zero_electric_angle", cfg.get("zero_electric_angle", 0.0)))
            m2_dir = int(cfg.get("m2_sensor_direction", cfg.get("sensor_direction", 1)))
            m2_lut = cfg.get("m2_lut_128", [0.0] * 128)
            m2_cfg = {
                "motor_id": 2,
                "zero_electric_angle": m2_zero,
                "sensor_direction": m2_dir,
                "sensor_lag_comp_us": float(cfg.get("m2_sensor_lag_comp_us", 35.0)),
                "motor_kv_rating": float(cfg.get("m2_motor_kv_rating", 130.0)),
                "is_lut_calibrated": bool(cfg.get("m2_is_lut_calibrated", False)),
                "pid_p": float(cfg.get("m2_pid_p", 0.15)),
                "pid_i": float(cfg.get("m2_pid_i", 1.5)),
                "pid_d": float(cfg.get("m2_pid_d", 0.0)),
                "lpf_tf": float(cfg.get("m2_lpf_tf", 0.0005)),
                "dwt_window_ms": float(cfg.get("m2_dwt_window_ms", 2.0)),
                "lut_128": m2_lut
            }

            self._update_motor_config_cache(1, m1_cfg)
            self._update_motor_config_cache(2, m2_cfg)

            save_path = getattr(self, "_pending_json_save_path", None)

            # Save distinct, non-conflicting calibration files
            if is_valid or m1_cfg["zero_electric_angle"] != 0.0 or m2_cfg["zero_electric_angle"] != 0.0:
                # 1. Distinct Motor 1 config & LUT
                with open(MOTOR_CONFIG_M1_JSON_PATH, "w") as f:
                    json.dump(m1_cfg, f, indent=2)

                with open(LUT_M1_JSON_PATH, "w") as f:
                    json.dump({
                        "motor_id": 1,
                        "lut_errors": m1_lut,
                        "zero_electric_angle": m1_cfg["zero_electric_angle"],
                        "sensor_direction": m1_cfg["sensor_direction"],
                        "sensor_lag_comp_us": m1_cfg["sensor_lag_comp_us"]
                    }, f, indent=2)

                # 2. Distinct Motor 2 config & LUT
                with open(MOTOR_CONFIG_M2_JSON_PATH, "w") as f:
                    json.dump(m2_cfg, f, indent=2)

                with open(LUT_M2_JSON_PATH, "w") as f:
                    json.dump({
                        "motor_id": 2,
                        "lut_errors": m2_lut,
                        "zero_electric_angle": m2_cfg["zero_electric_angle"],
                        "sensor_direction": m2_cfg["sensor_direction"],
                        "sensor_lag_comp_us": m2_cfg["sensor_lag_comp_us"]
                    }, f, indent=2)

                # 3. Combined file preserving both motor calibrations
                lut_dual = {}
                if os.path.exists(LUT_JSON_PATH):
                    try:
                        with open(LUT_JSON_PATH, "r") as f:
                            lut_dual = json.load(f)
                    except Exception:
                        lut_dual = {}
                lut_dual["motor1"] = {
                    "lut_errors": m1_lut,
                    "zero_electric_angle": m1_cfg["zero_electric_angle"],
                    "sensor_direction": m1_cfg["sensor_direction"],
                    "sensor_lag_comp_us": m1_cfg["sensor_lag_comp_us"]
                }
                lut_dual["motor2"] = {
                    "lut_errors": m2_lut,
                    "zero_electric_angle": m2_cfg["zero_electric_angle"],
                    "sensor_direction": m2_cfg["sensor_direction"],
                    "sensor_lag_comp_us": m2_cfg["sensor_lag_comp_us"]
                }
                with open(LUT_JSON_PATH, "w") as f:
                    json.dump(lut_dual, f, indent=2)

                # 4. Save custom destination if user used 'Save As'
                if save_path and save_path not in (MOTOR_CONFIG_M1_JSON_PATH, MOTOR_CONFIG_M2_JSON_PATH):
                    chosen_cfg = m1_cfg if target_motor == 1 else m2_cfg
                    with open(save_path, "w") as f:
                        json.dump(chosen_cfg, f, indent=2)

                # 5. Legacy config file
                with open(MOTOR_CONFIG_JSON_PATH, "w") as f:
                    json.dump(cfg, f, indent=2)

                # Update active GUI fields
                self.after(0, lambda: self._load_gui_from_motor_config(self.active_motor))

                self.after(0, lambda: self._log_console(
                    f"[SYS] Calibrations saved without conflict:\n"
                    f"      • Motor 1 -> {os.path.basename(MOTOR_CONFIG_M1_JSON_PATH)} & {os.path.basename(LUT_M1_JSON_PATH)}\n"
                    f"      • Motor 2 -> {os.path.basename(MOTOR_CONFIG_M2_JSON_PATH)} & {os.path.basename(LUT_M2_JSON_PATH)}\n"
                ))

                if not getattr(self, "_silent_json_export", False):
                    self.after(0, lambda: messagebox.showinfo(
                        "Calibrations Saved",
                        f"Saved calibrations without conflict:\n\n"
                        f"• Motor 1: {os.path.basename(MOTOR_CONFIG_M1_JSON_PATH)}\n"
                        f"• Motor 2: {os.path.basename(MOTOR_CONFIG_M2_JSON_PATH)}\n"
                        f"• LUT Files: {os.path.basename(LUT_M1_JSON_PATH)} / {os.path.basename(LUT_M2_JSON_PATH)}"
                    ))
            else:
                self.after(0, lambda: self._log_console(
                    f"[SYS] Live config received: M1={m1_cfg['zero_electric_angle']:.4f} rad, M2={m2_cfg['zero_electric_angle']:.4f} rad\n"
                ))
        except Exception as ex:
            self.after(0, lambda: self._log_console(f"[JSON ERR] Failed to parse exported JSON: {ex}\n"))

    def _upload_config_from_json(self):
        """Uploads calibration file into STM32 and flash, detecting whether it's M1 or M2."""
        default_file = MOTOR_CONFIG_M2_JSON_PATH if self.active_motor == 2 else MOTOR_CONFIG_M1_JSON_PATH
        if not os.path.exists(default_file):
            default_file = MOTOR_CONFIG_M1_JSON_PATH if os.path.exists(MOTOR_CONFIG_M1_JSON_PATH) else LUT_JSON_PATH

        path = filedialog.askopenfilename(
            title=f"Select Calibration JSON File for Motor {self.active_motor}",
            initialdir=os.path.dirname(default_file),
            initialfile=os.path.basename(default_file),
            filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")]
        )
        if not path:
            return

        try:
            with open(path, "r") as f:
                data = json.load(f)

            fname_lower = os.path.basename(path).lower()
            is_explicit_m2 = "m2" in fname_lower or "motor2" in fname_lower
            is_explicit_m1 = "m1" in fname_lower or "motor1" in fname_lower

            if is_explicit_m2:
                target_motors = [2]
            elif is_explicit_m1:
                target_motors = [1]
            elif "motor1" in data and "motor2" in data:
                target_motors = [1, 2]
            else:
                target_motors = [self.active_motor]

            for m in target_motors:
                if m == 1:
                    cfg = data.get("motor1", data) if isinstance(data.get("motor1"), dict) else data
                else:
                    cfg = data.get("motor2", data) if isinstance(data.get("motor2"), dict) else data

                # 1. Upload 128-point LUT
                lut = cfg.get("lut_128", cfg.get("lut_errors", None))
                if lut is None and m == 2:
                    lut = data.get("m2_lut_128", None)
                if lut is not None and len(lut) == 128:
                    self._log_console(f"[SYS] Uploading 128-point LUT curve for Motor {m}...\n")
                    for line_idx in range(8):
                        chunk = lut[line_idx * 16 : (line_idx + 1) * 16]
                        csv_str = ",".join(f"{float(x):.6f}" for x in chunk)
                        self.serial_engine.send_command(f"LUT {m} {line_idx} {csv_str}")
                        time.sleep(0.04)
                    self.serial_engine.send_command(f"LUT_MODE {m} 1")
                    time.sleep(0.03)

                # 2. Upload Zero Electric Angle
                z_val = cfg.get("zero_electric_angle", None)
                if z_val is None and m == 2:
                    z_val = data.get("m2_zero_electric_angle", None)
                if z_val is not None and float(z_val) != -12345.0:
                    self.serial_engine.send_command(f"O{m} {float(z_val):.4f}")
                    time.sleep(0.02)

                # 3. Sensor Direction
                d_val = cfg.get("sensor_direction", None)
                if d_val is None and m == 2:
                    d_val = data.get("m2_sensor_direction", None)
                if d_val is not None:
                    self.serial_engine.send_command(f"DIR{m} {int(d_val)}")
                    time.sleep(0.02)

                # 4. Sensor Lag
                lag_val = cfg.get("sensor_lag_comp_us", None)
                if lag_val is None and m == 2:
                    lag_val = data.get("m2_sensor_lag_comp_us", None)
                if lag_val is not None:
                    self.serial_engine.send_command(f"LAG{m} {float(lag_val):.1f}")
                    time.sleep(0.02)

                # 5. PID, filter, KV, and DWT
                for k, cmd_key in [("pid_p", f"P{m}"), ("pid_i", f"I{m}"), ("pid_d", f"D{m}"), ("lpf_tf", f"F{m}"), ("motor_kv_rating", f"KV{m}"), ("dwt_window_ms", f"DWT{m}")]:
                    val = cfg.get(k, None)
                    if val is None and m == 2:
                        val = data.get(f"m2_{k}", None)
                    if val is not None:
                        self.serial_engine.send_command(f"{cmd_key} {float(val)}")
                        time.sleep(0.02)

                # Update in-memory cached configs
                self._update_motor_config_cache(m, cfg)

            # Persist uploaded values to STM32 Flash
            self.serial_engine.send_command("SAVE")
            time.sleep(0.1)

            # Refresh GUI fields
            self._load_gui_from_motor_config(self.active_motor)

            target_str = ", ".join(f"Motor {m}" for m in target_motors)
            self._log_console(f"[SYS] Uploaded & applied profile for {target_str} from {os.path.basename(path)} and saved to Flash!\n")
            messagebox.showinfo("Upload Complete", f"Profile uploaded and saved to STM32 Flash for {target_str} from:\n{os.path.basename(path)}")
        except Exception as ex:
            messagebox.showerror("Upload Error", str(ex))

    # =========================================================================
    # SERIAL TELEMETRY & LOGGING CALLBACKS
    # =========================================================================
    def refresh_ports(self):
        ports = self.serial_engine.list_ports()
        usb_ports = [p[0] for p in ports if "USB" in p[1].upper() or "STM" in p[1].upper() or p[0] == "COM11"]
        other_ports = [p[0] for p in ports if p[0] not in usb_ports and "BLUETOOTH" not in p[1].upper()]
        bt_ports = [p[0] for p in ports if p[0] not in usb_ports and p[0] not in other_ports]
        dev_list = usb_ports + other_ports + bt_ports
        self.port_combo["values"] = dev_list
        if dev_list:
            if "COM11" in dev_list:
                self.port_combo.set("COM11")
            else:
                self.port_combo.set(dev_list[0])

    def toggle_connection(self):
        if not self.serial_engine.state["connected"]:
            port = self.port_combo.get()
            if not port:
                messagebox.showerror("Error", "Please select a serial COM port.")
                return
            ok, msg = self.serial_engine.connect(port=port)
            if ok:
                self.connect_btn.configure(text="DISCONNECT", bg=COLOR_ACCENT_RED)
                self.sys_status_lbl.configure(text=f"● CONNECTED ({port} @ 921.6k)", fg=COLOR_ACCENT_GREEN)
                self.after(300, lambda: self.serial_engine.send_command("s"))
                self.after(500, lambda: self.serial_engine.send_command("export"))
            else:
                messagebox.showerror("Connection Error", msg)
        else:
            self.serial_engine.disconnect()
            self.connect_btn.configure(text="CONNECT", bg="#00838F")
            self.sys_status_lbl.configure(text="● STANDBY (DISCONNECTED)", fg=COLOR_TEXT_MUTED)

    def global_estop(self):
        self.serial_engine.send_emergency_stop()
        self.coupled_slider.set(0)
        self.sys_status_lbl.configure(text="🛑 GLOBAL E-STOP TRIGGERED", fg=COLOR_ACCENT_RED)

    def _on_telemetry(self, state):
        pass

    def _on_serial_log(self, text):
        if not self.log_paused:
            self.after(0, lambda t=text: self._log_console(t))

        if "[CALIB_FAILED]" in text:
            reason = text.split("[CALIB_FAILED]:")[-1].strip()
            self._calib_last_success = False
            self._calib_last_reason = reason
            self._calib_result_event.set()
            self.after(0, lambda r=reason: self.sys_status_lbl.configure(
                text=f"❌ CALIBRATION FAILED: {r}", fg=COLOR_ACCENT_RED
            ))
        elif "[LUT_READY]" in text:
            m_id = text.split("[LUT_READY]:")[-1].strip()
            self._calib_last_success = True
            self._calib_last_reason = ""
            self._calib_result_event.set()
            self.after(0, lambda m=m_id: self.sys_status_lbl.configure(
                text=f"✅ {m} CALIBRATION SUCCESS (128-PT LUT ACTIVE)", fg=COLOR_ACCENT_GREEN
            ))

    def _log_console(self, text):
        self.console_text.insert(tk.END, text)
        lines = int(self.console_text.index("end-1c").split(".")[0])
        if lines > 600:
            self.console_text.delete("1.0", "150.0")
        self.console_text.see(tk.END)

    def _clear_console(self):
        self.console_text.delete("1.0", tk.END)

    def _toggle_log_pause(self):
        self.log_paused = not self.log_paused
        self.pause_btn.configure(text="Resume Log" if self.log_paused else "Pause Log")

    def _send_console_cmd(self, event=None):
        cmd = self.cmd_entry.get().strip()
        if not cmd:
            return
        self.cmd_history.append(cmd)
        self.cmd_history_idx = len(self.cmd_history)
        self.cmd_entry.delete(0, tk.END)
        self.serial_engine.send_command(cmd)

    def _prev_cmd(self, event=None):
        if self.cmd_history and self.cmd_history_idx > 0:
            self.cmd_history_idx -= 1
            self.cmd_entry.delete(0, tk.END)
            self.cmd_entry.insert(0, self.cmd_history[self.cmd_history_idx])

    def _next_cmd(self, event=None):
        if self.cmd_history and self.cmd_history_idx < len(self.cmd_history) - 1:
            self.cmd_history_idx += 1
            self.cmd_entry.delete(0, tk.END)
            self.cmd_entry.insert(0, self.cmd_history[self.cmd_history_idx])
        else:
            self.cmd_history_idx = len(self.cmd_history)
            self.cmd_entry.delete(0, tk.END)

    # =========================================================================
    # PERIODIC UI POLLING (25 Hz)
    # =========================================================================
    def _ui_poll_loop(self):
        st = self.serial_engine.state

        if st["connected"] and (time.time() - st["last_packet_time"] < 1.0):
            m = self.active_motor
            other_m = 2 if m == 1 else 1

            # 1. Primary Motor Telemetry (Left Column)
            self.m1_dial.update_angle(st[f"m{m}_raw_angle"])
            self.m1_act_rpm_lbl.configure(text=f"{st[f'm{m}_actual_rpm']:+06.1f} RPM")
            self.m1_tgt_rpm_lbl.configure(text=f"Target: {st[f'm{m}_target_rpm']:+06.1f} RPM")
            self.m1_uq_lbl.configure(text=f"Uq: {st[f'm{m}_uq_v']:+.2f} V")
            self.m1_raw_lbl.configure(text=f"Raw14: {st[f'm{m}_raw_angle']}")
            self.m1_zero_lbl.configure(text=f"Zero: {st[f'm{m}_zero_angle']:.4f} rad ({st[f'm{m}_direction']})")

            if st[f"m{m}_enabled"]:
                self.primary_status_badge.configure(text="ENABLED (CLOSED-LOOP)", bg="#1B5E20")
            else:
                self.primary_status_badge.configure(text="DISABLED (HI-Z)", bg="#37474F")

            self.m1_badge_parity.configure(
                text="PARITY: OK" if st[f"m{m}_parity_ok"] else "PARITY: ERR",
                bg="#1B5E20" if st[f"m{m}_parity_ok"] else COLOR_ACCENT_RED
            )
            self.m1_badge_mag.configure(
                text="MAG: OK" if st[f"m{m}_mag_ok"] else "MAG: WEAK",
                bg="#1B5E20" if st[f"m{m}_mag_ok"] else COLOR_ACCENT_AMBER
            )
            self.m1_badge_lut.configure(
                text="128-PT LUT: ON" if st[f"m{m}_lut_active"] else "LUT: OFF (RAW)",
                bg="#00838F" if st[f"m{m}_lut_active"] else "#37474F"
            )
            self.m1_badge_lat.configure(text=f"LAT: {st['isr_lat_us']} µs")

            # 2. Calibration Progress
            if st[f"m{m}_calib"]:
                self.m1_lut_progress["value"] = st["calib_progress"]
                if not self.is_master_calibrating:
                    self.calib_status_lbl.configure(
                        text=f"Calibrating M{m} ({st['calib_progress']}%): {st[f'm{m}_calib_stage']}"
                    )
            elif st[f"m{m}_lut_active"]:
                self.m1_lut_progress["value"] = 100
                if not self.is_master_calibrating and "GRADE" in st[f"m{m}_calib_grade"]:
                    self.calib_grade_badge.configure(text=st[f"m{m}_calib_grade"], bg="#059669")
            else:
                self.m1_lut_progress["value"] = 0

            # 3. Velocity Oscilloscope (Plots active motor)
            with self.serial_engine.lock:
                if m == 1:
                    t_hist = list(self.serial_engine.history_target_rpm)
                    a_hist = list(self.serial_engine.history_actual_rpm)
                    u_hist = list(self.serial_engine.history_uq)
                else:
                    t_hist = list(self.serial_engine.history_m2_target_rpm)
                    a_hist = list(self.serial_engine.history_m2_actual_rpm)
                    u_hist = list(self.serial_engine.history_m2_uq)
            self.scope_chart.redraw(t_hist, a_hist, u_hist)

            # 4. Secondary Motor / Encoder Diagnostics (Right Column)
            self.m2_dial.update_angle(st[f"m{other_m}_raw_angle"])
            self.m2_act_rpm_lbl.configure(text=f"{st[f'm{other_m}_actual_rpm']:+05.1f} RPM")
            self.m2_tgt_rpm_lbl.configure(text=f"Target: {st[f'm{other_m}_target_rpm']:+05.1f} RPM")
            self.m2_raw_lbl.configure(text=f"Raw: {st[f'm{other_m}_raw_angle']}")
            self.m2_uq_lbl.configure(text=f"Uq: {st[f'm{other_m}_uq_v']:+.2f} V")

            if st[f"m{other_m}_enabled"]:
                self.m2_status_badge.configure(text="RUNNING", bg="#1B5E20")
            else:
                self.m2_status_badge.configure(text="STANDBY", bg="#37474F")

            self.m2_badge_parity.configure(
                text="PARITY: OK" if st[f"m{other_m}_parity_ok"] else "PARITY: ERR",
                bg="#1B5E20" if st[f"m{other_m}_parity_ok"] else COLOR_ACCENT_RED
            )
            self.m2_badge_mag.configure(
                text="MAG: OK" if st[f"m{other_m}_mag_ok"] else "MAG: WEAK",
                bg="#1B5E20" if st[f"m{other_m}_mag_ok"] else COLOR_ACCENT_AMBER
            )
            self.m2_badge_lut.configure(
                text="LUT: ACTIVE" if st[f"m{other_m}_lut_active"] else "LUT: OFF",
                bg="#00838F" if st[f"m{other_m}_lut_active"] else "#37474F"
            )
            self.m2_badge_lat.configure(text=f"LAT: {st['isr_lat_us']} µs")

        self.after(40, self._ui_poll_loop)


def main():
    app = DualMotorDashboardApp()
    app.mainloop()


if __name__ == "__main__":
    main()
