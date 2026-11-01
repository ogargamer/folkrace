#!/usr/bin/env python3
"""
app.py
======
DA WINNA — STM32G474 Motor 1 & MT6816 Dual Encoders 20.0 kHz SimpleFOC Live GUI
Inspired by the proven ESP32 SimpleFOC Suite:
  • 1-Click Master Auto-Calibrator (Stepped 128-Point Dual-Pass LUT + 11-Pole Electrical Zero Alignment)
  • Overall Speed Control with 1.0 ms DWT Velocity Window, 11-Pole Zero Interpolation & Lag Compensation
  • Live Real-Time PID / LPF / Voltage Ramp / Lag / Zero / KV Tuning + Flash & JSON Persistence
  • Real-Time Velocity Tracking Oscilloscope & Dual 14-Bit Angle Compass Dials
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

LUT_JSON_PATH = os.path.join(CURRENT_DIR, "lut_calibrations.json")
MOTOR_CONFIG_JSON_PATH = os.path.abspath(os.path.join(CURRENT_DIR, "..", "motor_config_gm3506.json"))


class DualMotorDashboardApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("🏎️ DA WINNA — STM32G474 MASTER LUT CALIBRATION & SPEED CONTROL SUITE (20.0 kHz FOC)")
        self.geometry("1420x930")
        self.minsize(1240, 840)
        self.configure(bg=COLOR_BG)

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
        self._calib_result_event = threading.Event()
        self._calib_last_success = False
        self._calib_last_reason = ""

        self._init_styles()
        self._build_ui()

        # Keyboard Shortcuts
        self.bind("<Escape>", lambda e: self.global_estop())

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
            text="🏎️ DA WINNA | STM32G474 MASTER LUT CALIBRATION & SPEED CONTROL",
            font=("Segoe UI", 11, "bold"),
            bg=COLOR_CARD,
            fg=COLOR_ACCENT_CYAN
        )
        title_lbl.pack(side=tk.LEFT, padx=12)

        conn_frame = tk.Frame(top_bar, bg=COLOR_CARD)
        conn_frame.pack(side=tk.LEFT, padx=10)

        tk.Label(conn_frame, text="Port:", bg=COLOR_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=(0, 4))
        self.port_combo = ttk.Combobox(conn_frame, width=11, state="readonly")
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
            padx=6,
            pady=2
        ).pack(side=tk.LEFT, padx=2)

        tk.Label(conn_frame, text="921600 Baud", bg=COLOR_CARD, fg=COLOR_ACCENT_GREEN, font=("Consolas", 9, "bold")).pack(side=tk.LEFT, padx=6)

        self.connect_btn = tk.Button(
            conn_frame,
            text="CONNECT",
            command=self.toggle_connection,
            bg="#00838F",
            fg="#FFFFFF",
            activebackground=COLOR_ACCENT_CYAN,
            font=("Segoe UI", 9, "bold"),
            bd=0,
            padx=12,
            pady=3
        )
        self.connect_btn.pack(side=tk.LEFT, padx=6)

        self.sys_status_lbl = tk.Label(
            top_bar,
            text="● STANDBY (DISCONNECTED)",
            font=("Segoe UI", 9, "bold"),
            bg=COLOR_CARD,
            fg=COLOR_TEXT_MUTED
        )
        self.sys_status_lbl.pack(side=tk.LEFT, padx=10)

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
        self.estop_btn.pack(side=tk.RIGHT, padx=12)

        # =====================================================================
        # 2. MAIN WORKSPACE (Left: Master Calib & Speed Control | Right: Scope & Enc2)
        # =====================================================================
        main_panes = tk.Frame(self, bg=COLOR_BG)
        main_panes.pack(fill=tk.BOTH, expand=True, padx=8, pady=2)

        self.m1_frame = self._build_motor1_master_column(main_panes)
        self.m1_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 4))

        self.right_frame = self._build_right_telemetry_column(main_panes)
        self.right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False, padx=(4, 0))

        # =====================================================================
        # 3. BOTTOM INTERACTIVE SERIAL CONSOLE
        # =====================================================================
        bottom_frame = tk.Frame(self, bg=COLOR_CARD, height=145, bd=1, relief=tk.SOLID, highlightbackground=COLOR_BORDER)
        bottom_frame.pack(fill=tk.X, side=tk.BOTTOM, padx=8, pady=(2, 6))
        bottom_frame.pack_propagate(False)

        console_header = tk.Frame(bottom_frame, bg=COLOR_CARD)
        console_header.pack(fill=tk.X, padx=8, pady=(3, 1))

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
    # LEFT COLUMN: MOTOR 1 MASTER STEPPED LUT CALIBRATION & OVERALL SPEED CTRL
    # =========================================================================
    def _build_motor1_master_column(self, parent):
        frame = tk.Frame(parent, bg=COLOR_CARD, bd=1, relief=tk.SOLID, highlightbackground=COLOR_BORDER)

        # 1. Column Header
        hdr = tk.Frame(frame, bg=COLOR_CARD_SECONDARY, height=34)
        hdr.pack(fill=tk.X, side=tk.TOP)
        hdr.pack_propagate(False)

        tk.Label(
            hdr,
            text="MOTOR 1 — GM3506 FOC CONTROLLER (TIM2: PA0/PA1/PA2 | EN: PB1 | CS1: PB2)",
            font=("Segoe UI", 10, "bold"),
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_ACCENT_CYAN
        ).pack(side=tk.LEFT, padx=10)

        self.m1_status_badge = tk.Label(hdr, text="DISABLED", font=("Consolas", 9, "bold"), bg="#37474F", fg="#FFFFFF", padx=8, pady=2)
        self.m1_status_badge.pack(side=tk.RIGHT, padx=8)

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
        tk.Label(spd_left, text="ACTUAL VELOCITY (1ms DWT WINDOW):", font=("Segoe UI", 8, "bold"), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT_MUTED).pack(anchor="w")
        self.m1_act_rpm_lbl = tk.Label(spd_left, text="+0.0 RPM", font=("Consolas", 20, "bold"), bg=COLOR_CARD_SECONDARY, fg=COLOR_ACCENT_GREEN)
        self.m1_act_rpm_lbl.pack(anchor="w")
        self.m1_tgt_rpm_lbl = tk.Label(spd_left, text="Target: +0.0 RPM", font=("Consolas", 9, "bold"), bg=COLOR_CARD_SECONDARY, fg=COLOR_ACCENT_CYAN)
        self.m1_tgt_rpm_lbl.pack(anchor="w")

        # Quick Enable / Disable / Stop Pillar on the Right of Metrics Box
        quick_act = tk.Frame(spd_row, bg=COLOR_CARD_SECONDARY)
        quick_act.pack(side=tk.RIGHT, padx=4)
        tk.Button(quick_act, text="⚡ ENABLE M1 (E1)", command=lambda: self.serial_engine.send_command("E1"), bg="#1B5E20", fg="#FFFFFF", bd=0, padx=10, pady=3, font=("Segoe UI", 8, "bold")).pack(fill=tk.X, pady=1)
        tk.Button(quick_act, text="🛑 STOP 0 RPM (x)", command=self._stop_motor1, bg="#B71C1C", fg="#FFFFFF", bd=0, padx=10, pady=3, font=("Segoe UI", 8, "bold")).pack(fill=tk.X, pady=1)
        tk.Button(quick_act, text="DISABLE HI-Z (D1)", command=lambda: self.serial_engine.send_command("D1"), bg="#37474F", fg="#FFFFFF", bd=0, padx=10, pady=2, font=("Segoe UI", 8)).pack(fill=tk.X, pady=1)

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
        # 3. HIGH-PRECISION MASTER AUTO-CALIBRATION HUB (STEPPED LUT ONLY)
        # =====================================================================
        cal_sec = tk.LabelFrame(
            frame,
            text="🎯 HIGH-PRECISION MASTER AUTO-CALIBRATION (128-PT STEPPED LUT + 11-POLE ZERO ALIGNMENT)",
            bg=COLOR_CARD,
            fg=COLOR_ACCENT_AMBER,
            font=("Segoe UI", 8, "bold")
        )
        cal_sec.pack(fill=tk.X, padx=8, pady=4)

        cal_top = tk.Frame(cal_sec, bg=COLOR_CARD)
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

        self.m1_lut_progress = ttk.Progressbar(cal_sec, orient=tk.HORIZONTAL, length=100, mode="determinate")
        self.m1_lut_progress.pack(fill=tk.X, padx=6, pady=2)

        # Calibration Options Row (Align Voltage, Boost Retry, Verify Spin, LUT Mode Radio)
        cal_opt_row = tk.Frame(cal_sec, bg=COLOR_CARD)
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
            command=lambda: self.serial_engine.send_command("SET_MODE 0"),
            bg=COLOR_CARD, fg=COLOR_TEXT, selectcolor=COLOR_CARD_SECONDARY, activebackground=COLOR_CARD, font=("Segoe UI", 8)
        ).pack(side=tk.LEFT)
        tk.Radiobutton(
            mode_box, text="1: 128-PT LUT", variable=self.corr_mode_var, value=1,
            command=lambda: self.serial_engine.send_command("SET_MODE 1"),
            bg=COLOR_CARD, fg=COLOR_ACCENT_CYAN, selectcolor=COLOR_CARD_SECONDARY, activebackground=COLOR_CARD, font=("Segoe UI", 8, "bold")
        ).pack(side=tk.LEFT)

        # Prominent 1-Click Master Auto-Calibrate Button
        self.master_cal_btn = tk.Button(
            cal_sec,
            text="🚀 1-CLICK MASTER AUTO-CALIBRATE (128-PT STEPPED LUT + 11-POLE ZERO + AUTO-SAVE)",
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
        cal_sub_row = tk.Frame(cal_sec, bg=COLOR_CARD)
        cal_sub_row.pack(fill=tk.X, padx=6, pady=(1, 5))

        tk.Button(
            cal_sub_row,
            text="🔬 Stepped 128-Pt LUT + Align (w)",
            command=lambda: self.serial_engine.send_command("CALIB 1"),
            bg="#00838F",
            fg="#FFFFFF",
            bd=0,
            padx=6,
            pady=2,
            font=("Segoe UI", 8, "bold")
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        tk.Button(
            cal_sub_row,
            text="⚡ 11-Pole Zero Align Only (c)",
            command=lambda: self.serial_engine.send_command("C1"),
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_TEXT,
            bd=0,
            padx=6,
            pady=2,
            font=("Segoe UI", 8)
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        tk.Button(
            cal_sub_row,
            text="⚡ Trim Lag @ 1000 RPM (n)",
            command=lambda: self.serial_engine.send_command("N1"),
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_ACCENT_GREEN,
            bd=0,
            padx=6,
            pady=2,
            font=("Segoe UI", 8, "bold")
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        tk.Button(
            cal_sub_row,
            text="📋 Dump LUT Array (u)",
            command=lambda: self.serial_engine.send_command("u"),
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_TEXT_MUTED,
            bd=0,
            padx=6,
            pady=2,
            font=("Segoe UI", 8)
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # =====================================================================
        # 4. OVERALL SPEED CONTROL & DIRECT VOLTAGE TEST
        # =====================================================================
        spd_sec = tk.LabelFrame(
            frame,
            text="⚡ OVERALL SPEED CONTROL (CLOSED-LOOP RPM) & DIRECT VOLTAGE TEST",
            bg=COLOR_CARD,
            fg=COLOR_ACCENT_CYAN,
            font=("Segoe UI", 8, "bold")
        )
        spd_sec.pack(fill=tk.X, padx=8, pady=4)

        slider_row = tk.Frame(spd_sec, bg=COLOR_CARD)
        slider_row.pack(fill=tk.X, padx=6, pady=2)

        tk.Label(slider_row, text="Speed Slider (-1000..+1000 RPM):", bg=COLOR_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT)
        self.coupled_slider = tk.Scale(
            slider_row,
            from_=-1000,
            to=1000,
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
        preset_row = tk.Frame(spd_sec, bg=COLOR_CARD)
        preset_row.pack(fill=tk.X, padx=4, pady=2)
        for spd in [-800, -400, -200, -100, -50, 0, 50, 100, 200, 400, 800, 1000]:
            bg_c = "#B71C1C" if spd == 0 else COLOR_CARD_SECONDARY
            fg_c = "#FFFFFF" if spd == 0 else (COLOR_ACCENT_CYAN if spd > 0 else COLOR_TEXT_MUTED)
            tk.Button(
                preset_row,
                text=f"{spd}" if spd != 0 else "0 STOP",
                command=lambda s=spd: self._set_motor_speed(1, s),
                bg=bg_c,
                fg=fg_c,
                bd=0,
                padx=4,
                pady=2,
                font=("Consolas", 8, "bold" if spd == 0 else "normal")
            ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=1)

        # Fine Nudge & Direct Voltage Test Row
        nudge_row = tk.Frame(spd_sec, bg=COLOR_CARD)
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
        tk.Button(nudge_row, text="+2.0V CW", command=lambda: self.serial_engine.send_command("v2.0"), bg="#065F46", fg="#FFFFFF", bd=0, padx=8, pady=1, font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=2)
        tk.Button(nudge_row, text="-2.0V CCW", command=lambda: self.serial_engine.send_command("v-2.0"), bg="#065F46", fg="#FFFFFF", bd=0, padx=8, pady=1, font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=2)
        tk.Button(nudge_row, text="Tare (Z1)", command=lambda: self.serial_engine.send_command("Z1"), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=0, padx=8, pady=1, font=("Segoe UI", 8)).pack(side=tk.RIGHT, padx=4)

        # =====================================================================
        # 5. LIVE PARAMETER TUNING & FLASH / JSON PERSISTENCE
        # =====================================================================
        tune_sec = tk.LabelFrame(
            frame,
            text="🎛️ LIVE FOC PARAMETER TUNING & STM32 FLASH / JSON PERSISTENCE (PRESS ENTER OR APPLY)",
            bg=COLOR_CARD,
            fg=COLOR_TEXT_MUTED,
            font=("Segoe UI", 8, "bold")
        )
        tune_sec.pack(fill=tk.X, padx=8, pady=4)

        t_row1 = tk.Frame(tune_sec, bg=COLOR_CARD)
        t_row1.pack(fill=tk.X, padx=4, pady=2)
        self.p_ent = self._make_tune_field(t_row1, "P:", "0.120", width=6, cmd_prefix="P1 ")
        self.i_ent = self._make_tune_field(t_row1, "I:", "1.200", width=6, cmd_prefix="I1 ")
        self.d_ent = self._make_tune_field(t_row1, "D:", "0.0000", width=6, cmd_prefix="D1 ")
        self.tf_ent = self._make_tune_field(t_row1, "LPF Tf(s):", "0.003", width=6, cmd_prefix="F1 ")
        self.vramp_ent = self._make_tune_field(t_row1, "V-Ramp(V/s):", "200.0", width=6, cmd_prefix="RAMP1 ")
        self.vlim_ent = self._make_tune_field(t_row1, "V-Lim(V):", "6.93", width=5, cmd_prefix="V1 ")
        self.vlim_ent = self._make_tune_field(t_row1, "V-Lim(V):", "12.00", width=5, cmd_prefix="V1 ")

        t_row2 = tk.Frame(tune_sec, bg=COLOR_CARD)
        t_row2.pack(fill=tk.X, padx=4, pady=2)
        self.maxrpm_ent = self._make_tune_field(t_row2, "Max RPM:", "1000", width=6, cmd_prefix="MAXRPM1 ")
        self.lag_ent = self._make_tune_field(t_row2, "Lag(µs):", "35.0", width=5, cmd_prefix="LAG1 ")
        self.kv_ent = self._make_tune_field(t_row2, "KV:", "140.0", width=5, cmd_prefix="KV1 ")
        self.kv_ent = self._make_tune_field(t_row2, "KV:", "130.0", width=5, cmd_prefix="KV1 ")
        self.zero_ent = self._make_tune_field(t_row2, "Zero(rad):", "0.0000", width=7, cmd_prefix="ZERO1 ")

        tk.Button(t_row2, text="-0.02", command=lambda: self._nudge_zero(-0.02), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=0, padx=5, font=("Consolas", 8)).pack(side=tk.LEFT, padx=1)
        tk.Button(t_row2, text="+0.02", command=lambda: self._nudge_zero(+0.02), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=0, padx=5, font=("Consolas", 8)).pack(side=tk.LEFT, padx=1)

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

        tk.Button(
            t_row2,
            text="🔄 12V Defaults",
            command=self._reset_12v_defaults,
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_TEXT_MUTED,
            bd=0,
            padx=8,
            pady=1,
            font=("Segoe UI", 8)
        ).pack(side=tk.RIGHT, padx=3)

        # Flash & JSON Persistence Row
        persist_row = tk.Frame(tune_sec, bg=COLOR_CARD)
        persist_row.pack(fill=tk.X, padx=4, pady=(3, 5))

        tk.Button(
            persist_row,
            text="💾 Save to STM32 Flash (SAVE)",
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

        tk.Button(
            persist_row,
            text="💾 Export Config & LUT to JSON",
            command=self._export_config_to_json,
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_ACCENT_AMBER,
            bd=0,
            padx=8,
            pady=2,
            font=("Segoe UI", 8, "bold")
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        tk.Button(
            persist_row,
            text="📂 Upload JSON Config / LUT",
            command=self._upload_config_from_json,
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_TEXT,
            bd=0,
            padx=8,
            pady=2,
            font=("Segoe UI", 8)
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        return frame

    # =========================================================================
    # RIGHT COLUMN: VELOCITY OSCILLOSCOPE, ENCODER 2 & HARDWARE PINOUT
    # =========================================================================
    def _build_right_telemetry_column(self, parent):
        frame = tk.Frame(parent, bg=COLOR_CARD, width=500, bd=1, relief=tk.SOLID, highlightbackground=COLOR_BORDER)
        frame.pack_propagate(False)

        # 1. Real-Time Velocity Tracking Oscilloscope Card
        scope_hdr = tk.Frame(frame, bg=COLOR_CARD_SECONDARY, height=32)
        scope_hdr.pack(fill=tk.X, side=tk.TOP)
        scope_hdr.pack_propagate(False)
        tk.Label(
            scope_hdr,
            text="📈 REAL-TIME VELOCITY TRACKING OSCILLOSCOPE (MOTOR 1)",
            font=("Segoe UI", 9, "bold"),
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_ACCENT_GREEN
        ).pack(side=tk.LEFT, padx=10)

        self.scope_chart = VelocityOscilloscopeChart(frame, height=150)
        self.scope_chart.pack(fill=tk.X, padx=8, pady=6)

        # 2. Encoder 2 Card
        hdr2 = tk.Frame(frame, bg=COLOR_CARD_SECONDARY, height=30)
        hdr2.pack(fill=tk.X)
        hdr2.pack_propagate(False)

        tk.Label(hdr2, text="🧭 ENCODER 2 DIAGNOSTICS (SPI1 | CS2: PB3)", font=("Segoe UI", 9, "bold"), bg=COLOR_CARD_SECONDARY, fg=COLOR_ACCENT_GREEN).pack(side=tk.LEFT, padx=10)
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
        tk.Label(spd_row, text="ENCODER 2 SHAFT SPEED:", font=("Segoe UI", 8, "bold"), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT_MUTED).pack(anchor="w")
        self.m2_act_rpm_lbl = tk.Label(spd_row, text="+0.0 RPM", font=("Consolas", 16, "bold"), bg=COLOR_CARD_SECONDARY, fg=COLOR_ACCENT_GREEN)
        self.m2_act_rpm_lbl.pack(anchor="w")
        self.m2_tgt_rpm_lbl = tk.Label(spd_row, text="CS Pin: PB3 (SPI1)", font=("Consolas", 8), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT_MUTED)
        self.m2_tgt_rpm_lbl.pack(anchor="w")

        volt_row = tk.Frame(metrics_box, bg=COLOR_CARD_SECONDARY)
        volt_row.pack(fill=tk.X, padx=8, pady=2)
        self.m2_uq_lbl = tk.Label(volt_row, text="Bus: 2.65 MHz", font=("Consolas", 8), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT)
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
        self.m2_badge_foc = tk.Label(badge_row2, text="SPI1 MODE3", font=("Consolas", 8, "bold"), bg="#004D40", fg=COLOR_ACCENT_CYAN, padx=4)
        self.m2_badge_foc.pack(side=tk.LEFT, padx=2)
        self.m2_badge_lat = tk.Label(badge_row2, text="LAT: 12 µs", font=("Consolas", 8, "bold"), bg="#263238", fg="#ECEFF1", padx=4)
        self.m2_badge_lat.pack(side=tk.LEFT, padx=2)

        e_btn_row = tk.Frame(frame, bg=COLOR_CARD)
        e_btn_row.pack(fill=tk.X, padx=8, pady=2)
        tk.Button(e_btn_row, text="Tare Enc 2 (Z2)", command=lambda: self.serial_engine.send_command("Z2"), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=0, padx=6, pady=2, font=("Segoe UI", 8)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
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
            "STM32G474 @ 170MHz ACTIVE CONFIGURATION\n"
            "==============================================\n"
            "Motor 1 Driver (TIM2 @ 25 kHz Sine PWM):\n"
            "  • Phase A/B/C : PA0 / PA1 / PA2\n"
            "  • Driver EN   : PB1 (Gate Driver Enable)\n\n"
            "MT6816 Dual Magnetic Encoders (SPI1 @ 2.65 MHz):\n"
            "  • SCK / MOSI  : PA5 / PA7\n"
            "  • MISO (AF5)  : PB4 (Internal Pull-Up)\n"
            "  • CS1 / CS2   : PB2 (Motor 1) / PB3 (Enc 2)\n\n"
            "ESP32-Parity FOC Architecture:\n"
            "  • Hard ISR    : TIM6 @ 20.0 kHz (50 µs)\n"
            "  • Vel Window  : DWT 1.0 ms Cycle Integration\n"
            "  • Commutation : 128-Pt LUT + 11-Pole Zeros\n"
            "  • Speed Comp  : Dynamic Sensor Lag (35 µs)\n"
        )
        pin_text.insert(tk.END, pinout_info)
        pin_text.configure(state="disabled")

        return frame

    def _make_tune_field(self, parent, label_text, default_val, width=6, cmd_prefix=None):
        box = tk.Frame(parent, bg=COLOR_CARD)
        box.pack(side=tk.LEFT, padx=3)
        tk.Label(box, text=label_text, font=("Segoe UI", 8), bg=COLOR_CARD, fg=COLOR_TEXT_MUTED).pack(side=tk.LEFT)
        ent = tk.Entry(box, width=width, font=("Consolas", 8), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=1, relief=tk.SOLID, insertbackground=COLOR_ACCENT_CYAN)
        ent.insert(0, default_val)
        ent.pack(side=tk.LEFT, padx=1)
        if cmd_prefix:
            ent.bind("<Return>", lambda e, p=cmd_prefix, w=ent: self._send_single_tune(p, w.get()))
        return ent

    def _send_single_tune(self, prefix, val_str):
        try:
            v = float(val_str)
            self.serial_engine.send_command(f"{prefix}{v}")
        except ValueError:
            pass

    # =========================================================================
    # 1-CLICK MASTER AUTO-CALIBRATION ENGINE (STEPPED LUT + 11-POLE ZERO)
    # =========================================================================
    def start_master_auto_calibration(self):
        if not self.serial_engine.state["connected"]:
            messagebox.showwarning("Not Connected", "Please connect to the STM32 COM port first.")
            return
        if self.is_master_calibrating or self.serial_engine.state["m1_calib"]:
            messagebox.showinfo("Busy", "Calibration is already running.")
            return

        self.is_master_calibrating = True
        self.master_cal_btn.configure(text="⏳ MASTER AUTO-CALIBRATION IN PROGRESS...", bg="#B45309", state="disabled")
        threading.Thread(target=self._master_calibrate_worker, daemon=True).start()

    def _master_calibrate_worker(self):
        try:
            st = self.serial_engine.state

            # Step 0: Sensor Health Verification
            self._set_calib_ui("Step 0/4: Checking MT6816 SPI Sensor Health...", "CHECKING SENSOR", "#0284C7")
            self.serial_engine.send_command("ENC")
            time.sleep(0.4)
            if not st["m1_parity_ok"] or not st["m1_mag_ok"]:
                self._set_calib_ui("❌ Aborted: Encoder 1 SPI Parity or Magnet error!", "SENSOR FAULT", COLOR_ACCENT_RED)
                return

            # Step 1: Configure Align Voltage & Run Stepped 128-Point Dual-Pass LUT + 11-Pole Zero Sweep
            try:
                align_v = float(self.align_v_ent.get())
            except ValueError:
                align_v = 4.0

            self.serial_engine.send_command(f"A1 {align_v:.2f}")
            time.sleep(0.15)

            self._calib_result_event.clear()
            self._calib_last_success = False
            self._calib_last_reason = ""

            self._set_calib_ui(
                f"Step 1/4: Running 128-Pt Stepped LUT + 11-Pole Zero Sweep @ {align_v:.1f}V...",
                "SWEEPING LUT...",
                "#D97706"
            )
            self.serial_engine.send_command("CALIB 1")

            # Wait up to 95 seconds for dual-pass stepped LUT + cooldowns + 11-pole alignment
            t0 = time.time()
            while time.time() - t0 < 95.0:
                if not st["connected"]:
                    self._set_calib_ui("❌ Disconnected during calibration.", "DISCONNECTED", COLOR_ACCENT_RED)
                    return
                if self._calib_result_event.wait(timeout=0.25):
                    break

            # Optional Boost Voltage Retry if stalled
            if not self._calib_last_success and self.boost_var.get() and align_v < 4.2:
                boost_v = 4.2
                self._set_calib_ui(
                    f"⚠️ Stall detected ({self._calib_last_reason}) — Retrying at Boost {boost_v:.1f}V...",
                    "BOOST RETRY",
                    "#D97706"
                )
                self.serial_engine.send_command(f"A1 {boost_v:.2f}")
                time.sleep(0.2)
                self._calib_result_event.clear()
                self.serial_engine.send_command("CALIB 1")

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

            # Step 2: Ensure Mode 1 (128-Point LUT) is selected
            self.corr_mode_var.set(1)
            self.serial_engine.send_command("M1 1")
            time.sleep(0.2)

            # Step 3: Optional Closed-Loop Verification Spin (+120 RPM CW & -120 RPM CCW)
            if self.verify_spin_var.get():
                self._set_calib_ui("Step 3/4: Verifying Closed-Loop Velocity (+120 RPM CW / -120 RPM CCW)...", "VERIFYING SPIN", "#0284C7")
                self.serial_engine.send_command("E1")
                time.sleep(0.2)
                self.serial_engine.send_command("T1 120")
                time.sleep(1.6)
                cw_rpm = abs(st["m1_actual_rpm"])

                self.serial_engine.send_command("T1 -120")
                time.sleep(1.8)
                ccw_rpm = abs(st["m1_actual_rpm"])

                self.serial_engine.send_command("T1 0")
                time.sleep(0.4)
                self._log_console(f"[VERIFY] Closed-loop spin check: CW={cw_rpm:.1f} RPM, CCW={ccw_rpm:.1f} RPM\n")

                if cw_rpm < 40.0 or ccw_rpm < 40.0:
                    self._set_calib_ui(
                        f"❌ Verification Spin Failed: CW={cw_rpm:.1f} RPM, CCW={ccw_rpm:.1f} RPM (Target 120 RPM). Motor Stalled!",
                        "SPIN FAILED",
                        COLOR_ACCENT_RED
                    )
                    self._log_console(f"❌ [VERIFY FAILED] Motor failed to reach 120 RPM in closed loop (CW={cw_rpm:.1f}, CCW={ccw_rpm:.1f}). Check wiring/driver/power.\n")
                    return

            # Step 4: Save to STM32 Flash & Export JSON to Disk
            self._set_calib_ui("Step 4/4: Saving LUT & 11-Pole Zeros to STM32 Flash & JSON...", "SAVING...", COLOR_ACCENT_GREEN)
            self.serial_engine.send_command("SAVE")
            time.sleep(0.25)
            self._export_config_to_json(silent=True)
            time.sleep(0.4)

            grade = st.get("m1_calib_grade", "128-PT LUT ACTIVE")
            self._set_calib_ui(
                f"✅ Master Calibration Complete! Zero={st['m1_zero_angle']:.4f} rad ({st['m1_direction']}) | 128-Pt LUT Saved.",
                grade,
                "#059669"
            )

        finally:
            self.is_master_calibrating = False
            self.after(0, lambda: self.master_cal_btn.configure(
                text="🚀 1-CLICK MASTER AUTO-CALIBRATE (128-PT STEPPED LUT + 11-POLE ZERO + AUTO-SAVE)",
                bg="#D97706",
                state="normal"
            ))

    def _set_calib_ui(self, status_text, badge_text, badge_color):
        def _upd():
            self.calib_status_lbl.configure(text=status_text)
            self.calib_grade_badge.configure(text=badge_text, bg=badge_color)
        self.after(0, _upd)

    # =========================================================================
    # UI Control & Tuning Handlers
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
                self.sys_status_lbl.configure(text=f"● CONNECTED ({port} @ 921600)", fg=COLOR_ACCENT_GREEN)
                # Query status on connect
                self.after(300, lambda: self.serial_engine.send_command("s"))
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

    def _stop_motor1(self):
        self.coupled_slider.set(0)
        self.serial_engine.send_command("x")

    def _set_motor_speed(self, motor_id, spd):
        if motor_id == 1:
            self.coupled_slider.set(spd)
        self.serial_engine.send_command(f"T{motor_id} {spd}")

    def _nudge_motor_speed(self, delta_rpm):
        cur = self.serial_engine.state["m1_target_rpm"]
        new_spd = max(-1000.0, min(1000.0, cur + delta_rpm))
        self.coupled_slider.set(int(round(new_spd)))
        self.serial_engine.send_command(f"T1 {new_spd:.1f}")

    def _on_coupled_slider(self, val):
        self.serial_engine.send_command(f"T1 {val}")

    def _set_align_voltage(self):
        try:
            v = float(self.align_v_ent.get())
            self.serial_engine.send_command(f"A1 {v:.2f}")
        except ValueError:
            messagebox.showerror("Format Error", "Align voltage must be a valid number.")

    def _nudge_zero(self, delta_rad):
        try:
            cur = float(self.zero_ent.get())
        except ValueError:
            cur = self.serial_engine.state["m1_zero_angle"]
        new_z = (cur + delta_rad) % 6.2831853
        self.zero_ent.delete(0, tk.END)
        self.zero_ent.insert(0, f"{new_z:.4f}")
        self.serial_engine.send_command(f"O1 {new_z:.4f}")

    def _apply_all_tuning(self):
        try:
            p = float(self.p_ent.get())
            i = float(self.i_ent.get())
            d = float(self.d_ent.get())
            tf = float(self.tf_ent.get())
            vramp = float(self.vramp_ent.get())
            vlim = float(self.vlim_ent.get())
            maxrpm = float(self.maxrpm_ent.get())
            lag = float(self.lag_ent.get())
            kv = float(self.kv_ent.get())

            self.serial_engine.send_command(f"P1 {p}")
            self.serial_engine.send_command(f"I1 {i}")
            self.serial_engine.send_command(f"D1 {d}")
            self.serial_engine.send_command(f"F1 {tf}")
            self.serial_engine.send_command(f"RAMP1 {vramp}")
            self.serial_engine.send_command(f"V1 {vlim}")
            self.serial_engine.send_command(f"MAXRPM1 {maxrpm}")
            self.serial_engine.send_command(f"LAG1 {lag}")
            self.serial_engine.send_command(f"KV1 {kv}")
        except ValueError:
            messagebox.showerror("Format Error", "All tuning parameters must be valid numbers.")

    def _reset_12v_defaults(self):
        self.serial_engine.send_command("R1")
        self.p_ent.delete(0, tk.END); self.p_ent.insert(0, "0.120")
        self.i_ent.delete(0, tk.END); self.i_ent.insert(0, "1.200")
        self.d_ent.delete(0, tk.END); self.d_ent.insert(0, "0.0000")
        self.tf_ent.delete(0, tk.END); self.tf_ent.insert(0, "0.003")
        self.vramp_ent.delete(0, tk.END); self.vramp_ent.insert(0, "200.0")
        self.vlim_ent.delete(0, tk.END); self.vlim_ent.insert(0, "6.93")
        self.vlim_ent.delete(0, tk.END); self.vlim_ent.insert(0, "12.00")
        self.maxrpm_ent.delete(0, tk.END); self.maxrpm_ent.insert(0, "1000")
        self.lag_ent.delete(0, tk.END); self.lag_ent.insert(0, "35.0")
        self.kv_ent.delete(0, tk.END); self.kv_ent.insert(0, "140.0")
        self.kv_ent.delete(0, tk.END); self.kv_ent.insert(0, "130.0")

    # =========================================================================
    # JSON Config & 128-Point LUT Export / Import
    # =========================================================================
    def _export_config_to_json(self, silent=False):
        self._pending_json_save_path = MOTOR_CONFIG_JSON_PATH
        self._silent_json_export = silent
        self.serial_engine.send_command("export")

    def _on_config_json_received(self, json_str):
        try:
            cfg = json.loads(json_str)
            # 1. Save full ESP32-compatible motor_config_gm3506.json
            with open(MOTOR_CONFIG_JSON_PATH, "w") as f:
                json.dump(cfg, f, indent=2)

            # 2. Also update STM32/gui/lut_calibrations.json for backwards compatibility
            lut_data = {}
            if os.path.exists(LUT_JSON_PATH):
                try:
                    with open(LUT_JSON_PATH, "r") as f:
                        lut_data = json.load(f)
                except Exception:
                    lut_data = {}

            lut_data["motor1"] = {
                "lut_errors": cfg.get("lut_128", [0.0] * 128),
                "pole_zeros": cfg.get("pole_zeros", []),
                "zero_electric_angle": cfg.get("zero_electric_angle", 0.0),
                "sensor_direction": cfg.get("sensor_direction", "CW"),
                "sensor_lag_comp_us": cfg.get("sensor_lag_comp_us", 35.0)
            }
            with open(LUT_JSON_PATH, "w") as f:
                json.dump(lut_data, f, indent=2)

            self.after(0, lambda: self._log_console(
                f"[SYS] Saved calibrated LUT + 11-pole config to {os.path.basename(MOTOR_CONFIG_JSON_PATH)} & lut_calibrations.json\n"
            ))
            if not getattr(self, "_silent_json_export", False):
                self.after(0, lambda: messagebox.showinfo(
                    "Config Exported",
                    f"Saved full STM32 calibration & tuning profile to:\n• {MOTOR_CONFIG_JSON_PATH}\n• {LUT_JSON_PATH}"
                ))
        except Exception as ex:
            self.after(0, lambda: self._log_console(f"[JSON ERR] Failed to save exported JSON: {ex}\n"))

    def _upload_config_from_json(self):
        # Prefer motor_config_gm3506.json if present, or let user pick
        default_file = MOTOR_CONFIG_JSON_PATH if os.path.exists(MOTOR_CONFIG_JSON_PATH) else LUT_JSON_PATH
        path = filedialog.askopenfilename(
            title="Select Calibration JSON File",
            initialdir=os.path.dirname(default_file),
            initialfile=os.path.basename(default_file),
            filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")]
        )
        if not path:
            return

        try:
            with open(path, "r") as f:
                data = json.load(f)

            # Support both motor_config_gm3506.json format and lut_calibrations.json format
            if "lut_128" in data:
                errs = data["lut_128"]
                pole_zeros = data.get("pole_zeros", [])
                zero_ang = data.get("zero_electric_angle", None)
                lag_us = data.get("sensor_lag_comp_us", None)
            elif "motor1" in data and "lut_errors" in data["motor1"]:
                m1 = data["motor1"]
                errs = m1["lut_errors"]
                pole_zeros = m1.get("pole_zeros", [])
                zero_ang = m1.get("zero_electric_angle", None)
                lag_us = m1.get("sensor_lag_comp_us", None)
            else:
                messagebox.showerror("Invalid Format", "JSON file does not contain 'lut_128' or 'motor1.lut_errors'.")
                return

            if len(errs) != 128:
                messagebox.showerror("Size Error", f"Expected 128 LUT points, got {len(errs)}")
                return

            # 1. Transmit 8 lines of 16 comma-separated LUT floats
            for line_idx in range(8):
                chunk = errs[line_idx * 16 : (line_idx + 1) * 16]
                csv_str = ",".join(f"{x:.6f}" for x in chunk)
                self.serial_engine.send_command(f"LUT 1 {line_idx} {csv_str}")
                time.sleep(0.015)

            # 2. Transmit 11-pole zeros if present
            if len(pole_zeros) == 11:
                pz_csv = ",".join(f"{x:.5f}" for x in pole_zeros)
                self.serial_engine.send_command(f"POLE_ZEROS 1 {pz_csv}")
                time.sleep(0.015)

            # 3. Transmit zero electric angle & lag comp if present
            if zero_ang is not None:
                self.serial_engine.send_command(f"O1 {float(zero_ang):.4f}")
            if lag_us is not None:
                self.serial_engine.send_command(f"L1 {float(lag_us):.1f}")

            self._log_console(f"[SYS] Uploaded calibration profile from {os.path.basename(path)}\n")
        except Exception as ex:
            messagebox.showerror("Upload Error", str(ex))

    # =========================================================================
    # Telemetry & Console Callbacks
    # =========================================================================
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
                text=f"✅ {m} CALIBRATION SUCCESS (128-PT LUT + 11-POLE ZERO ACTIVE)", fg=COLOR_ACCENT_GREEN
            ))

    def _log_console(self, text):
        self.console_text.insert(tk.END, text)
        # Keep console buffer bounded
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
    # UI Periodic Update Loop (25 Hz)
    # =========================================================================
    def _ui_poll_loop(self):
        st = self.serial_engine.state

        if st["connected"] and (time.time() - st["last_packet_time"] < 1.0):
            # 1. Motor 1 Primary Telemetry
            self.m1_dial.update_angle(st["m1_raw_angle"])
            self.m1_act_rpm_lbl.configure(text=f"{st['m1_actual_rpm']:+06.1f} RPM")
            self.m1_tgt_rpm_lbl.configure(text=f"Target: {st['m1_target_rpm']:+06.1f} RPM")
            self.m1_uq_lbl.configure(text=f"Uq: {st['m1_uq_v']:+.2f} V")
            self.m1_raw_lbl.configure(text=f"Raw14: {st['m1_raw_angle']}")
            self.m1_zero_lbl.configure(text=f"Zero: {st['m1_zero_angle']:.4f} rad ({st['m1_direction']})")

            if st["m1_enabled"]:
                self.m1_status_badge.configure(text="ENABLED (CLOSED-LOOP)", bg="#1B5E20")
            else:
                self.m1_status_badge.configure(text="DISABLED (HI-Z)", bg="#37474F")

            self.m1_badge_parity.configure(
                text="PARITY: OK" if st["m1_parity_ok"] else "PARITY: ERR",
                bg="#1B5E20" if st["m1_parity_ok"] else COLOR_ACCENT_RED
            )
            self.m1_badge_mag.configure(
                text="MAG: OK" if st["m1_mag_ok"] else "MAG: WEAK",
                bg="#1B5E20" if st["m1_mag_ok"] else COLOR_ACCENT_AMBER
            )
            self.m1_badge_lut.configure(
                text="128-PT LUT: ON" if st["m1_lut_active"] else "LUT: OFF (RAW)",
                bg="#00838F" if st["m1_lut_active"] else "#37474F"
            )
            self.m1_badge_lat.configure(text=f"LAT: {st['isr_lat_us']} µs")

            # 2. Calibration Progress & Stage Status
            if st["m1_calib"]:
                self.m1_lut_progress["value"] = st["calib_progress"]
                if not self.is_master_calibrating:
                    self.calib_status_lbl.configure(
                        text=f"Calibrating ({st['calib_progress']}%): {st['m1_calib_stage']}"
                    )
            elif st["m1_lut_active"]:
                self.m1_lut_progress["value"] = 100
                if not self.is_master_calibrating and "GRADE" in st["m1_calib_grade"]:
                    self.calib_grade_badge.configure(text=st["m1_calib_grade"], bg="#059669")
            else:
                self.m1_lut_progress["value"] = 0

            # 3. Velocity Tracking Oscilloscope Redraw
            with self.serial_engine.lock:
                t_hist = list(self.serial_engine.history_target_rpm)
                a_hist = list(self.serial_engine.history_actual_rpm)
                u_hist = list(self.serial_engine.history_uq)
            self.scope_chart.redraw(t_hist, a_hist, u_hist)

            # 4. Encoder 2 Updates
            self.m2_dial.update_angle(st["m2_raw_angle"])
            self.m2_act_rpm_lbl.configure(text=f"{st['m2_actual_rpm']:+05.1f} RPM")
            self.m2_raw_lbl.configure(text=f"Raw: {st['m2_raw_angle']}")

            self.m2_badge_parity.configure(
                text="PARITY: OK" if st["m2_parity_ok"] else "PARITY: ERR",
                bg="#1B5E20" if st["m2_parity_ok"] else COLOR_ACCENT_RED
            )
            self.m2_badge_mag.configure(
                text="MAG: OK" if st["m2_mag_ok"] else "MAG: WEAK",
                bg="#1B5E20" if st["m2_mag_ok"] else COLOR_ACCENT_AMBER
            )
            self.m2_badge_lut.configure(
                text="LUT: ACTIVE" if st["m2_lut_active"] else "LUT: OFF",
                bg="#00838F" if st["m2_lut_active"] else "#37474F"
            )
            self.m2_badge_lat.configure(text=f"LAT: {st['isr_lat_us']} µs")

        self.after(40, self._ui_poll_loop)


def main():
    app = DualMotorDashboardApp()
    app.mainloop()


if __name__ == "__main__":
    main()
