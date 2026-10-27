#!/usr/bin/env python3
"""
app.py
======
DA WINNA — STM32G474 Dual-Motor 20.0 kHz SimpleFOC Live Dashboard & Calibration Bench
Side-by-Side Dual-Channel Live Interface with 14-Bit Angle Dials, 26-Byte Binary Telemetry,
Stepped 128-Point LUT Calibration, and Live Motion Parameter Tuning.
"""

import sys
import os
import json
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

# Add current directory to path
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from serial_comm import STM32DualSerial
from plots import MotorAngleDial

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

class DualMotorDashboardApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("🏎️ DA WINNA — STM32G474 DUAL-MOTOR 20 kHz FOC BENCH (GM3506 @ 12V)")
        self.geometry("1340x890")
        self.minsize(1180, 800)
        self.configure(bg=COLOR_BG)

        # Serial Communication Engine
        self.serial_engine = STM32DualSerial(
            port="COM3",
            baud=921600,
            on_telemetry=self._on_telemetry,
            on_log=self._on_serial_log
        )

        self.cmd_history = []
        self.cmd_history_idx = -1
        self.log_paused = False

        self._init_styles()
        self._build_ui()

        # Keyboard Shortcuts
        self.bind("<Escape>", lambda e: self.global_estop())

        # Periodic UI update timer
        self.after(40, self._ui_poll_loop)

    def _init_styles(self):
        style = ttk.Style(self)
        style.theme_use("clam")

        # Configure generic TTK elements for dark theme
        style.configure(".", background=COLOR_BG, foreground=COLOR_TEXT)
        style.configure("TLabel", background=COLOR_BG, foreground=COLOR_TEXT, font=("Segoe UI", 10))
        style.configure("Card.TFrame", background=COLOR_CARD)
        style.configure("CardSec.TFrame", background=COLOR_CARD_SECONDARY)

        style.configure("TCombobox", fieldbackground=COLOR_CARD_SECONDARY, background=COLOR_CARD_SECONDARY, foreground=COLOR_TEXT)
        style.map("TCombobox", fieldbackground=[("readonly", COLOR_CARD_SECONDARY)], foreground=[("readonly", COLOR_TEXT)])

        style.configure("TProgressbar", thickness=8, background=COLOR_ACCENT_CYAN, troughcolor=COLOR_CARD_SECONDARY)

    def _build_ui(self):
        # ---------------------------------------------------------------------
        # 1. TOP HEADER & CONNECTION BAR
        # ---------------------------------------------------------------------
        top_bar = tk.Frame(self, bg=COLOR_CARD, height=52, bd=1, relief=tk.SOLID, highlightbackground=COLOR_BORDER)
        top_bar.pack(fill=tk.X, side=tk.TOP, padx=8, pady=(8, 4))
        top_bar.pack_propagate(False)

        # Title / Logo
        title_lbl = tk.Label(
            top_bar,
            text="🏎️ DA WINNA | STM32G474 DUAL-MOTOR 20.0 kHz FOC BENCH",
            font=("Segoe UI", 12, "bold"),
            bg=COLOR_CARD,
            fg=COLOR_ACCENT_CYAN
        )
        title_lbl.pack(side=tk.LEFT, padx=12)

        # Connection Controls
        conn_frame = tk.Frame(top_bar, bg=COLOR_CARD)
        conn_frame.pack(side=tk.LEFT, padx=16)

        tk.Label(conn_frame, text="Port:", bg=COLOR_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=(0, 4))
        self.port_combo = ttk.Combobox(conn_frame, width=12, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=2)
        self.refresh_ports()

        refresh_btn = tk.Button(
            conn_frame,
            text="🔄",
            command=self.refresh_ports,
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_TEXT,
            activebackground=COLOR_BORDER,
            bd=0,
            padx=6,
            pady=2
        )
        refresh_btn.pack(side=tk.LEFT, padx=2)

        tk.Label(conn_frame, text="Baud: 921600", bg=COLOR_CARD, fg=COLOR_ACCENT_GREEN, font=("Consolas", 9, "bold")).pack(side=tk.LEFT, padx=8)

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
        self.connect_btn.pack(side=tk.LEFT, padx=8)

        # System Status Pill
        self.sys_status_lbl = tk.Label(
            top_bar,
            text="● STANDBY (DISCONNECTED)",
            font=("Segoe UI", 9, "bold"),
            bg=COLOR_CARD,
            fg=COLOR_TEXT_MUTED
        )
        self.sys_status_lbl.pack(side=tk.LEFT, padx=12)

        # Prominent Red Global E-STOP
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

        # ---------------------------------------------------------------------
        # 2. MAIN WORKSPACE (Side-by-Side Dual Motor Columns)
        # ---------------------------------------------------------------------
        main_panes = tk.Frame(self, bg=COLOR_BG)
        main_panes.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        # Left Column: Motor 1
        self.m1_frame = self._build_motor_column(main_panes, motor_id=1, title="MOTOR 1 (LEFT CHANNEL)", accent=COLOR_ACCENT_CYAN)
        self.m1_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 4))

        # Right Column: Motor 2
        self.m2_frame = self._build_motor_column(main_panes, motor_id=2, title="MOTOR 2 (RIGHT CHANNEL)", accent=COLOR_ACCENT_GREEN)
        self.m2_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(4, 0))

        # ---------------------------------------------------------------------
        # 3. COUPLED MASTER CONTROLS BAR
        # ---------------------------------------------------------------------
        master_bar = tk.Frame(self, bg=COLOR_CARD, height=44, bd=1, relief=tk.SOLID, highlightbackground=COLOR_BORDER)
        master_bar.pack(fill=tk.X, side=tk.TOP, padx=8, pady=4)
        master_bar.pack_propagate(False)

        tk.Label(master_bar, text="🔗 COUPLED DUAL CONTROL:", font=("Segoe UI", 9, "bold"), bg=COLOR_CARD, fg=COLOR_ACCENT_AMBER).pack(side=tk.LEFT, padx=12)

        self.coupled_slider = tk.Scale(
            master_bar,
            from_=-600,
            to=600,
            orient=tk.HORIZONTAL,
            bg=COLOR_CARD,
            fg=COLOR_TEXT,
            troughcolor=COLOR_CARD_SECONDARY,
            highlightthickness=0,
            length=280,
            command=self._on_coupled_slider
        )
        self.coupled_slider.set(0)
        self.coupled_slider.pack(side=tk.LEFT, padx=8)

        tk.Button(master_bar, text="⚡ Enable Both (E)", command=lambda: self.serial_engine.send_command("E"), bg="#1B5E20", fg="#FFFFFF", bd=0, padx=10, pady=2, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=6)
        tk.Button(master_bar, text="Disable Both (D)", command=lambda: self.serial_engine.send_command("D"), bg="#37474F", fg="#FFFFFF", bd=0, padx=10, pady=2, font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=6)
        tk.Button(master_bar, text="Zero / Tare Both (Z)", command=lambda: self.serial_engine.send_command("Z"), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=0, padx=10, pady=2, font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=6)
        tk.Button(master_bar, text="Reset Defaults (R)", command=lambda: self.serial_engine.send_command("R"), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=0, padx=10, pady=2, font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=6)

        # ---------------------------------------------------------------------
        # 4. BOTTOM INTERACTIVE SERIAL CONSOLE
        # ---------------------------------------------------------------------
        bottom_frame = tk.Frame(self, bg=COLOR_CARD, height=160, bd=1, relief=tk.SOLID, highlightbackground=COLOR_BORDER)
        bottom_frame.pack(fill=tk.X, side=tk.BOTTOM, padx=8, pady=(2, 8))
        bottom_frame.pack_propagate(False)

        console_header = tk.Frame(bottom_frame, bg=COLOR_CARD)
        console_header.pack(fill=tk.X, padx=8, pady=(4, 2))

        tk.Label(console_header, text="INTERACTIVE SERIAL CONSOLE (Thread-Safe Demuxed Log Stream)", font=("Segoe UI", 9, "bold"), bg=COLOR_CARD, fg=COLOR_TEXT_MUTED).pack(side=tk.LEFT)

        self.pause_btn = tk.Button(console_header, text="Pause Log", command=self._toggle_log_pause, bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=0, padx=8, pady=1, font=("Segoe UI", 8))
        self.pause_btn.pack(side=tk.RIGHT, padx=4)

        tk.Button(console_header, text="Clear", command=self._clear_console, bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=0, padx=8, pady=1, font=("Segoe UI", 8)).pack(side=tk.RIGHT, padx=4)

        # Log Text Area
        self.console_text = tk.Text(
            bottom_frame,
            bg="#0B0E14",
            fg="#A0AEC0",
            font=("Consolas", 9),
            bd=0,
            highlightthickness=0,
            height=5
        )
        self.console_text.pack(fill=tk.BOTH, expand=True, padx=8, pady=2)

        # Console Input Bar
        cmd_bar = tk.Frame(bottom_frame, bg=COLOR_CARD)
        cmd_bar.pack(fill=tk.X, padx=8, pady=(2, 6))

        tk.Label(cmd_bar, text=">>>", bg=COLOR_CARD, fg=COLOR_ACCENT_CYAN, font=("Consolas", 10, "bold")).pack(side=tk.LEFT, padx=(0, 4))
        self.cmd_entry = tk.Entry(cmd_bar, bg="#0B0E14", fg=COLOR_TEXT, font=("Consolas", 10), insertbackground=COLOR_ACCENT_CYAN, bd=1, relief=tk.SOLID)
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.cmd_entry.bind("<Return>", self._send_console_cmd)
        self.cmd_entry.bind("<Up>", self._prev_cmd)
        self.cmd_entry.bind("<Down>", self._next_cmd)

        send_btn = tk.Button(cmd_bar, text="Send", command=self._send_console_cmd, bg="#00838F", fg="#FFFFFF", bd=0, padx=12, pady=1, font=("Segoe UI", 9, "bold"))
        send_btn.pack(side=tk.RIGHT, padx=2)

    def _build_motor_column(self, parent, motor_id, title, accent):
        frame = tk.Frame(parent, bg=COLOR_CARD, bd=1, relief=tk.SOLID, highlightbackground=COLOR_BORDER)

        # 1. Header Card
        hdr = tk.Frame(frame, bg=COLOR_CARD_SECONDARY, height=36)
        hdr.pack(fill=tk.X, side=tk.TOP)
        hdr.pack_propagate(False)

        tk.Label(hdr, text=title, font=("Segoe UI", 11, "bold"), bg=COLOR_CARD_SECONDARY, fg=accent).pack(side=tk.LEFT, padx=12)

        status_badge = tk.Label(hdr, text="DISABLED", font=("Consolas", 9, "bold"), bg="#37474F", fg="#FFFFFF", padx=8, pady=2)
        status_badge.pack(side=tk.RIGHT, padx=8)
        if motor_id == 1:
            self.m1_status_badge = status_badge
        else:
            self.m2_status_badge = status_badge

        # 2. Top Visual Dial & Real-Time Primary Readout
        dial_panel = tk.Frame(frame, bg=COLOR_CARD)
        dial_panel.pack(fill=tk.X, padx=8, pady=6)

        dial = MotorAngleDial(dial_panel, size=210, title=f"M{motor_id}", accent_color=accent)
        dial.pack(side=tk.LEFT, padx=6)
        if motor_id == 1:
            self.m1_dial = dial
        else:
            self.m2_dial = dial

        # Primary Telemetry Metrics Block
        metrics_box = tk.Frame(dial_panel, bg=COLOR_CARD_SECONDARY, bd=1, relief=tk.SOLID, highlightbackground=COLOR_BORDER)
        metrics_box.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=6, pady=2)

        # Speed Readout
        spd_row = tk.Frame(metrics_box, bg=COLOR_CARD_SECONDARY)
        spd_row.pack(fill=tk.X, padx=8, pady=(6, 2))
        tk.Label(spd_row, text="ACTUAL SPEED:", font=("Segoe UI", 8, "bold"), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT_MUTED).pack(anchor="w")
        act_rpm_lbl = tk.Label(spd_row, text="+0.0 RPM", font=("Consolas", 18, "bold"), bg=COLOR_CARD_SECONDARY, fg=COLOR_ACCENT_GREEN)
        act_rpm_lbl.pack(anchor="w")
        tgt_rpm_lbl = tk.Label(spd_row, text="Target: 0.0 RPM", font=("Consolas", 9), bg=COLOR_CARD_SECONDARY, fg=COLOR_ACCENT_CYAN)
        tgt_rpm_lbl.pack(anchor="w")

        # Electrical Voltages & Diagnostics
        volt_row = tk.Frame(metrics_box, bg=COLOR_CARD_SECONDARY)
        volt_row.pack(fill=tk.X, padx=8, pady=2)
        uq_lbl = tk.Label(volt_row, text="Uq: 0.00 V", font=("Consolas", 9), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT)
        uq_lbl.pack(side=tk.LEFT, padx=(0, 10))
        raw_lbl = tk.Label(volt_row, text="Raw: 0", font=("Consolas", 9), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT_MUTED)
        raw_lbl.pack(side=tk.LEFT)

        # Health & Performance Badges
        badge_row1 = tk.Frame(metrics_box, bg=COLOR_CARD_SECONDARY)
        badge_row1.pack(fill=tk.X, padx=8, pady=(4, 2))
        badge_parity = tk.Label(badge_row1, text="PARITY: OK", font=("Consolas", 8, "bold"), bg="#1B5E20", fg="#FFFFFF", padx=4)
        badge_parity.pack(side=tk.LEFT, padx=2)
        badge_mag = tk.Label(badge_row1, text="MAG: OK", font=("Consolas", 8, "bold"), bg="#1B5E20", fg="#FFFFFF", padx=4)
        badge_mag.pack(side=tk.LEFT, padx=2)
        badge_lut = tk.Label(badge_row1, text="LUT: OFF", font=("Consolas", 8, "bold"), bg="#37474F", fg="#B0BEC5", padx=4)
        badge_lut.pack(side=tk.LEFT, padx=2)

        badge_row2 = tk.Frame(metrics_box, bg=COLOR_CARD_SECONDARY)
        badge_row2.pack(fill=tk.X, padx=8, pady=(2, 6))
        badge_foc = tk.Label(badge_row2, text="20.0 kHz", font=("Consolas", 8, "bold"), bg="#004D40", fg=COLOR_ACCENT_CYAN, padx=4)
        badge_foc.pack(side=tk.LEFT, padx=2)
        badge_lat = tk.Label(badge_row2, text="LAT: 12 µs", font=("Consolas", 8, "bold"), bg="#263238", fg="#ECEFF1", padx=4)
        badge_lat.pack(side=tk.LEFT, padx=2)

        if motor_id == 1:
            self.m1_act_rpm_lbl = act_rpm_lbl
            self.m1_tgt_rpm_lbl = tgt_rpm_lbl
            self.m1_uq_lbl = uq_lbl
            self.m1_raw_lbl = raw_lbl
            self.m1_badge_parity = badge_parity
            self.m1_badge_mag = badge_mag
            self.m1_badge_lut = badge_lut
            self.m1_badge_foc = badge_foc
            self.m1_badge_lat = badge_lat
        else:
            self.m2_act_rpm_lbl = act_rpm_lbl
            self.m2_tgt_rpm_lbl = tgt_rpm_lbl
            self.m2_uq_lbl = uq_lbl
            self.m2_raw_lbl = raw_lbl
            self.m2_badge_parity = badge_parity
            self.m2_badge_mag = badge_mag
            self.m2_badge_lut = badge_lut
            self.m2_badge_foc = badge_foc
            self.m2_badge_lat = badge_lat

        # 3. Direct Speed & Control Preset Buttons
        ctrl_sec = tk.LabelFrame(frame, text=f"SPEED PRESETS & CONTROLS (MOTOR {motor_id})", bg=COLOR_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8, "bold"))
        ctrl_sec.pack(fill=tk.X, padx=8, pady=4)

        btn_row1 = tk.Frame(ctrl_sec, bg=COLOR_CARD)
        btn_row1.pack(fill=tk.X, padx=4, pady=2)
        for spd in [0, 50, 100, 200, 400]:
            btn = tk.Button(
                btn_row1,
                text=f"{spd} RPM",
                command=lambda s=spd, m=motor_id: self._set_motor_speed(m, s),
                bg=COLOR_CARD_SECONDARY,
                fg=COLOR_TEXT,
                bd=0,
                padx=6,
                pady=2,
                font=("Segoe UI", 8)
            )
            btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        btn_row2 = tk.Frame(ctrl_sec, bg=COLOR_CARD)
        btn_row2.pack(fill=tk.X, padx=4, pady=2)
        for spd in [-50, -100, -200, -400]:
            btn = tk.Button(
                btn_row2,
                text=f"{spd} RPM",
                command=lambda s=spd, m=motor_id: self._set_motor_speed(m, s),
                bg=COLOR_CARD_SECONDARY,
                fg=COLOR_TEXT_MUTED,
                bd=0,
                padx=6,
                pady=2,
                font=("Segoe UI", 8)
            )
            btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # Single Motor Enable / Align Action Row
        act_row = tk.Frame(ctrl_sec, bg=COLOR_CARD)
        act_row.pack(fill=tk.X, padx=4, pady=(4, 2))
        tk.Button(act_row, text=f"Enable M{motor_id}", command=lambda: self.serial_engine.send_command(f"E{motor_id}"), bg="#1B5E20", fg="#FFFFFF", bd=0, padx=8, pady=2, font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(act_row, text=f"Disable M{motor_id}", command=lambda: self.serial_engine.send_command(f"D{motor_id}"), bg="#37474F", fg="#FFFFFF", bd=0, padx=8, pady=2, font=("Segoe UI", 8)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(act_row, text=f"Align M{motor_id}", command=lambda: self.serial_engine.send_command(f"C{motor_id}"), bg="#00838F", fg="#FFFFFF", bd=0, padx=8, pady=2, font=("Segoe UI", 8)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(act_row, text=f"Tare M{motor_id}", command=lambda: self.serial_engine.send_command(f"Z{motor_id}"), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=0, padx=8, pady=2, font=("Segoe UI", 8)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # 4. Live Motion Parameter Tuning Card
        tune_sec = tk.LabelFrame(frame, text=f"LIVE MOTION PARAMETER TUNING (M{motor_id})", bg=COLOR_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8, "bold"))
        tune_sec.pack(fill=tk.X, padx=8, pady=4)

        t_row1 = tk.Frame(tune_sec, bg=COLOR_CARD)
        t_row1.pack(fill=tk.X, padx=4, pady=2)
        p_ent = self._make_tune_field(t_row1, "P:", "0.120", width=6)
        i_ent = self._make_tune_field(t_row1, "I:", "1.200", width=6)
        d_ent = self._make_tune_field(t_row1, "D:", "0.0010", width=7)
        tf_ent = self._make_tune_field(t_row1, "Tf(s):", "0.015", width=6)

        t_row2 = tk.Frame(tune_sec, bg=COLOR_CARD)
        t_row2.pack(fill=tk.X, padx=4, pady=2)
        v_ent = self._make_tune_field(t_row2, "V-Lim(V):", "12.0", width=6)

        apply_btn = tk.Button(
            t_row2,
            text=f"Apply M{motor_id} Tuning",
            command=lambda: self._apply_tuning(motor_id, p_ent.get(), i_ent.get(), d_ent.get(), tf_ent.get(), v_ent.get()),
            bg="#00838F",
            fg="#FFFFFF",
            bd=0,
            padx=10,
            pady=1,
            font=("Segoe UI", 8, "bold")
        )
        apply_btn.pack(side=tk.RIGHT, padx=4)

        rst_btn = tk.Button(
            t_row2,
            text="⚡ Reset Defaults",
            command=lambda: self.serial_engine.send_command(f"R{motor_id}"),
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_TEXT_MUTED,
            bd=0,
            padx=8,
            pady=1,
            font=("Segoe UI", 8)
        )
        rst_btn.pack(side=tk.RIGHT, padx=4)

        # 5. Stepped 128-Point LUT Calibration Card
        lut_sec = tk.LabelFrame(frame, text=f"128-POINT STEPPED LUT CALIBRATION (M{motor_id})", bg=COLOR_CARD, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8, "bold"))
        lut_sec.pack(fill=tk.X, padx=8, pady=4)

        lut_bar = ttk.Progressbar(lut_sec, orient=tk.HORIZONTAL, length=100, mode='determinate')
        lut_bar.pack(fill=tk.X, padx=6, pady=4)
        if motor_id == 1:
            self.m1_lut_progress = lut_bar
        else:
            self.m2_lut_progress = lut_bar

        lut_btn_row = tk.Frame(lut_sec, bg=COLOR_CARD)
        lut_btn_row.pack(fill=tk.X, padx=4, pady=(2, 6))

        tk.Button(
            lut_btn_row,
            text=f"Start M{motor_id} Calib",
            command=lambda: self.serial_engine.send_command(f"CALIB {motor_id}"),
            bg="#E65100",
            fg="#FFFFFF",
            bd=0,
            padx=8,
            pady=2,
            font=("Segoe UI", 8, "bold")
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        tk.Button(
            lut_btn_row,
            text="⚡ Upload Saved LUT",
            command=lambda: self._upload_saved_lut(motor_id),
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_ACCENT_CYAN,
            bd=0,
            padx=8,
            pady=2,
            font=("Segoe UI", 8)
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        tk.Button(
            lut_btn_row,
            text="💾 Save to JSON",
            command=lambda: self._save_lut_profile(motor_id),
            bg=COLOR_CARD_SECONDARY,
            fg=COLOR_TEXT_MUTED,
            bd=0,
            padx=8,
            pady=2,
            font=("Segoe UI", 8)
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        return frame

    def _make_tune_field(self, parent, label_text, default_val, width=6):
        box = tk.Frame(parent, bg=COLOR_CARD)
        box.pack(side=tk.LEFT, padx=3)
        tk.Label(box, text=label_text, font=("Segoe UI", 8), bg=COLOR_CARD, fg=COLOR_TEXT_MUTED).pack(side=tk.LEFT)
        ent = tk.Entry(box, width=width, font=("Consolas", 8), bg=COLOR_CARD_SECONDARY, fg=COLOR_TEXT, bd=1, relief=tk.SOLID, insertbackground=COLOR_ACCENT_CYAN)
        ent.insert(0, default_val)
        ent.pack(side=tk.LEFT, padx=1)
        return ent

    # -------------------------------------------------------------------------
    # UI Control Handlers
    # -------------------------------------------------------------------------
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
            else:
                messagebox.showerror("Connection Error", msg)
        else:
            self.serial_engine.disconnect()
            self.connect_btn.configure(text="CONNECT", bg="#00838F")
            self.sys_status_lbl.configure(text="● STANDBY (DISCONNECTED)", fg=COLOR_TEXT_MUTED)

    def global_estop(self):
        self.serial_engine.send_command("STOP")
        self.coupled_slider.set(0)
        self.sys_status_lbl.configure(text="🛑 GLOBAL E-STOP TRIGGERED", fg=COLOR_ACCENT_RED)

    def _set_motor_speed(self, motor_id, spd):
        self.serial_engine.send_command(f"T{motor_id} {spd}")

    def _on_coupled_slider(self, val):
        self.serial_engine.send_command(f"T {val}")

    def _apply_tuning(self, motor_id, p_val, i_val, d_val, tf_val, v_val):
        try:
            p = float(p_val)
            i = float(i_val)
            d = float(d_val)
            tf = float(tf_val)
            v = float(v_val)
            self.serial_engine.send_command(f"P{motor_id} {p}")
            self.serial_engine.send_command(f"I{motor_id} {i}")
            self.serial_engine.send_command(f"D{motor_id} {d}")
            self.serial_engine.send_command(f"F{motor_id} {tf}")
            self.serial_engine.send_command(f"V{motor_id} {v}")
        except ValueError:
            messagebox.showerror("Format Error", "Tuning values must be valid floating point numbers.")

    def _upload_saved_lut(self, motor_id):
        if not os.path.exists(LUT_JSON_PATH):
            messagebox.showwarning("File Missing", f"No saved LUT found at {LUT_JSON_PATH}")
            return
        try:
            with open(LUT_JSON_PATH, "r") as f:
                data = json.load(f)
            key = f"motor{motor_id}"
            if key not in data or "lut_errors" not in data[key]:
                messagebox.showwarning("Key Missing", f"No '{key}' calibration profile in JSON.")
                return

            errs = data[key]["lut_errors"]
            if len(errs) != 128:
                messagebox.showerror("Size Error", f"Expected 128 LUT points, got {len(errs)}")
                return

            # Transmit 8 lines of 16 comma-separated floats
            for line_idx in range(8):
                chunk = errs[line_idx * 16 : (line_idx + 1) * 16]
                csv_str = ",".join(f"{x:.6f}" for x in chunk)
                cmd = f"LUT {motor_id} {line_idx} {csv_str}"
                self.serial_engine.send_command(cmd)
                time.sleep(0.015)

            self._log_console(f"[SYS] Uploaded 128-point LUT curve for Motor {motor_id}\n")
        except Exception as ex:
            messagebox.showerror("Upload Error", str(ex))

    def _save_lut_profile(self, motor_id):
        # Stub for saving profile
        messagebox.showinfo("Saved", f"Profile template active in {LUT_JSON_PATH}")

    # -------------------------------------------------------------------------
    # Telemetry & Console Callbacks
    # -------------------------------------------------------------------------
    def _on_telemetry(self, state):
        # Telemetry state is stored in serial_engine.state; UI polls periodically
        pass

    def _on_serial_log(self, text):
        if not self.log_paused:
            self._log_console(text)

    def _log_console(self, text):
        self.console_text.insert(tk.END, text)
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

    # -------------------------------------------------------------------------
    # UI Periodic Update Loop (25 Hz)
    # -------------------------------------------------------------------------
    def _ui_poll_loop(self):
        st = self.serial_engine.state

        if st["connected"] and (time.time() - st["last_packet_time"] < 1.0):
            # Motor 1 Updates
            self.m1_dial.update_angle(st["m1_raw_angle"])
            self.m1_act_rpm_lbl.configure(text=f"{st['m1_actual_rpm']:+05.1f} RPM")
            self.m1_tgt_rpm_lbl.configure(text=f"Target: {st['m1_target_rpm']:+05.1f} RPM")
            self.m1_uq_lbl.configure(text=f"Uq: {st['m1_uq_v']:+.2f} V")
            self.m1_raw_lbl.configure(text=f"Raw: {st['m1_raw_angle']}")

            if st["m1_enabled"]:
                self.m1_status_badge.configure(text="ENABLED", bg="#1B5E20")
            else:
                self.m1_status_badge.configure(text="DISABLED", bg="#37474F")

            self.m1_badge_parity.configure(text="PARITY: OK" if st["m1_parity_ok"] else "PARITY: ERR", bg="#1B5E20" if st["m1_parity_ok"] else COLOR_ACCENT_RED)
            self.m1_badge_mag.configure(text="MAG: OK" if st["m1_mag_ok"] else "MAG: WEAK", bg="#1B5E20" if st["m1_mag_ok"] else COLOR_ACCENT_AMBER)
            self.m1_badge_lut.configure(text="LUT: ACTIVE" if st["m1_lut_active"] else "LUT: OFF", bg="#00838F" if st["m1_lut_active"] else "#37474F")
            self.m1_badge_lat.configure(text=f"LAT: {st['isr_lat_us']} µs")

            # Motor 2 Updates
            self.m2_dial.update_angle(st["m2_raw_angle"])
            self.m2_act_rpm_lbl.configure(text=f"{st['m2_actual_rpm']:+05.1f} RPM")
            self.m2_tgt_rpm_lbl.configure(text=f"Target: {st['m2_target_rpm']:+05.1f} RPM")
            self.m2_uq_lbl.configure(text=f"Uq: {st['m2_uq_v']:+.2f} V")
            self.m2_raw_lbl.configure(text=f"Raw: {st['m2_raw_angle']}")

            if st["m2_enabled"]:
                self.m2_status_badge.configure(text="ENABLED", bg="#1B5E20")
            else:
                self.m2_status_badge.configure(text="DISABLED", bg="#37474F")

            self.m2_badge_parity.configure(text="PARITY: OK" if st["m2_parity_ok"] else "PARITY: ERR", bg="#1B5E20" if st["m2_parity_ok"] else COLOR_ACCENT_RED)
            self.m2_badge_mag.configure(text="MAG: OK" if st["m2_mag_ok"] else "MAG: WEAK", bg="#1B5E20" if st["m2_mag_ok"] else COLOR_ACCENT_AMBER)
            self.m2_badge_lut.configure(text="LUT: ACTIVE" if st["m2_lut_active"] else "LUT: OFF", bg="#00838F" if st["m2_lut_active"] else "#37474F")
            self.m2_badge_lat.configure(text=f"LAT: {st['isr_lat_us']} µs")

            # Calibration Progress Bars
            if st["m1_calib"]:
                self.m1_lut_progress["value"] = st["calib_progress"]
            if st["m2_calib"]:
                self.m2_lut_progress["value"] = st["calib_progress"]

        self.after(40, self._ui_poll_loop)

def main():
    app = DualMotorDashboardApp()
    app.mainloop()

if __name__ == "__main__":
    main()
