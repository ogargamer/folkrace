"""
================================================================================
  AUTONOMOUS TOF PATH PLANNERS INTERACTIVE GUI & LIVE TUNER
  - Live 15mm C++ ToF Costmap Visualization & Hazard Boundary Overlay
  - Real-Time Switching between all 3 Planners:
      1. Option 1: Target-Guided BVP State Lattice (Heun RK2)
      2. Option 2: Curvilinear Slack-Constrained SL-MPC (OSQP)
      3. Option 3: G^3 Septic Spline Pool + Bidirectional Traction Profiler
  - Displays All Potential Candidate Trajectory Lines + Winning Best Path
  - Live Interactive Tuning of Parameters (Inflation Radius, Target Speed, Slip Angle)
  - Full Simulation Control: Pause, Play, Single-Step (20ms)
================================================================================
"""

import os
import sys
import math
import json
import time
import socket
import threading
import zlib
import base64
from typing import Dict, List, Optional, Tuple
import numpy as np

# Ensure root paths
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
MY_PROJECT_DIR = os.path.dirname(CONTROLLERS_DIR)
WORKSPACE_ROOT = os.path.dirname(MY_PROJECT_DIR)

for p in [WORKSPACE_ROOT, MY_PROJECT_DIR, CONTROLLERS_DIR, CURR_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF
from PyQt5.QtGui import QColor, QPainter, QPen, QBrush, QFont, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QComboBox, QPushButton, QCheckBox, QGroupBox,
    QDoubleSpinBox, QTabWidget, QSplitter, QStatusBar, QFrame, QGridLayout
)


# ==============================================================================
# 1. 2D COSTMAP & TRAJECTORY CANVAS
# ==============================================================================
class PlannerCostmapCanvas(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(560, 620)
        self.setMouseTracking(True)

        # Costmap local array metric bounds
        self.map_x_min = -1.25
        self.map_x_max = 1.25
        self.map_y_min = 0.00
        self.map_y_max = 2.00

        # Viewport metric bounds
        self.x_min = -1.6
        self.x_max = 1.6
        self.y_min = -0.4
        self.y_max = 2.3

        # Grid data
        self.prob_map: Optional[np.ndarray] = None
        self.hazard_mask: Optional[np.ndarray] = None
        self.tof_readings: Dict[str, float] = {}

        self.best_line: Optional[List[List[float]]] = None
        self.candidate_lines: Optional[List[List[List[float]]]] = None
        self.gap_info: Optional[List[float]] = None  # [xg, yg, width]
        self.targets: Optional[List[List[float]]] = None  # [[tx, ty, ap], ...]

        # Display preferences
        self.show_grid = True
        self.show_candidates = True
        self.show_best = True
        self.show_gap = True
        self.show_hazard = True
        self.show_chassis = True
        self.colormap = "Turbo"

    def set_data(
        self,
        prob_map: Optional[np.ndarray],
        hazard_mask: Optional[np.ndarray],
        tof_readings: dict,
        best_line: Optional[List[List[float]]],
        candidate_lines: Optional[List[List[List[float]]]],
        gap: Optional[List[float]],
        targets: Optional[List[List[float]]] = None,
        grid_bounds: Optional[List[float]] = None,
    ):
        self.prob_map = prob_map
        self.hazard_mask = hazard_mask
        self.tof_readings = tof_readings
        self.best_line = best_line
        self.candidate_lines = candidate_lines
        self.gap_info = gap
        self.targets = targets
        if grid_bounds and len(grid_bounds) == 4:
            self.map_x_min, self.map_x_max, self.map_y_min, self.map_y_max = grid_bounds
        self.update()

    def world_to_screen(self, xm: float, ym: float) -> QPointF:
        w = max(10.0, float(self.width()))
        h = max(10.0, float(self.height()))
        u = (xm - self.x_min) / (self.x_max - self.x_min)
        v = (ym - self.y_min) / (self.y_max - self.y_min)
        px = u * w
        py = (1.0 - v) * h
        return QPointF(px, py)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor(12, 16, 22))

        # 1. Draw Costmap Image
        if self.prob_map is not None:
            H, W = self.prob_map.shape
            img_data = np.zeros((H, W, 4), dtype=np.uint8)
            val = (np.clip(self.prob_map, 0.0, 1.0) * 255.0).astype(np.uint8)

            # Turbo colormap
            r = np.clip(np.sin(val / 255.0 * np.pi - np.pi / 2.0) * 127 + 128, 0, 255).astype(np.uint8)
            g = np.clip(np.sin(val / 255.0 * np.pi) * 255, 0, 255).astype(np.uint8)
            b = np.clip(np.cos(val / 255.0 * np.pi / 2.0) * 255, 0, 255).astype(np.uint8)
            img_data[..., 0] = r
            img_data[..., 1] = g
            img_data[..., 2] = b
            img_data[..., 3] = (self.prob_map * 220 + 35).astype(np.uint8)

            if self.show_hazard and self.hazard_mask is not None:
                img_data[self.hazard_mask > 0, 0] = 255
                img_data[self.hazard_mask > 0, 1] = 40
                img_data[self.hazard_mask > 0, 2] = 60
                img_data[self.hazard_mask > 0, 3] = 230

            img_data = np.flipud(img_data)
            qimg = QImage(img_data.tobytes(), W, H, W * 4, QImage.Format_RGBA8888)

            p_tl = self.world_to_screen(self.map_x_min, self.map_y_max)
            p_br = self.world_to_screen(self.map_x_max, self.map_y_min)
            dest_rect = QRectF(p_tl.x(), p_tl.y(), p_br.x() - p_tl.x(), p_br.y() - p_tl.y())
            painter.drawImage(dest_rect, qimg)

            painter.setPen(QPen(QColor(80, 120, 160, 90), 1, Qt.DotLine))
            painter.setBrush(Qt.NoBrush)
            painter.drawRect(dest_rect)

        # 2. Grid lines & range circles
        if self.show_grid:
            grid_pen = QPen(QColor(50, 70, 90, 110), 1, Qt.DashLine)
            painter.setPen(grid_pen)
            painter.setFont(QFont("Consolas", 8))
            for ym in [0.0, 0.5, 1.0, 1.5, 2.0]:
                p1 = self.world_to_screen(self.x_min, ym)
                p2 = self.world_to_screen(self.x_max, ym)
                painter.drawLine(p1, p2)
                painter.setPen(QPen(QColor(100, 150, 200, 180)))
                painter.drawText(QPointF(p1.x() + 8, p1.y() - 4), f"{ym:.1f}m")
                painter.setPen(grid_pen)

            for xm in [-1.0, -0.5, 0.0, 0.5, 1.0]:
                p1 = self.world_to_screen(xm, self.y_min)
                p2 = self.world_to_screen(xm, self.y_max)
                painter.drawLine(p1, p2)

            for ring_r in [0.5, 1.0, 1.5, 2.0]:
                r_tl = self.world_to_screen(-ring_r, ring_r)
                r_br = self.world_to_screen(ring_r, -ring_r)
                painter.setPen(QPen(QColor(60, 90, 120, 50), 1, Qt.DotLine))
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(QRectF(r_tl.x(), r_tl.y(), r_br.x() - r_tl.x(), r_br.y() - r_tl.y()))

        # 3. Candidate Potential Trajectory Lines (Cyan / Blue Translucent Fan)
        if self.show_candidates and self.candidate_lines:
            cand_pen = QPen(QColor(0, 200, 255, 75), 1.2, Qt.SolidLine)
            painter.setPen(cand_pen)
            for line in self.candidate_lines:
                if len(line) > 1:
                    for i in range(len(line) - 1):
                        p1 = self.world_to_screen(line[i][0], line[i][1])
                        p2 = self.world_to_screen(line[i + 1][0], line[i + 1][1])
                        painter.drawLine(p1, p2)

        # 4. Best / Winning Trajectory Line (Bright Gold / Yellow Solid Line)
        if self.show_best and self.best_line and len(self.best_line) > 1:
            best_pen = QPen(QColor(255, 215, 0, 240), 3.0, Qt.SolidLine)
            painter.setPen(best_pen)
            for i in range(len(self.best_line) - 1):
                p1 = self.world_to_screen(self.best_line[i][0], self.best_line[i][1])
                p2 = self.world_to_screen(self.best_line[i + 1][0], self.best_line[i + 1][1])
                painter.drawLine(p1, p2)

        # 5. Scouted Open Gap Target Marker & Multi-Target Candidates
        if self.show_gap and self.targets:
            # Draw secondary candidate targets first (translucent cyan)
            for cand in self.targets:
                ctx, cty = cand[0], cand[1]
                pc = self.world_to_screen(ctx, cty)
                painter.setPen(QPen(QColor(0, 220, 255, 180), 1.5, Qt.DashLine))
                painter.setBrush(QBrush(QColor(0, 220, 255, 40)))
                painter.drawEllipse(pc, 6, 6)

        if self.show_gap and self.gap_info:
            xg, yg, width = self.gap_info
            pg = self.world_to_screen(xg, yg)
            painter.setPen(QPen(QColor(0, 255, 128, 240), 2))
            painter.setBrush(QBrush(QColor(0, 255, 128, 90)))
            painter.drawEllipse(pg, 8, 8)
            # Aperture width bar
            p_left = self.world_to_screen(xg - width * 0.5, yg)
            p_right = self.world_to_screen(xg + width * 0.5, yg)
            painter.drawLine(p_left, p_right)

        # 6. Chassis Footprint & Origin
        if self.show_chassis:
            # Chassis dimensions: L=0.1325m wheelbase, W=0.10m track
            c_tl = self.world_to_screen(-0.065, 0.12)
            c_br = self.world_to_screen(0.065, -0.06)
            painter.setPen(QPen(QColor(255, 255, 255, 200), 1.8))
            painter.setBrush(QBrush(QColor(255, 255, 255, 35)))
            painter.drawRect(QRectF(c_tl.x(), c_tl.y(), c_br.x() - c_tl.x(), c_br.y() - c_tl.y()))

            # Origin center marker
            p0 = self.world_to_screen(0.0, 0.0)
            painter.setPen(QPen(QColor(255, 50, 50), 2))
            painter.setBrush(QBrush(QColor(255, 50, 50)))
            painter.drawEllipse(p0, 4, 4)


# ==============================================================================
# 2. MAIN COMPREHENSIVE GUI WINDOW
# ==============================================================================
class ComprehensivePlannerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Autonomous ToF Multi-Planner Studio & Live Tuner (50 Hz)")
        self.resize(1180, 780)

        # Network sockets
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.last_packet_time = 0.0
        self.udp_connected = False
        self.latest_pkt: Optional[dict] = None
        self._lock = threading.Lock()

        # Start UDP listener
        self.running = True
        self.listener_thread = threading.Thread(target=self._udp_worker, daemon=True)
        self.listener_thread.start()

        # Build UI
        self._build_ui()

        # Timer for UI updates (25 FPS / 40 ms matching 25 Hz telemetry)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_gui)
        self.timer.start(40)

    def _build_ui(self):
        central = QWidget(self)
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(12)

        # Left Canvas
        self.canvas = PlannerCostmapCanvas(self)
        layout.addWidget(self.canvas, stretch=3)

        # Right Controls Panel
        panel = QWidget()
        panel_layout = QVBoxLayout(panel)
        panel_layout.setContentsMargins(0, 0, 0, 0)
        panel_layout.setSpacing(10)

        # 1. PLANNER SELECTOR
        switch_group = QGroupBox("Active Path Planner (Live Switch)")
        switch_layout = QVBoxLayout(switch_group)

        self.cb_planner = QComboBox()
        self.cb_planner.addItem("Option 1: BVP State Lattice (Heun RK2)", "OPTION1_BVP")
        self.cb_planner.addItem("Option 2: Curvilinear SL-MPC (OSQP)", "OPTION2_SL_MPC")
        self.cb_planner.addItem("Option 3: G^3 Septic Spline + Traction", "OPTION3_SEPTIC")
        self.cb_planner.setStyleSheet("font-size: 13px; font-weight: bold; padding: 6px;")
        self.cb_planner.currentIndexChanged.connect(self._on_planner_selected)
        switch_layout.addWidget(self.cb_planner)

        # Check persisted planner choice on startup
        planner_state_file = os.path.join(CURR_DIR, "..", "tof_path_planner", "active_planner.json")
        if os.path.exists(planner_state_file):
            try:
                with open(planner_state_file, "r", encoding="utf-8") as f:
                    saved_p = json.load(f).get("planner")
                    idx = self.cb_planner.findData(saved_p)
                    if idx >= 0:
                        self.cb_planner.blockSignals(True)
                        self.cb_planner.setCurrentIndex(idx)
                        self.cb_planner.blockSignals(False)
            except Exception:
                pass

        btn_switch = QPushButton("⚡ Switch Active Planner in Webots")
        btn_switch.setStyleSheet("background-color: #00838f; color: white; font-weight: bold; padding: 6px;")
        btn_switch.clicked.connect(self._send_planner_switch)
        switch_layout.addWidget(btn_switch)
        panel_layout.addWidget(switch_group)

        # 2. SIMULATION CONTROLS
        sim_group = QGroupBox("Simulation Controls")
        sim_layout = QVBoxLayout(sim_group)

        btn_row = QHBoxLayout()
        self.btn_play_pause = QPushButton("⏸ Pause")
        self.btn_play_pause.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold; padding: 6px;")
        self.btn_play_pause.clicked.connect(self._toggle_pause)
        self.sim_paused = False

        self.btn_step = QPushButton("⏭ Step (8.3ms)")
        self.btn_step.setStyleSheet("background-color: #1565c0; color: white; font-weight: bold; padding: 6px;")
        self.btn_step.clicked.connect(self._send_step)
        btn_row.addWidget(self.btn_play_pause)
        btn_row.addWidget(self.btn_step)
        sim_layout.addLayout(btn_row)

        # Simulation Speed Multiplier (down to 0.05x)
        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("Sim Speed:"))
        self.cb_sim_speed = QComboBox()
        self.sim_speed_options = [
            ("0.05x (20x Slow-Mo)", 0.05),
            ("0.10x (10x Slow-Mo)", 0.10),
            ("0.25x (Quarter Speed)", 0.25),
            ("0.50x (Half Speed)", 0.50),
            ("1.00x (Real-Time 1x)", 1.00),
            ("2.00x (Double Speed)", 2.00),
            ("5.00x (High Speed)", 5.00),
            ("Max Speed (Uncapped)", 100.0),
        ]
        for label, val in self.sim_speed_options:
            self.cb_sim_speed.addItem(label, val)
        self.cb_sim_speed.setCurrentIndex(3)  # 0.50x default (Half Speed)
        self.cb_sim_speed.currentIndexChanged.connect(self._on_sim_speed_changed)
        speed_row.addWidget(self.cb_sim_speed)
        sim_layout.addLayout(speed_row)

        panel_layout.addWidget(sim_group)

        # 3. LIVE PARAMETER TUNING
        tune_group = QGroupBox("Live Parameter Tuning")
        tune_layout = QVBoxLayout(tune_group)

        # Inflation Radius (7.5cm bare minimum for 15cm chassis, 9.0cm default)
        tune_layout.addWidget(QLabel("Obstacle Inflation Radius (Bare Min 7.5 cm):"))
        inf_row = QHBoxLayout()
        self.slider_inflation = QSlider(Qt.Horizontal)
        self.slider_inflation.setRange(75, 140)  # 7.5 cm bare minimum to 14.0 cm
        self.slider_inflation.setValue(90)       # 9.0 cm default
        self.slider_inflation.valueChanged.connect(self._on_inflation_slider)
        self.lbl_inflation = QLabel("9.0 cm")
        self.lbl_inflation.setFont(QFont("Consolas", 10, QFont.Bold))
        inf_row.addWidget(self.slider_inflation)
        inf_row.addWidget(self.lbl_inflation)
        tune_layout.addLayout(inf_row)

        # Cruising Speed Ceiling
        tune_layout.addWidget(QLabel("Cruising Speed Ceiling (m/s):"))
        spd_row = QHBoxLayout()
        self.slider_speed = QSlider(Qt.Horizontal)
        self.slider_speed.setRange(8, 38)  # 0.8 to 3.8 m/s
        self.slider_speed.setValue(24)
        self.slider_speed.valueChanged.connect(self._on_speed_slider)
        self.lbl_speed = QLabel("2.40 m/s")
        self.lbl_speed.setFont(QFont("Consolas", 10, QFont.Bold))
        spd_row.addWidget(self.slider_speed)
        spd_row.addWidget(self.lbl_speed)
        tune_layout.addLayout(spd_row)

        # Peak Slip Angle
        tune_layout.addWidget(QLabel("Tire Peak Slip Angle (deg):"))
        slip_row = QHBoxLayout()
        self.slider_slip = QSlider(Qt.Horizontal)
        self.slider_slip.setRange(60, 200)  # 6.0 to 20.0 deg
        self.slider_slip.setValue(126)
        self.slider_slip.valueChanged.connect(self._on_slip_slider)
        self.lbl_slip = QLabel("12.6°")
        self.lbl_slip.setFont(QFont("Consolas", 10, QFont.Bold))
        slip_row.addWidget(self.slider_slip)
        slip_row.addWidget(self.lbl_slip)
        # Steering Architecture Mode Selector
        tune_layout.addWidget(QLabel("Steering Architecture Mode:"))
        self.cb_steering_mode = QComboBox()
        self.cb_steering_mode.addItem("Option 1: Two-Stage Midpoint", 1)
        self.cb_steering_mode.addItem("Option 2: Corridor-Anchored", 2)
        self.cb_steering_mode.addItem("Option 0: Classic Septic", 0)
        self.cb_steering_mode.setStyleSheet("background-color: #2b2b2b; color: #00e676; font-weight: bold; padding: 4px;")
        self.cb_steering_mode.currentIndexChanged.connect(self._on_steering_mode_changed)
        tune_layout.addWidget(self.cb_steering_mode)

        panel_layout.addWidget(tune_group)

        # 4. LIVE TELEMETRY
        telemetry_group = QGroupBox("Live Controller Telemetry (120 Hz / 60 Hz)")
        telem_layout = QVBoxLayout(telemetry_group)
        self.lbl_conn = QLabel("🔴 WAITING FOR WEBOTS...")
        self.lbl_conn.setStyleSheet("font-weight: bold; color: #ff5252;")
        self.lbl_dyn = QLabel("Speed: 0.00 m/s (tgt: 0.00) | Steer: +0.0°")
        self.lbl_dyn.setFont(QFont("Consolas", 9))
        self.lbl_gap = QLabel("Gap: (0.00, 0.00)m | Width: 0.0 cm")
        self.lbl_gap.setFont(QFont("Consolas", 9))
        self.lbl_perf = QLabel("Planner: 0.0 ms | Costmap: 0.0 ms | Status: --")
        self.lbl_perf.setFont(QFont("Consolas", 9))
        self.lbl_dist = QLabel("Distance Traveled: 0.00 m")
        self.lbl_dist.setFont(QFont("Consolas", 10, QFont.Bold))
        self.lbl_dist.setStyleSheet("color: #ffd54f;")

        telem_layout.addWidget(self.lbl_conn)
        telem_layout.addWidget(self.lbl_dyn)
        telem_layout.addWidget(self.lbl_gap)
        telem_layout.addWidget(self.lbl_perf)
        telem_layout.addWidget(self.lbl_dist)
        panel_layout.addWidget(telemetry_group)

        # 5. DISPLAY TOGGLES
        disp_group = QGroupBox("Layer Display Options")
        disp_layout = QGridLayout(disp_group)

        self.chk_cand = QCheckBox("Candidate Lines Fan")
        self.chk_cand.setChecked(True)
        self.chk_cand.toggled.connect(lambda v: setattr(self.canvas, "show_candidates", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_cand, 0, 0)

        self.chk_best = QCheckBox("Best Trajectory (Gold)")
        self.chk_best.setChecked(True)
        self.chk_best.toggled.connect(lambda v: setattr(self.canvas, "show_best", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_best, 0, 1)

        self.chk_hazard = QCheckBox("Hazard Mask (Red)")
        self.chk_hazard.setChecked(True)
        self.chk_hazard.toggled.connect(lambda v: setattr(self.canvas, "show_hazard", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_hazard, 1, 0)

        self.chk_gap = QCheckBox("Scouted Gap (Green)")
        self.chk_gap.setChecked(True)
        self.chk_gap.toggled.connect(lambda v: setattr(self.canvas, "show_gap", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_gap, 1, 1)

        panel_layout.addWidget(disp_group)

        panel_layout.addStretch()
        layout.addWidget(panel, stretch=1)

    def _udp_worker(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        bound = False
        for bind_ip in ["127.0.0.1", "0.0.0.0"]:
            try:
                sock.bind((bind_ip, 9876))
                sock.settimeout(0.5)
                bound = True
                print(f"[GUI UDP] Bound listening socket to {bind_ip}:9876", flush=True)
                break
            except Exception as e:
                print(f"[GUI UDP] Note: Could not bind {bind_ip}:9876: {e}", flush=True)

        if not bound:
            print("[GUI UDP] Critical: Failed to bind to port 9876 on any interface.", flush=True)
            return

        while self.running:
            try:
                data, _ = sock.recvfrom(65535)
                pkt = json.loads(data.decode("utf-8"))
                with self._lock:
                    self.latest_pkt = pkt
                    self.last_packet_time = time.time()
                    self.udp_connected = True
            except socket.timeout:
                if time.time() - self.last_packet_time > 1.5:
                    with self._lock:
                        self.udp_connected = False
            except Exception as e:
                # Log JSON or payload decoding errors
                print(f"[GUI UDP Error] {e}", flush=True)

    def _send_cmd(self, cmd_dict: dict):
        try:
            raw = json.dumps(cmd_dict).encode("utf-8")
            # Send to localhost on port 9877
            self.udp_sock.sendto(raw, ("127.0.0.1", 9877))
        except Exception as e:
            print(f"[GUI Send Error] {e}", flush=True)

    def _on_planner_selected(self, idx: int):
        p_type = self.cb_planner.itemData(idx)
        if p_type:
            print(f"[GUI] User selected planner in dropdown: {p_type}", flush=True)
            self._pending_planner_switch = p_type
            for _ in range(3):
                self._send_cmd({"cmd": "switch_planner", "planner": p_type})

    def _send_planner_switch(self):
        p_type = self.cb_planner.currentData()
        if p_type:
            print(f"[GUI] User clicked switch button for: {p_type}", flush=True)
            self._pending_planner_switch = p_type
            for _ in range(3):
                self._send_cmd({"cmd": "switch_planner", "planner": p_type})

    def _toggle_pause(self):
        self.sim_paused = not self.sim_paused
        if self.sim_paused:
            self.btn_play_pause.setText("▶ Resume")
            self.btn_play_pause.setStyleSheet("background-color: #f57f17; color: white; font-weight: bold; padding: 6px;")
            self._send_cmd({"cmd": "pause"})
        else:
            self.btn_play_pause.setText("⏸ Pause")
            self.btn_play_pause.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold; padding: 6px;")
            self._send_cmd({"cmd": "play"})

    def _send_step(self):
        self.sim_paused = True
        self.btn_play_pause.setText("▶ Resume")
        self.btn_play_pause.setStyleSheet("background-color: #f57f17; color: white; font-weight: bold; padding: 6px;")
        self._send_cmd({"cmd": "step"})

    def _on_sim_speed_changed(self, idx: int):
        speed_val = self.cb_sim_speed.currentData()
        self._send_cmd({"cmd": "set_speed", "val": speed_val})

    def _on_inflation_slider(self, val: int):
        r_m = val / 1000.0
        self.lbl_inflation.setText(f"{r_m*100:.1f} cm")
        self._send_cmd({"cmd": "set_param", "name": "r_inflation_m", "val": r_m})

    def _on_speed_slider(self, val: int):
        spd = val / 10.0
        self.lbl_speed.setText(f"{spd:.2f} m/s")
        self._send_cmd({"cmd": "set_param", "name": "target_speed", "val": spd})

    def _on_slip_slider(self, val: int):
        deg = val / 10.0
        rad = math.radians(deg)
        self.lbl_slip.setText(f"{deg:.1f}°")
        self._send_cmd({"cmd": "set_param", "name": "alpha_peak", "val": rad})

    def _on_steering_mode_changed(self, idx: int):
        mode_val = self.cb_steering_mode.currentData()
        if mode_val is not None:
            print(f"[GUI] User selected Steering Mode: {mode_val}", flush=True)
            self._send_cmd({"cmd": "set_param", "name": "steering_mode", "val": float(mode_val)})

    def _update_gui(self):
        pkt = None
        is_conn = False
        with self._lock:
            pkt = self.latest_pkt
            is_conn = self.udp_connected

        if is_conn:
            self.lbl_conn.setText("🟢 LIVE WEBOTS 50Hz STREAM CONNECTED")
            self.lbl_conn.setStyleSheet("font-weight: bold; color: #00e676;")
        else:
            self.lbl_conn.setText("🔴 WAITING FOR WEBOTS: Run Cyan_Champion_ToF_Track.wbt")
            self.lbl_conn.setStyleSheet("font-weight: bold; color: #ff5252;")

        if pkt is not None:
            # Decompress grid
            prob_map = None
            hazard_mask = None
            if "grid_data" in pkt and "grid_shape" in pkt:
                try:
                    shape = tuple(pkt["grid_shape"])
                    raw_b = zlib.decompress(base64.b64decode(pkt["grid_data"]))
                    prob_map = (np.frombuffer(raw_b, dtype=np.uint8).reshape(shape) / 255.0).astype(np.float32)

                    raw_h = zlib.decompress(base64.b64decode(pkt["hazard_data"]))
                    hazard_mask = np.frombuffer(raw_h, dtype=np.uint8).reshape(shape)
                except Exception:
                    pass

            best_line = pkt.get("best_line", [])
            cand_lines = pkt.get("candidate_lines", [])
            gap = pkt.get("gap", None)
            targets = pkt.get("targets", [])
            grid_bounds = pkt.get("grid_bounds", None)
            tof = pkt.get("tof", {})

            self.canvas.set_data(
                prob_map,
                hazard_mask,
                tof,
                best_line,
                cand_lines,
                gap,
                targets=targets,
                grid_bounds=grid_bounds,
            )

            # Update Labels
            p_name = pkt.get("planner", "--")
            v_est = pkt.get("v_est", 0.0)
            v_tgt = pkt.get("v_tgt", 0.0)
            steer = pkt.get("steer_deg", 0.0)
            dist = pkt.get("dist_m", 0.0)
            p_ms = pkt.get("plan_ms", 0.0)
            m_ms = pkt.get("map_ms", 0.0)
            status = pkt.get("status", "--")
            surviving = pkt.get("surviving", 0)

            # Sync combobox if planner was switched remotely and no local switch is pending
            idx = self.cb_planner.findData(p_name)
            pending = getattr(self, "_pending_planner_switch", None)
            if pending is not None:
                if p_name == pending:
                    self._pending_planner_switch = None
            elif idx >= 0 and self.cb_planner.currentIndex() != idx:
                self.cb_planner.blockSignals(True)
                self.cb_planner.setCurrentIndex(idx)
                self.cb_planner.blockSignals(False)

            self.lbl_dyn.setText(f"Speed: {v_est:.2f} m/s (tgt: {v_tgt:.2f}) | Steer: {steer:+5.1f}°")
            if gap and len(gap) >= 3:
                self.lbl_gap.setText(f"Gap: ({gap[0]:+.2f}, {gap[1]:.2f})m | Width: {gap[2]*100:.1f} cm")
            self.lbl_perf.setText(f"Plan: {p_ms:.2f}ms | Costmap: {m_ms:.2f}ms | Surviving: {surviving} | {status}")
            self.lbl_dist.setText(f"Distance Traveled: {dist:.2f} m ({p_name})")


def launch_gui():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    gui = ComprehensivePlannerGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    launch_gui()
