"""
================================================================================
  C++ SEPTIC SPLINE LIVE PREVIEW MAP & REAL-TIME INTERACTIVE TUNER (50 Hz)
================================================================================
  - Dedicated real-time 15mm C++ ToF Costmap Visualization & Hazard Overlay.
  - Exclusively tailored for OPTION 3: C++ Septic Spline Planner.
  - Live Septic Trajectory Spline Fan & Gold Winning Path Rendering.
  - Real-Time Bidirectional Parameter Tuning:
      * r_inflation_m (Obstacle Inflation Radius)
      * v_entry_max (Max Cruising Speed Ceiling)
      * v_cruise_min (Corner Entry Min Speed Floor)
      * kappa_smooth_gain (Curvature Smoothness Weight)
      * w_clearance (Obstacle Clearance Bias Weight)
      * w_steer_effort (Steering Stability / Damping Weight)
  - UDP 50 Hz Streaming Bridge connected to Webots champion_live_preview.
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
from typing import Dict, List, Optional, Tuple, Any
import numpy as np

WORKSPACE_ROOT = os.path.dirname(os.path.abspath(__file__))
GA_OPTIMIZER_DIR = os.path.join(WORKSPACE_ROOT, "ga_planner_optimizer")
PLANNERS_DIR = os.path.join(GA_OPTIMIZER_DIR, "planners")

for p in [WORKSPACE_ROOT, GA_OPTIMIZER_DIR, PLANNERS_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF
from PyQt5.QtGui import QColor, QPainter, QPen, QBrush, QFont, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QComboBox, QPushButton, QCheckBox, QGroupBox,
    QGridLayout, QFrame, QScrollArea, QDoubleSpinBox, QSplitter, QStatusBar
)


# ==============================================================================
# 1. 2D COSTMAP & SEPTIC TRAJECTORY CANVAS
# ==============================================================================
class SepticCostmapCanvas(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(580, 640)
        self.setMouseTracking(True)

        # Costmap metric grid boundaries
        self.map_x_min = -1.00
        self.map_x_max = 1.00
        self.map_y_min = -0.50
        self.map_y_max = 2.00

        # Viewport display range (vehicle centered near bottom)
        self.x_min = -1.40
        self.x_max = 1.40
        self.y_min = -0.60
        self.y_max = 2.20

        # Dynamic sensor and trajectory frame data
        self.prob_map: Optional[np.ndarray] = None
        self.hazard_mask: Optional[np.ndarray] = None
        self.tof_readings: Dict[str, float] = {}

        self.best_line: Optional[List[List[float]]] = None
        self.candidate_lines: Optional[List[List[List[float]]]] = None
        self.gap_info: Optional[List[float]] = None
        self.targets: Optional[List[List[float]]] = None

        # Visual layer toggles
        self.show_grid = True
        self.show_candidates = True
        self.show_best = True
        self.show_gap = True
        self.show_hazard = True
        self.show_chassis = True

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
        painter.fillRect(self.rect(), QColor(10, 14, 20))

        # 1. Draw Costmap Image Overlay
        if self.prob_map is not None:
            H, W = self.prob_map.shape
            img_data = np.zeros((H, W, 4), dtype=np.uint8)
            val = (np.clip(self.prob_map, 0.0, 1.0) * 255.0).astype(np.uint8)

            # High-contrast Turbo color mapping
            r = np.clip(np.sin(val / 255.0 * np.pi - np.pi / 2.0) * 127 + 128, 0, 255).astype(np.uint8)
            g = np.clip(np.sin(val / 255.0 * np.pi) * 255, 0, 255).astype(np.uint8)
            b = np.clip(np.cos(val / 255.0 * np.pi / 2.0) * 255, 0, 255).astype(np.uint8)
            img_data[..., 0] = r
            img_data[..., 1] = g
            img_data[..., 2] = b
            img_data[..., 3] = (self.prob_map * 210 + 35).astype(np.uint8)

            if self.show_hazard and self.hazard_mask is not None:
                img_data[self.hazard_mask > 0, 0] = 255
                img_data[self.hazard_mask > 0, 1] = 30
                img_data[self.hazard_mask > 0, 2] = 50
                img_data[self.hazard_mask > 0, 3] = 230

            img_data = np.flipud(img_data)
            qimg = QImage(img_data.tobytes(), W, H, W * 4, QImage.Format_RGBA8888)

            p_tl = self.world_to_screen(self.map_x_min, self.map_y_max)
            p_br = self.world_to_screen(self.map_x_max, self.map_y_min)
            dest_rect = QRectF(p_tl.x(), p_tl.y(), p_br.x() - p_tl.x(), p_br.y() - p_tl.y())
            painter.drawImage(dest_rect, qimg)

            painter.setPen(QPen(QColor(70, 100, 130, 90), 1, Qt.DotLine))
            painter.setBrush(Qt.NoBrush)
            painter.drawRect(dest_rect)

        # 2. Grid lines & range distance rings
        if self.show_grid:
            grid_pen = QPen(QColor(45, 60, 80, 110), 1, Qt.DashLine)
            painter.setPen(grid_pen)
            painter.setFont(QFont("Consolas", 8))
            for ym in [-0.5, 0.0, 0.5, 1.0, 1.5, 2.0]:
                p1 = self.world_to_screen(self.x_min, ym)
                p2 = self.world_to_screen(self.x_max, ym)
                painter.drawLine(p1, p2)
                painter.setPen(QPen(QColor(100, 160, 220, 180)))
                painter.drawText(QPointF(p1.x() + 6, p1.y() - 3), f"{ym:+.1f}m")
                painter.setPen(grid_pen)

            for xm in [-1.0, -0.5, 0.0, 0.5, 1.0]:
                p1 = self.world_to_screen(xm, self.y_min)
                p2 = self.world_to_screen(xm, self.y_max)
                painter.drawLine(p1, p2)

            for ring_r in [0.5, 1.0, 1.5, 2.0]:
                r_tl = self.world_to_screen(-ring_r, ring_r)
                r_br = self.world_to_screen(ring_r, -ring_r)
                painter.setPen(QPen(QColor(50, 80, 110, 50), 1, Qt.DotLine))
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(QRectF(r_tl.x(), r_tl.y(), r_br.x() - r_tl.x(), r_br.y() - r_tl.y()))

        # 3. Septic Candidate Spline Fan (Cyan / Translucent Blue)
        if self.show_candidates and self.candidate_lines:
            cand_pen = QPen(QColor(0, 210, 255, 80), 1.2, Qt.SolidLine)
            painter.setPen(cand_pen)
            for line in self.candidate_lines:
                if len(line) > 1:
                    for i in range(len(line) - 1):
                        p1 = self.world_to_screen(line[i][0], line[i][1])
                        p2 = self.world_to_screen(line[i + 1][0], line[i + 1][1])
                        painter.drawLine(p1, p2)

        # 4. Winning Best C++ Septic Spline Trajectory (Bright Gold)
        if self.show_best and self.best_line and len(self.best_line) > 1:
            best_pen = QPen(QColor(255, 215, 0, 245), 3.2, Qt.SolidLine)
            painter.setPen(best_pen)
            for i in range(len(self.best_line) - 1):
                p1 = self.world_to_screen(self.best_line[i][0], self.best_line[i][1])
                p2 = self.world_to_screen(self.best_line[i + 1][0], self.best_line[i + 1][1])
                painter.drawLine(p1, p2)

        # 5. Scouted Gap Targets & Aperture Width
        if self.show_gap and self.targets:
            for cand in self.targets:
                ctx, cty = cand[0], cand[1]
                pc = self.world_to_screen(ctx, cty)
                painter.setPen(QPen(QColor(0, 230, 255, 180), 1.5, Qt.DashLine))
                painter.setBrush(QBrush(QColor(0, 230, 255, 40)))
                painter.drawEllipse(pc, 6, 6)

        if self.show_gap and self.gap_info:
            xg, yg, width = self.gap_info
            pg = self.world_to_screen(xg, yg)
            painter.setPen(QPen(QColor(0, 255, 130, 240), 2))
            painter.setBrush(QBrush(QColor(0, 255, 130, 90)))
            painter.drawEllipse(pg, 8, 8)
            p_left = self.world_to_screen(xg - width * 0.5, yg)
            p_right = self.world_to_screen(xg + width * 0.5, yg)
            painter.drawLine(p_left, p_right)

        # 6. Chassis Footprint & Origin Anchor
        if self.show_chassis:
            c_tl = self.world_to_screen(-0.065, 0.12)
            c_br = self.world_to_screen(0.065, -0.06)
            painter.setPen(QPen(QColor(255, 255, 255, 210), 1.8))
            painter.setBrush(QBrush(QColor(255, 255, 255, 30)))
            painter.drawRect(QRectF(c_tl.x(), c_tl.y(), c_br.x() - c_tl.x(), c_br.y() - c_tl.y()))

            p0 = self.world_to_screen(0.0, 0.0)
            painter.setPen(QPen(QColor(255, 50, 50), 2))
            painter.setBrush(QBrush(QColor(255, 50, 50)))
            painter.drawEllipse(p0, 4, 4)


# ==============================================================================
# 2. MAIN C++ SEPTIC LIVE PREVIEW GUI WINDOW
# ==============================================================================
class SepticLivePreviewGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Live Preview Map - C++ Septic Spline Trajectory Engine (50 Hz)")
        self.resize(1200, 800)

        # Network UDP sockets for telemetry & live tuning
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.last_packet_time = 0.0
        self.udp_connected = False
        self.latest_pkt: Optional[dict] = None
        self._lock = threading.Lock()

        # Start UDP listener on Port 9876
        self.running = True
        self.listener_thread = threading.Thread(target=self._udp_worker, daemon=True)
        self.listener_thread.start()

        # Genome path for live reading/saving
        self.genome_file = os.path.join(GA_OPTIMIZER_DIR, "best_planner_genome.json")
        self.loaded_genes = self._load_current_genes()

        # Build UI layout
        self._build_ui()

        # Timer for UI rendering (30 FPS)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_gui)
        self.timer.start(33)

    def _load_current_genes(self) -> Dict[str, Any]:
        if os.path.exists(self.genome_file):
            try:
                with open(self.genome_file, "r", encoding="utf-8") as f:
                    d = json.load(f)
                    return d.get("genes", {})
            except Exception as e:
                print(f"[GUI] Failed to read {self.genome_file}: {e}")
        return {}

    def _save_gene_to_file(self, gene_name: str, val: float):
        if not os.path.exists(self.genome_file):
            return
        try:
            with open(self.genome_file, "r", encoding="utf-8") as f:
                d = json.load(f)
            if "genes" not in d:
                d["genes"] = {}
            d["genes"][gene_name] = float(val)
            with open(self.genome_file, "w", encoding="utf-8") as f:
                json.dump(d, f, indent=2)
            print(f"[GUI File Sync] Updated {gene_name} = {val} in best_planner_genome.json", flush=True)
        except Exception as e:
            print(f"[GUI File Sync Error] {e}", flush=True)

    def _build_ui(self):
        central = QWidget(self)
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(14)

        # Left: Costmap & Trajectory Canvas
        self.canvas = SepticCostmapCanvas(self)
        layout.addWidget(self.canvas, stretch=3)

        # Right: Septic Controls & Tuning Panel
        panel = QWidget()
        panel_layout = QVBoxLayout(panel)
        panel_layout.setContentsMargins(0, 0, 0, 0)
        panel_layout.setSpacing(12)

        # 1. LIVE TELEMETRY & STATUS
        telemetry_group = QGroupBox("C++ Septic Engine Telemetry (50 Hz)")
        telem_layout = QVBoxLayout(telemetry_group)

        self.lbl_conn = QLabel("🔴 WAITING FOR WEBOTS PREVIEW...")
        self.lbl_conn.setStyleSheet("font-weight: bold; color: #ff5252; font-size: 13px;")

        self.lbl_mode = QLabel("Mode: PLANNER")
        self.lbl_mode.setFont(QFont("Consolas", 10, QFont.Bold))
        self.lbl_mode.setStyleSheet("color: #00e5ff;")

        self.lbl_dyn = QLabel("Speed: 0.00 m/s (cmd: 0.00) | Steer: +0.0°")
        self.lbl_dyn.setFont(QFont("Consolas", 9))

        self.lbl_gap = QLabel("Gap Target: (0.00, 0.00)m | Aperture: 0.0 cm")
        self.lbl_gap.setFont(QFont("Consolas", 9))

        self.lbl_perf = QLabel("Plan: 0.0 ms | Costmap: 0.0 ms | Surviving: 0/0")
        self.lbl_perf.setFont(QFont("Consolas", 9))

        self.lbl_dist = QLabel("Distance Traveled: 0.00 m | Lap: 0")
        self.lbl_dist.setFont(QFont("Consolas", 10, QFont.Bold))
        self.lbl_dist.setStyleSheet("color: #ffd54f;")

        telem_layout.addWidget(self.lbl_conn)
        telem_layout.addWidget(self.lbl_mode)
        telem_layout.addWidget(self.lbl_dyn)
        telem_layout.addWidget(self.lbl_gap)
        telem_layout.addWidget(self.lbl_perf)
        telem_layout.addWidget(self.lbl_dist)
        panel_layout.addWidget(telemetry_group)

        # 2. SIMULATION CONTROLS
        sim_group = QGroupBox("Webots Simulation Controls")
        sim_layout = QVBoxLayout(sim_group)

        btn_row = QHBoxLayout()
        self.btn_play_pause = QPushButton("⏸ Pause")
        self.btn_play_pause.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold; padding: 6px;")
        self.btn_play_pause.clicked.connect(self._toggle_pause)
        self.sim_paused = False

        self.btn_step = QPushButton("⏭ Step (20ms)")
        self.btn_step.setStyleSheet("background-color: #1565c0; color: white; font-weight: bold; padding: 6px;")
        self.btn_step.clicked.connect(self._send_step)
        btn_row.addWidget(self.btn_play_pause)
        btn_row.addWidget(self.btn_step)
        sim_layout.addLayout(btn_row)

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
        self.cb_sim_speed.setCurrentIndex(0)  # 0.05x default (20x Slow-Mo)
        self.cb_sim_speed.currentIndexChanged.connect(self._on_sim_speed_changed)
        speed_row.addWidget(self.cb_sim_speed)
        sim_layout.addLayout(speed_row)

        panel_layout.addWidget(sim_group)

        # 3. LIVE PARAMETER TUNER FOR C++ SEPTIC
        tune_group = QGroupBox("Live C++ Septic Spline Parameters (Syncs to Genome)")
        tune_layout = QVBoxLayout(tune_group)

        # 0. Steering Architecture / Planner Mode (Option 1 vs Option 2 vs Option 0)
        tune_layout.addWidget(QLabel("Steering Architecture / Planner Mode:"))
        self.cb_steering_mode = QComboBox()
        self.cb_steering_mode.addItem("Option 1: Two-Stage Midpoint Spline", 1)
        self.cb_steering_mode.addItem("Option 2: Corridor-Anchored Pruned Spline", 2)
        self.cb_steering_mode.addItem("Option 0: Classic Septic Single Spline", 0)
        self.cb_steering_mode.setStyleSheet("background-color: #2b2b2b; color: #00e676; font-weight: bold; font-size: 11px; padding: 4px;")
        
        init_mode = int(self.loaded_genes.get("steering_mode", 1))
        idx_to_set = 0 if init_mode == 1 else (1 if init_mode == 2 else 2)
        self.cb_steering_mode.setCurrentIndex(idx_to_set)
        self.cb_steering_mode.currentIndexChanged.connect(self._on_steering_mode_changed)
        tune_layout.addWidget(self.cb_steering_mode)

        # A. Inflation Radius (cm)
        init_infl = float(self.loaded_genes.get("r_inflation_m", 0.09))
        tune_layout.addWidget(QLabel("Obstacle Inflation Radius (cm):"))
        r_row = QHBoxLayout()
        self.slider_infl = QSlider(Qt.Horizontal)
        self.slider_infl.setRange(70, 140)  # 7.0 to 14.0 cm
        self.slider_infl.setValue(int(round(init_infl * 1000.0)))
        self.slider_infl.valueChanged.connect(self._on_infl_slider)
        self.lbl_infl = QLabel(f"{init_infl*100:.1f} cm")
        self.lbl_infl.setFont(QFont("Consolas", 10, QFont.Bold))
        r_row.addWidget(self.slider_infl)
        r_row.addWidget(self.lbl_infl)
        tune_layout.addLayout(r_row)

        # B. v_entry_max (m/s)
        init_vmax = float(self.loaded_genes.get("v_entry_max", 4.92))
        tune_layout.addWidget(QLabel("Max Cruising Speed Ceiling (m/s):"))
        vmax_row = QHBoxLayout()
        self.slider_vmax = QSlider(Qt.Horizontal)
        self.slider_vmax.setRange(15, 65)  # 1.5 to 6.5 m/s
        self.slider_vmax.setValue(int(round(init_vmax * 10.0)))
        self.slider_vmax.valueChanged.connect(self._on_vmax_slider)
        self.lbl_vmax = QLabel(f"{init_vmax:.2f} m/s")
        self.lbl_vmax.setFont(QFont("Consolas", 10, QFont.Bold))
        vmax_row.addWidget(self.slider_vmax)
        vmax_row.addWidget(self.lbl_vmax)
        tune_layout.addLayout(vmax_row)

        # C. v_cruise_min (m/s)
        init_vmin = float(self.loaded_genes.get("v_cruise_min", 1.70))
        tune_layout.addWidget(QLabel("Min Corner Entry Speed Floor (m/s):"))
        vmin_row = QHBoxLayout()
        self.slider_vmin = QSlider(Qt.Horizontal)
        self.slider_vmin.setRange(8, 30)  # 0.8 to 3.0 m/s
        self.slider_vmin.setValue(int(round(init_vmin * 10.0)))
        self.slider_vmin.valueChanged.connect(self._on_vmin_slider)
        self.lbl_vmin = QLabel(f"{init_vmin:.2f} m/s")
        self.lbl_vmin.setFont(QFont("Consolas", 10, QFont.Bold))
        vmin_row.addWidget(self.slider_vmin)
        vmin_row.addWidget(self.lbl_vmin)
        tune_layout.addLayout(vmin_row)

        # D. Steering Stability / Damping Weight (w_steer_effort)
        init_wsteer = float(self.loaded_genes.get("w_steer_effort", 3.0))
        tune_layout.addWidget(QLabel("Steering Damping Weight (w_steer_effort):"))
        wsteer_row = QHBoxLayout()
        self.slider_wsteer = QSlider(Qt.Horizontal)
        self.slider_wsteer.setRange(5, 80)  # 0.5 to 8.0
        self.slider_wsteer.setValue(int(round(init_wsteer * 10.0)))
        self.slider_wsteer.valueChanged.connect(self._on_wsteer_slider)
        self.lbl_wsteer = QLabel(f"{init_wsteer:.1f}")
        self.lbl_wsteer.setFont(QFont("Consolas", 10, QFont.Bold))
        wsteer_row.addWidget(self.slider_wsteer)
        wsteer_row.addWidget(self.lbl_wsteer)
        tune_layout.addLayout(wsteer_row)

        # E. Curvature Smoothness Weight (kappa_smooth_gain)
        init_ksmooth = float(self.loaded_genes.get("kappa_smooth_gain", 3.0))
        tune_layout.addWidget(QLabel("Curvature Smoothness Weight (kappa_smooth_gain):"))
        ksmooth_row = QHBoxLayout()
        self.slider_ksmooth = QSlider(Qt.Horizontal)
        self.slider_ksmooth.setRange(5, 70)  # 0.5 to 7.0
        self.slider_ksmooth.setValue(int(round(init_ksmooth * 10.0)))
        self.slider_ksmooth.valueChanged.connect(self._on_ksmooth_slider)
        self.lbl_ksmooth = QLabel(f"{init_ksmooth:.1f}")
        self.lbl_ksmooth.setFont(QFont("Consolas", 10, QFont.Bold))
        ksmooth_row.addWidget(self.slider_ksmooth)
        ksmooth_row.addWidget(self.lbl_ksmooth)
        tune_layout.addLayout(ksmooth_row)

        panel_layout.addWidget(tune_group)

        # 4. DISPLAY LAYER TOGGLES
        disp_group = QGroupBox("Layer Display Options")
        disp_layout = QGridLayout(disp_group)

        self.chk_cand = QCheckBox("Septic Candidate Fan")
        self.chk_cand.setChecked(True)
        self.chk_cand.toggled.connect(lambda v: setattr(self.canvas, "show_candidates", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_cand, 0, 0)

        self.chk_best = QCheckBox("Winning Spline (Gold)")
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
                print(f"[Septic GUI UDP] Bound listening socket to {bind_ip}:9876", flush=True)
                break
            except Exception as e:
                print(f"[Septic GUI UDP] Note: Could not bind {bind_ip}:9876: {e}", flush=True)

        if not bound:
            print("[Septic GUI UDP] Critical: Failed to bind to port 9876 on any interface.", flush=True)
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
                print(f"[Septic GUI UDP Error] {e}", flush=True)

    def _send_cmd(self, cmd_dict: dict):
        try:
            raw = json.dumps(cmd_dict).encode("utf-8")
            self.udp_sock.sendto(raw, ("127.0.0.1", 9877))
        except Exception as e:
            print(f"[Septic GUI Send Error] {e}", flush=True)

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

    def _on_steering_mode_changed(self, idx: int):
        mode_val = self.cb_steering_mode.currentData()
        if mode_val is not None:
            print(f"[Septic GUI] User selected Steering Mode: {mode_val}", flush=True)
            self._send_cmd({"cmd": "set_param", "name": "steering_mode", "val": float(mode_val)})
            self._save_gene_to_file("steering_mode", int(mode_val))

    def _on_infl_slider(self, val: int):
        r_m = val / 1000.0
        self.lbl_infl.setText(f"{r_m*100:.1f} cm")
        self._send_cmd({"cmd": "set_param", "name": "r_inflation_m", "val": r_m})
        self._save_gene_to_file("r_inflation_m", r_m)

    def _on_vmax_slider(self, val: int):
        v = val / 10.0
        self.lbl_vmax.setText(f"{v:.2f} m/s")
        self._send_cmd({"cmd": "set_param", "name": "v_entry_max", "val": v})
        self._save_gene_to_file("v_entry_max", v)

    def _on_vmin_slider(self, val: int):
        v = val / 10.0
        self.lbl_vmin.setText(f"{v:.2f} m/s")
        self._send_cmd({"cmd": "set_param", "name": "v_cruise_min", "val": v})
        self._save_gene_to_file("v_cruise_min", v)

    def _on_wsteer_slider(self, val: int):
        w = val / 10.0
        self.lbl_wsteer.setText(f"{w:.1f}")
        self._send_cmd({"cmd": "set_param", "name": "w_steer_effort", "val": w})
        self._save_gene_to_file("w_steer_effort", w)

    def _on_ksmooth_slider(self, val: int):
        w = val / 10.0
        self.lbl_ksmooth.setText(f"{w:.1f}")
        self._send_cmd({"cmd": "set_param", "name": "kappa_smooth_gain", "val": w})
        self._save_gene_to_file("kappa_smooth_gain", w)

    def _update_gui(self):
        pkt = None
        is_conn = False
        with self._lock:
            pkt = self.latest_pkt
            is_conn = self.udp_connected

        if is_conn:
            self.lbl_conn.setText("🟢 LIVE WEBOTS 50Hz STREAM CONNECTED")
            self.lbl_conn.setStyleSheet("font-weight: bold; color: #00e676; font-size: 13px;")
        else:
            self.lbl_conn.setText("🔴 WAITING FOR WEBOTS: Run champion preview")
            self.lbl_conn.setStyleSheet("font-weight: bold; color: #ff5252; font-size: 13px;")

        if pkt is not None:
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
            mode = pkt.get("mode", "PLANNER")
            v_est = pkt.get("v_est", 0.0)
            v_cmd = pkt.get("v_cmd", pkt.get("v_tgt", 0.0))
            steer = pkt.get("steer_deg", 0.0)
            dist = pkt.get("dist_m", 0.0)
            lap = pkt.get("lap", 0)
            p_ms = pkt.get("plan_ms", 0.0)
            m_ms = pkt.get("map_ms", 0.0)
            status = pkt.get("status", "OK")
            surviving = pkt.get("surviving", 0)
            cands = pkt.get("candidates", len(cand_lines))

            self.lbl_mode.setText(f"Mode: {mode}")
            self.lbl_dyn.setText(f"Speed: {v_est:.2f} m/s (cmd: {v_cmd:.2f}) | Steer: {steer:+5.1f}°")
            if gap and len(gap) >= 3:
                self.lbl_gap.setText(f"Gap Target: ({gap[0]:+.2f}, {gap[1]:.2f})m | Aperture: {gap[2]*100:.1f} cm")
            self.lbl_perf.setText(f"Plan: {p_ms:.2f}ms | Costmap: {m_ms:.2f}ms | Surviving: {surviving}/{cands} | {status}")
            self.lbl_dist.setText(f"Distance Traveled: {dist:.2f} m | Lap: {lap}")


def launch_septic_live_gui():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    gui = SepticLivePreviewGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    launch_septic_live_gui()
