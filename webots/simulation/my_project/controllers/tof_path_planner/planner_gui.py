"""
================================================================================
  AUTONOMOUS TOF PATH PLANNER INTERACTIVE GUI & LIVE TUNER
  - Live 15mm C++ ToF Costmap Visualization & Hazard Boundary Overlay
  - Real-Time Native C++ Curvilinear SL-MPC (OSQP) Trajectory Display
  - Primary Racing Line + Challenger Line & Scouted Dynamic Gaps
  - Reverse Backup Logic Kinematic Checking Curves (Left/Right/Center Escape Arcs)
  - Live Interactive Tuning of Parameters (Inflation Radius, Speed, Slip, Challenger Hysteresis)
  - Full Simulation Control: Pause, Play, Single-Step (8.3ms), Speed Multiplier
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
from typing import Dict, List, Optional
import numpy as np

# Ensure root paths
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
MY_PROJECT_DIR = os.path.dirname(CONTROLLERS_DIR)
WORKSPACE_ROOT = os.path.dirname(MY_PROJECT_DIR)

for p in [WORKSPACE_ROOT, MY_PROJECT_DIR, CONTROLLERS_DIR, CURR_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from PyQt5 import QtGui, QtWidgets
from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF
from PyQt5.QtGui import QColor, QPainter, QPen, QBrush, QFont, QImage, QPalette
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QComboBox, QPushButton, QCheckBox, QGroupBox,
    QTabWidget, QFrame, QGridLayout, QProgressBar, QScrollArea
)


# 16-Channel Physical ToF Sensor Mounting Poses (only_front_tofs.proto)
FRONT_TOF_SENSOR_RIG = [
    # Front-Left Array (X = -0.070 m, Y = +0.095 m, cross-pointing Right)
    ("tof_front_left_4.1deg",  -0.070, 0.095, 1.4987, "FL 4.1°"),
    ("tof_front_left_15.4deg", -0.070, 0.095, 1.3024, "FL 15.4°"),
    ("tof_front_left_26.6deg", -0.070, 0.095, 1.1060, "FL 26.6°"),
    ("tof_front_left_37.9deg", -0.070, 0.095, 0.9097, "FL 37.9°"),
    ("tof_front_left_49.1deg", -0.070, 0.095, 0.7134, "FL 49.1°"),
    ("tof_front_left_60.4deg", -0.070, 0.095, 0.5170, "FL 60.4°"),
    ("tof_front_left_71.6deg", -0.070, 0.095, 0.3207, "FL 71.6°"),
    ("tof_front_left_82.9deg", -0.070, 0.095, 0.1243, "FL 82.9°"),
    # Front-Right Array (X = +0.070 m, Y = +0.095 m, cross-pointing Left)
    ("tof_front_right_4.1deg",   0.070, 0.095, 1.6428, "FR 4.1°"),
    ("tof_front_right_15.4deg",  0.070, 0.095, 1.8391, "FR 15.4°"),
    ("tof_front_right_26.6deg",  0.070, 0.095, 2.0356, "FR 26.6°"),
    ("tof_front_right_37.9deg",  0.070, 0.095, 2.2318, "FR 37.9°"),
    ("tof_front_right_49.1deg",  0.070, 0.095, 2.4282, "FR 49.1°"),
    ("tof_front_right_60.4deg",  0.070, 0.095, 2.6245, "FR 60.4°"),
    ("tof_front_right_71.6deg",  0.070, 0.095, 2.8208, "FR 71.6°"),
    ("tof_front_right_82.9deg",  0.070, 0.095, 3.0172, "FR 82.9°"),
]


# Dark Theme Global QSS Stylesheet
DARK_THEME_QSS = """
    QMainWindow, QWidget#CentralWidget {
        background-color: #0b0e14;
        color: #e6edf3;
    }
    QWidget {
        background-color: transparent;
        color: #e6edf3;
        font-family: 'Segoe UI', 'Consolas', sans-serif;
    }
    QGroupBox {
        font-size: 11px;
        font-weight: bold;
        color: #58a6ff;
        border: 1px solid #262c36;
        border-radius: 6px;
        margin-top: 10px;
        padding-top: 10px;
        background-color: #131720;
    }
    QGroupBox::title {
        subcontrol-origin: margin;
        subcontrol-position: top left;
        padding: 0 6px;
        left: 10px;
        background-color: #131720;
    }
    QLabel {
        color: #c9d1d9;
    }
    QTabWidget::pane {
        border: 1px solid #262c36;
        background-color: #131720;
        border-radius: 6px;
    }
    QTabBar::tab {
        background: #0b0e14;
        color: #8b949e;
        padding: 8px 16px;
        font-weight: bold;
        font-size: 11px;
        border: 1px solid #262c36;
        border-bottom: none;
        border-top-left-radius: 6px;
        border-top-right-radius: 6px;
        margin-right: 2px;
    }
    QTabBar::tab:selected {
        background: #131720;
        color: #00e676;
        border-bottom: 2px solid #00e676;
    }
    QTabBar::tab:hover {
        color: #f0f6fc;
        background: #1c212c;
    }
    QSlider::groove:horizontal {
        height: 6px;
        background: #1c212c;
        border-radius: 3px;
    }
    QSlider::sub-page:horizontal {
        background: #00897b;
        border-radius: 3px;
    }
    QSlider::handle:horizontal {
        background: #00e676;
        border: 1px solid #00c853;
        width: 14px;
        margin-top: -4px;
        margin-bottom: -4px;
        border-radius: 7px;
    }
    QSlider::handle:horizontal:hover {
        background: #69f0ae;
    }
    QComboBox {
        background-color: #1c212c;
        border: 1px solid #262c36;
        border-radius: 4px;
        color: #00e5ff;
        padding: 4px 8px;
        font-weight: bold;
    }
    QComboBox QAbstractItemView {
        background-color: #131720;
        color: #c9d1d9;
        selection-background-color: #00838f;
        selection-color: #ffffff;
        border: 1px solid #262c36;
    }
    QPushButton {
        background-color: #1c212c;
        border: 1px solid #262c36;
        border-radius: 5px;
        color: #c9d1d9;
        padding: 6px 12px;
        font-weight: bold;
    }
    QPushButton:hover {
        background-color: #2a3242;
        color: #f0f6fc;
    }
    QCheckBox {
        color: #c9d1d9;
        font-size: 11px;
    }
    QCheckBox::indicator {
        width: 14px;
        height: 14px;
        border-radius: 3px;
        border: 1px solid #262c36;
        background-color: #1c212c;
    }
    QCheckBox::indicator:checked {
        background-color: #00b0ff;
        border-color: #40c4ff;
    }
    QScrollArea {
        border: none;
        background: transparent;
    }
    QScrollBar:vertical {
        background: #0b0e14;
        width: 8px;
        margin: 0;
    }
    QScrollBar::handle:vertical {
        background: #262c36;
        min-height: 20px;
        border-radius: 4px;
    }
    QScrollBar::handle:vertical:hover {
        background: #58a6ff;
    }
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
        height: 0px;
    }
"""


# ==============================================================================
# 1. 2D COSTMAP, ESCAPE ARCS & TRAJECTORY CANVAS
# ==============================================================================
class PlannerCostmapCanvas(QtWidgets.QWidget):
    """
    High-performance 2D Canvas visualizing:
    - 15mm C++ ToF Costmap Probability Map & Hazard Overlay
    - Physical Chassis Footprint & Wheel Heading
    - 16-Channel ToF Cones
    - Candidate Trajectory Fan (Cyan) & Winning SL-MPC Trajectory (Gold)
    - Scouted Open Gaps & Multi-Target Candidates (Green/Cyan)
    - Reverse Backup Logic Kinematic Checking Curves (Thin Magenta/Purple Escape Arcs)
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(560, 620)
        self.setMouseTracking(True)

        # Costmap metric bounds
        self.map_x_min = -1.25
        self.map_x_max = 1.25
        self.map_y_min = -0.10
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
        self.steer_deg: float = 0.0
        self.recovery_state: str = "DRIVE"

        # Precompute Kinematic Reverse Backup Logic Checking Curves (Origin at Robot Center (0.0, 0.0))
        # Wheelbase = 0.1325m, Max Steer = 27.0 deg (0.471 rad), R_min ~ 0.260m, Lookahead = 0.45m
        wheelbase = 0.1325
        max_steer_rad = 0.47123889
        self.r_min_turn = wheelbase / max(1e-3, math.tan(max_steer_rad))
        self.origin_y = 0.0   # Robot center (0.0, 0.0)
        self.bumper_y = 0.095 # Front bumper mount Y in vehicle frame
        self.arc_lookahead_m = 0.45
        n_pts = 45
        self._arc_s = np.linspace(0.0, self.arc_lookahead_m, n_pts, dtype=np.float32)

        # Left Escape Arc (Full left lock: sweeps left -X from robot center)
        self._arc_pts_left = np.zeros((n_pts, 2), dtype=np.float32)
        self._arc_pts_left[:, 0] = -self.r_min_turn * (1.0 - np.cos(self._arc_s / self.r_min_turn))
        self._arc_pts_left[:, 1] = self.origin_y + self.r_min_turn * np.sin(self._arc_s / self.r_min_turn)

        # Right Escape Arc (Full right lock: sweeps right +X from robot center)
        self._arc_pts_right = np.zeros((n_pts, 2), dtype=np.float32)
        self._arc_pts_right[:, 0] = self.r_min_turn * (1.0 - np.cos(self._arc_s / self.r_min_turn))
        self._arc_pts_right[:, 1] = self.origin_y + self.r_min_turn * np.sin(self._arc_s / self.r_min_turn)

        # Center Straight Sightline (Forward along +Y from robot center)
        self._arc_pts_center = np.zeros((n_pts, 2), dtype=np.float32)
        self._arc_pts_center[:, 0] = 0.0
        self._arc_pts_center[:, 1] = self.origin_y + self._arc_s

        # Display layer toggles
        self.show_grid = True
        self.show_rays = True
        self.show_costmap = True
        self.show_candidates = True
        self.show_best = True
        self.show_gap = True
        self.show_hazard = True
        self.show_chassis = True
        self.show_backup_curves = True

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
        steer_deg: float = 0.0,
        recovery_state: str = "DRIVE",
    ):
        if prob_map is not None:
            self.prob_map = prob_map
        if hazard_mask is not None:
            self.hazard_mask = hazard_mask
        self.tof_readings = tof_readings
        self.best_line = best_line
        self.candidate_lines = candidate_lines
        self.gap_info = gap
        self.targets = targets
        self.steer_deg = float(steer_deg)
        self.recovery_state = str(recovery_state)
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
        painter.fillRect(self.rect(), QColor(10, 13, 18))

        # 1. Costmap Probability Image
        if self.show_costmap and self.prob_map is not None:
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
            img_data[..., 3] = (np.clip(self.prob_map * 210.0 + 45.0, 0, 255)).astype(np.uint8)

            if self.show_hazard and self.hazard_mask is not None:
                img_data[self.hazard_mask > 0, 0] = 255
                img_data[self.hazard_mask > 0, 1] = 40
                img_data[self.hazard_mask > 0, 2] = 60
                img_data[self.hazard_mask > 0, 3] = 230

            img_data = np.ascontiguousarray(np.flipud(img_data))
            qimg = QImage(img_data.data, W, H, W * 4, QImage.Format_RGBA8888).copy()

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

        # 3. ToF Sensor Rays & Conical Beams (16 Channels)
        if self.show_rays and self.tof_readings:
            half_fov = 0.19634954 / 2.0  # 5.625 deg half FOV

            for sensor in FRONT_TOF_SENSOR_RIG:
                s_name, sx, sy, heading, _ = sensor
                rng = self.tof_readings.get(s_name)
                if rng is None:
                    continue
                disp_r = min(max(float(rng), 0.02), 1.80)
                is_hit = (float(rng) < 1.78)

                p_origin = self.world_to_screen(sx, sy)

                # Arc endpoints across true 11.25 deg cone
                n_arc = 6
                angles = np.linspace(heading - half_fov, heading + half_fov, n_arc)
                arc_pts = []
                for a in angles:
                    ax = sx + disp_r * math.cos(a)
                    ay = sy + disp_r * math.sin(a)
                    arc_pts.append(self.world_to_screen(ax, ay))

                # Midpoint optical axis
                cx = sx + disp_r * math.cos(heading)
                cy = sy + disp_r * math.sin(heading)
                p_c = self.world_to_screen(cx, cy)

                if is_hit:
                    if disp_r < 0.25:
                        c_fill = QColor(255, 40, 40, 50)
                        c_line = QColor(255, 60, 60, 220)
                    elif disp_r < 0.55:
                        c_fill = QColor(255, 180, 0, 40)
                        c_line = QColor(255, 190, 0, 200)
                    else:
                        c_fill = QColor(0, 230, 180, 30)
                        c_line = QColor(0, 230, 180, 170)
                else:
                    c_fill = QColor(70, 110, 160, 12)
                    c_line = QColor(80, 130, 180, 60)

                # Conical sector polygon
                poly = QtGui.QPolygonF([p_origin] + arc_pts)
                painter.setPen(Qt.NoPen)
                painter.setBrush(QBrush(c_fill))
                painter.drawPolygon(poly)

                # Boundary rays
                edge_pen = QPen(QColor(c_line.red(), c_line.green(), c_line.blue(), 90 if is_hit else 40), 1.0, Qt.SolidLine if is_hit else Qt.DotLine)
                painter.setPen(edge_pen)
                painter.drawLine(p_origin, arc_pts[0])
                painter.drawLine(p_origin, arc_pts[-1])

                # Contact arc on obstacle surface
                if is_hit:
                    arc_pen = QPen(c_line, 2.5, Qt.SolidLine, Qt.RoundCap)
                    painter.setPen(arc_pen)
                    for k in range(len(arc_pts) - 1):
                        painter.drawLine(arc_pts[k], arc_pts[k + 1])

                # Center optical axis ray
                painter.setPen(QPen(c_line, 1.2 if is_hit else 0.8, Qt.DashLine if is_hit else Qt.DotLine))
                painter.drawLine(p_origin, p_c)

                # Sensor origin mount dot
                painter.setPen(Qt.NoPen)
                painter.setBrush(QBrush(QColor(0, 220, 255, 200)))
                painter.drawEllipse(p_origin, 2.5, 2.5)

                # Obstacle hit marker and range tag
                if is_hit:
                    painter.setPen(QPen(c_line, 2.0))
                    painter.setBrush(QBrush(c_line))
                    painter.drawEllipse(p_c, 3.5, 3.5)
                    if disp_r < 1.30:
                        painter.setFont(QFont("Consolas", 7))
                        painter.setPen(QPen(QColor(255, 255, 255, 220)))
                        painter.drawText(QPointF(p_c.x() + 4, p_c.y() - 3), f"{disp_r*100:.0f}cm")

        # 4. Candidate Trajectory Lines (Cyan Fan)
        if self.show_candidates and self.candidate_lines:
            cand_pen = QPen(QColor(0, 200, 255, 75), 1.2, Qt.SolidLine)
            painter.setPen(cand_pen)
            for line in self.candidate_lines:
                if len(line) > 1:
                    for i in range(len(line) - 1):
                        p1 = self.world_to_screen(line[i][0], line[i][1])
                        p2 = self.world_to_screen(line[i + 1][0], line[i + 1][1])
                        painter.drawLine(p1, p2)

        # 5. Best / Winning Trajectory Line (Bright Gold Solid Line)
        if self.show_best and self.best_line and len(self.best_line) > 1:
            best_pen = QPen(QColor(255, 215, 0, 240), 3.0, Qt.SolidLine)
            painter.setPen(best_pen)
            for i in range(len(self.best_line) - 1):
                p1 = self.world_to_screen(self.best_line[i][0], self.best_line[i][1])
                p2 = self.world_to_screen(self.best_line[i + 1][0], self.best_line[i + 1][1])
                painter.drawLine(p1, p2)

        # 6. Reverse Backup Logic Checking Curves (Left Turn Arc, Right Turn Arc, Center Forward Sightline)
        if self.show_backup_curves:
            curves = [
                ("Left Arc", self._arc_pts_left),
                ("Center Sightline", self._arc_pts_center),
                ("Right Arc", self._arc_pts_right),
            ]

            h_mask = self.hazard_mask
            has_h = h_mask is not None and isinstance(h_mask, np.ndarray) and h_mask.size > 0
            if has_h:
                h_rows, h_cols = h_mask.shape
                dx = max(1e-4, self.map_x_max - self.map_x_min)
                dy = max(1e-4, self.map_y_max - self.map_y_min)

            is_recovering = (
                "RECOVERY" in self.recovery_state.upper()
                or "PIVOT" in self.recovery_state.upper()
                or "DECEL" in self.recovery_state.upper()
            )

            for _, pts in curves:
                hit_idx = -1
                if has_h:
                    for i in range(len(pts)):
                        px, py = pts[i, 0], pts[i, 1]
                        if py < self.bumper_y:
                            continue  # Skip inside vehicle footprint
                        col = int((px - self.map_x_min) / dx * h_cols)
                        row = int((py - self.map_y_min) / dy * h_rows)
                        if 0 <= row < h_rows and 0 <= col < h_cols:
                            if h_mask[row, col] > 0:
                                hit_idx = i
                                break

                clear_end = hit_idx if hit_idx != -1 else len(pts)

                # Draw clear portion (Thin vivid magenta line)
                if is_recovering:
                    pen_clear = QPen(QColor(255, 64, 230, 240), 1.8, Qt.SolidLine)
                else:
                    pen_clear = QPen(QColor(225, 60, 255, 200), 1.3, Qt.DashLine)

                painter.setPen(pen_clear)
                for i in range(max(0, clear_end - 1)):
                    p1 = self.world_to_screen(pts[i, 0], pts[i, 1])
                    p2 = self.world_to_screen(pts[i + 1, 0], pts[i + 1, 1])
                    painter.drawLine(p1, p2)

                # If obstructed by hazard mask, draw blocked tail as thin dotted red
                if hit_idx != -1 and hit_idx < len(pts):
                    pen_blocked = QPen(QColor(255, 75, 75, 140), 1.1, Qt.DotLine)
                    painter.setPen(pen_blocked)
                    for i in range(max(0, hit_idx - 1), len(pts) - 1):
                        p1 = self.world_to_screen(pts[i, 0], pts[i, 1])
                        p2 = self.world_to_screen(pts[i + 1, 0], pts[i + 1, 1])
                        painter.drawLine(p1, p2)

                    # Collision hit dot on obstacle boundary
                    p_hit = self.world_to_screen(pts[hit_idx, 0], pts[hit_idx, 1])
                    painter.setPen(QPen(QColor(255, 80, 80, 230), 1.5))
                    painter.setBrush(QBrush(QColor(255, 50, 50, 210)))
                    painter.drawEllipse(p_hit, 3.0, 3.0)

                # End-point tick
                tip_idx = min(clear_end, len(pts) - 1)
                p_tip = self.world_to_screen(pts[tip_idx, 0], pts[tip_idx, 1])
                painter.setPen(Qt.NoPen)
                painter.setBrush(QBrush(QColor(255, 120, 255, 230)))
                painter.drawEllipse(p_tip, 2.5, 2.5)

            # Center sightline emergency backup threshold ticks (9cm emergency threshold, 15cm blocked threshold)
            for d_thresh, t_col in [(0.09, QColor(255, 60, 60, 220)), (0.15, QColor(255, 190, 0, 200))]:
                y_t = self.bumper_y + d_thresh
                p_tl = self.world_to_screen(-0.025, y_t)
                p_tr = self.world_to_screen(+0.025, y_t)
                painter.setPen(QPen(t_col, 1.2, Qt.SolidLine))
                painter.drawLine(p_tl, p_tr)

        # 7. Scouted Open Gap Target Marker & Candidates
        if self.show_gap and self.targets:
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

        # 8. Chassis Footprint & Origin (150x200mm Robot Geometry)
        if self.show_chassis:
            car_w = 0.150
            car_l = 0.200

            # Chassis outer boundary
            c_tl = self.world_to_screen(-car_w / 2.0, car_l / 2.0)
            c_br = self.world_to_screen(car_w / 2.0, -car_l / 2.0)
            body_rect = QRectF(c_tl.x(), c_tl.y(), c_br.x() - c_tl.x(), c_br.y() - c_tl.y())
            painter.setPen(QPen(QColor(0, 229, 255, 220), 1.8))
            painter.setBrush(QBrush(QColor(18, 30, 45, 200)))
            painter.drawRoundedRect(body_rect, 4, 4)

            # Steered front wheels and fixed rear wheels
            steer_rad = math.radians(getattr(self, "steer_deg", 0.0))
            wheel_pos = [
                (-0.0411, +0.0625, steer_rad),   # FL (steered)
                (+0.0411, +0.0625, steer_rad),   # FR (steered)
                (-0.0500, -0.0700, 0.0),         # RL (fixed)
                (+0.0500, -0.0700, 0.0),         # RR (fixed)
            ]
            wheel_pen = QPen(QColor(255, 215, 64, 240), 1.5)
            wheel_brush = QBrush(QColor(40, 40, 40, 240))
            painter.setPen(wheel_pen)
            painter.setBrush(wheel_brush)

            for wx, wy, w_steer in wheel_pos:
                wp = self.world_to_screen(wx, wy)
                painter.save()
                painter.translate(wp)
                painter.rotate(-math.degrees(w_steer))
                painter.drawRoundedRect(QRectF(-4, -8, 8, 16), 2, 2)
                painter.restore()

            # Front bumper highlight line (y = +0.100 m)
            p_fl = self.world_to_screen(-car_w / 2.0, 0.100)
            p_fr = self.world_to_screen(car_w / 2.0, 0.100)
            painter.setPen(QPen(QColor(0, 255, 180, 250), 2.5))
            painter.drawLine(p_fl, p_fr)

            # Front bumper center origin dot (x = 0.0, y = +0.095)
            p_bmp = self.world_to_screen(0.0, 0.095)
            painter.setPen(QPen(QColor(0, 255, 180), 1.5))
            painter.setBrush(QBrush(QColor(0, 255, 180)))
            painter.drawEllipse(p_bmp, 2.5, 2.5)

            # Origin center marker (0.0, 0.0)
            p0 = self.world_to_screen(0.0, 0.0)
            painter.setPen(QPen(QColor(255, 50, 50), 2))
            painter.setBrush(QBrush(QColor(255, 50, 50)))
            painter.drawEllipse(p0, 3.5, 3.5)


# ==============================================================================
# 2. MAIN COMPREHENSIVE GUI WINDOW & TUNER
# ==============================================================================
class ComprehensivePlannerGUI(QMainWindow):
    """Main Telemetry & Costmap GUI Window with Dark Theme Live Studio."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Curvilinear SL-MPC Autonomous Racing Studio (60 Hz)")
        self.resize(1260, 820)
        self.setStyleSheet(DARK_THEME_QSS)

        # UDP network sockets
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.last_packet_time = 0.0
        self.udp_connected = False
        self.latest_pkt: Optional[dict] = None
        self._lock = threading.Lock()

        # Start UDP listener thread
        self.running = True
        self.listener_thread = threading.Thread(target=self._udp_worker, daemon=True)
        self.listener_thread.start()

        # Build UI layout
        self._build_ui()

        # 60 FPS UI refresh timer (16 ms)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_gui)
        self.timer.start(16)

    def _build_ui(self):
        central = QWidget(self)
        central.setObjectName("CentralWidget")
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(12)

        # Left Canvas
        self.canvas = PlannerCostmapCanvas(self)
        layout.addWidget(self.canvas, stretch=3)

        # Right Panel Tabs
        self.tabs = QTabWidget()

        # Tab 1: Planner Controls & Tuner
        tab_controls = QWidget()
        ctrl_scroll = QScrollArea()
        ctrl_scroll.setWidgetResizable(True)

        panel = QWidget()
        panel_layout = QVBoxLayout(panel)
        panel_layout.setContentsMargins(6, 6, 6, 6)
        panel_layout.setSpacing(10)

        # 1. CONTROLLER BANNER
        status_group = QGroupBox("Active Controller Architecture")
        status_layout = QVBoxLayout(status_group)
        self.lbl_arch = QLabel("🚀 Native C++ Curvilinear SL-MPC (OSQP)")
        self.lbl_arch.setFont(QFont("Consolas", 11, QFont.Bold))
        self.lbl_arch.setStyleSheet("color: #00e5ff; padding: 2px;")
        self.lbl_mode_badge = QLabel("Mode: DRIVE | 60 Hz Control Loop")
        self.lbl_mode_badge.setFont(QFont("Consolas", 9, QFont.Bold))
        self.lbl_mode_badge.setStyleSheet("color: #00e676;")
        status_layout.addWidget(self.lbl_arch)
        status_layout.addWidget(self.lbl_mode_badge)
        panel_layout.addWidget(status_group)

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
        self.cb_sim_speed.setCurrentIndex(4)  # 1.00x default
        self.cb_sim_speed.currentIndexChanged.connect(self._on_sim_speed_changed)
        speed_row.addWidget(self.cb_sim_speed)
        sim_layout.addLayout(speed_row)

        panel_layout.addWidget(sim_group)

        # 3. LIVE PARAMETER TUNING
        tune_group = QGroupBox("Live SL-MPC & Scout Parameter Tuner")
        tune_layout = QVBoxLayout(tune_group)

        # Cruising Speed Ceiling
        tune_layout.addWidget(QLabel("Cruising Speed Target (m/s):"))
        spd_row = QHBoxLayout()
        self.slider_speed = QSlider(Qt.Horizontal)
        self.slider_speed.setRange(8, 55)  # 0.8 to 5.5 m/s
        self.slider_speed.setValue(55)
        self.slider_speed.valueChanged.connect(self._on_speed_slider)
        self.lbl_speed = QLabel("5.50 m/s")
        self.lbl_speed.setFont(QFont("Consolas", 10, QFont.Bold))
        spd_row.addWidget(self.slider_speed)
        spd_row.addWidget(self.lbl_speed)
        tune_layout.addLayout(spd_row)

        # Obstacle Inflation Radius
        tune_layout.addWidget(QLabel("Obstacle Inflation Radius (Bare Min 7.5 cm):"))
        inf_row = QHBoxLayout()
        self.slider_inflation = QSlider(Qt.Horizontal)
        self.slider_inflation.setRange(75, 140)
        self.slider_inflation.setValue(75)
        self.slider_inflation.valueChanged.connect(self._on_inflation_slider)
        self.lbl_inflation = QLabel("7.5 cm")
        self.lbl_inflation.setFont(QFont("Consolas", 10, QFont.Bold))
        inf_row.addWidget(self.slider_inflation)
        inf_row.addWidget(self.lbl_inflation)
        tune_layout.addLayout(inf_row)

        # Peak Slip Angle
        tune_layout.addWidget(QLabel("Tire Peak Slip Angle (deg):"))
        slip_row = QHBoxLayout()
        self.slider_slip = QSlider(Qt.Horizontal)
        self.slider_slip.setRange(60, 200)
        self.slider_slip.setValue(126)
        self.slider_slip.valueChanged.connect(self._on_slip_slider)
        self.lbl_slip = QLabel("12.6°")
        self.lbl_slip.setFont(QFont("Consolas", 10, QFont.Bold))
        slip_row.addWidget(self.slider_slip)
        slip_row.addWidget(self.lbl_slip)
        tune_layout.addLayout(slip_row)

        # Challenger Switch Hysteresis
        tune_layout.addWidget(QLabel("Challenger Switch Hysteresis (Anti-Flicker):"))
        hyst_row = QHBoxLayout()
        self.slider_hysteresis = QSlider(Qt.Horizontal)
        self.slider_hysteresis.setRange(2, 80)
        self.slider_hysteresis.setValue(8)
        self.slider_hysteresis.valueChanged.connect(self._on_hysteresis_slider)
        self.lbl_hysteresis = QLabel("8.0")
        self.lbl_hysteresis.setFont(QFont("Consolas", 10, QFont.Bold))
        self.lbl_hysteresis.setToolTip("MPC cost margin required for Challenger to switch/overthrow Main line.")
        hyst_row.addWidget(self.slider_hysteresis)
        hyst_row.addWidget(self.lbl_hysteresis)
        tune_layout.addLayout(hyst_row)

        # Challenger Minimum Lateral Separation
        tune_layout.addWidget(QLabel("Challenger Min Corridor Separation (cm):"))
        sep_row = QHBoxLayout()
        self.slider_separation = QSlider(Qt.Horizontal)
        self.slider_separation.setRange(10, 50)
        self.slider_separation.setValue(15)
        self.slider_separation.valueChanged.connect(self._on_separation_slider)
        self.lbl_separation = QLabel("15 cm")
        self.lbl_separation.setFont(QFont("Consolas", 10, QFont.Bold))
        self.lbl_separation.setToolTip("Minimum lateral distance between Main and Challenger targets.")
        sep_row.addWidget(self.slider_separation)
        sep_row.addWidget(self.lbl_separation)
        tune_layout.addLayout(sep_row)

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
        self.lbl_recovery = QLabel("Recovery Mode: NORMAL | Arcs: Clear")
        self.lbl_recovery.setFont(QFont("Consolas", 9))
        self.lbl_recovery.setStyleSheet("color: #ce93d8;")
        self.lbl_dist = QLabel("Distance Traveled: 0.00 m | Lap: 0")
        self.lbl_dist.setFont(QFont("Consolas", 10, QFont.Bold))
        self.lbl_dist.setStyleSheet("color: #ffd54f;")

        telem_layout.addWidget(self.lbl_conn)
        telem_layout.addWidget(self.lbl_dyn)
        telem_layout.addWidget(self.lbl_gap)
        telem_layout.addWidget(self.lbl_perf)
        telem_layout.addWidget(self.lbl_recovery)
        telem_layout.addWidget(self.lbl_dist)
        panel_layout.addWidget(telemetry_group)

        # 5. DISPLAY LAYER TOGGLES
        disp_group = QGroupBox("Layer Display Options")
        disp_layout = QGridLayout(disp_group)

        self.chk_best = QCheckBox("Best Trajectory (Gold)")
        self.chk_best.setChecked(True)
        self.chk_best.toggled.connect(lambda v: setattr(self.canvas, "show_best", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_best, 0, 0)

        self.chk_cand = QCheckBox("Candidate Fan (Cyan)")
        self.chk_cand.setChecked(True)
        self.chk_cand.toggled.connect(lambda v: setattr(self.canvas, "show_candidates", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_cand, 0, 1)

        self.chk_backup = QCheckBox("Backup Escape Curves (Magenta)")
        self.chk_backup.setChecked(True)
        self.chk_backup.setToolTip("Thin kinematic escape arcs evaluated against costmap to decide reverse direction")
        self.chk_backup.toggled.connect(lambda v: setattr(self.canvas, "show_backup_curves", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_backup, 1, 0)

        self.chk_gap = QCheckBox("Scouted Gap (Green)")
        self.chk_gap.setChecked(True)
        self.chk_gap.toggled.connect(lambda v: setattr(self.canvas, "show_gap", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_gap, 1, 1)

        self.chk_hazard = QCheckBox("Hazard Mask (Red)")
        self.chk_hazard.setChecked(True)
        self.chk_hazard.toggled.connect(lambda v: setattr(self.canvas, "show_hazard", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_hazard, 2, 0)

        self.chk_costmap = QCheckBox("Costmap (15mm)")
        self.chk_costmap.setChecked(True)
        self.chk_costmap.toggled.connect(lambda v: setattr(self.canvas, "show_costmap", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_costmap, 2, 1)

        self.chk_rays = QCheckBox("ToF Cones (16ch)")
        self.chk_rays.setChecked(True)
        self.chk_rays.toggled.connect(lambda v: setattr(self.canvas, "show_rays", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_rays, 3, 0)

        self.chk_chassis = QCheckBox("Chassis Footprint")
        self.chk_chassis.setChecked(True)
        self.chk_chassis.toggled.connect(lambda v: setattr(self.canvas, "show_chassis", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_chassis, 3, 1)

        self.chk_grid = QCheckBox("Grid & Circles")
        self.chk_grid.setChecked(True)
        self.chk_grid.toggled.connect(lambda v: setattr(self.canvas, "show_grid", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_grid, 4, 0)

        panel_layout.addWidget(disp_group)

        panel_layout.addStretch()
        ctrl_scroll.setWidget(panel)

        tab_ctrl_layout = QVBoxLayout(tab_controls)
        tab_ctrl_layout.setContentsMargins(0, 0, 0, 0)
        tab_ctrl_layout.addWidget(ctrl_scroll)

        # Tab 2: 16-Channel ToF Radar Dashboard
        tab_tof = self._build_tof_dashboard_tab()

        self.tabs.addTab(tab_controls, "⚙️ Controls & Tuning")
        self.tabs.addTab(tab_tof, "📡 16-Ch ToF Gauges")
        layout.addWidget(self.tabs, stretch=2)

    def _build_tof_dashboard_tab(self) -> QWidget:
        container = QWidget()
        vbox = QVBoxLayout(container)
        vbox.setContentsMargins(8, 8, 8, 8)
        vbox.setSpacing(8)

        # 1. Summary Clearance Cards Group
        summary_group = QGroupBox("Live Obstacle Clearance Metrics")
        summary_layout = QGridLayout(summary_group)
        summary_layout.setContentsMargins(6, 8, 6, 8)
        summary_layout.setSpacing(6)

        def make_card(title: str, default_val: str, default_col: str):
            f = QFrame()
            f.setStyleSheet("background-color: #1a202c; border: 1px solid #2d3748; border-radius: 6px;")
            l = QVBoxLayout(f)
            l.setContentsMargins(4, 4, 4, 4)
            l.setSpacing(2)
            lbl_title = QLabel(title)
            lbl_title.setStyleSheet("color: #a0aec0; font-size: 9px; font-weight: bold;")
            lbl_val = QLabel(default_val)
            lbl_val.setFont(QFont("Consolas", 11, QFont.Bold))
            lbl_val.setStyleSheet(f"color: {default_col};")
            l.addWidget(lbl_title, alignment=Qt.AlignCenter)
            l.addWidget(lbl_val, alignment=Qt.AlignCenter)
            return f, lbl_val

        card_front, self.lbl_card_front = make_card("FRONT CLEARANCE", "> 1.80 m", "#00e676")
        card_left, self.lbl_card_left = make_card("LEFT FLANK", "> 1.80 m", "#00e676")
        card_right, self.lbl_card_right = make_card("RIGHT FLANK", "> 1.80 m", "#00e676")
        card_min, self.lbl_card_min = make_card("MINIMUM RANGE", "> 1.80 m", "#00e676")

        summary_layout.addWidget(card_front, 0, 0)
        summary_layout.addWidget(card_left, 0, 1)
        summary_layout.addWidget(card_right, 1, 0)
        summary_layout.addWidget(card_min, 1, 1)
        vbox.addWidget(summary_group)

        # 2. Scrollable 16-Channel Bars
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)

        bars_widget = QWidget()
        bars_layout = QVBoxLayout(bars_widget)
        bars_layout.setContentsMargins(2, 2, 2, 2)
        bars_layout.setSpacing(8)

        fl_group = QGroupBox("Front-Left Array (X = -7.0cm, Cross-Pointing Right)")
        fl_layout = QVBoxLayout(fl_group)
        fl_layout.setContentsMargins(6, 6, 6, 6)
        fl_layout.setSpacing(4)

        fr_group = QGroupBox("Front-Right Array (X = +7.0cm, Cross-Pointing Left)")
        fr_layout = QVBoxLayout(fr_group)
        fr_layout.setContentsMargins(6, 6, 6, 6)
        fr_layout.setSpacing(4)

        self.tof_bar_widgets = {}

        for sensor in FRONT_TOF_SENSOR_RIG:
            s_name, sx, sy, heading, s_label = sensor
            row = QHBoxLayout()
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(6)

            name_lbl = QLabel(f"{s_label}")
            name_lbl.setFont(QFont("Consolas", 8, QFont.Bold))
            name_lbl.setStyleSheet("color: #ccc; min-width: 65px;")

            bar = QProgressBar()
            bar.setRange(0, 180)
            bar.setValue(180)
            bar.setTextVisible(False)
            bar.setFixedHeight(12)
            bar.setStyleSheet("""
                QProgressBar {
                    background-color: #1a202c;
                    border: 1px solid #2d3748;
                    border-radius: 3px;
                }
                QProgressBar::chunk {
                    background-color: #00e676;
                    border-radius: 2px;
                }
            """)

            val_lbl = QLabel("1.80m")
            val_lbl.setFont(QFont("Consolas", 8))
            val_lbl.setStyleSheet("color: #00e676; min-width: 44px;")
            val_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

            row.addWidget(name_lbl)
            row.addWidget(bar, stretch=1)
            row.addWidget(val_lbl)

            self.tof_bar_widgets[s_name] = (bar, val_lbl, name_lbl)

            if "left" in s_name:
                fl_layout.addLayout(row)
            else:
                fr_layout.addLayout(row)

        bars_layout.addWidget(fl_group)
        bars_layout.addWidget(fr_group)
        bars_layout.addStretch()

        scroll.setWidget(bars_widget)
        vbox.addWidget(scroll, stretch=1)

        return container

    def _update_tof_dashboard(self, tof: dict):
        if not tof or not hasattr(self, "tof_bar_widgets"):
            return

        min_overall = 999.0
        front_beams = ["tof_front_left_4.1deg", "tof_front_left_15.4deg", "tof_front_right_4.1deg", "tof_front_right_15.4deg"]
        left_beams = ["tof_front_right_60.4deg", "tof_front_right_71.6deg", "tof_front_right_82.9deg", "tof_front_right_49.1deg"]
        right_beams = ["tof_front_left_60.4deg", "tof_front_left_71.6deg", "tof_front_left_82.9deg", "tof_front_left_49.1deg"]

        min_f = min([float(tof.get(k, 1.80)) for k in front_beams], default=1.80)
        min_l = min([float(tof.get(k, 1.80)) for k in left_beams], default=1.80)
        min_r = min([float(tof.get(k, 1.80)) for k in right_beams], default=1.80)

        for s_name, (bar, val_lbl, name_lbl) in self.tof_bar_widgets.items():
            if s_name in tof:
                r_m = float(tof[s_name])
                if r_m < min_overall:
                    min_overall = r_m

                cm_val = int(min(max(r_m * 100.0, 0.0), 180.0))
                bar.setValue(cm_val)

                if r_m < 0.25:
                    bar_col = "#ff3d00"
                elif r_m < 0.50:
                    bar_col = "#ffb300"
                else:
                    bar_col = "#00e676"

                bar.setStyleSheet(f"""
                    QProgressBar {{
                        background-color: #1a202c;
                        border: 1px solid #2d3748;
                        border-radius: 3px;
                    }}
                    QProgressBar::chunk {{
                        background-color: {bar_col};
                        border-radius: 2px;
                    }}
                """)
                val_lbl.setText(f"{r_m:.2f}m")
                val_lbl.setStyleSheet(f"color: {bar_col}; font-weight: bold; min-width: 44px;")

        def format_card(lbl, val_m):
            if val_m >= 1.78:
                lbl.setText("> 1.80 m")
                lbl.setStyleSheet("color: #00e676; font-weight: bold;")
            elif val_m < 0.25:
                lbl.setText(f"{val_m*100:.0f} cm")
                lbl.setStyleSheet("color: #ff3d00; font-weight: bold;")
            elif val_m < 0.50:
                lbl.setText(f"{val_m*100:.0f} cm")
                lbl.setStyleSheet("color: #ffb300; font-weight: bold;")
            else:
                lbl.setText(f"{val_m*100:.0f} cm")
                lbl.setStyleSheet("color: #00e676; font-weight: bold;")

        if hasattr(self, "lbl_card_front"):
            format_card(self.lbl_card_front, min_f)
            format_card(self.lbl_card_left, min_l)
            format_card(self.lbl_card_right, min_r)
            if min_overall < 900.0:
                format_card(self.lbl_card_min, min_overall)

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
                print(f"[GUI UDP Error] {e}", flush=True)

    def _send_cmd(self, cmd_dict: dict):
        try:
            raw = json.dumps(cmd_dict).encode("utf-8")
            self.udp_sock.sendto(raw, ("127.0.0.1", 9877))
        except Exception as e:
            print(f"[GUI Send Error] {e}", flush=True)

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
        if speed_val is not None:
            print(f"[GUI] Live Simulation Speed command sent: {speed_val:.2f}x", flush=True)
            for _ in range(3):
                self._send_cmd({"cmd": "set_speed", "val": float(speed_val)})

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

    def _on_hysteresis_slider(self, val: int):
        self.lbl_hysteresis.setText(f"{val:.1f}")
        self._send_cmd({"cmd": "set_param", "name": "alt_switch_hysteresis", "val": float(val)})

    def _on_separation_slider(self, val: int):
        sep_m = val / 100.0
        self.lbl_separation.setText(f"{val} cm")
        self._send_cmd({"cmd": "set_param", "name": "alt_min_separation_m", "val": sep_m})

    def _update_gui(self):
        pkt = None
        is_conn = False
        with self._lock:
            pkt = self.latest_pkt
            is_conn = self.udp_connected

        if is_conn:
            self.lbl_conn.setText("🟢 LIVE WEBOTS 60Hz STREAM CONNECTED")
            self.lbl_conn.setStyleSheet("font-weight: bold; color: #00e676;")
        else:
            self.lbl_conn.setText("🔴 WAITING FOR WEBOTS: Run track_racing.wbt")
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

            # Update Labels
            p_name = pkt.get("planner", "Curvilinear SL-MPC")
            mode = pkt.get("mode", "DRIVE")
            v_est = pkt.get("v_est", 0.0)
            v_tgt = pkt.get("v_tgt", 0.0)
            steer = pkt.get("steer_deg", 0.0)
            dist = pkt.get("dist_m", 0.0)
            p_ms = pkt.get("plan_ms", 0.0)
            m_ms = pkt.get("map_ms", 0.0)
            status = pkt.get("status", "--")
            surviving = pkt.get("surviving", 0)
            rec_state = pkt.get("recovery_state", mode)
            sim_time = pkt.get("sim_time", 0.0)

            self.canvas.set_data(
                prob_map,
                hazard_mask,
                tof,
                best_line,
                cand_lines,
                gap,
                targets=targets,
                grid_bounds=grid_bounds,
                steer_deg=steer,
                recovery_state=rec_state,
            )

            self.lbl_mode_badge.setText(f"Mode: {mode} | Sim Time: {sim_time:.1f}s")
            self.lbl_dyn.setText(f"Speed: {v_est:.2f} m/s (tgt: {v_tgt:.2f}) | Steer: {steer:+5.1f}°")
            if gap and len(gap) >= 3:
                self.lbl_gap.setText(f"Gap: ({gap[0]:+.2f}, {gap[1]:.2f})m | Width: {gap[2]*100:.1f} cm")
            self.lbl_perf.setText(f"Plan: {p_ms:.2f}ms | Costmap: {m_ms:.2f}ms | Surviving: {surviving} | {status}")
            
            # Recovery state label
            if "RECOVERY" in str(rec_state).upper() or "PIVOT" in str(rec_state).upper():
                self.lbl_recovery.setText(f"Recovery: ⚠️ {rec_state} (Active Escape Arc)")
                self.lbl_recovery.setStyleSheet("color: #ff5252; font-weight: bold;")
            elif "DECEL" in str(rec_state).upper():
                self.lbl_recovery.setText(f"Recovery: 🔄 {rec_state} (Centering Steering)")
                self.lbl_recovery.setStyleSheet("color: #ffd740; font-weight: bold;")
            else:
                self.lbl_recovery.setText(f"Recovery: 🟢 Normal Drive ({rec_state})")
                self.lbl_recovery.setStyleSheet("color: #ce93d8;")

            self.lbl_dist.setText(f"Distance Traveled: {dist:.2f} m")
            self._update_tof_dashboard(tof)


def launch_gui():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # Force dark palette across entire Qt application
    dark_palette = QPalette()
    dark_palette.setColor(QPalette.Window, QColor(11, 14, 20))
    dark_palette.setColor(QPalette.WindowText, QColor(230, 237, 243))
    dark_palette.setColor(QPalette.Base, QColor(19, 23, 32))
    dark_palette.setColor(QPalette.AlternateBase, QColor(28, 33, 44))
    dark_palette.setColor(QPalette.ToolTipBase, QColor(230, 237, 243))
    dark_palette.setColor(QPalette.ToolTipText, QColor(11, 14, 20))
    dark_palette.setColor(QPalette.Text, QColor(230, 237, 243))
    dark_palette.setColor(QPalette.Button, QColor(28, 33, 44))
    dark_palette.setColor(QPalette.ButtonText, QColor(230, 237, 243))
    dark_palette.setColor(QPalette.BrightText, QColor(255, 64, 129))
    dark_palette.setColor(QPalette.Link, QColor(88, 166, 255))
    dark_palette.setColor(QPalette.Highlight, QColor(0, 131, 143))
    dark_palette.setColor(QPalette.HighlightedText, QColor(255, 255, 255))
    app.setPalette(dark_palette)

    gui = ComprehensivePlannerGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    launch_gui()
