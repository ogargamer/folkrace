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
import struct
import threading
import zlib
import base64
from typing import Dict, List, Optional
import numpy as np

# Ensure root paths
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
MY_PROJECT_DIR = os.path.dirname(CONTROLLERS_DIR)
SIMULATION_DIR = os.path.dirname(MY_PROJECT_DIR)
WEBOTS_DIR = os.path.dirname(SIMULATION_DIR)
WORKSPACE_ROOT = os.path.dirname(WEBOTS_DIR)
GA_OPTIMIZER_DIR = os.path.join(WEBOTS_DIR, "optimization", "ga_planner_optimizer")

for p in [WORKSPACE_ROOT, WEBOTS_DIR, GA_OPTIMIZER_DIR, MY_PROJECT_DIR, CONTROLLERS_DIR, CURR_DIR]:
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


def wrap_to_pi(angle_rad: float) -> float:
    """Wraps an angle in radians into [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


# 16-Channel Physical ToF Sensor Mounting Poses (only_front_tofs.proto)
FRONT_TOF_SENSOR_RIG = [
    # Front-Left Array (cross-pointing Right, rotated 3 deg outwards)
    # Leftmost ToF (pointing straight and right, zones 1.1 deg to 34.9 deg)
    ("tof_front_left_4.1deg",  -0.058, 0.092, 1.5512, "FL 1.1°"),
    ("tof_front_left_15.4deg", -0.058, 0.092, 1.3548, "FL 12.4°"),
    ("tof_front_left_26.6deg", -0.058, 0.092, 1.1585, "FL 23.6°"),
    ("tof_front_left_37.9deg", -0.058, 0.092, 0.9621, "FL 34.9°"),
    # Left ToF (pointing right, zones 46.1 deg to 79.9 deg)
    ("tof_front_left_49.1deg", -0.057, 0.090, 0.7658, "FL 46.1°"),
    ("tof_front_left_60.4deg", -0.057, 0.090, 0.5694, "FL 57.4°"),
    ("tof_front_left_71.6deg", -0.057, 0.090, 0.3731, "FL 68.6°"),
    ("tof_front_left_82.9deg", -0.057, 0.090, 0.1767, "FL 79.9°"),
    # Front-Right Array (cross-pointing Left, mirrored along Y axis, rotated 3 deg outwards)
    # Rightmost ToF (pointing straight and left, zones 1.1 deg to 34.9 deg)
    ("tof_front_right_4.1deg",   0.058, 0.092, 1.5904, "FR 1.1°"),
    ("tof_front_right_15.4deg",  0.058, 0.092, 1.7868, "FR 12.4°"),
    ("tof_front_right_26.6deg",  0.058, 0.092, 1.9831, "FR 23.6°"),
    ("tof_front_right_37.9deg",  0.058, 0.092, 2.1795, "FR 34.9°"),
    # Right ToF (pointing left, zones 46.1 deg to 79.9 deg)
    ("tof_front_right_49.1deg",  0.057, 0.090, 2.3758, "FR 46.1°"),
    ("tof_front_right_60.4deg",  0.057, 0.090, 2.5722, "FR 57.4°"),
    ("tof_front_right_71.6deg",  0.057, 0.090, 2.7685, "FR 68.6°"),
    ("tof_front_right_82.9deg",  0.057, 0.090, 2.9649, "FR 79.9°"),
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
# Precomputed 256-color lookup table for instant zero-allocation costmap rendering
_COLOR_LUT = np.zeros((256, 4), dtype=np.uint8)
_COLOR_LUT[:97] = [14, 22, 34, 150]      # Free (< 0.38)
_COLOR_LUT[97:166] = [245, 215, 30, 130] # Unknown (0.38..0.65)
_COLOR_LUT[166:] = [255, 0, 0, 255]      # Obstacle (> 0.65)


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
        self.map_y_min = 0.00
        self.map_y_max = 2.60

        # Viewport metric bounds (2.50m upper bound covers 2.40m ToF visibility ceiling)
        self.x_min = -1.6
        self.x_max = 1.6
        self.y_min = -0.85
        self.y_max = 2.50

        # Grid data
        self.prob_map: Optional[np.ndarray] = None
        self.hazard_mask: Optional[np.ndarray] = None
        self.tof_readings: Dict[str, float] = {}

        self.best_line: Optional[List[List[float]]] = None
        self.candidate_lines: Optional[List[List[List[float]]]] = None
        self.guide_line: Optional[List[List[float]]] = None
        self.corridor_left: Optional[List[List[float]]] = None
        self.corridor_right: Optional[List[List[float]]] = None
        self.shield_info: Optional[dict] = None
        self.gap_info: Optional[List[float]] = None  # [xg, yg, width]
        self.targets: Optional[List[List[float]]] = None  # [[tx, ty, ap], ...]
        self.steer_deg: float = 0.0
        self.recovery_state: str = "DRIVE"

        # Anti-Wrong-Way & Historical Breadcrumb Tracking
        self.trail: List[List[float]] = []
        self.sightlines: List[dict] = []
        self.pose: List[float] = [0.0, 0.0, 0.0]
        self.is_wrong_way: bool = False
        self.target_yaw: float = 0.0

        # Load Kinematic Reverse Backup Logic parameters from active preview config / best genome
        self.r_min_turn = 0.1325 / max(1e-3, math.tan(0.562285))
        self.origin_y = 0.0   # Robot center (0.0, 0.0)
        self.bumper_y = 0.095 # Front bumper mount Y in vehicle frame
        self.arc_lookahead_m = 0.13
        self._arc_s = np.zeros(3, dtype=np.float32)
        self._arc_pts_left = np.zeros((3, 2), dtype=np.float32)
        self._arc_pts_right = np.zeros((3, 2), dtype=np.float32)
        self._arc_pts_center = np.zeros((3, 2), dtype=np.float32)

        self._last_cfg_mtime: float = 0.0
        self.reload_config()

        # Display layer toggles
        self.show_grid = True
        self.show_rays = True
        self.show_costmap = True
        self.show_corridor = True
        self.show_guide = True
        self.show_candidates = True
        self.show_best = True
        self.show_shield = True
        self.show_gap = True
        self.show_hazard = True
        self.show_chassis = True
        self.show_backup_curves = True
        self.show_trail = True
        self.show_sightlines = True

    def reload_config(self, force: bool = False) -> bool:
        """Loads recovery arc kinematics and parameters from active_preview_config.json or best_planner_genome.json."""
        wheelbase = 0.1325
        max_steer_rad = 0.562285  # Default ~32.22 deg (~0.5623 rad)
        arc_lookahead = 0.13   # Default 0.13 m

        for cand_name in ["best_planner_genome.json", "ga_checkpoint.json"]:
            cand_file = os.path.join(GA_OPTIMIZER_DIR, cand_name)
            if os.path.isfile(cand_file):
                try:
                    mtime = os.path.getmtime(cand_file)
                    if not force and mtime == self._last_cfg_mtime:
                        return False
                    self._last_cfg_mtime = mtime

                    with open(cand_file, "r", encoding="utf-8") as f:
                        cfg_data = json.load(f)
                    cfg_genes = cfg_data.get("genes") or cfg_data.get("best_genome") or cfg_data
                    if "recovery_steer_angle_rad" in cfg_genes:
                        max_steer_rad = min(0.562285, float(cfg_genes["recovery_steer_angle_rad"]))
                    if "recovery_arc_lookahead_m" in cfg_genes:
                        arc_lookahead = float(cfg_genes["recovery_arc_lookahead_m"])
                    break
                except Exception:
                    pass

        self.update_kinematic_arcs(wheelbase=wheelbase, max_steer_rad=max_steer_rad, arc_lookahead_m=arc_lookahead)
        return True

    def update_kinematic_arcs(self, wheelbase: float = 0.1325, max_steer_rad: float = 0.562285, arc_lookahead_m: float = 0.13):
        self.wheelbase = float(wheelbase)
        self.max_steer_rad = float(max_steer_rad)
        self.arc_lookahead_m = float(arc_lookahead_m)
        self.r_min_turn = self.wheelbase / max(1e-3, math.tan(self.max_steer_rad))
        self.origin_y = 0.0   # Robot center (0.0, 0.0)
        self.bumper_y = 0.095 # Front bumper mount Y in vehicle frame

        n_pts = max(3, int(self.arc_lookahead_m / 0.015)) if self.arc_lookahead_m >= 0.045 else max(3, int(self.arc_lookahead_m * 100))
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
        guide_line: Optional[List[List[float]]] = None,
        corridor_left: Optional[List[List[float]]] = None,
        corridor_right: Optional[List[List[float]]] = None,
        shield_info: Optional[dict] = None,
        trail: Optional[List[List[float]]] = None,
        sightlines: Optional[List[dict]] = None,
        is_wrong_way: bool = False,
        target_yaw: float = 0.0,
        pose: Optional[List[float]] = None,
    ):
        if prob_map is not None:
            self.prob_map = prob_map
        if hazard_mask is not None:
            self.hazard_mask = hazard_mask
        if best_line and len(best_line) > 1:
            self.best_line = best_line
        elif best_line is not None:
            self.best_line = best_line

        if candidate_lines is not None:
            self.candidate_lines = candidate_lines

        if guide_line and len(guide_line) > 1:
            self.guide_line = guide_line

        if corridor_left and len(corridor_left) > 1:
            self.corridor_left = corridor_left

        if corridor_right and len(corridor_right) > 1:
            self.corridor_right = corridor_right

        if shield_info and any(len(shield_info.get(k, [])) > 1 for k in ("k1_left", "k1_right", "k2_left", "k2_right", "straight")):
            self.shield_info = shield_info
        elif shield_info is not None and not getattr(self, "shield_info", None):
            self.shield_info = shield_info

        self.gap_info = gap
        self.targets = targets
        self.steer_deg = float(steer_deg)
        self.recovery_state = str(recovery_state)

        if trail is not None:
            self.trail = trail
        if sightlines is not None:
            self.sightlines = sightlines
        self.is_wrong_way = bool(is_wrong_way)
        self.target_yaw = float(target_yaw)
        if pose is not None and len(pose) >= 3:
            self.pose = [float(pose[0]), float(pose[1]), float(pose[2])]

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

        # 1. Costmap Probability Image (Optimized 256-color LUT with zero boolean mask allocations)
        if self.show_costmap and self.prob_map is not None:
            H, W = self.prob_map.shape
            prob_u8 = (np.clip(self.prob_map, 0.0, 1.0) * 255.0).astype(np.uint8)
            img_data = _COLOR_LUT[prob_u8]
            if self.show_hazard and self.hazard_mask is not None:
                img_data = img_data.copy()
                haz_mask = (self.hazard_mask > 0) & (prob_u8 < 166)
                img_data[haz_mask] = [255, 42, 64, 175]

            img_flipped = np.ascontiguousarray(np.flipud(img_data))
            qimg = QImage(img_flipped.data, W, H, W * 4, QImage.Format_RGBA8888)

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
            for ym in [-0.5, 0.0, 0.5, 1.0, 1.5, 1.8]:
                p1 = self.world_to_screen(self.x_min, ym)
                p2 = self.world_to_screen(self.x_max, ym)
                painter.drawLine(p1, p2)
                painter.setPen(QPen(QColor(100, 150, 200, 180)))
                lbl_text = "1.8m (ToF Limit)" if ym == 1.8 else f"{ym:.1f}m"
                painter.drawText(QPointF(p1.x() + 8, p1.y() - 4), lbl_text)
                painter.setPen(grid_pen)

            for xm in [-1.0, -0.5, 0.0, 0.5, 1.0]:
                p1 = self.world_to_screen(xm, self.y_min)
                p2 = self.world_to_screen(xm, self.y_max)
                painter.drawLine(p1, p2)

            for ring_r in [0.5, 1.0, 1.5, 1.8]:
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
                disp_r = min(max(float(rng), 0.02), 2.40)
                is_hit = (float(rng) < 2.38)

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

        # 4. Curvilinear SL-MPC Dynamic Safety Corridor (Teal Ribbon & Boundaries)
        if self.show_corridor and self.corridor_left and self.corridor_right and len(self.corridor_left) > 1 and len(self.corridor_right) > 1:
            poly_pts = []
            for pt in self.corridor_left:
                poly_pts.append(self.world_to_screen(pt[0], pt[1]))
            for pt in reversed(self.corridor_right):
                poly_pts.append(self.world_to_screen(pt[0], pt[1]))

            # Semi-transparent teal ribbon fill
            corr_poly = QtGui.QPolygonF(poly_pts)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(QColor(0, 229, 255, 25)))
            painter.drawPolygon(corr_poly)

            # Corridor left boundary (Teal dashed)
            pen_corr_left = QPen(QColor(0, 229, 255, 150), 1.3, Qt.DashLine)
            painter.setPen(pen_corr_left)
            for i in range(len(self.corridor_left) - 1):
                p1 = self.world_to_screen(self.corridor_left[i][0], self.corridor_left[i][1])
                p2 = self.world_to_screen(self.corridor_left[i + 1][0], self.corridor_left[i + 1][1])
                painter.drawLine(p1, p2)

            # Corridor right boundary (Teal dashed)
            pen_corr_right = QPen(QColor(0, 229, 255, 150), 1.3, Qt.DashLine)
            painter.setPen(pen_corr_right)
            for i in range(len(self.corridor_right) - 1):
                p1 = self.world_to_screen(self.corridor_right[i][0], self.corridor_right[i][1])
                p2 = self.world_to_screen(self.corridor_right[i + 1][0], self.corridor_right[i + 1][1])
                painter.drawLine(p1, p2)

        # 5. Curvilinear SL-MPC Reference Guide Spline (Cyan Dotted / Dashed)
        if self.show_guide and self.guide_line and len(self.guide_line) > 1:
            guide_pen = QPen(QColor(80, 200, 255, 160), 1.4, Qt.DashLine)
            painter.setPen(guide_pen)
            for i in range(len(self.guide_line) - 1):
                p1 = self.world_to_screen(self.guide_line[i][0], self.guide_line[i][1])
                p2 = self.world_to_screen(self.guide_line[i + 1][0], self.guide_line[i + 1][1])
                painter.drawLine(p1, p2)

        # 6. Candidate Trajectory Lines (Cyan Fan & Challenger Line)
        if self.show_candidates and self.candidate_lines:
            for idx, line in enumerate(self.candidate_lines):
                if len(line) > 1:
                    if idx == 0:
                        cand_pen = QPen(QColor(0, 200, 255, 90), 1.2, Qt.SolidLine)
                    else:
                        cand_pen = QPen(QColor(180, 100, 255, 160), 1.6, Qt.DashLine)
                    painter.setPen(cand_pen)
                    for i in range(len(line) - 1):
                        p1 = self.world_to_screen(line[i][0], line[i][1])
                        p2 = self.world_to_screen(line[i + 1][0], line[i + 1][1])
                        painter.drawLine(p1, p2)

        # 7. Best / Winning Trajectory Line (Bright Gold Solid Line with Step Dots)
        if self.show_best and self.best_line and len(self.best_line) > 1:
            best_pen = QPen(QColor(255, 215, 0, 240), 3.0, Qt.SolidLine)
            painter.setPen(best_pen)
            for i in range(len(self.best_line) - 1):
                p1 = self.world_to_screen(self.best_line[i][0], self.best_line[i][1])
                p2 = self.world_to_screen(self.best_line[i + 1][0], self.best_line[i + 1][1])
                painter.drawLine(p1, p2)

            # Tiny yellow step dots along the trajectory
            painter.setPen(QPen(QColor(30, 25, 0, 220), 0.8))
            painter.setBrush(QBrush(QColor(255, 245, 100, 255)))
            for pt in self.best_line:
                p = self.world_to_screen(pt[0], pt[1])
                painter.drawEllipse(p, 2.5, 2.5)

        # 8. Emergency Shield Micro-Rollouts & Predictive Course Probe
        if self.show_shield and self.shield_info:
            s_active = self.shield_info.get("active", False)
            chosen_id = self.shield_info.get("chosen_id", -1)
            all_blocked_reverse = s_active and chosen_id == 4  # All evasion arcs exhausted → reverse

            branches = [
                # (pts, is_safe, is_chosen, label, step_num)
                (self.shield_info.get("straight_zero", []), self.shield_info.get("straight_zero_safe", False), False, "0-Steer Straight", -1),
                (self.shield_info.get("straight_mpc",  self.shield_info.get("straight", [])), self.shield_info.get("straight_mpc_safe", self.shield_info.get("straight_safe", False)), False, "MPC Course", 0),
                (self.shield_info.get("k1_left",   []), self.shield_info.get("k1_left_safe",   False), s_active and chosen_id == 2, "Step 0 Left",  1),
                (self.shield_info.get("k1_right",  []), self.shield_info.get("k1_right_safe",  False), s_active and chosen_id == 3, "Step 0 Right", 1),
                (self.shield_info.get("k2_left",   []), self.shield_info.get("k2_left_safe",   False), s_active and chosen_id == 0, "Step 2 Left",  2),
                (self.shield_info.get("k2_right",  []), self.shield_info.get("k2_right_safe",  False), s_active and chosen_id == 1, "Step 2 Right", 2),
            ]

            for branch, is_safe, is_chosen, label, step_num in branches:
                if len(branch) > 1:
                    if step_num == -1:
                        # Pure zero-steering straight probe: dotted cyan/white if clear, orange-red dotted if blocked
                        col = QColor(190, 220, 255, 190) if is_safe else QColor(255, 100, 50, 210)
                        pen = QPen(col, 2.0 if is_safe else 2.6, Qt.DotLine)
                    elif step_num == 0:
                        # Forward MPC course probe: white/cyan dashed if clear, bright orange-red if blocked
                        col = QColor(240, 245, 255, 220) if is_safe else QColor(255, 60, 20, 240)
                        pen = QPen(col, 2.4 if is_safe else 3.2, Qt.DashLine)
                    elif all_blocked_reverse and step_num == 1:
                        # Step-0 branches are the reverse trigger indicator — draw bold red/magenta
                        pen = QPen(QColor(255, 30, 80, 240), 2.8, Qt.SolidLine)
                    elif is_chosen:
                        pen = QPen(QColor(0, 255, 128, 255), 3.2, Qt.SolidLine)
                    elif is_safe:
                        col = QColor(0, 230, 160, 210) if step_num == 1 else QColor(0, 255, 200, 220)
                        pen = QPen(col, 2.0, Qt.SolidLine)
                    else:
                        pen = QPen(QColor(255, 70, 70, 190), 1.6, Qt.DashLine)

                    painter.setPen(pen)
                    for i in range(len(branch) - 1):
                        p1 = self.world_to_screen(branch[i][0], branch[i][1])
                        p2 = self.world_to_screen(branch[i + 1][0], branch[i + 1][1])
                        painter.drawLine(p1, p2)

                    if step_num > 0:
                        # Root dot at the trajectory step origin
                        p_root = self.world_to_screen(branch[0][0], branch[0][1])
                        painter.setPen(Qt.NoPen)
                        dot_col = QColor(255, 30, 80, 240) if all_blocked_reverse and step_num == 1 else QColor(255, 215, 0, 220)
                        painter.setBrush(QBrush(dot_col))
                        painter.drawEllipse(p_root, 2.5, 2.5)

                    # Tip dot & collision hit marker
                    p_tip = self.world_to_screen(branch[-1][0], branch[-1][1])
                    if step_num == -1:
                        tip_col = QColor(140, 230, 255) if is_safe else QColor(255, 90, 50)
                    elif step_num == 0:
                        tip_col = QColor(0, 255, 128) if is_safe else QColor(255, 50, 50)
                    elif all_blocked_reverse and step_num == 1:
                        tip_col = QColor(255, 30, 80)  # Vivid red for reverse-trigger branch tips
                    else:
                        tip_col = QColor(0, 255, 128) if is_safe else QColor(255, 50, 50)
                    painter.setPen(Qt.NoPen)
                    painter.setBrush(QBrush(tip_col))
                    painter.drawEllipse(p_tip, 4.0 if is_chosen else 2.6, 4.0 if is_chosen else 2.6)

            # All-blocked reverse indicator: bright red ring at robot origin
            if all_blocked_reverse:
                p_origin = self.world_to_screen(0.0, 0.0)
                painter.setPen(QPen(QColor(255, 30, 80, 230), 2.5, Qt.SolidLine))
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(p_origin, 9.0, 9.0)
                painter.setFont(QFont("Consolas", 7, QFont.Bold))
                painter.setPen(QPen(QColor(255, 30, 80, 240)))
                painter.drawText(QPointF(p_origin.x() + 11, p_origin.y() + 4), "◄ REV")

        # 9. Reverse Backup Logic Checking Curves (Left Turn Arc, Right Turn Arc, Center Forward Sightline)
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

        # 10. Breadcrumb Historical Trail
        if self.show_trail and getattr(self, "trail", None):
            gx, gy, gyaw = self.pose if (self.pose and len(self.pose) >= 3) else (0.0, 0.0, 0.0)
            cos_yaw = math.cos(gyaw)
            sin_yaw = math.sin(gyaw)

            local_pts = []
            for pt in self.trail:
                if isinstance(pt, (list, tuple)) and len(pt) >= 2:
                    dx = float(pt[0]) - gx
                    dy = float(pt[1]) - gy
                    xv = cos_yaw * dx + sin_yaw * dy
                    yv = -sin_yaw * dx + cos_yaw * dy
                    local_pts.append((xv, yv))

            # Connect newest breadcrumb to vehicle rear axle (0.0, -0.070)
            local_pts.append((0.0, -0.070))

            if len(local_pts) > 1:
                n_pts = len(local_pts)
                for i in range(n_pts - 1):
                    p1 = self.world_to_screen(local_pts[i][0], local_pts[i][1])
                    p2 = self.world_to_screen(local_pts[i + 1][0], local_pts[i + 1][1])

                    # Progressive alpha: older points are translucent cyan, newer points are vivid cyan
                    frac = (i + 1) / float(n_pts)
                    alpha = int(70 + 170 * frac)
                    pen_seg = QPen(QColor(0, 220, 255, alpha), 2.2, Qt.SolidLine, Qt.RoundCap)
                    painter.setPen(pen_seg)
                    painter.drawLine(p1, p2)

                    # Subtle node dots along the trail
                    if i % 2 == 0:
                        painter.setPen(Qt.NoPen)
                        painter.setBrush(QBrush(QColor(0, 229, 255, int(alpha * 0.8))))
                        painter.drawEllipse(p1, 2.2, 2.2)

        # 11. Wrong-Way Line-of-Sight Raycasts
        if self.show_sightlines and getattr(self, "sightlines", None):
            p_origin = self.world_to_screen(0.0, 0.095)  # Front bumper origin
            for ray in self.sightlines:
                rx_v = float(ray.get("xv", 0.0))
                ry_v = float(ray.get("yv", 0.0))
                is_clear = bool(ray.get("clear", True))
                is_opp = bool(ray.get("opposing", False))
                p_tgt = self.world_to_screen(rx_v, ry_v)

                if is_clear and is_opp:
                    # Unobstructed opposing candidate: vivid neon emerald/cyan line
                    painter.setPen(QPen(QColor(0, 255, 180, 230), 2.4, Qt.SolidLine))
                    painter.drawLine(p_origin, p_tgt)
                    painter.setBrush(QBrush(QColor(0, 255, 180, 240)))
                    painter.drawEllipse(p_tgt, 4.0, 4.0)

                    h_diff = float(ray.get("heading_diff_deg", 0.0))
                    painter.setFont(QFont("Consolas", 7, QFont.Bold))
                    painter.setPen(QPen(QColor(0, 255, 180, 250)))
                    painter.drawText(QPointF(p_tgt.x() + 5, p_tgt.y() - 3), f"LOS {h_diff:.0f}°")
                elif not is_clear:
                    # Blocked by wall/obstacle in costmap: dark red dashed line with square marker
                    painter.setPen(QPen(QColor(255, 60, 60, 130), 1.2, Qt.DashLine))
                    painter.drawLine(p_origin, p_tgt)
                    painter.setBrush(QBrush(QColor(255, 60, 60, 160)))
                    painter.drawRect(QRectF(p_tgt.x() - 2.5, p_tgt.y() - 2.5, 5, 5))

        # 12. Corrective 180 Target Vector (Gold Arrow)
        if self.is_wrong_way or "180" in str(self.recovery_state):
            gx, gy, gyaw = self.pose if (self.pose and len(self.pose) >= 3) else (0.0, 0.0, 0.0)
            tgt_world = float(getattr(self, "target_yaw", 0.0) or 0.0)
            rel_yaw = wrap_to_pi(tgt_world - gyaw)

            arrow_len_m = 0.85
            arrow_xv = arrow_len_m * math.sin(rel_yaw)
            arrow_yv = arrow_len_m * math.cos(rel_yaw)

            p_orig = self.world_to_screen(0.0, 0.0)
            p_arrow = self.world_to_screen(arrow_xv, arrow_yv)

            painter.setPen(QPen(QColor(255, 215, 0, 240), 3.5, Qt.SolidLine))
            painter.drawLine(p_orig, p_arrow)

            # Arrowhead
            ah_len = 14.0
            dx_screen = p_arrow.x() - p_orig.x()
            dy_screen = p_arrow.y() - p_orig.y()
            screen_angle = math.atan2(dy_screen, dx_screen)

            p1_head = QPointF(
                p_arrow.x() - ah_len * math.cos(screen_angle - 0.45),
                p_arrow.y() - ah_len * math.sin(screen_angle - 0.45),
            )
            p2_head = QPointF(
                p_arrow.x() - ah_len * math.cos(screen_angle + 0.45),
                p_arrow.y() - ah_len * math.sin(screen_angle + 0.45),
            )
            painter.setBrush(QBrush(QColor(255, 215, 0)))
            painter.setPen(Qt.NoPen)
            painter.drawPolygon(QtGui.QPolygonF([p_arrow, p1_head, p2_head]))

            painter.setFont(QFont("Consolas", 8, QFont.Bold))
            painter.setPen(QPen(QColor(255, 215, 0)))
            painter.drawText(QPointF(p_arrow.x() + 8, p_arrow.y() + 4), "180° TARGET")

        # 13. Chassis Footprint & Origin (150x200mm Robot Geometry)
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

        # 14. Wrong-Way Alert Banner
        if self.is_wrong_way or "180" in str(self.recovery_state):
            painter.setFont(QFont("Segoe UI", 10, QFont.Bold))
            banner_rect = QRectF(14, 12, 360, 26)
            painter.setPen(QPen(QColor(255, 60, 60, 220), 1.5))
            painter.setBrush(QBrush(QColor(40, 10, 16, 220)))
            painter.drawRoundedRect(banner_rect, 5, 5)
            painter.setPen(QPen(QColor(255, 80, 80)))
            painter.drawText(banner_rect, Qt.AlignCenter, "🚨 WRONG-WAY TRIPPED — EXECUTING 180°")

        # 15. Costmap Category & Breadcrumb Legend (Free / Unknown / Obstacle / C-Space / Trail / LOS Ray)
        painter.setFont(QFont("Consolas", 8, QFont.Bold))
        legend_items = [
            ("Free", QColor(14, 22, 34, 230), QColor(0, 229, 255)),
            ("Unknown", QColor(245, 215, 30, 210), QColor(20, 20, 20)),
            ("Obstacle", QColor(255, 0, 0, 250), QColor(255, 255, 255)),
            ("C-Space", QColor(255, 42, 64, 210), QColor(255, 255, 255)),
            ("Trail", QColor(0, 220, 255, 200), QColor(10, 20, 30)),
            ("LOS Ray", QColor(0, 255, 180, 220), QColor(10, 20, 20)),
        ]
        lx = 14
        ly = self.height() - 26
        for name, bg_col, fg_col in legend_items:
            painter.setPen(QPen(QColor(50, 70, 95, 180), 1))
            painter.setBrush(QBrush(bg_col))
            painter.drawRoundedRect(QRectF(lx, ly, 60, 18), 3, 3)
            painter.setPen(QPen(fg_col))
            painter.drawText(QRectF(lx, ly, 60, 18), Qt.AlignCenter, name)
            lx += 66


def parse_binary_packet(data: bytes) -> Optional[dict]:
    """Decodes compact binary telemetry packet received from Pi 4 or Webots streaming bridge."""
    if len(data) < 232 or data[:4] != b"FOLK":
        return None
    header_fmt = "<4s I I d 16f 4f 4f 16f 3f 2H 4f 5I"
    try:
        fields = struct.unpack_from(header_fmt, data, 0)
    except Exception:
        return None

    time_step = fields[2]
    ts = fields[3]
    sim_time = fields[4]
    v_cmd = fields[5]
    v_real = fields[6]
    steer_cmd_deg = fields[7]
    steer_actual_deg = fields[8]
    yaw_deg = fields[9]
    pitch_deg = fields[10]
    roll_deg = fields[11]
    gyro_z = fields[12]
    accel_fwd = fields[13]
    accel_lat = fields[14]
    slip_ratio = fields[15]
    f_tof = fields[16]
    plan_ms = fields[17]
    map_ms = fields[18]
    scout_ms = fields[19]
    rpms_cmd = list(fields[20:24])
    rpms_act = list(fields[24:28])
    tof_ranges = list(fields[28:44])
    gap = list(fields[44:47])
    rows = fields[47]
    cols = fields[48]
    grid_bounds = list(fields[49:53])
    prob_comp_len = fields[53]
    prob_raw_len = fields[54]
    haz_comp_len = fields[55]
    haz_raw_len = fields[56]
    meta_len = fields[57]

    header_end = 232
    meta = {}
    if meta_len > 0 and len(data) >= header_end + prob_comp_len + haz_comp_len + meta_len:
        try:
            meta_raw = data[header_end + prob_comp_len + haz_comp_len : header_end + prob_comp_len + haz_comp_len + meta_len]
            meta = json.loads(meta_raw.decode("utf-8"))
        except Exception:
            pass

    pkt = {
        "time_step": time_step,
        "timestamp": ts,
        "sim_time": sim_time,
        "v_cmd": v_cmd,
        "v_tgt": v_cmd,
        "v_real": v_real,
        "v_est": v_real,
        "steer_cmd_deg": steer_cmd_deg,
        "steer_deg": steer_cmd_deg,
        "steer_actual_deg": steer_actual_deg,
        "yaw_deg": yaw_deg,
        "pitch_deg": pitch_deg,
        "roll_deg": roll_deg,
        "gyro_z": gyro_z,
        "accel_fwd": accel_fwd,
        "accel_lat": accel_lat,
        "slip_ratio": slip_ratio,
        "f_tof": f_tof,
        "plan_ms": plan_ms,
        "map_ms": map_ms,
        "scout_ms": scout_ms,
        "rpms_cmd": rpms_cmd,
        "rpms_act": rpms_act,
        "tof_ranges": tof_ranges,
        "tof": {FRONT_TOF_SENSOR_RIG[i][0]: tof_ranges[i] for i in range(min(len(FRONT_TOF_SENSOR_RIG), len(tof_ranges)))},
        "gap": gap,
        "grid_shape": [rows, cols],
        "grid_bounds": grid_bounds,
    }
    if prob_comp_len > 0 and rows > 0 and cols > 0 and len(data) >= header_end + prob_comp_len:
        prob_slice = data[header_end : header_end + prob_comp_len]
        if prob_comp_len < prob_raw_len:
            try:
                prob_raw = zlib.decompress(prob_slice)
            except Exception:
                prob_raw = prob_slice
        else:
            prob_raw = prob_slice
        prob_u8 = np.frombuffer(prob_raw, dtype=np.uint8).reshape((rows, cols))
        pkt["prob_map"] = prob_u8.astype(np.float32) / 255.0

    if haz_comp_len > 0 and rows > 0 and cols > 0 and len(data) >= header_end + prob_comp_len + haz_comp_len:
        haz_slice = data[header_end + prob_comp_len : header_end + prob_comp_len + haz_comp_len]
        if haz_comp_len < haz_raw_len:
            try:
                haz_raw = zlib.decompress(haz_slice)
            except Exception:
                haz_raw = haz_slice
        else:
            haz_raw = haz_slice
        haz_u8 = np.frombuffer(haz_raw, dtype=np.uint8).reshape((rows, cols))
        pkt["hazard_mask"] = haz_u8

    pkt.update(meta)
    return pkt


# ==============================================================================
# 2. MAIN COMPREHENSIVE GUI WINDOW & TUNER
# ==============================================================================
class ComprehensivePlannerGUI(QMainWindow):
    """Main Telemetry & Costmap GUI Window with Dark Theme Live Studio."""

    def __init__(self, initial_telem_hz: int = 60, local_only: bool = False):
        super().__init__()
        self.local_only = bool(local_only)
        self.initial_telem_hz = max(1, min(60, int(os.environ.get("WEBOTS_TELEMETRY_HZ", initial_telem_hz))))
        self.setWindowTitle(f"Curvilinear SL-MPC Autonomous Racing Studio ({self.initial_telem_hz} Hz)")
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

        # 2b. DYNAMIC MOVING OBSTACLES (ON RACING LINE)
        obs_group = QGroupBox("Dynamic Moving Obstacles (Racing Line)")
        obs_layout = QVBoxLayout(obs_group)

        # Direction Control Row
        obs_dir_row = QHBoxLayout()
        obs_dir_row.addWidget(QLabel("Direction:"))
        self.obs_direction = 1
        self.btn_obs_dir = QPushButton("▶ Forward (+)")
        self.btn_obs_dir.setStyleSheet("background-color: #1b5e20; color: #a5d6a7; font-weight: bold; padding: 5px; border-radius: 4px;")
        self.btn_obs_dir.setToolTip("Toggle dynamic obstacle direction along track racing line: Forward (+) vs Reverse (-)")
        self.btn_obs_dir.clicked.connect(self._toggle_obs_direction)
        obs_dir_row.addWidget(self.btn_obs_dir)

        self.btn_obs_reset = QPushButton("↺ Respawn")
        self.btn_obs_reset.setStyleSheet("background-color: #37474f; color: #cfd8dc; font-weight: bold; padding: 5px; border-radius: 4px;")
        self.btn_obs_reset.setToolTip("Reset moving obstacle positions to uniform track distribution")
        self.btn_obs_reset.clicked.connect(self._reset_obs_positions)
        obs_dir_row.addWidget(self.btn_obs_reset)
        obs_layout.addLayout(obs_dir_row)

        # Speed Control Row
        obs_spd_label_row = QHBoxLayout()
        obs_spd_label_row.addWidget(QLabel("Obstacle Speed:"))
        self.lbl_obs_speed = QLabel("1.00 m/s")
        self.lbl_obs_speed.setFont(QFont("Consolas", 10, QFont.Bold))
        self.lbl_obs_speed.setStyleSheet("color: #ffca28;")
        obs_spd_label_row.addWidget(self.lbl_obs_speed, alignment=Qt.AlignRight)
        obs_layout.addLayout(obs_spd_label_row)

        obs_spd_row = QHBoxLayout()
        self.slider_obs_speed = QSlider(Qt.Horizontal)
        self.slider_obs_speed.setRange(0, 30)  # 0.0 to 3.0 m/s with 0.1 m/s steps
        self.slider_obs_speed.setValue(10)     # Default 1.0 m/s
        self.slider_obs_speed.valueChanged.connect(self._on_obs_speed_slider)
        obs_spd_row.addWidget(self.slider_obs_speed)
        obs_layout.addLayout(obs_spd_row)

        panel_layout.addWidget(obs_group)

        # 3. LIVE PARAMETER TUNING
        tune_group = QGroupBox("Live SL-MPC & Scout Parameter Tuner")
        tune_layout = QVBoxLayout(tune_group)

        # Telemetry & Terminal Stream Rate (1-60 Hz)
        tune_layout.addWidget(QLabel("Telemetry & Terminal Stream Rate (1-60 Hz):"))
        telem_hz_row = QHBoxLayout()
        self.slider_telem_hz = QSlider(Qt.Horizontal)
        self.slider_telem_hz.setRange(1, 60)
        self.slider_telem_hz.setValue(self.initial_telem_hz)
        self.slider_telem_hz.valueChanged.connect(self._on_telem_hz_slider)
        self.lbl_telem_hz = QLabel(f"{self.initial_telem_hz} Hz")
        self.lbl_telem_hz.setFont(QFont("Consolas", 10, QFont.Bold))
        self.lbl_telem_hz.setStyleSheet("color: #00e5ff;")
        self.lbl_telem_hz.setToolTip("Adjusts telemetry streaming frequency to GUI and Webots terminal between 1 and 60 Hz.")
        telem_hz_row.addWidget(self.slider_telem_hz)
        telem_hz_row.addWidget(self.lbl_telem_hz)
        tune_layout.addLayout(telem_hz_row)

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
        self.telemetry_group = QGroupBox(f"Live Controller Telemetry ({self.initial_telem_hz} Hz)")
        telem_layout = QVBoxLayout(self.telemetry_group)
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
        panel_layout.addWidget(self.telemetry_group)

        # 5. DISPLAY LAYER TOGGLES
        disp_group = QGroupBox("Layer Display Options")
        disp_layout = QGridLayout(disp_group)

        self.chk_best = QCheckBox("Best Trajectory (Gold)")
        self.chk_best.setChecked(True)
        self.chk_best.toggled.connect(lambda v: setattr(self.canvas, "show_best", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_best, 0, 0)

        self.chk_cand = QCheckBox("Candidate / Alt Line (Cyan/Purp)")
        self.chk_cand.setChecked(True)
        self.chk_cand.toggled.connect(lambda v: setattr(self.canvas, "show_candidates", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_cand, 0, 1)

        self.chk_corridor = QCheckBox("MPC Corridor (Teal)")
        self.chk_corridor.setChecked(True)
        self.chk_corridor.toggled.connect(lambda v: setattr(self.canvas, "show_corridor", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_corridor, 1, 0)

        self.chk_guide = QCheckBox("Spline Guide (Cyan Dash)")
        self.chk_guide.setChecked(True)
        self.chk_guide.toggled.connect(lambda v: setattr(self.canvas, "show_guide", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_guide, 1, 1)

        self.chk_shield = QCheckBox("Emergency Shield (Green/Red)")
        self.chk_shield.setChecked(True)
        self.chk_shield.setToolTip("Dynamic micro-rollout emergency escape branches evaluated when path is blocked")
        self.chk_shield.toggled.connect(lambda v: setattr(self.canvas, "show_shield", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_shield, 2, 0)

        self.chk_backup = QCheckBox("Backup Escape Curves (Magenta)")
        self.chk_backup.setChecked(True)
        self.chk_backup.setToolTip("Thin kinematic escape arcs evaluated against costmap to decide reverse direction")
        self.chk_backup.toggled.connect(lambda v: setattr(self.canvas, "show_backup_curves", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_backup, 2, 1)

        self.chk_gap = QCheckBox("Scouted Gap (Green)")
        self.chk_gap.setChecked(True)
        self.chk_gap.toggled.connect(lambda v: setattr(self.canvas, "show_gap", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_gap, 3, 0)

        self.chk_hazard = QCheckBox("Hazard Mask (Red)")
        self.chk_hazard.setChecked(True)
        self.chk_hazard.toggled.connect(lambda v: setattr(self.canvas, "show_hazard", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_hazard, 3, 1)

        self.chk_costmap = QCheckBox("Costmap (15mm)")
        self.chk_costmap.setChecked(True)
        self.chk_costmap.toggled.connect(lambda v: setattr(self.canvas, "show_costmap", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_costmap, 4, 0)

        self.chk_rays = QCheckBox("ToF Cones (16ch)")
        self.chk_rays.setChecked(True)
        self.chk_rays.toggled.connect(lambda v: setattr(self.canvas, "show_rays", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_rays, 4, 1)

        self.chk_chassis = QCheckBox("Chassis Footprint")
        self.chk_chassis.setChecked(True)
        self.chk_chassis.toggled.connect(lambda v: setattr(self.canvas, "show_chassis", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_chassis, 5, 0)

        self.chk_grid = QCheckBox("Grid & Circles")
        self.chk_grid.setChecked(True)
        self.chk_grid.toggled.connect(lambda v: setattr(self.canvas, "show_grid", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_grid, 5, 1)

        self.chk_trail = QCheckBox("Historic Trail (Cyan)")
        self.chk_trail.setChecked(True)
        self.chk_trail.toggled.connect(lambda v: setattr(self.canvas, "show_trail", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_trail, 6, 0)

        self.chk_sightlines = QCheckBox("Wrong-Way Rays (Cyan/Red)")
        self.chk_sightlines.setChecked(True)
        self.chk_sightlines.toggled.connect(lambda v: setattr(self.canvas, "show_sightlines", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_sightlines, 6, 1)

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

        card_front, self.lbl_card_front = make_card("FRONT CLEARANCE", "> 2.40 m", "#00e676")
        card_left, self.lbl_card_left = make_card("LEFT FLANK", "> 2.40 m", "#00e676")
        card_right, self.lbl_card_right = make_card("RIGHT FLANK", "> 2.40 m", "#00e676")
        card_min, self.lbl_card_min = make_card("MINIMUM RANGE", "> 2.40 m", "#00e676")

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

        fl_group = QGroupBox("Front-Left Array (Cross-Pointing Right)")
        fl_layout = QVBoxLayout(fl_group)
        fl_layout.setContentsMargins(6, 6, 6, 6)
        fl_layout.setSpacing(4)

        fr_group = QGroupBox("Front-Right Array (Cross-Pointing Left)")
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
            bar.setRange(0, 240)
            bar.setValue(240)
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

            val_lbl = QLabel("2.40m")
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

        min_f = min([float(tof.get(k, 2.40)) for k in front_beams], default=2.40)
        min_l = min([float(tof.get(k, 2.40)) for k in left_beams], default=2.40)
        min_r = min([float(tof.get(k, 2.40)) for k in right_beams], default=2.40)

        if not hasattr(self, "_last_bar_colors"):
            self._last_bar_colors = {}

        for s_name, (bar, val_lbl, name_lbl) in self.tof_bar_widgets.items():
            if s_name in tof:
                r_m = float(tof[s_name])
                if r_m < min_overall:
                    min_overall = r_m

                cm_val = int(min(max(r_m * 100.0, 0.0), 240.0))
                bar.setValue(cm_val)

                if r_m < 0.25:
                    bar_col = "#ff3d00"
                elif r_m < 0.50:
                    bar_col = "#ffb300"
                else:
                    bar_col = "#00e676"

                if self._last_bar_colors.get(s_name) != bar_col:
                    self._last_bar_colors[s_name] = bar_col
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
            if val_m >= 2.38:
                lbl.setText("> 2.40 m")
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
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except Exception:
            pass
        try:
            # 512 KB receive buffer to prevent frame dropping during 60 Hz rendering
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 524288)
        except Exception:
            pass

        bound = False
        # Bind to 127.0.0.1 only in --local-only mode (sim preview); otherwise 0.0.0.0 to accept Pi 4 over Wi-Fi too
        bind_ips = ["127.0.0.1"] if self.local_only else ["0.0.0.0", "127.0.0.1"]
        for bind_ip in bind_ips:
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
                # Drain backlog to always stay on newest real-time frame
                while True:
                    try:
                        sock.setblocking(False)
                        next_data, _ = sock.recvfrom(65535)
                        data = next_data
                    except (BlockingIOError, socket.error):
                        break
                    finally:
                        sock.setblocking(True)
                        sock.settimeout(0.5)

                if data[:4] == b"FOLK":
                    pkt = parse_binary_packet(data)
                else:
                    pkt = json.loads(data.decode("utf-8"))

                if pkt:
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

    def _on_telem_hz_slider(self, val: int):
        self.lbl_telem_hz.setText(f"{val} Hz")
        self.telemetry_group.setTitle(f"Live Controller Telemetry ({val} Hz)")
        self._send_cmd({"cmd": "set_telemetry_hz", "val": int(val)})
        self._send_cmd({"cmd": "set_param", "name": "telemetry_hz", "val": int(val)})

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

    def _toggle_obs_direction(self):
        self.obs_direction = -1 if getattr(self, "obs_direction", 1) > 0 else 1
        if self.obs_direction > 0:
            self.btn_obs_dir.setText("▶ Forward (+)")
            self.btn_obs_dir.setStyleSheet("background-color: #1b5e20; color: #a5d6a7; font-weight: bold; padding: 5px; border-radius: 4px;")
        else:
            self.btn_obs_dir.setText("◀ Reverse (-)")
            self.btn_obs_dir.setStyleSheet("background-color: #b71c1c; color: #ffcdd2; font-weight: bold; padding: 5px; border-radius: 4px;")
        print(f"[GUI] Dynamic obstacle direction toggled to: {'Forward (+)' if self.obs_direction > 0 else 'Reverse (-)'}", flush=True)
        self._send_cmd({"cmd": "set_obstacle_direction", "val": int(self.obs_direction)})
        self._send_cmd({"cmd": "set_param", "name": "obstacle_direction", "val": int(self.obs_direction)})

    def _on_obs_speed_slider(self, val: int):
        spd = val / 10.0
        self.lbl_obs_speed.setText(f"{spd:.2f} m/s")
        print(f"[GUI] Dynamic obstacle speed set to: {spd:.2f} m/s", flush=True)
        self._send_cmd({"cmd": "set_obstacle_speed", "val": float(spd)})
        self._send_cmd({"cmd": "set_param", "name": "obstacle_speed", "val": float(spd)})

    def _reset_obs_positions(self):
        print("[GUI] Reset dynamic moving obstacles positions", flush=True)
        self._send_cmd({"cmd": "reset_obstacles"})

    def _update_gui(self):
        # Periodic check for updated champion/active config file (e.g. every 0.5s)
        self._cfg_check_counter = getattr(self, "_cfg_check_counter", 0) + 1
        if self._cfg_check_counter >= 30:
            self._cfg_check_counter = 0
            if self.canvas.reload_config():
                self.canvas.update()

        pkt = None
        is_conn = False
        with self._lock:
            pkt = self.latest_pkt
            is_conn = self.udp_connected

        if is_conn:
            hz_val = self.slider_telem_hz.value()
            self.lbl_conn.setText(f"🟢 LIVE WEBOTS {hz_val}Hz STREAM CONNECTED")
            self.lbl_conn.setStyleSheet("font-weight: bold; color: #00e676;")
        else:
            self.lbl_conn.setText("🔴 WAITING FOR WEBOTS: Run track_racing.wbt")
            self.lbl_conn.setStyleSheet("font-weight: bold; color: #ff5252;")

        if pkt is not None:
            # Check for telemetry_hz in packet to sync slider if updated externally
            pkt_hz = pkt.get("telemetry_hz", None)
            if pkt_hz is not None and int(pkt_hz) != self.slider_telem_hz.value():
                self.slider_telem_hz.blockSignals(True)
                self.slider_telem_hz.setValue(int(pkt_hz))
                self.lbl_telem_hz.setText(f"{int(pkt_hz)} Hz")
                self.telemetry_group.setTitle(f"Live Controller Telemetry ({int(pkt_hz)} Hz)")
                self.slider_telem_hz.blockSignals(False)

            # Sync obstacle direction if received in packet
            if "obstacle_direction" in pkt:
                pkt_dir = int(pkt["obstacle_direction"])
                if getattr(self, "obs_direction", 1) != pkt_dir:
                    self.obs_direction = pkt_dir
                    if self.obs_direction > 0:
                        self.btn_obs_dir.setText("▶ Forward (+)")
                        self.btn_obs_dir.setStyleSheet("background-color: #1b5e20; color: #a5d6a7; font-weight: bold; padding: 5px; border-radius: 4px;")
                    else:
                        self.btn_obs_dir.setText("◀ Reverse (-)")
                        self.btn_obs_dir.setStyleSheet("background-color: #b71c1c; color: #ffcdd2; font-weight: bold; padding: 5px; border-radius: 4px;")

            # Decompress or unpack grid
            prob_map = pkt.get("prob_map")
            hazard_mask = pkt.get("hazard_mask")
            if prob_map is None and "grid_data" in pkt and "grid_shape" in pkt:
                try:
                    shape = tuple(pkt["grid_shape"])
                    raw_b = zlib.decompress(base64.b64decode(pkt["grid_data"]))
                    prob_map = (np.frombuffer(raw_b, dtype=np.uint8).reshape(shape) / 255.0).astype(np.float32)

                    raw_h = zlib.decompress(base64.b64decode(pkt["hazard_data"]))
                    hazard_mask = np.frombuffer(raw_h, dtype=np.uint8).reshape(shape)
                except Exception:
                    pass
            elif prob_map is None and ("tof_ranges" in pkt or "tof" in pkt):
                # Local 1:1 C++ costmap reconstruction on laptop (zero Pi overhead, bit-for-bit fidelity)
                if not hasattr(self, "_local_costmap"):
                    try:
                        from cpp_smart_hybrid_costmap import CppSmartHybridCostmap
                        self._local_costmap = CppSmartHybridCostmap()
                    except Exception:
                        self._local_costmap = None
                if self._local_costmap is not None:
                    try:
                        tof_data = pkt.get("tof") or pkt.get("tof_ranges")
                        prob_map, hazard_mask, _ = self._local_costmap.update(tof_data)
                    except Exception:
                        pass

            best_line = pkt.get("best_line", [])
            cand_lines = pkt.get("candidate_lines", [])
            guide_line = pkt.get("guide_line", [])
            corridor_left = pkt.get("corridor_left", [])
            corridor_right = pkt.get("corridor_right", [])
            shield_info = pkt.get("shield", {})
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
            scout_ms = pkt.get("scout_ms", 0.0)
            opt_prim_ms = pkt.get("opt_prim_ms", 0.0)
            opt_alt_ms = pkt.get("opt_alt_ms", 0.0)
            opt_shield_ms = pkt.get("opt_shield_ms", 0.0)
            step_ms = pkt.get("step_ms", 0.0)
            iters_prim = pkt.get("iters_prim", pkt.get("osqp_iters", 0))
            iters_alt = pkt.get("iters_alt", pkt.get("osqp_iters_alt", 0))
            step_cpu_ms = pkt.get("step_cpu_ms", 0.0)
            gc_ms = pkt.get("gc_ms", 0.0)
            status = pkt.get("status", "--")
            surviving = pkt.get("surviving", 0)
            rec_state = pkt.get("escape_state", pkt.get("recovery_state", mode))
            sim_time = pkt.get("sim_time", 0.0)

            # Anti-Wrong-Way & Breadcrumb Tracking telemetry
            aww = pkt.get("aww", {})
            opt1 = aww.get("opt1", {}) if isinstance(aww, dict) else {}
            trail = aww.get("trail", pkt.get("trail", [])) if isinstance(aww, dict) else pkt.get("trail", [])
            sightlines = opt1.get("sightlines", pkt.get("sightlines", [])) if isinstance(opt1, dict) else pkt.get("sightlines", [])
            is_wrong_way = bool(
                (isinstance(aww, dict) and (aww.get("is_wrong_way", False) or aww.get("action", "") == "CORRECTIVE_180"))
                or ("180" in str(rec_state))
            )
            target_yaw = float(aww.get("target_yaw", 0.0)) if isinstance(aww, dict) else 0.0
            pose = pkt.get("pose", [0.0, 0.0, 0.0])

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
                guide_line=guide_line,
                corridor_left=corridor_left,
                corridor_right=corridor_right,
                shield_info=shield_info,
                trail=trail,
                sightlines=sightlines,
                is_wrong_way=is_wrong_way,
                target_yaw=target_yaw,
                pose=pose,
            )

            self.lbl_mode_badge.setText(f"Mode: {mode} | Sim Time: {sim_time:.1f}s")
            self.lbl_dyn.setText(f"Speed: {v_est:.2f} m/s (tgt: {v_tgt:.2f}) | Steer: {steer:+5.1f}°")
            if gap and len(gap) >= 3:
                self.lbl_gap.setText(f"Gap: ({gap[0]:+.2f}, {gap[1]:.2f})m | Width: {gap[2]*100:.1f} cm")
            gc_str = f" GC:{gc_ms:.1f}ms" if gc_ms > 0.0 else ""
            self.lbl_perf.setText(
                f"Plan: {p_ms:.1f}ms (Map:{m_ms:.1f} Sc:{scout_ms:.1f} Prim:{opt_prim_ms:.1f} Alt:{opt_alt_ms:.1f} It:{iters_prim}/{iters_alt} Step:{step_ms:.1f}ms Cpu:{step_cpu_ms:.1f}ms{gc_str}) | Surv: {surviving} | {status}"
            )
            
            # Recovery & Shield state label (with dynamic obstacle & wrong-way indicators)
            if is_wrong_way:
                self.lbl_recovery.setText("Wrong-Way: 🚨 WRONG-WAY DETECTED (Corrective 180°)")
                self.lbl_recovery.setStyleSheet("color: #ff1744; font-weight: bold;")
            elif shield_info and shield_info.get("active", False):
                c_name = shield_info.get("chosen_name", "ACTIVE").replace("_", " ").upper()
                self.lbl_recovery.setText(f"Shield: 🛡️ ACTIVE ESCAPE ({c_name})")
                self.lbl_recovery.setStyleSheet("color: #00e676; font-weight: bold;")
            elif "RECOVERY" in str(rec_state).upper() or "PIVOT" in str(rec_state).upper():
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
    import argparse
    parser = argparse.ArgumentParser(description="Live Telemetry & Trajectory GUI")
    parser.add_argument("--telemetry-hz", "--hz", type=int, default=int(os.environ.get("WEBOTS_TELEMETRY_HZ", 60)), help="Initial streaming frequency (1-60 Hz)")
    parser.add_argument("--local-only", action="store_true", help="Bind to 127.0.0.1 only (ignore remote Pi 4 UDP telemetry)")
    args, _ = parser.parse_known_args()

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

    gui = ComprehensivePlannerGUI(initial_telem_hz=args.telemetry_hz, local_only=args.local_only)
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    launch_gui()
