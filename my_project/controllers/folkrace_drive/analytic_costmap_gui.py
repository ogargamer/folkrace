"""
================================================================================
  ANALYTIC 16-CHANNEL TOF COSTMAP & MPPI DECISION-MAKING DEBUGGER GUI
================================================================================
  Comprehensive Diagnostic & Analytical Visualization Suite:
    - Robot-Centric Dynamic Zooming & Multi-Touch/Drag Canvas Panning
    - MPPI Softmax Seed Allocation Gauges (Center, Straight, Left, Right, Cusp)
    - Granular MPPI Cost Breakdown (CTE, Heading, Speed, Hazard, Smoothness)
    - 16-Channel Real-Time ToF Range Radar & Graphical Bar Charts
    - Dual-Point Chassis Overhang Collision Inspector (Front + Rear Overhangs)
    - Full Simulation Controls (Play, Pause, Step 10ms, Speed 0.05x - Max)
    - Multi-Layer Toggleable Overlays & Turbo/Inferno/Viridis Colormaps
================================================================================
"""

import os
import sys
import math
import json
import time
import socket
import threading
import base64
import zlib
from typing import Dict, List, Optional, Tuple, Any
import numpy as np

# Add project roots to path
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
MY_PROJECT_DIR = os.path.dirname(CONTROLLERS_DIR)
WORKSPACE_ROOT = os.path.dirname(MY_PROJECT_DIR)
FOLKRACE_DIR = os.path.join(CONTROLLERS_DIR, "folkrace_drive")
MPPI_CONTROLLER_DIR = os.path.join(CONTROLLERS_DIR, "mppi_controller")

for p in [CURR_DIR, CONTROLLERS_DIR, MY_PROJECT_DIR, WORKSPACE_ROOT, FOLKRACE_DIR, MPPI_CONTROLLER_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from fast_tof_carto_costmap import (
    FastCostmapPipeline,
    FastToFLUTCostmap,
    SENSOR_RIG,
    ToFSensor,
    StampedPose,
    load_cartographer_map,
)

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
    from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF
    from PyQt5.QtGui import QColor, QPainter, QPen, QBrush, QFont, QImage, QPixmap, QPainterPath
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QLabel, QSlider, QComboBox, QPushButton, QCheckBox, QGroupBox,
        QDoubleSpinBox, QTabWidget, QSplitter, QStatusBar, QFrame,
        QProgressBar, QScrollArea, QSizePolicy
    )
    PYQT_AVAILABLE = True
except ImportError:
    PYQT_AVAILABLE = False


# ==============================================================================
# 1. ROBOT-CENTRIC 2D COSTMAP & MPPI DECISION CANVAS
# ==============================================================================
class AnalyticCostmapCanvas(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(600, 680)
        self.setMouseTracking(True)

        # Costmap grid data
        self.prob_map: Optional[np.ndarray] = None
        self.hazard_mask: Optional[np.ndarray] = None
        self.tof_readings: Dict[str, float] = {s.name: 1.80 for s in SENSOR_RIG}

        # Costmap metric grid extent (3.0m wide x 2.9m reach)
        self.map_x_min = -1.5
        self.map_x_max = 1.5
        self.map_y_min = -0.5
        self.map_y_max = 2.4

        # Viewport bounds centered around robot (0, 0)
        # Default view: shows -1.8m to +1.8m lateral, -0.6m rear to +2.6m forward
        self.x_min = -1.8
        self.x_max = 1.8
        self.y_min = -0.6
        self.y_max = 2.6

        # Mouse panning state
        self.is_panning = False
        self.pan_start_screen = QtCore.QPoint()
        self.pan_start_bounds = (self.x_min, self.x_max, self.y_min, self.y_max)

        # Hover inspector state
        self.hover_world: Optional[Tuple[float, float]] = None
        self.hover_prob: Optional[float] = None
        self.hover_hazard: Optional[bool] = None

        # Display settings & Layer toggles
        self.colormap_name = "Turbo"
        self.show_rays = True
        self.show_beam_labels = True
        self.show_hazard_mask = True
        self.show_chassis = True
        self.show_overhang_points = True
        self.show_grid = True
        self.show_rings = True
        self.show_all_rollouts = True
        self.show_best_rollout = True
        self.show_seed_lines = True
        self.show_track_ref = True
        self.show_walls = True
        self.show_speed_vectors = True

        # Live trajectory rollouts & diagnostic data
        self.best_rollout: Optional[List[List[float]]] = None
        self.seed_center: Optional[List[List[float]]] = None
        self.seed_straight: Optional[List[List[float]]] = None
        self.seed_left: Optional[List[List[float]]] = None
        self.seed_right: Optional[List[List[float]]] = None
        self.candidate_rollouts: Optional[List[List[List[float]]]] = None
        self.candidate_costs: Optional[List[float]] = None

        # Rollout styling & sensitivity parameters
        self.cost_gradient_cap: float = 300.0
        self.rollout_alpha: int = 175
        self.rollout_width: float = 1.6

        # Global track reference lines & walls (transformed to vehicle local frame)
        self.track_ref_center: Optional[List[List[float]]] = None
        self.track_ref_left: Optional[List[List[float]]] = None
        self.track_ref_right: Optional[List[List[float]]] = None
        self.track_walls: Optional[List[List[List[float]]]] = None

        # Vehicle active kinematics state
        self.active_v = 0.0
        self.active_steer = 0.0
        self.cmd_v = 0.0
        self.cmd_steer = 0.0

    def set_data(
        self,
        prob_map: Optional[np.ndarray],
        hazard_mask: Optional[np.ndarray],
        tof_readings: Dict[str, float],
        active_v: float = 0.0,
        active_steer: float = 0.0,
        cmd_v: float = 0.0,
        cmd_steer: float = 0.0,
    ):
        self.prob_map = prob_map
        self.hazard_mask = hazard_mask
        self.tof_readings = tof_readings
        self.active_v = active_v
        self.active_steer = active_steer
        self.cmd_v = cmd_v
        self.cmd_steer = cmd_steer
        self.update()

    def set_trajectories(
        self,
        best: Optional[List[List[float]]],
        candidates: Optional[List[List[List[float]]]],
        candidate_costs: Optional[List[float]] = None,
        seed_center: Optional[List[List[float]]] = None,
        seed_straight: Optional[List[List[float]]] = None,
        seed_left: Optional[List[List[float]]] = None,
        seed_right: Optional[List[List[float]]] = None,
        track_ref_center: Optional[List[List[float]]] = None,
        track_ref_left: Optional[List[List[float]]] = None,
        track_ref_right: Optional[List[List[float]]] = None,
        track_walls: Optional[List[List[List[float]]]] = None,
    ):
        self.best_rollout = best
        self.candidate_rollouts = candidates
        self.candidate_costs = candidate_costs
        self.seed_center = seed_center
        self.seed_straight = seed_straight
        self.seed_left = seed_left
        self.seed_right = seed_right
        self.track_ref_center = track_ref_center
        self.track_ref_left = track_ref_left
        self.track_ref_right = track_ref_right
        self.track_walls = track_walls
        self.update()

    def reset_view(self):
        """Resets viewport centered on the robot at (0.0, 0.0)."""
        self.x_min = -1.8
        self.x_max = 1.8
        self.y_min = -0.6
        self.y_max = 2.6
        self.update()

    def zoom_on_robot(self, factor: float):
        """
        Zooms precisely centered on the vehicle origin (0.0, 0.0).
        Preserves vehicle centering at all zoom levels.
        """
        # Calculate metric spans relative to robot origin (0.0, 0.0)
        span_x_left = (0.0 - self.x_min) * factor
        span_x_right = (self.x_max - 0.0) * factor
        span_y_rear = (0.0 - self.y_min) * factor
        span_y_fwd = (self.y_max - 0.0) * factor

        # Clamp zoom limits: max zoom-in (0.4m box) to max zoom-out (8.0m box)
        if (span_x_left + span_x_right) < 0.40 or (span_x_left + span_x_right) > 8.0:
            return

        self.x_min = -span_x_left
        self.x_max = span_x_right
        self.y_min = -span_y_rear
        self.y_max = span_y_fwd
        self.update()

    def wheelEvent(self, event):
        """Interactive mouse wheel zoom directly focused on the vehicle."""
        delta = event.angleDelta().y()
        if delta > 0:
            self.zoom_on_robot(0.85)  # Zoom in onto robot
        elif delta < 0:
            self.zoom_on_robot(1.18)  # Zoom out from robot

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_panning = True
            self.pan_start_screen = event.pos()
            self.pan_start_bounds = (self.x_min, self.x_max, self.y_min, self.y_max)
        elif event.button() == Qt.RightButton:
            self.reset_view()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_panning = False

    def mouseMoveEvent(self, event):
        if self.is_panning:
            w, h = self.width(), self.height()
            dx_px = event.x() - self.pan_start_screen.x()
            dy_px = event.y() - self.pan_start_screen.y()

            init_x_min, init_x_max, init_y_min, init_y_max = self.pan_start_bounds
            scale_x = (init_x_max - init_x_min) / w
            scale_y = (init_y_max - init_y_min) / h

            dx_m = -dx_px * scale_x
            dy_m = dy_px * scale_y

            self.x_min = init_x_min + dx_m
            self.x_max = init_x_max + dx_m
            self.y_min = init_y_min + dy_m
            self.y_max = init_y_max + dy_m
            self.update()
            return

        # Hover inspector
        xm, ym = self.screen_to_world(event.x(), event.y())
        self.hover_world = (xm, ym)

        if self.prob_map is not None:
            H, W = self.prob_map.shape
            gx = int((xm - self.map_x_min) / (self.map_x_max - self.map_x_min) * W)
            gy = int((ym - self.map_y_min) / (self.map_y_max - self.map_y_min) * H)
            if 0 <= gx < W and 0 <= gy < H:
                self.hover_prob = float(self.prob_map[gy, gx])
                self.hover_hazard = bool(self.hazard_mask[gy, gx] > 0) if self.hazard_mask is not None else False
            else:
                self.hover_prob = None
                self.hover_hazard = None
        self.update()

    def world_to_screen(self, xm: float, ym: float) -> QPointF:
        """Transforms vehicle local Cartesian coordinates (+X right, +Y forward) to screen pixels."""
        w, h = self.width(), self.height()
        px = (xm - self.x_min) / (self.x_max - self.x_min) * w
        py = (1.0 - (ym - self.y_min) / (self.y_max - self.y_min)) * h
        return QPointF(px, py)

    def screen_to_world(self, px: float, py: float) -> Tuple[float, float]:
        w, h = self.width(), self.height()
        xm = self.x_min + (px / w) * (self.x_max - self.x_min)
        ym = self.y_min + (1.0 - py / h) * (self.y_max - self.y_min)
        return float(xm), float(ym)

    def get_step_cost_color(self, xm: float, ym: float, base_cost_norm: float = 0.0, alpha: int = 255) -> QColor:
        """Computes high-visibility per-step cost gradient color at a specific local metric point."""
        step_risk = 0.0
        if self.prob_map is not None:
            H, W = self.prob_map.shape
            gx = int((xm - self.map_x_min) / (self.map_x_max - self.map_x_min) * W)
            gy = int((ym - self.map_y_min) / (self.map_y_max - self.map_y_min) * H)
            if 0 <= gx < W and 0 <= gy < H:
                p_occ = float(self.prob_map[gy, gx])
                is_haz = bool(self.hazard_mask[gy, gx] > 0) if self.hazard_mask is not None else False
                step_risk = p_occ + (1.2 if is_haz else 0.0)

        # Blend base rollout cost (40%) with instantaneous local costmap risk (60%)
        combined_norm = min(1.0, max(0.0, base_cost_norm * 0.40 + step_risk * 0.60))

        # Color gradient: 0.0 (Neon Cyan/Lime) -> 0.35 (Green) -> 0.70 (Amber/Yellow) -> 1.0 (Bright Neon Red)
        if combined_norm < 0.35:
            t = combined_norm / 0.35
            r = int(0 * (1 - t) + 0 * t)
            g = int(229 * (1 - t) + 230 * t)
            b = int(255 * (1 - t) + 118 * t)
        elif combined_norm < 0.70:
            t = (combined_norm - 0.35) / 0.35
            r = int(0 * (1 - t) + 255 * t)
            g = int(230 * (1 - t) + 215 * t)
            b = int(118 * (1 - t) + 64 * t)
        else:
            t = (combined_norm - 0.70) / 0.30
            r = int(255 * (1 - t) + 255 * t)
            g = int(215 * (1 - t) + 23 * t)
            b = int(64 * (1 - t) + 68 * t)

        return QColor(r, g, b, alpha)

    # --------------------------------------------------------------------------
    # COLORMAP GENERATION
    # --------------------------------------------------------------------------
    def _get_qimage_from_prob_map(self) -> QImage:
        """Converts (H, W) float prob map to colored QImage with selected palette."""
        H, W = self.prob_map.shape
        prob = np.clip(self.prob_map, 0.0, 1.0)
        h_mask = self.hazard_mask if self.hazard_mask is not None else np.zeros((H, W), dtype=bool)

        # Build RGB array
        rgb = np.zeros((H, W, 4), dtype=np.uint8)

        if self.colormap_name == "Turbo":
            # Turbo palette approximation
            r = np.clip(np.sin(np.pi * (prob - 0.25)) * 1.4, 0.0, 1.0) * 255.0
            g = np.clip(np.sin(np.pi * (prob - 0.05)) * 1.2, 0.0, 1.0) * 255.0
            b = np.clip(np.cos(np.pi * (prob - 0.15)) * 1.3, 0.0, 1.0) * 255.0
            rgb[..., 0] = r.astype(np.uint8)
            rgb[..., 1] = g.astype(np.uint8)
            rgb[..., 2] = b.astype(np.uint8)
            rgb[..., 3] = (np.clip(prob * 1.4, 0.15, 0.90) * 255).astype(np.uint8)
        elif self.colormap_name == "Inferno":
            rgb[..., 0] = np.clip(prob * 255 * 1.1, 0, 255).astype(np.uint8)
            rgb[..., 1] = np.clip((prob ** 1.8) * 255, 0, 255).astype(np.uint8)
            rgb[..., 2] = np.clip((prob ** 3.0) * 255, 0, 255).astype(np.uint8)
            rgb[..., 3] = (np.clip(prob * 1.5, 0.20, 0.95) * 255).astype(np.uint8)
        else:  # Grayscale
            gray = (prob * 255).astype(np.uint8)
            rgb[..., 0] = gray
            rgb[..., 1] = gray
            rgb[..., 2] = gray
            rgb[..., 3] = 230

        # Overlay bright red on lethal binary C-Space hazard cells
        if self.show_hazard_mask:
            haz_idx = (h_mask > 0)
            rgb[haz_idx, 0] = 255
            rgb[haz_idx, 1] = 20
            rgb[haz_idx, 2] = 20
            rgb[haz_idx, 3] = 240

        # Row 0 in numpy is y_min (-0.5m), row H is y_max (+2.4m) -> flip vertically for top-down image
        rgb_flipped = np.ascontiguousarray(np.flipud(rgb))
        return QImage(rgb_flipped.data, W, H, W * 4, QImage.Format_RGBA8888)

    # --------------------------------------------------------------------------
    # CANVAS RENDERING PIPELINE
    # --------------------------------------------------------------------------
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor(10, 14, 20))

        # 1. Draw Costmap Heatmap & Binary Hazard Layer
        if self.prob_map is not None:
            qimg = self._get_qimage_from_prob_map()
            p_tl = self.world_to_screen(self.map_x_min, self.map_y_max)
            p_br = self.world_to_screen(self.map_x_max, self.map_y_min)
            dest_rect = QRectF(p_tl.x(), p_tl.y(), p_br.x() - p_tl.x(), p_br.y() - p_tl.y())
            painter.drawImage(dest_rect, qimg)

            # Costmap bounding box
            painter.setPen(QPen(QColor(70, 100, 140, 120), 1, Qt.DotLine))
            painter.setBrush(Qt.NoBrush)
            painter.drawRect(dest_rect)

        # 2. Draw Distance Rings & Cartesian Grid
        if self.show_rings:
            ring_pen = QPen(QColor(55, 80, 110, 80), 1, Qt.DashLine)
            painter.setPen(ring_pen)
            painter.setFont(QFont("Consolas", 8))
            for r in [0.25, 0.50, 1.00, 1.50, 1.80]:
                r_tl = self.world_to_screen(-r, r)
                r_br = self.world_to_screen(r, -r)
                painter.drawEllipse(QRectF(r_tl.x(), r_tl.y(), r_br.x() - r_tl.x(), r_br.y() - r_tl.y()))
                p_lbl = self.world_to_screen(r * 0.707, r * 0.707)
                painter.setPen(QPen(QColor(100, 150, 200, 140)))
                painter.drawText(QPointF(p_lbl.x() + 4, p_lbl.y() - 2), f"{r:.2f}m")
                painter.setPen(ring_pen)

        if self.show_grid:
            grid_pen = QPen(QColor(40, 60, 85, 90), 1, Qt.DotLine)
            painter.setPen(grid_pen)
            painter.setFont(QFont("Consolas", 8))
            for ym in [-0.4, 0.0, 0.5, 1.0, 1.5, 2.0, 2.4]:
                p1 = self.world_to_screen(self.x_min, ym)
                p2 = self.world_to_screen(self.x_max, ym)
                painter.drawLine(p1, p2)
                painter.setPen(QPen(QColor(90, 130, 170, 160)))
                painter.drawText(QPointF(p1.x() + 6, p1.y() - 3), f"Y={ym:+.1f}m")
                painter.setPen(grid_pen)

            for xm in [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5]:
                p1 = self.world_to_screen(xm, self.y_min)
                p2 = self.world_to_screen(xm, self.y_max)
                painter.drawLine(p1, p2)

        # 3. Draw 16 Conical ToF Sensor Beams & Measured Distance Arcs
        if self.show_rays:
            for s in SENSOR_RIG:
                dist = self.tof_readings.get(s.name, 1.80)
                sp = self.world_to_screen(s.x0, s.y0)
                
                ang_left = s.heading_rad - s.fov_rad / 2.0
                ang_right = s.heading_rad + s.fov_rad / 2.0
                
                p_l = self.world_to_screen(s.x0 + dist * math.cos(ang_left), s.y0 + dist * math.sin(ang_left))
                p_r = self.world_to_screen(s.x0 + dist * math.cos(ang_right), s.y0 + dist * math.sin(ang_right))
                p_mid = self.world_to_screen(s.x0 + dist * math.cos(s.heading_rad), s.y0 + dist * math.sin(s.heading_rad))

                if dist < 0.15:
                    cone_color = QColor(255, 40, 40, 75)
                    edge_color = QColor(255, 60, 60, 200)
                elif dist < 0.40:
                    cone_color = QColor(255, 170, 0, 55)
                    edge_color = QColor(255, 180, 0, 180)
                else:
                    cone_color = QColor(0, 230, 118, 30)
                    edge_color = QColor(0, 230, 118, 120)

                path = QPainterPath()
                path.moveTo(sp)
                path.lineTo(p_l)
                path.lineTo(p_r)
                path.closeSubpath()

                painter.fillPath(path, QBrush(cone_color))
                painter.setPen(QPen(edge_color, 1, Qt.SolidLine))
                painter.drawLine(sp, p_l)
                painter.drawLine(sp, p_r)
                painter.drawLine(p_l, p_r)

                if self.show_beam_labels and dist < 1.70:
                    painter.setFont(QFont("Consolas", 7, QFont.Bold))
                    painter.setPen(QPen(QColor(255, 255, 255, 220)))
                    painter.drawText(QPointF(p_mid.x() - 10, p_mid.y() - 3), f"{dist*100:.0f}")

        # 3.5. Draw Track Outer and Inner Solid Walls (Neon Coral / Rose)
        if self.show_walls and self.track_walls:
            wall_pen = QPen(QColor(255, 64, 129, 230), 2.2, Qt.SolidLine)
            painter.setPen(wall_pen)
            for wall_poly in self.track_walls:
                if len(wall_poly) > 1:
                    for i in range(len(wall_poly) - 1):
                        p1 = self.world_to_screen(wall_poly[i][0], wall_poly[i][1])
                        p2 = self.world_to_screen(wall_poly[i + 1][0], wall_poly[i + 1][1])
                        painter.drawLine(p1, p2)
                    p_start = self.world_to_screen(wall_poly[0][0], wall_poly[0][1])
                    p_end = self.world_to_screen(wall_poly[-1][0], wall_poly[-1][1])
                    painter.drawLine(p_end, p_start)

        # 4. Draw Continuous Global Racing Lines
        if self.show_track_ref:
            if self.track_ref_left and len(self.track_ref_left) > 1:
                painter.setPen(QPen(QColor(224, 64, 251, 160), 1.4, Qt.DashLine))
                for i in range(len(self.track_ref_left) - 1):
                    painter.drawLine(
                        self.world_to_screen(self.track_ref_left[i][0], self.track_ref_left[i][1]),
                        self.world_to_screen(self.track_ref_left[i + 1][0], self.track_ref_left[i + 1][1])
                    )
                painter.drawLine(
                    self.world_to_screen(self.track_ref_left[-1][0], self.track_ref_left[-1][1]),
                    self.world_to_screen(self.track_ref_left[0][0], self.track_ref_left[0][1])
                )

            if self.track_ref_right and len(self.track_ref_right) > 1:
                painter.setPen(QPen(QColor(255, 215, 64, 160), 1.4, Qt.DashLine))
                for i in range(len(self.track_ref_right) - 1):
                    painter.drawLine(
                        self.world_to_screen(self.track_ref_right[i][0], self.track_ref_right[i][1]),
                        self.world_to_screen(self.track_ref_right[i + 1][0], self.track_ref_right[i + 1][1])
                    )
                painter.drawLine(
                    self.world_to_screen(self.track_ref_right[-1][0], self.track_ref_right[-1][1]),
                    self.world_to_screen(self.track_ref_right[0][0], self.track_ref_right[0][1])
                )

            if self.track_ref_center and len(self.track_ref_center) > 1:
                painter.setPen(QPen(QColor(0, 229, 255, 220), 1.8, Qt.SolidLine))
                for i in range(len(self.track_ref_center) - 1):
                    painter.drawLine(
                        self.world_to_screen(self.track_ref_center[i][0], self.track_ref_center[i][1]),
                        self.world_to_screen(self.track_ref_center[i + 1][0], self.track_ref_center[i + 1][1])
                    )
                painter.drawLine(
                    self.world_to_screen(self.track_ref_center[-1][0], self.track_ref_center[-1][1]),
                    self.world_to_screen(self.track_ref_center[0][0], self.track_ref_center[0][1])
                )

        # 5. Draw Candidate MPPI Rollouts with Dynamic Step-by-Step Cost Gradient
        if self.show_all_rollouts and self.candidate_rollouts:
            costs = self.candidate_costs if self.candidate_costs and len(self.candidate_costs) == len(self.candidate_rollouts) else [100.0] * len(self.candidate_rollouts)
            valid_costs = [c for c in costs if c < 100000.0]
            c_min = min(valid_costs) if valid_costs else 0.0
            c_max = max(valid_costs) if valid_costs else self.cost_gradient_cap
            c_max = min(c_max, max(c_min + 10.0, self.cost_gradient_cap))

            for idx, traj in enumerate(self.candidate_rollouts):
                cost = costs[idx] if idx < len(costs) else c_max
                is_lethal = cost >= 100000.0
                norm_base = 1.0 if is_lethal else min(1.0, max(0.0, (cost - c_min) / max(1e-3, (c_max - c_min))))
                pen_w = max(1.0, self.rollout_width - 0.4) if is_lethal else self.rollout_width

                for i in range(len(traj) - 1):
                    x1, y1 = traj[i][0], traj[i][1]
                    x2, y2 = traj[i + 1][0], traj[i + 1][1]
                    mid_x = (x1 + x2) * 0.5
                    mid_y = (y1 + y2) * 0.5
                    if is_lethal:
                        seg_color = QColor(213, 0, 0, min(self.rollout_alpha, 85))
                    else:
                        seg_color = self.get_step_cost_color(mid_x, mid_y, base_cost_norm=norm_base, alpha=self.rollout_alpha)

                    painter.setPen(QPen(seg_color, pen_w, Qt.SolidLine))
                    painter.drawLine(self.world_to_screen(x1, y1), self.world_to_screen(x2, y2))

        # 6. Draw 4 Distinct SVG-MPPI Nominal Seed Lines
        if self.show_seed_lines:
            if self.seed_left:
                painter.setPen(QPen(QColor(224, 64, 251, 220), 2.0, Qt.DashLine))
                for i in range(len(self.seed_left) - 1):
                    painter.drawLine(
                        self.world_to_screen(self.seed_left[i][0], self.seed_left[i][1]),
                        self.world_to_screen(self.seed_left[i + 1][0], self.seed_left[i + 1][1])
                    )
            if self.seed_right:
                painter.setPen(QPen(QColor(255, 171, 0, 220), 2.0, Qt.DashLine))
                for i in range(len(self.seed_right) - 1):
                    painter.drawLine(
                        self.world_to_screen(self.seed_right[i][0], self.seed_right[i][1]),
                        self.world_to_screen(self.seed_right[i + 1][0], self.seed_right[i + 1][1])
                    )
            if self.seed_straight:
                painter.setPen(QPen(QColor(255, 235, 59, 220), 2.0, Qt.DashDotLine))
                for i in range(len(self.seed_straight) - 1):
                    painter.drawLine(
                        self.world_to_screen(self.seed_straight[i][0], self.seed_straight[i][1]),
                        self.world_to_screen(self.seed_straight[i + 1][0], self.seed_straight[i + 1][1])
                    )
            if self.seed_center:
                painter.setPen(QPen(QColor(0, 229, 255, 240), 2.2, Qt.SolidLine))
                for i in range(len(self.seed_center) - 1):
                    painter.drawLine(
                        self.world_to_screen(self.seed_center[i][0], self.seed_center[i][1]),
                        self.world_to_screen(self.seed_center[i + 1][0], self.seed_center[i + 1][1])
                    )

        # 7. Draw Best Selected Trajectory with Dynamic Step-by-Step Cost Gradient & Waypoints
        if self.show_best_rollout and self.best_rollout:
            for i in range(len(self.best_rollout) - 1):
                x1, y1 = self.best_rollout[i][0], self.best_rollout[i][1]
                x2, y2 = self.best_rollout[i + 1][0], self.best_rollout[i + 1][1]
                mid_x = (x1 + x2) * 0.5
                mid_y = (y1 + y2) * 0.5

                step_color = self.get_step_cost_color(mid_x, mid_y, base_cost_norm=0.0, alpha=255)
                p1 = self.world_to_screen(x1, y1)
                p2 = self.world_to_screen(x2, y2)

                # Draw bold step segment
                best_pen = QPen(step_color, 3.4, Qt.SolidLine)
                painter.setPen(best_pen)
                painter.drawLine(p1, p2)

                # Draw glowing waypoint disc
                painter.setBrush(QBrush(step_color))
                painter.setPen(QPen(QColor(10, 15, 25, 200), 0.8))
                painter.drawEllipse(p1, 3.2, 3.2)

        # 8. Draw Vehicle Chassis, Steered Wheels & Overhang Collision Check Points
        if self.show_chassis:
            car_w = 0.150
            car_l = 0.200
            wb = 0.1325

            body_tl = self.world_to_screen(-car_w / 2.0, wb * 0.5 + 0.04)
            body_br = self.world_to_screen(car_w / 2.0, -wb * 0.5 - 0.04)
            body_rect = QRectF(body_tl.x(), body_tl.y(), body_br.x() - body_tl.x(), body_br.y() - body_tl.y())

            painter.setPen(QPen(QColor(0, 229, 255, 230), 2))
            painter.setBrush(QBrush(QColor(18, 30, 45, 210)))
            painter.drawRoundedRect(body_rect, 6, 6)

            if self.show_overhang_points:
                p_front = self.world_to_screen(0.0, 0.070)
                p_rear = self.world_to_screen(0.0, -0.070)

                painter.setPen(QPen(QColor(255, 255, 255, 200), 1))
                painter.setBrush(QBrush(QColor(0, 230, 118, 220)))
                painter.drawEllipse(p_front, 4.0, 4.0)

                painter.setBrush(QBrush(QColor(255, 171, 0, 220)))
                painter.drawEllipse(p_rear, 4.0, 4.0)

            wheel_w = 0.025
            wheel_r = 0.0275
            wheel_pos = [
                (-0.050, +wb * 0.5, self.cmd_steer),   # FL (steered)
                (+0.050, +wb * 0.5, self.cmd_steer),   # FR (steered)
                (-0.050, -wb * 0.5, 0.0),              # RL (fixed)
                (+0.050, -wb * 0.5, 0.0),              # RR (fixed)
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
                painter.drawRoundedRect(QRectF(-6, -12, 12, 24), 2, 2)
                painter.restore()

            # Heading direction arrow
            p_center = self.world_to_screen(0.0, 0.0)
            p_head = self.world_to_screen(0.0, 0.25)
            painter.setPen(QPen(QColor(0, 229, 255, 255), 2.5))
            painter.drawLine(p_center, p_head)

        # 9. Top-Left Inspector Overlay Box (Live metric hover data)
        if self.hover_world is not None:
            hx, hy = self.hover_world
            info_text = f"X: {hx:+.2f}m | Y: {hy:+.2f}m"
            if self.hover_prob is not None:
                info_text += f" | P(occ): {self.hover_prob*100:.1f}%"
            if self.hover_hazard:
                info_text += " [LETHAL HAZARD]"

            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(QColor(10, 15, 25, 210)))
            painter.drawRoundedRect(QRectF(10, 10, 270, 28), 5, 5)

            painter.setPen(QPen(QColor(0, 229, 255, 240)))
            painter.setFont(QFont("Consolas", 9, QFont.Bold))
            painter.drawText(QPointF(18, 28), info_text)


# ==============================================================================
# 2. FULL ANALYTIC DEBUGGER & CONTROL DASHBOARD WINDOW
# ==============================================================================
class AnalyticDebuggerGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Analytic 16-Channel ToF Costmap & MPPI Decision Debugger")
        self.resize(1500, 950)

        # 1. Costmap Fusion Pipeline
        self.r_inflation_m = 0.088
        self.costmap_pipe = FastCostmapPipeline(
            grid_bounds=(-1.5, 1.5, -0.5, 2.4),
            resolution=0.02,
            enable_latency_comp=True,
            r_inflation_m=self.r_inflation_m,
        )

        # 1.2. Load Cartographer 2D SLAM Baseline Static Map
        carto_yaml = os.path.join(WORKSPACE_ROOT, "cartographer_2d_slam_static_map.yaml")
        if not os.path.exists(carto_yaml):
            carto_yaml = os.path.join(WORKSPACE_ROOT, "ros2_cartographer", "maps", "folkrace_track_map.yaml")
        carto_data = self._load_cartographer_map(carto_yaml)
        if carto_data is not None:
            self.carto_grid, self.carto_origin, self.carto_res = carto_data
        else:
            self.carto_grid, self.carto_origin, self.carto_res = None, (0.0, 0.0), 0.02

        # 1.5. Load Global Track Walls & Trajectory Reference Paths
        self.global_walls = self._load_track_walls()
        self.global_traj_center = self._load_csv_points("cyan_champion_trajectory.csv")
        if self.global_traj_center is None:
            self.global_traj_center = self._load_csv_points("master_cyan_champion_optimal.csv")
        
        self.global_traj_left = self._load_csv_points("midpoint_offset_left.csv")
        if self.global_traj_left is None:
            self.global_traj_left = self._load_csv_points("wall_offset_left_25cm.csv")
            
        self.global_traj_right = self._load_csv_points("midpoint_offset_right.csv")
        if self.global_traj_right is None:
            self.global_traj_right = self._load_csv_points("wall_offset_right_25cm.csv")

        # 2. Live UDP State & Sockets (Ports: 9876 Telemetry, 9877 Commands, 9875 Cartographer)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self.map_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.map_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.map_sock.bind(("0.0.0.0", 9875))
            self.map_sock.setblocking(False)
        except Exception:
            pass

        self._latest_packet = None
        self._latest_map_pkt = None
        self._packet_lock = threading.Lock()
        self.udp_running = True
        self.sim_paused = False

        self.udp_thread = threading.Thread(target=self._udp_listener, daemon=True)
        self.udp_thread.start()

        self._build_ui()

        # Initialize clean canvas
        default_tof = {s.name: 1.80 for s in SENSOR_RIG}
        self.costmap_pipe.update_pose(0.0, 0.60, 0.00, 0.0, 0.0)
        prob, hazard, _ = self.costmap_pipe.process_tof_frame(default_tof, 0.0)
        self.canvas.set_data(prob, hazard, default_tof)

        # 50 Hz UI Refresh Timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_timer)
        self.timer.start(20)

    # --------------------------------------------------------------------------
    # UI CONSTRUCTION (Split 3-Pane Analytic Layout)
    # --------------------------------------------------------------------------
    def _build_ui(self):
        central = QWidget(self)
        self.setCentralWidget(central)
        root_layout = QHBoxLayout(central)
        root_layout.setContentsMargins(8, 8, 8, 8)
        root_layout.setSpacing(8)

        # LEFT PANE: Interactive 2D Costmap Canvas (Large)
        self.canvas = AnalyticCostmapCanvas(self)
        root_layout.addWidget(self.canvas, stretch=5)

        # RIGHT PANE: Multi-Tab Decision & Telemetry Dashboard
        sidebar = QWidget()
        side_layout = QVBoxLayout(sidebar)
        side_layout.setContentsMargins(0, 0, 0, 0)
        side_layout.setSpacing(8)

        # --- Top: Simulation Controls Bar ---
        sim_box = QGroupBox("Webots Simulation Control")
        sim_lay = QHBoxLayout(sim_box)
        sim_lay.setContentsMargins(6, 6, 6, 6)

        self.btn_play_pause = QPushButton("⏸ Pause")
        self.btn_play_pause.setStyleSheet("font-weight: bold; background-color: #2e7d32; color: white; padding: 6px;")
        self.btn_play_pause.clicked.connect(self._toggle_play_pause)
        sim_lay.addWidget(self.btn_play_pause)

        self.btn_step = QPushButton("⏭ Step 10ms")
        self.btn_step.setStyleSheet("font-weight: bold; background-color: #0277bd; color: white; padding: 6px;")
        self.btn_step.clicked.connect(self._send_step_command)
        sim_lay.addWidget(self.btn_step)

        self.btn_reset = QPushButton("↺ Reset")
        self.btn_reset.setStyleSheet("font-weight: bold; background-color: #c62828; color: white; padding: 6px;")
        self.btn_reset.clicked.connect(self._send_reset_command)
        sim_lay.addWidget(self.btn_reset)

        self.cb_speed = QComboBox()
        self.cb_speed.addItems(["0.05x", "0.10x", "0.25x", "0.50x", "1.00x", "2.00x", "Max"])
        self.cb_speed.setCurrentIndex(4)  # 1.00x default
        self.cb_speed.currentIndexChanged.connect(self._on_speed_changed)
        sim_lay.addWidget(QLabel("Speed:"))
        sim_lay.addWidget(self.cb_speed)

        side_layout.addWidget(sim_box)

        # --- Middle: Tabbed Analytic Panels ---
        self.tabs = QTabWidget()

        # TAB 1: MPPI Decisions & Cost Breakdown
        tab_mppi = QWidget()
        tab_mppi_lay = QVBoxLayout(tab_mppi)
        self._build_mppi_tab(tab_mppi_lay)
        self.tabs.addTab(tab_mppi, "MPPI Decisions")

        # TAB 2: 16-Channel ToF Radar Gauges
        tab_tof = QWidget()
        tab_tof_lay = QVBoxLayout(tab_tof)
        self._build_tof_tab(tab_tof_lay)
        self.tabs.addTab(tab_tof, "16-Channel ToF")

        # TAB 3: Layer Toggles & Viewport
        tab_layers = QWidget()
        tab_lay_layout = QVBoxLayout(tab_layers)
        self._build_layers_tab(tab_lay_layout)
        self.tabs.addTab(tab_layers, "Visual Layers")

        # TAB 4: Live Parameter Tuning
        tab_tuning = QWidget()
        tab_tun_layout = QVBoxLayout(tab_tuning)
        self._build_tuning_tab(tab_tun_layout)
        self.tabs.addTab(tab_tuning, "Live Tuning")

        side_layout.addWidget(self.tabs, stretch=1)
        root_layout.addWidget(sidebar, stretch=4)

    # --------------------------------------------------------------------------
    # TAB 1: MPPI DECISION BREAKDOWN & WEIGHTS
    # --------------------------------------------------------------------------
    def _build_mppi_tab(self, layout: QVBoxLayout):
        # 1. Decision Weights (Softmax distribution across seed channels)
        grp_weights = QGroupBox("MPPI Softmax Decision Allocation (%)")
        w_lay = QVBoxLayout(grp_weights)

        self.bar_center = QProgressBar()
        self.bar_center.setStyleSheet("QProgressBar::chunk { background-color: #00e5ff; }")
        self.bar_straight = QProgressBar()
        self.bar_straight.setStyleSheet("QProgressBar::chunk { background-color: #ffeb3b; }")
        self.bar_left = QProgressBar()
        self.bar_left.setStyleSheet("QProgressBar::chunk { background-color: #e040fb; }")
        self.bar_right = QProgressBar()
        self.bar_right.setStyleSheet("QProgressBar::chunk { background-color: #ff9100; }")
        self.bar_rev = QProgressBar()
        self.bar_rev.setStyleSheet("QProgressBar::chunk { background-color: #f44336; }")

        for lbl_name, bar in [
            ("Center Seed (Optimal Racing Line):", self.bar_center),
            ("Straight Seed (Gap Shooter delta=0):", self.bar_straight),
            ("Left Seed (+0.25m Corridor):", self.bar_left),
            ("Right Seed (-0.25m Corridor):", self.bar_right),
            ("Reverse & Cusp Recovery:", self.bar_rev),
        ]:
            row = QHBoxLayout()
            lbl = QLabel(lbl_name)
            lbl.setFont(QFont("Consolas", 8))
            row.addWidget(lbl, stretch=3)
            row.addWidget(bar, stretch=2)
            w_lay.addLayout(row)

        layout.addWidget(grp_weights)

        # 2. Performance & Pruning Metrics
        grp_metrics = QGroupBox("MPPI Optimization Performance")
        m_lay = QVBoxLayout(grp_metrics)
        self.lbl_exec_time = QLabel("Execution Time: 0.0 ms (SIMD JIT)")
        self.lbl_collision_rate = QLabel("Pruned / Collision Rollouts: 0.0% (0 / 512)")
        self.lbl_best_cost = QLabel("Best Trajectory Cost: 0.0")
        self.lbl_target_v = QLabel("Nominal Reference Speed: 0.00 m/s")

        for lbl in [self.lbl_exec_time, self.lbl_collision_rate, self.lbl_best_cost, self.lbl_target_v]:
            lbl.setFont(QFont("Consolas", 9, QFont.Bold))
            lbl.setStyleSheet("color: #00e5ff;")
            m_lay.addWidget(lbl)
        layout.addWidget(grp_metrics)

        # 3. Vehicle Kinematics Summary
        grp_veh = QGroupBox("Vehicle Kinematics & Tracking")
        v_lay = QVBoxLayout(grp_veh)
        self.lbl_speed_steer = QLabel("Speed: 0.00 m/s | Steer: +0.0°")
        self.lbl_cte_heading = QLabel("Cross-Track Error: +0.0 cm | Heading Error: +0.0°")
        self.lbl_yaw_rate = QLabel("Yaw Rate (Gyro Z): +0.00 rad/s")

        for lbl in [self.lbl_speed_steer, self.lbl_cte_heading, self.lbl_yaw_rate]:
            lbl.setFont(QFont("Consolas", 9))
            v_lay.addWidget(lbl)
        layout.addWidget(grp_veh)

        layout.addStretch()

    # --------------------------------------------------------------------------
    # TAB 2: 16-CHANNEL TOF RANGE GAUGES
    # --------------------------------------------------------------------------
    def _build_tof_tab(self, layout: QVBoxLayout):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        container = QWidget()
        c_lay = QVBoxLayout(container)

        self.tof_bars: Dict[str, QProgressBar] = {}
        self.tof_labels: Dict[str, QLabel] = {}

        groups = {
            "Front-Left Array (Cross-pointing Right)": ["tof_fl_0", "tof_fl_1", "tof_fl_2", "tof_fl_3"],
            "Front-Right Array (Cross-pointing Left)": ["tof_fr_0", "tof_fr_1", "tof_fr_2", "tof_fr_3"],
            "Back-Left Array (Fanning Left)": ["tof_bl_0", "tof_bl_1", "tof_bl_2", "tof_bl_3"],
            "Back-Right Array (Fanning Right)": ["tof_br_0", "tof_br_1", "tof_br_2", "tof_br_3"],
        }

        for grp_title, sensor_names in groups.items():
            grp = QGroupBox(grp_title)
            g_lay = QVBoxLayout(grp)
            for s_name in sensor_names:
                row = QHBoxLayout()
                lbl = QLabel(f"{s_name}:")
                lbl.setFont(QFont("Consolas", 8, QFont.Bold))
                val_lbl = QLabel("180 cm")
                val_lbl.setFont(QFont("Consolas", 8))
                val_lbl.setFixedWidth(50)

                bar = QProgressBar()
                bar.setRange(0, 180)
                bar.setValue(180)
                bar.setTextVisible(False)
                bar.setStyleSheet("QProgressBar::chunk { background-color: #00e676; }")

                self.tof_bars[s_name] = bar
                self.tof_labels[s_name] = val_lbl

                row.addWidget(lbl, stretch=2)
                row.addWidget(bar, stretch=4)
                row.addWidget(val_lbl, stretch=1)
                g_lay.addLayout(row)
            c_lay.addWidget(grp)

        scroll.setWidget(container)
        layout.addWidget(scroll)

    # --------------------------------------------------------------------------
    # TAB 3: VISUAL LAYERS & ROBOT-CENTRIC VIEWPORT CONTROLS
    # --------------------------------------------------------------------------
    def _build_layers_tab(self, layout: QVBoxLayout):
        grp_zoom = QGroupBox("Robot-Centric Zoom & Viewport")
        z_lay = QHBoxLayout(grp_zoom)
        btn_zin = QPushButton("🔍 Zoom In (On Robot)")
        btn_zin.clicked.connect(lambda: self.canvas.zoom_on_robot(0.80))
        btn_zout = QPushButton("🔎 Zoom Out")
        btn_zout.clicked.connect(lambda: self.canvas.zoom_on_robot(1.25))
        btn_reset = QPushButton("Reset View (Center Robot)")
        btn_reset.clicked.connect(self.canvas.reset_view)

        z_lay.addWidget(btn_zin)
        z_lay.addWidget(btn_zout)
        z_lay.addWidget(btn_reset)
        layout.addWidget(grp_zoom)

        grp_layers = QGroupBox("Toggleable Visual Overlays")
        l_lay = QVBoxLayout(grp_layers)

        toggles = [
            ("Show 16 Conical 11.25° ToF Beams", "show_rays", True),
            ("Show Measured Distance Numbers (cm)", "show_beam_labels", True),
            ("Show Red C-Space Binary Hazard Mask", "show_hazard_mask", True),
            ("Show Track Inner & Outer Walls (Neon Coral)", "show_walls", True),
            ("Show Racing Line (Cyan) & Corridors (Magenta/Amber)", "show_track_ref", True),
            ("Show Vehicle Chassis & Steered Wheels", "show_chassis", True),
            ("Show Front/Rear Overhang Collision Spheres", "show_overhang_points", True),
            ("Show MPPI Exploration Fan (All Rollouts)", "show_all_rollouts", True),
            ("Show Best Selected Trajectory Line", "show_best_rollout", True),
            ("Show 4 SVG-MPPI Nominal Seed Paths", "show_seed_lines", True),
            ("Show Concentric Distance Range Rings", "show_rings", True),
            ("Show Metric Cartesian Distance Grid", "show_grid", True),
        ]

        for text, attr_name, default_val in toggles:
            chk = QCheckBox(text)
            chk.setChecked(default_val)
            chk.toggled.connect(lambda val, a=attr_name: setattr(self.canvas, a, val) or self.canvas.update())
            l_lay.addWidget(chk)

        # Rollout Cost Gradient Sensitivity Slider
        self.lbl_cost_cap = QLabel(f"Rollout Cost Range Cap: {self.canvas.cost_gradient_cap:.0f} (Cyan=Low -> Red=High)")
        self.lbl_cost_cap.setFont(QFont("Consolas", 8, QFont.Bold))
        self.slider_cost_cap = QSlider(Qt.Horizontal)
        self.slider_cost_cap.setRange(50, 1000)
        self.slider_cost_cap.setValue(int(self.canvas.cost_gradient_cap))
        self.slider_cost_cap.valueChanged.connect(self._on_cost_cap_changed)
        l_lay.addWidget(self.lbl_cost_cap)
        l_lay.addWidget(self.slider_cost_cap)

        # Rollout Opacity Slider
        self.lbl_rollout_alpha = QLabel(f"Rollout Opacity / Alpha: {int(self.canvas.rollout_alpha / 2.55)}%")
        self.lbl_rollout_alpha.setFont(QFont("Consolas", 8))
        self.slider_rollout_alpha = QSlider(Qt.Horizontal)
        self.slider_rollout_alpha.setRange(30, 255)
        self.slider_rollout_alpha.setValue(self.canvas.rollout_alpha)
        self.slider_rollout_alpha.valueChanged.connect(self._on_rollout_alpha_changed)
        l_lay.addWidget(self.lbl_rollout_alpha)
        l_lay.addWidget(self.slider_rollout_alpha)

        # Rollout Line Thickness Slider
        self.lbl_rollout_w = QLabel(f"Rollout Line Width: {self.canvas.rollout_width:.1f} px")
        self.lbl_rollout_w.setFont(QFont("Consolas", 8))
        self.slider_rollout_w = QSlider(Qt.Horizontal)
        self.slider_rollout_w.setRange(10, 35)
        self.slider_rollout_w.setValue(int(self.canvas.rollout_width * 10))
        self.slider_rollout_w.valueChanged.connect(self._on_rollout_width_changed)
        l_lay.addWidget(self.lbl_rollout_w)
        l_lay.addWidget(self.slider_rollout_w)

        # Colormap selector
        c_row = QHBoxLayout()
        c_row.addWidget(QLabel("Costmap Colormap:"))
        cb_cmap = QComboBox()
        cb_cmap.addItems(["Turbo", "Inferno", "Grayscale"])
        cb_cmap.currentTextChanged.connect(lambda name: setattr(self.canvas, "colormap_name", name) or self.canvas.update())
        c_row.addWidget(cb_cmap)
        l_lay.addLayout(c_row)

        layout.addWidget(grp_layers)
        layout.addStretch()

    # --------------------------------------------------------------------------
    # TAB 4: COMPREHENSIVE REAL-TIME PARAMETER TUNING (HOT-RELOAD)
    # --------------------------------------------------------------------------
    def _build_tuning_tab(self, layout: QVBoxLayout):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        container = QWidget()
        c_lay = QVBoxLayout(container)
        c_lay.setContentsMargins(4, 4, 4, 4)
        c_lay.setSpacing(8)

        # Load initial genome dictionary
        self.tuned_params = self._load_initial_genome()
        self.param_controls = {}

        # 1. Horizon & Dynamics
        grp_dyn = QGroupBox("Horizon & Simulation Dynamics")
        g1_lay = QVBoxLayout(grp_dyn)
        self._add_param_spin(g1_lay, "dt", "Lookahead Step Time (dt in s):", 0.020, 0.100, 0.005, 3)
        self._add_param_spin(g1_lay, "H", "Horizon Steps (H):", 10, 50, 1, 0)
        self._add_param_spin(g1_lay, "K", "Rollout Samples (K):", 64, 4096, 64, 0)
        self._add_param_spin(g1_lay, "lambda_temp", "Softmax Temperature (lambda):", 0.05, 3.00, 0.05, 2)
        self._add_param_spin(g1_lay, "ar1_alpha", "AR-1 Temporal Smoothness (alpha):", 0.10, 0.98, 0.02, 2)
        self._add_param_spin(g1_lay, "gamma_discount", "Time Discount Factor (gamma):", 0.80, 1.00, 0.01, 2)
        self._add_param_spin(g1_lay, "mu_friction", "Tire Friction Coeff (mu):", 0.40, 1.50, 0.05, 2)
        c_lay.addWidget(grp_dyn)

        # 2. Cost Function Weights
        grp_cost = QGroupBox("Cost Function Weights")
        g2_lay = QVBoxLayout(grp_cost)
        self._add_param_spin(g2_lay, "w_cte", "Cross-Track Error (w_cte):", 0.0, 60.0, 0.5, 1)
        self._add_param_spin(g2_lay, "w_heading", "Heading Alignment (w_heading):", 0.0, 30.0, 0.5, 1)
        self._add_param_spin(g2_lay, "w_speed", "Speed Tracking (w_speed):", 0.0, 40.0, 0.5, 1)
        self._add_param_spin(g2_lay, "w_hazard", "Lethal Obstacle Cost (w_hazard):", 1000.0, 1000000.0, 5000.0, 0)
        self._add_param_spin(g2_lay, "w_prob_costmap", "Costmap Occupancy (w_prob):", 0.0, 250.0, 1.0, 1)
        self._add_param_spin(g2_lay, "w_unknown", "Unknown Space (w_unknown):", 0.0, 100.0, 1.0, 1)
        self._add_param_spin(g2_lay, "w_steer_rate", "Steer Rate Smoothness (w_steer):", 0.0, 5.0, 0.05, 2)
        self._add_param_spin(g2_lay, "w_accel_rate", "Acceleration Smoothness (w_accel):", 0.0, 2.0, 0.02, 2)
        self._add_param_spin(g2_lay, "w_terminal", "Terminal Goal Cost (w_terminal):", 0.0, 60.0, 0.5, 1)
        c_lay.addWidget(grp_cost)

        # 3. Safety, Offsets, Noise & Recovery
        grp_safety = QGroupBox("Safety, Offsets, Noise & Recovery")
        g3_lay = QVBoxLayout(grp_safety)
        self._add_param_spin(g3_lay, "r_inflation_m", "Obstacle Inflation Radius (m):", 0.060, 0.160, 0.002, 3)
        self._add_param_spin(g3_lay, "pure_pursuit_lookahead", "Pure Pursuit Lookahead (m):", 0.15, 1.50, 0.05, 2)
        self._add_param_spin(g3_lay, "ratio_center", "Center Seed Allocation Ratio:", 0.05, 0.80, 0.05, 2)
        self._add_param_spin(g3_lay, "ratio_straight", "Straight Seed Allocation Ratio:", 0.05, 0.50, 0.05, 2)
        self._add_param_spin(g3_lay, "ratio_left", "Left Seed Allocation Ratio:", 0.05, 0.50, 0.05, 2)
        self._add_param_spin(g3_lay, "sigma_steer", "Steering Perturbation Noise (rad):", 0.01, 0.40, 0.01, 2)
        self._add_param_spin(g3_lay, "sigma_speed", "Speed Perturbation Noise (m/s):", 0.05, 1.50, 0.05, 2)
        self._add_param_spin(g3_lay, "w_reverse", "Reverse Cost Penalty (w_reverse):", 50.0, 2000.0, 50.0, 0)
        self._add_param_spin(g3_lay, "reverse_speed", "Reverse Speed Magnitude (m/s):", 0.40, 2.50, 0.10, 2)
        self._add_param_spin(g3_lay, "t_rev_steps", "Reverse Backup Steps:", 1, 10, 1, 0)
        c_lay.addWidget(grp_safety)

        # Action Buttons Row
        btn_row = QVBoxLayout()
        self.btn_apply_params = QPushButton("⚡ Send & Apply Live to Webots")
        self.btn_apply_params.setStyleSheet("font-weight: bold; background-color: #00e676; color: black; padding: 8px;")
        self.btn_apply_params.clicked.connect(self._send_all_params_live)
        btn_row.addWidget(self.btn_apply_params)

        btn_sub_row = QHBoxLayout()
        btn_reload = QPushButton("📂 Reload from JSON")
        btn_reload.clicked.connect(self._reload_params_from_file)
        btn_reset_sim = QPushButton("↺ Reset Sim & Apply")
        btn_reset_sim.setStyleSheet("background-color: #d32f2f; color: white; font-weight: bold;")
        btn_reset_sim.clicked.connect(self._reset_sim_and_apply)
        btn_sub_row.addWidget(btn_reload)
        btn_sub_row.addWidget(btn_reset_sim)
        btn_row.addLayout(btn_sub_row)

        c_lay.addLayout(btn_row)
        scroll.setWidget(container)
        layout.addWidget(scroll)

    def _add_param_spin(self, layout: QVBoxLayout, key: str, label_text: str, v_min: float, v_max: float, step: float, decimals: int):
        val = float(self.tuned_params.get(key, (v_min + v_max) / 2.0))
        row = QHBoxLayout()
        lbl = QLabel(label_text)
        lbl.setFont(QFont("Consolas", 8))

        spin = QDoubleSpinBox()
        spin.setDecimals(decimals)
        spin.setRange(v_min, v_max)
        spin.setSingleStep(step)
        spin.setValue(val)
        spin.setFixedWidth(105)
        spin.valueChanged.connect(lambda v, k=key: self._on_param_spin_changed(k, v))

        self.param_controls[key] = spin
        row.addWidget(lbl, stretch=3)
        row.addWidget(spin, stretch=1)
        layout.addLayout(row)

    def _load_initial_genome(self) -> dict:
        win_json = os.path.join(WORKSPACE_ROOT, "winning_mppi_genome.json")
        if os.path.exists(win_json):
            try:
                with open(win_json, "r") as f:
                    data = json.load(f)
                if "genes" in data:
                    return data["genes"]
            except Exception:
                pass
        return {
            "dt": 0.055, "H": 28, "K": 512, "lambda_temp": 0.35, "ar1_alpha": 0.75,
            "gamma_discount": 0.95, "mu_friction": 0.70, "w_cte": 12.0, "w_heading": 4.5,
            "w_speed": 3.5, "w_hazard": 350000.0, "w_prob_costmap": 50.0, "w_unknown": 20.0,
            "w_steer_rate": 0.40, "w_accel_rate": 0.08, "w_terminal": 8.0, "r_inflation_m": 0.088,
            "pure_pursuit_lookahead": 0.50, "ratio_center": 0.35, "ratio_straight": 0.20,
            "ratio_left": 0.30, "sigma_steer": 0.10, "sigma_speed": 0.35
        }

    def _on_param_spin_changed(self, key: str, val: float):
        self.tuned_params[key] = val
        # Send individual live update
        self._send_cmd({"cmd": "set_params", "params": {key: val}})

    def _send_all_params_live(self):
        self._send_cmd({"cmd": "set_params", "params": self.tuned_params})

    def _reload_params_from_file(self):
        self.tuned_params = self._load_initial_genome()
        for k, spin in self.param_controls.items():
            if k in self.tuned_params:
                spin.blockSignals(True)
                spin.setValue(float(self.tuned_params[k]))
                spin.blockSignals(False)
        self._send_all_params_live()

    def _reset_sim_and_apply(self):
        self._send_all_params_live()
        self._send_reset_command()

    # --------------------------------------------------------------------------
    # UDP LISTENER & TELEMETRY DISPATCHER
    # --------------------------------------------------------------------------
    def _udp_listener(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("127.0.0.1", 9876))
        except Exception:
            try:
                sock.bind(("0.0.0.0", 9876))
            except Exception as e:
                print(f"[Analytic GUI] Error binding telemetry port 9876: {e}", flush=True)
                return

        sock.settimeout(0.2)
        while self.udp_running:
            try:
                data, _ = sock.recvfrom(65536)
                try:
                    decompressed = zlib.decompress(data).decode("utf-8")
                    pkt = json.loads(decompressed)
                except Exception:
                    pkt = json.loads(data.decode("utf-8"))

                with self._packet_lock:
                    self._latest_packet = pkt
            except socket.timeout:
                continue
            except Exception:
                pass

    def _load_cartographer_map(self, yaml_path: str):
        if not os.path.exists(yaml_path):
            return None
        try:
            cfg = {}
            with open(yaml_path, "r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if ":" in line and not line.startswith("#"):
                        k, v = line.split(":", 1)
                        k = k.strip()
                        v = v.strip()
                        if v.startswith("[") and v.endswith("]"):
                            v = [float(x.strip()) for x in v[1:-1].split(",") if x.strip()]
                        elif v.replace(".", "", 1).isdigit() or (v.startswith("-") and v[1:].replace(".", "", 1).isdigit()):
                            v = float(v)
                        else:
                            v = v.strip("\"'")
                        cfg[k] = v

            pgm_name = cfg.get("image", "")
            pgm_path = os.path.join(os.path.dirname(yaml_path), str(pgm_name))
            res = float(cfg.get("resolution", 0.02))
            origin_val = cfg.get("origin", [0.0, 0.0, 0.0])
            origin = tuple(origin_val[:2]) if isinstance(origin_val, (list, tuple)) else (0.0, 0.0)
            negate = int(cfg.get("negate", 0))
            occ_thresh = float(cfg.get("occupied_thresh", 0.65))
            free_thresh = float(cfg.get("free_thresh", 0.25))

            from PIL import Image
            img = Image.open(pgm_path)
            arr = np.array(img, dtype=np.float32)
            if negate:
                arr = 255.0 - arr
            occ_grid = np.full(arr.shape, 0.5, dtype=np.float32)
            occ_grid[arr <= (free_thresh * 255.0)] = 0.0
            occ_grid[arr >= (occ_thresh * 255.0)] = 1.0
            return occ_grid, origin, res
        except Exception as e:
            print(f"[Analytic Debugger] Failed loading Cartographer SLAM Map: {e}", flush=True)
            return None

    def _load_track_walls(self) -> List[List[List[float]]]:
        wall_file = os.path.join(CURR_DIR, "track_walls.json")
        if os.path.exists(wall_file):
            try:
                with open(wall_file, "r") as f:
                    return json.load(f)
            except Exception:
                pass
        return []

    def _load_csv_points(self, filename: str) -> Optional[np.ndarray]:
        search_dirs = [
            MPPI_CONTROLLER_DIR,
            os.path.join(WORKSPACE_ROOT, "racing_results", "tum_qp_obstacle_aware_25cm_offsets"),
            WORKSPACE_ROOT,
            FOLKRACE_DIR,
            CURR_DIR,
        ]
        for d in search_dirs:
            p = os.path.join(d, filename)
            if os.path.exists(p):
                try:
                    data = np.genfromtxt(p, delimiter=",", skip_header=1)
                    if data.ndim == 2 and data.shape[1] >= 2:
                        return data[:, 0:2]
                except Exception:
                    pass
        return None

    def _on_cost_cap_changed(self, val: int):
        self.canvas.cost_gradient_cap = float(val)
        self.lbl_cost_cap.setText(f"Rollout Cost Range Cap: {val} (Cyan=Low -> Red=High)")
        self.canvas.update()

    def _on_rollout_alpha_changed(self, val: int):
        self.canvas.rollout_alpha = val
        self.lbl_rollout_alpha.setText(f"Rollout Opacity / Alpha: {int(val / 2.55)}%")
        self.canvas.update()

    def _on_rollout_width_changed(self, val: int):
        self.canvas.rollout_width = val / 10.0
        self.lbl_rollout_w.setText(f"Rollout Line Width: {self.canvas.rollout_width:.1f} px")
        self.canvas.update()

    def _on_timer(self):
        pkt = None
        with self._packet_lock:
            if self._latest_packet:
                pkt = self._latest_packet
                self._latest_packet = None

        if pkt is None:
            return

        pos = pkt.get("pos", [0.0, 0.0])
        rx = float(pkt.get("rx", pos[0] if isinstance(pos, (list, tuple)) and len(pos) >= 1 else 0.0))
        ry = float(pkt.get("ry", pos[1] if isinstance(pos, (list, tuple)) and len(pos) >= 2 else 0.0))
        ryaw = float(pkt.get("ryaw", pkt.get("yaw", 0.0)))
        v_mps = float(pkt.get("v_mps", pkt.get("v", 0.0)))
        steer_rad = float(pkt.get("active_steer_rad", pkt.get("steer", 0.0)))
        cmd_steer_rad = float(pkt.get("cmd_steer_rad", pkt.get("steer", 0.0)))
        cmd_speed_mps = float(pkt.get("cmd_speed_mps", pkt.get("speed", 0.0)))
        tof_readings = pkt.get("tof_readings", pkt.get("tof", {}))
        diag = pkt.get("diag", pkt)

        # Process costmap frame (Static SLAM map + Realtime ToF)
        if self.carto_grid is not None:
            pose_stamped = StampedPose(time.time(), rx, ry, ryaw, v_mps)
            self.costmap_pipe.update_slam_map(self.carto_grid, self.carto_origin, self.carto_res, pose_stamped)

        self.costmap_pipe.update_pose(time.time(), rx, ry, ryaw, v_mps)
        prob_map, hazard_mask, _ = self.costmap_pipe.process_tof_frame(tof_readings, time.time())

        # Update Canvas
        self.canvas.set_data(
            prob_map=prob_map,
            hazard_mask=hazard_mask,
            tof_readings=tof_readings,
            active_v=v_mps,
            active_steer=steer_rad,
            cmd_v=cmd_speed_mps,
            cmd_steer=cmd_steer_rad
        )

        # Transform global track walls to local robot coordinate frame (+X right, +Y forward)
        cos_y = math.cos(ryaw)
        sin_y = math.sin(ryaw)

        track_walls_local = []
        if self.global_walls:
            for wall_pts in self.global_walls:
                local_wall = []
                for pt in wall_pts:
                    dx = pt[0] - rx
                    dy = pt[1] - ry
                    fwd = dx * cos_y + dy * sin_y
                    lat = dx * sin_y - dy * cos_y
                    local_wall.append([lat, fwd])
                track_walls_local.append(local_wall)

        # Transform full continuous global racing lines to local robot coordinate frame
        track_ref_center = []
        if self.global_traj_center is not None:
            for pt in self.global_traj_center:
                dx = pt[0] - rx
                dy = pt[1] - ry
                fwd = dx * cos_y + dy * sin_y
                lat = dx * sin_y - dy * cos_y
                track_ref_center.append([lat, fwd])

        track_ref_left = []
        if self.global_traj_left is not None:
            for pt in self.global_traj_left:
                dx = pt[0] - rx
                dy = pt[1] - ry
                fwd = dx * cos_y + dy * sin_y
                lat = dx * sin_y - dy * cos_y
                track_ref_left.append([lat, fwd])

        track_ref_right = []
        if self.global_traj_right is not None:
            for pt in self.global_traj_right:
                dx = pt[0] - rx
                dy = pt[1] - ry
                fwd = dx * cos_y + dy * sin_y
                lat = dx * sin_y - dy * cos_y
                track_ref_right.append([lat, fwd])

        self.canvas.set_trajectories(
            best=diag.get("best_traj_local", diag.get("best_traj")),
            candidates=diag.get("cand_trajs_local", diag.get("cand_trajs")),
            candidate_costs=diag.get("cand_costs", None),
            seed_center=diag.get("seed_center_local", diag.get("seed_center")),
            seed_straight=diag.get("seed_straight_local", diag.get("seed_straight")),
            seed_left=diag.get("seed_left_local", diag.get("seed_left")),
            seed_right=diag.get("seed_right_local", diag.get("seed_right")),
            track_ref_center=track_ref_center,
            track_ref_left=track_ref_left,
            track_ref_right=track_ref_right,
            track_walls=track_walls_local,
        )

        # Update Decision Weights & Gauges
        seed_w = diag.get("seed_weights", {})
        self.bar_center.setValue(int(round(float(seed_w.get("center", 0.0)) * 100)))
        self.bar_straight.setValue(int(round(float(seed_w.get("straight", 0.0)) * 100)))
        self.bar_left.setValue(int(round(float(seed_w.get("left", 0.0)) * 100)))
        self.bar_right.setValue(int(round(float(seed_w.get("right", 0.0)) * 100)))
        self.bar_rev.setValue(int(round(float(seed_w.get("reverse", 0.0)) * 100)))

        # Update Metric Labels
        exec_ms = float(diag.get("exec_time_ms", 0.0))
        self.lbl_exec_time.setText(f"Execution Time: {exec_ms:.2f} ms (SIMD JIT)")
        col_rate = float(diag.get("collision_rate", 0.0)) * 100.0
        self.lbl_collision_rate.setText(f"Pruned / Collision Rollouts: {col_rate:.1f}%")
        self.lbl_best_cost.setText(f"Best Trajectory Cost: {float(diag.get('best_cost', 0.0)):.1f}")
        self.lbl_target_v.setText(f"Nominal Reference Speed: {float(diag.get('v_ref', cmd_speed_mps)):.2f} m/s")

        self.lbl_speed_steer.setText(f"Speed: {v_mps:.2f} m/s | Steer: {math.degrees(steer_rad):+.1f}°")
        cte_cm = float(diag.get("cte_m", diag.get("cte", 0.0))) * 100.0
        self.lbl_cte_heading.setText(f"Cross-Track Error: {cte_cm:+.1f} cm")

        # Update 16-Channel Bars
        for s_name, s_bar in self.tof_bars.items():
            cm_val = int(round(float(tof_readings.get(s_name, 1.80)) * 100.0))
            s_bar.setValue(min(180, cm_val))
            self.tof_labels[s_name].setText(f"{cm_val} cm")
            if cm_val < 15:
                s_bar.setStyleSheet("QProgressBar::chunk { background-color: #ff1744; }")
            elif cm_val < 35:
                s_bar.setStyleSheet("QProgressBar::chunk { background-color: #ff9100; }")
            else:
                s_bar.setStyleSheet("QProgressBar::chunk { background-color: #00e676; }")

    # --------------------------------------------------------------------------
    # COMMANDS & EVENT HANDLERS
    # --------------------------------------------------------------------------
    def _send_cmd(self, cmd_dict: dict):
        try:
            raw = json.dumps(cmd_dict).encode("utf-8")
            self.udp_sock.sendto(raw, ("127.0.0.1", 9877))
        except Exception:
            pass

    def _toggle_play_pause(self):
        self.sim_paused = not self.sim_paused
        if self.sim_paused:
            self.btn_play_pause.setText("▶ Resume")
            self.btn_play_pause.setStyleSheet("font-weight: bold; background-color: #f57f17; color: white; padding: 6px;")
            self._send_cmd({"cmd": "pause"})
        else:
            self.btn_play_pause.setText("⏸ Pause")
            self.btn_play_pause.setStyleSheet("font-weight: bold; background-color: #2e7d32; color: white; padding: 6px;")
            self._send_cmd({"cmd": "play"})

    def _send_step_command(self):
        self._send_cmd({"cmd": "step", "step_ms": 10})

    def _send_reset_command(self):
        self._send_cmd({"cmd": "reset"})

    def _on_speed_changed(self, idx: int):
        speeds = [0.05, 0.10, 0.25, 0.50, 1.00, 2.00, 100.0]
        val = speeds[idx] if idx < len(speeds) else 1.00
        self._send_cmd({"cmd": "set_speed", "speed_multiplier": val})

    def _on_inflation_slider(self, val: int):
        self.r_inflation_m = val / 1000.0
        self.lbl_inflation.setText(f"Obstacle Inflation Radius: {self.r_inflation_m*100:.1f} cm")
        self.costmap_pipe.set_inflation_radius(self.r_inflation_m)
        self._send_cmd({"cmd": "set_inflation", "r_inflation_m": self.r_inflation_m})

    def closeEvent(self, event):
        self.udp_running = False
        super().closeEvent(event)


def launch_analytic_gui():
    if not PYQT_AVAILABLE:
        print("[Analytic GUI] PyQt5 is required. Install via 'pip install PyQt5'.", flush=True)
        return
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    gui = AnalyticDebuggerGUI()
    gui.show()
    app.exec_()


if __name__ == "__main__":
    launch_analytic_gui()
