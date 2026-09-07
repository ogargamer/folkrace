"""
================================================================================
  LIVE REAL-TIME 16-CHANNEL TOF COSTMAP & MPPI WEBOTS GUI
  - 100% Dedicated Live Webots Telemetry & Cartographer Baseline Stream (Port 9876)
  - Zoom Controls: Mouse Wheel Zoom + Zoom Buttons + Expanded Default View
  - Google Cartographer 15 Hz Baseline Fusion & 16-Channel 50 Hz ToF Cone Perception
  - Live Obstacle C-Space Inflation (8 to 13 cm) & Binary Hazard Layer Rendering
  - Full Simulation Controls: Play, Pause, Single-Step (10ms), Speed Multiplier (0.05x - 5x)
  - Bidirectional UDP Integration with Webots Supervisor (Ports 9876 & 9877)
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
from typing import Dict, List, Optional, Tuple
import numpy as np

# Ensure directory is in path
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
MY_PROJECT_DIR = os.path.dirname(CONTROLLERS_DIR)
WORKSPACE_ROOT = os.path.dirname(MY_PROJECT_DIR)

for p in [CURR_DIR, CONTROLLERS_DIR, MY_PROJECT_DIR, WORKSPACE_ROOT]:
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
    from PyQt5.QtGui import QColor, QPainter, QPen, QBrush, QFont, QImage, QPixmap
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QLabel, QSlider, QComboBox, QPushButton, QCheckBox, QGroupBox,
        QDoubleSpinBox, QTabWidget, QSplitter, QStatusBar, QFrame
    )
    PYQT_AVAILABLE = True
except ImportError:
    PYQT_AVAILABLE = False


# ==============================================================================
# 1. 2D COSTMAP CANVAS (Zoomable, Multi-Layer Visualizer)
# ==============================================================================
class CostmapCanvas(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(540, 600)
        self.setMouseTracking(True)

        # Costmap grid data
        self.prob_map: Optional[np.ndarray] = None
        self.hazard_mask: Optional[np.ndarray] = None
        self.tof_readings: Dict[str, float] = {s.name: 1.80 for s in SENSOR_RIG}

        # Costmap local array metric bounds (3.0m wide x 2.9m reach)
        self.map_x_min = -1.5
        self.map_x_max = 1.5
        self.map_y_min = -0.5
        self.map_y_max = 2.4

        # Display viewport bounds (Zoomed out to show vehicle, surround walls & 1.8m cones)
        self.x_min = -2.0
        self.x_max = 2.0
        self.y_min = -0.8
        self.y_max = 2.8

        # Display preferences
        self.colormap_name = "Turbo"
        self.show_rays = True
        self.show_hazard_mask = True
        self.show_chassis = True
        self.show_grid = True
        self.show_rollouts = True
        self.show_carto_base = True
        self.show_track_ref = True

        # Live trajectory rollouts (from MPPI / UDP)
        self.best_rollout: Optional[List[List[float]]] = None
        self.seed_center: Optional[List[List[float]]] = None
        self.seed_straight: Optional[List[List[float]]] = None
        self.seed_left: Optional[List[List[float]]] = None
        self.seed_right: Optional[List[List[float]]] = None
        self.candidate_rollouts: Optional[List[List[List[float]]]] = None

        # Static Global Track Racing Lines (projected into local frame)
        self.track_ref_center: Optional[List[List[float]]] = None
        self.track_ref_left: Optional[List[List[float]]] = None
        self.track_ref_right: Optional[List[List[float]]] = None

        # Hover inspector
        self.hover_m: Optional[Tuple[float, float]] = None
        self.hover_prob: Optional[float] = None
        self.is_connected = False

    def set_data(self, prob_map: np.ndarray, hazard_mask: np.ndarray, tof_readings: Dict[str, float]):
        self.prob_map = prob_map
        self.hazard_mask = hazard_mask
        self.tof_readings = tof_readings
        self.update()

    def set_rollouts(
        self,
        best: Optional[List[List[float]]],
        candidates: Optional[List[List[List[float]]]],
        seed_center: Optional[List[List[float]]] = None,
        seed_straight: Optional[List[List[float]]] = None,
        seed_left: Optional[List[List[float]]] = None,
        seed_right: Optional[List[List[float]]] = None,
        track_ref_center: Optional[List[List[float]]] = None,
        track_ref_left: Optional[List[List[float]]] = None,
        track_ref_right: Optional[List[List[float]]] = None,
    ):
        self.best_rollout = best
        self.candidate_rollouts = candidates
        self.seed_center = seed_center
        self.seed_straight = seed_straight
        self.seed_left = seed_left
        self.seed_right = seed_right
        self.track_ref_center = track_ref_center
        self.track_ref_left = track_ref_left
        self.track_ref_right = track_ref_right
        self.update()

    def reset_view(self):
        """Resets zoom viewport to default wide view."""
        self.x_min = -2.0
        self.x_max = 2.0
        self.y_min = -0.8
        self.y_max = 2.8
        self.update()

    def zoom(self, factor: float):
        """Zooms centered on current view."""
        cx = (self.x_min + self.x_max) / 2.0
        cy = (self.y_min + self.y_max) / 2.0
        dx = (self.x_max - self.x_min) * factor / 2.0
        dy = (self.y_max - self.y_min) * factor / 2.0
        self.x_min, self.x_max = cx - dx, cx + dx
        self.y_min, self.y_max = cy - dy, cy + dy
        self.update()

    def wheelEvent(self, event):
        """Interactive mouse wheel zoom."""
        delta = event.angleDelta().y()
        if delta > 0:
            self.zoom(0.88)  # Zoom In
        elif delta < 0:
            self.zoom(1.14)  # Zoom Out

    def world_to_screen(self, xm: float, ym: float) -> QPointF:
        """Maps vehicle local coordinates (X lateral, Y forward) to screen pixels."""
        w = self.width()
        h = self.height()
        px = (xm - self.x_min) / (self.x_max - self.x_min) * w
        py = (1.0 - (ym - self.y_min) / (self.y_max - self.y_min)) * h
        return QPointF(px, py)

    def screen_to_world(self, px: float, py: float) -> Tuple[float, float]:
        w = self.width()
        h = self.height()
        xm = self.x_min + (px / w) * (self.x_max - self.x_min)
        ym = self.y_min + (1.0 - py / h) * (self.y_max - self.y_min)
        return float(xm), float(ym)

    def mouseMoveEvent(self, event):
        xm, ym = self.screen_to_world(event.x(), event.y())
        self.hover_m = (xm, ym)

        if self.prob_map is not None:
            H, W = self.prob_map.shape
            gx = int((xm - self.map_x_min) / (self.map_x_max - self.map_x_min) * W)
            gy = int((ym - self.map_y_min) / (self.map_y_max - self.map_y_min) * H)
            if 0 <= gx < W and 0 <= gy < H:
                self.hover_prob = float(self.prob_map[gy, gx])
            else:
                self.hover_prob = None
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor(14, 18, 24))

        w = self.width()
        h = self.height()

        # 1. Draw 2D Costmap Image Layer (Aligned to [map_x_min, map_x_max] x [map_y_min, map_y_max])
        if self.prob_map is not None:
            H, W = self.prob_map.shape
            img_data = np.zeros((H, W, 4), dtype=np.uint8)

            probs = self.prob_map.copy()
            val = (probs * 255.0).astype(np.uint8)

            if self.colormap_name == "Turbo":
                r = np.clip(np.sin(val / 255.0 * np.pi - np.pi / 2.0) * 127 + 128, 0, 255).astype(np.uint8)
                g = np.clip(np.sin(val / 255.0 * np.pi) * 255, 0, 255).astype(np.uint8)
                b = np.clip(np.cos(val / 255.0 * np.pi / 2.0) * 255, 0, 255).astype(np.uint8)
                img_data[..., 0] = r
                img_data[..., 1] = g
                img_data[..., 2] = b
                img_data[..., 3] = (probs * 210 + 40).astype(np.uint8)
            elif self.colormap_name == "Inferno":
                img_data[..., 0] = np.clip(val * 1.4, 0, 255).astype(np.uint8)
                img_data[..., 1] = np.clip(val * 0.9, 0, 255).astype(np.uint8)
                img_data[..., 2] = np.clip(val * 0.3, 0, 255).astype(np.uint8)
                img_data[..., 3] = (probs * 220 + 35).astype(np.uint8)
            else:
                img_data[..., 0] = val
                img_data[..., 1] = val
                img_data[..., 2] = val
                img_data[..., 3] = 220

            # C-Space Dilated Hazard overlay (Bright Red boundary)
            if self.show_hazard_mask and self.hazard_mask is not None:
                img_data[self.hazard_mask, 0] = 255
                img_data[self.hazard_mask, 1] = 30
                img_data[self.hazard_mask, 2] = 50
                img_data[self.hazard_mask, 3] = 240

            img_data = np.flipud(img_data)
            qimg = QImage(img_data.tobytes(), W, H, W * 4, QImage.Format_RGBA8888)

            # Map the QImage to exact metric world bounds on canvas
            p_tl = self.world_to_screen(self.map_x_min, self.map_y_max)
            p_br = self.world_to_screen(self.map_x_max, self.map_y_min)
            dest_rect = QRectF(p_tl.x(), p_tl.y(), p_br.x() - p_tl.x(), p_br.y() - p_tl.y())
            painter.drawImage(dest_rect, qimg)

            # Costmap border
            painter.setPen(QPen(QColor(80, 120, 160, 90), 1, Qt.DotLine))
            painter.setBrush(Qt.NoBrush)
            painter.drawRect(dest_rect)

        # 2. Draw Grid Reference Distance Lines & Concentric Range Circles
        if self.show_grid:
            grid_pen = QPen(QColor(50, 70, 90, 110), 1, Qt.DashLine)
            painter.setPen(grid_pen)
            painter.setFont(QFont("Consolas", 8))
            for ym in [-0.4, 0.0, 0.5, 1.0, 1.5, 2.0, 2.4]:
                p1 = self.world_to_screen(self.x_min, ym)
                p2 = self.world_to_screen(self.x_max, ym)
                painter.drawLine(p1, p2)
                painter.setPen(QPen(QColor(100, 140, 180, 180)))
                painter.drawText(QPointF(p1.x() + 6, p1.y() - 3), f"Y={ym:+.1f}m")
                painter.setPen(grid_pen)

            for xm in [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5]:
                p1 = self.world_to_screen(xm, self.y_min)
                p2 = self.world_to_screen(xm, self.y_max)
                painter.drawLine(p1, p2)

            # Range rings (0.5m, 1.0m, 1.5m, 1.8m)
            for ring_r in [0.5, 1.0, 1.5, 1.8]:
                r_tl = self.world_to_screen(-ring_r, ring_r)
                r_br = self.world_to_screen(ring_r, -ring_r)
                painter.setPen(QPen(QColor(60, 90, 120, 60), 1, Qt.DotLine))
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(QRectF(r_tl.x(), r_tl.y(), r_br.x() - r_tl.x(), r_br.y() - r_tl.y()))

        # 3. Draw Continuous Global Track Racing Lines (Thin Reference Lines)
        if self.show_track_ref:
            # A. Global Left Corridor Line (Thin Magenta)
            if self.track_ref_left:
                ref_left_pen = QPen(QColor(224, 64, 251, 130), 1.0, Qt.SolidLine)
                painter.setPen(ref_left_pen)
                for i in range(len(self.track_ref_left) - 1):
                    p1 = self.world_to_screen(self.track_ref_left[i][0], self.track_ref_left[i][1])
                    p2 = self.world_to_screen(self.track_ref_left[i + 1][0], self.track_ref_left[i + 1][1])
                    painter.drawLine(p1, p2)

            # B. Global Right Corridor Line (Thin Amber)
            if self.track_ref_right:
                ref_right_pen = QPen(QColor(255, 215, 64, 130), 1.0, Qt.SolidLine)
                painter.setPen(ref_right_pen)
                for i in range(len(self.track_ref_right) - 1):
                    p1 = self.world_to_screen(self.track_ref_right[i][0], self.track_ref_right[i][1])
                    p2 = self.world_to_screen(self.track_ref_right[i + 1][0], self.track_ref_right[i + 1][1])
                    painter.drawLine(p1, p2)

            # C. Global Optimal Racing Centerline (Thin Crisp Cyan)
            if self.track_ref_center:
                ref_center_pen = QPen(QColor(0, 229, 255, 180), 1.4, Qt.SolidLine)
                painter.setPen(ref_center_pen)
                for i in range(len(self.track_ref_center) - 1):
                    p1 = self.world_to_screen(self.track_ref_center[i][0], self.track_ref_center[i][1])
                    p2 = self.world_to_screen(self.track_ref_center[i + 1][0], self.track_ref_center[i + 1][1])
                    painter.drawLine(p1, p2)

        # 4. Draw Candidate Rollouts (MPPI Exploration Fan)
        if self.show_rollouts and self.candidate_rollouts:
            cand_pen = QPen(QColor(60, 160, 240, 45), 1)
            painter.setPen(cand_pen)
            for traj in self.candidate_rollouts:
                for i in range(len(traj) - 1):
                    p1 = self.world_to_screen(traj[i][0], traj[i][1])
                    p2 = self.world_to_screen(traj[i + 1][0], traj[i + 1][1])
                    painter.drawLine(p1, p2)

        # 4. Draw 4 Distinct SVG-MPPI Nominal Candidate Paths
        if self.show_rollouts:
            # A. Left Nominal Path (+25cm lane) - Magenta Dashed
            if self.seed_left:
                left_pen = QPen(QColor(224, 64, 251, 200), 2.0, Qt.DashLine)
                painter.setPen(left_pen)
                for i in range(len(self.seed_left) - 1):
                    p1 = self.world_to_screen(self.seed_left[i][0], self.seed_left[i][1])
                    p2 = self.world_to_screen(self.seed_left[i + 1][0], self.seed_left[i + 1][1])
                    painter.drawLine(p1, p2)

            # B. Right Nominal Path (-25cm lane) - Amber/Yellow Dashed
            if self.seed_right:
                right_pen = QPen(QColor(255, 215, 64, 200), 2.0, Qt.DashLine)
                painter.setPen(right_pen)
                for i in range(len(self.seed_right) - 1):
                    p1 = self.world_to_screen(self.seed_right[i][0], self.seed_right[i][1])
                    p2 = self.world_to_screen(self.seed_right[i + 1][0], self.seed_right[i + 1][1])
                    painter.drawLine(p1, p2)

            # C. Center Nominal Path (Racing Line) - Bright Cyan Dashed
            if self.seed_center:
                center_pen = QPen(QColor(0, 229, 255, 220), 2.2, Qt.DashLine)
                painter.setPen(center_pen)
                for i in range(len(self.seed_center) - 1):
                    p1 = self.world_to_screen(self.seed_center[i][0], self.seed_center[i][1])
                    p2 = self.world_to_screen(self.seed_center[i + 1][0], self.seed_center[i + 1][1])
                    painter.drawLine(p1, p2)

            # D. Straight Nominal Path (Gap Shooter) - Crisp Bright White Dashed
            if self.seed_straight:
                straight_pen = QPen(QColor(255, 255, 255, 220), 2.0, Qt.DashLine)
                painter.setPen(straight_pen)
                for i in range(len(self.seed_straight) - 1):
                    p1 = self.world_to_screen(self.seed_straight[i][0], self.seed_straight[i][1])
                    p2 = self.world_to_screen(self.seed_straight[i + 1][0], self.seed_straight[i + 1][1])
                    painter.drawLine(p1, p2)

        # 5. Draw Optimal MPPI Selected Trajectory (Vibrant Glowing Green)
        if self.show_rollouts and self.best_rollout:
            # Outer glow
            glow_pen = QPen(QColor(0, 230, 118, 90), 6.0)
            painter.setPen(glow_pen)
            for i in range(len(self.best_rollout) - 1):
                p1 = self.world_to_screen(self.best_rollout[i][0], self.best_rollout[i][1])
                p2 = self.world_to_screen(self.best_rollout[i + 1][0], self.best_rollout[i + 1][1])
                painter.drawLine(p1, p2)

            # Core path
            best_pen = QPen(QColor(0, 255, 136, 255), 3.0)
            painter.setPen(best_pen)
            for i in range(len(self.best_rollout) - 1):
                p1 = self.world_to_screen(self.best_rollout[i][0], self.best_rollout[i][1])
                p2 = self.world_to_screen(self.best_rollout[i + 1][0], self.best_rollout[i + 1][1])
                painter.drawLine(p1, p2)

            # Waypoints along optimal path
            painter.setBrush(QBrush(QColor(255, 255, 255, 220)))
            painter.setPen(Qt.NoPen)
            for pt in self.best_rollout[::3]:
                pw = self.world_to_screen(pt[0], pt[1])
                painter.drawEllipse(pw, 2.5, 2.5)

        # 5. Draw 16 Conical 11.25° ToF Beams (True Optical FOV up to 1.80m)
        if self.show_rays:
            for s in SENSOR_RIG:
                dist = self.tof_readings.get(s.name, s.max_range)
                origin = self.world_to_screen(s.x0, s.y0)
                half_fov = s.fov_rad / 2.0  # 5.625 degrees

                # Generate arc points across the 11.25 deg cone
                n_arc = 6
                angles = np.linspace(s.heading_rad - half_fov, s.heading_rad + half_fov, n_arc)
                arc_points = []
                for a in angles:
                    ax = s.x0 + dist * math.cos(a)
                    ay = s.y0 + dist * math.sin(a)
                    arc_points.append(self.world_to_screen(ax, ay))

                poly = QtGui.QPolygonF([origin] + arc_points)

                if dist < (s.max_range - 0.05):
                    # Obstacle detected in zone:
                    # 1) Clear free-space cone interior
                    painter.setPen(Qt.NoPen)
                    painter.setBrush(QBrush(QColor(0, 200, 100, 30)))
                    painter.drawPolygon(poly)

                    # 2) Cone boundary ray lines
                    cone_pen = QPen(QColor(255, 150, 30, 190), 1.2, Qt.SolidLine)
                    painter.setPen(cone_pen)
                    painter.drawLine(origin, arc_points[0])
                    painter.drawLine(origin, arc_points[-1])

                    # 3) Center optical axis (faint)
                    center_pt = self.world_to_screen(
                        s.x0 + dist * math.cos(s.heading_rad),
                        s.y0 + dist * math.sin(s.heading_rad)
                    )
                    painter.setPen(QPen(QColor(255, 180, 50, 110), 1, Qt.DotLine))
                    painter.drawLine(origin, center_pt)

                    # 4) Thick bright obstacle contact arc (spanning the full 11.25 deg zone)
                    arc_pen = QPen(QColor(255, 40, 40, 240), 3.5, Qt.SolidLine, Qt.RoundCap)
                    painter.setPen(arc_pen)
                    for i in range(len(arc_points) - 1):
                        painter.drawLine(arc_points[i], arc_points[i + 1])

                    # Endpoint dots on arc
                    painter.setPen(Qt.NoPen)
                    painter.setBrush(QBrush(QColor(255, 60, 60, 255)))
                    painter.drawEllipse(arc_points[0], 2.5, 2.5)
                    painter.drawEllipse(arc_points[-1], 2.5, 2.5)
                    painter.drawEllipse(center_pt, 3.5, 3.5)

                else:
                    # Free clear cone up to max range (1.80m)
                    painter.setPen(Qt.NoPen)
                    painter.setBrush(QBrush(QColor(0, 220, 140, 15)))
                    painter.drawPolygon(poly)

                    # Boundary rays (dotted cyan/green)
                    cone_pen = QPen(QColor(0, 200, 130, 90), 1, Qt.DotLine)
                    painter.setPen(cone_pen)
                    painter.drawLine(origin, arc_points[0])
                    painter.drawLine(origin, arc_points[-1])

                    # Outer perimeter arc at max range
                    painter.setPen(QPen(QColor(0, 220, 140, 100), 1, Qt.DotLine))
                    for i in range(len(arc_points) - 1):
                        painter.drawLine(arc_points[i], arc_points[i + 1])

        # 6. Draw Vehicle Chassis Footprint (20x15cm with 2mm bumper outline)
        if self.show_chassis:
            c_tl = self.world_to_screen(-0.075, 0.10)
            c_br = self.world_to_screen(0.075, -0.10)
            chassis_rect = QRectF(c_tl.x(), c_tl.y(), c_br.x() - c_tl.x(), c_br.y() - c_tl.y())

            painter.setPen(QPen(QColor(0, 180, 255, 220), 2))
            painter.setBrush(QBrush(QColor(0, 120, 220, 60)))
            painter.drawRoundedRect(chassis_rect, 4, 4)

            p_arrow_start = self.world_to_screen(0.0, 0.0)
            p_arrow_end = self.world_to_screen(0.0, 0.14)
            painter.setPen(QPen(QColor(0, 255, 200, 240), 2))
            painter.drawLine(p_arrow_start, p_arrow_end)

            for wx, wy in [(-0.070, 0.066), (0.070, 0.066), (-0.070, -0.066), (0.070, -0.066)]:
                wp = self.world_to_screen(wx, wy)
                painter.setPen(Qt.NoPen)
                painter.setBrush(QBrush(QColor(240, 240, 240, 200)))
                painter.drawRect(QRectF(wp.x() - 3, wp.y() - 6, 6, 12))

        # 7. Draw Disconnected Overlay if Webots not streaming
        if not self.is_connected:
            painter.fillRect(QRectF(w * 0.1, h * 0.40, w * 0.8, 70), QColor(20, 25, 35, 210))
            painter.setPen(QPen(QColor(255, 170, 0), 2))
            painter.drawRoundedRect(QRectF(w * 0.1, h * 0.40, w * 0.8, 70), 8, 8)
            painter.setFont(QFont("Segoe UI", 11, QFont.Bold))
            painter.setPen(QPen(QColor(255, 200, 50)))
            painter.drawText(QRectF(w * 0.1, h * 0.41, w * 0.8, 30), Qt.AlignCenter, "Waiting for Webots Telemetry (Port 9876)")
            painter.setFont(QFont("Segoe UI", 9))
            painter.setPen(QPen(QColor(200, 210, 225)))
            painter.drawText(QRectF(w * 0.1, h * 0.45, w * 0.8, 30), Qt.AlignCenter, "Start Webots world Cyan_Champion_Track.wbt and press ▶ Play")

        # 8. Draw Hover Coordinates & Cell Info
        if self.hover_m:
            hx, hy = self.hover_m
            prob_text = f"| P(occ): {self.hover_prob*100:3.0f}%" if self.hover_prob is not None else ""
            info_str = f"X: {hx:+.2f}m | Y: {hy:+.2f}m {prob_text}"
            painter.setPen(QPen(QColor(220, 230, 245), 1))
            painter.setFont(QFont("Consolas", 9))
            painter.drawText(12, h - 12, info_str)


# ==============================================================================
# 2. MAIN GUI WINDOW WITH SIMULATION CONTROLS
# ==============================================================================
class CostmapGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Folkrace MPPI & 16-Channel ToF Costmap Live Visualizer")

        # Position window on the right half of the monitor (Split-Screen layout, excluding taskbar)
        screen = QApplication.primaryScreen()
        if screen is not None:
            avail = screen.availableGeometry()  # Area excluding Windows taskbar (e.g. 2560x1550)
            right_w = avail.width() // 2
            right_h = avail.height()
            right_x = avail.x() + right_w
            right_y = avail.y()
            self.setGeometry(right_x, right_y, right_w, right_h)
        else:
            self.setGeometry(1280, 0, 1280, 1540)

        # 1. Initialize Master Costmap Fusion Pipeline with C-Space Obstacle Inflation
        self.r_inflation_m = 0.085  # Default 8.5 cm C-Space inflation (8-13 cm range)
        self.costmap_pipe = FastCostmapPipeline(
            grid_bounds=(-1.5, 1.5, -0.4, 2.4),
            resolution=0.02,
            enable_latency_comp=True,
            r_inflation_m=self.r_inflation_m,
        )

        # 2. Pure Live Cartographer SLAM Map (Zero static map preloading, 100% live SLAM from Port 9875)
        self.carto_grid = None
        self.carto_origin = (0.0, 0.0)
        self.carto_res = 0.02
        self.carto_name = "Live SLAM Stream (Waiting for Cartographer...)"

        # Simulation control state (Default 0.50x speed for smooth laptop CPU real-time processing)
        self.sim_paused = False
        self.sim_speed_mult = 0.5
        self.last_packet_time = 0.0

        # Thread-safe packet buffers
        self._latest_packet = None
        self._latest_map_pkt = None
        self._packet_lock = threading.Lock()

        # UDP Sockets: Telemetry Receiver (9876), Command Sender (9877), and Live SLAM Map Receiver (9875)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self.map_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.map_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.map_sock.bind(("0.0.0.0", 9875))
            self.map_sock.setblocking(False)
        except Exception as e:
            print(f"[GUI] Warning: Could not bind live map port 9875: {e}", flush=True)

        self.udp_running = True
        self.udp_connected = False
        self.udp_thread = threading.Thread(target=self._udp_listener, daemon=True)
        self.udp_thread.start()

        self._build_ui()
        self.cb_speed.setCurrentIndex(3)  # 0.50x Half Speed default

        # Initialize clean canvas
        default_tof = {s.name: 1.80 for s in SENSOR_RIG}
        self.costmap_pipe.update_pose(0.0, 0.60, 0.00, 0.0, 0.0)
        prob, hazard, _ = self.costmap_pipe.process_tof_frame(default_tof, 0.0)
        self.canvas.set_data(prob, hazard, default_tof)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_timer)
        self.timer.start(20)

    def _build_ui(self):
        main_widget = QWidget(self)
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # Left: 2D Costmap Canvas
        self.canvas = CostmapCanvas(self)
        main_layout.addWidget(self.canvas, stretch=3)

        # Right: Controls & Telemetry Panel
        control_panel = QWidget()
        ctrl_layout = QVBoxLayout(control_panel)
        ctrl_layout.setContentsMargins(2, 2, 2, 2)
        ctrl_layout.setSpacing(8)

        # --- Group 1: Live Simulation Controls ---
        sim_group = QGroupBox("Webots Simulation Controls")
        sim_layout = QVBoxLayout(sim_group)
        sim_layout.setSpacing(6)

        # Play / Pause & Single Step Buttons
        btn_row = QHBoxLayout()
        self.btn_play_pause = QPushButton("⏸ Pause")
        self.btn_play_pause.setStyleSheet("font-weight: bold; background-color: #2e7d32; color: white; padding: 6px;")
        self.btn_play_pause.clicked.connect(self._toggle_play_pause)
        btn_row.addWidget(self.btn_play_pause)

        self.btn_step = QPushButton("⏭ Step (10ms)")
        self.btn_step.setStyleSheet("font-weight: bold; background-color: #0277bd; color: white; padding: 6px;")
        self.btn_step.clicked.connect(self._send_step_command)
        btn_row.addWidget(self.btn_step)
        sim_layout.addLayout(btn_row)

        # Speed Multiplier Selector
        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("Speed:"))
        self.cb_speed = QComboBox()
        self.speed_options = [
            ("0.05x (1/20th Speed)", 0.05),
            ("0.10x (10x Slow-Mo)", 0.10),
            ("0.25x (Quarter Speed)", 0.25),
            ("0.50x (Half Speed)", 0.50),
            ("1.00x (Real-Time 1x)", 1.00),
            ("2.00x (Double Speed)", 2.00),
            ("5.00x (High Speed)", 5.00),
            ("Max Speed (Uncapped)", 100.0),
        ]
        for label, val in self.speed_options:
            self.cb_speed.addItem(label, val)
        self.cb_speed.setCurrentIndex(4)  # 1.00x default
        self.cb_speed.currentIndexChanged.connect(self._on_speed_changed)
        speed_row.addWidget(self.cb_speed)
        sim_layout.addLayout(speed_row)

        # Action Row: Reset Simulation & Restart GUI
        act_row = QHBoxLayout()
        self.btn_reset = QPushButton("↺ Reset Sim")
        self.btn_reset.setStyleSheet("background-color: #c62828; color: white; font-weight: bold; padding: 5px;")
        self.btn_reset.clicked.connect(self._send_reset_command)
        act_row.addWidget(self.btn_reset)

        self.btn_restart_gui = QPushButton("🔄 Restart GUI")
        self.btn_restart_gui.setStyleSheet("background-color: #37474f; color: #00e5ff; font-weight: bold; padding: 5px; border: 1px solid #00e5ff;")
        self.btn_restart_gui.clicked.connect(self._restart_gui)
        act_row.addWidget(self.btn_restart_gui)
        sim_layout.addLayout(act_row)
        ctrl_layout.addWidget(sim_group)

        # --- Group 2: Viewport & Zoom Controls ---
        zoom_group = QGroupBox("Viewport & Zoom Controls")
        zoom_layout = QHBoxLayout(zoom_group)
        btn_zin = QPushButton("🔍 Zoom In")
        btn_zin.clicked.connect(lambda: self.canvas.zoom(0.85))
        btn_zout = QPushButton("🔎 Zoom Out")
        btn_zout.clicked.connect(lambda: self.canvas.zoom(1.18))
        btn_zreset = QPushButton("Reset View")
        btn_zreset.clicked.connect(self.canvas.reset_view)
        zoom_layout.addWidget(btn_zin)
        zoom_layout.addWidget(btn_zout)
        zoom_layout.addWidget(btn_zreset)
        ctrl_layout.addWidget(zoom_group)

        # --- Group 3: Live Status & Telemetry ---
        status_group = QGroupBox("Live Status & Vehicle Telemetry")
        st_layout = QVBoxLayout(status_group)
        self.lbl_status = QLabel("🔴 WAITING FOR WEBOTS...")
        self.lbl_status.setStyleSheet("font-weight: bold; color: #ff5252;")
        self.lbl_carto = QLabel("Cartographer: ⚪ Waiting for Live SLAM...")
        self.lbl_carto.setFont(QFont("Consolas", 8, QFont.Bold))
        self.lbl_carto.setStyleSheet("color: #00e676;")
        self.lbl_vehicle = QLabel("Speed: 0.00 m/s | Steer: +0.0° | CTE: +0.0 cm")
        self.lbl_vehicle.setFont(QFont("Consolas", 8))
        self.lbl_mppi = QLabel("MPPI: 0.0 ms | Cost: 0.0 | Pruned: 0.0%")
        self.lbl_mppi.setFont(QFont("Consolas", 8))
        self.lbl_seeds = QLabel("Seeds: Center=0.00 | Left=0.00 | Right=0.00")
        self.lbl_seeds.setFont(QFont("Consolas", 8))

        st_layout.addWidget(self.lbl_status)
        st_layout.addWidget(self.lbl_carto)
        st_layout.addWidget(self.lbl_vehicle)
        st_layout.addWidget(self.lbl_mppi)
        st_layout.addWidget(self.lbl_seeds)
        ctrl_layout.addWidget(status_group)

        # --- Group 4: Obstacle Expansion & Costmap Parameters ---
        param_group = QGroupBox("C-Space Obstacle Expansion & Tuning")
        p_layout = QVBoxLayout(param_group)

        self.lbl_inflation = QLabel(f"Obstacle Expansion Radius: {self.r_inflation_m*100:.1f} cm (GA: 8-13 cm)")
        self.slider_inflation = QSlider(Qt.Horizontal)
        self.slider_inflation.setRange(80, 130)  # 8.0 cm to 13.0 cm
        self.slider_inflation.setValue(int(self.r_inflation_m * 1000))
        self.slider_inflation.valueChanged.connect(self._on_inflation_changed)
        p_layout.addWidget(self.lbl_inflation)
        p_layout.addWidget(self.slider_inflation)

        ctrl_layout.addWidget(param_group)

        # --- Group 5: Display Layers ---
        disp_group = QGroupBox("Display Layers")
        disp_layout = QVBoxLayout(disp_group)

        self.cb_colormap = QComboBox()
        self.cb_colormap.addItems(["Turbo", "Inferno", "Grayscale"])
        self.cb_colormap.currentTextChanged.connect(self._on_colormap_change)
        disp_layout.addWidget(QLabel("Colormap:"))
        disp_layout.addWidget(self.cb_colormap)

        self.chk_hazard = QCheckBox("Show Red C-Space Hazard Boundary")
        self.chk_hazard.setChecked(True)
        self.chk_hazard.toggled.connect(lambda v: setattr(self.canvas, "show_hazard_mask", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_hazard)

        self.chk_rays = QCheckBox("Show 16 Conical 11.25° ToF Beams")
        self.chk_rays.setChecked(True)
        self.chk_rays.toggled.connect(lambda v: setattr(self.canvas, "show_rays", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_rays)

        self.chk_rollouts = QCheckBox("Show MPPI Trajectory Rollouts")
        self.chk_rollouts.setChecked(True)
        self.chk_rollouts.toggled.connect(lambda v: setattr(self.canvas, "show_rollouts", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_rollouts)

        self.chk_track_ref = QCheckBox("Show Thin Track Racing Lines")
        self.chk_track_ref.setChecked(True)
        self.chk_track_ref.toggled.connect(lambda v: setattr(self.canvas, "show_track_ref", v) or self.canvas.update())
        disp_layout.addWidget(self.chk_track_ref)
        ctrl_layout.addWidget(disp_group)

        # --- Group 6: 16-Channel Telemetry Table ---
        tof_group = QGroupBox("16-Channel Range Readings (cm)")
        tof_layout = QVBoxLayout(tof_group)
        self.lbl_tof_fl = QLabel("FL: 180, 180, 180, 180 cm")
        self.lbl_tof_fr = QLabel("FR: 180, 180, 180, 180 cm")
        self.lbl_tof_bl = QLabel("BL: 180, 180, 180, 180 cm")
        self.lbl_tof_br = QLabel("BR: 180, 180, 180, 180 cm")
        for lbl in [self.lbl_tof_fl, self.lbl_tof_fr, self.lbl_tof_bl, self.lbl_tof_br]:
            lbl.setFont(QFont("Consolas", 8))
            tof_layout.addWidget(lbl)
        ctrl_layout.addWidget(tof_group)

        ctrl_layout.addStretch()
        main_layout.addWidget(control_panel, stretch=1)

    def _send_cmd(self, cmd_dict: dict):
        """Sends a JSON command to mppi_controller on port 9877."""
        try:
            raw = json.dumps(cmd_dict).encode("utf-8")
            self.udp_sock.sendto(raw, ("127.0.0.1", 9877))
        except Exception as e:
            print(f"[GUI] Error sending command: {e}", flush=True)

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
        self.sim_paused = True
        self.btn_play_pause.setText("▶ Resume")
        self.btn_play_pause.setStyleSheet("font-weight: bold; background-color: #f57f17; color: white; padding: 6px;")
        self._send_cmd({"cmd": "step"})

    def _on_speed_changed(self, idx: int):
        speed_val = self.cb_speed.currentData()
        self.sim_speed_mult = speed_val
        self._send_cmd({"cmd": "set_speed", "val": speed_val})

    def _send_reset_command(self):
        self._send_cmd({"cmd": "reset"})

    def _restart_gui(self):
        """Restarts the GUI process in the right half of the screen."""
        try:
            python_bin = sys.executable
            workspace_launcher = os.path.join(WORKSPACE_ROOT, "launch_costmap_gui.py")
            if os.path.exists(workspace_launcher):
                target_script = workspace_launcher
            else:
                target_script = os.path.abspath(__file__)
            subprocess.Popen([python_bin, target_script])
            QApplication.quit()
        except Exception as e:
            print(f"[GUI] Error restarting GUI: {e}", flush=True)

    def _on_colormap_change(self, text: str):
        self.canvas.colormap_name = text
        self.canvas.update()

    def _on_inflation_changed(self, val: int):
        self.r_inflation_m = val / 1000.0
        self.lbl_inflation.setText(f"Obstacle Expansion Radius: {self.r_inflation_m*100:.1f} cm (GA: 8-13 cm)")
        self.costmap_pipe.set_inflation_radius(self.r_inflation_m)
        self.canvas.update()

    def _update_tof_labels(self, tof_readings: dict):
        fl = [int(tof_readings.get(f"tof_fl_{i}", 1.80) * 100) for i in range(4)]
        fr = [int(tof_readings.get(f"tof_fr_{i}", 1.80) * 100) for i in range(4)]
        bl = [int(tof_readings.get(f"tof_bl_{i}", 1.80) * 100) for i in range(4)]
        br = [int(tof_readings.get(f"tof_br_{i}", 1.80) * 100) for i in range(4)]
        self.lbl_tof_fl.setText(f"FL: {fl[0]:3d}, {fl[1]:3d}, {fl[2]:3d}, {fl[3]:3d} cm")
        self.lbl_tof_fr.setText(f"FR: {fr[0]:3d}, {fr[1]:3d}, {fr[2]:3d}, {fr[3]:3d} cm")
        self.lbl_tof_bl.setText(f"BL: {bl[0]:3d}, {bl[1]:3d}, {bl[2]:3d}, {bl[3]:3d} cm")
        self.lbl_tof_br.setText(f"BR: {br[0]:3d}, {br[1]:3d}, {br[2]:3d}, {br[3]:3d} cm")

    def _udp_listener(self):
        """Receives live costmap & MPPI rollouts broadcast from Webots on port 9876."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("127.0.0.1", 9876))
            sock.settimeout(0.5)
        except Exception:
            try:
                sock.bind(("", 9876))
                sock.settimeout(0.5)
            except Exception as e:
                print(f"[GUI UDP] Could not bind port 9876: {e}", flush=True)
                return

        while self.udp_running:
            # 1. Non-blocking read of Live Cartographer SLAM OccupancyGrid updates on port 9875
            try:
                while True:
                    m_data, _ = self.map_sock.recvfrom(65535)
                    m_pkt = json.loads(m_data.decode("utf-8"))
                    with self._packet_lock:
                        self._latest_map_pkt = m_pkt
            except (BlockingIOError, socket.error):
                pass
            except Exception:
                pass

            # 2. Read live telemetry on port 9876
            try:
                data, _ = sock.recvfrom(65535)
                packet = json.loads(data.decode("utf-8"))
                with self._packet_lock:
                    self._latest_packet = packet
                    self.last_packet_time = time.time()
                    self.udp_connected = True
            except socket.timeout:
                if time.time() - self.last_packet_time > 1.5:
                    with self._packet_lock:
                        self.udp_connected = False
            except Exception:
                pass

    def _on_timer(self):
        """Thread-safe UI render loop running at smooth 30 FPS on the main Qt thread."""
        pkt = None
        map_pkt = None
        is_conn = False

        with self._packet_lock:
            pkt = self._latest_packet
            map_pkt = self._latest_map_pkt
            is_conn = self.udp_connected
            # Clear processed live map packet
            self._latest_map_pkt = None

        self.canvas.is_connected = is_conn

        # 1. Process Live Cartographer OccupancyGrid update if received
        if map_pkt is not None:
            try:
                w = map_pkt["w"]
                h = map_pkt["h"]
                res = map_pkt["res"]
                origin = tuple(map_pkt["origin"])
                compressed = base64.b64decode(map_pkt["data_z64"])
                raw_bytes = zlib.decompress(compressed)
                self.carto_grid = np.frombuffer(raw_bytes, dtype=np.int8).reshape((h, w))
                self.carto_origin = origin
                self.carto_res = res
                self.carto_name = f"Live Cartographer SLAM ({h}x{w})"
            except Exception:
                pass

        # 2. Process Telemetry & Render Costmap
        if pkt is not None and is_conn:
            timestamp = pkt.get("time", 0.0)
            pos = pkt.get("pos", [0.0, 0.0])
            yaw = pkt.get("yaw", 0.0)
            v_cur = pkt.get("v", 0.0)
            tof_readings = pkt.get("tof", {})
            best_traj = pkt.get("best_traj", None)
            seed_center = pkt.get("seed_center", None)
            seed_straight = pkt.get("seed_straight", None)
            seed_left = pkt.get("seed_left", None)
            seed_right = pkt.get("seed_right", None)
            cand_trajs = pkt.get("cand_trajs", None)
            track_ref_center = pkt.get("track_ref_center", None)
            track_ref_left = pkt.get("track_ref_left", None)
            track_ref_right = pkt.get("track_ref_right", None)

            stamped_pose = StampedPose(timestamp, pos[0], pos[1], yaw, v_cur)
            if self.carto_grid is not None:
                self.costmap_pipe.update_slam_map(self.carto_grid, self.carto_origin, self.carto_res, stamped_pose)

            self.costmap_pipe.update_pose(timestamp, pos[0], pos[1], yaw, v_cur)
            prob_map, hazard_mask, dt_ms = self.costmap_pipe.process_tof_frame(tof_readings, timestamp)

            self.canvas.set_data(prob_map, hazard_mask, tof_readings)
            self.canvas.set_rollouts(
                best=best_traj,
                candidates=cand_trajs,
                seed_center=seed_center,
                seed_straight=seed_straight,
                seed_left=seed_left,
                seed_right=seed_right,
                track_ref_center=track_ref_center,
                track_ref_left=track_ref_left,
                track_ref_right=track_ref_right,
            )

            v_tgt = pkt.get("speed", 0.0)
            steer_deg = math.degrees(pkt.get("steer", 0.0))
            cte_cm = pkt.get("cte", 0.0) * 100.0
            cost = pkt.get("best_cost", 0.0)
            pruned = pkt.get("collision_rate", 0.0) * 100.0
            seeds = pkt.get("seed_weights", {})
            sim_state = pkt.get("sim_state", {})

            wall_cells = int(np.sum(prob_map > 0.55))
            hazard_cells = int(np.sum(hazard_mask))
            if wall_cells > 0:
                self.lbl_carto.setText(f"Cartographer: 🟢 {wall_cells} Wall Cells | {hazard_cells} Hazard")
                self.lbl_carto.setStyleSheet("color: #00e676;")
            else:
                self.lbl_carto.setText(f"Cartographer: ⚪ 0 Wall Cells (X={pos[0]:+.2f}, Y={pos[1]:+.2f})")
                self.lbl_carto.setStyleSheet("color: #b0bec5;")

            self.lbl_vehicle.setText(f"v={v_cur:.2f}m/s (tgt: {v_tgt:.2f}) | Steer={steer_deg:+5.1f}° | CTE={cte_cm:+5.1f}cm")
            self.lbl_mppi.setText(f"Cost: {cost:5.1f} | Pruned: {pruned:4.1f}% | Latency: {dt_ms:.2f}ms")
            self.lbl_seeds.setText(f"Seeds: C={seeds.get('center',0.0):.2f} | S={seeds.get('straight',0.0):.2f} | L={seeds.get('left',0.0):.2f} | R={seeds.get('right',0.0):.2f}")
            self._update_tof_labels(tof_readings)

            if "paused" in sim_state and sim_state["paused"] != self.sim_paused:
                self.sim_paused = sim_state["paused"]
                if self.sim_paused:
                    self.btn_play_pause.setText("▶ Resume")
                    self.btn_play_pause.setStyleSheet("font-weight: bold; background-color: #f57f17; color: white; padding: 6px;")
                else:
                    self.btn_play_pause.setText("⏸ Pause")
                    self.btn_play_pause.setStyleSheet("font-weight: bold; background-color: #2e7d32; color: white; padding: 6px;")

        # 3. Connection status indicator
        if not hasattr(self, "_last_gui_connected") or self._last_gui_connected != is_conn:
            self._last_gui_connected = is_conn
            if is_conn:
                self.lbl_status.setText("🟢 LIVE WEBOTS STREAM CONNECTED (0.50x Speed)")
                self.lbl_status.setStyleSheet("font-weight: bold; color: #00e676;")
            else:
                self.lbl_status.setText("🔴 WAITING FOR WEBOTS: Start world & press ▶ in Webots")
                self.lbl_status.setStyleSheet("font-weight: bold; color: #ff5252;")


def launch_gui():
    if not PYQT_AVAILABLE:
        print("[Costmap GUI] PyQt5 is required. Run: pip install PyQt5", flush=True)
        return
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    gui = CostmapGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    launch_gui()
