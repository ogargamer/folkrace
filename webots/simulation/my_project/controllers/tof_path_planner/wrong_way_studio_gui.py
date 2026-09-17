"""
================================================================================
  ANTI-WRONG-WAY WATCHDOG & 180 AUTO-CORRECTION STUDIO
  CUSTOM DESIGNED FOR RIGHT HALF OF 2560x1600 MONITOR (1280x1530)
================================================================================
  - Perfectly proportioned for 1280x1530 px (Right Half of 2560x1600).
  - 60 Hz Lean Telemetry Stream (450 bytes) with instantaneous visual feedback.
  - Tactical 2D Radar Canvas (480px x 400px):
      * Concentric polar range rings & chassis orientation
      * Dynamic breadcrumb trail & 180-deg gold recovery target vector
      * Front Pillar (2.60, 0.40) & Back Pillar (-0.21, 0.39) active markers
      * Turning radius arc circle vs R_min (0.32m)
  - Master Executive Verdict & Telemetry Hub Panel.
  - 2x2 Deep Thought Process Matrix with:
      * Dedicated high-contrast card frames with status pills
      * Live telemetry gauges & progress indicators
      * Real-time mathematical reasoning consoles (QTextEdit)
================================================================================
"""

import os
import sys
import math
import json
import time
import socket
import threading
from typing import Dict, List, Optional, Tuple, Any

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF
from PyQt5.QtGui import QColor, QPainter, QPen, QBrush, QFont, QLinearGradient, QPolygonF
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QComboBox, QPushButton, QGroupBox,
    QGridLayout, QFrame, QProgressBar, QTextEdit
)

UDP_RECV_PORT = 9876
UDP_CMD_PORT = 9877

FRONT_PILLAR_WORLD_POS = (2.60, 0.40)
BACK_PILLAR_WORLD_POS = (-0.21, 0.39)


# ==============================================================================
# 1. 2D TACTICAL RADAR CANVAS
# ==============================================================================
class WrongWayRadarCanvas(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(460, 380)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setMouseTracking(True)

        self.pose = [0.0, 0.0, 0.0]  # [x, y, yaw]
        self.target_yaw = 0.0
        self.action = "DRIVE"
        self.is_wrong_way = False
        self.trail: List[List[float]] = []
        self.turn_radius_m = 0.50
        self.min_radius_m = 0.32
        self.front_pillar_active = True
        self.back_pillar_active = True

    def set_telemetry(self, aww_data: dict, pose: list, action: str, fp_active: bool, bp_active: bool):
        self.pose = pose if pose and len(pose) >= 3 else [0.0, 0.0, 0.0]
        self.target_yaw = float(aww_data.get("target_yaw", 0.0))
        self.action = action
        self.is_wrong_way = bool(aww_data.get("is_wrong_way", False))
        self.trail = aww_data.get("trail", [])

        opt2 = aww_data.get("opt2", {})
        self.turn_radius_m = float(opt2.get("radius_m", 0.50))
        self.min_radius_m = float(opt2.get("min_radius_m", 0.32))

        self.front_pillar_active = fp_active
        self.back_pillar_active = bp_active
        self.update()

    def world_to_screen(self, xm: float, ym: float, cx: float, cy: float, scale: float) -> QPointF:
        dx = xm - self.pose[0]
        dy = ym - self.pose[1]
        sx = cx + dx * scale
        sy = cy - dy * scale
        return QPointF(sx, sy)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        cx = w / 2.0
        cy = h / 2.0
        scale = 120.0  # pixels per meter

        # 1. Dark Radar Background
        grad = QLinearGradient(0, 0, 0, h)
        grad.setColorAt(0.0, QColor(10, 14, 20))
        grad.setColorAt(1.0, QColor(6, 8, 12))
        painter.fillRect(0, 0, w, h, grad)

        # Border around canvas
        painter.setPen(QPen(QColor(36, 44, 56), 1))
        painter.drawRect(0, 0, w - 1, h - 1)

        # 2. Concentric Radar Range Rings (0.5m, 1.0m, 1.5m, 2.0m)
        painter.setBrush(Qt.NoBrush)
        for r_m in [0.5, 1.0, 1.5, 2.0]:
            r_px = r_m * scale
            painter.setPen(QPen(QColor(32, 42, 54), 1, Qt.DashLine))
            painter.drawEllipse(QPointF(cx, cy), r_px, r_px)
            painter.setPen(QPen(QColor(65, 85, 110), 1))
            painter.setFont(QFont("Consolas", 8))
            painter.drawText(int(cx + r_px - 26), int(cy - 4), f"{r_m:.1f}m")

        # 3. Crosshairs
        painter.setPen(QPen(QColor(38, 50, 64), 1))
        painter.drawLine(0, int(cy), w, int(cy))
        painter.drawLine(int(cx), 0, int(cx), h)

        # 4. Breadcrumb Historical Trail
        if len(self.trail) > 1:
            pen_trail = QPen(QColor(0, 220, 255, 160), 2.5, Qt.SolidLine, Qt.RoundCap)
            painter.setPen(pen_trail)
            for i in range(len(self.trail) - 1):
                p1 = self.world_to_screen(self.trail[i][0], self.trail[i][1], cx, cy, scale)
                p2 = self.world_to_screen(self.trail[i+1][0], self.trail[i+1][1], cx, cy, scale)
                if (-50 <= p1.x() <= w + 50 or -50 <= p2.x() <= w + 50) and (-50 <= p1.y() <= h + 50 or -50 <= p2.y() <= h + 50):
                    painter.drawLine(p1, p2)

        # 5. Front & Back Pillar Markers
        self._draw_pillar_marker(painter, FRONT_PILLAR_WORLD_POS[0], FRONT_PILLAR_WORLD_POS[1],
                                 cx, cy, scale, "FP (FRONT)", self.front_pillar_active, QColor(255, 60, 60), QColor(0, 230, 118))
        self._draw_pillar_marker(painter, BACK_PILLAR_WORLD_POS[0], BACK_PILLAR_WORLD_POS[1],
                                 cx, cy, scale, "BP (BACK)", self.back_pillar_active, QColor(255, 60, 60), QColor(64, 150, 255))

        # 6. Kinematic Turning Radius Circle (Option 2)
        if 0.05 < self.turn_radius_m < 2.5:
            r_circle_px = self.turn_radius_m * scale
            is_tight = (self.turn_radius_m < self.min_radius_m)
            col = QColor(255, 60, 60, 140) if is_tight else QColor(0, 230, 118, 80)
            painter.setPen(QPen(col, 2, Qt.DashDotLine))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QPointF(cx, cy), r_circle_px, r_circle_px)

        # 7. Corrective 180 Target Vector (Gold Arrow)
        rx, ry, ryaw = self.pose
        if self.action == "CORRECTIVE_180" or self.is_wrong_way:
            tgt_len_px = 1.30 * scale
            tx = cx + tgt_len_px * math.cos(self.target_yaw)
            ty = cy - tgt_len_px * math.sin(self.target_yaw)
            painter.setPen(QPen(QColor(255, 215, 0), 4, Qt.SolidLine))
            painter.drawLine(QPointF(cx, cy), QPointF(tx, ty))

            ah_len = 16
            ah_angle = self.target_yaw
            p_head = QPointF(tx, ty)
            p_left = QPointF(tx - ah_len * math.cos(ah_angle - 0.45), ty + ah_len * math.sin(ah_angle - 0.45))
            p_right = QPointF(tx - ah_len * math.cos(ah_angle + 0.45), ty + ah_len * math.sin(ah_angle + 0.45))
            painter.setBrush(QBrush(QColor(255, 215, 0)))
            painter.drawPolygon(QPolygonF([p_head, p_left, p_right]))

        # 8. Own Robot Chassis (Centered at cx, cy)
        # In Webots proto, the vehicle's forward longitudinal axis is local +Y,
        # lateral width is local X. Ground-truth supervisor rotation r = [0, 0, 1, yaw]
        # transforms local +Y into world heading angle: heading_rad = ryaw + pi/2.
        # Screen Y is inverted, so screen angle = -degrees(ryaw + pi/2).
        heading_world = ryaw + math.pi / 2.0
        painter.save()
        painter.translate(cx, cy)
        deg = -math.degrees(heading_world)
        painter.rotate(deg)

        # Vehicle physical dimensions: length 26cm (along forward X in rotated frame), width 16cm (lateral Y)
        car_l_px = 0.26 * scale
        car_w_px = 0.16 * scale
        r_rect = QRectF(-car_l_px / 2.0, -car_w_px / 2.0, car_l_px, car_w_px)

        chassis_color = QColor(255, 50, 70, 220) if self.is_wrong_way else QColor(31, 111, 235, 220)
        painter.setBrush(QBrush(chassis_color))
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawRoundedRect(r_rect, 5, 5)

        # Wheels (FL, FR, RL, RR)
        wh_l = 0.055 * scale
        wh_w = 0.025 * scale
        wh_pen = QPen(QColor(255, 215, 64, 240), 1.5)
        wh_brush = QBrush(QColor(25, 25, 30, 240))
        painter.setPen(wh_pen)
        painter.setBrush(wh_brush)
        # Front wheels (+X forward)
        painter.drawRoundedRect(QRectF(car_l_px * 0.22 - wh_l / 2.0, -car_w_px / 2.0 - wh_w * 0.4, wh_l, wh_w), 2, 2)
        painter.drawRoundedRect(QRectF(car_l_px * 0.22 - wh_l / 2.0, car_w_px / 2.0 - wh_w * 0.6, wh_l, wh_w), 2, 2)
        # Rear wheels (-X rear)
        painter.drawRoundedRect(QRectF(-car_l_px * 0.25 - wh_l / 2.0, -car_w_px / 2.0 - wh_w * 0.4, wh_l, wh_w), 2, 2)
        painter.drawRoundedRect(QRectF(-car_l_px * 0.25 - wh_l / 2.0, car_w_px / 2.0 - wh_w * 0.6, wh_l, wh_w), 2, 2)

        # Front bumper highlight (Distinctive bright cyan / neon green nose)
        fwd_edge_x = car_l_px / 2.0
        bumper_col = QColor(255, 70, 70) if self.is_wrong_way else QColor(0, 255, 180)
        painter.setPen(QPen(bumper_col, 3.5, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(QPointF(fwd_edge_x, -car_w_px / 2.0 + 3), QPointF(fwd_edge_x, car_w_px / 2.0 - 3))

        # Forward heading direction indicator vector (+X in rotated frame)
        painter.setPen(QPen(QColor(0, 229, 255), 3, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(int(fwd_edge_x), 0, int(fwd_edge_x + 22), 0)

        # Arrowhead on forward heading vector
        ah_len = 8
        p_arrow = [
            QPointF(fwd_edge_x + 24, 0),
            QPointF(fwd_edge_x + 24 - ah_len, -4),
            QPointF(fwd_edge_x + 24 - ah_len, +4),
        ]
        painter.setBrush(QBrush(QColor(0, 229, 255)))
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(QPolygonF(p_arrow))
        painter.restore()

        # 9. Top-Left Overlay Legend
        painter.setFont(QFont("Segoe UI", 10, QFont.Bold))
        if self.is_wrong_way or self.action == "CORRECTIVE_180":
            painter.setPen(QPen(QColor(255, 70, 70)))
            painter.drawText(12, 24, "RADAR: 🚨 WRONG-WAY TRIPPED — EXECUTING 180°")
        else:
            painter.setPen(QPen(QColor(0, 230, 118)))
            painter.drawText(12, 24, "RADAR: 🟢 FORWARD CLEAR — ALL WATCHDOGS SATISFIED")

    def _draw_pillar_marker(self, painter: QPainter, px: float, py: float, cx: float, cy: float,
                            scale: float, label: str, is_active: bool, col_active: QColor, col_inactive: QColor):
        pt = self.world_to_screen(px, py, cx, cy, scale)
        if pt.x() < -40 or pt.x() > self.width() + 40 or pt.y() < -40 or pt.y() > self.height() + 40:
            return

        radius = 11.0
        if is_active:
            # Pulsing outer ring
            halo_col = QColor(col_active)
            halo_col.setAlpha(60)
            painter.setBrush(QBrush(halo_col))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(pt, radius + 6, radius + 6)

            painter.setBrush(QBrush(col_active))
            painter.setPen(QPen(QColor(255, 255, 255), 2))
            painter.drawEllipse(pt, radius, radius)

            painter.setFont(QFont("Consolas", 8, QFont.Bold))
            painter.setPen(QPen(QColor(255, 255, 255)))
            painter.drawText(int(pt.x() + 14), int(pt.y() + 4), f"{label}: BLOCKED")
        else:
            painter.setBrush(QBrush(QColor(col_inactive.red(), col_inactive.green(), col_inactive.blue(), 50)))
            painter.setPen(QPen(col_inactive, 1, Qt.DashLine))
            painter.drawEllipse(pt, radius, radius)

            painter.setFont(QFont("Consolas", 8))
            painter.setPen(QPen(QColor(140, 160, 180)))
            painter.drawText(int(pt.x() + 14), int(pt.y() + 4), f"{label}: CLEARED")


# ==============================================================================
# 2. INTERACTIVE ANTI-WRONG-WAY WATCHDOG STUDIO WINDOW
# ==============================================================================
class WrongWayStudioGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Da Winna - Anti-Wrong-Way Watchdog & 180 Auto-Correction Studio")

        # Position cleanly on the right half of a 2560x1600 monitor
        self._position_for_right_screen_half()

        self.setStyleSheet("""
            QMainWindow { background-color: #090d13; color: #c9d1d9; font-family: 'Segoe UI', Tahoma, sans-serif; }
            QLabel { color: #c9d1d9; }
            QPushButton { background-color: #1f2733; border: 1px solid #303d4f; border-radius: 5px; color: #c9d1d9; padding: 6px 12px; font-weight: bold; font-size: 11px; }
            QPushButton:hover { background-color: #2b3648; border-color: #58a6ff; }
            QPushButton:pressed { background-color: #1f6feb; color: #ffffff; }
            QComboBox { background-color: #161e29; border: 1px solid #303d4f; border-radius: 5px; color: #58a6ff; padding: 5px 10px; font-weight: bold; font-size: 11px; }
            QComboBox QAbstractItemView { background-color: #121820; color: #c9d1d9; selection-background-color: #1f6feb; }
            QSlider::groove:horizontal { height: 6px; background: #1f2733; border-radius: 3px; }
            QSlider::sub-page:horizontal { background: #00e676; border-radius: 3px; }
            QSlider::handle:horizontal { background: #58a6ff; width: 14px; margin-top: -4px; margin-bottom: -4px; border-radius: 7px; }
        """)

        # Sockets and threading
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._lock = threading.Lock()
        self.running = True
        self.latest_pkt: Optional[dict] = None
        self.last_packet_time = 0.0
        self.udp_connected = False
        self.packet_count = 0

        self.front_pillar_active = True
        self.back_pillar_active = True

        # Start UDP listener worker thread
        self.listener_thread = threading.Thread(target=self._udp_worker, daemon=True)
        self.listener_thread.start()

        self._build_ui()

        # 50 Hz UI Refresh Timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._udp_poll_and_render)
        self.timer.start(20)

    def _position_for_right_screen_half(self):
        """Positions window on the exact right half of a 2560x1600 monitor."""
        screen = QApplication.primaryScreen()
        if screen:
            avail = screen.availableGeometry()
            half_w = avail.width() // 2
            self.setGeometry(avail.x() + half_w, avail.y(), half_w, avail.height())
        else:
            self.setGeometry(1280, 0, 1280, 1530)
        self.setMinimumSize(960, 750)

    def _build_ui(self):
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 8, 10, 8)
        main_layout.setSpacing(8)

        # ======================================================================
        # TOP TOOLBAR: Control & Pillar Action Bar (~70px)
        # ======================================================================
        ctrl_frame = QFrame()
        ctrl_frame.setStyleSheet("background-color: #121820; border: 1px solid #232b36; border-radius: 6px;")
        ctrl_vlayout = QVBoxLayout(ctrl_frame)
        ctrl_vlayout.setContentsMargins(10, 6, 10, 6)
        ctrl_vlayout.setSpacing(6)

        # Row 1: Mode, Speed, Reset
        r1 = QHBoxLayout()
        r1.setSpacing(8)

        lbl_m = QLabel("Watchdog Mode:")
        lbl_m.setStyleSheet("font-weight: bold; color: #8b949e; font-size: 11px;")
        r1.addWidget(lbl_m)

        self.cb_mode = QComboBox()
        self.cb_mode.addItems([
            "OPTION_COMBINED_HYBRID",
            "OPTION_1_BREADCRUMB",
            "OPTION_2_KINEMATIC_ARC",
            "OPTION_3_FLOW_COSTMAP",
            "OPTION_4_LATERAL_SEPARATION",
            "DISABLED",
        ])
        self.cb_mode.currentTextChanged.connect(self._on_mode_changed)
        r1.addWidget(self.cb_mode, stretch=3)

        r1.addSpacing(10)
        lbl_sp = QLabel("Simulation Speed:")
        lbl_sp.setStyleSheet("font-weight: bold; color: #8b949e; font-size: 11px;")
        r1.addWidget(lbl_sp)

        self.slider_speed = QSlider(Qt.Horizontal)
        self.slider_speed.setRange(5, 300)
        self.slider_speed.setValue(50)
        self.slider_speed.valueChanged.connect(self._on_speed_changed)
        r1.addWidget(self.slider_speed, stretch=2)

        self.lbl_speed_val = QLabel("0.50x")
        self.lbl_speed_val.setStyleSheet("font-family: Consolas; font-weight: bold; color: #00e676; font-size: 11px;")
        r1.addWidget(self.lbl_speed_val)

        r1.addSpacing(10)
        self.btn_reset = QPushButton("🔄 Reset Robot [R]")
        self.btn_reset.setStyleSheet("""
            QPushButton { background-color: #1f6feb; border: 1px solid #388bfd; border-radius: 5px; color: #ffffff; font-weight: bold; padding: 5px 12px; }
            QPushButton:hover { background-color: #388bfd; }
            QPushButton:pressed { background-color: #1158c7; }
        """)
        self.btn_reset.clicked.connect(self._on_reset_clicked)
        r1.addWidget(self.btn_reset)
        ctrl_vlayout.addLayout(r1)

        # Row 2: Pillar Toggles & Connection Status
        r2 = QHBoxLayout()
        r2.setSpacing(8)

        self.btn_front = QPushButton("🛑 Front Pillar: BLOCKED (Raised) [F]")
        self.btn_front.clicked.connect(self._on_toggle_front_pillar)
        self._update_front_button_style(True)
        r2.addWidget(self.btn_front, stretch=2)

        self.btn_back = QPushButton("🛑 Back Pillar: BLOCKED (Raised) [B]")
        self.btn_back.clicked.connect(self._on_toggle_back_pillar)
        self._update_back_button_style(True)
        r2.addWidget(self.btn_back, stretch=2)

        r2.addSpacing(12)
        self.lbl_conn_status = QLabel("🔴 Waiting for Webots: Run test_wrong_way.wbt")
        self.lbl_conn_status.setStyleSheet("color: #ff5252; font-weight: bold; font-size: 11px;")
        r2.addWidget(self.lbl_conn_status)
        ctrl_vlayout.addLayout(r2)

        main_layout.addWidget(ctrl_frame)

        # ======================================================================
        # MIDDLE ROW: Radar Canvas (Left) + Master Executive Verdict & HUD (Right)
        # ======================================================================
        mid_layout = QHBoxLayout()
        mid_layout.setSpacing(8)

        # Left: 2D Radar Canvas
        self.canvas = WrongWayRadarCanvas()
        mid_layout.addWidget(self.canvas, stretch=5)

        # Right: Master Executive Verdict & Telemetry Hub Panel
        hud_panel = QFrame()
        hud_panel.setStyleSheet("background-color: #121820; border: 1px solid #232b36; border-radius: 6px;")
        hud_vlayout = QVBoxLayout(hud_panel)
        hud_vlayout.setContentsMargins(10, 8, 10, 8)
        hud_vlayout.setSpacing(6)

        # Master Verdict Group
        self.grp_master = QGroupBox("Master Anti-Wrong-Way Executive Verdict")
        self.grp_master.setStyleSheet("QGroupBox { font-size: 12px; font-weight: bold; color: #00e676; border: 1px solid #238636; border-radius: 6px; margin-top: 6px; padding-top: 10px; background-color: #161f28; }")
        gm_layout = QVBoxLayout(self.grp_master)
        gm_layout.setContentsMargins(10, 6, 10, 6)
        gm_layout.setSpacing(4)

        self.lbl_verdict = QLabel("🟢 ALL SYSTEMS CLEAR — FORWARD RACING")
        self.lbl_verdict.setStyleSheet("font-size: 13px; font-weight: bold; color: #00e676;")
        gm_layout.addWidget(self.lbl_verdict)

        self.lbl_action = QLabel("Executive Action: NORMAL_DRIVE | Heading: +0.0° | Yaw Error to Forward: 0.0°")
        self.lbl_action.setStyleSheet("font-size: 11px; color: #8b949e;")
        gm_layout.addWidget(self.lbl_action)

        self.lbl_authority = QLabel("Steering Authority: STANDARD DYNAMIC PID LOCK (+-24.5°)")
        self.lbl_authority.setStyleSheet("font-size: 11px; color: #58a6ff;")
        gm_layout.addWidget(self.lbl_authority)
        hud_vlayout.addWidget(self.grp_master)

        # Quick Performance Chips (2x2 grid inside HUD)
        chips_grid = QGridLayout()
        chips_grid.setSpacing(6)

        self.chip_speed = self._create_hud_chip("Speed (v_est / v_cmd)", "0.00 / 0.00 m/s")
        self.chip_steer = self._create_hud_chip("Steering Angle", "+0.0°")
        self.chip_dist = self._create_hud_chip("Centerline Dist / Lap", "0.00 m (Lap 0)")
        self.chip_status = self._create_hud_chip("Watchdog Status", "ALL CLEAR")

        chips_grid.addWidget(self.chip_speed[0], 0, 0)
        chips_grid.addWidget(self.chip_steer[0], 0, 1)
        chips_grid.addWidget(self.chip_dist[0], 1, 0)
        chips_grid.addWidget(self.chip_status[0], 1, 1)
        hud_vlayout.addLayout(chips_grid)

        mid_layout.addWidget(hud_panel, stretch=6)
        main_layout.addLayout(mid_layout)

        # ======================================================================
        # BOTTOM SECTION: 2x2 THOUGHT PROCESS MATRIX (PROPORTIONED CARDS)
        # ======================================================================
        cards_panel = QFrame()
        cards_panel.setStyleSheet("background-color: transparent;")
        cards_panel.setFixedHeight(540)
        cards_grid = QGridLayout(cards_panel)
        cards_grid.setContentsMargins(0, 0, 0, 0)
        cards_grid.setSpacing(8)

        # ----------------------------------------------------------------------
        # CARD 1: Option 1 Breadcrumb Trail Overlap
        # ----------------------------------------------------------------------
        card1, self.badge_opt1, self.prog_opt1, self.lbl_opt1_metric, self.txt_opt1 = self._create_option_card(
            title="Option 1: Breadcrumb Trail Overlap & Auto-180 Correction",
            badge_init="[ PASS ] FORWARD PATH CLEAR",
            metric_init="Trail Overlap Score: 0%  (Threshold: >= 70%)",
            has_prog=True,
            initial_thought=(
                "Option 1 monitors the vehicle's historical breadcrumb path.\n"
                "- If the car reverses or loops back onto its own fresh breadcrumb trail with >70% overlap,\n"
                "  it triggers an immediate 180° corrective spin to face forward racing direction.\n"
                "- Status: Waiting for active breadcrumb stream..."
            )
        )
        cards_grid.addWidget(card1, 0, 0)

        # ----------------------------------------------------------------------
        # CARD 2: Option 2 Kinematic Turn Radius Filter
        # ----------------------------------------------------------------------
        card2, self.badge_opt2, self.prog_opt2, self.lbl_opt2_metric, self.txt_opt2 = self._create_option_card(
            title="Option 2: Kinematic Turn Radius Filter (R >= R_min 0.32m)",
            badge_init="[ PASS ] VALID RACING HAIRPIN ARC",
            metric_init="Measured Turn Radius: R = 0.50m  (Track Hairpin Limit: 0.32m)",
            has_prog=False,
            initial_thought=(
                "Option 2 computes instantaneous turning radius R = v_est / abs(yaw_rate).\n"
                "- If the vehicle pivots tighter than the physical track minimum hairpin radius (0.32m)\n"
                "  during a >90° turn, it flags an illegal in-lane pirouette and commands a 180° spin.\n"
                "- Status: Arc satisfied (R >= 0.32m)."
            )
        )
        cards_grid.addWidget(card2, 0, 1)

        # ----------------------------------------------------------------------
        # CARD 3: Option 3 Directional Flow Costmap Stamping
        # ----------------------------------------------------------------------
        card3, self.badge_opt3, self.prog_opt3, self.lbl_opt3_metric, self.txt_opt3 = self._create_option_card(
            title="Option 3: Directional Flow Costmap Stamping",
            badge_init="[ PASS ] FORWARD FLOW ALIGNED",
            metric_init="Flow Alignment Dot Product: +1.00  (Opposing Threshold: <= -0.50)",
            has_prog=False,
            initial_thought=(
                "Option 3 continuously stamps forward heading vectors into a spatial corridor flow map.\n"
                "- When lookahead queries find heading dot product <= -0.50 against historical flow,\n"
                "  the vehicle is driving against traffic and triggers an auto-180° recovery.\n"
                "- Status: Heading aligned with traffic (+1.00)."
            )
        )
        cards_grid.addWidget(card3, 1, 0)

        # ----------------------------------------------------------------------
        # CARD 4: Option 4 Lateral Corridor Separation Watchdog
        # ----------------------------------------------------------------------
        card4, self.badge_opt4, self.prog_opt4, self.lbl_opt4_metric, self.txt_opt4 = self._create_option_card(
            title="Option 4: Lateral Corridor Separation Watchdog",
            badge_init="[ PASS ] CORRIDOR DISPLACEMENT CLEAR",
            metric_init="Lateral Shift: 0.00m  (Minimum Lane Spacing: 0.40m)",
            has_prog=False,
            initial_thought=(
                "Option 4 measures lateral displacement perpendicular to the corridor during U-turns.\n"
                "- If the vehicle performs a 180° turn with <0.40m lateral shift, it is executing an\n"
                "  in-lane reversal inside the same single lane rather than navigating a true track bend.\n"
                "- Status: Corridor displacement clear."
            )
        )
        cards_grid.addWidget(card4, 1, 1)

        main_layout.addWidget(cards_panel)
        # Leave bottom empty with natural background to prevent element stretching
        main_layout.addStretch(1)

    def _create_option_card(
        self,
        title: str,
        badge_init: str,
        metric_init: str,
        has_prog: bool,
        initial_thought: str
    ) -> Tuple[QFrame, QLabel, Optional[QProgressBar], QLabel, QTextEdit]:
        """Builds a crisp, styled card container for one Anti-Wrong-Way option."""
        card = QFrame()
        card.setFixedHeight(260)
        card.setStyleSheet("""
            QFrame {
                background-color: #121820;
                border: 1px solid #283342;
                border-radius: 8px;
            }
        """)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(5)

        # Header Row: Title and Status Pill Badge
        hdr_row = QHBoxLayout()
        hdr_row.setSpacing(8)

        lbl_title = QLabel(title)
        lbl_title.setStyleSheet("font-size: 12px; font-weight: bold; color: #58a6ff; border: none;")
        hdr_row.addWidget(lbl_title, stretch=1)

        badge = QLabel(badge_init)
        badge.setStyleSheet("""
            QLabel {
                background-color: #0d2818;
                border: 1px solid #238636;
                color: #00e676;
                font-size: 11px;
                font-weight: bold;
                border-radius: 4px;
                padding: 3px 8px;
            }
        """)
        hdr_row.addWidget(badge)
        layout.addLayout(hdr_row)

        # Metric & Progress Bar Row
        metric_row = QHBoxLayout()
        metric_row.setSpacing(8)

        lbl_metric = QLabel(metric_init)
        lbl_metric.setStyleSheet("font-size: 11px; color: #c9d1d9; font-weight: 500; border: none;")
        metric_row.addWidget(lbl_metric, stretch=1)

        prog = None
        if has_prog:
            prog = QProgressBar()
            prog.setRange(0, 100)
            prog.setValue(0)
            prog.setFixedHeight(14)
            prog.setFixedWidth(120)
            prog.setStyleSheet("""
                QProgressBar {
                    background-color: #080c10;
                    border: 1px solid #252e3b;
                    border-radius: 3px;
                    text-align: center;
                    color: #ffffff;
                    font-size: 9px;
                    font-weight: bold;
                }
                QProgressBar::chunk {
                    background-color: #00e676;
                    border-radius: 2px;
                }
            """)
            metric_row.addWidget(prog)

        layout.addLayout(metric_row)

        # Live Mathematical Reasoning Console (QTextEdit)
        txt = QTextEdit()
        txt.setReadOnly(True)
        txt.setFixedHeight(120)
        txt.setStyleSheet("""
            QTextEdit {
                background-color: #080c12;
                border: 1px solid #1c2532;
                border-radius: 6px;
                color: #a5d6ff;
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 11px;
                line-height: 1.35;
                padding: 5px;
            }
        """)
        txt.setPlainText(initial_thought)
        layout.addWidget(txt)

        return card, badge, prog, lbl_metric, txt

    def _create_hud_chip(self, title: str, init_val: str) -> Tuple[QFrame, QLabel]:
        f = QFrame()
        f.setStyleSheet("background-color: #0d1117; border: 1px solid #21262d; border-radius: 5px; padding: 3px;")
        vl = QVBoxLayout(f)
        vl.setContentsMargins(8, 4, 8, 4)
        vl.setSpacing(2)
        lbl_t = QLabel(title)
        lbl_t.setStyleSheet("font-size: 10px; color: #8b949e; font-weight: bold; border: none;")
        lbl_v = QLabel(init_val)
        lbl_v.setStyleSheet("font-size: 12px; color: #58a6ff; font-weight: bold; font-family: Consolas; border: none;")
        vl.addWidget(lbl_t)
        vl.addWidget(lbl_v)
        return f, lbl_v

    def _send_cmd(self, cmd_dict: dict):
        try:
            self.cmd_sock.sendto(json.dumps(cmd_dict).encode("utf-8"), ("127.0.0.1", UDP_CMD_PORT))
        except Exception as e:
            print(f"[Studio GUI] Command send error: {e}")

    def _on_mode_changed(self, text: str):
        self._send_cmd({"cmd": "set_watchdog_mode", "mode": text})

    def _on_speed_changed(self, val: int):
        speed_mult = val / 100.0
        self.lbl_speed_val.setText(f"{speed_mult:.2f}x")
        self._send_cmd({"cmd": "set_speed", "val": speed_mult})

    def _on_reset_clicked(self):
        self._send_cmd({"cmd": "reset_robot"})

    def _on_toggle_front_pillar(self):
        self.front_pillar_active = not self.front_pillar_active
        self._update_front_button_style(self.front_pillar_active)
        self._send_cmd({"cmd": "toggle_front_pillar"})

    def _on_toggle_back_pillar(self):
        self.back_pillar_active = not self.back_pillar_active
        self._update_back_button_style(self.back_pillar_active)
        self._send_cmd({"cmd": "toggle_back_pillar"})

    def _update_front_button_style(self, active: bool):
        if active:
            self.btn_front.setText("🛑 Front Pillar: BLOCKED (Raised) [F]")
            self.btn_front.setStyleSheet("""
                QPushButton { background-color: #b91c1c; border: 1px solid #ef4444; border-radius: 5px; color: #ffffff; font-weight: bold; padding: 6px 12px; }
                QPushButton:hover { background-color: #dc2626; }
            """)
        else:
            self.btn_front.setText("✅ Front Pillar: CLEARED (Lowered) [F]")
            self.btn_front.setStyleSheet("""
                QPushButton { background-color: #065f46; border: 1px solid #10b981; border-radius: 5px; color: #ffffff; font-weight: bold; padding: 6px 12px; }
                QPushButton:hover { background-color: #047857; }
            """)

    def _update_back_button_style(self, active: bool):
        if active:
            self.btn_back.setText("🛑 Back Pillar: BLOCKED (Raised) [B]")
            self.btn_back.setStyleSheet("""
                QPushButton { background-color: #b91c1c; border: 1px solid #ef4444; border-radius: 5px; color: #ffffff; font-weight: bold; padding: 6px 12px; }
                QPushButton:hover { background-color: #dc2626; }
            """)
        else:
            self.btn_back.setText("✅ Back Pillar: CLEARED (Lowered) [B]")
            self.btn_back.setStyleSheet("""
                QPushButton { background-color: #065f46; border: 1px solid #10b981; border-radius: 5px; color: #ffffff; font-weight: bold; padding: 6px 12px; }
                QPushButton:hover { background-color: #047857; }
            """)

    def _udp_worker(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        bound = False
        for bind_ip in ["127.0.0.1", "0.0.0.0"]:
            try:
                sock.bind((bind_ip, UDP_RECV_PORT))
                sock.settimeout(0.5)
                bound = True
                print(f"[Studio UDP] Bound listening socket to {bind_ip}:{UDP_RECV_PORT}", flush=True)
                break
            except Exception as e:
                print(f"[Studio UDP] Note: Could not bind {bind_ip}:{UDP_RECV_PORT}: {e}", flush=True)

        if not bound:
            print(f"[Studio UDP] Critical: Failed to bind to port {UDP_RECV_PORT} on any interface.", flush=True)
            return

        while self.running:
            try:
                data, addr = sock.recvfrom(65535)
                pkt = json.loads(data.decode("utf-8"))
                with self._lock:
                    self.latest_pkt = pkt
                    self.last_packet_time = time.time()
                    self.udp_connected = True
                    self.packet_count += 1
                    if self.packet_count == 1:
                        print(f"[Studio UDP] Live telemetry stream connected from Webots! ({len(data)} bytes)", flush=True)
            except socket.timeout:
                if time.time() - self.last_packet_time > 1.5:
                    with self._lock:
                        self.udp_connected = False
            except Exception as e:
                print(f"[Studio UDP Error] {e}", flush=True)

    def closeEvent(self, event):
        self.running = False
        event.accept()

    def _udp_poll_and_render(self):
        pkt = None
        is_conn = False
        with self._lock:
            pkt = self.latest_pkt
            is_conn = self.udp_connected

        if is_conn:
            self.lbl_conn_status.setText(f"🟢 Webots Live Stream (60 Hz) [Pkts: {self.packet_count}]")
            self.lbl_conn_status.setStyleSheet("color: #00e676; font-weight: bold; font-size: 11px;")
        else:
            self.lbl_conn_status.setText("🔴 Waiting for Webots: Run test_wrong_way.wbt")
            self.lbl_conn_status.setStyleSheet("color: #ff5252; font-weight: bold; font-size: 11px;")

        if not pkt:
            return

        pkt = self.latest_pkt
        aww = pkt.get("aww", {})
        pose = pkt.get("pose", [0.0, 0.0, 0.0])
        action = aww.get("action", pkt.get("mode", "DRIVE"))

        # Update Pillar States from authoritative Webots telemetry
        if "front_pillar" in pkt:
            fp = bool(pkt["front_pillar"])
            if fp != self.front_pillar_active:
                self.front_pillar_active = fp
                self._update_front_button_style(fp)
        if "back_pillar" in pkt:
            bp = bool(pkt["back_pillar"])
            if bp != self.back_pillar_active:
                self.back_pillar_active = bp
                self._update_back_button_style(bp)

        # Update Watchdog Mode and Speed Slider if changed via keyboard in Webots
        if "watchdog_mode" in pkt:
            w_mode = str(pkt["watchdog_mode"])
            if self.cb_mode.currentText() != w_mode:
                idx = self.cb_mode.findText(w_mode)
                if idx >= 0:
                    self.cb_mode.blockSignals(True)
                    self.cb_mode.setCurrentIndex(idx)
                    self.cb_mode.blockSignals(False)

        if "sim_speed" in pkt:
            sp = float(pkt["sim_speed"])
            sp_val = int(round(sp * 100))
            if abs(self.slider_speed.value() - sp_val) > 2:
                self.slider_speed.blockSignals(True)
                self.slider_speed.setValue(sp_val)
                self.slider_speed.blockSignals(False)
                self.lbl_speed_val.setText(f"{sp:.2f}x")

        # Update 2D Radar Canvas
        self.canvas.set_telemetry(aww, pose, action, self.front_pillar_active, self.back_pillar_active)

        # Update Master Verdict
        is_ww = aww.get("is_wrong_way", False)
        det_by = aww.get("detected_by", "NONE")
        tgt_yaw = aww.get("target_yaw", 0.0)
        curr_yaw = pose[2] if len(pose) > 2 else 0.0
        yaw_err_deg = math.degrees(math.atan2(math.sin(tgt_yaw - curr_yaw), math.cos(tgt_yaw - curr_yaw)))

        if is_ww or action == "CORRECTIVE_180":
            self.lbl_verdict.setText(f"🚨 WRONG-WAY DETECTED BY {det_by}! EXECUTING 180° REVERSAL")
            self.lbl_verdict.setStyleSheet("font-size: 13px; font-weight: bold; color: #ff5252;")
            self.lbl_action.setText(f"Action: CORRECTIVE_180 | Yaw Error to Forward Flow: {yaw_err_deg:+.1f}° | Target Yaw: {math.degrees(tgt_yaw):+.1f}°")
            self.lbl_authority.setText("Steering Authority: MAXIMUM AUTHORITY (+-24.5° Full Reverse Arc Lock)")
            self.lbl_authority.setStyleSheet("font-size: 11px; color: #ffd166; font-weight: bold;")
            self.grp_master.setStyleSheet("QGroupBox { font-size: 12px; font-weight: bold; color: #ff5252; border: 2px solid #ff5252; border-radius: 6px; margin-top: 6px; padding-top: 10px; background-color: #2b1114; }")
            self.chip_status[1].setText(f"180° REVERSAL ({det_by})")
            self.chip_status[1].setStyleSheet("font-size: 12px; color: #ff5252; font-weight: bold; font-family: Consolas; border: none;")
        else:
            self.lbl_verdict.setText("🟢 ALL SYSTEMS CLEAR — FORWARD RACING")
            self.lbl_verdict.setStyleSheet("font-size: 13px; font-weight: bold; color: #00e676;")
            self.lbl_action.setText(f"Action: NORMAL_DRIVE | Heading: {math.degrees(curr_yaw):+.1f}° | Yaw Error: 0.0°")
            self.lbl_authority.setText("Steering Authority: STANDARD DYNAMIC PID LOCK (+-24.5°)")
            self.lbl_authority.setStyleSheet("font-size: 11px; color: #58a6ff;")
            self.grp_master.setStyleSheet("QGroupBox { font-size: 12px; font-weight: bold; color: #00e676; border: 1px solid #238636; border-radius: 6px; margin-top: 6px; padding-top: 10px; background-color: #161f28; }")
            self.chip_status[1].setText("ALL CLEAR (FORWARD)")
            self.chip_status[1].setStyleSheet("font-size: 12px; color: #00e676; font-weight: bold; font-family: Consolas; border: none;")

        # Update HUD Chips
        v_est = pkt.get("v_est", 0.0)
        v_cmd = pkt.get("v_cmd", 0.0)
        steer = pkt.get("steer_deg", 0.0)
        dist = pkt.get("dist_m", 0.0)
        lap = pkt.get("lap", 0)

        self.chip_speed[1].setText(f"{v_est:.2f} / {v_cmd:.2f} m/s")
        self.chip_steer[1].setText(f"{steer:+5.1f}°")
        self.chip_dist[1].setText(f"{dist:.1f} m (Lap {lap})")

        # Update Option 1 Card
        opt1 = aww.get("opt1", {})
        score1 = opt1.get("score", 0.0)
        is_tr1 = opt1.get("is_tripped", False)
        if self.prog_opt1:
            self.prog_opt1.setValue(int(score1 * 100))
        self.lbl_opt1_metric.setText(f"Trail Overlap Score: {score1*100:.0f}%  (Threshold: >= 70%)")
        if is_tr1:
            self.badge_opt1.setText("[ TRIPPED ] IN-LANE REVERSAL")
            self.badge_opt1.setStyleSheet("background-color: #381215; border: 1px solid #da3633; color: #ff5252; font-size: 11px; font-weight: bold; border-radius: 4px; padding: 3px 8px;")
            if self.prog_opt1:
                self.prog_opt1.setStyleSheet("QProgressBar { background-color: #080c10; border: 1px solid #252e3b; border-radius: 3px; text-align: center; color: #ffffff; font-size: 9px; font-weight: bold; } QProgressBar::chunk { background-color: #ff5252; border-radius: 2px; }")
        else:
            self.badge_opt1.setText("[ PASS ] FORWARD PATH CLEAR")
            self.badge_opt1.setStyleSheet("background-color: #0d2818; border: 1px solid #238636; color: #00e676; font-size: 11px; font-weight: bold; border-radius: 4px; padding: 3px 8px;")
            if self.prog_opt1:
                self.prog_opt1.setStyleSheet("QProgressBar { background-color: #080c10; border: 1px solid #252e3b; border-radius: 3px; text-align: center; color: #ffffff; font-size: 9px; font-weight: bold; } QProgressBar::chunk { background-color: #00e676; border-radius: 2px; }")
        self.txt_opt1.setPlainText(opt1.get("thought", ""))

        # Update Option 2 Card
        opt2 = aww.get("opt2", {})
        r_m = opt2.get("radius_m", 0.50)
        min_r = opt2.get("min_radius_m", 0.32)
        is_tr2 = opt2.get("is_tripped", False)
        self.lbl_opt2_metric.setText(f"Measured Turn Radius: R = {r_m:.2f}m  (Track Limit: {min_r:.2f}m)")
        if is_tr2:
            self.badge_opt2.setText("[ TRIPPED ] ILLEGAL TIGHT PIVOT")
            self.badge_opt2.setStyleSheet("background-color: #381215; border: 1px solid #da3633; color: #ff5252; font-size: 11px; font-weight: bold; border-radius: 4px; padding: 3px 8px;")
        else:
            self.badge_opt2.setText("[ PASS ] VALID RACING HAIRPIN ARC")
            self.badge_opt2.setStyleSheet("background-color: #0d2818; border: 1px solid #238636; color: #00e676; font-size: 11px; font-weight: bold; border-radius: 4px; padding: 3px 8px;")
        self.txt_opt2.setPlainText(opt2.get("thought", ""))

        # Update Option 3 Card
        opt3 = aww.get("opt3", {})
        dot = opt3.get("dot_product", 1.0)
        is_tr3 = opt3.get("is_tripped", False)
        self.lbl_opt3_metric.setText(f"Flow Alignment Dot Product: {dot:+.2f}  (Opposing <= -0.50)")
        if is_tr3:
            self.badge_opt3.setText("[ TRIPPED ] OPPOSING TRAFFIC FLOW")
            self.badge_opt3.setStyleSheet("background-color: #381215; border: 1px solid #da3633; color: #ff5252; font-size: 11px; font-weight: bold; border-radius: 4px; padding: 3px 8px;")
        else:
            self.badge_opt3.setText("[ PASS ] FORWARD FLOW ALIGNED")
            self.badge_opt3.setStyleSheet("background-color: #0d2818; border: 1px solid #238636; color: #00e676; font-size: 11px; font-weight: bold; border-radius: 4px; padding: 3px 8px;")
        self.txt_opt3.setPlainText(opt3.get("thought", ""))

        # Update Option 4 Card
        opt4 = aww.get("opt4", {})
        turn_deg = opt4.get("turn_deg", 0.0)
        lat_m = opt4.get("lateral_m", 0.0)
        min_lat = opt4.get("min_lateral_m", 0.40)
        is_tr4 = opt4.get("is_tripped", False)
        self.lbl_opt4_metric.setText(f"Turn: {turn_deg:.1f}° | Lateral Shift: {lat_m:.2f}m  (Min Lane Spacing: {min_lat:.2f}m)")
        if is_tr4:
            self.badge_opt4.setText("[ TRIPPED ] IN-LANE U-TURN (<0.40m)")
            self.badge_opt4.setStyleSheet("background-color: #381215; border: 1px solid #da3633; color: #ff5252; font-size: 11px; font-weight: bold; border-radius: 4px; padding: 3px 8px;")
        else:
            self.badge_opt4.setText("[ PASS ] CORRIDOR DISPLACEMENT CLEAR")
            self.badge_opt4.setStyleSheet("background-color: #0d2818; border: 1px solid #238636; color: #00e676; font-size: 11px; font-weight: bold; border-radius: 4px; padding: 3px 8px;")
        self.txt_opt4.setPlainText(opt4.get("thought", ""))


def launch_wrong_way_studio():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    gui = WrongWayStudioGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    launch_wrong_way_studio()
