#!/usr/bin/env python3
"""
===============================================================================
FOLKRACE — DUAL VL53L8CX 8-ZONE 90° TOP-DOWN 2D RADAR GUI
===============================================================================
Fused 8-Zone Top-Down 2D Field-of-View (FoV) Visualizer:
  - Fuses two parallel VL53L8CX sensors (one physically rotated 180°):
      * ToF 1 (Left):  Column 2, Row 4 is leftmost (Rows 4, 3, 2, 1 -> Zones 0..3)
      * ToF 2 (Right): Column 3, Row 4 is rightmost (Rows 1, 2, 3, 4 -> Zones 4..7)
  - 8 contiguous 11.25° sectors covering exactly 90° (-45° to +45° forward)
  - Max range: 3000 mm (3.0 m)
  - Proximity Blind-Spot & Open-Space Fallback:
      * Fallback occurs only when no target inside the zone reports anything else.
      * Active SPAD count == 1024 -> 0 mm (blind-spot contact / saturation).
      * Over 1024 / other -> 3000 mm (3.0 m open-space fallback).
"""

import sys
import os
import json
import time
import math
import threading
from collections import deque
import numpy as np
import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QFrame, QSizePolicy,
    QProgressBar, QToolTip, QSlider
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QPointF, QRectF
from PyQt5.QtGui import (
    QColor, QFont, QPainter, QBrush, QPen, QLinearGradient, QRadialGradient,
    QPolygonF, QPainterPath, QPixmap
)

if hasattr(Qt, 'HighDpiScaleFactorRoundingPolicy'):
    QApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)


# =============================================================================
# ASYNCHRONOUS DUAL-SENSOR SERIAL ENGINE
# =============================================================================

class DualSerialEngine(QThread):
    """
    Background worker thread for high-speed non-blocking UART telemetry.
    Maintains independent latest-frame slots for ToF 1 and ToF 2.
    """
    connection_changed = pyqtSignal(bool, str)
    ack_received = pyqtSignal(dict)
    config_received = pyqtSignal(dict)

    RX_BUFFER_BYTES = 1 << 20

    def __init__(self, port="COM7", baud=921600):
        super().__init__()
        self.port = port
        self.baud = baud
        self.ser = None
        self._running = False
        self._lock = threading.Lock()

        self.reset_stats()

    def reset_stats(self):
        with self._lock:
            self._latest_by_sensor = {1: None, 2: None}
            self._latest_arc_frame = None
            self.sensor_stats = {
                1: {"rx_packets": 0, "lost_packets": 0, "last_seq": None, "last_rx_time": 0.0, "current_dt_ms": 0.0, "timestamps": deque(maxlen=120), "smooth_hz": 0.0},
                2: {"rx_packets": 0, "lost_packets": 0, "last_seq": None, "last_rx_time": 0.0, "current_dt_ms": 0.0, "timestamps": deque(maxlen=120), "smooth_hz": 0.0}
            }

    def take_latest(self, sensor_id):
        with self._lock:
            pkt = self._latest_by_sensor.get(sensor_id)
            self._latest_by_sensor[sensor_id] = None
        return pkt

    def take_latest_arc(self):
        with self._lock:
            pkt = self._latest_arc_frame
            self._latest_arc_frame = None
        return pkt

    def is_sensor_active(self, sensor_id, timeout_s=1.2):
        with self._lock:
            st = self.sensor_stats.get(sensor_id)
            if not st or not st["last_rx_time"]:
                return False
            return (time.perf_counter() - st["last_rx_time"]) < timeout_s

    def get_sensor_hz(self, sensor_id):
        now = time.perf_counter()
        with self._lock:
            st = self.sensor_stats.get(sensor_id)
            if not st or not st["last_rx_time"]:
                return 0.0
            if (now - st["last_rx_time"]) > 1.2:
                st["smooth_hz"] = 0.0
                return 0.0

            dq = st.get("timestamps")
            if not dq:
                return 0.0

            # Prune timestamps older than 1.2s window
            cutoff = now - 1.2
            while dq and dq[0] < cutoff:
                dq.popleft()

            count = len(dq)
            if count < 2:
                return st.get("smooth_hz", 0.0)

            dt = dq[-1] - dq[0]
            if dt <= 0.001:
                return st.get("smooth_hz", 0.0)

            raw_hz = (count - 1) / dt

            # Low-pass EMA filter (0.7 * prev + 0.3 * raw) to eliminate residual USB buffer jitter
            prev_hz = st.get("smooth_hz", 0.0)
            if prev_hz <= 0.0:
                smooth_hz = raw_hz
            else:
                smooth_hz = 0.7 * prev_hz + 0.3 * raw_hz
            st["smooth_hz"] = smooth_hz
            return smooth_hz

    def _store_frame(self, packet):
        now = time.perf_counter()
        sensor_id = packet.get("id", 1)
        seq = packet.get("seq")
        with self._lock:
            if sensor_id not in self.sensor_stats:
                self.sensor_stats[sensor_id] = {
                    "rx_packets": 0, "lost_packets": 0, "last_seq": None, "last_rx_time": 0.0,
                    "current_dt_ms": 0.0, "timestamps": deque(maxlen=120), "smooth_hz": 0.0
                }
            st = self.sensor_stats[sensor_id]
            if st["last_rx_time"]:
                st["current_dt_ms"] = (now - st["last_rx_time"]) * 1000.0
            st["last_rx_time"] = now
            st["rx_packets"] += 1
            st.setdefault("timestamps", deque(maxlen=120)).append(now)
            if isinstance(seq, int):
                if st["last_seq"] is not None:
                    gap = seq - st["last_seq"]
                    if 1 < gap < 1000000:
                        st["lost_packets"] += gap - 1
                st["last_seq"] = seq
            self._latest_by_sensor[sensor_id] = packet

    def _store_arc_frame(self, packet):
        now = time.perf_counter()
        sensor_id = packet.get("id", 1)
        seq = packet.get("seq")
        with self._lock:
            # Update active stats for the sensor that triggered this arc frame
            if sensor_id not in self.sensor_stats:
                self.sensor_stats[sensor_id] = {
                    "rx_packets": 0, "lost_packets": 0, "last_seq": None, "last_rx_time": 0.0,
                    "current_dt_ms": 0.0, "timestamps": deque(maxlen=120), "smooth_hz": 0.0
                }
            st = self.sensor_stats[sensor_id]
            if st["last_rx_time"]:
                st["current_dt_ms"] = (now - st["last_rx_time"]) * 1000.0
            st["last_rx_time"] = now
            st["rx_packets"] += 1
            st.setdefault("timestamps", deque(maxlen=120)).append(now)
            if isinstance(seq, int):
                if st["last_seq"] is not None:
                    gap = seq - st["last_seq"]
                    if 1 < gap < 1000000:
                        st["lost_packets"] += gap - 1
                st["last_seq"] = seq
            self._latest_arc_frame = packet

    def run(self):
        try:
            self.ser = serial.Serial()
            self.ser.port = self.port
            self.ser.baudrate = self.baud
            self.ser.timeout = 0.1
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.open()
            try:
                self.ser.set_buffer_size(rx_size=self.RX_BUFFER_BYTES, tx_size=1 << 16)
            except Exception:
                pass
            self.reset_stats()
            self._running = True
            self.connection_changed.emit(True, f"Connected to {self.port} @ {self.baud}")
            self.send_command("SET_STREAM:arc")
            self.send_command("GET_CONFIG")
        except Exception as e:
            self._running = False
            self.connection_changed.emit(False, f"Connection Failed: {str(e)}")
            return

        buf = bytearray()
        while self._running:
            try:
                waiting = self.ser.in_waiting if (self.ser and self.ser.is_open) else 0
                if waiting:
                    raw = self.ser.read(waiting)
                    buf.extend(raw)
                    if len(buf) > 65536:
                        # Keep recent 8KB aligned to newline
                        nl_last = buf.rfind(b'\n', len(buf) - 8192)
                        if nl_last != -1:
                            del buf[:nl_last + 1]
                        else:
                            del buf[:-8192]
                    while True:
                        nl_pos = buf.find(b'\n')
                        if nl_pos == -1:
                            break
                        line_bytes = bytes(buf[:nl_pos]).strip()
                        del buf[:nl_pos + 1]
                        if line_bytes.startswith(b'{') and line_bytes.endswith(b'}'):
                            try:
                                packet = json.loads(line_bytes.decode('utf-8', errors='replace'))
                                ptype = packet.get("type", packet.get("t", ""))
                                if ptype == "A":
                                    self._store_arc_frame(packet)
                                elif ptype == "D":
                                    self._store_frame(packet)
                                elif ptype == "ACK":
                                    self.ack_received.emit(packet)
                                elif ptype == "CONFIG":
                                    self.config_received.emit(packet)
                            except Exception:
                                pass
                else:
                    time.sleep(0.002)
            except Exception as e:
                if self._running:
                    self.connection_changed.emit(False, f"Serial Error: {str(e)}")
                break

        if self.ser and self.ser.is_open:
            try: self.ser.close()
            except Exception: pass
        self.ser = None

    def send_command(self, cmd_str):
        if self.ser and self.ser.is_open and self._running:
            try:
                msg = (cmd_str + "\n").encode('utf-8')
                self.ser.write(msg)
                self.ser.flush()
            except Exception:
                pass

    def stop(self):
        self._running = False
        if self.ser and self.ser.is_open:
            try: self.ser.close()
            except Exception: pass
        self.wait(500)


# =============================================================================
# 8-ZONE 90° TOP-DOWN 2D RADAR CANVAS WIDGET
# =============================================================================

class RadarArcCanvas(QWidget):
    """
    Renders the 90° field-of-view (-45° to +45°) top-down 2D radar arc.
    8 sectors @ 11.25° each:
      - Sector 0 (-45.0° to -33.75°): ToF 1, Col 2, Row 4 (Leftmost)
      - Sector 1 (-33.75° to -22.5°):  ToF 1, Col 2, Row 3
      - Sector 2 (-22.5° to -11.25°):  ToF 1, Col 2, Row 2
      - Sector 3 (-11.25° to 0.0°):    ToF 1, Col 2, Row 1 (Center-Left)
      - Sector 4 (0.0° to +11.25°):    ToF 2, Col 3, Row 1 (Center-Right)
      - Sector 5 (+11.25° to +22.5°):  ToF 2, Col 3, Row 2
      - Sector 6 (+22.5° to +33.75°):  ToF 2, Col 3, Row 3
      - Sector 7 (+33.75° to +45.0°):  ToF 2, Col 3, Row 4 (Rightmost)
    """

    zone_hovered = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(420, 360)

        # Distances in mm, max 3000
        self.max_range_mm = 3000.0
        self.distances = [3000] * 8
        self.raw_distances = [None] * 8
        self.raw_statuses = [0] * 8
        self.is_blind_zero = [False] * 8
        self.is_clear_fallback = [False] * 8
        self.is_t2_fallback = [False] * 8
        self.is_t3_fallback = [False] * 8
        self.all_targets = [[] for _ in range(8)]
        self.spads = [0] * 8

        self.hovered_zone = -1
        self._bg_cache = None
        self.setMouseTracking(True)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._bg_cache = None

    def set_zone_data(self, dists, raw_dists, raw_stats, blind_zeros, clear_fallbacks, t2_fallbacks=None, t3_fallbacks=None, all_targets=None, spads=None):
        self.distances = list(dists)
        self.raw_distances = list(raw_dists)
        self.raw_statuses = list(raw_stats)
        self.is_blind_zero = list(blind_zeros)
        self.is_clear_fallback = list(clear_fallbacks)
        self.is_t2_fallback = list(t2_fallbacks) if t2_fallbacks else [False] * 8
        self.is_t3_fallback = list(t3_fallbacks) if t3_fallbacks else [False] * 8
        self.all_targets = list(all_targets) if all_targets else [[] for _ in range(8)]
        self.spads = list(spads) if spads else [0] * 8
        self.update()

    def mouseMoveEvent(self, event):
        w = self.width()
        h = self.height()
        origin_x = w / 2.0
        origin_y = h * 0.88

        dx = event.x() - origin_x
        dy = origin_y - event.y()

        dist_px = math.hypot(dx, dy)
        max_px = (origin_y - 25.0)

        if dist_px > 0 and dist_px <= max_px * 1.05 and dy > 0:
            angle_deg = math.degrees(math.atan2(dx, dy))
            if -45.0 <= angle_deg <= 45.0:
                z_idx = int((angle_deg + 45.0) / 11.25)
                z_idx = max(0, min(7, z_idx))
                if z_idx != self.hovered_zone:
                    self.hovered_zone = z_idx
                    self.zone_hovered.emit(z_idx)
                    sp = self.spads[z_idx] if z_idx < len(self.spads) else 0
                    d = self.distances[z_idx]
                    st = self.raw_statuses[z_idx]
                    status_str = "BLIND (0mm contact)" if self.is_blind_zero[z_idx] else ("OPEN (3.0m)" if self.is_clear_fallback[z_idx] else f"s{st}")
                    QToolTip.showText(event.globalPos(), f"Sector Z{z_idx}: {d} mm\nStatus: {status_str}\nSPADs: {sp}", self)
                    self.update()
                return

        if self.hovered_zone != -1:
            self.hovered_zone = -1
            self.zone_hovered.emit(-1)
            QToolTip.hideText()
            self.update()

    def leaveEvent(self, event):
        if self.hovered_zone != -1:
            self.hovered_zone = -1
            self.zone_hovered.emit(-1)
            QToolTip.hideText()
            self.update()

    def _build_bg_cache(self, w, h):
        pix = QPixmap(w, h)
        p = QPainter(pix)
        p.setRenderHint(QPainter.Antialiasing)

        # Canvas background gradient
        bg_grad = QLinearGradient(0, 0, 0, h)
        bg_grad.setColorAt(0.0, QColor(10, 14, 22))
        bg_grad.setColorAt(1.0, QColor(6, 8, 14))
        p.fillRect(0, 0, w, h, bg_grad)

        # Border
        p.setPen(QPen(QColor(28, 36, 52), 1))
        p.drawRect(0, 0, w - 1, h - 1)

        # Multi-target legend in top-left
        p.setFont(QFont("Segoe UI", 7, QFont.Bold))
        p.setBrush(QBrush(QColor("#00E5FF")))
        p.setPen(Qt.NoPen)
        p.drawEllipse(QPointF(18, 15), 3.5, 3.5)
        p.setPen(QColor(148, 163, 184))
        p.drawText(26, 19, "Primary (Closest)")

        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(QColor(245, 158, 11, 220), 1.6, Qt.SolidLine))
        p.drawEllipse(QPointF(130, 15), 4.0, 4.0)
        p.setPen(QColor(148, 163, 184))
        p.drawText(138, 19, "T2 Hit")

        p.setPen(QPen(QColor(168, 85, 247, 220), 1.6, Qt.DashLine))
        p.drawEllipse(QPointF(182, 15), 4.0, 4.0)
        p.setPen(QColor(148, 163, 184))
        p.drawText(190, 19, "T3 Hit")

        origin_x = w / 2.0
        origin_y = h * 0.88
        radius_max = min(origin_y - 25.0, (w / 2.0) / math.sin(math.radians(45.0)))
        scale = radius_max / self.max_range_mm

        def to_screen(deg, dist_mm):
            r_px = dist_mm * scale
            rad = math.radians(deg)
            sx = origin_x + r_px * math.sin(rad)
            sy = origin_y - r_px * math.cos(rad)
            return QPointF(sx, sy)

        # 1. 90° FOV BACKGROUND MESH & CONCENTRIC RANGE RINGS
        p_wedge = QPainterPath()
        p_wedge.moveTo(origin_x, origin_y)
        arc_rect = QRectF(origin_x - radius_max, origin_y - radius_max, radius_max * 2, radius_max * 2)
        p_wedge.arcTo(arc_rect, 45.0, 90.0)
        p_wedge.closeSubpath()

        wedge_grad = QRadialGradient(origin_x, origin_y, radius_max)
        wedge_grad.setColorAt(0.0, QColor(16, 26, 42, 190))
        wedge_grad.setColorAt(0.7, QColor(12, 18, 30, 220))
        wedge_grad.setColorAt(1.0, QColor(8, 12, 20, 240))
        p.fillPath(p_wedge, QBrush(wedge_grad))

        rings_m = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
        p.setFont(QFont("Consolas", 8, QFont.Bold))
        for r_m in rings_m:
            r_px = r_m * 1000.0 * scale
            ring_rect = QRectF(origin_x - r_px, origin_y - r_px, r_px * 2, r_px * 2)
            pen_ring = QPen(QColor(38, 54, 78, 130), 1, Qt.DashLine if r_m < 3.0 else Qt.SolidLine)
            p.setPen(pen_ring)
            p.drawArc(ring_rect, int(45.0 * 16), int(90.0 * 16))

            lbl_pt = to_screen(0.0, r_m * 1000.0)
            p.setPen(QColor(100, 130, 170, 200))
            p.drawText(int(lbl_pt.x() + 4), int(lbl_pt.y() + 11), f"{r_m:.1f}m")

        # 2. RADIAL SECTOR BOUNDARIES
        for i in range(9):
            ang = -45.0 + i * 11.25
            pt_end = to_screen(ang, self.max_range_mm)
            is_sensor_boundary = (abs(ang) < 0.01)
            is_edge = (i == 0 or i == 8)

            if is_edge:
                p.setPen(QPen(QColor(56, 189, 248, 180), 1.5))
            elif is_sensor_boundary:
                p.setPen(QPen(QColor(0, 229, 255, 230), 1.8))
            else:
                p.setPen(QPen(QColor(32, 44, 64, 150), 1, Qt.DotLine))

            p.drawLine(QPointF(origin_x, origin_y), pt_end)

            if i < 8:
                mid_ang = ang + 5.625
                tag_pt = to_screen(mid_ang, self.max_range_mm + 90.0)
                p.setFont(QFont("Segoe UI", 7, QFont.Bold))
                p.setPen(QColor(90, 115, 150))
                p.drawText(int(tag_pt.x() - 14), int(tag_pt.y() - 6), 28, 14, Qt.AlignCenter, f"Z{i}")

        # 3. SENSOR DOMAIN BANNER
        banner_y = int(origin_y + 12)
        p.setFont(QFont("Segoe UI", 8, QFont.Bold))
        p.setPen(QColor("#38BDF8"))
        p.drawText(int(origin_x - radius_max * 0.75), banner_y, 160, 18, Qt.AlignCenter, "◄ ToF 1 (Left / CS 10)")
        p.setPen(QColor("#34D399"))
        p.drawText(int(origin_x + radius_max * 0.75 - 160), banner_y, 160, 18, Qt.AlignCenter, "ToF 2 (Right / CS 9) ►")

        # 4. EGO-VEHICLE SILHOUETTE & NOSE BUMPER
        car_w = 48.0
        car_h = 36.0
        car_rect = QRectF(origin_x - car_w / 2.0, origin_y - 8.0, car_w, car_h)
        p.setBrush(QBrush(QColor(30, 41, 59)))
        p.setPen(QPen(QColor(56, 189, 248), 1.5))
        p.drawRoundedRect(car_rect, 6, 6)

        bumper_rect = QRectF(origin_x - car_w / 2.0 + 4, origin_y - 12.0, car_w - 8, 5)
        p.setBrush(QBrush(QColor(0, 229, 255)))
        p.setPen(Qt.NoPen)
        p.drawRoundedRect(bumper_rect, 2, 2)

        p.setFont(QFont("Segoe UI", 7, QFont.Bold))
        p.setPen(QColor(241, 245, 249))
        p.drawText(int(origin_x - 14), int(origin_y + 4), 28, 14, Qt.AlignCenter, "▲ FRONT")

        p.end()
        self._bg_cache = pix

    def paintEvent(self, event):
        w = self.width()
        h = self.height()
        if self._bg_cache is None or self._bg_cache.size() != self.size():
            self._build_bg_cache(w, h)

        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.drawPixmap(0, 0, self._bg_cache)

        origin_x = w / 2.0
        origin_y = h * 0.88
        radius_max = min(origin_y - 25.0, (w / 2.0) / math.sin(math.radians(45.0)))
        scale = radius_max / self.max_range_mm

        def to_screen(deg, dist_mm):
            r_px = dist_mm * scale
            rad = math.radians(deg)
            sx = origin_x + r_px * math.sin(rad)
            sy = origin_y - r_px * math.cos(rad)
            return QPointF(sx, sy)

        # ---------------------------------------------------------------------
        # 1. 8-ZONE FILLED DETECTION SECTOR WEDGES
        # ---------------------------------------------------------------------
        for z in range(8):
            ang_start = -45.0 + z * 11.25
            ang_end = ang_start + 11.25
            d = self.distances[z]
            is_blind = self.is_blind_zero[z]
            is_hover = (z == self.hovered_zone)

            if is_blind or d <= 50:
                col_sector = QColor(239, 68, 68, 190)
                col_border = QColor(255, 82, 82)
            elif d < 400:
                col_sector = QColor(249, 115, 22, 170)
                col_border = QColor(255, 138, 43)
            elif d < 1000:
                col_sector = QColor(245, 158, 11, 150)
                col_border = QColor(255, 179, 0)
            elif d < 2200:
                col_sector = QColor(6, 182, 212, 120)
                col_border = QColor(0, 229, 255)
            else:
                col_sector = QColor(16, 185, 129, 90)
                col_border = QColor(0, 230, 118)

            if is_hover:
                col_sector = col_sector.lighter(140)

            wedge_path = QPainterPath()
            wedge_path.moveTo(origin_x, origin_y)

            d_clamped = max(30.0, min(self.max_range_mm, d))
            r_z_px = d_clamped * scale
            z_rect = QRectF(origin_x - r_z_px, origin_y - r_z_px, r_z_px * 2, r_z_px * 2)

            qt_start = 90.0 - ang_end
            wedge_path.arcTo(z_rect, qt_start, 11.25)
            wedge_path.closeSubpath()

            p.fillPath(wedge_path, QBrush(col_sector))
            p.setPen(QPen(col_border, 2.5 if is_hover else 1.8))
            p.drawArc(z_rect, int(qt_start * 16), int(11.25 * 16))

        # ---------------------------------------------------------------------
        # 2. CONTOUR ENVELOPE POLYLINE
        # ---------------------------------------------------------------------
        contour_poly = QPolygonF()
        contour_poly.append(to_screen(-45.0, self.distances[0]))
        for z in range(8):
            mid_ang = -45.0 + z * 11.25 + 5.625
            contour_poly.append(to_screen(mid_ang, self.distances[z]))
        contour_poly.append(to_screen(45.0, self.distances[7]))

        p.setPen(QPen(QColor(255, 255, 255, 180), 2.0, Qt.SolidLine))
        p.drawPolyline(contour_poly)

        # ---------------------------------------------------------------------
        # 3. GLOWING TARGET POINTS AND MULTI-TARGET MARKERS
        # ---------------------------------------------------------------------
        for z in range(8):
            mid_ang = -45.0 + z * 11.25 + 5.625
            d = self.distances[z]
            tip_pt = to_screen(mid_ang, max(30.0, d))
            is_blind = self.is_blind_zero[z]
            is_hover = (z == self.hovered_zone)

            if not is_blind and z < len(self.all_targets):
                for (tgt_d, tgt_st, tgt_num) in self.all_targets[z]:
                    if abs(tgt_d - d) > 30:
                        sec_pt = to_screen(mid_ang, max(30.0, min(self.max_range_mm, tgt_d)))
                        if tgt_num == 1:
                            sec_col = QColor(56, 189, 248, 220)
                            line_style = Qt.SolidLine
                        elif tgt_num == 2:
                            sec_col = QColor(245, 158, 11, 220)
                            line_style = Qt.SolidLine
                        else:
                            sec_col = QColor(168, 85, 247, 220)
                            line_style = Qt.DashLine
                        p.setBrush(Qt.NoBrush)
                        p.setPen(QPen(sec_col, 1.8, line_style))
                        p.drawEllipse(sec_pt, 4.5, 4.5)

            pt_col = QColor("#EF4444") if is_blind else (QColor("#00E5FF") if d < 2200 else QColor("#00E676"))
            if is_hover:
                p.setBrush(QBrush(QColor(255, 255, 255)))
                p.setPen(QPen(pt_col, 2.5))
                p.drawEllipse(tip_pt, 5.0, 5.0)
            else:
                p.setBrush(QBrush(pt_col))
                p.setPen(QPen(QColor(10, 14, 22), 1.5))
                p.drawEllipse(tip_pt, 3.8, 3.8)

            p.setFont(QFont("Consolas", 8, QFont.Bold))
            txt_d = f"{d:.0f}" if d > 0 else "0"
            if self.is_clear_fallback[z]:
                txt_d = "3.0m*"
            elif is_blind:
                txt_d = "0mm!"
            elif self.is_t2_fallback[z]:
                txt_d = f"{d:.0f} (T2)"
            elif self.is_t3_fallback[z]:
                txt_d = f"{d:.0f} (T3)"

            txt_pt = to_screen(mid_ang, max(80.0, min(self.max_range_mm - 140.0, d - 40.0)))
            p.setPen(QColor(255, 255, 255, 230))
            p.drawText(int(txt_pt.x() - 22), int(txt_pt.y() - 7), 44, 14, Qt.AlignCenter, txt_d)


# =============================================================================
# MAIN WINDOW: 8-ZONE 90° RADAR TELEMETRY SUITE
# =============================================================================

class ToFArcRadarApp(QMainWindow):
    """
    Focused, lightweight 8-Zone 90° ToF Radar Telemetry Suite.
    Direct serial acquisition from physical ESP32 dual VL53L8CX rig.
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("FOLKRACE — VL53L8CX 8-Zone 90° Radar Telemetry Suite")
        self.resize(1080, 750)
        self.setMinimumSize(860, 620)
        self.setStyleSheet("background-color: #0b0f19; color: #f1f5f9; font-family: 'Segoe UI', sans-serif;")

        # Multi-target data matrices (16 zones, up to 3 targets each)
        self.tof1_dists = np.zeros((16, 3), dtype=int)
        self.tof1_stats = np.zeros((16, 3), dtype=int)
        self.tof1_spads = np.zeros(16, dtype=int)
        self.tof2_dists = np.zeros((16, 3), dtype=int)
        self.tof2_stats = np.zeros((16, 3), dtype=int)
        self.tof2_spads = np.zeros(16, dtype=int)

        # Processed 8 zones
        self.zone_dists = [3000] * 8
        self.raw_dists = [None] * 8
        self.raw_stats = [0] * 8
        self.zone_spads = [0] * 8
        self.is_t2_fallback = [False] * 8
        self.is_t3_fallback = [False] * 8
        self.is_blind_zero = [False] * 8
        self.is_clear_fallback = [False] * 8
        self.all_targets = [[] for _ in range(8)]

        # 2-Frame error latch (hold last good distance for 1 frame before fallback)
        self.error_frame_counts = [0] * 8
        self.last_valid_dists = [3000] * 8

        # UI styling cache: prevents 960 CSS re-parses and widget layout stalls per second
        self.card_alert_state = [-1] * 8
        self.card_last_dist_text = [""] * 8
        self.card_last_tag_text = [""] * 8
        self.card_last_pbar_val = [-1] * 8
        self.last_kpi_text = ""
        self.last_kpi_color = ""

        self.STYLE_PROGRESS = {
            0: "QProgressBar { background: #1e1b2e; border-radius: 2px; } QProgressBar::chunk { background: #EF4444; border-radius: 2px; }",
            1: "QProgressBar { background: #0f172a; border-radius: 2px; } QProgressBar::chunk { background: #F97316; border-radius: 2px; }",
            2: "QProgressBar { background: #0f172a; border-radius: 2px; } QProgressBar::chunk { background: #F59E0B; border-radius: 2px; }",
            3: "QProgressBar { background: #0f172a; border-radius: 2px; } QProgressBar::chunk { background: #00E5FF; border-radius: 2px; }",
            4: "QProgressBar { background: #0f172a; border-radius: 2px; } QProgressBar::chunk { background: #10B981; border-radius: 2px; }",
        }
        self.STYLE_DIST = {
            0: "color: #EF4444; font-weight: bold;",
            1: "color: #F97316; font-weight: bold;",
            2: "color: #F59E0B; font-weight: bold;",
            3: "color: #00E5FF; font-weight: bold;",
            4: "color: #10B981; font-weight: bold;",
        }

        # Watchdog auto-reset tracking (allows up to 2 attempts with 5.0s grace period)
        self.last_auto_reset_time = 0.0
        self.reset_grace_until = 0.0
        self.reset_status_until = 0.0
        self.auto_reset_count = 0
        self.reset_failed = False

        # Multi-target configuration (Order, Targets depth, Sharpener)
        self.target_order = 1       # 1: Closest, 2: Strongest
        self.max_targets = 3        # 2 or 3 targets
        self.sharpener_percent = 1  # 1% to 10%
        self.config_debounce_timer = QTimer(self)
        self.config_debounce_timer.setSingleShot(True)
        self.config_debounce_timer.timeout.connect(self.apply_config_to_sensor)

        self.serial_engine = DualSerialEngine()
        self.serial_engine.connection_changed.connect(self.on_connection_changed)
        self.serial_engine.ack_received.connect(self.on_ack_received)
        self.serial_engine.config_received.connect(self.on_config_received)

        self._build_ui()

        # UI refresh timer @ 60 Hz
        self.render_timer = QTimer(self)
        self.render_timer.timeout.connect(self.on_render_tick)
        self.render_timer.start(16)

        # Diagnostics & FPS update timer @ 2 Hz
        self.diag_timer = QTimer(self)
        self.diag_timer.timeout.connect(self.on_diag_tick)
        self.diag_timer.start(500)

        self.refresh_ports()

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(8)

        # ---------------------------------------------------------------------
        # TOP HEADER & SERIAL CONNECTION BAR
        # ---------------------------------------------------------------------
        top_bar = QFrame()
        top_bar.setStyleSheet("background-color: #111827; border: 1px solid #1f2937; border-radius: 8px; padding: 4px;")
        top_layout = QHBoxLayout(top_bar)
        top_layout.setContentsMargins(8, 4, 8, 4)
        top_layout.setSpacing(10)

        # Title / Brand
        lbl_brand = QLabel("🏹 TOF 8-ZONE 90° RADAR")
        lbl_brand.setFont(QFont("Segoe UI", 10, QFont.Bold))
        lbl_brand.setStyleSheet("color: #00E5FF; letter-spacing: 0.5px;")
        top_layout.addWidget(lbl_brand)

        top_layout.addStretch()

        # Dual Sensor Status Badges
        self.badge_tof1 = QLabel("ToF 1 (Left): -- Hz")
        self.badge_tof1.setStyleSheet("background-color: #1e293b; color: #38bdf8; font-weight: bold; padding: 3px 8px; border-radius: 4px; font-size: 8pt;")
        top_layout.addWidget(self.badge_tof1)

        self.badge_tof2 = QLabel("ToF 2 (Right): -- Hz")
        self.badge_tof2.setStyleSheet("background-color: #1e293b; color: #34d399; font-weight: bold; padding: 3px 8px; border-radius: 4px; font-size: 8pt;")
        top_layout.addWidget(self.badge_tof2)

        # Serial Port Selector
        top_layout.addWidget(QLabel("Port:"))
        self.combo_port = QComboBox()
        self.combo_port.setMinimumWidth(80)
        self.combo_port.setFixedHeight(26)
        top_layout.addWidget(self.combo_port)

        btn_refresh_ports = QPushButton("🔄")
        btn_refresh_ports.setFixedSize(26, 26)
        btn_refresh_ports.setToolTip("Refresh COM ports")
        btn_refresh_ports.clicked.connect(self.refresh_ports)
        top_layout.addWidget(btn_refresh_ports)

        top_layout.addWidget(QLabel("Baud:"))
        self.combo_baud = QComboBox()
        self.combo_baud.addItems(["921600", "460800", "115200"])
        self.combo_baud.setFixedHeight(26)
        top_layout.addWidget(self.combo_baud)

        self.btn_connect = QPushButton("🔌 Connect")
        self.btn_connect.setFixedHeight(26)
        self.btn_connect.setStyleSheet("background-color: #0284c7; color: white; font-weight: bold; border-radius: 4px; padding: 2px 12px;")
        self.btn_connect.clicked.connect(self.toggle_connection)
        top_layout.addWidget(self.btn_connect)

        self.btn_reset = QPushButton("🔄 Reset Sensors")
        self.btn_reset.setFixedHeight(26)
        self.btn_reset.setEnabled(False)
        self.btn_reset.setCursor(Qt.PointingHandCursor)
        self.btn_reset.setStyleSheet(
            "QPushButton { background-color: #374151; color: #f1f5f9; font-weight: bold; border-radius: 4px; padding: 2px 10px; border: 1px solid #4b5563; } "
            "QPushButton:hover { background-color: #4b5563; } "
            "QPushButton:disabled { background-color: #1f2937; color: #64748b; border-color: #374151; }"
        )
        self.btn_reset.setToolTip("Dispatch hardware RESET command to re-initialize both VL53L8CX sensors")
        self.btn_reset.clicked.connect(self.manual_reset_sensors)
        top_layout.addWidget(self.btn_reset)

        self.status_pill = QLabel("DISCONNECTED")
        self.status_pill.setFixedHeight(26)
        self.status_pill.setStyleSheet("background-color: #374151; color: #cbd5e1; font-weight: bold; padding: 2px 8px; border-radius: 4px; font-size: 8pt;")
        top_layout.addWidget(self.status_pill)

        main_layout.addWidget(top_bar, 0)

        # ---------------------------------------------------------------------
        # SENSOR CONFIGURATION & TUNING TOOLBAR
        # ---------------------------------------------------------------------
        self.cfg_bar = QFrame(central)
        self.cfg_bar.setStyleSheet("background-color: #111827; border: 1px solid #1f2937; border-radius: 8px; padding: 4px;")
        cfg_layout = QHBoxLayout(self.cfg_bar)
        cfg_layout.setContentsMargins(8, 4, 8, 4)
        cfg_layout.setSpacing(10)

        lbl_tuning = QLabel("⚙️ TUNING:", self.cfg_bar)
        lbl_tuning.setFont(QFont("Segoe UI", 8, QFont.Bold))
        lbl_tuning.setStyleSheet("color: #94a3b8; letter-spacing: 0.5px;")
        cfg_layout.addWidget(lbl_tuning)

        # 1. Target Order Switch Button (Closest vs Strongest)
        self.btn_order = QPushButton(self.cfg_bar)
        self.btn_order.setFixedHeight(26)
        self.btn_order.setCursor(Qt.PointingHandCursor)
        self.btn_order.setToolTip("Switch VL53L8CX target order between Closest (1) and Strongest (2)")
        self.btn_order.clicked.connect(self.toggle_target_order)
        self._update_order_button_style()
        cfg_layout.addWidget(self.btn_order)

        # 2. Target Count Switch Button (2 vs 3 Targets)
        self.btn_targets = QPushButton(self.cfg_bar)
        self.btn_targets.setFixedHeight(26)
        self.btn_targets.setCursor(Qt.PointingHandCursor)
        self.btn_targets.setToolTip("Toggle multi-target fallback depth between 2 targets and 3 targets")
        self.btn_targets.clicked.connect(self.toggle_max_targets)
        self._update_targets_button_style()
        cfg_layout.addWidget(self.btn_targets)

        # Separator
        sep = QFrame(self.cfg_bar)
        sep.setFrameShape(QFrame.VLine)
        sep.setFrameShadow(QFrame.Sunken)
        sep.setStyleSheet("color: #334155; margin: 2px 4px;")
        cfg_layout.addWidget(sep)

        # 3. Veiling Glare Sharpener Slider (1-10%)
        self.lbl_sharp = QLabel("✨ Sharpener: 1%", self.cfg_bar)
        self.lbl_sharp.setFont(QFont("Segoe UI", 8, QFont.Bold))
        self.lbl_sharp.setStyleSheet("color: #e2e8f0; min-width: 95px;")
        cfg_layout.addWidget(self.lbl_sharp)

        self.slider_sharp = QSlider(Qt.Horizontal, self.cfg_bar)
        self.slider_sharp.setRange(1, 10)
        self.slider_sharp.setValue(1)
        self.slider_sharp.setFixedWidth(110)
        self.slider_sharp.setFixedHeight(22)
        self.slider_sharp.setCursor(Qt.PointingHandCursor)
        self.slider_sharp.setToolTip("Veiling glare sharpener percentage (1% - 10%, default: 1%)")
        self.slider_sharp.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 4px;
                background: #1e293b;
                border-radius: 2px;
            }
            QSlider::sub-page:horizontal {
                background: #00E5FF;
                border-radius: 2px;
            }
            QSlider::handle:horizontal {
                background: #f8fafc;
                border: 2px solid #00E5FF;
                width: 14px;
                margin-top: -5px;
                margin-bottom: -5px;
                border-radius: 7px;
            }
            QSlider::handle:horizontal:hover {
                background: #00E5FF;
                border-color: #ffffff;
            }
        """)
        self.slider_sharp.valueChanged.connect(self.on_sharpener_changed)
        self.slider_sharp.sliderReleased.connect(self.apply_config_to_sensor)
        cfg_layout.addWidget(self.slider_sharp)

        cfg_layout.addStretch()

        # Status / Feedback label for sensor reconfiguration
        self.lbl_config_status = QLabel("", self.cfg_bar)
        self.lbl_config_status.setFont(QFont("Segoe UI", 8, QFont.Bold))
        self.lbl_config_status.setStyleSheet("color: #94a3b8;")
        cfg_layout.addWidget(self.lbl_config_status)

        main_layout.addWidget(self.cfg_bar, 0)

        # ---------------------------------------------------------------------
        # CENTER: RADAR CANVAS
        # ---------------------------------------------------------------------
        self.radar_canvas = RadarArcCanvas()
        self.radar_canvas.zone_hovered.connect(self.on_zone_hovered)
        main_layout.addWidget(self.radar_canvas, 1)

        # ---------------------------------------------------------------------
        # BOTTOM: 8-ZONE METRIC TELEMETRY CARDS
        # ---------------------------------------------------------------------
        cards_container = QFrame()
        cards_container.setStyleSheet("background-color: #111827; border: 1px solid #1f2937; border-radius: 8px;")
        cards_layout = QHBoxLayout(cards_container)
        cards_layout.setContentsMargins(6, 6, 6, 6)
        cards_layout.setSpacing(6)

        # Zone metadata definition:
        # Zone 0..3: ToF 1 (Left), Column 2 (0-indexed col 1), Rows 4..1 (0-indexed rows 3, 2, 1, 0)
        # Zone 4..7: ToF 2 (Right), Column 3 (0-indexed col 2), Rows 1..4 (0-indexed rows 0, 1, 2, 3)
        self.zone_meta = [
            {"id": 0, "name": "Z0 (Far L)",   "sensor": "ToF 1", "hw": "Col 2, Row 4", "ang": "-45.0° .. -33.8°"},
            {"id": 1, "name": "Z1 (Mid L)",   "sensor": "ToF 1", "hw": "Col 2, Row 3", "ang": "-33.8° .. -22.5°"},
            {"id": 2, "name": "Z2 (Inner L)", "sensor": "ToF 1", "hw": "Col 2, Row 2", "ang": "-22.5° .. -11.2°"},
            {"id": 3, "name": "Z3 (Front L)", "sensor": "ToF 1", "hw": "Col 2, Row 1", "ang": "-11.2° ..   0.0°"},
            {"id": 4, "name": "Z4 (Front R)", "sensor": "ToF 2", "hw": "Col 3, Row 1", "ang": "  0.0° .. +11.2°"},
            {"id": 5, "name": "Z5 (Inner R)", "sensor": "ToF 2", "hw": "Col 3, Row 2", "ang": "+11.2° .. +22.5°"},
            {"id": 6, "name": "Z6 (Mid R)",   "sensor": "ToF 2", "hw": "Col 3, Row 3", "ang": "+22.5° .. +33.8°"},
            {"id": 7, "name": "Z7 (Far R)",   "sensor": "ToF 2", "hw": "Col 3, Row 4", "ang": "+33.8° .. +45.0°"},
        ]

        self.card_widgets = []
        for meta in self.zone_meta:
            card = QFrame()
            card.setStyleSheet("background-color: #1a2234; border: 1px solid #283548; border-radius: 6px; padding: 2px;")
            c_layout = QVBoxLayout(card)
            c_layout.setContentsMargins(4, 4, 4, 4)
            c_layout.setSpacing(2)

            lbl_zname = QLabel(meta["name"])
            lbl_zname.setFont(QFont("Segoe UI", 8, QFont.Bold))
            lbl_zname.setStyleSheet("color: #38bdf8;" if "ToF 1" in meta["sensor"] else "color: #34d399;")
            lbl_zname.setAlignment(Qt.AlignCenter)
            c_layout.addWidget(lbl_zname)

            lbl_dist = QLabel("3000 mm")
            lbl_dist.setFont(QFont("Consolas", 11, QFont.Bold))
            lbl_dist.setStyleSheet("color: #f8fafc;")
            lbl_dist.setAlignment(Qt.AlignCenter)
            c_layout.addWidget(lbl_dist)

            pbar = QProgressBar()
            pbar.setFixedHeight(5)
            pbar.setTextVisible(False)
            pbar.setRange(0, 3000)
            pbar.setValue(3000)
            pbar.setStyleSheet("""
                QProgressBar { background-color: #0f172a; border-radius: 2px; }
                QProgressBar::chunk { background-color: #00E676; border-radius: 2px; }
            """)
            c_layout.addWidget(pbar)

            lbl_sub = QLabel(meta["ang"])
            lbl_sub.setFont(QFont("Segoe UI", 7))
            lbl_sub.setStyleSheet("color: #94a3b8;")
            lbl_sub.setAlignment(Qt.AlignCenter)
            c_layout.addWidget(lbl_sub)

            lbl_tag = QLabel(meta["sensor"] + " • " + meta["hw"])
            lbl_tag.setFont(QFont("Segoe UI", 7))
            lbl_tag.setStyleSheet("color: #64748b;")
            lbl_tag.setAlignment(Qt.AlignCenter)
            c_layout.addWidget(lbl_tag)

            cards_layout.addWidget(card, 1)
            self.card_widgets.append({
                "frame": card,
                "lbl_dist": lbl_dist,
                "pbar": pbar,
                "lbl_sub": lbl_sub,
                "lbl_tag": lbl_tag,
                "meta": meta
            })

        main_layout.addWidget(cards_container, 0)

        # ---------------------------------------------------------------------
        # FOOTER KPI & BLIND SPOT HEURISTIC BANNER
        # ---------------------------------------------------------------------
        foot_bar = QHBoxLayout()
        foot_bar.setContentsMargins(4, 0, 4, 0)

        order_str = "Closest" if self.target_order == 1 else "Strongest"
        self.lbl_rule_info = QLabel(f"🛡️ Fallback (0 targets): 1024 SPADs = 0mm | Over 1024 / Other = 3m | ToF DSP: {order_str} | Depth: {self.max_targets} Tgt | Sharpener: {self.sharpener_percent}%")
        self.lbl_rule_info.setStyleSheet("color: #60a5fa; font-size: 7.5pt; font-style: italic;")
        foot_bar.addWidget(self.lbl_rule_info)

        foot_bar.addStretch()

        self.lbl_kpi_min = QLabel("Min Distance: --- mm")
        self.lbl_kpi_min.setStyleSheet("color: #00E5FF; font-weight: bold; font-size: 8pt;")
        foot_bar.addWidget(self.lbl_kpi_min)

        main_layout.addLayout(foot_bar)

    def _update_order_button_style(self):
        if self.target_order == 1:
            self.btn_order.setText("🎯 ToF DSP: Closest")
            self.btn_order.setStyleSheet("""
                QPushButton {
                    background-color: #0c2838;
                    color: #38bdf8;
                    border: 1px solid #0284c7;
                    border-radius: 4px;
                    font-weight: bold;
                    padding: 3px 12px;
                    font-size: 8pt;
                }
                QPushButton:hover {
                    background-color: #0369a1;
                    color: #ffffff;
                }
            """)
        else:
            self.btn_order.setText("💪 ToF DSP: Strongest")
            self.btn_order.setStyleSheet("""
                QPushButton {
                    background-color: #2c1a08;
                    color: #fbbf24;
                    border: 1px solid #d97706;
                    border-radius: 4px;
                    font-weight: bold;
                    padding: 3px 12px;
                    font-size: 8pt;
                }
                QPushButton:hover {
                    background-color: #b45309;
                    color: #ffffff;
                }
            """)

    def _update_targets_button_style(self):
        if self.max_targets == 3:
            self.btn_targets.setText("🔢 Targets: 3 (T1, T2, T3)")
            self.btn_targets.setStyleSheet("""
                QPushButton {
                    background-color: #241432;
                    color: #c084fc;
                    border: 1px solid #9333ea;
                    border-radius: 4px;
                    font-weight: bold;
                    padding: 3px 12px;
                    font-size: 8pt;
                }
                QPushButton:hover {
                    background-color: #7e22ce;
                    color: #ffffff;
                }
            """)
        else:
            self.btn_targets.setText("🔢 Targets: 2 (T1, T2)")
            self.btn_targets.setStyleSheet("""
                QPushButton {
                    background-color: #062b20;
                    color: #34d399;
                    border: 1px solid #059669;
                    border-radius: 4px;
                    font-weight: bold;
                    padding: 3px 12px;
                    font-size: 8pt;
                }
                QPushButton:hover {
                    background-color: #047857;
                    color: #ffffff;
                }
            """)

    def toggle_target_order(self):
        self.target_order = 2 if self.target_order == 1 else 1
        self._update_order_button_style()
        self.queue_config_apply()

    def toggle_max_targets(self):
        self.max_targets = 2 if self.max_targets == 3 else 3
        self._update_targets_button_style()
        self.queue_config_apply()

    def on_sharpener_changed(self, val):
        self.sharpener_percent = val
        self.lbl_sharp.setText(f"✨ Sharpener: {val}%")
        self.queue_config_apply()

    def queue_config_apply(self):
        self._update_rule_info()
        self.config_debounce_timer.start(500)

    def apply_config_to_sensor(self):
        self.config_debounce_timer.stop()
        self._update_rule_info()
        if self.serial_engine.isRunning():
            now = time.perf_counter()
            self.reset_grace_until = now + 4.5
            self.reset_status_until = now + 2.5
            order_name = "Closest" if self.target_order == 1 else "Strongest"
            self.lbl_config_status.setText(f"⏳ ToF resetting ({order_name}, {self.max_targets} Tgt, {self.sharpener_percent}%)...")
            self.lbl_config_status.setStyleSheet("color: #f59e0b; font-weight: bold; font-size: 8pt;")
            cmd = f"SET_CONFIG:order={self.target_order},sharp={self.sharpener_percent},targets={self.max_targets}"
            self.serial_engine.send_command(cmd)
        else:
            self.lbl_config_status.setText("⚙️ Settings saved (connect to apply)")
            self.lbl_config_status.setStyleSheet("color: #94a3b8; font-size: 8pt;")

    def _update_rule_info(self):
        order_str = "Closest" if self.target_order == 1 else "Strongest"
        self.lbl_rule_info.setText(
            f"🛡️ Fallback (0 targets): 1024 SPADs = 0mm | Over 1024 / Other = 3m | ToF DSP: {order_str} | Depth: {self.max_targets} Tgt | Sharpener: {self.sharpener_percent}%"
        )

    def on_ack_received(self, pkt):
        try:
            cmd = pkt.get("cmd", "")
            if cmd == "SET_CONFIG":
                order_name = "Closest" if self.target_order == 1 else "Strongest"
                if hasattr(self, 'lbl_config_status'):
                    self.lbl_config_status.setText(f"✅ Applied ({order_name} | {self.max_targets} Tgt | Sharp {self.sharpener_percent}%)")
                    self.lbl_config_status.setStyleSheet("color: #34d399; font-weight: bold; font-size: 8pt;")
        except Exception as e:
            pass

    def on_config_received(self, pkt):
        try:
            order = pkt.get("order")
            if order in (1, 2):
                self.target_order = order
                if hasattr(self, 'btn_order'):
                    self._update_order_button_style()
            sharp = pkt.get("sharp")
            if sharp is not None and hasattr(self, 'slider_sharp'):
                clamped_sharp = max(1, min(10, sharp))
                self.sharpener_percent = clamped_sharp
                self.slider_sharp.blockSignals(True)
                self.slider_sharp.setValue(clamped_sharp)
                self.slider_sharp.blockSignals(False)
                if hasattr(self, 'lbl_sharp'):
                    self.lbl_sharp.setText(f"✨ Sharpener: {clamped_sharp}%")
            tgts = pkt.get("targets")
            if tgts in (2, 3):
                self.max_targets = tgts
                if hasattr(self, 'btn_targets'):
                    self._update_targets_button_style()
            if hasattr(self, 'lbl_rule_info'):
                self._update_rule_info()
            if hasattr(self, 'lbl_config_status'):
                self.lbl_config_status.setText("✅ Hardware Synced")
                self.lbl_config_status.setStyleSheet("color: #38bdf8; font-weight: bold; font-size: 8pt;")
        except Exception as e:
            pass

    def refresh_ports(self):
        self.combo_port.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_port.addItem(p.device)
        # Prefer COM7 if present
        idx = self.combo_port.findText("COM7")
        if idx >= 0:
            self.combo_port.setCurrentIndex(idx)

    def manual_reset_sensors(self):
        if not self.serial_engine.isRunning():
            return
        now = time.perf_counter()
        self.auto_reset_count = 0
        self.reset_failed = False
        self.reset_grace_until = now + 5.0
        self.reset_status_until = now + 5.0
        self.status_pill.setText("RESETTING...")
        self.status_pill.setStyleSheet("background-color: #b45309; color: #fef3c7; font-weight: bold; padding: 2px 8px; border-radius: 4px; font-size: 8pt;")
        self.serial_engine.reset_stats()
        self.serial_engine.send_command("RESET")

    def toggle_connection(self):
        if self.serial_engine.isRunning():
            self.serial_engine.stop()
            self.btn_connect.setText("🔌 Connect")
            self.btn_connect.setStyleSheet("background-color: #0284c7; color: white; font-weight: bold; border-radius: 4px; padding: 2px 12px;")
            self.status_pill.setText("DISCONNECTED")
            self.status_pill.setStyleSheet("background-color: #374151; color: #cbd5e1; font-weight: bold; padding: 2px 8px; border-radius: 4px; font-size: 8pt;")
            if hasattr(self, "btn_reset"):
                self.btn_reset.setEnabled(False)
        else:
            port = self.combo_port.currentText()
            baud = int(self.combo_baud.currentText())
            if not port:
                return
            now = time.perf_counter()
            self.auto_reset_count = 0
            self.reset_failed = False
            self.reset_grace_until = now + 3.0  # 3.0s startup grace period for telemetry stream
            self.reset_status_until = 0.0
            self.serial_engine.reset_stats()
            self.serial_engine.port = port
            self.serial_engine.baud = baud
            self.serial_engine.start()
            self.btn_connect.setText("🛑 Disconnect")
            self.btn_connect.setStyleSheet("background-color: #dc2626; color: white; font-weight: bold; border-radius: 4px; padding: 2px 12px;")

    def on_connection_changed(self, connected, msg):
        if connected:
            now = time.perf_counter()
            self.reset_grace_until = now + 3.0
            self.status_pill.setText("ONLINE")
            self.status_pill.setStyleSheet("background-color: #065f46; color: #34d399; font-weight: bold; padding: 2px 8px; border-radius: 4px; font-size: 8pt;")
            if hasattr(self, "btn_reset"):
                self.btn_reset.setEnabled(True)
        else:
            self.status_pill.setText("OFFLINE")
            self.status_pill.setStyleSheet("background-color: #7f1d1d; color: #f87171; font-weight: bold; padding: 2px 8px; border-radius: 4px; font-size: 8pt;")
            self.btn_connect.setText("🔌 Connect")
            self.btn_connect.setStyleSheet("background-color: #0284c7; color: white; font-weight: bold; border-radius: 4px; padding: 2px 12px;")
            if hasattr(self, "btn_reset"):
                self.btn_reset.setEnabled(False)

    def on_render_tick(self):
        new_data = False

        # 1. Primary path: check for lightweight fused 8-zone telemetry from ESP32 on-chip engine
        arc_pkt = self.serial_engine.take_latest_arc()
        if arc_pkt is not None:
            self._apply_arc_packet(arc_pkt)
            new_data = True
        else:
            # 2. Legacy fallback: check for raw 16-zone matrices (if older firmware or matrix mode)
            pkt1 = self.serial_engine.take_latest(1)
            if pkt1 is not None:
                self._process_raw_frame(1, pkt1)
                new_data = True

            pkt2 = self.serial_engine.take_latest(2)
            if pkt2 is not None:
                self._process_raw_frame(2, pkt2)
                new_data = True

            if new_data:
                self._fuse_and_filter_zones()

        # Watchdog: detect dead / offline sensor and dispatch RESET (with 1.2s timeout and grace period)
        if self.serial_engine.isRunning():
            now = time.perf_counter()
            if now >= self.reset_grace_until:
                st1 = self.serial_engine.sensor_stats.get(1, {})
                st2 = self.serial_engine.sensor_stats.get(2, {})

                rx1 = st1.get("rx_packets", 0) if st1 else 0
                rx2 = st2.get("rx_packets", 0) if st2 else 0
                last1 = st1.get("last_rx_time", 0.0) if st1 else 0.0
                last2 = st2.get("last_rx_time", 0.0) if st2 else 0.0

                s1_active = bool(rx1 >= 1 and (now - last1) <= 1.2)
                s2_active = bool(rx2 >= 1 and (now - last2) <= 1.2)

                if s1_active and s2_active:
                    self.auto_reset_count = 0
                    self.reset_failed = False
                else:
                    one_offline = (s1_active and not s2_active) or (s2_active and not s1_active)
                    both_stalled = (rx1 >= 1 or rx2 >= 1) and (not s1_active and not s2_active)
                    neither_started = (rx1 == 0 and rx2 == 0)

                    if (one_offline or both_stalled or neither_started) and not self.reset_failed:
                        if self.auto_reset_count < 2:
                            self.auto_reset_count += 1
                            self.last_auto_reset_time = now
                            self.reset_grace_until = now + 5.0
                            self.reset_status_until = now + 5.0
                            self.serial_engine.reset_stats()
                            self.serial_engine.send_command("RESET")
                        else:
                            self.reset_failed = True

    def _apply_arc_packet(self, pkt):
        dists = pkt.get("d", [])
        if len(dists) != 8:
            return
        stats = pkt.get("st", [0] * 8)
        spads = pkt.get("sp", [0] * 8)
        fallbacks = pkt.get("fb", [0] * 8)
        target_nums = pkt.get("tn", [0] * 8)
        t2_dists = pkt.get("t2", [0] * 8)
        t3_dists = pkt.get("t3", [0] * 8)
        min_d = pkt.get("min", 3000)

        self.zone_dists = list(dists)
        self.raw_stats = list(stats)
        self.zone_spads = list(spads)
        self.is_blind_zero = [fb == 1 for fb in fallbacks]
        self.is_clear_fallback = [fb == 2 for fb in fallbacks]
        self.is_t2_fallback = [tn == 2 for tn in target_nums]
        self.is_t3_fallback = [tn == 3 for tn in target_nums]

        all_targets = [[] for _ in range(8)]
        for z in range(8):
            d = dists[z]
            st = stats[z]
            fb = fallbacks[z]
            if fb == 0:
                all_targets[z].append((d, st, target_nums[z]))
            if z < len(t2_dists) and t2_dists[z] > 0 and t2_dists[z] != d:
                all_targets[z].append((t2_dists[z], 5, 2))
            if z < len(t3_dists) and t3_dists[z] > 0 and t3_dists[z] != d:
                all_targets[z].append((t3_dists[z], 5, 3))
        self.all_targets = all_targets

        # Update radar canvas
        self.radar_canvas.set_zone_data(
            self.zone_dists, self.raw_dists, self.raw_stats,
            self.is_blind_zero, self.is_clear_fallback,
            self.is_t2_fallback, self.is_t3_fallback,
            all_targets=self.all_targets,
            spads=self.zone_spads
        )

        # Update bottom cards with zero CSS re-parsing overhead
        self._update_cards_optimized(min_d)

    def _process_raw_frame(self, sid, pkt):
        res = pkt.get("r", 16)
        nt = max(1, pkt.get("nt", 2))
        d_list = pkt.get("d", [])
        st_list = pkt.get("st", [])
        sp_list = pkt.get("spad", [])
        if sid == 1 and len(sp_list) >= res:
            self.tof1_spads = np.array(sp_list[:res], dtype=int)
        elif sid == 2 and len(sp_list) >= res:
            self.tof2_spads = np.array(sp_list[:res], dtype=int)
        tot = res * nt
        if len(d_list) >= tot and len(st_list) >= tot:
            d_mat = np.array(d_list[:tot], dtype=int).reshape((res, nt))
            s_mat = np.array(st_list[:tot], dtype=int).reshape((res, nt))
            target_count = min(3, nt)
            if target_count < 3:
                pad_w = 3 - target_count
                d_pad = np.pad(d_mat[:, :target_count], ((0, 0), (0, pad_w)), mode='constant')
                s_pad = np.pad(s_mat[:, :target_count], ((0, 0), (0, pad_w)), mode='constant')
            else:
                d_pad = d_mat[:, :3]
                s_pad = s_mat[:, :3]
            if sid == 1:
                self.tof1_dists = d_pad
                self.tof1_stats = s_pad
            else:
                self.tof2_dists = d_pad
                self.tof2_stats = s_pad

    def _fuse_and_filter_zones(self):
        """Legacy fallback zone extraction with 1-frame error latching."""
        zone_sources = [
            (1, 13),  # Zone 0: ToF 1, Col 2, Row 4 (Leftmost)
            (1, 9),   # Zone 1: ToF 1, Col 2, Row 3
            (1, 5),   # Zone 2: ToF 1, Col 2, Row 2
            (1, 1),   # Zone 3: ToF 1, Col 2, Row 1 (Center-Left)
            (2, 2),   # Zone 4: ToF 2, Col 3, Row 1 (Center-Right)
            (2, 6),   # Zone 5: ToF 2, Col 3, Row 2
            (2, 10),  # Zone 6: ToF 2, Col 3, Row 3
            (2, 14),  # Zone 7: ToF 2, Col 3, Row 4 (Rightmost)
        ]

        final_d = [3000] * 8
        raw_d = [None] * 8
        raw_st = [0] * 8
        raw_spad = [0] * 8
        t2_fb = [False] * 8
        t3_fb = [False] * 8
        blind_zeros = [False] * 8
        clear_fallbacks = [False] * 8
        all_targets = [[] for _ in range(8)]

        min_d = 3000
        for z, (sid, src_idx) in enumerate(zone_sources):
            d_mat = self.tof1_dists if sid == 1 else self.tof2_dists
            s_mat = self.tof1_stats if sid == 1 else self.tof2_stats
            sp_arr = self.tof1_spads if sid == 1 else self.tof2_spads

            spad_val = int(sp_arr[src_idx]) if len(sp_arr) > src_idx else 0
            raw_spad[z] = spad_val

            d0 = int(d_mat[src_idx, 0]) if len(d_mat) > src_idx else 0
            st0 = int(s_mat[src_idx, 0]) if len(s_mat) > src_idx else 0

            d1 = int(d_mat[src_idx, 1]) if (len(d_mat) > src_idx and d_mat.shape[1] > 1) else 0
            st1 = int(s_mat[src_idx, 1]) if (len(s_mat) > src_idx and s_mat.shape[1] > 1) else 0

            d2 = int(d_mat[src_idx, 2]) if (len(d_mat) > src_idx and d_mat.shape[1] > 2) else 0
            st2 = int(s_mat[src_idx, 2]) if (len(s_mat) > src_idx and s_mat.shape[1] > 2) else 0

            candidate_targets = [(d0, st0, 1), (d1, st1, 2)]
            if self.max_targets >= 3:
                candidate_targets.append((d2, st2, 3))

            valid_hits = [
                (dt, stt, tid) for dt, stt, tid in candidate_targets
                if stt in (5, 6, 9, 10) and 0 < dt <= 4000
            ]
            all_targets[z] = valid_hits

            if valid_hits:
                self.error_frame_counts[z] = 0
                chosen = min(valid_hits, key=lambda x: x[0])
                final_d[z] = chosen[0]
                raw_d[z] = chosen[0]
                raw_st[z] = chosen[1]
                t2_fb[z] = (chosen[2] == 2)
                t3_fb[z] = (chosen[2] == 3)
                self.last_valid_dists[z] = chosen[0]
                blind_zeros[z] = False
                clear_fallbacks[z] = False
            else:
                self.error_frame_counts[z] += 1
                if self.error_frame_counts[z] < 2 and self.last_valid_dists[z] > 0:
                    # Hold last valid reading for 1 frame to eliminate flicker
                    final_d[z] = self.last_valid_dists[z]
                    raw_d[z] = self.last_valid_dists[z]
                    raw_st[z] = st0
                    t2_fb[z] = False
                    t3_fb[z] = False
                    blind_zeros[z] = False
                    clear_fallbacks[z] = False
                else:
                    raw_d[z] = None
                    raw_st[z] = st0
                    t2_fb[z] = False
                    t3_fb[z] = False
                    if spad_val == 1024:
                        final_d[z] = 0
                        blind_zeros[z] = True
                        clear_fallbacks[z] = False
                    else:
                        final_d[z] = 3000
                        blind_zeros[z] = False
                        clear_fallbacks[z] = True

            if final_d[z] > 0 and final_d[z] < min_d:
                min_d = final_d[z]

        self.zone_dists = final_d
        self.raw_dists = raw_d
        self.raw_stats = raw_st
        self.zone_spads = raw_spad
        self.is_t2_fallback = t2_fb
        self.is_t3_fallback = t3_fb
        self.is_blind_zero = blind_zeros
        self.is_clear_fallback = clear_fallbacks
        self.all_targets = all_targets

        # Update radar canvas
        self.radar_canvas.set_zone_data(
            self.zone_dists, self.raw_dists, self.raw_stats,
            self.is_blind_zero, self.is_clear_fallback,
            self.is_t2_fallback, self.is_t3_fallback,
            all_targets=self.all_targets,
            spads=self.zone_spads
        )

        # Update bottom cards
        self._update_cards_optimized(min_d)

    def _update_cards_optimized(self, min_d):
        """Zero-overhead card widget updates: only updates styles/text when values change."""
        for z in range(8):
            card = self.card_widgets[z]
            d = self.zone_dists[z]
            meta = card["meta"]
            st = self.raw_stats[z]
            sp_count = self.zone_spads[z]
            is_t2 = self.is_t2_fallback[z]
            is_t3 = self.is_t3_fallback[z]
            hits = self.all_targets[z]
            is_blind = self.is_blind_zero[z]
            is_clear = self.is_clear_fallback[z]

            # 1. Alert color state
            if is_blind or d <= 50:
                state = 0
            elif d < 400:
                state = 1
            elif d < 1000:
                state = 2
            elif d < 2200:
                state = 3
            else:
                state = 4

            if self.card_alert_state[z] != state:
                card["pbar"].setStyleSheet(self.STYLE_PROGRESS[state])
                card["lbl_dist"].setStyleSheet(self.STYLE_DIST[state])
                self.card_alert_state[z] = state

            # 2. Text and tag strings
            if is_blind:
                dist_txt = "0 mm"
                pbar_val = 0
                st_desc = f"s{st}" if st > 0 else "BLIND"
                tag_txt = f"{meta['sensor']} • 0mm ({st_desc}) • {sp_count} SP"
            elif is_clear:
                dist_txt = "3.00 m*"
                pbar_val = 3000
                st_desc = f"s{st}" if st in (14, 255) else "OPEN"
                tag_txt = f"{meta['sensor']} • 3m ({st_desc}) • {sp_count} SP"
            else:
                dist_txt = f"{d} mm"
                pbar_val = min(3000, max(0, d))
                if len(hits) > 1:
                    hits_summary = " | ".join([f"T{h[2]}:{h[0]}" for h in hits])
                    tag_txt = f"{meta['sensor']} • {hits_summary} • {sp_count} SP"
                else:
                    tgt_badge = " [T3]" if is_t3 else (" [T2]" if is_t2 else " [T1]")
                    tag_txt = f"{meta['sensor']} • s{st}{tgt_badge} • {sp_count} SP"

            if self.card_last_dist_text[z] != dist_txt:
                card["lbl_dist"].setText(dist_txt)
                self.card_last_dist_text[z] = dist_txt

            if self.card_last_tag_text[z] != tag_txt:
                card["lbl_tag"].setText(tag_txt)
                self.card_last_tag_text[z] = tag_txt

            if self.card_last_pbar_val[z] != pbar_val:
                card["pbar"].setValue(pbar_val)
                self.card_last_pbar_val[z] = pbar_val

            card["frame"].setToolTip(
                f"Sector {meta['name']} ({meta['ang']})\n"
                f"Sensor: {meta['sensor']} ({meta['hw']})\n"
                f"Distance: {d} mm\n"
                f"Status: s{st}\n"
                f"Active SPADs: {sp_count}"
            )

        # 3. KPI banner update
        if min_d < 3000:
            kpi_txt = f"Min Clearance: {min_d} mm"
            kpi_col = "color: #EF4444; font-weight: bold;" if min_d < 300 else "color: #00E5FF; font-weight: bold;"
        else:
            kpi_txt = "Min Clearance: > 3.0 m (All Clear)"
            kpi_col = "color: #10B981; font-weight: bold;"

        if self.last_kpi_text != kpi_txt:
            self.lbl_kpi_min.setText(kpi_txt)
            self.last_kpi_text = kpi_txt

        if self.last_kpi_color != kpi_col:
            self.lbl_kpi_min.setStyleSheet(kpi_col)
            self.last_kpi_color = kpi_col

    def on_zone_hovered(self, z_idx):
        for z, card in enumerate(self.card_widgets):
            if z == z_idx:
                card["frame"].setStyleSheet("background-color: #1e293b; border: 2px solid #00E5FF; border-radius: 6px; padding: 2px;")
            else:
                card["frame"].setStyleSheet("background-color: #1a2234; border: 1px solid #283548; border-radius: 6px; padding: 2px;")

    def on_diag_tick(self):
        # Update ToF 1 & ToF 2 live transmission badges
        tof1_active = self.serial_engine.is_sensor_active(1)
        tof2_active = self.serial_engine.is_sensor_active(2)
        hz1 = self.serial_engine.get_sensor_hz(1)
        hz2 = self.serial_engine.get_sensor_hz(2)

        dot1 = "🟢" if tof1_active else "⚪"
        dot2 = "🟢" if tof2_active else "⚪"

        self.badge_tof1.setText(f"{dot1} ToF 1 (Left): {hz1:.0f} Hz" if tof1_active and hz1 >= 0.5 else (f"{dot1} ToF 1 (Left): -- Hz" if tof1_active else f"{dot1} ToF 1 (Left): OFFLINE"))
        self.badge_tof2.setText(f"{dot2} ToF 2 (Right): {hz2:.0f} Hz" if tof2_active and hz2 >= 0.5 else (f"{dot2} ToF 2 (Right): -- Hz" if tof2_active else f"{dot2} ToF 2 (Right): OFFLINE"))

        self.badge_tof1.setStyleSheet(
            "background-color: #064e3b; color: #34d399; font-weight: bold; padding: 3px 8px; border-radius: 4px; font-size: 8pt;"
            if tof1_active else
            "background-color: #1e293b; color: #94a3b8; font-weight: bold; padding: 3px 8px; border-radius: 4px; font-size: 8pt;"
        )
        self.badge_tof2.setStyleSheet(
            "background-color: #064e3b; color: #34d399; font-weight: bold; padding: 3px 8px; border-radius: 4px; font-size: 8pt;"
            if tof2_active else
            "background-color: #1e293b; color: #94a3b8; font-weight: bold; padding: 3px 8px; border-radius: 4px; font-size: 8pt;"
        )

        now = time.perf_counter()
        if self.reset_failed:
            self.status_pill.setText("SENSOR DEAD")
            self.status_pill.setStyleSheet("background-color: #7f1d1d; color: #fca5a5; font-weight: bold; padding: 2px 8px; border-radius: 4px; font-size: 8pt;")
        elif now < self.reset_status_until:
            self.status_pill.setText("RECONFIGURING..." if self.auto_reset_count == 0 else "RESETTING...")
            self.status_pill.setStyleSheet("background-color: #b45309; color: #fef3c7; font-weight: bold; padding: 2px 8px; border-radius: 4px; font-size: 8pt;")
        elif self.serial_engine.isRunning():
            self.status_pill.setText("ONLINE")
            self.status_pill.setStyleSheet("background-color: #065f46; color: #34d399; font-weight: bold; padding: 2px 8px; border-radius: 4px; font-size: 8pt;")

    def closeEvent(self, event):
        if self.serial_engine.isRunning():
            self.serial_engine.stop()
        event.accept()


# =============================================================================
# APPLICATION ENTRYPOINT
# =============================================================================

def main():
    app = QApplication(sys.argv)
    window = ToFArcRadarApp()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
