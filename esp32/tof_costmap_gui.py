#!/usr/bin/env python3
"""
===============================================================================
FOLKRACE — 4X TOF 2D COSTMAP & DUAL-TARGET RADAR GUI
===============================================================================
Live 2D Costmap Generator and Multi-Target Visualizer for 4x VL53L8CX Time-of-Flight
sensors connected to an ESP32-S3 over high-speed UART (921,600 / 2,000,000 Baud).

CORE ALGORITHM SPECIFICATION:
  1. Multi-Target Obstacle Stamping:
     If a zone reports 2 (or more) valid targets (Target 1 at d1, Target 2 at d2),
     BOTH targets are stamped as confirmed obstacles in the 2D Costmap.
  2. Guarded Ray-Carving:
     The free-space clearing ray travels from the sensor origin ONLY up to the
     closest target (min(d1, d2) - margin). The space beyond the closest target
     remains UNKNOWN / unobserved (it is never falsely cleared).
  3. C-Space Safety Inflation:
     Obstacles are dilated by an adjustable safety margin (default 75mm),
     producing a Webots-identical hazard mask.
  4. Saturated Near-Blind Contact Fallback:
     SPAD count == 1024 -> 0mm blind-spot contact obstacle.
===============================================================================
"""

import sys
import os
import json
import struct
import time
import math
import threading
from collections import deque
from typing import Dict, List, Tuple, Optional

import numpy as np
try:
    from scipy.ndimage import binary_dilation
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QFrame, QSizePolicy,
    QSlider, QCheckBox, QGroupBox, QSplitter, QTableWidget,
    QTableWidgetItem, QHeaderView, QToolTip, QScrollArea
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QPointF, QRectF
from PyQt5.QtGui import (
    QColor, QFont, QPainter, QBrush, QPen, QLinearGradient, QRadialGradient,
    QPolygonF, QPainterPath, QImage, QPixmap, QPalette
)

if hasattr(Qt, 'HighDpiScaleFactorRoundingPolicy'):
    QApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)


# =============================================================================
# GLOBAL COMPREHENSIVE DARK THEME STYLESHEET (NO BLACK TEXT ON DARK SURFACES)
# =============================================================================

DARK_THEME_QSS = """
    QMainWindow, QWidget#CentralWidget {
        background-color: #0b0e14;
        color: #e6edf3;
    }
    QWidget {
        background-color: transparent;
        color: #e6edf3;
        font-family: 'Segoe UI', 'Consolas', -apple-system, sans-serif;
    }
    QLabel {
        color: #e6edf3;
        font-size: 11px;
    }
    QGroupBox {
        font-size: 11px;
        font-weight: bold;
        color: #58a6ff;
        border: 1px solid #1c2636;
        border-radius: 6px;
        margin-top: 10px;
        padding-top: 14px;
        background-color: #0d1117;
    }
    QGroupBox::title {
        subcontrol-origin: margin;
        subcontrol-position: top left;
        left: 10px;
        padding: 0 6px;
        background-color: #0d1117;
        color: #58a6ff;
    }
    QComboBox {
        background-color: #161b22;
        border: 1px solid #30363d;
        border-radius: 4px;
        color: #00ffcc;
        padding: 4px 8px;
        font-weight: bold;
        font-size: 11px;
        min-height: 20px;
    }
    QComboBox:hover {
        border-color: #58a6ff;
    }
    QComboBox::drop-down {
        border: none;
        width: 18px;
    }
    QComboBox QAbstractItemView {
        background-color: #161b22;
        color: #e6edf3;
        selection-background-color: #1f6feb;
        selection-color: #ffffff;
        border: 1px solid #30363d;
        outline: none;
    }
    QPushButton {
        background-color: #21262d;
        border: 1px solid #30363d;
        border-radius: 5px;
        color: #e6edf3;
        padding: 5px 12px;
        font-weight: bold;
        font-size: 11px;
    }
    QPushButton:hover {
        background-color: #30363d;
        border-color: #8b949e;
        color: #ffffff;
    }
    QPushButton:pressed {
        background-color: #161b22;
    }
    QCheckBox {
        color: #e6edf3;
        font-size: 11px;
        spacing: 6px;
    }
    QCheckBox::indicator {
        width: 14px;
        height: 14px;
        border-radius: 3px;
        border: 1px solid #30363d;
        background-color: #161b22;
    }
    QCheckBox::indicator:hover {
        border-color: #58a6ff;
    }
    QCheckBox::indicator:checked {
        background-color: #238636;
        border-color: #2ea043;
    }
    QSlider::groove:horizontal {
        height: 6px;
        background: #21262d;
        border-radius: 3px;
    }
    QSlider::sub-page:horizontal {
        background: #1f6feb;
        border-radius: 3px;
    }
    QSlider::handle:horizontal {
        background: #58a6ff;
        border: 1px solid #79c0ff;
        width: 14px;
        margin-top: -4px;
        margin-bottom: -4px;
        border-radius: 7px;
    }
    QSlider::handle:horizontal:hover {
        background: #79c0ff;
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
        background: #21262d;
        min-height: 20px;
        border-radius: 4px;
    }
    QScrollBar::handle:vertical:hover {
        background: #30363d;
    }
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
        height: 0px;
    }
    QToolTip {
        background-color: #161b22;
        color: #e6edf3;
        border: 1px solid #30363d;
        padding: 4px 6px;
        font-size: 11px;
    }
"""


# =============================================================================
# WEBOTS 16-CHANNEL CRISS-CROSS FRONT TOF RIG (only_front_tofs.proto)
# Robot Body Frame: +Y is Forward, +X is Right, -X is Left
# =============================================================================

WEBOTS_CRISS_CROSS_RIG = [
    {
        "id": 1,
        "name": "ToF 1 (Front-Left Center)",
        "x": -0.058,
        "y": 0.092,
        "beam_angles_rad": [1.5512, 1.3548, 1.1585, 0.9621],  # 88.88° to 55.12° (pointing right)
        "color": "#00ffcc"
    },
    {
        "id": 2,
        "name": "ToF 2 (Front-Left Flank)",
        "x": -0.057,
        "y": 0.090,
        "beam_angles_rad": [0.7658, 0.5694, 0.3731, 0.1767],  # 43.88° to 10.12° (pointing far right)
        "color": "#3399ff"
    },
    {
        "id": 3,
        "name": "ToF 3 (Front-Right Center)",
        "x": 0.058,
        "y": 0.092,
        "beam_angles_rad": [1.5904, 1.7868, 1.9831, 2.1795],  # 91.12° to 124.88° (pointing left)
        "color": "#ffaa00"
    },
    {
        "id": 4,
        "name": "ToF 4 (Front-Right Flank)",
        "x": 0.057,
        "y": 0.090,
        "beam_angles_rad": [2.3758, 2.5722, 2.7685, 2.9649],  # 136.12° to 169.88° (pointing far left)
        "color": "#ff007f"
    },
]


# =============================================================================
# ASYNCHRONOUS 4-SENSOR SERIAL COMMUNICATIONS ENGINE (REAL HARDWARE ONLY)
# =============================================================================

class QuadSerialEngine(QThread):
    connection_changed = pyqtSignal(bool, str)
    ack_received = pyqtSignal(dict)
    config_received = pyqtSignal(dict)

    def __init__(self, port="COM7", baud=921600):
        super().__init__()
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self._running = False
        self._lock = threading.Lock()

        self.reset_stats()

    def reset_stats(self):
        with self._lock:
            self._latest_packets: Dict[int, Optional[dict]] = {1: None, 2: None, 3: None, 4: None}
            self.sensor_stats: Dict[int, dict] = {
                i: {
                    "rx_packets": 0,
                    "last_seq": None,
                    "last_rx_time": 0.0,
                    "timestamps": deque(maxlen=60),
                    "smooth_hz": 0.0,
                    "temp_c": 0,
                    "multi_target_count": 0
                }
                for i in range(1, 5)
            }

    def take_latest_frame(self, sensor_id: int) -> Optional[dict]:
        with self._lock:
            pkt = self._latest_packets.get(sensor_id)
            self._latest_packets[sensor_id] = None
        return pkt

    def is_sensor_active(self, sensor_id: int, timeout_s=1.2) -> bool:
        with self._lock:
            st = self.sensor_stats.get(sensor_id)
            if not st or not st["last_rx_time"]:
                return False
            return (time.perf_counter() - st["last_rx_time"]) < timeout_s

    def get_sensor_hz(self, sensor_id: int) -> float:
        now = time.perf_counter()
        with self._lock:
            st = self.sensor_stats.get(sensor_id)
            if not st or not st["last_rx_time"]:
                return 0.0
            if (now - st["last_rx_time"]) > 1.2:
                st["smooth_hz"] = 0.0
                return 0.0
            dq = st.get("timestamps")
            if not dq or len(dq) < 2:
                return st.get("smooth_hz", 0.0)
            cutoff = now - 1.2
            while dq and dq[0] < cutoff:
                dq.popleft()
            if len(dq) < 2:
                return st.get("smooth_hz", 0.0)
            dt = dq[-1] - dq[0]
            if dt > 0.001:
                hz = (len(dq) - 1) / dt
                st["smooth_hz"] = round(st["smooth_hz"] * 0.7 + hz * 0.3, 1)
            return st.get("smooth_hz", 0.0)

    def send_command(self, cmd_str: str):
        if self.ser and self.ser.is_open and self._running:
            try:
                msg = (cmd_str.strip() + "\n").encode('utf-8')
                self.ser.write(msg)
                self.ser.flush()
            except Exception:
                pass

    def _handle_data_packet(self, sid: int, pkt: dict):
        if 1 <= sid <= 4:
            now = time.perf_counter()
            with self._lock:
                self._latest_packets[sid] = pkt
                st = self.sensor_stats[sid]
                st["rx_packets"] += 1
                st["last_rx_time"] = now
                st["timestamps"].append(now)
                st["temp_c"] = pkt.get("temp", 0)
                st["flags"] = pkt.get("flags", 0)
                st["min_d"] = pkt.get("min", 4000)

    def run(self):
        self._running = True
        PACKET_SIZE = 144
        buf = bytearray()

        while self._running:
            if not self.ser or not self.ser.is_open:
                try:
                    self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
                    try:
                        self.ser.dtr = False
                        self.ser.rts = False
                    except Exception:
                        pass
                    self.ser.reset_input_buffer()
                    buf.clear()
                    self.connection_changed.emit(True, f"Connected ({self.port} @ {self.baud})")
                    # Request binary stream by default
                    self.send_command("SET_STREAM:bin")
                except Exception as e:
                    self.connection_changed.emit(False, f"Offline ({e})")
                    time.sleep(0.5)
                    continue

            try:
                # Read whatever bytes are waiting in the OS serial buffer
                waiting = self.ser.in_waiting
                chunk = self.ser.read(max(1, waiting)) if waiting > 0 else self.ser.read(1)
                if not chunk:
                    continue
                buf.extend(chunk)

                # Process buffer
                while len(buf) >= 2:
                    # 1. Fast path: Binary Telemetry Packet (0xAA 0x55)
                    if buf[0] == 0xAA and buf[1] == 0x55:
                        if len(buf) < PACKET_SIZE:
                            break  # Need more bytes for full binary frame
                        pkt_bytes = bytes(buf[:PACKET_SIZE])
                        del buf[:PACKET_SIZE]

                        # Verify 16-bit additive checksum over bytes 0..141
                        calc_chk = sum(pkt_bytes[:142]) & 0xFFFF
                        exp_chk = struct.unpack('<H', pkt_bytes[142:144])[0]

                        if calc_chk == exp_chk:
                            p_type, sid, seq, ts, min_d, temp, flags = struct.unpack('<BBHIhBB', pkt_bytes[2:14])
                            d_list = list(struct.unpack('<32H', pkt_bytes[14:78]))
                            st_list = list(struct.unpack('<32B', pkt_bytes[78:110]))
                            spad_list = list(struct.unpack('<16H', pkt_bytes[110:142]))

                            pkt = {
                                "t": "D",
                                "id": sid,
                                "seq": seq,
                                "ts": ts,
                                "r": 16,
                                "nt": 2,
                                "min": min_d,
                                "temp": temp,
                                "flags": flags,
                                "d": d_list,
                                "st": st_list,
                                "spad": spad_list
                            }
                            self._handle_data_packet(sid, pkt)
                        else:
                            # Bad checksum: advance by 1 byte to re-sync
                            del buf[:1]
                        continue

                    # 2. Text / JSON Packet (ACK, CONFIG, or JSON matrix)
                    elif buf[0] == ord('{'):
                        nl_pos = buf.find(b'\n')
                        if nl_pos == -1:
                            if len(buf) > 4096:
                                del buf[:1]
                            break
                        line_bytes = buf[:nl_pos]
                        del buf[:nl_pos + 1]

                        try:
                            line_str = line_bytes.decode('utf-8', errors='ignore').strip()
                            if line_str.startswith("{") and line_str.endswith("}"):
                                pkt = json.loads(line_str)
                                p_type = pkt.get("t") or pkt.get("type")
                                if p_type == "D":
                                    sid = pkt.get("id", 1)
                                    self._handle_data_packet(sid, pkt)
                                elif p_type == "ACK":
                                    self.ack_received.emit(pkt)
                                elif p_type == "CONFIG":
                                    self.config_received.emit(pkt)
                        except Exception:
                            pass
                        continue

                    else:
                        # Advance past noise / unaligned byte
                        del buf[:1]

            except Exception:
                time.sleep(0.01)

        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def stop(self):
        self._running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.wait(500)


# =============================================================================
# EXACT 2D COSTMAP COMPUTATION ENGINE (WEBOTS ALIGNED)
# =============================================================================

class CostmapEngine:
    """
    High-performance 2D Occupancy Grid Costmap Processor aligned with Webots:
      - Coordinate Frame: +Y Forward, +X Right, -X Left
      - Metric Bounds: X in [-1.25, +1.25] m, Y in [-0.10, +1.90] m (1.80m ToF ceiling)
      - Resolution: 0.015 m (15 mm / cell)
      - Implements dual-target rule:
          1. Both Target 1 and Target 2 become confirmed obstacles inside the same zone.
          2. Clearing rays travel only until the closest target min(d1, d2) - margin.
    """

    def __init__(self, x_min=-1.25, x_max=1.25, y_min=-0.10, y_max=1.90, res=0.015, r_inflation=0.075):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.res = res
        self.r_inflation = r_inflation

        self.width = int(round((x_max - x_min) / res)) + 1
        self.height = int(round((y_max - y_min) / res)) + 1

        # Probability Map: 0.50 = Unknown (yellow tint), 0.0 = Free (dark navy), 1.0 = Obstacle (solid red)
        self.prob_map = np.full((self.height, self.width), 0.50, dtype=np.float32)
        self.hazard_mask = np.zeros((self.height, self.width), dtype=bool)

        # Active Multi-Target Detection records: list of dicts for inspector
        self.active_multi_targets: List[dict] = []
        self.all_beam_records: List[dict] = []

        self._build_dilation_kernel()

    def set_parameters(self, res: float, r_inflation: float):
        self.res = res
        self.r_inflation = r_inflation
        self.width = int(round((self.x_max - self.x_min) / res)) + 1
        self.height = int(round((self.y_max - self.y_min) / res)) + 1
        self.prob_map = np.full((self.height, self.width), 0.50, dtype=np.float32)
        self.hazard_mask = np.zeros((self.height, self.width), dtype=bool)
        self._build_dilation_kernel()

    def _build_dilation_kernel(self):
        rad_cells = int(math.ceil(self.r_inflation / self.res))
        y, x = np.ogrid[-rad_cells:rad_cells + 1, -rad_cells:rad_cells + 1]
        self.dilation_kernel = (x * x + y * y) <= (rad_cells * rad_cells)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int(round((x - self.x_min) / self.res))
        gy = int(round((y - self.y_min) / self.res))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = self.x_min + gx * self.res
        y = self.y_min + gy * self.res
        return x, y

    def clear(self):
        self.prob_map.fill(0.50)
        self.hazard_mask.fill(False)
        self.active_multi_targets.clear()
        self.all_beam_records.clear()

    def process_sensor_frames(self, sensor_rig: List[dict], raw_packets: Dict[int, Optional[dict]], max_range_m=1.80) -> None:
        """
        Updates the 2D Costmap with real ToF telemetry from all 4 sensors.
        """
        # Step 1: Reset grid to unobserved state
        self.prob_map.fill(0.50)
        self.active_multi_targets.clear()
        self.all_beam_records.clear()

        # Step 1b: Vehicle Chassis Origin Clearance (safe drivable floor directly behind front bumper)
        chassis_xmin, chassis_xmax = self.world_to_grid(-0.075, 0.075)
        chassis_ymin, chassis_ymax = self.world_to_grid(-0.060, 0.080)
        chassis_xmin = max(0, min(self.width, chassis_xmin))
        chassis_xmax = max(0, min(self.width, chassis_xmax + 1))
        chassis_ymin = max(0, min(self.height, chassis_ymin))
        chassis_ymax = max(0, min(self.height, chassis_ymax + 1))
        self.prob_map[chassis_ymin:chassis_ymax, chassis_xmin:chassis_xmax] = 0.0

        VALID_STATUSES = {5, 6, 9, 10}
        H, W = self.height, self.width

        # Step 2: Ray-Carving and Multi-Target Stamping per sensor
        for s_cfg in sensor_rig:
            sid = s_cfg["id"]
            pkt = raw_packets.get(sid)
            if not pkt:
                continue

            x0 = s_cfg["x"]
            y0 = s_cfg["y"]
            beam_angles = s_cfg.get("beam_angles_rad", [])

            res_zones = pkt.get("r", 16)
            nt = max(1, pkt.get("nt", 2))
            d_list = pkt.get("d", [])
            st_list = pkt.get("st", [])
            sp_list = pkt.get("spad", [])

            if len(d_list) < res_zones * nt:
                continue

            # In Webots 16-channel rig, each sensor has 4 explicit criss-cross beam angles
            num_cols = len(beam_angles) if beam_angles else (4 if res_zones in (16, 64) else res_zones)

            for col_idx in range(num_cols):
                if beam_angles and col_idx < len(beam_angles):
                    beam_ang = beam_angles[col_idx]
                else:
                    beam_ang = math.pi / 2.0

                cos_a = math.cos(beam_ang)
                sin_a = math.sin(beam_ang)

                # Collect all valid target distances for this column
                # In 4x4 mode, we aggregate zones belonging to this column (rows 0..3)
                col_targets: List[Tuple[float, int, int]] = []  # (dist_m, status, target_idx)
                col_spads = 0

                rows_to_check = [0, 1, 2, 3] if res_zones == 16 else [0]
                for row_idx in rows_to_check:
                    z = row_idx * 4 + col_idx if res_zones == 16 else col_idx
                    if z < len(sp_list):
                        col_spads = max(col_spads, sp_list[z])

                    for t_idx in range(nt):
                        flat_idx = z * nt + t_idx
                        if flat_idx < len(d_list):
                            d_mm = d_list[flat_idx]
                            st = st_list[flat_idx] if flat_idx < len(st_list) else 0
                            d_m = d_mm / 1000.0

                            # Valid if non-zero, valid ST status code, within max range
                            if 0.02 <= d_m <= max_range_m and st in VALID_STATUSES:
                                col_targets.append((d_m, st, t_idx + 1))

                # Deduplicate targets that are within 35mm of each other
                col_targets.sort(key=lambda item: item[0])
                unique_targets: List[Tuple[float, int, int]] = []
                for tgt in col_targets:
                    if not unique_targets or (tgt[0] - unique_targets[-1][0]) > 0.035:
                        unique_targets.append(tgt)

                # =================================================================
                # USER'S CORE LOGIC:
                # 1. Clearing rays go ONLY to the closest target min(d1, d2)
                # 2. If 2 targets report distance, BOTH become obstacles!
                # =================================================================
                beam_info = {
                    "sensor_id": sid,
                    "col": col_idx,
                    "x0": x0,
                    "y0": y0,
                    "angle": beam_ang,
                    "targets": unique_targets,
                    "spads": col_spads,
                    "d_clear": max_range_m,
                    "is_multi": len(unique_targets) >= 2
                }

                if not unique_targets:
                    # Check for saturated 1024 SPAD contact (<50mm blind spot)
                    if col_spads == 1024:
                        # Stamped as blind-spot obstacle right at sensor front
                        obs_x = x0 + 0.04 * cos_a
                        obs_y = y0 + 0.04 * sin_a
                        gx, gy = self.world_to_grid(obs_x, obs_y)
                        if 0 <= gx < W and 0 <= gy < H:
                            self.prob_map[gy, gx] = 1.0
                        beam_info["d_clear"] = 0.04
                    else:
                        # Full open-space clearing up to max range
                        r_steps = np.arange(0.04, max_range_m, self.res * 0.7)
                        rx = x0 + r_steps * cos_a
                        ry = y0 + r_steps * sin_a
                        gx = np.round((rx - self.x_min) / self.res).astype(int)
                        gy = np.round((ry - self.y_min) / self.res).astype(int)
                        valid_idx = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
                        self.prob_map[gy[valid_idx], gx[valid_idx]] = 0.0
                        beam_info["d_clear"] = max_range_m

                else:
                    # Target(s) detected!
                    d_closest = unique_targets[0][0]
                    beam_info["d_clear"] = d_closest

                    # STEP A: Ray clearing goes ONLY up to the closest target
                    # Safety margin: stop 35mm before obstacle so boundary cell is untouched
                    r_clear_max = max(0.04, d_closest - 0.035)
                    r_steps = np.arange(0.04, r_clear_max, self.res * 0.7)
                    if len(r_steps) > 0:
                        rx = x0 + r_steps * cos_a
                        ry = y0 + r_steps * sin_a
                        gx = np.round((rx - self.x_min) / self.res).astype(int)
                        gy = np.round((ry - self.y_min) / self.res).astype(int)
                        valid_idx = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
                        self.prob_map[gy[valid_idx], gx[valid_idx]] = 0.0

                    # STEP B: ALL targets become obstacles inside this same zone!
                    # For each target (Target 1, Target 2, etc.), stamp an obstacle cluster
                    for (d_tgt, st_code, t_num) in unique_targets:
                        r_band = np.linspace(d_tgt - 0.025, d_tgt + 0.025, 3)
                        ang_band = np.linspace(-math.radians(4.0), math.radians(4.0), 5)

                        for r in r_band:
                            for a_off in ang_band:
                                ox = x0 + r * math.cos(beam_ang + a_off)
                                oy = y0 + r * math.sin(beam_ang + a_off)
                                ogx, ogy = self.world_to_grid(ox, oy)
                                if 0 <= ogx < W and 0 <= ogy < H:
                                    self.prob_map[ogy, ogx] = 1.0

                    # Record multi-target zone for inspector UI if >= 2 targets exist
                    if len(unique_targets) >= 2:
                        t1 = unique_targets[0]
                        t2 = unique_targets[1]
                        rec = {
                            "sensor_id": sid,
                            "sensor_name": s_cfg["name"],
                            "col": col_idx,
                            "t1_dist_mm": int(t1[0] * 1000),
                            "t2_dist_mm": int(t2[0] * 1000),
                            "sep_mm": int((t2[0] - t1[0]) * 1000),
                            "cleared_to_mm": int(r_clear_max * 1000),
                            "t1_pos": (x0 + t1[0] * cos_a, y0 + t1[0] * sin_a),
                            "t2_pos": (x0 + t2[0] * cos_a, y0 + t2[0] * sin_a)
                        }
                        self.active_multi_targets.append(rec)

                self.all_beam_records.append(beam_info)

        # Step 3: C-Space Safety Dilation / Hazard Mask (prob > 0.65)
        obs_binary = (self.prob_map > 0.65)
        if HAS_SCIPY:
            self.hazard_mask = binary_dilation(obs_binary, structure=self.dilation_kernel)
        else:
            # Fallback fast flat dilation
            rad = int(math.ceil(self.r_inflation / self.res))
            self.hazard_mask = obs_binary.copy()
            for dy in range(-rad, rad + 1):
                for dx in range(-rad, rad + 1):
                    if dx * dx + dy * dy <= rad * rad:
                        self.hazard_mask |= np.roll(np.roll(obs_binary, dy, axis=0), dx, axis=1)


# =============================================================================
# TOP-DOWN 2D COSTMAP CANVAS WIDGET (EXACT WEBOTS GRID)
# =============================================================================

class CostmapCanvas(QWidget):
    """
    Renders top-down 2D costmap matching Webots Planner Costmap Canvas:
      - Coordinate Frame: +Y is Forward, +X is Right, -X is Left
      - Free Space (prob < 0.38): Clear dark navy drivable floor rgba(14, 22, 34, 150)
      - Unknown Space (0.38 <= prob <= 0.65): Distinct translucent yellow rgba(245, 215, 30, 130)
      - Confirmed Obstacles (prob > 0.65): Pure solid red rgba(255, 0, 0, 255)
      - C-Space Inflated Margin: Coral red rgba(255, 42, 64, 175)
      - Webots Cartesian Metric Grid lines and Distance Rings:
          * Horizontal lines across [-0.5, 0.0, 0.5, 1.0, 1.5, 1.8] with '1.8m (ToF Limit)'
          * Vertical lines across [-1.0, -0.5, 0.0, 0.5, 1.0]
          * Concentric range circles at [0.5, 1.0, 1.5, 1.8]
      - Scale vehicle chassis footprint (150x200mm) with wheels and front bumper
      - Conical ToF 11.25° ray sectors with dual-target contact arcs & shadow lines
    """
    cursor_moved = pyqtSignal(float, float, str)

    def __init__(self, costmap_engine: CostmapEngine, parent=None):
        super().__init__(parent)
        self.cm = costmap_engine
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(540, 500)
        self.setMouseTracking(True)

        # Viewport metric bounds matching Webots Planner GUI
        self.x_min = -1.60
        self.x_max = 1.60
        self.y_min = -0.35
        self.y_max = 2.05

        # Display layer toggles
        self.show_costmap = True
        self.show_grid = True
        self.show_rays = True
        self.show_inflation = True
        self.show_multi_markers = True
        self.show_chassis = True
        self.show_sensors = True

        # Viewport zoom & pan
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self._dragging = False
        self._last_drag_pos = QPointF()

    def reset_view(self):
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.update()

    def get_viewport_transform(self) -> Tuple[float, float, float, float, float]:
        w = max(10.0, float(self.width()))
        h = max(10.0, float(self.height()))
        dx = self.x_max - self.x_min
        dy = self.y_max - self.y_min

        # Preserve strict 1:1 metric aspect ratio (no distortion of circles or grid)
        scale = min(w / dx, h / dy) * self.zoom

        # Center of canvas widget
        cx = w * 0.5 + self.pan_x
        cy = h * 0.5 + self.pan_y

        # Center of metric world viewport
        mx0 = (self.x_min + self.x_max) * 0.5
        my0 = (self.y_min + self.y_max) * 0.5
        return cx, cy, mx0, my0, scale

    def world_to_screen(self, xm: float, ym: float) -> QPointF:
        cx, cy, mx0, my0, scale = self.get_viewport_transform()
        px = cx + (xm - mx0) * scale
        py = cy - (ym - my0) * scale  # +Y is forward (up)
        return QPointF(px, py)

    def screen_to_world(self, px: float, py: float) -> Tuple[float, float]:
        cx, cy, mx0, my0, scale = self.get_viewport_transform()
        xm = mx0 + (px - cx) / scale
        ym = my0 - (py - cy) / scale
        return xm, ym

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        factor = 1.15 if delta > 0 else 0.87
        self.zoom = max(0.4, min(4.0, self.zoom * factor))
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.MiddleButton or (event.button() == Qt.LeftButton and event.modifiers() & Qt.ControlModifier):
            self._dragging = True
            self._last_drag_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self._dragging:
            delta = event.pos() - self._last_drag_pos
            self.pan_x += delta.x()
            self.pan_y += delta.y()
            self._last_drag_pos = event.pos()
            self.update()

        wx, wy = self.screen_to_world(event.x(), event.y())
        gx, gy = self.cm.world_to_grid(wx, wy)
        state_str = "OUT OF BOUNDS"
        if 0 <= gx < self.cm.width and 0 <= gy < self.cm.height:
            val = self.cm.prob_map[gy, gx]
            is_haz = self.cm.hazard_mask[gy, gx]
            if val > 0.65:
                state_str = f"OBSTACLE (Direct Hit, prob={val:.2f})"
            elif is_haz:
                state_str = "HAZARD (C-Space Inflated Margin)"
            elif val < 0.38:
                state_str = f"FREE (Drivable Space, prob={val:.2f})"
            else:
                state_str = f"UNKNOWN (Unobserved, prob={val:.2f})"

        self.cursor_moved.emit(wx, wy, state_str)

    def mouseReleaseEvent(self, event):
        self._dragging = False

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setRenderHint(QPainter.SmoothPixmapTransform, True)

        # 1. Dark obsidian cockpit background matching Webots
        painter.fillRect(self.rect(), QColor(10, 13, 18))

        # 2. Webots Costmap Probability Image
        if self.show_costmap and self.cm.prob_map is not None:
            self._draw_costmap_image(painter)

        # 3. Webots Grid Lines & Range Circles
        if self.show_grid:
            self._draw_webots_grid(painter)

        # 4. Sensor Conical Sectors, Clearing Rays & Obstacle Contact Arcs
        if self.show_rays:
            self._draw_sensor_cones_and_rays(painter)

        # 5. Dual-Target Markers & Linking Shadow Lines
        if self.show_multi_markers:
            self._draw_multi_target_markers(painter)

        # 6. Webots Vehicle Chassis Footprint & Wheels (150x200mm)
        if self.show_chassis:
            self._draw_vehicle_chassis(painter)

        # 7. Front Sensor Mounting Badges
        if self.show_sensors:
            self._draw_sensor_modules(painter)

    def _draw_costmap_image(self, painter: QPainter):
        """Generates an RGBA8888 pixmap matching the authoritative Webots costmap palette."""
        H, W = self.cm.height, self.cm.width
        img_data = np.zeros((H, W, 4), dtype=np.uint8)

        # 1a. Free Space (prob < 0.38): Clear dark navy drivable floor
        free_mask = (self.cm.prob_map < 0.38)
        img_data[free_mask, 0] = 14
        img_data[free_mask, 1] = 22
        img_data[free_mask, 2] = 34
        img_data[free_mask, 3] = 150

        # 1b. Unknown Space (0.38 <= prob <= 0.65): Distinct translucent yellow
        unknown_mask = (self.cm.prob_map >= 0.38) & (self.cm.prob_map <= 0.65)
        img_data[unknown_mask, 0] = 245
        img_data[unknown_mask, 1] = 215
        img_data[unknown_mask, 2] = 30
        img_data[unknown_mask, 3] = 130

        # 1c. Confirmed Obstacle Hits (prob > 0.65): Pure solid red
        obs_mask = (self.cm.prob_map > 0.65)
        img_data[obs_mask, 0] = 255
        img_data[obs_mask, 1] = 0
        img_data[obs_mask, 2] = 0
        img_data[obs_mask, 3] = 255

        # 1d. C-Space Inflated Safety Margin (hazard_mask > 0 outside direct hit): Coral red
        if self.show_inflation and self.cm.hazard_mask is not None:
            inflated_mask = (self.cm.hazard_mask > 0) & (~obs_mask)
            img_data[inflated_mask, 0] = 255
            img_data[inflated_mask, 1] = 42
            img_data[inflated_mask, 2] = 64
            img_data[inflated_mask, 3] = 175

        # Flip vertically because row 0 in NumPy is at y_min (bottom)
        img_data = np.ascontiguousarray(np.flipud(img_data))
        qimg = QImage(img_data.data, W, H, W * 4, QImage.Format_RGBA8888).copy()

        p_tl = self.world_to_screen(self.cm.x_min, self.cm.y_max)
        p_br = self.world_to_screen(self.cm.x_max, self.cm.y_min)
        dest_rect = QRectF(p_tl.x(), p_tl.y(), p_br.x() - p_tl.x(), p_br.y() - p_tl.y())
        painter.drawImage(dest_rect, qimg)

        # Dotted costmap boundary rect matching Webots
        painter.setPen(QPen(QColor(80, 120, 160, 90), 1, Qt.DotLine))
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(dest_rect)

    def _draw_webots_grid(self, painter: QPainter):
        """Draws exact Webots rectangular Cartesian metric grid lines and concentric range rings."""
        grid_pen = QPen(QColor(50, 70, 90, 110), 1, Qt.DashLine)
        painter.setPen(grid_pen)
        painter.setFont(QFont("Consolas", 8))

        # Horizontal grid lines across [-0.5, 0.0, 0.5, 1.0, 1.5, 1.8]
        for ym in [-0.5, 0.0, 0.5, 1.0, 1.5, 1.8]:
            p1 = self.world_to_screen(self.cm.x_min, ym)
            p2 = self.world_to_screen(self.cm.x_max, ym)
            painter.drawLine(p1, p2)
            painter.setPen(QPen(QColor(100, 150, 200, 180)))
            lbl_text = "1.8m (ToF Limit)" if ym == 1.8 else (f"{ym:+.1f}m" if ym != 0 else "0.0m (Axle)")
            painter.drawText(QPointF(p1.x() + 8, p1.y() - 4), lbl_text)
            painter.setPen(grid_pen)

        # Vertical grid lines across [-1.0, -0.5, 0.0, 0.5, 1.0]
        for xm in [-1.0, -0.5, 0.0, 0.5, 1.0]:
            p1 = self.world_to_screen(xm, self.cm.y_min)
            p2 = self.world_to_screen(xm, self.cm.y_max)
            painter.drawLine(p1, p2)
            painter.setPen(QPen(QColor(100, 150, 200, 180)))
            painter.drawText(QPointF(p1.x() - 14, p1.y() + 14), f"{xm:+.1f}m")
            painter.setPen(grid_pen)

        # Concentric distance rings at [0.5, 1.0, 1.5, 1.8]
        for ring_r in [0.5, 1.0, 1.5, 1.8]:
            r_tl = self.world_to_screen(-ring_r, ring_r)
            r_br = self.world_to_screen(ring_r, -ring_r)
            painter.setPen(QPen(QColor(60, 90, 120, 50), 1, Qt.DotLine))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QRectF(r_tl.x(), r_tl.y(), r_br.x() - r_tl.x(), r_br.y() - r_tl.y()))

    def _draw_sensor_cones_and_rays(self, painter: QPainter):
        """Draws true 11.25° conical ToF sectors and contact arcs matching Webots."""
        half_fov = math.radians(5.625)  # 5.625° half FOV (11.25° total cone)

        for beam in self.cm.all_beam_records:
            x0 = beam["x0"]
            y0 = beam["y0"]
            heading = beam["angle"]
            d_clear = beam["d_clear"]
            targets = beam.get("targets", [])
            is_hit = len(targets) > 0
            is_multi = beam.get("is_multi", False)

            p_origin = self.world_to_screen(x0, y0)
            disp_r = min(max(float(d_clear), 0.04), 1.80)

            # Arc endpoints across true 11.25 deg cone
            n_arc = 6
            angles = np.linspace(heading - half_fov, heading + half_fov, n_arc)
            arc_pts = []
            for a in angles:
                ax = x0 + disp_r * math.cos(a)
                ay = y0 + disp_r * math.sin(a)
                arc_pts.append(self.world_to_screen(ax, ay))

            # Midpoint optical axis
            cx = x0 + disp_r * math.cos(heading)
            cy = y0 + disp_r * math.sin(heading)
            p_c = self.world_to_screen(cx, cy)

            if is_hit:
                if is_multi:
                    c_fill = QColor(255, 170, 0, 45)
                    c_line = QColor(255, 180, 0, 220)
                elif disp_r < 0.25:
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

            # 1. Conical sector polygon fill
            poly = QPolygonF([p_origin] + arc_pts)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(c_fill))
            painter.drawPolygon(poly)

            # 2. Boundary rays
            edge_pen = QPen(QColor(c_line.red(), c_line.green(), c_line.blue(), 90 if is_hit else 40), 1.0, Qt.SolidLine if is_hit else Qt.DotLine)
            painter.setPen(edge_pen)
            painter.drawLine(p_origin, arc_pts[0])
            painter.drawLine(p_origin, arc_pts[-1])

            # 3. Contact arc on closest obstacle surface (strictly stopping ray)
            if is_hit:
                arc_pen = QPen(c_line, 2.5, Qt.SolidLine, Qt.RoundCap)
                painter.setPen(arc_pen)
                for k in range(len(arc_pts) - 1):
                    painter.drawLine(arc_pts[k], arc_pts[k + 1])

            # 4. Center optical axis ray
            painter.setPen(QPen(c_line, 1.2 if is_hit else 0.8, Qt.DashLine if is_hit else Qt.DotLine))
            painter.drawLine(p_origin, p_c)

            # 5. Sensor origin mount dot
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(QColor(0, 220, 255, 200)))
            painter.drawEllipse(p_origin, 2.5, 2.5)

            # 6. Obstacle hit marker and range tag
            if is_hit:
                painter.setPen(QPen(c_line, 2.0))
                painter.setBrush(QBrush(c_line))
                painter.drawEllipse(p_c, 3.5, 3.5)
                if disp_r < 1.40 and not is_multi:
                    painter.setFont(QFont("Consolas", 7))
                    painter.setPen(QPen(QColor(255, 255, 255, 220)))
                    painter.drawText(QPointF(p_c.x() + 4, p_c.y() - 3), f"{disp_r*100:.0f}cm")

    def _draw_multi_target_markers(self, painter: QPainter):
        """Draws visual callout tags for multi-target zones showing both T1 and T2 as obstacles."""
        for rec in self.cm.active_multi_targets:
            t1_x, t1_y = rec["t1_pos"]
            t2_x, t2_y = rec["t2_pos"]
            p1 = self.world_to_screen(t1_x, t1_y)
            p2 = self.world_to_screen(t2_x, t2_y)

            # Dashed shadow line connecting Target 1 and Target 2
            painter.setPen(QPen(QColor(255, 170, 0, 180), 1.5, Qt.DashDotLine))
            painter.drawLine(p1, p2)

            # Target 1 Marker (Cyan with outer red pulse)
            painter.setPen(QPen(QColor(255, 40, 70), 2))
            painter.setBrush(QBrush(QColor(0, 255, 200)))
            painter.drawEllipse(p1, 5.0, 5.0)

            # Target 2 Marker (Golden Amber - also obstacle!)
            painter.setPen(QPen(QColor(255, 40, 70), 2))
            painter.setBrush(QBrush(QColor(255, 190, 0)))
            painter.drawEllipse(p2, 5.0, 5.0)

            # Target 2 Contact Arc in shadow space
            sid = rec["sensor_id"]
            half_fov = math.radians(5.625)
            # Find angle
            beam_ang = math.atan2(t2_y - self.world_to_screen(0, 0).y(), t2_x - self.world_to_screen(0, 0).x())
            # Badge labels
            painter.setFont(QFont("Consolas", 8, QFont.Bold))
            painter.setPen(QPen(QColor(0, 255, 200)))
            painter.drawText(int(p1.x() + 8), int(p1.y() - 4), f"T1: {rec['t1_dist_mm']}mm")

            painter.setPen(QPen(QColor(255, 200, 30)))
            painter.drawText(int(p2.x() + 8), int(p2.y() - 4), f"T2: {rec['t2_dist_mm']}mm (Both Obstacles)")

    def _draw_vehicle_chassis(self, painter: QPainter):
        """Draws authoritative Webots robot vehicle chassis footprint (150x200mm) with wheels."""
        car_w = 0.150
        car_l = 0.200

        # Chassis outer boundary rounded rect
        c_tl = self.world_to_screen(-car_w / 2.0, car_l / 2.0)
        c_br = self.world_to_screen(car_w / 2.0, -car_l / 2.0)
        body_rect = QRectF(c_tl.x(), c_tl.y(), c_br.x() - c_tl.x(), c_br.y() - c_tl.y())
        painter.setPen(QPen(QColor(0, 229, 255, 220), 1.8))
        painter.setBrush(QBrush(QColor(18, 30, 45, 200)))
        painter.drawRoundedRect(body_rect, 4, 4)

        # Wheels matching Webots physical positions
        wheel_pos = [
            (-0.0411, +0.0625),   # FL (steered)
            (+0.0411, +0.0625),   # FR (steered)
            (-0.0500, -0.0700),   # RL (fixed)
            (+0.0500, -0.0700),   # RR (fixed)
        ]
        painter.setPen(QPen(QColor(255, 215, 64, 240), 1.5))
        painter.setBrush(QBrush(QColor(40, 40, 40, 240)))
        wheel_w_m = 0.024
        wheel_l_m = 0.050
        for wx, wy in wheel_pos:
            wtl = self.world_to_screen(wx - wheel_w_m / 2.0, wy + wheel_l_m / 2.0)
            wbr = self.world_to_screen(wx + wheel_w_m / 2.0, wy - wheel_l_m / 2.0)
            wrect = QRectF(wtl.x(), wtl.y(), wbr.x() - wtl.x(), wbr.y() - wtl.y())
            painter.drawRoundedRect(wrect, 2, 2)

        # Front bumper line at y = 0.092m
        p_b1 = self.world_to_screen(-0.075, 0.092)
        p_b2 = self.world_to_screen(+0.075, 0.092)
        painter.setPen(QPen(QColor(0, 255, 200, 200), 2.0))
        painter.drawLine(p_b1, p_b2)

        # Forward nose heading arrow
        p_origin = self.world_to_screen(0.0, 0.0)
        p_nose = self.world_to_screen(0.0, car_l / 2.0 + 0.04)
        painter.setPen(QPen(QColor(0, 255, 200, 240), 2.0))
        painter.drawLine(p_origin, p_nose)

        # Center origin dot
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(0, 255, 200)))
        painter.drawEllipse(p_origin, 3.5, 3.5)

    def _draw_sensor_modules(self, painter: QPainter):
        """Draws small badges at each sensor mounting position."""
        drawn_sids = set()
        for beam in self.cm.all_beam_records:
            sid = beam["sensor_id"]
            if sid in drawn_sids:
                continue
            drawn_sids.add(sid)

            sx, sy = beam["x0"], beam["y0"]
            ps = self.world_to_screen(sx, sy)

            # Sensor base block
            painter.setPen(QPen(QColor(255, 255, 255, 180), 1))
            painter.setBrush(QBrush(QColor(30, 45, 65)))
            painter.drawRect(QRectF(ps.x() - 6, ps.y() - 6, 12, 12))

            # Sensor ID badge
            painter.setPen(QPen(QColor(0, 255, 200), 1))
            painter.setFont(QFont("Segoe UI", 7, QFont.Bold))
            painter.drawText(int(ps.x() - 4), int(ps.y() + 4), str(sid))


# =============================================================================
# MULTI-TARGET INSPECTOR HUD PANEL
# =============================================================================

class MultiTargetInspector(QGroupBox):
    """
    Dedicated dashboard panel listing zones where 2 targets are actively detected.
    Highlights that both are stamped as obstacles, with clearing limited to min(d1, d2).
    """

    def __init__(self, parent=None):
        super().__init__("🎯 DUAL-TARGET ZONE INSPECTOR", parent)
        self.setStyleSheet("""
            QGroupBox {
                font-family: 'Segoe UI', 'Consolas', sans-serif;
                font-size: 11px;
                font-weight: bold;
                color: #ffaa00;
                border: 1px solid #332810;
                border-radius: 6px;
                margin-top: 10px;
                padding-top: 14px;
                background-color: #0f1218;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # Rule banner
        self.lbl_rule = QLabel("Rule: Ray clears to min(T1, T2) -> BOTH T1 & T2 stamped as obstacles.")
        self.lbl_rule.setStyleSheet("color: #8b949e; font-size: 10px; font-style: italic;")
        self.lbl_rule.setWordWrap(True)
        layout.addWidget(self.lbl_rule)

        # Summary count badge
        self.lbl_count = QLabel("Active Multi-Target Zones: 0")
        self.lbl_count.setStyleSheet("font-size: 12px; font-weight: bold; color: #00ffcc;")
        layout.addWidget(self.lbl_count)

        # Scroll area for multi-target cards
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setStyleSheet("background: transparent; border: none;")
        self.card_container = QWidget()
        self.card_layout = QVBoxLayout(self.card_container)
        self.card_layout.setContentsMargins(0, 0, 0, 0)
        self.card_layout.setSpacing(6)
        self.scroll.setWidget(self.card_container)
        layout.addWidget(self.scroll)

        # Pre-allocated card pool (up to 16 columns) to eliminate runtime widget destruction/creation
        self.empty_lbl = QLabel("No zones currently reporting multiple targets.")
        self.empty_lbl.setStyleSheet("color: #8b949e; font-size: 11px; padding: 10px;")
        self.empty_lbl.setAlignment(Qt.AlignCenter)
        self.card_layout.addWidget(self.empty_lbl)

        self.cards_pool = []
        for _ in range(16):
            card = QFrame()
            card.setStyleSheet("""
                QFrame {
                    background-color: #141a24;
                    border: 1px solid #ffaa00;
                    border-radius: 5px;
                    padding: 4px;
                }
            """)
            c_lay = QVBoxLayout(card)
            c_lay.setContentsMargins(6, 4, 6, 4)
            c_lay.setSpacing(2)

            header = QLabel("")
            header.setStyleSheet("color: #ffaa00; font-size: 11px;")
            t1_lbl = QLabel("")
            t1_lbl.setStyleSheet("color: #e6edf3; font-size: 10px;")
            t2_lbl = QLabel("")
            t2_lbl.setStyleSheet("color: #e6edf3; font-size: 10px;")
            clear_lbl = QLabel("")
            clear_lbl.setStyleSheet("color: #79c0ff; font-size: 9px;")

            c_lay.addWidget(header)
            c_lay.addWidget(t1_lbl)
            c_lay.addWidget(t2_lbl)
            c_lay.addWidget(clear_lbl)
            card.hide()
            self.card_layout.addWidget(card)

            self.cards_pool.append({
                "card": card,
                "header": header,
                "t1": t1_lbl,
                "t2": t2_lbl,
                "clear": clear_lbl
            })

        self.card_layout.addStretch()

    def update_records(self, records: List[dict]):
        self.lbl_count.setText(f"Active Multi-Target Zones: {len(records)}")

        if not records:
            self.empty_lbl.show()
            for c in self.cards_pool:
                c["card"].hide()
            return

        self.empty_lbl.hide()
        for i, c in enumerate(self.cards_pool):
            if i < len(records):
                r = records[i]
                c["header"].setText(f"<b>{r['sensor_name']}</b> | Sector Column {r['col']}")
                c["t1"].setText(f"• <b>Target 1 (Closest):</b> <font color='#00ffcc'>{r['t1_dist_mm']} mm</font> ➜ Obstacle + Ray Limit")
                c["t2"].setText(f"• <b>Target 2 (Farther):</b> <font color='#ffaa00'>{r['t2_dist_mm']} mm</font> ➜ <b>Also Obstacle</b> (Δ={r['sep_mm']}mm)")
                c["clear"].setText(f"• <b>Ray Cleared:</b> 0 to {r['cleared_to_mm']} mm (Shadow space protected)")
                c["card"].show()
            else:
                c["card"].hide()


# =============================================================================
# 4-SENSOR LIVE STATUS HUD PANEL
# =============================================================================

class QuadSensorHUD(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("📡 4X TOF LIVE SENSOR RIG", parent)
        self.setStyleSheet("""
            QGroupBox {
                font-family: 'Segoe UI', 'Consolas', sans-serif;
                font-size: 11px;
                font-weight: bold;
                color: #58a6ff;
                border: 1px solid #1c2636;
                border-radius: 6px;
                margin-top: 10px;
                padding-top: 14px;
                background-color: #0b0e14;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
        """)

        layout = QGridLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        self.cards = {}
        for i in range(1, 5):
            card = QFrame()
            card.setStyleSheet("""
                QFrame {
                    background-color: #111620;
                    border: 1px solid #202a3a;
                    border-radius: 4px;
                    padding: 4px;
                }
            """)
            c_lay = QVBoxLayout(card)
            c_lay.setContentsMargins(6, 4, 6, 4)
            c_lay.setSpacing(2)

            TITLES = {
                1: "ToF 1 (FL Center)",
                2: "ToF 2 (FL Flank)",
                3: "ToF 3 (FR Center)",
                4: "ToF 4 (FR Flank)"
            }
            title = QLabel(TITLES.get(i, f"ToF {i}"))
            title.setStyleSheet("font-size: 11px; font-weight: bold; color: #e6edf3;")
            status = QLabel("OFFLINE")
            status.setStyleSheet("font-size: 9px; font-weight: bold; color: #ff4455;")
            hz_lbl = QLabel("0.0 Hz")
            hz_lbl.setStyleSheet("font-size: 10px; color: #8b949e;")
            temp_lbl = QLabel("Temp: -- °C")
            temp_lbl.setStyleSheet("font-size: 9px; color: #8b949e;")

            c_lay.addWidget(title)
            c_lay.addWidget(status)
            c_lay.addWidget(hz_lbl)
            c_lay.addWidget(temp_lbl)

            row = (i - 1) // 2
            col = (i - 1) % 2
            layout.addWidget(card, row, col)

            self.cards[i] = {
                "frame": card,
                "status": status,
                "hz": hz_lbl,
                "temp": temp_lbl
            }

    def update_sensor_state(self, sid: int, is_active: bool, hz: float, temp_c: int, flags: int = 0):
        c = self.cards.get(sid)
        if not c:
            return

        if is_active:
            if flags & 0x01:
                c["status"].setText("ONLINE ⚠️ E-STOP (<80mm)")
                c["status"].setStyleSheet("font-size: 9px; font-weight: bold; color: #ff3344;")
                c["frame"].setStyleSheet("background-color: #2b0d14; border: 1px solid #da3633; border-radius: 4px;")
            elif flags & 0x02:
                c["status"].setText("ONLINE ⚠️ CONTACT (SPAD)")
                c["status"].setStyleSheet("font-size: 9px; font-weight: bold; color: #ffaa00;")
                c["frame"].setStyleSheet("background-color: #261c0d; border: 1px solid #d29922; border-radius: 4px;")
            else:
                c["status"].setText("ONLINE")
                c["status"].setStyleSheet("font-size: 9px; font-weight: bold; color: #00ffcc;")
                c["frame"].setStyleSheet("background-color: #101a26; border: 1px solid #1c3c54; border-radius: 4px;")

            c["hz"].setText(f"{hz:.1f} Hz")
            c["hz"].setStyleSheet("font-size: 10px; font-weight: bold; color: #58a6ff;")
            c["temp"].setText(f"Temp: {temp_c} °C")
            c["temp"].setStyleSheet("font-size: 9px; color: #8b949e;")
        else:
            c["status"].setText("OFFLINE")
            c["status"].setStyleSheet("font-size: 9px; font-weight: bold; color: #ff3344;")
            c["hz"].setText("0.0 Hz")
            c["hz"].setStyleSheet("font-size: 10px; color: #8b949e;")
            c["temp"].setText("Temp: -- °C")
            c["temp"].setStyleSheet("font-size: 9px; color: #8b949e;")
            c["frame"].setStyleSheet("background-color: #111620; border: 1px solid #202a3a; border-radius: 4px;")


# =============================================================================
# MAIN WINDOW DASHBOARD
# =============================================================================

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FOLKRACE — 4X TOF 2D COSTMAP (WEBOTS GRID)")
        self.resize(1380, 860)
        self.setMinimumSize(1040, 680)

        # Webots Criss-Cross 16-Channel Front Rig (only_front_tofs.proto)
        self.sensor_rig = list(WEBOTS_CRISS_CROSS_RIG)
        self.costmap_engine = CostmapEngine(res=0.015, r_inflation=0.075)

        self.serial_engine = QuadSerialEngine(port="COM7", baud=921600)
        self.serial_engine.connection_changed.connect(self._on_connection_changed)

        # Persistent sensor cache to eliminate phase-jitter & beam dropouts
        self.cached_packets: Dict[int, Optional[dict]] = {1: None, 2: None, 3: None, 4: None}
        self.cached_packet_time: Dict[int, float] = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
        self._last_inspector_update = 0.0

        self._build_ui()

        # High-rate render timer (60 Hz)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_tick)
        self.timer.start(16)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root_lay = QVBoxLayout(central)
        root_lay.setContentsMargins(10, 10, 10, 10)
        root_lay.setSpacing(8)

        # 1. Top Action Toolbar
        top_bar = self._build_top_toolbar()
        root_lay.addWidget(top_bar)

        # 2. Main Workspace Splitter (Left: Costmap Canvas, Right: Inspector & Controls)
        splitter = QSplitter(Qt.Horizontal)
        splitter.setStyleSheet("QSplitter::handle { background-color: #1c2636; width: 3px; }")

        # Center Costmap Canvas container
        canvas_container = QWidget()
        can_lay = QVBoxLayout(canvas_container)
        can_lay.setContentsMargins(0, 0, 0, 0)
        can_lay.setSpacing(4)

        self.canvas = CostmapCanvas(self.costmap_engine)
        self.canvas.cursor_moved.connect(self._on_canvas_cursor_moved)
        can_lay.addWidget(self.canvas)

        # Bottom Canvas Status Bar
        self.lbl_cursor_info = QLabel("Cursor: (X: 0.00m, Y: 0.00m) | UNKNOWN")
        self.lbl_cursor_info.setStyleSheet("color: #778899; font-family: 'Consolas'; font-size: 10px;")
        can_lay.addWidget(self.lbl_cursor_info)

        splitter.addWidget(canvas_container)

        # Right Dock Panel
        right_panel = QWidget()
        right_panel.setFixedWidth(380)
        right_lay = QVBoxLayout(right_panel)
        right_lay.setContentsMargins(4, 0, 0, 0)
        right_lay.setSpacing(8)

        # Sensor HUD
        self.sensor_hud = QuadSensorHUD()
        right_lay.addWidget(self.sensor_hud)

        # Dual-Target Zone Inspector
        self.multi_inspector = MultiTargetInspector()
        right_lay.addWidget(self.multi_inspector, stretch=1)

        # Costmap Controls Box
        controls_box = self._build_controls_box()
        right_lay.addWidget(controls_box)

        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)

        root_lay.addWidget(splitter, stretch=1)

    def _build_top_toolbar(self) -> QFrame:
        bar = QFrame()
        bar.setStyleSheet("""
            QFrame {
                background-color: #0d1117;
                border: 1px solid #1e2633;
                border-radius: 6px;
                padding: 4px;
            }
        """)
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(8, 4, 8, 4)
        lay.setSpacing(10)

        # Title badge
        title_lbl = QLabel("🏎️ FOLKRACE 4X TOF COSTMAP")
        title_lbl.setStyleSheet("font-size: 13px; font-weight: bold; color: #00ffcc;")
        lay.addWidget(title_lbl)

        lay.addSpacing(10)

        # Port Selector
        lbl_port = QLabel("Port:")
        lbl_port.setStyleSheet("color: #e6edf3; font-weight: bold; font-size: 11px;")
        lay.addWidget(lbl_port)
        self.cb_port = QComboBox()
        self.cb_port.setMinimumWidth(95)
        self._refresh_ports()
        lay.addWidget(self.cb_port)

        btn_refresh = QPushButton("⟳")
        btn_refresh.setFixedWidth(28)
        btn_refresh.setToolTip("Refresh COM Ports")
        btn_refresh.clicked.connect(self._refresh_ports)
        lay.addWidget(btn_refresh)

        # Baud Selector
        lbl_baud = QLabel("Baud:")
        lbl_baud.setStyleSheet("color: #e6edf3; font-weight: bold; font-size: 11px;")
        lay.addWidget(lbl_baud)
        self.cb_baud = QComboBox()
        self.cb_baud.addItems(["921600", "2000000", "115200"])
        lay.addWidget(self.cb_baud)

        # Connect / Disconnect Button
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setStyleSheet("background-color: #1f6feb; color: white; font-weight: bold; padding: 4px 12px;")
        self.btn_connect.clicked.connect(self._toggle_connection)
        lay.addWidget(self.btn_connect)

        # Reset Sensor Button
        btn_reset = QPushButton("Reset Sensors")
        btn_reset.setStyleSheet("background-color: #30363d; color: #e6edf3; padding: 4px 8px;")
        btn_reset.clicked.connect(lambda: self.serial_engine.send_command("RESET"))
        lay.addWidget(btn_reset)

        lay.addStretch()

        # Connection status badge
        self.lbl_status = QLabel("Ready (Real ToF Hardware)")
        self.lbl_status.setStyleSheet("color: #8b949e; font-size: 11px;")
        lay.addWidget(self.lbl_status)

        return bar

    def _build_controls_box(self) -> QGroupBox:
        box = QGroupBox("⚙️ COSTMAP DISPLAY CONTROLS")
        box.setStyleSheet("""
            QGroupBox {
                font-family: 'Segoe UI', 'Consolas', sans-serif;
                font-size: 11px;
                font-weight: bold;
                color: #58a6ff;
                border: 1px solid #1c2636;
                border-radius: 6px;
                margin-top: 10px;
                padding-top: 14px;
                background-color: #0b0e14;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
        """)
        lay = QVBoxLayout(box)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(6)

        # Webots Criss-Cross Rig Badge (Single Authoritative Layout)
        rig_badge = QFrame()
        rig_badge.setStyleSheet("background-color: #111a24; border: 1px solid #1f3d54; border-radius: 5px; padding: 4px;")
        rb_lay = QVBoxLayout(rig_badge)
        rb_lay.setContentsMargins(6, 4, 6, 4)
        rb_lay.setSpacing(2)
        lbl_rig_title = QLabel("📐 WEBOTS CRISS-CROSS RIG (16-CH)")
        lbl_rig_title.setStyleSheet("color: #00ffcc; font-weight: bold; font-size: 11px;")
        lbl_rig_desc = QLabel("FL (-58mm, 92mm) ⇄ FR (+58mm, 92mm) Cross-Beams")
        lbl_rig_desc.setStyleSheet("color: #8b949e; font-size: 9px;")
        rb_lay.addWidget(lbl_rig_title)
        rb_lay.addWidget(lbl_rig_desc)
        lay.addWidget(rig_badge)

        # Inflation Slider
        inf_lay = QHBoxLayout()
        self.lbl_inf = QLabel(f"Safety Margin: {int(self.costmap_engine.r_inflation * 1000)} mm")
        self.lbl_inf.setStyleSheet("color: #e6edf3; font-weight: bold; font-size: 11px;")
        self.slider_inf = QSlider(Qt.Horizontal)
        self.slider_inf.setRange(20, 150)
        self.slider_inf.setValue(int(self.costmap_engine.r_inflation * 1000))
        self.slider_inf.valueChanged.connect(self._on_inflation_changed)
        inf_lay.addWidget(self.lbl_inf)
        inf_lay.addWidget(self.slider_inf)
        lay.addLayout(inf_lay)

        # Toggles Grid
        t_lay = QGridLayout()
        self.chk_costmap = QCheckBox("Show Costmap Image")
        self.chk_costmap.setChecked(True)
        self.chk_costmap.toggled.connect(lambda v: setattr(self.canvas, "show_costmap", v))

        self.chk_rays = QCheckBox("Show Conical Rays & Arcs")
        self.chk_rays.setChecked(True)
        self.chk_rays.toggled.connect(lambda v: setattr(self.canvas, "show_rays", v))

        self.chk_inflation = QCheckBox("Show C-Space Margin")
        self.chk_inflation.setChecked(True)
        self.chk_inflation.toggled.connect(lambda v: setattr(self.canvas, "show_inflation", v))

        self.chk_markers = QCheckBox("Show Dual-Target Tags")
        self.chk_markers.setChecked(True)
        self.chk_markers.toggled.connect(lambda v: setattr(self.canvas, "show_multi_markers", v))

        self.chk_grid = QCheckBox("Show Webots Grid & Rings")
        self.chk_grid.setChecked(True)
        self.chk_grid.toggled.connect(lambda v: setattr(self.canvas, "show_grid", v))

        self.chk_chassis = QCheckBox("Show Chassis Footprint")
        self.chk_chassis.setChecked(True)
        self.chk_chassis.toggled.connect(lambda v: setattr(self.canvas, "show_chassis", v))

        t_lay.addWidget(self.chk_costmap, 0, 0)
        t_lay.addWidget(self.chk_rays, 0, 1)
        t_lay.addWidget(self.chk_inflation, 1, 0)
        t_lay.addWidget(self.chk_markers, 1, 1)
        t_lay.addWidget(self.chk_grid, 2, 0)
        t_lay.addWidget(self.chk_chassis, 2, 1)
        lay.addLayout(t_lay)

        btn_reset_view = QPushButton("Reset Camera View")
        btn_reset_view.clicked.connect(self.canvas.reset_view)
        lay.addWidget(btn_reset_view)

        return box

    def _refresh_ports(self):
        self.cb_port.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.cb_port.addItem(p.device)
        if self.cb_port.count() == 0:
            self.cb_port.addItem("COM7")

    def _toggle_connection(self):
        if self.serial_engine.isRunning():
            self.serial_engine.stop()
            self.btn_connect.setText("Connect")
            self.btn_connect.setStyleSheet("background-color: #1f6feb; color: white; font-weight: bold; padding: 4px 12px;")
        else:
            self.serial_engine.port = self.cb_port.currentText()
            self.serial_engine.baud = int(self.cb_baud.currentText())
            self.serial_engine.start()
            self.btn_connect.setText("Disconnect")
            self.btn_connect.setStyleSheet("background-color: #da3633; color: white; font-weight: bold; padding: 4px 12px;")

    def _on_connection_changed(self, connected: bool, msg: str):
        self.lbl_status.setText(msg)
        self.lbl_status.setStyleSheet("color: #00ffcc;" if connected else "color: #ff3344;")

    def _on_inflation_changed(self, val_mm: int):
        self.lbl_inf.setText(f"Safety Margin: {val_mm} mm")
        self.costmap_engine.set_parameters(res=self.costmap_engine.res, r_inflation=val_mm / 1000.0)
        self.canvas.update()

    def _on_canvas_cursor_moved(self, wx: float, wy: float, state_str: str):
        self.lbl_cursor_info.setText(f"Cursor: (X: {wx:+.2f}m, Y: {wy:+.2f}m) | {state_str}")

    def _on_tick(self):
        """Called at 60 Hz to process incoming sensor frames and refresh canvas."""
        now = time.perf_counter()
        has_new_data = False

        for sid in range(1, 5):
            pkt = self.serial_engine.take_latest_frame(sid)
            if pkt:
                self.cached_packets[sid] = pkt
                self.cached_packet_time[sid] = now
                has_new_data = True

            is_act = self.serial_engine.is_sensor_active(sid)
            hz = self.serial_engine.get_sensor_hz(sid)
            st = self.serial_engine.sensor_stats[sid]
            temp = st.get("temp_c", 0)
            flags = st.get("flags", 0)
            self.sensor_hud.update_sensor_state(sid, is_act, hz, temp, flags)

        # Assemble persistent fresh frames (<0.15s) across all active sensors
        active_packets = {}
        for sid in range(1, 5):
            if self.cached_packets[sid] and (now - self.cached_packet_time[sid] < 0.15):
                active_packets[sid] = self.cached_packets[sid]

        # Run Costmap Processing if any new data arrived or fresh sensors are present
        if has_new_data and active_packets:
            self.costmap_engine.process_sensor_frames(self.sensor_rig, active_packets)
            # Throttle inspector updates to 10 Hz to prevent UI thread memory churn
            if (now - self._last_inspector_update) > 0.10:
                self._last_inspector_update = now
                self.multi_inspector.update_records(self.costmap_engine.active_multi_targets)
            self.canvas.update()

    def closeEvent(self, event):
        self.serial_engine.stop()
        super().closeEvent(event)


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # Dark fusion palette fallback
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(11, 14, 20))
    palette.setColor(QPalette.WindowText, QColor(230, 237, 243))
    palette.setColor(QPalette.Base, QColor(16, 22, 32))
    palette.setColor(QPalette.AlternateBase, QColor(22, 27, 34))
    palette.setColor(QPalette.ToolTipBase, QColor(22, 27, 34))
    palette.setColor(QPalette.ToolTipText, QColor(230, 237, 243))
    palette.setColor(QPalette.Text, QColor(230, 237, 243))
    palette.setColor(QPalette.Button, QColor(33, 38, 45))
    palette.setColor(QPalette.ButtonText, QColor(230, 237, 243))
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Highlight, QColor(31, 111, 235))
    palette.setColor(QPalette.HighlightedText, Qt.white)
    app.setPalette(palette)
    app.setStyleSheet(DARK_THEME_QSS)

    win = MainWindow()
    win.show()
    sys.exit(app.exec_())

