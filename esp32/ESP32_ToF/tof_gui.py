#!/usr/bin/env python3
"""
===============================================================================
FOLKRACE — ESP32 QUAD VL53L8CX 4-ZONE MULTI-TARGET TELEMETRY VIEWER
===============================================================================
Full-screen single-display dashboard showing complete multi-target raw & resolved
telemetry for the dedicated 4 zones per ToF sensor streamed to the Raspberry Pi 4:
  - Sensor 1 (Left Side / Right-angled) : Z1,  Z5,  Z9,  Z13 (49.1°..82.9° L)
  - Sensor 2 (Left Front)               : Z14, Z10, Z6,  Z2  (4.1°..37.9° L)
  - Sensor 4 (Right Front)              : Z13, Z9,  Z5,  Z1  (4.1°..37.9° R)
  - Sensor 3 (Right Side / Left-angled) : Z2,  Z6,  Z10, Z14 (49.1°..82.9° R)

Per-Zone Data Displayed:
  - Resolved Navigation Distance (mm & m) + Proximity Depth Meter
  - Target Selection Badge (T1 Closer, T2 Closer, Contact Error, Open Space)
  - Main Target (Target 0): Distance (mm) + ST Table 4 Status Pill
  - 2nd Target  (Target 1): Distance (mm) + ST Table 4 Status Pill
  - Active SPAD Count Meter & Lens Contact / Saturation Alert (1024 SPADs)
===============================================================================
"""

import sys
import os
import time
import struct
import threading
import subprocess
from collections import deque

import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QFrame, QSizePolicy,
    QProgressBar, QDialog, QTextEdit
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt5.QtGui import QColor, QFont

if hasattr(Qt, "HighDpiScaleFactorRoundingPolicy"):
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )

# -----------------------------------------------------------------------------
# PROTOCOL DEFINITIONS
# -----------------------------------------------------------------------------
PACKET_MAGIC = b"\xaa\x55"

# 52-byte multi-target 4-zone frame
PACKET_52_FMT = "<2sBII4H4H4H4B4B4HB"
PACKET_52_SIZE = struct.calcsize(PACKET_52_FMT)

# 20-byte legacy distance-only frame
PACKET_20_FMT = "<2sBII4HB"
PACKET_20_SIZE = struct.calcsize(PACKET_20_FMT)

# -----------------------------------------------------------------------------
# ST UM3109 TABLE 4 STATUS MAP
# -----------------------------------------------------------------------------
STATUS_MAP = {
    0: ("Not Updated", "#64748B", False),
    1: ("Low Signal (SPAD)", "#EF4444", False),
    2: ("Target Phase", "#A855F7", False),
    3: ("Sigma Too High", "#F97316", False),
    4: ("Consistency Fail", "#EC4899", False),
    5: ("Valid Range (100%)", "#22C55E", True),
    6: ("Wrap Around (50%)", "#06B6D4", True),
    7: ("Rate Consistency", "#EF4444", False),
    8: ("Low Signal (Target)", "#F43F5E", False),
    9: ("Large Pulse (50%)", "#EAB308", True),
    10: ("Valid (No Prev)", "#14B8A6", True),
    11: ("Measurement Fail", "#DC2626", False),
    12: ("Blurred (Sharpener)", "#8B5CF6", False),
    13: ("Secondary Inconsistent", "#A1A1AA", False),
    255: ("No Target", "#475569", False),
}

VALID_STATUS_CODES = {5, 6, 9, 10}

def get_status_info(st: int):
    return STATUS_MAP.get(st, (f"Status {st}", "#64748B", False))

# -----------------------------------------------------------------------------
# SENSOR DEFINITIONS & ORDER (Spatial Left-to-Right: 1 -> 2 -> 4 -> 3)
# -----------------------------------------------------------------------------
SENSOR_ORDER = [1, 2, 4, 3]

SENSOR_META = {
    1: {
        "name": "ToF 1 • Left Side",
        "role": "49.1°..82.9° Left",
        "zones": [1, 5, 9, 13],
        "angles_deg": [49.1, 60.4, 71.6, 82.9],
        "labels": ["Z1 (49.1°L)", "Z5 (60.4°L)", "Z9 (71.6°L)", "Z13 (82.9°L)"],
        "color": "#38BDF8",  # Sky Cyan
        "pins": "CS: 7 | INT: 6 | LPn: 5 | MISO: 4",
    },
    2: {
        "name": "ToF 2 • Left Front",
        "role": "4.1°..37.9° Left",
        "zones": [14, 10, 6, 2],
        "angles_deg": [4.1, 15.4, 26.6, 37.9],
        "labels": ["Z14 (4.1°L)", "Z10 (15.4°L)", "Z6 (26.6°L)", "Z2 (37.9°L)"],
        "color": "#34D399",  # Emerald
        "pins": "CS: 12 | INT: 11 | LPn: 10 | MISO: 9",
    },
    4: {
        "name": "ToF 4 • Right Front",
        "role": "4.1°..37.9° Right",
        "zones": [13, 9, 5, 1],
        "angles_deg": [-4.1, -15.4, -26.6, -37.9],
        "labels": ["Z13 (4.1°R)", "Z9 (15.4°R)", "Z5 (26.6°R)", "Z1 (37.9°R)"],
        "color": "#A855F7",  # Purple
        "pins": "CS: 17 | INT: 18 | LPn: 16 | MISO: 15",
    },
    3: {
        "name": "ToF 3 • Right Side",
        "role": "49.1°..82.9° Right",
        "zones": [2, 6, 10, 14],
        "angles_deg": [-49.1, -60.4, -71.6, -82.9],
        "labels": ["Z2 (49.1°R)", "Z6 (60.4°R)", "Z10 (71.6°R)", "Z14 (82.9°R)"],
        "color": "#F59E0B",  # Amber
        "pins": "CS: 41 | INT: 42 | LPn: 2 | MISO: 1",
    },
}

def distance_color(mm: int, is_valid: bool = True) -> QColor:
    """Color code distance based on proximity safety."""
    if mm <= 30:
        return QColor(220, 38, 38)   # <= 30 mm contact boundary
    if mm >= 3000 and not is_valid:
        return QColor(16, 185, 129)  # 3000 mm clear open space
    if mm < 200:
        return QColor(239, 68, 68)   # Red (<20cm)
    if mm < 500:
        return QColor(249, 115, 22)  # Orange (<50cm)
    if mm < 1000:
        return QColor(234, 179, 8)   # Amber (<1m)
    if mm < 2000:
        return QColor(6, 182, 212)   # Cyan (<2m)
    return QColor(34, 197, 94)       # Green (>2m)


# =============================================================================
# SERIAL RECEIVER THREAD
# =============================================================================
class TelemetryReceiverThread(QThread):
    connection_changed = pyqtSignal(bool, str)

    def __init__(self, port="COM7", baud=921600):
        super().__init__()
        self.port = port
        self.baud = baud
        self._running = False
        self.ser = None
        self._lock = threading.Lock()
        self.reset_state()

    def reset_state(self):
        with self._lock:
            self.latest_frames = {1: None, 2: None, 3: None, 4: None}
            self.stats = {
                i: {
                    "rx_count": 0,
                    "lost_count": 0,
                    "last_seq": None,
                    "last_rx_time": 0.0,
                    "ts_history": deque(maxlen=60),
                }
                for i in (1, 2, 3, 4)
            }
            self.checksum_errors = 0
            self.is_52_byte = False

    def get_snapshot(self):
        now = time.perf_counter()
        with self._lock:
            snap = {}
            for i in (1, 2, 3, 4):
                st = self.stats[i]
                th = st["ts_history"]
                hz = 0.0
                if len(th) >= 2 and (now - st["last_rx_time"]) < 0.8:
                    span = th[-1] - th[0]
                    if span > 0:
                        hz = (len(th) - 1) / span

                age_ms = (now - st["last_rx_time"]) * 1000.0 if st["last_rx_time"] else None
                active = bool(st["last_rx_time"] and (now - st["last_rx_time"]) < 0.5)

                snap[i] = {
                    "frame": self.latest_frames[i],
                    "rx_count": st["rx_count"],
                    "lost_count": st["lost_count"],
                    "hz": hz,
                    "age_ms": age_ms,
                    "active": active,
                }
            return snap, self.checksum_errors, self.is_52_byte

    def stop(self):
        self._running = False
        self.wait(500)

    def pulse_reset(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.dtr = True
                self.ser.rts = True
                time.sleep(0.08)
                self.ser.dtr = False
                self.ser.rts = False
            except Exception:
                pass

    def send_bytes(self, data: bytes):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data)
                self.ser.flush()
            except Exception:
                pass

    def run(self):
        try:
            self.ser = serial.Serial()
            self.ser.port = self.port
            self.ser.baudrate = self.baud
            self.ser.timeout = 0.05
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.open()
            try:
                self.ser.set_buffer_size(rx_size=1 << 20, tx_size=1 << 16)
            except Exception:
                pass
            self.reset_state()
            self._running = True
            self.connection_changed.emit(True, f"Connected: {self.port} @ {self.baud} baud")
        except Exception as e:
            self._running = False
            self.connection_changed.emit(False, f"Connection Failed: {e}")
            return

        buf = bytearray()

        while self._running:
            try:
                waiting = self.ser.in_waiting if (self.ser and self.ser.is_open) else 0
                chunk = self.ser.read(waiting if waiting > 0 else 1)
                if not chunk:
                    continue
                buf.extend(chunk)

                while True:
                    idx = buf.find(PACKET_MAGIC)
                    if idx == -1:
                        if len(buf) > 1:
                            del buf[:-1]
                        break

                    if idx > 0:
                        del buf[:idx]

                    if len(buf) < 3:
                        break

                    # Check for 52-byte frame first
                    if len(buf) >= PACKET_52_SIZE:
                        pkt_52 = bytes(buf[:PACKET_52_SIZE])
                        chk_calc = 0
                        for b in pkt_52[:-1]:
                            chk_calc ^= b

                        if chk_calc == pkt_52[-1] and 1 <= pkt_52[2] <= 4:
                            unp = struct.unpack(PACKET_52_FMT, pkt_52)
                            sensor_id = unp[1]
                            seq = unp[2]
                            ts_ms = unp[3]
                            dist_mm = list(unp[4:8])
                            target0_mm = list(unp[8:12])
                            target1_mm = list(unp[12:16])
                            target0_st = list(unp[16:20])
                            target1_st = list(unp[20:24])
                            spad_cnt = list(unp[24:28])

                            self._store_packet(
                                sensor_id, seq, ts_ms, dist_mm,
                                target0_mm, target1_mm, target0_st, target1_st, spad_cnt,
                                is_52=True
                            )
                            del buf[:PACKET_52_SIZE]
                            continue

                    # Fallback check for 20-byte legacy frame
                    if len(buf) >= PACKET_20_SIZE:
                        pkt_20 = bytes(buf[:PACKET_20_SIZE])
                        chk_calc = 0
                        for b in pkt_20[:-1]:
                            chk_calc ^= b

                        if chk_calc == pkt_20[-1] and 1 <= pkt_20[2] <= 4:
                            unp = struct.unpack(PACKET_20_FMT, pkt_20)
                            sensor_id = unp[1]
                            seq = unp[2]
                            ts_ms = unp[3]
                            dist_mm = list(unp[4:8])

                            self._store_packet(
                                sensor_id, seq, ts_ms, dist_mm,
                                target0_mm=dist_mm, target1_mm=[0]*4,
                                target0_st=[5]*4, target1_st=[255]*4,
                                spad_cnt=[1000]*4, is_52=False
                            )
                            del buf[:PACKET_20_SIZE]
                            continue

                    if len(buf) >= PACKET_52_SIZE:
                        with self._lock:
                            self.checksum_errors += 1
                        del buf[:2]
                        continue
                    else:
                        break

            except Exception as e:
                if self._running:
                    self.connection_changed.emit(False, f"Serial Error: {e}")
                break

        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _store_packet(self, sensor_id, seq, ts_ms, dist_mm,
                      target0_mm, target1_mm, target0_st, target1_st, spad_cnt, is_52):
        now = time.perf_counter()
        with self._lock:
            if sensor_id not in self.stats:
                return
            st = self.stats[sensor_id]
            st["rx_count"] += 1
            st["last_rx_time"] = now
            st["ts_history"].append(now)
            if st["last_seq"] is not None and seq > st["last_seq"] + 1:
                gap = seq - st["last_seq"] - 1
                if gap < 100000:
                    st["lost_count"] += gap
            st["last_seq"] = seq
            self.is_52_byte = is_52

            self.latest_frames[sensor_id] = {
                "seq": seq,
                "ts_ms": ts_ms,
                "dist_mm": dist_mm,
                "target0_mm": target0_mm,
                "target1_mm": target1_mm,
                "target0_st": target0_st,
                "target1_st": target1_st,
                "spad_cnt": spad_cnt,
                "is_52": is_52,
            }


# =============================================================================
# SINGLE ZONE CARD WIDGET (VERTICALLY EXPANDING)
# =============================================================================
class ZoneTelemetryCard(QFrame):
    """
    Renders all multi-target details for one single zone:
      - Zone name & angle label
      - Resolved distance (large bold readout) + Proximity Bar + Reason Badge
      - Main Target (Target 0) distance & ST status pill
      - 2nd Target  (Target 1) distance & ST status pill
      - Active SPAD count bar & lens contact alert
    """
    def __init__(self, sensor_id: int, zone_idx: int, parent=None):
        super().__init__(parent)
        self.sensor_id = sensor_id
        self.zone_idx = zone_idx
        self.meta = SENSOR_META[sensor_id]
        self.zone_id = self.meta["zones"][zone_idx]
        self.label_text = self.meta["labels"][zone_idx]

        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet("""
            ZoneTelemetryCard {
                background-color: #1E293B;
                border: 1px solid #334155;
                border-radius: 8px;
            }
            ZoneTelemetryCard:hover {
                border: 1px solid #475569;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(6)

        # Row 1: Header (Zone ID + Angle + Reason Pill)
        hdr = QHBoxLayout()
        hdr.setSpacing(6)
        self.lbl_zone = QLabel(self.label_text)
        self.lbl_zone.setStyleSheet(f"font-size: 13px; font-weight: 800; color: {self.meta['color']};")

        self.lbl_reason = QLabel("IDLE")
        self.lbl_reason.setStyleSheet("""
            font-size: 10px; font-weight: bold; color: #94A3B8;
            background-color: #0F172A; border-radius: 4px; padding: 3px 7px;
        """)
        hdr.addWidget(self.lbl_zone)
        hdr.addStretch()
        hdr.addWidget(self.lbl_reason)
        layout.addLayout(hdr)

        # Row 2: Resolved Distance (Large readout) + Distance Bar
        dist_row = QHBoxLayout()
        dist_row.setSpacing(8)

        self.lbl_main_dist = QLabel("--- mm")
        self.lbl_main_dist.setStyleSheet("font-size: 24px; font-weight: 900; color: #38BDF8; font-family: Consolas;")

        self.lbl_main_dist_m = QLabel("(--- m)")
        self.lbl_main_dist_m.setStyleSheet("font-size: 13px; font-weight: bold; color: #94A3B8; font-family: Consolas;")

        dist_row.addWidget(self.lbl_main_dist)
        dist_row.addWidget(self.lbl_main_dist_m)
        dist_row.addStretch()
        layout.addLayout(dist_row)

        # Depth Bar (0 to 3000 mm)
        self.bar_depth = QProgressBar()
        self.bar_depth.setRange(0, 3000)
        self.bar_depth.setValue(3000)
        self.bar_depth.setTextVisible(False)
        self.bar_depth.setFixedHeight(6)
        self.bar_depth.setStyleSheet("""
            QProgressBar {
                background-color: #0F172A;
                border: 1px solid #334155;
                border-radius: 3px;
            }
            QProgressBar::chunk {
                background-color: #38BDF8;
                border-radius: 2px;
            }
        """)
        layout.addWidget(self.bar_depth)

        # Row 3: Target 0 vs Target 1 Grid Box (Expands to fill vertical card space)
        targets_frame = QFrame()
        targets_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        targets_frame.setStyleSheet("""
            background-color: #0F172A;
            border-radius: 6px;
            border: 1px solid #1E293B;
        """)
        t_layout = QGridLayout(targets_frame)
        t_layout.setContentsMargins(10, 8, 10, 8)
        t_layout.setHorizontalSpacing(14)
        t_layout.setVerticalSpacing(4)

        # Col Headers
        lbl_t0_title = QLabel("TARGET 1 (MAIN)")
        lbl_t0_title.setStyleSheet("font-size: 10px; font-weight: bold; color: #64748B; letter-spacing: 0.5px;")
        lbl_t1_title = QLabel("TARGET 2")
        lbl_t1_title.setStyleSheet("font-size: 10px; font-weight: bold; color: #64748B; letter-spacing: 0.5px;")
        t_layout.addWidget(lbl_t0_title, 0, 0)
        t_layout.addWidget(lbl_t1_title, 0, 1)

        # Distances
        self.lbl_t0_dist = QLabel("--- mm")
        self.lbl_t0_dist.setStyleSheet("font-size: 16px; font-weight: 800; color: #F1F5F9; font-family: Consolas;")
        self.lbl_t1_dist = QLabel("--- mm")
        self.lbl_t1_dist.setStyleSheet("font-size: 16px; font-weight: 800; color: #F1F5F9; font-family: Consolas;")
        t_layout.addWidget(self.lbl_t0_dist, 1, 0)
        t_layout.addWidget(self.lbl_t1_dist, 1, 1)

        # Status Pills
        self.lbl_t0_status = QLabel("---")
        self.lbl_t0_status.setStyleSheet("font-size: 10px; color: #94A3B8; font-weight: bold;")
        self.lbl_t1_status = QLabel("---")
        self.lbl_t1_status.setStyleSheet("font-size: 10px; color: #94A3B8; font-weight: bold;")
        t_layout.addWidget(self.lbl_t0_status, 2, 0)
        t_layout.addWidget(self.lbl_t1_status, 2, 1)

        layout.addWidget(targets_frame, 1)

        # Row 4: SPAD Count Bar
        spad_row = QHBoxLayout()
        spad_row.setSpacing(8)
        lbl_spad_tag = QLabel("Active SPADs:")
        lbl_spad_tag.setStyleSheet("font-size: 10px; color: #94A3B8; font-weight: bold;")

        self.bar_spad = QProgressBar()
        self.bar_spad.setRange(0, 16384)
        self.bar_spad.setValue(0)
        self.bar_spad.setTextVisible(False)
        self.bar_spad.setFixedHeight(8)
        self.bar_spad.setStyleSheet("""
            QProgressBar {
                background-color: #0F172A;
                border: 1px solid #334155;
                border-radius: 4px;
            }
            QProgressBar::chunk {
                background-color: #38BDF8;
                border-radius: 3px;
            }
        """)

        self.lbl_spad_val = QLabel("---")
        self.lbl_spad_val.setStyleSheet("font-size: 11px; font-weight: bold; color: #CBD5E1; font-family: Consolas;")

        spad_row.addWidget(lbl_spad_tag)
        spad_row.addWidget(self.bar_spad, 1)
        spad_row.addWidget(self.lbl_spad_val)
        layout.addLayout(spad_row)

    def set_data(self, dist_mm: int, t0_mm: int, t1_mm: int,
                 t0_st: int, t1_st: int, spad_cnt: int, is_52: bool = True, accept_s10: bool = True):
        # 1. Resolve distance dynamically based on accept_s10 setting
        valid_statuses = {5, 6, 9, 10} if accept_s10 else {5, 6, 9}
        v0 = (t0_st in valid_statuses and t0_mm > 0)
        v1 = (t1_st in valid_statuses and t1_mm > 0)
        near0 = (t0_st != 0 and t0_st != 255 and 1 <= t0_mm <= 30)
        near1 = (t1_st != 0 and t1_st != 255 and 1 <= t1_mm <= 30)

        if is_52:
            if v0 and v1:
                dist_mm = min(t0_mm, t1_mm)
            elif v0:
                dist_mm = t0_mm
            elif v1:
                dist_mm = t1_mm
            else:
                if near0 and near1:
                    dist_mm = min(t0_mm, t1_mm)
                elif near0:
                    dist_mm = t0_mm
                elif near1:
                    dist_mm = t1_mm
                elif spad_cnt == 1024:
                    dist_mm = 30
                else:
                    dist_mm = 3000

        # Every reading below 30mm is reported as 30mm to pi
        if dist_mm < 30:
            dist_mm = 30

        m_val = dist_mm / 1000.0
        self.lbl_main_dist.setText(f"{dist_mm:d} mm")
        self.lbl_main_dist_m.setText(f"({m_val:.2f} m)")

        # Color
        c = distance_color(dist_mm)
        self.lbl_main_dist.setStyleSheet(f"font-size: 24px; font-weight: 900; color: {c.name()}; font-family: Consolas;")

        # Update depth bar
        self.bar_depth.setValue(min(3000, dist_mm))
        self.bar_depth.setStyleSheet(f"""
            QProgressBar {{
                background-color: #0F172A;
                border: 1px solid #334155;
                border-radius: 3px;
            }}
            QProgressBar::chunk {{
                background-color: {c.name()};
                border-radius: 2px;
            }}
        """)

        # 2. Reason badge
        if not is_52:
            self.lbl_reason.setText("LEGACY 20B")
            self.lbl_reason.setStyleSheet("font-size: 10px; font-weight: bold; color: #38BDF8; background-color: #0F172A; border-radius: 4px; padding: 3px 7px;")
        elif (near0 or near1) and not (v0 or v1):
            self.lbl_reason.setText("NEAR CONTACT (<30mm -> 30mm)")
            self.lbl_reason.setStyleSheet("font-size: 10px; font-weight: bold; color: #EF4444; background-color: #450A0A; border-radius: 4px; padding: 3px 7px;")
        elif spad_cnt == 1024 and not v0 and not v1:
            self.lbl_reason.setText("1024 SPAD CONTACT (30mm)")
            self.lbl_reason.setStyleSheet("font-size: 10px; font-weight: bold; color: #EF4444; background-color: #450A0A; border-radius: 4px; padding: 3px 7px;")
        elif not v0 and not v1:
            self.lbl_reason.setText("OPEN SPACE (3000mm)")
            self.lbl_reason.setStyleSheet("font-size: 10px; font-weight: bold; color: #10B981; background-color: #064E3B; border-radius: 4px; padding: 3px 7px;")
        elif v0 and v1:
            if t0_mm <= t1_mm:
                self.lbl_reason.setText("T1 CHOSEN (CLOSER)")
                self.lbl_reason.setStyleSheet("font-size: 10px; font-weight: bold; color: #22C55E; background-color: #14532D; border-radius: 4px; padding: 3px 7px;")
            else:
                self.lbl_reason.setText("T2 CHOSEN (CLOSER)")
                self.lbl_reason.setStyleSheet("font-size: 10px; font-weight: bold; color: #EAB308; background-color: #713F12; border-radius: 4px; padding: 3px 7px;")
        elif v0:
            tag = "T1 VALID (S10)" if t0_st == 10 else "T1 VALID"
            self.lbl_reason.setText(tag)
            self.lbl_reason.setStyleSheet("font-size: 10px; font-weight: bold; color: #22C55E; background-color: #14532D; border-radius: 4px; padding: 3px 7px;")
        elif v1:
            tag = "T2 VALID (S10)" if t1_st == 10 else "T2 VALID"
            self.lbl_reason.setText(tag)
            self.lbl_reason.setStyleSheet("font-size: 10px; font-weight: bold; color: #EAB308; background-color: #713F12; border-radius: 4px; padding: 3px 7px;")

        # 3. Target 0 / Target 1 Details
        self.lbl_t0_dist.setText(f"{t0_mm:d} mm" if t0_mm > 0 else "---")
        self.lbl_t1_dist.setText(f"{t1_mm:d} mm" if t1_mm > 0 else "---")

        st0_text, st0_color, _ = get_status_info(t0_st)
        st1_text, st1_color, _ = get_status_info(t1_st)

        if t0_st == 10:
            if accept_s10:
                st0_text = "Valid (No Prev)"
                st0_color = "#14B8A6"
            else:
                st0_text = "Ignored (S10 OFF)"
                st0_color = "#F59E0B"

        if t1_st == 10:
            if accept_s10:
                st1_text = "Valid (No Prev)"
                st1_color = "#14B8A6"
            else:
                st1_text = "Ignored (S10 OFF)"
                st1_color = "#F59E0B"

        self.lbl_t0_status.setText(f"{t0_st}: {st0_text}")
        self.lbl_t0_status.setStyleSheet(f"font-size: 10px; font-weight: bold; color: {st0_color};")

        self.lbl_t1_status.setText(f"{t1_st}: {st1_text}")
        self.lbl_t1_status.setStyleSheet(f"font-size: 10px; font-weight: bold; color: {st1_color};")

        # 4. SPAD bar
        self.bar_spad.setValue(min(16384, spad_cnt))
        if spad_cnt == 1024 and not v0 and not v1:
            self.lbl_spad_val.setText(f"{spad_cnt} (CONTACT)")
            self.lbl_spad_val.setStyleSheet("font-size: 11px; font-weight: bold; color: #EF4444; font-family: Consolas;")
            self.bar_spad.setStyleSheet("""
                QProgressBar { background-color: #0F172A; border: 1px solid #7F1D1D; border-radius: 4px; }
                QProgressBar::chunk { background-color: #EF4444; border-radius: 3px; }
            """)
        else:
            self.lbl_spad_val.setText(f"{spad_cnt}")
            self.lbl_spad_val.setStyleSheet("font-size: 11px; font-weight: bold; color: #CBD5E1; font-family: Consolas;")
            self.bar_spad.setStyleSheet("""
                QProgressBar { background-color: #0F172A; border: 1px solid #334155; border-radius: 4px; }
                QProgressBar::chunk { background-color: #38BDF8; border-radius: 3px; }
            """)


# =============================================================================
# SENSOR COLUMN COMPONENT (VERTICALLY EXPANDING)
# =============================================================================
class SensorTelemetryColumn(QFrame):
    """
    Renders 1 complete ToF sensor column (Header + 4 Vertically Distributed Cards).
    """
    def __init__(self, sensor_id: int, parent=None):
        super().__init__(parent)
        self.sensor_id = sensor_id
        self.meta = SENSOR_META[sensor_id]

        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet(f"""
            SensorTelemetryColumn {{
                background-color: #0F172A;
                border: 1px solid #334155;
                border-top: 4px solid {self.meta["color"]};
                border-radius: 8px;
            }}
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        # Header Section
        hdr_frame = QFrame()
        hdr_frame.setStyleSheet("background-color: #1E293B; border-radius: 6px; padding: 2px;")
        hdr_layout = QVBoxLayout(hdr_frame)
        hdr_layout.setContentsMargins(10, 8, 10, 8)
        hdr_layout.setSpacing(3)

        title_row = QHBoxLayout()
        self.lbl_title = QLabel(self.meta["name"])
        self.lbl_title.setStyleSheet(f"font-size: 14px; font-weight: 800; color: {self.meta['color']};")
        self.lbl_hz = QLabel("0.0 Hz")
        self.lbl_hz.setStyleSheet("font-size: 12px; font-weight: bold; color: #38BDF8; font-family: Consolas;")
        title_row.addWidget(self.lbl_title)
        title_row.addStretch()
        title_row.addWidget(self.lbl_hz)
        hdr_layout.addLayout(title_row)

        sub_row = QHBoxLayout()
        self.lbl_role = QLabel(self.meta["role"])
        self.lbl_role.setStyleSheet("font-size: 11px; color: #94A3B8;")
        self.lbl_rx = QLabel("Rx: 0")
        self.lbl_rx.setStyleSheet("font-size: 11px; color: #94A3B8; font-family: Consolas;")
        sub_row.addWidget(self.lbl_role)
        sub_row.addStretch()
        sub_row.addWidget(self.lbl_rx)
        hdr_layout.addLayout(sub_row)

        self.lbl_pins = QLabel(self.meta["pins"])
        self.lbl_pins.setStyleSheet("font-size: 9px; color: #64748B; font-family: Consolas;")
        hdr_layout.addWidget(self.lbl_pins)

        layout.addWidget(hdr_frame)

        # 4 Zone Cards: Each card gets stretch factor 1 to evenly fill vertical space
        self.cards = []
        for i in range(4):
            card = ZoneTelemetryCard(sensor_id, i, self)
            self.cards.append(card)
            layout.addWidget(card, 1)

    def update_sensor_data(self, sdata: dict, accept_s10: bool = True):
        hz = sdata.get("hz", 0.0)
        rx = sdata.get("rx_count", 0)
        active = sdata.get("active", False)

        if active:
            self.lbl_hz.setText(f"{hz:.1f} Hz")
            self.lbl_hz.setStyleSheet("font-size: 12px; font-weight: bold; color: #38BDF8; font-family: Consolas;")
        else:
            self.lbl_hz.setText("OFFLINE")
            self.lbl_hz.setStyleSheet("font-size: 12px; font-weight: bold; color: #EF4444; font-family: Consolas;")

        self.lbl_rx.setText(f"Rx: {rx:d}")

        f = sdata.get("frame")
        if not f:
            return

        dist_mm = f.get("dist_mm", [0]*4)
        t0_mm = f.get("target0_mm", [0]*4)
        t1_mm = f.get("target1_mm", [0]*4)
        t0_st = f.get("target0_st", [0]*4)
        t1_st = f.get("target1_st", [0]*4)
        spad_cnt = f.get("spad_cnt", [0]*4)
        is_52 = f.get("is_52", True)

        for i in range(4):
            if i < len(self.cards):
                self.cards[i].set_data(
                    dist_mm=dist_mm[i] if i < len(dist_mm) else 0,
                    t0_mm=t0_mm[i] if i < len(t0_mm) else 0,
                    t1_mm=t1_mm[i] if i < len(t1_mm) else 0,
                    t0_st=t0_st[i] if i < len(t0_st) else 0,
                    t1_st=t1_st[i] if i < len(t1_st) else 0,
                    spad_cnt=spad_cnt[i] if i < len(spad_cnt) else 0,
                    is_52=is_52,
                    accept_s10=accept_s10
                )


# =============================================================================
# FLASH ESP32 DIALOG (ONE-CLICK ASYNC FLASH)
# =============================================================================
class FlashDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("ESP32 ToF Firmware Uploader (PlatformIO)")
        self.resize(650, 400)
        self.setStyleSheet("background-color: #0F172A; color: #F8FAFC;")

        layout = QVBoxLayout(self)
        self.lbl_status = QLabel("Ready to upload ESP32 ToF firmware...")
        self.lbl_status.setStyleSheet("font-size: 13px; font-weight: bold; color: #38BDF8;")
        layout.addWidget(self.lbl_status)

        self.text_log = QTextEdit()
        self.text_log.setReadOnly(True)
        self.text_log.setStyleSheet("background-color: #020617; border: 1px solid #334155; font-family: Consolas; font-size: 11px;")
        layout.addWidget(self.text_log)

        btn_row = QHBoxLayout()
        self.btn_start = QPushButton("Start Upload (pio run -e esp32_tof -t upload)")
        self.btn_start.setStyleSheet("background-color: #2563EB; color: white; padding: 8px 16px; border-radius: 6px; font-weight: bold;")
        self.btn_start.clicked.connect(self.run_upload)
        self.btn_close = QPushButton("Close")
        self.btn_close.setStyleSheet("background-color: #334155; color: white; padding: 8px 16px; border-radius: 6px;")
        self.btn_close.clicked.connect(self.accept)
        btn_row.addWidget(self.btn_start)
        btn_row.addStretch()
        btn_row.addWidget(self.btn_close)
        layout.addLayout(btn_row)

    def run_upload(self):
        self.btn_start.setEnabled(False)
        self.lbl_status.setText("Compiling and flashing firmware to ESP32 on COM7...")
        self.lbl_status.setStyleSheet("font-size: 13px; font-weight: bold; color: #F59E0B;")

        def worker():
            try:
                cmd = ["pio", "run", "-e", "esp32_tof", "-t", "upload"]
                proc = subprocess.Popen(
                    cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    universal_newlines=True, cwd=os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
                )
                for line in proc.stdout:
                    self.text_log.append(line.rstrip())
                proc.wait()
                if proc.returncode == 0:
                    self.lbl_status.setText("SUCCESS: ESP32 ToF firmware flashed successfully!")
                    self.lbl_status.setStyleSheet("font-size: 13px; font-weight: bold; color: #22C55E;")
                else:
                    self.lbl_status.setText(f"FAILED with exit code {proc.returncode}")
                    self.lbl_status.setStyleSheet("font-size: 13px; font-weight: bold; color: #EF4444;")
            except Exception as e:
                self.text_log.append(f"\nError: {e}")
                self.lbl_status.setText("FAILED to start upload process")
                self.lbl_status.setStyleSheet("font-size: 13px; font-weight: bold; color: #EF4444;")
            finally:
                self.btn_start.setEnabled(True)

        threading.Thread(target=worker, daemon=True).start()


# =============================================================================
# MAIN WINDOW
# =============================================================================
class ToFMultiTargetMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Folkrace — ESP32 Quad VL53L8CX 4-Zone Telemetry Studio")
        self.resize(1600, 960)
        self.setMinimumSize(1100, 720)

        self.receiver = None
        self.accept_s10 = True

        # Root Dark Theme
        self.setStyleSheet("""
            QMainWindow {
                background-color: #090D16;
            }
            QLabel {
                color: #F8FAFC;
            }
            QComboBox {
                background-color: #1E293B;
                border: 1px solid #334155;
                border-radius: 6px;
                color: #F8FAFC;
                padding: 4px 8px;
                font-family: Consolas;
                font-size: 11px;
            }
            QComboBox::drop-down {
                border: none;
            }
            QPushButton {
                background-color: #2563EB;
                color: #FFFFFF;
                border: none;
                border-radius: 6px;
                padding: 6px 12px;
                font-weight: bold;
                font-size: 11px;
            }
            QPushButton:hover {
                background-color: #1D4ED8;
            }
            QPushButton:disabled {
                background-color: #334155;
                color: #64748B;
            }
        """)

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        root_layout = QVBoxLayout(main_widget)
        root_layout.setContentsMargins(10, 8, 10, 8)
        root_layout.setSpacing(8)

        # 1. Top Toolbar (Controls & Health)
        toolbar = self.create_toolbar()
        root_layout.addWidget(toolbar)

        # 2. 4 Sensor Columns (Spatial: ToF 1 -> ToF 2 -> ToF 4 -> ToF 3) filling 100% of the screen
        cols_container = QWidget()
        cols_layout = QHBoxLayout(cols_container)
        cols_layout.setContentsMargins(0, 0, 0, 0)
        cols_layout.setSpacing(8)

        self.columns = {}
        for sid in SENSOR_ORDER:
            col = SensorTelemetryColumn(sid, self)
            self.columns[sid] = col
            cols_layout.addWidget(col, 1)

        root_layout.addWidget(cols_container, 1)

        # 3. Refresh timer for GUI (60 FPS redraw)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(16)

        # Auto-connect if COM7 exists
        QTimer.singleShot(400, self.auto_connect)

    def create_toolbar(self) -> QFrame:
        bar = QFrame()
        bar.setStyleSheet("""
            QFrame {
                background-color: #0F172A;
                border: 1px solid #1E293B;
                border-radius: 8px;
            }
        """)
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(10)

        # Title
        lbl_app = QLabel("ESP32 TOF HUB")
        lbl_app.setStyleSheet("font-size: 13px; font-weight: 900; color: #38BDF8; letter-spacing: 1px;")
        layout.addWidget(lbl_app)

        # Port Dropdown
        layout.addWidget(QLabel("Port:"))
        self.combo_port = QComboBox()
        self.combo_port.setMinimumWidth(110)
        self.refresh_ports()
        layout.addWidget(self.combo_port)

        # Refresh Ports Button
        btn_refresh = QPushButton("↻")
        btn_refresh.setFixedWidth(28)
        btn_refresh.setToolTip("Scan serial ports")
        btn_refresh.clicked.connect(self.refresh_ports)
        layout.addWidget(btn_refresh)

        # Baud Dropdown
        layout.addWidget(QLabel("Baud:"))
        self.combo_baud = QComboBox()
        for b in [921600, 460800, 230400, 115200]:
            self.combo_baud.addItem(str(b))
        self.combo_baud.setCurrentText("921600")
        layout.addWidget(self.combo_baud)

        # Connect / Disconnect Button
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connection)
        layout.addWidget(self.btn_connect)

        # Reset MCU Button
        self.btn_reset = QPushButton("Reset ESP32")
        self.btn_reset.setStyleSheet("background-color: #475569; color: white;")
        self.btn_reset.clicked.connect(self.reset_mcu)
        layout.addWidget(self.btn_reset)

        # Flash Firmware Button
        self.btn_flash = QPushButton("Flash ESP32 (PIO)")
        self.btn_flash.setStyleSheet("background-color: #7C3AED; color: white;")
        self.btn_flash.clicked.connect(self.open_flash_dialog)
        layout.addWidget(self.btn_flash)

        # S10 Acceptance Toggle Button
        self.btn_s10 = QPushButton("S10: ACCEPTED")
        self.btn_s10.setStyleSheet("background-color: #16A34A; color: white; padding: 6px 12px; border-radius: 6px; font-weight: bold; font-size: 11px;")
        self.btn_s10.setToolTip("Toggle ST Table 4 Status 10 (Valid without previous history)")
        self.btn_s10.clicked.connect(self.toggle_s10)
        layout.addWidget(self.btn_s10)

        layout.addStretch()

        # Telemetry Status Labels
        self.lbl_protocol = QLabel("Proto: 52B Raw")
        self.lbl_protocol.setStyleSheet("font-size: 11px; font-weight: bold; color: #34D399; font-family: Consolas;")
        layout.addWidget(self.lbl_protocol)

        self.lbl_total_hz = QLabel("Total: 0.0 Hz")
        self.lbl_total_hz.setStyleSheet("font-size: 11px; font-weight: bold; color: #38BDF8; font-family: Consolas;")
        layout.addWidget(self.lbl_total_hz)

        self.lbl_chk_errs = QLabel("Errors: 0")
        self.lbl_chk_errs.setStyleSheet("font-size: 11px; color: #94A3B8; font-family: Consolas;")
        layout.addWidget(self.lbl_chk_errs)

        self.lbl_status = QLabel("Disconnected")
        self.lbl_status.setStyleSheet("font-size: 11px; color: #EF4444; font-weight: bold;")
        layout.addWidget(self.lbl_status)

        return bar

    def refresh_ports(self):
        current = self.combo_port.currentText()
        self.combo_port.clear()
        ports = serial.tools.list_ports.comports()
        preferred_idx = 0
        for i, p in enumerate(ports):
            self.combo_port.addItem(p.device)
            # Prioritize COM7 or USB-Enhanced / CH343 / ESP32
            desc = (p.description or "").lower()
            if "com7" in p.device.lower() or "ch343" in desc or "usb-enhanced" in desc or "esp" in desc:
                preferred_idx = i

        if self.combo_port.count() > 0:
            self.combo_port.setCurrentIndex(preferred_idx)

    def auto_connect(self):
        if self.combo_port.count() > 0 and not self.receiver:
            self.toggle_connection()

    def toggle_connection(self):
        if self.receiver and self.receiver.isRunning():
            try:
                self.receiver.send_bytes(b"L\n")
                time.sleep(0.05)
            except Exception:
                pass
            self.receiver.stop()
            self.receiver = None
            self.btn_connect.setText("Connect")
            self.btn_connect.setStyleSheet("background-color: #2563EB;")
            self.lbl_status.setText("Disconnected")
            self.lbl_status.setStyleSheet("font-size: 11px; color: #EF4444; font-weight: bold;")
        else:
            port = self.combo_port.currentText()
            baud = int(self.combo_baud.currentText())
            if not port:
                return

            self.receiver = TelemetryReceiverThread(port, baud)
            self.receiver.connection_changed.connect(self.on_connection_changed)
            self.receiver.start()
            self.btn_connect.setText("Disconnect")
            self.btn_connect.setStyleSheet("background-color: #DC2626;")

    def toggle_s10(self):
        self.accept_s10 = not self.accept_s10
        if self.accept_s10:
            self.btn_s10.setText("S10: ACCEPTED")
            self.btn_s10.setStyleSheet("background-color: #16A34A; color: white; padding: 6px 12px; border-radius: 6px; font-weight: bold; font-size: 11px;")
            if self.receiver and self.receiver.isRunning():
                self.receiver.send_bytes(b"1\n")
        else:
            self.btn_s10.setText("S10: REJECTED")
            self.btn_s10.setStyleSheet("background-color: #D97706; color: white; padding: 6px 12px; border-radius: 6px; font-weight: bold; font-size: 11px;")
            if self.receiver and self.receiver.isRunning():
                self.receiver.send_bytes(b"0\n")

    def on_connection_changed(self, connected: bool, msg: str):
        if connected:
            self.lbl_status.setText(msg)
            self.lbl_status.setStyleSheet("font-size: 11px; color: #22C55E; font-weight: bold;")
            if self.receiver and self.receiver.isRunning():
                # Switch ESP32 to 52-byte multi-target debug stream for GUI and send S10 mode
                self.receiver.send_bytes(b"D\n" + (b"1\n" if self.accept_s10 else b"0\n"))
        else:
            self.lbl_status.setText(msg)
            self.lbl_status.setStyleSheet("font-size: 11px; color: #EF4444; font-weight: bold;")
            self.btn_connect.setText("Connect")
            self.btn_connect.setStyleSheet("background-color: #2563EB;")

    def reset_mcu(self):
        if self.receiver and self.receiver.isRunning():
            self.receiver.pulse_reset()

    def open_flash_dialog(self):
        # Disconnect serial first so port is free
        was_connected = (self.receiver and self.receiver.isRunning())
        if was_connected:
            self.toggle_connection()

        dlg = FlashDialog(self)
        dlg.exec_()

        # Reconnect if was connected
        if was_connected:
            QTimer.singleShot(1000, self.toggle_connection)

    def update_gui(self):
        if not self.receiver or not self.receiver.isRunning():
            return

        snap, chk_errs, is_52 = self.receiver.get_snapshot()

        total_hz = sum(snap[s]["hz"] for s in snap)
        self.lbl_total_hz.setText(f"Total: {total_hz:.1f} Hz")
        self.lbl_chk_errs.setText(f"Errors: {chk_errs}")

        if is_52:
            self.lbl_protocol.setText("Proto: 52B Multi-Target")
            self.lbl_protocol.setStyleSheet("font-size: 11px; font-weight: bold; color: #34D399; font-family: Consolas;")
        else:
            self.lbl_protocol.setText("Proto: 20B Legacy")
            self.lbl_protocol.setStyleSheet("font-size: 11px; font-weight: bold; color: #F59E0B; font-family: Consolas;")

        # Update columns
        for sid in SENSOR_ORDER:
            if sid in self.columns and sid in snap:
                self.columns[sid].update_sensor_data(snap[sid], accept_s10=self.accept_s10)

    def closeEvent(self, event):
        if self.receiver:
            try:
                self.receiver.send_bytes(b"L\n")
                time.sleep(0.05)
            except Exception:
                pass
            self.receiver.stop()
        event.accept()


# =============================================================================
# ENTRY POINT
# =============================================================================
def main():
    app = QApplication(sys.argv)
    window = ToFMultiTargetMainWindow()
    window.showMaximized()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
