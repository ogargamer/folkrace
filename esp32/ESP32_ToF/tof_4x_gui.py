#!/usr/bin/env python3
"""
===============================================================================
4x VL53L8CX LIVE BINARY TELEMETRY STREAM VIEWER GUI
===============================================================================
Simple, low-latency PyQt5 GUI for monitoring all 4x VL53L8CX ToF sensors
streaming 20-byte binary frames (`ToF_Binary_Packet` from `esp32/tof_test/main.cpp`).

Packet layout (20 bytes, little-endian `<2B B I I 4H B`):
  - header[2]        : 0xAA, 0x55
  - sensor_id        : uint8  (1..4)
  - seq              : uint32 (frame sequence counter)
  - ts_ms            : uint32 (ESP32 millis timestamp)
  - dist_mm[4]       : uint16 (4 closest distances for the 4 active zones)
  - checksum         : uint8  (XOR of bytes 0..18)
===============================================================================
"""

import sys
import struct
import time
import threading
from collections import deque

import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QFrame, QSizePolicy,
    QPlainTextEdit
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QRectF
from PyQt5.QtGui import QColor, QFont, QPainter, QBrush, QPen

if hasattr(Qt, "HighDpiScaleFactorRoundingPolicy"):
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )

PACKET_MAGIC = b"\xaa\x55"
PACKET_FMT = "<2B B I I 4H B"
PACKET_SIZE = struct.calcsize(PACKET_FMT)  # 20 bytes

# Dedicated 4 zones per sensor matching 2D LiDAR ray projection (from center forward to side):
SENSOR_ZONES = {
    1: [1, 5, 9, 13],    # Left Side / Right-angled: 49.1° to 82.9° Left
    2: [14, 10, 6, 2],   # Left Front: 4.1° to 37.9° Left
    3: [2, 6, 10, 14],   # Right Side / Left-angled: 49.1° to 82.9° Right
    4: [13, 9, 5, 1],    # Right Front: 4.1° to 37.9° Right
}

# Webots LiDAR beam names matching each zone index 0..3
SENSOR_WEBOTS_NAMES = {
    1: [
        "tof_front_left_49.1deg (Z1)",
        "tof_front_left_60.4deg (Z5)",
        "tof_front_left_71.6deg (Z9)",
        "tof_front_left_82.9deg (Z13)",
    ],
    2: [
        "tof_front_left_4.1deg (Z14)",
        "tof_front_left_15.4deg (Z10)",
        "tof_front_left_26.6deg (Z6)",
        "tof_front_left_37.9deg (Z2)",
    ],
    3: [
        "tof_front_right_49.1deg (Z2)",
        "tof_front_right_60.4deg (Z6)",
        "tof_front_right_71.6deg (Z10)",
        "tof_front_right_82.9deg (Z14)",
    ],
    4: [
        "tof_front_right_4.1deg (Z13)",
        "tof_front_right_15.4deg (Z9)",
        "tof_front_right_26.6deg (Z5)",
        "tof_front_right_37.9deg (Z1)",
    ],
}

SENSOR_ROLES = {
    1: "Left Side (49.1°..82.9° Left)",
    2: "Left Front (4.1°..37.9° Left)",
    3: "Right Side (49.1°..82.9° Right)",
    4: "Right Front (4.1°..37.9° Right)",
}

SENSOR_COLORS = {
    1: "#38BDF8",  # Cyan
    2: "#34D399",  # Emerald
    3: "#F59E0B",  # Amber
    4: "#A855F7",  # Purple
}

SENSOR_PINOUTS = {
    1: "CS: GPIO7 | INT: GPIO6 | LPn: GPIO5 | MISO: GPIO4",
    2: "CS: GPIO12 | INT: GPIO11 | LPn: GPIO10 | MISO: GPIO9",
    3: "CS: GPIO41 | INT: GPIO42 | LPn: GPIO2 | MISO: GPIO1",
    4: "CS: GPIO17 | INT: GPIO18 | LPn: GPIO16 | MISO: GPIO15",
}

VALID_STATUSES = {5, 6, 9, 10}


def distance_color(mm: int, st: int) -> QColor:
    """Color code distance cell (near=red/orange, mid=cyan, far=green, invalid=slate)."""
    # 0 mm is only shown if it's a valid reading under 20mm or spad count is 1024 / contact error
    if mm == 0:
        if st in VALID_STATUSES:
            return QColor(220, 38, 38)  # Valid close contact reading (<20mm)
        else:
            return QColor(185, 28, 28)  # 1024 SPAD contact saturation fallback
    if mm >= 3000 and st not in VALID_STATUSES:
        # 3000mm open-space fallback
        return QColor(20, 83, 45)
    if mm < 0:
        return QColor(30, 41, 59)
    if mm < 250:
        return QColor(220, 38, 38)
    if mm < 600:
        return QColor(234, 88, 12)
    if mm < 1200:
        return QColor(202, 138, 4)
    if mm < 2200:
        return QColor(8, 145, 178)
    return QColor(22, 163, 74)


# =============================================================================
# SERIAL READER THREAD (BINARY + BOOT ASCII LOGS)
# =============================================================================

class ToFStreamReader(QThread):
    connection_changed = pyqtSignal(bool, str)
    ascii_line_received = pyqtSignal(str)

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
            self.latest = {1: None, 2: None, 3: None, 4: None}
            self.stats = {
                i: {
                    "rx": 0,
                    "lost": 0,
                    "last_seq": None,
                    "last_rx_time": 0.0,
                    "ts_history": deque(maxlen=40),
                }
                for i in (1, 2, 3, 4)
            }
            self.bad_checksums = 0

    def get_snapshot(self):
        now = time.perf_counter()
        with self._lock:
            snap = {}
            for i in (1, 2, 3, 4):
                st = self.stats[i]
                th = st["ts_history"]
                hz = 0.0
                if len(th) >= 2 and (now - st["last_rx_time"]) < 1.0:
                    span = th[-1] - th[0]
                    if span > 0:
                        hz = (len(th) - 1) / span
                snap[i] = {
                    "frame": self.latest[i],
                    "rx": st["rx"],
                    "lost": st["lost"],
                    "hz": hz,
                    "age_ms": (now - st["last_rx_time"]) * 1000.0 if st["last_rx_time"] else None,
                    "active": bool(st["last_rx_time"] and (now - st["last_rx_time"]) < 0.6),
                }
            return snap, self.bad_checksums

    def _record_packet(self, sensor_id, seq, ts_ms, dists):
        now = time.perf_counter()

        with self._lock:
            if sensor_id not in self.stats:
                return
            st = self.stats[sensor_id]
            st["rx"] += 1
            st["last_rx_time"] = now
            st["ts_history"].append(now)
            if st["last_seq"] is not None and seq > st["last_seq"] + 1:
                gap = seq - st["last_seq"] - 1
                if gap < 100000:
                    st["lost"] += gap
            st["last_seq"] = seq

            self.latest[sensor_id] = {
                "seq": seq,
                "ts_ms": ts_ms,
                "dist_mm": dists,
            }

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
            self.connection_changed.emit(True, f"Connected to {self.port} @ {self.baud}")
        except Exception as e:
            self._running = False
            self.connection_changed.emit(False, f"Connection failed: {e}")
            return

        buf = bytearray()
        ascii_buf = bytearray()

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
                        # Keep last byte in case it is 0xAA
                        if len(buf) > 1:
                            ascii_buf.extend(buf[:-1])
                            del buf[:-1]
                        break

                    if idx > 0:
                        ascii_buf.extend(buf[:idx])
                        del buf[:idx]

                    if len(buf) < PACKET_SIZE:
                        break

                    pkt_bytes = bytes(buf[:PACKET_SIZE])
                    chk = 0
                    for b in pkt_bytes[:-1]:
                        chk ^= b

                    if chk == pkt_bytes[-1] and 1 <= pkt_bytes[2] <= 4:
                        unpacked = struct.unpack(PACKET_FMT, pkt_bytes)
                        sensor_id = unpacked[2]
                        seq = unpacked[3]
                        ts_ms = unpacked[4]
                        dists = list(unpacked[5:9])
                        self._record_packet(sensor_id, seq, ts_ms, dists)
                        del buf[:PACKET_SIZE]
                    else:
                        with self._lock:
                            self.bad_checksums += 1
                        del buf[:2]

                # Flush any accumulated ASCII boot/init lines
                while b"\n" in ascii_buf:
                    line_bytes, _, rest = ascii_buf.partition(b"\n")
                    ascii_buf = bytearray(rest)
                    text = line_bytes.decode("utf-8", errors="ignore").strip()
                    if text:
                        self.ascii_line_received.emit(text)
                if len(ascii_buf) > 2048:
                    ascii_buf.clear()

            except Exception as e:
                if self._running:
                    self.connection_changed.emit(False, f"Serial error: {e}")
                break

        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def reset_esp32(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.dtr = False
                self.ser.rts = True
                time.sleep(0.05)
                self.ser.rts = False
                self.reset_state()
            except Exception:
                pass

    def stop(self):
        self._running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.wait(500)


# =============================================================================
# SENSOR PANEL WIDGET (1x4 2D LIDAR BEAM STRIP + STATS)
# =============================================================================

class SensorHeatmapCanvas(QWidget):
    """Draws the 4 active 2D LiDAR ray zones for one ToF (1 row x 4 columns)."""

    def __init__(self, sensor_id: int, parent=None):
        super().__init__(parent)
        self.sensor_id = sensor_id
        self.frame = None
        self.active = False
        self.setMinimumSize(280, 160)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_data(self, frame, active: bool):
        self.frame = frame
        self.active = active
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor("#0d1320"))

        zones = SENSOR_ZONES.get(self.sensor_id, [0, 1, 2, 3])
        webots_names = SENSOR_WEBOTS_NAMES.get(self.sensor_id, ["", "", "", ""])

        cols = 4
        pad = 6
        cell_w = (w - pad * (cols + 1)) / cols
        cell_h = h - pad * 2

        for c in range(cols):
            idx = c  # 0..3 (representing the 4 active 2D LiDAR rays)
            z_id = zones[idx]
            x = pad + c * (cell_w + pad)
            y = pad
            rect = QRectF(x, y, cell_w, cell_h)

            if self.frame is not None:
                d0 = self.frame["dist_mm"][idx]
                bg = distance_color(d0, 5 if 0 < d0 < 3000 else 0)
                if not self.active:
                    bg = bg.darker(180)
            else:
                d0 = 0
                bg = QColor("#161f30")

            p.setBrush(QBrush(bg))
            p.setPen(QPen(QColor("#26354d"), 1.2))
            p.drawRoundedRect(rect, 6, 6)

            # Zone tag top-left (e.g. Zone 14)
            p.setFont(QFont("Consolas", 8, QFont.Bold))
            p.setPen(QColor(203, 213, 225, 220))
            p.drawText(
                QRectF(x + 5, y + 4, cell_w - 10, 16),
                Qt.AlignLeft | Qt.AlignVCenter,
                f"Zone {z_id}",
            )

            # Fallback tag top-right (if 0mm contact or 3m open space)
            if self.frame is not None and d0 in (0, 3000):
                fb_txt = "SAT" if d0 == 0 else "OPEN"
                p.setFont(QFont("Consolas", 8, QFont.Bold))
                p.setPen(QColor("#f59e0b" if d0 == 0 else "#22c55e"))
                p.drawText(
                    QRectF(x + 5, y + 4, cell_w - 10, 16),
                    Qt.AlignRight | Qt.AlignVCenter,
                    fb_txt,
                )

            # Webots beam angle label
            p.setFont(QFont("Segoe UI", 7, QFont.Bold))
            p.setPen(QColor("#38bdf8"))
            beam_tag = webots_names[idx].split(" ")[0].replace("tof_front_", "")
            p.drawText(
                QRectF(x + 4, y + 22, cell_w - 8, 14),
                Qt.AlignCenter,
                beam_tag,
            )

            # Primary distance (Shortest distance to closest valid target in zone)
            p.setFont(QFont("Segoe UI", 15, QFont.Bold))
            p.setPen(QColor("#ffffff") if self.frame is not None else QColor("#475569"))
            main_txt = f"{d0} mm" if self.frame is not None else "---"
            p.drawText(
                QRectF(x, y + cell_h * 0.38, cell_w, cell_h * 0.40),
                Qt.AlignCenter,
                main_txt,
            )

            # Subtitle
            p.setFont(QFont("Consolas", 8))
            p.setPen(QColor(226, 232, 240, 130) if self.frame is not None else QColor("#475569"))
            sub_txt = "Closest Target" if self.frame is not None else "No Data"
            p.drawText(
                QRectF(x, y + cell_h * 0.76, cell_w, cell_h * 0.20),
                Qt.AlignCenter,
                sub_txt,
            )


class SensorCard(QFrame):
    def __init__(self, sensor_id: int, parent=None):
        super().__init__(parent)
        self.sensor_id = sensor_id
        accent = SENSOR_COLORS[sensor_id]
        self.setStyleSheet(
            f"SensorCard {{ background-color: #111827; border: 1px solid #1f2937; border-radius: 8px; }}"
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 10)
        layout.setSpacing(6)

        header = QHBoxLayout()
        role_desc = SENSOR_ROLES.get(sensor_id, "")
        self.lbl_title = QLabel(f"ToF {sensor_id} — {role_desc}")
        self.lbl_title.setFont(QFont("Segoe UI", 10, QFont.Bold))
        self.lbl_title.setStyleSheet(f"color: {accent};")
        header.addWidget(self.lbl_title)

        lbl_pins = QLabel(SENSOR_PINOUTS.get(sensor_id, ""))
        lbl_pins.setFont(QFont("Consolas", 8))
        lbl_pins.setStyleSheet("color: #64748b; margin-left: 6px;")
        header.addWidget(lbl_pins)

        header.addStretch()

        self.lbl_stats = QLabel("WAITING  |  0.0 Hz  |  Frames: 0")
        self.lbl_stats.setFont(QFont("Consolas", 9, QFont.Bold))
        self.lbl_stats.setStyleSheet(
            "background-color: #1e293b; color: #94a3b8; padding: 3px 8px; border-radius: 4px;"
        )
        header.addWidget(self.lbl_stats)
        layout.addLayout(header)

        self.canvas = SensorHeatmapCanvas(sensor_id, self)
        layout.addWidget(self.canvas, 1)

    def update_card(self, info: dict):
        active = info["active"]
        hz = info["hz"]
        rx = info["rx"]
        lost = info["lost"]
        frame = info["frame"]

        if active:
            status_str = "STREAMING"
            col = "#10b981" if hz >= 8.0 else "#f59e0b"
        elif rx > 0:
            status_str = "STALLED"
            col = "#ef4444"
        else:
            status_str = "WAITING"
            col = "#94a3b8"

        seq_str = f"#{frame['seq']}" if frame else "--"
        self.lbl_stats.setText(
            f"{status_str}  |  {hz:4.1f} Hz  |  RX: {rx} ({seq_str})  |  Lost: {lost}"
        )
        self.lbl_stats.setStyleSheet(
            f"background-color: #1e293b; color: {col}; padding: 3px 8px; border-radius: 4px;"
        )
        self.canvas.set_data(frame, active)


# =============================================================================
# MAIN WINDOW
# =============================================================================

class FourToFStreamGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("4x VL53L8CX ToF Live Stream Viewer")
        self.resize(1100, 720)
        self.setStyleSheet(
            "background-color: #090d16; color: #f1f5f9; font-family: 'Segoe UI', sans-serif;"
        )

        self.reader = ToFStreamReader()
        self.reader.connection_changed.connect(self.on_connection_changed)
        self.reader.ascii_line_received.connect(self.on_ascii_line)

        self._build_ui()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.on_tick)
        self.timer.start(30)  # ~33 FPS UI refresh

        self.refresh_ports(auto_connect=True)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        # Top connection bar
        bar = QFrame()
        bar.setStyleSheet(
            "background-color: #111827; border: 1px solid #1f2937; border-radius: 8px;"
        )
        bar_layout = QHBoxLayout(bar)
        bar_layout.setContentsMargins(10, 6, 10, 6)
        bar_layout.setSpacing(8)

        title = QLabel("4x VL53L8CX LIVE STREAM (Shared SCK: GPIO13 | MOSI: GPIO14)")
        title.setFont(QFont("Segoe UI", 10, QFont.Bold))
        title.setStyleSheet("color: #38bdf8;")
        bar_layout.addWidget(title)

        bar_layout.addStretch()

        self.lbl_summary = QLabel("Total: 0 pkts | Bad CRC: 0")
        self.lbl_summary.setFont(QFont("Consolas", 9))
        self.lbl_summary.setStyleSheet("color: #cbd5e1; margin-right: 8px;")
        bar_layout.addWidget(self.lbl_summary)

        bar_layout.addWidget(QLabel("Port:"))
        self.combo_port = QComboBox()
        self.combo_port.setMinimumWidth(95)
        self.combo_port.setStyleSheet(
            "background-color: #1e293b; color: #f8fafc; padding: 3px 6px; border: 1px solid #334155; border-radius: 4px;"
        )
        bar_layout.addWidget(self.combo_port)

        btn_refresh = QPushButton("Refresh")
        btn_refresh.setStyleSheet(
            "background-color: #1e293b; color: #e2e8f0; padding: 4px 8px; border-radius: 4px;"
        )
        btn_refresh.clicked.connect(lambda: self.refresh_ports(auto_connect=False))
        bar_layout.addWidget(btn_refresh)

        self.combo_baud = QComboBox()
        self.combo_baud.addItems(["921600", "460800", "115200"])
        self.combo_baud.setStyleSheet(
            "background-color: #1e293b; color: #f8fafc; padding: 3px 6px; border: 1px solid #334155; border-radius: 4px;"
        )
        bar_layout.addWidget(self.combo_baud)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setStyleSheet(
            "background-color: #0284c7; color: white; font-weight: bold; padding: 4px 12px; border-radius: 4px;"
        )
        self.btn_connect.clicked.connect(self.toggle_connection)
        bar_layout.addWidget(self.btn_connect)

        btn_reset = QPushButton("Reboot ESP32")
        btn_reset.setStyleSheet(
            "background-color: #334155; color: #f8fafc; padding: 4px 10px; border-radius: 4px;"
        )
        btn_reset.clicked.connect(self.reader.reset_esp32)
        bar_layout.addWidget(btn_reset)

        self.lbl_conn = QLabel("DISCONNECTED")
        self.lbl_conn.setFont(QFont("Consolas", 9, QFont.Bold))
        self.lbl_conn.setStyleSheet(
            "background-color: #374151; color: #cbd5e1; padding: 4px 8px; border-radius: 4px;"
        )
        bar_layout.addWidget(self.lbl_conn)

        root.addWidget(bar)

        # 2x2 grid for ToF 1..4 matching physical car layout:
        # Top-Left: ToF 2 (Left Front)       | Top-Right: ToF 4 (Right Front)
        # Bottom-Left: ToF 1 (Left Side)     | Bottom-Right: ToF 3 (Right Side)
        grid = QGridLayout()
        grid.setSpacing(8)
        self.cards = {}
        layout_positions = {
            2: (0, 0),  # Left Front
            4: (0, 1),  # Right Front
            1: (1, 0),  # Left Side
            3: (1, 1),  # Right Side
        }
        for i, (row, col) in layout_positions.items():
            card = SensorCard(i, self)
            self.cards[i] = card
            grid.addWidget(card, row, col)
        root.addLayout(grid, 1)

        # Compact boot/log console at bottom
        self.log_box = QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(200)
        self.log_box.setFixedHeight(90)
        self.log_box.setFont(QFont("Consolas", 8))
        self.log_box.setPlaceholderText("ESP32 setup / boot logs will appear here...")
        self.log_box.setStyleSheet(
            "background-color: #0b111e; color: #94a3b8; border: 1px solid #1f2937; border-radius: 6px; padding: 4px;"
        )
        root.addWidget(self.log_box)

    def refresh_ports(self, auto_connect=False):
        current = self.combo_port.currentText()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.combo_port.clear()
        if not ports:
            self.combo_port.addItem("COM7")
            return

        self.combo_port.addItems(ports)
        if current in ports:
            self.combo_port.setCurrentText(current)
        elif "COM7" in ports:
            self.combo_port.setCurrentText("COM7")
        elif "COM12" in ports:
            self.combo_port.setCurrentText("COM12")

        if auto_connect and not self.reader.isRunning():
            self.toggle_connection()

    def toggle_connection(self):
        if self.reader.isRunning():
            self.reader.stop()
            self.on_connection_changed(False, "Disconnected")
        else:
            port = self.combo_port.currentText().strip()
            baud = int(self.combo_baud.currentText())
            self.reader.port = port
            self.reader.baud = baud
            self.reader.start()

    def on_connection_changed(self, connected: bool, msg: str):
        if connected:
            self.btn_connect.setText("Disconnect")
            self.btn_connect.setStyleSheet(
                "background-color: #dc2626; color: white; font-weight: bold; padding: 4px 12px; border-radius: 4px;"
            )
            self.lbl_conn.setText(f"CONNECTED ({self.reader.port})")
            self.lbl_conn.setStyleSheet(
                "background-color: #065f46; color: #6ee7b7; padding: 4px 8px; border-radius: 4px;"
            )
        else:
            self.btn_connect.setText("Connect")
            self.btn_connect.setStyleSheet(
                "background-color: #0284c7; color: white; font-weight: bold; padding: 4px 12px; border-radius: 4px;"
            )
            self.lbl_conn.setText("DISCONNECTED")
            self.lbl_conn.setStyleSheet(
                "background-color: #374151; color: #cbd5e1; padding: 4px 8px; border-radius: 4px;"
            )
        self.log_box.appendPlainText(f"[SERIAL] {msg}")

    def on_ascii_line(self, line: str):
        self.log_box.appendPlainText(line)

    def on_tick(self):
        snap, bad_crc = self.reader.get_snapshot()
        total_rx = 0
        active_cnt = 0
        for i in (1, 2, 3, 4):
            info = snap[i]
            total_rx += info["rx"]
            if info["active"]:
                active_cnt += 1
            self.cards[i].update_card(info)

        self.lbl_summary.setText(
            f"Active: {active_cnt}/4  |  Total RX: {total_rx}  |  Bad CRC: {bad_crc}"
        )

    def closeEvent(self, event):
        self.reader.stop()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = FourToFStreamGUI()
    win.show()
    sys.exit(app.exec_())

