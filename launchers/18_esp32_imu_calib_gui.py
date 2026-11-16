#!/usr/bin/env python3
"""
================================================================================
  18. LAUNCHER: ESP32 BNO086 IMU HEADING & MOUNTING CALIBRATION TOOL
================================================================================
  Easy 3-step GUI tool to align the BNO086 IMU with the car's forward axis,
  verify Left/Right/Forward signs live, and persist the orientation to BNO086
  internal flash so it survives power cycles.

  Supports BOTH:
    - Via Raspberry Pi 4 (Wi-Fi / LAN -> http://<pi>:8080)
    - Direct USB Serial (COM port -> ESP32-S3 @ 921600 baud)

  Usage:
    python launchers/18_esp32_imu_calib_gui.py
================================================================================
"""

import json
import math
import socket
import sys
import time
import threading
import urllib.request
import urllib.error
from typing import Optional

import serial
import serial.tools.list_ports

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QRectF, QPointF
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QBrush, QPolygonF
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


DEFAULT_BAUD = 921600
PI_PORT = 8080
PI_CANDIDATES = ["192.168.20.120", "192.168.20.117", "robot", "robot.local", "127.0.0.1"]


def resolve_pi_host(preferred: Optional[str] = None, timeout_s: float = 0.8) -> Optional[str]:
    """Auto-detect reachable Pi 4 debug server host."""
    candidates = [preferred] if preferred else []
    for c in PI_CANDIDATES:
        if c not in candidates:
            candidates.append(c)
    for host in candidates:
        if not host:
            continue
        try:
            ip = socket.gethostbyname(host)
            with socket.create_connection((ip, PI_PORT), timeout=timeout_s):
                return host
        except OSError:
            continue
    return None


def detect_esp32_port() -> Optional[str]:
    """Auto-detect likely ESP32-S3 USB serial port."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(k in desc or k in hwid for k in ["ch343", "ch340", "cp210", "espressif", "usb-enhanced-serial", "uart"]):
            return p.device
    for p in ports:
        if p.device.upper() == "COM5":
            return "COM5"
    for p in ports:
        if "bluetooth" not in (p.description or "").lower():
            return p.device
    return None


class SerialBridge(QObject):
    """Background serial reader/writer for direct USB connection to ESP32_Main."""

    telemetry_received = pyqtSignal(dict)
    log_received = pyqtSignal(str)
    connection_changed = pyqtSignal(bool, str)

    def __init__(self):
        super().__init__()
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._ser is not None and self._ser.is_open

    def connect_port(self, port: str, baud: int = DEFAULT_BAUD) -> None:
        self.disconnect_link()
        try:
            ser = serial.Serial(port, baud, timeout=0.05)
            ser.reset_input_buffer()
            with self._lock:
                self._ser = ser
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            self.send_line("IMU_STREAM 1")
            self.connection_changed.emit(True, f"Connected via USB ({port} @ {baud})")
        except Exception as exc:
            self.connection_changed.emit(False, f"USB connection failed: {exc}")

    def disconnect_link(self) -> None:
        self._running = False
        with self._lock:
            ser = self._ser
            self._ser = None
        if ser is not None:
            try:
                if ser.is_open:
                    ser.write(b"IMU_STREAM 0\n")
                    ser.flush()
                    ser.close()
            except Exception:
                pass
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.5)
        self._thread = None
        self.connection_changed.emit(False, "Disconnected")

    def send_line(self, cmd: str) -> bool:
        with self._lock:
            ser = self._ser
            if ser is not None and ser.is_open:
                try:
                    ser.write((cmd.strip() + "\n").encode("utf-8"))
                    ser.flush()
                    return True
                except Exception as exc:
                    self.log_received.emit(f"[USB TX Error] {exc}")
        return False

    def _read_loop(self) -> None:
        raw_buf = bytearray()
        line_buf = ""
        while self._running:
            with self._lock:
                ser = self._ser
            if ser is None or not ser.is_open:
                break
            try:
                waiting = ser.in_waiting
                chunk = ser.read(waiting if waiting > 0 else 1)
                if not chunk:
                    continue
                raw_buf.extend(chunk)

                # Strip 54-byte binary ChassisTelemetryPacket frames (0xAA 0x55 0x01 ...)
                while True:
                    idx = raw_buf.find(b"\xaa\x55\x01")
                    if idx == -1:
                        if len(raw_buf) > 2:
                            line_buf += raw_buf[:-2].decode("utf-8", errors="ignore")
                            raw_buf = raw_buf[-2:]
                        break
                    if idx > 0:
                        line_buf += raw_buf[:idx].decode("utf-8", errors="ignore")
                        raw_buf = raw_buf[idx:]
                    if len(raw_buf) >= 54:
                        raw_buf = raw_buf[54:]
                    else:
                        break

                if "\n" in line_buf:
                    lines = line_buf.split("\n")
                    line_buf = lines[-1]
                    for raw_line in lines[:-1]:
                        line = raw_line.strip()
                        if not line:
                            continue
                        if line.startswith("{"):
                            try:
                                obj = json.loads(line)
                                if obj.get("imu") == 1:
                                    self.telemetry_received.emit(obj)
                                    continue
                            except Exception:
                                pass
                        self.log_received.emit(line)
            except Exception as exc:
                self.log_received.emit(f"[Serial Error] {exc}")
                break
        self._running = False


class Pi4Bridge(QObject):
    """HTTP bridge to Pi 4 debug_server.py (/api/live and /api/mcu/command)."""

    telemetry_received = pyqtSignal(dict)
    log_received = pyqtSignal(str)
    connection_changed = pyqtSignal(bool, str)

    def __init__(self):
        super().__init__()
        self._host: Optional[str] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

    @property
    def is_connected(self) -> bool:
        return self._running and self._host is not None

    def connect_host(self, host: str) -> None:
        self.disconnect_link()
        host = host.strip()
        if not host:
            self.connection_changed.emit(False, "Empty Pi host")
            return
        try:
            url = f"http://{host}:{PI_PORT}/api/status"
            with urllib.request.urlopen(url, timeout=1.5) as resp:
                if resp.status != 200:
                    raise RuntimeError(f"HTTP {resp.status}")
        except Exception as exc:
            self.connection_changed.emit(False, f"Pi 4 unreachable at {host}:{PI_PORT} ({exc})")
            return

        self._host = host
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
        self.connection_changed.emit(True, f"Connected via Pi 4 ({host}:{PI_PORT})")

    def disconnect_link(self) -> None:
        self._running = False
        self._host = None
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.5)
        self._thread = None
        self.connection_changed.emit(False, "Disconnected")

    def send_line(self, cmd: str) -> bool:
        if not self.is_connected:
            return False
        url = f"http://{self._host}:{PI_PORT}/api/mcu/command"
        payload = json.dumps({"action": "raw", "cmd": cmd.strip()}).encode("utf-8")
        req = urllib.request.Request(
            url,
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=1.5) as resp:
                body = json.loads(resp.read().decode("utf-8", errors="ignore"))
                if body.get("ok"):
                    return True
                self.log_received.emit(f"[Pi4 Warning] Command rejected: {body}")
                return False
        except Exception as exc:
            self.log_received.emit(f"[Pi4 TX Error] {exc}")
            return False

    def _poll_loop(self) -> None:
        url = f"http://{self._host}:{PI_PORT}/api/live"
        while self._running:
            try:
                with urllib.request.urlopen(url, timeout=0.8) as resp:
                    pkt = json.loads(resp.read().decode("utf-8"))
                flags = pkt.get("flags", {}) or {}
                online = 1 if (flags.get("imu") or pkt.get("mcu_main_connected")) else 0
                mapped = {
                    "imu": 1,
                    "online": online,
                    "y": float(pkt.get("yaw_deg", pkt.get("yaw_fused", 0.0))),
                    "p": float(pkt.get("pitch_deg", 0.0)),
                    "r": float(pkt.get("roll_deg", 0.0)),
                    "gz": float(pkt.get("gyro_z", 0.0)),
                    "lax": float(pkt.get("accel_fwd", 0.0)),
                    "lay": float(pkt.get("accel_lat", 0.0)),
                }
                self.telemetry_received.emit(mapped)
            except Exception:
                pass
            time.sleep(0.04)  # ~25 Hz UI update over Wi-Fi


class HeadingCompassWidget(QWidget):
    """Top-down car heading visualizer showing IMU yaw vs car front (0 deg)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.yaw_deg = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.gyro_z = 0.0
        self.lin_accel_x = 0.0
        self.online = False
        self.setMinimumSize(320, 320)

    def update_state(self, data: dict) -> None:
        self.online = bool(data.get("online", 0))
        self.yaw_deg = float(data.get("y", 0.0))
        self.roll_deg = float(data.get("r", 0.0))
        self.pitch_deg = float(data.get("p", 0.0))
        self.gyro_z = float(data.get("gz", 0.0))
        self.lin_accel_x = float(data.get("lax", 0.0))
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0
        radius = min(w, h) * 0.40

        painter.fillRect(self.rect(), QColor("#0f141e"))

        # Outer compass ring
        painter.setPen(QPen(QColor("#273349"), 2))
        painter.setBrush(QBrush(QColor("#141b29")))
        painter.drawEllipse(QPointF(cx, cy), radius, radius)

        # Degree tick marks (0 deg = UP / CAR FRONT, +90 = LEFT/CCW)
        for deg in range(0, 360, 15):
            rad = math.radians(deg)
            ux = -math.sin(rad)
            uy = -math.cos(rad)
            inner = radius * (0.86 if deg % 45 == 0 else 0.92)
            pen_col = QColor("#00e5ff") if deg == 0 else QColor("#3d4f6e")
            painter.setPen(QPen(pen_col, 2 if deg % 45 == 0 else 1))
            painter.drawLine(
                QPointF(cx + ux * inner, cy + uy * inner),
                QPointF(cx + ux * radius, cy + uy * radius),
            )

        # Cardinal labels
        painter.setFont(QFont("Segoe UI", 9, QFont.Bold))
        painter.setPen(QColor("#00e5ff"))
        painter.drawText(QRectF(cx - 50, cy - radius - 24, 100, 20), Qt.AlignCenter, "FRONT (0°)")
        painter.setPen(QColor("#8fa3bf"))
        painter.drawText(QRectF(cx - radius - 52, cy - 10, 48, 20), Qt.AlignRight | Qt.AlignVCenter, "+90° L")
        painter.drawText(QRectF(cx + radius + 4, cy - 10, 48, 20), Qt.AlignLeft | Qt.AlignVCenter, "-90° R")
        painter.drawText(QRectF(cx - 35, cy + radius + 4, 70, 20), Qt.AlignCenter, "±180°")

        # Fixed Car Chassis Outline pointing UP (0 deg)
        car_w, car_h = radius * 0.38, radius * 0.64
        car_rect = QRectF(-car_w / 2.0, -car_h / 2.0, car_w, car_h)

        painter.save()
        painter.translate(cx, cy)
        painter.setPen(QPen(QColor("#334766"), 2, Qt.DashLine))
        painter.setBrush(QBrush(QColor(30, 44, 68, 90)))
        painter.drawRoundedRect(car_rect, 8, 8)

        # Rotating IMU Heading Arrow (rotates by -yaw_deg in Qt screen coords because +yaw is CCW)
        painter.rotate(-self.yaw_deg)

        arrow_color = QColor("#00e676") if abs(self.yaw_deg) <= 3.0 else QColor("#ffab00")
        if not self.online:
            arrow_color = QColor("#64748b")

        painter.setPen(QPen(arrow_color, 3))
        painter.setBrush(QBrush(arrow_color))
        painter.drawLine(QPointF(0, radius * 0.45), QPointF(0, -radius * 0.78))

        head = QPolygonF([
            QPointF(0, -radius * 0.85),
            QPointF(-10, -radius * 0.65),
            QPointF(10, -radius * 0.65),
        ])
        painter.drawPolygon(head)
        painter.restore()

        # Center hub
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor("#e2e8f0")))
        painter.drawEllipse(QPointF(cx, cy), 5, 5)


class ImuCalibWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Folkrace — ESP32 BNO086 IMU Heading Calibration")
        self.resize(960, 640)

        self.serial_bridge = SerialBridge()
        self.pi_bridge = Pi4Bridge()
        self._pkt_count = 0
        self._last_hz_ts = time.time()

        self._build_ui()
        self._apply_styles()

        for br in (self.serial_bridge, self.pi_bridge):
            br.telemetry_received.connect(self._on_telemetry)
            br.log_received.connect(self._append_log)
            br.connection_changed.connect(self._on_conn_changed)

        self._hz_timer = QTimer(self)
        self._hz_timer.timeout.connect(self._update_hz)
        self._hz_timer.start(500)

        self._refresh_ports()
        self._auto_connect()

    @property
    def active_bridge(self):
        return self.pi_bridge if self.mode_combo.currentIndex() == 0 else self.serial_bridge

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        main_layout = QVBoxLayout(root)
        main_layout.setContentsMargins(14, 14, 14, 14)
        main_layout.setSpacing(12)

        # Top Connection Bar
        conn_bar = QHBoxLayout()
        title = QLabel("BNO086 IMU Heading Calibration")
        title.setFont(QFont("Segoe UI", 14, QFont.Bold))
        conn_bar.addWidget(title)
        conn_bar.addStretch()

        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Via Pi 4 (Wi-Fi)", "Direct USB (COM)"])
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        conn_bar.addWidget(self.mode_combo)

        self.pi_host_edit = QLineEdit("192.168.20.120")
        self.pi_host_edit.setFixedWidth(135)
        self.pi_host_edit.setPlaceholderText("Pi IP or robot")
        conn_bar.addWidget(self.pi_host_edit)

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(95)
        self.port_combo.setVisible(False)
        conn_bar.addWidget(self.port_combo)

        self.btn_refresh = QPushButton("Auto-Find")
        self.btn_refresh.clicked.connect(self._on_refresh_clicked)
        conn_bar.addWidget(self.btn_refresh)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self._toggle_connect)
        conn_bar.addWidget(self.btn_connect)

        self.lbl_status = QLabel("DISCONNECTED")
        self.lbl_status.setObjectName("statusPill")
        conn_bar.addWidget(self.lbl_status)

        self.lbl_hz = QLabel("0 Hz")
        self.lbl_hz.setObjectName("hzPill")
        conn_bar.addWidget(self.lbl_hz)
        main_layout.addLayout(conn_bar)

        # Middle Content: Left Visual + Right 3-Step Controls
        mid_layout = QHBoxLayout()
        mid_layout.setSpacing(14)

        # Left Panel: Compass + Readouts
        left_card = QFrame()
        left_card.setObjectName("card")
        left_layout = QVBoxLayout(left_card)
        self.compass = HeadingCompassWidget()
        left_layout.addWidget(self.compass, 1)

        readout_grid = QGridLayout()
        self.lbl_yaw = self._make_metric(readout_grid, 0, 0, "HEADING (YAW)", "0.0°")
        self.lbl_pitch = self._make_metric(readout_grid, 0, 1, "PITCH", "0.0°")
        self.lbl_roll = self._make_metric(readout_grid, 0, 2, "ROLL", "0.0°")
        self.lbl_gz = self._make_metric(readout_grid, 1, 0, "GYRO Z (TURN)", "0.00 rad/s")
        self.lbl_lax = self._make_metric(readout_grid, 1, 1, "FWD ACCEL", "0.00 m/s²")
        self.lbl_lay = self._make_metric(readout_grid, 1, 2, "LAT ACCEL", "0.00 m/s²")
        left_layout.addLayout(readout_grid)
        mid_layout.addWidget(left_card, 5)

        # Right Panel: 3-Step Calibration
        right_card = QFrame()
        right_card.setObjectName("card")
        right_layout = QVBoxLayout(right_card)
        right_layout.setSpacing(12)

        step1_title = QLabel("STEP 1 — Place Car Flat & Pointing Forward")
        step1_title.setFont(QFont("Segoe UI", 11, QFont.Bold))
        right_layout.addWidget(step1_title)

        step1_desc = QLabel(
            "Set the car on a level surface facing straight ahead.\n"
            "• Use 'Zero Heading & Level' if the IMU board is mounted rotated or tilted.\n"
            "• Use 'Zero Heading Only' if Roll/Pitch are already level."
        )
        step1_desc.setWordWrap(True)
        step1_desc.setObjectName("mutedText")
        right_layout.addWidget(step1_desc)

        self.btn_tare_all = QPushButton("Zero Heading & Level (Tare All Axes)")
        self.btn_tare_all.setObjectName("primaryBtn")
        self.btn_tare_all.setMinimumHeight(42)
        self.btn_tare_all.clicked.connect(lambda: self._send_cmd("IMU_TARE_ALL"))
        right_layout.addWidget(self.btn_tare_all)

        self.btn_tare_yaw = QPushButton("Zero Heading Only (Yaw = 0°)")
        self.btn_tare_yaw.setMinimumHeight(36)
        self.btn_tare_yaw.clicked.connect(lambda: self._send_cmd("IMU_TARE_YAW"))
        right_layout.addWidget(self.btn_tare_yaw)

        step2_title = QLabel("STEP 2 — Verify Heading Direction")
        step2_title.setFont(QFont("Segoe UI", 11, QFont.Bold))
        right_layout.addWidget(step2_title)

        self.lbl_verify = QLabel("Rotate car slightly Left (+) or Right (-) to verify arrow follows.")
        self.lbl_verify.setObjectName("verifyBox")
        self.lbl_verify.setWordWrap(True)
        right_layout.addWidget(self.lbl_verify)

        step3_title = QLabel("STEP 3 — Save Permanently to IMU")
        step3_title.setFont(QFont("Segoe UI", 11, QFont.Bold))
        right_layout.addWidget(step3_title)

        step3_desc = QLabel(
            "Saves the calibrated orientation into BNO086 internal flash so it stays aligned after power-off."
        )
        step3_desc.setWordWrap(True)
        step3_desc.setObjectName("mutedText")
        right_layout.addWidget(step3_desc)

        self.btn_save = QPushButton("Save Calibration Permanently (Flash)")
        self.btn_save.setObjectName("successBtn")
        self.btn_save.setMinimumHeight(42)
        self.btn_save.clicked.connect(lambda: self._send_cmd("IMU_SAVE"))
        right_layout.addWidget(self.btn_save)

        self.btn_clear = QPushButton("Reset to Factory Default (Clear Saved)")
        self.btn_clear.setObjectName("dangerBtn")
        self.btn_clear.setMinimumHeight(34)
        self.btn_clear.clicked.connect(lambda: self._send_cmd("IMU_CLEAR"))
        right_layout.addWidget(self.btn_clear)

        right_layout.addStretch()
        mid_layout.addWidget(right_card, 4)
        main_layout.addLayout(mid_layout, 1)

        # Bottom Log
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumHeight(95)
        self.log_box.setPlaceholderText("Connection & calibration log...")
        main_layout.addWidget(self.log_box)

    def _make_metric(self, grid: QGridLayout, row: int, col: int, label: str, init_val: str) -> QLabel:
        box = QFrame()
        box.setObjectName("metricBox")
        lay = QVBoxLayout(box)
        lay.setContentsMargins(8, 6, 8, 6)
        lbl_t = QLabel(label)
        lbl_t.setObjectName("metricTitle")
        lbl_v = QLabel(init_val)
        lbl_v.setFont(QFont("Consolas", 12, QFont.Bold))
        lay.addWidget(lbl_t)
        lay.addWidget(lbl_v)
        grid.addWidget(box, row, col)
        return lbl_v

    def _apply_styles(self) -> None:
        self.setStyleSheet("""
            QMainWindow, QWidget { background-color: #0b0f17; color: #e2e8f0; font-family: 'Segoe UI'; }
            #card { background-color: #141b29; border: 1px solid #243047; border-radius: 10px; padding: 10px; }
            #metricBox { background-color: #0f141e; border: 1px solid #222e42; border-radius: 6px; }
            #metricTitle { color: #7c93b5; font-size: 10px; font-weight: bold; }
            #mutedText { color: #94a3b8; font-size: 12px; }
            #verifyBox { background-color: #0f172a; border: 1px solid #1e3a5f; border-radius: 6px; padding: 8px; font-weight: bold; }
            #statusPill, #hzPill { background-color: #1e293b; border-radius: 6px; padding: 5px 10px; font-weight: bold; }
            QPushButton { background-color: #1e293b; border: 1px solid #334155; border-radius: 6px; padding: 6px 12px; font-weight: bold; }
            QPushButton:hover { background-color: #2b3b52; }
            #primaryBtn { background-color: #0284c7; border-color: #0ea5e9; color: white; }
            #primaryBtn:hover { background-color: #0369a1; }
            #successBtn { background-color: #059669; border-color: #10b981; color: white; }
            #successBtn:hover { background-color: #047857; }
            #dangerBtn { background-color: #7f1d1d; border-color: #ef4444; color: #fecaca; }
            #dangerBtn:hover { background-color: #991b1b; }
            QComboBox, QLineEdit { background-color: #1e293b; border: 1px solid #334155; border-radius: 6px; padding: 5px; color: #e2e8f0; }
            QTextEdit { background-color: #0f141e; border: 1px solid #243047; border-radius: 6px; font-family: Consolas; font-size: 11px; color: #94a3b8; }
        """)

    def _on_mode_changed(self, idx: int) -> None:
        self.pi_bridge.disconnect_link()
        self.serial_bridge.disconnect_link()
        is_pi = (idx == 0)
        self.pi_host_edit.setVisible(is_pi)
        self.port_combo.setVisible(not is_pi)

    def _on_refresh_clicked(self) -> None:
        if self.mode_combo.currentIndex() == 0:
            host = resolve_pi_host(self.pi_host_edit.text().strip())
            if host:
                self.pi_host_edit.setText(host)
                self._append_log(f"[Auto-Find] Found Pi 4 at {host}:{PI_PORT}")
            else:
                self._append_log("[Auto-Find] No Pi 4 debug server responded on port 8080.")
        else:
            self._refresh_ports()

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        self.port_combo.clear()
        for p in serial.tools.list_ports.comports():
            self.port_combo.addItem(p.device)
        if current:
            idx = self.port_combo.findText(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)

    def _auto_connect(self) -> None:
        host = resolve_pi_host()
        if host:
            self.mode_combo.setCurrentIndex(0)
            self.pi_host_edit.setText(host)
            self.pi_bridge.connect_host(host)
            return
        port = detect_esp32_port()
        if port:
            self.mode_combo.setCurrentIndex(1)
            idx = self.port_combo.findText(port)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)
            self.serial_bridge.connect_port(port, DEFAULT_BAUD)

    def _toggle_connect(self) -> None:
        if self.active_bridge.is_connected:
            self.active_bridge.disconnect_link()
        elif self.mode_combo.currentIndex() == 0:
            self.pi_bridge.connect_host(self.pi_host_edit.text().strip())
        else:
            port = self.port_combo.currentText()
            if port:
                self.serial_bridge.connect_port(port, DEFAULT_BAUD)

    def _on_conn_changed(self, connected: bool, msg: str) -> None:
        self.btn_connect.setText("Disconnect" if connected else "Connect")
        self.lbl_status.setText("ONLINE" if connected else "DISCONNECTED")
        self.lbl_status.setStyleSheet(
            "background-color: #064e3b; color: #6ee7b7; border-radius: 6px; padding: 5px 10px; font-weight: bold;"
            if connected else
            "background-color: #3f1d24; color: #fda4af; border-radius: 6px; padding: 5px 10px; font-weight: bold;"
        )
        self._append_log(f"[Link] {msg}")

    def _send_cmd(self, cmd: str) -> None:
        if self.active_bridge.send_line(cmd):
            self._append_log(f">>> Sent: {cmd}")
        else:
            self._append_log("[Error] Not connected.")

    def _on_telemetry(self, data: dict) -> None:
        self._pkt_count += 1
        self.compass.update_state(data)

        if not data.get("online", 0):
            self.lbl_verify.setText("IMU hardware offline on ESP32!")
            return

        yaw = float(data.get("y", 0.0))
        pitch = float(data.get("p", 0.0))
        roll = float(data.get("r", 0.0))
        gz = float(data.get("gz", 0.0))
        lax = float(data.get("lax", 0.0))
        lay = float(data.get("lay", 0.0))

        self.lbl_yaw.setText(f"{yaw:+.1f}°")
        self.lbl_pitch.setText(f"{pitch:+.1f}°")
        self.lbl_roll.setText(f"{roll:+.1f}°")
        self.lbl_gz.setText(f"{gz:+.2f} rad/s")
        self.lbl_lax.setText(f"{lax:+.2f} m/s²")
        self.lbl_lay.setText(f"{lay:+.2f} m/s²")

        if abs(yaw) <= 3.0:
            self.lbl_verify.setText(f"ALIGNED: Heading {yaw:+.1f}° (pointing straight forward)")
            self.lbl_verify.setStyleSheet("background-color: #064e3b; border: 1px solid #10b981; border-radius: 6px; padding: 8px; font-weight: bold; color: #a7f3d0;")
        elif yaw > 3.0:
            self.lbl_verify.setText(f"TURNED LEFT: {yaw:+.1f}° (CCW from forward)")
            self.lbl_verify.setStyleSheet("background-color: #1e293b; border: 1px solid #f59e0b; border-radius: 6px; padding: 8px; font-weight: bold; color: #fde68a;")
        else:
            self.lbl_verify.setText(f"TURNED RIGHT: {yaw:+.1f}° (CW from forward)")
            self.lbl_verify.setStyleSheet("background-color: #1e293b; border: 1px solid #f59e0b; border-radius: 6px; padding: 8px; font-weight: bold; color: #fde68a;")

    def _update_hz(self) -> None:
        now = time.time()
        dt = now - self._last_hz_ts
        if dt > 0:
            hz = self._pkt_count / dt
            self.lbl_hz.setText(f"{hz:.0f} Hz")
        self._pkt_count = 0
        self._last_hz_ts = now

    def _append_log(self, text: str) -> None:
        self.log_box.append(text)

    def closeEvent(self, event) -> None:
        self.pi_bridge.disconnect_link()
        self.serial_bridge.disconnect_link()
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    win = ImuCalibWindow()
    win.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
