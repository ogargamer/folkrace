#!/usr/bin/env python3
"""
===============================================================================
DA WINNA — HIGH-RATE 333Hz DIGITAL SERVO TEST BENCH & GUI
===============================================================================
Target Hardware: ESP32D N4 (ESP32-D0WD / ESP32-WROOM-32D)
Servo Specifications:
  - Modulation:       Digital
  - Rotational Range: 180°
  - Pulse Cycle:      3 ms (Refresh Rate: 333 Hz)
  - Pulse Width:      500 µs - 2500 µs
  - Neutral Position: 1520 µs (Center / 90°)
  - Dead Band:        2 µs
  - Control System:   Hardware PWM (LEDC)

Recommended Pin: GPIO 18 (Safe, full digital output driver, glitch-free)
===============================================================================
"""

import sys
import os
import json
import time
import math
import threading
import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QFrame, QSizePolicy,
    QSlider, QDoubleSpinBox, QSpinBox, QTextEdit, QGroupBox, QSplitter
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QPointF, QRectF
from PyQt5.QtGui import (
    QColor, QFont, QPainter, QBrush, QPen, QLinearGradient, QRadialGradient,
    QPolygonF, QPainterPath
)

if hasattr(Qt, 'HighDpiScaleFactorRoundingPolicy'):
    QApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)


# =============================================================================
# SERIAL COMMUNICATION WORKER THREAD
# =============================================================================
class SerialWorker(QThread):
    telemetry_received = pyqtSignal(dict)
    log_message = pyqtSignal(str, str)  # (text, level)
    connection_status = pyqtSignal(bool, str)

    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.port_name = ""
        self.baud_rate = 115200
        self.is_running = False
        self._lock = threading.Lock()

    def connect_port(self, port_name, baud_rate=115200):
        self.port_name = port_name
        self.baud_rate = baud_rate
        try:
            self.serial_port = serial.Serial(self.port_name, self.baud_rate, timeout=0.1)
            self.is_running = True
            self.start()
            self.connection_status.emit(True, f"Connected to {self.port_name} @ {self.baud_rate} baud")
            self.log_message.emit(f"Connected to {self.port_name} @ {self.baud_rate} baud", "SUCCESS")
            # Query initial status
            self.send_command("STATUS")
            return True
        except Exception as e:
            self.connection_status.emit(False, str(e))
            self.log_message.emit(f"Failed to connect to {self.port_name}: {e}", "ERROR")
            return False

    def disconnect_port(self):
        self.is_running = False
        self.wait(500)
        with self._lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
                self.serial_port = None
        self.connection_status.emit(False, "Disconnected")
        self.log_message.emit("Disconnected from serial port.", "INFO")

    def send_command(self, cmd_str):
        with self._lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    payload = (cmd_str.strip() + "\n").encode('utf-8')
                    self.serial_port.write(payload)
                    self.serial_port.flush()
                    self.log_message.emit(f">> {cmd_str.strip()}", "TX")
                    return True
                except Exception as e:
                    self.log_message.emit(f"TX Error: {e}", "ERROR")
                    return False
        return False

    def run(self):
        rx_buffer = ""
        while self.is_running:
            try:
                if self.serial_port and self.serial_port.is_open:
                    if self.serial_port.in_waiting > 0:
                        raw = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                        rx_buffer += raw
                        while '\n' in rx_buffer:
                            line, rx_buffer = rx_buffer.split('\n', 1)
                            line = line.strip()
                            if not line:
                                continue
                            if line.startswith('{') and line.endswith('}'):
                                try:
                                    data = json.loads(line)
                                    self.telemetry_received.emit(data)
                                    self.log_message.emit(f"<< {line}", "RX")
                                except json.JSONDecodeError:
                                    self.log_message.emit(f"<< {line}", "INFO")
                            else:
                                self.log_message.emit(f"<< {line}", "INFO")
                    else:
                        time.sleep(0.005)
                else:
                    break
            except Exception as e:
                self.log_message.emit(f"Serial RX Error: {e}", "ERROR")
                break
        self.is_running = False


# =============================================================================
# CUSTOM VISUAL SERVO ARC & HORN DIAL WIDGET
# =============================================================================
class ServoArcWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 260)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.angle_deg = 90.0
        self.pulse_us = 1520
        self.is_enabled = True
        self.min_us = 500
        self.neutral_us = 1520
        self.max_us = 2500

    def update_state(self, angle_deg, pulse_us, is_enabled=True):
        self.angle_deg = angle_deg
        self.pulse_us = pulse_us
        self.is_enabled = is_enabled
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        width = self.width()
        height = self.height()

        # Dark background
        painter.fillRect(0, 0, width, height, QColor(15, 23, 42))

        # Center point and radius
        center_x = width / 2.0
        center_y = height * 0.72
        radius = min(width * 0.42, height * 0.58)

        # Draw outer bounding arc (180 degrees, from 180° to 0° standard polar)
        # Note: Qt angles are in 1/16th of a degree, 0 at 3 o'clock, CCW.
        # Servo 0° is left (180° polar), 90° is top (90° polar), 180° is right (0° polar).
        arc_rect = QRectF(center_x - radius, center_y - radius, radius * 2, radius * 2)

        # Background track
        track_pen = QPen(QColor(30, 41, 59), 16, Qt.SolidLine, Qt.RoundCap)
        painter.setPen(track_pen)
        painter.drawArc(arc_rect, 0 * 16, 180 * 16)

        # Active progress arc
        # Angle mapped: 0 deg -> left (180 polar), current angle -> (180 - angle) polar
        span_angle = -int(self.angle_deg * 16)
        active_color = QColor(14, 165, 233) if self.is_enabled else QColor(100, 116, 139)
        active_pen = QPen(active_color, 12, Qt.SolidLine, Qt.RoundCap)
        painter.setPen(active_pen)
        painter.drawArc(arc_rect, 180 * 16, span_angle)

        # Tick marks & Labels
        tick_pen = QPen(QColor(148, 163, 184), 2)
        font_tick = QFont("Consolas", 8, QFont.Bold)
        painter.setFont(font_tick)

        angles_to_mark = [
            (0, "0°\n500µs"),
            (45, "45°\n1010µs"),
            (90, "90° (NEUTRAL)\n1520µs"),
            (135, "135°\n2010µs"),
            (180, "180°\n2500µs"),
        ]

        for mark_deg, label in angles_to_mark:
            polar_rad = math.radians(180.0 - mark_deg)
            is_center = (mark_deg == 90)

            # Draw tick
            r_inner = radius - (16 if is_center else 10)
            r_outer = radius + (14 if is_center else 8)

            p_in = QPointF(center_x + r_inner * math.cos(polar_rad), center_y - r_inner * math.sin(polar_rad))
            p_out = QPointF(center_x + r_outer * math.cos(polar_rad), center_y - r_outer * math.sin(polar_rad))

            if is_center:
                painter.setPen(QPen(QColor(16, 185, 129), 3))
            else:
                painter.setPen(tick_pen)
            painter.drawLine(p_in, p_out)

            # Draw text label
            r_text = radius + 32
            tx = center_x + r_text * math.cos(polar_rad)
            ty = center_y - r_text * math.sin(polar_rad)

            text_rect = QRectF(tx - 50, ty - 18, 100, 36)
            painter.setPen(QColor(16, 185, 129) if is_center else QColor(148, 163, 184))
            painter.drawText(text_rect, Qt.AlignCenter, label)

        # Neutral 1520 µs Reference Indicator Line
        neutral_polar = math.radians(90.0)
        p_n1 = QPointF(center_x, center_y - 25)
        p_n2 = QPointF(center_x, center_y - radius + 15)
        painter.setPen(QPen(QColor(16, 185, 129, 120), 2, Qt.DashLine))
        painter.drawLine(p_n1, p_n2)

        # Servo Rotating Horn / Arm
        arm_polar = math.radians(180.0 - self.angle_deg)
        horn_length = radius * 0.88
        tip_x = center_x + horn_length * math.cos(arm_polar)
        tip_y = center_y - horn_length * math.sin(arm_polar)

        # Servo horn polygon (tapered needle)
        perp_polar = arm_polar + math.pi / 2.0
        base_w = 9.0
        b1_x = center_x + base_w * math.cos(perp_polar)
        b1_y = center_y - base_w * math.sin(perp_polar)
        b2_x = center_x - base_w * math.cos(perp_polar)
        b2_y = center_y + base_w * math.sin(perp_polar)

        horn_poly = QPolygonF([QPointF(b1_x, b1_y), QPointF(tip_x, tip_y), QPointF(b2_x, b2_y)])

        horn_color = QColor(6, 182, 212) if self.is_enabled else QColor(100, 116, 139)
        painter.setBrush(QBrush(horn_color))
        painter.setPen(QPen(QColor(255, 255, 255, 180), 1.5))
        painter.drawPolygon(horn_poly)

        # Central Hub
        painter.setBrush(QBrush(QColor(30, 41, 59)))
        painter.setPen(QPen(QColor(148, 163, 184), 2))
        painter.drawEllipse(QPointF(center_x, center_y), 20, 20)

        inner_hub_color = QColor(16, 185, 129) if self.is_enabled else QColor(239, 68, 68)
        painter.setBrush(QBrush(inner_hub_color))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(QPointF(center_x, center_y), 8, 8)

        # Digital HUD readout below hub
        font_main = QFont("Consolas", 18, QFont.Bold)
        painter.setFont(font_main)
        painter.setPen(QColor(241, 245, 249))
        text_deg = f"{self.angle_deg:.1f}°"
        painter.drawText(QRectF(center_x - 120, center_y + 26, 240, 32), Qt.AlignCenter, text_deg)

        font_sub = QFont("Consolas", 12, QFont.Bold)
        painter.setFont(font_sub)
        pulse_color = QColor(16, 185, 129) if abs(self.pulse_us - 1520) <= 2 else QColor(56, 189, 248)
        painter.setPen(pulse_color)
        text_us = f"{self.pulse_us} µs"
        if abs(self.pulse_us - 1520) <= 2:
            text_us += " (NEUTRAL)"
        painter.drawText(QRectF(center_x - 120, center_y + 56, 240, 24), Qt.AlignCenter, text_us)

        font_hz = QFont("Segoe UI", 9)
        painter.setFont(font_hz)
        painter.setPen(QColor(148, 163, 184))
        state_str = "OUTPUT: ACTIVE (333 Hz / 3.0 ms)" if self.is_enabled else "OUTPUT: DISABLED (FREE-WHEEL)"
        painter.drawText(QRectF(center_x - 140, center_y + 80, 280, 20), Qt.AlignCenter, state_str)


# =============================================================================
# MAIN BENCH GUI WINDOW
# =============================================================================
class ServoBenchGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DA WINNA — ESP32D N4 High-Rate 333Hz Digital Servo Test Bench")
        self.resize(1100, 780)
        self.setMinimumSize(950, 680)

        self.worker = SerialWorker()
        self.worker.telemetry_received.connect(self.on_telemetry_received)
        self.worker.log_message.connect(self.append_log)
        self.worker.connection_status.connect(self.on_connection_status)

        # State
        self.current_angle = 90.0
        self.current_pulse = 1520
        self.is_enabled = True

        # Sweep Automation State
        self.sweep_timer = QTimer(self)
        self.sweep_timer.timeout.connect(self.on_sweep_tick)
        self.sweep_running = False
        self.sweep_direction = 1
        self.sweep_min_us = 500
        self.sweep_max_us = 2500
        self.sweep_step_us = 20
        self.sweep_interval_ms = 30

        self.init_ui()
        self.apply_dark_theme()
        self.refresh_ports()

    def init_ui(self):
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(14, 14, 14, 14)
        main_layout.setSpacing(12)

        # ---------------------------------------------------------------------
        # TOP TOOLBAR: Connection & Status
        # ---------------------------------------------------------------------
        top_bar = QFrame()
        top_bar.setObjectName("topBarFrame")
        top_layout = QHBoxLayout(top_bar)
        top_layout.setContentsMargins(10, 8, 10, 8)
        top_layout.setSpacing(10)

        title_lbl = QLabel("ESP32D N4 SERVO BENCH")
        title_lbl.setFont(QFont("Consolas", 12, QFont.Bold))
        title_lbl.setStyleSheet("color: #38bdf8; letter-spacing: 1px;")
        top_layout.addWidget(title_lbl)

        spec_badge = QLabel("333 Hz | 3.0 ms Cycle | 500-2500 µs | Neutral 1520 µs")
        spec_badge.setFont(QFont("Segoe UI", 9))
        spec_badge.setStyleSheet("color: #94a3b8; background: #1e293b; padding: 4px 8px; border-radius: 4px;")
        top_layout.addWidget(spec_badge)

        top_layout.addStretch()

        port_lbl = QLabel("Port:")
        port_lbl.setStyleSheet("color: #cbd5e1; font-weight: bold;")
        top_layout.addWidget(port_lbl)

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(160)
        top_layout.addWidget(self.port_combo)

        refresh_btn = QPushButton("↻ Refresh")
        refresh_btn.clicked.connect(self.refresh_ports)
        top_layout.addWidget(refresh_btn)

        baud_lbl = QLabel("Baud:")
        baud_lbl.setStyleSheet("color: #cbd5e1; font-weight: bold;")
        top_layout.addWidget(baud_lbl)

        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "921600", "500000", "230400"])
        top_layout.addWidget(self.baud_combo)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setObjectName("connectBtn")
        self.connect_btn.clicked.connect(self.toggle_connection)
        top_layout.addWidget(self.connect_btn)

        self.status_led = QLabel("● DISCONNECTED")
        self.status_led.setFont(QFont("Consolas", 9, QFont.Bold))
        self.status_led.setStyleSheet("color: #ef4444; padding-left: 6px;")
        top_layout.addWidget(self.status_led)

        main_layout.addWidget(top_bar)

        # ---------------------------------------------------------------------
        # SPLIT VIEW: Left (Controls + Dial), Right (Sweep + Specs + Log)
        # ---------------------------------------------------------------------
        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(8)

        # LEFT CONTAINER
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(12)

        # 1. Visual Dial Widget
        self.arc_dial = ServoArcWidget()
        left_layout.addWidget(self.arc_dial, stretch=2)

        # 2. Manual Controls Box
        ctrl_group = QGroupBox("MANUAL POSITION CONTROL")
        ctrl_layout = QVBoxLayout(ctrl_group)
        ctrl_layout.setSpacing(10)

        # Angle Slider + SpinBox
        angle_row = QHBoxLayout()
        angle_lbl = QLabel("Angle:")
        angle_lbl.setFixedWidth(65)
        angle_lbl.setFont(QFont("Segoe UI", 9, QFont.Bold))
        angle_row.addWidget(angle_lbl)

        self.angle_slider = QSlider(Qt.Horizontal)
        self.angle_slider.setRange(0, 1800)  # 0.1 deg resolution (0 - 180.0)
        self.angle_slider.setValue(900)
        self.angle_slider.valueChanged.connect(self.on_angle_slider_changed)
        angle_row.addWidget(self.angle_slider)

        self.angle_spin = QDoubleSpinBox()
        self.angle_spin.setRange(0.0, 180.0)
        self.angle_spin.setSingleStep(0.5)
        self.angle_spin.setDecimals(1)
        self.angle_spin.setValue(90.0)
        self.angle_spin.setSuffix(" °")
        self.angle_spin.setFixedWidth(85)
        self.angle_spin.valueChanged.connect(self.on_angle_spin_changed)
        angle_row.addWidget(self.angle_spin)

        ctrl_layout.addLayout(angle_row)

        # Pulse Width Slider + SpinBox
        pulse_row = QHBoxLayout()
        pulse_lbl = QLabel("Pulse:")
        pulse_lbl.setFixedWidth(65)
        pulse_lbl.setFont(QFont("Segoe UI", 9, QFont.Bold))
        pulse_row.addWidget(pulse_lbl)

        self.pulse_slider = QSlider(Qt.Horizontal)
        self.pulse_slider.setRange(500, 2500)
        self.pulse_slider.setValue(1520)
        self.pulse_slider.valueChanged.connect(self.on_pulse_slider_changed)
        pulse_row.addWidget(self.pulse_slider)

        self.pulse_spin = QSpinBox()
        self.pulse_spin.setRange(500, 2500)
        self.pulse_spin.setSingleStep(1)
        self.pulse_spin.setValue(1520)
        self.pulse_spin.setSuffix(" µs")
        self.pulse_spin.setFixedWidth(85)
        self.pulse_spin.valueChanged.connect(self.on_pulse_spin_changed)
        pulse_row.addWidget(self.pulse_spin)

        ctrl_layout.addLayout(pulse_row)

        # Dead-band Micro-Nudge Row (2 µs deadband test!)
        nudge_row = QHBoxLayout()
        nudge_lbl = QLabel("Fine Nudge:")
        nudge_lbl.setFixedWidth(75)
        nudge_lbl.setStyleSheet("color: #94a3b8; font-size: 11px;")
        nudge_row.addWidget(nudge_lbl)

        btn_m10 = QPushButton("◀◀ -10 µs")
        btn_m10.clicked.connect(lambda: self.nudge_pulse(-10))
        nudge_row.addWidget(btn_m10)

        btn_m2 = QPushButton("◀ -2 µs (Deadband)")
        btn_m2.setStyleSheet("background-color: #334155; color: #38bdf8;")
        btn_m2.clicked.connect(lambda: self.nudge_pulse(-2))
        nudge_row.addWidget(btn_m2)

        btn_p2 = QPushButton("+2 µs (Deadband) ▶")
        btn_p2.setStyleSheet("background-color: #334155; color: #38bdf8;")
        btn_p2.clicked.connect(lambda: self.nudge_pulse(+2))
        nudge_row.addWidget(btn_p2)

        btn_p10 = QPushButton("+10 µs ▶▶")
        btn_p10.clicked.connect(lambda: self.nudge_pulse(+10))
        nudge_row.addWidget(btn_p10)

        ctrl_layout.addLayout(nudge_row)

        # Quick Presets Buttons
        preset_row = QHBoxLayout()
        btn_0 = QPushButton("0° (500µs)")
        btn_0.clicked.connect(lambda: self.set_preset_pulse(500))
        preset_row.addWidget(btn_0)

        btn_45 = QPushButton("45° (1010µs)")
        btn_45.clicked.connect(lambda: self.set_preset_pulse(1010))
        preset_row.addWidget(btn_45)

        btn_neutral = QPushButton("★ NEUTRAL (1520µs / 90°)")
        btn_neutral.setObjectName("neutralBtn")
        btn_neutral.clicked.connect(lambda: self.set_preset_pulse(1520))
        preset_row.addWidget(btn_neutral)

        btn_135 = QPushButton("135° (2010µs)")
        btn_135.clicked.connect(lambda: self.set_preset_pulse(2010))
        preset_row.addWidget(btn_135)

        btn_180 = QPushButton("180° (2500µs)")
        btn_180.clicked.connect(lambda: self.set_preset_pulse(2500))
        preset_row.addWidget(btn_180)

        ctrl_layout.addLayout(preset_row)

        # Output Enable / Disable Row
        pwr_row = QHBoxLayout()
        self.btn_disable = QPushButton("⚠ DISABLE PWM (Free-Wheel / Rest)")
        self.btn_disable.setObjectName("disableBtn")
        self.btn_disable.clicked.connect(self.disable_output)
        pwr_row.addWidget(self.btn_disable)

        self.btn_enable = QPushButton("✔ ENABLE PWM OUTPUT")
        self.btn_enable.setObjectName("enableBtn")
        self.btn_enable.clicked.connect(self.enable_output)
        pwr_row.addWidget(self.btn_enable)

        ctrl_layout.addLayout(pwr_row)

        left_layout.addWidget(ctrl_group, stretch=1)
        splitter.addWidget(left_widget)

        # RIGHT CONTAINER: Automated Sweep + Hardware Reference + Live Log
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(12)

        # 1. Automated Sweep & Burn-In Test Group
        sweep_group = QGroupBox("AUTOMATED SWEEP & ENDURANCE TEST")
        sweep_layout = QGridLayout(sweep_group)
        sweep_layout.setSpacing(8)

        sweep_layout.addWidget(QLabel("Min Pulse:"), 0, 0)
        self.sweep_min_spin = QSpinBox()
        self.sweep_min_spin.setRange(500, 2500)
        self.sweep_min_spin.setValue(600)
        self.sweep_min_spin.setSuffix(" µs")
        sweep_layout.addWidget(self.sweep_min_spin, 0, 1)

        sweep_layout.addWidget(QLabel("Max Pulse:"), 0, 2)
        self.sweep_max_spin = QSpinBox()
        self.sweep_max_spin.setRange(500, 2500)
        self.sweep_max_spin.setValue(2400)
        self.sweep_max_spin.setSuffix(" µs")
        sweep_layout.addWidget(self.sweep_max_spin, 0, 3)

        sweep_layout.addWidget(QLabel("Step Size:"), 1, 0)
        self.sweep_step_spin = QSpinBox()
        self.sweep_step_spin.setRange(1, 100)
        self.sweep_step_spin.setValue(20)
        self.sweep_step_spin.setSuffix(" µs")
        sweep_layout.addWidget(self.sweep_step_spin, 1, 1)

        sweep_layout.addWidget(QLabel("Step Interval:"), 1, 2)
        self.sweep_interval_spin = QSpinBox()
        self.sweep_interval_spin.setRange(5, 500)
        self.sweep_interval_spin.setValue(30)
        self.sweep_interval_spin.setSuffix(" ms")
        sweep_layout.addWidget(self.sweep_interval_spin, 1, 3)

        self.btn_toggle_sweep = QPushButton("▶ START AUTO SWEEP")
        self.btn_toggle_sweep.setObjectName("sweepBtn")
        self.btn_toggle_sweep.clicked.connect(self.toggle_sweep)
        sweep_layout.addWidget(self.btn_toggle_sweep, 2, 0, 1, 4)

        right_layout.addWidget(sweep_group)

        # 2. Hardware Wiring & Pin Guide Card
        pin_group = QGroupBox("ESP32-S3 HARDWARE & PINOUT SPECIFICATION")
        pin_layout = QVBoxLayout(pin_group)
        pin_info = QLabel(
            "<b>Recommended PWM Pin:</b> <span style='color: #38bdf8; font-size: 13px;'>GPIO 21</span> (SERVO_FRONT_PWM)<br>"
            "<b>Alternatives:</b> GPIO 38, GPIO 47, GPIO 1, GPIO 2<br>"
            "<b>Forbidden on S3:</b> GPIO 22-25 (Don't exist in silicon), GPIO 26-37 (Octal Flash/PSRAM), GPIO 46 (Input-only)<br><br>"
            "<b>Wiring Diagram:</b><br>"
            "  • <b>Signal (PWM / White or Orange):</b> Connect to <b>GPIO 21</b><br>"
            "  • <b>GND (Black or Brown):</b> Connect to <b>ESP32 GND</b> (Common Ground)<br>"
            "  • <b>VCC (Red):</b> Connect to <b>External 5.0V - 7.4V/8.4V BEC / Battery</b><br>"
            "<span style='color: #f59e0b;'>⚠ CAUTION:</span> Never power high-torque digital servos directly from the ESP32 3.3V or 5V regulator pin!"
        )
        pin_info.setTextFormat(Qt.RichText)
        pin_info.setWordWrap(True)
        pin_layout.addWidget(pin_info)
        right_layout.addWidget(pin_group)

        # 3. Serial Log Console
        log_group = QGroupBox("SERIAL TELEMETRY & EVENT LOG")
        log_layout = QVBoxLayout(log_group)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 8))
        log_layout.addWidget(self.log_text)

        log_btn_row = QHBoxLayout()
        clear_log_btn = QPushButton("Clear Log")
        clear_log_btn.clicked.connect(self.log_text.clear)
        log_btn_row.addStretch()
        log_btn_row.addWidget(clear_log_btn)
        log_layout.addLayout(log_btn_row)

        right_layout.addWidget(log_group, stretch=1)
        splitter.addWidget(right_widget)

        splitter.setStretchFactor(0, 5)
        splitter.setStretchFactor(1, 4)
        main_layout.addWidget(splitter, stretch=1)

    # -------------------------------------------------------------------------
    # STYLING (Dark Mode Theme)
    # -------------------------------------------------------------------------
    def apply_dark_theme(self):
        self.setStyleSheet("""
            QMainWindow {
                background-color: #0f172a;
            }
            QWidget {
                background-color: #0f172a;
                color: #e2e8f0;
                font-family: 'Segoe UI', sans-serif;
            }
            QFrame#topBarFrame {
                background-color: #1e293b;
                border: 1px solid #334155;
                border-radius: 8px;
            }
            QGroupBox {
                background-color: #1e293b;
                border: 1px solid #334155;
                border-radius: 8px;
                margin-top: 18px;
                padding-top: 14px;
                font-size: 11px;
                font-weight: bold;
                color: #38bdf8;
                letter-spacing: 0.5px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 2px 8px;
                background-color: #0f172a;
                border: 1px solid #334155;
                border-radius: 4px;
            }
            QPushButton {
                background-color: #334155;
                color: #f8fafc;
                border: 1px solid #475569;
                border-radius: 5px;
                padding: 6px 12px;
                font-weight: bold;
                font-size: 11px;
            }
            QPushButton:hover {
                background-color: #475569;
                border-color: #64748b;
            }
            QPushButton:pressed {
                background-color: #1e293b;
            }
            QPushButton#connectBtn {
                background-color: #0284c7;
                border-color: #38bdf8;
            }
            QPushButton#connectBtn:hover {
                background-color: #0369a1;
            }
            QPushButton#neutralBtn {
                background-color: #065f46;
                color: #a7f3d0;
                border: 1px solid #10b981;
            }
            QPushButton#neutralBtn:hover {
                background-color: #047857;
            }
            QPushButton#disableBtn {
                background-color: #7f1d1d;
                color: #fecaca;
                border: 1px solid #ef4444;
            }
            QPushButton#disableBtn:hover {
                background-color: #991b1b;
            }
            QPushButton#enableBtn {
                background-color: #064e3b;
                color: #a7f3d0;
                border: 1px solid #10b981;
            }
            QPushButton#enableBtn:hover {
                background-color: #065f46;
            }
            QPushButton#sweepBtn {
                background-color: #6366f1;
                color: #ffffff;
                border: 1px solid #818cf8;
                padding: 8px;
            }
            QPushButton#sweepBtn:hover {
                background-color: #4f46e5;
            }
            QComboBox {
                background-color: #1e293b;
                color: #f8fafc;
                border: 1px solid #475569;
                border-radius: 4px;
                padding: 4px 8px;
                min-height: 22px;
            }
            QComboBox::drop-down {
                border: none;
                width: 20px;
            }
            QComboBox QAbstractItemView {
                background-color: #1e293b;
                color: #f8fafc;
                selection-background-color: #0284c7;
            }
            QSlider::groove:horizontal {
                height: 8px;
                background: #334155;
                border-radius: 4px;
            }
            QSlider::sub-page:horizontal {
                background: #0284c7;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #38bdf8;
                border: 2px solid #f8fafc;
                width: 18px;
                margin-top: -5px;
                margin-bottom: -5px;
                border-radius: 9px;
            }
            QSlider::handle:horizontal:hover {
                background: #7dd3fc;
            }
            QSpinBox, QDoubleSpinBox {
                background-color: #1e293b;
                color: #f8fafc;
                border: 1px solid #475569;
                border-radius: 4px;
                padding: 4px;
                font-family: 'Consolas', monospace;
                font-weight: bold;
            }
            QTextEdit {
                background-color: #0b1120;
                color: #cbd5e1;
                border: 1px solid #334155;
                border-radius: 6px;
            }
            QSplitter::handle {
                background-color: #1e293b;
            }
        """)

    # -------------------------------------------------------------------------
    # SERIAL CONNECTIONS & TELEMETRY
    # -------------------------------------------------------------------------
    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_combo.addItem(f"{p.device} ({p.description})", p.device)
        if not ports:
            self.port_combo.addItem("No COM ports found", "")

    def toggle_connection(self):
        if self.worker.is_running:
            self.stop_sweep()
            self.worker.disconnect_port()
        else:
            port = self.port_combo.currentData()
            if not port:
                self.append_log("Please select a valid COM port.", "ERROR")
                return
            baud = int(self.baud_combo.currentText())
            self.worker.connect_port(port, baud)

    def on_connection_status(self, connected, message):
        if connected:
            self.connect_btn.setText("Disconnect")
            self.connect_btn.setStyleSheet("background-color: #ef4444; border-color: #f87171;")
            self.status_led.setText("● CONNECTED")
            self.status_led.setStyleSheet("color: #10b981; padding-left: 6px;")
        else:
            self.connect_btn.setText("Connect")
            self.connect_btn.setStyleSheet("")
            self.status_led.setText("● DISCONNECTED")
            self.status_led.setStyleSheet("color: #ef4444; padding-left: 6px;")

    def on_telemetry_received(self, data):
        if "pulse_us" in data:
            self.current_pulse = data["pulse_us"]
        if "angle" in data:
            self.current_angle = data["angle"]
        if "enabled" in data:
            self.is_enabled = bool(data["enabled"])

        # Sync UI without feedback loop
        self.sync_controls_from_state()

    def append_log(self, text, level="INFO"):
        colors = {
            "TX": "#38bdf8",
            "RX": "#4ade80",
            "SUCCESS": "#34d399",
            "ERROR": "#f87171",
            "INFO": "#94a3b8"
        }
        color = colors.get(level, "#cbd5e1")
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.append(f"<span style='color: #64748b;'>[{timestamp}]</span> <span style='color: {color};'>{text}</span>")
        sb = self.log_text.verticalScrollBar()
        sb.setValue(sb.maximum())

    # -------------------------------------------------------------------------
    # POSITION CONTROL & SYNC
    # -------------------------------------------------------------------------
    def on_angle_slider_changed(self, val):
        angle = val / 10.0
        if abs(angle - self.current_angle) > 0.05:
            self.set_angle(angle)

    def on_angle_spin_changed(self, angle):
        if abs(angle - self.current_angle) > 0.05:
            self.set_angle(angle)

    def on_pulse_slider_changed(self, pulse):
        if pulse != self.current_pulse:
            self.set_pulse(pulse)

    def on_pulse_spin_changed(self, pulse):
        if pulse != self.current_pulse:
            self.set_pulse(pulse)

    def nudge_pulse(self, delta_us):
        target = self.current_pulse + delta_us
        self.set_pulse(target)

    def set_preset_pulse(self, pulse_us):
        self.set_pulse(pulse_us)

    def set_pulse(self, pulse_us):
        pulse_us = max(500, min(2500, int(pulse_us)))
        self.current_pulse = pulse_us
        # Piecewise calculation for display
        if pulse_us <= 1520:
            self.current_angle = ((pulse_us - 500) / 1020.0) * 90.0
        else:
            self.current_angle = 90.0 + ((pulse_us - 1520) / 980.0) * 90.0

        self.sync_controls_from_state()
        self.worker.send_command(f"P{pulse_us}")

    def set_angle(self, angle_deg):
        angle_deg = max(0.0, min(180.0, float(angle_deg)))
        self.current_angle = angle_deg
        if angle_deg <= 90.0:
            self.current_pulse = int(500 + (angle_deg / 90.0) * (1520 - 500) + 0.5)
        else:
            self.current_pulse = int(1520 + ((angle_deg - 90.0) / 90.0) * (2500 - 1520) + 0.5)

        self.sync_controls_from_state()
        self.worker.send_command(f"A{angle_deg:.2f}")

    def sync_controls_from_state(self):
        # Update arc dial
        self.arc_dial.update_state(self.current_angle, self.current_pulse, self.is_enabled)

        # Block signals to prevent recursion
        self.angle_slider.blockSignals(True)
        self.angle_spin.blockSignals(True)
        self.pulse_slider.blockSignals(True)
        self.pulse_spin.blockSignals(True)

        self.angle_slider.setValue(int(round(self.current_angle * 10.0)))
        self.angle_spin.setValue(self.current_angle)
        self.pulse_slider.setValue(self.current_pulse)
        self.pulse_spin.setValue(self.current_pulse)

        self.angle_slider.blockSignals(False)
        self.angle_spin.blockSignals(False)
        self.pulse_slider.blockSignals(False)
        self.pulse_spin.blockSignals(False)

    def disable_output(self):
        self.is_enabled = False
        self.arc_dial.update_state(self.current_angle, self.current_pulse, False)
        self.worker.send_command("DISABLE")

    def enable_output(self):
        self.is_enabled = True
        self.arc_dial.update_state(self.current_angle, self.current_pulse, True)
        self.worker.send_command("ENABLE")

    # -------------------------------------------------------------------------
    # AUTOMATED SWEEP MODE
    # -------------------------------------------------------------------------
    def toggle_sweep(self):
        if self.sweep_running:
            self.stop_sweep()
        else:
            self.start_sweep()

    def start_sweep(self):
        self.sweep_min_us = self.sweep_min_spin.value()
        self.sweep_max_us = self.sweep_max_spin.value()
        self.sweep_step_us = self.sweep_step_spin.value()
        self.sweep_interval_ms = self.sweep_interval_spin.value()

        if self.sweep_min_us >= self.sweep_max_us:
            self.append_log("Sweep Error: Min pulse must be strictly less than Max pulse.", "ERROR")
            return

        self.sweep_running = True
        self.btn_toggle_sweep.setText("■ STOP AUTO SWEEP")
        self.btn_toggle_sweep.setStyleSheet("background-color: #ef4444; border-color: #f87171;")
        self.sweep_timer.start(self.sweep_interval_ms)
        self.append_log(f"Auto Sweep Started: {self.sweep_min_us} -> {self.sweep_max_us} µs", "INFO")

    def stop_sweep(self):
        self.sweep_running = False
        self.sweep_timer.stop()
        self.btn_toggle_sweep.setText("▶ START AUTO SWEEP")
        self.btn_toggle_sweep.setStyleSheet("")
        self.append_log("Auto Sweep Stopped.", "INFO")

    def on_sweep_tick(self):
        next_pulse = self.current_pulse + (self.sweep_step_us * self.sweep_direction)
        if next_pulse >= self.sweep_max_us:
            next_pulse = self.sweep_max_us
            self.sweep_direction = -1
        elif next_pulse <= self.sweep_min_us:
            next_pulse = self.sweep_min_us
            self.sweep_direction = 1

        self.set_pulse(next_pulse)

    def closeEvent(self, event):
        self.stop_sweep()
        self.worker.disconnect_port()
        event.accept()


# =============================================================================
# ENTRY POINT
# =============================================================================
def main():
    app = QApplication(sys.argv)
    window = ServoBenchGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
