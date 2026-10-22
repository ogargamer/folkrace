#!/usr/bin/env python3
"""
===============================================================================
DA WINNA — VL53L8CX MULTIZONE TIME-OF-FLIGHT CONTROL & TELEMETRY SUITE
===============================================================================
Features:
  - Smart Fallback Multi-Target Detection (Target #1 with Target #2 Fallback)
  - Optical Signal Rate (kcps/SPAD), Range Sigma Noise (mm), Ambient Light & SPAD Counts
  - ST Table 4 Target Status Interpretation & Confidence Filtering (100% vs 50% vs Errors)
  - Continuous High-Speed 60 Hz 4x4 Ranging with Optimal 15 ms Integration Time Budget
  - Closest vs Strongest Target Order Selection
  - Veiling Glare Sharpener Tuning (0-99%, ST Rev 12 Default = 1%)
  - Crosstalk (Xtalk) Calibration Suite & Xtalk Margin Configuration
  - High-Visibility 2D Depth & Confidence Matrix Heatmap (Expanded Full-Width)
  - Historical Depth Telemetry Oscilloscope (Real-time Envelope & Selected Zone Trace)
  - Silicon Temperature (°C) Telemetry Monitoring
"""

import sys
import os
import re
import json
import time
import threading
import numpy as np
import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QSlider, QSpinBox,
    QGroupBox, QFrame, QSplitter, QTabWidget, QTextEdit, QProgressBar,
    QSizePolicy, QSpacerItem, QRadioButton, QButtonGroup, QCheckBox,
    QTableWidget, QTableWidgetItem, QHeaderView, QScrollArea
)

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QObject, QPointF
from PyQt5.QtGui import QColor, QFont, QPainter, QBrush, QPen, QLinearGradient, QPixmap, QPolygonF

if hasattr(Qt, 'HighDpiScaleFactorRoundingPolicy'):
    QApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)


# =============================================================================
# ST UM3109 TABLE 4 TARGET STATUS DICTIONARY & HELPERS
# =============================================================================

STATUS_INFO = {
    0: ("Not Updated", "#78909C", 0),
    1: ("Signal Low (SPAD)", "#EF4444", 0),
    2: ("Target Phase", "#AB47BC", 0),
    3: ("Sigma Too High", "#FF7043", 0),
    4: ("Consistency Fail", "#EC407A", 0),
    5: ("Range Valid (100%)", "#00E676", 100),
    6: ("Wrap Around (50%)", "#00E5FF", 50),
    7: ("Rate Consistency", "#FF5252", 0),
    8: ("Signal Low (Target)", "#EF4444", 0),
    9: ("Large Pulse/Merged (50%)", "#FFB300", 50),
    10: ("Valid (No Prev)", "#26A69A", 50),
    11: ("Measurement Fail", "#E53935", 0),
    12: ("Blurred by Sharpener", "#7E57C2", 0),
    13: ("Secondary Inconsistent", "#8D6E63", 25),
    255: ("No Target", "#374151", 0),
}

def get_status_info(st):
    return STATUS_INFO.get(st, (f"Status {st}", "#78909C", 0))


# =============================================================================
# ASYNCHRONOUS SERIAL COMMUNICATIONS THREAD
# =============================================================================

class SerialEngine(QThread):
    """
    Producer thread. Telemetry frames are NOT emitted as Qt signals (that floods the
    GUI event queue when rendering is slower than the sensor). Instead the newest
    frame is stored in a locked slot; the GUI pulls it with take_latest() on its own
    timer (latest-frame-wins). Sequence numbers are tracked here so loss is measured
    on every received packet, not only on the rendered ones.
    """
    config_received = pyqtSignal(dict)
    ack_received = pyqtSignal(dict)
    connection_changed = pyqtSignal(bool, str)
    raw_log = pyqtSignal(str)

    RX_BUFFER_BYTES = 1 << 20

    def __init__(self, port="COM7", baud=921600):
        super().__init__()
        self.port = port
        self.baud = baud
        self.ser = None
        self._running = False

        self._lock = threading.Lock()
        self._latest = None
        self.reset_stats()

    def reset_stats(self):
        with self._lock:
            self.rx_packets = 0
            self.lost_packets = 0
            self.parse_errors = 0
            self.last_seq = None
            self.last_rx_time = 0.0
            self.max_gap_s = 0.0
            self.rx_high_water = 0
            self._latest = None
            self.current_dt_ms = 0.0
            self.peak_dt_ms = 0.0
            self.spike_count = 0
            self.recent_dts = []

    def take_latest(self):
        """Return the newest unconsumed telemetry packet (or None) and clear the slot."""
        with self._lock:
            pkt = self._latest
            self._latest = None
        return pkt

    def get_stats(self):
        with self._lock:
            mean_dt = float(np.mean(self.recent_dts)) if self.recent_dts else 0.0
            jitter = float(np.std(self.recent_dts)) if len(self.recent_dts) > 1 else 0.0
            return {
                "rx": self.rx_packets,
                "lost": self.lost_packets,
                "bad": self.parse_errors,
                "last_rx": self.last_rx_time,
                "max_gap": self.max_gap_s,
                "hw": self.rx_high_water,
                "dt_ms": self.current_dt_ms,
                "mean_dt": mean_dt,
                "jitter": jitter,
                "peak_dt": self.peak_dt_ms,
                "spikes": self.spike_count,
            }

    def _store_frame(self, packet):
        now = time.perf_counter()
        seq = packet.get("seq")
        with self._lock:
            if self.last_rx_time:
                dt_ms = (now - self.last_rx_time) * 1000.0
                self.current_dt_ms = dt_ms
                if dt_ms > self.peak_dt_ms:
                    self.peak_dt_ms = dt_ms
                if dt_ms > 50.0:
                    self.spike_count += 1
                self.recent_dts.append(dt_ms)
                if len(self.recent_dts) > 60:
                    self.recent_dts.pop(0)
                self.max_gap_s = max(self.max_gap_s, now - self.last_rx_time)
            self.last_rx_time = now
            self.rx_packets += 1
            if isinstance(seq, int):
                if self.last_seq is not None:
                    gap = seq - self.last_seq
                    if 1 < gap < 1000000:
                        self.lost_packets += gap - 1
                self.last_seq = seq
            self._latest = packet

    def set_connection_params(self, port, baud):
        self.port = port
        self.baud = baud

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
            self.send_command("GET_CONFIG")
        except Exception as e:
            self._running = False
            self.connection_changed.emit(False, f"Connection Failed: {str(e)}")
            return

        buffer = ""
        while self._running:
            try:
                waiting = self.ser.in_waiting if (self.ser and self.ser.is_open) else 0
                if waiting:
                    if waiting > self.rx_high_water:
                        self.rx_high_water = waiting
                    raw = self.ser.read(waiting).decode('utf-8', errors='replace')
                    buffer += raw
                    if len(buffer) > 32768:
                        buffer = buffer[-4096:]
                        if "\n" in buffer:
                            _, buffer = buffer.split("\n", 1)
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue

                        if line.startswith("{") and line.endswith("}"):
                            try:
                                packet = json.loads(line)
                                ptype = packet.get("type", packet.get("t", ""))
                                if ptype == "D":
                                    self._store_frame(packet)
                                elif ptype == "CONFIG":
                                    self.config_received.emit(packet)
                                    try: self.raw_log.emit(line)
                                    except RuntimeError: break
                                elif ptype == "ACK":
                                    self.ack_received.emit(packet)
                                    try: self.raw_log.emit(line)
                                    except RuntimeError: break
                                elif ptype == "SYSTEM_BOOT":
                                    self.send_command("GET_CONFIG")
                                    try: self.raw_log.emit(line)
                                    except RuntimeError: break
                            except Exception:
                                with self._lock:
                                    self.parse_errors += 1
                        else:
                            try: self.raw_log.emit(line)
                            except RuntimeError: break
                else:
                    time.sleep(0.002)
            except Exception as e:
                if self._running:
                    try: self.connection_changed.emit(False, f"Serial Error: {str(e)}")
                    except RuntimeError: pass
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
                try: self.raw_log.emit(f">>> {cmd_str}")
                except RuntimeError: pass
            except Exception as e:
                try: self.raw_log.emit(f"[TX ERROR] {str(e)}")
                except RuntimeError: pass

    def stop(self):
        self._running = False
        if self.ser and self.ser.is_open:
            try: self.ser.close()
            except Exception: pass
        self.wait(500)


# =============================================================================
# COLORMAP ENGINE
# =============================================================================

def turbo_colormap(val):
    """Normalized [0.0, 1.0] -> RGB QColor (Near=Hot Orange/Red, Far=Deep Blue)"""
    x = max(0.0, min(1.0, val))
    r = int(255 * max(0.0, min(1.0, 0.1357 + x * (4.5974 - x * (42.681 - x * (153.27 - x * (205.8 - x * 96.347)))))))
    g = int(255 * max(0.0, min(1.0, 0.0914 + x * (2.1856 + x * (4.8052 - x * (14.019 + x * (4.2109 + x * 2.7747)))))))
    b = int(255 * max(0.0, min(1.0, 0.1067 + x * (12.583 - x * (60.118 - x * (109.07 - x * (88.5 - x * 26.85)))))))
    return QColor(r, g, b)

def amber_colormap(val):
    """Normalized [0.0, 1.0] -> Amber/Gold intensity for Signal & SPADs"""
    x = max(0.0, min(1.0, val))
    r = int(min(255, 30 + x * 225))
    g = int(min(255, 20 + x * 180))
    b = int(min(255, 40 + x * 50))
    return QColor(r, g, b)


# =============================================================================
# 2D MATRIX MULTI-METRIC HEATMAP WIDGET (TARGET 1 & TARGET 2 SMART FALLBACK)
# =============================================================================

class MatrixHeatmapWidget(QFrame):
    cell_selected = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet("background-color: #10141e; border-radius: 8px; border: 1px solid #232a3d;")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.resolution = 16
        self.grid_dim = 4
        self.nb_targets = 2  # Only Target 1 and Target 2

        # Multi-target data matrices (zones, 2)
        self.distances = np.zeros((16, 2), dtype=int)
        self.statuses = np.zeros((16, 2), dtype=int)
        self.sigmas = np.zeros((16, 2), dtype=int)
        self.signals = np.zeros((16, 2), dtype=int)
        self.target_counts = np.zeros(16, dtype=int)
        self.ambient = np.zeros(16, dtype=int)
        self.spads = np.zeros(16, dtype=int)

        self.selected_zone = 0
        self.target_layer = 100  # Default to 100 = Smart Fallback (Target 1 -> Target 2)
        self.active_metric = "dist"  # "dist", "sig", "sigm", "amb", "spad", "ntar", "st"
        
        # Pickiness & Quality Filters
        self.status_filter = "valid_5_6_9"  # "valid_5", "valid_5_6_9", "valid_extended", "all"
        self.max_sigma_filter = 0    # 0 = disabled, or 1..30 mm (max noise tolerance)
        self.min_signal_filter = 0   # 0 = disabled, or 1..100 kcps (min signal rate threshold)
        self.min_dist_gate = 20      # mm
        self.max_dist_gate = 4000    # mm

        self.min_dist_range = 50
        self.max_dist_range = 3000

        # Spatial transformation parameters
        self.flip_h = False
        self.flip_v = False
        self.rotation = 0

    def is_valid_reading(self, dist, st, sig=None, sigm=None):
        if dist <= 0:
            return False
        
        # 1. Status Filter
        if self.status_filter == "valid_5":
            if st != 5: return False
        elif self.status_filter == "valid_5_6_9":
            if st not in (5, 6, 9): return False
        elif self.status_filter == "valid_extended":
            if st not in (5, 6, 9, 10, 13): return False
        
        # 2. Distance Range Gate
        if dist < self.min_dist_gate or dist > self.max_dist_gate:
            return False
            
        # 3. Noise Sigma Limit (rejection of noisy signals)
        if self.max_sigma_filter > 0 and sigm is not None and sigm > self.max_sigma_filter:
            return False
            
        # 4. Minimum Signal Threshold (rejection of weak pulses)
        if self.min_signal_filter > 0 and sig is not None and sig < self.min_signal_filter:
            return False
            
        return True

    def update_telemetry(self, res, nt, ntar, dists, stats, sigs, sigms, ambs, spads_list):
        self.resolution = res
        self.grid_dim = 8 if res == 64 else 4
        raw_nt = nt if nt > 0 else 4
        n_zones = res
        tot_elems = n_zones * raw_nt

        # Reshape incoming raw data to (n_zones, raw_nt) and slice only Target 1 & Target 2
        d_arr = np.array(dists[:tot_elems], dtype=int) if len(dists) >= tot_elems else np.zeros(tot_elems, dtype=int)
        d_mat = d_arr.reshape((n_zones, raw_nt))
        self.distances = d_mat[:, :2]

        s_arr = np.array(stats[:tot_elems], dtype=int) if len(stats) >= tot_elems else np.zeros(tot_elems, dtype=int)
        s_mat = s_arr.reshape((n_zones, raw_nt))
        self.statuses = s_mat[:, :2]

        sig_arr = np.array(sigs[:tot_elems], dtype=int) if len(sigs) >= tot_elems else np.zeros(tot_elems, dtype=int)
        sig_mat = sig_arr.reshape((n_zones, raw_nt))
        self.signals = sig_mat[:, :2]

        sigm_arr = np.array(sigms[:tot_elems], dtype=int) if len(sigms) >= tot_elems else np.zeros(tot_elems, dtype=int)
        sigm_mat = sigm_arr.reshape((n_zones, raw_nt))
        self.sigmas = sigm_mat[:, :2]

        self.target_counts = np.array(ntar[:n_zones], dtype=int) if len(ntar) >= n_zones else np.zeros(n_zones, dtype=int)
        self.ambient = np.array(ambs[:n_zones], dtype=int) if len(ambs) >= n_zones else np.zeros(n_zones, dtype=int)
        self.spads = np.array(spads_list[:n_zones], dtype=int) if len(spads_list) >= n_zones else np.zeros(n_zones, dtype=int)

        self.update()

    def get_zone_data(self, z_idx):
        if z_idx >= self.resolution:
            return None
        return {
            "ntar": int(self.target_counts[z_idx]),
            "amb": int(self.ambient[z_idx]),
            "spad": int(self.spads[z_idx]),
            "targets": [
                {
                    "idx": t,
                    "dist": int(self.distances[z_idx, t]),
                    "stat": int(self.statuses[z_idx, t]),
                    "sig": int(self.signals[z_idx, t]),
                    "sigm": int(self.sigmas[z_idx, t]),
                }
                for t in range(min(2, self.distances.shape[1]))
            ]
        }

    def get_transformed_grid(self, data_1d):
        dim = self.grid_dim
        grid = np.array(data_1d[:dim*dim]).reshape((dim, dim))
        if self.flip_h:
            grid = np.fliplr(grid)
        if self.flip_v:
            grid = np.flipud(grid)
        if self.rotation == 90:
            grid = np.rot90(grid, -1)
        elif self.rotation == 180:
            grid = np.rot90(grid, 2)
        elif self.rotation == 270:
            grid = np.rot90(grid, 1)
        return grid

    def mousePressEvent(self, event):
        w = self.width()
        h = self.height()
        cell_w = w / self.grid_dim
        cell_h = h / self.grid_dim
        col = int(event.x() // cell_w)
        row = int(event.y() // cell_h)
        col = max(0, min(self.grid_dim - 1, col))
        row = max(0, min(self.grid_dim - 1, row))

        zones_grid = self.get_transformed_grid(np.arange(self.resolution))
        idx = int(zones_grid[row, col])
        if idx < self.resolution:
            self.selected_zone = idx
            self.cell_selected.emit(idx)
            self.update()

    def get_effective_layer_data(self):
        """
        Returns (dists, stats, sigs, sigms, tgt_indices) for current target_layer.
        If target_layer == 100 ('Smart Fallback'):
            For each zone, uses Target #1 if valid; otherwise automatically falls back
            to Target #2 if Target #2 is valid.
        """
        n_zones = self.resolution
        if self.target_layer == 100:
            dists = np.zeros(n_zones, dtype=int)
            stats = np.zeros(n_zones, dtype=int)
            sigs = np.zeros(n_zones, dtype=int)
            sigms = np.zeros(n_zones, dtype=int)
            tgt_indices = np.zeros(n_zones, dtype=int)

            for z in range(n_zones):
                # 1. Test Primary Target (Target #1, index 0)
                d0 = self.distances[z, 0] if len(self.distances) > z else 0
                st0 = self.statuses[z, 0] if len(self.statuses) > z else 0
                sig0 = self.signals[z, 0] if len(self.signals) > z else 0
                sigm0 = self.sigmas[z, 0] if len(self.sigmas) > z else 0

                if self.is_valid_reading(d0, st0, sig=sig0, sigm=sigm0):
                    dists[z] = d0
                    stats[z] = st0
                    sigs[z] = sig0
                    sigms[z] = sigm0
                    tgt_indices[z] = 0
                else:
                    # 2. Primary has no reading or is filtered; search secondary target (Target #2, index 1)
                    d1 = self.distances[z, 1] if (len(self.distances) > z and self.distances.shape[1] > 1) else 0
                    st1 = self.statuses[z, 1] if (len(self.statuses) > z and self.statuses.shape[1] > 1) else 0
                    sig1 = self.signals[z, 1] if (len(self.signals) > z and self.signals.shape[1] > 1) else 0
                    sigm1 = self.sigmas[z, 1] if (len(self.sigmas) > z and self.sigmas.shape[1] > 1) else 0

                    if self.is_valid_reading(d1, st1, sig=sig1, sigm=sigm1) or (d1 > 0 and st1 in (5, 6, 9, 10, 13)):
                        dists[z] = d1
                        stats[z] = st1
                        sigs[z] = sig1
                        sigms[z] = sigm1
                        tgt_indices[z] = 1
                    else:
                        dists[z] = d0
                        stats[z] = st0
                        sigs[z] = sig0
                        sigms[z] = sigm0
                        tgt_indices[z] = 0

            return dists, stats, sigs, sigms, tgt_indices
        else:
            t_idx = 0 if self.target_layer <= 0 else min(1, self.target_layer)
            dists = self.distances[:, t_idx] if len(self.distances) else np.zeros(n_zones, dtype=int)
            stats = self.statuses[:, t_idx] if len(self.statuses) else np.zeros(n_zones, dtype=int)
            sigs = self.signals[:, t_idx] if len(self.signals) else np.zeros(n_zones, dtype=int)
            sigms = self.sigmas[:, t_idx] if len(self.sigmas) else np.zeros(n_zones, dtype=int)
            tgt_indices = np.full(n_zones, t_idx, dtype=int)
            return dists, stats, sigs, sigms, tgt_indices

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        cell_w = w / self.grid_dim
        cell_h = h / self.grid_dim

        zones_grid = self.get_transformed_grid(np.arange(self.resolution))
        dists_layer, stats_layer, sigs_layer, sigms_layer, tgt_idx_layer = self.get_effective_layer_data()

        grid_d = self.get_transformed_grid(dists_layer)
        grid_st = self.get_transformed_grid(stats_layer)
        grid_sig = self.get_transformed_grid(sigs_layer)
        grid_sigm = self.get_transformed_grid(sigms_layer)
        grid_tgt_idx = self.get_transformed_grid(tgt_idx_layer)
        grid_nt = self.get_transformed_grid(self.target_counts)
        grid_amb = self.get_transformed_grid(self.ambient)
        grid_spad = self.get_transformed_grid(self.spads)

        for r in range(self.grid_dim):
            for c in range(self.grid_dim):
                x = c * cell_w
                y = r * cell_h

                dist = int(grid_d[r, c])
                st = int(grid_st[r, c])
                sig = int(grid_sig[r, c])
                sigm = int(grid_sigm[r, c])
                tgt_idx = int(grid_tgt_idx[r, c])
                nt = min(2, int(grid_nt[r, c]))
                amb = int(grid_amb[r, c])
                spad = int(grid_spad[r, c])
                z_idx = int(zones_grid[r, c])

                valid = self.is_valid_reading(dist, st, sig=sig, sigm=sigm)
                is_fallback = (self.target_layer == 100 and tgt_idx == 1 and valid)

                # Determine Tile Background Color based on Active Metric
                if not valid and self.active_metric in ("dist", "sig", "sigm"):
                    cell_color = QColor(20, 24, 35)
                else:
                    if self.active_metric == "dist":
                        norm = (dist - self.min_dist_range) / max(1.0, (self.max_dist_range - self.min_dist_range))
                        cell_color = turbo_colormap(1.0 - norm)
                    elif self.active_metric == "sig":
                        norm = min(1.0, max(0.0, sig / 500.0))
                        cell_color = amber_colormap(norm)
                    elif self.active_metric == "sigm":
                        norm = min(1.0, max(0.0, sigm / 20.0))
                        cell_color = turbo_colormap(norm)
                    elif self.active_metric == "amb":
                        norm = min(1.0, max(0.0, amb / 200.0))
                        cell_color = turbo_colormap(norm)
                    elif self.active_metric == "spad":
                        norm = min(1.0, max(0.0, spad / 250.0))
                        cell_color = amber_colormap(norm)
                    elif self.active_metric == "ntar":
                        norm = min(1.0, max(0.0, nt / 2.0))
                        cell_color = turbo_colormap(norm)
                    elif self.active_metric == "st":
                        _, col_hex, _ = get_status_info(st)
                        cell_color = QColor(col_hex)
                    else:
                        cell_color = QColor(20, 24, 35)

                # Background rounded tile
                painter.setBrush(QBrush(cell_color))
                if z_idx == self.selected_zone:
                    painter.setPen(QPen(QColor("#00E5FF"), 2.5))
                elif is_fallback:
                    painter.setPen(QPen(QColor("#38BDF8"), 2.0, Qt.DashLine))
                else:
                    painter.setPen(QPen(QColor("#141824"), 1))
                painter.drawRoundedRect(int(x + 2), int(y + 2), int(cell_w - 4), int(cell_h - 4), 6, 6)

                # --- Line 1: Primary Value (Large Bold) ---
                if self.active_metric == "dist":
                    txt_line1 = f"{dist} mm" if (valid and self.grid_dim == 4) else (f"{dist}" if valid else "---")
                elif self.active_metric == "sig":
                    txt_line1 = f"{sig} kcps" if self.grid_dim == 4 else f"{sig}k"
                elif self.active_metric == "sigm":
                    txt_line1 = f"±{sigm} mm"
                elif self.active_metric == "amb":
                    txt_line1 = f"{amb} kcps" if self.grid_dim == 4 else f"{amb}k"
                elif self.active_metric == "spad":
                    txt_line1 = f"{spad} SP"
                elif self.active_metric == "ntar":
                    txt_line1 = f"{nt} Targets" if self.grid_dim == 4 else f"{nt} TGT"
                elif self.active_metric == "st":
                    txt_line1 = f"Status {st}" if self.grid_dim == 4 else f"s{st}"
                else:
                    txt_line1 = f"{dist} mm" if valid else "---"

                target_pt1 = 15.0 if self.grid_dim == 4 else 8.5
                if len(txt_line1) > 6:
                    target_pt1 = 12.0 if self.grid_dim == 4 else 7.0
                painter.setFont(QFont("Segoe UI", int(target_pt1), QFont.Bold))
                # Slight shadow
                painter.setPen(QColor(0, 0, 0, 200))
                painter.drawText(int(x + 1), int(y + cell_h * 0.08 + 1), int(cell_w), int(cell_h * 0.38), Qt.AlignCenter, txt_line1)
                painter.setPen(QColor(255, 255, 255))
                painter.drawText(int(x), int(y + cell_h * 0.08), int(cell_w), int(cell_h * 0.38), Qt.AlignCenter, txt_line1)

                # --- Line 2: Secondary Info / Smart Fallback Sub-Badge ---
                sub_w = int(cell_w * 0.84 if self.grid_dim == 4 else cell_w * 0.88)
                sub_h = int(cell_h * 0.24 if self.grid_dim == 4 else cell_h * 0.24)
                sub_x = int(x + (cell_w - sub_w) / 2)
                sub_y = int(y + cell_h * 0.44 if self.grid_dim == 4 else y + cell_h * 0.42)
                sub_r = 4 if self.grid_dim == 4 else 3

                if is_fallback:
                    badge_bg = QColor(14, 116, 144, 230)
                    badge_fg = QColor("#E0F2FE")
                    badge_border = QColor("#38BDF8")
                    txt_sub = "🥈 T2 Fallback" if self.grid_dim == 4 else "T2 FB"
                elif valid:
                    badge_bg = QColor(18, 24, 38, 210)
                    badge_fg = QColor("#CBD5E1")
                    badge_border = QColor(35, 45, 65)
                    txt_sub = f"Sig: {sig}k | ±{sigm}mm" if self.grid_dim == 4 else f"{sig}k"
                else:
                    badge_bg = QColor(25, 28, 38, 180)
                    badge_fg = QColor("#94A3B8")
                    badge_border = QColor(40, 48, 65)
                    txt_sub = "No Target" if self.grid_dim == 4 else "---"

                painter.setBrush(QBrush(badge_bg))
                painter.setPen(QPen(badge_border, 1.2))
                painter.drawRoundedRect(sub_x, sub_y, sub_w, sub_h, sub_r, sub_r)

                target_pt2 = 9.5 if self.grid_dim == 4 else 7.0
                painter.setFont(QFont("Segoe UI", max(4, int(target_pt2)), QFont.Bold))
                painter.setPen(badge_fg)
                painter.drawText(sub_x, sub_y, sub_w, sub_h, Qt.AlignCenter, txt_sub)

                # --- Line 3: Bottom Pill Badge (Zone ID + Status) ---
                pill_w = int(cell_w * 0.80 if self.grid_dim == 4 else cell_w * 0.88)
                pill_h = int(cell_h * 0.20 if self.grid_dim == 4 else cell_h * 0.22)
                pill_x = int(x + (cell_w - pill_w) / 2)
                pill_y = int(y + cell_h * 0.72 if self.grid_dim == 4 else y + cell_h * 0.69)
                pill_r = 3 if self.grid_dim == 4 else 2

                painter.setBrush(QBrush(QColor(10, 14, 24, 215)))
                painter.setPen(QPen(QColor(40, 50, 70, 180), 1))
                painter.drawRoundedRect(pill_x, pill_y, pill_w, pill_h, pill_r, pill_r)

                status_desc, st_hex, _ = get_status_info(st)
                tgt_dot = " • T2" if is_fallback else (" • T1" if valid else "")

                if self.grid_dim == 4:
                    sub_txt = f"Z{z_idx} • s{st}{tgt_dot}"
                    pill_pt = 8.5
                else:
                    sub_txt = f"Z{z_idx} s{st}"
                    pill_pt = 6.5

                painter.setFont(QFont("Segoe UI", max(4, int(pill_pt)), QFont.Bold))
                painter.setPen(QColor(st_hex))
                painter.drawText(pill_x, pill_y, pill_w, pill_h, Qt.AlignCenter, sub_txt)


# =============================================================================
# HISTORICAL DISTANCE TELEMETRY OSCILLOSCOPE (DEPTH STRIP-CHART)
# =============================================================================

class TelemetryStripChart(QWidget):
    """Native QPainter oscilloscope (min/max envelope, mean and selected-zone traces)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumHeight(160)

        self.max_history = 120
        self.hist_min_d = []
        self.hist_mean_d = []
        self.hist_max_d = []
        self.hist_sel_d = []

        self.last_selected_idx = 0

    def add_telemetry_point(self, dists_layer, selected_idx):
        valid_d = [d for d in dists_layer if d > 0]
        if not valid_d:
            return

        min_d = min(valid_d)
        mean_d = float(np.mean(valid_d))
        max_d = max(valid_d)
        sel_d = dists_layer[selected_idx] if selected_idx < len(dists_layer) and dists_layer[selected_idx] > 0 else mean_d

        self.hist_min_d.append(min_d)
        self.hist_mean_d.append(mean_d)
        self.hist_max_d.append(max_d)
        self.hist_sel_d.append(sel_d)
        self.last_selected_idx = selected_idx

        if len(self.hist_mean_d) > self.max_history:
            self.hist_min_d.pop(0)
            self.hist_mean_d.pop(0)
            self.hist_max_d.pop(0)
            self.hist_sel_d.pop(0)

    def redraw_chart(self):
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor("#10141e"))

        ml, mr, mt, mb = 52, 12, 12, 24
        pw, ph = max(1, w - ml - mr), max(1, h - mt - mb)
        p.fillRect(ml, mt, pw, ph, QColor("#151a27"))

        n = len(self.hist_mean_d)
        max_y = max(2500, max(self.hist_max_d) + 200) if n else 2500
        x_span = max(self.max_history, n)

        def px(i):
            return ml + pw * i / max(1, x_span - 1)

        def py(v):
            return mt + ph * (1.0 - min(max(v, 0.0), max_y) / max_y)

        # Grid + Y labels
        p.setFont(QFont("Segoe UI", 8))
        grid_pen = QPen(QColor(40, 50, 74, 110), 1, Qt.DashLine)
        step = 500 if max_y <= 4000 else 1000
        v = 0
        while v <= max_y:
            y = py(v)
            p.setPen(grid_pen)
            p.drawLine(int(ml), int(y), int(ml + pw), int(y))
            p.setPen(QColor("#8fa0be"))
            p.drawText(0, int(y) - 8, ml - 6, 16, Qt.AlignRight | Qt.AlignVCenter, str(v))
            v += step
        p.setPen(QColor("#00E5FF"))
        p.drawText(ml + 4, 0, 160, mt + 12, Qt.AlignLeft | Qt.AlignVCenter, "Depth (mm)")
        p.setPen(QColor("#8fa0be"))
        p.drawText(ml, h - mb + 4, pw, mb - 4, Qt.AlignCenter, "Frames")

        if n >= 2:
            # Min/max envelope
            poly = QPolygonF()
            for i in range(n):
                poly.append(QPointF(px(i), py(self.hist_max_d[i])))
            for i in range(n - 1, -1, -1):
                poly.append(QPointF(px(i), py(self.hist_min_d[i])))
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(0, 229, 255, 40))
            p.drawPolygon(poly)
            p.setBrush(Qt.NoBrush)

            sel_poly = QPolygonF([QPointF(px(i), py(self.hist_sel_d[i])) for i in range(n)])
            p.setPen(QPen(QColor("#00E676"), 1.4, Qt.DashLine))
            p.drawPolyline(sel_poly)

            mean_poly = QPolygonF([QPointF(px(i), py(self.hist_mean_d[i])) for i in range(n)])
            p.setPen(QPen(QColor("#00E5FF"), 1.8))
            p.drawPolyline(mean_poly)

        # Legend
        lx = ml + pw - 150
        p.setPen(QPen(QColor("#232a3d"), 1))
        p.setBrush(QColor(16, 20, 30, 220))
        p.drawRect(lx, mt + 4, 144, 38)
        p.setPen(QPen(QColor("#00E5FF"), 1.8))
        p.drawLine(lx + 6, mt + 14, lx + 26, mt + 14)
        p.setPen(QPen(QColor("#00E676"), 1.4, Qt.DashLine))
        p.drawLine(lx + 6, mt + 30, lx + 26, mt + 30)
        p.setPen(QColor("#ffffff"))
        p.drawText(lx + 32, mt + 6, 110, 16, Qt.AlignLeft | Qt.AlignVCenter, "Mean Depth")
        p.drawText(lx + 32, mt + 22, 110, 16, Qt.AlignLeft | Qt.AlignVCenter, f"Zone {self.last_selected_idx} Depth")
        p.setPen(QPen(QColor("#232a3d"), 1))
        p.setBrush(Qt.NoBrush)
        p.drawRect(ml, mt, pw, ph)


# =============================================================================
# MAIN DASHBOARD APPLICATION
# =============================================================================

class VL53L8CXDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DA WINNA — VL53L8CX Multizone Time-of-Flight Live Engine & Tuning Suite")

        screen = QApplication.primaryScreen()
        if screen:
            avail = screen.availableGeometry()
            self.setGeometry(
                avail.x() + int(avail.width() * 0.08),
                avail.y() + int(avail.height() * 0.06),
                int(avail.width() * 0.84),
                int(avail.height() * 0.86)
            )
        else:
            self.resize(1280, 800)

        self.setStyleSheet("""
            QMainWindow { background-color: #0b0d14; }
            QWidget { color: #e2e8f0; font-family: 'Segoe UI', -apple-system, sans-serif; font-size: 8.5pt; }
            
            QGroupBox { 
                background-color: #12151f; 
                border: 1px solid #232a3d; 
                border-radius: 6px; 
                margin-top: 8px; 
                font-weight: bold; 
                font-size: 8.5pt; 
                color: #00E5FF; 
                padding-top: 8px;
            }
            QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; left: 8px; padding: 0 4px; }
            QLabel { font-size: 8.5pt; color: #cbd5e1; }
            
            QComboBox { 
                background-color: #181d2b; 
                border: 1px solid #333f5c; 
                border-radius: 4px; 
                padding: 2px 6px; 
                color: #ffffff; 
                font-size: 8.5pt;
                min-height: 22px;
            }
            QComboBox:hover { border-color: #00E5FF; }
            QComboBox::drop-down { border: none; width: 16px; }
            QComboBox QAbstractItemView {
                background-color: #181d2b;
                border: 1px solid #00E5FF;
                border-radius: 4px;
                selection-background-color: #00B4D8;
                selection-color: #ffffff;
                color: #e2e8f0;
                outline: none;
            }

            QSlider::groove:horizontal { height: 4px; background: #1c2233; border-radius: 2px; }
            QSlider::sub-page:horizontal { background: #00E5FF; border-radius: 2px; }
            QSlider::handle:horizontal { background: #ffffff; border: 2px solid #00E5FF; width: 10px; margin-top: -3px; margin-bottom: -3px; border-radius: 5px; }
            
            QSpinBox { 
                background-color: #181d2b; 
                border: 1px solid #333f5c; 
                border-radius: 4px; 
                padding: 2px; 
                color: #00E5FF; 
                font-weight: bold; 
                font-size: 8.5pt;
                min-height: 20px;
            }
            
            QPushButton { 
                background-color: #1c2233; 
                border: 1px solid #333f5c; 
                border-radius: 4px; 
                padding: 4px 8px; 
                color: #ffffff; 
                font-weight: bold; 
                font-size: 8.5pt;
            }
            QPushButton:hover { background-color: #28324a; border-color: #00E5FF; }
            QPushButton:pressed { background-color: #0f1422; }
            
            QPushButton#applyBtn { 
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #00B4D8, stop:1 #0077B6); 
                border: none; 
                font-size: 9pt; 
                padding: 8px; 
                color: white; 
                border-radius: 4px;
            }
            QPushButton#applyBtn:hover { background: #0096C7; }
            QPushButton#resetBtn { background-color: #7f1d1d; border-color: #991b1b; color: #fca5a5; }
            QPushButton#resetBtn:hover { background-color: #991b1b; }
            QPushButton#calBtn { background-color: #1e3a8a; border-color: #2563eb; color: #93c5fd; }
            QPushButton#calBtn:hover { background-color: #2563eb; color: white; }
            
            QTabWidget::pane { border: 1px solid #232a3d; background: #10141e; border-radius: 6px; }
            QTabBar::tab { background: #181d2b; color: #8fa0be; padding: 5px 12px; border-top-left-radius: 4px; border-top-right-radius: 4px; margin-right: 2px; font-size: 8.5pt; }
            QTabBar::tab:selected { background: #00E5FF; color: #0b0d14; font-weight: bold; }
        """)

        self.serial_engine = SerialEngine()
        self.serial_engine.config_received.connect(self.on_config_received)
        self.serial_engine.ack_received.connect(self.on_ack_received)
        self.serial_engine.connection_changed.connect(self.on_connection_changed)
        self.serial_engine.raw_log.connect(self.on_raw_log)

        # Pull-based rendering: the UI consumes only the newest frame at a fixed rate.
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.ui_tick)
        self.ui_timer.start(16)

        # Command ACK watchdog so Apply/Reset never leaves the UI locked on a lost ACK.
        self.ack_timer = QTimer(self)
        self.ack_timer.setSingleShot(True)
        self.ack_timer.timeout.connect(self.on_ack_timeout)

        self.last_frame_wall = 0.0
        self.diag_prev_rx = 0
        self.diag_prev_time = time.perf_counter()

        self.fps_timer = QTimer(self)
        self.fps_timer.timeout.connect(self.update_fps_calc)
        self.fps_timer.start(500)
        self.frame_counter = 0
        self.last_fps_time = time.perf_counter()
        self.current_fps = 0.0

        self.init_ui()
        self.refresh_com_ports()

        # Auto-connect to COM7
        self.port_combo.setCurrentText("COM7")
        self.toggle_connection()

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(6)

        # ---------------------------------------------------------------------
        # TOP SLIM NAVBAR (36px fixed height)
        # ---------------------------------------------------------------------
        nav_frame = QFrame()
        nav_frame.setFixedHeight(36)
        nav_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        nav_frame.setStyleSheet("background-color: #12151f; border-radius: 6px; border: 1px solid #232a3d;")
        nav_layout = QHBoxLayout(nav_frame)
        nav_layout.setContentsMargins(10, 2, 10, 2)
        nav_layout.setSpacing(8)
        nav_layout.setAlignment(Qt.AlignVCenter)

        title_lbl = QLabel("🎯 VL53L8CX CONTINUOUS TIME-OF-FLIGHT ENGINE")
        title_lbl.setStyleSheet("font-size: 9.5pt; font-weight: bold; color: #00E5FF; letter-spacing: 0.5px;")
        nav_layout.addWidget(title_lbl)

        nav_layout.addStretch()

        self.temp_badge = QLabel("🌡️ -- °C")
        self.temp_badge.setFixedHeight(24)
        self.temp_badge.setStyleSheet("background-color: #181d2b; color: #38bdf8; font-weight: bold; padding: 2px 8px; border-radius: 4px; border: 1px solid #232a3d; font-size: 8pt;")
        nav_layout.addWidget(self.temp_badge)

        self.fps_badge = QLabel("0.0 FPS")
        self.fps_badge.setFixedHeight(24)
        self.fps_badge.setStyleSheet("background-color: #0f1422; color: #00E676; font-weight: bold; padding: 2px 8px; border-radius: 4px; border: 1px solid #1c2233; font-size: 8pt;")
        nav_layout.addWidget(self.fps_badge)

        self.latency_badge = QLabel("⏱️ -- ms")
        self.latency_badge.setFixedHeight(24)
        self.latency_badge.setToolTip("Inter-frame arrival time ± jitter. Peak latency and lag spikes (>50ms) counter.")
        self.latency_badge.setStyleSheet("background-color: #0f1422; color: #00E5FF; font-weight: bold; padding: 2px 8px; border-radius: 4px; border: 1px solid #1c2233; font-size: 8pt;")
        nav_layout.addWidget(self.latency_badge)

        self.diag_badge = QLabel("RX -- Hz | loss -- | age -- ms")
        self.diag_badge.setFixedHeight(24)
        self.diag_badge.setToolTip("RX = frames/s received over serial, loss = missing sequence numbers, "
                                   "age = time since last frame, buf = peak OS RX buffer usage")
        self.diag_badge.setStyleSheet("background-color: #0f1422; color: #94a3b8; font-weight: bold; padding: 2px 8px; border-radius: 4px; border: 1px solid #1c2233; font-size: 8pt;")
        nav_layout.addWidget(self.diag_badge)

        nav_layout.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(75)
        self.port_combo.setFixedHeight(24)
        nav_layout.addWidget(self.port_combo)

        nav_layout.addWidget(QLabel("Baud:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["921600", "460800", "115200"])
        self.baud_combo.setFixedHeight(24)
        nav_layout.addWidget(self.baud_combo)

        self.connect_btn = QPushButton("🔌 Connect")
        self.connect_btn.setFixedHeight(24)
        self.connect_btn.clicked.connect(self.toggle_connection)
        nav_layout.addWidget(self.connect_btn)

        self.status_pill = QLabel("DISCONNECTED")
        self.status_pill.setFixedHeight(24)
        self.status_pill.setStyleSheet("background-color: #232a3d; color: #cbd5e1; font-weight: bold; padding: 2px 8px; border-radius: 4px; font-size: 8pt;")
        nav_layout.addWidget(self.status_pill)

        main_layout.addWidget(nav_frame, 0)

        # ---------------------------------------------------------------------
        # MAIN BODY SPLITTER (VISUALIZERS + CONTROL PANEL)
        # ---------------------------------------------------------------------
        body_splitter = QSplitter(Qt.Horizontal)
        body_splitter.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # === LEFT / CENTER: VISUALIZATION TABS ===
        vis_container = QWidget()
        vis_layout = QVBoxLayout(vis_container)
        vis_layout.setContentsMargins(0, 0, 0, 0)

        self.vis_tabs = QTabWidget()

        # Tab 1: 2D Matrix Heatmap (Expanded Full-Width)
        tab_2d = QWidget()
        tab_2d_layout = QVBoxLayout(tab_2d)
        tab_2d_layout.setContentsMargins(4, 4, 4, 4)
        tab_2d_layout.setSpacing(4)

        # Heatmap Filter & Layer Toolbar
        toolbar = QHBoxLayout()
        toolbar.setSpacing(6)

        toolbar.addWidget(QLabel("📊 Metric:"))
        self.metric_combo = QComboBox()
        self.metric_combo.addItem("📏 Distance (mm)", "dist")
        self.metric_combo.addItem("📶 Signal Rate (kcps/SPAD)", "sig")
        self.metric_combo.addItem("〰️ Range Noise (±mm)", "sigm")
        self.metric_combo.addItem("💡 Ambient Noise (kcps)", "amb")
        self.metric_combo.addItem("⚡ Active SPADs Count", "spad")
        self.metric_combo.addItem("🎯 Target Count (1-2)", "ntar")
        self.metric_combo.addItem("🏷️ Target Status & Validity", "st")
        self.metric_combo.currentIndexChanged.connect(self.on_metric_changed)
        toolbar.addWidget(self.metric_combo)

        toolbar.addWidget(QLabel("🎯 Target Layer:"))
        self.layer_combo = QComboBox()
        self.layer_combo.addItem("🔄 Smart Fallback (T1 → T2)", 100)
        self.layer_combo.addItem("🥇 Target #1 (Primary)", 0)
        self.layer_combo.addItem("🥈 Target #2 (Secondary)", 1)
        self.layer_combo.currentIndexChanged.connect(self.on_layer_changed)
        toolbar.addWidget(self.layer_combo)

        toolbar.addWidget(QLabel("🛡️ Filter:"))
        self.filter_combo = QComboBox()
        self.filter_combo.addItem("Valid & Merged (s5, s6, s9)", "valid_5_6_9")
        self.filter_combo.addItem("100% Valid Only (s5)", "valid_5")
        self.filter_combo.addItem("Show All (Include Errors)", "all")
        self.filter_combo.currentIndexChanged.connect(self.on_filter_changed)
        toolbar.addWidget(self.filter_combo)

        toolbar.addStretch()

        # Orientation buttons
        self.btn_flip_h = QPushButton("🪞 H")
        self.btn_flip_h.setCheckable(True)
        self.btn_flip_h.setFixedHeight(22)
        self.btn_flip_h.setToolTip("Mirror matrix horizontally")
        self.btn_flip_h.clicked.connect(self.on_toggle_flip_h)
        toolbar.addWidget(self.btn_flip_h)

        self.btn_flip_v = QPushButton("↕️ V")
        self.btn_flip_v.setCheckable(True)
        self.btn_flip_v.setFixedHeight(22)
        self.btn_flip_v.setToolTip("Flip matrix vertically")
        self.btn_flip_v.clicked.connect(self.on_toggle_flip_v)
        toolbar.addWidget(self.btn_flip_v)

        self.btn_rotate = QPushButton("🔄 0°")
        self.btn_rotate.setFixedHeight(22)
        self.btn_rotate.setToolTip("Rotate matrix 90° CW")
        self.btn_rotate.clicked.connect(self.on_rotate_clicked)
        toolbar.addWidget(self.btn_rotate)

        tab_2d_layout.addLayout(toolbar)

        # Quick Live Frame KPI Banner
        kpi_bar = QHBoxLayout()
        kpi_bar.setSpacing(14)
        
        self.kpi_mean_lbl = QLabel("Mean Depth: --- mm")
        self.kpi_mean_lbl.setStyleSheet("font-weight: bold; color: #00E5FF; font-size: 8.5pt;")
        kpi_bar.addWidget(self.kpi_mean_lbl)

        self.kpi_valid_lbl = QLabel("Valid Targets: ---")
        self.kpi_valid_lbl.setStyleSheet("font-weight: bold; color: #00E676; font-size: 8.5pt;")
        kpi_bar.addWidget(self.kpi_valid_lbl)

        self.kpi_sel_lbl = QLabel("Selected: Zone #0")
        self.kpi_sel_lbl.setStyleSheet("color: #94A3B8; font-size: 8.5pt;")
        kpi_bar.addWidget(self.kpi_sel_lbl)

        kpi_bar.addStretch()
        tab_2d_layout.addLayout(kpi_bar)

        self.heatmap = MatrixHeatmapWidget()
        self.heatmap.cell_selected.connect(self.on_cell_selected)
        tab_2d_layout.addWidget(self.heatmap, 1)

        self.vis_tabs.addTab(tab_2d, "🗺️ 2D Matrix Heatmap")

        # Tab 2: Historical Distance Oscilloscope
        self.strip_chart = TelemetryStripChart()
        self.vis_tabs.addTab(self.strip_chart, "📈 Distance Oscilloscope")

        # Tab 3: Raw Serial Logs
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("background-color: #0c0f17; color: #8b949e; font-family: 'Consolas', monospace; font-size: 8pt;")
        self.vis_tabs.addTab(self.log_text, "📜 Telemetry & Protocol Log")

        vis_layout.addWidget(self.vis_tabs)
        body_splitter.addWidget(vis_container)

        # === RIGHT PANEL: UNIFIED CONFIGURATION & TUNING SUITE ===
        ctrl_container = QWidget()
        ctrl_layout = QVBoxLayout(ctrl_container)
        ctrl_layout.setContentsMargins(2, 0, 2, 0)
        ctrl_layout.setSpacing(6)

        # --- Card 1: Optical & Ranging Parameters ---
        param_group = QGroupBox("⚙️ SENSOR RANGING PARAMETERS")
        param_layout = QVBoxLayout(param_group)
        param_layout.setContentsMargins(6, 6, 6, 6)
        param_layout.setSpacing(4)

        # Resolution & Target Order (Default: 4x4, Closest)
        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Resolution:"))
        self.res_combo = QComboBox()
        self.res_combo.addItem("4x4 (16 zones)", 16)
        self.res_combo.addItem("8x8 (64 zones)", 64)
        self.res_combo.currentIndexChanged.connect(self.on_resolution_changed)
        row1.addWidget(self.res_combo)

        row1.addWidget(QLabel("Order:"))
        self.order_combo = QComboBox()
        self.order_combo.addItem("Closest Target (1)", 1)
        self.order_combo.addItem("Strongest Target (2)", 2)
        row1.addWidget(self.order_combo)
        param_layout.addLayout(row1)

        # Ranging Frequency Slider (Default: 60 Hz)
        freq_layout = QHBoxLayout()
        self.freq_lbl = QLabel("Rate: 60 Hz")
        self.freq_lbl.setMinimumWidth(75)
        freq_layout.addWidget(self.freq_lbl)
        self.freq_slider = QSlider(Qt.Horizontal)
        self.freq_slider.setRange(1, 60)
        self.freq_slider.setValue(60)
        freq_layout.addWidget(self.freq_slider)
        self.freq_spin = QSpinBox()
        self.freq_spin.setRange(1, 60)
        self.freq_spin.setValue(60)
        self.freq_spin.setFixedWidth(46)
        freq_layout.addWidget(self.freq_spin)
        self.freq_slider.valueChanged.connect(self.freq_spin.setValue)
        self.freq_spin.valueChanged.connect(self.freq_slider.setValue)
        self.freq_spin.valueChanged.connect(self.on_freq_changed)
        param_layout.addLayout(freq_layout)

        # Integration Time Slider & Live SpinBox (Default: Optimal 15 ms for 60 Hz)
        integ_layout = QHBoxLayout()
        self.integ_lbl = QLabel("Integ Time: 15 ms")
        self.integ_lbl.setMinimumWidth(85)
        integ_layout.addWidget(self.integ_lbl)
        self.integ_slider = QSlider(Qt.Horizontal)
        self.integ_slider.setRange(2, 200)
        self.integ_slider.setValue(15)
        integ_layout.addWidget(self.integ_slider)
        self.integ_spin = QSpinBox()
        self.integ_spin.setRange(2, 1000)
        self.integ_spin.setValue(15)
        self.integ_spin.setSuffix(" ms")
        self.integ_spin.setFixedWidth(52)
        integ_layout.addWidget(self.integ_spin)
        self.integ_slider.valueChanged.connect(self.integ_spin.setValue)
        self.integ_spin.valueChanged.connect(self.integ_slider.setValue)
        self.integ_spin.valueChanged.connect(lambda val: self.integ_lbl.setText(f"Integ Time: {val} ms"))
        param_layout.addLayout(integ_layout)

        # Smart Integration Time Preset Buttons
        integ_btn_row = QHBoxLayout()
        integ_btn_row.setSpacing(3)

        btn_integ_auto = QPushButton("⚡ Auto / Max (15ms)")
        btn_integ_auto.setToolTip("Set integration time to maximum possible for current frequency budget (15 ms @ 60 Hz)")
        btn_integ_auto.clicked.connect(lambda: self.set_smart_integration("auto"))
        integ_btn_row.addWidget(btn_integ_auto)

        btn_integ_fast = QPushButton("🚀 Fast (5ms)")
        btn_integ_fast.setToolTip("Fast integration time (5 ms) for high-speed racing / minimal motion blur")
        btn_integ_fast.clicked.connect(lambda: self.set_smart_integration("fast"))
        integ_btn_row.addWidget(btn_integ_fast)

        btn_integ_bal = QPushButton("⚖️ Optimal Bal (15ms)")
        btn_integ_bal.setToolTip("Optimal balanced integration time (15 ms) for 60 Hz maximum SNR")
        btn_integ_bal.clicked.connect(lambda: self.set_smart_integration("balanced"))
        integ_btn_row.addWidget(btn_integ_bal)

        btn_integ_deep = QPushButton("🔭 Deep (50ms)")
        btn_integ_deep.setToolTip("Deep photon accumulation (50 ms) for lower frequency ranging")
        btn_integ_deep.clicked.connect(lambda: self.set_smart_integration("deep"))
        integ_btn_row.addWidget(btn_integ_deep)

        param_layout.addLayout(integ_btn_row)

        # Veiling Glare Sharpener Slider
        sharp_layout = QHBoxLayout()
        self.sharp_lbl = QLabel("Sharpener: 1%")
        self.sharp_lbl.setMinimumWidth(85)
        sharp_layout.addWidget(self.sharp_lbl)
        self.sharp_slider = QSlider(Qt.Horizontal)
        self.sharp_slider.setRange(0, 99)
        self.sharp_slider.setValue(1)
        sharp_layout.addWidget(self.sharp_slider)
        self.sharp_spin = QSpinBox()
        self.sharp_spin.setRange(0, 99)
        self.sharp_spin.setValue(1)
        self.sharp_spin.setFixedWidth(46)
        sharp_layout.addWidget(self.sharp_spin)
        self.sharp_slider.valueChanged.connect(self.sharp_spin.setValue)
        self.sharp_spin.valueChanged.connect(self.sharp_slider.setValue)
        self.sharp_spin.valueChanged.connect(lambda val: self.sharp_lbl.setText(f"Sharpener: {val}%"))
        param_layout.addLayout(sharp_layout)

        ctrl_layout.addWidget(param_group)

        # --- Card 2: Sensor Pickiness & Quality Gating ---
        picky_group = QGroupBox("🎯 SENSOR PICKINESS & QUALITY GATING")
        picky_layout = QVBoxLayout(picky_group)
        picky_layout.setContentsMargins(6, 6, 6, 6)
        picky_layout.setSpacing(4)

        # 1. Confidence Pickiness Level Selector
        conf_row = QHBoxLayout()
        conf_row.addWidget(QLabel("Confidence:"))
        self.conf_combo = QComboBox()
        self.conf_combo.addItem("🛡️ Ultra Picky (100% Valid / s5)", "valid_5")
        self.conf_combo.addItem("⚖️ Balanced (>=50% Conf / s5,s6,s9)", "valid_5_6_9")
        self.conf_combo.addItem("👁️ Sensitive (+ Secondary / s10,s13)", "valid_extended")
        self.conf_combo.addItem("🔬 Raw / Diagnostics (All 0-255)", "all")
        self.conf_combo.setCurrentIndex(1)
        self.conf_combo.currentIndexChanged.connect(self.on_pickiness_conf_changed)
        conf_row.addWidget(self.conf_combo)
        picky_layout.addLayout(conf_row)

        # Dynamic Status Description Banner
        self.conf_desc_lbl = QLabel("🟢 Accepting: s5 (100% Valid), s6 (Wrap-around), s9 (Merged pulse)")
        self.conf_desc_lbl.setStyleSheet("color: #38bdf8; font-size: 7.5pt; font-style: italic; padding: 2px 4px; background: #161c2b; border-radius: 3px;")
        self.conf_desc_lbl.setWordWrap(True)
        picky_layout.addWidget(self.conf_desc_lbl)

        # Toggleable ST Table 4 Status Glossary Drawer
        glossary_toggle_row = QHBoxLayout()
        self.btn_toggle_glossary = QPushButton("📋 ST Table 4 Status Code Glossary (All 15) ▾")
        self.btn_toggle_glossary.setCheckable(True)
        self.btn_toggle_glossary.setChecked(False)
        self.btn_toggle_glossary.setStyleSheet("""
            QPushButton { 
                background: #181d2b; 
                color: #94a3b8; 
                border: 1px solid #232a3d; 
                border-radius: 3px; 
                font-size: 7.5pt; 
                padding: 3px 6px; 
                text-align: left;
            }
            QPushButton:hover { background: #232a3d; color: #00E5FF; }
            QPushButton:checked { background: #1e293b; color: #00E5FF; border-color: #00E5FF; }
        """)
        glossary_toggle_row.addWidget(self.btn_toggle_glossary)
        picky_layout.addLayout(glossary_toggle_row)

        self.glossary_scroll = QScrollArea()
        self.glossary_scroll.setFixedHeight(240)
        self.glossary_scroll.setWidgetResizable(True)
        self.glossary_scroll.setStyleSheet("""
            QScrollArea { background: #0c0f17; border: 1px solid #1e293b; border-radius: 4px; }
            QScrollBar:vertical { background: #12151f; width: 6px; }
            QScrollBar::handle:vertical { background: #232a3d; border-radius: 3px; }
        """)

        glossary_content = QWidget()
        glossary_content.setStyleSheet("background: #0c0f17;")
        glossary_layout = QVBoxLayout(glossary_content)
        glossary_layout.setContentsMargins(6, 6, 6, 6)
        glossary_layout.setSpacing(3)

        # Full ST UM3109 Rev 12 Table 4 Target Status List
        all_statuses_help = [
            ("s5", "#00E676", "Range valid (100%)", "Highest confidence, clean single pulse return"),
            ("s6", "#00E5FF", "Wrap around not performed (50%)", "Valid range, wrap around not performed (first range)"),
            ("s9", "#FFB300", "Range valid with large pulse (50%)", "Valid range, merged target / multi-surface pulse"),
            ("s10", "#26A69A", "Range valid (No prev range)", "Valid target, not detected at previous range"),
            ("s13", "#8D6E63", "Target detected (Inconsistent)", "Low confidence return (frequent for secondary targets)"),
            ("s12", "#7E57C2", "Target blurred by sharpener", "Pulse blurred by neighboring zone sharpener"),
            ("s0", "#64748B", "Ranging data not updated", "Frame not ready or not updated yet"),
            ("s1", "#EF4444", "Signal rate too low on SPAD array", "Ambient light overpowering laser or target too dark"),
            ("s2", "#AB47BC", "Target phase", "Phase failure in ToF histogram accumulator"),
            ("s3", "#FF7043", "Sigma estimator too high", "Range standard deviation exceeds noise threshold"),
            ("s4", "#EC407A", "Target consistency failed", "Consecutive pulse consistency check failed"),
            ("s7", "#F43F5E", "Rate consistency failed", "Photon count rate consistency check failed"),
            ("s8", "#DC2626", "Signal rate too low for target", "Signal energy below minimum target threshold"),
            ("s11", "#B91C1C", "Measurement consistency failed", "Hardware measurement consistency check failed"),
            ("s255", "#475569", "No target detected", "Empty zone / no photon reflection detected in range"),
        ]

        for code, color, title, desc in all_statuses_help:
            item_row = QHBoxLayout()
            item_row.setSpacing(5)
            badge = QLabel(code)
            badge.setFixedWidth(34)
            badge.setFixedHeight(18)
            badge.setStyleSheet(f"background: {color}; color: #000000; font-weight: bold; font-size: 7pt; border-radius: 3px; padding: 1px 2px; qproperty-alignment: AlignCenter;")
            item_row.addWidget(badge)

            lbl = QLabel(f"<b>{title}</b>: {desc}")
            lbl.setStyleSheet("color: #cbd5e1; font-size: 7.2pt;")
            lbl.setWordWrap(True)
            item_row.addWidget(lbl, 1)
            glossary_layout.addLayout(item_row)

        self.glossary_scroll.setWidget(glossary_content)
        self.glossary_scroll.setVisible(False)
        self.btn_toggle_glossary.toggled.connect(lambda checked: self.glossary_scroll.setVisible(checked))
        self.btn_toggle_glossary.toggled.connect(lambda checked: self.btn_toggle_glossary.setText("📋 ST Table 4 Status Code Glossary (All 15) ▴" if checked else "📋 ST Table 4 Status Code Glossary (All 15) ▾"))
        picky_layout.addWidget(self.glossary_scroll)

        # 2. Max Noise Sigma Limit Slider
        sigma_row = QHBoxLayout()
        self.sigma_lbl = QLabel("Max Noise: Off")
        self.sigma_lbl.setMinimumWidth(85)
        sigma_row.addWidget(self.sigma_lbl)
        self.sigma_slider = QSlider(Qt.Horizontal)
        self.sigma_slider.setRange(0, 30)
        self.sigma_slider.setValue(0)
        sigma_row.addWidget(self.sigma_slider)
        self.sigma_spin = QSpinBox()
        self.sigma_spin.setRange(0, 30)
        self.sigma_spin.setValue(0)
        self.sigma_spin.setSuffix(" mm")
        self.sigma_spin.setFixedWidth(52)
        sigma_row.addWidget(self.sigma_spin)
        self.sigma_slider.valueChanged.connect(self.sigma_spin.setValue)
        self.sigma_spin.valueChanged.connect(self.sigma_slider.setValue)
        self.sigma_spin.valueChanged.connect(self.on_sigma_limit_changed)
        picky_layout.addLayout(sigma_row)

        # 3. Min Signal Threshold Slider
        sig_row = QHBoxLayout()
        self.min_sig_lbl = QLabel("Min Signal: Off")
        self.min_sig_lbl.setMinimumWidth(85)
        sig_row.addWidget(self.min_sig_lbl)
        self.min_sig_slider = QSlider(Qt.Horizontal)
        self.min_sig_slider.setRange(0, 100)
        self.min_sig_slider.setValue(0)
        sig_row.addWidget(self.min_sig_slider)
        self.min_sig_spin = QSpinBox()
        self.min_sig_spin.setRange(0, 100)
        self.min_sig_spin.setValue(0)
        self.min_sig_spin.setSuffix(" k")
        self.min_sig_spin.setFixedWidth(52)
        sig_row.addWidget(self.min_sig_spin)
        self.min_sig_slider.valueChanged.connect(self.min_sig_spin.setValue)
        self.min_sig_spin.valueChanged.connect(self.min_sig_slider.setValue)
        self.min_sig_spin.valueChanged.connect(self.on_min_sig_changed)
        picky_layout.addLayout(sig_row)

        # 4. Distance Range Gating
        dist_row = QHBoxLayout()
        dist_row.addWidget(QLabel("Range Gate:"))
        self.gate_min_spin = QSpinBox()
        self.gate_min_spin.setRange(20, 2000)
        self.gate_min_spin.setValue(20)
        self.gate_min_spin.setSuffix(" mm")
        self.gate_min_spin.valueChanged.connect(self.on_dist_gate_changed)
        dist_row.addWidget(self.gate_min_spin)

        dist_row.addWidget(QLabel("to"))
        self.gate_max_spin = QSpinBox()
        self.gate_max_spin.setRange(200, 4000)
        self.gate_max_spin.setValue(4000)
        self.gate_max_spin.setSuffix(" mm")
        self.gate_max_spin.valueChanged.connect(self.on_dist_gate_changed)
        dist_row.addWidget(self.gate_max_spin)
        picky_layout.addLayout(dist_row)

        ctrl_layout.addWidget(picky_group)

        # --- Card 3: Crosstalk (Xtalk) Calibration Suite ---
        xtalk_group = QGroupBox("📐 CROSSTALK (XTALK) CALIBRATION SUITE")
        xtalk_layout = QVBoxLayout(xtalk_group)
        xtalk_layout.setContentsMargins(6, 6, 6, 6)
        xtalk_layout.setSpacing(4)

        cal_row1 = QHBoxLayout()
        cal_row1.addWidget(QLabel("Calib Dist:"))
        self.cal_dist_spin = QSpinBox()
        self.cal_dist_spin.setRange(600, 3000)
        self.cal_dist_spin.setValue(600)
        self.cal_dist_spin.setSuffix(" mm")
        cal_row1.addWidget(self.cal_dist_spin)

        cal_row1.addWidget(QLabel("Samples:"))
        self.cal_samples_spin = QSpinBox()
        self.cal_samples_spin.setRange(1, 16)
        self.cal_samples_spin.setValue(4)
        cal_row1.addWidget(self.cal_samples_spin)
        xtalk_layout.addLayout(cal_row1)

        margin_row = QHBoxLayout()
        margin_row.addWidget(QLabel("Xtalk Margin:"))
        self.margin_spin = QSpinBox()
        self.margin_spin.setRange(0, 10000)
        self.margin_spin.setValue(50)
        self.margin_spin.setSuffix(" kcps")
        margin_row.addWidget(self.margin_spin)

        self.cal_btn = QPushButton("🎯 Calibrate Xtalk")
        self.cal_btn.setObjectName("calBtn")
        self.cal_btn.setToolTip("Run ST crosstalk calibration with target at specified distance")
        self.cal_btn.clicked.connect(self.calibrate_xtalk)
        margin_row.addWidget(self.cal_btn)
        xtalk_layout.addLayout(margin_row)

        ctrl_layout.addWidget(xtalk_group)

        # --- Card 4: Quick Performance Presets ---
        preset_group = QGroupBox("⚡ RACING & DETECTION PRESETS")
        preset_layout = QGridLayout(preset_group)
        preset_layout.setContentsMargins(6, 6, 6, 6)
        preset_layout.setSpacing(4)

        btn_preset_speed = QPushButton("🚀 4x4 @ 60 Hz (Optimal 15ms)")
        btn_preset_speed.clicked.connect(lambda: self.load_preset(16, 60, 1, 1, 50, "valid_5_6_9", 0, 0, integ=15))
        preset_layout.addWidget(btn_preset_speed, 0, 0)

        btn_preset_prec = QPushButton("🎯 8x8 @ 15 Hz (Max Res)")
        btn_preset_prec.clicked.connect(lambda: self.load_preset(64, 15, 1, 1, 50, "valid_5_6_9", 0, 0, integ=50))
        preset_layout.addWidget(btn_preset_prec, 0, 1)

        btn_preset_picky = QPushButton("🛡️ Ultra Picky (High Confidence)")
        btn_preset_picky.clicked.connect(lambda: self.load_preset(16, 60, 1, 5, 100, "valid_5", 5, 10, integ=15))
        preset_layout.addWidget(btn_preset_picky, 1, 0)

        btn_preset_sens = QPushButton("👁️ Ultra Sensitive (Faint Targets)")
        btn_preset_sens.clicked.connect(lambda: self.load_preset(16, 30, 2, 0, 0, "valid_extended", 0, 0, integ=30))
        preset_layout.addWidget(btn_preset_sens, 1, 1)

        ctrl_layout.addWidget(preset_group)

        # --- Card 5: Action Panel ---
        action_box = QFrame()
        action_box.setStyleSheet("background-color: #12151f; border-radius: 6px; border: 1px solid #232a3d; padding: 4px;")
        act_layout = QVBoxLayout(action_box)
        act_layout.setContentsMargins(6, 6, 6, 6)
        act_layout.setSpacing(4)

        self.apply_btn = QPushButton("⚡ APPLY SENSOR CHANGES")
        self.apply_btn.setObjectName("applyBtn")
        self.apply_btn.clicked.connect(self.apply_changes)
        act_layout.addWidget(self.apply_btn)

        self.apply_status_lbl = QLabel("Ready • Continuous 60 Hz Mode Active")
        self.apply_status_lbl.setAlignment(Qt.AlignCenter)
        self.apply_status_lbl.setStyleSheet("color: #8fa0be; font-size: 8pt; font-style: italic;")
        act_layout.addWidget(self.apply_status_lbl)

        btn_row = QHBoxLayout()
        self.reset_btn = QPushButton("🔄 Reset Sensor Hardware")
        self.reset_btn.setObjectName("resetBtn")
        self.reset_btn.clicked.connect(self.reset_sensor)
        btn_row.addWidget(self.reset_btn)
        act_layout.addLayout(btn_row)

        ctrl_layout.addWidget(action_box)
        ctrl_layout.addStretch()

        body_splitter.addWidget(ctrl_container)
        body_splitter.setStretchFactor(0, 5)
        body_splitter.setStretchFactor(1, 2)

        main_layout.addWidget(body_splitter, 1)

    # -------------------------------------------------------------------------
    # LIFECYCLE & EVENT HANDLERS
    # -------------------------------------------------------------------------

    def closeEvent(self, event):
        if self.serial_engine.isRunning():
            self.serial_engine.stop()
        event.accept()

    def refresh_com_ports(self):
        self.port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if "COM7" not in ports:
            ports.insert(0, "COM7")
        else:
            ports.remove("COM7")
            ports.insert(0, "COM7")
        self.port_combo.addItems(ports)

    def toggle_connection(self):
        if not self.serial_engine.isRunning():
            port = self.port_combo.currentText()
            baud = int(self.baud_combo.currentText())
            self.serial_engine.set_connection_params(port, baud)
            self.serial_engine.start()
        else:
            self.serial_engine.stop()
            self.on_connection_changed(False, "Disconnected")

    def on_connection_changed(self, connected, msg):
        if connected:
            self.connect_btn.setText("❌ Disconnect")
            self.connect_btn.setStyleSheet("background-color: #991b1b; color: white;")
            self.status_pill.setText("ONLINE")
            self.status_pill.setStyleSheet("background-color: #065f46; color: #6ee7b7; font-weight: bold; padding: 2px 8px; border-radius: 4px; font-size: 8pt;")
        else:
            self.connect_btn.setText("🔌 Connect")
            self.connect_btn.setStyleSheet("")
            self.status_pill.setText("DISCONNECTED")
            self.status_pill.setStyleSheet("background-color: #232a3d; color: #cbd5e1; font-weight: bold; padding: 2px 8px; border-radius: 4px; font-size: 8pt;")

    def on_resolution_changed(self, index):
        res = self.res_combo.currentData()
        if res == 64:
            self.freq_spin.setMaximum(15)
            self.freq_slider.setMaximum(15)
            if self.freq_spin.value() > 15:
                self.freq_spin.setValue(15)
        else:
            self.freq_spin.setMaximum(60)
            self.freq_slider.setMaximum(60)

    def on_metric_changed(self, index):
        self.heatmap.active_metric = self.metric_combo.currentData()
        self.heatmap.update()

    def on_layer_changed(self, index):
        self.heatmap.target_layer = self.layer_combo.currentData()
        self.heatmap.update()

    def on_filter_changed(self, index):
        val = self.filter_combo.currentData()
        self.heatmap.status_filter = val
        idx = self.conf_combo.findData(val)
        if idx >= 0 and self.conf_combo.currentIndex() != idx:
            self.conf_combo.setCurrentIndex(idx)
        self._update_conf_desc_text(val)
        self.heatmap.update()

    def on_pickiness_conf_changed(self, index):
        val = self.conf_combo.currentData()
        self.heatmap.status_filter = val
        idx = self.filter_combo.findData(val)
        if idx >= 0 and self.filter_combo.currentIndex() != idx:
            self.filter_combo.setCurrentIndex(idx)
        self._update_conf_desc_text(val)
        self.heatmap.update()

    def _update_conf_desc_text(self, val):
        if val == "valid_5":
            self.conf_desc_lbl.setText("🛡️ Accepting: s5 (100% High Confidence Valid Returns Only)")
            self.conf_desc_lbl.setStyleSheet("color: #4ade80; font-size: 7.5pt; font-style: italic; padding: 2px 4px; background: #161c2b; border-radius: 3px;")
        elif val == "valid_5_6_9":
            self.conf_desc_lbl.setText("⚖️ Accepting: s5 (100% Valid), s6 (Wrap-around), s9 (Merged pulse)")
            self.conf_desc_lbl.setStyleSheet("color: #38bdf8; font-size: 7.5pt; font-style: italic; padding: 2px 4px; background: #161c2b; border-radius: 3px;")
        elif val == "valid_extended":
            self.conf_desc_lbl.setText("👁️ Accepting: s5, s6, s9, s10 (New Entry), s13 (Secondary Targets)")
            self.conf_desc_lbl.setStyleSheet("color: #facc15; font-size: 7.5pt; font-style: italic; padding: 2px 4px; background: #161c2b; border-radius: 3px;")
        else:
            self.conf_desc_lbl.setText("🔬 Accepting ALL status codes (s0 to s255, including errors & noise)")
            self.conf_desc_lbl.setStyleSheet("color: #f87171; font-size: 7.5pt; font-style: italic; padding: 2px 4px; background: #161c2b; border-radius: 3px;")

    def on_sigma_limit_changed(self, val):
        self.heatmap.max_sigma_filter = val
        self.sigma_lbl.setText(f"Max Noise: {val} mm" if val > 0 else "Max Noise: Off")
        self.heatmap.update()

    def on_min_sig_changed(self, val):
        self.heatmap.min_signal_filter = val
        self.min_sig_lbl.setText(f"Min Signal: {val} k" if val > 0 else "Min Signal: Off")
        self.heatmap.update()

    def on_dist_gate_changed(self):
        self.heatmap.min_dist_gate = self.gate_min_spin.value()
        self.heatmap.max_dist_gate = self.gate_max_spin.value()
        self.heatmap.update()

    def on_freq_changed(self, val):
        self.freq_lbl.setText(f"Rate: {val} Hz")
        # Dynamic maximum integration budget: max ms <= 950 / freq
        max_integ = max(2, int(950 / val))
        self.integ_slider.setMaximum(min(200, max_integ))
        self.integ_spin.setMaximum(max_integ)
        if self.integ_spin.value() > max_integ:
            self.integ_spin.setValue(max_integ)

    def set_smart_integration(self, preset_type):
        freq = self.freq_spin.value()
        max_budget = max(2, int(900 / freq))
        if preset_type == "auto":
            val = max_budget
        elif preset_type == "fast":
            val = min(5, max_budget)
        elif preset_type == "balanced":
            val = min(15, max_budget)
        elif preset_type == "deep":
            val = min(50, max_budget)
        else:
            val = 15
        self.integ_spin.setValue(max(2, val))

    def on_toggle_flip_h(self):
        self.heatmap.flip_h = self.btn_flip_h.isChecked()
        self.update_orientation_state()

    def on_toggle_flip_v(self):
        self.heatmap.flip_v = self.btn_flip_v.isChecked()
        self.update_orientation_state()

    def on_rotate_clicked(self):
        self.heatmap.rotation = (self.heatmap.rotation + 90) % 360
        self.update_orientation_state()

    def update_orientation_state(self):
        rot = self.heatmap.rotation
        self.btn_rotate.setText(f"🔄 {rot}°")
        self.heatmap.update()

    def load_preset(self, res, freq, order, sharp, margin, conf="valid_5_6_9", sigma=0, min_sig=0, integ=15):
        idx = self.res_combo.findData(res)
        if idx >= 0: self.res_combo.setCurrentIndex(idx)
        self.freq_spin.setValue(freq)
        self.integ_spin.setValue(integ)
        idx = self.order_combo.findData(order)
        if idx >= 0: self.order_combo.setCurrentIndex(idx)
        self.sharp_spin.setValue(sharp)
        self.margin_spin.setValue(margin)
        
        idx_conf = self.conf_combo.findData(conf)
        if idx_conf >= 0: self.conf_combo.setCurrentIndex(idx_conf)
        self.sigma_spin.setValue(sigma)
        self.min_sig_spin.setValue(min_sig)
        
        self.apply_changes()

    def apply_changes(self):
        res = self.res_combo.currentData()
        freq = self.freq_spin.value()
        integ = self.integ_spin.value()
        order = self.order_combo.currentData()
        sharp = self.sharp_spin.value()
        margin = self.margin_spin.value()

        # Always Continuous High-Performance Ranging Mode (mode=1) & Active Wakeup (pwr=1)
        cmd = f"SET_CONFIG:res={res},freq={freq},integ={integ},order={order},sharp={sharp},mode=1,pwr=1,margin={margin}"
        self.apply_btn.setEnabled(False)
        self.apply_status_lbl.setText("⏳ Applying sensor changes...")
        self.apply_status_lbl.setStyleSheet("color: #00E5FF; font-weight: bold;")
        self.serial_engine.send_command(cmd)
        self.ack_timer.start(3000)

    def reset_sensor(self):
        self.apply_status_lbl.setText("🔄 Hardware resetting...")
        self.serial_engine.send_command("RESET")
        self.ack_timer.start(3000)

    def calibrate_xtalk(self):
        dist = self.cal_dist_spin.value()
        samples = self.cal_samples_spin.value()
        self.apply_status_lbl.setText(f"📐 Calibrating crosstalk @ {dist}mm...")
        self.serial_engine.send_command(f"CALIBRATE_XTALK:dist={dist},samples={samples},refl=3")

    def on_ack_timeout(self):
        self.apply_btn.setEnabled(True)
        self.apply_status_lbl.setText("⚠️ No ACK from sensor (command may be lost) - try again")
        self.apply_status_lbl.setStyleSheet("color: #FFB300; font-weight: bold;")

    def on_ack_received(self, data):
        self.ack_timer.stop()
        self.apply_btn.setEnabled(True)
        status = data.get("status", "")
        cmd = data.get("cmd", "")
        if status == "OK":
            if cmd == "CALIBRATE_XTALK":
                self.apply_status_lbl.setText("✅ Crosstalk Calibration Succeeded!")
            else:
                self.apply_status_lbl.setText("✅ Settings applied successfully!")
            self.apply_status_lbl.setStyleSheet("color: #00E676; font-weight: bold;")
        else:
            self.apply_status_lbl.setText(f"⚠️ Result: {status}")
            self.apply_status_lbl.setStyleSheet("color: #FF5252; font-weight: bold;")

    def on_config_received(self, data):
        res = data.get("res", 16)
        freq = data.get("freq", 60)
        integ = data.get("integ", 15)
        order = data.get("order", 1)
        sharp = data.get("sharp", 1)
        margin = data.get("margin", 50)

        idx = self.res_combo.findData(res)
        if idx >= 0: self.res_combo.setCurrentIndex(idx)
        self.freq_spin.setValue(freq)
        self.integ_spin.setValue(integ)
        idx = self.order_combo.findData(order)
        if idx >= 0: self.order_combo.setCurrentIndex(idx)
        self.sharp_spin.setValue(sharp)
        self.margin_spin.setValue(margin)

    def ui_tick(self):
        data = self.serial_engine.take_latest()
        if data is None:
            return
        self.last_frame_wall = time.perf_counter()
        self.render_frame(data)

    def render_frame(self, data):
        self.frame_counter += 1
        res = data.get("r", 16)
        nt = data.get("nt", 4)
        ntar = data.get("ntar", [])
        dists = data.get("d", [])
        statuses = data.get("st", [])
        signals = data.get("sig", [])
        sigmas = data.get("sigm", [])
        ambs = data.get("amb", [])
        spads = data.get("spad", [])
        temp_c = data.get("temp", None)

        if temp_c is not None:
            self.temp_badge.setText(f"🌡️ {temp_c} °C")

        # Update Heatmap data model (retaining Target 1 & Target 2)
        self.heatmap.update_telemetry(res, nt, ntar, dists, statuses, signals, sigmas, ambs, spads)

        # Route updates to active tab
        tab_idx = self.vis_tabs.currentIndex()
        dists_layer, _, _, _, _ = self.heatmap.get_effective_layer_data()

        if tab_idx == 1:
            self.strip_chart.add_telemetry_point(dists_layer, self.heatmap.selected_zone)
            self.strip_chart.redraw_chart()

        # Update Live KPIs
        valid_indices = [
            z for z in range(res)
            if self.heatmap.is_valid_reading(
                dists_layer[z],
                self.heatmap.statuses[z, 0],
                sig=self.heatmap.signals[z, 0],
                sigm=self.heatmap.sigmas[z, 0]
            )
        ]
        if valid_indices:
            valid_d = [dists_layer[z] for z in valid_indices]
            self.kpi_mean_lbl.setText(f"Mean Depth: {np.mean(valid_d):.1f} mm")
        else:
            self.kpi_mean_lbl.setText("Mean Depth: --- mm")

        valid_count = len(valid_indices)
        self.kpi_valid_lbl.setText(f"Valid Targets: {valid_count} / {res} ({int(100*valid_count/res)}%)")
        self.kpi_sel_lbl.setText(f"Selected: Zone #{self.heatmap.selected_zone}")

    def on_cell_selected(self, idx):
        self.kpi_sel_lbl.setText(f"Selected: Zone #{idx}")
        self.strip_chart.last_selected_idx = idx

    def on_raw_log(self, text):
        if self.vis_tabs.currentIndex() == 2:
            self.log_text.append(text)
            if self.log_text.document().blockCount() > 300:
                cursor = self.log_text.textCursor()
                cursor.movePosition(cursor.Start)
                cursor.select(cursor.BlockUnderCursor)
                cursor.removeSelectedText()
                cursor.deleteChar()

    def update_fps_calc(self):
        now = time.perf_counter()
        dt = now - self.last_fps_time
        if dt >= 0.2:
            self.current_fps = self.frame_counter / dt
            self.fps_badge.setText(f"{self.current_fps:.1f} FPS")
            self.frame_counter = 0
            self.last_fps_time = now
        self.update_diagnostics(now)

    def update_diagnostics(self, now):
        st = self.serial_engine.get_stats()
        ddt = now - self.diag_prev_time
        if ddt <= 0:
            return
        rx_hz = (st["rx"] - self.diag_prev_rx) / ddt
        self.diag_prev_rx = st["rx"]
        self.diag_prev_time = now

        total = st["rx"] + st["lost"]
        loss_pct = (100.0 * st["lost"] / total) if total else 0.0
        age_ms = (now - st["last_rx"]) * 1000.0 if st["last_rx"] else -1
        connected = self.serial_engine.isRunning() and st["last_rx"] > 0

        if not connected:
            self.diag_badge.setText("RX -- Hz | no data")
            self.latency_badge.setText("⏱️ -- ms")
            color = "#94a3b8"
            lat_color = "#94a3b8"
        else:
            mean_dt = st.get("mean_dt", 0.0)
            jitter = st.get("jitter", 0.0)
            peak_dt = st.get("peak_dt", 0.0)
            spikes = st.get("spikes", 0)

            self.latency_badge.setText(f"⏱️ {mean_dt:.1f}±{jitter:.1f}ms (peak: {peak_dt:.0f}ms)")
            if peak_dt > 200 or spikes > 5:
                lat_color = "#EF4444"
            elif peak_dt > 50 or spikes > 0:
                lat_color = "#FFB300"
            else:
                lat_color = "#00E676"

            self.diag_badge.setText(
                f"RX {rx_hz:.0f} Hz | loss {loss_pct:.1f}% ({st['lost']}) | spikes {spikes} | "
                f"age {age_ms:.0f} ms | buf {st['hw']}B"
            )
            if age_ms > 1000:
                color = "#EF4444"
            elif age_ms > 250 or loss_pct > 5:
                color = "#FFB300"
            else:
                color = "#00E676"

        self.latency_badge.setStyleSheet(
            f"background-color: #0f1422; color: {lat_color}; font-weight: bold; padding: 2px 8px; "
            "border-radius: 4px; border: 1px solid #1c2233; font-size: 8pt;")
        self.diag_badge.setStyleSheet(
            f"background-color: #0f1422; color: {color}; font-weight: bold; padding: 2px 8px; "
            "border-radius: 4px; border: 1px solid #1c2233; font-size: 8pt;")


# =============================================================================
# APPLICATION ENTRYPOINT
# =============================================================================

def main():
    app = QApplication(sys.argv)
    window = VL53L8CXDashboard()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
