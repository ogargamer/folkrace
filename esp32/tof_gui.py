#!/usr/bin/env python3
"""
===============================================================================
DA WINNA — VL53L8CX MULTIZONE TIME-OF-FLIGHT CONTROL & TELEMETRY SUITE
===============================================================================
Features:
  - Full Multi-Target Detection & Analysis (up to 4 targets per zone)
  - Target Reflectance Estimation (%) Heatmaps, Histograms & Oscilloscope
  - Optical Signal Rate (kcps/SPAD), Range Sigma Noise (mm), Ambient Light & SPAD Counts
  - ST Table 4 Target Status Interpretation & Confidence Filtering (100% vs 50% vs Errors)
  - Continuous High-Speed Ranging Mode (up to 60 Hz 4x4, 15 Hz 8x8)
  - Closest vs Strongest Target Order Selection
  - Veiling Glare Sharpener Tuning (0-99%, ST Rev 12 Default = 1%)
  - Crosstalk (Xtalk) Calibration Suite & Xtalk Margin Configuration
  - Interactive 2D Depth / Reflectance / Multi-Metric Matrix Heatmap
  - 3D Elevation Depth Point Cloud & Surface Mesh Visualizer
  - Multi-Target Range Distribution Radar / Depth Slice Visualizer
  - Dual-Trace Historical Oscilloscope (Distance & Reflectance Strip-Charts)
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

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QObject
from PyQt5.QtGui import QColor, QFont, QPainter, QBrush, QPen, QLinearGradient, QPixmap

if hasattr(Qt, 'HighDpiScaleFactorRoundingPolicy'):
    QApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D


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
    data_received = pyqtSignal(dict)
    config_received = pyqtSignal(dict)
    ack_received = pyqtSignal(dict)
    connection_changed = pyqtSignal(bool, str)
    raw_log = pyqtSignal(str)

    def __init__(self, port="COM5", baud=921600):
        super().__init__()
        self.port = port
        self.baud = baud
        self.ser = None
        self._running = False

    def set_connection_params(self, port, baud):
        self.port = port
        self.baud = baud

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
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
                if self.ser and self.ser.is_open and self.ser.in_waiting:
                    raw = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='replace')
                    buffer += raw
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
                                    self.data_received.emit(packet)
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
                                pass
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

def viridis_colormap(val):
    """Normalized [0.0, 1.0] -> RGB QColor (0%=Deep Purple, 50%=Teal, 100%=Bright Yellow)"""
    x = max(0.0, min(1.0, val))
    r = int(255 * max(0.0, min(1.0, 0.267 + x * (0.0048 + x * (2.28 - x * (2.11))))))
    g = int(255 * max(0.0, min(1.0, 0.004 + x * (1.42 + x * (0.83 - x * (1.38))))))
    b = int(255 * max(0.0, min(1.0, 0.329 + x * (1.36 - x * (2.41 + x * (0.88))))))
    return QColor(r, g, b)

def amber_colormap(val):
    """Normalized [0.0, 1.0] -> Amber/Gold intensity for Signal & SPADs"""
    x = max(0.0, min(1.0, val))
    r = int(min(255, 30 + x * 225))
    g = int(min(255, 20 + x * 180))
    b = int(min(255, 40 + x * 50))
    return QColor(r, g, b)

def reflectance_badge_colors(refl):
    """
    Returns (bg_color, text_color, border_color) for the reflectivity sub-tile badge.
    Color coding:
      - 70-100%: Bright Gold / Radiant Amber (High reflectivity, e.g. white wall, reflector)
      - 40-69%: Vibrant Emerald / Teal (Medium-high, e.g. light painted surface)
      - 15-39%: Rich Indigo / Blue (Medium-low, e.g. wood, gray object)
      - 1-14%: Deep Slate / Plum (Low reflectivity, e.g. black foam, tire)
      - 0% / Invalid: Dark Charcoal
    """
    if refl <= 0:
        return QColor(24, 30, 44, 220), QColor(148, 163, 184), QColor(45, 55, 75)
    elif refl >= 70:
        norm = min(1.0, (refl - 70) / 30.0)
        return QColor(int(217 + norm * 38), int(145 + norm * 60), 10, 235), QColor(15, 23, 42), QColor(254, 240, 138)
    elif refl >= 40:
        norm = min(1.0, (refl - 40) / 30.0)
        return QColor(int(5 + norm * 15), int(130 + norm * 50), int(100 + norm * 30), 235), QColor(255, 255, 255), QColor(167, 243, 208)
    elif refl >= 15:
        norm = min(1.0, (refl - 15) / 25.0)
        return QColor(int(45 + norm * 30), int(75 + norm * 45), int(180 + norm * 45), 235), QColor(255, 255, 255), QColor(191, 219, 254)
    else:
        return QColor(40, 52, 70, 230), QColor(226, 232, 240), QColor(100, 116, 139)


# =============================================================================
# 2D MATRIX MULTI-METRIC HEATMAP WIDGET
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
        self.nb_targets = 4

        # Multi-target data matrices (zones, 4)
        self.distances = np.zeros((16, 4), dtype=int)
        self.reflectances = np.zeros((16, 4), dtype=int)
        self.statuses = np.zeros((16, 4), dtype=int)
        self.sigmas = np.zeros((16, 4), dtype=int)
        self.signals = np.zeros((16, 4), dtype=int)
        self.target_counts = np.zeros(16, dtype=int)
        self.ambient = np.zeros(16, dtype=int)
        self.spads = np.zeros(16, dtype=int)

        self.selected_zone = 0
        self.target_layer = 0   # 0=Target 1 (Primary), 1=Target 2, 2=Target 3, 3=Target 4, -1=Composite
        self.active_metric = "dist"  # "dist", "refl", "sig", "sigm", "amb", "spad", "ntar", "st"
        
        # Pickiness & Quality Filters
        self.status_filter = "valid_5_6_9"  # "valid_5", "valid_5_6_9", "valid_extended", "all"
        self.max_sigma_filter = 0    # 0 = disabled, or 1..30 mm (max noise tolerance)
        self.min_signal_filter = 0   # 0 = disabled, or 1..100 kcps (min signal rate threshold)
        self.min_refl_filter = 0     # 0 = disabled, or 1..30 % (min reflectance gate)
        self.min_dist_gate = 20      # mm
        self.max_dist_gate = 4000    # mm

        self.min_dist_range = 50
        self.max_dist_range = 3000

        # Spatial transformation parameters
        self.flip_h = False
        self.flip_v = False
        self.rotation = 0

    def is_valid_reading(self, dist, st, sig=None, sigm=None, refl=None):
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
            
        # 5. Minimum Reflectance Gate (rejection of ghost reflections)
        if self.min_refl_filter > 0 and refl is not None and refl < self.min_refl_filter:
            return False
            
        return True

    def update_telemetry(self, res, nt, ntar, dists, refls, stats, sigs, sigms, ambs, spads_list):
        self.resolution = res
        self.grid_dim = 8 if res == 64 else 4
        self.nb_targets = nt if nt > 0 else 4

        n_zones = res
        tot_elems = n_zones * self.nb_targets

        # Reshape into (n_zones, nb_targets)
        d_arr = np.array(dists[:tot_elems], dtype=int) if len(dists) >= tot_elems else np.zeros(tot_elems, dtype=int)
        self.distances = d_arr.reshape((n_zones, self.nb_targets))

        r_arr = np.array(refls[:tot_elems], dtype=int) if len(refls) >= tot_elems else np.zeros(tot_elems, dtype=int)
        self.reflectances = r_arr.reshape((n_zones, self.nb_targets))

        s_arr = np.array(stats[:tot_elems], dtype=int) if len(stats) >= tot_elems else np.zeros(tot_elems, dtype=int)
        self.statuses = s_arr.reshape((n_zones, self.nb_targets))

        sig_arr = np.array(sigs[:tot_elems], dtype=int) if len(sigs) >= tot_elems else np.zeros(tot_elems, dtype=int)
        self.signals = sig_arr.reshape((n_zones, self.nb_targets))

        sigm_arr = np.array(sigms[:tot_elems], dtype=int) if len(sigms) >= tot_elems else np.zeros(tot_elems, dtype=int)
        self.sigmas = sigm_arr.reshape((n_zones, self.nb_targets))

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
                    "refl": int(self.reflectances[z_idx, t]),
                    "stat": int(self.statuses[z_idx, t]),
                    "sig": int(self.signals[z_idx, t]),
                    "sigm": int(self.sigmas[z_idx, t]),
                }
                for t in range(self.nb_targets)
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
        Returns (dists, refls, stats, sigs, sigms, tgt_indices) for current target_layer.
        If target_layer == 100 ('Smart Fallback'):
            For each zone, uses Target #1 if valid; otherwise automatically falls back
            to the next valid secondary/tertiary target (Target #2, #3, #4).
        """
        n_zones = self.resolution
        if self.target_layer == 100:
            dists = np.zeros(n_zones, dtype=int)
            refls = np.zeros(n_zones, dtype=int)
            stats = np.zeros(n_zones, dtype=int)
            sigs = np.zeros(n_zones, dtype=int)
            sigms = np.zeros(n_zones, dtype=int)
            tgt_indices = np.zeros(n_zones, dtype=int)

            for z in range(n_zones):
                # 1. Test Primary Target (index 0)
                d0 = self.distances[z, 0] if len(self.distances) > z else 0
                st0 = self.statuses[z, 0] if len(self.statuses) > z else 0
                r0 = self.reflectances[z, 0] if len(self.reflectances) > z else 0
                sig0 = self.signals[z, 0] if len(self.signals) > z else 0
                sigm0 = self.sigmas[z, 0] if len(self.sigmas) > z else 0

                if self.is_valid_reading(d0, st0, sig=sig0, sigm=sigm0, refl=r0):
                    dists[z] = d0
                    refls[z] = r0
                    stats[z] = st0
                    sigs[z] = sig0
                    sigms[z] = sigm0
                    tgt_indices[z] = 0
                else:
                    # 2. Primary has no reading or is filtered; search secondary targets (1, 2, 3)
                    found_fallback = False
                    for t in range(1, self.nb_targets):
                        dt = self.distances[z, t] if len(self.distances) > z else 0
                        stt = self.statuses[z, t] if len(self.statuses) > z else 0
                        rt = self.reflectances[z, t] if len(self.reflectances) > z else 0
                        sigt = self.signals[z, t] if len(self.signals) > z else 0
                        sigmt = self.sigmas[z, t] if len(self.sigmas) > z else 0

                        if self.is_valid_reading(dt, stt, sig=sigt, sigm=sigmt, refl=rt) or (dt > 0 and stt in (5, 6, 9, 10, 13)):
                            dists[z] = dt
                            refls[z] = rt
                            stats[z] = stt
                            sigs[z] = sigt
                            sigms[z] = sigmt
                            tgt_indices[z] = t
                            found_fallback = True
                            break

                    if not found_fallback:
                        dists[z] = d0
                        refls[z] = r0
                        stats[z] = st0
                        sigs[z] = sig0
                        sigms[z] = sigm0
                        tgt_indices[z] = 0
            return dists, refls, stats, sigs, sigms, tgt_indices
        else:
            t_idx = 0 if self.target_layer < 0 else min(self.nb_targets - 1, self.target_layer)
            dists = self.distances[:, t_idx] if len(self.distances) else np.zeros(n_zones, dtype=int)
            refls = self.reflectances[:, t_idx] if len(self.reflectances) else np.zeros(n_zones, dtype=int)
            stats = self.statuses[:, t_idx] if len(self.statuses) else np.zeros(n_zones, dtype=int)
            sigs = self.signals[:, t_idx] if len(self.signals) else np.zeros(n_zones, dtype=int)
            sigms = self.sigmas[:, t_idx] if len(self.sigmas) else np.zeros(n_zones, dtype=int)
            tgt_indices = np.full(n_zones, t_idx, dtype=int)
            return dists, refls, stats, sigs, sigms, tgt_indices

    def is_valid_status(self, st):
        if self.status_filter == "all":
            return True
        if self.status_filter == "valid_5":
            return st == 5
        if self.status_filter == "valid_5_6_9":
            return st in (5, 6, 9)
        return True

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        cell_w = w / self.grid_dim
        cell_h = h / self.grid_dim

        zones_grid = self.get_transformed_grid(np.arange(self.resolution))

        dists_layer, refls_layer, stats_layer, sigs_layer, sigms_layer, tgt_idx_layer = self.get_effective_layer_data()

        grid_d = self.get_transformed_grid(dists_layer)
        grid_r = self.get_transformed_grid(refls_layer)
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
                refl = int(grid_r[r, c])
                st = int(grid_st[r, c])
                sig = int(grid_sig[r, c])
                sigm = int(grid_sigm[r, c])
                tgt_idx = int(grid_tgt_idx[r, c])
                nt = int(grid_nt[r, c])
                amb = int(grid_amb[r, c])
                spad = int(grid_spad[r, c])
                z_idx = int(zones_grid[r, c])

                valid = self.is_valid_reading(dist, st, sig=sig, sigm=sigm, refl=refl)
                is_fallback = (self.target_layer == 100 and tgt_idx > 0 and valid)

                # Determine Tile Background Color based on Active Metric
                if not valid and self.active_metric in ("dist", "refl", "sig", "sigm"):
                    cell_color = QColor(20, 24, 35)
                else:
                    if self.active_metric == "dist":
                        norm = (dist - self.min_dist_range) / max(1.0, (self.max_dist_range - self.min_dist_range))
                        cell_color = turbo_colormap(1.0 - norm)
                    elif self.active_metric == "refl":
                        norm = min(1.0, max(0.0, refl / 100.0))
                        cell_color = viridis_colormap(norm)
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
                        norm = min(1.0, max(0.0, nt / 4.0))
                        cell_color = viridis_colormap(norm)
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
                    painter.setPen(QPen(QColor("#38BDF8"), 1.8, Qt.DashLine))
                else:
                    painter.setPen(QPen(QColor("#141824"), 1))
                painter.drawRoundedRect(int(x + 2), int(y + 2), int(cell_w - 4), int(cell_h - 4), 4, 4)

                # --- Line 1: Primary Distance / Metric Value (Large Bold) ---
                if self.active_metric == "dist":
                    txt_line1 = f"{dist} mm" if (valid and self.grid_dim == 4) else (f"{dist}" if valid else "---")
                elif self.active_metric == "refl":
                    txt_line1 = f"{refl}%" if valid else "---"
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

                max_txt_w = cell_w - 8
                target_pt1 = 15.5 if self.grid_dim == 4 else 9.5
                while target_pt1 > 6:
                    test_font = QFont("Segoe UI", int(target_pt1), QFont.Bold)
                    metrics = painter.fontMetrics()
                    if metrics.horizontalAdvance(txt_line1) <= max_txt_w:
                        break
                    target_pt1 -= 0.5

                painter.setFont(QFont("Segoe UI", max(5, int(target_pt1)), QFont.Bold))
                # Slight text shadow for readability against bright turbo background
                painter.setPen(QColor(0, 0, 0, 180))
                painter.drawText(int(x + 1), int(y + cell_h * 0.05 + 1), int(cell_w), int(cell_h * 0.35), Qt.AlignCenter, txt_line1)
                painter.setPen(QColor(255, 255, 255))
                painter.drawText(int(x), int(y + cell_h * 0.05), int(cell_w), int(cell_h * 0.35), Qt.AlignCenter, txt_line1)

                # --- Line 2: Inner Sub-Tile (Color-Coded Reflectivity / Metric Badge) ---
                refl_bg, refl_fg, refl_border = reflectance_badge_colors(refl if valid else 0)
                if is_fallback:
                    refl_border = QColor("#38BDF8")

                sub_w = int(cell_w * 0.82 if self.grid_dim == 4 else cell_w * 0.88)
                sub_h = int(cell_h * 0.28 if self.grid_dim == 4 else cell_h * 0.27)
                sub_x = int(x + (cell_w - sub_w) / 2)
                sub_y = int(y + cell_h * 0.41 if self.grid_dim == 4 else y + cell_h * 0.39)
                sub_r = 4 if self.grid_dim == 4 else 3

                painter.setBrush(QBrush(refl_bg))
                painter.setPen(QPen(refl_border, 1.5 if is_fallback else 1))
                painter.drawRoundedRect(sub_x, sub_y, sub_w, sub_h, sub_r, sub_r)

                if is_fallback:
                    txt_sub = f"🥈 {refl}% (T{tgt_idx+1} Fallback)" if (valid and self.grid_dim == 4) else f"{refl}% T{tgt_idx+1}"
                elif self.active_metric == "dist":
                    txt_sub = f"🔆 {refl}% Refl" if (valid and self.grid_dim == 4) else (f"{refl}%" if valid else "-- %")
                elif self.active_metric == "refl":
                    txt_sub = f"{dist} mm" if (valid and self.grid_dim == 4) else (f"{dist}" if valid else "---")
                else:
                    txt_sub = f"{refl}% Refl" if self.grid_dim == 4 else f"{refl}%"

                target_pt2 = 10.5 if self.grid_dim == 4 else 7.5
                while target_pt2 > 5:
                    test_font = QFont("Segoe UI", int(target_pt2), QFont.Bold)
                    metrics = painter.fontMetrics()
                    if metrics.horizontalAdvance(txt_sub) <= sub_w - 4:
                        break
                    target_pt2 -= 0.5

                painter.setFont(QFont("Segoe UI", max(4, int(target_pt2)), QFont.Bold))
                painter.setPen(refl_fg)
                painter.drawText(sub_x, sub_y, sub_w, sub_h, Qt.AlignCenter, txt_sub)

                # --- Line 3: Bottom Pill Badge (Zone ID + Status + Multi-Target Dots) ---
                pill_w = int(cell_w * 0.80 if self.grid_dim == 4 else cell_w * 0.88)
                pill_h = int(cell_h * 0.22 if self.grid_dim == 4 else cell_h * 0.24)
                pill_x = int(x + (cell_w - pill_w) / 2)
                pill_y = int(y + cell_h * 0.72 if self.grid_dim == 4 else y + cell_h * 0.69)
                pill_r = 3 if self.grid_dim == 4 else 2

                painter.setBrush(QBrush(QColor(10, 14, 24, 215)))
                painter.setPen(QPen(QColor(40, 50, 70, 180), 1))
                painter.drawRoundedRect(pill_x, pill_y, pill_w, pill_h, pill_r, pill_r)

                status_desc, st_hex, _ = get_status_info(st)
                tgt_dots = "•" * min(4, nt) if nt > 0 else ""

                if is_fallback:
                    if self.grid_dim == 4:
                        sub_txt = f"Z{z_idx} • s{st} [T{tgt_idx+1} FB] {tgt_dots}"
                        pill_pt = 7.8
                    else:
                        sub_txt = f"Z{z_idx} T{tgt_idx+1} s{st}"
                        pill_pt = 6.0
                else:
                    if self.grid_dim == 4:
                        sub_txt = f"Z{z_idx} • s{st} {tgt_dots}"
                        pill_pt = 8.5
                    else:
                        sub_txt = f"Z{z_idx} s{st}"
                        pill_pt = 6.5

                painter.setFont(QFont("Segoe UI", max(4, int(pill_pt)), QFont.Bold))
                painter.setPen(QColor(st_hex))
                painter.drawText(pill_x, pill_y, pill_w, pill_h, Qt.AlignCenter, sub_txt)





# =============================================================================
# 3D DEPTH & REFLECTANCE SURFACE / POINT CLOUD VISUALIZER
# =============================================================================

class Depth3DVisualizer(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.figure = Figure(facecolor="#10141e", edgecolor='none')
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.ax.set_facecolor("#10141e")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.canvas)
        self._style_3d_axes()
        self.last_draw_time = 0

    def _style_3d_axes(self):
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False
        self.ax.xaxis.pane.set_edgecolor("#232a3d")
        self.ax.yaxis.pane.set_edgecolor("#232a3d")
        self.ax.zaxis.pane.set_edgecolor("#232a3d")
        self.ax.tick_params(colors='#8fa0be', labelsize=8)
        self.ax.set_xlabel('X (Zone)', color='#8fa0be', fontsize=8.5)
        self.ax.set_ylabel('Y (Zone)', color='#8fa0be', fontsize=8.5)
        self.ax.set_zlabel('Depth (mm)', color='#00E5FF', fontsize=8.5)

    def update_3d_view(self, res, dists_layer, refls_layer, flip_h=False, flip_v=False, rotation=0, metric="dist"):
        now = time.perf_counter()
        if now - self.last_draw_time < 0.04:  # Max 25 FPS
            return
        self.last_draw_time = now

        dim = 8 if res == 64 else 4
        if len(dists_layer) < dim * dim:
            return

        grid_d = np.array(dists_layer[:dim*dim]).reshape((dim, dim))
        grid_r = np.array(refls_layer[:dim*dim]).reshape((dim, dim)) if len(refls_layer) >= dim*dim else np.zeros((dim, dim))

        if flip_h:
            grid_d = np.fliplr(grid_d)
            grid_r = np.fliplr(grid_r)
        if flip_v:
            grid_d = np.flipud(grid_d)
            grid_r = np.flipud(grid_r)
        if rotation == 90:
            grid_d = np.rot90(grid_d, -1)
            grid_r = np.rot90(grid_r, -1)
        elif rotation == 180:
            grid_d = np.rot90(grid_d, 2)
            grid_r = np.rot90(grid_r, 2)
        elif rotation == 270:
            grid_d = np.rot90(grid_d, 1)
            grid_r = np.rot90(grid_r, 1)

        grid_d_clamped = np.where(grid_d <= 0, 3000, grid_d)

        x = np.arange(dim)
        y = np.arange(dim)
        X, Y = np.meshgrid(x, y)

        elev = self.ax.elev
        azim = self.ax.azim

        self.ax.clear()
        self.ax.set_facecolor("#10141e")
        self._style_3d_axes()

        cmap = 'viridis' if metric == "refl" else 'turbo_r'
        color_grid = grid_r if metric == "refl" else grid_d_clamped

        self.ax.plot_surface(X, Y, grid_d_clamped, cmap=cmap, edgecolor='#232a3d', linewidth=0.5, alpha=0.88)
        self.ax.scatter(X, Y, grid_d_clamped, color='#00E5FF', s=25)

        self.ax.set_zlim(0, 3000)
        self.ax.view_init(elev=elev, azim=azim)
        self.figure.tight_layout()
        self.canvas.draw_idle()


# =============================================================================
# MULTI-TARGET RANGE DISTRIBUTION RADAR / DEPTH SLICES
# =============================================================================

class MultiTargetRadarWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.figure = Figure(facecolor="#10141e", edgecolor='none')
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_facecolor("#151a27")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self._style_chart()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.canvas)
        self.last_draw_time = 0

    def _style_chart(self):
        self.ax.spines['bottom'].set_color('#232a3d')
        self.ax.spines['top'].set_color('#232a3d')
        self.ax.spines['right'].set_color('#232a3d')
        self.ax.spines['left'].set_color('#232a3d')
        self.ax.tick_params(colors='#8fa0be', labelsize=8)
        self.ax.grid(True, linestyle='--', alpha=0.3, color='#28324a')
        self.ax.set_ylabel('Distance (mm)', color='#00E5FF', fontsize=8.5)
        self.ax.set_xlabel('Zone Index #', color='#8fa0be', fontsize=8.5)
        self.ax.set_ylim(0, 3500)

    def update_targets(self, res, distances_2d, refl_2d, stats_2d, signals_2d=None, sigmas_2d=None, is_valid_func=None):
        now = time.perf_counter()
        if now - self.last_draw_time < 0.05:  # Max 20 FPS
            return
        self.last_draw_time = now

        self.ax.clear()
        self.ax.set_facecolor("#151a27")
        self._style_chart()

        colors = ['#00E5FF', '#FFB300', '#EC407A', '#00E676']
        labels = ['Target 1 (Primary)', 'Target 2', 'Target 3', 'Target 4']

        n_zones = min(res, len(distances_2d))
        self.ax.set_xlim(-0.5, n_zones - 0.5)

        for t in range(min(4, distances_2d.shape[1])):
            zone_indices = []
            dists = []
            sizes = []
            for z in range(n_zones):
                d = distances_2d[z, t]
                st = stats_2d[z, t] if stats_2d.shape[1] > t else 0
                r = refl_2d[z, t] if refl_2d.shape[1] > t else 0
                sig = signals_2d[z, t] if (signals_2d is not None and signals_2d.shape[1] > t) else None
                sigm = sigmas_2d[z, t] if (sigmas_2d is not None and sigmas_2d.shape[1] > t) else None

                valid = is_valid_func(d, st, sig=sig, sigm=sigm, refl=r) if is_valid_func else (d > 0 and st in (5, 6, 9))
                if valid:
                    zone_indices.append(z)
                    dists.append(d)
                    sizes.append(max(18, min(160, r * 1.6)))

            if zone_indices:
                self.ax.scatter(zone_indices, dists, color=colors[t], s=sizes, alpha=0.85, label=labels[t], edgecolors='white', linewidth=0.5)

        self.ax.legend(loc='upper right', facecolor='#10141e', edgecolor='#232a3d', fontsize=8, labelcolor='#ffffff')
        self.figure.tight_layout()
        self.canvas.draw_idle()



# =============================================================================
# DUAL-TRACE TELEMETRY OSCILLOSCOPE (DEPTH + REFLECTANCE)
# =============================================================================

class TelemetryStripChart(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.figure = Figure(facecolor="#10141e", edgecolor='none')
        self.canvas = FigureCanvas(self.figure)
        
        # Dual subplots: Top = Distance (mm), Bottom = Reflectance (%)
        self.ax_dist = self.figure.add_subplot(211)
        self.ax_dist.set_facecolor("#151a27")
        self.ax_refl = self.figure.add_subplot(212, sharex=self.ax_dist)
        self.ax_refl.set_facecolor("#151a27")
        
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.max_history = 120
        self.hist_min_d = []
        self.hist_mean_d = []
        self.hist_max_d = []
        self.hist_sel_d = []

        self.hist_mean_r = []
        self.hist_sel_r = []

        self.last_selected_idx = 0
        self.last_draw_time = 0

        self._style_charts()
        self.line_mean_d, = self.ax_dist.plot([], [], color='#00E5FF', label='Mean Depth', linewidth=1.8)
        self.line_sel_d, = self.ax_dist.plot([], [], color='#00E676', label='Selected Zone', linewidth=1.4, linestyle='--')
        self.poly_fill_d = None
        self.ax_dist.legend(loc='upper right', facecolor='#10141e', edgecolor='#232a3d', fontsize=7.5, labelcolor='#ffffff')

        self.line_mean_r, = self.ax_refl.plot([], [], color='#FFB300', label='Mean Reflectance', linewidth=1.6)
        self.line_sel_r, = self.ax_refl.plot([], [], color='#FF4081', label='Selected Zone Refl', linewidth=1.2, linestyle=':')
        self.ax_refl.legend(loc='upper right', facecolor='#10141e', edgecolor='#232a3d', fontsize=7.5, labelcolor='#ffffff')

        self.figure.tight_layout()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.canvas)

    def _style_charts(self):
        for ax, ylabel in [(self.ax_dist, 'Depth (mm)'), (self.ax_refl, 'Reflectance (%)')]:
            ax.spines['bottom'].set_color('#232a3d')
            ax.spines['top'].set_color('#232a3d')
            ax.spines['right'].set_color('#232a3d')
            ax.spines['left'].set_color('#232a3d')
            ax.tick_params(colors='#8fa0be', labelsize=8)
            ax.grid(True, linestyle='--', alpha=0.25, color='#28324a')
            ax.set_ylabel(ylabel, color='#8fa0be', fontsize=8)
        self.ax_refl.set_xlabel('Frames', color='#8fa0be', fontsize=8)
        self.ax_dist.set_ylim(0, 3000)
        self.ax_refl.set_ylim(0, 100)

    def add_telemetry_point(self, dists_t0, refls_t0, selected_idx):
        valid_d = [d for d in dists_t0 if d > 0]
        if not valid_d:
            return

        min_d = min(valid_d)
        mean_d = float(np.mean(valid_d))
        max_d = max(valid_d)
        sel_d = dists_t0[selected_idx] if selected_idx < len(dists_t0) and dists_t0[selected_idx] > 0 else mean_d

        valid_r = [refls_t0[i] for i in range(len(dists_t0)) if dists_t0[i] > 0 and i < len(refls_t0)]
        mean_r = float(np.mean(valid_r)) if valid_r else 0.0
        sel_r = refls_t0[selected_idx] if selected_idx < len(refls_t0) else mean_r

        self.hist_min_d.append(min_d)
        self.hist_mean_d.append(mean_d)
        self.hist_max_d.append(max_d)
        self.hist_sel_d.append(sel_d)

        self.hist_mean_r.append(mean_r)
        self.hist_sel_r.append(sel_r)
        self.last_selected_idx = selected_idx

        if len(self.hist_mean_d) > self.max_history:
            self.hist_min_d.pop(0)
            self.hist_mean_d.pop(0)
            self.hist_max_d.pop(0)
            self.hist_sel_d.pop(0)
            self.hist_mean_r.pop(0)
            self.hist_sel_r.pop(0)

    def redraw_chart(self):
        now = time.perf_counter()
        if now - self.last_draw_time < 0.05:  # Max 20 FPS
            return
        self.last_draw_time = now

        if not self.hist_mean_d:
            return

        x = np.arange(len(self.hist_mean_d))
        self.line_mean_d.set_data(x, self.hist_mean_d)
        self.line_sel_d.set_data(x, self.hist_sel_d)
        self.line_sel_d.set_label(f'Zone {self.last_selected_idx} Depth')

        self.line_mean_r.set_data(x, self.hist_mean_r)
        self.line_sel_r.set_data(x, self.hist_sel_r)
        self.line_sel_r.set_label(f'Zone {self.last_selected_idx} Refl')

        if self.poly_fill_d:
            try: self.poly_fill_d.remove()
            except Exception: pass
        self.poly_fill_d = self.ax_dist.fill_between(x, self.hist_min_d, self.hist_max_d, color='#00E5FF', alpha=0.15)

        max_y = max(2500, max(self.hist_max_d) + 200)
        self.ax_dist.set_ylim(0, max_y)
        self.ax_dist.set_xlim(0, max(self.max_history, len(x)))
        self.canvas.draw_idle()


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
            self.resize(1360, 840)

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
        self.serial_engine.data_received.connect(self.on_data_received)
        self.serial_engine.config_received.connect(self.on_config_received)
        self.serial_engine.ack_received.connect(self.on_ack_received)
        self.serial_engine.connection_changed.connect(self.on_connection_changed)
        self.serial_engine.raw_log.connect(self.on_raw_log)

        self.fps_timer = QTimer(self)
        self.fps_timer.timeout.connect(self.update_fps_calc)
        self.fps_timer.start(500)
        self.frame_counter = 0
        self.last_fps_time = time.perf_counter()
        self.current_fps = 0.0

        self.init_ui()
        self.refresh_com_ports()

        # Auto-connect if COM7, COM5, or another active serial port is available
        available_ports = [p.device for p in serial.tools.list_ports.comports()]
        target_port = "COM7" if "COM7" in available_ports else ("COM5" if "COM5" in available_ports else (available_ports[0] if available_ports else None))
        if target_port:
            self.port_combo.setCurrentText(target_port)
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

        # Tab 1: 2D Matrix Multi-Metric Heatmap & Multi-Target Zone Inspector
        tab_2d = QWidget()
        tab_2d_layout = QHBoxLayout(tab_2d)
        tab_2d_layout.setContentsMargins(4, 4, 4, 4)
        tab_2d_layout.setSpacing(6)

        # Left Column: Toolbar + Heatmap
        hm_col = QWidget()
        hm_col_layout = QVBoxLayout(hm_col)
        hm_col_layout.setContentsMargins(0, 0, 0, 0)
        hm_col_layout.setSpacing(4)

        # Heatmap Filter & Layer Toolbar
        toolbar = QHBoxLayout()
        toolbar.setSpacing(6)

        toolbar.addWidget(QLabel("📊 Metric:"))
        self.metric_combo = QComboBox()
        self.metric_combo.addItem("📏 Distance (mm)", "dist")
        self.metric_combo.addItem("🔆 Reflectance (%)", "refl")
        self.metric_combo.addItem("📶 Signal Rate (kcps/SPAD)", "sig")
        self.metric_combo.addItem("〰️ Range Noise (±mm)", "sigm")
        self.metric_combo.addItem("💡 Ambient Noise (kcps)", "amb")
        self.metric_combo.addItem("⚡ Active SPADs Count", "spad")
        self.metric_combo.addItem("🎯 Target Count (0-4)", "ntar")
        self.metric_combo.addItem("🏷️ Target Status & Validity", "st")
        self.metric_combo.currentIndexChanged.connect(self.on_metric_changed)
        toolbar.addWidget(self.metric_combo)

        toolbar.addWidget(QLabel("🎯 Target Layer:"))
        self.layer_combo = QComboBox()
        self.layer_combo.addItem("🥇 Target #1 (Primary)", 0)
        self.layer_combo.addItem("🔄 Smart Fallback (T1 → Secondary)", 100)
        self.layer_combo.addItem("🥈 Target #2 (Secondary Only)", 1)
        self.layer_combo.addItem("🥉 Target #3 (Tertiary Only)", 2)
        self.layer_combo.addItem("4️⃣ Target #4 (Quaternary Only)", 3)
        self.layer_combo.addItem("👥 Multi-Target Composite", -1)
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

        hm_col_layout.addLayout(toolbar)

        self.heatmap = MatrixHeatmapWidget()
        self.heatmap.cell_selected.connect(self.on_cell_selected)
        hm_col_layout.addWidget(self.heatmap, 1)

        tab_2d_layout.addWidget(hm_col, 3)

        # Right Column: Per-Zone Multi-Target Inspector Sidebar
        inspector_frame = QFrame()
        inspector_frame.setStyleSheet("background-color: #12151f; border-radius: 6px; border: 1px solid #232a3d;")
        ins_layout = QVBoxLayout(inspector_frame)
        ins_layout.setContentsMargins(8, 8, 8, 8)
        ins_layout.setSpacing(6)

        ins_title = QLabel("🔍 ZONE MULTI-TARGET INSPECTOR")
        ins_title.setStyleSheet("font-weight: bold; color: #00E5FF; font-size: 8.5pt;")
        ins_layout.addWidget(ins_title)

        self.ins_zone_lbl = QLabel("Zone #0 (Row 0, Col 0)")
        self.ins_zone_lbl.setStyleSheet("font-size: 9.5pt; font-weight: bold; color: #ffffff;")
        ins_layout.addWidget(self.ins_zone_lbl)

        self.ins_ntar_lbl = QLabel("🎯 Targets Detected: 0 / 4")
        self.ins_ntar_lbl.setStyleSheet("color: #38bdf8; font-weight: bold;")
        ins_layout.addWidget(self.ins_ntar_lbl)

        # Target breakdown cards (4 targets)
        self.target_cards = []
        for t in range(4):
            card = QFrame()
            card.setStyleSheet("background-color: #181d2b; border: 1px solid #232a3d; border-radius: 4px; padding: 4px;")
            card_layout = QVBoxLayout(card)
            card_layout.setContentsMargins(4, 4, 4, 4)
            card_layout.setSpacing(2)

            t_title = QLabel(f"Target #{t+1} {'(Primary)' if t==0 else ''}")
            t_title.setStyleSheet(f"font-weight: bold; color: {['#00E5FF', '#FFB300', '#EC407A', '#00E676'][t]}; font-size: 8pt;")
            card_layout.addWidget(t_title)

            t_val_row = QHBoxLayout()
            d_lbl = QLabel("--- mm")
            d_lbl.setStyleSheet("font-size: 10.5pt; font-weight: bold; color: #ffffff;")
            t_val_row.addWidget(d_lbl)

            r_lbl = QLabel("--- %")
            r_lbl.setStyleSheet("font-size: 9pt; font-weight: bold; color: #FFB300;")
            t_val_row.addWidget(r_lbl)
            card_layout.addLayout(t_val_row)

            meta_lbl = QLabel("Sig: --- kcps | Noise: ±-- mm")
            meta_lbl.setStyleSheet("font-size: 7.5pt; color: #8fa0be;")
            card_layout.addWidget(meta_lbl)

            st_lbl = QLabel("Status: ---")
            st_lbl.setStyleSheet("font-size: 7.5pt; color: #cbd5e1;")
            card_layout.addWidget(st_lbl)

            ins_layout.addWidget(card)
            self.target_cards.append({
                "frame": card,
                "d_lbl": d_lbl,
                "r_lbl": r_lbl,
                "meta_lbl": meta_lbl,
                "st_lbl": st_lbl
            })

        ins_layout.addStretch()

        # Zone Environment & SPADs
        env_box = QFrame()
        env_box.setStyleSheet("background-color: #161b28; border-radius: 4px; padding: 4px;")
        env_layout = QVBoxLayout(env_box)
        env_layout.setContentsMargins(4, 4, 4, 4)
        env_layout.setSpacing(2)
        
        self.ins_amb_lbl = QLabel("💡 Ambient Noise: --- kcps/SPAD")
        env_layout.addWidget(self.ins_amb_lbl)
        self.ins_spad_lbl = QLabel("⚡ SPADs Enabled: ---")
        env_layout.addWidget(self.ins_spad_lbl)
        ins_layout.addWidget(env_box)

        # Global Frame Summary
        kpi_title = QLabel("📊 FRAME SUMMARY")
        kpi_title.setStyleSheet("font-weight: bold; color: #00E5FF; font-size: 8pt; margin-top: 4px;")
        ins_layout.addWidget(kpi_title)

        self.kpi_mean_lbl = QLabel("Mean Depth: --- mm")
        ins_layout.addWidget(self.kpi_mean_lbl)

        self.kpi_refl_lbl = QLabel("Mean Refl: --- %")
        ins_layout.addWidget(self.kpi_refl_lbl)

        self.kpi_valid_lbl = QLabel("Valid Targets: 0 / 16 (0%)")
        ins_layout.addWidget(self.kpi_valid_lbl)

        tab_2d_layout.addWidget(inspector_frame, 1)
        self.vis_tabs.addTab(tab_2d, "🗺️ 2D Multi-Target Heatmap")

        # Tab 2: 3D Point Cloud / Surface
        self.visualizer_3d = Depth3DVisualizer()
        self.vis_tabs.addTab(self.visualizer_3d, "🌐 3D Point Cloud & Surface")

        # Tab 3: Multi-Target Range Distribution Radar
        self.target_radar = MultiTargetRadarWidget()
        self.vis_tabs.addTab(self.target_radar, "🎯 Multi-Target Range Slices")

        # Tab 4: Dual-Trace Historical Strip Chart
        self.strip_chart = TelemetryStripChart()
        self.vis_tabs.addTab(self.strip_chart, "📈 Depth & Reflectance Oscilloscope")

        # Tab 5: Raw Serial Logs
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

        # Resolution & Target Order
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

        # Ranging Frequency Slider
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

        # Integration Time Slider & Live SpinBox
        integ_layout = QHBoxLayout()
        self.integ_lbl = QLabel("Integ Time: 5 ms")
        self.integ_lbl.setMinimumWidth(85)
        integ_layout.addWidget(self.integ_lbl)
        self.integ_slider = QSlider(Qt.Horizontal)
        self.integ_slider.setRange(2, 200)
        self.integ_slider.setValue(5)
        integ_layout.addWidget(self.integ_slider)
        self.integ_spin = QSpinBox()
        self.integ_spin.setRange(2, 1000)
        self.integ_spin.setValue(5)
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

        btn_integ_auto = QPushButton("⚡ Auto / Max")
        btn_integ_auto.setToolTip("Set integration time to maximum possible for current frequency budget")
        btn_integ_auto.clicked.connect(lambda: self.set_smart_integration("auto"))
        integ_btn_row.addWidget(btn_integ_auto)

        btn_integ_fast = QPushButton("🚀 Fast (5ms)")
        btn_integ_fast.setToolTip("Fast integration time (5 ms) for high-speed racing / minimal motion blur")
        btn_integ_fast.clicked.connect(lambda: self.set_smart_integration("fast"))
        integ_btn_row.addWidget(btn_integ_fast)

        btn_integ_bal = QPushButton("⚖️ Bal (15ms)")
        btn_integ_bal.setToolTip("Balanced integration time (15 ms) for high signal-to-noise ratio")
        btn_integ_bal.clicked.connect(lambda: self.set_smart_integration("balanced"))
        integ_btn_row.addWidget(btn_integ_bal)

        btn_integ_deep = QPushButton("🔭 Deep (50ms)")
        btn_integ_deep.setToolTip("Deep photon accumulation (50 ms) for dark or low-reflectance surfaces")
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


        # Mode & Power
        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Mode:"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("Continuous (High Perf)", 1)
        self.mode_combo.addItem("Autonomous (Low Power)", 3)
        row2.addWidget(self.mode_combo)

        row2.addWidget(QLabel("Power:"))
        self.pwr_combo = QComboBox()
        self.pwr_combo.addItem("Active Wakeup", 1)
        self.pwr_combo.addItem("Low Power Sleep", 0)
        row2.addWidget(self.pwr_combo)
        param_layout.addLayout(row2)

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

        # Toggleable ST Table 4 Status Glossary Drawer (Full 15 Status Codes)
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

        # 4. Min Reflectance Gate Slider
        refl_row = QHBoxLayout()
        self.min_refl_lbl = QLabel("Min Refl: Off")
        self.min_refl_lbl.setMinimumWidth(85)
        refl_row.addWidget(self.min_refl_lbl)
        self.min_refl_slider = QSlider(Qt.Horizontal)
        self.min_refl_slider.setRange(0, 30)
        self.min_refl_slider.setValue(0)
        refl_row.addWidget(self.min_refl_slider)
        self.min_refl_spin = QSpinBox()
        self.min_refl_spin.setRange(0, 30)
        self.min_refl_spin.setValue(0)
        self.min_refl_spin.setSuffix(" %")
        self.min_refl_spin.setFixedWidth(52)
        refl_row.addWidget(self.min_refl_spin)
        self.min_refl_slider.valueChanged.connect(self.min_refl_spin.setValue)
        self.min_refl_spin.valueChanged.connect(self.min_refl_slider.setValue)
        self.min_refl_spin.valueChanged.connect(self.on_min_refl_changed)
        picky_layout.addLayout(refl_row)

        # 5. Distance Range Gating
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

        cal_row1.addWidget(QLabel("Refl:"))
        self.cal_refl_spin = QSpinBox()
        self.cal_refl_spin.setRange(1, 99)
        self.cal_refl_spin.setValue(3)
        self.cal_refl_spin.setSuffix(" %")
        cal_row1.addWidget(self.cal_refl_spin)
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
        self.cal_btn.setToolTip("Run ST crosstalk calibration with target at specified distance & reflectance")
        self.cal_btn.clicked.connect(self.calibrate_xtalk)
        margin_row.addWidget(self.cal_btn)
        xtalk_layout.addLayout(margin_row)

        ctrl_layout.addWidget(xtalk_group)

        # --- Card 4: Quick Performance & Pickiness Presets ---
        preset_group = QGroupBox("⚡ RACING & DETECTION PRESETS")
        preset_layout = QGridLayout(preset_group)
        preset_layout.setContentsMargins(6, 6, 6, 6)
        preset_layout.setSpacing(4)

        btn_preset_speed = QPushButton("🚀 4x4 @ 60 Hz (Max Speed)")
        btn_preset_speed.clicked.connect(lambda: self.load_preset(16, 60, 1, 1, 1, 50, "valid_5_6_9", 0, 0, 0))
        preset_layout.addWidget(btn_preset_speed, 0, 0)

        btn_preset_prec = QPushButton("🎯 8x8 @ 15 Hz (Max Res)")
        btn_preset_prec.clicked.connect(lambda: self.load_preset(64, 15, 1, 1, 1, 50, "valid_5_6_9", 0, 0, 0))
        preset_layout.addWidget(btn_preset_prec, 0, 1)

        btn_preset_picky = QPushButton("🛡️ Ultra Picky (High Confidence)")
        btn_preset_picky.clicked.connect(lambda: self.load_preset(16, 60, 1, 5, 1, 100, "valid_5", 5, 10, 3))
        preset_layout.addWidget(btn_preset_picky, 1, 0)

        btn_preset_sens = QPushButton("👁️ Ultra Sensitive (Faint Targets)")
        btn_preset_sens.clicked.connect(lambda: self.load_preset(16, 30, 2, 0, 1, 0, "valid_extended", 0, 0, 0))
        preset_layout.addWidget(btn_preset_sens, 1, 1)

        ctrl_layout.addWidget(preset_group)


        # --- Card 4: Action Panel ---
        action_box = QFrame()
        action_box.setStyleSheet("background-color: #12151f; border-radius: 6px; border: 1px solid #232a3d; padding: 4px;")
        act_layout = QVBoxLayout(action_box)
        act_layout.setContentsMargins(6, 6, 6, 6)
        act_layout.setSpacing(4)

        self.apply_btn = QPushButton("⚡ APPLY SENSOR CHANGES")
        self.apply_btn.setObjectName("applyBtn")
        self.apply_btn.clicked.connect(self.apply_changes)
        act_layout.addWidget(self.apply_btn)

        self.apply_status_lbl = QLabel("Ready • Continuous Mode Active")
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
        if not ports:
            ports = ["COM5"]
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
        self.update_active_3d_view()

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

    def on_min_refl_changed(self, val):
        self.heatmap.min_refl_filter = val
        self.min_refl_lbl.setText(f"Min Refl: {val} %" if val > 0 else "Min Refl: Off")
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
            val = 5
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
        self.update_active_3d_view()

    def load_preset(self, res, freq, order, sharp, mode, margin, conf="valid_5_6_9", sigma=0, min_sig=0, min_refl=0, integ=5):
        idx = self.res_combo.findData(res)
        if idx >= 0: self.res_combo.setCurrentIndex(idx)
        self.freq_spin.setValue(freq)
        self.integ_spin.setValue(integ)
        idx = self.order_combo.findData(order)
        if idx >= 0: self.order_combo.setCurrentIndex(idx)
        self.sharp_spin.setValue(sharp)
        idx = self.mode_combo.findData(mode)
        if idx >= 0: self.mode_combo.setCurrentIndex(idx)
        self.margin_spin.setValue(margin)
        
        idx_conf = self.conf_combo.findData(conf)
        if idx_conf >= 0: self.conf_combo.setCurrentIndex(idx_conf)
        self.sigma_spin.setValue(sigma)
        self.min_sig_spin.setValue(min_sig)
        self.min_refl_spin.setValue(min_refl)
        
        self.apply_changes()

    def apply_changes(self):
        res = self.res_combo.currentData()
        freq = self.freq_spin.value()
        integ = self.integ_spin.value()
        order = self.order_combo.currentData()
        sharp = self.sharp_spin.value()
        mode = self.mode_combo.currentData()
        pwr = self.pwr_combo.currentData()
        margin = self.margin_spin.value()

        cmd = f"SET_CONFIG:res={res},freq={freq},integ={integ},order={order},sharp={sharp},mode={mode},pwr={pwr},margin={margin}"
        self.apply_btn.setEnabled(False)
        self.apply_status_lbl.setText("⏳ Applying sensor changes...")
        self.apply_status_lbl.setStyleSheet("color: #00E5FF; font-weight: bold;")
        self.serial_engine.send_command(cmd)

    def reset_sensor(self):
        self.apply_status_lbl.setText("🔄 Hardware resetting...")
        self.serial_engine.send_command("RESET")

    def calibrate_xtalk(self):
        dist = self.cal_dist_spin.value()
        samples = self.cal_samples_spin.value()
        refl = self.cal_refl_spin.value()
        self.apply_status_lbl.setText(f"📐 Calibrating crosstalk @ {dist}mm, {refl}% refl...")
        self.serial_engine.send_command(f"CALIBRATE_XTALK:dist={dist},samples={samples},refl={refl}")

    def on_ack_received(self, data):
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
        integ = data.get("integ", 5)
        order = data.get("order", 1)
        sharp = data.get("sharp", 1)
        mode = data.get("mode", 1)
        pwr = data.get("power", 1)
        margin = data.get("margin", 50)

        idx = self.res_combo.findData(res)
        if idx >= 0: self.res_combo.setCurrentIndex(idx)
        self.freq_spin.setValue(freq)
        self.integ_spin.setValue(integ)
        idx = self.order_combo.findData(order)
        if idx >= 0: self.order_combo.setCurrentIndex(idx)
        self.sharp_spin.setValue(sharp)
        idx = self.mode_combo.findData(mode)
        if idx >= 0: self.mode_combo.setCurrentIndex(idx)
        if idx >= 0: self.pwr_combo.setCurrentIndex(idx)
        self.margin_spin.setValue(margin)

    def on_data_received(self, data):

        self.frame_counter += 1
        res = data.get("r", 16)
        nt = data.get("nt", 4)
        ntar = data.get("ntar", [])
        dists = data.get("d", [])
        refls = data.get("refl", [])
        statuses = data.get("st", [])
        signals = data.get("sig", [])
        sigmas = data.get("sigm", [])
        ambs = data.get("amb", [])
        spads = data.get("spad", [])
        temp_c = data.get("temp", None)

        if temp_c is not None:
            self.temp_badge.setText(f"🌡️ {temp_c} °C")

        # Update Heatmap data model
        self.heatmap.update_telemetry(res, nt, ntar, dists, refls, statuses, signals, sigmas, ambs, spads)

        # Route updates to active tab
        tab_idx = self.vis_tabs.currentIndex()
        if tab_idx == 1:
            self.update_active_3d_view()
        elif tab_idx == 2:
            self.target_radar.update_targets(
                res,
                self.heatmap.distances,
                self.heatmap.reflectances,
                self.heatmap.statuses,
                signals_2d=self.heatmap.signals,
                sigmas_2d=self.heatmap.sigmas,
                is_valid_func=self.heatmap.is_valid_reading
            )

        elif tab_idx == 3:
            d_t0 = self.heatmap.distances[:, 0] if len(self.heatmap.distances) else []
            r_t0 = self.heatmap.reflectances[:, 0] if len(self.heatmap.reflectances) else []
            self.strip_chart.add_telemetry_point(d_t0, r_t0, self.heatmap.selected_zone)
            self.strip_chart.redraw_chart()

        # Update Zone Inspector Sidebar
        self.update_zone_inspector(self.heatmap.selected_zone)

        # Update Global KPIs using active pickiness & quality gating
        valid_indices = [
            z for z in range(res)
            if self.heatmap.is_valid_reading(
                self.heatmap.distances[z, 0],
                self.heatmap.statuses[z, 0],
                sig=self.heatmap.signals[z, 0],
                sigm=self.heatmap.sigmas[z, 0],
                refl=self.heatmap.reflectances[z, 0]
            )
        ]
        if valid_indices:
            valid_d = [self.heatmap.distances[z, 0] for z in valid_indices]
            valid_r = [self.heatmap.reflectances[z, 0] for z in valid_indices]
            self.kpi_mean_lbl.setText(f"Mean Depth: {np.mean(valid_d):.1f} mm")
            self.kpi_refl_lbl.setText(f"Mean Refl: {np.mean(valid_r):.1f} %")
        else:
            self.kpi_mean_lbl.setText("Mean Depth: --- mm")
            self.kpi_refl_lbl.setText("Mean Refl: --- %")

        valid_count = len(valid_indices)
        self.kpi_valid_lbl.setText(f"Valid Targets: {valid_count} / {res} ({int(100*valid_count/res)}%)")


    def update_zone_inspector(self, z_idx):
        zdata = self.heatmap.get_zone_data(z_idx)
        if not zdata:
            return

        dim = self.heatmap.grid_dim
        self.ins_zone_lbl.setText(f"Zone #{z_idx} (Row {z_idx//dim}, Col {z_idx%dim})")
        self.ins_ntar_lbl.setText(f"🎯 Targets Detected: {zdata['ntar']} / {self.heatmap.nb_targets}")

        self.ins_amb_lbl.setText(f"💡 Ambient Noise: {zdata['amb']} kcps/SPAD")
        self.ins_spad_lbl.setText(f"⚡ SPADs Enabled: {zdata['spad']}")

        for t in range(4):
            card = self.target_cards[t]
            if t < len(zdata['targets']):
                tgt = zdata['targets'][t]
                dist = tgt['dist']
                refl = tgt['refl']
                st = tgt['stat']
                sig = tgt['sig']
                sigm = tgt['sigm']

                st_desc, st_hex, conf = get_status_info(st)

                if dist > 0 and st in (5, 6, 9, 10, 13):
                    card['d_lbl'].setText(f"{dist} mm")
                    card['r_lbl'].setText(f"{refl}% refl")
                    card['meta_lbl'].setText(f"Sig: {sig} kcps | Noise: ±{sigm} mm")
                    card['st_lbl'].setText(f"s{st}: {st_desc}")
                    card['st_lbl'].setStyleSheet(f"font-size: 7.5pt; font-weight: bold; color: {st_hex};")
                else:
                    card['d_lbl'].setText("---")
                    card['r_lbl'].setText("-- %")
                    card['meta_lbl'].setText("Sig: -- kcps | Noise: ±-- mm")
                    card['st_lbl'].setText(f"s{st}: {st_desc}" if st != 0 else "No Target Detected")
                    card['st_lbl'].setStyleSheet("font-size: 7.5pt; color: #64748b;")
            else:
                card['d_lbl'].setText("---")
                card['r_lbl'].setText("-- %")
                card['meta_lbl'].setText("---")
                card['st_lbl'].setText("Inactive")

    def update_active_3d_view(self):
        dists, refls, stats, sigs, sigms, tgt_indices = self.heatmap.get_effective_layer_data()
        self.visualizer_3d.update_3d_view(
            self.heatmap.resolution,
            dists, refls,
            flip_h=self.heatmap.flip_h,
            flip_v=self.heatmap.flip_v,
            rotation=self.heatmap.rotation,
            metric="refl" if self.heatmap.active_metric == "refl" else "dist"
        )


    def on_cell_selected(self, idx):
        self.update_zone_inspector(idx)

    def on_raw_log(self, text):
        if self.vis_tabs.currentIndex() == 4:
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
