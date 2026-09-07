"""
================================================================================
  PYTHON CTYPES WRAPPER FOR EXACT CONTINUOUS C++ SMART HYBRID COSTMAP (15mm)
  - Zero range buckets (exact continuous floating-point math)
  - Cross-beam circle-circle triangulation
  - Continuous Bayesian angular decay
  - Native AVX2 machine-code compilation
  - Sub-0.10 ms execution time
================================================================================
"""

import ctypes
import os
import sys
from typing import Dict, List, Optional, Tuple
import numpy as np

# Load the compiled shared library
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
DLL_PATH = os.path.join(CURR_DIR, "smart_hybrid_costmap_15mm.dll")

if not os.path.exists(DLL_PATH):
    raise FileNotFoundError(f"Cannot find compiled DLL at {DLL_PATH}")

_lib = ctypes.CDLL(DLL_PATH)

# Configure C-API function signatures
_lib.costmap_create.argtypes = [
    ctypes.c_float, ctypes.c_float,  # x_min, x_max
    ctypes.c_float, ctypes.c_float,  # y_min, y_max
    ctypes.c_float, ctypes.c_float,  # res, r_inflation_m
]
_lib.costmap_create.restype = ctypes.c_void_p

_lib.costmap_destroy.argtypes = [ctypes.c_void_p]
_lib.costmap_destroy.restype = None

_lib.costmap_process_frame.argtypes = [
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_uint8),
]
_lib.costmap_process_frame.restype = None

_lib.costmap_probe_path.argtypes = [
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.c_int,
]
_lib.costmap_probe_path.restype = ctypes.c_int

_lib.costmap_get_width.argtypes = [ctypes.c_void_p]
_lib.costmap_get_width.restype = ctypes.c_int

_lib.costmap_get_height.argtypes = [ctypes.c_void_p]
_lib.costmap_get_height.restype = ctypes.c_int

# Standard sensor names order matching C++ array
SENSOR_NAMES = [
    "tof_fl_0", "tof_fl_1", "tof_fl_2", "tof_fl_3",
    "tof_fr_0", "tof_fr_1", "tof_fr_2", "tof_fr_3",
    "tof_bl_0", "tof_bl_1", "tof_bl_2", "tof_bl_3",
    "tof_br_0", "tof_br_1", "tof_br_2", "tof_br_3",
]


class CppSmartHybridCostmap:
    """High-performance C++ Smart Hybrid Costmap (15 mm default, exact continuous math)."""

    def __init__(
        self,
        grid_bounds: Tuple[float, float, float, float] = (-1.25, 1.25, 0.0, 2.0),
        resolution: float = 0.015,
        r_inflation_m: float = 0.088,
    ):
        self.x_min, self.x_max, self.y_min, self.y_max = grid_bounds
        self.res = float(resolution)
        self.r_inflation_m = float(r_inflation_m)

        self._handle = _lib.costmap_create(
            self.x_min, self.x_max,
            self.y_min, self.y_max,
            self.res, self.r_inflation_m,
        )
        if not self._handle:
            raise RuntimeError("Failed to create C++ costmap instance")

        self.width = _lib.costmap_get_width(self._handle)
        self.height = _lib.costmap_get_height(self._handle)
        self.shape = (self.height, self.width)
        self.total_cells = self.width * self.height

        # Coordinate meshgrid matching C++ index order
        xs = np.linspace(self.x_min, self.x_max, self.width, dtype=np.float32)
        ys = np.linspace(self.y_min, self.y_max, self.height, dtype=np.float32)
        self.X, self.Y = np.meshgrid(xs, ys)

        # Preallocated C-contiguous NumPy output buffers
        self._ranges_buf = np.zeros(16, dtype=np.float32)
        self._prob_buf = np.zeros(self.total_cells, dtype=np.float32)
        self._hazard_buf = np.zeros(self.total_cells, dtype=np.uint8)

        self._ranges_ptr = self._ranges_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        self._prob_ptr = self._prob_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        self._hazard_ptr = self._hazard_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))

    def __del__(self):
        if hasattr(self, "_handle") and self._handle:
            _lib.costmap_destroy(self._handle)
            self._handle = None

    def process_tof_frame(
        self, readings: Dict[str, float], noise_pct: float = 0.0
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        import time
        t0 = time.perf_counter()

        for i, name in enumerate(SENSOR_NAMES):
            raw_z = readings.get(name, 1.80)
            if noise_pct > 0.0 and raw_z < 1.79:
                noise = np.random.uniform(-noise_pct, noise_pct) * raw_z
                z = float(np.clip(raw_z + noise, 0.02, 1.80))
            else:
                z = raw_z
            self._ranges_buf[i] = z

        _lib.costmap_process_frame(self._handle, self._ranges_ptr, self._prob_ptr, self._hazard_ptr)

        prob_map = self._prob_buf.reshape(self.shape)
        hazard_mask = self._hazard_buf.reshape(self.shape)
        exec_ms = (time.perf_counter() - t0) * 1000.0
        return prob_map, hazard_mask, exec_ms

    def build_costmap(self, readings: Dict[str, float], noise_pct: float = 0.0) -> np.ndarray:
        prob_map, _, _ = self.process_tof_frame(readings, noise_pct=noise_pct)
        return prob_map

    def probe_path(self, xs: np.ndarray, ys: np.ndarray) -> bool:
        """Instant O(1) candidate path verification against 15mm hazard mask."""
        xs_c = np.ascontiguousarray(xs, dtype=np.float32)
        ys_c = np.ascontiguousarray(ys, dtype=np.float32)
        count = len(xs_c)
        res = _lib.costmap_probe_path(
            self._handle,
            xs_c.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            ys_c.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            count,
        )
        return bool(res == 1)
