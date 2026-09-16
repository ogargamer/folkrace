"""
================================================================================
  PYTHON CTYPES WRAPPER FOR C++ SMART HYBRID COSTMAP (FRONT-ONLY TOF RIG)
  - 16 Front ToF Sensors (8 Left + 8 Right cross-pointing)
  - 64 Cross-beam circle-circle triangulation pairs
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
DLL_PATH = os.path.join(CURR_DIR, "smart_hybrid_costmap_front.dll")

if not os.path.exists(DLL_PATH):
    raise FileNotFoundError(f"Cannot find compiled DLL at {DLL_PATH}. Run compile_dll.py first.")

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

if hasattr(_lib, "costmap_scout_gaps"):
    _lib.costmap_scout_gaps.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),  # ranges_16
        ctypes.c_float, ctypes.c_float,  # max_lookahead_m, steer_current
        ctypes.c_float, ctypes.c_float,  # w_gap_distance, w_gap_width
        ctypes.c_float, ctypes.c_float,  # w_align_steer, w_align_straight
        ctypes.c_float,                  # w_side_clearance
        ctypes.c_float, ctypes.c_float,  # gap_dist_exp_k, gap_progress_margin_m
        ctypes.c_float, ctypes.c_float,  # alt_min_separation_m, alt_aperture_bias
        ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), # out_targets_x, y, ap
        ctypes.c_int,                    # max_targets
    ]
    _lib.costmap_scout_gaps.restype = ctypes.c_int

# Standard sensor names order matching C++ array and only_front_tofs.proto
SENSOR_NAMES = [
    # Front-Left (0..7)
    "tof_front_left_4.1deg",
    "tof_front_left_15.4deg",
    "tof_front_left_26.6deg",
    "tof_front_left_37.9deg",
    "tof_front_left_49.1deg",
    "tof_front_left_60.4deg",
    "tof_front_left_71.6deg",
    "tof_front_left_82.9deg",
    # Front-Right (8..15)
    "tof_front_right_4.1deg",
    "tof_front_right_15.4deg",
    "tof_front_right_26.6deg",
    "tof_front_right_37.9deg",
    "tof_front_right_49.1deg",
    "tof_front_right_60.4deg",
    "tof_front_right_71.6deg",
    "tof_front_right_82.9deg",
]


class CppSmartHybridCostmap:
    """High-performance C++ Smart Hybrid Costmap for 16-channel Front-Only ToF rig."""

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

        # Coordinate meshgrid matching C++ index order (row-major: Y is rows, X is cols)
        xs = np.linspace(self.x_min, self.x_max, self.width, dtype=np.float32)
        ys = np.linspace(self.y_min, self.y_max, self.height, dtype=np.float32)
        self.X, self.Y = np.meshgrid(xs, ys)

        # Preallocated C-contiguous NumPy output buffers
        self.prob_map = np.zeros((self.height, self.width), dtype=np.float32)
        self.hazard_mask = np.zeros((self.height, self.width), dtype=np.uint8)
        self._lib = _lib
        self.raw_ranges_c = (ctypes.c_float * 16)()

        # Preallocated C-types outputs for fast C++ gap scouting
        self._out_act_x = ctypes.c_float(0.0)
        self._out_act_y = ctypes.c_float(0.0)
        self._out_targets_x = (ctypes.c_float * 4)()
        self._out_targets_y = (ctypes.c_float * 4)()
        self._out_targets_ap = (ctypes.c_float * 4)()

    def __del__(self):
        if hasattr(self, "_handle") and self._handle:
            _lib.costmap_destroy(self._handle)
            self._handle = None

    def clear(self) -> None:
        """Clears probability and hazard output buffers."""
        self.prob_map.fill(0.0)
        self.hazard_mask.fill(0)

    def update(self, readings: any, current_steer_rad: float = 0.0) -> Tuple[np.ndarray, np.ndarray, float]:
        """Convenience method accepting either list/array of 16 ranges or dict."""
        if isinstance(readings, (list, tuple, np.ndarray)):
            r_dict = {name: float(readings[i]) for i, name in enumerate(SENSOR_NAMES) if i < len(readings)}
            return self.process_tof_frame(r_dict)
        elif isinstance(readings, dict):
            return self.process_tof_frame(readings)
        return self.prob_map, self.hazard_mask, 0.0

    @staticmethod
    def _extract_range(readings: Dict[str, float], name: str, idx: int) -> float:
        if name in readings:
            return float(readings[name])
        alt = name.replace("15.4", "15.3").replace("37.9", "37.8").replace("60.4", "60.3")
        if alt in readings:
            return float(readings[alt])
        alt2 = name.replace("15.3", "15.4").replace("37.8", "37.9").replace("60.3", "60.4")
        if alt2 in readings:
            return float(readings[alt2])
        # Legacy 4-quadrant fallback
        legacy_fl = ["tof_fl_0", "tof_fl_1", "tof_fl_2", "tof_fl_3"]
        legacy_fr = ["tof_fr_0", "tof_fr_1", "tof_fr_2", "tof_fr_3"]
        if idx < 8:
            leg_name = legacy_fl[idx // 2]
            if leg_name in readings:
                return float(readings[leg_name])
        else:
            leg_name = legacy_fr[(idx - 8) // 2]
            if leg_name in readings:
                return float(readings[leg_name])
        return 1.80

    def process_tof_frame(
        self, readings: Dict[str, float], noise_pct: float = 0.0
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """Drop-in 50/60 Hz Webots controller execution method returning (prob_map, hazard_mask, exec_ms)."""
        import time
        t0 = time.perf_counter()

        for i, name in enumerate(SENSOR_NAMES):
            raw_z = self._extract_range(readings, name, i)
            if noise_pct > 0.0 and raw_z < 1.799:
                noise = np.random.uniform(-noise_pct, noise_pct) * raw_z
                z = float(np.clip(raw_z + noise, 0.001, 1.80))
            else:
                z = raw_z
            self.raw_ranges_c[i] = z

        _lib.costmap_process_frame(
            self._handle,
            self.raw_ranges_c,
            self.prob_map.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            self.hazard_mask.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
        )
        exec_ms = (time.perf_counter() - t0) * 1000.0
        return self.prob_map, self.hazard_mask, exec_ms

    def process_frame(
        self, readings: Dict[str, float]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Runs the complete C++ costmap update."""
        for i, name in enumerate(SENSOR_NAMES):
            self.raw_ranges_c[i] = self._extract_range(readings, name, i)

        _lib.costmap_process_frame(
            self._handle,
            self.raw_ranges_c,
            self.prob_map.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            self.hazard_mask.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
        )

        return self.prob_map, self.hazard_mask

    def build_costmap(
        self, readings: Dict[str, float], noise_pct: float = 0.0
    ) -> np.ndarray:
        """Compatibility wrapper for benchmark evaluation suites."""
        if noise_pct > 0.0:
            noisy = {}
            for k, v in readings.items():
                if v < 1.799:
                    noise = np.random.uniform(-noise_pct, noise_pct) * v
                    noisy[k] = float(np.clip(v + noise, 0.001, 1.80))
                else:
                    noisy[k] = v
            p_map, _ = self.process_frame(noisy)
        else:
            p_map, _ = self.process_frame(readings)

        return p_map

    def probe_path(self, xs: np.ndarray, ys: np.ndarray) -> bool:
        """Instant collision check for an entire trajectory in local frame."""
        xs_c = np.ascontiguousarray(xs, dtype=np.float32)
        ys_c = np.ascontiguousarray(ys, dtype=np.float32)
        n = len(xs_c)

        res = _lib.costmap_probe_path(
            self._handle,
            xs_c.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            ys_c.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            n,
        )
        return bool(res == 1)

    def scout_gaps_native(
        self,
        max_lookahead_m: float = 1.40,
        steer_current: float = 0.0,
        w_gap_distance: float = 3.50,
        w_gap_width: float = 2.00,
        w_align_steer: float = 0.80,
        w_align_straight: float = 0.40,
        w_side_clearance: float = 2.80,
        gap_dist_exp_k: float = 0.60,
        gap_progress_margin_m: float = 0.15,
        alt_min_separation_m: float = 0.25,
        alt_aperture_bias: float = 1.80,
        top_k: int = 2,
    ) -> List[Tuple[float, float, float]]:
        """
        Executes C++ native gap scouting across the active hazard_mask.
        Returns: targets_list [(tx, ty, aperture), ...]
        """
        n_found = _lib.costmap_scout_gaps(
            self._handle,
            self.raw_ranges_c,
            float(max_lookahead_m),
            float(steer_current),
            float(w_gap_distance),
            float(w_gap_width),
            float(w_align_steer),
            float(w_align_straight),
            float(w_side_clearance),
            float(gap_dist_exp_k),
            float(gap_progress_margin_m),
            float(alt_min_separation_m),
            float(alt_aperture_bias),
            self._out_targets_x,
            self._out_targets_y,
            self._out_targets_ap,
            int(top_k),
        )

        targets = []
        for i in range(max(1, min(n_found, top_k))):
            targets.append((
                float(self._out_targets_x[i]),
                float(self._out_targets_y[i]),
                float(self._out_targets_ap[i]),
            ))

        return targets
