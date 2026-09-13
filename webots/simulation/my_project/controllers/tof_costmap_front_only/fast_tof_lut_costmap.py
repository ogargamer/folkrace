"""
================================================================================
  FAST PYTHON LUT COSTMAP ENGINE (FRONT-ONLY TOF RIG)
  Uses precomputed polar beam lookup table (.npz) for sub-0.10 ms Python execution.
================================================================================
"""

import os
from typing import Dict, List, Optional, Tuple
import numpy as np

from generate_tof_lut import SENSOR_RIG, ToFSensor, generate_lut


class FastToFLUTCostmap:
    """Precomputed Polar Beam LUT Costmap Engine for 16-channel Front-Only ToF rig."""

    def __init__(
        self,
        grid_bounds: Tuple[float, float, float, float] = (-1.25, 1.25, 0.0, 2.0),
        resolution: float = 0.020,
        range_step: float = 0.010,
        l_occ: float = 4.0,
        l_free: float = -1.5,
        lut_npz_path: Optional[str] = None,
    ):
        self.x_min, self.x_max, self.y_min, self.y_max = grid_bounds
        self.res = float(resolution)
        self.range_step = float(range_step)
        self.l_occ = np.float32(l_occ)
        self.l_free = np.float32(l_free)
        self.max_range = 1.80
        self.num_buckets = int(round(self.max_range / self.range_step)) + 1

        self.xs = np.arange(self.x_min, self.x_max + self.res / 2.0, self.res, dtype=np.float32)
        self.ys = np.arange(self.y_min, self.y_max + self.res / 2.0, self.res, dtype=np.float32)
        self.X, self.Y = np.meshgrid(self.xs, self.ys)
        self.shape = self.X.shape
        self.num_cells = self.shape[0] * self.shape[1]

        # Sensor lookup tables: list of arrays per sensor indexed by bucket
        self.lut_free: Dict[str, List[np.ndarray]] = {}
        self.lut_hit: Dict[str, List[np.ndarray]] = {}
        self.lut_hit_weights: Dict[str, List[np.ndarray]] = {}

        if lut_npz_path and os.path.exists(lut_npz_path):
            self._load_lut_from_npz(lut_npz_path)
        else:
            self._load_or_generate_lut(lut_npz_path)

    def _load_lut_from_npz(self, npz_path: str):
        data = np.load(npz_path)
        for s_idx, s in enumerate(SENSOR_RIG):
            self.lut_free[s.name] = [data[f"free_{s_idx}_{b}"] for b in range(self.num_buckets)]
            self.lut_hit[s.name] = [data[f"hit_{s_idx}_{b}"] for b in range(self.num_buckets)]
            self.lut_hit_weights[s.name] = [data[f"weights_{s_idx}_{b}"] for b in range(self.num_buckets)]

    def _load_or_generate_lut(self, npz_path: Optional[str]):
        curr_dir = os.path.dirname(os.path.abspath(__file__))
        default_path = os.path.join(curr_dir, "front_tofs_lut_20mm.npz")
        target_path = npz_path or default_path

        if os.path.exists(target_path):
            self._load_lut_from_npz(target_path)
        else:
            raw_data = generate_lut(
                grid_bounds=(self.x_min, self.x_max, self.y_min, self.y_max),
                resolution=self.res,
                range_step=self.range_step,
                output_npz=target_path,
            )
            for s_idx, s in enumerate(SENSOR_RIG):
                self.lut_free[s.name] = [raw_data[f"free_{s_idx}_{b}"] for b in range(self.num_buckets)]
                self.lut_hit[s.name] = [raw_data[f"hit_{s_idx}_{b}"] for b in range(self.num_buckets)]
                self.lut_hit_weights[s.name] = [raw_data[f"weights_{s_idx}_{b}"] for b in range(self.num_buckets)]

    def build_log_odds(self, readings: Dict[str, float]) -> np.ndarray:
        """Returns flat 1D log-odds buffer updated with 16 ToF readings."""
        log_odds = np.zeros(self.num_cells, dtype=np.float32)

        for s in SENSOR_RIG:
            z = readings.get(s.name, self.max_range)
            b = min(self.num_buckets - 1, max(0, int(round(z / self.range_step))))

            # 1. Carve free space
            free_idx = self.lut_free[s.name][b]
            if len(free_idx) > 0:
                log_odds[free_idx] += self.l_free

            # 2. Accumulate hit evidence with angular falloff
            if z < (self.max_range - 1e-3):
                hit_idx = self.lut_hit[s.name][b]
                weights = self.lut_hit_weights[s.name][b]
                if len(hit_idx) > 0:
                    log_odds[hit_idx] += weights

        return log_odds

    def build_costmap(self, readings: Dict[str, float], noise_pct: float = 0.0) -> np.ndarray:
        """Computes 2D posterior probability occupancy map [0.0, 1.0]."""
        if noise_pct > 0.0:
            noisy = {}
            for k, v in readings.items():
                if v < (self.max_range - 1e-3):
                    noise = np.random.uniform(-noise_pct, noise_pct) * v
                    noisy[k] = float(np.clip(v + noise, 0.001, self.max_range))
                else:
                    noisy[k] = v
            lo = self.build_log_odds(noisy)
        else:
            lo = self.build_log_odds(readings)

        # Sigmoid probability
        lo_clamped = np.clip(lo, -10.0, 10.0)
        prob = 1.0 / (1.0 + np.exp(-lo_clamped))
        return prob.reshape(self.shape)
