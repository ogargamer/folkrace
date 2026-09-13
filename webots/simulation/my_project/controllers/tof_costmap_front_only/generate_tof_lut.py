"""
================================================================================
  LOOKUP TABLE (LUT) GENERATOR FOR 16-CHANNEL FRONT-ONLY TOF RIG
  Layout from: repo-restructure-tiered-folders/webots/simulation/my_project/protos/only_front_tofs.proto
  
  Precomputes:
  - Range bucket lookups (free_indices, hit_indices, hit_weights)
  - Beam polar cones & Gaussian angular decay factors
  - Exports compressed .npz archive for sub-0.10 ms instant loading
================================================================================
"""

import json
import os
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Tuple
import numpy as np

# Sensor definition
@dataclass(frozen=True)
class ToFSensor:
    name: str
    x0: float
    y0: float
    heading_rad: float
    fov_rad: float = 0.19634954  # 11.25 degrees
    min_range: float = 0.001
    max_range: float = 1.80

# 16 Front Sensors verbatim from only_front_tofs.proto
# Robot Body Frame: +Y is Forward, +X is Right, -X is Left, +Z is Up
SENSOR_RIG: List[ToFSensor] = [
    # Front-Left Array (X = -0.070, Y = +0.095, cross-pointing Right)
    ToFSensor("tof_front_left_4.1deg",  -0.070, 0.095, 1.4987),
    ToFSensor("tof_front_left_15.4deg", -0.070, 0.095, 1.3024),
    ToFSensor("tof_front_left_26.6deg", -0.070, 0.095, 1.1060),
    ToFSensor("tof_front_left_37.9deg", -0.070, 0.095, 0.9097),
    ToFSensor("tof_front_left_49.1deg", -0.070, 0.095, 0.7134),
    ToFSensor("tof_front_left_60.4deg", -0.070, 0.095, 0.5170),
    ToFSensor("tof_front_left_71.6deg", -0.070, 0.095, 0.3207),
    ToFSensor("tof_front_left_82.9deg", -0.070, 0.095, 0.1243),

    # Front-Right Array (X = +0.070, Y = +0.095, cross-pointing Left)
    ToFSensor("tof_front_right_4.1deg",   0.070, 0.095, 1.6428),
    ToFSensor("tof_front_right_15.4deg",  0.070, 0.095, 1.8391),
    ToFSensor("tof_front_right_26.6deg",  0.070, 0.095, 2.0356),
    ToFSensor("tof_front_right_37.9deg",  0.070, 0.095, 2.2318),
    ToFSensor("tof_front_right_49.1deg",  0.070, 0.095, 2.4282),
    ToFSensor("tof_front_right_60.4deg",  0.070, 0.095, 2.6245),
    ToFSensor("tof_front_right_71.6deg",  0.070, 0.095, 2.8208),
    ToFSensor("tof_front_right_82.9deg",  0.070, 0.095, 3.0172),
]


def generate_lut(
    grid_bounds: Tuple[float, float, float, float] = (-1.25, 1.25, 0.0, 2.0),
    resolution: float = 0.020,     # 20 mm
    range_step: float = 0.010,     # 10 mm range discretization
    sigma_front: float = 0.5,
    sigma_back: float = 1.5,
    output_npz: str = "",
) -> Dict[str, any]:
    """Generates precomputed beam indices and weights for all 16 front ToFs."""
    x_min, x_max, y_min, y_max = grid_bounds
    xs = np.arange(x_min, x_max + resolution / 2.0, resolution, dtype=np.float32)
    ys = np.arange(y_min, y_max + resolution / 2.0, resolution, dtype=np.float32)
    X, Y = np.meshgrid(xs, ys)
    shape = X.shape
    num_cells = shape[0] * shape[1]

    max_range = 1.80
    num_buckets = int(round(max_range / range_step)) + 1
    sigma_theta = (0.19634954 / 2.0) / 2.145966  # Gaussian half-angle decay

    print(f"[LUT BUILDER] Grid: {shape[1]}x{shape[0]} ({num_cells} cells), Res: {resolution*1000:.1f} mm, Buckets: {num_buckets}")

    npz_data = {
        "x_min": np.float32(x_min),
        "x_max": np.float32(x_max),
        "y_min": np.float32(y_min),
        "y_max": np.float32(y_max),
        "resolution": np.float32(resolution),
        "range_step": np.float32(range_step),
        "width": np.int32(shape[1]),
        "height": np.int32(shape[0]),
        "num_buckets": np.int32(num_buckets),
    }

    t0 = time.perf_counter()

    for s_idx, s in enumerate(SENSOR_RIG):
        dx = X - s.x0
        dy = Y - s.y0
        r = np.hypot(dx, dy)
        phi = np.arctan2(dy, dx)
        delta_theta = np.abs((phi - s.heading_rad + np.pi) % (2.0 * np.pi) - np.pi)
        cone = (delta_theta <= (s.fov_rad / 2.0)) & (r >= s.min_range) & (r <= s.max_range)

        angular_factor = np.exp(-0.5 * (delta_theta / sigma_theta) ** 2)
        log_odds_weights_grid = (1.5 + (5.0 - 1.5) * angular_factor).astype(np.float32)

        for b in range(num_buckets):
            z = b * range_step
            sigma_r = max(0.05 * z, resolution)

            if z >= (max_range - 1e-3):
                free_idx = np.flatnonzero(cone).astype(np.int32)
                hit_idx = np.array([], dtype=np.int32)
                hit_weights = np.array([], dtype=np.float32)
            else:
                free_mask = cone & (r < (z - sigma_front * sigma_r))
                hit_mask = cone & (r >= (z - sigma_front * sigma_r)) & (r <= (z + sigma_back * sigma_r))
                free_idx = np.flatnonzero(free_mask).astype(np.int32)
                hit_idx = np.flatnonzero(hit_mask).astype(np.int32)
                hit_weights = log_odds_weights_grid.flat[hit_idx]

            npz_data[f"free_{s_idx}_{b}"] = free_idx
            npz_data[f"hit_{s_idx}_{b}"] = hit_idx
            npz_data[f"weights_{s_idx}_{b}"] = hit_weights

    elapsed = time.perf_counter() - t0
    print(f"[LUT BUILDER] Completed in {elapsed:.3f} s ({len(SENSOR_RIG) * num_buckets} bucket entries)")

    if output_npz:
        os.makedirs(os.path.dirname(os.path.abspath(output_npz)), exist_ok=True)
        np.savez_compressed(output_npz, **npz_data)
        file_size_kb = os.path.getsize(output_npz) / 1024.0
        print(f"[LUT BUILDER] Saved compressed LUT archive: {output_npz} ({file_size_kb:.1f} KB)")

    return npz_data


if __name__ == "__main__":
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    out_file = os.path.join(curr_dir, "front_tofs_lut_20mm.npz")
    generate_lut(
        grid_bounds=(-1.25, 1.25, 0.0, 2.0),
        resolution=0.020,
        range_step=0.010,
        output_npz=out_file,
    )
