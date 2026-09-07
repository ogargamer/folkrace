"""
================================================================================
  FACT-CHECKING & BENCHMARK SUITE: ADVANCED TOF COSTMAP ARCHITECTURES
  
  Compares:
    1. Baseline (SimpleToFCostmap - Cached Polar)
    2. Two-Pass Arc Carving (ArcCarvingToFCostmap)
    3. Bayesian Angular Decay (BayesianAngularDecayToFCostmap)
    4. Cross-Beam Arc Triangulation (TriangulationToFCostmap)
    
  Evaluates:
    - Speed: Mean Exec Time (ms), Max Exec Time (ms), Throughput (Hz)
    - Accuracy: Containment Rate (%), Surface Error (Mean, Median in cm), OK Rate (%)
    - Angular Smearing & Overextension: Close / Med / Far Frontal Overextension (cm)
    - Crossover Gap Recovery: Recovered corridor width between cross-beams (cm)
================================================================================
"""

import csv
import math
import os
import sys
import time
from typing import Any, Dict, List, Tuple
import numpy as np

curr_dir = os.path.dirname(os.path.abspath(__file__))
if curr_dir not in sys.path:
    sys.path.insert(0, curr_dir)

from advanced_tof_costmaps import (
    ArcCarvingToFCostmap,
    BayesianAngularDecayToFCostmap,
    TriangulationToFCostmap,
    SENSOR_RIG,
)
from tof_costmap_validator import (
    compute_surface_error,
    check_containment,
)

DATASETS_DIR = os.path.join(curr_dir, "data", "datasets")
REPORTS_DIR = os.path.join(curr_dir, "data", "reports")
os.makedirs(REPORTS_DIR, exist_ok=True)


class FastSimpleToFCostmap:
    """Baseline SimpleToFCostmap with precomputed polar coordinates (same math, no redundant arctan2)."""

    def __init__(
        self,
        grid_bounds: Tuple[float, float, float, float] = (-1.0, 1.0, 0.0, 2.0),
        resolution: float = 0.025,
        l_occ: float = 4.0,
        l_free: float = -1.5,
        sigma_front: float = 0.5,
        sigma_back: float = 1.5,
    ):
        self.x_min, self.x_max, self.y_min, self.y_max = grid_bounds
        self.res = resolution
        self.l_occ = l_occ
        self.l_free = l_free
        self.sigma_front = sigma_front
        self.sigma_back = sigma_back

        xs = np.arange(self.x_min, self.x_max + self.res / 2.0, self.res)
        ys = np.arange(self.y_min, self.y_max + self.res / 2.0, self.res)
        self.X, self.Y = np.meshgrid(xs, ys)
        self.shape = self.X.shape

        # Precompute static polar grids per sensor
        self.sensor_grids = {}
        for s_name, s_x0, s_y0, s_heading in SENSOR_RIG:
            dx = self.X - s_x0
            dy = self.Y - s_y0
            r = np.hypot(dx, dy)
            phi = np.arctan2(dy, dx)
            delta_theta = np.abs((phi - s_heading + np.pi) % (2 * np.pi) - np.pi)
            cone = (delta_theta <= (0.19634954 / 2.0)) & (r >= 0.02) & (r <= 1.80)
            self.sensor_grids[s_name] = (r, cone)

    def build_costmap(self, readings: Dict[str, float], noise_pct: float = 0.0) -> np.ndarray:
        log_odds = np.zeros(self.shape, dtype=np.float32)

        for s_name, _, _, _ in SENSOR_RIG:
            raw_z = readings.get(s_name, 1.80)
            if noise_pct > 0.0 and raw_z < 1.799:
                noise = np.random.uniform(-noise_pct, noise_pct) * raw_z
                z = float(np.clip(raw_z + noise, 0.02, 1.80))
            else:
                z = raw_z

            r, cone = self.sensor_grids[s_name]
            sigma_r = max(0.05 * z, self.res)

            if z >= 1.799:
                log_odds[cone] += self.l_free
            else:
                free_mask = cone & (r < (z - self.sigma_front * sigma_r))
                log_odds[free_mask] += self.l_free

                hit_mask = cone & (r >= (z - self.sigma_front * sigma_r)) & (r <= (z + self.sigma_back * sigma_r))
                log_odds[hit_mask] += self.l_occ

        prob_map = 1.0 / (1.0 + np.exp(-log_odds))
        return prob_map


def evaluate_engine_on_dataset(
    engine: Any,
    csv_file: str,
    shape_type: str,
    size_m: float,
    grid_res: float,
    noise_pct: float = 0.05,
    name: str = "Engine",
) -> Dict[str, Any]:
    half_w = size_m / 2.0
    with open(csv_file, mode="r") as f:
        records = list(csv.DictReader(f))

    sensor_keys = [s[0] for s in SENSOR_RIG]
    exec_times_ms = []
    errors = []
    contained_count = 0
    total_detected = 0

    close_overexts = []
    med_overexts = []
    far_overexts = []

    for row in records:
        x_gt = float(row["x_m"])
        y_gt = float(row["y_m"])

        readings = {k: float(row[k]) for k in sensor_keys if k in row}
        active_sensors = [k for k, v in readings.items() if v < 1.79]

        if not active_sensors:
            continue

        total_detected += 1

        t0 = time.perf_counter()
        if hasattr(engine, "process_tof_frame"):
            prob_map, _, _ = engine.process_tof_frame(readings, noise_pct=noise_pct)
        else:
            prob_map = engine.build_costmap(readings, noise_pct=noise_pct)
        exec_times_ms.append((time.perf_counter() - t0) * 1000.0)

        hit_cells = prob_map > 0.55
        if not np.any(hit_cells):
            continue

        weights = prob_map[hit_cells]
        x_est_surf = float(np.sum(engine.X[hit_cells] * weights) / np.sum(weights))
        y_est_surf = float(np.sum(engine.Y[hit_cells] * weights) / np.sum(weights))

        surface_error = compute_surface_error(
            x_est_surf, y_est_surf, x_gt, y_gt, shape_type, half_w
        )
        errors.append(surface_error)

        r_hit_cells = np.hypot(engine.X[hit_cells], engine.Y[hit_cells])
        min_r_occ = float(np.min(r_hit_cells))
        r_gt_center = math.hypot(x_gt, y_gt)
        r_true_front = max(0.0, r_gt_center - half_w)
        front_overextension = max(0.0, r_true_front - min_r_occ)

        if r_gt_center < 0.6:
            close_overexts.append(front_overextension)
        elif r_gt_center < 1.2:
            med_overexts.append(front_overextension)
        else:
            far_overexts.append(front_overextension)

        if check_containment(prob_map, engine.X, engine.Y, x_gt, y_gt, shape_type, half_w, engine.res):
            contained_count += 1

    return {
        "engine_name": name,
        "total_tested": len(records),
        "total_detected": total_detected,
        "mean_exec_ms": float(np.mean(exec_times_ms)) if exec_times_ms else 0.0,
        "max_exec_ms": float(np.max(exec_times_ms)) if exec_times_ms else 0.0,
        "p99_exec_ms": float(np.percentile(exec_times_ms, 99)) if exec_times_ms else 0.0,
        "throughput_hz": (1000.0 / np.mean(exec_times_ms)) if exec_times_ms else 0.0,
        "containment_pct": (contained_count / total_detected * 100.0) if total_detected else 0.0,
        "mean_error_cm": float(np.mean(errors) * 100.0) if errors else 0.0,
        "median_error_cm": float(np.median(errors) * 100.0) if errors else 0.0,
        "close_overext_mean_cm": float(np.mean(close_overexts) * 100.0) if close_overexts else 0.0,
        "close_overext_max_cm": float(np.max(close_overexts) * 100.0) if close_overexts else 0.0,
        "med_overext_mean_cm": float(np.mean(med_overexts) * 100.0) if med_overexts else 0.0,
        "far_overext_mean_cm": float(np.mean(far_overexts) * 100.0) if far_overexts else 0.0,
        "ok_rate_pct": (sum(1 for e in errors if e <= max(0.05, grid_res)) / len(errors) * 100.0) if errors else 0.0,
    }


def evaluate_crossover_gap_recovery():
    """
    Dedicated Crossover Experiment:
    Measures how much of an open corridor is choked by angular smearing vs recovered by each engine.
    """
    print("\n" + "=" * 80, flush=True)
    print("  EXPERIMENT: CROSS-BEAM CORRIDOR CLEARANCE & GAP RECOVERY", flush=True)
    print("=" * 80, flush=True)

    readings = {s[0]: 1.80 for s in SENSOR_RIG}
    readings["tof_fl_3"] = 0.72
    readings["tof_fr_3"] = 0.72

    grid_bounds = (-1.0, 1.0, 0.0, 2.0)
    res = 0.02

    e_base = FastSimpleToFCostmap(grid_bounds=grid_bounds, resolution=res)
    e_carv = ArcCarvingToFCostmap(grid_bounds=grid_bounds, resolution=res, r_inflation_m=0.0)
    e_bayes = BayesianAngularDecayToFCostmap(grid_bounds=grid_bounds, resolution=res, r_inflation_m=0.0)
    e_tri = TriangulationToFCostmap(grid_bounds=grid_bounds, resolution=res, r_inflation_m=0.0)

    m_base = e_base.build_costmap(readings)
    m_carv = e_carv.build_costmap(readings)
    m_bayes = e_bayes.build_costmap(readings)
    m_tri = e_tri.build_costmap(readings)

    y_idx = int(round((0.70 - grid_bounds[2]) / res))
    x_mid = int(round((0.0 - grid_bounds[0]) / res))

    def measure_clear_corridor(prob_grid):
        row = prob_grid[y_idx, :]
        occ = row > 0.55
        left_occ = np.where(occ[:x_mid])[0]
        right_occ = np.where(occ[x_mid:])[0]

        x_left_edge = (left_occ[-1] * res + grid_bounds[0]) if len(left_occ) > 0 else grid_bounds[0]
        x_right_edge = ((right_occ[0] + x_mid) * res + grid_bounds[0]) if len(right_occ) > 0 else grid_bounds[1]
        return (x_right_edge - x_left_edge) * 100.0

    gap_base = measure_clear_corridor(m_base)
    gap_carv = measure_clear_corridor(m_carv)
    gap_bayes = measure_clear_corridor(m_bayes)
    gap_tri = measure_clear_corridor(m_tri)

    print(f"Baseline Open Corridor Width:          {gap_base:.1f} cm (Severe smearing chokes corridor)", flush=True)
    print(f"Two-Pass Arc Carving Corridor Width:    {gap_carv:.1f} cm (Recovered: +{gap_carv - gap_base:.1f} cm)", flush=True)
    print(f"Bayesian Angular Decay Corridor Width:  {gap_bayes:.1f} cm (Recovered: +{gap_bayes - gap_base:.1f} cm)", flush=True)
    print(f"Arc Triangulation Corridor Width:       {gap_tri:.1f} cm (Recovered: +{gap_tri - gap_base:.1f} cm)", flush=True)
    print("=" * 80 + "\n", flush=True)


def run_full_comparison_suite():
    print("\n" + "=" * 95, flush=True)
    print("  COMPREHENSIVE FACT-CHECKING SUITE: 4 TOF COSTMAP ENGINES ON 8 GROUND-TRUTH DATASETS", flush=True)
    print("=" * 95, flush=True)

    test_configs = [
        ("pillar", 0.05, "5 cm Pillar (Dia 5 cm, Grid 2.5 cm)"),
        ("cube",   0.05, "5 cm Cube (Width 5 cm, Grid 2.5 cm)"),
        ("pillar", 0.10, "10 cm Pillar (Dia 10 cm, Grid 5.0 cm)"),
        ("cube",   0.10, "10 cm Cube (Width 10 cm, Grid 5.0 cm)"),
        ("pillar", 0.15, "15 cm Pillar (Dia 15 cm, Grid 7.5 cm)"),
        ("cube",   0.15, "15 cm Cube (Width 15 cm, Grid 7.5 cm)"),
        ("pillar", 0.20, "20 cm Pillar (Dia 20 cm, Grid 10.0 cm)"),
        ("cube",   0.20, "20 cm Cube (Width 20 cm, Grid 10.0 cm)"),
    ]

    all_results = []

    for shape, size, label in test_configs:
        grid_res = size / 2.0
        size_cm = int(size * 100)
        csv_name = f"tof_readings_{shape}_{size_cm}cm.csv"
        if shape == "pillar" and size == 0.05:
            legacy_file = os.path.join(DATASETS_DIR, "tof_16ch_grid_readings.csv")
            csv_path = legacy_file if os.path.exists(legacy_file) else os.path.join(DATASETS_DIR, csv_name)
        else:
            csv_path = os.path.join(DATASETS_DIR, csv_name)

        if not os.path.exists(csv_path):
            print(f"Skipping {label}: {csv_path} not found.", flush=True)
            continue

        print(f"\n--- Testing Dataset: {label} ---", flush=True)

        grid_bounds = (-1.0, 1.0, 0.0, 2.0)
        e_base = FastSimpleToFCostmap(grid_bounds=grid_bounds, resolution=grid_res)
        e_carv = ArcCarvingToFCostmap(grid_bounds=grid_bounds, resolution=grid_res)
        e_bayes = BayesianAngularDecayToFCostmap(grid_bounds=grid_bounds, resolution=grid_res)
        e_tri = TriangulationToFCostmap(grid_bounds=grid_bounds, resolution=grid_res)

        res_base = evaluate_engine_on_dataset(e_base, csv_path, shape, size, grid_res, name="1. Baseline SimpleToF")
        res_carv = evaluate_engine_on_dataset(e_carv, csv_path, shape, size, grid_res, name="2. Two-Pass Arc Carving")
        res_bayes = evaluate_engine_on_dataset(e_bayes, csv_path, shape, size, grid_res, name="3. Bayesian Angular Decay")
        res_tri = evaluate_engine_on_dataset(e_tri, csv_path, shape, size, grid_res, name="4. Cross-Beam Triangulation")

        for r in (res_base, res_carv, res_bayes, res_tri):
            r["dataset_label"] = label
            r["shape"] = shape
            r["size_m"] = size
            all_results.append(r)
            print(
                f"  {r['engine_name']:<30} | "
                f"Time: {r['mean_exec_ms']:.3f} ms ({r['throughput_hz']:>5.0f} Hz) | "
                f"Contain: {r['containment_pct']:>5.1f}% | "
                f"Err: {r['mean_error_cm']:>4.2f} cm | "
                f"Close Overext: {r['close_overext_mean_cm']:>4.2f} cm",
                flush=True,
            )

    report_csv = os.path.join(REPORTS_DIR, "all_architectures_benchmark_report.csv")
    fieldnames = list(all_results[0].keys())
    with open(report_csv, mode="w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_results)

    print(f"\n[Summary report written to {report_csv}]", flush=True)

    evaluate_crossover_gap_recovery()

    return all_results


if __name__ == "__main__":
    run_full_comparison_suite()
