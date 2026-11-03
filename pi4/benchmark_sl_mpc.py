#!/usr/bin/env python3
"""
Benchmark Script for Curvilinear SL-MPC & Perception Pipeline on Raspberry Pi 4.
Runs recorded Webots 16-channel ToF sensor datasets through:
  1. C++ Native Smart Hybrid Costmap
  2. ToF Gap Scout
  3. C++ Native Curvilinear SL-MPC (OSQP Quadratic Program)
Measures execution latency with microsecond precision.
"""

import os
import sys
import time
import csv
import numpy as np

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
if CURR_DIR not in sys.path:
    sys.path.insert(0, CURR_DIR)

from cpp_smart_hybrid_costmap import CppSmartHybridCostmap, SENSOR_NAMES
from tof_gap_scout import ToFGapScout
from planner_sl_mpc import CurvilinearSLMPC, CPP_AVAILABLE
from planner_config import (
    GRID_X_MIN,
    GRID_X_MAX,
    GRID_Y_MIN,
    GRID_Y_MAX,
    GRID_RES,
    R_INFLATION_M,
)


def load_dataset(csv_path: str):
    """Loads 16-channel ToF frames from recorded CSV file."""
    frames = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame = {}
            for k in SENSOR_NAMES:
                if k in row:
                    frame[k] = float(row[k])
            if len(frame) == 16:
                frames.append(frame)
    return frames


def run_benchmark():
    print("=" * 70)
    print("  FOLKRACE PI 4: CURVILINEAR SL-MPC & PERCEPTION BENCHMARK")
    print(f"  Platform: {sys.platform} | Python: {sys.version.split()[0]}")
    print(f"  C++ Native SL-MPC Engine Available: {CPP_AVAILABLE}")
    print("=" * 70)

    dataset_path = os.path.join(CURR_DIR, "data", "tof_readings_cube_10cm.csv")
    if not os.path.exists(dataset_path):
        print(f"[ERR] Dataset not found: {dataset_path}")
        return

    frames = load_dataset(dataset_path)
    print(f"[INFO] Loaded {len(frames)} ToF frames from {os.path.basename(dataset_path)}")

    costmap = CppSmartHybridCostmap(
        grid_bounds=(GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX),
        resolution=GRID_RES,
        r_inflation_m=R_INFLATION_M,
    )
    gap_scout = ToFGapScout()
    planner = CurvilinearSLMPC(costmap_engine=costmap)
    planner.costmap = costmap

    v_curr = 2.50       # m/s
    steer_curr = 0.0    # rad
    yaw_rate_curr = 0.0 # rad/s
    target_lookahead = 2.40

    # 1. Warmup phase (50 iterations)
    print("\n[INFO] Running 50 warmup iterations...")
    for i in range(min(50, len(frames))):
        tof = frames[i]
        prob_map, hazard_mask, _ = costmap.process_tof_frame(tof)
        xg, yg, width, theta_g = gap_scout.scout_gap(
            tof_readings=tof,
            hazard_mask=hazard_mask,
            costmap_obj=costmap,
            target_lookahead_m=target_lookahead,
            steer_current=steer_curr,
            prob_map=prob_map,
        )
        plan_res = planner.plan(
            xg=xg,
            yg=yg,
            theta_g=theta_g,
            aperture_width=width,
            v_current=v_curr,
            steer_current=steer_curr,
            yaw_rate_current=yaw_rate_curr,
            extra_targets=getattr(gap_scout, "last_targets", None),
        )

    # 2. Benchmark Measurement across all frames
    print(f"[INFO] Benchmarking {len(frames)} frames...")
    costmap_times = []
    scout_times = []
    sl_mpc_times = []
    total_pipeline_times = []
    osqp_iterations = []
    cmd_speeds = []
    cmd_steers = []

    for i, tof in enumerate(frames):
        t_start_total = time.perf_counter_ns()

        t0 = time.perf_counter_ns()
        prob_map, hazard_mask, _ = costmap.process_tof_frame(tof)
        t_costmap_us = (time.perf_counter_ns() - t0) / 1000.0

        t0 = time.perf_counter_ns()
        xg, yg, width, theta_g = gap_scout.scout_gap(
            tof_readings=tof,
            hazard_mask=hazard_mask,
            costmap_obj=costmap,
            target_lookahead_m=target_lookahead,
            steer_current=steer_curr,
            prob_map=prob_map,
        )
        t_scout_us = (time.perf_counter_ns() - t0) / 1000.0

        t0 = time.perf_counter_ns()
        plan_res = planner.plan(
            xg=xg,
            yg=yg,
            theta_g=theta_g,
            aperture_width=width,
            v_current=v_curr,
            steer_current=steer_curr,
            yaw_rate_current=yaw_rate_curr,
            extra_targets=getattr(gap_scout, "last_targets", None),
        )
        t_sl_mpc_us = (time.perf_counter_ns() - t0) / 1000.0

        t_total_us = (time.perf_counter_ns() - t_start_total) / 1000.0

        costmap_times.append(t_costmap_us)
        scout_times.append(t_scout_us)
        sl_mpc_times.append(t_sl_mpc_us)
        total_pipeline_times.append(t_total_us)

        osqp_iterations.append(plan_res.get("osqp_iters", 0))
        cmd_speeds.append(plan_res.get("cmd_speed_mps", 0.0))
        cmd_steers.append(plan_res.get("cmd_steer_rad", 0.0))

        cmd_steer = plan_res.get("cmd_steer_rad", 0.0)
        steer_curr = steer_curr + 0.3 * (cmd_steer - steer_curr)
        v_curr = max(0.5, min(5.0, plan_res.get("cmd_speed_mps", v_curr)))

    def stats_ms(arr_us):
        arr_ms = np.array(arr_us) / 1000.0
        return {
            "mean": np.mean(arr_ms),
            "median": np.median(arr_ms),
            "min": np.min(arr_ms),
            "max": np.max(arr_ms),
            "p95": np.percentile(arr_ms, 95),
            "p99": np.percentile(arr_ms, 99),
            "std": np.std(arr_ms),
        }

    c_stats = stats_ms(costmap_times)
    s_stats = stats_ms(scout_times)
    m_stats = stats_ms(sl_mpc_times)
    t_stats = stats_ms(total_pipeline_times)

    print("\n" + "=" * 70)
    print("  PERFORMANCE BENCHMARK RESULTS (Raspberry Pi 4 Model B)")
    print("=" * 70)
    print(f"{'Component':<32} {'Mean':<8} {'Median':<8} {'P95':<8} {'P99':<8} {'Max':<8}")
    print("-" * 70)
    print(f"{'1. C++ Costmap Update':<32} {c_stats['mean']:>6.3f}ms {c_stats['median']:>6.3f}ms {c_stats['p95']:>6.3f}ms {c_stats['p99']:>6.3f}ms {c_stats['max']:>6.3f}ms")
    print(f"{'2. ToF Gap Scout':<32} {s_stats['mean']:>6.3f}ms {s_stats['median']:>6.3f}ms {s_stats['p95']:>6.3f}ms {s_stats['p99']:>6.3f}ms {s_stats['max']:>6.3f}ms")
    print(f"{'3. C++ Curvilinear SL-MPC (OSQP)':<32} {m_stats['mean']:>6.3f}ms {m_stats['median']:>6.3f}ms {m_stats['p95']:>6.3f}ms {m_stats['p99']:>6.3f}ms {m_stats['max']:>6.3f}ms")
    print("-" * 70)
    print(f"{'TOTAL PLANNING PIPELINE':<32} {t_stats['mean']:>6.3f}ms {t_stats['median']:>6.3f}ms {t_stats['p95']:>6.3f}ms {t_stats['p99']:>6.3f}ms {t_stats['max']:>6.3f}ms")
    print("=" * 70)

    avg_hz = 1000.0 / t_stats['mean']
    p99_hz = 1000.0 / t_stats['p99']
    min_hz = 1000.0 / t_stats['max']
    sl_mpc_only_hz = 1000.0 / m_stats['mean']

    print(f"\n[RATES & FREQUENCIES]")
    print(f"  * Curvilinear SL-MPC Alone:     {sl_mpc_only_hz:>6.1f} Hz (mean {m_stats['mean']:.3f} ms)")
    print(f"  * Full End-to-End Pipeline:    {avg_hz:>6.1f} Hz (mean {t_stats['mean']:.3f} ms)")
    print(f"  * 99th Percentile Guaranteed:  {p99_hz:>6.1f} Hz ({t_stats['p99']:.3f} ms)")
    print(f"  * Worst-Case Single Frame:     {min_hz:>6.1f} Hz ({t_stats['max']:.3f} ms)")

    print(f"\n[SOLVER METRICS]")
    print(f"  * OSQP Mean Iterations:         {np.mean(osqp_iterations):.1f}")
    print(f"  * OSQP Max Iterations:          {np.max(osqp_iterations)}")
    print(f"  * Mean Commanded Speed:         {np.mean(cmd_speeds):.2f} m/s")
    print(f"  * Steering Output Range:        [{np.min(cmd_steers):.3f}, {np.max(cmd_steers):.3f}] rad")

    print("\n" + "=" * 70)
    print("  BENCHMARK COMPLETED SUCCESSFULLY")
    print("=" * 70)


if __name__ == "__main__":
    run_benchmark()

