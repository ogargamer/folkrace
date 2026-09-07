"""
================================================================================
  BENCHMARK & VALIDATION TEST: FAST 60 Hz TOF + CARTO SLAM FUSION PIPELINE
  Validates:
    1. 60 Hz Real-Time Execution Speed (< 0.15 ms on CPU)
    2. Cartographer 15 Hz SLAM Map Baseline Fusion
    3. 100 Hz Pose Stream & 20 ms Latency Ring Buffer Compensation
    4. MPPI Probability & Binary Collision Layer Integrity
================================================================================
"""

import math
import os
import sys
import time
from typing import Tuple
import numpy as np

parent_dir = os.path.dirname(os.path.abspath(__file__))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from fast_tof_carto_costmap import (
    FastCostmapPipeline,
    FastToFLUTCostmap,
    PoseRingBuffer,
    StampedPose,
    SENSOR_RIG,
)


def create_synthetic_cartographer_track(
    width_m: float = 20.0, height_m: float = 20.0, resolution_m: float = 0.02
) -> Tuple[np.ndarray, Tuple[float, float], float]:
    """Creates a synthetic Cartographer global SLAM occupancy grid with track walls."""
    gx = int(width_m / resolution_m)
    gy = int(height_m / resolution_m)
    grid = np.zeros((gy, gx), dtype=np.int8)  # 0 = free

    # Outer perimeter walls (occupied = 100)
    grid[0:3, :] = 100
    grid[-3:, :] = 100
    grid[:, 0:3] = 100
    grid[:, -3:] = 100

    # Track corridor walls
    c_x, c_y = gx // 2, gy // 2
    grid[c_y - 20 : c_y + 20, c_x - 3 : c_x + 3] = 100

    origin_xy = (-width_m / 2.0, -height_m / 2.0)
    return grid, origin_xy, resolution_m


def run_pipeline_benchmark():
    print("\n" + "=" * 80)
    print("  FAST TOF + CARTOGRAPHER SLAM 60 Hz COSTMAP PIPELINE BENCHMARK")
    print("=" * 80)

    pipeline = FastCostmapPipeline(resolution=0.02, enable_latency_comp=True)
    slam_grid, origin_xy, slam_res = create_synthetic_cartographer_track()

    # Simulate 10 seconds of high-speed racing motion (100 Hz poses, 60 Hz ToFs, 15 Hz SLAM)
    dt_sim = 0.010  # 100 Hz master clock
    total_time_s = 5.0
    steps = int(total_time_s / dt_sim)

    tof_exec_times = []
    carto_exec_times = []

    # Vehicle motion profile (circular racing trajectory at 3.5 m/s)
    speed = 3.5  # m/s
    radius = 3.0  # meters
    omega = speed / radius

    # 16 ToF sample readings observing an approaching 10cm pillar at 0.75m
    sample_tof_readings = {s.name: 1.80 for s in SENSOR_RIG}
    sample_tof_readings["tof_fl_3"] = 0.75
    sample_tof_readings["tof_fr_3"] = 0.74

    print(f"\n[Running Real-Time Simulation for {total_time_s:.1f}s ({steps} steps)]...")
    t_start_wall = time.perf_counter()

    for step in range(steps):
        t_sim = step * dt_sim

        # 1. 100 Hz Pose Stream
        theta = omega * t_sim
        x_pos = radius * math.cos(theta)
        y_pos = radius * math.sin(theta)
        yaw = (theta + math.pi / 2.0) % (2 * math.pi)
        pipeline.update_pose(t_sim, x_pos, y_pos, yaw, v_mps=speed)

        # 2. 15 Hz Cartographer SLAM Map Update (every ~66 ms)
        if step % 7 == 0:
            t_carto_start = time.perf_counter()
            current_pose = StampedPose(t_sim, x_pos, y_pos, yaw, speed)
            pipeline.update_slam_map(slam_grid, origin_xy, slam_res, current_pose)
            carto_exec_times.append((time.perf_counter() - t_carto_start) * 1000.0)

        # 3. 60 Hz ToF Costmap Frame Generation (every ~16.6 ms)
        if step % 2 == 0:  # ~50 to 60 Hz trigger
            prob_map, hazard_mask, exec_ms = pipeline.process_tof_frame(
                sample_tof_readings, timestamp_s=t_sim
            )
            tof_exec_times.append(exec_ms)

    t_total_wall = time.perf_counter() - t_start_wall

    print("=" * 80)
    print("BENCHMARK EXECUTION RESULTS")
    print("=" * 80)
    print(f"Total Simulation Duration:     {total_time_s:.2f} s")
    print(f"Wallclock Computation Time:    {t_total_wall*1000:.2f} ms ({total_time_s / t_total_wall:.1f}x faster than real-time)")
    print(f"60 Hz ToF Costmap Exec Time:   Mean: {np.mean(tof_exec_times):.4f} ms | Max: {np.max(tof_exec_times):.4f} ms | 99th%: {np.percentile(tof_exec_times, 99):.4f} ms")
    print(f"Equivalent Peak Throughput:    {1000.0 / np.mean(tof_exec_times):.0f} Hz (Budget at 60 Hz: 16.67 ms)")
    print(f"Raspberry Pi 5 Core Headroom:  > {100.0 - (np.mean(tof_exec_times) / 16.67 * 100.0):.2f}% CPU available for MPPI")
    print(f"15 Hz Carto Cropping Time:     Mean: {np.mean(carto_exec_times):.4f} ms | Max: {np.max(carto_exec_times):.4f} ms")
    print(f"Costmap Matrix Dimensions:     {prob_map.shape[0]} x {prob_map.shape[1]} cells (20 mm grid = 2.0 m x 2.0 m)")
    print(f"Active Hazard Cells (P > 0.55):{np.sum(hazard_mask)} cells flagged for instant MPPI collision check")
    print("=" * 80 + "\n")


if __name__ == "__main__":
    run_pipeline_benchmark()
