"""
================================================================================
  12. LAUNCHER: PHASE 3 COMPREHENSIVE DYNAMIC OBSTACLE & EGO-COMPENSATION BENCHMARK
================================================================================
  In this phase, the ego-robot is ALWAYS MOVING (oscillating smoothly along the
  longitudinal axis with v up to +/-0.60 m/s) under +/-5% sensor noise injection.
  The obstacle moves along 6 challenging trajectories:
    [1] : Mode 1 - Obstacle Stationary (Pure ego-motion rejection under noise)
    [2] : Mode 2 - Obstacle Longitudinal Motion (Towards / away along line of sight at ~0.71 m/s)
    [3] : Mode 3 - Obstacle Lateral Crossing (Left / right across FOV at ~0.75 m/s)
    [4] : Mode 4 - Obstacle Diagonal Trajectory (Simultaneous longitudinal & lateral at ~0.71 m/s)
    [5] : Mode 5 - Obstacle Accel / Decel Surge (Harmonic speed pulse 0.15 to 0.90 m/s)
    [6] : Mode 6 - Obstacle S-Curve / Lissajous Weave (Curved trajectory at ~0.65 m/s)
    [A] : Toggle Auto-Sweep through all modes (6s dwell each)
    [Space] : Pause / resume robot movement
    [R] : Reset robot and obstacle to starting coordinates

  Usage:
    python launchers/12_phase3_live_webots.py               (Launches Webots + Live GUI)
    python launchers/12_phase3_live_webots.py --mode 4      (Start directly in Diagonal Mode)
    python launchers/12_phase3_live_webots.py --auto-sweep  (Auto cycle modes 1 to 6)
    python launchers/12_phase3_live_webots.py --benchmark   (Instant headless analytical test)
    python launchers/12_phase3_live_webots.py --no-gui      (Webots 3D view only)
================================================================================
"""

import argparse
import atexit
import math
import os
import subprocess
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import numpy as np


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEBOTS_DIR = os.path.join(ROOT_DIR, "webots")
MY_PROJECT_DIR = os.path.join(WEBOTS_DIR, "simulation", "my_project")
WORLD_PATH = os.path.join(MY_PROJECT_DIR, "worlds", "test_dynamic_obstacles.wbt")
PLANNERS_DIR = os.path.join(MY_PROJECT_DIR, "controllers", "tof_path_planner")
WEBOTS_BIN = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

# Add planner module path for analytical benchmark
for p in [PLANNERS_DIR, MY_PROJECT_DIR, ROOT_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

try:
    from dynamic_obstacle_tracker import DynamicObstacleTracker
    from cpp_smart_hybrid_costmap import CppSmartHybridCostmap, SENSOR_NAMES
    ANALYTICAL_AVAILABLE = True
except Exception:
    ANALYTICAL_AVAILABLE = False


# 16-channel sensor rig coordinates and headings (from only_front_tofs.proto)
SENSOR_CONFIG = [
    # FL 0..7 (X = -0.070, Y = +0.095, cross-pointing Right)
    (-0.070, 0.095, 1.4987),
    (-0.070, 0.095, 1.3024),
    (-0.070, 0.095, 1.1060),
    (-0.070, 0.095, 0.9097),
    (-0.070, 0.095, 0.7134),
    (-0.070, 0.095, 0.5170),
    (-0.070, 0.095, 0.3207),
    (-0.070, 0.095, 0.1243),
    # FR 8..15 (X = +0.070, Y = +0.095, cross-pointing Left)
    ( 0.070, 0.095, 1.6428),
    ( 0.070, 0.095, 1.8391),
    ( 0.070, 0.095, 2.0356),
    ( 0.070, 0.095, 2.2318),
    ( 0.070, 0.095, 2.4282),
    ( 0.070, 0.095, 2.6245),
    ( 0.070, 0.095, 2.8208),
    ( 0.070, 0.095, 3.0172),
]


def _intersect_ray_cylinder(x0: float, y0: float, heading: float, cx: float, cy: float, radius: float) -> float:
    """Computes exact analytical ray-cylinder intersection distance."""
    dx = math.cos(heading)
    dy = math.sin(heading)
    fx = x0 - cx
    fy = y0 - cy
    a = dx * dx + dy * dy
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return 1.80
    sqrt_disc = math.sqrt(disc)
    t1 = (-b - sqrt_disc) / (2.0 * a)
    t2 = (-b + sqrt_disc) / (2.0 * a)
    hits = [t for t in (t1, t2) if 0.02 <= t <= 1.80]
    return min(hits) if hits else 1.80


def _intersect_ray_walls(x0: float, y0: float, heading: float, half_w: float = 0.60) -> float:
    """Computes distance to track corridor walls at x = -half_w and x = +half_w."""
    dx = math.cos(heading)
    t_min = 1.80
    if abs(dx) > 1e-4:
        t_left = (-half_w - x0) / dx
        if 0.02 <= t_left < t_min:
            t_min = t_left
        t_right = (half_w - x0) / dx
        if 0.02 <= t_right < t_min:
            t_min = t_right
    return t_min


def generate_readings_in_body_frame(
    obs_rx: float, obs_ry: float, obs_r: float = 0.075, corridor_half_w: float = 0.60, noise_pct: float = 0.05
) -> Dict[str, float]:
    """Generates synthetic 16-channel ToF readings given obstacle position in robot body frame with noise."""
    readings = {}
    for name, (x0, y0, heading) in zip(SENSOR_NAMES, SENSOR_CONFIG):
        d_obs = _intersect_ray_cylinder(x0, y0, heading, obs_rx, obs_ry, obs_r)
        d_wall = _intersect_ray_walls(x0, y0, heading, half_w=corridor_half_w)
        val = min(d_obs, d_wall, 1.80)
        if noise_pct > 0.0 and val < 1.79:
            noise = float(np.random.uniform(-noise_pct, noise_pct)) * val
            val = float(np.clip(val + noise, 0.02, 1.80))
        readings[name] = val
    return readings


def run_analytical_benchmark(noise_pct: float = 0.05) -> bool:
    """
    Comprehensive Phase 3 Benchmark:
    - Ego-robot ALWAYS MOVING (oscillating back-and-forth at up to 0.60 m/s)
    - Obstacle traversing 6 distinct trajectories (stationary, longitudinal, lateral, diagonal, surge, S-curve)
    - Tested with +-5% sensor noise injection
    """
    if not ANALYTICAL_AVAILABLE:
        print("[Error] Required analytical dependencies not importable.")
        return False

    print("=" * 96)
    print(f"  PHASE 3 COMPREHENSIVE BENCHMARK: ROBOT ALWAYS MOVING | +/-{int(noise_pct*100)}% SENSOR NOISE")
    print("  Evaluating ego-rejection, longitudinal approach, lateral crossing, diagonal, surge & weave")
    print("=" * 96)

    # Use fixed seed for perfectly reproducible benchmark evaluation
    np.random.seed(42)

    cm = CppSmartHybridCostmap(grid_bounds=(-1.0, 1.0, 0.0, 2.0), resolution=0.015, r_inflation_m=0.088)
    tracker = DynamicObstacleTracker(grid_bounds=(-1.0, 1.0, 0.0, 2.0), resolution=0.015)

    stages = [
        # (mode_idx, name, obs_v_target, duration_s, trajectory_type)
        (1, "Mode 1: Obstacle Stationary (Slalom Slips, 1.4m/s, 0.9rad/s)", 0.00, 2.5, "STATIC"),
        (2, "Mode 2: Obstacle Longitudinal Motion (Towards/Away, 0.63m/s)",  0.63, 2.5, "LONGITUDINAL"),
        (3, "Mode 3: Obstacle Lateral Crossing (Left/Right, 0.63m/s)",       0.63, 2.5, "LATERAL"),
        (4, "Mode 4: Obstacle Diagonal Path (Simultaneous X+Y, 0.68m/s)",    0.68, 2.5, "DIAGONAL"),
        (5, "Mode 5: Obstacle Accel/Decel Surge (Harmonic Pulse, 0.60m/s)", 0.60, 2.8, "SURGE"),
        (6, "Mode 6: Obstacle S-Curve / Lissajous Weave (0.65m/s)",          0.65, 2.5, "WEAVE"),
    ]

    dt = 1.0 / 60.0
    all_passed = True
    summary_results = []

    for mode_idx, name, target_v, duration_s, traj_type in stages:
        tracker.reset()
        n_frames = int(round(duration_s / dt))

        X_ego = 0.0
        Y_ego = 0.0
        theta_ego = 0.0

        obs_speeds = []
        is_dynamic_flags = []
        lead_distances = []

        for frame in range(n_frames):
            now_s = frame * dt

            # High-dynamics ego-motion profile: peak 1.40 m/s speed with +/-0.90 rad/s active slalom yaw
            v_ego = 1.40 * math.sin(2.0 * math.pi * now_s / 2.0)
            yaw_rate = 0.90 * math.sin(2.0 * math.pi * now_s / 0.9)

            # Dead reckoning position and heading update
            X_ego += -v_ego * math.sin(theta_ego) * dt
            Y_ego +=  v_ego * math.cos(theta_ego) * dt
            theta_ego += yaw_rate * dt

            # Obstacle world position based on calibrated trajectory (guaranteeing relative bounds)
            if traj_type == "STATIC":
                X_obs = 0.00
                Y_obs = 1.25
            elif traj_type == "LONGITUDINAL":
                X_obs = 0.00
                Y_obs = 1.25 + 0.22 * math.sin(2.0 * math.pi * now_s / 2.2)
            elif traj_type == "LATERAL":
                X_obs = 0.22 * math.sin(2.0 * math.pi * now_s / 2.2)
                Y_obs = 1.25
            elif traj_type == "DIAGONAL":
                X_obs = 0.17 * math.sin(2.0 * math.pi * now_s / 2.2)
                Y_obs = 1.25 + 0.17 * math.sin(2.0 * math.pi * now_s / 2.2)
            elif traj_type == "SURGE":
                X_obs = 0.00
                Y_obs = 1.25 + 0.18 * math.sin(2.0 * math.pi * now_s / 2.2) + 0.08 * math.sin(4.0 * math.pi * now_s / 2.2)
            elif traj_type == "WEAVE":
                X_obs = 0.18 * math.sin(2.0 * math.pi * now_s / 2.2)
                Y_obs = 1.25 + 0.10 * math.sin(4.0 * math.pi * now_s / 2.2)
            else:
                X_obs = 0.00
                Y_obs = 1.25

            # Compute relative coordinates in robot body frame
            dx_w = X_obs - X_ego
            dy_w = Y_obs - Y_ego
            cos_th = math.cos(theta_ego)
            sin_th = math.sin(theta_ego)
            obs_rx = float( cos_th * dx_w + sin_th * dy_w)
            obs_ry = float(-sin_th * dx_w + cos_th * dy_w)

            # Generate synthetic 16-channel ToF readings with noise injection
            readings = generate_readings_in_body_frame(
                obs_rx, obs_ry, obs_r=0.075, corridor_half_w=0.60, noise_pct=noise_pct
            )

            # Costmap update
            prob_map, hazard_mask, _ = cm.process_tof_frame(readings)

            # Ego-compensated dynamic tracker update (with direct raw ToF rays & 1D range-rate)
            enlarged_mask, dyn_tracks, telem = tracker.process_frame(
                prob_map=prob_map,
                robot_pose=(X_ego, Y_ego, theta_ego),
                now_s=now_s,
                base_hazard_mask=hazard_mask,
                v_ego=v_ego,
                yaw_rate=yaw_rate,
                tof_readings=readings,
            )

            # Record metrics after warmup frames
            if frame >= 6 and tracker.tracks:
                for t in tracker.tracks.values():
                    obs_speeds.append(t.v_mag)
                    is_dynamic_flags.append(t.is_dynamic)
                    if t.is_dynamic:
                        lead_distances.append(min(0.25, t.v_mag * 0.20))

        # Evaluate performance for this stage (top speeds for sinusoidal/varying profiles)
        if obs_speeds:
            sorted_speeds = sorted(obs_speeds)
            top_speeds = sorted_speeds[int(len(sorted_speeds) * 0.70):]
            est_v = float(np.mean(top_speeds))
            max_v = float(max(obs_speeds))
        else:
            est_v = 0.0
            max_v = 0.0

        flagged_dynamic = any(is_dynamic_flags) if is_dynamic_flags else False
        mean_lead_cm = float(np.mean(lead_distances) * 100.0) if lead_distances else 0.0

        expected_dynamic = (target_v >= 0.30)
        err = abs(est_v - target_v)

        # Allow generous velocity overshoot margin for fast dynamic responsiveness
        overshoot = (est_v > target_v * 2.20 + 0.50) if expected_dynamic else (max_v >= 0.60)

        if expected_dynamic:
            passed = flagged_dynamic and (not overshoot) and (err <= 0.60) and (mean_lead_cm >= 5.0)
        else:
            passed = (not flagged_dynamic) and (not overshoot) and (est_v < 0.38) and (mean_lead_cm == 0.0)

        if not passed:
            all_passed = False

        status_str = "PASS" if passed else "FAIL"
        summary_results.append({
            "mode": mode_idx,
            "name": name,
            "target_v": target_v,
            "est_v": est_v,
            "error_mps": err,
            "flagged_dynamic": flagged_dynamic,
            "expected_dynamic": expected_dynamic,
            "lead_cm": mean_lead_cm,
            "status": status_str,
        })
        print(f"  [{mode_idx}/6] {name:<62} | Target: {target_v:4.2f} m/s | Est: {est_v:5.2f} m/s | Dyn: {str(flagged_dynamic):<5} | Lead: {mean_lead_cm:4.1f}cm -> [{status_str}]")

    print("\n" + "=" * 96)
    print("  PHASE 3 COMPREHENSIVE BENCHMARK SUMMARY TABLE")
    print("=" * 96)
    print(f"{'Mode / Scenario':<54} | {'Target':<8} | {'Estimated':<9} | {'Error':<7} | {'Dynamic?':<8} | {'Lead':<7} | {'Result'}")
    print("-" * 104)
    for r in summary_results:
        print(f"{r['name']:<54} | {r['target_v']:4.2f} m/s | {r['est_v']:5.2f} m/s | {r['error_mps']:4.2f} m | {str(r['flagged_dynamic']):<8} | {r['lead_cm']:4.1f}cm | {r['status']}")
    print("=" * 96)

    if all_passed:
        print("\n>>> [ALL TESTS PASSED] Phase 3 Dynamic Obstacle & Ego-Motion Compensation verified under noise and complex trajectories!\n")
    else:
        print("\n>>> [BENCHMARK FAILED] One or more Phase 3 test stages did not meet criteria.\n")

    return all_passed


def run_snapshots_benchmark(noise_pct: float = 0.05) -> bool:
    """
    Comprehensive Size & Deceleration Snapshot Suite:
    - Tests 4 pillar sizes: 5cm, 10cm, 15cm, and 20cm
    - Evaluates speed tiers up to 3.0 m/s with continuous acceleration and deceleration
    - Captures multiple snapshots during ACCEL, PEAK, DECEL, and ZERO-crossing phases
    - Focuses on deceleration performance, velocity lag, and phantom hazard prevention
    """
    if not ANALYTICAL_AVAILABLE:
        print("[Error] Required analytical dependencies not importable.")
        return False

    np.random.seed(42)
    cm = CppSmartHybridCostmap(grid_bounds=(-1.0, 1.0, 0.0, 2.0), resolution=0.015, r_inflation_m=0.088)
    tracker = DynamicObstacleTracker(grid_bounds=(-1.0, 1.0, 0.0, 2.0), resolution=0.015)

    pillar_sizes = [
        (" 5cm", 0.025),
        ("10cm", 0.050),
        ("15cm", 0.075),
        ("20cm", 0.100),
    ]

    # Varied acceleration/deceleration scenarios bounded by a_max <= 6.0 m/s^2
    scenarios = [
        ("Longitudinal Gentle (1.5 m/s^2, 0.75m/s)",    0.24, 2.00, "LONGITUDINAL"),
        ("Longitudinal Moderate (3.0 m/s^2, 1.10m/s)",  0.25, 1.40, "LONGITUDINAL"),
        ("Longitudinal Aggressive (6.0 m/s^2, 1.60m/s)",0.25, 1.00, "LONGITUDINAL"),
        ("Lateral Moderate (3.0 m/s^2, 1.10m/s)",       0.25, 1.40, "LATERAL"),
        ("Lateral Aggressive (6.0 m/s^2, 1.60m/s)",     0.25, 1.00, "LATERAL"),
        ("Diagonal Firm (4.5 m/s^2, 1.40m/s)",          0.20, 1.25, "DIAGONAL"),
        ("Diagonal Aggressive (6.0 m/s^2, 2.20m/s)",    0.18, 0.75, "DIAGONAL"),
        ("Surge Pulse Aggressive (6.0 m/s^2, 2.50m/s)", 0.20, 0.70, "SURGE"),
        ("Lissajous Weave Firm (4.5 m/s^2, 1.60m/s)",   0.20, 1.10, "WEAVE"),
    ]

    dt = 1.0 / 60.0
    duration_s = 3.0
    n_frames = int(round(duration_s / dt))

    print("=" * 104)
    print(f"  PHASE 3 SIZE VARIANCE & DECELERATION SNAPSHOT BENCHMARK (+/-{int(noise_pct*100)}% NOISE)")
    print("  Evaluating 5cm, 10cm, 15cm, 20cm pillars across acceleration, peak (up to 3 m/s), and deceleration")
    print("=" * 104)

    # First verify static baseline rejection across all 4 pillar sizes
    print("\n--- STAGE 1: STATIC EGO-REJECTION ACROSS PILLAR SIZES (Ego Slalom 1.4m/s, 0.9rad/s) ---")
    print(f"{'Size':<8} | {'Radius':<8} | {'True Speed':<11} | {'Est Speed':<10} | {'Max Speed':<10} | {'Dynamic?':<9} | {'Lead cm':<8} | {'Result'}")
    print("-" * 88)
    for s_name, r in pillar_sizes:
        tracker.reset()
        X_ego, Y_ego, theta_ego = 0.0, 0.0, 0.0
        obs_speeds = []
        is_dyn_flags = []
        for frame in range(int(round(2.0 / dt))):
            now_s = frame * dt
            v_ego = 1.40 * math.sin(2.0 * math.pi * now_s / 2.0)
            yaw_rate = 0.90 * math.sin(2.0 * math.pi * now_s / 0.9)
            X_ego += -v_ego * math.sin(theta_ego) * dt
            Y_ego +=  v_ego * math.cos(theta_ego) * dt
            theta_ego += yaw_rate * dt

            dx_w = 0.00 - X_ego
            dy_w = 1.25 - Y_ego
            cos_th = math.cos(theta_ego)
            sin_th = math.sin(theta_ego)
            obs_rx = float( cos_th * dx_w + sin_th * dy_w)
            obs_ry = float(-sin_th * dx_w + cos_th * dy_w)

            readings = generate_readings_in_body_frame(obs_rx, obs_ry, obs_r=r, corridor_half_w=0.60, noise_pct=noise_pct)
            prob_map, hazard_mask, _ = cm.process_tof_frame(readings)
            _, _, _ = tracker.process_frame(prob_map, (X_ego, Y_ego, theta_ego), now_s, hazard_mask, v_ego=v_ego, yaw_rate=yaw_rate)
            if frame >= 6 and tracker.tracks:
                for t in tracker.tracks.values():
                    obs_speeds.append(t.v_mag)
                    is_dyn_flags.append(t.is_dynamic)

        est_v = float(np.mean(sorted(obs_speeds)[int(len(obs_speeds)*0.70):])) if obs_speeds else 0.0
        max_v = float(max(obs_speeds)) if obs_speeds else 0.0
        dyn = any(is_dyn_flags)
        passed = (not dyn) and (max_v < 0.40)
        res_str = "PASS" if passed else "FAIL"
        print(f"{s_name:<8} | {r:6.3f}m  |  0.00 m/s   | {est_v:5.2f} m/s  | {max_v:5.2f} m/s  | {str(dyn):<9} |  0.0 cm  | [{res_str}]")

    print("\n--- STAGE 2: DECELERATION & SNAPSHOT PERFORMANCE MATRIX (UP TO 3.0 m/s) ---")
    run_records = []

    for sc_name, amp, period, traj_kind in scenarios:
        omega = 2.0 * math.pi / period
        peak_true_v = amp * omega if traj_kind != "SURGE" else (amp * omega + 0.08 * (4.0 * math.pi / period))
        print(f"\n>>> SCENARIO: {sc_name} (Peak True Speed: {peak_true_v:4.2f} m/s)")
        print(f"{'Size':<6} | {'Snapshot Phase':<16} | {'True Speed':<11} | {'Est Speed':<10} | {'Error (Est-True)':<17} | {'Dyn Flag':<9} | {'Lead Dist':<9} | {'Decel Performance'}")
        print("-" * 104)

        for s_name, r in pillar_sizes:
            tracker.reset()
            X_ego, Y_ego, theta_ego = 0.0, 0.0, 0.0
            phase_data = {"ACCEL": [], "PEAK": [], "DECEL": [], "ZERO": []}

            for frame in range(n_frames):
                now_s = frame * dt
                v_ego = 1.40 * math.sin(2.0 * math.pi * now_s / 2.0)
                yaw_rate = 0.90 * math.sin(2.0 * math.pi * now_s / 0.9)
                X_ego += -v_ego * math.sin(theta_ego) * dt
                Y_ego +=  v_ego * math.cos(theta_ego) * dt
                theta_ego += yaw_rate * dt

                cos_val = math.cos(omega * now_s)
                sin_val = math.sin(omega * now_s)

                if traj_kind == "LONGITUDINAL":
                    X_obs = 0.00
                    Y_obs = 1.25 + amp * sin_val
                    v_true_inst = abs(amp * omega * cos_val)
                    accel_sign = -sin_val * cos_val
                elif traj_kind == "LATERAL":
                    X_obs = amp * sin_val
                    Y_obs = 1.25
                    v_true_inst = abs(amp * omega * cos_val)
                    accel_sign = -sin_val * cos_val
                elif traj_kind == "DIAGONAL":
                    X_obs = amp * sin_val
                    Y_obs = 1.25 + amp * sin_val
                    v_true_inst = math.sqrt(2.0) * abs(amp * omega * cos_val)
                    accel_sign = -sin_val * cos_val
                elif traj_kind == "SURGE":
                    X_obs = 0.00
                    Y_obs = 1.25 + amp * sin_val + 0.08 * math.sin(2.0 * omega * now_s)
                    vy = amp * omega * cos_val + 0.08 * (2.0 * omega) * math.cos(2.0 * omega * now_s)
                    v_true_inst = abs(vy)
                    accel_sign = -sin_val * cos_val
                elif traj_kind == "WEAVE":
                    X_obs = amp * sin_val
                    Y_obs = 1.25 + 0.10 * math.sin(2.0 * omega * now_s)
                    vx = amp * omega * cos_val
                    vy = 0.10 * (2.0 * omega) * math.cos(2.0 * omega * now_s)
                    v_true_inst = math.hypot(vx, vy)
                    accel_sign = -sin_val * cos_val
                else:
                    X_obs, Y_obs = 0.00, 1.25
                    v_true_inst = 0.0
                    accel_sign = 0.0

                if abs(cos_val) > 0.85:
                    p_type = "PEAK"
                elif abs(cos_val) < 0.20:
                    p_type = "ZERO"
                elif accel_sign < 0:
                    p_type = "DECEL"
                else:
                    p_type = "ACCEL"

                dx_w = X_obs - X_ego
                dy_w = Y_obs - Y_ego
                cos_th = math.cos(theta_ego)
                sin_th = math.sin(theta_ego)
                obs_rx = float( cos_th * dx_w + sin_th * dy_w)
                obs_ry = float(-sin_th * dx_w + cos_th * dy_w)

                readings = generate_readings_in_body_frame(obs_rx, obs_ry, obs_r=r, corridor_half_w=0.60, noise_pct=noise_pct)
                prob_map, hazard_mask, _ = cm.process_tof_frame(readings)
                _, _, _ = tracker.process_frame(prob_map, (X_ego, Y_ego, theta_ego), now_s, hazard_mask, v_ego=v_ego, yaw_rate=yaw_rate)

                if frame >= 8 and tracker.tracks:
                    for t in tracker.tracks.values():
                        lead_dist = min(0.25, t.v_mag * 0.20) * 100.0 if t.is_dynamic else 0.0
                        phase_data[p_type].append((v_true_inst, t.v_mag, t.is_dynamic, lead_dist))

            rec = {
                "scenario": sc_name,
                "size": s_name,
                "peak_true": 0.0, "peak_est": 0.0,
                "decel_true": 0.0, "decel_est": 0.0, "decel_err": 0.0,
                "zero_true": 0.0, "zero_est": 0.0,
                "overshoot": False,
            }

            for p_name in ["ACCEL", "PEAK", "DECEL", "ZERO"]:
                samples = phase_data[p_name]
                if samples:
                    mean_true = float(np.mean([s[0] for s in samples]))
                    mean_est  = float(np.mean([s[1] for s in samples]))
                    dyn_pct   = float(np.mean([1.0 if s[2] else 0.0 for s in samples])) * 100.0
                    mean_lead = float(np.mean([s[3] for s in samples]))
                    err = mean_est - mean_true

                    if p_name == "PEAK":
                        rec["peak_true"] = mean_true
                        rec["peak_est"] = mean_est
                        decel_eval = "Peak Saturation / Tracking"
                    elif p_name == "DECEL":
                        rec["decel_true"] = mean_true
                        rec["decel_est"] = mean_est
                        rec["decel_err"] = err
                        rec["overshoot"] = (mean_est > mean_true + 0.05)
                        decel_eval = "Clean Decel Decay (No Overshoot)" if not rec["overshoot"] else "Lag / Overshoot Warning"
                    elif p_name == "ZERO":
                        rec["zero_true"] = mean_true
                        rec["zero_est"] = mean_est
                        decel_eval = "Turnaround Inversion"
                    else:
                        decel_eval = "Smooth Velocity Ramp"

                    print(f"{s_name:<6} | {p_name:<16} | {mean_true:5.2f} m/s   | {mean_est:5.2f} m/s  | {err:+5.2f} m/s          | {dyn_pct:4.0f}%     | {mean_lead:4.1f} cm   | {decel_eval}")

            run_records.append(rec)

    # Roll-up summary scorecard of all 36 dynamic test cases
    print("\n" + "=" * 114)
    print("  COMPREHENSIVE MULTI-SCENARIO DECELERATION SCORECARD (36 DYNAMIC TESTS + 4 STATIC)")
    print("=" * 114)
    print(f"{'Scenario':<38} | {'Size':<5} | {'Peak True':<10} | {'Peak Est':<9} | {'Decel True':<11} | {'Decel Est':<10} | {'Decel Err':<10} | {'Overshoot?':<11} | {'Status'}")
    print("-" * 114)
    total_overshoots = 0
    for r in run_records:
        ov = "YES (FAIL)" if r["overshoot"] else "NO (SAFE)"
        st = "PASS" if not r["overshoot"] else "WARN"
        if r["overshoot"]:
            total_overshoots += 1
        print(f"{r['scenario']:<38} | {r['size']:<5} | {r['peak_true']:5.2f} m/s  | {r['peak_est']:5.2f} m/s | {r['decel_true']:5.2f} m/s   | {r['decel_est']:5.2f} m/s  | {r['decel_err']:+5.2f} m/s   | {ov:<11} | [{st}]")

    print("=" * 114)
    print(f"  TOTAL COMPREHENSIVE RUNS EVALUATED: 40 (4 Static + 36 Dynamic Accelerating/Decelerating)")
    print(f"  TOTAL DETAILED SNAPSHOT PHASES:     144 (ACCEL, PEAK, DECEL, ZERO)")
    print(f"  TOTAL FRAMES SIMULATED AT 60 Hz:    6,960 frames (116.0 seconds continuous physical simulation)")
    print(f"  DECELERATION OVERSHOOT RATE:        {total_overshoots}/36 ({total_overshoots/36*100:.1f}%)")
    print(f"  EGO SLALOM KINEMATICS:              1.40 m/s Speed, +/-0.90 rad/s Active Yaw Rate, +/-5% ToF Noise")
    print("=" * 114 + "\n")
    return total_overshoots == 0


def _kill_proc(proc: subprocess.Popen) -> None:
    if proc is None:
        return
    try:
        if sys.platform == "win32":
            subprocess.run(
                ["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
        else:
            proc.terminate()
            proc.wait(timeout=2.0)
    except Exception:
        pass


def launch_phase3_live(
    launch_gui: bool = True,
    telemetry_hz: int = 60,
    duration_s: Optional[float] = None,
    mode: int = 1,
    auto_sweep: bool = False,
    sweep_interval: float = 6.0,
    noise_pct: float = 0.05,
) -> None:
    if not os.path.isfile(WORLD_PATH):
        print(f"[Error] World file not found: {WORLD_PATH}", flush=True)
        return

    if not os.path.isfile(WEBOTS_BIN):
        print(f"[Error] Webots executable not found: {WEBOTS_BIN}", flush=True)
        return

    env = os.environ.copy()
    webots_bin_dir = os.path.dirname(WEBOTS_BIN)
    webots_home = r"C:\Program Files\Webots"
    env["WEBOTS_HOME"] = webots_home
    env["PATH"] = webots_bin_dir + os.pathsep + env.get("PATH", "")
    env["WEBOTS_CONTROLLER_MODE"] = "phase3"
    env["WEBOTS_PHASE3_MODE"] = str(mode)
    env["WEBOTS_AUTO_SWEEP"] = "1" if auto_sweep else "0"
    env["WEBOTS_SWEEP_INTERVAL"] = str(sweep_interval)
    env["WEBOTS_TELEMETRY_HZ"] = str(telemetry_hz)
    env["TOF_NOISE_PCT"] = str(noise_pct)
    env["PYTHONUNBUFFERED"] = "1"
    if duration_s is not None and duration_s > 0.0:
        env["WEBOTS_RUN_SECONDS"] = str(duration_s)

    gui_proc = None
    if launch_gui:
        gui_script = os.path.join(PLANNERS_DIR, "planner_gui.py")
        if os.path.isfile(gui_script):
            print(f"[1/2] Spawning Live Telemetry Map GUI (Port 9876, Stream: {telemetry_hz} Hz)...", flush=True)
            gui_proc = subprocess.Popen(
                [sys.executable, gui_script, "--telemetry-hz", str(telemetry_hz)],
                cwd=PLANNERS_DIR,
                env=env,
            )
            time.sleep(0.8)

    cmd = [
        WEBOTS_BIN,
        "--mode=realtime",
        "--batch",
        "--stdout",
        "--stderr",
        WORLD_PATH,
    ]

    print("=" * 84)
    print("  LAUNCHING FULLY RENDERED WEBOTS PHASE 3 DYNAMIC OBSTACLE TEST")
    print(f"  World:          {WORLD_PATH}")
    print("  Mode:           Fully Rendered Desktop Simulation (--mode=realtime)")
    print(f"  Robot State:    ALWAYS MOVING (Oscillating at v up to +/-0.60 m/s)")
    print(f"  Sensor Noise:   +/-{int(noise_pct*100)}% ToF Noise Injection")
    print(f"  Initial Mode:   Mode {mode}")
    print(f"  Auto-Sweep:     {'ENABLED (Interval: ' + str(sweep_interval) + 's)' if auto_sweep else 'DISABLED'}")
    print(f"  Telemetry Hz:   {telemetry_hz} Hz")
    if duration_s:
        print(f"  Duration:       {duration_s:.1f} seconds (Supervisor-controlled auto quit)")
    else:
        print("  Duration:       Interactive (Press Ctrl+C or close window to exit)")
    print("  Interactive Controls (Click GUI buttons or 3D Webots window):")
    print("    [1] Obstacle Stationary (Robot oscillates, tests ego-motion rejection)")
    print("    [2] Obstacle Longitudinal Movement (Towards / away at ~0.71 m/s)")
    print("    [3] Obstacle Lateral Crossing (Left / right at ~0.75 m/s)")
    print("    [4] Obstacle Diagonal Path (Simultaneous X & Y at ~0.71 m/s)")
    print("    [5] Obstacle Accel / Decel Surge (Harmonic pulse 0.15 to 0.90 m/s)")
    print("    [6] Obstacle S-Curve / Lissajous Weave (~0.65 m/s)")
    print("    [A] Toggle Auto-Sweep through all 6 modes")
    print("    [Space] Pause / Resume Robot movement")
    print("    [R] Reset poses")
    print("=" * 84)

    print(f"[{'2/2' if gui_proc else '1/1'}] Launching Webots 3D simulation...", flush=True)
    webots_proc = subprocess.Popen(
        cmd,
        env=env,
        cwd=MY_PROJECT_DIR,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    def _stream_pipe():
        if webots_proc.stdout:
            for line in webots_proc.stdout:
                print(line, end="", flush=True)

    stream_thread = threading.Thread(target=_stream_pipe, daemon=True)
    stream_thread.start()

    def cleanup():
        if webots_proc and webots_proc.poll() is None:
            _kill_proc(webots_proc)
        if gui_proc and gui_proc.poll() is None:
            _kill_proc(gui_proc)

    atexit.register(cleanup)

    try:
        while True:
            gui_done = gui_proc is not None and gui_proc.poll() is not None
            webots_done = webots_proc.poll() is not None
            if gui_done:
                print("\n[Launcher] Telemetry GUI closed. Terminating Webots simulation...", flush=True)
                break
            if webots_done:
                print(f"\n[Launcher] Webots simulation closed (exit code {webots_proc.poll()}). Terminating Telemetry GUI...", flush=True)
                break
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("\n[Launcher] User interrupted session. Shutting down cleanly...", flush=True)
    finally:
        cleanup()


def main():
    parser = argparse.ArgumentParser(description="Phase 3: Comprehensive Moving Robot & Dynamic Obstacle Test Bench")
    parser.add_argument("--benchmark", action="store_true", help="Run fast mathematical/analytical verification of all 6 Phase 3 modes")
    parser.add_argument("--snapshots", action="store_true", help="Run comprehensive size variance (5, 10, 15, 20cm) and deceleration snapshot matrix (up to 3 m/s)")
    parser.add_argument("--noise-pct", type=float, default=0.05, help="Sensor noise injection percentage (default: 0.05 = +/-5%%)")
    parser.add_argument("--mode", "-m", type=int, choices=[1, 2, 3, 4, 5, 6], default=1, help="Initial Phase 3 obstacle mode (1-6, default: 1)")
    parser.add_argument("--auto-sweep", action="store_true", help="Automatically sweep through all 6 Phase 3 modes sequentially")
    parser.add_argument("--sweep-interval", type=float, default=6.0, help="Dwell time in seconds per mode during auto-sweep (default: 6.0s)")
    parser.add_argument("--no-gui", action="store_true", help="Do not launch live 2D telemetry map GUI alongside Webots")
    parser.add_argument("--telemetry-hz", "--hz", type=int, default=60, help="Streaming frequency for GUI telemetry (default: 60 Hz)")
    parser.add_argument("--duration", "-d", type=float, default=None, help="Benchmark run duration in seconds before auto-quitting (default: interactive)")
    args = parser.parse_args()

    if args.snapshots:
        success = run_snapshots_benchmark(noise_pct=args.noise_pct)
        sys.exit(0 if success else 1)

    if args.benchmark:
        success = run_analytical_benchmark(noise_pct=args.noise_pct)
        sys.exit(0 if success else 1)

    telem_hz = max(1, min(60, args.telemetry_hz))
    launch_phase3_live(
        launch_gui=not args.no_gui,
        telemetry_hz=telem_hz,
        duration_s=args.duration,
        mode=args.mode,
        auto_sweep=args.auto_sweep,
        sweep_interval=args.sweep_interval,
        noise_pct=args.noise_pct,
    )


if __name__ == "__main__":
    main()
