"""
================================================================================
  18-RUN CARTO SLAM 15MM HIGH-DETAIL SWEEP (SUB36 + BALANCED CARVE)
================================================================================
"""

import os
import sys
import time
import json
import itertools
import subprocess
import numpy as np

# ==============================================================================
# 1. PARAMETER SWEEP DEFINITIONS (EXACTLY 18 RUNS: 3 x 3 x 2)
# ==============================================================================
SWEEP_CONFIG = {
    # Run & Path Settings
    "re_run_webots": False,
    "world_file": r"my_project\worlds\Cyan_Champion_Track_Static.wbt",
    "sim_duration_s": 23.0,
    "max_laps": 3,
    "output_dir": r"slam_results",

    # --- 15mm High-Resolution Grid ---
    "resolution": 0.02,
    "voxel_filter_size": 0.01,
    "min_range": 0.12,
    "max_range": 3.90,
    "missing_data_ray_length": 0.0,
    "num_accumulated_range_data": 1,

    # --- Locked Submap & Midpoint Carving Baseline ---
    "submaps_num_range_data": 36,                          # Locked
    "ceres_occupied_space_weight": 52.5,                   # Locked midpoint (45.0 <-> 60.0)
    "hit_probability": 0.78,                               # Locked midpoint (0.70 <-> 0.85)
    "miss_probability": 0.38,                              # Locked midpoint (0.42 <-> 0.35)
    "ceres_translation_weight": 10.0,
    "ceres_max_iterations": 30,

    # Local Correlative Matcher
    "use_online_correlative_scan_matching": True,
    "linear_search_window": 0.15,
    "angular_search_window_deg": 12.0,

    # Global Pose Graph & Branch-and-Bound
    "optimize_every_n_nodes": 2,
    "constraint_sampling_ratio": 1.0,
    "global_sampling_ratio": 1.0,
    "global_constraint_search_after_n_seconds": 0.0,
    "max_constraint_distance": 25.0,
    "fast_csm_linear_search_window": 3.5,
    "fast_csm_angular_search_window_deg": 25.0,
    "fast_csm_branch_and_bound_depth": 5,                  # 0.015m * 2^5 = 0.48m coarse root cell
    "loop_closure_translation_weight": 5e5,
    "loop_closure_rotation_weight": 5e6,
    "huber_scale": 10.0,
    "pose_graph_max_iterations": 60,

    # Motion Filter Constants
    "motion_filter_max_angle_deg": 0.5,
    "motion_filter_max_time_s": 1.0,

    # ==========================================================================
    # ACTIVE SWEPT PARAMETERS (3 x 3 x 2 = 18 RUNS)
    # ==========================================================================
    # 1. Motion Gating Distance: 15mm (1 cell), 30mm (2 cells), 50mm (skips slow stacking)
    "motion_filter_max_dist_m": 0.02,

    # 2. Local Ceres Rotation Stiffness (Gyro Yaw vs Contour Snapping)
    "ceres_rotation_weight": 30.0,

    # 3. Global Loop Closure Min Score Cutoff
    "loop_closure_min_score": 0.36,
}


def build_parameter_grid(sweep_dict):
    meta_keys = {"re_run_webots", "world_file", "sim_duration_s", "max_laps", "output_dir"}
    param_options = {}
    swept_keys = []
    fixed_params = {}
    
    for k, v in sweep_dict.items():
        if k not in meta_keys:
            if isinstance(v, list):
                param_options[k] = v
                if len(v) > 1:
                    swept_keys.append(k)
                else:
                    fixed_params[k] = v[0]
            else:
                param_options[k] = [v]
                fixed_params[k] = v

    keys = list(param_options.keys())
    value_lists = [param_options[k] for k in keys]
    
    combinations = []
    for combo in itertools.product(*value_lists):
        combinations.append(dict(zip(keys, combo)))
        
    return combinations, swept_keys, fixed_params

def print_fixed_setup_header(fixed_params, swept_keys, total_runs):
    print("=" * 80)
    print("  CARTO SLAM 18-RUN 15MM DETAIL SWEEP (SUB36 + MIDPOINT CARVE)")
    print("=" * 80)
    print(f" Total Runs Scheduled : {total_runs}")
    print(f" Active Variables ({len(swept_keys)}) : {', '.join(swept_keys)}")
    print("-" * 80)
    print(" [FIXED BASELINE CONFIGURATION]")
    items = list(fixed_params.items())
    for i in range(0, len(items), 2):
        k1, v1 = items[i]
        col1 = f"{k1}: {v1}"
        if i + 1 < len(items):
            k2, v2 = items[i+1]
            col2 = f"{k2}: {v2}"
            print(f"   * {col1:<37} | {col2}")
        else:
            print(f"   * {col1}")
    print("=" * 80 + "\n")

def generate_carto_lua(cfg):
    use_corr = "true" if cfg.get('use_online_correlative_scan_matching', True) else "false"
    
    return f"""include "map_builder.lua"
include "trajectory_builder.lua"

options = {{
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = true,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- Trajectory Builder 2D (15mm Grid)
TRAJECTORY_BUILDER_2D.min_range = {cfg.get('min_range', 0.12)}
TRAJECTORY_BUILDER_2D.max_range = {cfg.get('max_range', 3.00)}
TRAJECTORY_BUILDER_2D.missing_data_ray_length = {cfg.get('missing_data_ray_length', 0.0)}
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = {cfg.get('voxel_filter_size', 0.008)}

-- Ceres Scan Matcher (Local SLAM)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = {cfg.get('ceres_occupied_space_weight', 52.5)}
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = {cfg.get('ceres_translation_weight', 10.0)}
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = {cfg.get('ceres_rotation_weight', 50.0)}
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = {cfg.get('ceres_max_iterations', 30)}

-- Real-Time Correlative Scan Matcher
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = {use_corr}
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = {cfg.get('linear_search_window', 0.15)}
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad({cfg.get('angular_search_window_deg', 12.0)})

-- Motion Filter (Spatial Gating)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = {cfg.get('motion_filter_max_dist_m', 0.03)}
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad({cfg.get('motion_filter_max_angle_deg', 0.5)})
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = {cfg.get('motion_filter_max_time_s', 1.0)}

-- Submaps (Locked Sub36 + Midpoint Active Carve)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = {cfg.get('submaps_num_range_data', 36)}
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = {cfg.get('resolution', 0.015)}
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = {cfg.get('hit_probability', 0.78)}
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = {cfg.get('miss_probability', 0.38)}

-- Pose Graph Global SLAM
POSE_GRAPH.optimize_every_n_nodes = {cfg.get('optimize_every_n_nodes', 2)}
POSE_GRAPH.global_constraint_search_after_n_seconds = {cfg.get('global_constraint_search_after_n_seconds', 0.0)}
POSE_GRAPH.constraint_builder.sampling_ratio = {cfg.get('constraint_sampling_ratio', 1.0)}
POSE_GRAPH.global_sampling_ratio = {cfg.get('global_sampling_ratio', 1.0)}
POSE_GRAPH.constraint_builder.max_constraint_distance = {cfg.get('max_constraint_distance', 25.0)}
POSE_GRAPH.constraint_builder.min_score = {cfg.get('loop_closure_min_score', 0.38)}
POSE_GRAPH.constraint_builder.global_localization_min_score = {cfg.get('loop_closure_min_score', 0.38)}

-- Fast Correlative Scan Matcher
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = {cfg.get('fast_csm_linear_search_window', 3.5)}
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad({cfg.get('fast_csm_angular_search_window_deg', 25.0)})
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = {cfg.get('fast_csm_branch_and_bound_depth', 5)}

-- Optimization Problem Weights
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = {cfg.get('loop_closure_translation_weight', 5e5)}
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = {cfg.get('loop_closure_rotation_weight', 5e6)}
POSE_GRAPH.optimization_problem.huber_scale = {cfg.get('huber_scale', 10.0)}
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = {cfg.get('pose_graph_max_iterations', 60)}

return options
"""

def to_wsl_path(win_path):
    p = os.path.abspath(win_path).replace("\\", "/")
    if len(p) >= 2 and p[1] == ":":
        drive = p[0].lower()
        p = f"/mnt/{drive}" + p[2:]
    return p

def main():
    root_dir = os.path.abspath(os.path.dirname(__file__))
    os.chdir(root_dir)

    scan_jsonl_win = os.path.abspath(r"my_project\controllers\folkrace_drive\live_webots_slam_scans.jsonl")
    if SWEEP_CONFIG.get("re_run_webots", True) or not os.path.exists(scan_jsonl_win):
        world_abs = os.path.abspath(SWEEP_CONFIG["world_file"])
        webots_bin = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
        
        print(f"\n[STEP 1] Running Webots simulation on {os.path.basename(world_abs)}...")
        env = os.environ.copy()
        env["WEBOTS_RUN_SECONDS"] = str(SWEEP_CONFIG["sim_duration_s"])
        env["MAX_BENCHMARK_LAPS"] = str(SWEEP_CONFIG["max_laps"])
        env["ENABLE_MOVING_OBSTACLE"] = "0"
        env["WEBOTS_HOME"] = r"C:\Program Files\Webots"

        webots_cmd = [webots_bin, "--mode=fast", "--batch", "--minimize", world_abs]
        sim_proc = subprocess.Popen(webots_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace")
        
        for line in sim_proc.stdout:
            if "LAP COMPLETED" in line or "BENCHMARK" in line or "CRASH" in line:
                print("  " + line.strip(), flush=True)

        sim_proc.wait()
        print("  Webots simulation finished.\n")

    combinations, swept_keys, fixed_params = build_parameter_grid(SWEEP_CONFIG)
    total_configs = len(combinations)
    print_fixed_setup_header(fixed_params, swept_keys, total_configs)

    out_dir_abs = os.path.abspath(SWEEP_CONFIG["output_dir"])
    os.makedirs(out_dir_abs, exist_ok=True)
    out_dir_wsl = to_wsl_path(out_dir_abs)

    feeder_wsl = to_wsl_path(r"ros2_cartographer\scripts\feed_pure_lidar_imu.py")
    exporter_wsl = to_wsl_path(r"ros2_cartographer\scripts\export_cartographer_png.py")
    scan_jsonl_wsl = to_wsl_path(scan_jsonl_win)

    for idx, cfg in enumerate(combinations):
        m_dist = int(round(cfg["motion_filter_max_dist_m"] * 1000))
        rot_w = int(cfg["ceres_rotation_weight"])
        loop_s = f"{cfg['loop_closure_min_score']:.2f}".replace(".", "")

        cfg_name = f"map_{idx+1:02d}_15mm_motDist{m_dist:02d}mm_rotW{rot_w}_loop{loop_s}"
        out_png_win = os.path.join(out_dir_abs, f"{cfg_name}.png")
        out_png_wsl = to_wsl_path(out_png_win)

        print(f"[{idx+1:02d}/{total_configs}] {cfg_name}")
        for k in swept_keys:
            print(f"     > {k:<28}: {cfg[k]}")

        lua_temp_win = os.path.abspath(r"ros2_cartographer\config\sweep_temp.lua")
        os.makedirs(os.path.dirname(lua_temp_win), exist_ok=True)
        with open(lua_temp_win, "w", encoding="utf-8") as f:
            f.write(generate_carto_lua(cfg))
        lua_wsl = to_wsl_path(lua_temp_win)

        sh_content = f"""#!/usr/bin/env bash
source /opt/ros/humble/setup.bash

mkdir -p /home/gold/folkrace_cartographer_ws/sweep_run
cp "{lua_wsl}" /home/gold/folkrace_cartographer_ws/sweep_run/sweep_temp.lua

pkill -9 -f cartographer_node 2>/dev/null || true
pkill -9 -f cartographer_occupancy_grid_node 2>/dev/null || true
rm -rf /dev/shm/* 2>/dev/null || true
sleep 0.3

nohup ros2 run cartographer_ros cartographer_node -configuration_directory /home/gold/folkrace_cartographer_ws/sweep_run -configuration_basename sweep_temp.lua --ros-args -p use_sim_time:=true --remap scan:=/scan --remap imu:=/imu --remap landmark:=/landmark > /tmp/carto_sweep.log 2>&1 &
nohup ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution {cfg['resolution']} -publish_period_sec 0.2 --ros-args -p use_sim_time:=true > /tmp/grid_sweep.log 2>&1 &
sleep 1.2

export USE_ODOMETRY=0

python3 "{feeder_wsl}" "{scan_jsonl_wsl}"

ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{{trajectory_id: 0}}" 2>/dev/null || true

python3 "{exporter_wsl}" "{out_png_wsl}"

pkill -9 -f cartographer_node 2>/dev/null || true
pkill -9 -f cartographer_occupancy_grid_node 2>/dev/null || true
"""
        sh_win = os.path.abspath(r"ros2_cartographer\scripts\run_temp_sweep.sh")
        os.makedirs(os.path.dirname(sh_win), exist_ok=True)
        with open(sh_win, "w", encoding="utf-8", newline="\n") as f:
            f.write(sh_content)
        sh_wsl = to_wsl_path(sh_win)

        res = subprocess.run(["wsl", "-e", "bash", sh_wsl], text=True, capture_output=True)

        if os.path.exists(out_png_win) and os.path.getsize(out_png_win) > 5000:
            print(f"     [SUCCESS] -> {os.path.basename(out_png_win)} ({os.path.getsize(out_png_win)/1024.0:.1f} KB)\n")
        else:
            print(f"     [ERROR] Failed to generate {os.path.basename(out_png_win)}")
            if res.stderr.strip():
                print("--- Bash Stderr ---")
                print(res.stderr.strip())
            print()

    print("=" * 80)
    print(f"SWEEP COMPLETE! All maps generated in: {out_dir_abs}")
    print("=" * 80)


if __name__ == "__main__":
    main()