#!/usr/bin/env python3
"""
================================================================================
  15-CONFIGURATION PARAMETER SWEEP FOR CARTOGRAPHER 2D SLAM
  Tests varying Ceres scan matcher weights, loop closure thresholds, submap sizes,
  correlative search windows, and odometry trust weights.
================================================================================
"""

import os
import sys
import time
import subprocess

CONFIGS = [
    # 1. Baseline High-Trust Ceres Scan Matcher
    {
        "id": "cfg01_baseline_high_ceres",
        "desc": "High Ceres Occupied Space Weight (40.0) + Large Search Window (20cm, 25deg)",
        "num_submaps": 60,
        "ceres_occ": 40.0,
        "ceres_trans": 10.0,
        "ceres_rot": 40.0,
        "search_lin": 0.20,
        "search_ang": 25.0,
        "opt_nodes": 15,
        "loop_score": 0.60,
        "loop_global_score": 0.65,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
    },
    # 2. Heavy Odometry Prior
    {
        "id": "cfg02_heavy_odom_prior",
        "desc": "Dominant Odometry Weight (1e6) + Modest Ceres (15.0)",
        "num_submaps": 60,
        "ceres_occ": 15.0,
        "ceres_trans": 5.0,
        "ceres_rot": 10.0,
        "search_lin": 0.10,
        "search_ang": 10.0,
        "opt_nodes": 20,
        "loop_score": 0.65,
        "loop_global_score": 0.70,
        "use_odom": "true",
        "odom_rot_w": 1e6,
        "odom_trans_w": 1e6,
    },
    # 3. Small Rapid Submaps (30 scans)
    {
        "id": "cfg03_rapid_small_submaps",
        "desc": "Small Submaps (30 scans) for Fast Loop Closures",
        "num_submaps": 30,
        "ceres_occ": 30.0,
        "ceres_trans": 10.0,
        "ceres_rot": 30.0,
        "search_lin": 0.15,
        "search_ang": 20.0,
        "opt_nodes": 10,
        "loop_score": 0.55,
        "loop_global_score": 0.60,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
    },
    # 4. Large Stable Submaps (120 scans)
    {
        "id": "cfg04_large_stable_submaps",
        "desc": "Large Submaps (120 scans) for Maximum Intra-Submap Rigidity",
        "num_submaps": 120,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.15,
        "search_ang": 15.0,
        "opt_nodes": 5,
        "loop_score": 0.65,
        "loop_global_score": 0.70,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
    },
    # 5. Aggressive Loop Closure (min_score 0.45, opt_every_5_nodes)
    {
        "id": "cfg05_aggressive_loop_closure",
        "desc": "Aggressive Pose Graph Optimization (min_score 0.45, optimize every 5 nodes)",
        "num_submaps": 45,
        "ceres_occ": 25.0,
        "ceres_trans": 10.0,
        "ceres_rot": 25.0,
        "search_lin": 0.25,
        "search_ang": 30.0,
        "opt_nodes": 5,
        "loop_score": 0.45,
        "loop_global_score": 0.50,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
    },
    # 6. Ultra-Strict Loop Closure (min_score 0.80)
    {
        "id": "cfg06_strict_loop_closure",
        "desc": "Strict Loop Closure (min_score 0.80) to Reject False Associations",
        "num_submaps": 60,
        "ceres_occ": 30.0,
        "ceres_trans": 10.0,
        "ceres_rot": 30.0,
        "search_lin": 0.10,
        "search_ang": 15.0,
        "opt_nodes": 20,
        "loop_score": 0.80,
        "loop_global_score": 0.85,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
    },
    # 7. Pure Correlative Scan Matching (No Odometry)
    {
        "id": "cfg07_pure_scan_match_no_odom",
        "desc": "Pure Real-Time Correlative Scan Matcher (use_odometry = false)",
        "num_submaps": 50,
        "ceres_occ": 50.0,
        "ceres_trans": 20.0,
        "ceres_rot": 50.0,
        "search_lin": 0.30,
        "search_ang": 35.0,
        "opt_nodes": 10,
        "loop_score": 0.55,
        "loop_global_score": 0.60,
        "use_odom": "false",
        "odom_rot_w": 1e3,
        "odom_trans_w": 1e3,
    },
    # 8. High Rotation Penalty (Rot Weight 100.0)
    {
        "id": "cfg08_heavy_rotation_penalty",
        "desc": "Heavy Rotation Penalty (Ceres Rot Weight 100.0) to Lock Heading",
        "num_submaps": 60,
        "ceres_occ": 25.0,
        "ceres_trans": 10.0,
        "ceres_rot": 100.0,
        "search_lin": 0.15,
        "search_ang": 10.0,
        "opt_nodes": 15,
        "loop_score": 0.60,
        "loop_global_score": 0.65,
        "use_odom": "true",
        "odom_rot_w": 5e5,
        "odom_trans_w": 1e5,
    },
    # 9. Wide Correlative Search Window (35cm, 45deg)
    {
        "id": "cfg09_wide_correlative_window",
        "desc": "Wide Real-Time Correlative Window (35cm linear, 45deg angular)",
        "num_submaps": 50,
        "ceres_occ": 30.0,
        "ceres_trans": 10.0,
        "ceres_rot": 30.0,
        "search_lin": 0.35,
        "search_ang": 45.0,
        "opt_nodes": 10,
        "loop_score": 0.55,
        "loop_global_score": 0.60,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
    },
    # 10. High-Precision Grid (10 mm resolution)
    {
        "id": "cfg10_ultra_fine_10mm_res",
        "desc": "Ultra-Fine 10 mm Resolution Grid Model",
        "num_submaps": 60,
        "ceres_occ": 30.0,
        "ceres_trans": 10.0,
        "ceres_rot": 30.0,
        "search_lin": 0.15,
        "search_ang": 15.0,
        "opt_nodes": 15,
        "loop_score": 0.60,
        "loop_global_score": 0.65,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
        "resolution": 0.010,
    },
    # 11. Coarser Fast-Snapping Grid (30 mm resolution)
    {
        "id": "cfg11_coarse_snap_30mm_res",
        "desc": "Coarse 30 mm Resolution Grid Model for Fast Basin Convergence",
        "num_submaps": 60,
        "ceres_occ": 30.0,
        "ceres_trans": 10.0,
        "ceres_rot": 30.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 15,
        "loop_score": 0.55,
        "loop_global_score": 0.60,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
        "resolution": 0.030,
    },
    # 12. Fast Probability Accumulation (Hit 0.70, Miss 0.40)
    {
        "id": "cfg12_fast_hit_prob_accumulation",
        "desc": "Fast Hit Probability (Hit=0.70, Miss=0.40) for Sharp Solid Walls",
        "num_submaps": 60,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.15,
        "search_ang": 15.0,
        "opt_nodes": 15,
        "loop_score": 0.60,
        "loop_global_score": 0.65,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
        "hit_prob": 0.70,
        "miss_prob": 0.40,
    },
    # 13. Strong Pose Graph Huber Regularization
    {
        "id": "cfg13_strong_huber_regularization",
        "desc": "Strong Huber Loss Regularization (Huber Scale 1.0) to Suppress Outliers",
        "num_submaps": 60,
        "ceres_occ": 30.0,
        "ceres_trans": 10.0,
        "ceres_rot": 30.0,
        "search_lin": 0.15,
        "search_ang": 20.0,
        "opt_nodes": 10,
        "loop_score": 0.55,
        "loop_global_score": 0.60,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
        "huber_scale": 1.0,
    },
    # 14. Global 360-Degree Fast Correlative Matcher
    {
        "id": "cfg14_full_360_global_search",
        "desc": "Full 360-Degree Fast Correlative Global Loop Matcher",
        "num_submaps": 40,
        "ceres_occ": 30.0,
        "ceres_trans": 10.0,
        "ceres_rot": 30.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 8,
        "loop_score": 0.50,
        "loop_global_score": 0.55,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
        "global_search_window": 6.0,
    },
    # 15. Optimal Tuned Racing Balanced Preset
    {
        "id": "cfg15_optimal_racing_balanced",
        "desc": "Optimal Tuned Racing Balanced Preset (Submaps 50, Ceres 35/10/40, Odom 5e5, Opt 12)",
        "num_submaps": 50,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 40.0,
        "search_lin": 0.15,
        "search_ang": 20.0,
        "opt_nodes": 12,
        "loop_score": 0.58,
        "loop_global_score": 0.62,
        "use_odom": "true",
        "odom_rot_w": 5e5,
        "odom_trans_w": 2e5,
    }
]

def generate_lua(cfg):
    res = cfg.get("resolution", 0.02)
    hit_prob = cfg.get("hit_prob", 0.58)
    miss_prob = cfg.get("miss_prob", 0.46)
    huber = cfg.get("huber_scale", 10.0)
    global_search = cfg.get("global_search_window", 4.0)

    lua_str = f"""include "map_builder.lua"
include "trajectory_builder.lua"

options = {{
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = {cfg['use_odom']},
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
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

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 4.50
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 0.0
TRAJECTORY_BUILDER_2D.use_imu_data = false

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = {cfg['ceres_occ']}
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = {cfg['ceres_trans']}
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = {cfg['ceres_rot']}

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = {cfg['search_lin']}
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad({cfg['search_ang']})
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

TRAJECTORY_BUILDER_2D.submaps.num_range_data = {cfg['num_submaps']}
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = {res}
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = {hit_prob}
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = {miss_prob}

POSE_GRAPH.optimize_every_n_nodes = {cfg['opt_nodes']}
POSE_GRAPH.constraint_builder.min_score = {cfg['loop_score']}
POSE_GRAPH.constraint_builder.global_localization_min_score = {cfg['loop_global_score']}
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = {global_search}
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(180.)

POSE_GRAPH.optimization_problem.huber_scale = {huber}
POSE_GRAPH.optimization_problem.odometry_translation_weight = {cfg['odom_trans_w']}
POSE_GRAPH.optimization_problem.odometry_rotation_weight = {cfg['odom_rot_w']}

return options
"""
    return lua_str

def main():
    print(f"[SWEEP] Starting 15-configuration Cartographer parameter benchmark sweep...")
    results_dir = r"/mnt/c/Users/nurmi/Documents/Da Winna/maps/cyan_champion_clean_static/sweep_results"
    
    for idx, cfg in enumerate(CONFIGS):
        cfg_id = cfg["id"]
        lua_filename = f"{cfg_id}.lua"
        lua_path = f"/home/gold/folkrace_cartographer_ws/param_sweep/{lua_filename}"
        png_out_path = f"{results_dir}/{cfg_id}.png"
        
        print(f"\n================================================================================")
        print(f"[{idx+1}/15] Running {cfg_id}")
        print(f"       Description: {cfg['desc']}")
        print(f"================================================================================")

        # 1. Write Lua
        with open(lua_path, "w", encoding="utf-8") as f:
            f.write(generate_lua(cfg))

        # 2. Kill existing Cartographer nodes (do NOT kill python3 so the runner survives)
        subprocess.run("pkill -9 -f cartographer_node; pkill -9 -f cartographer_occupancy_grid_node; sleep 0.5", shell=True)

        # 3. Launch Cartographer nodes with this config
        res = cfg.get("resolution", 0.02)
        carto_cmd = f"nohup bash -c 'source /opt/ros/humble/local_setup.bash && ros2 run cartographer_ros cartographer_node -configuration_directory /home/gold/folkrace_cartographer_ws/param_sweep -configuration_basename {lua_filename} --ros-args --remap scan:=/scan --remap odom:=/odom' > /tmp/carto_sweep.log 2>&1 &"
        grid_cmd = f"nohup bash -c 'source /opt/ros/humble/local_setup.bash && ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution {res} -publish_period_sec 0.5' > /tmp/grid_sweep.log 2>&1 &"
        
        subprocess.run(carto_cmd, shell=True)
        subprocess.run(grid_cmd, shell=True)
        time.sleep(1.5)

        # 4. Feed live Webots stream
        feed_cmd = "bash -c 'source /opt/ros/humble/local_setup.bash && python3 ~/folkrace_cartographer_ws/scripts/feed_cyberbotics_native_to_cartographer.py'"
        subprocess.run(feed_cmd, shell=True)

        # 5. Export PNG
        export_cmd = f"bash -c 'source /opt/ros/humble/local_setup.bash && python3 ~/folkrace_cartographer_ws/scripts/export_cartographer_png.py \"{png_out_path}\"'"
        subprocess.run(export_cmd, shell=True)
        print(f"[SAVED] -> {png_out_path}")

    print("\n[SWEEP COMPLETE] All 15 configurations rendered successfully.")

if __name__ == '__main__':
    main()
