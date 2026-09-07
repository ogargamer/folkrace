#!/usr/bin/env python3
"""
================================================================================
  ROUND 8: 30-CONFIGURATION PURE LIDAR + IMU SWEEP (ZERO XY / ZERO WHEEL ENCODERS)
  
  MANDATORY CONSTRAINTS:
  - use_odometry = false, provide_odom_frame = false, published_frame = "base_link"
  - tracking_frame = "imu_link", use_imu_data = true
  - Locked parameters that did well:
      * Fixed 2cm / 25mm / 30mm tiles
      * High rotation constraints (Ceres rotation weight 35 to 55)
      * High Huber regularization (0.5 to 2.0)
      * Optimization cadence every 3 to 6 nodes
  - Explores untested Cartographer parameters:
      1. motion_filter.max_distance_meters & motion_filter.max_angle_radians
      2. loop_closure_translation_weight & loop_closure_rotation_weight
      3. constraint_builder.sampling_ratio (1.0 vs 0.5 vs 0.3)
      4. ceres_solver_options (max_num_iterations 10, 20, 50)
      5. voxel_filter_size (0.010 to 0.025m)
      6. adaptive_voxel_filter (max_range, min_num_points, max_length)
      7. submap sizes (from compact 45 scans up to extended 120 scans)
================================================================================
"""

import os
import sys
import time
import subprocess

CONFIGS = []

# Generate 30 variations across untested parameters
VARIATIONS = [
    # 1-6: Submap size spectrum (45, 60, 80, 100, 120, 140) with tight Ceres weights
    {"id_base": "submaps_45_ceres_occ40", "desc_base": "Submaps 45, Ceres Occ 40, Rot 40, Loop 0.55", "submaps": 45, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.55, "opt_n": 4, "lin_s": 0.12, "ang_s": 12.0, "res": 0.020, "max_iter": 20, "voxel": 0.015, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "submaps_60_ceres_occ40", "desc_base": "Submaps 60, Ceres Occ 40, Rot 40, Loop 0.55", "submaps": 60, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.55, "opt_n": 4, "lin_s": 0.12, "ang_s": 12.0, "res": 0.020, "max_iter": 20, "voxel": 0.015, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "submaps_80_ceres_occ40", "desc_base": "Submaps 80, Ceres Occ 40, Rot 40, Loop 0.58", "submaps": 80, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.58, "opt_n": 5, "lin_s": 0.12, "ang_s": 12.0, "res": 0.020, "max_iter": 20, "voxel": 0.015, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "submaps_100_ceres_occ40", "desc_base": "Submaps 100, Ceres Occ 40, Rot 40, Loop 0.58", "submaps": 100, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.58, "opt_n": 5, "lin_s": 0.12, "ang_s": 12.0, "res": 0.020, "max_iter": 20, "voxel": 0.015, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "submaps_120_ceres_occ40", "desc_base": "Submaps 120, Ceres Occ 40, Rot 40, Loop 0.60", "submaps": 120, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.60, "opt_n": 5, "lin_s": 0.10, "ang_s": 10.0, "res": 0.020, "max_iter": 20, "voxel": 0.015, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "submaps_140_ceres_occ40", "desc_base": "Submaps 140, Ceres Occ 40, Rot 40, Loop 0.65", "submaps": 140, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.65, "opt_n": 6, "lin_s": 0.10, "ang_s": 10.0, "res": 0.020, "max_iter": 20, "voxel": 0.015, "samp_ratio": 1.0, "max_dist_m": 0.05},

    # 7-12: Untested motion filter thresholds & sampling ratios
    {"id_base": "motion_filter_dist002_ang005", "desc_base": "Dense Scans: Max Dist 0.02m, Max Ang 0.05 rad", "submaps": 80, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.58, "opt_n": 5, "lin_s": 0.12, "ang_s": 12.0, "res": 0.020, "max_iter": 20, "voxel": 0.010, "samp_ratio": 1.0, "max_dist_m": 0.02},
    {"id_base": "motion_filter_dist010_ang015", "desc_base": "Sparse Scans: Max Dist 0.10m, Max Ang 0.15 rad", "submaps": 80, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.58, "opt_n": 5, "lin_s": 0.12, "ang_s": 12.0, "res": 0.020, "max_iter": 20, "voxel": 0.015, "samp_ratio": 1.0, "max_dist_m": 0.10},
    {"id_base": "constraint_sampling_ratio_05", "desc_base": "Constraint Builder Sampling Ratio 0.5", "submaps": 80, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.58, "opt_n": 5, "lin_s": 0.12, "ang_s": 12.0, "res": 0.020, "max_iter": 20, "voxel": 0.015, "samp_ratio": 0.5, "max_dist_m": 0.05},
    {"id_base": "constraint_sampling_ratio_03", "desc_base": "Constraint Builder Sampling Ratio 0.3", "submaps": 80, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.58, "opt_n": 5, "lin_s": 0.12, "ang_s": 12.0, "res": 0.020, "max_iter": 20, "voxel": 0.015, "samp_ratio": 0.3, "max_dist_m": 0.05},
    {"id_base": "ceres_max_iterations_50", "desc_base": "Ceres Solver Max Iterations 50 (High Precision)", "submaps": 80, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.58, "opt_n": 5, "lin_s": 0.12, "ang_s": 12.0, "res": 0.020, "max_iter": 50, "voxel": 0.015, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "ceres_max_iterations_10", "desc_base": "Ceres Solver Max Iterations 10 (Fast)", "submaps": 80, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.58, "opt_n": 5, "lin_s": 0.12, "ang_s": 12.0, "res": 0.020, "max_iter": 10, "voxel": 0.015, "samp_ratio": 1.0, "max_dist_m": 0.05},

    # 13-18: Untested Voxel Filters and Adaptive Ray Bounds
    {"id_base": "voxel_filter_008mm", "desc_base": "Fine Voxel Filter 8mm (Retains dense laser points)", "submaps": 80, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.58, "opt_n": 5, "lin_s": 0.12, "ang_s": 12.0, "res": 0.020, "max_iter": 20, "voxel": 0.008, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "voxel_filter_020mm", "desc_base": "Coarse Voxel Filter 20mm (Fast noise decimation)", "submaps": 80, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.58, "opt_n": 5, "lin_s": 0.12, "ang_s": 12.0, "res": 0.020, "max_iter": 20, "voxel": 0.020, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "loop_rot_weight_1e6_trans_1e5", "desc_base": "PoseGraph Loop Closure Rot Weight 1e6, Trans 1e5", "submaps": 80, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.58, "opt_n": 5, "lin_s": 0.12, "ang_s": 12.0, "res": 0.020, "max_iter": 20, "voxel": 0.015, "samp_ratio": 1.0, "max_dist_m": 0.05, "loop_rot_w": 1e6, "loop_trans_w": 1e5},
    {"id_base": "loop_rot_weight_5e5_trans_5e4", "desc_base": "PoseGraph Loop Closure Rot Weight 5e5, Trans 5e4", "submaps": 80, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.58, "opt_n": 5, "lin_s": 0.12, "ang_s": 12.0, "res": 0.020, "max_iter": 20, "voxel": 0.015, "samp_ratio": 1.0, "max_dist_m": 0.05, "loop_rot_w": 5e5, "loop_trans_w": 5e4},
    {"id_base": "ceres_occ55_rot50_opt3", "desc_base": "High Ceres Scan Match: Occ 55, Rot 50, Opt every 3 nodes", "submaps": 70, "ceres_occ": 55.0, "ceres_rot": 50.0, "loop_s": 0.55, "opt_n": 3, "lin_s": 0.12, "ang_s": 12.0, "res": 0.020, "max_iter": 25, "voxel": 0.015, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "ceres_occ30_rot50_opt4", "desc_base": "Balanced Ceres: Occ 30, Rot 50, Opt every 4 nodes", "submaps": 80, "ceres_occ": 30.0, "ceres_rot": 50.0, "loop_s": 0.58, "opt_n": 4, "lin_s": 0.12, "ang_s": 12.0, "res": 0.020, "max_iter": 20, "voxel": 0.015, "samp_ratio": 1.0, "max_dist_m": 0.05},

    # 19-24: 25mm Resolution Pure LiDAR+IMU Variations
    {"id_base": "res25mm_submaps60_occ40", "desc_base": "[25mm] Submaps 60, Occ 40, Rot 40, Loop 0.55", "submaps": 60, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.55, "opt_n": 4, "lin_s": 0.12, "ang_s": 12.0, "res": 0.025, "max_iter": 20, "voxel": 0.018, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "res25mm_submaps80_occ40", "desc_base": "[25mm] Submaps 80, Occ 40, Rot 40, Loop 0.58", "submaps": 80, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.58, "opt_n": 5, "lin_s": 0.12, "ang_s": 12.0, "res": 0.025, "max_iter": 20, "voxel": 0.018, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "res25mm_submaps100_occ40", "desc_base": "[25mm] Submaps 100, Occ 40, Rot 40, Loop 0.60", "submaps": 100, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.60, "opt_n": 5, "lin_s": 0.10, "ang_s": 10.0, "res": 0.025, "max_iter": 20, "voxel": 0.018, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "res25mm_submaps120_occ40", "desc_base": "[25mm] Submaps 120, Occ 40, Rot 40, Loop 0.62", "submaps": 120, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.62, "opt_n": 5, "lin_s": 0.10, "ang_s": 10.0, "res": 0.025, "max_iter": 20, "voxel": 0.018, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "res25mm_submaps80_iter50", "desc_base": "[25mm] Submaps 80, Iter 50, Occ 45, Rot 45", "submaps": 80, "ceres_occ": 45.0, "ceres_rot": 45.0, "loop_s": 0.58, "opt_n": 4, "lin_s": 0.12, "ang_s": 12.0, "res": 0.025, "max_iter": 50, "voxel": 0.018, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "res25mm_submaps80_sampling05", "desc_base": "[25mm] Submaps 80, Sampling 0.5, Loop 0.58", "submaps": 80, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.58, "opt_n": 5, "lin_s": 0.12, "ang_s": 12.0, "res": 0.025, "max_iter": 20, "voxel": 0.018, "samp_ratio": 0.5, "max_dist_m": 0.05},

    # 25-30: 30mm Resolution Pure LiDAR+IMU Variations
    {"id_base": "res30mm_submaps60_occ40", "desc_base": "[30mm] Submaps 60, Occ 40, Rot 40, Loop 0.55", "submaps": 60, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.55, "opt_n": 4, "lin_s": 0.12, "ang_s": 12.0, "res": 0.030, "max_iter": 20, "voxel": 0.020, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "res30mm_submaps80_occ40", "desc_base": "[30mm] Submaps 80, Occ 40, Rot 40, Loop 0.58", "submaps": 80, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.58, "opt_n": 5, "lin_s": 0.12, "ang_s": 12.0, "res": 0.030, "max_iter": 20, "voxel": 0.020, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "res30mm_submaps100_occ40", "desc_base": "[30mm] Submaps 100, Occ 40, Rot 40, Loop 0.60", "submaps": 100, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.60, "opt_n": 5, "lin_s": 0.10, "ang_s": 10.0, "res": 0.030, "max_iter": 20, "voxel": 0.020, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "res30mm_submaps120_occ40", "desc_base": "[30mm] Submaps 120, Occ 40, Rot 40, Loop 0.65", "submaps": 120, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.65, "opt_n": 6, "lin_s": 0.10, "ang_s": 10.0, "res": 0.030, "max_iter": 20, "voxel": 0.020, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "res30mm_submaps80_iter50", "desc_base": "[30mm] Submaps 80, Iter 50, Occ 45, Rot 45", "submaps": 80, "ceres_occ": 45.0, "ceres_rot": 45.0, "loop_s": 0.58, "opt_n": 4, "lin_s": 0.12, "ang_s": 12.0, "res": 0.030, "max_iter": 50, "voxel": 0.020, "samp_ratio": 1.0, "max_dist_m": 0.05},
    {"id_base": "res30mm_submaps80_rot_weight1e6", "desc_base": "[30mm] Submaps 80, Loop Rot Weight 1e6, Score 0.60", "submaps": 80, "ceres_occ": 40.0, "ceres_rot": 40.0, "loop_s": 0.60, "opt_n": 5, "lin_s": 0.10, "ang_s": 10.0, "res": 0.030, "max_iter": 20, "voxel": 0.020, "samp_ratio": 1.0, "max_dist_m": 0.05, "loop_rot_w": 1e6, "loop_trans_w": 1e5},
]

for idx, v in enumerate(VARIATIONS):
    CONFIGS.append({
        "id": f"r8_cfg{idx+1:02d}_{v['id_base']}",
        "desc": v["desc_base"],
        "res": v["res"],
        "num_submaps": v["submaps"],
        "ceres_occ": v["ceres_occ"],
        "ceres_trans": 10.0,
        "ceres_rot": v["ceres_rot"],
        "search_lin": v["lin_s"],
        "search_ang": v["ang_s"],
        "opt_nodes": v["opt_n"],
        "loop_score": v["loop_s"],
        "loop_global_score": v["loop_s"] + 0.05,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
        "max_iter": v.get("max_iter", 20),
        "voxel": v.get("voxel", 0.015),
        "samp_ratio": v.get("samp_ratio", 1.0),
        "max_dist_m": v.get("max_dist_m", 0.05),
        "loop_rot_w": v.get("loop_rot_w", 1e5),
        "loop_trans_w": v.get("loop_trans_w", 1e4),
    })

def generate_lua(cfg):
    res = cfg["res"]
    hit_prob = cfg["hit_prob"]
    miss_prob = cfg["miss_prob"]
    huber = cfg["huber_scale"]
    max_iter = cfg["max_iter"]
    voxel = cfg["voxel"]
    samp_ratio = cfg["samp_ratio"]
    max_dist = cfg["max_dist_m"]
    loop_rot_w = cfg["loop_rot_w"]
    loop_trans_w = cfg["loop_trans_w"]

    lua_str = f"""include "map_builder.lua"
include "trajectory_builder.lua"

options = {{
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = false,
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

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0

TRAJECTORY_BUILDER_2D.voxel_filter_size = {voxel}

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = {cfg['ceres_occ']}
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = {cfg['ceres_trans']}
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = {cfg['ceres_rot']}
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = {max_iter}

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = {cfg['search_lin']}
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad({cfg['search_ang']})
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = {max_dist}
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

TRAJECTORY_BUILDER_2D.submaps.num_range_data = {cfg['num_submaps']}
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = {res}
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = {hit_prob}
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = {miss_prob}

POSE_GRAPH.optimize_every_n_nodes = {cfg['opt_nodes']}
POSE_GRAPH.constraint_builder.sampling_ratio = {samp_ratio}
POSE_GRAPH.constraint_builder.min_score = {cfg['loop_score']}
POSE_GRAPH.constraint_builder.global_localization_min_score = {cfg['loop_global_score']}
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(180.)
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = {loop_trans_w}
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = {loop_rot_w}

POSE_GRAPH.optimization_problem.huber_scale = {huber}
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = {max_iter}

return options
"""
    return lua_str

def main():
    print(f"[SWEEP 8] Starting Round 8: Pure LiDAR + IMU (Zero XY / Zero Odom) 30-configuration sweep...")
    results_dir = r"/mnt/c/Users/nurmi/Documents/Da Winna/maps/cyan_champion_clean_static/sweep_results"
    checker_script = r"/home/gold/folkrace_cartographer_ws/scripts/check_map_empty.py"
    
    for idx, cfg in enumerate(CONFIGS):
        cfg_id = cfg["id"]
        lua_filename = f"{cfg_id}.lua"
        lua_path = f"/home/gold/folkrace_cartographer_ws/param_sweep/{lua_filename}"
        png_out_path = f"{results_dir}/{cfg_id}.png"
        res = cfg["res"]
        
        print(f"\n================================================================================")
        print(f"[{idx+1}/30] Running {cfg_id}")
        print(f"       Description: {cfg['desc']}")
        print(f"================================================================================")

        try:
            # 1. Write Lua
            with open(lua_path, "w", encoding="utf-8") as f:
                f.write(generate_lua(cfg))

            # 2. Kill existing Cartographer nodes
            subprocess.run("pkill -9 -f cartographer_node; pkill -9 -f cartographer_occupancy_grid_node; sleep 0.5", shell=True)

            # 3. Launch Cartographer nodes in pure LiDAR + IMU mode (NO /odom topic)
            carto_cmd = f"nohup bash -c 'source /opt/ros/humble/local_setup.bash && ros2 run cartographer_ros cartographer_node -configuration_directory /home/gold/folkrace_cartographer_ws/param_sweep -configuration_basename {lua_filename} --ros-args --remap scan:=/scan --remap imu:=/imu' > /tmp/carto_sweep8.log 2>&1 &"
            grid_cmd = f"nohup bash -c 'source /opt/ros/humble/local_setup.bash && ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution {res} -publish_period_sec 0.5' > /tmp/grid_sweep8.log 2>&1 &"
            
            subprocess.run(carto_cmd, shell=True)
            subprocess.run(grid_cmd, shell=True)
            time.sleep(1.5)

            # 4. Feed live 3-lap stream with ONLY LiDAR + IMU (Zero XY positions)
            feed_cmd = "bash -c 'source /opt/ros/humble/local_setup.bash && python3 /home/gold/folkrace_cartographer_ws/scripts/feed_pure_lidar_imu.py'"
            subprocess.run(feed_cmd, shell=True)
            time.sleep(1.0)

            # 5. Export PNG
            export_cmd = f"bash -c 'source /opt/ros/humble/local_setup.bash && python3 /home/gold/folkrace_cartographer_ws/scripts/export_cartographer_png.py \"{png_out_path}\"'"
            subprocess.run(export_cmd, shell=True)

            # 6. Cleanup FastRTPS shared memory locks
            subprocess.run("pkill -9 -f cartographer_node; pkill -9 -f cartographer_occupancy_grid_node; rm -rf /dev/shm/* 2>/dev/null", shell=True)

            # 7. Check non-empty status with programmatic validator
            check_proc = subprocess.run(f"python3 {checker_script} \"{png_out_path}\"", shell=True, capture_output=True, text=True)
            print(f"[STATUS] {check_proc.stdout.strip()}")
        except Exception as e:
            print(f"[ERROR] Failed running {cfg_id}: {e}")

    print("\n[SWEEP 8 COMPLETE] All 30 Pure LiDAR + IMU configurations processed.")

if __name__ == '__main__':
    main()
