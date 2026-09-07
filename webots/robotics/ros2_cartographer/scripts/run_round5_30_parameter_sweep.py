#!/usr/bin/env python3
"""
================================================================================
  ROUND 5: 30-CONFIGURATION PARAMETER SWEEP
  FOCUSED ARCHITECTURE:
  - Explores the configurations directly surrounding cfg08, cfg07, cfg03, and
    cartographer_cyberbotics_native_aligned.
  - Mandatory IMU + Odometry integration with rock-solid coordinate alignment.
  - Strict 2cm tiles (resolution = 0.020).
  - Sweeps Hit/Miss probabilities (pixel addition / clearing rates), Submaps sizes,
    Ceres Occupied / Rotation weights, and PoseGraph Optimization frequencies.
  - Validates non-empty output for all 30 configurations via check_map_empty.py.
================================================================================
"""

import os
import sys
import time
import subprocess

CONFIGS = []

# Base clusters centered on the top performers
# Cluster A: Around cfg08 (Pure Correlative Scan Matcher / Low odom weight, High Ceres 40/40, Submaps 35-50)
for idx, (hit_p, miss_p, submaps, occ_w, rot_w, opt_n, loop_s) in enumerate([
    (0.68, 0.44, 40, 40.0, 40.0, 4, 0.45),
    (0.70, 0.42, 40, 40.0, 40.0, 4, 0.45),
    (0.72, 0.40, 40, 40.0, 40.0, 4, 0.45),
    (0.75, 0.38, 40, 40.0, 40.0, 4, 0.45),
    (0.70, 0.42, 30, 40.0, 40.0, 3, 0.44),
    (0.70, 0.42, 50, 40.0, 40.0, 4, 0.46),
    (0.70, 0.42, 40, 45.0, 45.0, 4, 0.45),
    (0.70, 0.42, 40, 35.0, 35.0, 4, 0.45),
    (0.65, 0.45, 40, 40.0, 40.0, 4, 0.45),
    (0.78, 0.35, 40, 40.0, 40.0, 4, 0.45),
]):
    CONFIGS.append({
        "id": f"r5_cfg{len(CONFIGS)+1:02d}_cfg08_variant",
        "desc": f"cfg08-cluster (Submaps {submaps}, Hit {hit_p}, Miss {miss_p}, Occ {occ_w}, Opt {opt_n})",
        "num_submaps": submaps,
        "ceres_occ": occ_w,
        "ceres_trans": 12.0,
        "ceres_rot": rot_w,
        "search_lin": 0.24,
        "search_ang": 24.0,
        "opt_nodes": opt_n,
        "loop_score": loop_s,
        "loop_global_score": loop_s + 0.05,
        "hit_prob": hit_p,
        "miss_prob": miss_p,
        "huber_scale": 1.0,
        "use_imu": "true",
        "tracking_frame": "imu_link",
        "odom_trans_w": 1e3,
        "odom_rot_w": 1e3,
    })

# Cluster B: Around cfg07 (High Odometry Rotation Trust 5e5, Ceres 30-40, Submaps 45-60)
for idx, (hit_p, miss_p, submaps, rot_w, odom_rot, opt_n, loop_s) in enumerate([
    (0.68, 0.44, 50, 40.0, 5e5, 4, 0.48),
    (0.70, 0.42, 50, 40.0, 5e5, 4, 0.48),
    (0.72, 0.40, 50, 40.0, 5e5, 4, 0.48),
    (0.70, 0.42, 35, 40.0, 5e5, 3, 0.46),
    (0.70, 0.42, 60, 40.0, 5e5, 4, 0.48),
    (0.70, 0.42, 50, 50.0, 5e5, 4, 0.48),
    (0.70, 0.42, 50, 40.0, 2e5, 4, 0.48),
    (0.70, 0.42, 50, 40.0, 8e5, 4, 0.48),
    (0.75, 0.38, 50, 40.0, 5e5, 4, 0.48),
    (0.65, 0.45, 50, 40.0, 5e5, 4, 0.48),
]):
    CONFIGS.append({
        "id": f"r5_cfg{len(CONFIGS)+1:02d}_cfg07_variant",
        "desc": f"cfg07-cluster (OdomRot {odom_rot:.0e}, Submaps {submaps}, Hit {hit_p}, Miss {miss_p})",
        "num_submaps": submaps,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": rot_w,
        "search_lin": 0.18,
        "search_ang": 18.0,
        "opt_nodes": opt_n,
        "loop_score": loop_s,
        "loop_global_score": loop_s + 0.05,
        "hit_prob": hit_p,
        "miss_prob": miss_p,
        "huber_scale": 1.0,
        "use_imu": "true",
        "tracking_frame": "imu_link",
        "odom_trans_w": 1e5,
        "odom_rot_w": odom_rot,
    })

# Cluster C: Around cfg03 & cartographer_cyberbotics_native_aligned (High Ceres Occ 35-50, Huber 1.0, Opt 3-4, Score 0.46-0.50)
for idx, (hit_p, miss_p, submaps, occ_w, opt_n, huber, loop_s) in enumerate([
    (0.70, 0.42, 45, 38.0, 3, 1.0, 0.46),
    (0.72, 0.40, 45, 38.0, 3, 1.0, 0.46),
    (0.68, 0.44, 45, 38.0, 3, 1.0, 0.46),
    (0.70, 0.42, 35, 38.0, 2, 1.0, 0.44),
    (0.70, 0.42, 55, 38.0, 4, 1.0, 0.48),
    (0.70, 0.42, 45, 45.0, 3, 1.0, 0.46),
    (0.70, 0.42, 45, 38.0, 3, 0.5, 0.46),
    (0.70, 0.42, 45, 38.0, 3, 2.0, 0.46),
    (0.74, 0.38, 45, 42.0, 3, 1.0, 0.46),
    (0.70, 0.42, 45, 40.0, 3, 1.0, 0.50),
]):
    CONFIGS.append({
        "id": f"r5_cfg{len(CONFIGS)+1:02d}_native_aligned_variant",
        "desc": f"native_aligned-cluster (Submaps {submaps}, Occ {occ_w}, Huber {huber}, Opt {opt_n})",
        "num_submaps": submaps,
        "ceres_occ": occ_w,
        "ceres_trans": 10.0,
        "ceres_rot": 38.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": opt_n,
        "loop_score": loop_s,
        "loop_global_score": loop_s + 0.05,
        "hit_prob": hit_p,
        "miss_prob": miss_p,
        "huber_scale": huber,
        "use_imu": "true",
        "tracking_frame": "imu_link",
        "odom_trans_w": 1e4,
        "odom_rot_w": 1e4,
    })

def generate_lua(cfg):
    # NOTE: Strictly 2cm (0.020m) tiles for all configurations (DO NOT CHANGE).
    res = 0.020
    hit_prob = cfg.get("hit_prob", 0.58)
    miss_prob = cfg.get("miss_prob", 0.46)
    huber = cfg.get("huber_scale", 1.0)
    odom_trans_w = cfg.get("odom_trans_w", 1e3)
    odom_rot_w = cfg.get("odom_rot_w", 1e3)

    lua_str = f"""include "map_builder.lua"
include "trajectory_builder.lua"

options = {{
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  -- NOTE: Rock-solid IMU tracking frame
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
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

-- NOTE: MUST USE IMU (DO NOT USE PURE SCAN MATCHING)
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = {cfg['ceres_occ']}
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = {cfg['ceres_trans']}
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = {cfg['ceres_rot']}

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = {cfg['search_lin']}
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad({cfg['search_ang']})
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

TRAJECTORY_BUILDER_2D.submaps.num_range_data = {cfg['num_submaps']}
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
-- NOTE: Strictly 2cm tiles for all configurations (DO NOT CHANGE)
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.02
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = {hit_prob}
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = {miss_prob}

POSE_GRAPH.optimize_every_n_nodes = {cfg['opt_nodes']}
POSE_GRAPH.constraint_builder.min_score = {cfg['loop_score']}
POSE_GRAPH.constraint_builder.global_localization_min_score = {cfg['loop_global_score']}
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(180.)

POSE_GRAPH.optimization_problem.huber_scale = {huber}
POSE_GRAPH.optimization_problem.odometry_translation_weight = {odom_trans_w}
POSE_GRAPH.optimization_problem.odometry_rotation_weight = {odom_rot_w}

return options
"""
    return lua_str

def main():
    print(f"[SWEEP 5] Starting Round 5: 30-configuration IMU-ENABLED 2cm Cartographer sweep...")
    results_dir = r"/mnt/c/Users/nurmi/Documents/Da Winna/maps/cyan_champion_clean_static/sweep_results"
    checker_script = r"/home/gold/folkrace_cartographer_ws/scripts/check_map_empty.py"
    
    for idx, cfg in enumerate(CONFIGS):
        cfg_id = cfg["id"]
        lua_filename = f"{cfg_id}.lua"
        lua_path = f"/home/gold/folkrace_cartographer_ws/param_sweep/{lua_filename}"
        png_out_path = f"{results_dir}/{cfg_id}.png"
        
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

            # 3. Launch Cartographer nodes with IMU topic remapping (Strictly 2cm tiles)
            carto_cmd = f"nohup bash -c 'source /opt/ros/humble/local_setup.bash && ros2 run cartographer_ros cartographer_node -configuration_directory /home/gold/folkrace_cartographer_ws/param_sweep -configuration_basename {lua_filename} --ros-args --remap scan:=/scan --remap odom:=/odom --remap imu:=/imu' > /tmp/carto_sweep5.log 2>&1 &"
            grid_cmd = f"nohup bash -c 'source /opt/ros/humble/local_setup.bash && ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.02 -publish_period_sec 0.5' > /tmp/grid_sweep5.log 2>&1 &"
            
            subprocess.run(carto_cmd, shell=True)
            subprocess.run(grid_cmd, shell=True)
            time.sleep(1.5)

            # 4. Feed live Webots multi-lap stream with rock-solid IMU
            feed_cmd = "bash -c 'source /opt/ros/humble/local_setup.bash && python3 /home/gold/folkrace_cartographer_ws/scripts/feed_cyberbotics_rigid_imu.py'"
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

    print("\n[SWEEP 5 COMPLETE] All 30 IMU-enabled configurations processed.")

if __name__ == '__main__':
    main()
