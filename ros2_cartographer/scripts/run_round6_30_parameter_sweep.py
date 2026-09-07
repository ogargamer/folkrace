#!/usr/bin/env python3
"""
================================================================================
  ROUND 6: 30-CONFIGURATION PARAMETER SWEEP
  FOCUSED DIRECTLY ON COMMON STRONG ASPECTS:
  1. Large submap scan count (50 to 140 scans per submap).
  2. High odometry/IMU rotation weights (1e5, 3e5, 5e5, 8e5, 1e6).
  3. Moderate-to-strict loop closure thresholds (0.48 to 0.75).
  4. Tight correlative search windows (linear 0.10 to 0.18m, angular 12 to 18 deg).
  5. Optimization cadence every 4 to 6 nodes.
  6. Strict 2cm tiles (resolution = 0.020) and Rock-Solid IMU tracking.
  7. Hit/Miss probability combinations to fine-tune wall crispness.
================================================================================
"""

import os
import sys
import time
import subprocess

CONFIGS = []

# Group 1: Submaps 50-60, Odom Rot 3e5 - 8e5, Loop 0.48 - 0.55
for (submaps, odom_rot, loop_s, hit_p, miss_p, opt_n, lin_s, ang_s) in [
    (50, 3e5, 0.48, 0.70, 0.42, 4, 0.15, 15.0),
    (50, 5e5, 0.48, 0.70, 0.42, 4, 0.15, 15.0),
    (50, 5e5, 0.50, 0.70, 0.42, 4, 0.15, 15.0),
    (50, 5e5, 0.52, 0.72, 0.40, 4, 0.15, 15.0),
    (50, 8e5, 0.48, 0.70, 0.42, 4, 0.15, 15.0),
    (50, 8e5, 0.52, 0.70, 0.42, 4, 0.12, 14.0),
    (60, 3e5, 0.48, 0.70, 0.42, 4, 0.15, 15.0),
    (60, 5e5, 0.48, 0.70, 0.42, 4, 0.15, 15.0),
    (60, 5e5, 0.52, 0.72, 0.40, 4, 0.15, 15.0),
    (60, 8e5, 0.50, 0.70, 0.42, 4, 0.12, 14.0),
]:
    CONFIGS.append({
        "id": f"r6_cfg{len(CONFIGS)+1:02d}_large_submap_group1",
        "desc": f"Group 1 (Submaps {submaps}, OdomRot {odom_rot:.0e}, Loop {loop_s}, Hit {hit_p}, Miss {miss_p})",
        "num_submaps": submaps,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": lin_s,
        "search_ang": ang_s,
        "opt_nodes": opt_n,
        "loop_score": loop_s,
        "loop_global_score": loop_s + 0.05,
        "hit_prob": hit_p,
        "miss_prob": miss_p,
        "huber_scale": 1.0,
        "odom_trans_w": 1e5,
        "odom_rot_w": odom_rot,
    })

# Group 2: Submaps 70-90 (Extended rigidity), Odom Rot 2e5 - 1e6, Loop 0.50 - 0.65
for (submaps, odom_rot, loop_s, hit_p, miss_p, opt_n, lin_s, ang_s) in [
    (75, 3e5, 0.50, 0.70, 0.42, 4, 0.15, 15.0),
    (75, 5e5, 0.50, 0.70, 0.42, 4, 0.15, 15.0),
    (75, 5e5, 0.55, 0.72, 0.40, 4, 0.14, 14.0),
    (75, 8e5, 0.50, 0.70, 0.42, 4, 0.14, 14.0),
    (75, 1e6, 0.52, 0.70, 0.42, 4, 0.12, 12.0),
    (90, 3e5, 0.50, 0.70, 0.42, 5, 0.15, 15.0),
    (90, 5e5, 0.50, 0.70, 0.42, 5, 0.15, 15.0),
    (90, 5e5, 0.58, 0.74, 0.38, 5, 0.14, 14.0),
    (90, 8e5, 0.52, 0.70, 0.42, 5, 0.12, 12.0),
    (90, 1e6, 0.55, 0.70, 0.42, 5, 0.12, 12.0),
]:
    CONFIGS.append({
        "id": f"r6_cfg{len(CONFIGS)+1:02d}_extended_rigidity_group2",
        "desc": f"Group 2 (Submaps {submaps}, OdomRot {odom_rot:.0e}, Loop {loop_s}, Hit {hit_p}, Miss {miss_p})",
        "num_submaps": submaps,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": lin_s,
        "search_ang": ang_s,
        "opt_nodes": opt_n,
        "loop_score": loop_s,
        "loop_global_score": loop_s + 0.05,
        "hit_prob": hit_p,
        "miss_prob": miss_p,
        "huber_scale": 1.0,
        "odom_trans_w": 1e5,
        "odom_rot_w": odom_rot,
    })

# Group 3: Submaps 100-140 (Ultra-large submaps cfg04 style), Odom Rot 5e5 - 1e6, Strict Loop 0.55 - 0.75
for (submaps, odom_rot, loop_s, hit_p, miss_p, opt_n, lin_s, ang_s) in [
    (110, 5e5, 0.55, 0.70, 0.42, 5, 0.14, 14.0),
    (110, 5e5, 0.62, 0.72, 0.40, 5, 0.12, 12.0),
    (110, 8e5, 0.55, 0.70, 0.42, 5, 0.12, 12.0),
    (110, 1e6, 0.60, 0.70, 0.42, 5, 0.10, 12.0),
    (120, 5e5, 0.55, 0.70, 0.42, 5, 0.14, 14.0),
    (120, 5e5, 0.65, 0.72, 0.40, 5, 0.12, 12.0),
    (120, 8e5, 0.60, 0.70, 0.42, 5, 0.12, 12.0),
    (120, 1e6, 0.65, 0.70, 0.42, 6, 0.10, 10.0),
    (140, 5e5, 0.60, 0.70, 0.42, 6, 0.12, 12.0),
    (140, 1e6, 0.70, 0.70, 0.42, 6, 0.10, 10.0),
]:
    CONFIGS.append({
        "id": f"r6_cfg{len(CONFIGS)+1:02d}_ultra_large_group3",
        "desc": f"Group 3 (Submaps {submaps}, OdomRot {odom_rot:.0e}, Loop {loop_s}, Hit {hit_p}, Miss {miss_p})",
        "num_submaps": submaps,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": lin_s,
        "search_ang": ang_s,
        "opt_nodes": opt_n,
        "loop_score": loop_s,
        "loop_global_score": loop_s + 0.05,
        "hit_prob": hit_p,
        "miss_prob": miss_p,
        "huber_scale": 1.0,
        "odom_trans_w": 1e5,
        "odom_rot_w": odom_rot,
    })

def generate_lua(cfg):
    # NOTE: Strictly 2cm (0.020m) tiles for all configurations (DO NOT CHANGE).
    res = 0.020
    hit_prob = cfg.get("hit_prob", 0.58)
    miss_prob = cfg.get("miss_prob", 0.46)
    huber = cfg.get("huber_scale", 1.0)
    odom_trans_w = cfg.get("odom_trans_w", 1e5)
    odom_rot_w = cfg.get("odom_rot_w", 5e5)

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
    print(f"[SWEEP 6] Starting Round 6: 30-configuration Large-Submap / High-Heading Cartographer sweep...")
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
            carto_cmd = f"nohup bash -c 'source /opt/ros/humble/local_setup.bash && ros2 run cartographer_ros cartographer_node -configuration_directory /home/gold/folkrace_cartographer_ws/param_sweep -configuration_basename {lua_filename} --ros-args --remap scan:=/scan --remap odom:=/odom --remap imu:=/imu' > /tmp/carto_sweep6.log 2>&1 &"
            grid_cmd = f"nohup bash -c 'source /opt/ros/humble/local_setup.bash && ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.02 -publish_period_sec 0.5' > /tmp/grid_sweep6.log 2>&1 &"
            
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

    print("\n[SWEEP 6 COMPLETE] All 30 configurations processed.")

if __name__ == '__main__':
    main()
