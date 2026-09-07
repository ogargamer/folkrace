#!/usr/bin/env python3
"""
================================================================================
  ROUND 4: 20-CONFIGURATION PARAMETER SWEEP (STRICTLY IMU-ENABLED + 2CM TILES)
  
  MANDATORY USER CONSTRAINTS:
  - DO NOT DO PURE SCAN MATCHING (TRAJECTORY_BUILDER_2D.use_imu_data = true).
  - MUST USE IMU (tracking_frame = "imu_link").
  - 2cm (0.020m) tiles strictly for all configurations (DO NOT CHANGE).
  - Sweeps hit_probability (speed of adding pixels) and miss_probability (speed of erasing pixels).
  - Programmatic verification via check_map_empty.py to ensure zero empty/blank outputs.
================================================================================
"""

import os
import sys
import time
import subprocess

CONFIGS = [
    # 1. IMU Assisted - Default Probabilities (Hit 0.55, Miss 0.49)
    {
        "id": "r4_cfg01_imu_hit055_miss049",
        "desc": "IMU-Assisted - Hit 0.55, Miss 0.49 (Slow add/erase)",
        "num_submaps": 45,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 4,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "hit_prob": 0.55,
        "miss_prob": 0.49,
        "huber_scale": 1.0,
        "imu_gravity_w": 10.0,
    },
    # 2. IMU Assisted - Moderate Probabilities (Hit 0.65, Miss 0.45)
    {
        "id": "r4_cfg02_imu_hit065_miss045",
        "desc": "IMU-Assisted - Hit 0.65, Miss 0.45 (Moderate add/erase)",
        "num_submaps": 45,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 4,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "hit_prob": 0.65,
        "miss_prob": 0.45,
        "huber_scale": 1.0,
        "imu_gravity_w": 10.0,
    },
    # 3. IMU Assisted - Fast Hit Probabilities (Hit 0.72, Miss 0.42)
    {
        "id": "r4_cfg03_imu_hit072_miss042",
        "desc": "IMU-Assisted - Hit 0.72, Miss 0.42 (Fast crisp wall formation)",
        "num_submaps": 45,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 4,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "hit_prob": 0.72,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
        "imu_gravity_w": 10.0,
    },
    # 4. IMU Assisted - Instant Hit (Hit 0.82, Miss 0.38)
    {
        "id": "r4_cfg04_imu_hit082_miss038",
        "desc": "IMU-Assisted - Hit 0.82, Miss 0.38 (Instant 1-pass solid walls)",
        "num_submaps": 45,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 4,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "hit_prob": 0.82,
        "miss_prob": 0.38,
        "huber_scale": 1.0,
        "imu_gravity_w": 10.0,
    },
    # 5. IMU Assisted - Sticky Walls (Hit 0.70, Miss 0.48)
    {
        "id": "r4_cfg05_imu_sticky_hit070_miss048",
        "desc": "IMU-Assisted - Hit 0.70, Miss 0.48 (Hard to erase obstacles)",
        "num_submaps": 45,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 4,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "hit_prob": 0.70,
        "miss_prob": 0.48,
        "huber_scale": 1.0,
        "imu_gravity_w": 10.0,
    },
    # 6. IMU Assisted - Rapid Clearing (Hit 0.62, Miss 0.32)
    {
        "id": "r4_cfg06_imu_rapid_clear_hit062_miss032",
        "desc": "IMU-Assisted - Hit 0.62, Miss 0.32 (Aggressive track clearing)",
        "num_submaps": 45,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 4,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "hit_prob": 0.62,
        "miss_prob": 0.32,
        "huber_scale": 1.0,
        "imu_gravity_w": 10.0,
    },
    # 7. IMU Assisted - Rapid Submap Rotation (Submaps 30, Opt 3)
    {
        "id": "r4_cfg07_imu_submaps30_opt3",
        "desc": "IMU-Assisted - Submaps 30, Opt every 3 nodes",
        "num_submaps": 30,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.22,
        "search_ang": 22.0,
        "opt_nodes": 3,
        "loop_score": 0.45,
        "loop_global_score": 0.50,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
        "imu_gravity_w": 10.0,
    },
    # 8. IMU Assisted - Stable Submaps (Submaps 60)
    {
        "id": "r4_cfg08_imu_submaps60",
        "desc": "IMU-Assisted - Submaps 60 for High Intra-Submap Stability",
        "num_submaps": 60,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 4,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
        "imu_gravity_w": 10.0,
    },
    # 9. IMU Assisted - High Ceres Occupied Space (50.0)
    {
        "id": "r4_cfg09_imu_ceres_occ50",
        "desc": "IMU-Assisted - Ceres Occupied Weight 50.0, Rot Weight 40.0",
        "num_submaps": 45,
        "ceres_occ": 50.0,
        "ceres_trans": 10.0,
        "ceres_rot": 40.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 4,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
        "imu_gravity_w": 10.0,
    },
    # 10. IMU Assisted - Heavy Ceres Rotation Constraint (60.0)
    {
        "id": "r4_cfg10_imu_ceres_rot60",
        "desc": "IMU-Assisted - Ceres Rotation Weight 60.0 to Enforce IMU Heading",
        "num_submaps": 45,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 60.0,
        "search_lin": 0.20,
        "search_ang": 15.0,
        "opt_nodes": 4,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
        "imu_gravity_w": 10.0,
    },
    # 11. IMU Assisted - Strong Huber Loss Regularization (Huber 0.5)
    {
        "id": "r4_cfg11_imu_huber05",
        "desc": "IMU-Assisted - Huber Scale 0.5 for Suppressing Saccade Outliers",
        "num_submaps": 45,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 4,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 0.5,
        "imu_gravity_w": 10.0,
    },
    # 12. IMU Assisted - High IMU Gravity Time Constant (Gravity Constant 50.0)
    {
        "id": "r4_cfg12_imu_gravity_w50",
        "desc": "IMU-Assisted - IMU Gravity Constant 50.0",
        "num_submaps": 45,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 4,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
        "imu_gravity_w": 50.0,
    },
    # 13. IMU Assisted - Wide Correlative Search Window (30cm, 30deg)
    {
        "id": "r4_cfg13_imu_wide_search_30cm",
        "desc": "IMU-Assisted - Real-time Search Window 30cm, 30deg",
        "num_submaps": 45,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.30,
        "search_ang": 30.0,
        "opt_nodes": 4,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
        "imu_gravity_w": 10.0,
    },
    # 14. IMU Assisted - Strict Loop Closure Score (min_score 0.55)
    {
        "id": "r4_cfg14_imu_strict_score055",
        "desc": "IMU-Assisted - Constraint Min Score 0.55",
        "num_submaps": 45,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 4,
        "loop_score": 0.55,
        "loop_global_score": 0.60,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
        "imu_gravity_w": 10.0,
    },
    # 15. IMU Assisted - Ultra-Dense Graph (Opt every 2 nodes)
    {
        "id": "r4_cfg15_imu_opt2_dense_graph",
        "desc": "IMU-Assisted - Optimize Every 2 Nodes",
        "num_submaps": 35,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 2,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
        "imu_gravity_w": 10.0,
    },
    # 16. IMU Assisted - High Dynamic Range (Hit 0.78, Miss 0.36)
    {
        "id": "r4_cfg16_imu_hit078_miss036",
        "desc": "IMU-Assisted - Hit 0.78, Miss 0.36 + Occ Weight 40.0",
        "num_submaps": 45,
        "ceres_occ": 40.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 4,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "hit_prob": 0.78,
        "miss_prob": 0.36,
        "huber_scale": 1.0,
        "imu_gravity_w": 10.0,
    },
    # 17. IMU Assisted - High Odometry Fusion (Odom Rot 5e5, IMU Active)
    {
        "id": "r4_cfg17_imu_odom_rot5e5",
        "desc": "IMU-Assisted + High Odometry Rot Weight 5e5",
        "num_submaps": 45,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.18,
        "search_ang": 18.0,
        "opt_nodes": 4,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "odom_rot_w": 5e5,
        "odom_trans_w": 1e5,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
        "imu_gravity_w": 10.0,
    },
    # 18. IMU Assisted - Relaxed Angular Search Window (40 deg)
    {
        "id": "r4_cfg18_imu_angular_40deg",
        "desc": "IMU-Assisted - Angular Window 40 deg + Lin 25cm",
        "num_submaps": 45,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.25,
        "search_ang": 40.0,
        "opt_nodes": 3,
        "loop_score": 0.45,
        "loop_global_score": 0.50,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
        "imu_gravity_w": 10.0,
    },
    # 19. IMU Assisted - Balanced Hit/Miss (Hit 0.68, Miss 0.44, Opt 3)
    {
        "id": "r4_cfg19_imu_balanced_opt3",
        "desc": "IMU-Assisted - Hit 0.68, Miss 0.44, Opt every 3 nodes",
        "num_submaps": 40,
        "ceres_occ": 38.0,
        "ceres_trans": 10.0,
        "ceres_rot": 38.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 3,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "hit_prob": 0.68,
        "miss_prob": 0.44,
        "huber_scale": 1.0,
        "imu_gravity_w": 10.0,
    },
    # 20. IMU Assisted - Champion Master IMU Tuned Preset
    {
        "id": "r4_cfg20_champion_master_imu",
        "desc": "Champion Master IMU Tuned - Hit 0.72, Miss 0.40, Ceres 40/40, Huber 1.0, Opt 3",
        "num_submaps": 45,
        "ceres_occ": 40.0,
        "ceres_trans": 12.0,
        "ceres_rot": 40.0,
        "search_lin": 0.22,
        "search_ang": 22.0,
        "opt_nodes": 3,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "hit_prob": 0.72,
        "miss_prob": 0.40,
        "huber_scale": 1.0,
        "imu_gravity_w": 10.0,
    }
]

def generate_lua(cfg):
    # NOTE: Strictly 2cm (0.020m) tiles for all configurations (DO NOT CHANGE).
    res = 0.020
    hit_prob = cfg.get("hit_prob", 0.58)
    miss_prob = cfg.get("miss_prob", 0.46)
    huber = cfg.get("huber_scale", 1.0)
    gravity_w = cfg.get("imu_gravity_w", 10.0)
    odom_trans_w = cfg.get("odom_trans_w", 1e3)
    odom_rot_w = cfg.get("odom_rot_w", 1e3)

    lua_str = f"""include "map_builder.lua"
include "trajectory_builder.lua"

options = {{
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  -- NOTE: Strictly IMU-enabled tracking frame (DO NOT CHANGE)
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
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = {gravity_w}

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
    print(f"[SWEEP 4] Starting Round 4: 20-configuration IMU-ENABLED 2cm Cartographer parameter sweep...")
    results_dir = r"/mnt/c/Users/nurmi/Documents/Da Winna/maps/cyan_champion_clean_static/sweep_results"
    checker_script = r"/home/gold/folkrace_cartographer_ws/scripts/check_map_empty.py"
    
    for idx, cfg in enumerate(CONFIGS):
        cfg_id = cfg["id"]
        lua_filename = f"{cfg_id}.lua"
        lua_path = f"/home/gold/folkrace_cartographer_ws/param_sweep/{lua_filename}"
        png_out_path = f"{results_dir}/{cfg_id}.png"
        
        print(f"\n================================================================================")
        print(f"[{idx+1}/20] Running {cfg_id}")
        print(f"       Description: {cfg['desc']}")
        print(f"================================================================================")

        try:
            # 1. Write Lua
            with open(lua_path, "w", encoding="utf-8") as f:
                f.write(generate_lua(cfg))

            # 2. Kill existing Cartographer nodes
            subprocess.run("pkill -9 -f cartographer_node; pkill -9 -f cartographer_occupancy_grid_node; sleep 0.5", shell=True)

            # 3. Launch Cartographer nodes with IMU topic remapping (Strictly 2cm tiles)
            carto_cmd = f"nohup bash -c 'source /opt/ros/humble/local_setup.bash && ros2 run cartographer_ros cartographer_node -configuration_directory /home/gold/folkrace_cartographer_ws/param_sweep -configuration_basename {lua_filename} --ros-args --remap scan:=/scan --remap odom:=/odom --remap imu:=/imu' > /tmp/carto_sweep4.log 2>&1 &"
            grid_cmd = f"nohup bash -c 'source /opt/ros/humble/local_setup.bash && ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.02 -publish_period_sec 0.5' > /tmp/grid_sweep4.log 2>&1 &"
            
            subprocess.run(carto_cmd, shell=True)
            subprocess.run(grid_cmd, shell=True)
            time.sleep(1.5)

            # 4. Feed live Webots multi-lap stream with IMU messages
            feed_cmd = "bash -c 'source /opt/ros/humble/local_setup.bash && python3 ~/folkrace_cartographer_ws/scripts/feed_cyberbotics_imu_to_cartographer.py'"
            subprocess.run(feed_cmd, shell=True)
            time.sleep(1.0)

            # 5. Export PNG (skips uninitialized 0x0 frames until populated)
            export_cmd = f"bash -c 'source /opt/ros/humble/local_setup.bash && python3 ~/folkrace_cartographer_ws/scripts/export_cartographer_png.py \"{png_out_path}\"'"
            subprocess.run(export_cmd, shell=True)

            # 6. Cleanup FastRTPS shared memory locks
            subprocess.run("pkill -9 -f cartographer_node; pkill -9 -f cartographer_occupancy_grid_node; rm -rf /dev/shm/* 2>/dev/null", shell=True)

            # 7. Check non-empty status with programmatic validator
            check_proc = subprocess.run(f"python3 {checker_script} \"{png_out_path}\"", shell=True, capture_output=True, text=True)
            print(f"[STATUS] {check_proc.stdout.strip()}")
        except Exception as e:
            print(f"[ERROR] Failed running {cfg_id}: {e}")

    print("\n[SWEEP 4 COMPLETE] All 20 IMU-enabled configurations processed.")

if __name__ == '__main__':
    main()
