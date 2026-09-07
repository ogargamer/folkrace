#!/usr/bin/env python3
"""
================================================================================
  ROUND 3: 20-CONFIGURATION PARAMETER SWEEP
  CRITICAL CONSTRAINTS:
  - 2cm (0.020m) tiles strictly for all configurations (DO NOT CHANGE).
  - Tests hit_probability and miss_probability sweeps (pixel add/remove rates).
  - Explores variations based on cfg08/cfg07/cfg03 successes (varying submaps,
    Ceres weights, Huber loss, search windows, and odometry inclusion).
  - Uses check_map_empty.py to programmatically ensure images are non-empty.
================================================================================
"""

import os
import sys
import time
import subprocess

CONFIGS = [
    # 1. Baseline Pure Scan Matching with Default Probabilities (Hit 0.55, Miss 0.49)
    {
        "id": "r3_cfg01_pure_hit055_miss049",
        "desc": "Pure Scan Match (no odom) - Hit 0.55, Miss 0.49 (Slow add, slow remove)",
        "use_odom": "false",
        "num_submaps": 40,
        "ceres_occ": 40.0,
        "ceres_trans": 15.0,
        "ceres_rot": 40.0,
        "search_lin": 0.25,
        "search_ang": 25.0,
        "opt_nodes": 4,
        "loop_score": 0.45,
        "loop_global_score": 0.50,
        "hit_prob": 0.55,
        "miss_prob": 0.49,
        "huber_scale": 1.0,
    },
    # 2. Pure Scan Match with Rapid Hit (Hit 0.65, Miss 0.45)
    {
        "id": "r3_cfg02_pure_hit065_miss045",
        "desc": "Pure Scan Match - Hit 0.65, Miss 0.45 (Moderate add/remove)",
        "use_odom": "false",
        "num_submaps": 40,
        "ceres_occ": 40.0,
        "ceres_trans": 15.0,
        "ceres_rot": 40.0,
        "search_lin": 0.25,
        "search_ang": 25.0,
        "opt_nodes": 4,
        "loop_score": 0.45,
        "loop_global_score": 0.50,
        "hit_prob": 0.65,
        "miss_prob": 0.45,
        "huber_scale": 1.0,
    },
    # 3. Pure Scan Match with Aggressive Hit (Hit 0.75, Miss 0.40)
    {
        "id": "r3_cfg03_pure_hit075_miss040",
        "desc": "Pure Scan Match - Hit 0.75, Miss 0.40 (Fast add, fast clear)",
        "use_odom": "false",
        "num_submaps": 40,
        "ceres_occ": 40.0,
        "ceres_trans": 15.0,
        "ceres_rot": 40.0,
        "search_lin": 0.25,
        "search_ang": 25.0,
        "opt_nodes": 4,
        "loop_score": 0.45,
        "loop_global_score": 0.50,
        "hit_prob": 0.75,
        "miss_prob": 0.40,
        "huber_scale": 1.0,
    },
    # 4. Pure Scan Match with Instant Hit (Hit 0.85, Miss 0.35)
    {
        "id": "r3_cfg04_pure_hit085_miss035",
        "desc": "Pure Scan Match - Hit 0.85, Miss 0.35 (Instant 1-2 ray wall creation)",
        "use_odom": "false",
        "num_submaps": 40,
        "ceres_occ": 40.0,
        "ceres_trans": 15.0,
        "ceres_rot": 40.0,
        "search_lin": 0.25,
        "search_ang": 25.0,
        "opt_nodes": 4,
        "loop_score": 0.45,
        "loop_global_score": 0.50,
        "hit_prob": 0.85,
        "miss_prob": 0.35,
        "huber_scale": 1.0,
    },
    # 5. Pure Scan Match with Sticky Walls (Hit 0.70, Miss 0.48)
    {
        "id": "r3_cfg05_pure_sticky_hit070_miss048",
        "desc": "Pure Scan Match - Hit 0.70, Miss 0.48 (Hard to erase walls)",
        "use_odom": "false",
        "num_submaps": 40,
        "ceres_occ": 40.0,
        "ceres_trans": 15.0,
        "ceres_rot": 40.0,
        "search_lin": 0.25,
        "search_ang": 25.0,
        "opt_nodes": 4,
        "loop_score": 0.45,
        "loop_global_score": 0.50,
        "hit_prob": 0.70,
        "miss_prob": 0.48,
        "huber_scale": 1.0,
    },
    # 6. Pure Scan Match with Rapid Clearing (Hit 0.60, Miss 0.30)
    {
        "id": "r3_cfg06_pure_rapid_clear_hit060_miss030",
        "desc": "Pure Scan Match - Hit 0.60, Miss 0.30 (Aggressive free-space clearing)",
        "use_odom": "false",
        "num_submaps": 40,
        "ceres_occ": 40.0,
        "ceres_trans": 15.0,
        "ceres_rot": 40.0,
        "search_lin": 0.25,
        "search_ang": 25.0,
        "opt_nodes": 4,
        "loop_score": 0.45,
        "loop_global_score": 0.50,
        "hit_prob": 0.60,
        "miss_prob": 0.30,
        "huber_scale": 1.0,
    },
    # 7. Pure Scan Match with Submaps = 30 (Faster loop formation)
    {
        "id": "r3_cfg07_pure_submaps30",
        "desc": "Pure Scan Match - Submaps 30, Hit 0.70, Miss 0.42",
        "use_odom": "false",
        "num_submaps": 30,
        "ceres_occ": 40.0,
        "ceres_trans": 15.0,
        "ceres_rot": 40.0,
        "search_lin": 0.25,
        "search_ang": 25.0,
        "opt_nodes": 3,
        "loop_score": 0.45,
        "loop_global_score": 0.50,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
    },
    # 8. Pure Scan Match with Submaps = 60 (Longer rigidity)
    {
        "id": "r3_cfg08_pure_submaps60",
        "desc": "Pure Scan Match - Submaps 60, Hit 0.70, Miss 0.42",
        "use_odom": "false",
        "num_submaps": 60,
        "ceres_occ": 40.0,
        "ceres_trans": 15.0,
        "ceres_rot": 40.0,
        "search_lin": 0.25,
        "search_ang": 25.0,
        "opt_nodes": 4,
        "loop_score": 0.45,
        "loop_global_score": 0.50,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
    },
    # 9. Pure Scan Match with Higher Ceres Occupied Space (55.0)
    {
        "id": "r3_cfg09_pure_ceres_occ55",
        "desc": "Pure Scan Match - Ceres Occupied Weight 55.0, Hit 0.70, Miss 0.42",
        "use_odom": "false",
        "num_submaps": 45,
        "ceres_occ": 55.0,
        "ceres_trans": 15.0,
        "ceres_rot": 45.0,
        "search_lin": 0.25,
        "search_ang": 25.0,
        "opt_nodes": 4,
        "loop_score": 0.45,
        "loop_global_score": 0.50,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
    },
    # 10. Pure Scan Match with Lower Huber Scale (0.5)
    {
        "id": "r3_cfg10_pure_huber_05",
        "desc": "Pure Scan Match - Huber Scale 0.5 (Heavy outlier suppression)",
        "use_odom": "false",
        "num_submaps": 40,
        "ceres_occ": 40.0,
        "ceres_trans": 15.0,
        "ceres_rot": 40.0,
        "search_lin": 0.25,
        "search_ang": 25.0,
        "opt_nodes": 4,
        "loop_score": 0.45,
        "loop_global_score": 0.50,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 0.5,
    },
    # 11. Pure Scan Match with Wider Search Window (30cm, 35deg)
    {
        "id": "r3_cfg11_pure_wide_search_30cm",
        "desc": "Pure Scan Match - Linear Search 0.30m, Angular 35 deg",
        "use_odom": "false",
        "num_submaps": 40,
        "ceres_occ": 40.0,
        "ceres_trans": 15.0,
        "ceres_rot": 40.0,
        "search_lin": 0.30,
        "search_ang": 35.0,
        "opt_nodes": 4,
        "loop_score": 0.45,
        "loop_global_score": 0.50,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
    },
    # 12. Pure Scan Match with Strict Loop Threshold (0.52)
    {
        "id": "r3_cfg12_pure_strict_loop052",
        "desc": "Pure Scan Match - Constraint Min Score 0.52",
        "use_odom": "false",
        "num_submaps": 40,
        "ceres_occ": 40.0,
        "ceres_trans": 15.0,
        "ceres_rot": 40.0,
        "search_lin": 0.25,
        "search_ang": 25.0,
        "opt_nodes": 4,
        "loop_score": 0.52,
        "loop_global_score": 0.55,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
    },
    # 13. High Odometry Rotation Trust with Moderate Hit (Hit 0.65, Miss 0.45)
    {
        "id": "r3_cfg13_odom_rot5e5_hit065_miss045",
        "desc": "Odom Rot Weight 5e5 - Hit 0.65, Miss 0.45",
        "use_odom": "true",
        "num_submaps": 50,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 40.0,
        "search_lin": 0.18,
        "search_ang": 18.0,
        "opt_nodes": 4,
        "loop_score": 0.48,
        "loop_global_score": 0.52,
        "odom_rot_w": 5e5,
        "odom_trans_w": 1e5,
        "hit_prob": 0.65,
        "miss_prob": 0.45,
        "huber_scale": 1.0,
    },
    # 14. High Odometry Rotation Trust with Fast Hit (Hit 0.75, Miss 0.38)
    {
        "id": "r3_cfg14_odom_rot5e5_hit075_miss038",
        "desc": "Odom Rot Weight 5e5 - Hit 0.75, Miss 0.38",
        "use_odom": "true",
        "num_submaps": 50,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 40.0,
        "search_lin": 0.18,
        "search_ang": 18.0,
        "opt_nodes": 4,
        "loop_score": 0.48,
        "loop_global_score": 0.52,
        "odom_rot_w": 5e5,
        "odom_trans_w": 1e5,
        "hit_prob": 0.75,
        "miss_prob": 0.38,
        "huber_scale": 1.0,
    },
    # 15. High Odometry Rotation Trust with Submaps = 35
    {
        "id": "r3_cfg15_odom_rot5e5_submaps35",
        "desc": "Odom Rot Weight 5e5 - Submaps 35 (Fast submap rotation alignment)",
        "use_odom": "true",
        "num_submaps": 35,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 40.0,
        "search_lin": 0.18,
        "search_ang": 18.0,
        "opt_nodes": 3,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "odom_rot_w": 5e5,
        "odom_trans_w": 1e5,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
    },
    # 16. Moderate Odometry (1e5) + Hit 0.72 + Miss 0.40
    {
        "id": "r3_cfg16_odom_rot1e5_hit072_miss040",
        "desc": "Odom Rot Weight 1e5 - Hit 0.72, Miss 0.40, Opt every 3 nodes",
        "use_odom": "true",
        "num_submaps": 45,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 3,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
        "hit_prob": 0.72,
        "miss_prob": 0.40,
        "huber_scale": 1.0,
    },
    # 17. High Ceres Occupied Space (50.0) + Odometry (2e5)
    {
        "id": "r3_cfg17_odom_rot2e5_ceres_occ50",
        "desc": "Odom Rot Weight 2e5, Ceres Occ 50.0, Hit 0.70, Miss 0.42",
        "use_odom": "true",
        "num_submaps": 50,
        "ceres_occ": 50.0,
        "ceres_trans": 10.0,
        "ceres_rot": 40.0,
        "search_lin": 0.18,
        "search_ang": 18.0,
        "opt_nodes": 4,
        "loop_score": 0.48,
        "loop_global_score": 0.52,
        "odom_rot_w": 2e5,
        "odom_trans_w": 1e5,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
    },
    # 18. Pure Scan Match with Opt every 2 nodes + Hit 0.68 + Miss 0.44
    {
        "id": "r3_cfg18_pure_opt2_hit068_miss044",
        "desc": "Pure Scan Match - Opt every 2 nodes, Hit 0.68, Miss 0.44",
        "use_odom": "false",
        "num_submaps": 35,
        "ceres_occ": 40.0,
        "ceres_trans": 15.0,
        "ceres_rot": 40.0,
        "search_lin": 0.22,
        "search_ang": 22.0,
        "opt_nodes": 2,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "hit_prob": 0.68,
        "miss_prob": 0.44,
        "huber_scale": 1.0,
    },
    # 19. Pure Scan Match with Moderate Correlative Window (20cm, 20deg)
    {
        "id": "r3_cfg19_pure_correl_20cm_20deg",
        "desc": "Pure Scan Match - Search Window 20cm, 20deg, Hit 0.72, Miss 0.42",
        "use_odom": "false",
        "num_submaps": 45,
        "ceres_occ": 40.0,
        "ceres_trans": 15.0,
        "ceres_rot": 40.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 4,
        "loop_score": 0.45,
        "loop_global_score": 0.50,
        "hit_prob": 0.72,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
    },
    # 20. Champion Refined Pure Scan Matcher (Hit 0.70, Miss 0.42, Occ 45, Rot 45)
    {
        "id": "r3_cfg20_champion_refined_pure",
        "desc": "Champion Refined Pure Scan Matcher - Hit 0.70, Miss 0.42, Occ 45, Rot 45, Opt 3",
        "use_odom": "false",
        "num_submaps": 45,
        "ceres_occ": 45.0,
        "ceres_trans": 15.0,
        "ceres_rot": 45.0,
        "search_lin": 0.24,
        "search_ang": 24.0,
        "opt_nodes": 3,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "hit_prob": 0.70,
        "miss_prob": 0.42,
        "huber_scale": 1.0,
    }
]

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
    print(f"[SWEEP 3] Starting Round 3: 20-configuration 2cm Cartographer parameter benchmark sweep...")
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

        # 1. Write Lua
        with open(lua_path, "w", encoding="utf-8") as f:
            f.write(generate_lua(cfg))

        # 2. Kill existing Cartographer nodes
        subprocess.run("pkill -9 -f cartographer_node; pkill -9 -f cartographer_occupancy_grid_node; sleep 0.5", shell=True)

        # 3. Launch Cartographer nodes (Resolution strictly 0.02m)
        carto_cmd = f"nohup bash -c 'source /opt/ros/humble/local_setup.bash && ros2 run cartographer_ros cartographer_node -configuration_directory /home/gold/folkrace_cartographer_ws/param_sweep -configuration_basename {lua_filename} --ros-args --remap scan:=/scan --remap odom:=/odom' > /tmp/carto_sweep3.log 2>&1 &"
        grid_cmd = f"nohup bash -c 'source /opt/ros/humble/local_setup.bash && ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.02 -publish_period_sec 0.5' > /tmp/grid_sweep3.log 2>&1 &"
        
        subprocess.run(carto_cmd, shell=True)
        subprocess.run(grid_cmd, shell=True)
        time.sleep(1.5)

        # 4. Feed live Webots multi-lap stream
        feed_cmd = "bash -c 'source /opt/ros/humble/local_setup.bash && python3 ~/folkrace_cartographer_ws/scripts/feed_cyberbotics_native_to_cartographer.py'"
        subprocess.run(feed_cmd, shell=True)

        # 5. Export PNG
        export_cmd = f"bash -c 'source /opt/ros/humble/local_setup.bash && python3 ~/folkrace_cartographer_ws/scripts/export_cartographer_png.py \"{png_out_path}\"'"
        subprocess.run(export_cmd, shell=True)

        # 6. Check non-empty status with programmatic validator
        check_proc = subprocess.run(f"python3 {checker_script} \"{png_out_path}\"", shell=True, capture_output=True, text=True)
        print(f"[STATUS] {check_proc.stdout.strip()}")

    print("\n[SWEEP 3 COMPLETE] All 20 configurations processed.")

if __name__ == '__main__':
    main()
