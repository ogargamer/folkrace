#!/usr/bin/env python3
"""
================================================================================
  ROUND 2: 15-CONFIGURATION PARAMETER SWEEP TARGETING LOOP CLOSURE & SHARP WALLS
  Based on 60-second 0.40-pace multi-lap Webots dataset.
================================================================================
"""

import os
import sys
import time
import subprocess

CONFIGS = [
    # 1. High Global Pose Graph Optimization Frequency (every 3 nodes)
    {
        "id": "r2_cfg01_high_freq_opt_every_3",
        "desc": "High Frequency Loop Optimization (optimize_every_n_nodes = 3)",
        "num_submaps": 60,
        "ceres_occ": 30.0,
        "ceres_trans": 10.0,
        "ceres_rot": 30.0,
        "search_lin": 0.15,
        "search_ang": 15.0,
        "opt_nodes": 3,
        "loop_score": 0.50,
        "loop_global_score": 0.55,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
        "resolution": 0.02,
        "huber_scale": 1.0,
    },
    # 2. Relaxed Loop Closure Score (min_score = 0.40)
    {
        "id": "r2_cfg02_relaxed_loop_score_040",
        "desc": "Relaxed Loop Closure Score (min_score 0.40, global 0.45)",
        "num_submaps": 50,
        "ceres_occ": 30.0,
        "ceres_trans": 10.0,
        "ceres_rot": 30.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 5,
        "loop_score": 0.40,
        "loop_global_score": 0.45,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
        "resolution": 0.02,
        "huber_scale": 1.0,
    },
    # 3. 30mm Coarse Fast-Snapping + High Frequency Loop Closure (Hybrid of cfg11 & cfg13)
    {
        "id": "r2_cfg03_coarse_30mm_fast_loop",
        "desc": "30mm Grid + Huber 1.0 + Optimize every 4 nodes",
        "num_submaps": 50,
        "ceres_occ": 30.0,
        "ceres_trans": 10.0,
        "ceres_rot": 30.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 4,
        "loop_score": 0.48,
        "loop_global_score": 0.52,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
        "resolution": 0.030,
        "huber_scale": 1.0,
    },
    # 4. 25mm Grid + Strong Huber Loss (1.0)
    {
        "id": "r2_cfg04_grid_25mm_huber_1",
        "desc": "25mm Grid + Huber 1.0 + Opt every 5 nodes",
        "num_submaps": 55,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.15,
        "search_ang": 15.0,
        "opt_nodes": 5,
        "loop_score": 0.50,
        "loop_global_score": 0.55,
        "use_odom": "true",
        "odom_rot_w": 2e5,
        "odom_trans_w": 1e5,
        "resolution": 0.025,
        "huber_scale": 1.0,
    },
    # 5. Large Submaps (90 scans) + Relaxed Closure
    {
        "id": "r2_cfg05_large_submap_90_loop",
        "desc": "90-scan Submaps + Loop Score 0.48 + Opt every 3 nodes",
        "num_submaps": 90,
        "ceres_occ": 30.0,
        "ceres_trans": 10.0,
        "ceres_rot": 30.0,
        "search_lin": 0.15,
        "search_ang": 15.0,
        "opt_nodes": 3,
        "loop_score": 0.48,
        "loop_global_score": 0.52,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
        "resolution": 0.02,
        "huber_scale": 1.0,
    },
    # 6. Global Search Window 5.0m + Score 0.50
    {
        "id": "r2_cfg06_global_search_5m",
        "desc": "5.0m Global Loop Search Window + Opt every 4 nodes",
        "num_submaps": 60,
        "ceres_occ": 30.0,
        "ceres_trans": 10.0,
        "ceres_rot": 30.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 4,
        "loop_score": 0.50,
        "loop_global_score": 0.55,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
        "resolution": 0.02,
        "global_search_window": 5.0,
        "huber_scale": 1.0,
    },
    # 7. High Odometry Rotation Trust (5e5) + 30mm Resolution
    {
        "id": "r2_cfg07_odom_rot_5e5_30mm",
        "desc": "Odom Rot Weight 5e5 + 30mm Resolution + Huber 1.0",
        "num_submaps": 50,
        "ceres_occ": 30.0,
        "ceres_trans": 10.0,
        "ceres_rot": 40.0,
        "search_lin": 0.15,
        "search_ang": 15.0,
        "opt_nodes": 4,
        "loop_score": 0.48,
        "loop_global_score": 0.52,
        "use_odom": "true",
        "odom_rot_w": 5e5,
        "odom_trans_w": 1e5,
        "resolution": 0.030,
        "huber_scale": 1.0,
    },
    # 8. Pure Scan Matching (No Odometry) + 30mm Resolution
    {
        "id": "r2_cfg08_pure_scan_matching_30mm",
        "desc": "Pure Scan Matching (no odom) + 30mm Grid + Frequent Loop Closure",
        "num_submaps": 40,
        "ceres_occ": 40.0,
        "ceres_trans": 15.0,
        "ceres_rot": 40.0,
        "search_lin": 0.25,
        "search_ang": 25.0,
        "opt_nodes": 4,
        "loop_score": 0.45,
        "loop_global_score": 0.50,
        "use_odom": "false",
        "odom_rot_w": 1e3,
        "odom_trans_w": 1e3,
        "resolution": 0.030,
        "huber_scale": 1.0,
    },
    # 9. Fast Hit Probability (0.75 / 0.35) + Huber 1.0
    {
        "id": "r2_cfg09_fast_prob_huber_1",
        "desc": "Hit Prob 0.75, Miss Prob 0.35 + Huber 1.0 + 20mm Res",
        "num_submaps": 60,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.15,
        "search_ang": 15.0,
        "opt_nodes": 5,
        "loop_score": 0.50,
        "loop_global_score": 0.55,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
        "resolution": 0.020,
        "hit_prob": 0.75,
        "miss_prob": 0.35,
        "huber_scale": 1.0,
    },
    # 10. Multi-Lap Pose Graph Snapping (opt_nodes=2, score=0.42)
    {
        "id": "r2_cfg10_ultra_dense_pose_graph",
        "desc": "Ultra-Dense Pose Graph (Optimize every 2 nodes, min_score 0.42)",
        "num_submaps": 40,
        "ceres_occ": 30.0,
        "ceres_trans": 10.0,
        "ceres_rot": 30.0,
        "search_lin": 0.20,
        "search_ang": 20.0,
        "opt_nodes": 2,
        "loop_score": 0.42,
        "loop_global_score": 0.48,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
        "resolution": 0.025,
        "huber_scale": 1.0,
    },
    # 11. High Ceres Geometric Rigidity (Occ Weight 60.0)
    {
        "id": "r2_cfg11_high_ceres_occ_60",
        "desc": "Ceres Occupied Space Weight 60.0 + Rot Weight 50.0",
        "num_submaps": 60,
        "ceres_occ": 60.0,
        "ceres_trans": 15.0,
        "ceres_rot": 50.0,
        "search_lin": 0.15,
        "search_ang": 15.0,
        "opt_nodes": 4,
        "loop_score": 0.48,
        "loop_global_score": 0.52,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
        "resolution": 0.020,
        "huber_scale": 1.0,
    },
    # 12. Balanced 20mm Multi-Lap Precision
    {
        "id": "r2_cfg12_balanced_20mm_multi_lap",
        "desc": "Balanced 20mm Resolution (Submaps 50, Opt 4, Loop 0.48, Huber 1.0)",
        "num_submaps": 50,
        "ceres_occ": 35.0,
        "ceres_trans": 10.0,
        "ceres_rot": 35.0,
        "search_lin": 0.18,
        "search_ang": 18.0,
        "opt_nodes": 4,
        "loop_score": 0.48,
        "loop_global_score": 0.53,
        "use_odom": "true",
        "odom_rot_w": 2e5,
        "odom_trans_w": 1e5,
        "resolution": 0.020,
        "huber_scale": 1.0,
    },
    # 13. Wide Angular Correlative Matcher (30 deg)
    {
        "id": "r2_cfg13_wide_angular_30deg",
        "desc": "Angular Search Window 30 deg + Lin 20cm + Opt every 3 nodes",
        "num_submaps": 50,
        "ceres_occ": 30.0,
        "ceres_trans": 10.0,
        "ceres_rot": 30.0,
        "search_lin": 0.20,
        "search_ang": 30.0,
        "opt_nodes": 3,
        "loop_score": 0.46,
        "loop_global_score": 0.50,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
        "resolution": 0.025,
        "huber_scale": 1.0,
    },
    # 14. 35mm Fast Convergence Basin
    {
        "id": "r2_cfg14_fast_basin_35mm",
        "desc": "35mm Grid Resolution + Score 0.45 + Opt every 3 nodes",
        "num_submaps": 45,
        "ceres_occ": 30.0,
        "ceres_trans": 10.0,
        "ceres_rot": 30.0,
        "search_lin": 0.25,
        "search_ang": 25.0,
        "opt_nodes": 3,
        "loop_score": 0.45,
        "loop_global_score": 0.50,
        "use_odom": "true",
        "odom_rot_w": 1e5,
        "odom_trans_w": 1e5,
        "resolution": 0.035,
        "huber_scale": 1.0,
    },
    # 15. The Champion Master Tuning (Submaps 60, Res 20mm, Occ 40, Rot 40, Huber 1.0, Opt 3, Score 0.45)
    {
        "id": "r2_cfg15_champion_master_tuned",
        "desc": "Champion Master Tuned (Submaps 60, 20mm Res, Occ 40, Huber 1.0, Opt 3, Loop 0.45)",
        "num_submaps": 60,
        "ceres_occ": 40.0,
        "ceres_trans": 10.0,
        "ceres_rot": 40.0,
        "search_lin": 0.18,
        "search_ang": 20.0,
        "opt_nodes": 3,
        "loop_score": 0.45,
        "loop_global_score": 0.50,
        "use_odom": "true",
        "odom_rot_w": 3e5,
        "odom_trans_w": 1e5,
        "resolution": 0.020,
        "huber_scale": 1.0,
    }
]

def generate_lua(cfg):
    res = cfg.get("resolution", 0.02)
    hit_prob = cfg.get("hit_prob", 0.58)
    miss_prob = cfg.get("miss_prob", 0.46)
    huber = cfg.get("huber_scale", 1.0)
    global_search = cfg.get("global_search_window", 4.5)

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
    print(f"[SWEEP 2] Starting Round 2 15-configuration Cartographer parameter benchmark sweep...")
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

        # 2. Kill existing Cartographer nodes
        subprocess.run("pkill -9 -f cartographer_node; pkill -9 -f cartographer_occupancy_grid_node; sleep 0.5", shell=True)

        # 3. Launch Cartographer nodes with this config
        res = cfg.get("resolution", 0.02)
        carto_cmd = f"nohup bash -c 'source /opt/ros/humble/local_setup.bash && ros2 run cartographer_ros cartographer_node -configuration_directory /home/gold/folkrace_cartographer_ws/param_sweep -configuration_basename {lua_filename} --ros-args --remap scan:=/scan --remap odom:=/odom' > /tmp/carto_sweep2.log 2>&1 &"
        grid_cmd = f"nohup bash -c 'source /opt/ros/humble/local_setup.bash && ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution {res} -publish_period_sec 0.5' > /tmp/grid_sweep2.log 2>&1 &"
        
        subprocess.run(carto_cmd, shell=True)
        subprocess.run(grid_cmd, shell=True)
        time.sleep(1.5)

        # 4. Feed live Webots multi-lap stream
        feed_cmd = "bash -c 'source /opt/ros/humble/local_setup.bash && python3 ~/folkrace_cartographer_ws/scripts/feed_cyberbotics_native_to_cartographer.py'"
        subprocess.run(feed_cmd, shell=True)

        # 5. Export PNG
        export_cmd = f"bash -c 'source /opt/ros/humble/local_setup.bash && python3 ~/folkrace_cartographer_ws/scripts/export_cartographer_png.py \"{png_out_path}\"'"
        subprocess.run(export_cmd, shell=True)
        print(f"[SAVED] -> {png_out_path}")

    print("\n[SWEEP 2 COMPLETE] All 15 configurations rendered successfully.")

if __name__ == '__main__':
    main()
