#!/usr/bin/env python3
"""
================================================================================
  ROUND 7: MULTI-RESOLUTION PARAMETER SWEEP (20mm, 25mm, 30mm)
  Based directly on the best performing cluster settings:
  - 3-Lap full run at 0.20 race pace (6,499 scans)
  - 0-Noise LiDAR + IMU / Gyro / Accel
  - Submaps: 90, 110, 120, 140
  - Odometry/IMU Rotation weights: 5e5, 8e5, 1e6
  - Loop closure thresholds: 0.55, 0.60, 0.65, 0.70
  - Resolutions tested: 20mm (0.020m), 25mm (0.025m), 30mm (0.030m)
================================================================================
"""

import os
import sys
import time
import subprocess

CONFIGS = []

# Base parameter variations to test across all 3 grid resolutions (20mm, 25mm, 30mm)
TEMPLATES = [
    # 1. Submaps 90, OdomRot 8e5, Loop 0.55
    {"id_base": "submap90_rot8e5_loop055", "desc_base": "Submaps 90, OdomRot 8e5, Loop 0.55", "submaps": 90, "odom_rot": 8e5, "loop_s": 0.55, "opt_n": 5, "hit_p": 0.70, "miss_p": 0.42, "lin_s": 0.12, "ang_s": 12.0},
    # 2. Submaps 90, OdomRot 1e6, Loop 0.58
    {"id_base": "submap90_rot1e6_loop058", "desc_base": "Submaps 90, OdomRot 1e6, Loop 0.58", "submaps": 90, "odom_rot": 1e6, "loop_s": 0.58, "opt_n": 5, "hit_p": 0.70, "miss_p": 0.42, "lin_s": 0.12, "ang_s": 12.0},
    # 3. Submaps 110, OdomRot 5e5, Loop 0.55
    {"id_base": "submap110_rot5e5_loop055", "desc_base": "Submaps 110, OdomRot 5e5, Loop 0.55", "submaps": 110, "odom_rot": 5e5, "loop_s": 0.55, "opt_n": 5, "hit_p": 0.70, "miss_p": 0.42, "lin_s": 0.14, "ang_s": 14.0},
    # 4. Submaps 110, OdomRot 8e5, Loop 0.58
    {"id_base": "submap110_rot8e5_loop058", "desc_base": "Submaps 110, OdomRot 8e5, Loop 0.58", "submaps": 110, "odom_rot": 8e5, "loop_s": 0.58, "opt_n": 5, "hit_p": 0.70, "miss_p": 0.42, "lin_s": 0.12, "ang_s": 12.0},
    # 5. Submaps 110, OdomRot 1e6, Loop 0.60
    {"id_base": "submap110_rot1e6_loop060", "desc_base": "Submaps 110, OdomRot 1e6, Loop 0.60", "submaps": 110, "odom_rot": 1e6, "loop_s": 0.60, "opt_n": 5, "hit_p": 0.70, "miss_p": 0.42, "lin_s": 0.10, "ang_s": 12.0},
    # 6. Submaps 120, OdomRot 5e5, Loop 0.55
    {"id_base": "submap120_rot5e5_loop055", "desc_base": "Submaps 120, OdomRot 5e5, Loop 0.55", "submaps": 120, "odom_rot": 5e5, "loop_s": 0.55, "opt_n": 5, "hit_p": 0.70, "miss_p": 0.42, "lin_s": 0.14, "ang_s": 14.0},
    # 7. Submaps 120, OdomRot 8e5, Loop 0.60
    {"id_base": "submap120_rot8e5_loop060", "desc_base": "Submaps 120, OdomRot 8e5, Loop 0.60", "submaps": 120, "odom_rot": 8e5, "loop_s": 0.60, "opt_n": 5, "hit_p": 0.70, "miss_p": 0.42, "lin_s": 0.12, "ang_s": 12.0},
    # 8. Submaps 120, OdomRot 1e6, Loop 0.65
    {"id_base": "submap120_rot1e6_loop065", "desc_base": "Submaps 120, OdomRot 1e6, Loop 0.65", "submaps": 120, "odom_rot": 1e6, "loop_s": 0.65, "opt_n": 6, "hit_p": 0.70, "miss_p": 0.42, "lin_s": 0.10, "ang_s": 10.0},
    # 9. Submaps 140, OdomRot 5e5, Loop 0.60
    {"id_base": "submap140_rot5e5_loop060", "desc_base": "Submaps 140, OdomRot 5e5, Loop 0.60", "submaps": 140, "odom_rot": 5e5, "loop_s": 0.60, "opt_n": 6, "hit_p": 0.70, "miss_p": 0.42, "lin_s": 0.12, "ang_s": 12.0},
    # 10. Submaps 140, OdomRot 1e6, Loop 0.70
    {"id_base": "submap140_rot1e6_loop070", "desc_base": "Submaps 140, OdomRot 1e6, Loop 0.70", "submaps": 140, "odom_rot": 1e6, "loop_s": 0.70, "opt_n": 6, "hit_p": 0.70, "miss_p": 0.42, "lin_s": 0.10, "ang_s": 10.0},
]

# Build 30 total configurations (10 for 20mm, 10 for 25mm, 10 for 30mm)
for res_label, res_val in [("20mm", 0.020), ("25mm", 0.025), ("30mm", 0.030)]:
    for t in TEMPLATES:
        cfg_id = f"r7_cfg{len(CONFIGS)+1:02d}_{res_label}_{t['id_base']}"
        CONFIGS.append({
            "id": cfg_id,
            "desc": f"[{res_label}] {t['desc_base']} (res={res_val}m)",
            "res": res_val,
            "num_submaps": t["submaps"],
            "ceres_occ": 35.0,
            "ceres_trans": 10.0,
            "ceres_rot": 35.0,
            "search_lin": t["lin_s"],
            "search_ang": t["ang_s"],
            "opt_nodes": t["opt_n"],
            "loop_score": t["loop_s"],
            "loop_global_score": t["loop_s"] + 0.05,
            "hit_prob": t["hit_p"],
            "miss_prob": t["miss_p"],
            "huber_scale": 1.0,
            "odom_trans_w": 1e5,
            "odom_rot_w": t["odom_rot"],
        })

def generate_lua(cfg):
    res = cfg["res"]
    hit_prob = cfg.get("hit_prob", 0.70)
    miss_prob = cfg.get("miss_prob", 0.42)
    huber = cfg.get("huber_scale", 1.0)
    odom_trans_w = cfg.get("odom_trans_w", 1e5)
    odom_rot_w = cfg.get("odom_rot_w", 8e5)

    lua_str = f"""include "map_builder.lua"
include "trajectory_builder.lua"

options = {{
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
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
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = {res}
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
    print(f"[SWEEP 7] Starting Round 7: Multi-Resolution (20mm, 25mm, 30mm) 30-configuration sweep...")
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

            # 3. Launch Cartographer nodes with resolution matching config
            carto_cmd = f"nohup bash -c 'source /opt/ros/humble/local_setup.bash && ros2 run cartographer_ros cartographer_node -configuration_directory /home/gold/folkrace_cartographer_ws/param_sweep -configuration_basename {lua_filename} --ros-args --remap scan:=/scan --remap odom:=/odom --remap imu:=/imu' > /tmp/carto_sweep7.log 2>&1 &"
            grid_cmd = f"nohup bash -c 'source /opt/ros/humble/local_setup.bash && ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution {res} -publish_period_sec 0.5' > /tmp/grid_sweep7.log 2>&1 &"
            
            subprocess.run(carto_cmd, shell=True)
            subprocess.run(grid_cmd, shell=True)
            time.sleep(1.5)

            # 4. Feed live 3-lap Webots stream (6,499 scans) with rigid IMU
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

    print("\n[SWEEP 7 COMPLETE] All 30 multi-resolution configurations processed.")

if __name__ == '__main__':
    main()
