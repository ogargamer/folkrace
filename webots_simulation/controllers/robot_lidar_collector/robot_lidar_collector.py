#!/usr/bin/env python3
"""
================================================================================
   WEBOTS CONTROLLER: ZERO-NOISE HORIZONTAL PLANAR LIDAR COLLECTOR
================================================================================
"""

import sys
import os
import json
import numpy as np

if "WEBOTS_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["WEBOTS_HOME"], "lib", "controller", "python"))

from controller import Supervisor

def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep()) # 10 ms

    lidar = robot.getDevice("ms200k_lidar")
    lidar.enable(int(1000 / 12)) # 12 Hz scan rate (~83.3 ms)
    lidar.enablePointCloud()

    traj_path = os.path.join(os.path.dirname(__file__), "..", "..", "..", "racing_results", "yellow_line_backup", "07_track_7_yellow_trajectory.csv")
    traj_path = os.path.abspath(traj_path)

    traj = np.loadtxt(traj_path, delimiter=",", skiprows=1)
    N = len(traj)
    pts = traj[:, :2]
    vels = traj[:, 2] * 0.25 # 0.25x speed
    headings = traj[:, 3]

    robot_node = robot.getSelf()
    trans_field = robot_node.getField("translation")
    rot_field = robot_node.getField("rotation")

    print("[INFO] Webots Zero-Noise Horizontal Planar Collector Started!")

    collected_scans = []
    
    cur_idx = 0
    cur_dist = 0.0
    total_dist = 0.0
    ds_list = [np.hypot(pts[(i+1)%N, 0] - pts[i, 0], pts[(i+1)%N, 1] - pts[i, 1]) for i in range(N)]
    lap_length = sum(ds_list)
    
    dt = timestep / 1000.0
    step_count = 0
    lidar_scan_count = 0

    while robot.step(timestep) != -1:
        step_count += 1
        current_v = vels[cur_idx]
        cur_dist += current_v * dt
        total_dist += current_v * dt
        
        while cur_dist >= ds_list[cur_idx]:
            cur_dist -= ds_list[cur_idx]
            cur_idx = (cur_idx + 1) % N

        next_idx = (cur_idx + 1) % N
        interp_factor = cur_dist / max(ds_list[cur_idx], 1e-4)
        
        rx = pts[cur_idx, 0] + interp_factor * (pts[next_idx, 0] - pts[cur_idx, 0])
        rz = pts[cur_idx, 1] + interp_factor * (pts[next_idx, 1] - pts[cur_idx, 1])
        
        dx_next = pts[next_idx, 0] - pts[cur_idx, 0]
        dz_next = pts[next_idx, 1] - pts[cur_idx, 1]
        
        # Webots yaw rotation angle around Y-axis
        webots_yaw = np.arctan2(dx_next, dz_next)

        # Update pose in Webots
        trans_field.setSFVec3f([float(rx), 0.05, float(rz)])
        rot_field.setSFRotation([0.0, 1.0, 0.0, float(webots_yaw)])

        # Read LiDAR 3D Point Cloud in global coordinates
        raw_cloud = lidar.getPointCloud()
        if raw_cloud is not None and len(raw_cloud) > 0:
            lidar_scan_count += 1
            
            pos = np.array(robot_node.getPosition())
            rot_mat = np.array(robot_node.getOrientation()).reshape(3, 3)
            
            valid_pts = []
            for p in raw_cloud:
                p_local = np.array([p.x, p.y, p.z])
                dist = np.linalg.norm(p_local)
                if 0.12 < dist <= 3.98 and not np.isnan(dist) and not np.isinf(dist):
                    # Exact 3D Transform: P_global = R * P_local + Pos
                    p_global = rot_mat @ p_local + pos
                    valid_pts.append([p_global[0], p_global[2]])
                    
            if len(valid_pts) > 0:
                collected_scans.extend(valid_pts)

        if total_dist >= lap_length:
            print(f"[COMPLETE] Finished Track 7 Lap! Traversed: {total_dist:.2f}m")
            print(f"[COMPLETE] Captured {len(collected_scans)} zero-noise points across {lidar_scan_count} sweeps.")
            break

    output_dir = os.path.dirname(__file__)
    np_pts = np.array(collected_scans)
    np.save(os.path.join(output_dir, "track7_deskewed_lidar_points.npy"), np_pts)
    
    with open(os.path.join(output_dir, "track7_sim_summary.json"), "w") as f:
        json.dump({
            "track": "Track 7 (Multi-Bottleneck Slalom)",
            "total_points": len(collected_scans),
            "lidar_scans": lidar_scan_count,
            "lap_distance_m": total_dist,
            "lap_duration_s": step_count * dt,
            "zero_noise": True
        }, f, indent=2)
        
    print(f"[OK] Saved Track 7 Zero-Noise LiDAR map: {len(np_pts)} points.")
    robot.simulationQuit(0)

if __name__ == "__main__":
    main()
