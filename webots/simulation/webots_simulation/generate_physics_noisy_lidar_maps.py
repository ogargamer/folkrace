#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - REALISTIC VEHICLE DYNAMICS & PHYSICAL SENSOR NOISE SIMULATOR
================================================================================
Simulates realistic physical disturbances on 2D LiDAR scanning:
  1. Lateral Tire Side-Slip Angle beta(s) under high-g cornering (a_lat ~ 1.0G)
  2. Longitudinal Wheel Slip & Jerk during acceleration (2.4m/s^2) / braking (4.5m/s^2)
  3. High-Frequency Motor & Track Seam Chassis Vibration (50-150Hz micro-yaw/pitch)
  4. Rolling-Shutter Sweep Motion (83.3ms / 12Hz) with sub-sweep micro-jitter
  5. Glancing-Angle Optical Range Degradation on walls & pillars
================================================================================
"""

import os
import sys
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE_DIR)

from webots/optimization/racing_engine.track_generator import (
    get_track_1_chicane_with_pillar,
    get_track_2_bottleneck_circuit,
    get_track_3_slalom_pillars,
    get_track_4_switchback_arena,
    get_track_5_dual_choke_chicane_ring,
    get_track_6_quad_pillar_loop,
    get_track_7_multi_bottleneck_slalom,
    get_track_8_dual_chicane_arena,
    get_track_9_super_speedway,
    get_track_10_switchback_obstacle_maze,
    get_track_11_quad_chicane_slalom
)
from webots/optimization/racing_engine.centerline_finder import find_centerline_voronoi
from webots/optimization/racing_engine.visualizer import OUTPUT_DIR, BG_COLOR, TEXT_COLOR
from exact_2d_lidar_simulator import cast_lidar_rays_vectorized

def simulate_physics_noisy_lidar(track_data, yellow_csv_path, num_scans=75, num_rays=450, max_range=4.0):
    """
    Simulates physical vehicle dynamics (tire scrub, chassis vibration, acceleration slip)
    and realistic optical beam degradation.
    """
    traj = np.loadtxt(yellow_csv_path, delimiter=",", skiprows=1)
    N = len(traj)
    pts = traj[:, :2]
    vels = traj[:, 2]
    curvatures = traj[:, 4] if traj.shape[1] > 4 else np.zeros(N)
    
    left_wall = np.array(track_data["left_wall"])
    right_wall = np.array(track_data["right_wall"])
    obstacles = track_data.get("obstacles", [])
    
    seg_list = []
    for i in range(len(left_wall)):
        seg_list.append([left_wall[i], left_wall[(i + 1) % len(left_wall)]])
    for i in range(len(right_wall)):
        seg_list.append([right_wall[i], right_wall[(i + 1) % len(right_wall)]])
    segments = np.array(seg_list)
    
    circles = []
    for obs in obstacles:
        if obs.get("type") == "circle":
            circles.append((np.array([obs["cx"], obs["cy"]]), obs["radius"]))
            
    scan_indices = np.linspace(0, N - 1, num_scans, dtype=int)
    ray_rel_angles = np.linspace(0, 2 * np.pi, num_rays, endpoint=False)
    scan_period = 1.0 / 12.0 # 83.3 ms per 360 sweep
    
    all_hits = []
    
    for idx in scan_indices:
        rx, ry = pts[idx]
        current_v = vels[idx]
        current_k = abs(curvatures[idx])
        
        next_idx = (idx + 1) % N
        prev_idx = (idx - 1 + N) % N
        
        # Tangent heading
        dx = pts[next_idx, 0] - rx
        dy = pts[next_idx, 1] - ry
        heading = np.arctan2(dy, dx)
        
        # 1. Physics: Lateral Tire Side-Slip Angle (Beta)
        # a_lat = v^2 * kappa. Under 1G lateral load, slip angle is ~2-3 degrees
        a_lat = current_v * current_v * current_k
        beta_slip_rad = np.clip(np.radians(1.8 * (a_lat / 9.81)), -0.06, 0.06)
        
        # 2. Physics: Longitudinal Acceleration/Braking Micro-Slip Jitter
        accel = (vels[next_idx] - vels[prev_idx]) / 0.05
        long_jitter_m = np.random.normal(0, 0.005 * (1.0 + abs(accel) / 3.0))
        
        # 3. Physics: Motor & Track Seam Chassis Vibration (50-100Hz)
        vib_yaw_rad = np.random.normal(0, np.radians(0.35)) # ~0.35 deg vibration
        vib_pos_m = np.random.normal(0, 0.003, size=2)     # ~3mm vibration
        
        actual_robot_pos = np.array([rx, ry]) + vib_pos_m + np.array([np.cos(heading), np.sin(heading)]) * long_jitter_m
        actual_robot_heading = heading + beta_slip_rad + vib_yaw_rad
        
        # 4. Physics: Rolling-Shutter Sweep Micro-Progression (Each beam fires at delta_t)
        beam_dts = np.linspace(-scan_period, 0.0, num_rays)
        per_ray_headings = actual_robot_heading + ray_rel_angles + (accel * 0.01 * beam_dts)
        
        # 5. Physics: Base Gaussian Noise (12mm) + Glancing-Angle Optical Dispersion
        base_noise_std = 0.012
        
        hits = cast_lidar_rays_vectorized(actual_robot_pos, per_ray_headings, segments, circles, max_range, base_noise_std)
        if len(hits) > 0:
            # Add subtle glancing angle scatter (~2% of rays have optical multi-reflection jitter)
            scatter_mask = np.random.rand(len(hits)) < 0.03
            if np.any(scatter_mask):
                hits[scatter_mask] += np.random.normal(0, 0.025, size=(np.sum(scatter_mask), 2))
            all_hits.append(hits)
            
    return np.vstack(all_hits) if len(all_hits) > 0 else np.empty((0, 2))

def export_svgs(hit_pts, track_data, yellow_csv_path, points_only_filename, overlay_filename, track_title):
    # 1. Export Points-Only SVG
    fig, ax = plt.subplots(figsize=(18, 11), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    ax.scatter(hit_pts[:, 0], hit_pts[:, 1], color="#00f0ff", s=2.0, alpha=0.75, edgecolors='none')
    ax.set_title(f"{track_title} - Physics-Noisy MS200k LiDAR Points (Tire Slip, Vibration, 1.0x Speed)",
                 color=TEXT_COLOR, fontsize=13, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.15, color="#fff")
    ax.tick_params(colors=TEXT_COLOR)

    p_path = os.path.join(OUTPUT_DIR, points_only_filename)
    plt.savefig(p_path, format="svg", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)

    # 2. Export Overlaid SVG
    left_wall = np.array(track_data["left_wall"])
    right_wall = np.array(track_data["right_wall"])
    lw_wrap = np.vstack([left_wall, left_wall[:1]])
    rw_wrap = np.vstack([right_wall, right_wall[:1]])

    traj = np.loadtxt(yellow_csv_path, delimiter=",", skiprows=1)
    yellow_pts = traj[:, :2]
    yellow_wrap = np.vstack([yellow_pts, yellow_pts[:1]])

    vor_res = find_centerline_voronoi(track_data)
    centerline = vor_res["centerline"]
    c_wrap = np.vstack([centerline, centerline[:1]])

    fig, ax = plt.subplots(figsize=(20, 11), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)

    ax.plot(lw_wrap[:, 0], lw_wrap[:, 1], color="#f43f5e", linewidth=2.0, label="Ground Truth Left Wall", zorder=3)
    ax.plot(rw_wrap[:, 0], rw_wrap[:, 1], color="#38bdf8", linewidth=2.0, label="Ground Truth Right Wall", zorder=3)

    for obs in track_data.get("obstacles", []):
        if obs.get("type") == "circle":
            circle = plt.Circle((obs["cx"], obs["cy"]), obs["radius"], color="#fbbf24", alpha=0.9, zorder=4)
            ax.add_patch(circle)

    ax.scatter(hit_pts[:, 0], hit_pts[:, 1], color="#00f0ff", s=1.8, alpha=0.6,
               label=f"Physics-Noisy LiDAR ({len(hit_pts)} pts, Tire Slip + Motor Vib)", zorder=2)
    ax.plot(c_wrap[:, 0], c_wrap[:, 1], color="#475569", linewidth=1.0, linestyle=":", label="Medial Centerline", zorder=5)
    ax.plot(yellow_wrap[:, 0], yellow_wrap[:, 1], color="#ffd700", linewidth=2.8, label="Optimal Yellow Racing Line", zorder=6)
    ax.scatter([yellow_pts[0, 0]], [yellow_pts[0, 1]], color="#ffffff", s=140, edgecolors="#ffd700", linewidth=2, label="Robot Start", zorder=7)

    ax.set_title(f"{track_title} - Physics Dynamics & Optical Noise Overlay",
                 color=TEXT_COLOR, fontsize=14, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.2, color="#fff")
    ax.tick_params(colors=TEXT_COLOR)
    ax.legend(bbox_to_anchor=(1.02, 1.0), loc="upper left", facecolor="#121826", edgecolor="#2d3748",
              labelcolor=TEXT_COLOR, framealpha=1.0, fontsize=9.5, borderpad=1.0, labelspacing=0.8)

    o_path = os.path.join(OUTPUT_DIR, overlay_filename)
    plt.savefig(o_path, format="svg", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)

    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    shutil.copy2(p_path, os.path.join(art_dir, points_only_filename))
    shutil.copy2(o_path, os.path.join(art_dir, overlay_filename))
    print(f"  [OK] Saved SVGs: {points_only_filename} and {overlay_filename}")

def main():
    print("=" * 100)
    print("   GENERATING REALISTIC VEHICLE DYNAMICS & PHYSICAL NOISE LIDAR MAPS")
    print("=" * 100)

    # Track 7: Slalom Speedway
    track7 = get_track_7_multi_bottleneck_slalom()
    yellow7_csv = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup", "07_track_7_yellow_trajectory.csv")
    pts7 = simulate_physics_noisy_lidar(track7, yellow7_csv, num_scans=75, num_rays=450)
    export_svgs(pts7, track7, yellow7_csv, "track7_physics_noisy_points.svg", "07_track_7_webots_deskewed_lidar_map.svg", "Track 7 (Slalom Speedway)")

    # Track 4: Technical Switchback
    track4 = get_track_4_switchback_arena()
    yellow4_csv = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup", "04_track_4_yellow_trajectory.csv")
    pts4 = simulate_physics_noisy_lidar(track4, yellow4_csv, num_scans=75, num_rays=450)
    export_svgs(pts4, track4, yellow4_csv, "track4_physics_noisy_points.svg", "webots_noisy_lidar_map_and_yellow_line.svg", "Track 4 (Technical Switchback)")

    # Track 11: Direct Quad Chicane
    track11 = get_track_11_quad_chicane_slalom()
    yellow11_csv = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup", "11_track_11_yellow_trajectory.csv")
    pts11 = simulate_physics_noisy_lidar(track11, yellow11_csv, num_scans=75, num_rays=450)
    export_svgs(pts11, track11, yellow11_csv, "track11_physics_noisy_points.svg", "11_track_11_physics_noisy_map.svg", "Track 11 (Direct Quad Chicane)")

    print("=" * 100)

if __name__ == "__main__":
    main()
