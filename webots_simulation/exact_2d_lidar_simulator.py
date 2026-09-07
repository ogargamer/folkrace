#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - EXACT 2D LiDAR SIMULATOR (2x SPEED + REALISTIC SENSOR NOISE)
================================================================================
Simulates 2x speed robot progression with realistic MS200k LiDAR noise:
  - Exact first-hit mathematical ray-segment & ray-circle occlusion
  - 8mm Gaussian range standard deviation (sigma = 0.008m)
  - 2x speed (0.50x base speed, ~100 dense sweeps per lap)
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

from racing_engine.track_generator import (
    get_track_4_switchback_arena,
    get_track_7_multi_bottleneck_slalom
)
from racing_engine.centerline_finder import find_centerline_voronoi
from racing_engine.visualizer import OUTPUT_DIR, BG_COLOR, TEXT_COLOR

def cast_lidar_rays_vectorized(origin, headings, segments, circles, max_range=4.0, noise_std=0.008):
    """
    Vectorized raycaster with exact first-hit occlusion and Gaussian range noise.
    """
    K = len(headings)
    M = len(segments)
    
    ray_dirs = np.column_stack([np.cos(headings), np.sin(headings)])
    
    p1 = segments[:, 0, :]
    p2 = segments[:, 1, :]
    v2 = p2 - p1
    v1 = origin[None, :] - p1
    
    v3 = np.column_stack([-ray_dirs[:, 1], ray_dirs[:, 0]])
    dot = np.dot(v2, v3.T)
    
    cross_num = (v2[:, 0, None] * v1[:, 1, None] - v2[:, 1, None] * v1[:, 0, None])
    t2_num = np.dot(v1, v3.T)
    
    valid_dot = np.abs(dot) > 1e-8
    t1 = np.where(valid_dot, cross_num / dot, np.inf)
    t2 = np.where(valid_dot, t2_num / dot, -1.0)
    
    valid_mask = (t1 >= 0.05) & (t1 <= max_range) & (t2 >= 0.0) & (t2 <= 1.0)
    t1_filtered = np.where(valid_mask, t1, np.inf)
    
    min_dists = np.min(t1_filtered, axis=0)
    
    # Check circular obstacles
    for c_pos, r_val in circles:
        oc = origin - c_pos
        b = 2.0 * np.dot(ray_dirs, oc)
        c = np.dot(oc, oc) - r_val * r_val
        disc = b * b - 4.0 * c
        
        valid_disc = disc >= 0.0
        sq = np.sqrt(np.maximum(0.0, disc))
        t_c1 = (-b - sq) * 0.5
        t_c2 = (-b + sq) * 0.5
        
        c_mask1 = valid_disc & (t_c1 >= 0.05) & (t_c1 <= max_range)
        c_mask2 = valid_disc & (t_c2 >= 0.05) & (t_c2 <= max_range)
        
        t_circ = np.where(c_mask1, t_c1, np.where(c_mask2, t_c2, np.inf))
        min_dists = np.minimum(min_dists, t_circ)
        
    valid_rays = min_dists < max_range
    if not np.any(valid_rays):
        return np.empty((0, 2))
        
    final_dists = min_dists[valid_rays]
    if noise_std > 0:
        # Realistic Gaussian range noise (sigma = 8mm)
        final_dists += np.random.normal(0, noise_std, size=len(final_dists))
        
    hit_pts = origin[None, :] + final_dists[:, None] * ray_dirs[valid_rays]
    return hit_pts

def simulate_exact_lidar(track_data, yellow_csv_path, num_scans=100, num_rays=450, max_range=4.0, noise_std=0.008):
    traj = np.loadtxt(yellow_csv_path, delimiter=",", skiprows=1)
    N = len(traj)
    pts = traj[:, :2]
    
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
            
    # 2x Speed -> 100 scans per lap
    scan_indices = np.linspace(0, N - 1, num_scans, dtype=int)
    ray_rel_angles = np.linspace(0, 2 * np.pi, num_rays, endpoint=False)
    
    all_hits = []
    for idx in scan_indices:
        rx, ry = pts[idx]
        next_idx = (idx + 1) % N
        heading = np.arctan2(pts[next_idx, 1] - ry, pts[next_idx, 0] - rx)
        
        global_headings = heading + ray_rel_angles
        hits = cast_lidar_rays_vectorized(np.array([rx, ry]), global_headings, segments, circles, max_range, noise_std)
        if len(hits) > 0:
            all_hits.append(hits)
            
    return np.vstack(all_hits) if len(all_hits) > 0 else np.empty((0, 2))

def plot_track_exact_lidar(track_data, yellow_csv_path, filename, track_title, noise_std=0.008):
    print(f"Simulating 2x speed + {noise_std*1000:.0f}mm noise on {track_title}...")
    hit_pts = simulate_exact_lidar(track_data, yellow_csv_path, num_scans=100, num_rays=450, noise_std=noise_std)
    print(f"  [OK] Captured {len(hit_pts)} noisy LiDAR points across 100 sweeps.")
    
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

    # 1. Ground Truth Outer & Inner Walls
    ax.plot(lw_wrap[:, 0], lw_wrap[:, 1], color="#f43f5e", linewidth=2.5, label="Ground Truth Left Wall (Outer)", zorder=3)
    ax.plot(rw_wrap[:, 0], rw_wrap[:, 1], color="#38bdf8", linewidth=2.5, label="Ground Truth Right Wall (Inner)", zorder=3)

    # 2. Obstacles (Circles)
    for obs in track_data.get("obstacles", []):
        if obs.get("type") == "circle":
            circle = plt.Circle((obs["cx"], obs["cy"]), obs["radius"], color="#fbbf24", alpha=0.95, zorder=4)
            ax.add_patch(circle)

    ax.plot([], [], color="#fbbf24", linewidth=6, label="Ground Truth Obstacle Pillars")

    # 3. Noisy LiDAR Raycast Hits (Cyan)
    ax.scatter(hit_pts[:, 0], hit_pts[:, 1], color="#00f0ff", s=1.6, alpha=0.55,
               label=f"MS200k 2D LiDAR (2x Speed, {noise_std*1000:.0f}mm Gaussian Noise, {len(hit_pts)} pts)", zorder=2)

    # 4. Medial Centerline
    ax.plot(c_wrap[:, 0], c_wrap[:, 1], color="#475569", linewidth=1.2, linestyle=":", label="Medial Centerline (Voronoi)", zorder=5)

    # 5. Optimal Yellow Line
    ax.plot(yellow_wrap[:, 0], yellow_wrap[:, 1], color="#ffd700", linewidth=3.0, label="Optimal Yellow Racing Line (2x Speed)", zorder=6)

    # 6. Start Position
    ax.scatter([yellow_pts[0, 0]], [yellow_pts[0, 1]], color="#ffffff", s=150, edgecolors="#ffd700", linewidth=2, label="Robot Start Position", zorder=7)

    ax.set_title(f"{track_title} - 2D LiDAR Simulation (2x Speed, {noise_std*1000:.0f}mm Noise) & Yellow Line",
                 color=TEXT_COLOR, fontsize=14, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.25, color="#fff")
    ax.tick_params(colors=TEXT_COLOR)
    ax.legend(bbox_to_anchor=(1.02, 1.0), loc="upper left", facecolor="#121826", edgecolor="#2d3748",
              labelcolor=TEXT_COLOR, framealpha=1.0, fontsize=9.5, borderpad=1.0, labelspacing=0.8)

    svg_path = os.path.join(OUTPUT_DIR, filename)
    plt.savefig(svg_path, format="svg", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)

    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    shutil.copy2(svg_path, os.path.join(art_dir, filename))
    print(f"  [OK] Saved SVG to: {svg_path}")

def main():
    print("=" * 100)
    print("   2D LiDAR SIMULATION (2x SPEED + 8mm REALISTIC GAUSSIAN NOISE)")
    print("=" * 100)

    # Track 7: Slalom Speedway
    track7 = get_track_7_multi_bottleneck_slalom()
    yellow7_csv = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup", "07_track_7_yellow_trajectory.csv")
    plot_track_exact_lidar(track7, yellow7_csv, "07_track_7_webots_deskewed_lidar_map.svg", "Track 7 (Slalom Speedway)", noise_std=0.008)

    # Track 4: Technical Switchback
    track4 = get_track_4_switchback_arena()
    yellow4_csv = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup", "04_track_4_yellow_trajectory.csv")
    plot_track_exact_lidar(track4, yellow4_csv, "webots_noisy_lidar_map_and_yellow_line.svg", "Track 4 (Technical Switchback)", noise_std=0.008)

    print("\n" + "=" * 100)

if __name__ == "__main__":
    main()
