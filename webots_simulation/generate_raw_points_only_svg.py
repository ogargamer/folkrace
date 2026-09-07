#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - FULL RACING SPEED (1.0x) + 15mm NOISE + POINTS-ONLY SVG EXPORTER
================================================================================
- 1.0x Full Racing Speed (2.2 to 4.0 m/s)
- 15mm Gaussian LiDAR Range Noise (sigma = 0.015m)
- Generates:
    1. Pure Raw LiDAR Point Cloud SVG (Points only, no lines or walls)
    2. Overlaid Trajectory Verification SVG
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
from exact_2d_lidar_simulator import cast_lidar_rays_vectorized

def simulate_full_speed_lidar(track_data, yellow_csv_path, num_scans=60, num_rays=450, max_range=4.0, noise_std=0.015):
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
            
    # Full 1.0x Racing Speed -> ~60 sweeps per lap
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

def export_points_only_svg(hit_pts, filename, title):
    """
    Exports an SVG containing ONLY the raw LiDAR point cloud on a clean dark background.
    """
    fig, ax = plt.subplots(figsize=(18, 11), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)

    # Plot ONLY raw LiDAR scan points
    ax.scatter(hit_pts[:, 0], hit_pts[:, 1], color="#00f0ff", s=2.2, alpha=0.75, edgecolors='none')

    ax.set_title(f"{title} - Raw MS200k LiDAR Points Only (Full 1.0x Speed, 15mm Noise, {len(hit_pts)} pts)",
                 color=TEXT_COLOR, fontsize=13, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.15, color="#fff")
    ax.tick_params(colors=TEXT_COLOR)

    svg_path = os.path.join(OUTPUT_DIR, filename)
    plt.savefig(svg_path, format="svg", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)

    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    shutil.copy2(svg_path, os.path.join(art_dir, filename))
    print(f"  [OK] Saved Points-Only SVG: {svg_path}")

def export_overlaid_svg(hit_pts, track_data, yellow_csv_path, filename, title):
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

    # 1. Ground Truth Walls
    ax.plot(lw_wrap[:, 0], lw_wrap[:, 1], color="#f43f5e", linewidth=2.0, label="Ground Truth Left Wall", zorder=3)
    ax.plot(rw_wrap[:, 0], rw_wrap[:, 1], color="#38bdf8", linewidth=2.0, label="Ground Truth Right Wall", zorder=3)

    # 2. Obstacles
    for obs in track_data.get("obstacles", []):
        if obs.get("type") == "circle":
            circle = plt.Circle((obs["cx"], obs["cy"]), obs["radius"], color="#fbbf24", alpha=0.9, zorder=4)
            ax.add_patch(circle)

    # 3. Noisy LiDAR Raycast Hits (Cyan)
    ax.scatter(hit_pts[:, 0], hit_pts[:, 1], color="#00f0ff", s=1.8, alpha=0.6,
               label=f"MS200k LiDAR (1.0x Full Speed, 15mm Noise, {len(hit_pts)} pts)", zorder=2)

    # 4. Centerline & Yellow Line
    ax.plot(c_wrap[:, 0], c_wrap[:, 1], color="#475569", linewidth=1.0, linestyle=":", label="Medial Centerline", zorder=5)
    ax.plot(yellow_wrap[:, 0], yellow_wrap[:, 1], color="#ffd700", linewidth=2.8, label="Optimal Yellow Racing Line (1.0x Speed)", zorder=6)

    # 5. Start Position
    ax.scatter([yellow_pts[0, 0]], [yellow_pts[0, 1]], color="#ffffff", s=140, edgecolors="#ffd700", linewidth=2, label="Robot Start", zorder=7)

    ax.set_title(f"{title} - 1.0x Full Speed & 15mm Noise Overlay",
                 color=TEXT_COLOR, fontsize=14, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.2, color="#fff")
    ax.tick_params(colors=TEXT_COLOR)
    ax.legend(bbox_to_anchor=(1.02, 1.0), loc="upper left", facecolor="#121826", edgecolor="#2d3748",
              labelcolor=TEXT_COLOR, framealpha=1.0, fontsize=9.5, borderpad=1.0, labelspacing=0.8)

    svg_path = os.path.join(OUTPUT_DIR, filename)
    plt.savefig(svg_path, format="svg", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)

    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    shutil.copy2(svg_path, os.path.join(art_dir, filename))
    print(f"  [OK] Saved Overlaid SVG: {svg_path}")

def main():
    print("=" * 100)
    print("   GENERATING 1.0x FULL RACING SPEED + 15mm NOISE LIDAR MAPS")
    print("=" * 100)

    # Track 7: Slalom Speedway (Full 1.0x Speed + 15mm Noise)
    track7 = get_track_7_multi_bottleneck_slalom()
    yellow7_csv = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup", "07_track_7_yellow_trajectory.csv")
    pts7 = simulate_full_speed_lidar(track7, yellow7_csv, num_scans=60, num_rays=450, noise_std=0.015)
    
    export_points_only_svg(pts7, "track7_points_only.svg", "Track 7 (Slalom Speedway)")
    export_overlaid_svg(pts7, track7, yellow7_csv, "07_track_7_webots_deskewed_lidar_map.svg", "Track 7 (Slalom Speedway)")

    # Track 4: Technical Switchback (Full 1.0x Speed + 15mm Noise)
    track4 = get_track_4_switchback_arena()
    yellow4_csv = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup", "04_track_4_yellow_trajectory.csv")
    pts4 = simulate_full_speed_lidar(track4, yellow4_csv, num_scans=60, num_rays=450, noise_std=0.015)
    
    export_points_only_svg(pts4, "track4_points_only.svg", "Track 4 (Technical Switchback)")
    export_overlaid_svg(pts4, track4, yellow4_csv, "webots_noisy_lidar_map_and_yellow_line.svg", "Track 4 (Technical Switchback)")

    print("=" * 100)

if __name__ == "__main__":
    main()
