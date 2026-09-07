#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - WEBOTS MS200k LiDAR MAP & TRAJECTORY EXTRACTOR
================================================================================
Visualizes:
  - Ground Truth Track Walls (Left & Right) + Obstacle Pillars
  - Exact Yellow Line followed by the robot in Webots
  - MS200k 2D LiDAR Scan Hits (Cyan)
================================================================================
"""

import os
import sys
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import scipy.spatial as spatial

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE_DIR)

from racing_engine.track_generator import get_track_4_switchback_arena
from racing_engine.centerline_finder import find_centerline_voronoi
from racing_engine.visualizer import OUTPUT_DIR, BG_COLOR, TEXT_COLOR

def main():
    print("=" * 90)
    print("   GENERATING HIGH-ACCURACY WEBOTS LiDAR & GROUND TRUTH OVERLAY")
    print("=" * 90)

    pts_path = os.path.join(BASE_DIR, "webots_simulation", "controllers", "robot_lidar_collector", "noisy_lidar_map_points.npy")
    if not os.path.exists(pts_path):
        print(f"[ERROR] LiDAR points not found: {pts_path}")
        return

    lidar_pts = np.load(pts_path)
    
    # Filter valid LiDAR points (exclude max range floor/boundary hits outside track envelope)
    valid_mask = (lidar_pts[:, 0] >= -2.0) & (lidar_pts[:, 0] <= 5.5) & (lidar_pts[:, 1] >= -1.5) & (lidar_pts[:, 1] <= 5.0)
    filtered_lidar = lidar_pts[valid_mask]
    
    # 20mm Voxel grid quantization
    quantized = np.round(filtered_lidar / 0.02) * 0.02
    unique_pts = np.unique(quantized, axis=0)

    track = get_track_4_switchback_arena()
    lw = np.array(track["left_wall"])
    rw = np.array(track["right_wall"])
    lw_wrap = np.vstack([lw, lw[:1]])
    rw_wrap = np.vstack([rw, rw[:1]])
    
    # Load exact Yellow trajectory followed by the robot
    yellow_csv = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup", "04_track_4_yellow_trajectory.csv")
    traj = np.loadtxt(yellow_csv, delimiter=",", skiprows=1)
    yellow_pts = traj[:, :2]
    yellow_wrap = np.vstack([yellow_pts, yellow_pts[:1]])

    vor_res = find_centerline_voronoi(track)
    centerline = vor_res["centerline"]
    c_wrap = np.vstack([centerline, centerline[:1]])

    fig, ax = plt.subplots(figsize=(18, 12), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)

    # 1. Ground Truth Outer & Inner Walls
    ax.plot(lw_wrap[:, 0], lw_wrap[:, 1], color="#f43f5e", linewidth=2.5, label="Ground Truth Left Wall (Outer)", zorder=3)
    ax.plot(rw_wrap[:, 0], rw_wrap[:, 1], color="#38bdf8", linewidth=2.5, label="Ground Truth Right Wall (Inner)", zorder=3)

    # 2. Obstacles / Pillars
    for obs in track.get("obstacles", []):
        if obs.get("type") == "circle":
            circle = plt.Circle((obs["cx"], obs["cy"]), obs["radius"], color="#fbbf24", alpha=0.9, zorder=4)
            ax.add_patch(circle)
    ax.plot([], [], color="#fbbf24", linewidth=6, label="Ground Truth Pillar Obstacles")

    # 3. Webots MS200k Raw LiDAR Hits (Subsampled 1-in-2 for crisp rendering)
    sub_pts = filtered_lidar[::2]
    ax.scatter(sub_pts[:, 0], sub_pts[:, 1], color="#00f0ff", s=1.8, alpha=0.5,
               label=f"Webots MS200k LiDAR Scans (4m Max Range, 4mm Noise, {len(filtered_lidar)} pts)", zorder=2)

    # 4. Medial Centerline
    ax.plot(c_wrap[:, 0], c_wrap[:, 1], color="#475569", linewidth=1.2, linestyle=":", label="Medial Centerline (Voronoi)", zorder=5)

    # 5. Exact Yellow Line followed
    ax.plot(yellow_wrap[:, 0], yellow_wrap[:, 1], color="#ffd700", linewidth=3.0, label="Optimal Yellow Line (Followed in Webots @ 0.5x Speed)", zorder=6)

    # 6. Start Position
    ax.scatter([yellow_pts[0, 0]], [yellow_pts[0, 1]], color="#ffffff", s=150, edgecolors="#ffd700", linewidth=2, label="Robot Start Position", zorder=7)

    ax.set_title("Track 4: Webots MS200k LiDAR Map (4m Range, 4mm Noise) & Optimal Yellow Racing Line",
                 color=TEXT_COLOR, fontsize=14, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.25, color="#fff")
    ax.tick_params(colors=TEXT_COLOR)
    ax.set_xlim(-1.8, 5.2)
    ax.set_ylim(-1.2, 4.8)
    ax.legend(bbox_to_anchor=(1.02, 1.0), loc="upper left", facecolor="#121826", edgecolor="#2d3748",
              labelcolor=TEXT_COLOR, framealpha=1.0, fontsize=9.5, borderpad=1.0, labelspacing=0.8)

    filename = "webots_noisy_lidar_map_and_yellow_line.svg"
    svg_path = os.path.join(OUTPUT_DIR, filename)
    plt.savefig(svg_path, format="svg", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)

    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    shutil.copy2(svg_path, os.path.join(art_dir, filename))

    print(f"[OK] Visualized SVG saved to: {svg_path} ({os.path.getsize(svg_path)/1024:.1f} KB)")
    print("=" * 90)

if __name__ == "__main__":
    main()
