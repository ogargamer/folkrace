#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - AUTOMATED CAD YELLOW RACING LINE GENERATOR FROM TRACK JSON
================================================================================
Takes a Track Studio layout JSON, directly uses track_generator's exact corridor
builder and Voronoi Medial Axis skeletonizer, optimizes the TUM Minimum Curvature
CAD Yellow Racing Line, computes the friction velocity profile, and exports:
  1. Trajectory CSV (x, y, v, yaw, curvature, distance, time, accel)
  2. High-Resolution CAD PNG & Vector SVG with Sidebar Telemetry
  3. Webots 3D Simulation World (.wbt in ENU Standard)
================================================================================
"""

import os
import sys
import json
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE_DIR)

from racing_engine.track_generator import load_track_from_json, build_raw_track_corridor
from racing_engine.centerline_finder import find_centerline_voronoi
from racing_engine.racing_line_optimizer import TUMMinimumCurvatureOptimizer
from track_builder.export_webots import convert_json_to_webots_wbt

# Visual styling
BG_COLOR = "#07090e"
CARD_BG = "#0d121d"
BORDER_COLOR = "#334155"
WALL_COLOR = "#ff0055"
YELLOW_LINE_COLOR = "#ffea00"
CENTERLINE_COLOR = "#00f0ff"
OBSTACLE_COLOR = "#ffb800"
TEXT_COLOR = "#f0f4f8"

def generate_yellow_racing_line(json_path, output_dir=None, safety_margin=0.10, max_speed=4.0, friction_coeff=0.954):
    print("=" * 72)
    print(f"  DA WINNA - TUM CAD YELLOW RACING LINE OPTIMIZATION")
    print(f"  Input Track JSON: {json_path}")
    print("=" * 72)

    # 1. Load exact track geometry via track_generator
    track_data = load_track_from_json(json_path)

    if output_dir is None:
        base_name = os.path.splitext(os.path.basename(json_path))[0]
        output_dir = os.path.join(BASE_DIR, "racing_results", base_name)
    os.makedirs(output_dir, exist_ok=True)

    left_wall = track_data["left_wall"]
    right_wall = track_data["right_wall"]
    num_blocks = len(track_data.get("blocks", []))
    num_obs = len(track_data.get("obstacles", []))

    print(f"[1/4] Reconstructed track corridor: {num_blocks} blocks, {num_obs} obstacles.")

    # 2. Compute Medial Axis / Voronoi Centerline
    vor_res = find_centerline_voronoi(track_data, num_points=360)
    vor_centerline = vor_res["centerline"] if isinstance(vor_res, dict) else vor_res
    print(f"[2/4] Calculated Voronoi reference centerline ({len(vor_centerline)} waypoints).")

    # 3. Optimize TUM Minimum Curvature Yellow Racing Line
    optimizer = TUMMinimumCurvatureOptimizer(
        entry_aggressiveness=1.0,
        exit_aggressiveness=1.0,
        num_progression_stages=10,
        max_solver_sub_iters=800,
        robot_safety_margin=safety_margin,
        weight_curvature=3.50,
        weight_path_smoothness=0.03,
        weight_jerk=0.35,
        macro_sigma=4.0,
        spline_s=0.015,
        max_speed=max_speed,
        friction_coeff=friction_coeff,
        max_accel=2.4,
        max_brake=3.8
    )

    opt_res = optimizer.optimize_racing_line(
        track_data,
        {"centerline": vor_centerline}
    )

    racing_line = opt_res["racing_line"]
    velocities = opt_res["velocities"]
    curvature = opt_res["curvature"]
    tangents = opt_res["tangents"]
    ds = opt_res["ds"]
    N = len(racing_line)

    yaw = np.arctan2(tangents[:, 1], tangents[:, 0])
    s_dist = np.cumsum(np.concatenate([[0.0], ds[:-1]]))
    v_safe = np.maximum(velocities, 0.1)
    dt = ds / v_safe
    t_time = np.cumsum(np.concatenate([[0.0], dt[:-1]]))
    a_lon = np.gradient(velocities**2) / (2.0 * ds + 1e-6)

    track_len = float(np.sum(ds))
    lap_time = float(np.sum(dt))
    v_top = float(np.max(velocities))
    v_avg = float(track_len / max(lap_time, 1e-3))
    max_lat_g = float(np.max(velocities**2 * np.abs(curvature)) / 9.81)

    print(f"[3/4] QP Optimization Complete!")
    print(f"      • Track Length:  {track_len:.2f} m")
    print(f"      • Lap Time:      {lap_time:.2f} s")
    print(f"      • Top Speed:     {v_top:.2f} m/s ({v_top * 3.6:.1f} km/h)")
    print(f"      • Average Speed: {v_avg:.2f} m/s ({v_avg * 3.6:.1f} km/h)")
    print(f"      • Max Lateral G: {max_lat_g:.2f} G")

    # 4. Save Trajectory CSV
    csv_path = os.path.join(output_dir, "yellow_racing_line_trajectory.csv")
    traj_data = np.column_stack([
        racing_line[:, 0],
        racing_line[:, 1],
        velocities,
        yaw,
        curvature,
        s_dist,
        t_time,
        a_lon
    ])
    header = "x_m,y_m,v_mps,yaw_rad,curvature_inv_m,dist_m,time_s,a_lon_mps2"
    np.savetxt(csv_path, traj_data, delimiter=",", header=header, comments="", fmt="%.5f")
    print(f"[4/4] Saved Trajectory CSV: {csv_path}")

    # 5. Export CAD Vector Overlay Plot (PNG & SVG)
    fig, ax = plt.subplots(figsize=(15, 10), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)

    # Road boundaries
    ax.plot(left_wall[:, 0], left_wall[:, 1], color=WALL_COLOR, linewidth=3.0, label="Track Outer Wall", zorder=3)
    ax.plot(right_wall[:, 0], right_wall[:, 1], color=WALL_COLOR, linewidth=3.0, label="Track Inner Wall", zorder=3)

    # Reference Centerline
    ax.plot(vor_centerline[:, 0], vor_centerline[:, 1], color=CENTERLINE_COLOR, linestyle="--", linewidth=1.2, alpha=0.6, label="Voronoi Centerline", zorder=4)

    # Obstacles
    for obs in track_data.get("obstacles", []):
        obs_type = obs.get("type", "circle")
        if obs_type == "circle":
            c = patches.Circle((obs["cx"], obs["cy"]), obs["radius"], color=OBSTACLE_COLOR, alpha=0.9, zorder=5)
            ax.add_patch(c)
        elif obs_type in ["rect", "wall_rectangle"]:
            cx, cy = obs["center_x"], obs["center_y"]
            rw, rh = obs["width"], obs["extrusion"]
            rect = patches.Rectangle((cx - rw/2.0, cy - rh/2.0), rw, rh, color=OBSTACLE_COLOR, alpha=0.9, zorder=5)
            ax.add_patch(rect)

    # CAD Yellow Racing Line
    ax.plot(racing_line[:, 0], racing_line[:, 1], color=YELLOW_LINE_COLOR, linewidth=3.5, label="TUM CAD Yellow Racing Line (QP)", zorder=6)

    # Start line / Robot pose
    init_pose = track_data.get("initial_pose", {})
    sx, sy = init_pose.get("x", 0.6), init_pose.get("y", 0.0)
    ax.scatter([sx], [sy], color="#00ff88", s=120, edgecolors="white", linewidth=1.5, label="Start / Spawn Point", zorder=7)

    # Sidebar text info
    info_text = (
        f"DA WINNA - RACING ENGINE\n"
        f"------------------------\n"
        f"Track Blocks:  {num_blocks} tiles\n"
        f"Track Length:  {track_len:.2f} m\n"
        f"Lap Time:      {lap_time:.2f} s\n"
        f"Top Speed:     {v_top:.2f} m/s\n"
        f"Average Speed: {v_avg:.2f} m/s\n"
        f"Max Lateral G: {max_lat_g:.2f} G\n"
        f"Safety Margin: {safety_margin * 100:.0f} cm\n"
        f"Friction (mu): {friction_coeff:.2f}\n"
    )
    plt.gcf().text(0.80, 0.65, info_text, fontsize=11, family="monospace", color="#00f0ff",
                   bbox=dict(boxstyle="round,pad=0.8", facecolor=CARD_BG, edgecolor=BORDER_COLOR, alpha=0.95))

    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", color="#1e293b", alpha=0.6)
    ax.set_title("DA WINNA - TUM CAD YELLOW RACING LINE (OPTIMAL QP TRAJECTORY)", color=TEXT_COLOR, fontsize=14, weight="bold", pad=15)
    ax.legend(loc="lower left", facecolor=CARD_BG, edgecolor=BORDER_COLOR, labelcolor=TEXT_COLOR, fontsize=9)
    plt.subplots_adjust(right=0.76)

    png_path = os.path.join(output_dir, "yellow_racing_line.png")
    svg_path = os.path.join(output_dir, "yellow_racing_line.svg")
    plt.savefig(png_path, dpi=200, bbox_inches="tight", facecolor=BG_COLOR)
    plt.savefig(svg_path, format="svg", bbox_inches="tight", facecolor=BG_COLOR)
    plt.close()

    print(f"      • Saved CAD Visual PNG: {png_path}")
    print(f"      • Saved CAD Visual SVG: {svg_path}")

    # 6. Generate Webots .wbt World
    wbt_path = os.path.join(output_dir, "track_world_enu.wbt")
    convert_json_to_webots_wbt(json_path, wbt_path)

    print("=" * 72)
    print("  ALL ASSETS GENERATED SUCCESSFULLY!")
    print("=" * 72)
    return {
        "csv": csv_path,
        "png": png_path,
        "svg": svg_path,
        "wbt": wbt_path,
        "lap_time": lap_time,
        "track_len": track_len,
        "top_speed": v_top
    }

def main():
    parser = argparse.ArgumentParser(description="Generate TUM CAD Yellow Racing Line from Modular Track JSON")
    parser.add_argument("json_file", help="Path to track layout JSON file")
    parser.add_argument("-o", "--output-dir", help="Output directory for results", default=None)
    parser.add_argument("--margin", help="Safety margin in meters (default: 0.10)", type=float, default=0.10)
    parser.add_argument("--max-speed", help="Max velocity in m/s (default: 4.0)", type=float, default=4.0)
    parser.add_argument("--friction", help="Tire friction coefficient (default: 0.954)", type=float, default=0.954)
    args = parser.parse_args()

    if not os.path.exists(args.json_file):
        print(f"Error: JSON file not found: {args.json_file}")
        sys.exit(1)

    generate_yellow_racing_line(
        json_path=args.json_file,
        output_dir=args.output_dir,
        safety_margin=args.margin,
        max_speed=args.max_speed,
        friction_coeff=args.friction
    )

if __name__ == "__main__":
    main()
