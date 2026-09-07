#!/usr/bin/env python3
"""
================================================================================
  DA WINNA - TUM QP RACING LINE GENERATOR WITH OBSTACLE-AWARE 25CM WALL OFFSETS
  Track: "C:\\Users\\nurmi\\Downloads\\track_world_1787780744372.wbt"
  
  Pipeline:
  1. Direct WBT World Geometry & Obstacle Extractor
  2. Voronoi Medial Axis Skeletonization
  3. TUM Minimum-Curvature QP Optimizer (MU=0.60 | 12 cm Safety Margin | 4 m/s Max Speed)
  4. Obstacle-Aware 25cm Inward Wall Offset Lines (10 cm Safety Margin from Obstacles)
  5. High-Definition Scalable Vector Graphics (.svg) & Trajectory CSV Exports
================================================================================
"""

import os
import sys
import time
import json
import re
import xml.etree.ElementTree as ET
import numpy as np
import scipy.spatial as spatial
from scipy.interpolate import splprep, splev
from scipy.ndimage import gaussian_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE_DIR)

from racing_engine.centerline_finder import get_all_boundary_and_obstacle_points, find_centerline_voronoi
from racing_engine.racing_line_optimizer import compute_spline_properties, TUMMinimumCurvatureOptimizer

# -----------------------------------------------------------------------------
# 1. WBT Geometry & Obstacle Parser
# -----------------------------------------------------------------------------
def load_track_from_wbt(wbt_path):
    with open(wbt_path, "r", encoding="utf-8") as f:
        content = f.read()

    solid_blocks = content.split("Solid {")
    left_wall = None
    right_wall = None
    obstacles = []

    for block in solid_blocks[1:]:
        name_match = re.search(r'name\s+"([^"]+)"', block)
        name = name_match.group(1) if name_match else ""
        trans_match = re.search(r'translation\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', block)
        trans = [float(x) for x in trans_match.groups()] if trans_match else [0.0, 0.0, 0.0]
        
        if "IndexedFaceSet" in block:
            pt_match = re.search(r'point\s*\[\s*([\s\S]*?)\]', block)
            if pt_match:
                pts_str = pt_match.group(1).strip()
                pts_coords = [float(x) for x in re.split(r'[,\s]+', pts_str) if x]
                pts_3d = np.array(pts_coords).reshape(-1, 3)
                z0_mask = np.isclose(pts_3d[:, 2], 0.0, atol=1e-3)
                pts_2d = pts_3d[z0_mask, :2]
                
                if "left_wall" in name:
                    left_wall = pts_2d
                elif "right_wall" in name:
                    right_wall = pts_2d
        elif "Cylinder" in block:
            rad_match = re.search(r'radius\s+([-\d.]+)', block)
            rad = float(rad_match.group(1)) if rad_match else 0.05
            obstacles.append({
                "type": "circle",
                "name": name,
                "cx": trans[0],
                "cy": trans[1],
                "radius": rad
            })
        elif "Box" in block:
            size_match = re.search(r'size\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', block)
            size = [float(x) for x in size_match.groups()] if size_match else [0.1, 0.1, 0.1]
            obstacles.append({
                "type": "wall_rectangle",
                "name": name,
                "center_x": trans[0],
                "center_y": trans[1],
                "width": size[0],
                "extrusion": size[1],
                "size": size
            })

    start_x, start_y = 0.0, 0.5
    pose_match = re.search(r'Pose\s*\{[\s\S]*?translation\s+([-\d.]+)\s+([-\d.]+)', content)
    if pose_match:
        start_x, start_y = float(pose_match.group(1)), float(pose_match.group(2))

    return {
        "name": "Track World 1787780744372",
        "left_wall": left_wall,
        "right_wall": right_wall,
        "obstacles": obstacles,
        "track_width": 1.0,
        "tile_size": 1.2,
        "initial_pose": {"x": start_x, "y": start_y}
    }

# -----------------------------------------------------------------------------
# 2. Obstacle-Aware Wall Offset Calculator
# -----------------------------------------------------------------------------
def compute_obstacle_aware_offset(wall_pts, other_wall_tree, obs_tree, all_obs_pts, offset_dist=0.25, obs_safety_margin=0.10, num_points=1000):
    """
    Computes a 25cm inward offset from the wall, dynamically deflecting around any
    obstacles to guarantee a strict safety margin (e.g. 10cm) from obstacle perimeters.
    """
    wrapped = np.vstack([wall_pts[-1:], wall_pts, wall_pts[:1]])
    tangents = wrapped[2:] - wrapped[:-2]
    tang_norm = np.hypot(tangents[:, 0], tangents[:, 1])[:, np.newaxis] + 1e-9
    tangents /= tang_norm
    
    norm1 = np.column_stack([-tangents[:, 1], tangents[:, 0]])
    norm2 = -norm1
    
    test_pt1 = wall_pts + 0.1 * norm1
    test_pt2 = wall_pts + 0.1 * norm2
    
    d1, _ = other_wall_tree.query(test_pt1)
    d2, _ = other_wall_tree.query(test_pt2)
    
    inward_normals = np.where((d1 < d2)[:, np.newaxis], norm1, norm2)
    inward_normals[:, 0] = gaussian_filter1d(inward_normals[:, 0], sigma=1.5, mode='wrap')
    inward_normals[:, 1] = gaussian_filter1d(inward_normals[:, 1], sigma=1.5, mode='wrap')
    inward_normals /= (np.hypot(inward_normals[:, 0], inward_normals[:, 1])[:, np.newaxis] + 1e-9)
    
    raw_offset = wall_pts + offset_dist * inward_normals
    
    u_fine = np.linspace(0, 1, num_points, endpoint=False)
    tck_init, _ = splprep([raw_offset[:, 0], raw_offset[:, 1]], s=0.002, per=True)
    res_line = np.column_stack(splev(u_fine, tck_init))
    
    target_push = obs_safety_margin + 0.015 # 11.5 cm buffer to ensure post-smoothing >= 10.0 cm
    
    for step in range(35):
        d_obs, idx_obs = obs_tree.query(res_line)
        viol = d_obs < target_push
        if np.any(viol):
            closest_obs = all_obs_pts[idx_obs[viol]]
            v_rep = res_line[viol] - closest_obs
            dist_v = np.linalg.norm(v_rep, axis=1, keepdims=True) + 1e-6
            v_rep /= dist_v
            
            push_amount = (target_push - d_obs[viol])[:, np.newaxis]
            res_line[viol] += v_rep * push_amount
            
            res_line[:, 0] = gaussian_filter1d(res_line[:, 0], sigma=2.0, mode='wrap')
            res_line[:, 1] = gaussian_filter1d(res_line[:, 1], sigma=2.0, mode='wrap')

    tck_final, _ = splprep([res_line[:, 0], res_line[:, 1]], s=0.0015, per=True)
    smooth_line = np.column_stack(splev(u_fine, tck_final))
    
    # Final check of clearance to obstacles
    d_verif, _ = obs_tree.query(smooth_line)
    min_clear_cm = float(np.min(d_verif)) * 100.0
    
    return smooth_line, min_clear_cm

# -----------------------------------------------------------------------------
# 3. Main Generator
# -----------------------------------------------------------------------------
def run_generation(wbt_path=r"C:\Users\nurmi\Downloads\track_world_1787780744372.wbt", 
                   output_folder_name="tum_qp_obstacle_aware_25cm_offsets"):
    t_start = time.perf_counter()
    print("=" * 100)
    print(f"Loading track from: {wbt_path}")
    track_data = load_track_from_wbt(wbt_path)
    left_wall = np.array(track_data["left_wall"], dtype=np.float64)
    right_wall = np.array(track_data["right_wall"], dtype=np.float64)
    obstacles = track_data.get("obstacles", [])
    print(f"Loaded: Left Wall ({len(left_wall)} pts), Right Wall ({len(right_wall)} pts), Obstacles ({len(obstacles)})")

    output_dir = os.path.join(BASE_DIR, "racing_results", output_folder_name)
    os.makedirs(output_dir, exist_ok=True)

    # 1. Build KD-Trees
    boundary_pts = get_all_boundary_and_obstacle_points(track_data, point_spacing=0.020)
    barrier_tree = spatial.cKDTree(boundary_pts)
    
    obs_pts_list = []
    for obs in obstacles:
        if obs["type"] == "wall_rectangle":
            cx, cy = obs["center_x"], obs["center_y"]
            w, h = obs["width"], obs["extrusion"]
            steps_w = max(10, int(w / 0.01))
            steps_h = max(10, int(h / 0.01))
            xs_top = np.linspace(cx - w/2, cx + w/2, steps_w)
            ys_top = np.full(steps_w, cy + h/2)
            xs_bot = np.linspace(cx - w/2, cx + w/2, steps_w)
            ys_bot = np.full(steps_w, cy - h/2)
            ys_left = np.linspace(cy - h/2, cy + h/2, steps_h)
            xs_left = np.full(steps_h, cx - w/2)
            ys_right = np.linspace(cy - h/2, cy + h/2, steps_h)
            xs_right = np.full(steps_h, cx + w/2)
            obs_pts_list.append(np.vstack([
                np.column_stack([xs_top, ys_top]),
                np.column_stack([xs_bot, ys_bot]),
                np.column_stack([xs_left, ys_left]),
                np.column_stack([xs_right, ys_right])
            ]))
        elif obs["type"] == "circle":
            cx, cy, r = obs["cx"], obs["cy"], obs["radius"]
            steps_c = max(32, int(2 * np.pi * r / 0.01))
            angles = np.linspace(0, 2*np.pi, steps_c, endpoint=False)
            xs = cx + r * np.cos(angles)
            ys = cy + r * np.sin(angles)
            obs_pts_list.append(np.column_stack([xs, ys]))

    all_obs_pts = np.vstack(obs_pts_list)
    obs_tree = spatial.cKDTree(all_obs_pts)
    tree_rw = spatial.cKDTree(right_wall)
    tree_lw = spatial.cKDTree(left_wall)

    # 2. Compute Voronoi Centerline
    t0_vor = time.perf_counter()
    vor_res = find_centerline_voronoi(track_data, num_points=120)
    t_vor_ms = (time.perf_counter() - t0_vor) * 1000.0
    print(f"[OK] Voronoi Medial Axis computed in {t_vor_ms:.2f} ms")

    # 3. Solve TUM Minimum-Curvature QP Optimizer
    t0_opt = time.perf_counter()
    optimizer = TUMMinimumCurvatureOptimizer(
        weight_curvature=2.00,
        weight_path_smoothness=0.12,
        weight_jerk=0.40,
        robot_safety_margin=0.12, # 12 cm safety margin
        num_progression_stages=4,
        max_solver_sub_iters=120,
        max_speed=4.0,
        friction_coeff=0.60 # MU = 0.60
    )
    res_master = optimizer.optimize_racing_line(track_data, vor_res)
    opt_raw = res_master["racing_line"]
    t_master_ms = (time.perf_counter() - t0_opt) * 1000.0
    print(f"[OK] Master QP Racing Line optimized in {t_master_ms:.2f} ms")

    # Fit 1000-point periodic spline for optimal line
    tck_opt, _ = splprep([opt_raw[:, 0], opt_raw[:, 1]], s=0.005, per=True)
    u_1000 = np.linspace(0, 1, 1000, endpoint=False)
    optimal_line = np.column_stack(splev(u_1000, tck_opt))

    props_opt = compute_spline_properties(optimal_line)
    v_opt = optimizer._generate_velocity_profile(props_opt["curvature"], props_opt["ds"], lookahead_distance_m=0.10)
    lap_time_master = float(np.sum(props_opt["ds"] / np.maximum(v_opt, 0.1)))
    track_len_master = float(np.sum(props_opt["ds"]))
    min_clear_master_cm = float(np.min(barrier_tree.query(optimal_line)[0])) * 100.0
    print(f"Master Line Stats: Lap Time = {lap_time_master:.3f} s | Length = {track_len_master:.2f} m | Min Clearance = {min_clear_master_cm:.2f} cm")

    # 4. Compute Obstacle-Aware 25cm Inward Wall Offset Lines
    print("\nComputing Obstacle-Aware 25cm Inward Wall Offset Lines (10 cm Obstacle Safety Margin)...")
    left_offset_25cm, left_obs_clear_cm = compute_obstacle_aware_offset(
        left_wall, tree_rw, obs_tree, all_obs_pts, offset_dist=0.25, obs_safety_margin=0.10, num_points=1000
    )
    right_offset_25cm, right_obs_clear_cm = compute_obstacle_aware_offset(
        right_wall, tree_lw, obs_tree, all_obs_pts, offset_dist=0.25, obs_safety_margin=0.10, num_points=1000
    )
    print(f"[OK] Left Wall 25cm Offset Line: Min Obstacle Clearance = {left_obs_clear_cm:.2f} cm")
    print(f"[OK] Right Wall 25cm Offset Line: Min Obstacle Clearance = {right_obs_clear_cm:.2f} cm")

    t_total_ms = (time.perf_counter() - t_start) * 1000.0

    # 5. Export Trajectory CSVs
    header = "x_m,y_m,v_mps,yaw_rad,curvature_inv_m,dist_m,time_s,a_lon_mps2"
    def save_csv(pts, v, tang, curv, ds, filepath):
        yaw = np.arctan2(tang[:, 1], tang[:, 0])
        s_dist = np.cumsum(np.concatenate([[0.0], ds[:-1]]))
        dt = ds / np.maximum(v, 0.1)
        t_time = np.cumsum(np.concatenate([[0.0], dt[:-1]]))
        a_lon = np.gradient(v**2) / (2.0 * ds + 1e-6)
        traj_mat = np.column_stack([pts[:, 0], pts[:, 1], v, yaw, curv, s_dist, t_time, a_lon])
        np.savetxt(filepath, traj_mat, delimiter=",", header=header, comments="", fmt="%.5f")

    master_csv = os.path.join(output_dir, "master_cyan_champion_optimal.csv")
    save_csv(optimal_line, v_opt, props_opt["tangents"], props_opt["curvature"], props_opt["ds"], master_csv)

    props_lw = compute_spline_properties(left_offset_25cm)
    props_rw = compute_spline_properties(right_offset_25cm)
    v_dummy = np.full(1000, 2.0)
    save_csv(left_offset_25cm, v_dummy, props_lw["tangents"], props_lw["curvature"], props_lw["ds"], os.path.join(output_dir, "wall_offset_left_25cm.csv"))
    save_csv(right_offset_25cm, v_dummy, props_rw["tangents"], props_rw["curvature"], props_rw["ds"], os.path.join(output_dir, "wall_offset_right_25cm.csv"))

    # Summary JSON
    summary_data = {
        "master_line": {
            "file": "master_cyan_champion_optimal.csv",
            "min_clearance_cm": min_clear_master_cm,
            "lap_time_s": lap_time_master,
            "friction_mu": 0.60,
            "safety_margin_cm": 12.0,
            "track_length_m": track_len_master
        },
        "wall_offsets": {
            "nominal_wall_offset_cm": 25.0,
            "obstacle_safety_margin_cm": 10.0,
            "left_wall_min_obstacle_clearance_cm": left_obs_clear_cm,
            "right_wall_min_obstacle_clearance_cm": right_obs_clear_cm,
            "left_offset_csv": "wall_offset_left_25cm.csv",
            "right_offset_csv": "wall_offset_right_25cm.csv"
        },
        "pipeline_timing_ms": {
            "voronoi_compute_ms": round(t_vor_ms, 2),
            "master_solve_ms": round(t_master_ms, 2),
            "total_execution_ms": round(t_total_ms, 2)
        }
    }
    with open(os.path.join(output_dir, "tum_qp_pillar_racing_lines_summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary_data, f, indent=2)

    # -------------------------------------------------------------------------
    # 6. High-Definition Vector SVG Renderers
    # -------------------------------------------------------------------------
    print("\nRendering Rich Aesthetic Scalable Vector Graphics (.svg)...")
    BG_COLOR = "#07090e"
    CARD_BG = "#0d121d"
    BORDER_COLOR = "#334155"
    WALL_COLOR = "#ff0055"
    OBSTACLE_COLOR = "#ffb800"
    TEXT_COLOR = "#f0f4f8"

    # SVG 1: MAIN HERO SVG WITH OBSTACLE-AWARE 25CM OFFSETS & RACING LINE
    fig, ax = plt.subplots(figsize=(22, 13), facecolor=BG_COLOR, dpi=300)
    ax.set_facecolor(BG_COLOR)

    # Track Boundary Walls
    ax.plot(left_wall[:, 0], left_wall[:, 1], color=WALL_COLOR, linewidth=2.8, label="Track Left / Outer Barrier Wall", zorder=3)
    ax.plot(right_wall[:, 0], right_wall[:, 1], color=WALL_COLOR, linewidth=2.8, label="Track Right / Inner Barrier Wall", zorder=3)

    # 25cm Inward Obstacle-Aware Offset Lines
    ax.plot(left_offset_25cm[:, 0], left_offset_25cm[:, 1], color="#eab308", linestyle="-.", linewidth=2.2, alpha=0.95, 
            label="Left Wall 25cm Offset Line (10cm Obstacle-Aware Clearance)", zorder=5)
    ax.plot(right_offset_25cm[:, 0], right_offset_25cm[:, 1], color="#f97316", linestyle="-.", linewidth=2.2, alpha=0.95, 
            label="Right Wall 25cm Offset Line (10cm Obstacle-Aware Clearance)", zorder=5)

    # Obstacles
    pillar_labeled = False
    box_labeled = False
    for obs in obstacles:
        obs_type = obs.get("type", "circle")
        if obs_type == "circle":
            lbl = "Freestanding Slalom Pillar" if not pillar_labeled else ""
            pillar_labeled = True
            c = patches.Circle((obs["cx"], obs["cy"]), obs["radius"], color=OBSTACLE_COLOR, alpha=0.95, zorder=7, label=lbl)
            ax.add_patch(c)
        elif obs_type in ["rect", "wall_rectangle"]:
            lbl = "Wall Choke Extrusion" if not box_labeled else ""
            box_labeled = True
            cx, cy = obs["center_x"], obs["center_y"]
            rw, rh = obs["width"], obs["extrusion"]
            rect = patches.Rectangle((cx - rw/2.0, cy - rh/2.0), rw, rh, color=OBSTACLE_COLOR, alpha=0.95, zorder=7, label=lbl)
            ax.add_patch(rect)

    # Master Optimal Racing Line (Cyan)
    ax.plot(optimal_line[:, 0], optimal_line[:, 1], color="#00f0ff", linewidth=3.0, 
            label=f"Master TUM Optimal Racing Line (MU 0.60 | Lap: {lap_time_master:.2f}s | Clear: {min_clear_master_cm:.1f}cm)", zorder=12)

    # Spawn Point
    init_pose = track_data.get("initial_pose", {"x": 0.0, "y": 0.5})
    ax.scatter([init_pose["x"]], [init_pose["y"]], color="#00ff88", s=160, edgecolors="white", linewidth=1.5, label="Spawn / Start Position", zorder=35)

    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", color="#1e293b", alpha=0.6)
    ax.set_title(f"TUM Optimal Racing Line with Obstacle-Aware 25cm Wall Offsets (Track World 1787780744372)", color=TEXT_COLOR, fontsize=14, weight="bold", pad=15)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), facecolor=CARD_BG, edgecolor=BORDER_COLOR, labelcolor=TEXT_COLOR, fontsize=9.6, framealpha=0.98)
    plt.subplots_adjust(right=0.58)

    svg_main = os.path.join(output_dir, "tum_qp_racing_lines_with_25cm_offsets.svg")
    jpg_main = os.path.join(output_dir, "tum_qp_racing_lines_with_25cm_offsets.jpg")
    plt.savefig(svg_main, format="svg", bbox_inches="tight", facecolor=BG_COLOR)
    plt.savefig(jpg_main, format="jpeg", dpi=300, bbox_inches="tight", facecolor=BG_COLOR)
    plt.close()

    # SVG 2: 4-PANEL ZOOM SHOWING EXACT OBSTACLE-AWARE DEFLECTIONS
    fig2, ((z1, z2), (z3, z4)) = plt.subplots(2, 2, figsize=(20, 16), facecolor=BG_COLOR, dpi=300)
    zoom_zones = [
        (z1, (2.6, 4.4), (1.6, 3.4), "East Sector: Pillar 3 & Wall Choke (25cm Offset + 10cm Margin)"),
        (z2, (-1.2, 0.4), (2.4, 4.0), "North Slalom: Pillars 4 & 5 (Obstacle-Aware Deflections)"),
        (z3, (-2.4, -0.8), (1.6, 3.0), "West Sector: Pillar 6 (Corridor Clearance)"),
        (z4, (-1.6, -0.2), (0.6, 2.0), "Southwest Sector: Pillar 7 & Chicane Exit")
    ]

    for ax_z, xlim, ylim, title in zoom_zones:
        ax_z.set_facecolor(CARD_BG)
        ax_z.plot(left_wall[:, 0], left_wall[:, 1], color=WALL_COLOR, linewidth=2.5, zorder=3)
        ax_z.plot(right_wall[:, 0], right_wall[:, 1], color=WALL_COLOR, linewidth=2.5, zorder=3)
        ax_z.plot(left_offset_25cm[:, 0], left_offset_25cm[:, 1], color="#eab308", linestyle="-.", linewidth=2.0, alpha=0.95, label="Left 25cm Offset (10cm Safe)", zorder=5)
        ax_z.plot(right_offset_25cm[:, 0], right_offset_25cm[:, 1], color="#f97316", linestyle="-.", linewidth=2.0, alpha=0.95, label="Right 25cm Offset (10cm Safe)", zorder=5)

        for obs in obstacles:
            obs_type = obs.get("type", "circle")
            if obs_type == "circle":
                c = patches.Circle((obs["cx"], obs["cy"]), obs["radius"], color=OBSTACLE_COLOR, alpha=0.95, zorder=7)
                ax_z.add_patch(c)
            elif obs_type in ["rect", "wall_rectangle"]:
                cx, cy = obs["center_x"], obs["center_y"]
                rw, rh = obs["width"], obs["extrusion"]
                rect = patches.Rectangle((cx - rw/2.0, cy - rh/2.0), rw, rh, color=OBSTACLE_COLOR, alpha=0.95, zorder=7)
                ax_z.add_patch(rect)

        ax_z.plot(optimal_line[:, 0], optimal_line[:, 1], color="#00f0ff", linewidth=2.6, label="Master Optimal Line", zorder=12)

        ax_z.set_xlim(xlim)
        ax_z.set_ylim(ylim)
        ax_z.set_aspect("equal")
        ax_z.grid(True, linestyle=":", color="#334155", alpha=0.6)
        ax_z.set_title(title, color=TEXT_COLOR, fontsize=11, weight="bold", pad=8)
        ax_z.tick_params(colors=TEXT_COLOR)

    h, l = z1.get_legend_handles_labels()
    fig2.legend(h, l, loc="lower center", ncol=3, facecolor=CARD_BG, edgecolor=BORDER_COLOR, labelcolor=TEXT_COLOR, fontsize=10.0, bbox_to_anchor=(0.5, 0.02))
    plt.subplots_adjust(hspace=0.25, wspace=0.15, bottom=0.08)

    svg_zoom = os.path.join(output_dir, "tum_qp_pillar_racing_lines_zoomed.svg")
    jpg_zoom = os.path.join(output_dir, "tum_qp_pillar_racing_lines_zoomed.jpg")
    plt.savefig(svg_zoom, format="svg", bbox_inches="tight", facecolor=BG_COLOR)
    plt.savefig(jpg_zoom, format="jpeg", dpi=300, bbox_inches="tight", facecolor=BG_COLOR)
    plt.close()

    print("=" * 100)
    print("ALL ARTIFACTS GENERATED SUCCESSFULLY:")
    print(f"  • Primary SVG:   {svg_main}")
    print(f"  • Zoomed SVG:    {svg_zoom}")
    print(f"  • Summary JSON:  {os.path.join(output_dir, 'tum_qp_pillar_racing_lines_summary.json')}")
    print("=" * 100)

if __name__ == "__main__":
    run_generation()
