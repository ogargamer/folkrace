#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - YELLOW LINE (CORNER-PHASE SIGMOID LATE-APEX) PRODUCTION SUITE
================================================================================
Runs the single Yellow Line across all 11 tracks, generates high-res vector CAD
SVGs, exports complete waypoint trajectories (CSV/JSON/NPY), and backs everything
up into a dedicated folder alongside snapshots:
  -> racing_results/yellow_line_backup/
================================================================================
"""

import os
import sys
import shutil
import json
import numpy as np
import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import scipy.spatial as spatial
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import splprep, splev
import scipy.optimize as opt

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
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
from webots/optimization/racing_engine.centerline_finder import find_centerline_voronoi, get_all_boundary_and_obstacle_points
from webots/optimization/racing_engine.racing_line_optimizer import (
    compute_spline_properties,
    TUMMinimumCurvatureOptimizer
)
from webots/optimization/racing_engine.blindspot_simulator import (
    compute_corridor_bounds,
    simulate_135deg_tof_frustum
)
from webots/optimization/racing_engine.visualizer import (
    draw_track_walls_and_obstacles,
    OUTPUT_DIR,
    BG_COLOR,
    TEXT_COLOR,
    PAN_ZOOM_SCRIPT
)

BACKUP_DIR = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup")
os.makedirs(BACKUP_DIR, exist_ok=True)
os.makedirs(OUTPUT_DIR, exist_ok=True)

def post_process_cad_svg(svg_path):
    try:
        ET.register_namespace('', 'http://www.w3.org/2000/svg')
        tree = ET.parse(svg_path)
        root = tree.getroot()
        for elem in root.iter():
            tag = elem.tag.split('}')[-1]
            if tag in ['path', 'line', 'polyline', 'polygon', 'circle', 'rect', 'ellipse']:
                elem.set('vector-effect', 'non-scaling-stroke')
                style = elem.get('style')
                if style:
                    parts = [p for p in style.split(';') if not p.strip().startswith('stroke-width')]
                    parts.append('stroke-width: 1.2px')
                    elem.set('style', ';'.join(parts))
                else:
                    elem.set('stroke-width', '1.2px')
        tree.write(svg_path, xml_declaration=True, encoding='utf-8')
        
        with open(svg_path, 'r', encoding='utf-8') as f:
            svg_content = f.read()
        css_rule = "<style>*{vector-effect: non-scaling-stroke !important; } path, line, polyline { stroke-width: 1.2px !important; }</style>"
        if "</svg>" in svg_content:
            svg_content = svg_content.replace("</svg>", f"{css_rule}\n{PAN_ZOOM_SCRIPT}\n</svg>")
        with open(svg_path, 'w', encoding='utf-8') as f:
            f.write(svg_content)
    except Exception as e:
        print(f"Warning: SVG XML post-processing failed: {e}")

def solve_yellow_line(centerline, min_bounds, max_bounds, barrier_tree,
                      entry_agg=0.55, exit_agg=1.50):
    N = len(centerline)
    props = compute_spline_properties(centerline)
    ds = props['ds']
    kc = props['curvature']
    normals = props['normals']
    
    kc_macro = gaussian_filter1d(kc, sigma=4.0, mode='wrap')
    curv_abs = np.abs(kc_macro)
    curv_smooth = gaussian_filter1d(curv_abs, sigma=8.0, mode='wrap')
    grad_curv = gaussian_filter1d(np.gradient(curv_smooth), sigma=6.0, mode='wrap')
    phase = np.arctan2(grad_curv * 15.0, curv_smooth + 1e-3)
    blend = 0.5 * (1.0 - np.sin(phase))
    local_agg = (1.0 - blend) * entry_agg + blend * exit_agg

    M2 = np.zeros((N, N))
    M1 = np.zeros((N, N))
    M3 = np.zeros((N, N))
    for i in range(N):
        ds_p = ds[(i - 1) % N]
        ds_c = ds[i]
        ds_avg = 0.5 * (ds_p + ds_c)
        M2[i, (i - 1) % N] = 1.0 / (ds_p * ds_avg)
        M2[i, i] = -(1.0 / ds_p + 1.0 / ds_c) / ds_avg
        M2[i, (i + 1) % N] = 1.0 / (ds_c * ds_avg)

        M1[i, i] = -1.0
        M1[i, (i + 1) % N] = 1.0

        M3[i, (i - 2) % N] = -1.0
        M3[i, (i - 1) % N] = 2.0
        M3[i, (i + 1) % N] = -2.0
        M3[i, (i + 2) % N] = 1.0

    W_ds = np.diag(ds)
    weight_curvature = 3.60
    weight_smoothness = 0.040
    weight_jerk = 0.50

    H = (
        weight_curvature * (M2.T @ W_ds @ M2)
        + weight_smoothness * (M1.T @ M1)
        + weight_jerk * (M3.T @ M3)
        + 1e-5 * np.eye(N)
    )

    f_base = weight_curvature * (M2.T @ W_ds @ (kc_macro * local_agg))

    bounds = list(zip(min_bounds, max_bounds))
    num_stages = 7
    weights_prog = np.linspace(0.10, 1.0, num_stages)
    cur_alpha = np.zeros(N)

    for step_idx, w_step in enumerate(weights_prog):
        f_step = f_base * w_step
        def qp_obj(a):
            return 0.5 * a @ H @ a + f_step @ a, H @ a + f_step

        res = opt.minimize(qp_obj, cur_alpha, method='L-BFGS-B', jac=True, bounds=bounds,
                           options={'maxiter': 300, 'ftol': 1e-9})
        cur_alpha = res.x

    raw_pts = centerline + normals * cur_alpha[:, None]
    tck, _ = splprep([raw_pts[:, 0], raw_pts[:, 1]], s=0.005, per=True)
    u_fine = np.linspace(0, 1, N, endpoint=False)
    final_pts = np.column_stack(splev(u_fine, tck))
    final_props = compute_spline_properties(final_pts)
    
    final_headings = np.arctan2(final_props["tangents"][:, 1], final_props["tangents"][:, 0])
    tof_res = simulate_135deg_tof_frustum(final_pts, final_headings, barrier_tree)

    opt_engine = TUMMinimumCurvatureOptimizer()
    vels = opt_engine._generate_velocity_profile(final_props["curvature"], final_props["ds"])
    lap_t = float(np.sum(final_props["ds"] / np.maximum(vels, 0.5)))
    min_spd = float(np.min(vels))
    top_spd = float(np.max(vels))
    min_sight = float(np.min(tof_res["forward_sight"]))

    dists, _ = barrier_tree.query(final_pts)
    min_clearance = float(np.min(dists)) * 100.0

    return {
        "racing_line": final_pts,
        "alpha": cur_alpha,
        "props": final_props,
        "tof": tof_res,
        "velocities": vels,
        "lap_time": lap_t,
        "min_apex_speed": min_spd,
        "top_speed": top_spd,
        "min_sight": min_sight,
        "min_clearance_cm": min_clearance
    }

def plot_standalone_yellow_line(track, centerline, yel_data, filename, track_title):
    fig, ax = plt.subplots(figsize=(20, 12), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    # Medial Centerline
    c_wrap = np.vstack([centerline, centerline[:1]])
    ax.plot(c_wrap[:, 0], c_wrap[:, 1], color="#334155", linewidth=1.0, linestyle=":", label="Medial Centerline (Voronoi)")

    # Yellow Line
    p_yel = np.vstack([yel_data["racing_line"], yel_data["racing_line"][:1]])
    lbl_yel = f"Yellow Line (Late-Apex Sigmoid) [Lap: {yel_data['lap_time']:.2f}s | Min Apex: {yel_data['min_apex_speed']:.2f} m/s | Top: {yel_data['top_speed']:.2f} m/s | Sight: {yel_data['min_sight']:.2f}m]"
    ax.plot(p_yel[:, 0], p_yel[:, 1], color="#ffd700", linewidth=2.4, linestyle="-", label=lbl_yel)

    # Annotate 135° ToF Blind Apex Sightlines
    worst_idx = np.argmin(yel_data["tof"]["forward_sight"])
    px, py = yel_data["racing_line"][worst_idx]
    heading = np.arctan2(yel_data["props"]["tangents"][worst_idx, 1], yel_data["props"]["tangents"][worst_idx, 0])
    z_ranges = yel_data["tof"]["zone_ranges"][worst_idx]
    z_angles = np.linspace(-np.radians(135.0 / 2.0), np.radians(135.0 / 2.0), 12)
    
    for a, r in zip(z_angles, z_ranges):
        beam_ang = heading + a
        bx = px + r * np.cos(beam_ang)
        by = py + r * np.sin(beam_ang)
        ax.plot([px, bx], [py, by], color="#ffd700", linewidth=0.9, alpha=0.45)
        
    ax.scatter([px], [py], color="#ff0055", s=90, zorder=8)
    ax.annotate(f"Min 135° Sight: {yel_data['min_sight']:.2f}m\nApex Speed: {yel_data['velocities'][worst_idx]:.2f} m/s",
                (px, py), textcoords="offset points", xytext=(12, 10),
                color="#ffffff", fontsize=8.6, fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.35", fc="#1e293b", ec="#ffd700", alpha=0.95))

    # Start / Finish Point
    start_pt = yel_data["racing_line"][0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#ffffff", s=130, zorder=8, label="Start / Finish")

    num_obs = len(track.get("obstacles", []))
    ax.set_title(f"{track_title} ({num_obs} Obstacles): Yellow Line (Corner-Phase Sigmoid Late-Apex)",
                 color=TEXT_COLOR, fontsize=14, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.2, color="#fff")
    ax.tick_params(colors=TEXT_COLOR)
    ax.legend(bbox_to_anchor=(1.02, 1.0), loc="upper left", facecolor="#121826", edgecolor="#2d3748",
              labelcolor=TEXT_COLOR, framealpha=1.0, fontsize=9.4, borderpad=1.0, labelspacing=0.8)

    # Save to snapshots
    svg_snap_path = os.path.join(OUTPUT_DIR, filename)
    plt.savefig(svg_snap_path, format="svg", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    post_process_cad_svg(svg_snap_path)

    # Copy to backup directory
    svg_backup_path = os.path.join(BACKUP_DIR, filename)
    shutil.copy2(svg_snap_path, svg_backup_path)

    return svg_snap_path

def main():
    print("=" * 125)
    print("   DA WINNA - YELLOW LINE PRODUCTION & BACKUP SUITE (11 TRACKS)")
    print("=" * 125)

    track_getters = [
        (1, get_track_1_chicane_with_pillar, "Track 1: Chicane Arena"),
        (2, get_track_2_bottleneck_circuit, "Track 2: Double Hairpin"),
        (3, get_track_3_slalom_pillars, "Track 3: Grand Prix Arena"),
        (4, get_track_4_switchback_arena, "Track 4: Technical Switchback"),
        (5, get_track_5_dual_choke_chicane_ring, "Track 5: Quad-Choke Ring"),
        (6, get_track_6_quad_pillar_loop, "Track 6: Multi-Apex Circuit"),
        (7, get_track_7_multi_bottleneck_slalom, "Track 7: Slalom Speedway"),
        (8, get_track_8_dual_chicane_arena, "Track 8: Dual Chicane Speedway"),
        (9, get_track_9_super_speedway, "Track 9: Double Switchback Arena"),
        (10, get_track_10_switchback_obstacle_maze, "Track 10: Obstacle Maze"),
        (11, get_track_11_quad_chicane_slalom, "Track 11: Quad Chicane Arena (Direct L-R-L-R)")
    ]

    benchmark_results = []

    for idx, getter, title in track_getters:
        track = getter()
        vor = find_centerline_voronoi(track)
        centerline = vor["centerline"]
        
        all_barriers = get_all_boundary_and_obstacle_points(track, point_spacing=0.02)
        barrier_tree = spatial.cKDTree(all_barriers)
        min_bounds, max_bounds = compute_corridor_bounds(centerline, barrier_tree)

        # Solve Yellow Line
        yel_data = solve_yellow_line(centerline, min_bounds, max_bounds, barrier_tree,
                                     entry_agg=0.55, exit_agg=1.50)

        # Plot & Save SVG
        filename = f"{idx:02d}_track_{idx}_yellow_line.svg"
        plot_standalone_yellow_line(track, centerline, yel_data, filename, title)

        # Export Trajectory Array (.npy and .csv) into Backup Folder
        traj_array = np.column_stack([
            yel_data["racing_line"][:, 0], # x
            yel_data["racing_line"][:, 1], # y
            yel_data["velocities"],        # v (m/s)
            np.arctan2(yel_data["props"]["tangents"][:, 1], yel_data["props"]["tangents"][:, 0]), # heading (rad)
            yel_data["props"]["curvature"], # curvature (1/m)
            yel_data["tof"]["forward_sight"] # ToF 135 sight (m)
        ])
        
        npy_path = os.path.join(BACKUP_DIR, f"{idx:02d}_track_{idx}_yellow_trajectory.npy")
        csv_path = os.path.join(BACKUP_DIR, f"{idx:02d}_track_{idx}_yellow_trajectory.csv")
        np.save(npy_path, traj_array)
        np.savetxt(csv_path, traj_array, delimiter=",", header="x_m,y_m,velocity_mps,heading_rad,curvature_inv_m,tof_sight_m", comments="")

        benchmark_results.append({
            "track_idx": idx,
            "track_name": title,
            "lap_time": yel_data["lap_time"],
            "min_apex_speed": yel_data["min_apex_speed"],
            "top_speed": yel_data["top_speed"],
            "min_sight": yel_data["min_sight"],
            "min_clearance_cm": yel_data["min_clearance_cm"]
        })

        print(f"[{idx:02d}/11] {title:38s} | Lap: {yel_data['lap_time']:.2f}s | Min Apex: {yel_data['min_apex_speed']:.2f} m/s | Top: {yel_data['top_speed']:.2f} m/s | Sight: {yel_data['min_sight']:.2f}m | Clearance: {yel_data['min_clearance_cm']:.1f}cm")

    # Save JSON summary to backup
    json_path = os.path.join(BACKUP_DIR, "yellow_line_benchmark_telemetry.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(benchmark_results, f, indent=2)

    # Copy to artifacts directory
    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    for f in os.listdir(OUTPUT_DIR):
        if "_yellow_line.svg" in f:
            shutil.copy2(os.path.join(OUTPUT_DIR, f), os.path.join(art_dir, f))

    print("\n" + "=" * 125)
    print(f"ALL 11 TRACKS YELLOW LINE RUN COMPLETE & BACKED UP TO: {BACKUP_DIR}")
    print("=" * 125)

if __name__ == "__main__":
    main()
