#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - 10 TRACKS: SIGMA 1 (TIGHT) vs SIGMA 8 (ULTRA SMOOTH)
================================================================================
Runs Solution 1 (Spatial Bound Smoothing) across all 10 verified tracks:
  - Sigma 1.0 (Tight / Raw Track Bounds)
  - Sigma 8.0 (Ultra Smooth Aerodynamic Funnel Bounds)
  - Direct Overlay Comparison for Every Track (Sigma 1 vs Sigma 8)
================================================================================
"""

import os
import sys
import time
import shutil
import numpy as np
import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import scipy.optimize as opt
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import splprep, splev

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE_DIR)

from racing_engine.track_generator import (
    get_track_1_chicane_with_pillar,
    get_track_2_bottleneck_circuit,
    get_track_3_slalom_pillars,
    get_track_4_switchback_arena,
    get_track_5_dual_choke_chicane_ring,
    get_track_6_quad_pillar_loop,
    get_track_7_multi_bottleneck_slalom,
    get_track_8_dual_chicane_arena,
    get_track_9_super_speedway,
    get_track_10_switchback_obstacle_maze
)
from racing_engine.centerline_finder import find_centerline_voronoi
from racing_engine.racing_line_optimizer import compute_spline_properties
from racing_engine.solution_comparator import (
    compute_track_bounds,
    build_finite_difference_matrices
)
from racing_engine.visualizer import (
    draw_track_walls_and_obstacles,
    OUTPUT_DIR,
    BG_COLOR,
    TEXT_COLOR,
    PAN_ZOOM_SCRIPT
)

def post_process_cad_svg(svg_path):
    """Enforces CAD non-scaling stroke and embeds wheel-zoom pan script"""
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
                    parts.append('stroke-width: 1.5px')
                    elem.set('style', ';'.join(parts))
                else:
                    elem.set('stroke-width', '1.5px')
        tree.write(svg_path, xml_declaration=True, encoding='utf-8')
        
        with open(svg_path, 'r', encoding='utf-8') as f:
            svg_content = f.read()
        css_rule = "<style>*{vector-effect: non-scaling-stroke !important; } path, line, polyline { stroke-width: 1.5px !important; }</style>"
        if "</svg>" in svg_content:
            svg_content = svg_content.replace("</svg>", f"{css_rule}\n{PAN_ZOOM_SCRIPT}\n</svg>")
        with open(svg_path, 'w', encoding='utf-8') as f:
            f.write(svg_content)
    except Exception as e:
        print(f"Warning: SVG XML post-processing failed: {e}")

def solve_solution_1_sigma(track_data, voronoi_centerline_data, sigma_val, label):
    centerline = voronoi_centerline_data["centerline"]
    N = len(centerline)
    props = compute_spline_properties(centerline)
    normals, kc, ds = props["normals"], props["curvature"], props["ds"]
    
    min_b, max_b, _ = compute_track_bounds(centerline, normals, track_data)
    
    # Filter bounds by specified sigma
    min_b_smooth = gaussian_filter1d(min_b, sigma=sigma_val, mode='wrap')
    max_b_smooth = gaussian_filter1d(max_b, sigma=sigma_val, mode='wrap')
    
    # Conservative bounds (never breach physical walls)
    min_bounds = np.maximum(min_b_smooth, min_b)
    max_bounds = np.minimum(max_b_smooth, max_b)
    bounds = list(zip(min_bounds, max_bounds))
    
    M1, M2, M3 = build_finite_difference_matrices(ds)
    W_ds = np.diag(ds)
    kc_macro = gaussian_filter1d(kc, sigma=4.0, mode='wrap')
    
    H = 2.0 * (M2.T @ W_ds @ M2) + 0.05 * (M1.T @ M1) + 0.40 * (M3.T @ M3) + 1e-5 * np.eye(N)
    f = 2.0 * (M2.T @ W_ds @ kc_macro)
    
    iteration_history = [{"iter": 0, "stage": 0, "alpha": np.zeros(N), "points": centerline.copy(), "description": "Iteration 0: Medial Axis"}]
    cur_alpha = np.zeros(N)
    cumulative_iters = 0
    num_stages = 10
    
    for s_idx, w_step in enumerate(np.linspace(0.10, 1.0, num_stages)):
        f_step = f * w_step
        def qp_obj(a):
            return 0.5 * a @ H @ a + f_step @ a, H @ a + f_step
        res = opt.minimize(qp_obj, cur_alpha, method='L-BFGS-B', jac=True, bounds=bounds, options={'maxiter': 600, 'ftol': 1e-9})
        cur_alpha = res.x
        cumulative_iters += res.nit
        pts_stage = centerline + normals * cur_alpha[:, None]
        pct = int((s_idx + 1) / num_stages * 100)
        iteration_history.append({
            "iter": cumulative_iters, "stage": s_idx + 1, "points": pts_stage.copy(),
            "description": f"Stage {s_idx+1}/10 ({pct}%): {label} (Total {cumulative_iters} iters)"
        })
        
    tck, u = splprep([pts_stage[:, 0], pts_stage[:, 1]], s=0.015, per=True)
    final_pts = np.column_stack(splev(np.linspace(0, 1, N, endpoint=False), tck))
    final_props = compute_spline_properties(final_pts)
    
    return {
        "label": label,
        "sigma": sigma_val,
        "racing_line": final_pts,
        "iteration_history": iteration_history,
        "curvature": final_props["curvature"]
    }

def plot_track_iterations(track, sol_result, filename, title):
    fig, ax = plt.subplots(figsize=(16, 10), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    colors = [
        "#4a5568", "#3a86ff", "#00b4d8", "#06d6a0", "#7209b7",
        "#b5179e", "#f72585", "#ff0054", "#ff5400", "#ffb800", "#00ff88"
    ]
    history = sol_result["iteration_history"]
    for idx, item in enumerate(history):
        c = colors[idx % len(colors)]
        pts = item["points"]
        pts_c = np.vstack([pts, pts[:1]])
        is_final = (idx == len(history) - 1)
        is_initial = (idx == 0)
        
        if is_initial:
            lw = 1.8
            ls = ":"
            lbl = "Initial: Medial Axis (Centerline)"
        elif is_final:
            lw = 2.8
            ls = "-"
            lbl = f"Final Optimal Line ({sol_result['label']})"
        else:
            lw = 1.2
            ls = "--"
            lbl = item["description"]
            
        ax.plot(pts_c[:, 0], pts_c[:, 1], color=c, linewidth=lw, linestyle=ls, label=lbl)

    start_pt = history[-1]["points"][0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#00f0ff", s=120, zorder=7, label="Start / Finish")
    ax.set_title(f"{title} [CAD Pan & Wheel Zoom Active]", color=TEXT_COLOR, fontsize=13, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.2, color="#fff")
    ax.tick_params(colors=TEXT_COLOR)
    ax.legend(bbox_to_anchor=(1.02, 1.0), loc="upper left", facecolor="#121826", edgecolor="#2d3748", labelcolor=TEXT_COLOR, framealpha=1.0, fontsize=9, borderpad=1.0, labelspacing=0.8)

    svg_path = os.path.join(OUTPUT_DIR, filename)
    plt.savefig(svg_path, format="svg", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    post_process_cad_svg(svg_path)
    return svg_path

def plot_track_overlay(track, res_s1, res_s8, filename, track_title):
    fig, ax = plt.subplots(figsize=(16, 10), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    # Initial Centerline
    init_pts = res_s1["iteration_history"][0]["points"]
    init_c = np.vstack([init_pts, init_pts[:1]])
    ax.plot(init_c[:, 0], init_c[:, 1], color="#64748b", linewidth=1.5, linestyle=":", label="Medial Axis (Centerline)")

    # Sigma 1 Line (Cyan)
    pts1 = res_s1["racing_line"]
    pts1_c = np.vstack([pts1, pts1[:1]])
    ax.plot(pts1_c[:, 0], pts1_c[:, 1], color="#00b4d8", linewidth=2.8, label="Sigma = 1.0 (Tight / Raw Bounds)")

    # Sigma 8 Line (Neon Green)
    pts8 = res_s8["racing_line"]
    pts8_c = np.vstack([pts8, pts8[:1]])
    ax.plot(pts8_c[:, 0], pts8_c[:, 1], color="#00ff88", linewidth=2.8, label="Sigma = 8.0 (Ultra Smooth Funnel)")

    start_pt = pts1[0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#00f0ff", s=120, zorder=7, label="Start / Finish")
    ax.set_title(f"{track_title}: Sigma 1.0 vs Sigma 8.0 Direct Overlay", color=TEXT_COLOR, fontsize=13, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.2, color="#fff")
    ax.tick_params(colors=TEXT_COLOR)
    ax.legend(bbox_to_anchor=(1.02, 1.0), loc="upper left", facecolor="#121826", edgecolor="#2d3748", labelcolor=TEXT_COLOR, framealpha=1.0, fontsize=9.5, borderpad=1.0, labelspacing=0.9)

    svg_path = os.path.join(OUTPUT_DIR, filename)
    plt.savefig(svg_path, format="svg", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    post_process_cad_svg(svg_path)
    return svg_path

def main():
    print("=" * 75)
    print("  DA WINNA - 10 TRACKS: SIGMA 1.0 vs SIGMA 8.0 SUITE")
    print("=" * 75)

    track_getters = [
        (1, get_track_1_chicane_with_pillar, "Track 1: High-Speed Chicane Arena"),
        (2, get_track_2_bottleneck_circuit, "Track 2: Double Hairpin & Slalom"),
        (3, get_track_3_slalom_pillars, "Track 3: Grand Prix Arena"),
        (4, get_track_4_switchback_arena, "Track 4: Switchback Arena [KEPT]"),
        (5, get_track_5_dual_choke_chicane_ring, "Track 5: Quad-Choke Bottleneck Ring"),
        (6, get_track_6_quad_pillar_loop, "Track 6: Multi-Apex High-Speed Circuit"),
        (7, get_track_7_multi_bottleneck_slalom, "Track 7: Slalom Speedway"),
        (8, get_track_8_dual_chicane_arena, "Track 8: Dual Chicane Speedway"),
        (9, get_track_9_super_speedway, "Track 9: Double Switchback Arena"),
        (10, get_track_10_switchback_obstacle_maze, "Track 10: Obstacle Maze")
    ]

    t_suite_start = time.perf_counter()

    for idx, getter, title in track_getters:
        print(f"\n[{idx}/10] Processing {title}...")
        track = getter()
        vor = find_centerline_voronoi(track)

        # 1. Run Sigma 1.0
        t0 = time.perf_counter()
        res_s1 = solve_solution_1_sigma(track, vor, 1.0, "Sigma = 1.0 (Tight)")
        t_s1 = (time.perf_counter() - t0) * 1000.0
        svg_s1 = plot_track_iterations(track, res_s1, f"{idx:02d}_track_{idx}_sigma1.svg", f"{title} (Sigma 1.0)")
        print(f"    [OK] Sigma 1.0 computed in {t_s1:.1f} ms -> {os.path.basename(svg_s1)}")

        # 2. Run Sigma 8.0
        t0 = time.perf_counter()
        res_s8 = solve_solution_1_sigma(track, vor, 8.0, "Sigma = 8.0 (Ultra Smooth)")
        t_s8 = (time.perf_counter() - t0) * 1000.0
        svg_s8 = plot_track_iterations(track, res_s8, f"{idx:02d}_track_{idx}_sigma8.svg", f"{title} (Sigma 8.0)")
        print(f"    [OK] Sigma 8.0 computed in {t_s8:.1f} ms -> {os.path.basename(svg_s8)}")

        # 3. Direct Overlay (Sigma 1 vs Sigma 8)
        svg_over = plot_track_overlay(track, res_s1, res_s8, f"{idx:02d}_track_{idx}_overlay_sigma1_vs_sigma8.svg", title)
        print(f"    [OK] Overlay saved -> {os.path.basename(svg_over)}")

    # Sync to artifacts
    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    for f in os.listdir(OUTPUT_DIR):
        if f.endswith('.svg'):
            shutil.copy2(os.path.join(OUTPUT_DIR, f), os.path.join(art_dir, f))

    t_total = time.perf_counter() - t_suite_start
    print("\n" + "=" * 75)
    print(f"  ALL 10 TRACKS (SIGMA 1 vs SIGMA 8) COMPLETED IN {t_total:.2f} s!")
    print("=" * 75)

if __name__ == "__main__":
    main()
