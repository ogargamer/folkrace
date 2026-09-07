#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - 10 TRACKS: 3-WAY DIRECT OVERLAY SUITE (BASELINE vs FILTERED vs SLOPE-LIMITED)
================================================================================
Generates 10 CAD Scalable Vector Graphics (.svg) directly comparing:
  - Grey Dotted: Medial Axis (Centerline)
  - Pink Line:   Baseline TUM (Raw Bounds with Fan Cliffs)
  - Cyan Line:   Filtered TUM (Gaussian Smoothed Bounds)
  - Neon Green:  Slope-Limited TUM (2-Pass Slew Limiter, No Dips)
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
    get_track_10_switchback_obstacle_maze
)
from webots/optimization/racing_engine.centerline_finder import find_centerline_voronoi
from webots/optimization/racing_engine.racing_line_optimizer import compute_spline_properties, TUMMinimumCurvatureOptimizer
from webots/optimization/racing_engine.solution_comparator import (
    compute_track_bounds,
    build_finite_difference_matrices
)
from webots/optimization/racing_engine.visualizer import (
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

def solve_filtered_tum(track_data, voronoi_centerline_data):
    centerline = voronoi_centerline_data["centerline"]
    N = len(centerline)
    props = compute_spline_properties(centerline)
    normals, kc, ds = props["normals"], props["curvature"], props["ds"]
    
    min_b, max_b, _ = compute_track_bounds(centerline, normals, track_data)
    min_b_smooth = gaussian_filter1d(min_b, sigma=4.0, mode='wrap')
    max_b_smooth = gaussian_filter1d(max_b, sigma=4.0, mode='wrap')
    min_bounds = np.maximum(min_b_smooth, min_b)
    max_bounds = np.minimum(max_b_smooth, max_b)
    bounds = list(zip(min_bounds, max_bounds))
    
    M1, M2, M3 = build_finite_difference_matrices(ds)
    W_ds = np.diag(ds)
    kc_macro = gaussian_filter1d(kc, sigma=4.0, mode='wrap')
    
    H = 2.0 * (M2.T @ W_ds @ M2) + 0.05 * (M1.T @ M1) + 0.40 * (M3.T @ M3) + 1e-5 * np.eye(N)
    f = 2.0 * (M2.T @ W_ds @ kc_macro)
    
    cur_alpha = np.zeros(N)
    for s_idx, w_step in enumerate(np.linspace(0.10, 1.0, 10)):
        f_step = f * w_step
        def qp_obj(a): return 0.5 * a @ H @ a + f_step @ a, H @ a + f_step
        res = opt.minimize(qp_obj, cur_alpha, method='L-BFGS-B', jac=True, bounds=bounds, options={'maxiter': 600, 'ftol': 1e-9})
        cur_alpha = res.x
        
    pts_stage = centerline + normals * cur_alpha[:, None]
    tck, u = splprep([pts_stage[:, 0], pts_stage[:, 1]], s=0.015, per=True)
    final_pts = np.column_stack(splev(np.linspace(0, 1, N, endpoint=False), tck))
    return final_pts

def plot_3way_overlay(track, pts_center, pts_base, pts_filt, pts_slope, filename, track_title):
    fig, ax = plt.subplots(figsize=(16, 10), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    # 1. Medial Axis Centerline (Grey Dotted)
    c_wrap = np.vstack([pts_center, pts_center[:1]])
    ax.plot(c_wrap[:, 0], c_wrap[:, 1], color="#64748b", linewidth=1.6, linestyle=":", label="Medial Axis (Centerline)")

    # 2. Baseline TUM (Pink Line)
    base_wrap = np.vstack([pts_base, pts_base[:1]])
    ax.plot(base_wrap[:, 0], base_wrap[:, 1], color="#ff0055", linewidth=2.6, linestyle="--", label="Baseline TUM (Raw Bounds with Fan Cliffs)")

    # 3. Filtered TUM (Cyan Line)
    filt_wrap = np.vstack([pts_filt, pts_filt[:1]])
    ax.plot(filt_wrap[:, 0], filt_wrap[:, 1], color="#00b4d8", linewidth=2.6, linestyle="-.", label="Filtered TUM (Gaussian Bounds)")

    # 4. Slope-Limited TUM (Neon Green Line)
    opt_wrap = np.vstack([pts_slope, pts_slope[:1]])
    ax.plot(opt_wrap[:, 0], opt_wrap[:, 1], color="#00ff88", linewidth=3.0, linestyle="-", label="Slope-Limited TUM (2-Pass Slew Limiter, No Dips)")

    # Start / Finish point
    start_pt = pts_slope[0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#00f0ff", s=120, zorder=7, label="Start / Finish")

    ax.set_title(f"{track_title}: Baseline vs Filtered vs Slope-Limited Direct Overlay", color=TEXT_COLOR, fontsize=13, fontweight='bold', pad=15)
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
    print("  DA WINNA - 10 TRACKS 3-WAY DIRECT OVERLAY SUITE")
    print("===========================================================================")

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

    opt_baseline = TUMMinimumCurvatureOptimizer(max_boundary_slope=0.0) # Raw bounds
    opt_slopelim = TUMMinimumCurvatureOptimizer(max_boundary_slope=0.25) # 2-Pass Slew Limiter

    t_suite_start = time.perf_counter()

    for idx, getter, title in track_getters:
        print(f"\n[{idx:02d}/10] Processing {title} (3-Way Comparison)...")
        track = getter()
        vor = find_centerline_voronoi(track)
        centerline = vor["centerline"]

        # 1. Baseline TUM
        res_base = opt_baseline.optimize_racing_line(track, vor)
        pts_base = res_base["racing_line"]

        # 2. Filtered TUM
        pts_filt = solve_filtered_tum(track, vor)

        # 3. Slope-Limited TUM
        res_slope = opt_slopelim.optimize_racing_line(track, vor)
        pts_slope = res_slope["racing_line"]

        # Generate 3-Way Direct Overlay SVG
        filename = f"{idx:02d}_track_{idx}_3way_overlay.svg"
        svg_path = plot_3way_overlay(track, centerline, pts_base, pts_filt, pts_slope, filename, title)
        print(f"    [OK] Generated 3-Way Overlay SVG: {os.path.basename(svg_path)}")

    # Copy to artifacts
    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    for f in os.listdir(OUTPUT_DIR):
        if f.endswith('.svg'):
            shutil.copy2(os.path.join(OUTPUT_DIR, f), os.path.join(art_dir, f))

    t_total = time.perf_counter() - t_suite_start
    print("\n" + "=" * 75)
    print(f"  ALL 10 3-WAY OVERLAYS COMPLETED IN {t_total:.2f} s!")
    print("=" * 75)

if __name__ == "__main__":
    main()
