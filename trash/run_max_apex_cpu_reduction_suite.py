#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - 10 TRACKS: MAX APEX CPU REDUCTION SUITE (TRACKING & OVERLAYS)
================================================================================
Compares Original Max Apex with 3 CPU-optimized variants across all 10 tracks:
  1. Pink:         Original Max Apex Baseline (10 Stages, 800 Iters, N=360) ~2800ms
  2. Orange:       Fast Homotopy (8 Stages, 600 Iters, N=360)              ~1800ms (1.5x)
  3. Yellow:       Grid-Downsampled (8 Stages, 600 Iters, N=180)            ~1100ms (2.5x)
  4. Neon Green:   Ultra-Fast Embedded (5 Stages, 400 Iters, N=150)         ~650ms  (4.3x)
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
from racing_engine.racing_line_optimizer import TUMMinimumCurvatureOptimizer
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

def solve_grid_downsampled(track_data, vor_data, N_grid=180, stages=8, max_sub=600):
    c_raw = vor_data['centerline']
    tck, u = splprep([c_raw[:, 0], c_raw[:, 1]], s=0.005, per=True)
    c_sub = np.column_stack(splev(np.linspace(0, 1, N_grid, endpoint=False), tck))
    vor_sub = {'centerline': c_sub}
    opt_sub = TUMMinimumCurvatureOptimizer(
        weight_curvature=3.50,
        weight_path_smoothness=0.03,
        weight_jerk=0.35,
        robot_safety_margin=0.10,
        num_progression_stages=stages,
        max_solver_sub_iters=max_sub,
        max_boundary_slope=0.0
    )
    res_sub = opt_sub.optimize_racing_line(track_data, vor_sub)
    pts_sub = res_sub['racing_line']
    tck_f, u_f = splprep([pts_sub[:, 0], pts_sub[:, 1]], s=0.015, per=True)
    final_pts = np.column_stack(splev(np.linspace(0, 1, len(c_raw), endpoint=False), tck_f))
    return final_pts, res_sub

def plot_cpu_variants_overlay(track, centerline, results, filename, track_title):
    fig, ax = plt.subplots(figsize=(16, 10), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    # 1. Medial Axis Centerline (Grey Dotted)
    c_wrap = np.vstack([centerline, centerline[:1]])
    ax.plot(c_wrap[:, 0], c_wrap[:, 1], color="#64748b", linewidth=1.5, linestyle=":", label="Medial Axis (Centerline)")

    # 2. Plot all 4 Variants
    for item in results:
        cfg = item["config"]
        perf = item["perf"]
        pts = item["points"]
        pts_wrap = np.vstack([pts, pts[:1]])
        lbl = f"{cfg['name']} [{perf['solve_ms']:.0f}ms ({perf['speedup']:.1f}x) | Error: {perf['err_cm']:.1f}cm]"
        ax.plot(pts_wrap[:, 0], pts_wrap[:, 1], color=cfg["color"], linewidth=cfg["lw"], linestyle=cfg["linestyle"], label=lbl)

    # Start / Finish point
    start_pt = results[0]["points"][0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#00f0ff", s=120, zorder=7, label="Start / Finish")

    num_obs = len(track.get("obstacles", []))
    ax.set_title(f"{track_title} ({num_obs} Obstacles): Max Apex CPU Reduction Benchmark", color=TEXT_COLOR, fontsize=13, fontweight='bold', pad=15)
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
    print("=" * 85)
    print("   DA WINNA - 10 TRACKS: MAX APEX CPU REDUCTION SUITE (OVERLAYS)")
    print("=" * 85)

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
        print(f"\n[{idx:02d}/10] Processing {title}...")
        track = getter()
        vor = find_centerline_voronoi(track)
        centerline = vor["centerline"]

        # 1. Baseline: Original Max Apex (10 Stages, 800 Iters)
        opt_orig = TUMMinimumCurvatureOptimizer(
            weight_curvature=3.50, weight_path_smoothness=0.03, weight_jerk=0.35,
            robot_safety_margin=0.10, num_progression_stages=10, max_solver_sub_iters=800, max_boundary_slope=0.0
        )
        t0 = time.perf_counter()
        res_orig = opt_orig.optimize_racing_line(track, vor)
        t_orig = (time.perf_counter() - t0) * 1000.0
        pts_orig = res_orig["racing_line"]

        # 2. Variant 1: Fast Homotopy (8 Stages, 600 Iters)
        opt_v1 = TUMMinimumCurvatureOptimizer(
            weight_curvature=3.50, weight_path_smoothness=0.03, weight_jerk=0.35,
            robot_safety_margin=0.10, num_progression_stages=8, max_solver_sub_iters=600, max_boundary_slope=0.0
        )
        t0 = time.perf_counter()
        res_v1 = opt_v1.optimize_racing_line(track, vor)
        t_v1 = (time.perf_counter() - t0) * 1000.0
        pts_v1 = res_v1["racing_line"]
        err_v1 = np.max(np.hypot(pts_orig[:, 0] - pts_v1[:, 0], pts_orig[:, 1] - pts_v1[:, 1])) * 100

        # 3. Variant 2: Grid-Downsampled (N=180, 8 Stages)
        t0 = time.perf_counter()
        pts_v2, res_v2 = solve_grid_downsampled(track, vor, N_grid=180, stages=8, max_sub=600)
        t_v2 = (time.perf_counter() - t0) * 1000.0
        err_v2 = np.max(np.hypot(pts_orig[:, 0] - pts_v2[:, 0], pts_orig[:, 1] - pts_v2[:, 1])) * 100

        # 4. Variant 3: Ultra-Fast Embedded (N=150, 5 Stages)
        t0 = time.perf_counter()
        pts_v3, res_v3 = solve_grid_downsampled(track, vor, N_grid=150, stages=5, max_sub=400)
        t_v3 = (time.perf_counter() - t0) * 1000.0
        err_v3 = np.max(np.hypot(pts_orig[:, 0] - pts_v3[:, 0], pts_orig[:, 1] - pts_v3[:, 1])) * 100

        track_results = [
            {
                "config": {"name": "Original Max Apex (Baseline)", "color": "#ff0055", "lw": 3.0, "linestyle": "-"},
                "perf": {"solve_ms": t_orig, "speedup": 1.0, "err_cm": 0.0},
                "points": pts_orig
            },
            {
                "config": {"name": "Fast Homotopy (8 Stages)", "color": "#ff7700", "lw": 2.6, "linestyle": "-"},
                "perf": {"solve_ms": t_v1, "speedup": t_orig / t_v1, "err_cm": err_v1},
                "points": pts_v1
            },
            {
                "config": {"name": "Grid-Downsampled (N=180)", "color": "#ffd000", "lw": 2.4, "linestyle": "-"},
                "perf": {"solve_ms": t_v2, "speedup": t_orig / t_v2, "err_cm": err_v2},
                "points": pts_v2
            },
            {
                "config": {"name": "Ultra-Fast Embedded (N=150)", "color": "#00ff88", "lw": 2.6, "linestyle": "--"},
                "perf": {"solve_ms": t_v3, "speedup": t_orig / t_v3, "err_cm": err_v3},
                "points": pts_v3
            }
        ]

        filename = f"{idx:02d}_track_{idx}_max_apex_cpu_reduction_overlay.svg"
        svg_path = plot_cpu_variants_overlay(track, centerline, track_results, filename, title)
        print(f"    [OK] Generated Overlay SVG: {os.path.basename(svg_path)} (Baseline: {t_orig:.0f}ms -> Ultra: {t_v3:.0f}ms [{t_orig/t_v3:.1f}x])")

    # Copy to artifacts
    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    for f in os.listdir(OUTPUT_DIR):
        if f.endswith('.svg'):
            shutil.copy2(os.path.join(OUTPUT_DIR, f), os.path.join(art_dir, f))

    t_total = time.perf_counter() - t_suite_start
    print("\n" + "=" * 85)
    print(f"  ALL 10 OVERLAYS COMPLETED IN {t_total:.2f} s!")
    print("=" * 85)

if __name__ == "__main__":
    main()
