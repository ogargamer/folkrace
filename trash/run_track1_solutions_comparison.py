#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - TRACK 1 SOLUTION 1 vs HYBRID BENCHMARK & COMPARISON
================================================================================
Compares:
  1. Baseline TUM / F1TENTH
  2. Solution 1: Pure Spatial Bound Smoothing
  3. Hybrid: Bound Smoothing + Singularity-Clamped Scale Damping
  4. Direct Overlay Comparison
================================================================================
"""

import os
import sys
import time
import numpy as np
import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE_DIR)

from racing_engine.track_generator import get_track_1_chicane_with_pillar
from racing_engine.centerline_finder import find_centerline_voronoi
from racing_engine.racing_line_optimizer import TUMMinimumCurvatureOptimizer
from racing_engine.solution_comparator import (
    solve_solution_1_bound_smoothing,
    solve_solution_hybrid_bound_and_scale
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

def plot_solution_iterations(track, sol_result, filename, title):
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
            lbl = f"Final Optimal Line: {item['description']}"
        else:
            lw = 1.2
            ls = "--"
            lbl = item["description"]
            
        ax.plot(pts_c[:, 0], pts_c[:, 1], color=c, linewidth=lw, linestyle=ls, label=lbl)

    start_pt = history[-1]["points"][0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#00f0ff", s=120, zorder=7, label="Start / Finish Line")
    ax.set_title(f"{title} [Track 1 CAD Pan & Wheel Zoom Active]", color=TEXT_COLOR, fontsize=13, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.2, color="#fff")
    ax.tick_params(colors=TEXT_COLOR)
    ax.legend(bbox_to_anchor=(1.02, 1.0), loc="upper left", facecolor="#121826", edgecolor="#2d3748", labelcolor=TEXT_COLOR, framealpha=1.0, fontsize=9, borderpad=1.0, labelspacing=0.8)

    svg_path = os.path.join(OUTPUT_DIR, filename)
    plt.savefig(svg_path, format="svg", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    post_process_cad_svg(svg_path)
    return svg_path

def plot_overlay_comparison(track, solutions, filename):
    fig, ax = plt.subplots(figsize=(16, 10), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    sol_colors = {
        "Baseline TUM": "#ff0055",
        "Solution 1: Pure Bound Smoothing": "#00b4d8",
        "Hybrid: Bound Smoothing + Clamped Scale": "#00ff88"
    }

    for name, data in solutions.items():
        pts = data["racing_line"]
        pts_c = np.vstack([pts, pts[:1]])
        c = sol_colors.get(name, "#ffffff")
        ax.plot(pts_c[:, 0], pts_c[:, 1], color=c, linewidth=2.8, label=name)

    start_pt = list(solutions.values())[0]["racing_line"][0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#00f0ff", s=120, zorder=7, label="Start / Finish")
    ax.set_title("Track 1: Baseline vs Solution 1 vs Hybrid Overlay Comparison", color=TEXT_COLOR, fontsize=13, fontweight='bold', pad=15)
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
    print("  TRACK 1: SOLUTION 1 vs HYBRID BENCHMARK & COMPARISON")
    print("=" * 75)

    track = get_track_1_chicane_with_pillar()
    print(f"[+] Loaded Track 1: {track['name']}")
    vor = find_centerline_voronoi(track)

    solutions = {}

    # 0. Baseline TUM
    print("\n[+] Running Baseline TUM / F1TENTH Optimizer...")
    opt_base = TUMMinimumCurvatureOptimizer()
    t0 = time.perf_counter()
    res_base = opt_base.optimize_racing_line(track, vor)
    t_base = (time.perf_counter() - t0) * 1000.0
    print(f"    [OK] Baseline completed in {t_base:.1f} ms.")
    svg_base = plot_solution_iterations(track, res_base, "01_track_1_baseline_tum.svg", "Track 1: Baseline TUM / F1TENTH")
    solutions["Baseline TUM"] = res_base

    # 1. Solution 1: Pure Bound Smoothing
    print("\n[+] Running Solution 1: Pure Bound Smoothing...")
    t0 = time.perf_counter()
    res_sol1 = solve_solution_1_bound_smoothing(track, vor)
    t_sol1 = (time.perf_counter() - t0) * 1000.0
    print(f"    [OK] Solution 1 completed in {t_sol1:.1f} ms.")
    svg_sol1 = plot_solution_iterations(track, res_sol1, "01_track_1_solution_1_bound_smoothing.svg", "Track 1 - Solution 1: Pure Bound Smoothing")
    solutions["Solution 1: Pure Bound Smoothing"] = res_sol1

    # 2. Hybrid Solution: Bound Smoothing + Clamped Scale Damping
    print("\n[+] Running Hybrid Solution: Bound Smoothing + Clamped Scale Damping...")
    t0 = time.perf_counter()
    res_hybrid = solve_solution_hybrid_bound_and_scale(track, vor)
    t_hyb = (time.perf_counter() - t0) * 1000.0
    print(f"    [OK] Hybrid completed in {t_hyb:.1f} ms.")
    svg_hyb = plot_solution_iterations(track, res_hybrid, "01_track_1_hybrid_bound_and_scale.svg", "Track 1 - Hybrid: Bound Smoothing + Clamped Scale")
    solutions["Hybrid: Bound Smoothing + Clamped Scale"] = res_hybrid

    # 3. Direct Overlay Comparison
    print("\n[+] Generating Direct Overlay Comparison Plot...")
    svg_overlay = plot_overlay_comparison(track, solutions, "01_track_1_solutions_all_overlay_comparison.svg")

    print("\n" + "=" * 75)
    print("  TRACK 1 BENCHMARK COMPLETED SUCCESSFULLY!")
    print("=" * 75)
    print(f"  * Baseline:   {svg_base}")
    print(f"  * Solution 1: {svg_sol1}")
    print(f"  * Hybrid:     {svg_hyb}")
    print(f"  * Overlay:    {svg_overlay}")

if __name__ == "__main__":
    main()
