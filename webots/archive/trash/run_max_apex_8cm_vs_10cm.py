#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - 10 TRACKS: MAX APEX (8CM MARGIN vs 10CM MARGIN)
================================================================================
Compares the identical Max Apex optimizer parameters with only the safety margin changed:
  - Grey Dotted: Medial Axis (Centerline)
  - Bold Pink:   Max Apex (8cm Margin)  - Wk=3.5, Wpath=0.03, Wjerk=0.35
  - Neon Green:  Max Apex (10cm Margin) - Wk=3.5, Wpath=0.03, Wjerk=0.35
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

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
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
from webots/optimization/racing_engine.racing_line_optimizer import TUMMinimumCurvatureOptimizer
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

def plot_2margin_overlay(track, centerline, res_8cm, res_10cm, filename, track_title):
    fig, ax = plt.subplots(figsize=(16, 10), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    # 1. Medial Axis Centerline (Grey Dotted)
    c_wrap = np.vstack([centerline, centerline[:1]])
    ax.plot(c_wrap[:, 0], c_wrap[:, 1], color="#64748b", linewidth=1.5, linestyle=":", label="Medial Axis (Centerline)")

    # 2. Max Apex (8cm Margin) - Pink
    pts_8 = res_8cm["racing_line"]
    pts_8_wrap = np.vstack([pts_8, pts_8[:1]])
    spd_8 = np.min(res_8cm["velocities"])
    ax.plot(pts_8_wrap[:, 0], pts_8_wrap[:, 1], color="#ff0055", linewidth=2.8, linestyle="-", label=f"Max Apex (8cm Margin) [Apex: {spd_8:.2f} m/s]")

    # 3. Max Apex (10cm Margin) - Neon Green
    pts_10 = res_10cm["racing_line"]
    pts_10_wrap = np.vstack([pts_10, pts_10[:1]])
    spd_10 = np.min(res_10cm["velocities"])
    ax.plot(pts_10_wrap[:, 0], pts_10_wrap[:, 1], color="#00ff88", linewidth=2.8, linestyle="-", label=f"Max Apex (10cm Margin) [Apex: {spd_10:.2f} m/s]")

    # Start / Finish
    start_pt = pts_8[0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#00f0ff", s=120, zorder=7, label="Start / Finish")

    ax.set_title(f"{track_title}: Max Apex (8cm Margin vs 10cm Margin)", color=TEXT_COLOR, fontsize=13, fontweight='bold', pad=15)
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
    print("=" * 80)
    print("   DA WINNA - 10 TRACKS: MAX APEX (8CM vs 10CM MARGIN)")
    print("=" * 80)

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

    # Exactly identical Max Apex parameters
    opt_8cm = TUMMinimumCurvatureOptimizer(
        weight_curvature=3.50,
        weight_path_smoothness=0.03,
        weight_jerk=0.35,
        robot_safety_margin=0.08,
        num_progression_stages=10,
        max_solver_sub_iters=800,
        max_boundary_slope=0.0
    )

    opt_10cm = TUMMinimumCurvatureOptimizer(
        weight_curvature=3.50,
        weight_path_smoothness=0.03,
        weight_jerk=0.35,
        robot_safety_margin=0.10,
        num_progression_stages=10,
        max_solver_sub_iters=800,
        max_boundary_slope=0.0
    )

    t_suite_start = time.perf_counter()

    for idx, getter, title in track_getters:
        print(f"\n[{idx:02d}/10] Processing {title}...")
        track = getter()
        vor = find_centerline_voronoi(track)
        centerline = vor["centerline"]

        # Run 8cm Max Apex
        res_8cm = opt_8cm.optimize_racing_line(track, vor)

        # Run 10cm Max Apex
        res_10cm = opt_10cm.optimize_racing_line(track, vor)

        filename = f"{idx:02d}_track_{idx}_max_apex_8cm_vs_10cm.svg"
        svg_path = plot_2margin_overlay(track, centerline, res_8cm, res_10cm, filename, title)
        print(f"    [OK] Generated 8cm vs 10cm SVG: {os.path.basename(svg_path)}")

    # Copy to artifacts
    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    for f in os.listdir(OUTPUT_DIR):
        if f.endswith('.svg'):
            shutil.copy2(os.path.join(OUTPUT_DIR, f), os.path.join(art_dir, f))

    t_total = time.perf_counter() - t_suite_start
    print("\n" + "=" * 80)
    print(f"  ALL 10 TRACKS COMPLETED IN {t_total:.2f} s!")
    print("=" * 80)

if __name__ == "__main__":
    main()
