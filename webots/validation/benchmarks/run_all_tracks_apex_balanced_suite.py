#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - 10 TRACKS: APEX SPEED vs BALANCED TUNING SPECTRUM (4 OPTIONS)
================================================================================
Evaluates 4 high-performance options along the Max Apex vs Balanced spectrum:
  1. Option 1: Extreme Apex Outlaw (7cm Margin, Wk = 4.0, Wpath = 0.02) - Pink
  2. Option 2: Aggressive Apex (8cm Margin, Wk = 3.2, Wpath = 0.05) - Orange
  3. Option 3: Optimal Grand Prix (10cm Margin, Wk = 2.4, Wpath = 0.08) - Neon Green
  4. Option 4: Safe Balanced (12cm Margin, Wk = 1.8, Wpath = 0.12) - Cyan Blue
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
from webots/optimization/racing_engine.racing_line_optimizer import TUMMinimumCurvatureOptimizer
from webots/optimization/racing_engine.visualizer import (
    draw_track_walls_and_obstacles,
    OUTPUT_DIR,
    BG_COLOR,
    TEXT_COLOR,
    PAN_ZOOM_SCRIPT
)

OPTIONS_SPECTRUM = [
    {
        "name": "Opt 1: Extreme Apex Outlaw",
        "weight_curvature": 4.00,
        "weight_path_smoothness": 0.02,
        "weight_jerk": 0.35,
        "robot_safety_margin": 0.07,
        "num_progression_stages": 8,
        "max_solver_sub_iters": 500,
        "color": "#ff0055",
        "linestyle": "-",
        "lw": 2.8
    },
    {
        "name": "Opt 2: Aggressive Apex",
        "weight_curvature": 3.20,
        "weight_path_smoothness": 0.05,
        "weight_jerk": 0.40,
        "robot_safety_margin": 0.08,
        "num_progression_stages": 8,
        "max_solver_sub_iters": 500,
        "color": "#ff7700",
        "linestyle": "-",
        "lw": 2.6
    },
    {
        "name": "Opt 3: Optimal Grand Prix",
        "weight_curvature": 2.40,
        "weight_path_smoothness": 0.08,
        "weight_jerk": 0.45,
        "robot_safety_margin": 0.10,
        "num_progression_stages": 8,
        "max_solver_sub_iters": 500,
        "color": "#00ff88",
        "linestyle": "-",
        "lw": 2.8
    },
    {
        "name": "Opt 4: Safe Balanced",
        "weight_curvature": 1.80,
        "weight_path_smoothness": 0.12,
        "weight_jerk": 0.50,
        "robot_safety_margin": 0.12,
        "num_progression_stages": 8,
        "max_solver_sub_iters": 500,
        "color": "#00b4d8",
        "linestyle": "--",
        "lw": 2.4
    }
]

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

def plot_4option_overlay(track, centerline, results, filename, track_title):
    fig, ax = plt.subplots(figsize=(16, 10), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    # 1. Medial Axis Centerline (Grey Dotted)
    c_wrap = np.vstack([centerline, centerline[:1]])
    ax.plot(c_wrap[:, 0], c_wrap[:, 1], color="#64748b", linewidth=1.5, linestyle=":", label="Medial Axis (Centerline)")

    # 2. Plot all 4 Options
    for item in results:
        opt_cfg = item["config"]
        perf = item["perf"]
        pts = item["result"]["racing_line"]
        pts_wrap = np.vstack([pts, pts[:1]])
        lbl = f"{opt_cfg['name']} [Apex: {perf['apex_speed']:.2f}m/s | Margin: {opt_cfg['robot_safety_margin']*100:.0f}cm | {perf['solve_ms']:.0f}ms]"
        ax.plot(pts_wrap[:, 0], pts_wrap[:, 1], color=opt_cfg["color"], linewidth=opt_cfg["lw"], linestyle=opt_cfg["linestyle"], label=lbl)

    # Start / Finish
    start_pt = results[0]["result"]["racing_line"][0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#00f0ff", s=120, zorder=7, label="Start / Finish")

    ax.set_title(f"{track_title}: Apex vs Balanced Tuning Spectrum (4 Options)", color=TEXT_COLOR, fontsize=13, fontweight='bold', pad=15)
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
    print("   DA WINNA - 10 TRACKS: APEX SPEED vs BALANCED SPECTRUM (4 OPTIONS)")
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

        track_results = []
        for opt_cfg in OPTIONS_SPECTRUM:
            opt = TUMMinimumCurvatureOptimizer(
                weight_curvature=opt_cfg["weight_curvature"],
                weight_path_smoothness=opt_cfg["weight_path_smoothness"],
                weight_jerk=opt_cfg["weight_jerk"],
                robot_safety_margin=opt_cfg["robot_safety_margin"],
                num_progression_stages=opt_cfg["num_progression_stages"],
                max_solver_sub_iters=opt_cfg["max_solver_sub_iters"],
                max_boundary_slope=0.25
            )

            t0 = time.perf_counter()
            res = opt.optimize_racing_line(track, vor)
            t_solve = (time.perf_counter() - t0) * 1000.0

            apex_spd = np.min(res["velocities"])
            controller_cycle_ms = 0.04 + 0.0002 * len(res["racing_line"])
            cpu_load = (controller_cycle_ms / 16.67) * 100.0

            perf = {
                "solve_ms": t_solve,
                "apex_speed": apex_spd,
                "cpu_load_pct": cpu_load
            }
            track_results.append({
                "config": opt_cfg,
                "perf": perf,
                "result": res
            })

        filename = f"{idx:02d}_track_{idx}_apex_vs_balanced_suite.svg"
        svg_path = plot_4option_overlay(track, centerline, track_results, filename, title)
        print(f"    [OK] Generated 4-Option Overlay SVG: {os.path.basename(svg_path)}")

    # Copy to artifacts
    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    for f in os.listdir(OUTPUT_DIR):
        if f.endswith('.svg'):
            shutil.copy2(os.path.join(OUTPUT_DIR, f), os.path.join(art_dir, f))

    t_total = time.perf_counter() - t_suite_start
    print("\n" + "=" * 85)
    print(f"  ALL 10 TRACK OVERLAYS COMPLETED IN {t_total:.2f} s!")
    print("=" * 85)

if __name__ == "__main__":
    main()
