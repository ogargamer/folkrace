#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - GENETIC HYBRIDIZATION (GREEN x MAGENTA) ACROSS 10 TRACKS
================================================================================
Evolves and benchmarks offspring combining:
  - Parent Green (Balanced GP):   Fastest Solve (2137 ms), 9 Stages, Wk=2.8, Wp=0.06
  - Parent Magenta (Ultra Curv):  Highest Apex Speed (2.36 m/s), 10 Stages, Wk=5.0, Wp=0.03
================================================================================
"""

import os
import sys
import time
import json
import shutil
import numpy as np
import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
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

# PARENTS + GENETIC OFFSPRING POPULATION
GENETIC_POPULATION = [
    # --- PARENTS ---
    {
        "id": "P_GREEN",
        "name": "Parent A: Green (Balanced GP)",
        "type": "Parent",
        "wk": 2.80, "wp": 0.060, "wj": 0.40, "stages": 9, "iters": 700,
        "color": "#00ff88", "lw": 2.6, "ls": "--"
    },
    {
        "id": "P_MAGENTA",
        "name": "Parent B: Magenta (Ultra Curv)",
        "type": "Parent",
        "wk": 5.00, "wp": 0.030, "wj": 0.35, "stages": 10, "iters": 800,
        "color": "#e01e84", "lw": 2.6, "ls": ":"
    },

    # --- OFFSPRING (CROSSOVER + MUTATIONS) ---
    {
        "id": "F1_BALANCED_5050",
        "name": "Child 1: 50/50 Hybrid (Neon Lime)",
        "type": "Offspring",
        "wk": 3.90, "wp": 0.045, "wj": 0.375, "stages": 9, "iters": 700,
        "color": "#aacc00", "lw": 2.4, "ls": "-"
    },
    {
        "id": "F1_CURV_DOMINANT",
        "name": "Child 2: Curvature-Dominant 9-Stg (Electric Rose)",
        "type": "Offspring",
        "wk": 4.50, "wp": 0.040, "wj": 0.360, "stages": 9, "iters": 700,
        "color": "#ff007f", "lw": 2.8, "ls": "-"
    },
    {
        "id": "F1_FAST_CONV_8STG",
        "name": "Child 3: Fast-Conv 8-Stg (Bright Amber)",
        "type": "Offspring",
        "wk": 3.80, "wp": 0.045, "wj": 0.370, "stages": 8, "iters": 650,
        "color": "#ffaa00", "lw": 2.4, "ls": "-"
    },
    {
        "id": "F1_LOW_TENSION_SPEED",
        "name": "Child 4: Free-Apex Slicer (Cyan-Green)",
        "type": "Offspring",
        "wk": 4.20, "wp": 0.035, "wj": 0.360, "stages": 9, "iters": 700,
        "color": "#00f0ff", "lw": 2.5, "ls": "-"
    },
    {
        "id": "F1_HIGH_DAMPED_APEX",
        "name": "Child 5: Stiff-Damped High-Arc (Royal Violet)",
        "type": "Offspring",
        "wk": 4.60, "wp": 0.050, "wj": 0.420, "stages": 9, "iters": 720,
        "color": "#b5179e", "lw": 2.3, "ls": "-"
    }
]

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

def plot_genetic_overlay(track, centerline, results, filename, track_title):
    fig, ax = plt.subplots(figsize=(18, 11), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    # 1. Medial Axis Centerline (Grey Dotted)
    c_wrap = np.vstack([centerline, centerline[:1]])
    ax.plot(c_wrap[:, 0], c_wrap[:, 1], color="#64748b", linewidth=1.4, linestyle=":", label="Medial Axis (Centerline)")

    # 2. Plot Parents & Offspring
    for item in results:
        cfg = item["config"]
        perf = item["perf"]
        pts = item["points"]
        pts_wrap = np.vstack([pts, pts[:1]])
        lbl = f"{cfg['name']} [{perf['solve_ms']:.0f}ms | Apex: {perf['apex_spd']:.2f}m/s | Lap: {perf['lap_time']:.2f}s]"
        ax.plot(pts_wrap[:, 0], pts_wrap[:, 1], color=cfg["color"], linewidth=cfg["lw"], linestyle=cfg["ls"], label=lbl)

    # Start / Finish point
    start_pt = results[0]["points"][0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#00f0ff", s=130, zorder=8, label="Start / Finish")

    num_obs = len(track.get("obstacles", []))
    ax.set_title(f"{track_title} ({num_obs} Obstacles): Genetic Evolution (Green x Magenta)", color=TEXT_COLOR, fontsize=14, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.2, color="#fff")
    ax.tick_params(colors=TEXT_COLOR)
    ax.legend(bbox_to_anchor=(1.02, 1.0), loc="upper left", facecolor="#121826", edgecolor="#2d3748", labelcolor=TEXT_COLOR, framealpha=1.0, fontsize=9.2, borderpad=1.0, labelspacing=0.8)

    svg_path = os.path.join(OUTPUT_DIR, filename)
    plt.savefig(svg_path, format="svg", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    post_process_cad_svg(svg_path)
    return svg_path

def main():
    print("=" * 95)
    print("   DA WINNA - GENETIC HYBRIDIZATION (GREEN x MAGENTA) ACROSS 10 TRACKS")
    print("=" * 95)

    track_getters = [
        (1, get_track_1_chicane_with_pillar, "Track 1: Chicane Arena"),
        (2, get_track_2_bottleneck_circuit, "Track 2: Double Hairpin"),
        (3, get_track_3_slalom_pillars, "Track 3: Grand Prix Arena"),
        (4, get_track_4_switchback_arena, "Track 4: Technical Switchback [KEPT]"),
        (5, get_track_5_dual_choke_chicane_ring, "Track 5: Quad-Choke Ring"),
        (6, get_track_6_quad_pillar_loop, "Track 6: Multi-Apex Circuit"),
        (7, get_track_7_multi_bottleneck_slalom, "Track 7: Slalom Speedway"),
        (8, get_track_8_dual_chicane_arena, "Track 8: Dual Chicane Speedway"),
        (9, get_track_9_super_speedway, "Track 9: Double Switchback Arena"),
        (10, get_track_10_switchback_obstacle_maze, "Track 10: Obstacle Maze")
    ]

    # Pre-cache tracks and centerlines
    track_cache = []
    for idx, getter, title in track_getters:
        track = getter()
        vor = find_centerline_voronoi(track)
        track_cache.append((idx, track, vor, title))

    genetic_data = {
        p["id"]: {
            "name": p["name"],
            "type": p["type"],
            "wk": p["wk"],
            "wp": p["wp"],
            "wj": p["wj"],
            "stages": p["stages"],
            "times": {},
            "apex_speeds": {},
            "lap_times": {}
        } for p in GENETIC_POPULATION
    }

    t_suite_start = time.perf_counter()

    for idx, track, vor, title in track_cache:
        centerline = vor["centerline"]
        track_results = []
        print(f"\n[{idx:02d}/10] Running Genetic Hybridization on {title}...")

        for p_cfg in GENETIC_POPULATION:
            opt = TUMMinimumCurvatureOptimizer(
                weight_curvature=p_cfg["wk"],
                weight_path_smoothness=p_cfg["wp"],
                weight_jerk=p_cfg["wj"],
                macro_sigma=4.0,
                spline_s=0.015,
                robot_safety_margin=0.10,
                num_progression_stages=p_cfg["stages"],
                max_solver_sub_iters=p_cfg["iters"],
                max_boundary_slope=0.0
            )

            t0 = time.perf_counter()
            res = opt.optimize_racing_line(track, vor)
            t_solve = (time.perf_counter() - t0) * 1000.0

            vels = res["velocities"]
            apex_spd = float(np.min(vels))
            pts = res["racing_line"]
            diffs = np.diff(pts, axis=0)
            diffs = np.vstack([diffs, pts[0] - pts[-1]])
            ds = np.hypot(diffs[:, 0], diffs[:, 1])
            lap_time = float(np.sum(ds / np.maximum(vels, 0.5)))

            genetic_data[p_cfg["id"]]["times"][idx] = t_solve
            genetic_data[p_cfg["id"]]["apex_speeds"][idx] = apex_spd
            genetic_data[p_cfg["id"]]["lap_times"][idx] = lap_time

            track_results.append({
                "config": p_cfg,
                "perf": {"solve_ms": t_solve, "apex_spd": apex_spd, "lap_time": lap_time},
                "points": pts
            })
            print(f"    - {p_cfg['name']:<48}: {t_solve:5.0f} ms | Apex: {apex_spd:.2f} m/s | Lap: {lap_time:.2f}s")

        filename = f"{idx:02d}_track_{idx}_genetic_hybrid_overlay.svg"
        plot_genetic_overlay(track, centerline, track_results, filename, title)

    # Save results to JSON
    json_path = os.path.join(BASE_DIR, "racing_results", "genetic_green_magenta_results.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(genetic_data, f, indent=2)

    # Copy SVGs to artifacts
    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    for f in os.listdir(OUTPUT_DIR):
        if f.endswith('.svg'):
            shutil.copy2(os.path.join(OUTPUT_DIR, f), os.path.join(art_dir, f))

    t_total = time.perf_counter() - t_suite_start
    print("\n" + "=" * 95)
    print(f"  GENETIC EXPERIMENT COMPLETED ACROSS ALL 10 TRACKS IN {t_total:.2f} s!")
    print("=" * 95)

if __name__ == "__main__":
    main()
