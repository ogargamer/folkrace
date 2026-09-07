#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - DEFINITIVE 10-TRACK BENCHMARK & RE-EVALUATION SUITE
================================================================================
Evaluates the 7 remaining candidates (excluding rejected blue and orange):
  1. Profile 1: Baseline Max Apex (Pink)        - Wk=3.50, Wp=0.03, Wj=0.35, 10 Stg
  2. Profile 2: Ultra Curvature Arc (Magenta)   - Wk=5.00, Wp=0.03, Wj=0.35, 10 Stg
  3. Profile 3: Low-Tension Sweeper (Coral)     - Wk=3.50, Wp=0.015, Wj=0.35, 10 Stg
  4. Profile 4: Taut Straightaways (Gold)       - Wk=3.50, Wp=0.08, Wj=0.35, 10 Stg
  5. Profile 5: Razor Fit Spline (Lime)         - Wk=3.50, Wp=0.03, Wj=0.35, s=0.005, 10 Stg
  6. Profile 6: Stiff Damped Steering (Purple)  - Wk=3.50, Wp=0.03, Wj=0.70, 10 Stg
  7. Profile 7: Balanced-Apex Blend (Green)     - Wk=2.80, Wp=0.06, Wj=0.40, 9 Stg
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

# 7 CANDIDATES (EXCLUDING BLUE & ORANGE)
CANDIDATE_PROFILES = [
    {
        "id": 1,
        "name": "Line 1: Baseline Max Apex (Pink)",
        "short_name": "Pink (Baseline)",
        "wk": 3.50, "wp": 0.03, "wj": 0.35, "sigma": 4.0, "s": 0.015, "stages": 10, "iters": 800,
        "color": "#ff0055", "lw": 3.2, "ls": "-"
    },
    {
        "id": 2,
        "name": "Line 3: Ultra Curvature (Magenta)",
        "short_name": "Magenta (Ultra Curv)",
        "wk": 5.00, "wp": 0.03, "wj": 0.35, "sigma": 4.0, "s": 0.015, "stages": 10, "iters": 800,
        "color": "#e01e84", "lw": 2.4, "ls": "-"
    },
    {
        "id": 3,
        "name": "Line 4: Low-Tension (Coral)",
        "short_name": "Coral (Low-Tension)",
        "wk": 3.50, "wp": 0.015, "wj": 0.35, "sigma": 4.0, "s": 0.015, "stages": 10, "iters": 800,
        "color": "#ff5a36", "lw": 2.2, "ls": "-"
    },
    {
        "id": 4,
        "name": "Line 6: Taut Straights (Gold)",
        "short_name": "Gold (Taut Straights)",
        "wk": 3.50, "wp": 0.08, "wj": 0.35, "sigma": 4.0, "s": 0.015, "stages": 10, "iters": 800,
        "color": "#f5a623", "lw": 2.2, "ls": "-"
    },
    {
        "id": 5,
        "name": "Line 8: Razor Fit Spline (Lime)",
        "short_name": "Lime (Razor Spline)",
        "wk": 3.50, "wp": 0.03, "wj": 0.35, "sigma": 4.0, "s": 0.005, "stages": 10, "iters": 800,
        "color": "#aacc00", "lw": 2.0, "ls": "-"
    },
    {
        "id": 6,
        "name": "Line 5: Stiff Damped (Purple)",
        "short_name": "Purple (Stiff Damped)",
        "wk": 3.50, "wp": 0.03, "wj": 0.70, "sigma": 4.0, "s": 0.015, "stages": 10, "iters": 800,
        "color": "#9d4edd", "lw": 2.2, "ls": "-"
    },
    {
        "id": 7,
        "name": "Line 9: Balanced-Apex (Green)",
        "short_name": "Green (Balanced GP)",
        "wk": 2.80, "wp": 0.06, "wj": 0.40, "sigma": 4.0, "s": 0.015, "stages": 9, "iters": 700,
        "color": "#00ff88", "lw": 2.6, "ls": "-"
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

def plot_candidates_overlay(track, centerline, results, filename, track_title):
    fig, ax = plt.subplots(figsize=(18, 11), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    # 1. Medial Axis Centerline (Grey Dotted)
    c_wrap = np.vstack([centerline, centerline[:1]])
    ax.plot(c_wrap[:, 0], c_wrap[:, 1], color="#64748b", linewidth=1.4, linestyle=":", label="Medial Axis (Centerline)")

    # 2. Plot all 7 Profiles
    for item in results:
        cfg = item["config"]
        perf = item["perf"]
        pts = item["points"]
        pts_wrap = np.vstack([pts, pts[:1]])
        lbl = f"{cfg['name']} [{perf['solve_ms']:.0f}ms | Apex: {perf['apex_spd']:.2f}m/s]"
        ax.plot(pts_wrap[:, 0], pts_wrap[:, 1], color=cfg["color"], linewidth=cfg["lw"], linestyle=cfg["ls"], label=lbl)

    # Start / Finish point
    start_pt = results[0]["points"][0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#00f0ff", s=130, zorder=8, label="Start / Finish")

    num_obs = len(track.get("obstacles", []))
    ax.set_title(f"{track_title} ({num_obs} Obstacles): 7-Candidate Definitive Benchmark", color=TEXT_COLOR, fontsize=14, fontweight='bold', pad=15)
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
    print("   DA WINNA - DEFINITIVE 10-TRACK BENCHMARK & RE-EVALUATION SUITE")
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

    benchmark_data = {p["name"]: {"short_name": p["short_name"], "times": {}, "apex_speeds": {}, "lengths": {}} for p in CANDIDATE_PROFILES}

    t_suite_start = time.perf_counter()

    for idx, track, vor, title in track_cache:
        centerline = vor["centerline"]
        track_results = []
        print(f"\n[{idx:02d}/10] Running Benchmark on {title}...")

        for p_cfg in CANDIDATE_PROFILES:
            opt = TUMMinimumCurvatureOptimizer(
                weight_curvature=p_cfg["wk"],
                weight_path_smoothness=p_cfg["wp"],
                weight_jerk=p_cfg["wj"],
                macro_sigma=p_cfg["sigma"],
                spline_s=p_cfg["s"],
                robot_safety_margin=0.10,
                num_progression_stages=p_cfg["stages"],
                max_solver_sub_iters=p_cfg["iters"],
                max_boundary_slope=0.0
            )

            t0 = time.perf_counter()
            res = opt.optimize_racing_line(track, vor)
            t_solve = (time.perf_counter() - t0) * 1000.0

            apex_spd = float(np.min(res["velocities"]))
            pts = res["racing_line"]
            diffs = np.diff(pts, axis=0)
            diffs = np.vstack([diffs, pts[0] - pts[-1]])
            path_len = float(np.sum(np.hypot(diffs[:, 0], diffs[:, 1])))

            benchmark_data[p_cfg["name"]]["times"][idx] = t_solve
            benchmark_data[p_cfg["name"]]["apex_speeds"][idx] = apex_spd
            benchmark_data[p_cfg["name"]]["lengths"][idx] = path_len

            track_results.append({
                "config": p_cfg,
                "perf": {"solve_ms": t_solve, "apex_spd": apex_spd},
                "points": pts
            })
            print(f"    - {p_cfg['short_name']:<22}: {t_solve:5.0f} ms | Apex: {apex_spd:.2f} m/s")

        filename = f"{idx:02d}_track_{idx}_definitive_benchmark_overlay.svg"
        plot_candidates_overlay(track, centerline, track_results, filename, title)

    # Save benchmark data to JSON
    json_path = os.path.join(BASE_DIR, "racing_results", "definitive_benchmark_results.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(benchmark_data, f, indent=2)

    # Copy SVGs to artifacts
    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    for f in os.listdir(OUTPUT_DIR):
        if f.endswith('.svg'):
            shutil.copy2(os.path.join(OUTPUT_DIR, f), os.path.join(art_dir, f))

    t_total = time.perf_counter() - t_suite_start
    print("\n" + "=" * 95)
    print(f"  ALL 10 TRACKS FULLY BENCHMARKED AND OVERLAY SVGS GENERATED IN {t_total:.2f} s!")
    print("=" * 95)

if __name__ == "__main__":
    main()
