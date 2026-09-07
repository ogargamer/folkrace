#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - 15 GENETIC CHILDREN: 2 HIGH-VARIANCE TRACKS (TRACK 2 & TRACK 6)
================================================================================
Evaluates 15 diverse hybrid offspring generated from Green x Magenta genetic space
on the 2 circuits with the highest geometric and velocity variance:
  - Track 2: Double Hairpin & Slalom (Tight chicanes, high apex sensitivity)
  - Track 6: Multi-Apex High-Speed Loop (Wide sweepers, high curvature sensitivity)
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
    get_track_2_bottleneck_circuit,
    get_track_6_quad_pillar_loop
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

# 15 DIVERSE CHILDREN
CHILDREN_POPULATION = [
    # --- PARENTS FOR REFERENCE ---
    {"id": "P_GREEN", "name": "Parent A: Green (Balanced GP)", "wk": 2.80, "wp": 0.060, "wj": 0.40, "stages": 9, "iters": 700, "s": 0.015, "sigma": 4.0, "color": "#00ff88", "lw": 2.6, "ls": "--"},
    {"id": "P_MAGENTA", "name": "Parent B: Magenta (Ultra Curv)", "wk": 5.00, "wp": 0.030, "wj": 0.35, "stages": 10, "iters": 800, "s": 0.015, "sigma": 4.0, "color": "#e01e84", "lw": 2.6, "ls": ":"},

    # --- GROUP A: CURVATURE MAXIMIZERS ---
    {"id": "C01", "name": "Child 01: Hyper Arc 9-Stg", "wk": 5.20, "wp": 0.025, "wj": 0.35, "stages": 9, "iters": 700, "s": 0.015, "sigma": 4.0, "color": "#ff0055", "lw": 2.2, "ls": "-"},
    {"id": "C02", "name": "Child 02: Extreme Sweeper", "wk": 4.80, "wp": 0.020, "wj": 0.34, "stages": 9, "iters": 720, "s": 0.015, "sigma": 4.0, "color": "#ff3366", "lw": 2.0, "ls": "-"},
    {"id": "C03", "name": "Child 03: Curv Slicer 8-Stg", "wk": 4.60, "wp": 0.030, "wj": 0.36, "stages": 8, "iters": 650, "s": 0.015, "sigma": 4.0, "color": "#ff5500", "lw": 2.2, "ls": "-"},

    # --- GROUP B: HIGH-EFFICIENCY FAST CONVERGENCE ---
    {"id": "C04", "name": "Child 04: Rapid Pulse 8-Stg", "wk": 3.60, "wp": 0.050, "wj": 0.38, "stages": 8, "iters": 600, "s": 0.015, "sigma": 4.0, "color": "#ffaa00", "lw": 2.2, "ls": "-"},
    {"id": "C05", "name": "Child 05: Sprint 7-Stg", "wk": 3.20, "wp": 0.055, "wj": 0.38, "stages": 7, "iters": 600, "s": 0.015, "sigma": 4.0, "color": "#ffd000", "lw": 2.0, "ls": "-"},
    {"id": "C06", "name": "Child 06: Ultra Sprint 7-Stg", "wk": 4.00, "wp": 0.045, "wj": 0.36, "stages": 7, "iters": 650, "s": 0.015, "sigma": 4.0, "color": "#e5e500", "lw": 2.2, "ls": "-"},

    # --- GROUP C: JERK & DAMPING TUNING ---
    {"id": "C07", "name": "Child 07: Stiff Rail (Wj=0.55)", "wk": 4.40, "wp": 0.040, "wj": 0.55, "stages": 9, "iters": 700, "s": 0.015, "sigma": 4.0, "color": "#9d4edd", "lw": 2.0, "ls": "-"},
    {"id": "C08", "name": "Child 08: Ultra Damped (Wj=0.65)", "wk": 3.80, "wp": 0.050, "wj": 0.65, "stages": 9, "iters": 700, "s": 0.015, "sigma": 4.0, "color": "#7209b7", "lw": 2.0, "ls": "-"},
    {"id": "C09", "name": "Child 09: Quick-Flick (Wj=0.25)", "wk": 4.50, "wp": 0.035, "wj": 0.25, "stages": 9, "iters": 700, "s": 0.015, "sigma": 4.0, "color": "#f72585", "lw": 2.2, "ls": "-"},

    # --- GROUP D: PATH TENSION & STRAITS ---
    {"id": "C10", "name": "Child 10: Taut Apex Hybrid", "wk": 4.20, "wp": 0.075, "wj": 0.38, "stages": 9, "iters": 700, "s": 0.015, "sigma": 4.0, "color": "#f5a623", "lw": 2.0, "ls": "-"},
    {"id": "C11", "name": "Child 11: Extreme Taut String", "wk": 3.50, "wp": 0.090, "wj": 0.40, "stages": 8, "iters": 650, "s": 0.015, "sigma": 4.0, "color": "#d48b10", "lw": 1.8, "ls": "-"},
    {"id": "C12", "name": "Child 12: Loose Rubber Band", "wk": 4.60, "wp": 0.012, "wj": 0.35, "stages": 9, "iters": 720, "s": 0.015, "sigma": 4.0, "color": "#ff4d6d", "lw": 2.0, "ls": "-"},

    # --- GROUP E: GEOMETRY & SPLINE HYBRIDS ---
    {"id": "C13", "name": "Child 13: Razor Spline (s=0.008)", "wk": 4.30, "wp": 0.038, "wj": 0.36, "stages": 9, "iters": 700, "s": 0.008, "sigma": 4.0, "color": "#aacc00", "lw": 2.0, "ls": "-"},
    {"id": "C14", "name": "Child 14: Macro Filter (Sig=6.0)", "wk": 4.50, "wp": 0.040, "wj": 0.36, "stages": 9, "iters": 700, "s": 0.015, "sigma": 6.0, "color": "#00b4d8", "lw": 2.0, "ls": "-"},
    {"id": "C15", "name": "Child 15: Grand Prix Master", "wk": 4.10, "wp": 0.042, "wj": 0.37, "stages": 9, "iters": 700, "s": 0.015, "sigma": 4.0, "color": "#00f0ff", "lw": 2.6, "ls": "-"}
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

def plot_15children_overlay(track, centerline, results, filename, track_title):
    fig, ax = plt.subplots(figsize=(20, 12), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    # 1. Medial Axis Centerline (Grey Dotted)
    c_wrap = np.vstack([centerline, centerline[:1]])
    ax.plot(c_wrap[:, 0], c_wrap[:, 1], color="#64748b", linewidth=1.4, linestyle=":", label="Medial Axis (Centerline)")

    # 2. Plot all 17 Profiles (2 Parents + 15 Children)
    for item in results:
        cfg = item["config"]
        perf = item["perf"]
        pts = item["points"]
        pts_wrap = np.vstack([pts, pts[:1]])
        lbl = f"{cfg['name']} [{perf['solve_ms']:.0f}ms | Apex: {perf['apex_spd']:.2f}m/s | Lap: {perf['lap_time']:.2f}s]"
        ax.plot(pts_wrap[:, 0], pts_wrap[:, 1], color=cfg["color"], linewidth=cfg["lw"], linestyle=cfg["ls"], label=lbl)

    # Start / Finish point
    start_pt = results[0]["points"][0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#00f0ff", s=140, zorder=9, label="Start / Finish")

    num_obs = len(track.get("obstacles", []))
    ax.set_title(f"{track_title} ({num_obs} Obstacles): 15-Children Genetic Laboratory", color=TEXT_COLOR, fontsize=15, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.2, color="#fff")
    ax.tick_params(colors=TEXT_COLOR)
    ax.legend(bbox_to_anchor=(1.02, 1.0), loc="upper left", facecolor="#121826", edgecolor="#2d3748", labelcolor=TEXT_COLOR, framealpha=1.0, fontsize=8.6, borderpad=0.9, labelspacing=0.65)

    svg_path = os.path.join(OUTPUT_DIR, filename)
    plt.savefig(svg_path, format="svg", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    post_process_cad_svg(svg_path)
    return svg_path

def main():
    print("=" * 95)
    print("   DA WINNA - 15 GENETIC CHILDREN ON 2 HIGH-VARIANCE TRACKS")
    print("=" * 95)

    test_tracks = [
        (2, get_track_2_bottleneck_circuit, "Track 2: Double Hairpin & Slalom (Chicanes & Hairpins)"),
        (6, get_track_6_quad_pillar_loop, "Track 6: Multi-Apex High-Speed Loop (High-Speed Sweepers)")
    ]

    all_results = {}
    t_start = time.perf_counter()

    for idx, getter, title in test_tracks:
        print(f"\n[TRACK {idx}] Running 17 Evaluations on {title}...")
        track = getter()
        vor = find_centerline_voronoi(track)
        centerline = vor["centerline"]

        track_results = []
        track_data = {}

        for p_cfg in CHILDREN_POPULATION:
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

            vels = res["velocities"]
            apex_spd = float(np.min(vels))
            pts = res["racing_line"]
            diffs = np.diff(pts, axis=0)
            diffs = np.vstack([diffs, pts[0] - pts[-1]])
            ds = np.hypot(diffs[:, 0], diffs[:, 1])
            lap_time = float(np.sum(ds / np.maximum(vels, 0.5)))

            track_data[p_cfg["id"]] = {
                "solve_ms": t_solve,
                "apex_spd": apex_spd,
                "lap_time": lap_time
            }

            track_results.append({
                "config": p_cfg,
                "perf": {"solve_ms": t_solve, "apex_spd": apex_spd, "lap_time": lap_time},
                "points": pts
            })
            print(f"    - {p_cfg['name']:<35}: {t_solve:5.0f} ms | Apex: {apex_spd:4.2f} m/s | Lap: {lap_time:4.2f} s")

        all_results[f"Track_{idx}"] = track_data
        filename = f"0{idx}_track_{idx}_15children_genetic_overlay.svg"
        plot_15children_overlay(track, centerline, track_results, filename, title)

    # Save JSON data
    json_path = os.path.join(BASE_DIR, "racing_results", "15_children_genetic_results.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(all_results, f, indent=2)

    # Copy SVGs to artifacts
    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    for f in os.listdir(OUTPUT_DIR):
        if f.endswith('.svg'):
            shutil.copy2(os.path.join(OUTPUT_DIR, f), os.path.join(art_dir, f))

    t_total = time.perf_counter() - t_start
    print("\n" + "=" * 95)
    print(f"  15 CHILDREN EVALUATION COMPLETE IN {t_total:.2f} s!")
    print("=" * 95)

if __name__ == "__main__":
    main()
