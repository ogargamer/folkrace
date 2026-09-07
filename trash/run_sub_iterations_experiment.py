#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - SUB-ITERATION SWEEP ON CHILD 05, CHILD 06, AND CHILD 11
================================================================================
Evaluates 10 distinct sub-iteration limits [50, 100, 150, 200, 300, 400, 500, 600, 800, 1200]
on Child 05, Child 06, and Child 11 across Track 2 and Track 6 to map the exact
pareto frontier of Solve Time (ms) vs Apex Speed (m/s) vs Trajectory Error.
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

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
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

# 3 TARGET PROFILES
BASE_PROFILES = [
    {
        "id": "C05",
        "name": "Child 05 (Sprint)",
        "wk": 3.20, "wp": 0.055, "wj": 0.38, "stages": 7,
        "base_color": "#ff007f"
    },
    {
        "id": "C06",
        "name": "Child 06 (Ultra Sprint)",
        "wk": 4.00, "wp": 0.045, "wj": 0.36, "stages": 7,
        "base_color": "#00ff88"
    },
    {
        "id": "C11",
        "name": "Child 11 (Taut String)",
        "wk": 3.50, "wp": 0.090, "wj": 0.40, "stages": 8,
        "base_color": "#f5a623"
    }
]

SUB_ITER_VALUES = [50, 100, 150, 200, 300, 400, 500, 600, 800, 1200]

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

def plot_subiter_overlay(track, centerline, results_by_profile, filename, track_title):
    fig, ax = plt.subplots(figsize=(20, 12), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    # Medial Axis
    c_wrap = np.vstack([centerline, centerline[:1]])
    ax.plot(c_wrap[:, 0], c_wrap[:, 1], color="#64748b", linewidth=1.4, linestyle=":", label="Medial Axis (Centerline)")

    # Color ramps for 10 sub-iter values
    cmap_c05 = matplotlib.colormaps['spring'].resampled(len(SUB_ITER_VALUES))
    cmap_c06 = matplotlib.colormaps['viridis'].resampled(len(SUB_ITER_VALUES))
    cmap_c11 = matplotlib.colormaps['autumn'].resampled(len(SUB_ITER_VALUES))

    cmaps = {"C05": cmap_c05, "C06": cmap_c06, "C11": cmap_c11}

    for p_id, runs in results_by_profile.items():
        cm = cmaps[p_id]
        # Highlight key iterations [50, 150, 300, 600, 1200]
        for idx, run in enumerate(runs):
            s_val = run["sub_iters"]
            pts = run["points"]
            pts_wrap = np.vstack([pts, pts[:1]])
            col = cm(idx / len(SUB_ITER_VALUES))
            
            if s_val in [50, 150, 300, 600, 1200]:
                lbl = f"{run['p_name']} [Iters={s_val:4d} | {run['solve_ms']:4.0f}ms | Apex: {run['apex_spd']:.2f}m/s]"
                lw = 2.4 if s_val in [150, 600] else 1.8
                ls = "-" if s_val in [150, 600] else "--"
                ax.plot(pts_wrap[:, 0], pts_wrap[:, 1], color=col, linewidth=lw, linestyle=ls, label=lbl)
            else:
                ax.plot(pts_wrap[:, 0], pts_wrap[:, 1], color=col, linewidth=1.2, alpha=0.6, linestyle=":")

    start_pt = results_by_profile["C06"][0]["points"][0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#00f0ff", s=140, zorder=9, label="Start / Finish")

    num_obs = len(track.get("obstacles", []))
    ax.set_title(f"{track_title} ({num_obs} Obstacles): Sub-Iteration Sweep (Child 05, 06, 11)", color=TEXT_COLOR, fontsize=15, fontweight='bold', pad=15)
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
    print("   DA WINNA - SUB-ITERATION SWEEP (10 VARIATIONS ON CHILD 05, 06, 11)")
    print("=" * 95)

    test_tracks = [
        (2, get_track_2_bottleneck_circuit, "Track 2: Double Hairpin & Slalom"),
        (6, get_track_6_quad_pillar_loop, "Track 6: Multi-Apex High-Speed Loop")
    ]

    all_experiment_data = {}
    t_start = time.perf_counter()

    for t_idx, getter, title in test_tracks:
        print(f"\n[TRACK {t_idx}] Evaluating 30 Sub-Iteration Variations on {title}...")
        track = getter()
        vor = find_centerline_voronoi(track)
        centerline = vor["centerline"]

        results_by_profile = {"C05": [], "C06": [], "C11": []}
        track_dict = {}

        for p_cfg in BASE_PROFILES:
            p_id = p_cfg["id"]
            p_name = p_cfg["name"]
            track_dict[p_id] = {}
            print(f"  --> Testing {p_name} across 10 sub-iteration settings:")

            # First compute ground-truth reference at max 1200 iters
            opt_ref = TUMMinimumCurvatureOptimizer(
                weight_curvature=p_cfg["wk"], weight_path_smoothness=p_cfg["wp"], weight_jerk=p_cfg["wj"],
                macro_sigma=4.0, spline_s=0.015, robot_safety_margin=0.10,
                num_progression_stages=p_cfg["stages"], max_solver_sub_iters=1200, max_boundary_slope=0.0
            )
            res_ref = opt_ref.optimize_racing_line(track, vor)
            pts_ref = res_ref["racing_line"]

            for sub_it in SUB_ITER_VALUES:
                opt = TUMMinimumCurvatureOptimizer(
                    weight_curvature=p_cfg["wk"], weight_path_smoothness=p_cfg["wp"], weight_jerk=p_cfg["wj"],
                    macro_sigma=4.0, spline_s=0.015, robot_safety_margin=0.10,
                    num_progression_stages=p_cfg["stages"], max_solver_sub_iters=sub_it, max_boundary_slope=0.0
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

                # Deviation from 1200 iters ground truth in cm
                err_cm = float(np.max(np.hypot(pts[:, 0] - pts_ref[:, 0], pts[:, 1] - pts_ref[:, 1])) * 100.0)

                run_entry = {
                    "sub_iters": sub_it,
                    "solve_ms": t_solve,
                    "apex_spd": apex_spd,
                    "lap_time": lap_time,
                    "err_cm": err_cm,
                    "p_name": p_name,
                    "points": pts
                }
                results_by_profile[p_id].append(run_entry)
                track_dict[p_id][str(sub_it)] = {
                    "solve_ms": t_solve,
                    "apex_spd": apex_spd,
                    "lap_time": lap_time,
                    "err_cm": err_cm
                }
                print(f"      [Iters={sub_it:4d}] -> {t_solve:5.0f} ms | Apex: {apex_spd:4.2f} m/s | Lap: {lap_time:4.2f} s | Err: {err_cm:5.2f} cm")

        all_experiment_data[f"Track_{t_idx}"] = track_dict
        filename = f"0{t_idx}_track_{t_idx}_subiters_sweep_overlay.svg"
        plot_subiter_overlay(track, centerline, results_by_profile, filename, title)

    # Save JSON
    json_path = os.path.join(BASE_DIR, "racing_results", "sub_iterations_experiment_results.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(all_experiment_data, f, indent=2)

    # Copy SVGs to artifacts
    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    for f in os.listdir(OUTPUT_DIR):
        if f.endswith('.svg'):
            shutil.copy2(os.path.join(OUTPUT_DIR, f), os.path.join(art_dir, f))

    t_total = time.perf_counter() - t_start
    print("\n" + "=" * 95)
    print(f"  30 SUB-ITERATION VARIATIONS COMPLETED IN {t_total:.2f} s!")
    print("=" * 95)

if __name__ == "__main__":
    main()
