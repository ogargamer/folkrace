#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - NATIVE C++ vs PYTHON RACING ENGINE BENCHMARK SUITE
================================================================================
Compares the native C++ Engine vs Python Engine across all 10 tracks on:
  - Solve Time (ms / us)
  - Speedup Factor
  - Minimum Apex Speed (m/s)
  - Lap Time (s)
  - Obstacle Clearance (cm)
  - Trajectory Equivalence (mm deviation)
================================================================================
"""

import os
import sys
import time
import shutil
import json
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
from racing_engine.centerline_finder import find_centerline_voronoi, get_all_boundary_and_obstacle_points
from racing_engine.racing_line_optimizer import TUMMinimumCurvatureOptimizer
from racing_engine.visualizer import (
    draw_track_walls_and_obstacles,
    OUTPUT_DIR,
    BG_COLOR,
    TEXT_COLOR,
    PAN_ZOOM_SCRIPT
)
from racing_engine_cpp.racing_engine_wrapper import TUM_RacingEngine_CPP_Wrapper

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

def plot_cpp_vs_python(track, py_res, cpp_res, filename, track_title):
    fig, ax = plt.subplots(figsize=(18, 11), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    # 1. Medial Axis Centerline
    c_pts = cpp_res["centerline"]
    c_wrap = np.vstack([c_pts, c_pts[:1]])
    ax.plot(c_wrap[:, 0], c_wrap[:, 1], color="#64748b", linewidth=1.4, linestyle=":", label="Medial Centerline (Voronoi)")

    # 2. Python Baseline (Pink Dash)
    py_pts = py_res["racing_line"]
    py_wrap = np.vstack([py_pts, py_pts[:1]])
    py_lbl = f"Python Engine [{py_res['solve_time_ms']:.0f} ms | Apex: {py_res['min_apex_speed']:.2f} m/s | Lap: {py_res['lap_time']:.2f}s]"
    ax.plot(py_wrap[:, 0], py_wrap[:, 1], color="#ff007f", linewidth=2.4, linestyle="--", alpha=0.85, label=py_lbl)

    # 3. Native C++ Engine (Cyan Solid)
    cpp_pts = cpp_res["racing_line"]
    cpp_wrap = np.vstack([cpp_pts, cpp_pts[:1]])
    speedup = py_res['solve_time_ms'] / max(cpp_res['solve_time_ms'], 0.01)
    cpp_lbl = f"Native C++ Engine [{cpp_res['solve_time_ms']:.2f} ms ({speedup:.0f}x Faster!) | Apex: {cpp_res['min_apex_speed']:.2f} m/s | Lap: {cpp_res['lap_time']:.2f}s]"
    ax.plot(cpp_wrap[:, 0], cpp_wrap[:, 1], color="#00f0ff", linewidth=2.8, linestyle="-", label=cpp_lbl)

    # Start / Finish Point
    start_pt = cpp_pts[0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#00ff88", s=130, zorder=8, label="Start / Finish")

    num_obs = len(track.get("obstacles", []))
    ax.set_title(f"{track_title} ({num_obs} Obstacles): C++ vs Python Real-Time Benchmark", color=TEXT_COLOR, fontsize=14, fontweight='bold', pad=15)
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
    print("=" * 115)
    print("   DA WINNA - NATIVE C++ vs PYTHON BENCHMARK ON ALL 10 TRACKS")
    print("=" * 115)

    track_getters = [
        (1, get_track_1_chicane_with_pillar, "Track 1: Chicane Arena"),
        (2, get_track_2_bottleneck_circuit, "Track 2: Double Hairpin"),
        (3, get_track_3_slalom_pillars, "Track 3: Grand Prix Arena"),
        (4, get_track_4_switchback_arena, "Track 4: Technical Switchback"),
        (5, get_track_5_dual_choke_chicane_ring, "Track 5: Quad-Choke Ring"),
        (6, get_track_6_quad_pillar_loop, "Track 6: Multi-Apex Circuit"),
        (7, get_track_7_multi_bottleneck_slalom, "Track 7: Slalom Speedway"),
        (8, get_track_8_dual_chicane_arena, "Track 8: Dual Chicane Speedway"),
        (9, get_track_9_super_speedway, "Track 9: Double Switchback Arena"),
        (10, get_track_10_switchback_obstacle_maze, "Track 10: Obstacle Maze")
    ]

    py_opt = TUMMinimumCurvatureOptimizer(
        weight_curvature=3.20, weight_path_smoothness=0.055, weight_jerk=0.38,
        robot_safety_margin=0.10, num_progression_stages=7, max_solver_sub_iters=300
    )

    cpp_engine = TUM_RacingEngine_CPP_Wrapper(
        weight_curvature=3.20, weight_path_smoothness=0.055, weight_jerk=0.38,
        robot_safety_margin=0.10, num_progression_stages=7, max_solver_sub_iters=300
    )

    results_table = []

    for idx, getter, title in track_getters:
        track = getter()

        # 1. Run Python Engine
        vor = find_centerline_voronoi(track)
        t0_py = time.perf_counter()
        py_raw = py_opt.optimize_racing_line(track, vor)
        py_solve_ms = (time.perf_counter() - t0_py) * 1000.0

        py_vels = py_raw["velocities"]
        py_min_spd = float(np.min(py_vels))
        py_ds = py_raw["ds"]
        py_lap_t = float(np.sum(py_ds / np.maximum(py_vels, 0.5)))
        py_res = {
            "racing_line": py_raw["racing_line"],
            "solve_time_ms": py_solve_ms,
            "min_apex_speed": py_min_spd,
            "lap_time": py_lap_t
        }

        # 2. Run Native C++ Engine (run 10 times for accurate microsecond timing)
        # Warmup
        cpp_res = cpp_engine.optimize_racing_line(track, vor["centerline"])
        cpp_times = []
        for _ in range(10):
            t0 = time.perf_counter()
            cpp_res = cpp_engine.optimize_racing_line(track, vor["centerline"])
            cpp_times.append((time.perf_counter() - t0) * 1000.0)
        cpp_solve_ms = float(np.median(cpp_times))
        cpp_res["solve_time_ms"] = cpp_solve_ms

        speedup = py_solve_ms / max(cpp_solve_ms, 0.001)

        # 3. Trajectory Equivalence (mean and max deviation in mm)
        py_pts = py_res["racing_line"]
        cpp_pts = cpp_res["racing_line"]
        diffs_mm = np.hypot(cpp_pts[:, 0] - py_pts[:, 0], cpp_pts[:, 1] - py_pts[:, 1]) * 1000.0
        max_dev_mm = float(np.max(diffs_mm))
        mean_dev_mm = float(np.mean(diffs_mm))

        results_table.append({
            "track_idx": idx,
            "track_name": title,
            "py_solve_ms": py_solve_ms,
            "cpp_solve_ms": cpp_solve_ms,
            "speedup": speedup,
            "py_apex_spd": py_min_spd,
            "cpp_apex_spd": cpp_res["min_apex_speed"],
            "py_lap_t": py_lap_t,
            "cpp_lap_t": cpp_res["lap_time"],
            "min_clearance_cm": cpp_res["min_clearance_cm"],
            "max_dev_mm": max_dev_mm,
            "mean_dev_mm": mean_dev_mm
        })

        filename = f"{idx:02d}_track_{idx}_cpp_vs_python_overlay.svg"
        plot_cpp_vs_python(track, py_res, cpp_res, filename, title)
        print(f"[{idx:02d}/10] {title:32s} | Py: {py_solve_ms:5.0f}ms -> C++: {cpp_solve_ms:5.2f}ms ({speedup:4.0f}x) | Clearance: {cpp_res['min_clearance_cm']:4.1f}cm | Max Dev: {max_dev_mm:4.1f}mm")

    # Save JSON results
    json_path = os.path.join(BASE_DIR, "racing_results", "cpp_vs_python_benchmark_results.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(results_table, f, indent=2)

    # Copy SVGs to artifacts
    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    for f in os.listdir(OUTPUT_DIR):
        if f.endswith('.svg'):
            shutil.copy2(os.path.join(OUTPUT_DIR, f), os.path.join(art_dir, f))

    print("\n" + "=" * 115)
    print("ALL 10 TRACKS C++ vs PYTHON BENCHMARK COMPLETE!")
    print("=" * 115)

if __name__ == "__main__":
    main()
