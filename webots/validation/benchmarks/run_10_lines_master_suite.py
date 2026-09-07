#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - 10 TRACKS: 10-LINE MAX APEX MASTER SUITE
================================================================================
Generates 10 CAD Scalable Vector Graphics (.svg) with 10 systematically varied
racing line profiles anchored around the Pink Baseline Max Apex:
  1. Line 1: Baseline Max Apex (Pink)        - Wk=3.50, Wp=0.03, Wj=0.35, Stg=10, Iter=800
  2. Line 2: Fast Homotopy (Orange)          - Wk=3.50, Wp=0.03, Wj=0.35, Stg=8,  Iter=600 (1.6x faster)
  3. Line 3: Ultra Curvature Arc (Magenta)   - Wk=5.00, Wp=0.03, Wj=0.35, Stg=10 (Max curvature pull)
  4. Line 4: Low-Tension Sweeper (Coral)     - Wk=3.50, Wp=0.015, Wj=0.35, Stg=10 (Half path tension)
  5. Line 5: Stiff Damped Steering (Purple)  - Wk=3.50, Wp=0.03, Wj=0.70, Stg=10 (Double jerk damping)
  6. Line 6: Taut Straightaways (Gold)       - Wk=3.50, Wp=0.08, Wj=0.35, Stg=10 (Stiffer straight line)
  7. Line 7: Macro Smoothing Sigma 6 (Teal)  - Wk=3.50, Wp=0.03, Wj=0.35, Sigma=6.0
  8. Line 8: Razor Fit Spline s=0.005 (Lime) - Wk=3.50, Wp=0.03, Wj=0.35, s=0.005
  9. Line 9: Balanced-Apex Blend (Green)     - Wk=2.80, Wp=0.06, Wj=0.40, Stg=9
 10. Line 10: Rapid 6-Stage Conv (Cyan)      - Wk=3.50, Wp=0.03, Wj=0.35, Stg=6,  Iter=700 (2.2x faster)
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

# 10 DEFINITIVE PROFILES
LINE_PROFILES = [
    {
        "id": 1,
        "name": "Line 1: Baseline Max Apex (Pink)",
        "desc": "Reference baseline configuration",
        "wk": 3.50, "wp": 0.03, "wj": 0.35, "sigma": 4.0, "s": 0.015, "stages": 10, "iters": 800,
        "color": "#ff0055", "lw": 3.2, "ls": "-"
    },
    {
        "id": 2,
        "name": "Line 2: Fast Homotopy 8-Stg (Orange)",
        "desc": "Baseline params with 8 stages (1.6x faster)",
        "wk": 3.50, "wp": 0.03, "wj": 0.35, "sigma": 4.0, "s": 0.015, "stages": 8, "iters": 600,
        "color": "#ff7700", "lw": 2.4, "ls": "-"
    },
    {
        "id": 3,
        "name": "Line 3: Ultra Curvature Wk=5.0 (Magenta)",
        "desc": "Higher curvature pull, wider outer swing",
        "wk": 5.00, "wp": 0.03, "wj": 0.35, "sigma": 4.0, "s": 0.015, "stages": 10, "iters": 800,
        "color": "#e01e84", "lw": 2.2, "ls": "-"
    },
    {
        "id": 4,
        "name": "Line 4: Low-Tension Wp=0.015 (Coral)",
        "desc": "Half path tension, freer lateral exploration",
        "wk": 3.50, "wp": 0.015, "wj": 0.35, "sigma": 4.0, "s": 0.015, "stages": 10, "iters": 800,
        "color": "#ff5a36", "lw": 2.0, "ls": "-"
    },
    {
        "id": 5,
        "name": "Line 5: Stiff Damped Wj=0.70 (Purple)",
        "desc": "Double jerk damping, smoother steering rate",
        "wk": 3.50, "wp": 0.03, "wj": 0.70, "sigma": 4.0, "s": 0.015, "stages": 10, "iters": 800,
        "color": "#9d4edd", "lw": 2.2, "ls": "-"
    },
    {
        "id": 6,
        "name": "Line 6: Taut Straights Wp=0.08 (Gold)",
        "desc": "Stiffer straight lines, identical apex points",
        "wk": 3.50, "wp": 0.08, "wj": 0.35, "sigma": 4.0, "s": 0.015, "stages": 10, "iters": 800,
        "color": "#f5a623", "lw": 2.0, "ls": "-"
    },
    {
        "id": 7,
        "name": "Line 7: Macro Smoothing Sigma 6 (Teal)",
        "desc": "Broader curvature spatial filter",
        "wk": 3.50, "wp": 0.03, "wj": 0.35, "sigma": 6.0, "s": 0.015, "stages": 10, "iters": 800,
        "color": "#00b4d8", "lw": 2.0, "ls": "-"
    },
    {
        "id": 8,
        "name": "Line 8: Razor Fit Spline s=0.005 (Lime)",
        "desc": "Tight spline tolerance to raw QP output",
        "wk": 3.50, "wp": 0.03, "wj": 0.35, "sigma": 4.0, "s": 0.005, "stages": 10, "iters": 800,
        "color": "#aacc00", "lw": 1.8, "ls": "-"
    },
    {
        "id": 9,
        "name": "Line 9: Balanced-Apex Blend (Green)",
        "desc": "Intermediate blend (Wk=2.8, Wp=0.06)",
        "wk": 2.80, "wp": 0.06, "wj": 0.40, "sigma": 4.0, "s": 0.015, "stages": 9, "iters": 700,
        "color": "#00ff88", "lw": 2.4, "ls": "-"
    },
    {
        "id": 10,
        "name": "Line 10: Rapid 6-Stage Conv (Cyan)",
        "desc": "6 stages with 700 iters (2.2x faster)",
        "wk": 3.50, "wp": 0.03, "wj": 0.35, "sigma": 4.0, "s": 0.015, "stages": 6, "iters": 700,
        "color": "#48cae4", "lw": 2.2, "ls": "--"
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

def plot_10lines_overlay(track, centerline, results, filename, track_title):
    fig, ax = plt.subplots(figsize=(18, 11), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    # 1. Medial Axis Centerline (Grey Dotted)
    c_wrap = np.vstack([centerline, centerline[:1]])
    ax.plot(c_wrap[:, 0], c_wrap[:, 1], color="#64748b", linewidth=1.4, linestyle=":", label="Medial Axis (Centerline)")

    # 2. Plot all 10 Profiles
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
    ax.set_title(f"{track_title} ({num_obs} Obstacles): 10-Line Max Apex Master Suite", color=TEXT_COLOR, fontsize=14, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.2, color="#fff")
    ax.tick_params(colors=TEXT_COLOR)
    ax.legend(bbox_to_anchor=(1.02, 1.0), loc="upper left", facecolor="#121826", edgecolor="#2d3748", labelcolor=TEXT_COLOR, framealpha=1.0, fontsize=9.0, borderpad=1.0, labelspacing=0.7)

    svg_path = os.path.join(OUTPUT_DIR, filename)
    plt.savefig(svg_path, format="svg", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    post_process_cad_svg(svg_path)
    return svg_path

def main():
    print("=" * 85)
    print("   DA WINNA - 10 TRACKS: 10-LINE MAX APEX MASTER SUITE")
    print("=" * 85)

    track_getters = [
        (1, get_track_1_chicane_with_pillar, "Track 1: High-Speed Chicane Arena"),
        (2, get_track_2_bottleneck_circuit, "Track 2: Double Hairpin & Slalom"),
        (3, get_track_3_slalom_pillars, "Track 3: Grand Prix Arena"),
        (4, get_track_4_switchback_arena, "Track 4: Technical Switchback Arena [KEPT]"),
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
        for p_cfg in LINE_PROFILES:
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

            apex_spd = np.min(res["velocities"])
            perf = {
                "solve_ms": t_solve,
                "apex_spd": apex_spd
            }
            track_results.append({
                "config": p_cfg,
                "perf": perf,
                "points": res["racing_line"]
            })

        filename = f"{idx:02d}_track_{idx}_10lines_master_overlay.svg"
        svg_path = plot_10lines_overlay(track, centerline, track_results, filename, title)
        print(f"    [OK] Generated 10-Line Master Overlay SVG: {os.path.basename(svg_path)}")

    # Copy to artifacts
    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    for f in os.listdir(OUTPUT_DIR):
        if f.endswith('.svg'):
            shutil.copy2(os.path.join(OUTPUT_DIR, f), os.path.join(art_dir, f))

    t_total = time.perf_counter() - t_suite_start
    print("\n" + "=" * 85)
    print(f"  ALL 10 MASTER OVERLAYS COMPLETED IN {t_total:.2f} s!")
    print("=" * 85)

if __name__ == "__main__":
    main()
