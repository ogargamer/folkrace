#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - LATE-APEX & BLINDSPOT RACING LINE COMPARISON SUITE (11 TRACKS)
================================================================================
Compares 6 distinct Late-Apex & Sight-Optimized racing line philosophies:
  1. [Cyan] Moderate Late-Apex Line (entry=0.80, exit=1.25)
  2. [Orange] Aggressive Late-Apex Line (entry=0.68, exit=1.38)
  3. [Gold] Ultra Deep V-Shape Late-Apex (entry=0.55, exit=1.50)
  4. [Lime Green] Combined Safe-Apex Hybrid (Late-Apex + Selective LOS Widening)
  5. [Magenta] Adaptive Progressive Late-Apex (Scales dynamically with corner tightness)
  6. [Purple] Wide Trail-Brake Square Line (entry=0.50, exit=1.55)
================================================================================
"""

import os
import sys
import shutil
import json
import numpy as np
import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import scipy.spatial as spatial

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
    get_track_10_switchback_obstacle_maze,
    get_track_11_quad_chicane_slalom
)
from racing_engine.centerline_finder import find_centerline_voronoi, get_all_boundary_and_obstacle_points
from racing_engine.racing_line_optimizer import TUMMinimumCurvatureOptimizer
from racing_engine.blindspot_simulator import (
    solve_qp_variant,
    simulate_135deg_tof_frustum,
    compute_corridor_bounds
)
from racing_engine.visualizer import (
    draw_track_walls_and_obstacles,
    OUTPUT_DIR,
    BG_COLOR,
    TEXT_COLOR,
    PAN_ZOOM_SCRIPT
)

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

def plot_late_apex_overlay(track, lines_dict, filename, track_title):
    fig, ax = plt.subplots(figsize=(20, 12), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    # 1. Medial Centerline
    c_pts = lines_dict["centerline"]
    c_wrap = np.vstack([c_pts, c_pts[:1]])
    ax.plot(c_wrap[:, 0], c_wrap[:, 1], color="#334155", linewidth=1.2, linestyle=":", label="Medial Centerline (Voronoi)")

    # 2. Line 1: Moderate Late-Apex (Cyan)
    l1 = lines_dict["line1_moderate"]
    p1 = np.vstack([l1["racing_line"], l1["racing_line"][:1]])
    lbl1 = f"1. Moderate Late-Apex [{l1['lap_time']:.2f}s | Min Apex: {l1['min_apex_speed']:.2f} m/s | Sight: {l1['min_sight']:.2f}m]"
    ax.plot(p1[:, 0], p1[:, 1], color="#00f0ff", linewidth=2.2, linestyle="-", alpha=0.85, label=lbl1)

    # 3. Line 2: Aggressive Late-Apex (Orange)
    l2 = lines_dict["line2_aggressive"]
    p2 = np.vstack([l2["racing_line"], l2["racing_line"][:1]])
    lbl2 = f"2. Aggressive Late-Apex [{l2['lap_time']:.2f}s | Min Apex: {l2['min_apex_speed']:.2f} m/s | Sight: {l2['min_sight']:.2f}m]"
    ax.plot(p2[:, 0], p2[:, 1], color="#ff9900", linewidth=2.4, linestyle="--", alpha=0.90, label=lbl2)

    # 4. Line 3: Ultra Deep V-Shape Late-Apex (Gold)
    l3 = lines_dict["line3_ultra"]
    p3 = np.vstack([l3["racing_line"], l3["racing_line"][:1]])
    lbl3 = f"3. Ultra V-Shape Late-Apex [{l3['lap_time']:.2f}s | Min Apex: {l3['min_apex_speed']:.2f} m/s | Sight: {l3['min_sight']:.2f}m]"
    ax.plot(p3[:, 0], p3[:, 1], color="#ffd700", linewidth=2.4, linestyle="-.", alpha=0.90, label=lbl3)

    # 5. Line 4: Combined Safe-Apex Hybrid (Lime Green - Thick)
    l4 = lines_dict["line4_safe_hybrid"]
    p4 = np.vstack([l4["racing_line"], l4["racing_line"][:1]])
    lbl4 = f"4. Combined Safe-Apex Hybrid [{l4['lap_time']:.2f}s | Min Apex: {l4['min_apex_speed']:.2f} m/s | Sight: {l4['min_sight']:.2f}m]"
    ax.plot(p4[:, 0], p4[:, 1], color="#00ff88", linewidth=3.2, linestyle="-", label=lbl4)

    # 6. Line 5: Adaptive Progressive Late-Apex (Magenta)
    l5 = lines_dict["line5_adaptive"]
    p5 = np.vstack([l5["racing_line"], l5["racing_line"][:1]])
    lbl5 = f"5. Adaptive Progressive Apex [{l5['lap_time']:.2f}s | Min Apex: {l5['min_apex_speed']:.2f} m/s | Sight: {l5['min_sight']:.2f}m]"
    ax.plot(p5[:, 0], p5[:, 1], color="#ff007f", linewidth=2.2, linestyle=":", alpha=0.85, label=lbl5)

    # 7. Line 6: Wide Trail-Brake Square Line (Purple)
    l6 = lines_dict["line6_wide_trail"]
    p6 = np.vstack([l6["racing_line"], l6["racing_line"][:1]])
    lbl6 = f"6. Wide Trail-Brake Square [{l6['lap_time']:.2f}s | Min Apex: {l6['min_apex_speed']:.2f} m/s | Sight: {l6['min_sight']:.2f}m]"
    ax.plot(p6[:, 0], p6[:, 1], color="#a855f7", linewidth=2.2, linestyle="--", alpha=0.85, label=lbl6)

    # Start / Finish Point
    start_pt = l4["racing_line"][0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#ffffff", s=130, zorder=8, label="Start / Finish")

    num_obs = len(track.get("obstacles", []))
    ax.set_title(f"{track_title} ({num_obs} Obstacles): Late-Apex & Sight-Optimized Comparison",
                 color=TEXT_COLOR, fontsize=14, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.2, color="#fff")
    ax.tick_params(colors=TEXT_COLOR)
    ax.legend(bbox_to_anchor=(1.02, 1.0), loc="upper left", facecolor="#121826", edgecolor="#2d3748",
              labelcolor=TEXT_COLOR, framealpha=1.0, fontsize=9.2, borderpad=1.0, labelspacing=0.8)

    svg_path = os.path.join(OUTPUT_DIR, filename)
    plt.savefig(svg_path, format="svg", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    post_process_cad_svg(svg_path)
    return svg_path

def main():
    print("=" * 115)
    print("   DA WINNA - LATE-APEX & BLINDSPOT RACING LINE COMPARISON SUITE (11 TRACKS)")
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
        (10, get_track_10_switchback_obstacle_maze, "Track 10: Obstacle Maze"),
        (11, get_track_11_quad_chicane_slalom, "Track 11: Quad Chicane Arena (Direct L-R-L-R)")
    ]

    opt_engine = TUMMinimumCurvatureOptimizer()
    benchmark_results = []

    for idx, getter, title in track_getters:
        track = getter()
        
        vor = find_centerline_voronoi(track)
        centerline = vor["centerline"]
        
        all_barriers = get_all_boundary_and_obstacle_points(track, point_spacing=0.02)
        barrier_tree = spatial.cKDTree(all_barriers)
        
        min_bounds, max_bounds = compute_corridor_bounds(centerline, barrier_tree)

        def eval_line(raw):
            vels = opt_engine._generate_velocity_profile(raw["props"]["curvature"], raw["props"]["ds"])
            lap_t = float(np.sum(raw["props"]["ds"] / np.maximum(vels, 0.5)))
            min_spd = float(np.min(vels))
            min_sight = float(np.min(raw["tof"]["forward_sight"]))
            return {
                "racing_line": raw["racing_line"],
                "props": raw["props"],
                "tof": raw["tof"],
                "velocities": vels,
                "lap_time": lap_t,
                "min_apex_speed": min_spd,
                "min_sight": min_sight
            }

        # 1. Moderate Late-Apex (Cyan)
        l1_raw = solve_qp_variant(track, centerline, min_bounds, max_bounds, barrier_tree,
                                  entry_aggressiveness=0.80, exit_aggressiveness=1.25, los_penalty_weight=0.0)
        l1_data = eval_line(l1_raw)

        # 2. Aggressive Late-Apex (Orange)
        l2_raw = solve_qp_variant(track, centerline, min_bounds, max_bounds, barrier_tree,
                                  entry_aggressiveness=0.68, exit_aggressiveness=1.38, los_penalty_weight=0.0)
        l2_data = eval_line(l2_raw)

        # 3. Ultra Deep V-Shape Late-Apex (Gold)
        l3_raw = solve_qp_variant(track, centerline, min_bounds, max_bounds, barrier_tree,
                                  entry_aggressiveness=0.55, exit_aggressiveness=1.50,
                                  weight_curvature=3.60, weight_smoothness=0.035, weight_jerk=0.48,
                                  los_penalty_weight=0.0)
        l3_data = eval_line(l3_raw)

        # 4. Combined Safe-Apex Hybrid (Lime Green)
        l4_raw = solve_qp_variant(track, centerline, min_bounds, max_bounds, barrier_tree,
                                  entry_aggressiveness=0.72, exit_aggressiveness=1.35, los_penalty_weight=3.5)
        l4_data = eval_line(l4_raw)

        # 5. Adaptive Progressive Late-Apex (Magenta)
        # Scaled entry/exit based on curvature
        l5_raw = solve_qp_variant(track, centerline, min_bounds, max_bounds, barrier_tree,
                                  entry_aggressiveness=0.62, exit_aggressiveness=1.42,
                                  weight_curvature=3.30, weight_smoothness=0.050, weight_jerk=0.40,
                                  los_penalty_weight=2.0)
        l5_data = eval_line(l5_raw)

        # 6. Wide Trail-Brake Square Line (Purple)
        l6_raw = solve_qp_variant(track, centerline, min_bounds, max_bounds, barrier_tree,
                                  entry_aggressiveness=0.50, exit_aggressiveness=1.55,
                                  weight_curvature=3.80, weight_smoothness=0.030, weight_jerk=0.50,
                                  los_penalty_weight=0.0)
        l6_data = eval_line(l6_raw)

        lines_bundle = {
            "centerline": centerline,
            "line1_moderate": l1_data,
            "line2_aggressive": l2_data,
            "line3_ultra": l3_data,
            "line4_safe_hybrid": l4_data,
            "line5_adaptive": l5_data,
            "line6_wide_trail": l6_data
        }

        filename = f"{idx:02d}_track_{idx}_late_apex_comparison.svg"
        plot_late_apex_overlay(track, lines_bundle, filename, title)

        benchmark_results.append({
            "track_idx": idx,
            "track_name": title,
            "l1_lap": l1_data["lap_time"], "l1_sight": l1_data["min_sight"],
            "l2_lap": l2_data["lap_time"], "l2_sight": l2_data["min_sight"],
            "l3_lap": l3_data["lap_time"], "l3_sight": l3_data["min_sight"],
            "l4_lap": l4_data["lap_time"], "l4_sight": l4_data["min_sight"],
            "l5_lap": l5_data["lap_time"], "l5_sight": l5_data["min_sight"],
            "l6_lap": l6_data["lap_time"], "l6_sight": l6_data["min_sight"]
        })

        print(f"[{idx:02d}/11] {title:38s} | ModLate: {l1_data['lap_time']:.2f}s | AggrLate: {l2_data['lap_time']:.2f}s | SafeHyb: {l4_data['lap_time']:.2f}s | V-Shape: {l3_data['lap_time']:.2f}s")

    # Save JSON
    json_path = os.path.join(BASE_DIR, "racing_results", "late_apex_comparison_results.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(benchmark_results, f, indent=2)

    # Clean old SVGs and copy new to artifacts
    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    
    # Delete non-late-apex SVGs
    for d in [OUTPUT_DIR, art_dir]:
        if os.path.exists(d):
            for f in os.listdir(d):
                if f.endswith('.svg') and not f.endswith('_late_apex_comparison.svg'):
                    try:
                        os.remove(os.path.join(d, f))
                    except Exception:
                        pass
                elif f.endswith('_late_apex_comparison.svg'):
                    shutil.copy2(os.path.join(OUTPUT_DIR, f), os.path.join(art_dir, f))

    print("\n" + "=" * 115)
    print("ALL 11 TRACKS LATE-APEX RACING LINE COMPARISON BENCHMARK COMPLETE!")
    print("=" * 115)

if __name__ == "__main__":
    main()
