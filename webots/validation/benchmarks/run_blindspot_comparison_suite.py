#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - 135° TOF BLINDSPOT RACING LINE COMPARISON SUITE (10 TRACKS)
================================================================================
Compares 5 racing line variants across all 10 tracks:
  1. [Cyan] Baseline Minimum Curvature (Pure geometric traction line)
  2. [Orange] Late-Apex Geometric Line (Asymmetric entry=0.75, exit=1.30)
  3. [Magenta] Active 135° LOS QP Line (QP penalty on blind corners)
  4. [Yellow] ToF Horizon Velocity Line (Baseline path + 135° ToF velocity limit)
  5. [Green] Full Combined Safe-Racing Line (Late-Apex + LOS + ToF Profile)
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
import matplotlib.patches as patches
import scipy.spatial as spatial

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
from webots/optimization/racing_engine.centerline_finder import find_centerline_voronoi, get_all_boundary_and_obstacle_points
from webots/optimization/racing_engine.racing_line_optimizer import TUMMinimumCurvatureOptimizer
from webots/optimization/racing_engine.blindspot_simulator import (
    solve_qp_variant,
    compute_tof_safe_velocity_profile,
    simulate_135deg_tof_frustum,
    compute_corridor_bounds
)
from webots/optimization/racing_engine.visualizer import (
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

def plot_5_variant_blindspot_overlay(track, variants_data, filename, track_title):
    fig, ax = plt.subplots(figsize=(20, 12), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    # 1. Medial Centerline
    centerline = variants_data["centerline"]
    c_wrap = np.vstack([centerline, centerline[:1]])
    ax.plot(c_wrap[:, 0], c_wrap[:, 1], color="#475569", linewidth=1.2, linestyle=":", label="Medial Centerline (Voronoi)")

    # 2. Variant 1: Pure Minimum Curvature Baseline (Cyan)
    v1 = variants_data["v1_baseline"]
    p1 = np.vstack([v1["racing_line"], v1["racing_line"][:1]])
    lbl1 = f"1. Baseline Min-Curvature [{v1['lap_time']:.2f}s | Min Apex: {v1['min_apex_speed']:.2f} m/s | Sight: {v1['min_sight']:.2f}m]"
    ax.plot(p1[:, 0], p1[:, 1], color="#00f0ff", linewidth=2.2, linestyle="-", alpha=0.75, label=lbl1)

    # 3. Variant 2: Late-Apex Geometric Line (Orange)
    v2 = variants_data["v2_late_apex"]
    p2 = np.vstack([v2["racing_line"], v2["racing_line"][:1]])
    lbl2 = f"2. Late-Apex Line [{v2['lap_time']:.2f}s | Min Apex: {v2['min_apex_speed']:.2f} m/s | Sight: {v2['min_sight']:.2f}m]"
    ax.plot(p2[:, 0], p2[:, 1], color="#ff9900", linewidth=2.2, linestyle="--", alpha=0.85, label=lbl2)

    # 4. Variant 3: Active 135° LOS QP Line (Magenta)
    v3 = variants_data["v3_los_qp"]
    p3 = np.vstack([v3["racing_line"], v3["racing_line"][:1]])
    lbl3 = f"3. Active 135° LOS QP Line [{v3['lap_time']:.2f}s | Min Apex: {v3['min_apex_speed']:.2f} m/s | Sight: {v3['min_sight']:.2f}m]"
    ax.plot(p3[:, 0], p3[:, 1], color="#ff007f", linewidth=2.2, linestyle="-.", alpha=0.85, label=lbl3)

    # 5. Variant 4: ToF Horizon Velocity Line (Yellow)
    v4 = variants_data["v4_tof_velocity"]
    p4 = np.vstack([v4["racing_line"], v4["racing_line"][:1]])
    lbl4 = f"4. ToF Stopping-Horizon Profile [{v4['lap_time']:.2f}s | Min Apex: {v4['min_apex_speed']:.2f} m/s | Sight: {v4['min_sight']:.2f}m]"
    ax.plot(p4[:, 0], p4[:, 1], color="#ffd700", linewidth=2.0, linestyle=":", alpha=0.80, label=lbl4)

    # 6. Variant 5: Full Combined Safe-Racing Line (Lime Green - Thick)
    v5 = variants_data["v5_combined"]
    p5 = np.vstack([v5["racing_line"], v5["racing_line"][:1]])
    lbl5 = f"5. Full Combined Safe-Racing Line [{v5['lap_time']:.2f}s | Min Apex: {v5['min_apex_speed']:.2f} m/s | Sight: {v5['min_sight']:.2f}m]"
    ax.plot(p5[:, 0], p5[:, 1], color="#00ff88", linewidth=3.2, linestyle="-", label=lbl5)

    # 7. Draw Sample 135° 12-Zone ToF Frustums at Blind Apexes
    # Find the top 3 worst sightline waypoints on Variant 5
    worst_sight_indices = np.argsort(v5["tof"]["forward_sight"])[:3]
    for w_idx in worst_sight_indices:
        px, py = v5["racing_line"][w_idx]
        heading = v5["props"]["heading"][w_idx]
        z_ranges = v5["tof"]["zone_ranges"][w_idx]
        z_angles = np.linspace(-np.radians(135.0 / 2.0), np.radians(135.0 / 2.0), 12)
        
        for a, r in zip(z_angles, z_ranges):
            beam_ang = heading + a
            bx = px + r * np.cos(beam_ang)
            by = py + r * np.sin(beam_ang)
            ax.plot([px, bx], [py, by], color="#00ff88", linewidth=0.8, alpha=0.35)
            
        ax.scatter([px], [py], color="#ff0055", s=80, zorder=7)
        ax.annotate(f"Blind Apex\nSight: {v5['tof']['forward_sight'][w_idx]:.2f}m\nSpeed: {v5['velocities'][w_idx]:.2f} m/s",
                    (px, py), textcoords="offset points", xytext=(12, 10),
                    color="#ffffff", fontsize=8.2, fontweight="bold",
                    bbox=dict(boxstyle="round,pad=0.3", fc="#1e293b", ec="#ff0055", alpha=0.9))

    # Start / Finish point
    start_pt = v5["racing_line"][0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#ffffff", s=120, zorder=8, label="Start / Finish")

    num_obs = len(track.get("obstacles", []))
    ax.set_title(f"{track_title} ({num_obs} Obstacles): 135° Matrix ToF Blindspot Reduction Comparison",
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
    print("   DA WINNA - 135° MATRIX TOF BLINDSPOT RACING LINE COMPARISON SUITE")
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

    opt_engine = TUMMinimumCurvatureOptimizer()
    benchmark_results = []

    for idx, getter, title in track_getters:
        track = getter()
        
        # 1. Medial Centerline & Corridor Boundaries
        vor = find_centerline_voronoi(track)
        centerline = vor["centerline"]
        
        # Spatial Tree of barriers & obstacles
        all_barriers = get_all_boundary_and_obstacle_points(track, point_spacing=0.02)
        barrier_tree = spatial.cKDTree(all_barriers)
        
        # Corridor Raycast Bounds
        min_bounds, max_bounds = compute_corridor_bounds(centerline, barrier_tree)

        # =========================================================================
        # 2. VARIANT 1: Baseline Minimum Curvature (Pure Geometric Traction)
        # =========================================================================
        v1_raw = solve_qp_variant(track, centerline, min_bounds, max_bounds, barrier_tree,
                                  entry_aggressiveness=1.0, exit_aggressiveness=1.0, los_penalty_weight=0.0)
        v1_vels = opt_engine._generate_velocity_profile(v1_raw["props"]["curvature"], v1_raw["props"]["ds"])
        v1_data = {
            "racing_line": v1_raw["racing_line"],
            "props": v1_raw["props"],
            "tof": v1_raw["tof"],
            "velocities": v1_vels,
            "lap_time": float(np.sum(v1_raw["props"]["ds"] / np.maximum(v1_vels, 0.5))),
            "min_apex_speed": float(np.min(v1_vels)),
            "min_sight": float(np.min(v1_raw["tof"]["forward_sight"]))
        }

        # =========================================================================
        # 3. VARIANT 2: Late-Apex Geometric Line (Asymmetric Entry/Exit)
        # =========================================================================
        v2_raw = solve_qp_variant(track, centerline, min_bounds, max_bounds, barrier_tree,
                                  entry_aggressiveness=0.75, exit_aggressiveness=1.30, los_penalty_weight=0.0)
        v2_vels = opt_engine._generate_velocity_profile(v2_raw["props"]["curvature"], v2_raw["props"]["ds"])
        v2_data = {
            "racing_line": v2_raw["racing_line"],
            "props": v2_raw["props"],
            "tof": v2_raw["tof"],
            "velocities": v2_vels,
            "lap_time": float(np.sum(v2_raw["props"]["ds"] / np.maximum(v2_vels, 0.5))),
            "min_apex_speed": float(np.min(v2_vels)),
            "min_sight": float(np.min(v2_raw["tof"]["forward_sight"]))
        }

        # =========================================================================
        # 4. VARIANT 3: Active 135° LOS QP Line (Blindspot Penalty in QP)
        # =========================================================================
        v3_raw = solve_qp_variant(track, centerline, min_bounds, max_bounds, barrier_tree,
                                  entry_aggressiveness=1.0, exit_aggressiveness=1.0, los_penalty_weight=4.5)
        v3_vels = opt_engine._generate_velocity_profile(v3_raw["props"]["curvature"], v3_raw["props"]["ds"])
        v3_data = {
            "racing_line": v3_raw["racing_line"],
            "props": v3_raw["props"],
            "tof": v3_raw["tof"],
            "velocities": v3_vels,
            "lap_time": float(np.sum(v3_raw["props"]["ds"] / np.maximum(v3_vels, 0.5))),
            "min_apex_speed": float(np.min(v3_vels)),
            "min_sight": float(np.min(v3_raw["tof"]["forward_sight"]))
        }

        # =========================================================================
        # 5. VARIANT 4: ToF Horizon Velocity Line (Baseline Path + ToF Velocity)
        # =========================================================================
        v4_vel = compute_tof_safe_velocity_profile(v1_raw["props"], v1_raw["tof"])
        v4_data = {
            "racing_line": v1_raw["racing_line"],
            "props": v1_raw["props"],
            "tof": v1_raw["tof"],
            "velocities": v4_vel["velocities"],
            "lap_time": v4_vel["lap_time"],
            "min_apex_speed": v4_vel["min_apex_speed"],
            "min_sight": float(np.min(v1_raw["tof"]["forward_sight"]))
        }

        # =========================================================================
        # 6. VARIANT 5: Full Combined Safe-Racing Line (Late-Apex + LOS + ToF Profile)
        # =========================================================================
        v5_raw = solve_qp_variant(track, centerline, min_bounds, max_bounds, barrier_tree,
                                  entry_aggressiveness=0.75, exit_aggressiveness=1.30, los_penalty_weight=3.5)
        v5_vel = compute_tof_safe_velocity_profile(v5_raw["props"], v5_raw["tof"])
        v5_data = {
            "racing_line": v5_raw["racing_line"],
            "props": v5_raw["props"],
            "tof": v5_raw["tof"],
            "velocities": v5_vel["velocities"],
            "lap_time": v5_vel["lap_time"],
            "min_apex_speed": v5_vel["min_apex_speed"],
            "min_sight": float(np.min(v5_raw["tof"]["forward_sight"]))
        }

        variants_bundle = {
            "centerline": centerline,
            "v1_baseline": v1_data,
            "v2_late_apex": v2_data,
            "v3_los_qp": v3_data,
            "v4_tof_velocity": v4_data,
            "v5_combined": v5_data
        }

        filename = f"{idx:02d}_track_{idx}_blindspot_variants_overlay.svg"
        plot_5_variant_blindspot_overlay(track, variants_bundle, filename, title)

        benchmark_results.append({
            "track_idx": idx,
            "track_name": title,
            "v1_lap_time": v1_data["lap_time"],
            "v1_min_sight": v1_data["min_sight"],
            "v2_lap_time": v2_data["lap_time"],
            "v2_min_sight": v2_data["min_sight"],
            "v3_lap_time": v3_data["lap_time"],
            "v3_min_sight": v3_data["min_sight"],
            "v4_lap_time": v4_data["lap_time"],
            "v4_min_sight": v4_data["min_sight"],
            "v5_lap_time": v5_data["lap_time"],
            "v5_min_sight": v5_data["min_sight"],
            "sight_gain_pct": ((v5_data["min_sight"] - v1_data["min_sight"]) / max(v1_data["min_sight"], 0.01)) * 100.0,
            "lap_time_diff_s": v5_data["lap_time"] - v1_data["lap_time"]
        })

        print(f"[{idx:02d}/10] {title:30s} | Base: {v1_data['lap_time']:.2f}s (Sight: {v1_data['min_sight']:.2f}m) -> Safe: {v5_data['lap_time']:.2f}s (Sight: {v5_data['min_sight']:.2f}m | +{benchmark_results[-1]['sight_gain_pct']:.0f}% Sight)")

    # Save JSON results
    json_path = os.path.join(BASE_DIR, "racing_results", "blindspot_comparison_results.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(benchmark_results, f, indent=2)

    # Sync to artifacts
    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    for f in os.listdir(OUTPUT_DIR):
        if "blindspot" in f and f.endswith('.svg'):
            shutil.copy2(os.path.join(OUTPUT_DIR, f), os.path.join(art_dir, f))

    print("\n" + "=" * 115)
    print("ALL 10 TRACKS 135° TOF BLINDSPOT COMPARISON BENCHMARK COMPLETE!")
    print("=" * 115)

if __name__ == "__main__":
    main()
