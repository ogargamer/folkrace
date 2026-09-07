#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - TUM / F1TENTH RACING LINE BENCHMARK (10 COMPLEX TRACKS)
================================================================================
Features:
  1. Method 1: Medial Axis (Voronoi Skeleton) Obstacle-Aware Centerline
  2. TUM / F1TENTH Minimum Curvature Quadratic Programming (QP) Architecture
  3. Smart Geometric Obstacle Validator (100% verified inside-track obstacles)
  4. Track 4 preserved as requested, Tracks 1-3 & 5-10 completely redesigned
  5. ONLY Scalable Vector Graphics (.svg) with CAD non-scaling-stroke (No PNGs)
================================================================================
"""

import os
import sys
import time
import numpy as np

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
from webots/optimization/racing_engine.realtime_pathfinder import RealtimePathfinder
from webots/optimization/racing_engine.visualizer import plot_racing_line_with_iterations, OUTPUT_DIR

def print_header(title):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)

def benchmark_tum_track(track_getter, prefix, title):
    print_header(title)
    track = track_getter()
    print(f"[+] Loaded: {track['name']} ({len(track['blocks'])} tiles)")
    print(f"    Validated Obstacles (100% Inside Corridor): {len(track.get('obstacles', []))}")
    for idx, obs in enumerate(track.get('obstacles', [])):
        print(f"      * Obs {idx+1}: {obs['name']} ({obs['type']})")

    # 1. Compute Centerline via Method 1: Medial Axis (Voronoi Skeleton)
    print("\n[+] Step 1: Computing Obstacle-Aware Medial Axis (Method 1: Voronoi)...")
    t0 = time.perf_counter()
    vor_res = find_centerline_voronoi(track, num_points=360)
    t_vor_ms = (time.perf_counter() - t0) * 1000.0
    print(f"    [OK] Voronoi Medial Axis Centerline generated in {t_vor_ms:.1f} ms.")

    # 2. Run TUM / F1TENTH Minimum Curvature QP Optimizer
    print("\n[+] Step 2: Running TUM Minimum Curvature Quadratic Programming (QP)...")
    optimizer = TUMMinimumCurvatureOptimizer()
    t0 = time.perf_counter()
    opt_result = optimizer.optimize_racing_line(track, vor_res)
    t_opt_ms = (time.perf_counter() - t0) * 1000.0

    print(f"    [OK] TUM Optimization complete in {t_opt_ms:.1f} ms.")
    print(f"    * Captured {len(opt_result['iteration_history'])} evolution milestones.")
    print(f"    * Apex Cornering Speed: {np.min(opt_result['velocities']):.2f} m/s")
    print(f"    * Straightaway / Chicane Cut Speed: {np.max(opt_result['velocities']):.2f} m/s (100% Full Throttle)")

    # Save CAD-like Vector SVG Snapshot ONLY (No PNG)
    img_base = os.path.join(OUTPUT_DIR, f"{prefix}_tum_racing_line_iterations.svg")
    out_files = plot_racing_line_with_iterations(track, opt_result, img_base)
    print(f"    [OK] Saved CAD Vector SVG (Infinite Zoom): {out_files['svg']}")

    # 3. Benchmark 60 Hz Real-Time Controller
    pathfinder = RealtimePathfinder(wheelbase=0.25, max_steer_deg=25.0)
    racing_line = opt_result["racing_line"]
    velocities = opt_result["velocities"]
    
    times_ms = []
    robot_pose = [0.0, 0.0, 0.0, 1.2]
    for i in range(400):
        v, steer_deg, diag = pathfinder.compute_control_step(
            robot_pose=robot_pose,
            global_racing_line=racing_line,
            global_velocities=velocities
        )
        times_ms.append(diag["compute_ms"])
        dt = 0.0166
        robot_pose[0] += v * np.cos(robot_pose[2]) * dt
        robot_pose[1] += v * np.sin(robot_pose[2]) * dt
        robot_pose[2] += (v / 0.25) * np.tan(np.radians(steer_deg)) * dt
        robot_pose[3] = v

    avg_ms = np.mean(times_ms)
    p99_ms = np.percentile(times_ms, 99)
    print(f"[OK] 60 Hz Real-Time Controller Timing on 1 Core:")
    print(f"    * Average Cycle Time: {avg_ms:.3f} ms  (Uses only {avg_ms/16.6*100:.1f}% CPU of 1 Core)")
    print(f"    * 99th Percentile:    {p99_ms:.3f} ms")

def main():
    print("=" * 70)
    print("  DA WINNA - 10-TRACK TUM / F1TENTH RACING LINE SUITE (SVG ONLY)")
    print("=" * 70)
    print(f"Artifact Snapshots Directory: {OUTPUT_DIR}\n")
    
    t_start = time.time()
    
    # Remove all old PNG and SVG files
    for f in os.listdir(OUTPUT_DIR):
        if f.endswith('.png') or f.endswith('.svg'):
            try:
                os.remove(os.path.join(OUTPUT_DIR, f))
            except:
                pass

    # All 10 Tracks (Max 20 Pieces Each)
    tracks_to_run = [
        (get_track_1_chicane_with_pillar, "01_track_1", "TRACK 1: High-Speed Chicane Arena (16 Tiles, 4 Obstacles)"),
        (get_track_2_bottleneck_circuit, "02_track_2", "TRACK 2: Double Hairpin & Slalom (16 Tiles, 4 Obstacles)"),
        (get_track_3_slalom_pillars, "03_track_3", "TRACK 3: Grand Prix Arena (18 Tiles, 5 Obstacles)"),
        (get_track_4_switchback_arena, "04_track_4", "TRACK 4: Technical Switchback Arena (14 Tiles, 2 Obstacles) [KEPT]"),
        (get_track_5_dual_choke_chicane_ring, "05_track_5", "TRACK 5: Quad-Choke Bottleneck Ring (16 Tiles, 4 Obstacles)"),
        (get_track_6_quad_pillar_loop, "06_track_6", "TRACK 6: Multi-Apex High-Speed Circuit (18 Tiles, 5 Obstacles)"),
        (get_track_7_multi_bottleneck_slalom, "07_track_7", "TRACK 7: Slalom Speedway (18 Tiles, 5 Obstacles)"),
        (get_track_8_dual_chicane_arena, "08_track_8", "TRACK 8: Long-Course Championship Track (20 Tiles, 6 Obstacles)"),
        (get_track_9_super_speedway, "09_track_9", "TRACK 9: Double Switchback Arena (20 Tiles, 6 Obstacles)"),
        (get_track_10_switchback_obstacle_maze, "10_track_10", "TRACK 10: Complex 20-Tile Obstacle Maze (20 Tiles, 6 Obstacles)")
    ]

    for getter, prefix, title in tracks_to_run:
        benchmark_tum_track(getter, prefix, title)

    elapsed = time.time() - t_start

    print_header("ALL 10 TUM CAD VECTOR SVG BENCHMARKS COMPLETED!")
    print(f"Total Suite Execution Time: {elapsed:.2f} s")
    print(f"\nGenerated CAD Vector SVG Files (No PNGs):")
    for f in sorted(os.listdir(OUTPUT_DIR)):
        if f.endswith('.svg'):
            fpath = os.path.join(OUTPUT_DIR, f)
            print(f"  [SVG] {f} ({os.path.getsize(fpath)/1024:.1f} KB)")
            print(f"     Path: {fpath}")

if __name__ == "__main__":
    main()
