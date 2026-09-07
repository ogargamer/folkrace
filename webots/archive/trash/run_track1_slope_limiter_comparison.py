#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - TRACK 1: BASELINE vs FILTERED vs SLOPE-LIMITED BOUNDARIES
================================================================================
Compares on Track 1:
  1. Baseline TUM / F1TENTH (Raw bounds with 800% fan cliffs)
  2. Filtered TUM (Gaussian smoothed bounds)
  3. Slope-Limited TUM (2-Pass Slew-Rate Boundary Limiter, max_slope = 0.25)
  4. Direct Overlay Comparison Plot
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
import scipy.optimize as opt
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import splprep, splev

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE_DIR)

from webots/optimization/racing_engine.track_generator import get_track_1_chicane_with_pillar
from webots/optimization/racing_engine.centerline_finder import find_centerline_voronoi
from webots/optimization/racing_engine.racing_line_optimizer import (
    compute_spline_properties,
    TUMMinimumCurvatureOptimizer
)
from webots/optimization/racing_engine.solution_comparator import (
    compute_track_bounds,
    build_finite_difference_matrices
)
from webots/optimization/racing_engine.visualizer import (
    draw_track_walls_and_obstacles,
    OUTPUT_DIR,
    BG_COLOR,
    TEXT_COLOR,
    PAN_ZOOM_SCRIPT
)

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

# ==============================================================================
#  METHOD 2: Filtered TUM (Gaussian Bound Filter)
# ==============================================================================
def solve_filtered_tum(track_data, voronoi_centerline_data):
    centerline = voronoi_centerline_data["centerline"]
    N = len(centerline)
    props = compute_spline_properties(centerline)
    normals, kc, ds = props["normals"], props["curvature"], props["ds"]
    
    min_b, max_b, _ = compute_track_bounds(centerline, normals, track_data)
    min_b_smooth = gaussian_filter1d(min_b, sigma=4.0, mode='wrap')
    max_b_smooth = gaussian_filter1d(max_b, sigma=4.0, mode='wrap')
    min_bounds = np.maximum(min_b_smooth, min_b)
    max_bounds = np.minimum(max_b_smooth, max_b)
    bounds = list(zip(min_bounds, max_bounds))
    
    M1, M2, M3 = build_finite_difference_matrices(ds)
    W_ds = np.diag(ds)
    kc_macro = gaussian_filter1d(kc, sigma=4.0, mode='wrap')
    
    H = 2.0 * (M2.T @ W_ds @ M2) + 0.05 * (M1.T @ M1) + 0.40 * (M3.T @ M3) + 1e-5 * np.eye(N)
    f = 2.0 * (M2.T @ W_ds @ kc_macro)
    
    iteration_history = [{"iter": 0, "stage": 0, "alpha": np.zeros(N), "points": centerline.copy(), "description": "Iteration 0: Medial Axis"}]
    cur_alpha = np.zeros(N)
    cumulative_iters = 0
    num_stages = 10
    
    for s_idx, w_step in enumerate(np.linspace(0.10, 1.0, num_stages)):
        f_step = f * w_step
        def qp_obj(a): return 0.5 * a @ H @ a + f_step @ a, H @ a + f_step
        res = opt.minimize(qp_obj, cur_alpha, method='L-BFGS-B', jac=True, bounds=bounds, options={'maxiter': 600, 'ftol': 1e-9})
        cur_alpha = res.x
        cumulative_iters += res.nit
        pts_stage = centerline + normals * cur_alpha[:, None]
        pct = int((s_idx + 1) / num_stages * 100)
        iteration_history.append({
            "iter": cumulative_iters, "stage": s_idx + 1, "points": pts_stage.copy(),
            "description": f"Stage {s_idx+1}/10 ({pct}%): Filtered TUM (Total {cumulative_iters} iters)"
        })
        
    tck, u = splprep([pts_stage[:, 0], pts_stage[:, 1]], s=0.015, per=True)
    final_pts = np.column_stack(splev(np.linspace(0, 1, N, endpoint=False), tck))
    final_props = compute_spline_properties(final_pts)
    
    return {
        "name": "Filtered TUM (Gaussian Bounds)",
        "racing_line": final_pts,
        "iteration_history": iteration_history,
        "curvature": final_props["curvature"]
    }

# ==============================================================================
#  METHOD 3: Slope-Limited TUM (2-Pass Slew Rate Limiter)
# ==============================================================================
def solve_slope_limited_tum(track_data, voronoi_centerline_data, max_slope=0.25):
    """
    Applies a strict 2-pass forward-backward slope limiter on lateral bounds:
    - Caps |db/ds| <= max_slope (e.g. 0.25 m/m = 25% gradient)
    - Replaces 800% vertical fan cliffs with smooth, continuous lead-in tapers
    - Strictly conservative: never violates physical barrier walls
    """
    centerline = voronoi_centerline_data["centerline"]
    N = len(centerline)
    props = compute_spline_properties(centerline)
    normals, kc, ds = props["normals"], props["curvature"], props["ds"]
    
    min_b_raw, max_b_raw, _ = compute_track_bounds(centerline, normals, track_data)
    
    min_bounds = min_b_raw.copy()
    max_bounds = max_b_raw.copy()
    
    # 2-Pass Forward-Backward Slew Rate Limiter
    for _ in range(2):
        # Forward pass (taper entering corners)
        for i in range(N):
            i_prev = (i - 1) % N
            max_bounds[i] = min(max_bounds[i], max_bounds[i_prev] + max_slope * ds[i_prev])
            min_bounds[i] = max(min_bounds[i], min_bounds[i_prev] - max_slope * ds[i_prev])
        # Backward pass (taper exiting corners)
        for i in range(N - 1, -1, -1):
            i_next = (i + 1) % N
            max_bounds[i] = min(max_bounds[i], max_bounds[i_next] + max_slope * ds[i])
            min_bounds[i] = max(min_bounds[i], min_bounds[i_next] - max_slope * ds[i])
            
    bounds = list(zip(min_bounds, max_bounds))
    
    M1, M2, M3 = build_finite_difference_matrices(ds)
    W_ds = np.diag(ds)
    kc_macro = gaussian_filter1d(kc, sigma=4.0, mode='wrap')
    
    H = 2.0 * (M2.T @ W_ds @ M2) + 0.05 * (M1.T @ M1) + 0.40 * (M3.T @ M3) + 1e-5 * np.eye(N)
    f = 2.0 * (M2.T @ W_ds @ kc_macro)
    
    iteration_history = [{"iter": 0, "stage": 0, "alpha": np.zeros(N), "points": centerline.copy(), "description": "Iteration 0: Medial Axis"}]
    cur_alpha = np.zeros(N)
    cumulative_iters = 0
    num_stages = 10
    
    for s_idx, w_step in enumerate(np.linspace(0.10, 1.0, num_stages)):
        f_step = f * w_step
        def qp_obj(a): return 0.5 * a @ H @ a + f_step @ a, H @ a + f_step
        res = opt.minimize(qp_obj, cur_alpha, method='L-BFGS-B', jac=True, bounds=bounds, options={'maxiter': 600, 'ftol': 1e-9})
        cur_alpha = res.x
        cumulative_iters += res.nit
        pts_stage = centerline + normals * cur_alpha[:, None]
        pct = int((s_idx + 1) / num_stages * 100)
        iteration_history.append({
            "iter": cumulative_iters, "stage": s_idx + 1, "points": pts_stage.copy(),
            "description": f"Stage {s_idx+1}/10 ({pct}%): Slope-Limited TUM (Total {cumulative_iters} iters)"
        })
        
    tck, u = splprep([pts_stage[:, 0], pts_stage[:, 1]], s=0.015, per=True)
    final_pts = np.column_stack(splev(np.linspace(0, 1, N, endpoint=False), tck))
    final_props = compute_spline_properties(final_pts)
    
    return {
        "name": "Slope-Limited TUM (2-Pass Slew Limiter)",
        "racing_line": final_pts,
        "iteration_history": iteration_history,
        "curvature": final_props["curvature"],
        "min_bounds": min_bounds,
        "max_bounds": max_bounds
    }

def plot_track_iterations(track, sol_result, filename, title):
    fig, ax = plt.subplots(figsize=(16, 10), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    colors = [
        "#4a5568", "#3a86ff", "#00b4d8", "#06d6a0", "#7209b7",
        "#b5179e", "#f72585", "#ff0054", "#ff5400", "#ffb800", "#00ff88"
    ]
    history = sol_result["iteration_history"]
    for idx, item in enumerate(history):
        c = colors[idx % len(colors)]
        pts = item["points"]
        pts_c = np.vstack([pts, pts[:1]])
        is_final = (idx == len(history) - 1)
        is_initial = (idx == 0)
        
        if is_initial:
            lw = 1.8
            ls = ":"
            lbl = "Initial: Medial Axis (Centerline)"
        elif is_final:
            lw = 2.8
            ls = "-"
            lbl = f"Final Optimal Line ({sol_result.get('name', 'Baseline TUM')})"
        else:
            lw = 1.2
            ls = "--"
            lbl = item["description"]
            
        ax.plot(pts_c[:, 0], pts_c[:, 1], color=c, linewidth=lw, linestyle=ls, label=lbl)

    start_pt = history[-1]["points"][0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#00f0ff", s=120, zorder=7, label="Start / Finish")
    ax.set_title(f"{title} [Track 1 CAD Pan & Wheel Zoom Active]", color=TEXT_COLOR, fontsize=13, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.2, color="#fff")
    ax.tick_params(colors=TEXT_COLOR)
    ax.legend(bbox_to_anchor=(1.02, 1.0), loc="upper left", facecolor="#121826", edgecolor="#2d3748", labelcolor=TEXT_COLOR, framealpha=1.0, fontsize=9, borderpad=1.0, labelspacing=0.8)

    svg_path = os.path.join(OUTPUT_DIR, filename)
    plt.savefig(svg_path, format="svg", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    post_process_cad_svg(svg_path)
    return svg_path

def plot_overlay_comparison(track, solutions, filename):
    fig, ax = plt.subplots(figsize=(16, 10), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    sol_colors = {
        "Baseline TUM": "#ff0055",
        "Filtered TUM (Gaussian Bounds)": "#00b4d8",
        "Slope-Limited TUM (2-Pass Slew Limiter)": "#00ff88"
    }

    for name, data in solutions.items():
        pts = data["racing_line"]
        pts_c = np.vstack([pts, pts[:1]])
        c = sol_colors.get(name, "#ffffff")
        ax.plot(pts_c[:, 0], pts_c[:, 1], color=c, linewidth=2.8, label=name)

    start_pt = list(solutions.values())[0]["racing_line"][0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#00f0ff", s=120, zorder=7, label="Start / Finish")
    ax.set_title("Track 1: Baseline vs Filtered vs Slope-Limited Direct Overlay", color=TEXT_COLOR, fontsize=13, fontweight='bold', pad=15)
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
    print("=" * 75)
    print("  TRACK 1: BASELINE vs FILTERED vs SLOPE-LIMITED BENCHMARK")
    print("=" * 75)

    track = get_track_1_chicane_with_pillar()
    print(f"[+] Loaded Track 1: {track['name']}")
    vor = find_centerline_voronoi(track)

    solutions = {}

    # 1. Baseline TUM
    print("\n[+] Running Baseline TUM (Raw Bounds)...")
    opt_base = TUMMinimumCurvatureOptimizer()
    t0 = time.perf_counter()
    res_base = opt_base.optimize_racing_line(track, vor)
    t_base = (time.perf_counter() - t0) * 1000.0
    print(f"    [OK] Baseline completed in {t_base:.1f} ms.")
    svg_base = plot_track_iterations(track, res_base, "01_track_1_baseline_tum.svg", "Track 1: Baseline TUM (Raw Bounds)")
    solutions["Baseline TUM"] = res_base

    # 2. Filtered TUM
    print("\n[+] Running Filtered TUM (Gaussian Bounds)...")
    t0 = time.perf_counter()
    res_filt = solve_filtered_tum(track, vor)
    t_filt = (time.perf_counter() - t0) * 1000.0
    print(f"    [OK] Filtered TUM completed in {t_filt:.1f} ms.")
    svg_filt = plot_track_iterations(track, res_filt, "01_track_1_filtered_tum.svg", "Track 1: Filtered TUM (Gaussian Bounds)")
    solutions["Filtered TUM (Gaussian Bounds)"] = res_filt

    # 3. Slope-Limited TUM
    print("\n[+] Running Slope-Limited TUM (2-Pass Slew Limiter)...")
    t0 = time.perf_counter()
    res_slope = solve_slope_limited_tum(track, vor, max_slope=0.25)
    t_slope = (time.perf_counter() - t0) * 1000.0
    print(f"    [OK] Slope-Limited TUM completed in {t_slope:.1f} ms.")
    svg_slope = plot_track_iterations(track, res_slope, "01_track_1_slope_limited_tum.svg", "Track 1: Slope-Limited TUM (2-Pass Limiter)")
    solutions["Slope-Limited TUM (2-Pass Slew Limiter)"] = res_slope

    # 4. Direct Overlay
    print("\n[+] Generating Direct Overlay Comparison Plot...")
    svg_overlay = plot_overlay_comparison(track, solutions, "01_track_1_baseline_vs_filtered_vs_slope_limited_overlay.svg")

    # Copy to artifacts
    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    for f in os.listdir(OUTPUT_DIR):
        if f.endswith('.svg'):
            shutil.copy2(os.path.join(OUTPUT_DIR, f), os.path.join(art_dir, f))

    print("\n" + "=" * 75)
    print("  TRACK 1 BENCHMARK COMPLETED SUCCESSFULLY!")
    print("=" * 75)
    print(f"  * Baseline:      {svg_base}")
    print(f"  * Filtered:      {svg_filt}")
    print(f"  * Slope-Limited: {svg_slope}")
    print(f"  * Overlay:       {svg_overlay}")

if __name__ == "__main__":
    main()
