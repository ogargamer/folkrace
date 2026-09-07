#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - SOLUTION 1 (BOUND SMOOTHING) WEIGHT VARIATIONS BENCHMARK
================================================================================
Evaluates 4 distinct spatial bound smoothing weights on Track 1:
  1. Light Smoothing (Sigma = 2.0)
  2. Medium Smoothing (Sigma = 4.0)
  3. Strong Smoothing (Sigma = 6.0)
  4. Ultra Smoothing (Sigma = 8.0)
  5. Multi-Weight Overlay Comparison Plot
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
from webots/optimization/racing_engine.racing_line_optimizer import compute_spline_properties
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

def solve_solution_1_with_sigma(track_data, voronoi_centerline_data, sigma_val, weight_label):
    centerline = voronoi_centerline_data["centerline"]
    N = len(centerline)
    props = compute_spline_properties(centerline)
    normals, kc, ds = props["normals"], props["curvature"], props["ds"]
    
    min_b, max_b, _ = compute_track_bounds(centerline, normals, track_data)
    
    # Filter bounds by specified sigma
    min_b_smooth = gaussian_filter1d(min_b, sigma=sigma_val, mode='wrap')
    max_b_smooth = gaussian_filter1d(max_b, sigma=sigma_val, mode='wrap')
    
    # Conservative enforcement (never exceed physical track wall limits)
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
        def qp_obj(a):
            return 0.5 * a @ H @ a + f_step @ a, H @ a + f_step
        res = opt.minimize(qp_obj, cur_alpha, method='L-BFGS-B', jac=True, bounds=bounds, options={'maxiter': 600, 'ftol': 1e-9})
        cur_alpha = res.x
        cumulative_iters += res.nit
        pts_stage = centerline + normals * cur_alpha[:, None]
        pct = int((s_idx + 1) / num_stages * 100)
        iteration_history.append({
            "iter": cumulative_iters, "stage": s_idx + 1, "points": pts_stage.copy(),
            "description": f"Stage {s_idx+1}/10 ({pct}%): {weight_label} (Total {cumulative_iters} iters)"
        })
        
    tck, u = splprep([pts_stage[:, 0], pts_stage[:, 1]], s=0.015, per=True)
    final_pts = np.column_stack(splev(np.linspace(0, 1, N, endpoint=False), tck))
    final_props = compute_spline_properties(final_pts)
    
    return {
        "label": weight_label,
        "sigma": sigma_val,
        "racing_line": final_pts,
        "iteration_history": iteration_history,
        "curvature": final_props["curvature"],
        "min_bounds": min_bounds,
        "max_bounds": max_bounds
    }

def plot_weight_iterations(track, sol_result, filename, title):
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
            lbl = f"Final Optimal Line ({sol_result['label']})"
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

def plot_all_weights_overlay(track, weights_data, filename):
    fig, ax = plt.subplots(figsize=(16, 10), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)
    draw_track_walls_and_obstacles(ax, track)

    colors = {
        "Light (Sigma = 2.0)": "#00b4d8",
        "Medium (Sigma = 4.0)": "#00ff88",
        "Strong (Sigma = 6.0)": "#ffb800",
        "Ultra (Sigma = 8.0)": "#f72585"
    }

    for item in weights_data:
        lbl = item["label"]
        pts = item["racing_line"]
        pts_c = np.vstack([pts, pts[:1]])
        c = colors.get(lbl, "#ffffff")
        ax.plot(pts_c[:, 0], pts_c[:, 1], color=c, linewidth=2.6, label=lbl)

    start_pt = weights_data[0]["racing_line"][0]
    ax.scatter([start_pt[0]], [start_pt[1]], color="#00f0ff", s=120, zorder=7, label="Start / Finish")
    ax.set_title("Track 1: Solution 1 (Bound Smoothing) All 4 Weights Direct Overlay", color=TEXT_COLOR, fontsize=13, fontweight='bold', pad=15)
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
    print("  DA WINNA - SOLUTION 1 WEIGHT VARIATIONS ON TRACK 1")
    print("=" * 75)

    track = get_track_1_chicane_with_pillar()
    print(f"[+] Loaded: {track['name']}")
    vor = find_centerline_voronoi(track)

    weight_configs = [
        {"sigma": 2.0, "label": "Light (Sigma = 2.0)", "file": "01_track_1_sol1_weight_light_sigma2.svg"},
        {"sigma": 4.0, "label": "Medium (Sigma = 4.0)", "file": "01_track_1_sol1_weight_medium_sigma4.svg"},
        {"sigma": 6.0, "label": "Strong (Sigma = 6.0)", "file": "01_track_1_sol1_weight_strong_sigma6.svg"},
        {"sigma": 8.0, "label": "Ultra (Sigma = 8.0)", "file": "01_track_1_sol1_weight_ultra_sigma8.svg"}
    ]

    results = []
    for cfg in weight_configs:
        print(f"\n[+] Running Solution 1 with {cfg['label']}...")
        t0 = time.perf_counter()
        res = solve_solution_1_with_sigma(track, vor, cfg["sigma"], cfg["label"])
        t_el = (time.perf_counter() - t0) * 1000.0
        print(f"    [OK] Finished in {t_el:.1f} ms.")
        svg_p = plot_weight_iterations(track, res, cfg["file"], f"Track 1 - Solution 1: {cfg['label']}")
        print(f"    [SVG Saved] {svg_p}")
        results.append(res)

    print("\n[+] Generating All-Weights Direct Overlay Comparison Plot...")
    overlay_svg = plot_all_weights_overlay(track, results, "01_track_1_sol1_all_weights_overlay.svg")
    print(f"    [SVG Saved] {overlay_svg}")

    # Copy to artifacts
    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    for f in os.listdir(OUTPUT_DIR):
        if f.endswith('.svg'):
            shutil.copy2(os.path.join(OUTPUT_DIR, f), os.path.join(art_dir, f))

    print("\n" + "=" * 75)
    print("  ALL SOLUTION 1 WEIGHT VARIATIONS GENERATED & SAVED!")
    print("=" * 75)

if __name__ == "__main__":
    main()
