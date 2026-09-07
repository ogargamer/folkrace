#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - SLAM YELLOW RACING LINE (ALPHA-SHAPE WALLS + VORONOI SEED)
   SIDEBAR LAYOUT: All legend and text info placed in a dedicated right sidebar.
================================================================================
"""

import os
import sys
import shutil
import cv2
import numpy as np
import scipy.spatial as spatial
import scipy.optimize as opt
import scipy.ndimage as ndimage
from scipy.interpolate import splprep, splev
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, BASE_DIR)

from racing_engine.track_generator import get_track_3_slalom_pillars, get_track_4_switchback_arena
from racing_engine.centerline_finder import clean_and_resample_spline
from webots_simulation.simulate_cartographer_2cm_grid import simulate_cartographer_slam, rotate_points_2d, OUT_SLAM_DIR

from racing_engine.slam_pipeline.benchmark_all_wall_reconstruction_methods import (
    get_raw_corridor_contours,
    method_3_rolling_disc_alpha_shape
)
from racing_engine.slam_pipeline.run_track3_track4_slam_voronoi import (
    generate_obstacle_aware_voronoi_centerline,
    compute_500mm_reconstructed_walls
)

ART_DIR = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"

def optimize_slam_yellow_line(cl_obs_vor, syn_left, syn_right, slam_res, margin=0.10, weight_smoothness=0.08):
    N = len(cl_obs_vor)
    dx = np.gradient(cl_obs_vor[:, 0])
    dy = np.gradient(cl_obs_vor[:, 1])
    headings = np.arctan2(dy, dx)
    normals_left = np.column_stack([-np.sin(headings), np.cos(headings)])
    
    kdt_walls = spatial.cKDTree(np.vstack([syn_left, syn_right]))
    kdt_occ = spatial.cKDTree(slam_res["occupied_pts"])
    
    # Vectorized multi-ray clearance evaluation
    steps = np.linspace(0.03, 0.70, 35)
    ray_left = cl_obs_vor[:, None, :] + normals_left[:, None, :] * steps[None, :, None]
    ray_right = cl_obs_vor[:, None, :] - normals_left[:, None, :] * steps[None, :, None]
    
    d_wl, _ = kdt_walls.query(ray_left)
    d_ol, _ = kdt_occ.query(ray_left)
    d_wr, _ = kdt_walls.query(ray_right)
    d_or, _ = kdt_occ.query(ray_right)
    
    coll_left = (d_wl < margin) | (d_ol < margin)
    coll_right = (d_wr < margin) | (d_or < margin)
    
    w_left_list = []
    w_right_list = []
    
    for i in range(N):
        # Left wall clearance
        hits_l = np.where(coll_left[i])[0]
        w_L = steps[hits_l[0]] - margin if len(hits_l) > 0 else 0.42
        w_left_list.append(max(0.04, min(0.42, w_L)))
        
        # Right wall clearance
        hits_r = np.where(coll_right[i])[0]
        w_R = steps[hits_r[0]] - margin if len(hits_r) > 0 else 0.42
        w_right_list.append(max(0.04, min(0.42, w_R)))
        
    w_left = np.array(w_left_list)
    w_right = np.array(w_right_list)
    
    ds = np.hypot(dx, dy)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    kappa_c = (dx * ddy - dy * ddx) / (ds**3 + 1e-6)
    
    D2 = np.zeros((N, N))
    for i in range(N):
        D2[i, (i - 1) % N] = 1.0 / (ds[i]**2)
        D2[i, i] = -2.0 / (ds[i]**2)
        D2[i, (i + 1) % N] = 1.0 / (ds[i]**2)
        
    H = D2.T @ D2 + weight_smoothness * np.eye(N)
    f = D2.T @ kappa_c
    bounds = opt.Bounds(-w_right, w_left)
    
    res_opt = opt.minimize(
        fun=lambda a: 0.5 * a.T @ H @ a + f.T @ a,
        x0=np.zeros(N),
        jac=lambda a: H @ a + f,
        bounds=bounds,
        method='L-BFGS-B'
    )
    
    alpha_opt = res_opt.x
    yellow_pts = cl_obs_vor + alpha_opt[:, None] * normals_left
    return clean_and_resample_spline(yellow_pts, num_output_points=N, sigma=1.2)

def export_slam_yellow_line_svg(filename, track_data, track_idx, slam_res, yellow_line, cl_macro, syn_left, syn_right, cl_obs_vor, cad_left_rot, cad_right_rot, macro_cl_rot):
    name = track_data["name"]
    occ_pts = slam_res["occupied_pts"]
    
    all_pts = np.vstack([occ_pts, cad_left_rot, cad_right_rot, syn_left, syn_right, yellow_line, cl_macro, cl_obs_vor])
    min_x, min_y = np.min(all_pts, axis=0) - 0.4
    max_x, max_y = np.max(all_pts, axis=0) + 0.4
    scale = 100.0
    
    track_w = int((max_x - min_x) * scale)
    track_h = int((max_y - min_y) * scale)
    
    # Side panel settings
    sidebar_w = 420
    svg_w = track_w + sidebar_w
    svg_h = max(track_h, 750)
    
    def tx(x): return (x - min_x) * scale
    def ty(y): return track_h - (y - min_y) * scale + (svg_h - track_h) / 2
    
    svg = []
    svg.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {svg_w} {svg_h}" width="{svg_w}" height="{svg_h}" style="background:#080c14; font-family:-apple-system, BlinkMacSystemFont, Segoe UI, Roboto, sans-serif;">')
    
    # Technical Background Grid on Track Area
    svg.append('<defs>')
    svg.append('  <pattern id="grid_minor" width="20" height="20" patternUnits="userSpaceOnUse">')
    svg.append('    <path d="M 20 0 L 0 0 0 20" fill="none" stroke="#172033" stroke-width="0.75" stroke-opacity="0.4"/>')
    svg.append('  </pattern>')
    svg.append('  <pattern id="grid_major" width="100" height="100" patternUnits="userSpaceOnUse">')
    svg.append('    <rect width="100" height="100" fill="url(#grid_minor)"/>')
    svg.append('    <path d="M 100 0 L 0 0 0 100" fill="none" stroke="#23324d" stroke-width="1.2" stroke-opacity="0.6"/>')
    svg.append('  </pattern>')
    svg.append('</defs>')
    svg.append(f'<rect width="{track_w}" height="{svg_h}" fill="url(#grid_major)" />')
    
    # SLAM Grid Barrier Voxels
    svg.append('<g id="slam_barrier_voxels" opacity="0.80">')
    for pt in occ_pts:
        svg.append(f'<rect x="{tx(pt[0])-1.1:.1f}" y="{ty(pt[1])-1.1:.1f}" width="2.2" height="2.2" fill="#00f0ff" opacity="0.75"/>')
    svg.append('</g>')
    
    # Rotated Obstacles
    svg.append('<g id="obstacles">')
    for obs in slam_res["obstacles"]:
        if obs["type"] == "circle":
            cx, cy, r = tx(obs["cx"]), ty(obs["cy"]), obs["radius"] * scale
            svg.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="#fbbf24" stroke="#ffffff" stroke-width="2.5" opacity="0.95"/>')
        elif obs["type"] == "wall_rectangle":
            poly = obs.get("polygon")
            if poly is not None:
                p_str = " ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in poly])
                svg.append(f'<polygon points="{p_str}" fill="#fbbf24" stroke="#ffffff" stroke-width="2.5" opacity="0.95"/>')
            else:
                cx, cy = tx(obs["center_x"]), ty(obs["center_y"])
                w_px, ext_px = obs["width"] * scale, obs["extrusion"] * scale
                svg.append(f'<rect x="{cx-w_px/2:.1f}" y="{cy-ext_px/2:.1f}" width="{w_px:.1f}" height="{ext_px:.1f}" fill="#fbbf24" stroke="#ffffff" stroke-width="2.5" opacity="0.95" rx="2"/>')
    svg.append('</g>')
    
    # CAD Reference Lines (DOTTED)
    lw_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in cad_left_rot]) + " Z"
    svg.append(f'<path d="{lw_str}" fill="none" stroke="#38bdf8" stroke-width="1.8" stroke-dasharray="4,4" opacity="0.60"/>')
    
    rw_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in cad_right_rot]) + " Z"
    svg.append(f'<path d="{rw_str}" fill="none" stroke="#f43f5e" stroke-width="1.8" stroke-dasharray="4,4" opacity="0.60"/>')
    
    mc_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in macro_cl_rot]) + " Z"
    svg.append(f'<path d="{mc_str}" fill="none" stroke="#ffd700" stroke-width="2.0" stroke-dasharray="3,3" opacity="0.65"/>')
    
    # Reconstructed +/-500mm Vector Walls
    sl_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in syn_left]) + " Z"
    sr_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in syn_right]) + " Z"
    svg.append(f'<path d="{sl_str}" fill="none" stroke="#06b6d4" stroke-width="2.6" opacity="0.85"/>')
    svg.append(f'<path d="{sr_str}" fill="none" stroke="#fb7185" stroke-width="2.6" opacity="0.85"/>')
    
    # Macro Centerline
    mcl_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in cl_macro]) + " Z"
    svg.append(f'<path d="{mcl_str}" fill="none" stroke="#c084fc" stroke-width="2.5" opacity="0.75"/>')
    
    # Obstacle-Aware Voronoi Seed
    ov_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in cl_obs_vor]) + " Z"
    svg.append(f'<path d="{ov_str}" fill="none" stroke="#38bdf8" stroke-width="2.2" stroke-dasharray="4,3" opacity="0.70"/>')
    
    # Optimal Yellow Racing Line
    yl_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in yellow_line]) + " Z"
    svg.append(f'<path d="{yl_str}" fill="none" stroke="#facc15" stroke-width="4.2" stroke-linecap="round" stroke-linejoin="round" opacity="0.98"/>')
    
    # =========================================================================
    # DEDICATED SIDEBAR PANEL (Placed completely to the right)
    # =========================================================================
    sb_x = track_w
    svg.append(f'<rect x="{sb_x}" y="0" width="{sidebar_w}" height="{svg_h}" fill="#0f172a" stroke="#1e293b" stroke-width="1.5" />')
    
    # Header inside sidebar
    svg.append(f'<text x="{sb_x + 24}" y="48" fill="#f8fafc" font-size="20" font-weight="700">{name}</text>')
    svg.append(f'<text x="{sb_x + 24}" y="74" fill="#94a3b8" font-size="12">45° Webots SLAM Yellow Racing Line</text>')
    svg.append(f'<line x1="{sb_x + 24}" y1="92" x2="{sb_x + sidebar_w - 24}" y2="92" stroke="#334155" stroke-width="1.2"/>')
    
    # Legend Entries
    cur_y = 125
    
    # 1. Yellow Racing Line
    svg.append(f'<line x1="{sb_x+24}" y1="{cur_y}" x2="{sb_x+64}" y2="{cur_y}" stroke="#facc15" stroke-width="4.5"/>')
    svg.append(f'<text x="{sb_x+76}" y="{cur_y+5}" fill="#facc15" font-size="13" font-weight="700">Yellow Racing Line (TUM QP)</text>')
    svg.append(f'<text x="{sb_x+76}" y="{cur_y+20}" fill="#94a3b8" font-size="11">Minimum-curvature apex path</text>')
    cur_y += 50
    
    # 2. Alpha-Shape Centerline
    svg.append(f'<line x1="{sb_x+24}" y1="{cur_y}" x2="{sb_x+64}" y2="{cur_y}" stroke="#c084fc" stroke-width="3"/>')
    svg.append(f'<text x="{sb_x+76}" y="{cur_y+5}" fill="#c084fc" font-size="13" font-weight="600">Alpha-Shape Macro Centerline</text>')
    svg.append(f'<text x="{sb_x+76}" y="{cur_y+20}" fill="#94a3b8" font-size="11">Ignores wall boxes &amp; pillars</text>')
    cur_y += 50
    
    # 3. Reconstructed 500mm Walls
    svg.append(f'<line x1="{sb_x+24}" y1="{cur_y-3}" x2="{sb_x+64}" y2="{cur_y-3}" stroke="#06b6d4" stroke-width="2.6"/>')
    svg.append(f'<line x1="{sb_x+24}" y1="{cur_y+5}" x2="{sb_x+64}" y2="{cur_y+5}" stroke="#fb7185" stroke-width="2.6"/>')
    svg.append(f'<text x="{sb_x+76}" y="{cur_y+3}" fill="#f1f5f9" font-size="13" font-weight="600">Reconstructed ±500mm Walls</text>')
    svg.append(f'<text x="{sb_x+76}" y="{cur_y+18}" fill="#94a3b8" font-size="11">Normal offsets from Alpha-Shape</text>')
    cur_y += 50
    
    # 4. Obstacle Voronoi Seed
    svg.append(f'<line x1="{sb_x+24}" y1="{cur_y}" x2="{sb_x+64}" y2="{cur_y}" stroke="#38bdf8" stroke-width="2.2" stroke-dasharray="4,3"/>')
    svg.append(f'<text x="{sb_x+76}" y="{cur_y+5}" fill="#38bdf8" font-size="13" font-weight="600">Obstacle-Aware Voronoi Seed</text>')
    svg.append(f'<text x="{sb_x+76}" y="{cur_y+20}" fill="#94a3b8" font-size="11">Routes through chokes/pillars</text>')
    cur_y += 50
    
    # 5. CAD Centerline Dotted
    svg.append(f'<line x1="{sb_x+24}" y1="{cur_y}" x2="{sb_x+64}" y2="{cur_y}" stroke="#ffd700" stroke-width="2" stroke-dasharray="3,3"/>')
    svg.append(f'<text x="{sb_x+76}" y="{cur_y+5}" fill="#ffd700" font-size="13">CAD Ground Truth Centerline</text>')
    svg.append(f'<text x="{sb_x+76}" y="{cur_y+20}" fill="#94a3b8" font-size="11">Dotted reference benchmark</text>')
    cur_y += 50
    
    # 6. CAD Walls Dotted
    svg.append(f'<line x1="{sb_x+24}" y1="{cur_y-3}" x2="{sb_x+64}" y2="{cur_y-3}" stroke="#38bdf8" stroke-width="1.8" stroke-dasharray="4,4"/>')
    svg.append(f'<line x1="{sb_x+24}" y1="{cur_y+5}" x2="{sb_x+64}" y2="{cur_y+5}" stroke="#f43f5e" stroke-width="1.8" stroke-dasharray="4,4"/>')
    svg.append(f'<text x="{sb_x+76}" y="{cur_y+3}" fill="#94a3b8" font-size="13">CAD Wall Boundaries (Dotted)</text>')
    svg.append(f'<text x="{sb_x+76}" y="{cur_y+18}" fill="#64748b" font-size="11">Theoretical left/right walls</text>')
    cur_y += 50
    
    # 7. SLAM Voxels
    svg.append(f'<rect x="{sb_x+24}" y="{cur_y-6}" width="40" height="12" fill="#00f0ff" opacity="0.80"/>')
    svg.append(f'<text x="{sb_x+76}" y="{cur_y+5}" fill="#00f0ff" font-size="12">Cartographer 2cm Barrier Voxels</text>')
    cur_y += 40
    
    # 8. Obstacles
    svg.append(f'<circle cx="{sb_x+44}" cy="{cur_y}" r="8" fill="#fbbf24" stroke="#ffffff" stroke-width="2"/>')
    svg.append(f'<text x="{sb_x+76}" y="{cur_y+4}" fill="#fbbf24" font-size="12">Obstacle Pillars &amp; Wall Boxes</text>')
    
    svg.append('</svg>')
    
    with open(filename, "w", encoding="utf-8") as f:
        f.write("\n".join(svg))

def main():
    tasks = [
        (get_track_3_slalom_pillars, 3, "Track 3: Grand Prix Arena"),
        (get_track_4_switchback_arena, 4, "Track 4: Technical Switchback Arena")
    ]
    
    for getter, idx, name in tasks:
        track = getter()
        rot_rad = np.radians(45.0)
        raw_left = np.array(track["left_wall"])
        anchor = np.mean(raw_left, axis=0)
        
        temp_traj_path = os.path.join(BASE_DIR, "webots_simulation", f"temp_yellow_t{idx}_traj.csv")
        np.savetxt(temp_traj_path, np.column_stack([track["centerline"], np.zeros(len(track["centerline"])), np.zeros(len(track["centerline"]))]), delimiter=",", header="x,y,z,yaw", comments="")
        slam_res = simulate_cartographer_slam(track, temp_traj_path, angle_deg=45.0, resolution=0.02)
        
        drivable, outer_c, inner_c = get_raw_corridor_contours(slam_res["occupancy_grid"], 0.02)
        cl_macro, _, _ = method_3_rolling_disc_alpha_shape(slam_res, outer_c, inner_c, num_points=400)
        syn_left, syn_right = compute_500mm_reconstructed_walls(cl_macro)
        cl_obs_vor = generate_obstacle_aware_voronoi_centerline(slam_res, min_clearance=0.14)
        yellow_line = optimize_slam_yellow_line(cl_obs_vor, syn_left, syn_right, slam_res, margin=0.10)
        
        cad_left_rot = rotate_points_2d(raw_left, rot_rad, anchor)
        cad_right_rot = rotate_points_2d(np.array(track["right_wall"]), rot_rad, anchor)
        macro_cl_rot = rotate_points_2d(track["centerline"], rot_rad, anchor)
        
        svg_name = f"track{idx:02d}_45deg_yellow_racing_line_alpha_hull.svg"
        svg_path = os.path.join(OUT_SLAM_DIR, svg_name)
        export_slam_yellow_line_svg(svg_path, track, idx, slam_res, yellow_line, cl_macro, syn_left, syn_right, cl_obs_vor, cad_left_rot, cad_right_rot, macro_cl_rot)
        shutil.copy2(svg_path, os.path.join(ART_DIR, svg_name))
        print(f"  [OK] Exported Track {idx} with Sidebar Layout -> {svg_name}")

if __name__ == "__main__":
    main()
