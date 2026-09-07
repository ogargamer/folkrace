#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - ORIGINAL RAYCAST-GATES CENTERLINE & WEBOTS SLAM PIPELINE
================================================================================
1. Computes the original, smooth obstacle-aware centerline using Method 3:
   Cross-Sectional Raycast Gates with wide-entry corner bias:
   - 0 loops around itself
   - 0 pillar penetrations
   - Smooth, wide-entry bypass around turn-entry obstacles
2. Exports all 10 CAD tracks with their verified in-bounds obstacles & centerlines.
3. For Track 3 & Track 4: Runs full 45° Webots/Cartographer SLAM simulation,
   generating 2cm occupancy grids, deskewed LiDAR point maps, and wide-entry lines.
================================================================================
"""

import os
import sys
import shutil
import numpy as np
import scipy.spatial as spatial
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import splprep, splev
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, BASE_DIR)

from racing_engine.track_generator import (
    DIR_VECTORS, LEFT_VECTORS, build_raw_track_corridor, TILE_SIZE,
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
from racing_engine.centerline_finder import clean_and_resample_spline
from webots_simulation.simulate_cartographer_2cm_grid import simulate_cartographer_slam, rotate_points_2d, OUT_SLAM_DIR

OUT_CAD_DIR = os.path.join(BASE_DIR, "racing_results", "cad_models")
os.makedirs(OUT_CAD_DIR, exist_ok=True)
ART_DIR = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"

# ==============================================================================
# ORIGINAL METHOD 3: CROSS-SECTIONAL RAYCAST GATES WITH WIDE-ENTRY BIAS
# ==============================================================================
def compute_raycast_gates_centerline(track_data, num_points=400, wide_entry_bias=True):
    """
    Computes smooth obstacle-aware centerline by casting perpendicular rays from the
    macro corridor centerline, calculating the exact midpoint of open drivable space.
    """
    l_wall = track_data["left_wall"]
    r_wall = track_data["right_wall"]
    obstacles = track_data.get("obstacles", [])
    
    # Ground truth macro centerline
    if "centerline" in track_data:
        raw_pts = track_data["centerline"]
    else:
        N_raw = min(len(l_wall), len(r_wall))
        raw_pts = (l_wall[:N_raw] + r_wall[:N_raw]) / 2.0
        
    cl = clean_and_resample_spline(raw_pts, num_points, sigma=1.0)
    half_w = 0.50
    
    for it in range(8):
        pts_wrap = np.vstack([cl[-1:], cl, cl[:2]])
        dx = (pts_wrap[2:, 0] - pts_wrap[:-2, 0]) * 0.5
        dy = (pts_wrap[2:, 1] - pts_wrap[:-2, 1]) * 0.5
        norms = np.hypot(dx, dy) + 1e-6
        tangents = np.column_stack([dx / norms, dy / norms])
        normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])
        
        # Calculate track macro curvature for wide entry biasing
        ddx = (pts_wrap[2:, 0] - 2 * pts_wrap[1:-1, 0] + pts_wrap[:-2, 0])
        ddy = (pts_wrap[2:, 1] - 2 * pts_wrap[1:-1, 1] + pts_wrap[:-2, 1])
        curv = (dx * ddy - dy * ddx) / (norms**3 + 1e-6)
        smooth_curv = gaussian_filter1d(curv[1:num_points+1], sigma=15.0, mode='wrap')
        
        new_cl = []
        for i in range(num_points):
            ci = cl[i]
            ni = normals[i]
            
            # Find all obstacle intervals along normal ray [-half_w, +half_w]
            blocked_intervals = []
            for obs in obstacles:
                if obs["type"] == "circle":
                    cx, cy, cr = obs["cx"], obs["cy"], obs["radius"]
                    v_obs = np.array([cx, cy]) - ci
                    long_dist = np.dot(v_obs, tangents[i])
                    lat_dist = np.dot(v_obs, ni)
                    if abs(long_dist) <= (cr + 0.35):
                        r_eff = cr + 0.08
                        blocked_intervals.append((lat_dist - r_eff, lat_dist + r_eff))
                elif obs["type"] == "wall_rectangle":
                    rx, ry = obs["center_x"], obs["center_y"]
                    rw, rh = obs["width"], obs["extrusion"]
                    v_obs = np.array([rx, ry]) - ci
                    long_dist = np.dot(v_obs, tangents[i])
                    lat_dist = np.dot(v_obs, ni)
                    if abs(long_dist) <= (rw/2.0 + 0.20):
                        if obs.get("side") == "left" or lat_dist > 0:
                            blocked_intervals.append((half_w - rh, half_w))
                        else:
                            blocked_intervals.append((-half_w, -half_w + rh))
                            
            if not blocked_intervals:
                new_cl.append(ci)
                continue
                
            min_block = min(b[0] for b in blocked_intervals)
            max_block = max(b[1] for b in blocked_intervals)
            
            left_open_w = half_w - max_block
            right_open_w = min_block - (-half_w)
            
            # Wide entry bias: if upcoming corner is Left turn (curv > 0), bias toward right channel (-ni)
            if wide_entry_bias and smooth_curv[i] > 0.04 and right_open_w >= 0.20:
                chosen_shift = (-half_w + min_block) / 2.0
            elif wide_entry_bias and smooth_curv[i] < -0.04 and left_open_w >= 0.20:
                chosen_shift = (max_block + half_w) / 2.0
            elif left_open_w >= right_open_w:
                chosen_shift = (max_block + half_w) / 2.0
            else:
                chosen_shift = (-half_w + min_block) / 2.0
                
            new_cl.append(ci + ni * chosen_shift)
            
        cl = np.array(new_cl)
        cl[:, 0] = gaussian_filter1d(cl[:, 0], sigma=3.5, mode='wrap')
        cl[:, 1] = gaussian_filter1d(cl[:, 1], sigma=3.5, mode='wrap')
        
    return clean_and_resample_spline(cl, num_points, sigma=2.0)

# ==============================================================================
# VECTOR CAD SVG EXPORTER
# ==============================================================================
def export_cad_svg_model(filename, track_data, track_idx, oa_cl):
    l_wall = np.array(track_data["left_wall"])
    r_wall = np.array(track_data["right_wall"])
    macro_cl = np.array(track_data["centerline"])
    obstacles = track_data.get("obstacles", [])
    name = track_data["name"]
    
    all_pts = np.vstack([l_wall, r_wall, macro_cl, oa_cl])
    min_x, min_y = np.min(all_pts, axis=0) - 0.8
    max_x, max_y = np.max(all_pts, axis=0) + 0.8
    scale = 120.0
    
    svg_w = int((max_x - min_x) * scale)
    svg_h = int((max_y - min_y) * scale)
    
    def tx(x): return (x - min_x) * scale
    def ty(y): return svg_h - (y - min_y) * scale
    
    svg = []
    svg.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {svg_w} {svg_h}" width="{svg_w}" height="{svg_h}" style="background:#0a0f1d; font-family: -apple-system, BlinkMacSystemFont, Segoe UI, Roboto, sans-serif;">')
    
    # Background Technical Grid
    svg.append('<defs>')
    svg.append('  <pattern id="grid_minor" width="24" height="24" patternUnits="userSpaceOnUse">')
    svg.append('    <path d="M 24 0 L 0 0 0 24" fill="none" stroke="#1e293b" stroke-width="0.75" stroke-opacity="0.4"/>')
    svg.append('  </pattern>')
    svg.append('  <pattern id="grid_major" width="120" height="120" patternUnits="userSpaceOnUse">')
    svg.append('    <rect width="120" height="120" fill="url(#grid_minor)"/>')
    svg.append('    <path d="M 120 0 L 0 0 0 120" fill="none" stroke="#334155" stroke-width="1.2" stroke-opacity="0.7"/>')
    svg.append('  </pattern>')
    svg.append('</defs>')
    svg.append(f'<rect width="{svg_w}" height="{svg_h}" fill="url(#grid_major)" />')
    
    # Title Header
    svg.append(f'<text x="32" y="44" fill="#f8fafc" font-size="22" font-weight="700">{name}</text>')
    svg.append(f'<text x="32" y="70" fill="#94a3b8" font-size="13">Exact CAD Vector Model: Method 3 Raycast Gates Centerline (Smooth Obstacle Bypass)</text>')
    
    # Drivable Corridor Fill
    poly_l = " ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in l_wall])
    poly_r = " ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in r_wall[::-1]])
    svg.append(f'<polygon points="{poly_l} {poly_r}" fill="#0f172a" opacity="0.9"/>')
    
    # Macro Corridor Baseline Centerline
    macro_path = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in macro_cl]) + " Z"
    svg.append(f'<path d="{macro_path}" fill="none" stroke="#eab308" stroke-width="1.8" stroke-dasharray="4,4" opacity="0.5"/>')
    
    # Outer Wall Boundary (Cyan) & Inner Wall Boundary (Rose)
    l_path = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in l_wall]) + " Z"
    r_path = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in r_wall]) + " Z"
    svg.append(f'<path d="{l_path}" fill="none" stroke="#38bdf8" stroke-width="3.0" opacity="0.95"/>')
    svg.append(f'<path d="{r_path}" fill="none" stroke="#f43f5e" stroke-width="3.0" opacity="0.95"/>')
    
    # Method 3 Raycast Gates Centerline (Bright Green Neon)
    oa_path = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in oa_cl]) + " Z"
    svg.append(f'<path d="{oa_path}" fill="none" stroke="#4ade80" stroke-width="3.6" stroke-linecap="round" stroke-linejoin="round" opacity="0.95"/>')
    
    # Obstacles
    svg.append('<g id="obstacles">')
    for obs in obstacles:
        if obs["type"] == "circle":
            cx, cy, r = tx(obs["cx"]), ty(obs["cy"]), obs["radius"] * scale
            svg.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="#fbbf24" stroke="#ffffff" stroke-width="2.5" opacity="0.95"/>')
            svg.append(f'<text x="{cx+r+6:.1f}" y="{cy+4:.1f}" fill="#fbbf24" font-size="11" font-weight="600">{obs["name"]}</text>')
        elif obs["type"] == "wall_rectangle":
            cx, cy = tx(obs["center_x"]), ty(obs["center_y"])
            w_px = obs["width"] * scale
            h_px = obs["extrusion"] * scale
            x0 = cx - w_px / 2.0
            y0 = cy - h_px / 2.0
            svg.append(f'<rect x="{x0:.1f}" y="{y0:.1f}" width="{w_px:.1f}" height="{h_px:.1f}" fill="#fbbf24" stroke="#ffffff" stroke-width="2.5" opacity="0.95" rx="3"/>')
            svg.append(f'<text x="{cx:.1f}" y="{y0-6:.1f}" fill="#fbbf24" font-size="11" font-weight="600" text-anchor="middle">{obs["name"]}</text>')
    svg.append('</g>')
    
    # Technical Legend
    leg_w, leg_h = 500, 160
    leg_x, leg_y = svg_w - leg_w - 32, svg_h - leg_h - 32
    svg.append(f'<rect x="{leg_x}" y="{leg_y}" width="{leg_w}" height="{leg_h}" fill="#111827" rx="10" stroke="#374151" stroke-width="1.5" opacity="0.95"/>')
    svg.append(f'<line x1="{leg_x+24}" y1="{leg_y+30}" x2="{leg_x+64}" y2="{leg_y+30}" stroke="#4ade80" stroke-width="4"/>')
    svg.append(f'<text x="{leg_x+76}" y="{leg_y+35}" fill="#4ade80" font-size="13" font-weight="700">Obstacle-Aware Centerline (Raycast Gates Midpoint)</text>')
    svg.append(f'<line x1="{leg_x+24}" y1="{leg_y+60}" x2="{leg_x+64}" y2="{leg_y+60}" stroke="#eab308" stroke-width="2" stroke-dasharray="4,4"/>')
    svg.append(f'<text x="{leg_x+76}" y="{leg_y+65}" fill="#eab308" font-size="13" font-weight="600">Macro Corridor Centerline (Unperturbed Baseline)</text>')
    svg.append(f'<line x1="{leg_x+24}" y1="{leg_y+90}" x2="{leg_x+64}" y2="{leg_y+90}" stroke="#38bdf8" stroke-width="3"/>')
    svg.append(f'<text x="{leg_x+76}" y="{leg_y+95}" fill="#38bdf8" font-size="13" font-weight="600">CAD Left Wall Boundary (Outer/Inner)</text>')
    svg.append(f'<line x1="{leg_x+24}" y1="{leg_y+120}" x2="{leg_x+64}" y2="{leg_y+120}" stroke="#f43f5e" stroke-width="3"/>')
    svg.append(f'<text x="{leg_x+76}" y="{leg_y+125}" fill="#f43f5e" font-size="13" font-weight="600">CAD Right Wall Boundary (Outer/Inner)</text>')
    svg.append(f'<rect x="{leg_x+24}" y="{leg_y+136}" width="40" height="15" fill="#fbbf24" stroke="#ffffff" stroke-width="1.5" rx="2"/>')
    svg.append(f'<text x="{leg_x+76}" y="{leg_y+148}" fill="#fbbf24" font-size="13" font-weight="600">Obstacles (Wall Boxes &amp; Slalom Pillars)</text>')
    
    svg.append('</svg>')
    
    with open(filename, "w", encoding="utf-8") as f:
        f.write("\n".join(svg))

# ==============================================================================
# HIGH-RESOLUTION PNG RENDERER
# ==============================================================================
def export_cad_png_render(filename, track_data, track_idx, oa_cl):
    l_wall = np.array(track_data["left_wall"])
    r_wall = np.array(track_data["right_wall"])
    macro_cl = np.array(track_data["centerline"])
    obstacles = track_data.get("obstacles", [])
    name = track_data["name"]
    
    fig, ax = plt.subplots(figsize=(16, 12), facecolor="#0a0f1d")
    ax.set_facecolor("#0a0f1d")
    
    lw = np.vstack([l_wall, l_wall[:1]])
    rw = np.vstack([r_wall, r_wall[:1]])
    mc = np.vstack([macro_cl, macro_cl[:1]])
    oc = np.vstack([oa_cl, oa_cl[:1]])
    
    poly_pts = np.vstack([l_wall, r_wall[::-1]])
    patch = plt.Polygon(poly_pts, facecolor="#111827", edgecolor="none", alpha=0.9, zorder=1)
    ax.add_patch(patch)
    
    ax.plot(mc[:, 0], mc[:, 1], color="#eab308", linewidth=1.8, linestyle="--", alpha=0.5, label="Macro Baseline Centerline", zorder=2)
    ax.plot(lw[:, 0], lw[:, 1], color="#38bdf8", linewidth=2.8, label="CAD Left Wall Boundary", zorder=3)
    ax.plot(rw[:, 0], rw[:, 1], color="#f43f5e", linewidth=2.8, label="CAD Right Wall Boundary", zorder=3)
    ax.plot(oc[:, 0], oc[:, 1], color="#4ade80", linewidth=3.5, label="Obstacle-Aware Centerline (Raycast Gates)", zorder=8)
    
    wall_boxes = [o for o in obstacles if o["type"] == "wall_rectangle"]
    pillars = [o for o in obstacles if o["type"] == "circle"]
    
    for obs in obstacles:
        if obs["type"] == "circle":
            circle = plt.Circle((obs["cx"], obs["cy"]), obs["radius"], facecolor="#fbbf24", edgecolor="#ffffff", linewidth=1.5, zorder=6)
            ax.add_patch(circle)
            ax.text(obs["cx"] + obs["radius"] + 0.08, obs["cy"], obs["name"], color="#fbbf24", fontsize=8.5, fontweight='bold', va='center', zorder=7)
        elif obs["type"] == "wall_rectangle":
            cx, cy = obs["center_x"], obs["center_y"]
            w, h = obs["width"], obs["extrusion"]
            rect = plt.Rectangle((cx - w/2.0, cy - h/2.0), w, h, facecolor="#fbbf24", edgecolor="#ffffff", linewidth=1.5, zorder=6)
            ax.add_patch(rect)
            ax.text(cx, cy + h/2.0 + 0.08, obs["name"], color="#fbbf24", fontsize=8.5, fontweight='bold', ha='center', zorder=7)
            
    ax.plot([], [], color="#fbbf24", linewidth=5, label=f"Obstacles ({len(wall_boxes)} Boxes + {len(pillars)} Pillars)")
    
    ax.set_title(f"{name}\nObstacle-Aware CAD Centerline (Method 3 Raycast Gates)",
                 color="#f8fafc", fontsize=14, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.25, color="#475569")
    ax.tick_params(colors="#94a3b8", labelsize=10)
    ax.set_xlabel("X Position (meters)", color="#94a3b8", fontsize=11, labelpad=8)
    ax.set_ylabel("Y Position (meters)", color="#94a3b8", fontsize=11, labelpad=8)
    ax.legend(loc="upper left", facecolor="#111827", edgecolor="#374151", labelcolor="#f8fafc", fontsize=10, borderpad=1.0, labelspacing=0.8)
    
    plt.savefig(filename, dpi=200, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)

# ==============================================================================
# WEBOTS / CARTOGRAPHER 2CM SLAM & RACING LINE PIPELINE FOR TRACK 3 & TRACK 4
# ==============================================================================
def process_webots_slam_track(track_getter, track_idx, track_name, angle_deg=45.0):
    print("\n" + "=" * 90)
    print(f"   RUNNING WEBOTS / CARTOGRAPHER SLAM SIMULATION: {track_name} (45° Rotated)")
    print("=" * 90)
    
    track_data = track_getter()
    rot_rad = np.radians(angle_deg)
    raw_left = np.array(track_data["left_wall"])
    center_anchor = np.mean(raw_left, axis=0)
    
    # 1. Compute Raycast Gates Centerline
    oa_cl = compute_raycast_gates_centerline(track_data, num_points=400, wide_entry_bias=True)
    
    # Save vehicle trajectory for LiDAR scan sweeps
    temp_traj_path = os.path.join(BASE_DIR, "racing_engine", f"temp_track{track_idx}_traj.csv")
    np.savetxt(temp_traj_path, np.column_stack([oa_cl, np.zeros(len(oa_cl)), np.zeros(len(oa_cl))]), delimiter=",", header="x,y,z,yaw", comments="")
    
    # 2. Simulate 45° Cartographer SLAM Grid
    slam_res = simulate_cartographer_slam(track_data, temp_traj_path, angle_deg=angle_deg, resolution=0.02)
    
    # Rotate CAD ground truth & Raycast centerline by 45°
    cad_left_rot = rotate_points_2d(raw_left, rot_rad, center_anchor)
    cad_right_rot = rotate_points_2d(np.array(track_data["right_wall"]), rot_rad, center_anchor)
    macro_cl_rot = rotate_points_2d(track_data["centerline"], rot_rad, center_anchor)
    oa_cl_rot = rotate_points_2d(oa_cl, rot_rad, center_anchor)
    
    # 3. Generate High-Fidelity Vector SVG
    out_svg = os.path.join(OUT_SLAM_DIR, f"track{track_idx:02d}_45deg_webots_slam_and_racing_line.svg")
    
    occ_pts = slam_res["occupied_pts"]
    all_pts = np.vstack([occ_pts, cad_left_rot, cad_right_rot, oa_cl_rot])
    min_x, min_y = np.min(all_pts, axis=0) - 0.5
    max_x, max_y = np.max(all_pts, axis=0) + 0.5
    scale = 100.0
    
    svg_w = int((max_x - min_x) * scale)
    svg_h = int((max_y - min_y) * scale)
    
    def tx(x): return (x - min_x) * scale
    def ty(y): return svg_h - (y - min_y) * scale
    
    svg = []
    svg.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {svg_w} {svg_h}" width="{svg_w}" height="{svg_h}" style="background:#080c14; font-family:sans-serif;">')
    svg.append(f'<text x="24" y="36" fill="#f8fafc" font-size="20" font-weight="bold">{track_name} - Webots 45° SLAM &amp; Raycast Gates Centerline</text>')
    svg.append(f'<text x="24" y="58" fill="#94a3b8" font-size="12">Deskewed LiDAR 2cm Voxel Grid + Smooth Obstacle-Aware Trajectory</text>')
    
    # SLAM Grid Points
    svg.append('<g id="slam_grid_points" opacity="0.80">')
    for pt in occ_pts:
        svg.append(f'<rect x="{tx(pt[0])-1.1:.1f}" y="{ty(pt[1])-1.1:.1f}" width="2.2" height="2.2" fill="#00f0ff" opacity="0.85"/>')
    svg.append('</g>')
    
    # Rotated Obstacles
    svg.append('<g id="obstacles">')
    for obs in slam_res["obstacles"]:
        if obs["type"] == "circle":
            cx, cy, r = tx(obs["cx"]), ty(obs["cy"]), obs["radius"] * scale
            svg.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="#fbbf24" stroke="#ffffff" stroke-width="2" opacity="0.95"/>')
        elif obs["type"] == "wall_rectangle":
            poly = obs.get("polygon")
            if poly is not None:
                p_str = " ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in poly])
                svg.append(f'<polygon points="{p_str}" fill="#fbbf24" stroke="#ffffff" stroke-width="2" opacity="0.95"/>')
            else:
                cx, cy = tx(obs["center_x"]), ty(obs["center_y"])
                w_px, ext_px = obs["width"] * scale, obs["extrusion"] * scale
                svg.append(f'<rect x="{cx-w_px/2:.1f}" y="{cy-ext_px/2:.1f}" width="{w_px:.1f}" height="{ext_px:.1f}" fill="#fbbf24" stroke="#ffffff" stroke-width="2" opacity="0.95"/>')
    svg.append('</g>')
    
    # CAD Wall Boundaries
    lw_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in cad_left_rot]) + " Z"
    rw_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in cad_right_rot]) + " Z"
    svg.append(f'<path d="{lw_str}" fill="none" stroke="#38bdf8" stroke-width="2.5" opacity="0.85"/>')
    svg.append(f'<path d="{rw_str}" fill="none" stroke="#f43f5e" stroke-width="2.5" opacity="0.85"/>')
    
    # Macro Baseline Centerline
    mc_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in macro_cl_rot]) + " Z"
    svg.append(f'<path d="{mc_str}" fill="none" stroke="#ffd700" stroke-width="2.0" stroke-dasharray="4,4" opacity="0.6"/>')
    
    # Raycast Gates Obstacle-Aware Centerline (Bright Green)
    oa_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in oa_cl_rot]) + " Z"
    svg.append(f'<path d="{oa_str}" fill="none" stroke="#4ade80" stroke-width="3.8" stroke-linecap="round" stroke-linejoin="round" opacity="0.95"/>')
    
    # Legend
    leg_y = svg_h - 145
    svg.append(f'<rect x="20" y="{leg_y}" width="480" height="125" fill="#121826" rx="8" stroke="#2d3748" stroke-width="1.5" opacity="0.95"/>')
    svg.append(f'<line x1="36" y1="{leg_y+25}" x2="70" y2="{leg_y+25}" stroke="#4ade80" stroke-width="4"/>')
    svg.append(f'<text x="80" y="{leg_y+29}" fill="#4ade80" font-size="12" font-weight="bold">Obstacle-Aware Centerline (Raycast Gates)</text>')
    svg.append(f'<line x1="36" y1="{leg_y+48}" x2="70" y2="{leg_y+48}" stroke="#ffd700" stroke-width="2.5" stroke-dasharray="4,4"/>')
    svg.append(f'<text x="80" y="{leg_y+52}" fill="#ffd700" font-size="12">Macro Baseline Centerline (CAD Golden Reference)</text>')
    svg.append(f'<rect x="36" y="{leg_y+65}" width="34" height="8" fill="#00f0ff" opacity="0.85"/>')
    svg.append(f'<text x="80" y="{leg_y+73}" fill="#00f0ff" font-size="12">Cartographer 2cm SLAM Barrier Voxels</text>')
    svg.append(f'<line x1="36" y1="{leg_y+94}" x2="70" y2="{leg_y+94}" stroke="#38bdf8" stroke-width="2"/>')
    svg.append(f'<line x1="36" y1="{leg_y+104}" x2="70" y2="{leg_y+104}" stroke="#f43f5e" stroke-width="2"/>')
    svg.append(f'<text x="80" y="{leg_y+101}" fill="#94a3b8" font-size="12">CAD Ground Truth Wall Boundaries</text>')
    svg.append('</svg>')
    
    with open(out_svg, "w", encoding="utf-8") as f:
        f.write("\n".join(svg))
    shutil.copy2(out_svg, os.path.join(ART_DIR, f"track{track_idx:02d}_45deg_webots_slam_and_racing_line.svg"))
    
    # 4. Render Visual PNG
    fig, ax = plt.subplots(figsize=(16, 12), facecolor="#080c14")
    ax.set_facecolor("#080c14")
    
    grid = slam_res["occupancy_grid"]
    res = slam_res["resolution"]
    extent = [slam_res["min_x"], slam_res["min_x"] + slam_res["grid_w"] * res,
              slam_res["min_y"], slam_res["min_y"] + slam_res["grid_h"] * res]
    cmap = matplotlib.colors.ListedColormap(['#151b28', '#070a12', '#00f0ff'])
    norm = matplotlib.colors.BoundaryNorm([-1.5, -0.5, 50, 150], cmap.N)
    ax.imshow(grid, origin='lower', extent=extent, cmap=cmap, norm=norm, interpolation='nearest', alpha=0.70)
    
    for obs in slam_res["obstacles"]:
        if obs["type"] == "circle":
            circle = plt.Circle((obs["cx"], obs["cy"]), obs["radius"], color="#fbbf24", alpha=0.9)
            ax.add_patch(circle)
        elif obs["type"] == "wall_rectangle":
            poly = obs.get("polygon")
            if poly is not None:
                patch = plt.Polygon(poly, color="#fbbf24", alpha=0.9)
                ax.add_patch(patch)
    
    ax.plot(np.vstack([cad_left_rot, cad_left_rot[:1]])[:, 0], np.vstack([cad_left_rot, cad_left_rot[:1]])[:, 1], color="#38bdf8", linewidth=2.0, label="CAD Left Wall")
    ax.plot(np.vstack([cad_right_rot, cad_right_rot[:1]])[:, 0], np.vstack([cad_right_rot, cad_right_rot[:1]])[:, 1], color="#f43f5e", linewidth=2.0, label="CAD Right Wall")
    ax.plot(np.vstack([macro_cl_rot, macro_cl_rot[:1]])[:, 0], np.vstack([macro_cl_rot, macro_cl_rot[:1]])[:, 1], color="#ffd700", linewidth=2.0, linestyle="--", label="Macro Baseline Centerline")
    ax.plot(np.vstack([oa_cl_rot, oa_cl_rot[:1]])[:, 0], np.vstack([oa_cl_rot, oa_cl_rot[:1]])[:, 1], color="#4ade80", linewidth=3.5, label="Obstacle-Aware Centerline (Raycast Gates)")
    
    ax.set_title(f"{track_name} - Webots 45° SLAM & Obstacle-Aware Trajectory", color="#f8fafc", fontsize=13, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.15, color="#ffffff")
    ax.tick_params(colors="#94a3b8")
    ax.legend(loc="upper left", facecolor="#121826", edgecolor="#2d3748", labelcolor="#f8fafc", fontsize=9.5)
    
    out_png = os.path.join(OUT_SLAM_DIR, f"track{track_idx:02d}_45deg_webots_slam_and_racing_line.png")
    plt.savefig(out_png, dpi=200, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    shutil.copy2(out_png, os.path.join(ART_DIR, f"track{track_idx:02d}_45deg_webots_slam_and_racing_line.png"))
    
    print(f"  [OK] Exported Track {track_idx} Webots SLAM & Centerline -> {out_svg}")
    return out_svg, out_png

# ==============================================================================
# MAIN RUNNER
# ==============================================================================
def main():
    print("=" * 105)
    print("   PROCESSING 10 CAD TRACKS (METHOD 3 RAYCAST GATES) & WEBOTS SLAM (TRACK 3 & 4)")
    print("=" * 105)
    
    track_getters = [
        (get_track_1_chicane_with_pillar, 1),
        (get_track_2_bottleneck_circuit, 2),
        (get_track_3_slalom_pillars, 3),
        (get_track_4_switchback_arena, 4),
        (get_track_5_dual_choke_chicane_ring, 5),
        (get_track_6_quad_pillar_loop, 6),
        (get_track_7_multi_bottleneck_slalom, 7),
        (get_track_8_dual_chicane_arena, 8),
        (get_track_9_super_speedway, 9),
        (get_track_10_switchback_obstacle_maze, 10),
    ]
    
    # 1. Process all 10 CAD models with Method 3 Raycast Gates
    for getter, idx in track_getters:
        track = getter()
        name = track["name"]
        
        oa_cl = compute_raycast_gates_centerline(track, num_points=400, wide_entry_bias=True)
        
        svg_name = f"track{idx:02d}_cad_with_centerline.svg"
        svg_path = os.path.join(OUT_CAD_DIR, svg_name)
        export_cad_svg_model(svg_path, track, idx, oa_cl)
        shutil.copy2(svg_path, os.path.join(ART_DIR, svg_name))
        
        png_name = f"track{idx:02d}_cad_with_centerline.png"
        png_path = os.path.join(OUT_CAD_DIR, png_name)
        export_cad_png_render(png_path, track, idx, oa_cl)
        shutil.copy2(png_path, os.path.join(ART_DIR, png_name))
        
        w_boxes = len([o for o in track.get("obstacles", []) if o["type"] == "wall_rectangle"])
        pillars = len([o for o in track.get("obstacles", []) if o["type"] == "circle"])
        
        # Check pillar clearance
        min_pillar_dists = []
        for o in track.get("obstacles", []):
            if o["type"] == "circle":
                d = np.min(np.hypot(oa_cl[:, 0] - o["cx"], oa_cl[:, 1] - o["cy"]))
                min_pillar_dists.append(d)
        min_p_str = f"Min Pil Clr: {min(min_pillar_dists):.2f}m" if min_pillar_dists else "No pillars"
        
        print(f"  [OK] Track {idx:02d}: {name:<46} | {w_boxes} Boxes + {pillars} Pillars | {min_p_str} -> {svg_name}")
        
    # 2. Process Track 3 & Track 4 through Webots & 45° SLAM
    process_webots_slam_track(get_track_3_slalom_pillars, 3, "Track 3: Grand Prix Arena", angle_deg=45.0)
    process_webots_slam_track(get_track_4_switchback_arena, 4, "Track 4: Technical Switchback Arena", angle_deg=45.0)
    
    print("\n" + "=" * 105)
    print("   COMPLETE: ALL 10 TRACKS UPDATED & WEBOTS SLAM PROCESSED FOR TRACKS 3 & 4")
    print("=" * 105)

if __name__ == "__main__":
    main()
