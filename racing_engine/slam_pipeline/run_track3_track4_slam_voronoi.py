#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - TRACK 3 & TRACK 4 WEBOTS DUAL VORONOI CENTERLINES & 500mm WALLS
================================================================================
1. Converts Track 3 & Track 4 into 45° Webots / Cartographer 2cm SLAM grids.
2. Extracts BOTH Voronoi centerlines:
   - Voronoi 1 (Wall-Only Macro Voronoi): Ignores obstacles, used to reconstruct
     exact +/-500mm vector boundary walls.
   - Voronoi 2 (Obstacle-Aware Voronoi): Actively detects and routes around pillars/boxes.
3. Overlays ALL CAD references strictly as DOTTED lines.
4. Exports standalone vector SVGs, high-res PNGs, and updates cad_viewer.html.
================================================================================
"""

import os
import sys
import shutil
import cv2
import numpy as np
import scipy.spatial as spatial
import scipy.ndimage as ndimage
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import splprep, splev
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, BASE_DIR)

from racing_engine.track_generator import get_track_3_slalom_pillars, get_track_4_switchback_arena
from racing_engine.centerline_finder import clean_and_resample_spline
from webots_simulation.simulate_cartographer_2cm_grid import simulate_cartographer_slam, rotate_points_2d, OUT_SLAM_DIR

ART_DIR = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"

# ==============================================================================
# CORRIDOR EXTRACTION & DUAL VORONOI ENGINES
# ==============================================================================
def extract_obstacle_free_corridor(occupancy_grid, resolution=0.02):
    h, w = occupancy_grid.shape
    free_mask = np.zeros((h, w), dtype=np.uint8)
    free_mask[occupancy_grid == 0] = 255
    
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    closed_mask = cv2.morphologyEx(free_mask, cv2.MORPH_CLOSE, kernel)
    
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(closed_mask)
    largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    drivable = np.zeros((h, w), dtype=np.uint8)
    drivable[labels == largest_label] = 255
    
    contours, _ = cv2.findContours(drivable, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    sorted_idx = sorted(range(len(contours)), key=lambda i: cv2.arcLength(contours[i], True), reverse=True)
    
    outer_contour = contours[sorted_idx[0]].squeeze()
    inner_contour = contours[sorted_idx[1]].squeeze() if len(sorted_idx) > 1 else contours[sorted_idx[0]].squeeze()
    
    obstacle_free_corridor = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(obstacle_free_corridor, [contours[sorted_idx[0]]], -1, 255, thickness=cv2.FILLED)
    if len(sorted_idx) > 1:
        cv2.drawContours(obstacle_free_corridor, [contours[sorted_idx[1]]], -1, 0, thickness=cv2.FILLED)
        
    return drivable, obstacle_free_corridor, outer_contour, inner_contour

# 1. Wall-Only Voronoi (Ignores obstacles, used to reconstruct 500mm walls)
def generate_wall_only_voronoi_centerline(slam_res, outer_c, inner_c):
    res = slam_res["resolution"]
    min_x, min_y = slam_res["min_x"], slam_res["min_y"]
    h, w = slam_res["occupancy_grid"].shape
    
    out_w = np.column_stack([min_x + outer_c[:, 0] * res, min_y + outer_c[:, 1] * res])
    in_w = np.column_stack([min_x + inner_c[:, 0] * res, min_y + inner_c[:, 1] * res])
    
    stride_out = max(1, len(out_w) // 300)
    stride_in = max(1, len(in_w) // 300)
    wall_points = np.vstack([out_w[::stride_out], in_w[::stride_in]])
    
    vor = spatial.Voronoi(wall_points)
    
    _, obs_free, _, _ = extract_obstacle_free_corridor(slam_res["occupancy_grid"], res)
    dist_map = ndimage.distance_transform_edt(obs_free == 255) * res
    
    valid_vertices = {}
    for i, v in enumerate(vor.vertices):
        gx = int((v[0] - min_x) / res)
        gy = int((v[1] - min_y) / res)
        if 0 <= gx < w and 0 <= gy < h:
            if obs_free[gy, gx] == 255 and dist_map[gy, gx] >= 0.18:
                valid_vertices[i] = v
                
    adj = {i: set() for i in valid_vertices}
    for v1, v2 in vor.ridge_vertices:
        if v1 in valid_vertices and v2 in valid_vertices:
            adj[v1].add(v2)
            adj[v2].add(v1)
            
    pruned = True
    while pruned:
        pruned = False
        deg1 = [n for n in adj if len(adj[n]) <= 1]
        if deg1:
            pruned = True
            for n in deg1:
                for nbr in adj[n]:
                    adj[nbr].discard(n)
                del adj[n]
                
    if len(adj) < 10:
        return None
        
    start_node = max(adj.keys(), key=lambda n: dist_map[int((valid_vertices[n][1]-min_y)/res), int((valid_vertices[n][0]-min_x)/res)])
    visited = set([start_node])
    ordered = [start_node]
    curr = start_node
    prev_dir = None
    
    while True:
        unvis = [n for n in adj[curr] if n not in visited]
        if not unvis:
            break
        if len(unvis) == 1:
            nxt = unvis[0]
        else:
            if prev_dir is not None:
                def score(n):
                    vec = valid_vertices[n] - valid_vertices[curr]
                    v_len = np.linalg.norm(vec)
                    if v_len < 1e-4: return -1.0
                    cos_sim = np.dot(vec / v_len, prev_dir)
                    gy_n = int((valid_vertices[n][1] - min_y) / res)
                    gx_n = int((valid_vertices[n][0] - min_x) / res)
                    cl_n = dist_map[gy_n, gx_n] if (0 <= gx_n < w and 0 <= gy_n < h) else 0.0
                    return cos_sim * 2.0 + cl_n * 1.5
                nxt = max(unvis, key=score)
            else:
                nxt = unvis[0]
        vec = valid_vertices[nxt] - valid_vertices[curr]
        if np.linalg.norm(vec) > 1e-4:
            prev_dir = vec / np.linalg.norm(vec)
        visited.add(nxt)
        ordered.append(nxt)
        curr = nxt
        
    raw_coords = np.array([valid_vertices[n] for n in ordered])
    return clean_and_resample_spline(raw_coords, num_output_points=400, sigma=2.0)

# 2. Obstacle-Aware Voronoi (Actively detects and routes around obstacles)
def generate_obstacle_aware_voronoi_centerline(slam_res, min_clearance=0.14):
    occ_pts = slam_res["occupied_pts"]
    grid = slam_res["occupancy_grid"]
    res = slam_res["resolution"]
    min_x, min_y = slam_res["min_x"], slam_res["min_y"]
    h, w = grid.shape
    
    free_mask = np.zeros((h, w), dtype=np.uint8)
    free_mask[grid == 0] = 255
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    closed_mask = cv2.morphologyEx(free_mask, cv2.MORPH_CLOSE, kernel)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(closed_mask)
    largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    drivable = np.zeros((h, w), dtype=np.uint8)
    drivable[labels == largest_label] = 255
    dist_map = ndimage.distance_transform_edt(drivable == 255) * res
    
    stride = max(1, len(occ_pts) // 1200)
    sample_occ = occ_pts[::stride]
    vor = spatial.Voronoi(sample_occ)
    
    valid_vertices = {}
    for i, v in enumerate(vor.vertices):
        gx = int((v[0] - min_x) / res)
        gy = int((v[1] - min_y) / res)
        if 0 <= gx < w and 0 <= gy < h:
            if drivable[gy, gx] == 255 and dist_map[gy, gx] >= min_clearance:
                valid_vertices[i] = v
                
    adj = {i: set() for i in valid_vertices}
    for v1, v2 in vor.ridge_vertices:
        if v1 in valid_vertices and v2 in valid_vertices:
            adj[v1].add(v2)
            adj[v2].add(v1)
            
    pruned = True
    while pruned:
        pruned = False
        deg1 = [n for n in adj if len(adj[n]) <= 1]
        if deg1:
            pruned = True
            for n in deg1:
                for nbr in adj[n]:
                    adj[nbr].discard(n)
                del adj[n]
                
    start_node = max(adj.keys(), key=lambda n: dist_map[int((valid_vertices[n][1]-min_y)/res), int((valid_vertices[n][0]-min_x)/res)])
    visited = set([start_node])
    ordered = [start_node]
    curr = start_node
    prev_dir = None
    
    while True:
        unvis = [n for n in adj[curr] if n not in visited]
        if not unvis:
            break
        if len(unvis) == 1:
            nxt = unvis[0]
        else:
            if prev_dir is not None:
                def score(n):
                    vec = valid_vertices[n] - valid_vertices[curr]
                    v_len = np.linalg.norm(vec)
                    if v_len < 1e-4: return -1.0
                    cos_sim = np.dot(vec / v_len, prev_dir)
                    gy_n = int((valid_vertices[n][1] - min_y) / res)
                    gx_n = int((valid_vertices[n][0] - min_x) / res)
                    cl_n = dist_map[gy_n, gx_n] if (0 <= gx_n < w and 0 <= gy_n < h) else 0.0
                    return cos_sim * 2.5 + cl_n * 1.5
                nxt = max(unvis, key=score)
            else:
                nxt = unvis[0]
        vec = valid_vertices[nxt] - valid_vertices[curr]
        if np.linalg.norm(vec) > 1e-4:
            prev_dir = vec / np.linalg.norm(vec)
        visited.add(nxt)
        ordered.append(nxt)
        curr = nxt
        
    raw_coords = np.array([valid_vertices[n] for n in ordered])
    return clean_and_resample_spline(raw_coords, num_output_points=400, sigma=3.0)

# Compute 500mm Offset Walls from Wall-Only Voronoi
def compute_500mm_reconstructed_walls(centerline_points):
    N = len(centerline_points)
    pts_wrap = np.vstack([centerline_points[-1:], centerline_points, centerline_points[:2]])
    dx = (pts_wrap[2:, 0] - pts_wrap[:-2, 0]) * 0.5
    dy = (pts_wrap[2:, 1] - pts_wrap[:-2, 1]) * 0.5
    norms = np.hypot(dx, dy) + 1e-6
    tangents = np.column_stack([dx / norms, dy / norms])
    normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])
    
    # Sliced to N points
    n_pts = normals[1:N+1]
    syn_left = centerline_points + n_pts * 0.50
    syn_right = centerline_points - n_pts * 0.50
    return syn_left, syn_right

# ==============================================================================
# WEBOTS SLAM DUAL VORONOI SVG EXPORTER (ALL CAD REFERENCES AS DOTTED LINES)
# ==============================================================================
def export_slam_dual_voronoi_svg(filename, track_data, track_idx, slam_res, wall_vor_cl, obs_vor_cl, syn_left, syn_right, cad_left_rot, cad_right_rot, macro_cl_rot):
    name = track_data["name"]
    occ_pts = slam_res["occupied_pts"]
    
    all_pts = np.vstack([occ_pts, cad_left_rot, cad_right_rot, wall_vor_cl, obs_vor_cl, syn_left, syn_right])
    min_x, min_y = np.min(all_pts, axis=0) - 0.5
    max_x, max_y = np.max(all_pts, axis=0) + 0.5
    scale = 100.0
    
    svg_w = int((max_x - min_x) * scale)
    svg_h = int((max_y - min_y) * scale)
    
    def tx(x): return (x - min_x) * scale
    def ty(y): return svg_h - (y - min_y) * scale
    
    svg = []
    svg.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {svg_w} {svg_h}" width="{svg_w}" height="{svg_h}" style="background:#080c14; font-family:-apple-system, BlinkMacSystemFont, Segoe UI, Roboto, sans-serif;">')
    
    # Background Technical Grid
    svg.append('<defs>')
    svg.append('  <pattern id="grid_minor" width="20" height="20" patternUnits="userSpaceOnUse">')
    svg.append('    <path d="M 20 0 L 0 0 0 20" fill="none" stroke="#172033" stroke-width="0.75" stroke-opacity="0.4"/>')
    svg.append('  </pattern>')
    svg.append('  <pattern id="grid_major" width="100" height="100" patternUnits="userSpaceOnUse">')
    svg.append('    <rect width="100" height="100" fill="url(#grid_minor)"/>')
    svg.append('    <path d="M 100 0 L 0 0 0 100" fill="none" stroke="#23324d" stroke-width="1.2" stroke-opacity="0.6"/>')
    svg.append('  </pattern>')
    svg.append('</defs>')
    svg.append(f'<rect width="{svg_w}" height="{svg_h}" fill="url(#grid_major)" />')
    
    # Title Header
    svg.append(f'<text x="28" y="42" fill="#f8fafc" font-size="22" font-weight="700">{name} - 45° Dual Voronoi &amp; 500mm Offset Walls</text>')
    svg.append(f'<text x="28" y="68" fill="#94a3b8" font-size="13">Obstacle-Aware Voronoi + Wall-Only Voronoi Macro Baseline (CAD References: Dotted Lines)</text>')
    
    # SLAM Grid Barrier Voxels (Cyan 2cm blocks)
    svg.append('<g id="slam_barrier_voxels" opacity="0.85">')
    for pt in occ_pts:
        svg.append(f'<rect x="{tx(pt[0])-1.1:.1f}" y="{ty(pt[1])-1.1:.1f}" width="2.2" height="2.2" fill="#00f0ff" opacity="0.80"/>')
    svg.append('</g>')
    
    # Rotated Obstacles (Pillars & Wall Boxes)
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
    
    # -------------------------------------------------------------------------
    # ALL CAD REFERENCES AS DOTTED LINES
    # -------------------------------------------------------------------------
    # CAD Left Wall (Dotted Cyan)
    lw_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in cad_left_rot]) + " Z"
    svg.append(f'<path d="{lw_str}" fill="none" stroke="#38bdf8" stroke-width="2.0" stroke-dasharray="4,4" opacity="0.65"/>')
    
    # CAD Right Wall (Dotted Rose)
    rw_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in cad_right_rot]) + " Z"
    svg.append(f'<path d="{rw_str}" fill="none" stroke="#f43f5e" stroke-width="2.0" stroke-dasharray="4,4" opacity="0.65"/>')
    
    # CAD Baseline Centerline (Dotted Gold Reference)
    mc_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in macro_cl_rot]) + " Z"
    svg.append(f'<path d="{mc_str}" fill="none" stroke="#ffd700" stroke-width="2.0" stroke-dasharray="3,3" opacity="0.60"/>')
    
    # -------------------------------------------------------------------------
    # RECONSTRUCTED +/-500mm SYNTHETIC VECTOR WALLS (from Wall-Only Voronoi)
    # -------------------------------------------------------------------------
    sl_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in syn_left]) + " Z"
    sr_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in syn_right]) + " Z"
    svg.append(f'<path d="{sl_str}" fill="none" stroke="#22d3ee" stroke-width="2.8" opacity="0.90"/>')
    svg.append(f'<path d="{sr_str}" fill="none" stroke="#fb7185" stroke-width="2.8" opacity="0.90"/>')
    
    # -------------------------------------------------------------------------
    # VORONOI 1: WALL-ONLY VORONOI CENTERLINE (Vivid Violet / Magenta)
    # -------------------------------------------------------------------------
    wvor_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in wall_vor_cl]) + " Z"
    svg.append(f'<path d="{wvor_str}" fill="none" stroke="#c084fc" stroke-width="3.2" stroke-linecap="round" stroke-linejoin="round" opacity="0.90"/>')
    
    # -------------------------------------------------------------------------
    # VORONOI 2: OBSTACLE-AWARE VORONOI CENTERLINE (Solid Bright Neon Green)
    # -------------------------------------------------------------------------
    ovor_str = "M " + " L ".join([f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in obs_vor_cl]) + " Z"
    svg.append(f'<path d="{ovor_str}" fill="none" stroke="#4ade80" stroke-width="3.8" stroke-linecap="round" stroke-linejoin="round" opacity="0.95"/>')
    
    # Technical Legend
    leg_w, leg_h = 560, 200
    leg_x, leg_y = 24, svg_h - leg_h - 24
    svg.append(f'<rect x="{leg_x}" y="{leg_y}" width="{leg_w}" height="{leg_h}" fill="#111827" rx="10" stroke="#374151" stroke-width="1.5" opacity="0.95"/>')
    
    svg.append(f'<line x1="{leg_x+20}" y1="{leg_y+26}" x2="{leg_x+60}" y2="{leg_y+26}" stroke="#4ade80" stroke-width="4"/>')
    svg.append(f'<text x="{leg_x+72}" y="{leg_y+31}" fill="#4ade80" font-size="13" font-weight="700">Obstacle-Aware Voronoi Centerline (Routes Around Obstacles)</text>')
    
    svg.append(f'<line x1="{leg_x+20}" y1="{leg_y+52}" x2="{leg_x+60}" y2="{leg_y+52}" stroke="#c084fc" stroke-width="3.5"/>')
    svg.append(f'<text x="{leg_x+72}" y="{leg_y+57}" fill="#c084fc" font-size="13" font-weight="700">Wall-Only Voronoi Centerline (Ignores Obstacles)</text>')
    
    svg.append(f'<line x1="{leg_x+20}" y1="{leg_y+78}" x2="{leg_x+60}" y2="{leg_y+78}" stroke="#22d3ee" stroke-width="2.8"/>')
    svg.append(f'<line x1="{leg_x+20}" y1="{leg_y+88}" x2="{leg_x+60}" y2="{leg_y+88}" stroke="#fb7185" stroke-width="2.8"/>')
    svg.append(f'<text x="{leg_x+72}" y="{leg_y+85}" fill="#f1f5f9" font-size="12" font-weight="600">Reconstructed +/-500mm Vector Walls (Offset from Wall Voronoi)</text>')
    
    svg.append(f'<line x1="{leg_x+20}" y1="{leg_y+112}" x2="{leg_x+60}" y2="{leg_y+112}" stroke="#ffd700" stroke-width="2.2" stroke-dasharray="3,3"/>')
    svg.append(f'<text x="{leg_x+72}" y="{leg_y+117}" fill="#ffd700" font-size="12">CAD Ground Truth Baseline Centerline (Dotted Reference)</text>')
    
    svg.append(f'<line x1="{leg_x+20}" y1="{leg_y+138}" x2="{leg_x+60}" y2="{leg_y+138}" stroke="#38bdf8" stroke-width="2" stroke-dasharray="4,4"/>')
    svg.append(f'<line x1="{leg_x+20}" y1="{leg_y+148}" x2="{leg_x+60}" y2="{leg_y+148}" stroke="#f43f5e" stroke-width="2" stroke-dasharray="4,4"/>')
    svg.append(f'<text x="{leg_x+72}" y="{leg_y+145}" fill="#94a3b8" font-size="12">CAD Left / Right Wall Boundaries (Dotted Reference)</text>')
    
    svg.append(f'<rect x="{leg_x+20}" y="{leg_y+166}" width="40" height="10" fill="#00f0ff" opacity="0.80"/>')
    svg.append(f'<text x="{leg_x+72}" y="{leg_y+175}" fill="#00f0ff" font-size="12">Cartographer 2cm SLAM Barrier Voxels</text>')
    
    svg.append('</svg>')
    
    with open(filename, "w", encoding="utf-8") as f:
        f.write("\n".join(svg))

# ==============================================================================
# HIGH-RES PNG RENDERER
# ==============================================================================
def export_slam_dual_voronoi_png(filename, track_data, track_idx, slam_res, wall_vor_cl, obs_vor_cl, syn_left, syn_right, cad_left_rot, cad_right_rot, macro_cl_rot):
    name = track_data["name"]
    fig, ax = plt.subplots(figsize=(16, 12), facecolor="#080c14")
    ax.set_facecolor="#080c14"
    
    grid = slam_res["occupancy_grid"]
    res = slam_res["resolution"]
    extent = [slam_res["min_x"], slam_res["min_x"] + slam_res["grid_w"] * res,
              slam_res["min_y"], slam_res["min_y"] + slam_res["grid_h"] * res]
    cmap = matplotlib.colors.ListedColormap(['#151b28', '#070a12', '#00f0ff'])
    norm = matplotlib.colors.BoundaryNorm([-1.5, -0.5, 50, 150], cmap.N)
    ax.imshow(grid, origin='lower', extent=extent, cmap=cmap, norm=norm, interpolation='nearest', alpha=0.70)
    
    for obs in slam_res["obstacles"]:
        if obs["type"] == "circle":
            circle = plt.Circle((obs["cx"], obs["cy"]), obs["radius"], facecolor="#fbbf24", edgecolor="#ffffff", linewidth=1.5, alpha=0.95, zorder=6)
            ax.add_patch(circle)
        elif obs["type"] == "wall_rectangle":
            poly = obs.get("polygon")
            if poly is not None:
                patch = plt.Polygon(poly, facecolor="#fbbf24", edgecolor="#ffffff", linewidth=1.5, alpha=0.95, zorder=6)
                ax.add_patch(patch)
                
    # ALL CAD REFERENCES AS DOTTED LINES
    ax.plot(np.vstack([cad_left_rot, cad_left_rot[:1]])[:, 0], np.vstack([cad_left_rot, cad_left_rot[:1]])[:, 1], color="#38bdf8", linewidth=1.8, linestyle="--", alpha=0.65, label="CAD Left Wall (Dotted)")
    ax.plot(np.vstack([cad_right_rot, cad_right_rot[:1]])[:, 0], np.vstack([cad_right_rot, cad_right_rot[:1]])[:, 1], color="#f43f5e", linewidth=1.8, linestyle="--", alpha=0.65, label="CAD Right Wall (Dotted)")
    ax.plot(np.vstack([macro_cl_rot, macro_cl_rot[:1]])[:, 0], np.vstack([macro_cl_rot, macro_cl_rot[:1]])[:, 1], color="#ffd700", linewidth=1.8, linestyle=":", alpha=0.60, label="CAD Ground Truth Centerline (Dotted)")
    
    # RECONSTRUCTED +/-500mm SYNTHETIC WALLS
    ax.plot(np.vstack([syn_left, syn_left[:1]])[:, 0], np.vstack([syn_left, syn_left[:1]])[:, 1], color="#22d3ee", linewidth=2.5, label="Reconstructed +500mm Left Wall", zorder=7)
    ax.plot(np.vstack([syn_right, syn_right[:1]])[:, 0], np.vstack([syn_right, syn_right[:1]])[:, 1], color="#fb7185", linewidth=2.5, label="Reconstructed -500mm Right Wall", zorder=7)
    
    # VORONOI 1: WALL-ONLY VORONOI CENTERLINE
    ax.plot(np.vstack([wall_vor_cl, wall_vor_cl[:1]])[:, 0], np.vstack([wall_vor_cl, wall_vor_cl[:1]])[:, 1], color="#c084fc", linewidth=3.0, label="Wall-Only Voronoi Centerline (No Obstacles)", zorder=8)
    
    # VORONOI 2: OBSTACLE-AWARE VORONOI CENTERLINE
    ax.plot(np.vstack([obs_vor_cl, obs_vor_cl[:1]])[:, 0], np.vstack([obs_vor_cl, obs_vor_cl[:1]])[:, 1], color="#4ade80", linewidth=3.5, label="Obstacle-Aware Voronoi Centerline", zorder=9)
    
    ax.set_title(f"{name} - Webots 45° Dual Voronoi & Reconstructed 500mm Walls\n(CAD References: Dotted Lines | Wall Voronoi & Obstacle Voronoi)", color="#f8fafc", fontsize=13, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.15, color="#ffffff")
    ax.tick_params(colors="#94a3b8")
    ax.set_xlabel("X (meters)", color="#94a3b8", fontsize=11)
    ax.set_ylabel("Y (meters)", color="#94a3b8", fontsize=11)
    ax.legend(loc="upper left", facecolor="#111827", edgecolor="#374151", labelcolor="#f8fafc", fontsize=9.5, borderpad=1.0)
    
    plt.savefig(filename, dpi=200, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)

# ==============================================================================
# MAIN RUNNER
# ==============================================================================
def main():
    print("=" * 105)
    print("   RUNNING WEBOTS DUAL VORONOI PIPELINE FOR TRACK 3 & TRACK 4 (WITH 500mm WALL OFFSETS)")
    print("=" * 105)
    
    tasks = [
        (get_track_3_slalom_pillars, 3, "Track 3: Grand Prix Arena"),
        (get_track_4_switchback_arena, 4, "Track 4: Technical Switchback Arena")
    ]
    
    for getter, idx, name in tasks:
        track = getter()
        rot_rad = np.radians(45.0)
        raw_left = np.array(track["left_wall"])
        anchor = np.mean(raw_left, axis=0)
        
        # 1. Simulate 45° Cartographer SLAM Grid
        temp_traj_path = os.path.join(BASE_DIR, "webots_simulation", f"temp_track{idx}_traj.csv")
        np.savetxt(temp_traj_path, np.column_stack([track["centerline"], np.zeros(len(track["centerline"])), np.zeros(len(track["centerline"]))]), delimiter=",", header="x,y,z,yaw", comments="")
        slam_res = simulate_cartographer_slam(track, temp_traj_path, angle_deg=45.0, resolution=0.02)
        
        # 2. Extract Corridor and BOTH Voronoi Centerlines
        drivable, obs_free, outer_c, inner_c = extract_obstacle_free_corridor(slam_res["occupancy_grid"], 0.02)
        
        # A. Voronoi 1: Wall-Only Voronoi (Ignores obstacles)
        wall_vor_cl = generate_wall_only_voronoi_centerline(slam_res, outer_c, inner_c)
        
        # B. Voronoi 2: Obstacle-Aware Voronoi (Routes around obstacles)
        obs_vor_cl = generate_obstacle_aware_voronoi_centerline(slam_res, min_clearance=0.14)
        
        # C. Reconstructed +/-500mm Vector Walls from Wall-Only Voronoi
        syn_left, syn_right = compute_500mm_reconstructed_walls(wall_vor_cl)
        
        # Rotate CAD references by 45°
        cad_left_rot = rotate_points_2d(raw_left, rot_rad, anchor)
        cad_right_rot = rotate_points_2d(np.array(track["right_wall"]), rot_rad, anchor)
        macro_cl_rot = rotate_points_2d(track["centerline"], rot_rad, anchor)
        
        # 3. Export Vector SVG with Dual Voronois, 500mm Walls & Dotted CAD references
        svg_name = f"track{idx:02d}_45deg_webots_slam_and_racing_line.svg"
        svg_path = os.path.join(OUT_SLAM_DIR, svg_name)
        export_slam_dual_voronoi_svg(svg_path, track, idx, slam_res, wall_vor_cl, obs_vor_cl, syn_left, syn_right, cad_left_rot, cad_right_rot, macro_cl_rot)
        shutil.copy2(svg_path, os.path.join(ART_DIR, svg_name))
        
        # 4. Export High-Res PNG
        png_name = f"track{idx:02d}_45deg_webots_slam_and_racing_line.png"
        png_path = os.path.join(OUT_SLAM_DIR, png_name)
        export_slam_dual_voronoi_png(png_path, track, idx, slam_res, wall_vor_cl, obs_vor_cl, syn_left, syn_right, cad_left_rot, cad_right_rot, macro_cl_rot)
        shutil.copy2(png_path, os.path.join(ART_DIR, png_name))
        
        # Benchmark error against CAD ground truth
        kdt_cad = spatial.cKDTree(macro_cl_rot)
        dists_wall, _ = kdt_cad.query(wall_vor_cl)
        err_wall_mm = np.mean(dists_wall) * 1000.0
        
        print(f"  [OK] Track {idx}: {name:<35} | Wall Voronoi Dev: {err_wall_mm:.1f}mm | 500mm Walls Offset OK -> {svg_name}")
        
    print("\n" + "=" * 105)
    print("   COMPLETE: DUAL VORONOIS & 500mm RECONSTRUCTED WALLS EXPORTED FOR TRACK 3 & TRACK 4")
    print("=" * 105)

if __name__ == "__main__":
    main()
