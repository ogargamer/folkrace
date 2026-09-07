import os
import re
import time
import json
import numpy as np
import scipy.spatial as spatial
from scipy.interpolate import splprep, splev
from scipy.ndimage import gaussian_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import sys
sys.path.insert(0, r"c:\Users\nurmi\Documents\Da Winna")

from racing_engine.centerline_finder import get_all_boundary_and_obstacle_points, find_centerline_voronoi
from racing_engine.racing_line_optimizer import compute_spline_properties, TUMMinimumCurvatureOptimizer

wbt_path = r"C:\Users\nurmi\Downloads\track_world_1787780744372.wbt"
with open(wbt_path, "r", encoding="utf-8") as f:
    content = f.read()

solid_blocks = content.split("Solid {")
left_wall = None
right_wall = None
obstacles = []

for block in solid_blocks[1:]:
    name_match = re.search(r'name\s+"([^"]+)"', block)
    name = name_match.group(1) if name_match else ""
    trans_match = re.search(r'translation\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', block)
    trans = [float(x) for x in trans_match.groups()] if trans_match else [0.0, 0.0, 0.0]
    
    if "IndexedFaceSet" in block:
        pt_match = re.search(r'point\s*\[\s*([\s\S]*?)\]', block)
        if pt_match:
            pts_str = pt_match.group(1).strip()
            pts_coords = [float(x) for x in re.split(r'[,\s]+', pts_str) if x]
            pts_3d = np.array(pts_coords).reshape(-1, 3)
            z0_mask = np.isclose(pts_3d[:, 2], 0.0, atol=1e-3)
            pts_2d = pts_3d[z0_mask, :2]
            
            if "left_wall" in name:
                left_wall = pts_2d
            elif "right_wall" in name:
                right_wall = pts_2d
    elif "Cylinder" in block:
        rad_match = re.search(r'radius\s+([-\d.]+)', block)
        rad = float(rad_match.group(1)) if rad_match else 0.05
        obstacles.append({
            "type": "circle",
            "name": name,
            "cx": trans[0],
            "cy": trans[1],
            "radius": rad
        })
    elif "Box" in block:
        size_match = re.search(r'size\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', block)
        size = [float(x) for x in size_match.groups()] if size_match else [0.1, 0.1, 0.1]
        obstacles.append({
            "type": "wall_rectangle",
            "name": name,
            "center_x": trans[0],
            "center_y": trans[1],
            "width": size[0],
            "extrusion": size[1],
            "size": size
        })

track_data = {
    "name": "Track World 1787780744372",
    "left_wall": left_wall,
    "right_wall": right_wall,
    "obstacles": obstacles,
    "track_width": 1.0,
    "tile_size": 1.2
}

lw_p1, lw_p2 = left_wall[:-1], left_wall[1:]
rw_p1, rw_p2 = right_wall[:-1], right_wall[1:]

def dist_to_segment_array(pt, p1_arr, p2_arr):
    p_to_p1 = pt - p1_arr
    seg_vec = p2_arr - p1_arr
    proj = np.clip(np.sum(p_to_p1 * seg_vec, axis=1) / (np.sum(seg_vec**2, axis=1) + 1e-12), 0.0, 1.0)
    closest = p1_arr + proj[:, np.newaxis] * seg_vec
    dists = np.hypot(pt[0] - closest[:, 0], pt[1] - closest[:, 1])
    min_idx = np.argmin(dists)
    return dists[min_idx], closest[min_idx]

freestanding_pillars = []
for idx, obs in enumerate(obstacles):
    if obs.get("type") == "circle":
        cx, cy, r = obs["cx"], obs["cy"], obs["radius"]
        pt = np.array([cx, cy])
        d_lw, p_lw = dist_to_segment_array(pt, lw_p1, lw_p2)
        d_rw, p_rw = dist_to_segment_array(pt, rw_p1, rw_p2)
        w_left = d_lw - r
        w_right = d_rw - r
        print(f"Pillar '{obs['name']}' at ({cx:.3f}, {cy:.3f}), r={r:.3f}: d_lw={d_lw:.3f}, d_rw={d_rw:.3f}, w_left={w_left:.3f}, w_right={w_right:.3f}")
        if w_left >= 0.15 and w_right >= 0.15:
            left_center = pt + ((p_lw - pt) / d_lw) * (r + w_left / 2.0)
            right_center = pt + ((p_rw - pt) / d_rw) * (r + w_right / 2.0)
            freestanding_pillars.append({
                "id": obs["name"],
                "name": f"{obs['name']} (r={r*100:.0f}cm)",
                "center": pt,
                "radius": r,
                "left_channel_pt": left_center,
                "right_channel_pt": right_center
            })

print(f"Total freestanding pillars: {len(freestanding_pillars)}")
