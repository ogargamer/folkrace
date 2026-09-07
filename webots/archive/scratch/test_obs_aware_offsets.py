import os
import sys
import time
import json
import re
import numpy as np
import scipy.spatial as spatial
from scipy.interpolate import splprep, splev
from scipy.ndimage import gaussian_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

BASE_DIR = r"c:\Users\nurmi\Documents\Da Winna"
sys.path.insert(0, BASE_DIR)

from webots/optimization/racing_engine.centerline_finder import get_all_boundary_and_obstacle_points, find_centerline_voronoi
from webots/optimization/racing_engine.racing_line_optimizer import compute_spline_properties, TUMMinimumCurvatureOptimizer

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

# 1. Obstacle Points Tree
obs_pts_list = []
for obs in obstacles:
    if obs["type"] == "wall_rectangle":
        cx, cy = obs["center_x"], obs["center_y"]
        w, h = obs["width"], obs["extrusion"]
        steps_w = max(10, int(w / 0.01))
        steps_h = max(10, int(h / 0.01))
        xs_top = np.linspace(cx - w/2, cx + w/2, steps_w)
        ys_top = np.full(steps_w, cy + h/2)
        xs_bot = np.linspace(cx - w/2, cx + w/2, steps_w)
        ys_bot = np.full(steps_w, cy - h/2)
        ys_left = np.linspace(cy - h/2, cy + h/2, steps_h)
        xs_left = np.full(steps_h, cx - w/2)
        ys_right = np.linspace(cy - h/2, cy + h/2, steps_h)
        xs_right = np.full(steps_h, cx + w/2)
        obs_pts_list.append(np.vstack([
            np.column_stack([xs_top, ys_top]),
            np.column_stack([xs_bot, ys_bot]),
            np.column_stack([xs_left, ys_left]),
            np.column_stack([xs_right, ys_right])
        ]))
    elif obs["type"] == "circle":
        cx, cy, r = obs["cx"], obs["cy"], obs["radius"]
        steps_c = max(32, int(2 * np.pi * r / 0.01))
        angles = np.linspace(0, 2*np.pi, steps_c, endpoint=False)
        xs = cx + r * np.cos(angles)
        ys = cy + r * np.sin(angles)
        obs_pts_list.append(np.column_stack([xs, ys]))

all_obs_pts = np.vstack(obs_pts_list)
obs_tree = spatial.cKDTree(all_obs_pts)

# Inward offset function from wall
def compute_inward_wall_offset(wall_pts, other_wall_tree, offset_dist=0.25):
    wrapped = np.vstack([wall_pts[-1:], wall_pts, wall_pts[:1]])
    tangents = wrapped[2:] - wrapped[:-2]
    tang_norm = np.hypot(tangents[:, 0], tangents[:, 1])[:, np.newaxis] + 1e-9
    tangents /= tang_norm
    norm1 = np.column_stack([-tangents[:, 1], tangents[:, 0]])
    norm2 = -norm1
    test_pt1 = wall_pts + 0.1 * norm1
    test_pt2 = wall_pts + 0.1 * norm2
    d1, _ = other_wall_tree.query(test_pt1)
    d2, _ = other_wall_tree.query(test_pt2)
    inward_normals = np.where((d1 < d2)[:, np.newaxis], norm1, norm2)
    inward_normals[:, 0] = gaussian_filter1d(inward_normals[:, 0], sigma=1.5, mode='wrap')
    inward_normals[:, 1] = gaussian_filter1d(inward_normals[:, 1], sigma=1.5, mode='wrap')
    inward_normals /= (np.hypot(inward_normals[:, 0], inward_normals[:, 1])[:, np.newaxis] + 1e-9)
    return wall_pts + offset_dist * inward_normals

tree_rw = spatial.cKDTree(right_wall)
tree_lw = spatial.cKDTree(left_wall)

left_offset_raw = compute_inward_wall_offset(left_wall, tree_rw, 0.25)
right_offset_raw = compute_inward_wall_offset(right_wall, tree_lw, 0.25)

u_1000 = np.linspace(0, 1, 1000, endpoint=False)
tck_l, _ = splprep([left_offset_raw[:, 0], left_offset_raw[:, 1]], s=0.002, per=True)
left_line = np.column_stack(splev(u_1000, tck_l))

tck_r, _ = splprep([right_offset_raw[:, 0], right_offset_raw[:, 1]], s=0.002, per=True)
right_line = np.column_stack(splev(u_1000, tck_r))

# Make Obstacle Aware with >= 10cm margin from obstacles
def make_obstacle_aware(line_pts, other_wall_tree, obs_tree, all_obs_pts, margin=0.10):
    res_line = line_pts.copy()
    # Iterate elastic repulsion with smoothing
    for step in range(25):
        d_obs, idx_obs = obs_tree.query(res_line)
        viol = d_obs < margin
        if np.any(viol):
            closest_obs = all_obs_pts[idx_obs[viol]]
            # Direction away from obstacle towards the track corridor / other wall
            v_rep = res_line[viol] - closest_obs
            dist_v = np.linalg.norm(v_rep, axis=1, keepdims=True) + 1e-6
            v_rep /= dist_v
            
            # Push point out to obstacle + margin
            push_amount = (margin - d_obs[viol])[:, np.newaxis]
            res_line[viol] += v_rep * (push_amount + 0.005)
            
            # Smooth
            res_line[:, 0] = gaussian_filter1d(res_line[:, 0], sigma=2.0, mode='wrap')
            res_line[:, 1] = gaussian_filter1d(res_line[:, 1], sigma=2.0, mode='wrap')
            
    # Final C2 periodic spline fit
    tck, _ = splprep([res_line[:, 0], res_line[:, 1]], s=0.003, per=True)
    res_fine = np.column_stack(splev(u_1000, tck))
    
    # Verify clearances
    d_verif, _ = obs_tree.query(res_fine)
    print(f"Obstacle-aware line minimum clearance to obstacles: {np.min(d_verif)*100:.2f} cm (Target: {margin*100:.0f} cm)")
    return res_fine

print("Processing Left 25cm Offset Line with 10cm obstacle safety margin...")
left_obs_aware = make_obstacle_aware(left_line, tree_rw, obs_tree, all_obs_pts, margin=0.10)

print("Processing Right 25cm Offset Line with 10cm obstacle safety margin...")
right_obs_aware = make_obstacle_aware(right_line, tree_lw, obs_tree, all_obs_pts, margin=0.10)
