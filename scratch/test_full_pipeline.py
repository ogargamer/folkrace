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

print(f"Loaded track from WBT. Left wall: {len(left_wall)}, Right wall: {len(right_wall)}, Obstacles: {len(obstacles)}")

# Voronoi Medial Axis
t0 = time.perf_counter()
vor_res = find_centerline_voronoi(track_data, num_points=120)
print(f"Voronoi computed in {(time.perf_counter() - t0)*1000:.2f} ms")

# TUM Minimum Curvature QP Optimizer
t1 = time.perf_counter()
optimizer = TUMMinimumCurvatureOptimizer(
    weight_curvature=2.00,
    weight_path_smoothness=0.12,
    weight_jerk=0.40,
    robot_safety_margin=0.12, # 12 cm margin everywhere
    num_progression_stages=4,
    max_solver_sub_iters=120,
    max_speed=4.0,
    friction_coeff=0.60 # MU = 0.60
)
res_master = optimizer.optimize_racing_line(track_data, vor_res)
opt_raw = res_master["racing_line"]
print(f"QP Optimization finished in {(time.perf_counter() - t1)*1000:.2f} ms")

# Fit 1000-point smooth spline
tck_opt, _ = splprep([opt_raw[:, 0], opt_raw[:, 1]], s=0.005, per=True)
u_1000 = np.linspace(0, 1, 1000, endpoint=False)
optimal_line = np.column_stack(splev(u_1000, tck_opt))

props_opt = compute_spline_properties(optimal_line)
v_opt = optimizer._generate_velocity_profile(props_opt["curvature"], props_opt["ds"], lookahead_distance_m=0.10)
lap_time = float(np.sum(props_opt["ds"] / np.maximum(v_opt, 0.1)))
track_len = float(np.sum(props_opt["ds"]))

boundary_pts = get_all_boundary_and_obstacle_points(track_data, point_spacing=0.020)
barrier_tree = spatial.cKDTree(boundary_pts)
min_clear_cm = float(np.min(barrier_tree.query(optimal_line)[0])) * 100.0

print(f"Master Line: Lap time = {lap_time:.3f} s, Track Length = {track_len:.2f} m, Min Clearance = {min_clear_cm:.2f} cm")

# Now let's calculate the 25cm offset lines from walls
# 1. From raw left_wall and right_wall:
# For each wall, compute inward normals and offset 25cm:
def compute_inward_wall_offset(wall_pts, target_other_wall_tree, offset_dist=0.25):
    N = len(wall_pts)
    # Wrap for continuous loop
    wrapped = np.vstack([wall_pts[-1:], wall_pts, wall_pts[:1]])
    tangents = wrapped[2:] - wrapped[:-2]
    tang_norm = np.hypot(tangents[:, 0], tangents[:, 1])[:, np.newaxis] + 1e-9
    tangents /= tang_norm
    
    # 2 possible normals: (+ny, -nx) or (-ny, +nx)
    norm1 = np.column_stack([-tangents[:, 1], tangents[:, 0]])
    norm2 = -norm1
    
    # Test which normal points towards the other wall
    test_pt1 = wall_pts + 0.1 * norm1
    test_pt2 = wall_pts + 0.1 * norm2
    
    d1, _ = target_other_wall_tree.query(test_pt1)
    d2, _ = target_other_wall_tree.query(test_pt2)
    
    # The inward normal brings us closer to the other wall
    inward_normals = np.where((d1 < d2)[:, np.newaxis], norm1, norm2)
    
    # Smooth normals slightly
    inward_normals[:, 0] = gaussian_filter1d(inward_normals[:, 0], sigma=1.5, mode='wrap')
    inward_normals[:, 1] = gaussian_filter1d(inward_normals[:, 1], sigma=1.5, mode='wrap')
    inward_normals /= (np.hypot(inward_normals[:, 0], inward_normals[:, 1])[:, np.newaxis] + 1e-9)
    
    offset_pts = wall_pts + offset_dist * inward_normals
    return offset_pts

tree_rw = spatial.cKDTree(right_wall)
tree_lw = spatial.cKDTree(left_wall)

left_offset_25cm = compute_inward_wall_offset(left_wall, tree_rw, 0.25)
right_offset_25cm = compute_inward_wall_offset(right_wall, tree_lw, 0.25)

# Fit smooth periodic splines for 25cm offset lines
tck_lw25, _ = splprep([left_offset_25cm[:, 0], left_offset_25cm[:, 1]], s=0.002, per=True)
left_offset_25cm_fine = np.column_stack(splev(u_1000, tck_lw25))

tck_rw25, _ = splprep([right_offset_25cm[:, 0], right_offset_25cm[:, 1]], s=0.002, per=True)
right_offset_25cm_fine = np.column_stack(splev(u_1000, tck_rw25))

print("Offset lines computed successfully!")
