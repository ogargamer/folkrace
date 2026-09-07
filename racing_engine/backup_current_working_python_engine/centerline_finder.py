"""
Multi-Method Obstacle-Aware Centerline Calculator
Calculates the true drivable centerline through complex tracks with obstacles:
1. Method 1: Medial Axis / Voronoi Diagram Skeletonization
2. Method 2: 2D Signed Distance Field (SDF) Ridge Tracing
3. Method 3: Cross-Sectional Raycasting & Free-Space Midpoint Gate Profiler
"""

import numpy as np
import scipy.spatial as spatial
import scipy.ndimage as ndimage
from scipy.interpolate import splprep, splev
from scipy.ndimage import gaussian_filter1d

def get_all_boundary_and_obstacle_points(track_data, point_spacing=0.06):
    """
    Extracts high-density 2D points from left walls, right walls, and obstacle perimeters.
    """
    points = []
    points.append(track_data["left_wall"])
    points.append(track_data["right_wall"])
    
    for obs in track_data.get("obstacles", []):
        if obs["type"] == "wall_rectangle":
            cx, cy = obs["center_x"], obs["center_y"]
            w, h = obs["width"], obs["extrusion"]
            steps_w = max(4, int(w / point_spacing))
            steps_h = max(4, int(h / point_spacing))
            
            xs_top = np.linspace(cx - w/2, cx + w/2, steps_w)
            ys_top = np.full(steps_w, cy + h/2)
            xs_bot = np.linspace(cx - w/2, cx + w/2, steps_w)
            ys_bot = np.full(steps_w, cy - h/2)
            
            ys_left = np.linspace(cy - h/2, cy + h/2, steps_h)
            xs_left = np.full(steps_h, cx - w/2)
            ys_right = np.linspace(cy - h/2, cy + h/2, steps_h)
            xs_right = np.full(steps_h, cx + w/2)
            
            obs_pts = np.vstack([
                np.column_stack([xs_top, ys_top]),
                np.column_stack([xs_bot, ys_bot]),
                np.column_stack([xs_left, ys_left]),
                np.column_stack([xs_right, ys_right])
            ])
            points.append(obs_pts)
            
        elif obs["type"] == "circle":
            cx, cy, r = obs["cx"], obs["cy"], obs["radius"]
            steps_c = max(16, int(2 * np.pi * r / point_spacing))
            angles = np.linspace(0, 2*np.pi, steps_c, endpoint=False)
            xs = cx + r * np.cos(angles)
            ys = cy + r * np.sin(angles)
            points.append(np.column_stack([xs, ys]))

    return np.vstack(points)

def clean_and_resample_spline(points, num_output_points=360, sigma=4.0):
    """
    Cleans consecutive duplicate points and fits a periodic C2 continuous smoothing spline.
    """
    pts = np.array(points)
    # Deduplicate consecutive duplicates
    diffs = np.hypot(pts[1:, 0] - pts[:-1, 0], pts[1:, 1] - pts[:-1, 1])
    mask = np.concatenate([[True], diffs > 1e-3])
    pts_clean = pts[mask]
    
    # Gaussian pre-filter
    pts_clean[:, 0] = gaussian_filter1d(pts_clean[:, 0], sigma=sigma, mode='wrap')
    pts_clean[:, 1] = gaussian_filter1d(pts_clean[:, 1], sigma=sigma, mode='wrap')
    
    # Fit C2 periodic spline
    try:
        tck, u = splprep([pts_clean[:, 0], pts_clean[:, 1]], s=0.015, per=True)
        u_fine = np.linspace(0, 1, num_output_points, endpoint=False)
        resampled = np.column_stack(splev(u_fine, tck))
        return resampled
    except Exception:
        N = len(pts_clean)
        indices = np.linspace(0, N, num_output_points, endpoint=False)
        resampled = np.zeros((num_output_points, 2))
        for i, idx in enumerate(indices):
            i0 = int(idx) % N
            i1 = (i0 + 1) % N
            t = idx - int(idx)
            resampled[i] = (1 - t) * pts_clean[i0] + t * pts_clean[i1]
        return resampled

# ==============================================================================
#  METHOD 1: MEDIAL AXIS / VORONOI DIAGRAM SKELETONIZATION
# ==============================================================================
def find_centerline_voronoi(track_data, num_points=360):
    """
    Computes the Medial Axis by constructing a 2D Voronoi Diagram of all boundaries 
    and obstacles, filtering internal vertices, and fitting a smooth topological loop.
    """
    all_pts = get_all_boundary_and_obstacle_points(track_data, point_spacing=0.04)
    vor = spatial.Voronoi(all_pts)
    
    l_wall = track_data["left_wall"]
    r_wall = track_data["right_wall"]
    N_raw = min(len(l_wall), len(r_wall))
    raw_mid = (l_wall[:N_raw] + r_wall[:N_raw]) / 2.0
    
    tree = spatial.cKDTree(all_pts)
    valid_vertices = []
    
    for v in vor.vertices:
        dist, _ = tree.query(v)
        if 0.18 < dist < 0.65:
            d_mid = np.min(np.hypot(raw_mid[:, 0] - v[0], raw_mid[:, 1] - v[1]))
            if d_mid < 0.45:
                valid_vertices.append(v)
                
    valid_vertices = np.array(valid_vertices)
    if len(valid_vertices) < 10:
        valid_vertices = raw_mid.copy()
    
    ordered_centerline = []
    for i in range(num_points):
        target_pt = raw_mid[int(i * N_raw / num_points)]
        dists = np.hypot(valid_vertices[:, 0] - target_pt[0], valid_vertices[:, 1] - target_pt[1])
        closest_idx = np.argmin(dists)
        ordered_centerline.append(valid_vertices[closest_idx])
        
    ordered_centerline = np.array(ordered_centerline)
    
    # Obstacle Repulsion Elastic Pass: Ensure centerline never cuts through or clips obstacles
    for step in range(12):
        for i in range(num_points):
            d, idx = tree.query(ordered_centerline[i])
            if d < 0.22:
                closest_p = all_pts[idx]
                v = ordered_centerline[i] - closest_p
                norm_v = np.linalg.norm(v)
                if norm_v < 1e-4:
                    v = np.array([-ordered_centerline[i, 1], ordered_centerline[i, 0]])
                    norm_v = np.linalg.norm(v)
                direction = v / norm_v
                ordered_centerline[i] = closest_p + direction * 0.24
                
        ordered_centerline = clean_and_resample_spline(ordered_centerline, num_points, sigma=1.2)
        
    smooth_centerline = clean_and_resample_spline(ordered_centerline, num_points, sigma=2.0)
    
    return {
        "method": "Method 1: Medial Axis (Voronoi Skeleton)",
        "centerline": smooth_centerline,
        "voronoi_vertices": valid_vertices
    }

# ==============================================================================
#  METHOD 2: 2D DISTANCE TRANSFORM / SDF RIDGE TRACING
# ==============================================================================
def find_centerline_sdf_ridge(track_data, resolution=0.02, num_points=360):
    """
    Computes the centerline by generating a 2D Euclidean Distance Field (SDF) 
    and tracing the ridge (local maxima) of maximum clearance.
    """
    all_pts = get_all_boundary_and_obstacle_points(track_data, point_spacing=0.03)
    
    min_x = np.min(all_pts[:, 0]) - 0.5
    max_x = np.max(all_pts[:, 0]) + 0.5
    min_y = np.min(all_pts[:, 1]) - 0.5
    max_y = np.max(all_pts[:, 1]) + 0.5
    
    nx = int((max_x - min_x) / resolution)
    ny = int((max_y - min_y) / resolution)
    
    grid = np.ones((ny, nx), dtype=bool)
    for p in all_pts:
        gx = int((p[0] - min_x) / resolution)
        gy = int((p[1] - min_y) / resolution)
        if 0 <= gx < nx and 0 <= gy < ny:
            grid[gy, gx] = False
            
    dist_map = ndimage.distance_transform_edt(grid) * resolution
    
    l_wall = track_data["left_wall"]
    r_wall = track_data["right_wall"]
    N_raw = min(len(l_wall), len(r_wall))
    raw_mid = (l_wall[:N_raw] + r_wall[:N_raw]) / 2.0
    
    ridge_points = []
    for i in range(num_points):
        ref_pt = raw_mid[int(i * N_raw / num_points)]
        best_pt = ref_pt.copy()
        max_d = -1.0
        for dx in np.linspace(-0.35, 0.35, 17):
            for dy in np.linspace(-0.35, 0.35, 17):
                px = ref_pt[0] + dx
                py = ref_pt[1] + dy
                gx = int((px - min_x) / resolution)
                gy = int((py - min_y) / resolution)
                if 0 <= gx < nx and 0 <= gy < ny:
                    d = dist_map[gy, gx]
                    if d > max_d:
                        max_d = d
                        best_pt = np.array([px, py])
        ridge_points.append(best_pt)
        
    smooth_centerline = clean_and_resample_spline(ridge_points, num_points, sigma=3.5)
    
    return {
        "method": "Method 2: 2D Distance Field (SDF Ridge)",
        "centerline": smooth_centerline,
        "dist_map": dist_map,
        "grid_origin": (min_x, min_y),
        "resolution": resolution
    }

# ==============================================================================
#  METHOD 3: CROSS-SECTIONAL RAYCASTING & GATE MIDPOINTS (Recommended)
# ==============================================================================
def find_centerline_raycast_gates(track_data, num_points=360):
    """
    Computes obstacle-aware centerline by casting perpendicular left and right rays 
    against all walls and obstacle perimeters, calculating the exact midpoint of free space,
    and returning precise left/right drivable corridor widths.
    """
    l_wall = track_data["left_wall"]
    r_wall = track_data["right_wall"]
    obstacles = track_data.get("obstacles", [])
    
    N_raw = min(len(l_wall), len(r_wall))
    raw_pts = (l_wall[:N_raw] + r_wall[:N_raw]) / 2.0
    centerline = clean_and_resample_spline(raw_pts, num_points, sigma=1.0)
    
    w_left_arr = np.full(num_points, 0.50)
    w_right_arr = np.full(num_points, 0.50)
    
    for iteration in range(6):
        pts_wrap = np.vstack([centerline[-1:], centerline, centerline[:2]])
        dx = (pts_wrap[2:, 0] - pts_wrap[:-2, 0]) * 0.5
        dy = (pts_wrap[2:, 1] - pts_wrap[:-2, 1]) * 0.5
        norms = np.hypot(dx, dy) + 1e-6
        tangents = np.column_stack([dx / norms, dy / norms])
        normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])
        
        new_centerline = []
        
        for i in range(num_points):
            ci = centerline[i]
            ni = normals[i]
            
            left_hit_dist = 0.50
            right_hit_dist = 0.50
            
            for obs in obstacles:
                if obs["type"] == "wall_rectangle":
                    rx, ry = obs["center_x"], obs["center_y"]
                    rw, rh = obs["width"], obs["extrusion"]
                    
                    for r_dist in np.linspace(0.05, 0.70, 30):
                        p_left = ci + ni * r_dist
                        if (abs(p_left[0] - rx) <= rw/2.0) and (abs(p_left[1] - ry) <= rh/2.0):
                            left_hit_dist = min(left_hit_dist, r_dist)
                            
                        p_right = ci - ni * r_dist
                        if (abs(p_right[0] - rx) <= rw/2.0) and (abs(p_right[1] - ry) <= rh/2.0):
                            right_hit_dist = min(right_hit_dist, r_dist)
                            
                elif obs["type"] == "circle":
                    cx, cy, cr = obs["cx"], obs["cy"], obs["radius"]
                    for r_dist in np.linspace(0.05, 0.70, 30):
                        p_left = ci + ni * r_dist
                        if np.hypot(p_left[0] - cx, p_left[1] - cy) <= cr:
                            left_hit_dist = min(left_hit_dist, r_dist)
                            
                        p_right = ci - ni * r_dist
                        if np.hypot(p_right[0] - cx, p_right[1] - cy) <= cr:
                            right_hit_dist = min(right_hit_dist, r_dist)
            
            shift = (left_hit_dist - right_hit_dist) / 2.0
            mid_pt = ci + ni * shift
            new_centerline.append(mid_pt)
            
            w_left_arr[i] = left_hit_dist
            w_right_arr[i] = right_hit_dist
            
        centerline = np.array(new_centerline)
        centerline[:, 0] = gaussian_filter1d(centerline[:, 0], sigma=2.0, mode='wrap')
        centerline[:, 1] = gaussian_filter1d(centerline[:, 1], sigma=2.0, mode='wrap')

    final_centerline = clean_and_resample_spline(centerline, num_points, sigma=2.0)
    
    return {
        "method": "Method 3: Cross-Sectional Raycasting (Gate Midpoints)",
        "centerline": final_centerline,
        "corridor_width_left": w_left_arr,
        "corridor_width_right": w_right_arr
    }
