"""
Mathematical Solutions to Eliminate Corner-Transition Dips on Track 1:

Solution 1: Pure Spatial Bound Smoothing (Continuous Constraint Envelope)
Solution 3: Frenet Curvature Scale Correction
Hybrid: Solution 1 (Bound Smoothing) + Singularity-Clamped Frenet Scale Damping
"""

import numpy as np
import scipy.optimize as opt
import scipy.spatial as spatial
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import splprep, splev

from racing_engine.racing_line_optimizer import (
    compute_spline_properties,
    TUMMinimumCurvatureOptimizer,
    ROBOT_SAFETY_MARGIN,
    DEFAULT_MAX_SPEED,
    DEFAULT_FRICTION_COEFF
)

def compute_track_bounds(centerline, normals, track_data, safety_margin=ROBOT_SAFETY_MARGIN):
    """Computes min and max lateral bounds along normals"""
    N = len(centerline)
    l_wall = track_data["left_wall"]
    r_wall = track_data["right_wall"]
    all_barriers = np.vstack([l_wall, r_wall])
    
    for obs in track_data.get("obstacles", []):
        if obs["type"] == "wall_rectangle":
            cx, cy = obs["center_x"], obs["center_y"]
            w, ext = obs["width"], obs["extrusion"]
            pts_rect = np.array([
                [cx - w/2, cy - ext/2], [cx + w/2, cy - ext/2],
                [cx + w/2, cy + ext/2], [cx - w/2, cy + ext/2]
            ])
            all_barriers = np.vstack([all_barriers, pts_rect])
        elif obs["type"] == "circle":
            cx, cy, r = obs["cx"], obs["cy"], obs["radius"]
            angles = np.linspace(0, 2*np.pi, 16, endpoint=False)
            circle_pts = np.column_stack([cx + r*np.cos(angles), cy + r*np.sin(angles)])
            all_barriers = np.vstack([all_barriers, circle_pts])

    barrier_tree = spatial.cKDTree(all_barriers)
    min_bounds = np.zeros(N)
    max_bounds = np.zeros(N)
    
    test_alphas = np.linspace(-0.60, 0.60, 121)
    for i in range(N):
        pt = centerline[i]
        norm = normals[i]
        
        pos_limit = 0.50
        for a in test_alphas[test_alphas >= 0]:
            test_p = pt + norm * a
            d, _ = barrier_tree.query(test_p)
            if d < safety_margin:
                pos_limit = max(0.0, a - 0.02)
                break
        
        neg_limit = -0.50
        for a in test_alphas[test_alphas <= 0][::-1]:
            test_p = pt + norm * a
            d, _ = barrier_tree.query(test_p)
            if d < safety_margin:
                neg_limit = min(0.0, a + 0.02)
                break
                
        min_bounds[i] = neg_limit
        max_bounds[i] = pos_limit
        if min_bounds[i] >= max_bounds[i]:
            mid = (min_bounds[i] + max_bounds[i]) / 2.0
            min_bounds[i] = mid - 0.02
            max_bounds[i] = mid + 0.02
            
    return min_bounds, max_bounds, barrier_tree

def build_finite_difference_matrices(ds):
    N = len(ds)
    M2 = np.zeros((N, N))
    for i in range(N):
        ds_prev = ds[(i - 1) % N]
        ds_curr = ds[i]
        ds_avg = 0.5 * (ds_prev + ds_curr)
        M2[i, (i - 1) % N] = 1.0 / (ds_prev * ds_avg)
        M2[i, i] = -(1.0 / ds_prev + 1.0 / ds_curr) / ds_avg
        M2[i, (i + 1) % N] = 1.0 / (ds_curr * ds_avg)

    M1 = np.zeros((N, N))
    M3 = np.zeros((N, N))
    for i in range(N):
        M1[i, i] = -1.0
        M1[i, (i + 1) % N] = 1.0
        M3[i, (i - 2) % N] = -1.0
        M3[i, (i - 1) % N] = 2.0
        M3[i, (i + 1) % N] = -2.0
        M3[i, (i + 2) % N] = 1.0
        
    return M1, M2, M3

# ==============================================================================
#  OPTION A: Pure Solution 1 (Spatial Bound Smoothing)
# ==============================================================================
def solve_solution_1_bound_smoothing(track_data, voronoi_centerline_data):
    """
    Passes lateral bounds [alpha_min, alpha_max] through a continuous spatial 
    smoothing filter to eliminate knife-edge boundary step pinches.
    """
    centerline = voronoi_centerline_data["centerline"]
    N = len(centerline)
    props = compute_spline_properties(centerline)
    normals, kc, ds = props["normals"], props["curvature"], props["ds"]
    
    min_b, max_b, _ = compute_track_bounds(centerline, normals, track_data)
    
    # Smooth bounds to eliminate sharp step pinch points
    min_b_smooth = gaussian_filter1d(min_b, sigma=4.0, mode='wrap')
    max_b_smooth = gaussian_filter1d(max_b, sigma=4.0, mode='wrap')
    # Keep conservative (never breach physical walls)
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
            "description": f"Stage {s_idx+1}/10 ({pct}%): Bound Smoothing (Total {cumulative_iters} iters)"
        })
        
    tck, u = splprep([pts_stage[:, 0], pts_stage[:, 1]], s=0.015, per=True)
    final_pts = np.column_stack(splev(np.linspace(0, 1, N, endpoint=False), tck))
    final_props = compute_spline_properties(final_pts)
    
    return {
        "name": "Solution 1: Pure Bound Smoothing",
        "racing_line": final_pts,
        "iteration_history": iteration_history,
        "curvature": final_props["curvature"]
    }

# ==============================================================================
#  OPTION B: Hybrid Solution (Bound Smoothing + Singularity-Clamped Scale Damping)
# ==============================================================================
def solve_solution_hybrid_bound_and_scale(track_data, voronoi_centerline_data):
    """
    Combines Solution 1 (Bound Smoothing) with Singularity-Clamped Frenet Scale Damping:
    - Eliminates obstacle step-snagging (via bound smoothing)
    - Dampens large-offset overshoots on straights
    - Strictly prevents U-turn / hairpin singularities by clamping alpha * kc in [-0.35, +0.35]
    """
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
    
    iteration_history = [{"iter": 0, "stage": 0, "alpha": np.zeros(N), "points": centerline.copy(), "description": "Iteration 0: Medial Axis"}]
    cur_alpha = np.zeros(N)
    cumulative_iters = 0
    num_stages = 10
    
    for s_idx, w_step in enumerate(np.linspace(0.10, 1.0, num_stages)):
        # Singularity-Clamped Scale Factor:
        # Clamps alpha * kc into [-0.35, +0.35], keeping (1 - alpha * kc) safely between 0.65 and 1.35
        # This completely eliminates the U-turn / hairpin singularity!
        clamped_prod = np.clip(cur_alpha * kc_macro, -0.35, 0.35)
        w_scale = 1.0 / (1.0 - clamped_prod)
        
        M2_scaled = np.diag(w_scale) @ M2
        H = 2.0 * (M2_scaled.T @ W_ds @ M2_scaled) + 0.06 * (M1.T @ M1) + 0.40 * (M3.T @ M3) + 1e-5 * np.eye(N)
        f_step = 2.0 * (M2_scaled.T @ W_ds @ (kc_macro * w_scale)) * w_step
        
        def qp_obj(a):
            return 0.5 * a @ H @ a + f_step @ a, H @ a + f_step
        res = opt.minimize(qp_obj, cur_alpha, method='L-BFGS-B', jac=True, bounds=bounds, options={'maxiter': 600, 'ftol': 1e-9})
        cur_alpha = res.x
        cumulative_iters += res.nit
        pts_stage = centerline + normals * cur_alpha[:, None]
        pct = int((s_idx + 1) / num_stages * 100)
        iteration_history.append({
            "iter": cumulative_iters, "stage": s_idx + 1, "points": pts_stage.copy(),
            "description": f"Stage {s_idx+1}/10 ({pct}%): Hybrid Bound+Scale (Total {cumulative_iters} iters)"
        })
        
    tck, u = splprep([pts_stage[:, 0], pts_stage[:, 1]], s=0.015, per=True)
    final_pts = np.column_stack(splev(np.linspace(0, 1, N, endpoint=False), tck))
    final_props = compute_spline_properties(final_pts)
    
    return {
        "name": "Hybrid: Bound Smoothing + Clamped Scale",
        "racing_line": final_pts,
        "iteration_history": iteration_history,
        "curvature": final_props["curvature"]
    }
