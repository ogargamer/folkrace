#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - 135° MATRIX TOF SENSOR & BLINDSPOT RACING LINE SIMULATOR
================================================================================
Simulates 3x Matrix ToF sensors (135° FOV, 12 discrete zones, 1.5m range)
and computes 5 distinct racing line variants:
  1. [Cyan] Baseline Minimum Curvature (Pure geometric traction line)
  2. [Orange] Late-Apex Geometric Line (Asymmetric entry=0.78, exit=1.28)
  3. [Magenta] Active 135° LOS QP Line (QP penalty on occluded sightlines)
  4. [Yellow] ToF Horizon Velocity Line (Speed capped by 135° ToF stopping horizon)
  5. [Green] Full Combined Safe-Racing Line (Late-Apex + LOS + ToF Profile)
================================================================================
"""

import numpy as np
import scipy.spatial as spatial
from scipy.ndimage import gaussian_filter1d
import scipy.optimize as opt
from racing_engine.racing_line_optimizer import compute_spline_properties

TOF_FOV_DEG = 135.0
TOF_NUM_ZONES = 12
TOF_MAX_RANGE = 1.50   # 1.5 meters max range
BRAKE_DECEL = 4.50     # 4.5 m/s^2 emergency braking
STANDOFF_BUFFER = 0.20 # 20 cm safety cushion

def compute_corridor_bounds(centerline, barrier_tree, robot_safety_margin=0.10):
    N = len(centerline)
    props = compute_spline_properties(centerline)
    normals = props['normals']
    min_bounds = np.zeros(N)
    max_bounds = np.zeros(N)
    effective_margin = robot_safety_margin + 0.015
    test_alphas = np.linspace(-0.60, 0.60, 121)
    
    for i in range(N):
        pt = centerline[i]
        norm = normals[i]
        pts_line = pt + norm[None, :] * test_alphas[:, None]
        dists, _ = barrier_tree.query(pts_line)
        is_safe = dists >= effective_margin
        safe_indices = np.where(is_safe)[0]
        if len(safe_indices) == 0:
            best_idx = np.argmax(dists)
            best_a = test_alphas[best_idx]
            min_bounds[i], max_bounds[i] = best_a - 0.02, best_a + 0.02
        else:
            splits = np.where(np.diff(safe_indices) > 1)[0]
            segments = np.split(safe_indices, splits + 1)
            best_seg = None
            min_dist_to_zero = 999.0
            for seg in segments:
                a_start, a_end = test_alphas[seg[0]], test_alphas[seg[-1]]
                if a_start <= 0.0 <= a_end:
                    best_seg = (a_start, a_end)
                    break
                dist_zero = min(abs(a_start), abs(a_end))
                if dist_zero < min_dist_to_zero:
                    min_dist_to_zero = dist_zero
                    best_seg = (a_start, a_end)
            min_bounds[i], max_bounds[i] = best_seg[0], best_seg[1]
            if min_bounds[i] >= max_bounds[i]:
                mid = (min_bounds[i] + max_bounds[i]) / 2.0
                min_bounds[i] = mid - 0.02
                max_bounds[i] = mid + 0.02
    return min_bounds, max_bounds

def simulate_135deg_tof_frustum(pts, headings, barrier_tree, max_range=TOF_MAX_RANGE):
    """
    Simulates the 135-degree 12-zone ToF fan from every waypoint along a path.
    Returns:
      - zone_ranges: (N, 12) array of clear distance in each of the 12 zones
      - max_forward_sight: (N,) maximum clear forward distance along corridor
      - min_blindspot_dist: (N,) minimum clearance to occluding edge in lateral zones
    """
    N = len(pts)
    zone_angles = np.linspace(-np.radians(TOF_FOV_DEG / 2.0), np.radians(TOF_FOV_DEG / 2.0), TOF_NUM_ZONES)
    zone_ranges = np.full((N, TOF_NUM_ZONES), max_range)
    
    # Raycast steps along each beam (2cm resolution)
    test_dists = np.linspace(0.06, max_range, 72)
    
    for i in range(N):
        px, py = pts[i]
        theta = headings[i]
        
        for z_idx, d_ang in enumerate(zone_angles):
            ray_ang = theta + d_ang
            c_a, s_a = np.cos(ray_ang), np.sin(ray_ang)
            
            # Vectorized sample points along beam
            rx = px + c_a * test_dists
            ry = py + s_a * test_dists
            r_pts = np.column_stack([rx, ry])
            
            dists, _ = barrier_tree.query(r_pts)
            # Wall collision threshold: 0.05m from barrier surface
            hit_indices = np.where(dists < 0.05)[0]
            if len(hit_indices) > 0:
                hit_step = hit_indices[0]
                zone_ranges[i, z_idx] = test_dists[hit_step]
            else:
                zone_ranges[i, z_idx] = max_range

    # Forward sight is the clear view along central zones (zones 4 to 7)
    center_zones = zone_ranges[:, 4:8]
    max_forward_sight = np.max(center_zones, axis=1)
    
    # Lateral blindspot risk: minimum distance seen on left/right peripheral zones
    lateral_zones = np.column_stack([zone_ranges[:, :3], zone_ranges[:, 9:]])
    min_lateral_dist = np.min(lateral_zones, axis=1)
    
    return {
        "zone_ranges": zone_ranges,
        "forward_sight": max_forward_sight,
        "lateral_sight": min_lateral_dist
    }

def compute_tof_safe_velocity_profile(final_props, tof_data, friction_coeff=0.90, max_speed=4.0, max_accel=2.4, max_brake=4.5):
    """
    Computes velocity profile bounded by traction limits AND 135° ToF stopping sight distance.
    """
    N = len(final_props["ds"])
    ds = final_props["ds"]
    kappa = final_props["curvature"]
    fwd_sight = tof_data["forward_sight"]
    
    v_profile = np.full(N, max_speed)
    g = 9.81

    # 1. Cornering limit: v_corner = sqrt(mu * g / kappa)
    for i in range(N):
        k = max(abs(kappa[i]), 1e-4)
        v_corner = np.sqrt((friction_coeff * g) / k)
        v_profile[i] = min(max_speed, v_corner)

    # 2. ToF Stopping Sight Distance Limit: v_vis = sqrt(2 * a_brake * (d_sight - d_buffer))
    for i in range(N):
        d_clear = max(fwd_sight[i] - STANDOFF_BUFFER, 0.05)
        v_tof = np.sqrt(2.0 * BRAKE_DECEL * d_clear)
        v_profile[i] = min(v_profile[i], v_tof)

    # 3. Backward Pass (Braking limit)
    for _ in range(2):
        for i in reversed(range(N)):
            i_next = (i + 1) % N
            v_allowed = np.sqrt(v_profile[i_next]**2 + 2.0 * max_brake * ds[i])
            v_profile[i] = min(v_profile[i], v_allowed)

    # 4. Forward Pass (Acceleration limit)
    for _ in range(2):
        for i in range(N):
            i_prev = (i - 1) % N
            v_allowed = np.sqrt(v_profile[i_prev]**2 + 2.0 * max_accel * ds[i_prev])
            v_profile[i] = min(v_profile[i], v_allowed)

    # Lap Time
    lap_time = float(np.sum(ds / np.maximum(v_profile, 0.5)))
    min_apex_spd = float(np.min(v_profile))
    
    return {
        "velocities": v_profile,
        "lap_time": lap_time,
        "min_apex_speed": min_apex_spd,
        "min_forward_sight": float(np.min(fwd_sight))
    }

def solve_qp_variant(track, centerline, min_bounds, max_bounds, barrier_tree,
                     weight_curvature=3.20, weight_smoothness=0.055, weight_jerk=0.38,
                     entry_aggressiveness=1.0, exit_aggressiveness=1.0,
                     los_penalty_weight=0.0):
    """
    Generalized TUM QP solver that handles:
      - Baseline symmetric tuning
      - Late-apex asymmetric tuning
      - Active 135° Line-of-Sight (LOS) QP penalty
    """
    N = len(centerline)
    props = compute_spline_properties(centerline)
    ds = props['ds']
    kc = props['curvature']
    normals = props['normals']
    tangents = props['tangents']
    headings = np.arctan2(tangents[:, 1], tangents[:, 0])
    
    # Gaussian macro-curvature filter
    kc_macro = gaussian_filter1d(kc, sigma=4.0, mode='wrap')
    
    # Asymmetric Entry/Exit Aggressiveness
    if abs(entry_aggressiveness - 1.0) > 1e-3 or abs(exit_aggressiveness - 1.0) > 1e-3:
        d_abs_k = np.zeros(N)
        for i in range(N):
            i_prev = (i - 1) % N
            i_next = (i + 1) % N
            d_abs_k[i] = (abs(kc_macro[i_next]) - abs(kc_macro[i_prev])) / (ds[i] + ds[i_prev] + 1e-6)
        exit_mask = 1.0 / (1.0 + np.exp(d_abs_k * 5.0))
        local_agg = (1.0 - exit_mask) * entry_aggressiveness + exit_mask * exit_aggressiveness
    else:
        local_agg = 1.0

    # Build M1, M2, M3
    M2 = np.zeros((N, N))
    M1 = np.zeros((N, N))
    M3 = np.zeros((N, N))
    for i in range(N):
        ds_p = ds[(i - 1) % N]
        ds_c = ds[i]
        ds_avg = 0.5 * (ds_p + ds_c)
        M2[i, (i - 1) % N] = 1.0 / (ds_p * ds_avg)
        M2[i, i] = -(1.0 / ds_p + 1.0 / ds_c) / ds_avg
        M2[i, (i + 1) % N] = 1.0 / (ds_c * ds_avg)

        M1[i, i] = -1.0
        M1[i, (i + 1) % N] = 1.0

        M3[i, (i - 2) % N] = -1.0
        M3[i, (i - 1) % N] = 2.0
        M3[i, (i + 1) % N] = -2.0
        M3[i, (i + 2) % N] = 1.0

    W_ds = np.diag(ds)
    H = (
        weight_curvature * (M2.T @ W_ds @ M2)
        + weight_smoothness * (M1.T @ M1)
        + weight_jerk * (M3.T @ M3)
        + 1e-5 * np.eye(N)
    )

    f_base = weight_curvature * (M2.T @ W_ds @ (kc_macro * local_agg))

    # Active 135° LOS Blindspot Penalty
    if los_penalty_weight > 0.0:
        tof_initial = simulate_135deg_tof_frustum(centerline, headings, barrier_tree)
        fwd_sight = tof_initial["forward_sight"]
        
        # Where forward sight drops below 1.2m on a turn, penalize hugging the inside wall
        los_cost = np.zeros(N)
        for i in range(N):
            if fwd_sight[i] < 1.20 and abs(kc_macro[i]) > 0.30:
                # kc_macro > 0 is left turn (inside is left, alpha > 0 pushes right/outside)
                # kc_macro < 0 is right turn (inside is right, alpha < 0 pushes left/outside)
                push_dir = -np.sign(kc_macro[i])
                blindness_severity = (1.20 - fwd_sight[i]) / 1.20
                los_cost[i] = push_dir * blindness_severity * los_penalty_weight
                
        f_base = f_base + los_cost

    bounds = list(zip(min_bounds, max_bounds))
    num_stages = 7
    weights_prog = np.linspace(0.10, 1.0, num_stages)
    cur_alpha = np.zeros(N)

    for step_idx, w_step in enumerate(weights_prog):
        f_step = f_base * w_step
        def qp_obj(a):
            return 0.5 * a @ H @ a + f_step @ a, H @ a + f_step

        res = opt.minimize(qp_obj, cur_alpha, method='L-BFGS-B', jac=True, bounds=bounds,
                           options={'maxiter': 300, 'ftol': 1e-9})
        cur_alpha = res.x

    # Reconstruct 2D trajectory points
    raw_pts = centerline + normals * cur_alpha[:, None]
    from scipy.interpolate import splprep, splev
    tck, _ = splprep([raw_pts[:, 0], raw_pts[:, 1]], s=0.005, per=True)
    u_fine = np.linspace(0, 1, N, endpoint=False)
    final_pts = np.column_stack(splev(u_fine, tck))
    final_props = compute_spline_properties(final_pts)
    
    # Simulate 135° ToF from final trajectory
    final_headings = np.arctan2(final_props["tangents"][:, 1], final_props["tangents"][:, 0])
    final_props["heading"] = final_headings
    tof_res = simulate_135deg_tof_frustum(final_pts, final_headings, barrier_tree)

    return {
        "racing_line": final_pts,
        "alpha": cur_alpha,
        "props": final_props,
        "tof": tof_res
    }
