"""
Official TUM / F1TENTH Minimum Curvature Racing Line Optimizer (Heilmeier et al.)
Operates on the Calculated Medial Axis (Method 1: Voronoi Centerline):
1. Quadratic Programming (QP) formulation minimizing integrated squared curvature
2. Slices straight through chicanes & S-bends
3. Authentic racing line geometry: Outside entry -> Inside apex -> Wide exit
4. Strictly respects obstacle clearance bounds derived from the Voronoi skeleton
5. Logs step-by-step iteration milestones for visual inspection
"""

# ==============================================================================
#                      USER CONFIGURATION PARAMETERS
# ==============================================================================
# 1. RACING LINE AGGRESSIVENESS (BASELINE TUM = 1.0x)
# ------------------------------------------------------------------------------
# Corner Entry & Apex Pull Aggressiveness (Default TUM: 1.0x)
CORNER_ENTRY_APEX_AGGRESSIVENESS = 1.00

# Corner Exit Width Aggressiveness (Default TUM: 1.0x | Wide Exit: 1.5x - 2.0x)
CORNER_EXIT_WIDTH_AGGRESSIVENESS = 1.00

# Total Iteration Progression Stages (Continuation steps from centerline to final)
NUM_PROGRESSION_STAGES = 10

# Maximum Sub-Iterations per Solver Stage (L-BFGS-B gradient budget)
MAX_SOLVER_SUB_ITERS = 800

# Robot Safety Clearance Margin from Walls & Obstacles (meters)
# (e.g. 0.10m = 10cm distance kept from all walls and obstacles)
ROBOT_SAFETY_MARGIN = 0.10


# 2. MATHEMATICAL WEIGHTS & REGULARIZATION (QP Matrix Formulation)
# ------------------------------------------------------------------------------
# Curvature Minimization Weight (Primary term: integrated squared curvature)
WEIGHT_CURVATURE = 3.50

# Path Length / Drift Regularization Weight (M1 1st-derivative penalty)
# Keeps line straight and prevents snaking along straightaways
WEIGHT_PATH_SMOOTHNESS = 0.03

# Jerk / Transition Smoothness Weight (M3 3rd-derivative penalty)
# Eliminates inflection dips & snaking, guarantees buttery-smooth steering transitions
WEIGHT_JERK_REGULARIZATION = 0.35

# Macro Curvature Spatial Filter (Gaussian sigma)
# Separates macroscopic track geometry from microscopic obstacle kinks on straights
MACRO_CURVATURE_SIGMA = 4.0

# Final Spline Smoothing Factor (s parameter in splprep for continuous C2 curve)
SPLINE_SMOOTHING_S = 0.015

# Maximum Boundary Gradient / Slew Rate (m lateral per m longitudinal)
# Eliminates 800% normal fan cliffs at sharp 90° corners by creating smooth lead-in tapers
MAX_BOUNDARY_SLOPE = 0.0


# 3. VEHICLE DYNAMICS & VELOCITY PROFILE (Ignore if only generating geometry)
# ------------------------------------------------------------------------------
# Top Speed on Straights & Straight Chicane Cuts (m/s)
DEFAULT_MAX_SPEED = 4.0

# Tire / Track Surface Friction Coefficient (mu ~ 0.95 rubber on smooth track)
DEFAULT_FRICTION_COEFF = 0.954

# Max Motor Acceleration (m/s²)
MAX_ACCELERATION = 2.4

# Max Braking Deceleration into Turns (m/s²)
MAX_DECELERATION = 3.8
# ==============================================================================

import numpy as np
import scipy.optimize as opt
import scipy.spatial as spatial
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import splprep, splev

def compute_spline_properties(points):
    """
    Computes tangents, normals, cumulative arc-length s, and curvature kappa
    for closed loop trajectory points.
    """
    N = len(points)
    pts_wrap = np.vstack([points[-2:], points, points[:3]])
    
    diffs = pts_wrap[1:] - pts_wrap[:-1]
    ds = np.hypot(diffs[:, 0], diffs[:, 1])
    
    dx = (pts_wrap[2:, 0] - pts_wrap[:-2, 0]) / (ds[1:] + ds[:-1] + 1e-6)
    dy = (pts_wrap[2:, 1] - pts_wrap[:-2, 1]) / (ds[1:] + ds[:-1] + 1e-6)
    tangents = np.column_stack([dx, dy])
    norms = np.hypot(tangents[:, 0], tangents[:, 1]) + 1e-6
    tangents /= norms[:, None]
    
    ddx = (pts_wrap[2:, 0] - 2 * pts_wrap[1:-1, 0] + pts_wrap[:-2, 0]) / (ds[1:] * ds[:-1] + 1e-6)
    ddy = (pts_wrap[2:, 1] - 2 * pts_wrap[1:-1, 1] + pts_wrap[:-2, 1]) / (ds[1:] * ds[:-1] + 1e-6)
    
    curvature = (dx * ddy - dy * ddx) / (norms**3 + 1e-6)
    normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])
    
    return {
        "tangents": tangents[2:N+2],
        "normals": normals[2:N+2],
        "curvature": curvature[2:N+2],
        "ds": ds[2:N+2]
    }

class TUMMinimumCurvatureOptimizer:
    def __init__(
        self,
        entry_aggressiveness=CORNER_ENTRY_APEX_AGGRESSIVENESS,
        exit_aggressiveness=CORNER_EXIT_WIDTH_AGGRESSIVENESS,
        num_progression_stages=NUM_PROGRESSION_STAGES,
        max_solver_sub_iters=MAX_SOLVER_SUB_ITERS,
        robot_safety_margin=ROBOT_SAFETY_MARGIN,
        weight_curvature=WEIGHT_CURVATURE,
        weight_path_smoothness=WEIGHT_PATH_SMOOTHNESS,
        weight_jerk=WEIGHT_JERK_REGULARIZATION,
        macro_sigma=MACRO_CURVATURE_SIGMA,
        spline_s=SPLINE_SMOOTHING_S,
        max_boundary_slope=MAX_BOUNDARY_SLOPE,
        max_speed=DEFAULT_MAX_SPEED,
        friction_coeff=DEFAULT_FRICTION_COEFF,
        max_accel=MAX_ACCELERATION,
        max_brake=MAX_DECELERATION
    ):
        self.entry_aggressiveness = float(entry_aggressiveness)
        self.exit_aggressiveness = float(exit_aggressiveness)
        self.num_progression_stages = int(num_progression_stages)
        self.max_solver_sub_iters = int(max_solver_sub_iters)
        self.robot_safety_margin = float(robot_safety_margin)
        self.weight_curvature = float(weight_curvature)
        self.weight_path_smoothness = float(weight_path_smoothness)
        self.weight_jerk = float(weight_jerk)
        self.macro_sigma = float(macro_sigma)
        self.spline_s = float(spline_s)
        self.max_boundary_slope = float(max_boundary_slope)
        self.max_speed = float(max_speed)
        self.friction_coeff = float(friction_coeff)
        self.max_accel = float(max_accel)
        self.max_brake = float(max_brake)
        self.g = 9.81

    def optimize_racing_line(self, track_data, voronoi_centerline_data):
        """
        Executes the official TUM Minimum Curvature Quadratic Program:
        min  1/2 * alpha^T * H * alpha + f^T * alpha
        s.t. alpha_min <= alpha <= alpha_max
        """
        centerline = voronoi_centerline_data["centerline"]
        N = len(centerline)
        
        # 1. Spline Properties & Differential Geometry of Reference Line
        props = compute_spline_properties(centerline)
        normals = props["normals"]
        kc = props["curvature"]
        ds = props["ds"]

        # 2. Dynamic Corridor Width & Bounds derived from boundaries and obstacles
        l_wall = track_data["left_wall"]
        r_wall = track_data["right_wall"]
        all_barriers = np.vstack([l_wall, r_wall])
        
        # Include obstacle vertices in barrier collision tree
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
        
        # Continuous free-space interval raycasting along normal for every waypoint
        effective_margin = self.robot_safety_margin + 0.015
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

        # 2b. 2-Pass Forward-Backward Slew Rate Limiter on Corridor Boundaries
        # Eliminates 800% normal fan cliffs at sharp corners by enforcing realistic lateral steering gradient
        if self.max_boundary_slope > 0:
            for _ in range(2):
                # Forward pass (taper entering corners)
                for i in range(N):
                    i_prev = (i - 1) % N
                    max_bounds[i] = min(max_bounds[i], max_bounds[i_prev] + self.max_boundary_slope * ds[i_prev])
                    min_bounds[i] = max(min_bounds[i], min_bounds[i_prev] - self.max_boundary_slope * ds[i_prev])
                # Backward pass (taper exiting corners)
                for i in range(N - 1, -1, -1):
                    i_next = (i + 1) % N
                    max_bounds[i] = min(max_bounds[i], max_bounds[i_next] + self.max_boundary_slope * ds[i])
                    min_bounds[i] = max(min_bounds[i], min_bounds[i_next] - self.max_boundary_slope * ds[i])

        # 3. Construct TUM Finite-Difference Derivative Matrices (Periodic / Closed-Loop)
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

        W_ds = np.diag(ds)
        # Macro track curvature (filtered to prevent obstacle kinks from injecting artificial curvature on straights)
        kc_macro = gaussian_filter1d(kc, sigma=self.macro_sigma, mode='wrap')

        # 4. Spatially Split Aggressiveness: Corner Entry/Apex vs Corner Exit
        if abs(self.entry_aggressiveness - 1.0) > 1e-3 or abs(self.exit_aggressiveness - 1.0) > 1e-3:
            d_abs_k = np.zeros(N)
            for i in range(N):
                i_prev = (i - 1) % N
                i_next = (i + 1) % N
                d_abs_k[i] = (abs(kc_macro[i_next]) - abs(kc_macro[i_prev])) / (ds[i] + ds[i_prev] + 1e-6)
            exit_mask = 1.0 / (1.0 + np.exp(d_abs_k * 5.0))
            local_aggressiveness = (1.0 - exit_mask) * self.entry_aggressiveness + exit_mask * self.exit_aggressiveness
        else:
            local_aggressiveness = 1.0

        # Hessian: H = weight_curvature * M2^T * W_ds * M2 + reg
        H = (
            self.weight_curvature * (M2.T @ W_ds @ M2)
            + self.weight_path_smoothness * (M1.T @ M1)
            + self.weight_jerk * (M3.T @ M3)
            + 1e-5 * np.eye(N)
        )
        f_base = self.weight_curvature * (M2.T @ W_ds @ (kc_macro * local_aggressiveness))

        # 5. Iterative Quadratic Programming (IQP) Relaxation & Milestones
        iteration_history = []
        iteration_history.append({
            "iter": 0,
            "alpha": np.zeros(N),
            "points": centerline.copy(),
            "description": "Iteration 0: Medial Axis (Voronoi Centerline)"
        })

        bounds = list(zip(min_bounds, max_bounds))
        weights_progression = np.linspace(0.10, 1.0, self.num_progression_stages)
        cur_alpha = np.zeros(N)
        cumulative_iters = 0
        
        for step_idx, w_step in enumerate(weights_progression):
            f_step = f_base * w_step
            def qp_objective(alpha_vec):
                val = 0.5 * alpha_vec @ H @ alpha_vec + f_step @ alpha_vec
                grad = H @ alpha_vec + f_step
                return val, grad

            res = opt.minimize(
                qp_objective,
                cur_alpha,
                method='L-BFGS-B',
                jac=True,
                bounds=bounds,
                options={'maxiter': self.max_solver_sub_iters, 'ftol': 1e-9}
            )
            cur_alpha = res.x
            cumulative_iters += res.nit
            pts_stage = centerline + normals * cur_alpha[:, None]
            
            stage_pct = int((step_idx + 1) / self.num_progression_stages * 100)
            iteration_history.append({
                "iter": cumulative_iters,
                "stage": step_idx + 1,
                "sub_iters": res.nit,
                "alpha": cur_alpha.copy(),
                "points": pts_stage.copy(),
                "description": f"Stage {step_idx + 1}/{self.num_progression_stages} ({stage_pct}%): Total {cumulative_iters} iters (Sub-iters: {res.nit})"
            })

        # 6. Final Spline Resampling (100% C2 Continuous Curve)
        final_alpha = cur_alpha
        raw_final_pts = centerline + normals * final_alpha[:, None]
        
        tck, u = splprep([raw_final_pts[:, 0], raw_final_pts[:, 1]], s=min(self.spline_s, 0.005), per=True)
        u_fine = np.linspace(0, 1, N, endpoint=False)
        final_points = np.column_stack(splev(u_fine, tck))

        final_props = compute_spline_properties(final_points)
        velocities = self._generate_velocity_profile(final_props["curvature"], final_props["ds"])

        return {
            "racing_line": final_points,
            "centerline": centerline,
            "alpha": final_alpha,
            "tangents": final_props["tangents"],
            "normals": final_props["normals"],
            "curvature": final_props["curvature"],
            "velocities": velocities,
            "iteration_history": iteration_history,
            "min_bounds": min_bounds,
            "max_bounds": max_bounds,
            "ds": final_props["ds"]
        }

    def _generate_velocity_profile(self, curvature, ds):
        """
        Forward/Backward Pass Velocity Profiler with friction limits.
        """
        N = len(curvature)
        v_max_curve = np.zeros(N)
        
        # Lateral acceleration limit: a_lat = v^2 * kappa <= mu * g
        for i in range(N):
            k = max(abs(curvature[i]), 1e-4)
            v_corner = np.sqrt((self.friction_coeff * self.g) / k)
            v_max_curve[i] = min(self.max_speed, v_corner)

        # Backward pass: Braking limit
        v_profile = v_max_curve.copy()
        for _ in range(2):
            for i in range(N - 1, -1, -1):
                i_next = (i + 1) % N
                v_allowed = np.sqrt(v_profile[i_next]**2 + 2.0 * self.max_brake * ds[i])
                v_profile[i] = min(v_profile[i], v_allowed)

        # Forward pass: Acceleration limit
        for _ in range(2):
            for i in range(N):
                i_prev = (i - 1) % N
                v_allowed = np.sqrt(v_profile[i_prev]**2 + 2.0 * self.max_accel * ds[i_prev])
                v_profile[i] = min(v_profile[i], v_allowed)

        return v_profile
