"""
================================================================================
  RACING-LINE-CENTRIC 12-CHANNEL ToF PATHFINDING ARCHITECTURE SUITE
  Optimized for Folkrace 4WD Robotic Car with 3x VL53L8CX (12 Zones, 150° FOV)
================================================================================
  Architectures Implemented:
    1. Option 1: FrenetCorridorQP (1D-QP Frenet Micro-Corridor + Stanley Tracker)
    2. Option 2: GaussianElasticBand (GEB Analytical Line Deformation)
    3. Option 3: FrenetLatticePlanner (Spatiotemporal Quintic Polynomial Lattice)
    4. Option 4: BiasedDisparityExtender (Polar Disparity Extender with Line Target)
    5. Non-Blocking Recovery Manager & Wrong-Way Lap Prevention
================================================================================
"""

import math
import numpy as np

# --- 12-CHANNEL ToF SENSOR GEOMETRY ---
# 3x VL53L8CX sensors mounted across front bumper
# 4 horizontal zones per sensor = 12 zones total (150° span)
# Zone 0: +68.75° (Far Left) ... Zone 5: +6.25° ... Zone 6: -6.25° ... Zone 11: -68.75° (Far Right)
NUM_TOF_ZONES = 12
ZONE_FOV_DEG = 12.5
ZONE_FOV_RAD = math.radians(ZONE_FOV_DEG)
TOTAL_SPAN_RAD = math.radians(150.0)

TOF_ZONE_ANGLES = np.array([
    math.radians(+68.75), math.radians(+56.25), math.radians(+43.75), math.radians(+31.25),
    math.radians(+18.75), math.radians(+6.25),  math.radians(-6.25),  math.radians(-18.75),
    math.radians(-31.25), math.radians(-43.75), math.radians(-56.25), math.radians(-68.75)
])

TOF_MAX_RANGE_M = 1.50
TOF_MIN_RANGE_M = 0.02
HALF_CAR_WIDTH_M = 0.075  # 75mm half-width
SAFETY_MARGIN_M = 0.05   # 50mm safety buffer


# ==============================================================================
# FRENET FRAME COORDINATE TRANSFORMER HELPER
# ==============================================================================
class FrenetTransformer:
    """
    High-performance converter between Global Cartesian (x, y) and Frenet (s, d)
    frames along the precalculated champion trajectory.
    """
    def __init__(self, traj_s, traj_x, traj_y, traj_yaw, traj_k):
        self.traj_s = np.ascontiguousarray(traj_s)
        self.traj_x = np.ascontiguousarray(traj_x)
        self.traj_y = np.ascontiguousarray(traj_y)
        self.traj_yaw = np.ascontiguousarray(traj_yaw)
        self.traj_k = np.ascontiguousarray(traj_k)
        self.n_pts = len(traj_s)
        self.total_length = traj_s[-1]

    def cartesian_to_frenet(self, x, y, guess_idx=0, search_window=160):
        """
        Projects point (x, y) onto the closest trajectory station.
        Returns: (s, d, idx, t_yaw, t_k)
          s: Arc-length along trajectory (m)
          d: Signed cross-track distance (m, +Left / -Right)
        """
        indices = [(guess_idx + offset) % self.n_pts for offset in range(-search_window // 4, search_window)]
        d_sq = (self.traj_x[indices] - x)**2 + (self.traj_y[indices] - y)**2
        best_local = int(np.argmin(d_sq))
        idx = indices[best_local]

        tx, ty = self.traj_x[idx], self.traj_y[idx]
        t_yaw = self.traj_yaw[idx]
        t_k = self.traj_k[idx]
        s = self.traj_s[idx]

        dx = x - tx
        dy = y - ty
        # Signed lateral offset (+Left, -Right)
        d = -dx * math.sin(t_yaw) + dy * math.cos(t_yaw)

        return s, d, idx, t_yaw, t_k

    def cartesian_to_frenet_batch(self, xs, ys, guess_idx=0, search_window=60):
        """
        Vectorized batch projection of multiple (x, y) points onto trajectory in < 0.1 ms.
        """
        offsets = np.arange(-search_window // 4, search_window)
        indices = (guess_idx + offsets) % self.n_pts
        
        tx_sub = self.traj_x[indices, None]
        ty_sub = self.traj_y[indices, None]
        
        d_sq = (tx_sub - xs[None, :])**2 + (ty_sub - ys[None, :])**2
        best_local = np.argmin(d_sq, axis=0)
        best_indices = indices[best_local]
        
        tx = self.traj_x[best_indices]
        ty = self.traj_y[best_indices]
        t_yaw = self.traj_yaw[best_indices]
        s_arr = self.traj_s[best_indices]
        
        dx = xs - tx
        dy = ys - ty
        d_arr = -dx * np.sin(t_yaw) + dy * np.cos(t_yaw)
        return s_arr, d_arr

    def frenet_to_cartesian(self, s, d, base_idx=0):
        """
        Converts (s, d) into Cartesian (x, y) world coordinates.
        """
        idx = int(base_idx) % self.n_pts
        tx, ty = self.traj_x[idx], self.traj_y[idx]
        t_yaw = self.traj_yaw[idx]

        # Normal vector pointing to the left
        nx = -math.sin(t_yaw)
        ny = math.cos(t_yaw)

        x = tx + d * nx
        y = ty + d * ny
        return x, y


FOV_HALF_ANGLE_RAD = math.radians(6.25) # 12.5 deg FoV cone half-angle
TOF_FRONT_OFFSET_M = 0.090 # 9.0 cm forward of vehicle center (at front bumper)

def extract_36_point_tof_cloud(rx, ry, ryaw, tof_distances, frenet_helper, curr_idx):
    """
    Expands 12 raw ToF distance readings into dense obstacle boundary points
    emanating from the front bumper (TOF_FRONT_OFFSET_M) using vectorized batch projection.
    Applies a forward sector filter (+/- 45 deg) to ignore parallel sideways wall glances.
    """
    curr_s, curr_d, _, _, _ = frenet_helper.cartesian_to_frenet(rx, ry, curr_idx)
    min_dist = float(np.min(tof_distances))

    front_x = rx + TOF_FRONT_OFFSET_M * math.cos(ryaw)
    front_y = ry + TOF_FRONT_OFFSET_M * math.sin(ryaw)

    ox_list = []
    oy_list = []
    dist_list = []

    for i in range(1, 11): # Zones 1 to 10
        dist = tof_distances[i]
        if dist >= TOF_MAX_RANGE_M or dist <= TOF_MIN_RANGE_M:
            continue
        
        # Sector filter: Ignore purely sideways wall glancing rays (> 48 deg)
        if abs(TOF_ZONE_ANGLES[i]) > math.radians(48.0):
            continue

        center_yaw = ryaw + TOF_ZONE_ANGLES[i]
        cone_angle = min(FOV_HALF_ANGLE_RAD, math.atan2(0.040, max(dist, 0.08)))

        for d_ang in (-cone_angle, 0.0, +cone_angle):
            beam_yaw = center_yaw + d_ang
            ox_list.append(front_x + dist * math.cos(beam_yaw))
            oy_list.append(front_y + dist * math.sin(beam_yaw))
            dist_list.append(dist)

    if not ox_list:
        return [], min_dist

    xs = np.array(ox_list)
    ys = np.array(oy_list)
    s_arr, d_arr = frenet_helper.cartesian_to_frenet_batch(xs, ys, curr_idx)

    obs_frenet = []
    for k in range(len(s_arr)):
        d_obs = float(d_arr[k])
        # Only consider obstacles within the drivable envelope
        if abs(d_obs) <= 0.32:
            ds_ahead = float(s_arr[k] - curr_s)
            if ds_ahead < 0:
                ds_ahead += frenet_helper.total_length
            obs_frenet.append((ds_ahead, d_obs, dist_list[k]))

    return obs_frenet, min_dist


# ==============================================================================
# OPTION 1: HIGH-RESOLUTION 1D-QP FRENET MICRO-CORRIDOR (60 NODES, 1-2 MS)
# ==============================================================================
class FrenetCorridorQP:
    """
    High-Resolution 1D-QP Frenet Micro-Corridor Planner (60 Nodes, 1.5m Horizon, 1-2 ms):
    1. Static Wall vs Dynamic Obstacle Filtering (Fix 1)
    2. Coupled Speed-Curvature Friction Governor (Fix 2)
    3. Speed-Adaptive Lookahead Horizon (Fix 3)
    """
    def __init__(self, num_nodes=60, node_spacing_m=0.025, w_line=1.0, w_smooth=24.0):
        self.num_nodes = num_nodes
        self.ds = node_spacing_m
        self.w_line = w_line
        self.w_smooth = w_smooth

    def plan(self, rx, ry, ryaw, tof_distances, frenet_helper, curr_idx, current_steer_rad=0.0, v_mps=2.0):
        curr_s, curr_d, _, t_yaw, t_k = frenet_helper.cartesian_to_frenet(rx, ry, curr_idx)

        # 1. Build dense corridor bounds with Static Wall Filtering (Fix 1)
        d_min = np.full(self.num_nodes, -0.38)
        d_max = np.full(self.num_nodes, +0.38)
        obs_cloud, min_corridor_dist = extract_36_point_tof_cloud(rx, ry, ryaw, tof_distances, frenet_helper, curr_idx)

        safety_buf = HALF_CAR_WIDTH_M + SAFETY_MARGIN_M
        for ds_ahead, d_obs, dist in obs_cloud:
            if 0.02 < ds_ahead < (self.num_nodes * self.ds + 0.10):
                node_idx = min(int(round(ds_ahead / self.ds)), self.num_nodes - 1)
                # FIX 1: Only constrict corridor for obstacles encroaching inside the active racing lane (|d_obs| <= 0.16m)
                # Static outer/inner boundary walls (|d_obs| > 0.16m) are already safe via the racing line
                if abs(d_obs) <= 0.16:
                    if d_obs > 0:
                        d_max[node_idx] = min(d_max[node_idx], d_obs - safety_buf)
                    else:
                        d_min[node_idx] = max(d_min[node_idx], d_obs + safety_buf)

        # Enforce valid corridor bounds
        for k in range(self.num_nodes):
            if d_min[k] > d_max[k]:
                mid = 0.5 * (d_min[k] + d_max[k])
                d_min[k] = mid - 0.03
                d_max[k] = mid + 0.03

        # 2. Initial Physical Steering Derivative
        heading_err = (ryaw - t_yaw + math.pi) % (2.0 * math.pi) - math.pi
        d_prime_0 = math.tan(heading_err)

        # 3. Solve 60-Node Banded QP via Projected Gauss-Seidel with SOR (60 iterations)
        d_opt = np.clip(np.zeros(self.num_nodes), d_min, d_max)
        d_opt[0] = np.clip(curr_d, d_min[0], d_max[0])
        d_opt[1] = np.clip(curr_d + d_prime_0 * self.ds, d_min[1], d_max[1])
        omega_sor = 1.35

        for _ in range(60):
            for k in range(2, self.num_nodes - 1):
                d_prev = d_opt[k - 1]
                d_next = d_opt[k + 1]
                target_smooth = 0.5 * (d_prev + d_next)
                d_target_val = (self.w_smooth * target_smooth) / (self.w_line + self.w_smooth)
                d_new = (1.0 - omega_sor) * d_opt[k] + omega_sor * d_target_val
                d_opt[k] = np.clip(d_new, d_min[k], d_max[k])

        d_opt[-1] = d_opt[-2]

        # FIX 3: Speed-Adaptive Lookahead Horizon (180ms preview time)
        t_preview_s = 0.18
        v_eff = max(abs(v_mps), 1.0)
        look_dist = max(0.15, min(0.65, v_eff * t_preview_s))
        look_node = min(int(round(look_dist / self.ds)), self.num_nodes - 1)
        target_d = float(d_opt[look_node])
        feedforward_k = float(t_k)

        return target_d, feedforward_k, min_corridor_dist


# ==============================================================================
# OPTION 2: MULTI-KERNEL GAUSSIAN ELASTIC BAND (1 MS)
# ==============================================================================
class GaussianElasticBand:
    """
    Multi-Kernel Gaussian Elastic Band (GEB, 1 ms):
    Superimposes Gaussian repulsive potentials generated by front-bumper ToF points.
    Only active for obstacles directly encroaching on the drivable corridor (|d_obs| < 0.26m).
    """
    def __init__(self, sigma_m=0.35, max_push_m=0.32):
        self.sigma = sigma_m
        self.max_push = max_push_m

    def plan(self, rx, ry, ryaw, tof_distances, frenet_helper, curr_idx, current_steer_rad=0.0):
        curr_s, curr_d, _, _, _ = frenet_helper.cartesian_to_frenet(rx, ry, curr_idx)
        obs_cloud, min_dist = extract_36_point_tof_cloud(rx, ry, ryaw, tof_distances, frenet_helper, curr_idx)
        total_offset = 0.0

        for ds_ahead, d_obs, dist in obs_cloud:
            if 0.04 < ds_ahead < 1.20 and dist < 0.65 and abs(d_obs) < 0.26:
                spatial_weight = math.exp(-(ds_ahead**2) / (2.0 * self.sigma**2))
                push_dir = -1.0 if d_obs > 0 else +1.0
                intensity = max(0.0, (0.65 - dist) / 0.65)
                offset_i = push_dir * self.max_push * intensity * spatial_weight
                total_offset += offset_i

        target_d = float(np.clip(total_offset, -0.35, +0.35))
        return target_d, min_dist


# ==============================================================================
# OPTION 3: 105-CANDIDATE FRENET SPATIOTEMPORAL LATTICE (1-2 MS)
# ==============================================================================
class FrenetLatticePlanner:
    """
    105-Candidate Frenet Spatiotemporal Lattice Planner (1-2 ms):
    Evaluates 105 candidate quintic polynomial trajectories anchored to the
    physical steering servo state. Evaluates collision against true corridor
    obstacles while allowing clean apex clipping.
    """
    def __init__(self, num_lat_samples=15, wheelbase_m=0.1325, max_steer_rate_rad_s=10.47):
        self.num_lat_samples = num_lat_samples
        self.wheelbase = wheelbase_m
        self.max_steer_rate = max_steer_rate_rad_s
        self.horizons = [0.50, 0.70, 0.90, 1.10, 1.30, 1.50, 1.80]

    def plan(self, rx, ry, ryaw, tof_distances, frenet_helper, curr_idx, current_steer_rad=0.0, v_mps=2.5):
        curr_s, curr_d, _, t_yaw, t_k = frenet_helper.cartesian_to_frenet(rx, ry, curr_idx)

        # 1. Physical Initial State Boundary Conditions
        heading_err = (ryaw - t_yaw + math.pi) % (2.0 * math.pi) - math.pi
        d_prime_0 = math.tan(heading_err)
        kappa_vehicle = math.tan(current_steer_rad) / self.wheelbase
        d_pprime_0 = (1.0 + d_prime_0**2) * kappa_vehicle - t_k

        # 2. Extract front-bumper obstacle points
        obs_cloud, min_dist = extract_36_point_tof_cloud(rx, ry, ryaw, tof_distances, frenet_helper, curr_idx)

        # 3. Evaluate 105 candidate trajectories across 7 horizons and 15 lateral targets
        d_targets = np.linspace(-0.35, +0.35, self.num_lat_samples)
        best_cost = 1e9
        best_d = 0.0
        best_k = t_k

        a0 = curr_d
        a1 = d_prime_0
        a2 = 0.5 * d_pprime_0

        for L in self.horizons:
            delta_v = 0.0 - (a1 + 2.0 * a2 * L)
            delta_a = 0.0 - (2.0 * a2)

            for d_t in d_targets:
                delta_d = d_t - (a0 + a1 * L + a2 * (L**2))
                a3 = (10.0 * delta_d - 4.0 * delta_v * L + 0.5 * delta_a * (L**2)) / (L**3)
                a4 = (-15.0 * delta_d + 7.0 * delta_v * L - delta_a * (L**2)) / (L**4)
                a5 = (6.0 * delta_d - 3.0 * delta_v * L + 0.5 * delta_a * (L**2)) / (L**5)

                # Actuator rate check at lead evaluation station (s_eval = 0.15m)
                s_eval = min(0.20, L * 0.25)
                d_eval_pp = 2.0 * a2 + 6.0 * a3 * s_eval + 12.0 * a4 * (s_eval**2) + 20.0 * a5 * (s_eval**3)
                req_k = float(t_k + d_eval_pp)
                req_steer = math.atan(self.wheelbase * req_k)
                dt_lead = max(s_eval / max(v_mps, 1.0), 0.02)
                steer_rate_req = abs(req_steer - current_steer_rad) / dt_lead

                if steer_rate_req > self.max_steer_rate * 1.30:
                    continue

                # Collision check across corridor obstacle points
                collision = False
                for s_rel, d_o, dist_m in obs_cloud:
                    if 0.04 < s_rel < L and abs(d_o) < 0.26:
                        d_curve = a0 + a1 * s_rel + a2 * (s_rel**2) + a3 * (s_rel**3) + a4 * (s_rel**4) + a5 * (s_rel**5)
                        if abs(d_curve - d_o) < (HALF_CAR_WIDTH_M + 0.03):
                            collision = True
                            break

                if collision:
                    continue

                # Cost: Line deviation + Steering effort + Horizon preference
                cost = 1.0 * (d_t**2) + 1.0 * (steer_rate_req / self.max_steer_rate)**2 + 0.2 * abs(d_t - curr_d) + 0.05 * (1.5 - L)**2
                if cost < best_cost:
                    best_cost = cost
                    best_d = d_t
                    best_k = req_k

        return float(best_d), float(best_k), min_dist


# ==============================================================================
# OPTION 4: 36-RAY BIASED DISPARITY EXTENDER (1 MS)
# ==============================================================================
class BiasedDisparityExtender:
    """
    36-Ray Biased Disparity Extender (BDE, 1 ms):
    Subdivides the 12 ToF zones into 36 continuous rays emanating from front bumper,
    inflates depth discontinuities, and steers toward the safest open gap.
    """
    def __init__(self, disparity_threshold_m=0.30):
        self.disparity_threshold = disparity_threshold_m

    def compute(self, tof_distances, target_line_steer):
        ray_angles = []
        ray_ranges = []
        for i in range(NUM_TOF_ZONES):
            dist = tof_distances[i]
            center_ang = TOF_ZONE_ANGLES[i]
            cone_angle = min(FOV_HALF_ANGLE_RAD, math.atan2(0.040, max(dist, 0.08)))
            for d_ang in (-cone_angle, 0.0, +cone_angle):
                ray_angles.append(center_ang + d_ang)
                ray_ranges.append(dist)

        ray_angles = np.array(ray_angles)
        inflated_ranges = np.array(ray_ranges)
        n = len(inflated_ranges)

        # Disparity expansion across 36 sub-rays
        for i in range(n - 1):
            diff = inflated_ranges[i] - inflated_ranges[i + 1]
            if abs(diff) > self.disparity_threshold:
                closer_dist = min(inflated_ranges[i], inflated_ranges[i + 1])
                if closer_dist > 0.05:
                    expand_angle = math.atan2(HALF_CAR_WIDTH_M + 0.04, closer_dist)
                    expand_rays = int(math.ceil(expand_angle / math.radians(5.0)))
                    if diff > 0:
                        start_idx = max(0, i + 1 - expand_rays)
                        inflated_ranges[start_idx:i + 1] = np.minimum(inflated_ranges[start_idx:i + 1], closer_dist)
                    else:
                        end_idx = min(n, i + 1 + expand_rays)
                        inflated_ranges[i:end_idx] = np.minimum(inflated_ranges[i:end_idx], closer_dist)

        safe_mask = inflated_ranges >= 0.40
        min_dist = float(np.min(tof_distances))

        if np.any(safe_mask):
            safe_indices = np.where(safe_mask)[0]
            zone_errors = np.abs(ray_angles[safe_indices] - target_line_steer)
            best_idx = safe_indices[int(np.argmin(zone_errors))]
            cmd_steer = float(ray_angles[best_idx])
        else:
            best_idx = int(np.argmax(inflated_ranges))
            cmd_steer = float(ray_angles[best_idx])

        speed_factor = float(np.clip(min_dist / 0.80, 0.40, 1.0))
        return cmd_steer, speed_factor


# ==============================================================================
# OPTION 5: MODEL PREDICTIVE CONTOURING CONTROL (MPCC / FRENET LTV-MPC VIA OSQP)
# ==============================================================================
import scipy.sparse as sp
try:
    import osqp
    HAS_OSQP = True
except ImportError:
    HAS_OSQP = False

class FrenetMPCC:
    """
    Model Predictive Contouring Controller (Frenet LTV-MPC via OSQP, 60 Hz):
    Jointly optimizes physical steering angle and longitudinal speed over a receding horizon
    subject to physical actuator slew limits (|delta_dot| <= 10.47 rad/s), Ackermann
    steering limits (|delta| <= 28.0°), and dynamic 12-ToF corridor clearance envelopes.
    """
    def __init__(self, N=10, dt=0.035, wheelbase_m=0.1325, max_steer_rate=10.47):
        self.N = N
        self.dt = dt
        self.L = wheelbase_m
        self.max_steer_rate = max_steer_rate
        self.max_steer = math.radians(28.0)
        self.solver = None
        if HAS_OSQP:
            self.solver = osqp.OSQP()

    def plan(self, rx, ry, ryaw, current_v, current_steer, tof_distances, frenet_helper, curr_idx, target_v=3.5):
        """
        Returns:
            cmd_steer (float): Optimal physical front wheel steering angle (rad).
            cmd_v (float): Optimal longitudinal target velocity (m/s).
            min_dist (float): Minimum obstacle clearance (m).
        """
        curr_s, curr_d, _, t_yaw, t_k = frenet_helper.cartesian_to_frenet(rx, ry, curr_idx)
        heading_err = (ryaw - t_yaw + math.pi) % (2.0 * math.pi) - math.pi
        min_dist = float(np.min(tof_distances))

        # Fallback if OSQP not available
        if not HAS_OSQP or self.solver is None:
            delta_ff = math.atan(self.L * t_k)
            stanley_steer = math.atan2(1.50 * (-curr_d), max(abs(current_v), 0.80))
            return delta_ff + stanley_steer, target_v, min_dist

        # Build dynamic ToF lateral corridor bounds
        d_min_tof = -0.38
        d_max_tof = +0.38
        
        # Vehicle-centric corridor narrowing from 12 ToFs
        right_tof = min(tof_distances[8:12])
        left_tof = min(tof_distances[0:4])
        if right_tof < 0.60:
            d_min_tof = max(d_min_tof, curr_d - (right_tof - (HALF_CAR_WIDTH_M + SAFETY_MARGIN_M)))
        if left_tof < 0.60:
            d_max_tof = min(d_max_tof, curr_d + (left_tof - (HALF_CAR_WIDTH_M + SAFETY_MARGIN_M)))
        if d_min_tof > d_max_tof:
            mid = 0.5 * (d_min_tof + d_max_tof)
            d_min_tof, d_max_tof = mid - 0.04, mid + 0.04

        # Discretized Linear Time-Varying (LTV) MPC Formulation
        # State: x = [e_lat, e_yaw, v, delta] (nx = 4)
        # Input: u = [a_lon, delta_dot] (nu = 2)
        nx, nu = 4, 2
        N = self.N
        dt = self.dt
        v0 = max(abs(current_v), 1.0)

        # Weights
        q_lat = 25.0
        q_yaw = 8.0
        q_v = 1.5
        q_delta = 0.5
        r_a = 0.08
        r_drate = 0.05

        n_vars = (N + 1) * nx + N * nu
        P_diag = np.zeros(n_vars)
        for i in range(N):
            idx_x = i * (nx + nu)
            P_diag[idx_x + 0] = q_lat
            P_diag[idx_x + 1] = q_yaw
            P_diag[idx_x + 2] = q_v
            P_diag[idx_x + 3] = q_delta
            P_diag[idx_x + nx + 0] = r_a
            P_diag[idx_x + nx + 1] = r_drate
        idx_terminal = N * (nx + nu)
        P_diag[idx_terminal + 0] = q_lat * 2.0
        P_diag[idx_terminal + 1] = q_yaw * 2.0
        P_diag[idx_terminal + 2] = q_v
        P_diag[idx_terminal + 3] = q_delta

        P_mat = sp.diags(P_diag, format='csc')
        q_vec = np.zeros(n_vars)
        for i in range(N + 1):
            idx_x = i * (nx + nu) if i < N else idx_terminal
            q_vec[idx_x + 2] = -q_v * target_v

        n_eq = (N + 1) * nx
        A_dyn = sp.lil_matrix((n_eq, n_vars))
        l_dyn = np.zeros(n_eq)
        u_dyn = np.zeros(n_eq)

        # Initial state constraint
        A_dyn[0:nx, 0:nx] = sp.eye(nx)
        l_dyn[0:nx] = [curr_d, heading_err, current_v, current_steer]
        u_dyn[0:nx] = [curr_d, heading_err, current_v, current_steer]

        # Dynamics constraints
        for k in range(N):
            idx_xk = k * (nx + nu)
            idx_uk = idx_xk + nx
            idx_xk1 = (k + 1) * (nx + nu) if (k + 1) < N else idx_terminal

            idx_track = (curr_idx + int(k * v0 * dt / 0.05)) % len(frenet_helper.x)
            kappa_k = frenet_helper.curvature[idx_track]
            row = (k + 1) * nx
            
            A_dyn[row + 0, idx_xk + 0] = -1.0
            A_dyn[row + 0, idx_xk + 1] = -v0 * dt
            
            A_dyn[row + 1, idx_xk + 1] = -1.0
            A_dyn[row + 1, idx_xk + 3] = -(v0 / self.L) * dt
            
            A_dyn[row + 2, idx_xk + 2] = -1.0
            A_dyn[row + 3, idx_xk + 3] = -1.0

            A_dyn[row + 2, idx_uk + 0] = -dt
            A_dyn[row + 3, idx_uk + 1] = -dt

            A_dyn[row + 0, idx_xk1 + 0] = 1.0
            A_dyn[row + 1, idx_xk1 + 1] = 1.0
            A_dyn[row + 2, idx_xk1 + 2] = 1.0
            A_dyn[row + 3, idx_xk1 + 3] = 1.0

            l_dyn[row + 1] = -v0 * kappa_k * dt
            u_dyn[row + 1] = -v0 * kappa_k * dt

        l_bounds = np.full(n_vars, -np.inf)
        u_bounds = np.full(n_vars, +np.inf)

        for k in range(N + 1):
            idx_x = k * (nx + nu) if k < N else idx_terminal
            l_bounds[idx_x + 0] = d_min_tof
            u_bounds[idx_x + 0] = d_max_tof
            l_bounds[idx_x + 1] = -math.pi / 2.0
            u_bounds[idx_x + 1] = +math.pi / 2.0
            l_bounds[idx_x + 2] = 0.50
            u_bounds[idx_x + 2] = target_v
            l_bounds[idx_x + 3] = -self.max_steer
            u_bounds[idx_x + 3] = +self.max_steer

            if k < N:
                idx_u = idx_x + nx
                l_bounds[idx_u + 0] = -10.0
                u_bounds[idx_u + 0] = +6.0
                l_bounds[idx_u + 1] = -self.max_steer_rate
                u_bounds[idx_u + 1] = +self.max_steer_rate

        A_full = sp.vstack([A_dyn.tocsc(), sp.eye(n_vars, format='csc')], format='csc')
        l_full = np.concatenate([l_dyn, l_bounds])
        u_full = np.concatenate([u_dyn, u_bounds])

        solver = osqp.OSQP()
        solver.setup(P=P_mat, q=q_vec, A=A_full, l=l_full, u=u_full,
                     eps_abs=2e-3, eps_rel=2e-3, max_iter=60, verbose=False, polish=False)
        res = solver.solve()

        if res.info.status_val in (1, 2):
            sol = res.x
            cmd_steer = float(sol[3] + sol[nx + 1] * dt)
            cmd_v = float(sol[2] + sol[nx + 0] * dt)
            return np.clip(cmd_steer, -self.max_steer, self.max_steer), np.clip(cmd_v, 0.5, target_v), min_dist
        else:
            delta_ff = math.atan(self.L * t_k)
            stanley_steer = math.atan2(1.50 * (-curr_d), max(abs(current_v), 0.80))
            return delta_ff + stanley_steer, target_v, min_dist


# ==============================================================================
# WRONG-WAY LAP PREVENTION
# ==============================================================================
class WrongWayDetector:
    """
    Monitors vehicle heading relative to valid race flow.
    Only triggers if the vehicle is spun (> 135°) AND nearly stationary (v < 0.60 m/s).
    """
    def __init__(self, max_allowed_divergence_rad=math.radians(135.0)):
        self.max_divergence = max_allowed_divergence_rad
        self.is_wrong_way = False
        self.consecutive_wrong_way_steps = 0

    def evaluate(self, current_yaw, track_flow_yaw, current_v=1.0):
        heading_diff = (track_flow_yaw - current_yaw + math.pi) % (2.0 * math.pi) - math.pi

        if abs(heading_diff) > self.max_divergence and abs(current_v) < 0.60:
            self.consecutive_wrong_way_steps += 1
            if self.consecutive_wrong_way_steps >= 25: # ~400ms confirmation
                self.is_wrong_way = True
        else:
            self.consecutive_wrong_way_steps = 0
            self.is_wrong_way = False

        return self.is_wrong_way, heading_diff


# ==============================================================================
# NON-BLOCKING STUCK & REVERSE RECOVERY MANAGER
# ==============================================================================
class StuckRecoveryState:
    NORMAL_DRIVE = "DRIVE"
    PIVOT_REVERSE = "PIVOT_REVERSE"
    PIVOT_FORWARD = "PIVOT_FORWARD"


class StuckRecoveryManager:
    """
    Intelligent Autonomous Multi-Point K-Turn & Clearance Recovery Controller:
    - Eliminates endless reverse backing up into rear walls or competitors.
    - If facing backwards or blocked, executes rapid alternating 3-point K-turns
      (Reverse Arc <-> Forward Arc) that continuously rotate the chassis toward
      track flow yaw in confined track spaces.
    - Instantly exits to NORMAL_DRIVE the moment front clearance (> 28cm) and
      forward heading alignment (|heading_err| <= 45°) are achieved.
    """
    def __init__(self, reverse_speed_m_s=1.35):
        self.state = StuckRecoveryState.NORMAL_DRIVE
        self.reverse_speed = reverse_speed_m_s
        self.stall_timer = 0.0
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        self.preferred_yaw_sign = 1.0

    def trigger_emergency_recovery(self, heading_err=0.0, steer_dir=None):
        """Immediately triggers intelligent recovery maneuver."""
        self.state = StuckRecoveryState.PIVOT_REVERSE
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        if abs(heading_err) > math.radians(25.0):
            self.preferred_yaw_sign = +1.0 if heading_err > 0 else -1.0
        elif steer_dir is not None:
            self.preferred_yaw_sign = +1.0 if steer_dir > 0 else -1.0
        else:
            self.preferred_yaw_sign = 1.0
        print(f"[RECOVERY] Multi-point recovery engaged (Yaw Target Sign: {self.preferred_yaw_sign:+.0f})...", flush=True)

    def update(self, dt, current_v, commanded_v, tof_distances, heading_err=0.0):
        min_all_dist = min(tof_distances)
        min_front_cone = min(tof_distances[3:9])  # Front 6 ToF sensors
        left_dist = min(tof_distances[0:5])
        right_dist = min(tof_distances[7:12])

        # 1. Spinout / Wrong-Way Detection (|heading_err| > 75 deg)
        is_facing_backwards = abs(heading_err) > math.radians(75.0)

        # 2. Physical Stall Detection (Forward throttle commanded but vehicle blocked)
        is_stalled_forward = (commanded_v > 0.30 and abs(current_v) < 0.10 and min_front_cone < 0.25)

        if self.state == StuckRecoveryState.NORMAL_DRIVE:
            if is_facing_backwards:
                self.trigger_emergency_recovery(heading_err=heading_err)
            elif is_stalled_forward:
                self.stall_timer += dt
                if self.stall_timer > 0.15:  # 150ms confirmation
                    side_steer = +1.0 if left_dist < right_dist else -1.0
                    self.trigger_emergency_recovery(heading_err=heading_err, steer_dir=side_steer)
            else:
                self.stall_timer = max(0.0, self.stall_timer - dt * 2.0)
            return False, 0.0, 0.0

        # Active Recovery Loop
        self.stroke_timer += dt
        self.total_recovery_timer += dt

        is_aligned_forward = abs(heading_err) <= math.radians(45.0)
        has_sufficient_clearance = (min_front_cone >= 0.28 and min_all_dist >= 0.16)

        # EXIT GATE: Front clear + heading facing forward
        if (self.total_recovery_timer >= 0.08 and has_sufficient_clearance and is_aligned_forward) or self.total_recovery_timer >= 1.20:
            self.state = StuckRecoveryState.NORMAL_DRIVE
            self.stall_timer = 0.0
            print(f"[RECOVERY] K-Turn complete (Front={min_front_cone*100:.1f}cm, HeadErr={math.degrees(heading_err):+.1f}° in {self.total_recovery_timer*1000:.0f}ms). Resuming full race pace!", flush=True)
            return False, 0.0, 0.0

        # Update preferred yaw rotation if heading error is substantial
        if abs(heading_err) > math.radians(30.0):
            self.preferred_yaw_sign = +1.0 if heading_err > 0 else -1.0

        # --- STROKE 1: REVERSE ARC ---
        if self.state == StuckRecoveryState.PIVOT_REVERSE:
            target_v = -self.reverse_speed
            # In reverse: steer in OPPOSITE direction of desired yaw rotation (steer right -> yaw CCW)
            target_steer = -self.preferred_yaw_sign * math.radians(26.0)

            # Rear barrier contact or reverse stroke limit reached -> switch to FORWARD ARC
            # Stall in reverse: commanded reverse but vehicle stopped by rear wall/obstacle
            is_stalled_reverse = (self.stroke_timer > 0.12 and abs(current_v) < 0.08)
            stroke_timeout = self.stroke_timer >= 0.28

            if is_stalled_reverse or stroke_timeout:
                self.state = StuckRecoveryState.PIVOT_FORWARD
                self.stroke_timer = 0.0
                print(f"[RECOVERY] Rear limit reached (v={current_v:.2f}m/s). Switching to forward pivot arc...", flush=True)

            return True, target_v, target_steer

        # --- STROKE 2: FORWARD ARC ---
        elif self.state == StuckRecoveryState.PIVOT_FORWARD:
            target_v = 1.30
            # In forward: steer in SAME direction of desired yaw rotation (steer left -> yaw CCW)
            target_steer = +self.preferred_yaw_sign * math.radians(26.0)

            # Front barrier clearance closed or forward stroke limit reached -> switch back to REVERSE ARC
            front_blocked = (self.stroke_timer > 0.10 and min_front_cone < 0.18)
            stroke_timeout = self.stroke_timer >= 0.25

            if front_blocked or stroke_timeout:
                self.state = StuckRecoveryState.PIVOT_REVERSE
                self.stroke_timer = 0.0
                print(f"[RECOVERY] Front obstacle approached. Alternating back to reverse arc...", flush=True)

            return True, target_v, target_steer

        return False, 0.0, 0.0
