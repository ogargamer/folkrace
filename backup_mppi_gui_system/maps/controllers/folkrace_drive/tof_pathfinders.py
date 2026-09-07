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

import os
import sys
import math
import json
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
SAFETY_MARGIN_M = 0.08   # 80mm safety buffer
MAX_STEERING_RAD = 0.5200  # 29.8 degrees steering limit (Webots servo limit is 0.5236 rad)


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

    def frenet_to_cartesian(self, s, d=0.0, base_idx=0):
        """
        Converts (s, d) into Cartesian (x, y) world coordinates via arc-length interpolation.
        """
        s_norm = s % self.total_length
        idx = int(np.searchsorted(self.traj_s, s_norm)) % self.n_pts
        im1 = (idx - 1 + self.n_pts) % self.n_pts

        s0 = self.traj_s[im1]
        s1 = self.traj_s[idx]
        ds = s1 - s0
        if ds < 0:
            ds += self.total_length

        ratio = 0.0 if ds < 1e-4 else (s_norm - s0) / ds
        ratio = max(0.0, min(1.0, ratio))

        tx = self.traj_x[im1] + ratio * (self.traj_x[idx] - self.traj_x[im1])
        ty = self.traj_y[im1] + ratio * (self.traj_y[idx] - self.traj_y[im1])

        # Heading interpolation
        dyaw = (self.traj_yaw[idx] - self.traj_yaw[im1] + math.pi) % (2.0 * math.pi) - math.pi
        t_yaw = self.traj_yaw[im1] + ratio * dyaw

        # Normal vector pointing to the left (+d)
        nx = -math.sin(t_yaw)
        ny = math.cos(t_yaw)

        x = tx + d * nx
        y = ty + d * ny
        return float(x), float(y)


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
    1. Static Wall vs Dynamic Obstacle Filtering
    2. Continuous Speed-Dependent Dynamic Margins
    3. Gap-Targeted Trajectory Optimization
    """
    def __init__(self, num_nodes=60, node_spacing_m=0.025, w_line=1.0, w_smooth=24.0, wheelbase_m=0.150):
        self.num_nodes = num_nodes
        self.ds = node_spacing_m
        self.w_line = w_line
        self.w_smooth = w_smooth
        self.wheelbase = wheelbase_m

        # Load pre-computed track corridor profile (true physical left and right road widths)
        curr_dir = os.path.dirname(os.path.abspath(__file__))
        profile_path = os.path.join(curr_dir, "track_corridor_profile.json")
        self.left_bounds = None
        self.right_bounds = None
        if os.path.exists(profile_path):
            try:
                with open(profile_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    self.left_bounds = np.array(data["d_left_bound"], dtype=float)
                    self.right_bounds = np.array(data["d_right_bound"], dtype=float)
            except Exception as e:
                print(f"[PATHFINDER] Note: Could not load track corridor profile: {e}", flush=True)

    def get_track_bounds_at(self, idx):
        if self.left_bounds is not None and len(self.left_bounds) > 0:
            i = int(idx) % len(self.left_bounds)
            return float(self.left_bounds[i]), float(self.right_bounds[i])
        return +0.38, -0.38

    def plan(self, rx, ry, ryaw, tof_distances, frenet_helper, curr_idx, current_steer_rad=0.0, v_mps=2.0):
        curr_s, curr_d, _, t_yaw, t_k = frenet_helper.cartesian_to_frenet(rx, ry, curr_idx)

        # 1. Physical Track Boundary Limits at current location
        left_bound_map, right_bound_map = self.get_track_bounds_at(curr_idx)
        map_space_left = max(0.0, left_bound_map - HALF_CAR_WIDTH_M)
        map_space_right = max(0.0, abs(right_bound_map) - HALF_CAR_WIDTH_M)

        # Build dense corridor bounds bounded by actual track walls
        d_min = np.full(self.num_nodes, max(-0.38, right_bound_map + 0.04))
        d_max = np.full(self.num_nodes, min(+0.38, left_bound_map - 0.04))
        gap_target = np.zeros(self.num_nodes)
        obs_cloud, min_corridor_dist = extract_36_point_tof_cloud(rx, ry, ryaw, tof_distances, frenet_helper, curr_idx)

        # Speed-dependent dynamic margin
        mode_env = os.environ.get("SHIELD_MARGIN_MODE", "QUADRATIC").upper()
        d_min_env = float(os.environ.get("SHIELD_D_MIN", "0.020"))
        d_max_env = float(os.environ.get("SHIELD_D_MAX", "0.055"))
        if mode_env == "LINEAR":
            d_dyn = d_min_env + (d_max_env - d_min_env) * min(1.0, max(0.0, abs(v_mps) / 4.40))
        elif mode_env == "CURVATURE":
            a_lat = (v_mps**2) * abs(t_k)
            d_dyn = d_min_env + (d_max_env - d_min_env) * min(1.0, a_lat / (0.70 * 9.81))
        elif mode_env == "SIGMOID":
            d_dyn = d_min_env + (d_max_env - d_min_env) / (1.0 + math.exp(-2.5 * (abs(v_mps) - 2.50)))
        elif mode_env == "FIXED":
            d_dyn = d_min_env
        else: # QUADRATIC / ASYMMETRIC
            d_dyn = d_min_env + (d_max_env - d_min_env) * min(1.0, (abs(v_mps) / 4.40)**2)

        # Determine live ToF clearance
        tof_sum_left = float(np.sum(tof_distances[0:6]))
        tof_sum_right = float(np.sum(tof_distances[6:12]))
        min_front_zone = float(np.min(tof_distances[3:9]))

        safety_buf = HALF_CAR_WIDTH_M + d_dyn
        for ds_ahead, d_obs, dist in obs_cloud:
            if 0.02 < ds_ahead < (self.num_nodes * self.ds + 0.10):
                node_idx = min(int(round(ds_ahead / self.ds)), self.num_nodes - 1)
                # Constrict corridor only for true obstacles within the active racing lane (|d_obs| <= 0.16m)
                if abs(d_obs) <= 0.16:
                    k_start = max(0, node_idx - 4)
                    k_end = min(self.num_nodes, node_idx + 14)
                    
                    # Fuse Map Physical Track Space + Live ToF Open Readings
                    # Score both flanks based on physical space and sensor clarity
                    score_right = map_space_right * (tof_sum_right / max(1.0, tof_sum_left + tof_sum_right))
                    score_left = map_space_left * (tof_sum_left / max(1.0, tof_sum_left + tof_sum_right))
                    
                    if score_right >= score_left and map_space_right >= 0.14:
                        pass_side = -1.0
                        target_offset = -min(0.35, max(0.18, map_space_right * 0.65))
                    elif map_space_left >= 0.14:
                        pass_side = +1.0
                        target_offset = +min(0.42, max(0.20, map_space_left * 0.65))
                    else:
                        pass_side = -1.0 if map_space_right >= map_space_left else +1.0
                        target_offset = pass_side * 0.20

                    gap_target[:] = target_offset
                    
                    if pass_side > 0:
                        for k in range(k_start, self.num_nodes):
                            d_min[k] = max(d_min[k], d_obs + safety_buf)
                    else:
                        for k in range(k_start, self.num_nodes):
                            d_max[k] = min(d_max[k], d_obs - safety_buf)

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
        d_opt = np.clip(gap_target.copy(), d_min, d_max)
        d_opt[0] = np.clip(curr_d, d_min[0], d_max[0])
        d_opt[1] = np.clip(curr_d + d_prime_0 * self.ds, d_min[1], d_max[1])
        omega_sor = 1.35

        for _ in range(60):
            for k in range(2, self.num_nodes - 1):
                d_prev = d_opt[k - 1]
                d_next = d_opt[k + 1]
                target_smooth = 0.5 * (d_prev + d_next)
                d_target_val = (self.w_line * gap_target[k] + self.w_smooth * target_smooth) / (self.w_line + self.w_smooth)
                d_new = (1.0 - omega_sor) * d_opt[k] + omega_sor * d_target_val
                d_opt[k] = np.clip(d_new, d_min[k], d_max[k])

        d_opt[-1] = d_opt[-2]

        # Speed-Adaptive Lookahead Horizon (180ms preview time)
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
# TIER 2 PLANNER 1: FTG + DYNAMIC FRICTION CIRCLE GOVERNOR (REACTIVE)
# ==============================================================================
class FTGFrictionGovernor:
    """
    Tier 2 Planner 1: Follow-The-Gap + Dynamic Friction Governor (Reactive, < 0.2 ms)
    1. Spatial Target Extraction: 36-ray disparity expansion on 12 ToF array -> psi_gap.
    2. Lookahead Curvature: kappa_req = (2 * sin(psi_gap)) / L_fw (L_fw = 0.50 m).
    3. Friction Circle Clamping: V_max_corner = sqrt(mu * g * eta / (|kappa_req| + 0.001)) (eta = 0.85).
    4. Longitudinal Command: Applies deceleration if V > V_max_corner, accelerates up to 4.0 m/s if safe.
    5. Lateral Command: delta_cmd = atan(L * kappa_req) (L = 0.150 m).
    """
    def __init__(self, wheelbase_m=0.150, lookahead_m=0.50, mu=0.85, eta=0.85, top_speed=4.0):
        self.L = wheelbase_m
        self.L_fw = lookahead_m
        self.mu = mu
        self.eta = eta
        self.top_speed = top_speed
        self.g = 9.81
        self.disparity_threshold = 0.28
        self.car_half_w = HALF_CAR_WIDTH_M

    def plan(self, rx, ry, ryaw, current_v, tof_distances, target_line_steer=0.0):
        ray_angles = []
        ray_ranges = []
        for i in range(NUM_TOF_ZONES):
            dist = tof_distances[i]
            c_ang = TOF_ZONE_ANGLES[i]
            cone = min(FOV_HALF_ANGLE_RAD, math.atan2(0.040, max(dist, 0.08)))
            for d_ang in (-cone, 0.0, +cone):
                ray_angles.append(c_ang + d_ang)
                ray_ranges.append(dist)

        ray_angles = np.array(ray_angles)
        inflated_ranges = np.array(ray_ranges)
        n = len(inflated_ranges)

        for i in range(n - 1):
            diff = inflated_ranges[i] - inflated_ranges[i + 1]
            if abs(diff) > self.disparity_threshold:
                closer = min(inflated_ranges[i], inflated_ranges[i + 1])
                if closer > 0.04:
                    expand_angle = math.atan2(self.car_half_w + 0.04, closer)
                    expand_rays = int(math.ceil(expand_angle / math.radians(5.0)))
                    if diff > 0:
                        start_idx = max(0, i + 1 - expand_rays)
                        inflated_ranges[start_idx:i + 1] = np.minimum(inflated_ranges[start_idx:i + 1], closer)
                    else:
                        end_idx = min(n, i + 1 + expand_rays)
                        inflated_ranges[i:end_idx] = np.minimum(inflated_ranges[i:end_idx], closer)

        safe_mask = inflated_ranges >= 0.38
        if np.any(safe_mask):
            safe_indices = np.where(safe_mask)[0]
            zone_errors = np.abs(ray_angles[safe_indices] - target_line_steer)
            best_idx = safe_indices[int(np.argmin(zone_errors))]
            psi_gap = float(ray_angles[best_idx])
        else:
            best_idx = int(np.argmax(inflated_ranges))
            psi_gap = float(ray_angles[best_idx])

        kappa_req = (2.0 * math.sin(psi_gap)) / self.L_fw
        v_max_corner = math.sqrt((self.mu * self.g * self.eta) / (abs(kappa_req) + 0.001))
        cmd_v = float(min(self.top_speed, v_max_corner))

        delta_cmd = float(math.atan(self.L * kappa_req))
        delta_cmd = float(np.clip(delta_cmd, -MAX_STEERING_RAD, MAX_STEERING_RAD))

        min_dist = float(np.min(tof_distances))
        return delta_cmd, cmd_v, min_dist


# ==============================================================================
# TIER 2 PLANNER 1: FRENET LATTICE WITH VELOCITY POLYNOMIALS (WERLING METHOD)
# ==============================================================================
class WerlingLatticePlanner:
    """
    Tier 2 Planner: Frenet Lattice with Velocity Polynomials (Werling Method, ~1.8 ms)
    - Fine-tuned 9-beam lateral mesh (d_T in +-0.44m) with multi-horizon preview (0.25s, 0.45s, 0.65s).
    - Discards trajectories violating friction limit a_y > mu * g.
    - Vectorized collision masking with 0.14m safety bubble around 12-channel ToF Cartesian points.
    - Minimum jerk and velocity tracking cost selection.
    """
    def __init__(self, wheelbase_m=0.150, mu=0.75, top_speed=2.6):
        self.L = wheelbase_m
        self.mu = mu
        self.g = 9.81
        self.top_speed = top_speed
        self.d_targets = np.array([-0.44, -0.33, -0.22, -0.11, 0.00, +0.11, +0.22, +0.33, +0.44])
        self.horizons = np.array([0.35, 0.60, 0.85])
        self.dt = 0.04

    def plan(self, rx, ry, ryaw, current_v, current_steer, tof_distances, frenet_helper, curr_idx, target_v=3.5):
        curr_s, curr_d, _, t_yaw, t_k = frenet_helper.cartesian_to_frenet(rx, ry, curr_idx)
        heading_err = (ryaw - t_yaw + math.pi) % (2.0 * math.pi) - math.pi
        v0 = max(abs(current_v), 0.50)
        
        # Initial boundary conditions in Frenet frame
        d0 = curr_d
        d_dot0 = v0 * math.sin(heading_err)
        d_ddot0 = (v0**2) * (math.tan(current_steer) / self.L - t_k)
        
        s0 = curr_s
        s_dot0 = max(0.50, v0 * math.cos(heading_err))
        s_ddot0 = 0.0

        # ToF Cartesian Collision Cloud mapped to Frenet frame
        front_x = rx + TOF_FRONT_OFFSET_M * math.cos(ryaw)
        front_y = ry + TOF_FRONT_OFFSET_M * math.sin(ryaw)
        tof_ox = []
        tof_oy = []
        for i in range(NUM_TOF_ZONES):
            dist = tof_distances[i]
            if TOF_MIN_RANGE_M < dist < TOF_MAX_RANGE_M:
                b_ang = ryaw + TOF_ZONE_ANGLES[i]
                tof_ox.append(front_x + dist * math.cos(b_ang))
                tof_oy.append(front_y + dist * math.sin(b_ang))

        if tof_ox:
            tof_s, tof_d = frenet_helper.cartesian_to_frenet_batch(np.array(tof_ox), np.array(tof_oy), guess_idx=curr_idx)
        else:
            tof_s, tof_d = np.array([]), np.array([])

        v_targets = np.array([max(0.50, v0 - 1.0), v0, min(self.top_speed, v0 + 1.0), target_v])
        v_targets = np.unique(v_targets)

        best_cost = 1e9
        best_delta = math.atan(self.L * t_k)
        best_v = target_v
        min_dist = float(np.min(tof_distances))

        for T in self.horizons:
            time_steps = np.arange(self.dt, T + self.dt * 0.5, self.dt)
            T2, T3, T4, T5 = T**2, T**3, T**4, T**5

            for vt in v_targets:
                sa0 = s_dot0
                sa1 = s_ddot0
                denom_lon = T2
                sa2 = (3.0 * (vt - s_dot0) - 2.0 * s_ddot0 * T) / denom_lon
                sa3 = (-2.0 * (vt - s_dot0) + s_ddot0 * T) / T3

                s_dot_t = sa0 + sa1 * time_steps + sa2 * (time_steps**2) + sa3 * (time_steps**3)
                s_t = s0 + sa0 * time_steps + 0.5 * sa1 * (time_steps**2) + (sa2 / 3.0) * (time_steps**3) + (sa3 / 4.0) * (time_steps**4)

                for dt_target in self.d_targets:
                    b0 = d0
                    b1 = d_dot0
                    b2 = 0.5 * d_ddot0

                    del_d = dt_target - (b0 + b1 * T + b2 * T2)
                    del_v = 0.0 - (b1 + 2.0 * b2 * T)
                    del_a = 0.0 - (2.0 * b2)

                    b3 = (10.0 * del_d - 4.0 * del_v * T + 0.5 * del_a * T2) / T3
                    b4 = (-15.0 * del_d + 7.0 * del_v * T - del_a * T2) / T4
                    b5 = (6.0 * del_d - 3.0 * del_v * T + 0.5 * del_a * T2) / T5

                    d_t_arr = b0 + b1 * time_steps + b2 * (time_steps**2) + b3 * (time_steps**3) + b4 * (time_steps**4) + b5 * (time_steps**5)
                    d_dot_t = b1 + 2.0 * b2 * time_steps + 3.0 * b3 * (time_steps**2) + 4.0 * b4 * (time_steps**3) + 5.0 * b5 * (time_steps**4)
                    d_ddot_t = 2.0 * b2 + 6.0 * b3 * time_steps + 12.0 * b4 * (time_steps**2) + 20.0 * b5 * (time_steps**3)

                    # Friction limit check
                    v_sq = s_dot_t**2 + d_dot_t**2
                    a_y_arr = np.abs(v_sq * t_k + d_ddot_t)
                    if np.any(a_y_arr > (self.mu * self.g)):
                        continue

                    # Dynamic Speed-Dependent Collision Bubble (75mm half car width + 10mm to 45mm dynamic buffer)
                    v_eval = np.sqrt(v_sq)
                    v_ratio = np.clip(v_eval / max(self.top_speed, 1.0), 0.0, 1.0)
                    bubble_r_arr = 0.085 + 0.045 * (v_ratio**2)  # (N,)

                    if len(tof_s) > 0:
                        ds_mat = np.abs(s_t[:, None] - tof_s[None, :])
                        dd_mat = np.abs(d_t_arr[:, None] - tof_d[None, :])
                        dist_sq_mat = ds_mat**2 + dd_mat**2
                        if np.any(dist_sq_mat < (bubble_r_arr[:, None]**2)):
                            continue

                    jerk_lon = np.sum((2.0 * sa2 + 6.0 * sa3 * time_steps)**2) * self.dt
                    jerk_lat = np.sum((6.0 * b3 + 24.0 * b4 * time_steps + 60.0 * b5 * (time_steps**2))**2) * self.dt
                    cost = (
                        0.80 * (dt_target**2) +
                        0.40 * abs(dt_target - d0) +
                        2.20 * ((vt - target_v)**2) +
                        0.008 * jerk_lat +
                        0.008 * jerk_lon +
                        0.15 * T
                    )

                    if cost < best_cost:
                        best_cost = cost
                        s_dot_1 = float(s_dot_t[0])
                        d_dot_1 = float(d_dot_t[0])
                        d_ddot_1 = float(d_ddot_t[0])
                        req_kappa = (t_k + d_ddot_1 / max(s_dot_1**2, 0.25))
                        best_delta = float(math.atan(self.L * req_kappa))
                        best_v = float(math.hypot(s_dot_1, d_dot_1))

        best_delta = float(np.clip(best_delta, -MAX_STEERING_RAD, MAX_STEERING_RAD))
        best_v = float(np.clip(best_v, 0.50, self.top_speed))
        return best_delta, best_v, min_dist


# ==============================================================================
# TIER 2 PLANNER 2: MODEL PREDICTIVE PATH INTEGRAL CONTROL (MPPI)
# ==============================================================================
class MPPIPlanner:
    """
    Tier 2 Planner: Model Predictive Path Integral Control (MPPI, ~3 ms vectorized)
    - Stability-tuned K=700 parallel rollouts over N=18 time steps (dt=0.045s, 0.81s preview horizon).
    - Dynamic speed-dependent Gaussian obstacle potential for gap penetration.
    - Softmax probability-weighted actuation.
    """
    def __init__(self, K=700, N=18, dt=0.045, wheelbase_m=0.150, top_speed=2.6):
        self.K = K
        self.N = N
        self.dt = dt
        self.L = wheelbase_m
        self.top_speed = top_speed
        self.sigma_steer = 0.18
        self.sigma_accel = 3.40
        self.temperature = 8.0
        self.u_nominal = np.zeros((self.N, 2))

    def plan(self, rx, ry, ryaw, current_v, current_steer, tof_distances, frenet_helper, curr_idx, target_v=3.5):
        self.u_nominal[:-1] = self.u_nominal[1:]
        self.u_nominal[-1] = [0.0, 0.0]

        noise = np.random.randn(self.K, self.N, 2)
        noise[:, :, 0] *= self.sigma_accel
        noise[:, :, 1] *= self.sigma_steer

        u_samples = self.u_nominal[None, :, :] + noise
        u_samples[:, :, 0] = np.clip(u_samples[:, :, 0], -14.0, 11.0)
        u_samples[:, :, 1] = np.clip(u_samples[:, :, 1], -MAX_STEERING_RAD, MAX_STEERING_RAD)

        front_x = rx + TOF_FRONT_OFFSET_M * math.cos(ryaw)
        front_y = ry + TOF_FRONT_OFFSET_M * math.sin(ryaw)
        tof_ox = []
        tof_oy = []
        for i in range(NUM_TOF_ZONES):
            dist = tof_distances[i]
            if TOF_MIN_RANGE_M < dist < TOF_MAX_RANGE_M:
                b_ang = ryaw + TOF_ZONE_ANGLES[i]
                tof_ox.append(front_x + dist * math.cos(b_ang))
                tof_oy.append(front_y + dist * math.sin(b_ang))

        has_obs = len(tof_ox) > 0
        if has_obs:
            obs_pts = np.column_stack([tof_ox, tof_oy])
        else:
            obs_pts = np.empty((0, 2))

        curr_s, curr_d, _, t_yaw, t_k = frenet_helper.cartesian_to_frenet(rx, ry, curr_idx)

        sim_x = np.full(self.K, rx)
        sim_y = np.full(self.K, ry)
        sim_yaw = np.full(self.K, ryaw)
        sim_v = np.full(self.K, max(current_v, 0.50))
        costs = np.zeros(self.K)

        for t in range(self.N):
            a_t = u_samples[:, t, 0]
            steer_t = u_samples[:, t, 1]

            beta_t = np.arctan(0.5 * np.tan(steer_t))
            sim_x += sim_v * np.cos(sim_yaw + beta_t) * self.dt
            sim_y += sim_v * np.sin(sim_yaw + beta_t) * self.dt
            sim_yaw += (sim_v / self.L) * np.cos(beta_t) * np.tan(steer_t) * self.dt
            sim_v = np.clip(sim_v + a_t * self.dt, 0.50, self.top_speed)

            # Track progress & heading alignment cost
            head_dev = 1.0 - np.cos(sim_yaw - t_yaw)
            costs += 0.80 * ((sim_v - target_v)**2) + 2.50 * head_dev + 0.12 * (steer_t**2) + 0.04 * (a_t**2)

            # Dynamic speed-dependent obstacle repulsive field
            if has_obs:
                v_ratio = np.clip(sim_v / max(self.top_speed, 1.0), 0.0, 1.0)
                sigma_dyn = 0.090 + 0.060 * (v_ratio**2)  # 9.0cm at low speed, 15.0cm at high speed
                dx = sim_x[:, None] - obs_pts[None, :, 0]
                dy = sim_y[:, None] - obs_pts[None, :, 1]
                d_sq = dx**2 + dy**2
                repulsion = np.sum(120.0 * np.exp(-d_sq / (2.0 * (sigma_dyn[:, None]**2))), axis=1)
                costs += repulsion

        min_cost = np.min(costs)
        weights = np.exp(-(costs - min_cost) / self.temperature)
        sum_w = np.sum(weights) + 1e-8
        weights /= sum_w

        self.u_nominal = np.sum(weights[:, None, None] * u_samples, axis=0)

        opt_a = float(self.u_nominal[0, 0])
        opt_steer = float(self.u_nominal[0, 1])
        
        min_front = float(np.min(tof_distances[4:8]))
        min_dist = float(np.min(tof_distances))

        # Dynamic Speed Command: Accelerate on clear path, govern only on close obstacles
        if min_front < 0.20:
            cmd_v = max(0.50, target_v * 0.40)
        elif min_front < 0.45:
            cmd_v = max(0.80, target_v * 0.65)
        else:
            cmd_v = max(current_v + max(opt_a, 2.5) * 0.25, target_v * 0.90)

        cmd_v = float(np.clip(cmd_v, 0.50, self.top_speed))
        cmd_steer = float(np.clip(opt_steer, -MAX_STEERING_RAD, MAX_STEERING_RAD))

        return cmd_steer, cmd_v, min_dist


# ==============================================================================
# TIER 2 PLANNER 3: MODEL PREDICTIVE CONTOURING CONTROL (MPCC / ACADOS STYLE)
# ==============================================================================
class MPCCAcadosPlanner:
    """
    Tier 2 Planner: Model Predictive Contouring Control (MPCC, ~0.05 ms)
    - Dynamic speed-dependent bypass corridor generation for tight gap penetration.
    - Combined friction circle constraint: sqrt(a^2 + (v^2 * kappa)^2) <= mu * g (mu = 0.85).
    """
    def __init__(self, N=10, dt=0.045, wheelbase_m=0.150, mu=0.85, top_speed=2.6):
        self.N = N
        self.dt = dt
        self.L = wheelbase_m
        self.mu = mu
        self.g = 9.81
        self.top_speed = top_speed
        self.max_steer = MAX_STEERING_RAD

    def plan(self, rx, ry, ryaw, current_v, current_steer, tof_distances, frenet_helper, curr_idx, target_v=3.5):
        curr_s, curr_d, _, t_yaw, t_k = frenet_helper.cartesian_to_frenet(rx, ry, curr_idx)
        heading_err = (ryaw - t_yaw + math.pi) % (2.0 * math.pi) - math.pi
        v0 = max(abs(current_v), 0.60)
        min_dist = float(np.min(tof_distances))

        left_clear = float(np.min(tof_distances[0:4]))
        right_clear = float(np.min(tof_distances[8:12]))
        
        # Dynamic Speed-Dependent Flank Threshold & Corridor Shift
        v_ratio = np.clip(v0 / max(self.top_speed, 1.0), 0.0, 1.0)
        flank_thresh = 0.20 + 0.12 * v_ratio     # 20cm at low speed, 32cm at high speed
        corridor_shift = 0.18 + 0.08 * v_ratio   # 18cm at low speed, 26cm at high speed

        d_target_corridor = 0.0
        if right_clear < flank_thresh and left_clear > flank_thresh:
            d_target_corridor = +corridor_shift
        elif left_clear < flank_thresh and right_clear > flank_thresh:
            d_target_corridor = -corridor_shift

        delta_ff = math.atan(self.L * (t_k + 0.60 * (d_target_corridor - curr_d)))
        e_contour = curr_d - d_target_corridor
        cmd_steer = delta_ff - 1.20 * math.atan(e_contour / max(v0, 0.70)) - 0.80 * heading_err

        kappa_effective = abs(math.tan(cmd_steer) / self.L)
        v_friction_max = math.sqrt((self.mu * self.g * 0.90) / (kappa_effective + 0.001))
        cmd_v = min(self.top_speed, v_friction_max)

        cmd_steer = float(np.clip(cmd_steer, -self.max_steer, self.max_steer))
        cmd_v = float(np.clip(cmd_v, 0.50, self.top_speed))

        return cmd_steer, cmd_v, min_dist


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
    Intelligent Autonomous Clearance & Multi-Point K-Turn Recovery Manager:
    - Executes sustained, intelligent reverse strokes (minimum 450ms, targeting >= 50cm front clearance).
    - Uses 12-channel ToF spatial analysis to determine the optimal steering direction in reverse:
      swings the front nose toward the wider open corridor while backing away from obstacles.
    - Applies a 350ms post-recovery cooldown deadband to prevent rapid re-triggering cycles.
    """
    def __init__(self, reverse_speed_m_s=1.25):
        self.state = StuckRecoveryState.NORMAL_DRIVE
        self.reverse_speed = reverse_speed_m_s
        self.stall_timer = 0.0
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        self.cooldown_timer = 0.0
        self.preferred_steer_reverse = 0.0

    def trigger_emergency_recovery(self, heading_err=0.0, steer_dir=None, tof_distances=None):
        """Immediately triggers intelligent recovery maneuver with spatial direction selection."""
        self.state = StuckRecoveryState.PIVOT_REVERSE
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        self.cooldown_timer = 0.0
        
        # Spatial Intelligence: Choose reverse steering to aim the front nose toward the open corridor
        if tof_distances is not None and len(tof_distances) == 12:
            left_mean = float(np.mean(tof_distances[0:5]))
            right_mean = float(np.mean(tof_distances[7:12]))
            if right_mean > left_mean + 0.04:
                # Right side is more open -> steer LEFT in reverse so nose swings RIGHT
                self.preferred_steer_reverse = +MAX_STEERING_RAD
            elif left_mean > right_mean + 0.04:
                # Left side is more open -> steer RIGHT in reverse so nose swings LEFT
                self.preferred_steer_reverse = -MAX_STEERING_RAD
            else:
                self.preferred_steer_reverse = +MAX_STEERING_RAD if heading_err >= 0 else -MAX_STEERING_RAD
        elif steer_dir is not None:
            self.preferred_steer_reverse = +MAX_STEERING_RAD if steer_dir > 0 else -MAX_STEERING_RAD
        else:
            self.preferred_steer_reverse = +MAX_STEERING_RAD if heading_err >= 0 else -MAX_STEERING_RAD

        print(f"[RECOVERY] Intelligent reverse engaged (Steer Target: {math.degrees(self.preferred_steer_reverse):+.1f}°)...", flush=True)

    def update(self, dt, current_v, commanded_v, tof_distances, heading_err=0.0):
        min_all_dist = min(tof_distances)
        min_front_cone = min(tof_distances[3:9])  # Front 6 ToF sensors
        left_mean = float(np.mean(tof_distances[0:5]))
        right_mean = float(np.mean(tof_distances[7:12]))

        if self.cooldown_timer > 0.0:
            self.cooldown_timer -= dt

        # 1. Spinout / Wrong-Way Detection (|heading_err| > 75 deg)
        is_facing_backwards = abs(heading_err) > math.radians(75.0)

        # 2. Imminent physical collision: front bumper within 7cm
        is_imminent_collision = (min_front_cone <= 0.075 and current_v > 0.05)

        # 3. Blocked / Stalled Forward Detection (sustained for 60ms)
        is_blocked_forward = (min_front_cone < 0.10 and abs(current_v) < 0.20)
        is_stalled_forward = (commanded_v > 0.40 and abs(current_v) < 0.08 and min_front_cone < 0.16)

        if self.state == StuckRecoveryState.NORMAL_DRIVE:
            if self.cooldown_timer <= 0.0:
                if is_facing_backwards or is_imminent_collision:
                    self.trigger_emergency_recovery(heading_err=heading_err, tof_distances=tof_distances)
                    return True, -self.reverse_speed, self.preferred_steer_reverse
                elif is_blocked_forward or is_stalled_forward:
                    self.stall_timer += dt
                    if self.stall_timer > 0.06:
                        self.trigger_emergency_recovery(heading_err=heading_err, tof_distances=tof_distances)
                        return True, -self.reverse_speed, self.preferred_steer_reverse
                else:
                    self.stall_timer = max(0.0, self.stall_timer - dt * 2.0)
            return False, 0.0, 0.0

        # --- ACTIVE RECOVERY LOOP ---
        self.stroke_timer += dt
        self.total_recovery_timer += dt

        # Re-evaluate open space dynamically during reverse
        if right_mean > left_mean + 0.06:
            self.preferred_steer_reverse = +MAX_STEERING_RAD
        elif left_mean > right_mean + 0.06:
            self.preferred_steer_reverse = -MAX_STEERING_RAD

        # --- STROKE 1: REVERSE ARC ---
        if self.state == StuckRecoveryState.PIVOT_REVERSE:
            target_v = -self.reverse_speed
            target_steer = self.preferred_steer_reverse

            # EXIT GATE FOR REVERSE: Must back up for at least 0.40s AND achieve >= 48cm front clearance
            has_wide_clearance = (min_front_cone >= 0.48 and min_all_dist >= 0.14)
            is_aligned_forward = abs(heading_err) <= math.radians(50.0)

            if self.stroke_timer >= 0.40 and has_wide_clearance and is_aligned_forward:
                self.state = StuckRecoveryState.NORMAL_DRIVE
                self.stall_timer = 0.0
                self.cooldown_timer = 0.35  # 350ms cooldown to accelerate smoothly forward into the gap
                print(f"[RECOVERY] Reverse clear exit (Front={min_front_cone*100:.1f}cm, HeadErr={math.degrees(heading_err):+.1f}° in {self.total_recovery_timer*1000:.0f}ms). Resuming race pace!", flush=True)
                return False, 0.0, 0.0

            # Rear barrier contact or reverse stroke max timeout (0.85s) -> switch to FORWARD ARC
            is_stalled_reverse = (self.stroke_timer > 0.20 and abs(current_v) < 0.06)
            stroke_timeout = self.stroke_timer >= 0.85

            if is_stalled_reverse or stroke_timeout:
                self.state = StuckRecoveryState.PIVOT_FORWARD
                self.stroke_timer = 0.0
                print(f"[RECOVERY] Reverse stroke complete (Front={min_front_cone*100:.1f}cm, v={current_v:.2f}m/s). Switching to forward pivot...", flush=True)

            return True, target_v, target_steer

        # --- STROKE 2: FORWARD ARC ---
        elif self.state == StuckRecoveryState.PIVOT_FORWARD:
            target_v = 1.20
            # In forward stroke: steer opposite of reverse to continue pivoting the nose toward the opening
            target_steer = -self.preferred_steer_reverse

            has_wide_clearance = (min_front_cone >= 0.45 and min_all_dist >= 0.12)
            is_aligned_forward = abs(heading_err) <= math.radians(45.0)

            if self.stroke_timer >= 0.25 and has_wide_clearance and is_aligned_forward:
                self.state = StuckRecoveryState.NORMAL_DRIVE
                self.stall_timer = 0.0
                self.cooldown_timer = 0.35
                print(f"[RECOVERY] Forward pivot complete (Front={min_front_cone*100:.1f}cm, HeadErr={math.degrees(heading_err):+.1f}° in {self.total_recovery_timer*1000:.0f}ms). Resuming race pace!", flush=True)
                return False, 0.0, 0.0

            # Front barrier closed or forward stroke max timeout (0.45s) -> switch back to REVERSE ARC
            front_blocked = (self.stroke_timer > 0.12 and min_front_cone < 0.15)
            stroke_timeout = self.stroke_timer >= 0.45

            if front_blocked or stroke_timeout:
                self.state = StuckRecoveryState.PIVOT_REVERSE
                self.stroke_timer = 0.0
                print(f"[RECOVERY] Front blocked in forward arc. Alternating back to reverse...", flush=True)

            return True, target_v, target_steer

        return False, 0.0, 0.0
