"""
Predictive Collision Risk & Dynamic Guard Module for Da Winna Folkrace Robot.
Provides 3 distinct predictive safety architectures:
1. Dynamic Swept-Volume Polygon Projection (Forward Rollout Collision Check)
2. Predictive Time-To-Collision (TTC) Dynamic Braking Shield
3. Predictive Evasion Corridor QP (Proactive Path Deviation when Swept Volume intersects obstacle)
"""

import math
import numpy as np

HALF_CAR_WIDTH_M = 0.0750  # 7.5 cm half-width (15.0 cm full body width)
CAR_LENGTH_M = 0.2000      # 20.0 cm chassis length
WHEELBASE_M = 0.1325       # 13.25 cm wheelbase

class PredictiveSafetyShield:
    """
    Simulates the vehicle's future physical footprint forward in time (0.8s to 1.2s horizon)
    along the planned trajectory/command and tests for intersection with live ToF point cloud.
    Supports 5 continuous speed-dependent dynamic margin architectures:
    1. QUADRATIC: d_min + (d_max - d_min) * (v / v_max)^2
    2. LINEAR:    d_min + (d_max - d_min) * (v / v_max)
    3. CURVATURE: d_min + (d_max - d_min) * min(1.0, (v^2 * abs(kappa)) / (0.60 * 9.81))
    4. SIGMOID:   d_min + (d_max - d_min) / (1 + exp(-2.5 * (v - v_mid)))
    5. ASYMMETRIC: Lateral and Longitudinal independent velocity scaling
    """
    def __init__(self, horizon_s=0.90, dt_step=0.045, margin_mode="QUADRATIC", d_min_m=0.020, d_max_m=0.055, v_max_m_s=4.40):
        self.horizon_s = horizon_s
        self.dt_step = dt_step
        self.num_steps = int(round(horizon_s / dt_step))
        self.margin_mode = margin_mode
        self.d_min = d_min_m
        self.d_max = d_max_m
        self.v_max = v_max_m_s

    def compute_dynamic_margins(self, v_mps, kappa=0.0):
        """Computes continuous velocity-dependent lateral and longitudinal safety buffers (no discrete steps)."""
        v_norm = np.clip(abs(v_mps) / max(self.v_max, 1.0), 0.0, 1.0)
        
        if self.margin_mode == "QUADRATIC":
            lat_margin = self.d_min + (self.d_max - self.d_min) * (v_norm ** 2)
            lon_margin = lat_margin * 1.5

        elif self.margin_mode == "LINEAR":
            lat_margin = self.d_min + (self.d_max - self.d_min) * v_norm
            lon_margin = lat_margin * 1.5

        elif self.margin_mode == "CURVATURE":
            a_lat = (v_mps ** 2) * abs(kappa)
            g_util = np.clip(a_lat / (0.60 * 9.81), 0.0, 1.0)
            lat_margin = self.d_min + (self.d_max - self.d_min) * g_util
            lon_margin = self.d_min + (self.d_max - self.d_min) * v_norm

        elif self.margin_mode == "SIGMOID":
            v_mid = 0.60 * self.v_max
            s_curve = 1.0 / (1.0 + math.exp(-2.5 * (abs(v_mps) - v_mid)))
            lat_margin = self.d_min + (self.d_max - self.d_min) * s_curve
            lon_margin = lat_margin * 1.5

        elif self.margin_mode == "ASYMMETRIC":
            lat_margin = self.d_min + (self.d_max - self.d_min) * (v_norm ** 1.5)
            lon_margin = 0.05 + 0.08 * abs(v_mps)

        else: # Default scalar
            lat_margin = self.d_min
            lon_margin = self.d_min

        return lat_margin, lon_margin

    def evaluate_rollout(self, rx, ry, ryaw, v_mps, steer_rad, tof_distances, frenet_helper, curr_idx, kappa=0.0):
        """
        Rolls out the kinematic bicycle model forward for `self.num_steps`.
        Returns:
            has_risk (bool): True if forward swept volume intersects obstacles.
            min_ttc (float): Estimated time to collision in seconds.
            closest_obs_dist (float): Minimum obstacle clearance.
            evasion_suggested_d (float): Lateral offset in Frenet frame to steer away from threat.
        """
        min_tof = float(np.min(tof_distances))
        
        # 1. Extract 2D obstacle world points from ToF
        ox_list = []
        oy_list = []
        from tof_pathfinders import TOF_FRONT_OFFSET_M, TOF_ZONE_ANGLES, TOF_MAX_RANGE_M, TOF_MIN_RANGE_M
        
        front_x = rx + TOF_FRONT_OFFSET_M * math.cos(ryaw)
        front_y = ry + TOF_FRONT_OFFSET_M * math.sin(ryaw)
        
        for i in range(12):
            dist = tof_distances[i]
            if TOF_MIN_RANGE_M < dist < TOF_MAX_RANGE_M:
                b_yaw = ryaw + TOF_ZONE_ANGLES[i]
                p_x = front_x + dist * math.cos(b_yaw)
                p_y = front_y + dist * math.sin(b_yaw)
                _, d_obs, _, _, _ = frenet_helper.cartesian_to_frenet(p_x, p_y, curr_idx)
                if abs(d_obs) <= 0.18:
                    ox_list.append(p_x)
                    oy_list.append(p_y)

        if not ox_list:
            return False, 99.0, min_tof, 0.0

        obs_pts = np.column_stack((ox_list, oy_list)) # (M, 2)

        # 2. Continuous Dynamic Margin Calculation
        lat_margin, lon_margin = self.compute_dynamic_margins(v_mps, kappa)
        car_half_w = HALF_CAR_WIDTH_M + lat_margin
        car_half_l = 0.10 + lon_margin

        # 3. Kinematic Forward Rollout
        sim_x = rx
        sim_y = ry
        sim_yaw = ryaw
        sim_v = max(abs(v_mps), 0.80)
        sim_steer = steer_rad
        
        has_risk = False
        min_ttc = 99.0
        threat_lateral_offsets = []

        for step in range(1, self.num_steps + 1):
            t_elapsed = step * self.dt_step
            # Kinematic step
            beta = math.atan(0.5 * math.tan(sim_steer))
            sim_x += sim_v * math.cos(sim_yaw + beta) * self.dt_step
            sim_y += sim_v * math.sin(sim_yaw + beta) * self.dt_step
            sim_yaw += (sim_v / WHEELBASE_M) * math.cos(beta) * math.tan(sim_steer) * self.dt_step

            # Transform obstacle points to vehicle local frame
            dx = obs_pts[:, 0] - sim_x
            dy = obs_pts[:, 1] - sim_y
            
            cos_y = math.cos(sim_yaw)
            sin_y = math.sin(sim_yaw)
            
            local_x = dx * cos_y + dy * sin_y
            local_y = -dx * sin_y + dy * cos_y

            # Swept volume bounds with speed-dependent dynamic margins
            in_x = (-car_half_l <= local_x) & (local_x <= car_half_l)
            in_y = np.abs(local_y) <= car_half_w
            penetrations = in_x & in_y

            if np.any(penetrations):
                has_risk = True
                min_ttc = min(min_ttc, t_elapsed)
                threat_y = np.mean(local_y[penetrations])
                threat_lateral_offsets.append(threat_y)

        # Calculate evasion suggestion if risk exists
        evasion_d = 0.0
        if has_risk and threat_lateral_offsets:
            avg_threat_y = np.mean(threat_lateral_offsets)
            evasion_d = -0.14 if avg_threat_y >= 0.0 else +0.14

        return has_risk, min_ttc, min_tof, evasion_d
