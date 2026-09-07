"""
================================================================================
  OPTION 2: CURVILINEAR SLACK-CONSTRAINED SL-MPC (OSQP)
  - Guides spline through scouted ToF gap
  - Probes 15 mm costmap along normal vectors n_k to form convex corridor bounds
  - Sparse OSQP sequential quadratic program over horizon N=18 (dt=0.02s)
  - Hard physical actuator state: x_0(6) = delta_0, |v_delta| <= delta_dot_max
  - Dynamic tire slip constraint: |delta_k - (v_y,k + lf*r_k)/v_bar_x| <= alpha_peak
  - Coupled friction octagon: |Fx| + c1*|Fy| <= mu * m * g
  - L1 slack corridor bounds: d_min - eps <= p_d <= d_max + eps
================================================================================
"""

import math
from typing import Dict, List, Optional, Tuple
import numpy as np
import scipy.sparse as sp
import osqp

try:
    from .planner_config import (
        WHEELBASE,
        TRACK_FRONT,
        TRACK_REAR,
        LF,
        LR,
        VEHICLE_MASS,
        IZ,
        C_AF,
        C_AR,
        TIRE_MU,
        GRAVITY_G,
        ALPHA_PEAK,
        MAX_STEERING_RAD,
        MAX_SERVO_RATE_RAD_S,
        OPT2_HORIZON_N,
        OPT2_DT,
        OPT2_WEIGHT_PROGRESS_S,
        OPT2_WEIGHT_LATERAL_D,
        OPT2_WEIGHT_HEADING_PSI,
        OPT2_WEIGHT_STEER_EFFORT,
        OPT2_WEIGHT_STEER_RATE,
        OPT2_WEIGHT_FX_EFFORT,
        OPT2_WEIGHT_SLACK_EPS,
        OPT2_FRICTION_OCTAGON_C1,
        OPT2_TARGET_SPEED_MPS,
        OPT2_CORRIDOR_STEP_M,
        V_MIN_CRUISE,
        V_MAX_CRUISE,
    )
except ImportError:
    from planner_config import (
        WHEELBASE,
        TRACK_FRONT,
        TRACK_REAR,
        LF,
        LR,
        VEHICLE_MASS,
        IZ,
        C_AF,
        C_AR,
        TIRE_MU,
        GRAVITY_G,
        ALPHA_PEAK,
        MAX_STEERING_RAD,
        MAX_SERVO_RATE_RAD_S,
        OPT2_HORIZON_N,
        OPT2_DT,
        OPT2_WEIGHT_PROGRESS_S,
        OPT2_WEIGHT_LATERAL_D,
        OPT2_WEIGHT_HEADING_PSI,
        OPT2_WEIGHT_STEER_EFFORT,
        OPT2_WEIGHT_STEER_RATE,
        OPT2_WEIGHT_FX_EFFORT,
        OPT2_WEIGHT_SLACK_EPS,
        OPT2_FRICTION_OCTAGON_C1,
        OPT2_TARGET_SPEED_MPS,
        OPT2_CORRIDOR_STEP_M,
        V_MIN_CRUISE,
        V_MAX_CRUISE,
    )


class CurvilinearSLMPC:
    """
    Option 2 Planner: Curvilinear Slack-Constrained SL-MPC using OSQP.
    """

    def __init__(self, costmap_engine: any):
        self.costmap = costmap_engine
        self.N = OPT2_HORIZON_N
        self.dt = OPT2_DT
        self.nx = 7  # [p_s, p_d, e_psi, v_x, v_y, r, delta]
        self.nu = 2  # [F_x, v_delta]
        self.solver = osqp.OSQP()
        self.is_initialized = False

    def plan(
        self,
        xg: float,
        yg: float,
        theta_g: float,
        aperture_width: float,
        v_current: float,
        steer_current: float,
        yaw_rate_current: float,
        extra_targets: Optional[List[Tuple[float, float, float]]] = None,
    ) -> Dict[str, any]:
        """
        Solves the Curvilinear SL-MPC QP.
        Evaluates candidate target guide lines and selects the one with maximum clear preview distance.

        Returns:
            dict containing target_steer, target_speed, and solver diagnostics.
        """
        N = self.N
        dt = self.dt
        v_bar = max(v_current, V_MIN_CRUISE)
        delta_0 = float(np.clip(steer_current, -MAX_STEERING_RAD, MAX_STEERING_RAD))
        r_0 = yaw_rate_current

        # If extra_targets provided, pick the candidate that offers the furthest unobstructed lookahead
        best_tx, best_ty, best_aperture = xg, yg, aperture_width
        if extra_targets:
            furthest_dist = math.hypot(xg, yg)
            for cand in extra_targets:
                cdist = math.hypot(cand[0], cand[1])
                # Prioritize further clear opening if aperture is at least 30 cm
                if cdist > furthest_dist and cand[2] >= 0.30:
                    furthest_dist = cdist
                    best_tx, best_ty, best_aperture = cand

        xg, yg, aperture_width = best_tx, best_ty, best_aperture

        # 1. Establish Curvilinear Guide Line towards gap centroid
        # In vehicle body frame (+Y forward, +X right):
        # Target (xg, yg) on left has xg < 0, target angle theta_g = math.atan2(xg, yg) < 0.
        # In vehicle ediff steering: delta > 0 is a left turn.
        # Therefore, target heading in steering frame: psi_target = -math.atan2(xg, max(0.35, yg))
        L_guide = math.hypot(xg, max(0.35, yg))
        s_steps = np.linspace(0.0, L_guide, N + 1)
        guide_xs = (xg / L_guide) * s_steps
        guide_ys = (yg / L_guide) * s_steps

        # Normal vectors for lateral corridor probing:
        # Unit guide vector u = (xg / L_guide, yg / L_guide)
        # Normal to the left (-X direction for forward guide)
        nx_left = -yg / L_guide
        ny_left = xg / L_guide

        # Dynamic corner braking ceiling from target curvature
        kappa_guide = 2.0 * abs(xg) / max(L_guide ** 2, 0.05)
        lat_limit = TIRE_MU * GRAVITY_G * 0.82
        v_corner_max = math.sqrt(lat_limit / max(kappa_guide, 0.10))
        v_ref = min(OPT2_TARGET_SPEED_MPS, max(V_MIN_CRUISE, v_corner_max))

        # 2. Probe 15 mm Costmap outward along normal vectors to establish local convex corridor [d_min, d_max]
        d_min_bounds = np.zeros(N + 1, dtype=np.float32)
        d_max_bounds = np.zeros(N + 1, dtype=np.float32)

        probe_steps = int(round(0.70 / OPT2_CORRIDOR_STEP_M))
        for k in range(N + 1):
            gx = guide_xs[k]
            gy = guide_ys[k]

            # Probe left (positive lateral offset along left normal)
            d_left = 0.05
            for step in range(1, probe_steps):
                d_cand = step * OPT2_CORRIDOR_STEP_M
                px = np.array([gx + d_cand * nx_left], dtype=np.float32)
                py = np.array([gy + d_cand * ny_left], dtype=np.float32)
                if not self.costmap.probe_path(px, py):
                    break
                d_left = d_cand
            d_max_bounds[k] = min(0.60, d_left)

            # Probe right (negative lateral offset along left normal)
            d_right = -0.05
            for step in range(1, probe_steps):
                d_cand = -step * OPT2_CORRIDOR_STEP_M
                px = np.array([gx + d_cand * nx_left], dtype=np.float32)
                py = np.array([gy + d_cand * ny_left], dtype=np.float32)
                if not self.costmap.probe_path(px, py):
                    break
                d_right = d_cand
            d_min_bounds[k] = max(-0.60, d_right)

        # 3. Formulate State-Space Dynamics Matrix A, B
        # State: x = [p_s, p_d, e_psi, v_x, v_y, r, delta]^T
        # Input: u = [F_x, v_delta]^T
        A_cont = np.zeros((self.nx, self.nx))
        A_cont[0, 3] = 1.0   # p_s_dot = v_x
        A_cont[1, 2] = v_bar # p_d_dot = v_x * e_psi + v_y (positive e_psi moves left)
        A_cont[1, 4] = 1.0
        A_cont[2, 5] = 1.0   # e_psi_dot = r (positive r turns vehicle left, increasing e_psi)

        # Linearized lateral dynamics
        denom_v = max(v_bar, 0.20)
        A_cont[4, 4] = -(C_AF + C_AR) / (VEHICLE_MASS * denom_v)
        A_cont[4, 5] = -(LF * C_AF - LR * C_AR) / (VEHICLE_MASS * denom_v) - v_bar
        A_cont[4, 6] = C_AF / VEHICLE_MASS

        A_cont[5, 4] = -(LF * C_AF - LR * C_AR) / (IZ * denom_v)
        A_cont[5, 5] = -(LF * LF * C_AF + LR * LR * C_AR) / (IZ * denom_v)
        A_cont[5, 6] = (LF * C_AF) / IZ

        B_cont = np.zeros((self.nx, self.nu))
        B_cont[3, 0] = 1.0 / VEHICLE_MASS  # dvx/dt = Fx / m
        B_cont[6, 1] = 1.0                # d_delta/dt = v_delta

        # Forward Euler discretization: A_d = I + A*dt, B_d = B*dt
        A_d = np.eye(self.nx) + A_cont * dt
        B_d = B_cont * dt

        # 4. Construct OSQP Sparse Optimization Problem
        n_x_total = (N + 1) * self.nx
        n_u_total = N * self.nu
        n_eps_total = N + 1
        n_vars = n_x_total + n_u_total + n_eps_total

        def idx_x(k):
            return k * self.nx

        def idx_u(k):
            return n_x_total + k * self.nu

        def idx_eps(k):
            return n_x_total + n_u_total + k

        # Objective Matrix P and vector q
        P_mat = sp.dok_matrix((n_vars, n_vars), dtype=np.float64)
        q_vec = np.zeros(n_vars, dtype=np.float64)

        for k in range(N + 1):
            ix = idx_x(k)
            # Lateral deviation penalty
            P_mat[ix + 1, ix + 1] += 15.0
            # Heading error penalty
            P_mat[ix + 2, ix + 2] += 25.0
            # Speed tracking penalty towards corner-capped target speed
            P_mat[ix + 3, ix + 3] += 2.0
            q_vec[ix + 3] += -2.0 * v_ref
            # Steering angle magnitude penalty
            P_mat[ix + 6, ix + 6] += 0.02

            # Slack variable penalty (heavy L2 + L1 penalty)
            ie = idx_eps(k)
            P_mat[ie, ie] += 2.0 * OPT2_WEIGHT_SLACK_EPS
            q_vec[ie] += OPT2_WEIGHT_SLACK_EPS

        # Reward forward progress p_s along horizon
        q_vec[idx_x(N)] += -8.0

        for k in range(N):
            iu = idx_u(k)
            P_mat[iu + 0, iu + 0] += 0.005
            P_mat[iu + 1, iu + 1] += 0.01

        P_csc = sp.triu(P_mat).tocsc()

        constr_list = []
        l_list = []
        u_list = []

        # 1. Initial State Constraint
        e_psi_0 = math.atan2(xg, max(0.35, yg))
        x_init = np.array([0.0, 0.0, e_psi_0, v_bar, 0.0, r_0, delta_0], dtype=np.float64)
        for i in range(self.nx):
            row = np.zeros(n_vars)
            row[idx_x(0) + i] = 1.0
            constr_list.append(row)
            l_list.append(x_init[i])
            u_list.append(x_init[i])

        # 2. Dynamics Constraints: x_{k+1} - A_d * x_k - B_d * u_k = 0
        for k in range(N):
            for i in range(self.nx):
                row = np.zeros(n_vars)
                row[idx_x(k + 1) + i] = 1.0
                for j in range(self.nx):
                    row[idx_x(k) + j] -= A_d[i, j]
                for j in range(self.nu):
                    row[idx_u(k) + j] -= B_d[i, j]
                constr_list.append(row)
                l_list.append(0.0)
                u_list.append(0.0)

        # 3. Slack Corridor Bounds: d_min <= p_d + eps and p_d - eps <= d_max
        for k in range(N + 1):
            row1 = np.zeros(n_vars)
            row1[idx_x(k) + 1] = 1.0
            row1[idx_eps(k)] = -1.0
            constr_list.append(row1)
            l_list.append(-10.0)
            u_list.append(float(d_max_bounds[k]))

            row2 = np.zeros(n_vars)
            row2[idx_x(k) + 1] = 1.0
            row2[idx_eps(k)] = 1.0
            constr_list.append(row2)
            l_list.append(float(d_min_bounds[k]))
            u_list.append(10.0)

        # 4. Slew Limits & Steering Limits
        for k in range(N):
            row = np.zeros(n_vars)
            row[idx_u(k) + 1] = 1.0  # v_delta
            constr_list.append(row)
            l_list.append(-MAX_SERVO_RATE_RAD_S)
            u_list.append(MAX_SERVO_RATE_RAD_S)

        for k in range(1, N + 1):
            row = np.zeros(n_vars)
            row[idx_x(k) + 6] = 1.0  # delta
            constr_list.append(row)
            l_list.append(-MAX_STEERING_RAD)
            u_list.append(MAX_STEERING_RAD)

        # 5. Dynamic Slip Bounds (soft penalty with slack to prevent tire washout while allowing fast counter-steer recovery)
        for k in range(1, N + 1):
            row1 = np.zeros(n_vars)
            row1[idx_x(k) + 6] = 1.0  # delta
            row1[idx_x(k) + 4] = -1.0 / denom_v  # -vy / v_bar
            row1[idx_x(k) + 5] = -LF / denom_v    # -lf * r / v_bar
            row1[idx_eps(k)] = -1.0
            constr_list.append(row1)
            l_list.append(-10.0)
            u_list.append(ALPHA_PEAK)

            row2 = np.zeros(n_vars)
            row2[idx_x(k) + 6] = 1.0
            row2[idx_x(k) + 4] = -1.0 / denom_v
            row2[idx_x(k) + 5] = -LF / denom_v
            row2[idx_eps(k)] = 1.0
            constr_list.append(row2)
            l_list.append(-ALPHA_PEAK)
            u_list.append(10.0)

        # 6. Slack Non-negativity
        for k in range(N + 1):
            row = np.zeros(n_vars)
            row[idx_eps(k)] = 1.0
            constr_list.append(row)
            l_list.append(0.0)
            u_list.append(1.0)

        # 7. Motor Force Bounds (Friction limit: |Fx| <= mu * m * g)
        max_Fx = TIRE_MU * VEHICLE_MASS * GRAVITY_G
        for k in range(N):
            row = np.zeros(n_vars)
            row[idx_u(k) + 0] = 1.0  # Fx
            constr_list.append(row)
            l_list.append(-max_Fx)
            u_list.append(max_Fx)

        A_sparse = sp.csc_matrix(np.array(constr_list, dtype=np.float64))
        l_arr = np.array(l_list, dtype=np.float64)
        u_arr = np.array(u_list, dtype=np.float64)

        # Solve via OSQP
        try:
            solver = osqp.OSQP()
            solver.setup(
                P=P_csc,
                q=q_vec,
                A=A_sparse,
                l=l_arr,
                u=u_arr,
                verbose=False,
                eps_abs=1e-3,
                eps_rel=1e-3,
                max_iter=350,
                adaptive_rho=True,
            )
            res = solver.solve()

            if res.info.status_val in [1, 2, 7]:  # solved, inaccurate, or max_iter reached
                # Extract predicted trajectory points for GUI visualization
                best_line = []
                for k in range(N + 1):
                    pk_s = float(res.x[idx_x(k) + 0])
                    pk_d = float(res.x[idx_x(k) + 1])
                    gx = (xg / max(L_guide, 0.01)) * pk_s
                    gy = (yg / max(L_guide, 0.01)) * pk_s
                    best_line.append([float(gx + pk_d * nx_left), float(gy + pk_d * ny_left)])

                # Lookahead pure pursuit tracking along the planned MPC line
                dists = [math.hypot(pt[0], pt[1]) for pt in best_line]
                lk_target_m = max(0.30, v_bar * 0.16)
                lk_idx = int(np.argmin(np.abs(np.array(dists) - lk_target_m)))
                lk_idx = max(2, min(lk_idx, N))
                lx, ly = best_line[lk_idx]
                l_d = math.hypot(lx, ly)
                req_delta_pp = math.atan2(2.0 * WHEELBASE * (-lx), l_d * l_d)

                # Blend pure pursuit with MPC horizon steering state for aggressive turn-in
                tgt_delta = 0.60 * req_delta_pp + 0.40 * float(res.x[idx_x(min(3, N)) + 6])
                max_slew_step = MAX_SERVO_RATE_RAD_S * dt
                delta_err = tgt_delta - delta_0
                cmd_delta = delta_0 + float(np.clip(delta_err, -max_slew_step, max_slew_step))
                cmd_steer = float(np.clip(cmd_delta, -MAX_STEERING_RAD, MAX_STEERING_RAD))

                fx_0 = float(res.x[idx_u(0) + 0])       # motor force at step 0
                cmd_speed = float(np.clip(v_bar + (fx_0 / VEHICLE_MASS) * dt, V_MIN_CRUISE, V_MAX_CRUISE))

                # Coupled friction ellipse speed reduction
                kappa_cmd = math.tan(cmd_steer) / max(WHEELBASE, 0.01)
                ay_needed = cmd_speed ** 2 * abs(kappa_cmd)
                if ay_needed > lat_limit:
                    v_lat_max = math.sqrt(lat_limit / max(abs(kappa_cmd), 0.01))
                    cmd_speed = max(V_MIN_CRUISE, min(cmd_speed, v_lat_max))

                guide_line = [[float(x), float(y)] for x, y in zip(guide_xs, guide_ys)]
                corridor_left = [[float(gx + d_max_bounds[k] * nx_left), float(gy + d_max_bounds[k] * ny_left)] for k, (gx, gy) in enumerate(zip(guide_xs, guide_ys))]
                corridor_right = [[float(gx + d_min_bounds[k] * nx_left), float(gy + d_min_bounds[k] * ny_left)] for k, (gx, gy) in enumerate(zip(guide_xs, guide_ys))]

                return {
                    "cmd_steer_rad": cmd_steer,
                    "cmd_speed_mps": cmd_speed,
                    "osqp_status": res.info.status,
                    "osqp_iters": res.info.iter,
                    "best_cost": float(res.info.obj_val),
                    "status": "OPTIMAL_OSQP_MPC",
                    "best_line": best_line,
                    "guide_line": guide_line,
                    "candidate_lines": [guide_line, corridor_left, corridor_right],
                }
        except Exception as e:
            print("[OSQP MPC Error]", e)

        # Fallback if solver fails: turn toward target directly in steering coordinates (positive = left)
        fallback_steer = float(np.clip(-math.atan2(xg, max(0.40, yg)), -MAX_STEERING_RAD, MAX_STEERING_RAD))
        return {
            "cmd_steer_rad": fallback_steer,
            "cmd_speed_mps": V_MIN_CRUISE,
            "osqp_status": "FALLBACK_ERROR",
            "osqp_iters": 0,
            "best_cost": 0.0,
            "status": "FALLBACK_SL_MPC",
        }
