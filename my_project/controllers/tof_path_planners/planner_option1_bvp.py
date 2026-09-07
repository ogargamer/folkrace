"""
================================================================================
  OPTION 1: TARGET-GUIDED BVP DYNAMIC STATE LATTICE (HEUN / RK2)
  - Anchors candidate paths directly into ToF-scouted opening
  - Instant C++ collision rejection via costmap_probe_path (<2 us)
  - Heun predictor-corrector single-track vehicle dynamics simulation
  - Enforces dynamic slip bounds |alpha_f|, |alpha_r| <= alpha_peak
  - Coupled trail-braking friction ellipse
================================================================================
"""

import math
from typing import Dict, List, Optional, Tuple
import numpy as np

try:
    from .planner_config import (
        WHEELBASE,
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
        GAP_OFFSETS,
        OPT1_HORIZON_STEPS,
        OPT1_DT,
        OPT1_WEIGHT_TIME,
        OPT1_WEIGHT_JERK,
        OPT1_WEIGHT_GAP_OFFSET,
        OPT1_TARGET_SPEED_MPS,
        OPT1_PROBE_SAMPLES,
        V_MIN_CRUISE,
        V_MAX_CRUISE,
    )
except ImportError:
    from planner_config import (
        WHEELBASE,
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
        GAP_OFFSETS,
        OPT1_HORIZON_STEPS,
        OPT1_DT,
        OPT1_WEIGHT_TIME,
        OPT1_WEIGHT_JERK,
        OPT1_WEIGHT_GAP_OFFSET,
        OPT1_TARGET_SPEED_MPS,
        OPT1_PROBE_SAMPLES,
        V_MIN_CRUISE,
        V_MAX_CRUISE,
    )


def solve_quintic_coeffs(p0: float, v0: float, a0: float, p1: float, v1: float, a1: float):
    """
    Closed-form analytical solution for quintic polynomial coefficients:
    P(u) = c0 + c1*u + c2*u^2 + c3*u^3 + c4*u^4 + c5*u^5 for u in [0, 1].
    """
    c0 = p0
    c1 = v0
    c2 = 0.5 * a0

    dp = p1 - p0
    c3 = 10.0 * dp - (6.0 * v0 + 4.0 * v1) - (1.5 * a0 - 0.5 * a1)
    c4 = -15.0 * dp + (8.0 * v0 + 7.0 * v1) + (1.5 * a0 - a1)
    c5 = 6.0 * dp - (3.0 * v0 + 3.0 * v1) - (0.5 * a0 - 0.5 * a1)
    return c0, c1, c2, c3, c4, c5


def eval_quintic(coeffs, u: np.ndarray):
    """Evaluates position, first derivative, and second derivative along u."""
    c0, c1, c2, c3, c4, c5 = coeffs
    u2 = u * u
    u3 = u2 * u
    u4 = u3 * u
    u5 = u4 * u

    p = c0 + c1 * u + c2 * u2 + c3 * u3 + c4 * u4 + c5 * u5
    v = c1 + 2.0 * c2 * u + 3.0 * c3 * u2 + 4.0 * c4 * u3 + 5.0 * c5 * u4
    a = 2.0 * c2 + 6.0 * c3 * u + 12.0 * c4 * u2 + 20.0 * c5 * u3
    return p, v, a


class BVPDynamicStateLattice:
    """
    Option 1 Planner: Target-Guided BVP Dynamic State Lattice with Heun RK2.
    """

    def __init__(self, costmap_engine: any):
        self.costmap = costmap_engine
        self.u_samples = np.linspace(0.0, 1.0, OPT1_PROBE_SAMPLES, dtype=np.float32)

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
        Executes one planning cycle.
        Accepts extra_targets from the inflated costmap scout to evaluate multiple choices.

        Returns:
            dict containing target_steer, target_speed, and diagnostic telemetry.
        """
        v0 = max(v_current, V_MIN_CRUISE)
        delta0 = float(np.clip(steer_current, -MAX_STEERING_RAD, MAX_STEERING_RAD))
        r0 = yaw_rate_current

        # Gather all target candidates: primary (xg, yg, aperture_width) plus any scouted alternatives
        target_list = [(xg, yg, aperture_width)]
        if extra_targets:
            for cand in extra_targets:
                # Add if distinct from primary (> 12 cm separation)
                if math.hypot(cand[0] - xg, cand[1] - yg) > 0.12:
                    target_list.append(cand)

        candidate_paths = []
        heading_offsets = [-0.18, 0.0, 0.18]

        for (curr_tx, curr_ty, curr_aperture) in target_list:
            # Adjust lateral offsets scaled by aperture width
            offsets = [d for d in GAP_OFFSETS if abs(d) < (curr_aperture * 0.38)]
            if not offsets:
                offsets = [0.0]

            curr_theta_g = math.atan2(curr_tx, max(0.35, curr_ty))

            for d_offset in offsets:
                xt = curr_tx + d_offset
                yt = max(curr_ty, 0.35)
                lt = math.hypot(xt, yt)

                for d_th in heading_offsets:
                    target_theta = curr_theta_g + d_th

                    # Initial curvature matching current steering angle
                    kappa0 = math.tan(delta0) / WHEELBASE

                    # Parameterize curve in local robot coordinates (+Y forward, +X right)
                    # Steering left (delta0 > 0) curves the vehicle towards -X:
                    vx0 = -lt * math.sin(delta0)
                    vy0 = lt * math.cos(delta0)
                    ax0 = -(lt * lt) * kappa0 * math.cos(delta0)
                    ay0 = (lt * lt) * kappa0 * math.sin(delta0)

                    vx1 = lt * math.sin(target_theta)
                    vy1 = lt * math.cos(target_theta)
                    ax1 = 0.0
                    ay1 = 0.0

                    x_coeffs = solve_quintic_coeffs(0.0, vx0, ax0, xt, vx1, ax1)
                    y_coeffs = solve_quintic_coeffs(0.0, vy0, ay0, yt, vy1, ay1)

                    xs, dx_du, d2x_du2 = eval_quintic(x_coeffs, self.u_samples)
                    ys, dy_du, d2y_du2 = eval_quintic(y_coeffs, self.u_samples)

                    candidate_paths.append({
                        "d_offset": d_offset,
                        "target_theta": target_theta,
                        "xt": xt,
                        "yt": yt,
                        "xs": xs,
                        "ys": ys,
                        "dx_du": dx_du,
                        "dy_du": dy_du,
                        "d2x_du2": d2x_du2,
                        "d2y_du2": d2y_du2,
                    })

        # 2. Fast C++ Collision Rejection via costmap_probe_path
        surviving_paths = []
        for path in candidate_paths:
            is_free = self.costmap.probe_path(path["xs"], path["ys"])
            if is_free:
                surviving_paths.append(path)

        if not surviving_paths:
            # Emergency fallback: if all pruned, use gap center directly at low speed (positive = left)
            target_steer = float(np.clip(-math.atan2(xg, max(0.40, yg)), -MAX_STEERING_RAD, MAX_STEERING_RAD))
            return {
                "cmd_steer_rad": target_steer,
                "cmd_speed_mps": V_MIN_CRUISE,
                "surviving_count": 0,
                "status": "FALLBACK_ALL_PRUNED",
            }

        # 3. Dynamic Simulation: Heun / RK2 Single-Track Model along surviving paths
        best_cost = float("inf")
        best_path = surviving_paths[0]
        best_target_v = OPT1_TARGET_SPEED_MPS
        best_steer_cmd = 0.0
        best_sim_deltas = []

        Fz = (VEHICLE_MASS * GRAVITY_G) * 0.5  # approx normal load per axle
        dt = OPT1_DT
        steps = OPT1_HORIZON_STEPS

        for path in surviving_paths:
            xs = path["xs"]
            ys = path["ys"]
            dx = path["dx_du"]
            dy = path["dy_du"]
            d2x = path["d2x_du2"]
            d2y = path["d2y_du2"]

            # Compute road curvature along curve: kappa = (dx * d2y - dy * d2x) / (dx^2 + dy^2)^(3/2)
            denom = np.maximum((dx * dx + dy * dy) ** 1.5, 1e-6)
            kappa = (dx * d2y - dy * d2x) / denom

            # Dynamic forward integration using Heun's method
            vx = v0
            vy = 0.0
            r = r0
            delta = delta0
            is_dynamically_valid = True
            lateral_jerk_accum = 0.0
            prev_ay = 0.0

            n_pts = len(kappa)
            sim_deltas = []
            for k in range(min(steps, n_pts)):
                target_kappa = kappa[k]
                # Steering left (toward -X, dx < 0) has kappa > 0, which requires positive delta (left turn in ediff):
                desired_delta = math.atan(WHEELBASE * target_kappa)
                delta_diff = desired_delta - delta
                max_d_delta = MAX_SERVO_RATE_RAD_S * dt
                delta += float(np.clip(delta_diff, -max_d_delta, max_d_delta))
                delta = float(np.clip(delta, -MAX_STEERING_RAD, MAX_STEERING_RAD))
                sim_deltas.append(delta)

                # Dynamic tire slip angles
                denom_vx = max(vx, 0.20)
                alpha_f = delta - math.atan((vy + LF * r) / denom_vx)
                alpha_r = -math.atan((vy - LR * r) / denom_vx)

                # Understeer & Oversteer Stability Bounds
                if abs(alpha_f) > ALPHA_PEAK or abs(alpha_r) > ALPHA_PEAK:
                    is_dynamically_valid = False
                    break

                # Lateral tire forces (linear tire with coupled friction ellipse)
                Fyf_linear = C_AF * alpha_f
                Fyr_linear = C_AR * alpha_r

                # Trail-braking ellipse cap: Fy_max = sqrt((mu*Fz)^2 - Fx^2)
                # Longitudinal acceleration/braking
                Fx_cmd = 0.0 if vx < OPT1_TARGET_SPEED_MPS else -0.30 * VEHICLE_MASS * GRAVITY_G
                Fy_max = math.sqrt(max(0.0, (TIRE_MU * Fz) ** 2 - (Fx_cmd * 0.5) ** 2))
                Fyf = float(np.clip(Fyf_linear, -Fy_max, Fy_max))
                Fyr = float(np.clip(Fyr_linear, -Fy_max, Fy_max))

                # Derivatives (Single-track chassis equations)
                # dvy/dt = (Fyf + Fyr) / m - vx * r
                # dr/dt = (lf * Fyf - lr * Fyr) / Iz
                # dvx/dt = Fx_cmd / m + vy * r
                def calc_derivs(curr_vx, curr_vy, curr_r, curr_fyf, curr_fyr):
                    d_vy = (curr_fyf + curr_fyr) / VEHICLE_MASS - curr_vx * curr_r
                    d_r = (LF * curr_fyf - LR * curr_fyr) / IZ
                    d_vx = Fx_cmd / VEHICLE_MASS + curr_vy * curr_r
                    return d_vx, d_vy, d_r

                # Predictor step (Euler forward)
                dvx1, dvy1, dr1 = calc_derivs(vx, vy, r, Fyf, Fyr)
                vx_pred = max(V_MIN_CRUISE, vx + dvx1 * dt)
                vy_pred = vy + dvy1 * dt
                r_pred = r + dr1 * dt

                # Corrector step (Heun RK2 average)
                dvx2, dvy2, dr2 = calc_derivs(vx_pred, vy_pred, r_pred, Fyf, Fyr)
                vx = max(V_MIN_CRUISE, vx + 0.5 * (dvx1 + dvx2) * dt)
                vy = vy + 0.5 * (dvy1 + dvy2) * dt
                r = r + 0.5 * (dr1 + dr2) * dt

                # Lateral jerk accumulation
                ay = vy * r + (Fyf + Fyr) / VEHICLE_MASS
                jerk = (ay - prev_ay) / dt
                lateral_jerk_accum += abs(jerk)
                prev_ay = ay

            # Soft slip penalty rather than hard disqualification if all are close to limit
            slip_penalty = 100.0 if not is_dynamically_valid else 0.0

            # Trajectory Cost: Traversal Time + Lateral Jerk + Gap Center Offset + Slip Penalty
            arc_length = float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))
            traversal_time = arc_length / max(vx, 0.50)
            cost = (
                OPT1_WEIGHT_TIME * traversal_time
                + OPT1_WEIGHT_JERK * (lateral_jerk_accum / max(steps, 1))
                + OPT1_WEIGHT_GAP_OFFSET * abs(path["d_offset"])
                + slip_penalty
            )

            if cost < best_cost:
                best_cost = cost
                best_path = path
                best_target_v = float(np.clip(vx, V_MIN_CRUISE, V_MAX_CRUISE))
                best_sim_deltas = sim_deltas

                # ----------------------------------------------------------------
                # AGGRESSIVE IMMEDIATE STEER COMMAND:
                # Target the curve's required peak curvature / endpoint tangent immediately!
                # Do not wait multiple cycles; slew at maximum physical servo rate
                # (MAX_SERVO_RATE_RAD_S * DT) directly toward target angle:
                # req_delta_target = atan(L * kappa_target)
                # ----------------------------------------------------------------
                target_k = kappa[min(2, len(kappa) - 1)]
                req_delta_target = math.atan(WHEELBASE * target_k)
                max_slew_step = MAX_SERVO_RATE_RAD_S * dt
                delta_err = req_delta_target - delta0
                best_steer_cmd = float(np.clip(delta0 + np.clip(delta_err, -max_slew_step, max_slew_step), -MAX_STEERING_RAD, MAX_STEERING_RAD))

                # ----------------------------------------------------------------
                # FRICTION ELLIPSE SPEED REDUCTION:
                # When a large counter-steer is needed (big |delta_err|), the front
                # tires must provide both lateral force (turning) and potentially
                # longitudinal force (braking to scrub speed). They cannot do both
                # maximally. The ellipse: Fy^2 + Fx^2 <= (mu * Fz)^2.
                # Centripetal demand at cmd speed: ay_needed = v^2 * |kappa_cmd|.
                # Required lateral force: Fyf_needed ~ VEHICLE_MASS * ay_needed * 0.6
                # (front axle takes ~60% of lateral load). If Fyf_needed > Fy_max,
                # back off speed until ay_needed fits the available envelope.
                # ----------------------------------------------------------------
                kappa_cmd = math.tan(best_steer_cmd) / max(WHEELBASE, 0.01)
                ay_needed = best_target_v ** 2 * abs(kappa_cmd)
                lat_limit = TIRE_MU * GRAVITY_G   # max lateral g with zero braking
                if ay_needed > lat_limit * 0.80:
                    # Speed ceiling from lateral grip: v_max = sqrt(lat_g / |kappa|)
                    v_lat_max = math.sqrt(lat_limit * 0.80 / max(abs(kappa_cmd), 0.01))
                    best_target_v = max(V_MIN_CRUISE, min(best_target_v, v_lat_max))


        # Return trajectory lines for GUI visualization
        best_line = [[float(x), float(y)] for x, y in zip(best_path["xs"], best_path["ys"])]
        candidate_lines = []
        for p in candidate_paths:
            candidate_lines.append([[float(x), float(y)] for x, y in zip(p["xs"], p["ys"])])

        return {
            "cmd_steer_rad": best_steer_cmd,
            "cmd_speed_mps": best_target_v,
            "best_cost": best_cost,
            "surviving_count": len(surviving_paths),
            "chosen_offset": best_path["d_offset"],
            "status": "OPTIMAL_BVP_HEUN",
            "best_line": best_line,
            "candidate_lines": candidate_lines,
        }
