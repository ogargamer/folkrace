"""
================================================================================
  OPTION 3: G^3 SEPTIC SPLINE POOL + TRACTION PROFILER
  - Generates 12 candidate 7th-order (septic) polynomials anchored into ToF gap
  - Matches G^3 actuator boundary conditions (curvature and curvature rate) at s=0
  - Instant C++ collision rejection via costmap_probe_path (<30 us)
  - Bidirectional numerical dynamic pass along arc-length s:
    * Dynamic steering with understeer gradient K_us
    * Understeer & servo slew velocity ceilings
    * Backward trail-braking integration
    * Forward motor acceleration integration
  - Selects trajectory minimizing traversal time T = sum(ds / v(s))
================================================================================
"""

import math
from typing import Dict, List, Optional, Tuple
import numpy as np

try:
    from .planner_config import (
        WHEELBASE,
        VEHICLE_MASS,
        C_AF,
        C_AR,
        UNDERSTEER_K_US,
        TIRE_MU,
        GRAVITY_G,
        ALPHA_PEAK,
        MAX_STEERING_RAD,
        MAX_SERVO_RATE_RAD_S,
        OPT3_NUM_SPLINES,
        OPT3_ACCEL_MAX,
        OPT3_DECEL_MAX,
        OPT3_DS,
        OPT3_TOP_SPEED_MPS,
        OPT3_PROBE_SAMPLES,
        OPT3_STRAIGHT_LOCK_DIST_M,
        OPT3_STRAIGHT_LOCK_SPEED_MPS,
        V_MIN_CRUISE,
        V_MAX_CRUISE,
        DT_PLANNER,
    )
except ImportError:
    from planner_config import (
        WHEELBASE,
        VEHICLE_MASS,
        C_AF,
        C_AR,
        UNDERSTEER_K_US,
        TIRE_MU,
        GRAVITY_G,
        ALPHA_PEAK,
        MAX_STEERING_RAD,
        MAX_SERVO_RATE_RAD_S,
        OPT3_NUM_SPLINES,
        OPT3_ACCEL_MAX,
        OPT3_DECEL_MAX,
        OPT3_DS,
        OPT3_TOP_SPEED_MPS,
        OPT3_PROBE_SAMPLES,
        OPT3_STRAIGHT_LOCK_DIST_M,
        OPT3_STRAIGHT_LOCK_SPEED_MPS,
        V_MIN_CRUISE,
        V_MAX_CRUISE,
        DT_PLANNER,
    )

# Precomputed inverse of the 4x4 septic system matrix M:
# [ 1,  1,   1,   1]
# [ 4,  5,   6,   7]
# [12, 20,  30,  42]
# [24, 60, 120, 210]
M_INV_SEPTIC = np.linalg.inv(np.array([
    [ 1.0,  1.0,   1.0,   1.0],
    [ 4.0,  5.0,   6.0,   7.0],
    [12.0, 20.0,  30.0,  42.0],
    [24.0, 60.0, 120.0, 210.0]
], dtype=np.float64))


def solve_septic_coeffs(
    p0: float, v0: float, a0: float, j0: float,
    p1: float, v1: float, a1: float, j1: float
) -> np.ndarray:
    """
    Computes coefficients c0..c7 for 7th-order polynomial:
    P(u) = sum_{j=0}^7 c_j * u^j, u in [0, 1].
    """
    c0 = p0
    c1 = v0
    c2 = 0.5 * a0
    c3 = (1.0 / 6.0) * j0

    rhs = np.array([
        p1 - (c0 + c1 + c2 + c3),
        v1 - (c1 + 2.0 * c2 + 3.0 * c3),
        a1 - (2.0 * c2 + 6.0 * c3),
        j1 - (6.0 * c3)
    ], dtype=np.float64)

    high_coeffs = M_INV_SEPTIC @ rhs
    return np.array([c0, c1, c2, c3, high_coeffs[0], high_coeffs[1], high_coeffs[2], high_coeffs[3]], dtype=np.float64)


def eval_septic(coeffs: np.ndarray, u: np.ndarray):
    """Evaluates position, first derivative, and second derivative of septic spline."""
    c = coeffs
    u2 = u * u
    u3 = u2 * u
    u4 = u3 * u
    u5 = u4 * u
    u6 = u5 * u
    u7 = u6 * u

    p = c[0] + c[1]*u + c[2]*u2 + c[3]*u3 + c[4]*u4 + c[5]*u5 + c[6]*u6 + c[7]*u7
    v = c[1] + 2.0*c[2]*u + 3.0*c[3]*u2 + 4.0*c[4]*u3 + 5.0*c[5]*u4 + 6.0*c[6]*u5 + 7.0*c[7]*u6
    a = 2.0*c[2] + 6.0*c[3]*u + 12.0*c[4]*u2 + 20.0*c[5]*u3 + 30.0*c[6]*u4 + 42.0*c[7]*u5
    return p, v, a


class SepticSplineTractionProfilerPython:
    """
    Option 3 Planner: Pure Python G^3 Septic Spline Pool + Bidirectional Traction Profiler.
    Preserved for numerical validation and CPU fallback.
    """

    def __init__(self, costmap_engine: any):
        self.costmap = costmap_engine
        self.u_samples = np.linspace(0.0, 1.0, OPT3_PROBE_SAMPLES, dtype=np.float32)

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
        steer_rate_current: float = 0.0,
    ) -> Dict[str, any]:
        """
        Executes one G^3 Septic planning and dynamic profiling cycle.
        Evaluates candidate splines toward all scouted targets to select the fastest
        dynamically feasible path.

        Returns:
            dict containing target_steer, target_speed, and profiling diagnostics.
        """
        v0 = max(v_current, V_MIN_CRUISE)
        delta0 = float(np.clip(steer_current, -MAX_STEERING_RAD, MAX_STEERING_RAD))
        cos_d0 = math.cos(delta0)
        cos2_d0 = max(cos_d0 * cos_d0, 0.10)

        # 0. Fast Straight-Line Lock (Zero Compute Acceleration Bypass)
        lock_dist = float(getattr(self, "straight_lock_dist_m", OPT3_STRAIGHT_LOCK_DIST_M))
        lock_spd = float(getattr(self, "straight_lock_speed_mps", OPT3_STRAIGHT_LOCK_SPEED_MPS))
        straight_xs = np.zeros(OPT3_PROBE_SAMPLES, dtype=np.float32)
        straight_ys = np.linspace(0.05, lock_dist, OPT3_PROBE_SAMPLES, dtype=np.float32)

        if self.costmap.probe_path(straight_xs, straight_ys) and abs(xg) <= 0.25 and abs(theta_g) <= 0.25:
            return {
                "cmd_steer_rad": 0.0,
                "cmd_speed_mps": lock_spd,
                "surviving_count": 1,
                "candidate_count": 1,
                "best_line": [[float(straight_xs[i]), float(straight_ys[i])] for i in range(OPT3_PROBE_SAMPLES)],
                "candidate_lines": [[[float(straight_xs[i]), float(straight_ys[i])] for i in range(OPT3_PROBE_SAMPLES)]],
                "lap_time": lock_dist / max(lock_spd, 1.0),
                "status": "STRAIGHT_LOCKED",
            }

        # 1. Generate Candidate Pool of Septic Polynomials
        # Fanning across primary target and any scouted alternatives
        target_list = [(xg, yg, aperture_width)]
        if extra_targets:
            for cand in extra_targets:
                if math.hypot(cand[0] - xg, cand[1] - yg) > 0.12:
                    target_list.append(cand)

        candidate_splines = []
        lat_offsets = [-0.08, 0.0, 0.08]
        heading_offsets = [-0.20, 0.0, 0.20]
        blend_weights = [0.30, 0.55, 0.85]

        for (curr_tx, curr_ty, curr_aperture) in target_list:
            curr_theta_g = math.atan2(curr_tx, max(0.35, curr_ty))
            for d_lat in lat_offsets:
                if abs(d_lat) > (curr_aperture * 0.40):
                    continue
                xt = curr_tx + d_lat
                yt = max(curr_ty, 0.35)
                lt = math.hypot(xt, yt)
                theta_chord = math.atan2(xt, yt)

                for d_head in heading_offsets:
                    target_head = curr_theta_g + d_head

                    for blend in blend_weights:
                        # Initial state at u=0: Tangent along chassis forward (+Y) with optional chord bias,
                        # curvature kappa_0 matches current steering angle tan(delta0)/wheelbase:
                        head_0 = blend * theta_chord
                        kappa_0 = math.tan(delta0) / WHEELBASE

                        # Initial state at u=0 (vehicle origin):
                        vx0 = lt * math.sin(head_0)
                        vy0 = lt * math.cos(head_0)
                        ax0 = -(lt * lt) * kappa_0 * math.cos(head_0)
                        ay0 = (lt * lt) * kappa_0 * math.sin(head_0)

                        # Exact G^3 Initial Actuator Jerk Matching with Calmed Proportional Slew Injection
                        turn_angle = -theta_chord
                        ref_angle = max(MAX_STEERING_RAD * 0.75, 0.25)
                        turn_ratio = float(np.clip(turn_angle / ref_angle, -1.0, 1.0))
                        turn_sign = 1.0 if turn_ratio >= 0.0 else -1.0
                        turn_eased = turn_sign * (abs(turn_ratio) ** 2)

                        slew_scale = getattr(self, "initial_slew_scale", 0.12)
                        effective_steer_rate = (
                            steer_rate_current
                            if abs(steer_rate_current) > 0.08
                            else (turn_eased * MAX_SERVO_RATE_RAD_S * slew_scale)
                        )

                        cos_d0 = math.cos(delta0)
                        cos2_d0 = max(cos_d0 * cos_d0, 0.10)
                        dkappa_ds_0 = effective_steer_rate / (WHEELBASE * cos2_d0 * max(v0, 0.5))
                        lt3 = lt * lt * lt
                        jx0 = -lt3 * dkappa_ds_0 * math.cos(head_0)
                        jy0 = lt3 * dkappa_ds_0 * math.sin(head_0)

                        # Target state at u=1:
                        vx1 = lt * math.sin(target_head)
                        vy1 = lt * math.cos(target_head)
                        ax1 = 0.0
                        ay1 = 0.0
                        jx1 = 0.0
                        jy1 = 0.0

                        x_coeffs = solve_septic_coeffs(0.0, vx0, ax0, jx0, xt, vx1, ax1, jx1)
                        y_coeffs = solve_septic_coeffs(0.0, vy0, ay0, jy0, yt, vy1, ay1, jy1)

                        xs, dx_du, d2x_du2 = eval_septic(x_coeffs, self.u_samples)
                        ys, dy_du, d2y_du2 = eval_septic(y_coeffs, self.u_samples)

                        candidate_splines.append({
                            "d_lat": d_lat,
                            "target_head": target_head,
                            "blend": blend,
                            "xs": xs,
                            "ys": ys,
                            "dx_du": dx_du,
                            "dy_du": dy_du,
                            "d2x_du2": d2x_du2,
                            "d2y_du2": d2y_du2,
                        })

        # 3. Fast C++ Collision Filtering via costmap_probe_path (<30 us)
        surviving_splines = []
        for spline in candidate_splines:
            if self.costmap.probe_path(spline["xs"], spline["ys"]):
                surviving_splines.append(spline)

        if not surviving_splines:
            # Check 3 short emergency escape probe lines (20cm straight and +-10 degrees)
            # before falling back to blind proportional steering:
            escape_dist = 0.20
            deg10_rad = 0.174533
            escape_pts = 10
            escapes = [
                {"steer_cmd": 0.0, "angle": 0.0, "clear": False},
                {"steer_cmd": deg10_rad, "angle": deg10_rad, "clear": False},
                {"steer_cmd": -deg10_rad, "angle": -deg10_rad, "clear": False},
            ]

            for esc in escapes:
                ang = esc["angle"]
                sin_a = math.sin(ang)
                cos_a = math.cos(ang)
                s_vals = np.linspace(0.02, escape_dist, escape_pts, dtype=np.float32)
                xs_esc = -s_vals * sin_a
                ys_esc = s_vals * cos_a
                esc["clear"] = bool(self.costmap.probe_path(xs_esc, ys_esc))
                esc["xs"] = xs_esc
                esc["ys"] = ys_esc

            chosen = None
            if xg < -0.05:
                if escapes[1]["clear"]: chosen = escapes[1]
                elif escapes[0]["clear"]: chosen = escapes[0]
                elif escapes[2]["clear"]: chosen = escapes[2]
            elif xg > 0.05:
                if escapes[2]["clear"]: chosen = escapes[2]
                elif escapes[0]["clear"]: chosen = escapes[0]
                elif escapes[1]["clear"]: chosen = escapes[1]
            else:
                if escapes[0]["clear"]: chosen = escapes[0]
                elif escapes[1]["clear"]: chosen = escapes[1]
                elif escapes[2]["clear"]: chosen = escapes[2]

            if chosen is not None:
                esc_line = [[float(x), float(y)] for x, y in zip(chosen["xs"], chosen["ys"])]
                return {
                    "cmd_steer_rad": chosen["steer_cmd"],
                    "cmd_speed_mps": V_MIN_CRUISE,
                    "surviving_count": 1,
                    "candidate_count": 3,
                    "status": "SURVIVED_SHORT_PROBE",
                    "best_line": esc_line,
                    "candidate_lines": [esc_line],
                    "lap_time": escape_dist / V_MIN_CRUISE,
                }

            # Safe fallback if all 3 short probe lines also fail (positive = left)
            fallback_steer = float(np.clip(-math.atan2(xg, max(0.40, yg)), -MAX_STEERING_RAD, MAX_STEERING_RAD))
            return {
                "cmd_steer_rad": fallback_steer,
                "cmd_speed_mps": V_MIN_CRUISE,
                "surviving_count": 0,
                "status": "FALLBACK_NO_SURVIVORS",
            }

        # 4. Bidirectional Dynamic Traction Profiler along Surviving Splines
        best_lap_time = float("inf")
        best_spline = surviving_splines[0]
        best_v_cmd = V_MIN_CRUISE
        best_steer_cmd = 0.0

        for spline in surviving_splines:
            xs = spline["xs"]
            ys = spline["ys"]
            dx = spline["dx_du"]
            dy = spline["dy_du"]
            d2x = spline["d2x_du2"]
            d2y = spline["d2y_du2"]

            # Step 1: Road Curvature kappa(s) and rate dkappa/ds
            denom = np.maximum((dx * dx + dy * dy) ** 1.5, 1e-6)
            kappa = (dx * d2y - dy * d2x) / denom
            ds_arr = np.hypot(np.diff(xs), np.diff(ys))
            ds_mean = float(np.mean(ds_arr)) if len(ds_arr) > 0 else OPT3_DS

            # Central differentiation for curvature rate dkappa/ds
            dkappa_ds = np.gradient(kappa, ds_mean)

            # Step 2: Understeer Ceiling & Servo Slew Ceiling with 15% Grip Reserve
            n_pts = len(kappa)
            v_max_k = np.zeros(n_pts, dtype=np.float64)
            mu_g = TIRE_MU * GRAVITY_G
            mu_g_eff = mu_g * 0.85

            for i in range(n_pts):
                k_abs = abs(kappa[i])
                v_corner = math.sqrt(mu_g_eff / (k_abs + UNDERSTEER_K_US * mu_g + 1e-4))
                dk_abs = max(abs(dkappa_ds[i]), 1e-3)
                v_slew = MAX_SERVO_RATE_RAD_S / (WHEELBASE * dk_abs * cos2_d0)
                v_max_k[i] = min(OPT3_TOP_SPEED_MPS, v_corner, v_slew)

            # Step 3: Backward Pass: Integrate trail-braking with Kamm Friction Circle
            v_profile = np.zeros(n_pts, dtype=np.float64)
            v_profile[-1] = v_max_k[-1]

            for i in range(n_pts - 2, -1, -1):
                v_next = v_profile[i + 1]
                lat_accel = (v_next * v_next) * abs(kappa[i + 1])
                grip_ratio = min(1.0, lat_accel / mu_g_eff)
                available_braking = mu_g_eff * math.sqrt(max(0.0, 1.0 - grip_ratio * grip_ratio))
                a_brake = min(OPT3_DECEL_MAX, available_braking)
                v_cand = math.sqrt(v_next * v_next + 2.0 * a_brake * ds_mean)
                v_profile[i] = min(v_max_k[i], v_cand)

            # Step 4: Forward Pass: Integrate motor acceleration with Kamm Friction Circle
            v_profile[0] = min(v_profile[0], max(v0, V_MIN_CRUISE))
            for i in range(n_pts - 1):
                v_curr = v_profile[i]
                lat_accel = (v_curr * v_curr) * abs(kappa[i])
                grip_ratio = min(1.0, lat_accel / mu_g_eff)
                available_accel = mu_g_eff * math.sqrt(max(0.0, 1.0 - grip_ratio * grip_ratio))
                a_accel_i = min(OPT3_ACCEL_MAX, available_accel)
                v_next_accel = math.sqrt(v_curr * v_curr + 2.0 * a_accel_i * ds_mean)
                v_profile[i + 1] = min(v_profile[i + 1], v_next_accel)

            # Traversal Time: T = sum(ds / v(s))
            lap_time = float(np.sum(ds_mean / np.maximum(v_profile, 0.20)))

            if lap_time < best_lap_time:
                best_lap_time = lap_time
                best_spline = spline
                best_v_cmd = float(np.clip(v_profile[1], V_MIN_CRUISE, V_MAX_CRUISE))

                # Dynamic lookahead anchor scaled with candidate speed:
                lookahead_target_m = float(np.clip(best_v_cmd * 0.30, 0.35, 0.80))
                dists = np.hypot(xs, ys)
                lk_idx = int(np.argmin(np.abs(dists - lookahead_target_m)))
                lk_idx = max(2, min(lk_idx, n_pts - 1))

                lx = xs[lk_idx]
                ly = ys[lk_idx]
                l_dist = math.hypot(lx, ly)
                l_dist_sq = max(1e-4, lx * lx + ly * ly)

                # Combined Stanley / Pure-Pursuit: Spline tangent angle + cross-track curvature
                theta_tangent = -math.atan2(best_spline["dx_du"][lk_idx], best_spline["dy_du"][lk_idx])
                pure_pursuit_angle = math.atan2(2.0 * WHEELBASE * (-lx), l_dist_sq)
                req_delta = 0.60 * theta_tangent + 0.80 * pure_pursuit_angle + UNDERSTEER_K_US * (best_v_cmd ** 2) * (2.0 * (-lx) / l_dist_sq)

                # Maximum slew-rate execution per 60 Hz planner step
                max_slew_step = MAX_SERVO_RATE_RAD_S * float(DT_PLANNER)
                delta_err = req_delta - delta0
                cmd_delta = delta0 + float(np.clip(delta_err, -max_slew_step, max_slew_step))
                best_steer_cmd = float(np.clip(cmd_delta, -MAX_STEERING_RAD, MAX_STEERING_RAD))

                # Friction ellipse speed reduction if lateral demand exceeds grip limit
                kappa_cmd = math.tan(best_steer_cmd) / max(WHEELBASE, 0.01)
                ay_needed = best_v_cmd ** 2 * abs(kappa_cmd)
                lat_limit = TIRE_MU * GRAVITY_G
                if ay_needed > lat_limit * 0.80:
                    v_lat_max = math.sqrt(lat_limit * 0.80 / max(abs(kappa_cmd), 0.01))
                    best_v_cmd = max(V_MIN_CRUISE, min(best_v_cmd, v_lat_max))

        # Side Flank Obstacle Guard: Prevent steering into obstacles touching the vehicle flanks
        left_flank_xs = np.array([-0.09, -0.09, -0.09, -0.09], dtype=np.float32)
        left_flank_ys = np.array([0.02, 0.07, 0.12, 0.18], dtype=np.float32)
        right_flank_xs = np.array([0.09, 0.09, 0.09, 0.09], dtype=np.float32)
        right_flank_ys = np.array([0.02, 0.07, 0.12, 0.18], dtype=np.float32)

        left_blocked = not bool(self.costmap.probe_path(left_flank_xs, left_flank_ys))
        right_blocked = not bool(self.costmap.probe_path(right_flank_xs, right_flank_ys))

        if left_blocked and right_blocked:
            best_steer_cmd = 0.0
        elif left_blocked and best_steer_cmd > 0.0:
            best_steer_cmd = 0.0
        elif right_blocked and best_steer_cmd < 0.0:
            best_steer_cmd = 0.0

        # Return spline curves for GUI visualization
        best_line = [[float(x), float(y)] for x, y in zip(best_spline["xs"], best_spline["ys"])]
        candidate_lines = []
        for s in candidate_splines:
            candidate_lines.append([[float(x), float(y)] for x, y in zip(s["xs"], s["ys"])])

        return {
            "cmd_steer_rad": best_steer_cmd,
            "cmd_speed_mps": best_v_cmd,
            "best_lap_time": best_lap_time,
            "surviving_count": len(surviving_splines),
            "status": "OPTIMAL_SEPTIC_TRACTION",
            "best_line": best_line,
            "candidate_lines": candidate_lines,
        }


# ==============================================================================
# HIGH-PERFORMANCE NATIVE C++ ENGINE BINDING WITH PYTHON FALLBACK
# ==============================================================================
try:
    from cpp_planner_option3_septic import CppSepticSplineTractionProfiler, CPP_AVAILABLE
except ImportError:
    try:
        from .cpp_planner_option3_septic import CppSepticSplineTractionProfiler, CPP_AVAILABLE
    except ImportError:
        CppSepticSplineTractionProfiler = None
        CPP_AVAILABLE = False

if CPP_AVAILABLE and CppSepticSplineTractionProfiler is not None:
    # Use optimized C++ AVX2 engine as default high-speed path planner
    SepticSplineTractionProfiler = CppSepticSplineTractionProfiler
else:
    # Transparent graceful fallback to Python implementation
    SepticSplineTractionProfiler = SepticSplineTractionProfilerPython

__all__ = [
    "SepticSplineTractionProfiler",
    "SepticSplineTractionProfilerPython",
    "CppSepticSplineTractionProfiler",
]

