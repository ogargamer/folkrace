"""
================================================================================
  16-CHANNEL TOF & 15mm COSTMAP MULTI-TARGET GAP SCOUT MODULE

  Key Upgrades:
  1. RAYTRACING ON INFLATED COSTMAP (Hazard Mask):
     Directly casts radial lines across the front FOV through the expanded
     hazard grid (15mm resolution, dilated for vehicle clearance).
     For each line, computes the exact unobstructed distance to the first obstacle.
  2. MULTI-TARGET DISCOVERY:
     Finds the top local maxima (distinct furthest clear targets) so the vehicle
     is not stuck to a single centerline point.
  3. APERTURE & HEADING ESTIMATION:
     Provides primary target (xg, yg, width, theta) plus alternative targets
     so the trajectory planners evaluate candidates toward multiple openings.
================================================================================
"""

import math
from typing import Dict, List, Optional, Tuple
import numpy as np

try:
    from .planner_config import (
        GAP_MIN_WIDTH_M,
        GAP_MIN_LOOKAHEAD_M,
        GAP_MAX_LOOKAHEAD_M,
        GRID_X_MIN,
        GRID_X_MAX,
        GRID_Y_MIN,
        GRID_Y_MAX,
        GRID_RES,
        TOF_MAX_RANGE_M,
        R_INFLATION_M,
        WEIGHT_GAP_DISTANCE,
        WEIGHT_GAP_WIDTH,
        WEIGHT_ALIGN_STEER,
        WEIGHT_ALIGN_STRAIGHT,
        WEIGHT_SIDE_CLEARANCE,
        TRACK_CORRIDOR_WIDTH_M,
        GAP_PROGRESS_MARGIN_M,
        GAP_DIST_EXP_K,
        ALT_MIN_SEPARATION_M,
        ALT_APERTURE_BIAS,
    )
except ImportError:
    from planner_config import (
        GAP_MIN_WIDTH_M,
        GAP_MIN_LOOKAHEAD_M,
        GAP_MAX_LOOKAHEAD_M,
        GRID_X_MIN,
        GRID_X_MAX,
        GRID_Y_MIN,
        GRID_Y_MAX,
        GRID_RES,
        TOF_MAX_RANGE_M,
        R_INFLATION_M,
        WEIGHT_GAP_DISTANCE,
        WEIGHT_GAP_WIDTH,
        WEIGHT_ALIGN_STEER,
        WEIGHT_ALIGN_STRAIGHT,
        WEIGHT_SIDE_CLEARANCE,
        TRACK_CORRIDOR_WIDTH_M,
        GAP_PROGRESS_MARGIN_M,
        GAP_DIST_EXP_K,
        ALT_MIN_SEPARATION_M,
        ALT_APERTURE_BIAS,
    )

# 16-Channel Physical ToF Sensor Mounting Poses
# Format: (sensor_name, mount_x, mount_y, heading_rad, max_range_m)
FRONT_TOF_SENSOR_RIG = [
    # Front-Left Array (X = -0.070 m, Y = +0.095 m, cross-pointing Right)
    ("tof_front_left_4.1deg",  -0.070, 0.095, 1.4987, 1.80),
    ("tof_front_left_15.4deg", -0.070, 0.095, 1.3024, 1.80),
    ("tof_front_left_26.6deg", -0.070, 0.095, 1.1060, 1.80),
    ("tof_front_left_37.9deg", -0.070, 0.095, 0.9097, 1.80),
    ("tof_front_left_49.1deg", -0.070, 0.095, 0.7134, 1.80),
    ("tof_front_left_60.4deg", -0.070, 0.095, 0.5170, 1.80),
    ("tof_front_left_71.6deg", -0.070, 0.095, 0.3207, 1.80),
    ("tof_front_left_82.9deg", -0.070, 0.095, 0.1243, 1.80),
    # Front-Right Array (X = +0.070 m, Y = +0.095 m, cross-pointing Left)
    ("tof_front_right_4.1deg",   0.070, 0.095, 1.6428, 1.80),
    ("tof_front_right_15.4deg",  0.070, 0.095, 1.8391, 1.80),
    ("tof_front_right_26.6deg",  0.070, 0.095, 2.0356, 1.80),
    ("tof_front_right_37.9deg",  0.070, 0.095, 2.2318, 1.80),
    ("tof_front_right_49.1deg",  0.070, 0.095, 2.4282, 1.80),
    ("tof_front_right_60.4deg",  0.070, 0.095, 2.6245, 1.80),
    ("tof_front_right_71.6deg",  0.070, 0.095, 2.8208, 1.80),
    ("tof_front_right_82.9deg",  0.070, 0.095, 3.0172, 1.80),
]

LEGACY_TOF_SENSOR_RIG = [
    ("tof_fl_0", -0.070,  0.095, 0.9097, 1.80),
    ("tof_fl_1", -0.070,  0.095, 1.1060, 1.80),
    ("tof_fl_2", -0.070,  0.095, 1.3024, 1.80),
    ("tof_fl_3", -0.070,  0.095, 1.4987, 1.80),
    ("tof_fr_0",  0.070,  0.095, 2.2318, 1.80),
    ("tof_fr_1",  0.070,  0.095, 2.0356, 1.80),
    ("tof_fr_2",  0.070,  0.095, 1.8391, 1.80),
    ("tof_fr_3",  0.070,  0.095, 1.6428, 1.80),
    ("tof_bl_0", -0.070, -0.095, 1.6957, 1.80),
    ("tof_bl_1", -0.070, -0.095, 1.8920, 1.80),
    ("tof_bl_2", -0.070, -0.095, 2.0883, 1.80),
    ("tof_bl_3", -0.070, -0.095, 2.2847, 1.80),
    ("tof_br_0",  0.070, -0.095, 1.4462, 1.80),
    ("tof_br_1",  0.070, -0.095, 1.2499, 1.80),
    ("tof_br_2",  0.070, -0.095, 1.0536, 1.80),
    ("tof_br_3",  0.070, -0.095, 0.8573, 1.80),
]

TOF_SENSOR_RIG = FRONT_TOF_SENSOR_RIG

# 16-Channel ToF Pod Geometry:
# 8 zones per corner, each zone has 11.25 deg FOV (8 * 11.25 deg = 90.0 deg total FOV per pod).
# Zone midpoints: 4.125 deg, 15.375 deg, ..., 82.875 deg.
# Zone outer bounds: from -1.5 deg (-0.0262 rad) to +88.5 deg (+1.5446 rad).
_NUM_CORNER_RAYS = 90

# Left Pod (-0.07m, 0.095m, cross-pointing Right): -1.5 deg to +88.5 deg rel to +Y
_LEFT_REL_ANGLES = np.linspace(math.radians(-1.5), math.radians(88.5), _NUM_CORNER_RAYS)
_LEFT_AZIMUTHS = (math.pi / 2.0) - _LEFT_REL_ANGLES
_LEFT_COS = np.cos(_LEFT_AZIMUTHS)
_LEFT_SIN = np.sin(_LEFT_AZIMUTHS)

# Right Pod (+0.07m, 0.095m, cross-pointing Left): +1.5 deg to -88.5 deg rel to +Y (i.e. -1.5 deg to +88.5 deg leftwards)
_RIGHT_REL_ANGLES = np.linspace(math.radians(1.5), -math.radians(88.5), _NUM_CORNER_RAYS)
_RIGHT_AZIMUTHS = (math.pi / 2.0) - _RIGHT_REL_ANGLES
_RIGHT_COS = np.cos(_RIGHT_AZIMUTHS)
_RIGHT_SIN = np.sin(_RIGHT_AZIMUTHS)


class ToFGapScout:
    """
    Scouts furthest points and navigable openings by combining direct ToF sensor
    beam sightlines (LUT) with C++ inflated obstacle raycasting.
    """

    def __init__(
        self,
        w_gap_distance: float = WEIGHT_GAP_DISTANCE,
        w_gap_width: float = WEIGHT_GAP_WIDTH,
        w_align_steer: float = WEIGHT_ALIGN_STEER,
        w_align_straight: float = WEIGHT_ALIGN_STRAIGHT,
        w_side_clearance: float = WEIGHT_SIDE_CLEARANCE,
        gap_dist_exp_k: float = GAP_DIST_EXP_K,
        gap_progress_margin_m: float = GAP_PROGRESS_MARGIN_M,
        alt_min_separation_m: float = ALT_MIN_SEPARATION_M,
        alt_aperture_bias: float = ALT_APERTURE_BIAS,
        gap_latch_hysteresis: float = 1.25,
        gap_latch_dist_m: float = 0.28,
        ray_origin_offset_m: float = 0.070,
        **kwargs,
    ):
        self.w_gap_distance = float(w_gap_distance)
        self.w_gap_width = float(w_gap_width)
        self.w_align_steer = float(w_align_steer)
        self.w_align_straight = float(w_align_straight)
        self.w_side_clearance = float(w_side_clearance)
        self.gap_dist_exp_k = float(gap_dist_exp_k)
        self.gap_progress_margin_m = float(gap_progress_margin_m)
        self.alt_min_separation_m = float(alt_min_separation_m)
        self.alt_aperture_bias = float(alt_aperture_bias)
        self.gap_latch_hysteresis = float(gap_latch_hysteresis)
        self.gap_latch_dist_m = float(gap_latch_dist_m)
        self.ray_origin_offset_m = float(ray_origin_offset_m)
        self.last_xg: float = 0.0
        self.last_yg: float = 1.20
        self.last_aperture: float = 0.50
        self.last_targets: List[Tuple[float, float, float]] = []
        self.active_corridor_x: Optional[float] = None
        self.active_corridor_y: Optional[float] = None

    def set_active_target(self, x: float, y: float) -> None:
        """Sets the active corridor anchor so Slot 0 reliably tracks this path branch."""
        self.active_corridor_x = float(x)
        self.active_corridor_y = float(y)

    def reset_active_target(self) -> None:
        """Resets active corridor anchor (e.g. after recovery or on wide-open track)."""
        self.active_corridor_x = None
        self.active_corridor_y = None

    def scout_gap(
        self,
        tof_readings: Dict[str, float],
        hazard_mask: Optional[np.ndarray] = None,
        costmap_obj: Optional[any] = None,
        target_lookahead_m: float = 1.70,
        steer_current: float = 0.0,
        prob_map: Optional[np.ndarray] = None,
    ) -> Tuple[float, float, float, float]:
        """
        Returns primary (x_g, y_g, aperture_width_m, recovery_heading_rad).
        Also stores multi-target options in self.last_targets: [(xg, yg, score), ...].
        """
        targets = self.find_furthest_targets(
            tof_readings=tof_readings,
            hazard_mask=hazard_mask,
            costmap_obj=costmap_obj,
            max_lookahead_m=target_lookahead_m,
            top_k=2,
            steer_current=steer_current,
        )

        if not targets:
            xg_prim = 0.0
            yg_prim = max(GAP_MIN_LOOKAHEAD_M, min(target_lookahead_m, 1.70))
            aperture = 0.45
        else:
            xg_prim, yg_prim, aperture = targets[0]

        # Raw physical target coordinates directly without EMA smoothing or midpoint blending
        self.last_xg = xg_prim
        self.last_yg = yg_prim
        self.last_aperture = aperture
        self.last_targets = targets

        theta_g = math.atan2(xg_prim, yg_prim)
        return xg_prim, yg_prim, aperture, theta_g

    def find_furthest_targets(
        self,
        tof_readings: Optional[Dict[str, float]] = None,
        hazard_mask: Optional[np.ndarray] = None,
        costmap_obj: Optional[any] = None,
        max_lookahead_m: float = 1.40,
        top_k: int = 3,
        steer_current: float = 0.0,
        prob_map: Optional[np.ndarray] = None,
    ) -> List[Tuple[float, float, float]]:
        """
        Extracts furthest targets by evaluating real ToF beam sightlines and dense radial
        rays through the C++ inflated costmap (hazard_mask).
        Guarantees targets lie on physical clear space at the true lookahead distance
        without colliding with inflated obstacle boundaries.
        Returns list of (x, y, aperture_width).
        """
        if hazard_mask is None or costmap_obj is None:
            return [(0.0, max_lookahead_m, 0.40)]

        res = costmap_obj.res
        x_min = costmap_obj.x_min
        y_min = costmap_obj.y_min
        h, w = hazard_mask.shape
        step_ds = max(res * 0.8, 0.012)

        candidates = []

        # ----------------------------------------------------------------------
        # 1. EVALUATE 16 PHYSICAL TOF BEAM SIGHTLINES (LUT + MEASURED RANGES)
        # ----------------------------------------------------------------------
        if tof_readings:
            active_rig = FRONT_TOF_SENSOR_RIG if any("front" in k for k in tof_readings) else LEGACY_TOF_SENSOR_RIG
            for s_name, sx, sy, heading, max_range in active_rig:
                raw_z = tof_readings.get(s_name, max_range)
                z = min(max_range, max(0.10, raw_z))
                cos_h = math.cos(heading)
                sin_h = math.sin(heading)

                clear_dist = z
                n_steps = int(z / step_ds)
                for s in range(1, n_steps + 1):
                    d = s * step_ds
                    px = sx + d * cos_h
                    py = sy + d * sin_h
                    col = int(round((px - x_min) / res))
                    row = int(round((py - y_min) / res))
                    if row < 0 or row >= h or col < 0 or col >= w:
                        clear_dist = max(GAP_MIN_LOOKAHEAD_M, d - step_ds)
                        break
                    if hazard_mask[row, col] > 0:
                        clear_dist = max(GAP_MIN_LOOKAHEAD_M, d - step_ds)
                        break

                # Target at maximum clear beam distance (backed off slightly for vehicle margin)
                eff_dist = max(GAP_MIN_LOOKAHEAD_M, clear_dist - 0.06)
                tx = sx + eff_dist * cos_h
                ty = max(0.18, sy + eff_dist * sin_h)
                total_dist = math.hypot(tx, ty)

                # Heading in steering coordinates (positive = left)
                rel_angle = math.atan2(tx, ty)
                steer_equiv = -rel_angle

                # Aperture estimation around this beam
                aperture_m = max(GAP_MIN_WIDTH_M, min(1.20, total_dist * 0.35))
                candidates.append({
                    "tx": tx, "ty": ty, "dist": total_dist,
                    "aperture": aperture_m, "steer_equiv": steer_equiv,
                    "rel_angle": rel_angle,
                })

        # ----------------------------------------------------------------------
        # 2. EVALUATE DENSE RADIAL RAYS (180 TOTAL: 90 PER FRONT CORNER POD)
        # Left Corner (-0.07m, 0.095m):  90 rays spanning -89 deg to 0 deg (Left to Center)
        # Right Corner (+0.07m, 0.095m): 90 rays spanning 0 deg to +89 deg (Center to Right)
        # ----------------------------------------------------------------------
        max_r = min(max_lookahead_m, GAP_MAX_LOOKAHEAD_M)
        n_steps = int(max_r / step_ds)

        corner_fans = [
            (-self.ray_origin_offset_m, 0.095, _LEFT_COS, _LEFT_SIN),
            (self.ray_origin_offset_m, 0.095, _RIGHT_COS, _RIGHT_SIN),
        ]

        for ox, oy, cos_arr, sin_arr in corner_fans:
            for i in range(_NUM_CORNER_RAYS):
                cos_az = float(cos_arr[i])
                sin_az = float(sin_arr[i])
                clear_dist = max_r

                for s in range(1, n_steps + 1):
                    d = s * step_ds
                    px = ox + d * cos_az
                    py = oy + d * sin_az
                    col = int(round((px - x_min) / res))
                    row = int(round((py - y_min) / res))
                    if row < 0 or row >= h or col < 0 or col >= w:
                        clear_dist = max(GAP_MIN_LOOKAHEAD_M, d - step_ds)
                        break
                    if hazard_mask[row, col] > 0:
                        clear_dist = max(GAP_MIN_LOOKAHEAD_M, d - step_ds)
                        break

                eff_dist = max(GAP_MIN_LOOKAHEAD_M, clear_dist - 0.06)
                # Physical target point in vehicle local coordinates (origin at vehicle center / rear axle)
                tx = ox + eff_dist * cos_az
                ty = max(0.18, oy + eff_dist * sin_az)
                total_dist = math.hypot(tx, ty)

                # True relative angle from vehicle origin (0, 0) to target (tx, ty)
                rel_angle = math.atan2(tx, ty)
                steer_equiv = -rel_angle
                aperture_m = max(GAP_MIN_WIDTH_M, min(1.20, total_dist * 0.30))

                candidates.append({
                    "tx": tx, "ty": ty, "dist": total_dist,
                    "aperture": aperture_m, "steer_equiv": steer_equiv,
                    "rel_angle": rel_angle,
                })

        if not candidates:
            return [(0.0, max_lookahead_m, 0.40)]

        max_clear_dist = max([c["dist"] for c in candidates])
        has_far_path = max_clear_dist > 0.85
        min_allowed_dist = max(GAP_MIN_LOOKAHEAD_M, max_clear_dist * 0.45) if has_far_path else GAP_MIN_LOOKAHEAD_M

        # ----------------------------------------------------------------------
        # 3. SCORE CANDIDATE TARGETS
        # ----------------------------------------------------------------------
        # Check forward clearance directly along vehicle centerline (|rel_angle| < 18 deg)
        center_dists = [c["dist"] for c in candidates if abs(c["rel_angle"]) < math.radians(18)]
        d_center = min(center_dists) if center_dists else max_clear_dist

        # Compute side clearance asymmetry from lateral ToF sensors:
        side_diff = 0.0
        if tof_readings:
            if "tof_front_right_82.9deg" in tof_readings:
                # Right sensors cross-point left; Left sensors cross-point right
                left_keys = ["tof_front_right_82.9deg", "tof_front_right_71.6deg", "tof_front_right_60.4deg"]
                right_keys = ["tof_front_left_82.9deg", "tof_front_left_71.6deg", "tof_front_left_60.4deg"]
            else:
                left_keys = ["tof_fl_0", "tof_fl_1", "tof_bl_0", "tof_bl_1"]
                right_keys = ["tof_fr_0", "tof_fr_1", "tof_br_0", "tof_br_1"]
            l_vals = [tof_readings.get(k, 1.80) for k in left_keys]
            r_vals = [tof_readings.get(k, 1.80) for k in right_keys]
            l_mean = sum(l_vals) / len(l_vals) if l_vals else 1.80
            r_mean = sum(r_vals) / len(r_vals) if r_vals else 1.80
            denom = max(0.25, 0.5 * (l_mean + r_mean))
            side_diff = (l_mean - r_mean) / denom

        # When forward space is closing (d_center < 1.10m), suppress the straight preference!
        # If d_center is very short (< 0.95m), heavily penalize straight targets to force committing to a turn
        if d_center < 1.10:
            w_straight = 0.0
            straight_block_penalty = max(0.0, (0.95 - d_center) / 0.35) * 3.0
        else:
            w_straight = self.w_align_straight
            straight_block_penalty = 0.0

        # Relax steer alignment penalty when facing an obstacle or turn
        w_steer = self.w_align_steer if (abs(steer_current) > 0.08 and d_center >= 1.0) else 0.20

        scored_candidates = []
        for c in candidates:
            if c["dist"] < min_allowed_dist:
                continue

            d_cand = c["dist"]
            d_thresh = TRACK_CORRIDOR_WIDTH_M + self.gap_progress_margin_m
            dist_base = d_cand / max(max_clear_dist, 0.10)
            if d_cand > d_thresh:
                # Slight exponential boost for distances exceeding 1.0m + margin (open track corridor towards progress)
                excess = d_cand - d_thresh
                dist_factor = dist_base * math.exp(self.gap_dist_exp_k * excess)
            else:
                dist_factor = dist_base

            width_norm = c["aperture"] / 1.00
            steer_diff = abs(c["steer_equiv"] - steer_current)
            straight_diff = abs(c["rel_angle"])

            # Side clearance alignment bonus:
            # c["rel_angle"] < 0 is left. -sin(rel_angle) > 0 for left targets.
            # If side_diff > 0 (left has longer reading), left targets receive a strong bonus!
            side_bonus = self.w_side_clearance * side_diff * (-math.sin(c["rel_angle"]))

            # Penalize center/straight targets if dead ahead is blocked by wall/obstacle
            center_penalty = straight_block_penalty if abs(c["rel_angle"]) < math.radians(20) else 0.0

            score = (
                self.w_gap_distance * dist_factor
                + self.w_gap_width * width_norm
                + side_bonus
                - center_penalty
                - w_steer * steer_diff
                - w_straight * straight_diff
            )
            scored_candidates.append((score, c))

        scored_candidates.sort(key=lambda item: item[0], reverse=True)

        # ----------------------------------------------------------------------
        # 4. SMART DUAL-PATH SELECTION (Primary Racing Line + 1 Smart Challenger)
        # ----------------------------------------------------------------------
        targets = []
        if scored_candidates:
            # Check if an active corridor anchor is present to lock Slot 0
            c0 = None
            best_score, best_cand = scored_candidates[0]
            if self.active_corridor_x is not None and self.active_corridor_y is not None:
                act_x = self.active_corridor_x
                act_y = self.active_corridor_y
                for sc, c in scored_candidates:
                    tx, ty = c["tx"], c["ty"]
                    same_side = (act_x < -0.05 and tx < -0.02) or (act_x > 0.05 and tx > 0.02)
                    near_active = math.hypot(tx - act_x, ty - act_y) <= self.gap_latch_dist_m
                    if same_side or near_active:
                        # Only retain the latched corridor if it isn't severely inferior to the best opening.
                        # Hysteresis buffer prevents chattering between equal left/right openings around center obstacles,
                        # while letting genuine open corners break through cleanly.
                        if (sc + self.gap_latch_hysteresis) >= best_score:
                            c0 = c
                        break

            # Fallback if no candidate matched the active corridor, on cold start, or when broken by superior opening
            if c0 is None:
                c0 = best_cand

            self.active_corridor_x = c0["tx"]
            self.active_corridor_y = c0["ty"]

            tx0, ty0, ap0 = c0["tx"], c0["ty"], c0["aperture"]
            targets.append((tx0, ty0, ap0))

            if top_k >= 2:
                # Target 1: Smart Challenger Line
                # Select the best distinct corridor opening on the opposite side of
                # the obstacle / centerline or separated by at least alt_min_separation_m.
                # The challenger is scored with alt_aperture_bias weighting safe clearance.
                challenger_candidates = []
                for _, c in scored_candidates:
                    tx, ty, ap = c["tx"], c["ty"], c["aperture"]
                    lat_sep = abs(tx - tx0)
                    is_opposite_side = (tx0 < -0.04 and tx > 0.04) or (tx0 > 0.04 and tx < -0.04)

                    if lat_sep >= self.alt_min_separation_m or is_opposite_side:
                        # Compute challenger score with aperture bias
                        d_cand = c["dist"]
                        d_thresh = TRACK_CORRIDOR_WIDTH_M + self.gap_progress_margin_m
                        dist_base = d_cand / max(max_clear_dist, 0.10)
                        if d_cand > d_thresh:
                            dist_factor = dist_base * math.exp(self.gap_dist_exp_k * (d_cand - d_thresh))
                        else:
                            dist_factor = dist_base

                        width_norm = ap / 1.00
                        steer_diff = abs(c["steer_equiv"] - steer_current)
                        straight_diff = abs(c["rel_angle"])
                        side_bonus_alt = (self.w_side_clearance * 0.45) * side_diff * (-math.sin(c["rel_angle"]))
                        center_penalty = straight_block_penalty if abs(c["rel_angle"]) < math.radians(20) else 0.0

                        score_alt = (
                            self.w_gap_distance * dist_factor
                            + (self.w_gap_width * self.alt_aperture_bias) * width_norm
                            + side_bonus_alt
                            - center_penalty
                            - w_steer * steer_diff
                            - w_straight * straight_diff
                        )
                        challenger_candidates.append((score_alt, c))

                if challenger_candidates:
                    challenger_candidates.sort(key=lambda item: item[0], reverse=True)
                    best_challenger = challenger_candidates[0][1]
                    targets.append((best_challenger["tx"], best_challenger["ty"], best_challenger["aperture"]))
                else:
                    # Fallback: next distinct candidate by spatial separation >= 15 cm
                    for _, c in scored_candidates:
                        tx, ty, ap = c["tx"], c["ty"], c["aperture"]
                        if math.hypot(tx - tx0, ty - ty0) >= 0.15:
                            targets.append((tx, ty, ap))
                            break

                # Additional targets if top_k > 2 requested
                if top_k > 2:
                    for _, c in scored_candidates:
                        tx, ty, ap = c["tx"], c["ty"], c["aperture"]
                        if any(math.hypot(tx - ex, ty - ey) < 0.18 for (ex, ey, _) in targets):
                            continue
                        targets.append((tx, ty, ap))
                        if len(targets) >= top_k:
                            break

        # Fallback if no target met threshold
        if not targets:
            best_c = max(candidates, key=lambda x: x["dist"])
            targets = [(best_c["tx"], best_c["ty"], 0.40)]

        return targets

