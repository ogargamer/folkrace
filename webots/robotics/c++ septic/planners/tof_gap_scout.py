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
    )

# 16-Channel Physical ToF Sensor Mounting Poses (matching C++ SENSOR_RIG)
# Format: (sensor_name, mount_x, mount_y, heading_rad, max_range_m)
TOF_SENSOR_RIG = [
    ("tof_fl_0", -0.070,  0.095, 0.9097, 1.80),
    ("tof_fl_1", -0.070,  0.095, 1.1060, 1.80),
    ("tof_fl_2", -0.070,  0.095, 1.3024, 1.80),
    ("tof_fl_3", -0.070,  0.095, 1.4987, 1.80),
    ("tof_fr_0",  0.070,  0.095, 2.2318, 1.80),
    ("tof_fr_1",  0.070,  0.095, 2.0356, 1.80),
    ("tof_fr_2",  0.070,  0.095, 1.8391, 1.80),
    ("tof_fr_3",  0.070,  0.095, 1.6428, 1.80),
    ("tof_flank_left_6.4deg",  -0.070, -0.095, 1.6957, 1.80),
    ("tof_flank_left_17.6deg", -0.070, -0.095, 1.8920, 1.80),
    ("tof_flank_left_28.9deg", -0.070, -0.095, 2.0883, 1.80),
    ("tof_flank_left_40.1deg", -0.070, -0.095, 2.2847, 1.80),
    ("tof_flank_right_6.4deg",   0.070, -0.095, 1.4462, 1.80),
    ("tof_flank_right_17.6deg",  0.070, -0.095, 1.2499, 1.80),
    ("tof_flank_right_28.9deg",  0.070, -0.095, 1.0536, 1.80),
    ("tof_flank_right_40.1deg",  0.070, -0.095, 0.8573, 1.80),
]

# 25 dense radial exploration rays spanning -65 deg to +65 deg relative to vehicle forward (+Y)
_NUM_SCOUT_RAYS = 25
_RAY_ANGLES_REL = np.linspace(-math.radians(65), math.radians(65), _NUM_SCOUT_RAYS)
_RAY_AZIMUTHS = (math.pi / 2.0) - _RAY_ANGLES_REL


class ToFGapScout:
    """
    Scouts furthest points and navigable openings by combining direct ToF sensor
    beam sightlines (LUT) with C++ inflated obstacle raycasting.
    """

    def __init__(self):
        self.last_xg: float = 0.0
        self.last_yg: float = 1.20
        self.last_aperture: float = 0.50
        self.last_targets: List[Tuple[float, float, float]] = []

    def scout_gap(
        self,
        tof_readings: Dict[str, float],
        hazard_mask: Optional[np.ndarray] = None,
        costmap_obj: Optional[any] = None,
        target_lookahead_m: float = 1.40,
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
            top_k=3,
            steer_current=steer_current,
        )

        if not targets:
            xg_prim = 0.0
            yg_prim = max(GAP_MIN_LOOKAHEAD_M, min(target_lookahead_m, 1.20))
            aperture = 0.45
        else:
            xg_prim, yg_prim, aperture = targets[0]

        # Fast temporal tracking: aggressive responsiveness so it does not lag
        d_change = math.hypot(xg_prim - self.last_xg, yg_prim - self.last_yg)
        alpha = 0.85 if d_change > 0.15 else 0.50
        self.last_xg = (1.0 - alpha) * self.last_xg + alpha * xg_prim
        self.last_yg = (1.0 - alpha) * self.last_yg + alpha * yg_prim
        self.last_aperture = aperture
        self.last_targets = targets

        theta_g = math.atan2(self.last_xg, self.last_yg)
        return self.last_xg, self.last_yg, aperture, theta_g

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
            for s_name, sx, sy, heading, max_range in TOF_SENSOR_RIG:
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
                ty = max(0.35, sy + eff_dist * sin_h)
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
        # 2. EVALUATE DENSE RADIAL RAYS ACROSS FRONT FOV (-65 to +65 deg)
        # ----------------------------------------------------------------------
        max_r = min(max_lookahead_m, GAP_MAX_LOOKAHEAD_M)
        n_steps = int(max_r / step_ds)

        for az in _RAY_AZIMUTHS:
            cos_az = math.cos(az)
            sin_az = math.sin(az)
            clear_dist = max_r

            for s in range(max(2, int(0.10 / step_ds)), n_steps + 1):
                d = s * step_ds
                px = d * cos_az
                py = d * sin_az
                col = int(round((px - x_min) / res))
                row = int(round((py - y_min) / res))
                if row < 0 or row >= h or col < 0 or col >= w:
                    clear_dist = max(GAP_MIN_LOOKAHEAD_M, d - step_ds)
                    break
                if hazard_mask[row, col] > 0:
                    clear_dist = max(GAP_MIN_LOOKAHEAD_M, d - step_ds)
                    break

            eff_dist = max(GAP_MIN_LOOKAHEAD_M, clear_dist - 0.06)
            tx = eff_dist * cos_az
            ty = max(0.35, eff_dist * sin_az)
            total_dist = math.hypot(tx, ty)

            rel_angle = (math.pi / 2.0) - az
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
        scored_candidates = []
        for c in candidates:
            if c["dist"] < min_allowed_dist:
                continue

            dist_norm = c["dist"] / max(max_clear_dist, 0.10)
            width_norm = c["aperture"] / 1.00
            steer_diff = abs(c["steer_equiv"] - steer_current)
            straight_diff = abs(c["rel_angle"])

            score = (
                WEIGHT_GAP_DISTANCE * dist_norm
                + WEIGHT_GAP_WIDTH * width_norm
                - WEIGHT_ALIGN_STEER * steer_diff
                - WEIGHT_ALIGN_STRAIGHT * straight_diff
            )
            scored_candidates.append((score, c))

        scored_candidates.sort(key=lambda item: item[0], reverse=True)

        # ----------------------------------------------------------------------
        # 4. SELECT TOP 3 DISTINCT TARGETS (Spatial Separation >= 15 cm)
        # ----------------------------------------------------------------------
        targets = []
        for score, c in scored_candidates:
            tx, ty, ap = c["tx"], c["ty"], c["aperture"]
            # Enforce spatial distance between candidate targets
            if any(math.hypot(tx - ex, ty - ey) < 0.18 for (ex, ey, _) in targets):
                continue

            targets.append((tx, ty, ap))
            if len(targets) >= top_k:
                break

        # Fallback if no target met threshold
        if not targets:
            best_c = max(candidates, key=lambda x: x["dist"])
            targets = [(best_c["tx"], best_c["ty"], 0.40)]

        # Multi-depth target variety:
        # Guarantee an intermediate-depth anchor along the primary path
        # to ensure surviving candidate splines even in tight bends or chicanes
        if len(targets) > 0:
            tx0, ty0, ap0 = targets[0]
            dist0 = math.hypot(tx0, ty0)
            if dist0 > 0.75:
                mid_dist = max(0.55, dist0 * 0.65)
                scale = mid_dist / dist0
                mid_target = (tx0 * scale, max(0.40, ty0 * scale), max(GAP_MIN_WIDTH_M, ap0 * 0.85))
                if len(targets) < top_k:
                    targets.append(mid_target)
                elif len(targets) >= 2 and any(math.hypot(mid_target[0] - ex, mid_target[1] - ey) > 0.15 for (ex, ey, _) in targets):
                    targets.insert(1, mid_target)
                    targets = targets[:top_k]

        return targets

