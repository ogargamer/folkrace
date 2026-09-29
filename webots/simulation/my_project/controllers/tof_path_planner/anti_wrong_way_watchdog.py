"""
================================================================================
  ANTI-WRONG-WAY & U-TURN DISCRIMINATION WATCHDOG (BNO08X IMU + ENCODERS)
================================================================================
  Distinguishes legitimate track hairpins (U-turns into adjacent corridors)
  from in-lane wrong-way reversals (spinning 180 within the same track corridor).

  Implements 4 selectable options plus a combined hybrid ensemble:
    - OPTION_1_BREADCRUMB        : Trail overlap & double-180 reverse correction
    - OPTION_2_KINEMATIC_ARC     : Dynamic curvature limit (R = v / yaw_rate >= R_min)
    - OPTION_3_FLOW_COSTMAP      : 2D directional flow stamping & one-way barrier
    - OPTION_4_LATERAL_SEPARATION: Lateral corridor shift across 180 yaw excursion
    - OPTION_COMBINED_HYBRID     : Multi-layered active protection ensemble
================================================================================
"""

import math
import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    from .planner_config import (
        ANTI_WRONG_WAY_MODE,
        AWW_BREADCRUMB_LOOKBACK_M,
        AWW_BREADCRUMB_SPACING_M,
        AWW_CORRIDOR_HALF_WIDTH_M,
        AWW_REVERSE_CONE_ANGLE_DEG,
        AWW_MIN_HAIRPIN_RADIUS_M,
        AWW_FLOW_GRID_RES_M,
        AWW_FLOW_DECAY_S,
        AWW_LATERAL_SEPARATION_MIN_M,
        AWW_AUTO_CORRECT_180_ENABLED,
        AWW_CORRECTIVE_180_SPEED_MPS,
        AWW_CORRECTIVE_180_STEER_RAD,
        AWW_HISTORY_WINDOW_S,
        AWW_LOOKAHEAD_M,
        GRID_X_MIN,
        GRID_Y_MIN,
        GRID_RES,
    )
except ImportError:
    try:
        from planner_config import (
            ANTI_WRONG_WAY_MODE,
            AWW_BREADCRUMB_LOOKBACK_M,
            AWW_BREADCRUMB_SPACING_M,
            AWW_CORRIDOR_HALF_WIDTH_M,
            AWW_REVERSE_CONE_ANGLE_DEG,
            AWW_MIN_HAIRPIN_RADIUS_M,
            AWW_FLOW_GRID_RES_M,
            AWW_FLOW_DECAY_S,
            AWW_LATERAL_SEPARATION_MIN_M,
            AWW_AUTO_CORRECT_180_ENABLED,
            AWW_CORRECTIVE_180_SPEED_MPS,
            AWW_CORRECTIVE_180_STEER_RAD,
            AWW_HISTORY_WINDOW_S,
            AWW_LOOKAHEAD_M,
            GRID_X_MIN,
            GRID_Y_MIN,
            GRID_RES,
        )
    except ImportError:
        ANTI_WRONG_WAY_MODE = "DISABLED"
        AWW_BREADCRUMB_LOOKBACK_M = 8.0
        AWW_BREADCRUMB_SPACING_M = 0.05
        AWW_CORRIDOR_HALF_WIDTH_M = 0.25
        AWW_REVERSE_CONE_ANGLE_DEG = 45.0
        AWW_MIN_HAIRPIN_RADIUS_M = 0.32
        AWW_FLOW_GRID_RES_M = 0.05
        AWW_FLOW_DECAY_S = 3.0
        AWW_LATERAL_SEPARATION_MIN_M = 0.40
        AWW_AUTO_CORRECT_180_ENABLED = True
        AWW_CORRECTIVE_180_SPEED_MPS = 1.20
        AWW_CORRECTIVE_180_STEER_RAD = 0.60
        AWW_HISTORY_WINDOW_S = 3.0
        AWW_LOOKAHEAD_M = 1.0
        GRID_X_MIN = -1.25
        GRID_Y_MIN = 0.00
        GRID_RES = 0.010


class WrongWayOptionMode(str, Enum):
    DISABLED = "DISABLED"
    OPTION_1_BREADCRUMB = "OPTION_1_BREADCRUMB"
    OPTION_2_KINEMATIC_ARC = "OPTION_2_KINEMATIC_ARC"
    OPTION_3_FLOW_COSTMAP = "OPTION_3_FLOW_COSTMAP"
    OPTION_4_LATERAL_SEPARATION = "OPTION_4_LATERAL_SEPARATION"
    OPTION_COMBINED_HYBRID = "OPTION_COMBINED_HYBRID"


def wrap_to_pi(angle_rad: float) -> float:
    """Wraps an angle into [-pi, pi]."""
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


@dataclass
class Breadcrumb:
    x: float
    y: float
    yaw: float
    v: float
    timestamp: float
    s_dist: float = 0.0


@dataclass
class WrongWayStatus:
    is_wrong_way: bool = False
    confidence: float = 0.0
    detected_by_option: str = "NONE"
    corrective_action: str = "NONE"
    target_recovery_yaw: Optional[float] = None
    turning_radius_m: float = 999.0
    lateral_separation_m: float = 0.0
    overlap_score: float = 0.0
    sightline_rays: List[Dict[str, Any]] = field(default_factory=list)


# ==============================================================================
# OPTION 1: BREADCRUMB REVERSE-PATH OVERLAP & AUTO DOUBLE-180 CORRECTION
# ==============================================================================
class BreadcrumbWatchdog:
    """
    Option 1: Maintains a dead-reckoned trail of recent vehicle positions.
    Evaluates Line-of-Sight (LOS) 2D Bresenham raycasting against the inflated costmap
    (hazard_mask) to confirm in-corridor wrong-way reversals while rejecting adjacent-lane
    track hairpins separated by walls.
    """

    # Speed clamp range for dynamic window calculation:
    #   below V_MIN the window stops growing (avoids infinite duration at standstill)
    #   above V_MAX the window stops shrinking (maintains minimum coverage at high speed)
    _V_MIN_MPS: float = 0.20   # m/s  – slowest speed used in t = d/v
    _V_MAX_MPS: float = 3.00   # m/s  – fastest speed used in t = d/v

    def __init__(
        self,
        lookback_m: float = AWW_BREADCRUMB_LOOKBACK_M,
        spacing_m: float = AWW_BREADCRUMB_SPACING_M,
        corridor_half_width_m: float = AWW_CORRIDOR_HALF_WIDTH_M,
        cone_angle_deg: float = AWW_REVERSE_CONE_ANGLE_DEG,
        auto_correct_180: bool = AWW_AUTO_CORRECT_180_ENABLED,
        max_history_s: float = AWW_HISTORY_WINDOW_S,
        lookahead_m: float = AWW_LOOKAHEAD_M,
    ):
        self.lookback_m = max(8.0, lookback_m)
        self.spacing_m = spacing_m
        self.corridor_half_width_m = corridor_half_width_m
        self.cone_angle_rad = math.radians(cone_angle_deg)
        self.auto_correct_180 = auto_correct_180
        self.max_history_s = max_history_s  # initial fallback; updated dynamically per call
        self.lookahead_m = lookahead_m

        self.breadcrumbs: deque[Breadcrumb] = deque()
        self.total_distance_logged_m = 0.0
        self.cumulative_dist_m = 0.0
        self.last_x: Optional[float] = None
        self.last_y: Optional[float] = None
        self.is_recovering_180 = False
        self.recovery_target_yaw = 0.0

    def reset(self):
        self.breadcrumbs.clear()
        self.total_distance_logged_m = 0.0
        self.cumulative_dist_m = 0.0
        self.last_x = None
        self.last_y = None
        self.is_recovering_180 = False
        self.recovery_target_yaw = 0.0

    def add_point(
        self,
        x: float,
        y: float,
        yaw: float,
        v: float,
        now_s: float,
        state: str = "DRIVE",
        yaw_rate: float = 0.0,
    ):
        # Anti-Contamination Gating:
        # Breadcrumbs are only recorded during valid forward racing (state == NORMAL_DRIVE/DRIVE,
        # v >= 0.25 m/s, and |yaw_rate| <= 3.5 rad/s).
        # Trail additions freeze during spins, collisions, or reverse maneuvers.
        if (state not in ("DRIVE", "NORMAL_DRIVE")) or v < 0.25 or abs(yaw_rate) > 3.5:
            return

        # Dynamic history window based on speed
        speed = max(self._V_MIN_MPS, min(abs(v), self._V_MAX_MPS))
        effective_lookback = max(self.lookback_m, 8.0)
        self.max_history_s = effective_lookback / speed

        # Prune points older than dynamic rolling history window or exceeding max spatial lookback
        while self.breadcrumbs and (
            (now_s - self.breadcrumbs[0].timestamp) > self.max_history_s
            or (self.cumulative_dist_m - self.breadcrumbs[0].s_dist) > effective_lookback
        ):
            self.breadcrumbs.popleft()

        if self.last_x is None:
            self.breadcrumbs.append(Breadcrumb(x, y, yaw, v, now_s, s_dist=self.cumulative_dist_m))
            self.last_x = x
            self.last_y = y
            return

        dx = x - self.last_x
        dy = y - self.last_y
        dist = math.hypot(dx, dy)

        if dist >= self.spacing_m:
            self.cumulative_dist_m += dist
            self.total_distance_logged_m += dist
            self.breadcrumbs.append(Breadcrumb(x, y, yaw, v, now_s, s_dist=self.cumulative_dist_m))
            self.last_x = x
            self.last_y = y

            while self.breadcrumbs and (
                (now_s - self.breadcrumbs[0].timestamp) > self.max_history_s
                or (self.cumulative_dist_m - self.breadcrumbs[0].s_dist) > effective_lookback
            ):
                self.breadcrumbs.popleft()

    @staticmethod
    def _bresenham_raycast_clear(
        r0: int, c0: int, r1: int, c1: int, hazard_mask: np.ndarray
    ) -> bool:
        """
        Discrete 2D Bresenham raycast from (r0, c0) to (r1, c1) across hazard_mask.
        Allows a 2-pixel tolerance at the start to clear any noise on origin cell.
        Returns True if unobstructed (all checked cells have hazard_mask == 0).
        """
        h, w = hazard_mask.shape
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dc - dr
        r, c = r0, c0
        step = 0

        while True:
            if 0 <= r < h and 0 <= c < w:
                if step > 2:
                    if hazard_mask[r, c] > 0:
                        return False
            else:
                return False

            if r == r1 and c == c1:
                break

            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                c += sc
            if e2 < dc:
                err += dc
                r += sr
            step += 1

        return True

    def check_wrong_way(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        hazard_mask: Optional[np.ndarray] = None,
        now_s: Optional[float] = None,
        lookahead_m: Optional[float] = None,
        **kwargs,
    ) -> Tuple[bool, float, float, List[Dict[str, Any]]]:
        """
        Line-of-Sight Raycasting on Inflated Costmap (hazard_mask).
        Evaluates candidate breadcrumbs:
          - Odometry gate: Delta S in [0.80 m, 8.0 m] excludes points under chassis and far laps.
          - Spatial search envelope: Euclidean distance d in [0.40 m, 1.70 m], y_v in [0.40 m, 1.70 m],
            forward cone |angle| <= 50 deg.
          - Direct 2D Bresenham raycast across hazard_mask.
          - Heading opposition check: diff >= 110 deg.
        Returns: (is_wrong_way, confidence, target_recovery_yaw, sightline_rays)
        """
        if len(self.breadcrumbs) < 5:
            return False, 0.0, 0.0, []

        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)

        # Origin cell in local costmap grid
        col_0 = int(math.floor((0.0 - GRID_X_MIN) / GRID_RES + 0.5))
        row_0 = int(math.floor((0.0 - GRID_Y_MIN) / GRID_RES + 0.5))

        sightline_rays: List[Dict[str, Any]] = []
        confirmed_candidates: List[Breadcrumb] = []

        for bc in reversed(self.breadcrumbs):
            # 1. Odometry gate: Delta S in [0.80m, 8.0m]
            delta_s = self.cumulative_dist_m - bc.s_dist
            if not (0.80 <= delta_s <= 8.0):
                continue

            # 2. World to vehicle frame: (+Y forward, +X right)
            dx = bc.x - current_x
            dy = bc.y - current_y
            xv = cos_yaw * dx + sin_yaw * dy
            yv = -sin_yaw * dx + cos_yaw * dy

            # Euclidean distance from robot origin
            d = math.hypot(xv, yv)

            # 3. Spatial search envelope:
            # Euclidean distance d in [0.40m, 1.70m], forward yv in [0.40m, 1.70m], |angle| <= 50 deg
            if d < 0.40 or d > 1.70 or yv < 0.40 or yv > 1.70:
                continue

            angle_rad = math.atan2(xv, yv)
            if abs(angle_rad) > math.radians(50.0):
                continue

            # 4. Heading opposition: diff >= 110 deg
            heading_diff = abs(wrap_to_pi(current_yaw - bc.yaw))
            is_opposing = (heading_diff >= math.radians(110.0))

            # 5. Raycast cell target in costmap
            col_tgt = int(math.floor((xv - GRID_X_MIN) / GRID_RES + 0.5))
            row_tgt = int(math.floor((yv - GRID_Y_MIN) / GRID_RES + 0.5))

            if hazard_mask is not None:
                h_m, w_m = hazard_mask.shape
                if 0 <= row_tgt < h_m and 0 <= col_tgt < w_m:
                    is_clear = self._bresenham_raycast_clear(row_0, col_0, row_tgt, col_tgt, hazard_mask)
                else:
                    is_clear = False
            else:
                is_clear = True

            ray_info = {
                "x": float(bc.x),
                "y": float(bc.y),
                "xv": float(xv),
                "yv": float(yv),
                "dist": float(d),
                "delta_s": float(delta_s),
                "clear": bool(is_clear),
                "opposing": bool(is_opposing),
                "heading_diff_deg": float(math.degrees(heading_diff)),
                "target_yaw": float(bc.yaw),
            }
            sightline_rays.append(ray_info)

            if is_clear and is_opposing:
                confirmed_candidates.append(bc)

        if confirmed_candidates:
            count_confirmed = len(confirmed_candidates)
            confidence = min(1.0, 0.70 + 0.10 * count_confirmed)
            # Corridor tangent vector of closest confirmed breadcrumb
            target_recovery_yaw = confirmed_candidates[0].yaw
            return True, confidence, target_recovery_yaw, sightline_rays

        return False, 0.0, 0.0, sightline_rays


# ==============================================================================
# OPTION 2: KINEMATIC ARC RADIUS FILTER (R = v / yaw_rate >= R_min)
# ==============================================================================
class KinematicArcFilter:
    """
    Option 2: Pre-emptive dynamic filter enforcing physical track minimum turn radius.
    A legitimate track hairpin has a physical centerline radius (e.g. R >= 0.32 m)
    traversed at speed with forward velocity.
    An in-lane pivot has high yaw rate with near-zero forward translation (R < 0.15 m).
    """

    def __init__(
        self,
        min_hairpin_radius_m: float = AWW_MIN_HAIRPIN_RADIUS_M,
    ):
        self.min_hairpin_radius_m = min_hairpin_radius_m
        self.turn_accumulator_rad = 0.0
        self.last_yaw: Optional[float] = None

    def reset(self):
        self.turn_accumulator_rad = 0.0
        self.last_yaw = None

    def evaluate_motion(
        self,
        v_est: float,
        yaw_rate: float,
        current_yaw: float,
    ) -> Tuple[bool, float]:
        """
        Computes instantaneous turning radius R = v / abs(yaw_rate).
        Returns: (is_invalid_pivot, current_radius_m)
        """
        abs_yaw_rate = abs(yaw_rate)
        abs_v = abs(v_est)

        if abs_yaw_rate < 0.30:
            # Driving relatively straight
            return False, 999.0

        current_radius = abs_v / max(0.001, abs_yaw_rate)

        # Track total rotation during tight turning
        if self.last_yaw is not None:
            dyaw = wrap_to_pi(current_yaw - self.last_yaw)
            self.turn_accumulator_rad += dyaw
        self.last_yaw = current_yaw

        # If performing a significant turn (> 90 deg) with turning radius tighter
        # than any physical track hairpin can ever be, flag as an invalid in-lane pivot
        if current_radius < self.min_hairpin_radius_m and abs(self.turn_accumulator_rad) > math.radians(90.0):
            return True, current_radius

        if abs_yaw_rate < 0.20:
            self.turn_accumulator_rad = 0.0

        return False, current_radius

    def filter_candidate_path(self, path_points: List[Tuple[float, float]]) -> bool:
        """
        Evaluates a candidate trajectory from the planner.
        Returns True if the path maintains an acceptable turning radius >= R_min.
        Returns False if the path commands an impossible/in-lane pirouette.
        """
        if len(path_points) < 4:
            return True

        for i in range(1, len(path_points) - 1):
            p0 = path_points[i - 1]
            p1 = path_points[i]
            p2 = path_points[i + 1]

            d1 = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            d2 = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            if d1 < 0.01 or d2 < 0.01:
                continue

            # Direction vectors
            t1 = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
            t2 = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
            dtheta = abs(wrap_to_pi(t2 - t1))
            ds = 0.5 * (d1 + d2)

            if dtheta > 1e-4:
                curvature = dtheta / ds
                radius = 1.0 / curvature
                if radius < (self.min_hairpin_radius_m * 0.75):
                    # Path forces a spin tighter than track corridor allows
                    return False

        return True


# ==============================================================================
# OPTION 3: DIRECTIONAL FLOW COSTMAP STAMPING
# ==============================================================================
class DirectionalFlowCostmap:
    """
    Option 3: Spatial 2D directional flow stamping.
    As the car advances through a corridor, grid cells are stamped with the
    forward heading unit vector (dx, dy) and timestamp.
    When evaluating candidates, paths pointing opposite to the stamped corridor
    flow are blocked as virtual barriers.
    In a track hairpin, the adjacent lane cells are unvisited, so they are not blocked.
    """

    def __init__(
        self,
        grid_res_m: float = AWW_FLOW_GRID_RES_M,
        decay_s: float = AWW_FLOW_DECAY_S,
    ):
        self.grid_res = grid_res_m
        self.decay_s = decay_s
        # Key: (cell_x, cell_y) -> Value: (flow_dx, flow_dy, timestamp)
        self.grid: Dict[Tuple[int, int], Tuple[float, float, float]] = {}

    def reset(self):
        self.grid.clear()

    def stamp_flow(self, x: float, y: float, yaw: float, now_s: float):
        """Stamps cell with normalized forward heading vector."""
        cx = int(round(x / self.grid_res))
        cy = int(round(y / self.grid_res))

        fdx = math.cos(yaw)
        fdy = math.sin(yaw)

        # Stamp 3x3 footprint cells around vehicle center
        for ox in (-1, 0, 1):
            for oy in (-1, 0, 1):
                self.grid[(cx + ox, cy + oy)] = (fdx, fdy, now_s)

        # Periodic cleanup of expired cells
        if len(self.grid) > 400:
            self._cleanup(now_s)

    def _cleanup(self, now_s: float):
        expired_keys = [
            k for k, v in self.grid.items()
            if (now_s - v[2]) > self.decay_s
        ]
        for k in expired_keys:
            del self.grid[k]

    def check_flow_alignment(
        self,
        target_x: float,
        target_y: float,
        target_yaw: float,
        now_s: float,
    ) -> Tuple[bool, float]:
        """
        Checks if a target pose attempts to drive against stamped corridor flow.
        Returns: (is_opposing, alignment_dot)
        """
        cx = int(round(target_x / self.grid_res))
        cy = int(round(target_y / self.grid_res))

        cell_data = self.grid.get((cx, cy))
        if cell_data is None:
            return False, 1.0

        fdx, fdy, t_stamp = cell_data
        if (now_s - t_stamp) > self.decay_s:
            return False, 1.0

        # Candidate forward vector
        cdx = math.cos(target_yaw)
        cdy = math.sin(target_yaw)

        # Dot product: +1.0 = aligned, -1.0 = driving opposite (wrong-way)
        dot = cdx * fdx + cdy * fdy
        if dot < -0.65:
            return True, dot

        return False, dot


# ==============================================================================
# OPTION 4: LATERAL CORRIDOR SEPARATION WATCHDOG (ToF + IMU Reference)
# ==============================================================================
class LateralSeparationWatchdog:
    """
    Option 4: Tracks lateral displacement across 180-degree yaw excursions.
    When entering a turn:
      Records entry pose (entry_x, entry_y, entry_yaw).
    When yaw rotation passes 140-180 degrees:
      Calculates lateral displacement perpendicular to entry corridor axis.
      - If lateral displacement >= AWW_LATERAL_SEPARATION_MIN_M:
        Confirmed track hairpin into adjacent corridor!
      - If lateral displacement < AWW_LATERAL_SEPARATION_MIN_M:
        In-lane spin-out inside the same corridor!
    """

    def __init__(
        self,
        min_separation_m: float = AWW_LATERAL_SEPARATION_MIN_M,
    ):
        self.min_separation_m = min_separation_m
        self.in_turn_tracking = False
        self.entry_x = 0.0
        self.entry_y = 0.0
        self.entry_yaw = 0.0
        self.accumulated_yaw_turn = 0.0
        self.last_yaw: Optional[float] = None
        self.last_completed_lateral_separation_m = 0.0

    def reset(self):
        self.in_turn_tracking = False
        self.accumulated_yaw_turn = 0.0
        self.last_yaw = None
        self.last_completed_lateral_separation_m = 0.0

    def update(
        self,
        x: float,
        y: float,
        yaw: float,
        yaw_rate: float,
    ) -> Tuple[bool, float]:
        """
        Evaluates turn progression and lateral separation.
        Returns: (is_in_lane_u_turn, lateral_separation_m)
        """
        if self.last_yaw is None:
            self.last_yaw = yaw
            return False, self.last_completed_lateral_separation_m

        dyaw = wrap_to_pi(yaw - self.last_yaw)
        self.last_yaw = yaw

        # Start tracking when significant turning starts
        if not self.in_turn_tracking:
            if abs(yaw_rate) > 0.40:
                self.in_turn_tracking = True
                self.entry_x = x
                self.entry_y = y
                self.entry_yaw = yaw
                self.accumulated_yaw_turn = dyaw
        else:
            self.accumulated_yaw_turn += dyaw

            # Check if turn completed a ~180 degree rotation (140 to 220 deg)
            total_turn_abs = abs(self.accumulated_yaw_turn)
            if total_turn_abs >= math.radians(140.0):
                # Calculate lateral displacement perpendicular to original entry axis
                dx = x - self.entry_x
                dy = y - self.entry_y
                # Entry corridor normal vector (-sin(entry_yaw), cos(entry_yaw))
                lat_disp = abs(-dx * math.sin(self.entry_yaw) + dy * math.cos(self.entry_yaw))
                self.last_completed_lateral_separation_m = lat_disp

                if total_turn_abs >= math.radians(165.0):
                    # Turn complete
                    self.in_turn_tracking = False
                    self.accumulated_yaw_turn = 0.0

                    if lat_disp < self.min_separation_m:
                        # In-lane U-turn caught!
                        return True, lat_disp

                return False, lat_disp

            # If vehicle stopped turning and straightened out without reaching 140 deg
            if abs(yaw_rate) < 0.15 and total_turn_abs < math.radians(100.0):
                self.in_turn_tracking = False
                self.accumulated_yaw_turn = 0.0

        return False, self.last_completed_lateral_separation_m



# ==============================================================================
# UNIFIED ANTI-WRONG-WAY WATCHDOG COORDINATOR
# ==============================================================================
class AntiWrongWayWatchdog:
    """
    Unified manager coordinating all 4 options based on ANTI_WRONG_WAY_MODE:
      - DISABLED
      - OPTION_1_BREADCRUMB
      - OPTION_2_KINEMATIC_ARC
      - OPTION_3_FLOW_COSTMAP
      - OPTION_4_LATERAL_SEPARATION
      - OPTION_COMBINED_HYBRID
    """

    def __init__(self, mode: str = ANTI_WRONG_WAY_MODE):
        self.mode = mode
        self.breadcrumb_watchdog = BreadcrumbWatchdog()
        self.kinematic_filter = KinematicArcFilter()
        self.flow_costmap = DirectionalFlowCostmap()
        self.lateral_watchdog = LateralSeparationWatchdog()

        self.latest_status = WrongWayStatus()
        self.active_recovery_mode = False
        self.recovery_start_time = 0.0
        self.consecutive_ww_steps = 0
        self.CONFIRM_STEPS_THRESHOLD = 5  # Require >= 5 consecutive steps (~42 ms @ 120 Hz)

    def set_mode(self, mode: str):
        self.mode = mode
        self.reset()

    def reset(self):
        self.breadcrumb_watchdog.reset()
        self.kinematic_filter.reset()
        self.flow_costmap.reset()
        self.lateral_watchdog.reset()
        self.latest_status = WrongWayStatus()
        self.active_recovery_mode = False
        self.recovery_start_time = 0.0
        self.consecutive_ww_steps = 0

    def update(
        self,
        x: float,
        y: float,
        yaw: float,
        v_est: float,
        yaw_rate: float,
        cached_tofs: Optional[Any] = None,
        now_s: Optional[float] = None,
        hazard_mask: Optional[np.ndarray] = None,
        state: str = "DRIVE",
        **kwargs,
    ) -> WrongWayStatus:
        """
        Executes one update step of the active anti-wrong-way detection mode.
        """
        if now_s is None:
            now_s = time.time()

        if self.mode == WrongWayOptionMode.DISABLED:
            self.latest_status = WrongWayStatus()
            self.consecutive_ww_steps = 0
            return self.latest_status

        # 1. Option 1: Breadcrumb logging with anti-contamination gating
        self.breadcrumb_watchdog.add_point(x, y, yaw, v_est, now_s, state=state, yaw_rate=yaw_rate)

        # 2. Option 3: Directional flow stamping (when moving forward)
        if v_est > 0.30:
            self.flow_costmap.stamp_flow(x, y, yaw, now_s)

        # Initialize status
        status = WrongWayStatus()

        # Evaluate based on active mode
        if self.mode == WrongWayOptionMode.OPTION_1_BREADCRUMB:
            is_ww, score, target_yaw, rays = self.breadcrumb_watchdog.check_wrong_way(
                x, y, yaw, hazard_mask=hazard_mask, now_s=now_s
            )
            status.sightline_rays = rays
            if is_ww:
                status.is_wrong_way = True
                status.confidence = score
                status.detected_by_option = "OPTION_1_BREADCRUMB"
                status.target_recovery_yaw = target_yaw
                status.corrective_action = "CORRECTIVE_180"

        elif self.mode == WrongWayOptionMode.OPTION_2_KINEMATIC_ARC:
            is_pivot, r_m = self.kinematic_filter.evaluate_motion(v_est, yaw_rate, yaw)
            status.turning_radius_m = r_m
            if is_pivot:
                status.is_wrong_way = True
                status.confidence = 0.85
                status.detected_by_option = "OPTION_2_KINEMATIC_ARC"
                status.target_recovery_yaw = wrap_to_pi(yaw + math.pi)
                status.corrective_action = "CORRECTIVE_180"

        elif self.mode == WrongWayOptionMode.OPTION_3_FLOW_COSTMAP:
            # Check configured lookahead along current heading
            ahead_x = x + AWW_LOOKAHEAD_M * math.cos(yaw)
            ahead_y = y + AWW_LOOKAHEAD_M * math.sin(yaw)
            is_opposing, dot = self.flow_costmap.check_flow_alignment(ahead_x, ahead_y, yaw, now_s)
            if is_opposing:
                status.is_wrong_way = True
                status.confidence = max(0.0, -dot)
                status.detected_by_option = "OPTION_3_FLOW_COSTMAP"
                status.target_recovery_yaw = wrap_to_pi(yaw + math.pi)
                status.corrective_action = "CORRECTIVE_180"

        elif self.mode == WrongWayOptionMode.OPTION_4_LATERAL_SEPARATION:
            is_in_lane, lat_m = self.lateral_watchdog.update(x, y, yaw, yaw_rate)
            status.lateral_separation_m = lat_m
            if is_in_lane:
                status.is_wrong_way = True
                status.confidence = 0.90
                status.detected_by_option = "OPTION_4_LATERAL_SEPARATION"
                status.target_recovery_yaw = wrap_to_pi(yaw + math.pi)
                status.corrective_action = "CORRECTIVE_180"

        elif self.mode == WrongWayOptionMode.OPTION_COMBINED_HYBRID:
            # Multi-layered ensemble:
            # 1. Option 1 overlap & LOS raycast check
            ww_bc, score_bc, target_yaw_bc, rays_bc = self.breadcrumb_watchdog.check_wrong_way(
                x, y, yaw, hazard_mask=hazard_mask, now_s=now_s
            )
            # 2. Option 4 lateral separation check
            ww_lat, lat_m = self.lateral_watchdog.update(x, y, yaw, yaw_rate)
            # 3. Option 2 kinematic check
            is_pivot, r_m = self.kinematic_filter.evaluate_motion(v_est, yaw_rate, yaw)
            # 4. Option 3 flow check (configured lookahead ahead)
            ahead_x = x + AWW_LOOKAHEAD_M * math.cos(yaw)
            ahead_y = y + AWW_LOOKAHEAD_M * math.sin(yaw)
            is_opposing, dot = self.flow_costmap.check_flow_alignment(ahead_x, ahead_y, yaw, now_s)

            status.turning_radius_m = r_m
            status.lateral_separation_m = lat_m
            status.overlap_score = score_bc
            status.sightline_rays = rays_bc

            if ww_bc:
                status.is_wrong_way = True
                status.confidence = max(0.80, score_bc)
                status.detected_by_option = "HYBRID_BREADCRUMB"
                status.target_recovery_yaw = target_yaw_bc
                status.corrective_action = "CORRECTIVE_180"
            elif ww_lat:
                status.is_wrong_way = True
                status.confidence = 0.92
                status.detected_by_option = "HYBRID_LATERAL_SEPARATION"
                status.target_recovery_yaw = wrap_to_pi(yaw + math.pi)
                status.corrective_action = "CORRECTIVE_180"
            elif is_opposing:
                status.is_wrong_way = True
                status.confidence = 0.88
                status.detected_by_option = "HYBRID_FLOW_COSTMAP"
                status.target_recovery_yaw = wrap_to_pi(yaw + math.pi)
                status.corrective_action = "CORRECTIVE_180"
            elif is_pivot and abs(v_est) < 0.40:
                status.is_wrong_way = True
                status.confidence = 0.75
                status.detected_by_option = "HYBRID_KINEMATIC_ARC"
                status.target_recovery_yaw = wrap_to_pi(yaw + math.pi)
                status.corrective_action = "CORRECTIVE_180"

        # Multi-step confirmation filter:
        # Requires wrong way to be detected for >= CONFIRM_STEPS_THRESHOLD consecutive steps
        if status.is_wrong_way:
            self.consecutive_ww_steps += 1
            if self.consecutive_ww_steps < self.CONFIRM_STEPS_THRESHOLD:
                # Not yet confirmed over multiple consecutive frames
                status.is_wrong_way = False
                status.corrective_action = "NONE"
        else:
            self.consecutive_ww_steps = max(0, self.consecutive_ww_steps - 1)

        self.latest_status = status
        return status

    def get_detailed_telemetry(
        self,
        x: float,
        y: float,
        yaw: float,
        v_est: float,
        yaw_rate: float,
        now_s: Optional[float] = None,
        hazard_mask: Optional[np.ndarray] = None,
    ) -> Dict[str, Any]:
        """
        Generates rich visual and narrative telemetry for every option simultaneously
        to stream to the Thought Process GUI.
        """
        if now_s is None:
            now_s = time.time()

        # Option 1: Breadcrumb LOS raycast
        ww_bc, score_bc, target_yaw_bc, rays_bc = self.breadcrumb_watchdog.check_wrong_way(
            x, y, yaw, hazard_mask=hazard_mask, now_s=now_s
        )
        n_bc = len(self.breadcrumb_watchdog.breadcrumbs)
        thought_bc = (
            f"Trail has {n_bc} points ({self.breadcrumb_watchdog.lookback_m:.1f}m window). Unobstructed line-of-sight to opposing trail confirmed (score: {score_bc:.2f}) -> IN-LANE WRONG-WAY DETECTED! Action: Spin 180° back."
            if ww_bc else
            f"Trail has {n_bc} points ({self.breadcrumb_watchdog.lookback_m:.1f}m window). Raycast scan: forward corridor is clear or adjacent lane blocked by wall. Heading towards open track."
        )

        # Option 2: Kinematic Arc
        is_pivot, r_m = self.kinematic_filter.evaluate_motion(v_est, yaw_rate, yaw)
        thought_arc = (
            f"Speed: {v_est:.2f} m/s, Yaw Rate: {yaw_rate:.2f} rad/s. Computed turn radius R = {r_m:.2f}m is BELOW minimum track hairpin radius ({AWW_MIN_HAIRPIN_RADIUS_M:.2f}m) -> IN-LANE TIGHT PIVOT! Action: Spin 180° back."
            if is_pivot else
            f"Speed: {v_est:.2f} m/s, Yaw Rate: {yaw_rate:.2f} rad/s. Turn radius R = {r_m:.2f}m satisfies minimum hairpin radius ({AWW_MIN_HAIRPIN_RADIUS_M:.2f}m) -> VALID RACING ARC."
        )

        # Option 3: Flow Costmap
        ahead_x = x + AWW_LOOKAHEAD_M * math.cos(yaw)
        ahead_y = y + AWW_LOOKAHEAD_M * math.sin(yaw)
        is_opposing, dot = self.flow_costmap.check_flow_alignment(ahead_x, ahead_y, yaw, now_s)
        n_cells = len(self.flow_costmap.grid)
        thought_flow = (
            f"Lookahead {AWW_LOOKAHEAD_M:.1f}m into corridor flow map ({n_cells} stamped cells, {AWW_HISTORY_WINDOW_S:.1f}s decay). Heading dot product = {dot:+.2f} (<= -0.50 threshold) -> HEADING WRONG WAY AGAINST TRAFFIC FLOW! Action: Spin 180° back."
            if is_opposing else
            f"Corridor flow map has {n_cells} stamped cells ({AWW_HISTORY_WINDOW_S:.1f}s decay). Ahead {AWW_LOOKAHEAD_M:.1f}m flow alignment dot product = {dot:+.2f} (> -0.50 threshold) -> FORWARD FLOW ALIGNED."
        )

        # Option 4: Lateral Separation
        in_turn = self.lateral_watchdog.in_turn_tracking
        acc_deg = math.degrees(self.lateral_watchdog.accumulated_yaw_turn)
        lat_m = self.lateral_watchdog.last_completed_lateral_separation_m
        ww_lat = (self.latest_status.detected_by_option in ("OPTION_4_LATERAL_SEPARATION", "HYBRID_LATERAL_SEPARATION")) and self.latest_status.is_wrong_way
        thought_lat = (
            f"Completed ~180° turn with only {lat_m:.2f}m lateral displacement (< {AWW_LATERAL_SEPARATION_MIN_M:.2f}m track lane spacing) -> U-TURN INSIDE SAME CORRIDOR! Action: Spin 180° back."
            if ww_lat else (
                f"Active turn tracking: {in_turn} (excursion: {acc_deg:.1f}°). Lateral displacement across turn axis: {lat_m:.2f}m (min spacing: {AWW_LATERAL_SEPARATION_MIN_M:.2f}m)."
            )
        )

        # Breadcrumb trail positions for 2D visualizer
        trail_pts = [[round(p.x, 3), round(p.y, 3)] for p in list(self.breadcrumb_watchdog.breadcrumbs)[::2]]

        return {
            "mode": self.mode.name if hasattr(self.mode, "name") else str(self.mode),
            "is_wrong_way": self.latest_status.is_wrong_way,
            "detected_by": self.latest_status.detected_by_option,
            "action": self.latest_status.corrective_action,
            "target_yaw": round(float(self.latest_status.target_recovery_yaw or 0.0), 3),
            "opt1": {
                "name": "Option 1: Breadcrumb Trail Overlap & LOS Raycast",
                "score": round(float(score_bc), 3),
                "is_tripped": bool(ww_bc),
                "trail_count": n_bc,
                "target_yaw": round(float(target_yaw_bc), 3),
                "sightlines": rays_bc,
                "thought": thought_bc,
            },
            "opt2": {
                "name": "Option 2: Kinematic Turn Radius Filter",
                "radius_m": round(float(r_m), 3),
                "min_radius_m": AWW_MIN_HAIRPIN_RADIUS_M,
                "is_tripped": bool(is_pivot),
                "v_est": round(float(v_est), 3),
                "yaw_rate": round(float(yaw_rate), 3),
                "thought": thought_arc,
            },
            "opt3": {
                "name": "Option 3: Directional Flow Costmap",
                "dot_product": round(float(dot), 3),
                "is_tripped": bool(is_opposing),
                "cells_count": n_cells,
                "thought": thought_flow,
            },
            "opt4": {
                "name": "Option 4: Lateral Corridor Separation",
                "in_turn": bool(in_turn),
                "turn_deg": round(float(acc_deg), 1),
                "lateral_m": round(float(lat_m), 3),
                "min_lateral_m": AWW_LATERAL_SEPARATION_MIN_M,
                "is_tripped": bool(ww_lat),
                "thought": thought_lat,
            },
            "trail": trail_pts[-60:] if trail_pts else [],
        }

    def filter_candidate_trajectories(
        self,
        candidate_paths: List[List[Tuple[float, float]]],
        now_s: Optional[float] = None,
    ) -> List[List[Tuple[float, float]]]:
        """
        Filters candidate planner paths to reject those pointing against corridor flow
        or requiring impossible in-lane turning radii.
        """
        if self.mode == WrongWayOptionMode.DISABLED or not candidate_paths:
            return candidate_paths

        if now_s is None:
            now_s = time.time()

        valid_paths = []
        for path in candidate_paths:
            if len(path) < 3:
                valid_paths.append(path)
                continue

            # Check kinematic radius if Option 2 or Combined
            if self.mode in (WrongWayOptionMode.OPTION_2_KINEMATIC_ARC, WrongWayOptionMode.OPTION_COMBINED_HYBRID):
                if not self.kinematic_filter.filter_candidate_path(path):
                    continue

            # Check flow map if Option 3 or Combined
            if self.mode in (WrongWayOptionMode.OPTION_3_FLOW_COSTMAP, WrongWayOptionMode.OPTION_COMBINED_HYBRID):
                end_p = path[-1]
                mid_p = path[len(path) // 2]
                path_yaw = math.atan2(end_p[1] - mid_p[1], end_p[0] - mid_p[0])
                is_opposing, _ = self.flow_costmap.check_flow_alignment(end_p[0], end_p[1], path_yaw, now_s)
                if is_opposing:
                    continue

            valid_paths.append(path)

        # Fallback to original paths if all filtered to prevent total stalling
        return valid_paths if valid_paths else candidate_paths
