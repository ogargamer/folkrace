"""
================================================================================
  ANTI-WRONG-WAY & U-TURN DISCRIMINATION WATCHDOG (BREADCRUMB & EARLY GAP EXIT)
================================================================================
  Distinguishes legitimate track hairpins (U-turns into adjacent corridors separated
  by walls) from in-lane wrong-way reversals (spinning 180 within the same corridor).

  Utilizes:
    - OPTION_1_BREADCRUMB: Line-of-Sight (LOS) 2D Bresenham raycasting against
      the inflated costmap (hazard_mask) to confirm in-corridor wrong-way reversals.
    - Narrow-Cone Early Gap Exit Sentry: Evaluates candidate scouted gaps for
      safe forward paths, preventing false positives and enabling swift exit from
      corrective maneuvers.
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
        AWW_BREADCRUMB_DURATION_SCALE,
        AWW_BREADCRUMB_SPACING_M,
        AWW_BREADCRUMB_MAX_TIMEOUT_S,
        AWW_EARLY_EXIT_MIN_GAP_M,
        AWW_EARLY_EXIT_CONE_DEG,
        AWW_EARLY_EXIT_OPPOSING_DEG,
        AWW_EARLY_EXIT_MIN_BC_DIST_M,
        GRID_X_MIN,
        GRID_Y_MIN,
        GRID_RES,
        V_MIN_CRUISE,
        V_MAX_CRUISE,
    )
except ImportError:
    try:
        from planner_config import (
            ANTI_WRONG_WAY_MODE,
            AWW_BREADCRUMB_LOOKBACK_M,
            AWW_BREADCRUMB_DURATION_SCALE,
            AWW_BREADCRUMB_SPACING_M,
            AWW_BREADCRUMB_MAX_TIMEOUT_S,
            AWW_EARLY_EXIT_MIN_GAP_M,
            AWW_EARLY_EXIT_CONE_DEG,
            AWW_EARLY_EXIT_OPPOSING_DEG,
            AWW_EARLY_EXIT_MIN_BC_DIST_M,
            GRID_X_MIN,
            GRID_Y_MIN,
            GRID_RES,
            V_MIN_CRUISE,
            V_MAX_CRUISE,
        )
    except ImportError:
        ANTI_WRONG_WAY_MODE = "OPTION_1_BREADCRUMB"
        AWW_BREADCRUMB_LOOKBACK_M = 8.0
        AWW_BREADCRUMB_DURATION_SCALE = 0.6
        AWW_BREADCRUMB_SPACING_M = 0.05
        AWW_BREADCRUMB_MAX_TIMEOUT_S = 6.0
        AWW_EARLY_EXIT_MIN_GAP_M = 1.4
        AWW_EARLY_EXIT_CONE_DEG = 30.0
        AWW_EARLY_EXIT_OPPOSING_DEG = 110.0
        AWW_EARLY_EXIT_MIN_BC_DIST_M = 0.15
        GRID_X_MIN = -1.25
        GRID_Y_MIN = 0.00
        GRID_RES = 0.010
        V_MIN_CRUISE = 1.166
        V_MAX_CRUISE = 5.00


class WrongWayOptionMode(str, Enum):
    DISABLED = "DISABLED"
    OPTION_1_BREADCRUMB = "OPTION_1_BREADCRUMB"


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
    early_exit_gap: Optional[Tuple[float, float]] = None
    early_exit_found: bool = False


# ==============================================================================
# BREADCRUMB REVERSE-PATH OVERLAP & NARROW-CONE EARLY GAP EXIT SENTRY
# ==============================================================================
class BreadcrumbWatchdog:
    """
    Maintains a dead-reckoned trail of recent vehicle positions.
    Evaluates Line-of-Sight (LOS) 2D Bresenham raycasting against the inflated costmap
    (hazard_mask) to confirm in-corridor wrong-way reversals while rejecting adjacent-lane
    track hairpins separated by walls.
    """

    # Speed limits anchored to vehicle operational envelope from planner_config:
    _V_MIN_MPS: float = float(V_MIN_CRUISE)   # m/s dynamic cornering floor (~1.17 m/s)
    _V_MAX_MPS: float = float(V_MAX_CRUISE)   # m/s maximum straight-line cruising speed (5.00 m/s)

    def __init__(
        self,
        lookback_m: float = AWW_BREADCRUMB_LOOKBACK_M,
        spacing_m: float = AWW_BREADCRUMB_SPACING_M,
        max_history_s: float = AWW_BREADCRUMB_MAX_TIMEOUT_S,
        cone_angle_deg: float = 50.0,
        duration_scale: float = AWW_BREADCRUMB_DURATION_SCALE,
        max_timeout_s: float = AWW_BREADCRUMB_MAX_TIMEOUT_S,
        **kwargs,
    ):
        self.lookback_m = float(lookback_m)
        self.duration_scale = float(kwargs.get("duration_scale", kwargs.get("lifetime_scale", duration_scale)))
        self.spacing_m = spacing_m
        self.max_timeout_s = float(kwargs.get("max_timeout_s", kwargs.get("max_history_s", max_timeout_s)))
        self.max_history_s = self.max_timeout_s
        self.cone_angle_rad = math.radians(cone_angle_deg)

        self.breadcrumbs: deque[Breadcrumb] = deque()
        self.cumulative_dist_m = 0.0
        self.last_x: Optional[float] = None
        self.last_y: Optional[float] = None

    @property
    def effective_lookback_m(self) -> float:
        """
        Effective spatial lookback distance accounting for duration_scale (how fast breadcrumbs disappear).
        Enforces a sensible minimum of 1.5m to ensure sufficient points for detection.
        """
        return max(1.5, self.lookback_m * self.duration_scale)

    def set_duration_scale(self, scale: float):
        """Dynamically updates how fast breadcrumbs disappear."""
        self.duration_scale = max(0.05, float(scale))

    def set_max_timeout(self, timeout_s: float):
        """Dynamically updates maximum breadcrumb history timeout."""
        self.max_timeout_s = max(0.5, float(timeout_s))
        self.max_history_s = self.max_timeout_s

    def reset(self):
        self.breadcrumbs.clear()
        self.cumulative_dist_m = 0.0
        self.last_x = None
        self.last_y = None

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

        # Effective spatial lookback distance accounting for duration scale
        effective_lookback = self.effective_lookback_m
        # Max history timeout (default 6.0s from planner_config prevents premature queue wipe during maneuvers)
        self.max_history_s = self.max_timeout_s

        # Prune points exceeding max timeout (6.0s) or exceeding spatial lookback (4.8m @ 0.6x)
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
        **kwargs,
    ) -> Tuple[bool, float, float, List[Dict[str, Any]]]:
        """
        Line-of-Sight Raycasting on Inflated Costmap (hazard_mask).
        Evaluates candidate breadcrumbs:
          - Odometry gate: Delta S in [0.80 m, 8.0 m] excludes points under chassis and far laps.
          - Spatial search envelope: Euclidean distance d in [0.40 m, 1.70 m], y_v in [0.40 m, 1.70 m],
            forward cone |angle| <= cone_angle_rad (default 50 deg).
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

        effective_lookback = self.effective_lookback_m
        for bc in reversed(self.breadcrumbs):
            # 1. Odometry gate: Delta S in [0.80m, effective_lookback]
            delta_s = self.cumulative_dist_m - bc.s_dist
            if not (0.80 <= delta_s <= effective_lookback):
                continue

            # 2. World to vehicle frame: (+Y forward, +X right)
            dx = bc.x - current_x
            dy = bc.y - current_y
            xv = cos_yaw * dx + sin_yaw * dy
            yv = -sin_yaw * dx + cos_yaw * dy

            # Euclidean distance from robot origin
            d = math.hypot(xv, yv)

            # 3. Spatial search envelope:
            # Euclidean distance d in [0.40m, 1.70m], forward yv in [0.40m, 1.70m], |angle| <= cone_angle_rad
            if d < 0.40 or d > 1.70 or yv < 0.40 or yv > 1.70:
                continue

            angle_rad = math.atan2(xv, yv)
            if abs(angle_rad) > self.cone_angle_rad:
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

    def check_gap_channel(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        gap_xg: float,
        gap_yg: float,
        cone_deg: float = AWW_EARLY_EXIT_CONE_DEG,
        opposing_deg: float = AWW_EARLY_EXIT_OPPOSING_DEG,
        min_gap_dist: float = AWW_EARLY_EXIT_MIN_GAP_M,
        min_bc_dist: float = AWW_EARLY_EXIT_MIN_BC_DIST_M,
        max_reach_margin_m: float = 0.80,
    ) -> Tuple[bool, int, List[Dict[str, Any]]]:
        """
        Narrow-Cone Early Gap Exit Sentry (Spatial Channel Filter).
        Evaluates whether a narrow cone centered directly along the scouted gap vector (gap_xg, gap_yg)
        contains historical breadcrumbs with opposing heading (diff >= opposing_deg).

        Key Requirements & Architecture:
          - Target-Centered Narrow Cone: The narrow cone (+-cone_deg/2, default +-15 deg = 30 deg total)
            is oriented along the vector towards (gap_xg, gap_yg) in the vehicle frame, NOT the robot's heading.
          - Obstacle Immunity: Purely spatial/geometric without Bresenham raycasting against the hazard mask.
            Obstacles, pillars, or opponent robots in the line of sight will not hide or blind the sentry
            to historical opposing breadcrumbs in that channel.
          - Close-Range Evaluation: Evaluates breadcrumbs starting from 15cm out (min_bc_dist = 0.15m),
            ensuring close opposing breadcrumbs in the channel cannot falsely trigger an early exit.
          - Odometry Gate: Requires Delta S >= min_bc_dist to exclude points under the immediate chassis center.

        Returns:
          (has_opposing_breadcrumbs, opposing_count, matching_breadcrumbs)
          - has_opposing_breadcrumbs: True if opposing breadcrumbs lie within the gap channel (wrong-way path).
                                      False if channel is completely clear of opposing breadcrumbs.
        """
        if len(self.breadcrumbs) < 5:
            return False, 0, []

        gap_dist = math.hypot(gap_xg, gap_yg)
        gap_angle_v = math.atan2(gap_xg, gap_yg)
        half_cone_rad = math.radians(cone_deg / 2.0)
        opposing_rad = math.radians(opposing_deg)

        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)

        # Breadcrumbs are evaluated up to the gap distance plus corridor reach margin
        max_eval_dist = max(gap_dist + max_reach_margin_m, 2.50)

        matching_bcs: List[Dict[str, Any]] = []

        for bc in reversed(self.breadcrumbs):
            # 1. Odometry gate: Delta S >= min_bc_dist (0.15m) excludes points under chassis center
            delta_s = self.cumulative_dist_m - bc.s_dist
            if delta_s < min_bc_dist:
                continue

            # 2. Transform breadcrumb to vehicle frame (+Y forward, +X right)
            dx = bc.x - current_x
            dy = bc.y - current_y
            xv = cos_yaw * dx + sin_yaw * dy
            yv = -sin_yaw * dx + cos_yaw * dy

            d_bc = math.hypot(xv, yv)
            if d_bc < min_bc_dist or d_bc > max_eval_dist:
                continue

            # 3. Narrow cone angular divergence relative to candidate gap vector
            bc_angle_v = math.atan2(xv, yv)
            angle_diff_from_gap = abs(wrap_to_pi(bc_angle_v - gap_angle_v))
            if angle_diff_from_gap > half_cone_rad:
                continue

            # 4. Heading opposition check
            heading_diff = abs(wrap_to_pi(current_yaw - bc.yaw))
            if heading_diff >= opposing_rad:
                matching_bcs.append({
                    "x": float(bc.x),
                    "y": float(bc.y),
                    "xv": float(xv),
                    "yv": float(yv),
                    "dist": float(d_bc),
                    "gap_angle_deg": float(math.degrees(gap_angle_v)),
                    "bc_angle_deg": float(math.degrees(bc_angle_v)),
                    "angle_diff_deg": float(math.degrees(angle_diff_from_gap)),
                    "heading_diff_deg": float(math.degrees(heading_diff)),
                    "delta_s": float(delta_s),
                })

        has_opposing = len(matching_bcs) > 0
        return has_opposing, len(matching_bcs), matching_bcs

    def is_gap_early_exit(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        gap_xg: float,
        gap_yg: float,
        min_gap_dist: float = AWW_EARLY_EXIT_MIN_GAP_M,
        cone_deg: float = AWW_EARLY_EXIT_CONE_DEG,
        opposing_deg: float = AWW_EARLY_EXIT_OPPOSING_DEG,
        min_bc_dist: float = AWW_EARLY_EXIT_MIN_BC_DIST_M,
    ) -> Tuple[bool, Dict[str, Any]]:
        """
        Determines if candidate gap qualifies as a valid early exit.
        Condition: gap distance >= min_gap_dist (1.0m) AND narrow cone towards gap
        does not contain historical opposing breadcrumbs starting from min_bc_dist (0.15m).
        """
        gap_dist = math.hypot(gap_xg, gap_yg)
        if gap_dist < min_gap_dist:
            return False, {
                "is_exit": False,
                "reason": "GAP_TOO_SHORT",
                "gap_dist": float(gap_dist),
                "min_gap_dist": float(min_gap_dist),
                "opposing_count": 0,
            }

        has_opposing, count, bcs = self.check_gap_channel(
            current_x=current_x,
            current_y=current_y,
            current_yaw=current_yaw,
            gap_xg=gap_xg,
            gap_yg=gap_yg,
            cone_deg=cone_deg,
            opposing_deg=opposing_deg,
            min_gap_dist=min_gap_dist,
            min_bc_dist=min_bc_dist,
        )

        if has_opposing:
            return False, {
                "is_exit": False,
                "reason": "OPPOSING_BREADCRUMBS_IN_CHANNEL",
                "gap_dist": float(gap_dist),
                "opposing_count": int(count),
                "breadcrumbs": bcs,
            }

        return True, {
            "is_exit": True,
            "reason": "SAFE_EARLY_EXIT",
            "gap_dist": float(gap_dist),
            "opposing_count": 0,
            "breadcrumbs": [],
        }


# ==============================================================================
# UNIFIED ANTI-WRONG-WAY WATCHDOG COORDINATOR
# ==============================================================================
class AntiWrongWayWatchdog:
    """
    Unified manager coordinating wrong-way protection via Breadcrumb LOS Raycasting.
    Supported modes:
      - DISABLED
      - OPTION_1_BREADCRUMB
    """

    CONFIRM_STEPS_THRESHOLD = 5  # Require >= 5 consecutive steps (~42 ms @ 120 Hz)

    def __init__(
        self,
        mode: str = ANTI_WRONG_WAY_MODE,
        lookback_m: float = AWW_BREADCRUMB_LOOKBACK_M,
        duration_scale: float = AWW_BREADCRUMB_DURATION_SCALE,
        max_timeout_s: float = AWW_BREADCRUMB_MAX_TIMEOUT_S,
        **kwargs,
    ):
        self.mode = mode
        self.breadcrumb_watchdog = BreadcrumbWatchdog(
            lookback_m=lookback_m,
            duration_scale=duration_scale,
            max_timeout_s=max_timeout_s,
            **kwargs,
        )
        self.latest_status = WrongWayStatus()
        self.consecutive_ww_steps = 0

    def set_duration_scale(self, scale: float):
        """Dynamically updates how fast breadcrumbs disappear."""
        self.breadcrumb_watchdog.set_duration_scale(scale)

    def set_max_timeout(self, timeout_s: float):
        """Dynamically updates maximum breadcrumb history timeout."""
        self.breadcrumb_watchdog.set_max_timeout(timeout_s)

    def set_mode(self, mode: str):
        self.mode = mode
        self.reset()

    def reset(self):
        self.breadcrumb_watchdog.reset()
        self.latest_status = WrongWayStatus()
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
        candidate_gaps: Optional[List[Any]] = None,
        **kwargs,
    ) -> WrongWayStatus:
        """
        Executes one update step of the active anti-wrong-way detection mode.
        """
        if now_s is None:
            now_s = time.time()

        if self.mode == WrongWayOptionMode.DISABLED or self.mode == "DISABLED":
            self.latest_status = WrongWayStatus()
            self.consecutive_ww_steps = 0
            return self.latest_status

        # 1. Breadcrumb logging with anti-contamination gating
        self.breadcrumb_watchdog.add_point(x, y, yaw, v_est, now_s, state=state, yaw_rate=yaw_rate)

        # 2. Check Line-of-Sight Breadcrumb Overlap
        status = WrongWayStatus()
        is_ww, score, target_yaw, rays = self.breadcrumb_watchdog.check_wrong_way(
            x, y, yaw, hazard_mask=hazard_mask, now_s=now_s
        )
        status.sightline_rays = rays
        status.overlap_score = score
        if is_ww:
            status.is_wrong_way = True
            status.confidence = score
            status.detected_by_option = "OPTION_1_BREADCRUMB"
            status.target_recovery_yaw = target_yaw
            status.corrective_action = "CORRECTIVE_180"

        # 3. Check Candidate Gaps for Long-Path Early Exits (Narrow-Cone Sentry)
        # Early exit is ONLY valid during U-turn recovery (CORRECTIVE_180) to terminate the turn early,
        # and ONLY when the vehicle has turned enough that forward Line-of-Sight is NO LONGER facing
        # confirmed opposing breadcrumbs (is_ww is False).
        # If the vehicle has confirmed opposing breadcrumbs in line of sight (is_ww is True),
        # it is strictly facing the wrong way and MUST NOT abort or suppress the 180!
        is_rec_180 = (
            state == "CORRECTIVE_180"
            or getattr(state, "name", "") == "CORRECTIVE_180"
            or getattr(state, "value", "") == "CORRECTIVE_180"
        )
        if is_rec_180 and not is_ww:
            c_gaps = candidate_gaps or kwargs.get("candidate_gaps", None)
            if c_gaps:
                for c_gap in c_gaps:
                    if isinstance(c_gap, (list, tuple)) and len(c_gap) >= 2:
                        gx, gy = float(c_gap[0]), float(c_gap[1])
                        is_exit, _ = self.is_gap_early_exit(x, y, yaw, gx, gy)
                        if is_exit:
                            status.early_exit_found = True
                            status.early_exit_gap = (gx, gy)
                            break

        # 4. Multi-step confirmation filter:
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

    def check_gap_channel(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        gap_xg: float,
        gap_yg: float,
        cone_deg: float = AWW_EARLY_EXIT_CONE_DEG,
        opposing_deg: float = AWW_EARLY_EXIT_OPPOSING_DEG,
        min_gap_dist: float = AWW_EARLY_EXIT_MIN_GAP_M,
        min_bc_dist: float = AWW_EARLY_EXIT_MIN_BC_DIST_M,
    ) -> Tuple[bool, int, List[Dict[str, Any]]]:
        """Exposes Narrow-Cone Early Gap Exit Sentry from BreadcrumbWatchdog."""
        return self.breadcrumb_watchdog.check_gap_channel(
            current_x=current_x,
            current_y=current_y,
            current_yaw=current_yaw,
            gap_xg=gap_xg,
            gap_yg=gap_yg,
            cone_deg=cone_deg,
            opposing_deg=opposing_deg,
            min_gap_dist=min_gap_dist,
            min_bc_dist=min_bc_dist,
        )

    def is_gap_early_exit(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        gap_xg: float,
        gap_yg: float,
        min_gap_dist: float = AWW_EARLY_EXIT_MIN_GAP_M,
        cone_deg: float = AWW_EARLY_EXIT_CONE_DEG,
        opposing_deg: float = AWW_EARLY_EXIT_OPPOSING_DEG,
        min_bc_dist: float = AWW_EARLY_EXIT_MIN_BC_DIST_M,
    ) -> Tuple[bool, Dict[str, Any]]:
        """Evaluates whether candidate gap qualifies as a safe long-path early exit."""
        return self.breadcrumb_watchdog.is_gap_early_exit(
            current_x=current_x,
            current_y=current_y,
            current_yaw=current_yaw,
            gap_xg=gap_xg,
            gap_yg=gap_yg,
            min_gap_dist=min_gap_dist,
            cone_deg=cone_deg,
            opposing_deg=opposing_deg,
            min_bc_dist=min_bc_dist,
        )

    def get_detailed_telemetry(
        self,
        x: float,
        y: float,
        yaw: float,
        v_est: float,
        yaw_rate: float,
        now_s: Optional[float] = None,
        hazard_mask: Optional[np.ndarray] = None,
        **kwargs,
    ) -> Dict[str, Any]:
        """
        Generates rich visual and narrative telemetry to stream to the Thought Process GUI.
        """
        if now_s is None:
            now_s = time.time()

        # Breadcrumb LOS raycast
        ww_bc, score_bc, target_yaw_bc, rays_bc = self.breadcrumb_watchdog.check_wrong_way(
            x, y, yaw, hazard_mask=hazard_mask, now_s=now_s
        )
        n_bc = len(self.breadcrumb_watchdog.breadcrumbs)
        eff_lookback = self.breadcrumb_watchdog.effective_lookback_m
        dur_scale = self.breadcrumb_watchdog.duration_scale
        thought_bc = (
            f"Trail has {n_bc} points ({eff_lookback:.1f}m window @ {dur_scale:.2f}x). Unobstructed line-of-sight to opposing trail confirmed (score: {score_bc:.2f}) -> IN-LANE WRONG-WAY DETECTED! Action: Spin 180° back."
            if ww_bc else
            f"Trail has {n_bc} points ({eff_lookback:.1f}m window @ {dur_scale:.2f}x). Raycast scan: forward corridor is clear or adjacent lane blocked by wall. Heading towards open track."
        )

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
                "duration_scale": round(float(dur_scale), 2),
                "effective_lookback_m": round(float(eff_lookback), 2),
                "thought": thought_bc,
            },
            "opt2": {
                "name": "Option 2: Kinematic Turn Radius Filter (Deprecated)",
                "radius_m": 0.50,
                "min_radius_m": 0.32,
                "is_tripped": False,
                "v_est": round(float(v_est), 3),
                "yaw_rate": round(float(yaw_rate), 3),
                "thought": "Option 2 retired in favor of Breadcrumb LOS Raycast.",
            },
            "opt3": {
                "name": "Option 3: Directional Flow Costmap (Deprecated)",
                "dot_product": 1.0,
                "is_tripped": False,
                "cells_count": 0,
                "thought": "Option 3 retired in favor of Breadcrumb LOS Raycast.",
            },
            "opt4": {
                "name": "Option 4: Lateral Corridor Separation (Deprecated)",
                "in_turn": False,
                "turn_deg": 0.0,
                "lateral_m": 0.0,
                "min_lateral_m": 0.40,
                "is_tripped": False,
                "thought": "Option 4 retired in favor of Breadcrumb LOS Raycast.",
            },
            "early_exit": {
                "found": bool(self.latest_status.early_exit_found),
                "gap": [round(float(v), 3) for v in self.latest_status.early_exit_gap] if self.latest_status.early_exit_gap else None,
            },
            "trail": trail_pts[-60:] if trail_pts else [],
        }
