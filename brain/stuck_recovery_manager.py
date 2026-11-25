"""
================================================================================
  INTELLIGENT AUTONOMOUS REVERSE, DECEL & CENTER RECOVERY CONTROLLER
  Streamlined 2-Phase Recovery:
  1. PIVOT_REVERSE: Reverse arc with escape steering until ToF clearance opens up.
  2. DECEL_AND_CENTER: Decelerates to 0 m/s concurrently while slewing servo back
     to 0 radians at physical servo speed (PTK 9497 8.4V @ 15.4 rad/s).
  3. Clean Handshake to Curvilinear SL-MPC with zero residual steering error.
================================================================================
"""

import math
from typing import Any, Dict, NamedTuple, Optional, Tuple
import numpy as np

try:
    from .planner_config import (
        WHEELBASE,
        MAX_STEERING_RAD,
        GRAVITY_G,
        MPC_MU_FRICTION,
        RECOVERY_REVERSE_SPEED_MPS,
        RECOVERY_CORRECTIVE_180_SPEED_MPS,
        RECOVERY_STEER_ANGLE_RAD,
        RECOVERY_REVERSE_STROKE_MAX_S,
        RECOVERY_TOTAL_TIMEOUT_S,
        RECOVERY_180_TIMEOUT_S,
        RECOVERY_180_TIMEOUT_WITH_REV_S,
        RECOVERY_180_REV_CLEAR_M,
        RECOVERY_180_REV_MIN_S,
        RECOVERY_180_ALIGN_DEG,
        RECOVERY_180_EARLY_EXIT_TURN_DEG,
        RECOVERY_CLEARANCE_EXIT_M,
        RECOVERY_ANY_TOF_EXIT_M,
        RECOVERY_EXIT_MIN_TIME_S,
        RECOVERY_SERVO_SLEW_RAD_S,
        RECOVERY_ARC_LOOKAHEAD_M,
        TOF_MAX_RANGE_M,
        V_MIN_CRUISE,
    )
except ImportError:
    from planner_config import (
        WHEELBASE,
        MAX_STEERING_RAD,
        GRAVITY_G,
        MPC_MU_FRICTION,
        RECOVERY_REVERSE_SPEED_MPS,
        RECOVERY_CORRECTIVE_180_SPEED_MPS,
        RECOVERY_STEER_ANGLE_RAD,
        RECOVERY_REVERSE_STROKE_MAX_S,
        RECOVERY_TOTAL_TIMEOUT_S,
        RECOVERY_180_TIMEOUT_S,
        RECOVERY_180_TIMEOUT_WITH_REV_S,
        RECOVERY_180_REV_CLEAR_M,
        RECOVERY_180_REV_MIN_S,
        RECOVERY_180_ALIGN_DEG,
        RECOVERY_180_EARLY_EXIT_TURN_DEG,
        RECOVERY_CLEARANCE_EXIT_M,
        RECOVERY_ANY_TOF_EXIT_M,
        RECOVERY_EXIT_MIN_TIME_S,
        RECOVERY_SERVO_SLEW_RAD_S,
        RECOVERY_ARC_LOOKAHEAD_M,
        TOF_MAX_RANGE_M,
        V_MIN_CRUISE,
    )


def wrap_to_pi(angle_rad: float) -> float:
    """Wraps an angle into [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


class TofSummary(NamedTuple):
    min_l: float
    min_r: float
    avg_l: float
    avg_r: float
    d_straight: float
    max_any: float
    count: int


class StuckRecoveryState:
    NORMAL_DRIVE = "DRIVE"
    PIVOT_REVERSE = "PIVOT_REVERSE"
    DECEL_AND_CENTER = "DECEL_AND_CENTER"
    CORRECTIVE_180 = "CORRECTIVE_180"


class StuckRecoveryManager:
    """
    Intelligent Autonomous Recovery Controller:
    - Evaluates forward kinematic escape arcs (Left/Right max steering lock & Center)
      projected directly onto the inflated costmap (hazard_mask).
    - Reverses away from obstacle along escape curvature until front clearance opens up.
    - Decelerates reverse motion smoothly to 0 m/s while concurrently centering the front servo.
    - Hands control cleanly back to the path planner with 0 steering offset and 0 yaw rate shock.
    """

    def __init__(
        self,
        reverse_speed_m_s: float = RECOVERY_REVERSE_SPEED_MPS,
        forward_speed_m_s: float = V_MIN_CRUISE,
        corrective_forward_speed_m_s: float = RECOVERY_CORRECTIVE_180_SPEED_MPS,
        steer_angle_rad: float = RECOVERY_STEER_ANGLE_RAD,
        clearance_exit_m: float = RECOVERY_CLEARANCE_EXIT_M,
        any_tof_exit_m: float = RECOVERY_ANY_TOF_EXIT_M,
        servo_slew_rad_s: float = RECOVERY_SERVO_SLEW_RAD_S,
        wheelbase_m: float = WHEELBASE,
        max_steer_rad: float = MAX_STEERING_RAD,
        arc_lookahead_m: float = RECOVERY_ARC_LOOKAHEAD_M,
        mu_friction: float = MPC_MU_FRICTION,
        **kwargs,
    ):
        self.state = StuckRecoveryState.NORMAL_DRIVE
        self.reverse_speed = float(reverse_speed_m_s)
        self.forward_speed = float(forward_speed_m_s)
        self.corrective_forward_speed = float(corrective_forward_speed_m_s)
        self.steer_angle = float(steer_angle_rad)
        self.clearance_exit_m = float(clearance_exit_m)
        self.any_tof_exit_m = float(any_tof_exit_m)
        self.servo_slew_rad_s = float(servo_slew_rad_s)
        self.mu_friction = float(mu_friction)

        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        self.preferred_yaw_sign = 1.0

        # State tracking for smooth ramping
        self.current_cmd_v = 0.0
        self.current_cmd_steer = 0.0

        # Anti-Wrong-Way CORRECTIVE_180 tracking & Reverse Support
        self.target_corrective_yaw: Optional[float] = None
        self.last_corrective_yaw: float = 0.0
        self.corrective_accumulated_turn: float = 0.0
        self.corrective_net_turn: float = 0.0
        self.corrective_used_reverse: bool = False
        self.corrective_substate: str = "FORWARD"  # "FORWARD" or "REVERSE"
        self.corrective_steer_dir: float = 1.0     # +1.0 (Left) or -1.0 (Right) towards more open side
        self.corrective_reverse_timer: float = 0.0
        self.forward_grace_timer: float = 0.0
        self._rev_best_d_straight: float = 0.0
        self._rev_stall_timer: float = 0.0
        self.corrective_rev_clear_m: float = float(kwargs.get("corrective_rev_clear_m", RECOVERY_180_REV_CLEAR_M))
        self.corrective_rev_min_s: float = float(kwargs.get("corrective_rev_min_s", RECOVERY_180_REV_MIN_S))
        self.corrective_align_rad: float = math.radians(float(kwargs.get("corrective_align_deg", RECOVERY_180_ALIGN_DEG)))
        self.corrective_early_exit_turn_rad: float = math.radians(float(kwargs.get("corrective_early_exit_turn_deg", RECOVERY_180_EARLY_EXIT_TURN_DEG)))

        # --- KINEMATIC ESCAPE ARCS PRECOMPUTATION ---
        self.wheelbase = float(wheelbase_m)
        self.max_steer_rad = float(max_steer_rad)
        self.r_min_turn = self.wheelbase / max(1e-3, math.tan(self.max_steer_rad))
        self.origin_y = 0.0   # Robot center origin (0.0, 0.0)
        self.bumper_y = 0.095 # Front bumper mount Y in vehicle frame

        # State tracking for pinned evaluation
        self.is_left_pinned: bool = False
        self.is_right_pinned: bool = False

        self.arc_lookahead_m = max(0.65, float(arc_lookahead_m))
        self.arc_step_ds = 0.015  # 15mm steps
        n_pts = max(3, int(self.arc_lookahead_m / self.arc_step_ds))
        self._arc_s = np.linspace(0.0, self.arc_lookahead_m, n_pts, dtype=np.float32)

        # Left Turn Arc (full left lock: sweeps left -X from robot center)
        self._arc_pts_left = np.zeros((n_pts, 2), dtype=np.float32)
        self._arc_pts_left[:, 0] = -self.r_min_turn * (1.0 - np.cos(self._arc_s / self.r_min_turn))
        self._arc_pts_left[:, 1] = self.origin_y + self.r_min_turn * np.sin(self._arc_s / self.r_min_turn)

        # Right Turn Arc (full right lock: sweeps right +X from robot center)
        self._arc_pts_right = np.zeros((n_pts, 2), dtype=np.float32)
        self._arc_pts_right[:, 0] = self.r_min_turn * (1.0 - np.cos(self._arc_s / self.r_min_turn))
        self._arc_pts_right[:, 1] = self.origin_y + self.r_min_turn * np.sin(self._arc_s / self.r_min_turn)

        # Center Straight Sightline (from robot center)
        self._arc_pts_center = np.zeros((n_pts, 2), dtype=np.float32)
        self._arc_pts_center[:, 0] = 0.0
        self._arc_pts_center[:, 1] = self.origin_y + self._arc_s

    # --- STATE TRANSITION & COMMAND HELPERS ---
    def reset(self) -> None:
        """Resets all recovery state machine timers, flags, and corrective tracking."""
        self.state = StuckRecoveryState.NORMAL_DRIVE
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        self.preferred_yaw_sign = 1.0
        self.current_cmd_v = 0.0
        self.current_cmd_steer = 0.0

        self.target_corrective_yaw = None
        self.last_corrective_yaw = 0.0
        self.corrective_accumulated_turn = 0.0
        self.corrective_net_turn = 0.0
        self.corrective_used_reverse = False
        self.corrective_substate = "FORWARD"
        self.corrective_steer_dir = 1.0
        self.corrective_reverse_timer = 0.0
        self.forward_grace_timer = 0.0
        self._rev_best_d_straight = 0.0
        self._rev_stall_timer = 0.0

        self.is_left_pinned = False
        self.is_right_pinned = False

    def _finish_recovery(self) -> Tuple[bool, float, float]:
        self.state = StuckRecoveryState.NORMAL_DRIVE
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        self.current_cmd_v = 0.0
        self.current_cmd_steer = 0.0
        return False, 0.0, 0.0

    def finish_recovery_early(self) -> Tuple[bool, float, float]:
        """Immediately terminates active recovery maneuvers when a safe early exit path is scouted."""
        return self._finish_recovery()

    def _enter_decel_and_center(self) -> Tuple[bool, float, float]:
        self.state = StuckRecoveryState.DECEL_AND_CENTER
        self.stroke_timer = 0.0
        self.current_cmd_v = 0.0
        self.current_cmd_steer = 0.0
        return True, 0.0, 0.0

    def _cmd(self, v: float, steer: float) -> Tuple[bool, float, float]:
        self.current_cmd_v = float(v)
        self.current_cmd_steer = float(steer)
        return True, self.current_cmd_v, self.current_cmd_steer

    # --- UNIFIED SENSOR PARSING ---
    @staticmethod
    def _is_sensor_left(key: str) -> bool:
        """Determines if a sensor observes the vehicle's left side."""
        k = key.lower()
        if "tof_front_left_4.1deg" in k:
            return True
        if "tof_front_right_4.1deg" in k:
            return False
        # Cross-pointing lateral sensors: front_right points LEFT (-X)
        if "tof_front_right_" in k:
            return True
        if "tof_front_left_" in k:
            return False
        if "side_left" in k or "tof_fl" in k:
            return True
        if "side_right" in k or "tof_fr" in k:
            return False
        return "left" in k

    @staticmethod
    def _is_sensor_right(key: str) -> bool:
        """Determines if a sensor observes the vehicle's right side."""
        k = key.lower()
        if "tof_front_right_4.1deg" in k:
            return True
        if "tof_front_left_4.1deg" in k:
            return False
        # Cross-pointing lateral sensors: front_left points RIGHT (+X)
        if "tof_front_left_" in k:
            return True
        if "tof_front_right_" in k:
            return False
        if "side_right" in k or "tof_fr" in k:
            return True
        if "side_left" in k or "tof_fl" in k:
            return False
        return "right" in k

    @classmethod
    def parse_cached_tofs(cls, cached_tofs: Optional[Any]) -> TofSummary:
        """Centralized parsing method for cached ToF sensor readings (ignoring <= 0.0 offline sentinels)."""
        if isinstance(cached_tofs, TofSummary):
            return cached_tofs
        if not cached_tofs:
            return TofSummary(TOF_MAX_RANGE_M, TOF_MAX_RANGE_M, TOF_MAX_RANGE_M, TOF_MAX_RANGE_M, TOF_MAX_RANGE_M, 0.0, 0)

        def _is_valid(val: Any) -> bool:
            try:
                f = float(val)
                return f > 0.0 and not math.isnan(f) and not math.isinf(f)
            except (TypeError, ValueError):
                return False

        if isinstance(cached_tofs, dict):
            f_vals = [
                float(cached_tofs[k])
                for k in ("tof_front_left_4.1deg", "tof_front_right_4.1deg")
                if k in cached_tofs and _is_valid(cached_tofs[k])
            ]
            d_str = min(f_vals) if f_vals else TOF_MAX_RANGE_M
            l_v = [float(v) for k, v in cached_tofs.items() if cls._is_sensor_left(k) and _is_valid(v)]
            r_v = [float(v) for k, v in cached_tofs.items() if cls._is_sensor_right(k) and _is_valid(v)]
            all_v = [float(v) for v in cached_tofs.values() if _is_valid(v)]
        else:
            vals = [float(v) for v in cached_tofs]
            half = len(vals) // 2
            l_v = [v for v in vals[:half] if _is_valid(v)]
            r_v = [v for v in vals[half:] if _is_valid(v)]
            all_v = [v for v in vals if _is_valid(v)]
            d_str = min(all_v) if all_v else TOF_MAX_RANGE_M

        min_l = min(l_v) if l_v else TOF_MAX_RANGE_M
        min_r = min(r_v) if r_v else TOF_MAX_RANGE_M
        avg_l = sum(l_v) / len(l_v) if l_v else TOF_MAX_RANGE_M
        avg_r = sum(r_v) / len(r_v) if r_v else TOF_MAX_RANGE_M
        max_any = max(all_v) if all_v else 0.0
        return TofSummary(min_l, min_r, avg_l, avg_r, d_str, max_any, len(all_v))

    def evaluate_costmap_escape_arcs(
        self,
        hazard_mask: Optional[np.ndarray],
        costmap_obj: Optional[Any],
    ) -> Tuple[float, float, float]:
        """Projects maximum steering lock arcs and center sightline onto the inflated costmap."""
        if hazard_mask is None or costmap_obj is None:
            return (TOF_MAX_RANGE_M, TOF_MAX_RANGE_M, TOF_MAX_RANGE_M)

        res = getattr(costmap_obj, "res", 0.015)
        x_min = getattr(costmap_obj, "x_min", -1.25)
        y_min = getattr(costmap_obj, "y_min", 0.00)
        h, w = hazard_mask.shape

        def _trace(pts: np.ndarray) -> float:
            cols = np.round((pts[:, 0] - x_min) / res).astype(np.int32)
            rows = np.round((pts[:, 1] - y_min) / res).astype(np.int32)
            valid = (pts[:, 1] >= self.bumper_y) & (0 <= rows) & (rows < h) & (0 <= cols) & (cols < w)
            hit_mask = np.zeros(len(pts), dtype=bool)
            hit_mask[valid] = hazard_mask[rows[valid], cols[valid]] > 0
            hits = np.flatnonzero(hit_mask)
            if len(hits) > 0:
                return float(self._arc_s[max(0, hits[0] - 1)])
            return float(self._arc_s[-1])

        return (_trace(self._arc_pts_left), _trace(self._arc_pts_right), _trace(self._arc_pts_center))

    def evaluate_pinned_state(
        self,
        hazard_mask: Optional[np.ndarray],
        costmap_obj: Optional[Any],
        cached_tofs: Optional[Any] = None,
        steer_dir: Optional[float] = None,
    ) -> Tuple[bool, bool]:
        """Sparse check if robot is pinned against obstacles in the inflated costmap."""
        tof = self.parse_cached_tofs(cached_tofs)
        is_l = (tof.min_l <= 0.035)
        is_r = (tof.min_r <= 0.035)

        if hazard_mask is not None and costmap_obj is not None:
            res = getattr(costmap_obj, "res", 0.015)
            x_min = getattr(costmap_obj, "x_min", -1.25)
            y_min = getattr(costmap_obj, "y_min", 0.00)
            h, w = hazard_mask.shape
            r = int(round((0.10 - y_min) / res))
            if 0 <= r < h:
                if steer_dir is None or steer_dir < 0.0:
                    cols = [int(round((x - x_min) / res)) for x in (-0.020, -0.035, -0.050)]
                    is_l = is_l or any(0 <= c < w and hazard_mask[r, c] > 0 for c in cols)
                if steer_dir is None or steer_dir > 0.0:
                    cols = [int(round((x - x_min) / res)) for x in (0.020, 0.035, 0.050)]
                    is_r = is_r or any(0 <= c < w and hazard_mask[r, c] > 0 for c in cols)

        self.is_left_pinned = bool(is_l)
        self.is_right_pinned = bool(is_r)
        return (self.is_left_pinned, self.is_right_pinned)

    # --- STEERING ARBITRATION ROUTINES ---
    @staticmethod
    def arbitrate_open_direction(
        arc_clearance: Tuple[float, float, float],
        cached_tofs: Optional[Any] = None,
        heading_err: float = 0.0,
        is_left_pinned: bool = False,
        is_right_pinned: bool = False,
        similar_eps_m: float = 0.030,
        default_dir: float = 1.0,
    ) -> float:
        """Unified direction arbitration (+1.0 Left, -1.0 Right)."""
        diff_arc = arc_clearance[0] - arc_clearance[1]
        if abs(diff_arc) > similar_eps_m:
            return 1.0 if diff_arc > 0.0 else -1.0

        if cached_tofs:
            tof = StuckRecoveryManager.parse_cached_tofs(cached_tofs)
            if tof.min_l < 0.12 or tof.min_r < 0.12:
                return 1.0 if tof.min_l > tof.min_r else -1.0
            diff_avg = tof.avg_l - tof.avg_r
            if abs(diff_avg) > similar_eps_m:
                return 1.0 if diff_avg > 0.0 else -1.0
            diff_min = tof.min_l - tof.min_r
            if abs(diff_min) > similar_eps_m:
                return 1.0 if diff_min > 0.0 else -1.0

        if is_left_pinned and not is_right_pinned:
            return -1.0
        if is_right_pinned and not is_left_pinned:
            return 1.0

        if math.radians(10.0) < abs(heading_err) < math.radians(170.0):
            return 1.0 if heading_err > 0.0 else -1.0

        return default_dir

    @staticmethod
    def resolve_escape_steer_dir(
        arc_clearance: Tuple[float, float, float],
        heading_err: float = 0.0,
        similar_eps_m: float = 0.020,
        is_left_pinned: bool = False,
        is_right_pinned: bool = False,
        **kwargs,
    ) -> float:
        """Arbitrates reverse escape steering (swings front nose away from obstacles in reverse)."""
        if is_left_pinned and not is_right_pinned:
            return 1.0
        if is_right_pinned and not is_left_pinned:
            return -1.0

        diff_arc = arc_clearance[0] - arc_clearance[1]
        if abs(diff_arc) > similar_eps_m:
            return 1.0 if diff_arc > 0.0 else -1.0

        if math.radians(10.0) < abs(heading_err) < math.radians(170.0):
            return 1.0 if heading_err > 0.0 else -1.0

        return -1.0

    @classmethod
    def resolve_open_side(
        cls,
        arc_clearance: Tuple[float, float, float],
        cached_tofs: Optional[Any] = None,
        heading_err: float = 0.0,
        similar_eps_m: float = 0.030,
        is_left_pinned: bool = False,
        is_right_pinned: bool = False,
        default_dir: float = 1.0,
        **kwargs,
    ) -> float:
        """Determines the clearer side for a forward U-turn (+1.0 Left, -1.0 Right).
        Must choose the side with MORE free space / clearance to complete the 180 spin.
        """
        # 1. Pinned obstacles override: steer AWAY from the pinned wall!
        if is_left_pinned and not is_right_pinned:
            return -1.0
        if is_right_pinned and not is_left_pinned:
            return 1.0

        # 2. Costmap Arc Clearances: Left (+1.0) vs Right (-1.0)
        diff_arc = arc_clearance[0] - arc_clearance[1]
        if abs(diff_arc) > similar_eps_m:
            return 1.0 if diff_arc > 0.0 else -1.0

        # 3. Lateral ToF Clearances: Left (+1.0) vs Right (-1.0)
        if cached_tofs:
            tof = cls.parse_cached_tofs(cached_tofs)
            if tof.min_l < 0.15 or tof.min_r < 0.15:
                return 1.0 if tof.min_l > tof.min_r else -1.0
            diff_avg = tof.avg_l - tof.avg_r
            if abs(diff_avg) > similar_eps_m:
                return 1.0 if diff_avg > 0.0 else -1.0
            diff_min = tof.min_l - tof.min_r
            if abs(diff_min) > similar_eps_m:
                return 1.0 if diff_min > 0.0 else -1.0

        # 4. Heading error bias (if clearances are symmetric)
        if math.radians(10.0) < abs(heading_err) < math.radians(170.0):
            return 1.0 if heading_err > 0.0 else -1.0

        return default_dir

    def compute_reverse_steer(
        self,
        min_l_tof: float = TOF_MAX_RANGE_M,
        min_r_tof: float = TOF_MAX_RANGE_M,
        current_yaw: float = 0.0,
        target_yaw: Optional[float] = None,
        current_v: float = 0.0,
        preferred_sign: Optional[float] = None,
        is_left_pinned: Optional[bool] = None,
        is_right_pinned: Optional[bool] = None,
        hazard_mask: Optional[np.ndarray] = None,
        costmap_obj: Optional[Any] = None,
        **kwargs,
    ) -> float:
        """Single Source of Truth for Reverse Steering Arbitration."""
        if target_yaw is not None:
            heading_err = wrap_to_pi(target_yaw - current_yaw)
            if abs(heading_err) > math.radians(10.0):
                intended_steer_dir = -1.0 if heading_err > 0.0 else 1.0
            else:
                pref = preferred_sign if preferred_sign is not None else self.preferred_yaw_sign
                intended_steer_dir = -pref
        else:
            pref = preferred_sign if preferred_sign is not None else self.preferred_yaw_sign
            intended_steer_dir = -pref

        if is_left_pinned is None or is_right_pinned is None:
            if hazard_mask is not None and costmap_obj is not None:
                l_pin, r_pin = self.evaluate_pinned_state(hazard_mask, costmap_obj)
                is_left_pinned = l_pin if is_left_pinned is None else is_left_pinned
                is_right_pinned = r_pin if is_right_pinned is None else is_right_pinned
            else:
                is_left_pinned = (min_l_tof <= 0.035) if is_left_pinned is None else is_left_pinned
                is_right_pinned = (min_r_tof <= 0.035) if is_right_pinned is None else is_right_pinned

        if is_left_pinned and is_right_pinned:
            reverse_steer = 0.0
        elif intended_steer_dir < 0.0:
            reverse_steer = +self.steer_angle if is_left_pinned else -self.steer_angle
        else:
            reverse_steer = -self.steer_angle if is_right_pinned else +self.steer_angle

        # Above 0.5 m/s forward speed, continue steering the opposite of reverse steer,
        # capped by the tire friction circle so braking + steering stays within grip.
        if current_v >= 0.5:
            if reverse_steer == 0.0:
                return 0.0
            a_lat_max = max(0.1, self.mu_friction) * GRAVITY_G
            friction_steer_cap = math.atan((a_lat_max * self.wheelbase) / (current_v * current_v))
            steer_mag = min(self.steer_angle, friction_steer_cap)
            return -math.copysign(steer_mag, reverse_steer)

        return reverse_steer

    def _step_reverse_stroke(
        self,
        tof: TofSummary,
        current_yaw: float,
        target_yaw: Optional[float],
        current_v: float,
        preferred_sign: float,
        is_l_pinned: bool,
        is_r_pinned: bool,
        hazard_mask: Optional[np.ndarray],
        costmap_obj: Optional[Any],
    ) -> Tuple[bool, float, float]:
        """Unified execution step for reverse motion across PIVOT_REVERSE and CORRECTIVE_180."""
        steer = self.compute_reverse_steer(
            min_l_tof=tof.min_l,
            min_r_tof=tof.min_r,
            current_yaw=current_yaw,
            target_yaw=target_yaw,
            current_v=current_v,
            preferred_sign=preferred_sign,
            is_left_pinned=is_l_pinned,
            is_right_pinned=is_r_pinned,
            hazard_mask=hazard_mask,
            costmap_obj=costmap_obj,
        )
        return self._cmd(-self.reverse_speed, steer)

    def trigger_emergency_recovery(
        self,
        heading_err: float = 0.0,
        steer_dir: Optional[float] = None,
        current_v: float = 0.0,
        current_steer: float = 0.0,
        instant_reverse: bool = True,
        target_breadcrumb_yaw: Optional[float] = None,
        current_yaw: float = 0.0,
        front_clearance_m: float = 0.0,
        hazard_mask: Optional[np.ndarray] = None,
        costmap_obj: Optional[Any] = None,
        cached_tofs: Optional[Any] = None,
        is_left_pinned: Optional[bool] = None,
        is_right_pinned: Optional[bool] = None,
        is_wrong_way: Optional[bool] = None,
        **kwargs,
    ):
        """Immediately engages recovery maneuver.
        If facing wrong way and front is open, executes a forward U-turn (CORRECTIVE_180).
        Otherwise, engages reverse recovery (PIVOT_REVERSE).
        """
        tof = self.parse_cached_tofs(cached_tofs)
        d_front = tof.d_straight if tof.count > 0 else float(front_clearance_m)

        if is_left_pinned is None or is_right_pinned is None:
            l_pin, r_pin = self.evaluate_pinned_state(hazard_mask, costmap_obj, cached_tofs)
            is_left_pinned = l_pin if is_left_pinned is None else is_left_pinned
            is_right_pinned = r_pin if is_right_pinned is None else is_right_pinned

        # Determine if facing wrong way (authoritatively provided by breadcrumb watchdog)
        is_facing_wrong_way = False
        yaw_diff = 0.0
        if target_breadcrumb_yaw is not None:
            yaw_diff = wrap_to_pi(target_breadcrumb_yaw - current_yaw)
            is_facing_wrong_way = bool(is_wrong_way) if is_wrong_way is not None else (abs(yaw_diff) > math.radians(75.0))

        # Front is considered open if clearance >= 0.22m and not pinned on both sides
        is_front_open = (d_front >= 0.22) and not (is_left_pinned and is_right_pinned)

        if is_facing_wrong_way and is_front_open:
            # Front is open: simply do a U-turn (CORRECTIVE_180 forward) towards the open side!
            arc_cl = self.evaluate_costmap_escape_arcs(hazard_mask, costmap_obj)
            open_side = steer_dir if steer_dir is not None else self.resolve_open_side(
                arc_clearance=arc_cl,
                cached_tofs=cached_tofs,
                heading_err=yaw_diff,
            )
            tgt_yaw = target_breadcrumb_yaw if target_breadcrumb_yaw is not None else wrap_to_pi(current_yaw + math.pi)
            self.trigger_corrective_180(
                target_yaw=tgt_yaw,
                steer_sign=open_side,
                current_yaw=current_yaw,
                arc_clearance=arc_cl,
                cached_tofs=cached_tofs,
                front_clearance_m=d_front,
                hazard_mask=hazard_mask,
                costmap_obj=costmap_obj,
                is_left_pinned=is_left_pinned,
                is_right_pinned=is_right_pinned,
            )
            return

        self.state = StuckRecoveryState.PIVOT_REVERSE
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0

        if steer_dir is not None:
            self.preferred_yaw_sign = 1.0 if steer_dir > 0 else -1.0
        elif is_left_pinned and not is_right_pinned:
            self.preferred_yaw_sign = 1.0
        elif is_right_pinned and not is_left_pinned:
            self.preferred_yaw_sign = -1.0
        elif abs(heading_err) > math.radians(25.0):
            self.preferred_yaw_sign = 1.0 if heading_err > 0 else -1.0
        else:
            self.preferred_yaw_sign = -1.0

        target_steer = self.compute_reverse_steer(
            current_yaw=current_yaw,
            target_yaw=target_breadcrumb_yaw,
            current_v=current_v,
            preferred_sign=self.preferred_yaw_sign,
            is_left_pinned=is_left_pinned,
            is_right_pinned=is_right_pinned,
            hazard_mask=hazard_mask,
            costmap_obj=costmap_obj,
        )
        self.current_cmd_v = -self.reverse_speed if instant_reverse else float(current_v)
        self.current_cmd_steer = target_steer if instant_reverse else float(current_steer)

    def trigger_corrective_180(
        self,
        target_yaw: Optional[float] = None,
        steer_sign: Optional[float] = None,
        current_yaw: float = 0.0,
        arc_clearance: Optional[Tuple[float, float, float]] = None,
        cached_tofs: Optional[Any] = None,
        front_clearance_m: float = 1.0,
        hazard_mask: Optional[np.ndarray] = None,
        costmap_obj: Optional[Any] = None,
        is_left_pinned: Optional[bool] = None,
        is_right_pinned: Optional[bool] = None,
    ):
        """Engages Anti-Wrong-Way corrective turn."""
        tof = self.parse_cached_tofs(cached_tofs)
        d_front = tof.d_straight if tof.count > 0 else float(front_clearance_m)

        # If already in PIVOT_REVERSE, only ignore if front is blocked (< 0.22m).
        # If front is open, allow transition from reverse to forward U-turn!
        if self.state == StuckRecoveryState.PIVOT_REVERSE and d_front < 0.22:
            return

        self.state = StuckRecoveryState.CORRECTIVE_180
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        self.target_corrective_yaw = target_yaw
        self.last_corrective_yaw = current_yaw
        self.corrective_accumulated_turn = 0.0
        self.corrective_net_turn = 0.0
        self.corrective_reverse_timer = 0.0
        self._rev_best_d_straight = float(d_front)
        self._rev_stall_timer = 0.0

        if steer_sign is None:
            cl = arc_clearance or (0.5, 0.5, 0.5)
            y_diff = wrap_to_pi(target_yaw - current_yaw) if target_yaw is not None else 0.0
            steer_sign = self.resolve_open_side(cl, cached_tofs=cached_tofs, heading_err=y_diff)

        self.corrective_steer_dir = 1.0 if steer_sign >= 0.0 else -1.0
        self.preferred_yaw_sign = self.corrective_steer_dir

        if d_front < 0.22:
            self.corrective_used_reverse = True
            self.corrective_substate = "REVERSE"
            target_steer = self.compute_reverse_steer(
                min_l_tof=tof.min_l,
                min_r_tof=tof.min_r,
                current_yaw=current_yaw,
                target_yaw=self.target_corrective_yaw,
                current_v=-self.reverse_speed,
                preferred_sign=self.corrective_steer_dir,
                is_left_pinned=is_left_pinned,
                is_right_pinned=is_right_pinned,
                hazard_mask=hazard_mask,
                costmap_obj=costmap_obj,
            )
            self._cmd(-self.reverse_speed, target_steer)
        else:
            self.corrective_used_reverse = False
            self.corrective_substate = "FORWARD"
            self.forward_grace_timer = 0.35
            self._cmd(self.corrective_forward_speed, self.corrective_steer_dir * self.steer_angle)

    def engage_corrective_reverse(self, force: bool = False):
        """Engages reverse support phase during CORRECTIVE_180 when an obstacle is encountered."""
        if self.state == StuckRecoveryState.CORRECTIVE_180 and self.corrective_substate != "REVERSE":
            if force or self.forward_grace_timer <= 0.0:
                self.corrective_substate = "REVERSE"
                self.corrective_reverse_timer = 0.0
                self._rev_best_d_straight = 0.0
                self._rev_stall_timer = 0.0

    def update(
        self,
        dt: float,
        current_v: float,
        commanded_v: float = 0.0,
        heading_err: float = 0.0,
        active_steer: float = 0.0,
        hazard_mask: Optional[np.ndarray] = None,
        costmap_obj: Optional[Any] = None,
        cached_tofs: Optional[Any] = None,
        current_yaw: float = 0.0,
        target_breadcrumb_yaw: Optional[float] = None,
        early_exit_available: bool = False,
        is_wrong_way: Optional[bool] = None,
        **kwargs,
    ) -> Tuple[bool, float, float]:
        """Updates recovery state machine."""
        if self.state == StuckRecoveryState.NORMAL_DRIVE:
            return False, 0.0, 0.0

        self.stroke_timer += dt
        self.total_recovery_timer += dt
        if self.forward_grace_timer > 0.0:
            self.forward_grace_timer -= dt
            if current_v >= 0.05:
                self.forward_grace_timer = 0.0

        # PHASE 3: DECELERATE SPEED TO 0 WHILE CENTERING SERVO
        if self.state == StuckRecoveryState.DECEL_AND_CENTER:
            is_centered = (abs(active_steer) <= 0.04) and (abs(self.current_cmd_steer) <= 0.02)
            if is_centered or (self.stroke_timer >= 0.08):
                return self._finish_recovery()
            return self._cmd(0.0, 0.0)

        # Parse ToF sensor summary and evaluate clearances
        tof = self.parse_cached_tofs(cached_tofs)
        is_l_pinned, is_r_pinned = self.evaluate_pinned_state(hazard_mask, costmap_obj, tof)
        front_blocked = (tof.d_straight < 0.15)
        is_unpinned = (not is_l_pinned and not is_r_pinned)
        front_cleared = (tof.d_straight >= self.clearance_exit_m and not front_blocked and is_unpinned)
        phys_unpinned = (tof.min_l > 0.035 and tof.min_r > 0.035)
        corrective_front_cleared = (tof.d_straight >= self.clearance_exit_m and not front_blocked and phys_unpinned)
        any_tof_cleared = (tof.max_any >= self.any_tof_exit_m) or (tof.count <= 2 and tof.max_any >= self.clearance_exit_m)

        # Track signed net turn across BOTH REVERSE and FORWARD substates of CORRECTIVE_180
        yaw_aligned = False
        is_turn_sufficient = False
        if self.state == StuckRecoveryState.CORRECTIVE_180:
            d_yaw_signed = wrap_to_pi(current_yaw - self.last_corrective_yaw)
            self.corrective_net_turn += d_yaw_signed
            self.corrective_accumulated_turn = abs(self.corrective_net_turn)
            self.last_corrective_yaw = current_yaw

            yaw_aligned = (
                self.target_corrective_yaw is not None
                and abs(wrap_to_pi(self.target_corrective_yaw - current_yaw)) <= self.corrective_align_rad
            )
            if self.target_corrective_yaw is not None:
                is_turn_sufficient = yaw_aligned and abs(self.corrective_net_turn) >= math.radians(60.0)
            else:
                is_turn_sufficient = abs(self.corrective_net_turn) >= math.radians(145.0)

        # REVERSE STROKE EXECUTION (Shared by PIVOT_REVERSE and CORRECTIVE_180[REVERSE])
        if self.state == StuckRecoveryState.PIVOT_REVERSE or (
            self.state == StuckRecoveryState.CORRECTIVE_180 and self.corrective_substate == "REVERSE"
        ):
            if self.state == StuckRecoveryState.PIVOT_REVERSE:
                # Check if facing wrong way with open front -> transition to U-turn!
                is_facing_ww = False
                y_diff = 0.0
                if target_breadcrumb_yaw is not None:
                    y_diff = wrap_to_pi(target_breadcrumb_yaw - current_yaw)
                    is_facing_ww = bool(is_wrong_way) if is_wrong_way is not None else (abs(y_diff) > math.radians(75.0))

                if is_facing_ww and tof.d_straight >= 0.22 and not (is_l_pinned and is_r_pinned):
                    arc_cl = self.evaluate_costmap_escape_arcs(hazard_mask, costmap_obj)
                    tgt_yaw = target_breadcrumb_yaw if target_breadcrumb_yaw is not None else wrap_to_pi(current_yaw + math.pi)
                    open_side = self.resolve_open_side(arc_cl, cached_tofs=tof, heading_err=y_diff)
                    self.trigger_corrective_180(
                        target_yaw=tgt_yaw,
                        steer_sign=open_side,
                        current_yaw=current_yaw,
                        arc_clearance=arc_cl,
                        cached_tofs=tof,
                        front_clearance_m=tof.d_straight,
                        hazard_mask=hazard_mask,
                        costmap_obj=costmap_obj,
                        is_left_pinned=is_l_pinned,
                        is_right_pinned=is_r_pinned,
                    )
                    return self._cmd(self.corrective_forward_speed, self.corrective_steer_dir * self.steer_angle)

                if self.total_recovery_timer >= RECOVERY_TOTAL_TIMEOUT_S and tof.d_straight >= self.clearance_exit_m:
                    return self._finish_recovery()
                if (self.stroke_timer >= RECOVERY_EXIT_MIN_TIME_S and front_cleared and any_tof_cleared) or (
                    self.stroke_timer >= RECOVERY_REVERSE_STROKE_MAX_S
                ):
                    return self._enter_decel_and_center()
                t_yaw = target_breadcrumb_yaw
                p_sign = self.preferred_yaw_sign
            else:
                self.corrective_used_reverse = True
                self.corrective_reverse_timer += dt
                if tof.d_straight > self._rev_best_d_straight + 0.02:
                    self._rev_best_d_straight = tof.d_straight
                    self._rev_stall_timer = 0.0
                else:
                    self._rev_stall_timer += dt

                # If reverse stroke completed alignment and clearance opened up, exit cleanly
                if is_turn_sufficient and corrective_front_cleared and any_tof_cleared:
                    return self._enter_decel_and_center()
                if self.total_recovery_timer >= RECOVERY_180_TIMEOUT_WITH_REV_S:
                    return self._enter_decel_and_center()

                has_min_reverse_time = (self.corrective_reverse_timer >= max(RECOVERY_EXIT_MIN_TIME_S, self.corrective_rev_min_s))
                reverse_timeout = (self.corrective_reverse_timer >= RECOVERY_REVERSE_STROKE_MAX_S)
                rev_clear_target = max(self.clearance_exit_m, self.corrective_rev_clear_m)
                rev_stalled_with_room = (
                    self.corrective_reverse_timer >= 0.45
                    and self._rev_stall_timer >= 0.25
                    and tof.d_straight >= 0.20
                )
                if (has_min_reverse_time and (tof.d_straight >= rev_clear_target or rev_stalled_with_room)) or reverse_timeout:
                    self.corrective_substate = "FORWARD"
                    self.stroke_timer = 0.0
                    self.forward_grace_timer = 0.35
                    d_esc_l, d_esc_r, _ = self.evaluate_costmap_escape_arcs(hazard_mask, costmap_obj)

                    chosen_dir = self.corrective_steer_dir
                    if self.target_corrective_yaw is not None:
                        rem_err = wrap_to_pi(self.target_corrective_yaw - current_yaw)
                        if abs(rem_err) < math.radians(135.0) and abs(rem_err) > math.radians(10.0):
                            desired = 1.0 if rem_err > 0.0 else -1.0
                            desired_blocked = (desired > 0.0 and (is_l_pinned or (d_esc_l < 0.18 and d_esc_r >= 0.28))) or (
                                desired < 0.0 and (is_r_pinned or (d_esc_r < 0.18 and d_esc_l >= 0.28))
                            )
                            if not desired_blocked:
                                chosen_dir = desired
                            elif abs(d_esc_l - d_esc_r) > 0.04:
                                chosen_dir = 1.0 if d_esc_l > d_esc_r else -1.0
                        elif abs(d_esc_l - d_esc_r) > 0.04:
                            chosen_dir = 1.0 if d_esc_l > d_esc_r else -1.0
                    elif abs(d_esc_l - d_esc_r) > 0.04:
                        chosen_dir = 1.0 if d_esc_l > d_esc_r else -1.0

                    self.corrective_steer_dir = chosen_dir
                    self.preferred_yaw_sign = self.corrective_steer_dir
                    return self._cmd(self.corrective_forward_speed, self.corrective_steer_dir * self.steer_angle)
                t_yaw = self.target_corrective_yaw
                p_sign = self.corrective_steer_dir
                rev_l_pin = (tof.min_l <= 0.04) if tof.d_straight >= 0.14 else is_l_pinned
                rev_r_pin = (tof.min_r <= 0.04) if tof.d_straight >= 0.14 else is_r_pinned
                return self._step_reverse_stroke(
                    tof, current_yaw, t_yaw, current_v, p_sign, rev_l_pin, rev_r_pin, hazard_mask, costmap_obj
                )

            return self._step_reverse_stroke(
                tof, current_yaw, t_yaw, current_v, p_sign, is_l_pinned, is_r_pinned, hazard_mask, costmap_obj
            )

        # PHASE 2: CORRECTIVE 180 AUTO-CORRECTION ARC (FORWARD SUBSTATE)
        if self.state == StuckRecoveryState.CORRECTIVE_180:
            # If a safe long-path early exit is found, front is open, and chassis has turned enough, terminate U-turn
            if self.target_corrective_yaw is not None:
                can_early_exit = abs(wrap_to_pi(self.target_corrective_yaw - current_yaw)) <= math.radians(55.0)
            else:
                can_early_exit = abs(self.corrective_net_turn) >= self.corrective_early_exit_turn_rad
            if (early_exit_available or kwargs.get("early_exit_available", False)) and can_early_exit and tof.d_straight >= 0.80 and self.corrective_substate == "FORWARD":
                return self._finish_recovery()

            if is_turn_sufficient and corrective_front_cleared and any_tof_cleared:
                return self._finish_recovery()

            max_180_timeout = RECOVERY_180_TIMEOUT_WITH_REV_S if self.corrective_used_reverse else RECOVERY_180_TIMEOUT_S
            if (self.total_recovery_timer >= 1.2 and tof.d_straight >= 0.20 and is_turn_sufficient) or self.total_recovery_timer >= max_180_timeout:
                return self._finish_recovery()

            # Proactive stopping-distance & corner-clearance reverse trigger so we stop BEFORE wall impact
            d_proactive_rev = max(0.16, min(0.28, 0.15 + 0.14 * max(0.0, current_v)))
            outer_corner_dist = tof.min_r if self.corrective_steer_dir > 0.0 else tof.min_l
            inner_corner_dist = tof.min_l if self.corrective_steer_dir > 0.0 else tof.min_r
            is_front_blocked = (tof.d_straight < 0.16)
            is_proactive_blocked = (self.forward_grace_timer <= 0.0 and tof.d_straight < d_proactive_rev)
            is_corner_imminent = (
                self.forward_grace_timer <= 0.0
                and (outer_corner_dist < 0.12 or inner_corner_dist < 0.08)
                and tof.d_straight < 0.35
            )
            is_stalled = (self.forward_grace_timer <= 0.0 and abs(current_v) < 0.10 and self.stroke_timer > 0.25)
            is_wall_collision = (
                is_front_blocked
                or is_proactive_blocked
                or (tof.d_straight < 0.20 and (tof.min_l <= 0.04 or tof.min_r <= 0.04))
                or is_corner_imminent
            )
            if is_wall_collision:
                self.engage_corrective_reverse(force=True)
            elif is_stalled:
                self.engage_corrective_reverse(force=False)

            if self.corrective_substate == "REVERSE":
                rev_l_pin = (tof.min_l <= 0.04) if tof.d_straight >= 0.14 else is_l_pinned
                rev_r_pin = (tof.min_r <= 0.04) if tof.d_straight >= 0.14 else is_r_pinned
                return self._step_reverse_stroke(
                    tof, current_yaw, self.target_corrective_yaw, current_v,
                    self.corrective_steer_dir, rev_l_pin, rev_r_pin, hazard_mask, costmap_obj
                )

            # Only taper forward U-turn speed in the final braking zone (< 0.30m)
            if tof.d_straight < 0.30:
                taper_v = max(0.65, 3.5 * (tof.d_straight - 0.12))
                fwd_cmd_v = min(self.corrective_forward_speed, taper_v)
            else:
                fwd_cmd_v = self.corrective_forward_speed
            return self._cmd(fwd_cmd_v, self.corrective_steer_dir * self.steer_angle)

        return False, 0.0, 0.0

