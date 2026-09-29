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
from typing import Any, Dict, Optional, Tuple
import numpy as np

try:
    from .planner_config import (
        WHEELBASE,
        MAX_STEERING_RAD,
        RECOVERY_REVERSE_SPEED_MPS,
        RECOVERY_STEER_ANGLE_RAD,
        RECOVERY_REVERSE_STROKE_MAX_S,
        RECOVERY_TOTAL_TIMEOUT_S,
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
        RECOVERY_REVERSE_SPEED_MPS,
        RECOVERY_STEER_ANGLE_RAD,
        RECOVERY_REVERSE_STROKE_MAX_S,
        RECOVERY_TOTAL_TIMEOUT_S,
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
        steer_angle_rad: float = RECOVERY_STEER_ANGLE_RAD,
        clearance_exit_m: float = RECOVERY_CLEARANCE_EXIT_M,
        any_tof_exit_m: float = RECOVERY_ANY_TOF_EXIT_M,
        servo_slew_rad_s: float = RECOVERY_SERVO_SLEW_RAD_S,
        wheelbase_m: float = WHEELBASE,
        max_steer_rad: float = MAX_STEERING_RAD,
        arc_lookahead_m: float = RECOVERY_ARC_LOOKAHEAD_M,
        **kwargs,
    ):
        self.state = StuckRecoveryState.NORMAL_DRIVE
        self.reverse_speed = float(reverse_speed_m_s)
        self.forward_speed = float(forward_speed_m_s)
        self.steer_angle = float(steer_angle_rad)
        self.clearance_exit_m = float(clearance_exit_m)
        self.any_tof_exit_m = float(any_tof_exit_m)
        self.servo_slew_rad_s = float(servo_slew_rad_s)

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
        self.corrective_substate: str = "FORWARD"  # "FORWARD" or "REVERSE"
        self.corrective_steer_dir: float = 1.0     # +1.0 (Left) or -1.0 (Right) towards more open side
        self.corrective_reverse_timer: float = 0.0
        self.corrective_reverse_stroke_count: int = 0
        self.forward_grace_timer: float = 0.0

        # --- KINEMATIC ESCAPE ARCS PRECOMPUTATION ---
        self.wheelbase = float(wheelbase_m)
        self.max_steer_rad = float(max_steer_rad)
        self.r_min_turn = self.wheelbase / max(1e-3, math.tan(self.max_steer_rad))
        self.origin_y = 0.0   # Robot center origin (0.0, 0.0)
        self.bumper_y = 0.095 # Front bumper mount Y in vehicle frame

        # State tracking for pinned evaluation
        self.is_left_pinned: bool = False
        self.is_right_pinned: bool = False

        self.arc_lookahead_m = float(arc_lookahead_m)
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

    def evaluate_costmap_escape_arcs(
        self,
        hazard_mask: Optional[np.ndarray],
        costmap_obj: Optional[Any],
    ) -> Tuple[float, float, float]:
        """
        Projects maximum steering lock arcs (Left, Right) and Center sightline onto the inflated costmap.
        Returns:
            (d_clear_left, d_clear_right, d_clear_center) in meters.
        """
        if hazard_mask is None or costmap_obj is None:
            return (TOF_MAX_RANGE_M, TOF_MAX_RANGE_M, TOF_MAX_RANGE_M)

        res = getattr(costmap_obj, "res", 0.015)
        x_min = getattr(costmap_obj, "x_min", -1.25)
        y_min = getattr(costmap_obj, "y_min", -0.10)
        h, w = hazard_mask.shape

        def _trace(pts: np.ndarray) -> float:
            for i in range(len(pts)):
                px = pts[i, 0]
                py = pts[i, 1]
                if py < self.bumper_y:
                    continue
                col = int(round((px - x_min) / res))
                row = int(round((py - y_min) / res))
                if 0 <= row < h and 0 <= col < w:
                    if hazard_mask[row, col] > 0:
                        return float(self._arc_s[max(0, i - 1)])
            return float(self._arc_s[-1])

        d_left = _trace(self._arc_pts_left)
        d_right = _trace(self._arc_pts_right)
        d_center = _trace(self._arc_pts_center)
        return (d_left, d_right, d_center)

    def evaluate_pinned_state(
        self,
        hazard_mask: Optional[np.ndarray],
        costmap_obj: Optional[Any],
        cached_tofs: Optional[Any] = None,
        steer_dir: Optional[float] = None,
    ) -> Tuple[bool, bool]:
        """
        Sparse check if robot is pinned against obstacles in the inflated costmap.
        Front cannot be pinned because the vehicle is reversing.
        Only evaluates the corner swing for the intended steering lock:
        - If steer_dir < 0 (Right lock): nose swings left -> check left corner swing cells.
        - If steer_dir > 0 (Left lock): nose swings right -> check right corner swing cells.
        - If steer_dir is None: checks both sides.
        """
        min_l_tof = TOF_MAX_RANGE_M
        min_r_tof = TOF_MAX_RANGE_M
        if cached_tofs:
            if isinstance(cached_tofs, dict):
                l_vals = [float(v) for k, v in cached_tofs.items() if "left" in k]
                r_vals = [float(v) for k, v in cached_tofs.items() if "right" in k]
                min_l_tof = min(l_vals) if l_vals else min_l_tof
                min_r_tof = min(r_vals) if r_vals else min_r_tof
            elif isinstance(cached_tofs, (list, tuple, np.ndarray)) and len(cached_tofs) > 0:
                half = len(cached_tofs) // 2
                if half > 0:
                    min_l_tof = float(min(cached_tofs[:half]))
                    min_r_tof = float(min(cached_tofs[half:]))

        is_l = False
        is_r = False

        if hazard_mask is not None and costmap_obj is not None:
            res = getattr(costmap_obj, "res", 0.010)
            x_min = getattr(costmap_obj, "x_min", -1.25)
            y_min = getattr(costmap_obj, "y_min", 0.00)
            h, w = hazard_mask.shape

            def _occ(x: float, y: float) -> bool:
                c = int(round((x - x_min) / res))
                r = int(round((y - y_min) / res))
                return 0 <= r < h and 0 <= c < w and hazard_mask[r, c] > 0

            # Inflated costmap accounts for vehicle half-width (75mm), so probing
            # x in [0.020, 0.050]m directly tests physical corner swing out to 0.125m.
            if steer_dir is None or steer_dir < 0.0:
                is_l = any(_occ(x, 0.10) for x in (-0.020, -0.035, -0.050)) or (min_l_tof <= 0.035)
            if steer_dir is None or steer_dir > 0.0:
                is_r = any(_occ(x, 0.10) for x in (0.020, 0.035, 0.050)) or (min_r_tof <= 0.035)
        else:
            if steer_dir is None or steer_dir < 0.0:
                is_l = (min_l_tof <= 0.035)
            if steer_dir is None or steer_dir > 0.0:
                is_r = (min_r_tof <= 0.035)

        self.is_left_pinned = bool(is_l)
        self.is_right_pinned = bool(is_r)

        return (self.is_left_pinned, self.is_right_pinned)

    @staticmethod
    def resolve_escape_steer_dir(
        arc_clearance: Tuple[float, float, float],
        heading_err: float = 0.0,
        similar_eps_m: float = 0.020,
        is_left_pinned: bool = False,
        is_right_pinned: bool = False,
        **kwargs,
    ) -> float:
        """
        Steering direction resolution when triggering emergency backup:
        If left pinned -> steer left (+1.0) so front nose swings right away from wall.
        If right pinned -> steer right (-1.0) so front nose swings left away from wall.
        Otherwise compares clearance along Right max-lock arc vs Left max-lock arc.
        Returns: -1.0 (steer right: right arc is clearer)
                 or +1.0 (steer left: left arc is clearer)
        """
        if is_left_pinned and not is_right_pinned:
            return 1.0
        if is_right_pinned and not is_left_pinned:
            return -1.0

        d_esc_left, d_esc_right, _ = arc_clearance
        diff_arc = d_esc_right - d_esc_left
        if abs(diff_arc) > similar_eps_m:
            return -1.0 if diff_arc > 0.0 else 1.0

        # Secondary fallback: Heading error alignment (+ is left, - is right)
        if abs(heading_err) > math.radians(10.0):
            return 1.0 if heading_err > 0.0 else -1.0

        return -1.0

    @staticmethod
    def resolve_open_side(
        arc_clearance: Tuple[float, float, float],
        cached_tofs: Optional[Any] = None,
        heading_err: float = 0.0,
        similar_eps_m: float = 0.030,
        **kwargs,
    ) -> float:
        """
        Determines the more open turning direction for CORRECTIVE_180 (+1.0 Left, -1.0 Right).
        1. Evaluates costmap kinematic escape arcs (Left max lock vs Right max lock).
        2. If arcs are symmetric/close, evaluates physical lateral ToF sensor clearance.
        3. Secondary fallback: heading error towards target yaw.
        4. Default fallback: +1.0 (Left).
        """
        d_esc_left, d_esc_right, _ = arc_clearance
        diff_arc = d_esc_left - d_esc_right  # > 0 means left has more clearance

        if abs(diff_arc) > similar_eps_m:
            return 1.0 if diff_arc > 0.0 else -1.0

        # Secondary: Physical ToF sensor evaluation
        if cached_tofs:
            min_l = TOF_MAX_RANGE_M
            min_r = TOF_MAX_RANGE_M
            avg_l = TOF_MAX_RANGE_M
            avg_r = TOF_MAX_RANGE_M
            if isinstance(cached_tofs, dict):
                l_vals = [float(v) for k, v in cached_tofs.items() if "left" in k]
                r_vals = [float(v) for k, v in cached_tofs.items() if "right" in k]
                if l_vals:
                    min_l = min(l_vals)
                    avg_l = sum(l_vals) / len(l_vals)
                if r_vals:
                    min_r = min(r_vals)
                    avg_r = sum(r_vals) / len(r_vals)
            elif isinstance(cached_tofs, (list, tuple, np.ndarray)) and len(cached_tofs) >= 2:
                half = len(cached_tofs) // 2
                l_vals = [float(v) for v in cached_tofs[:half]]
                r_vals = [float(v) for v in cached_tofs[half:]]
                if l_vals:
                    min_l = min(l_vals)
                    avg_l = sum(l_vals) / len(l_vals)
                if r_vals:
                    min_r = min(r_vals)
                    avg_r = sum(r_vals) / len(r_vals)

            # If one side is very close to a barrier / obstacle (< 12 cm), turn toward the other side
            if min_l < 0.12 or min_r < 0.12:
                return 1.0 if min_l > min_r else -1.0

            # Compare average side clearances
            diff_avg = avg_l - avg_r
            if abs(diff_avg) > similar_eps_m:
                return 1.0 if diff_avg > 0.0 else -1.0

            # Compare minimum clearances
            diff_min = min_l - min_r
            if abs(diff_min) > similar_eps_m:
                return 1.0 if diff_min > 0.0 else -1.0

        # Fallback: Heading error towards target yaw
        if abs(heading_err) > math.radians(10.0) and abs(heading_err) < math.radians(170.0):
            return 1.0 if heading_err > 0.0 else -1.0

        return 1.0

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
        """
        Contained Single Source of Truth for Reverse Steering Arbitration:
        Only checks pinned in terms of having steer lock to reverse towards the breadcrumb.
        Front cannot be pinned because the vehicle is reversing.
        """
        # 1. Determine intended steer lock to reverse towards the breadcrumb / target corridor
        if target_yaw is not None:
            heading_err = wrap_to_pi(target_yaw - current_yaw)
            if abs(heading_err) > math.radians(10.0):
                # In reverse (v < 0), psi_dot = (v / L) * tan(delta)
                # To turn towards left (heading_err > 0), steer RIGHT (-1.0)
                # To turn towards right (heading_err < 0), steer LEFT (+1.0)
                intended_steer_dir = -1.0 if heading_err > 0.0 else 1.0
            else:
                pref = preferred_sign if preferred_sign is not None else self.preferred_yaw_sign
                intended_steer_dir = pref if current_v > 0.1 else -pref
        else:
            pref = preferred_sign if preferred_sign is not None else self.preferred_yaw_sign
            intended_steer_dir = pref if current_v > 0.1 else -pref

        # 2. Check pinned state for the intended reverse steer lock
        if is_left_pinned is None or is_right_pinned is None:
            if hazard_mask is not None and costmap_obj is not None:
                l_pin, r_pin = self.evaluate_pinned_state(hazard_mask, costmap_obj)
                is_left_pinned = l_pin if is_left_pinned is None else is_left_pinned
                is_right_pinned = r_pin if is_right_pinned is None else is_right_pinned
            else:
                if is_left_pinned is None:
                    is_left_pinned = (min_l_tof <= 0.035)
                if is_right_pinned is None:
                    is_right_pinned = (min_r_tof <= 0.035)

        # 3. If both are pinned: reverse straight back (0.0 rad) and re-evaluate on next frame
        if is_left_pinned and is_right_pinned:
            return 0.0

        # 4. Check pinned in terms of the intended steer lock:
        # Reversing with steer RIGHT (-1.0) swings nose LEFT -> only pinned if LEFT is occupied
        if intended_steer_dir < 0.0:
            if is_left_pinned:
                return +self.steer_angle  # Pinned on left: override to steer LEFT (+delta_max)
            return -self.steer_angle      # Unpinned: execute full RIGHT lock towards breadcrumb

        # Reversing with steer LEFT (+1.0) swings nose RIGHT -> only pinned if RIGHT is occupied
        else:
            if is_right_pinned:
                return -self.steer_angle  # Pinned on right: override to steer RIGHT (-delta_max)
            return +self.steer_angle      # Unpinned: execute full LEFT lock towards breadcrumb

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
        **kwargs,
    ):
        """Immediately engages streamlined reverse recovery maneuver. Reverse ALWAYS takes priority."""
        self.state = StuckRecoveryState.PIVOT_REVERSE
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0

        # Evaluate pinned state if costmap or tofs provided and not passed
        if is_left_pinned is None or is_right_pinned is None:
            l_pin, r_pin = self.evaluate_pinned_state(hazard_mask, costmap_obj, cached_tofs)
            is_left_pinned = l_pin if is_left_pinned is None else is_left_pinned
            is_right_pinned = r_pin if is_right_pinned is None else is_right_pinned

        if steer_dir is not None:
            self.preferred_yaw_sign = 1.0 if steer_dir > 0 else -1.0
        elif is_left_pinned and not is_right_pinned:
            self.preferred_yaw_sign = 1.0  # Steer Left to swing nose away from left wall
        elif is_right_pinned and not is_left_pinned:
            self.preferred_yaw_sign = -1.0 # Steer Right to swing nose away from right wall
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

        if instant_reverse:
            self.current_cmd_v = -self.reverse_speed
            self.current_cmd_steer = target_steer
        else:
            self.current_cmd_v = float(current_v)
            self.current_cmd_steer = float(current_steer)

    def trigger_corrective_180(
        self,
        target_yaw: Optional[float] = None,
        steer_sign: Optional[float] = None,
        current_yaw: float = 0.0,
        arc_clearance: Optional[Tuple[float, float, float]] = None,
        cached_tofs: Optional[Any] = None,
        front_clearance_m: float = 1.0,
    ):
        """
        Immediately engages Anti-Wrong-Way corrective turn.
        Rotates the vehicle ~135 to 180 degrees back toward the valid forward corridor path.
        Chooses turning direction (+1.0 Left or -1.0 Right) towards the more open side.
        If front clearance is tight or an obstacle is encountered, reverse logic supports the turn.
        """
        # Reverse ALWAYS takes absolute priority over corrective turn if independently triggered
        if self.state == StuckRecoveryState.PIVOT_REVERSE:
            return

        self.state = StuckRecoveryState.CORRECTIVE_180
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        self.target_corrective_yaw = target_yaw
        self.last_corrective_yaw = current_yaw
        self.corrective_accumulated_turn = 0.0
        self.corrective_reverse_timer = 0.0
        self.corrective_reverse_stroke_count = 0

        # Resolve open side if not explicitly specified
        if steer_sign is None:
            cl = arc_clearance if arc_clearance is not None else (0.5, 0.5, 0.5)
            y_diff = wrap_to_pi(target_yaw - current_yaw) if target_yaw is not None else 0.0
            steer_sign = self.resolve_open_side(cl, cached_tofs=cached_tofs, heading_err=y_diff)

        self.preferred_yaw_sign = 1.0 if steer_sign >= 0.0 else -1.0
        self.corrective_steer_dir = self.preferred_yaw_sign

        # If front has an obstacle (< 0.22m), start in reverse supporting phase
        if front_clearance_m < 0.22:
            self.corrective_substate = "REVERSE"
            self.current_cmd_v = -self.reverse_speed
            self.current_cmd_steer = self.compute_reverse_steer(
                current_yaw=current_yaw,
                target_yaw=self.target_corrective_yaw,
                current_v=-self.reverse_speed,
                preferred_sign=self.corrective_steer_dir,
            )
        else:
            self.corrective_substate = "FORWARD"
            self.current_cmd_v = self.forward_speed
            self.current_cmd_steer = self.corrective_steer_dir * self.steer_angle

    def engage_corrective_reverse(self):
        """Engages reverse support phase during CORRECTIVE_180 when an obstacle is encountered."""
        if self.state == StuckRecoveryState.CORRECTIVE_180 and self.corrective_substate != "REVERSE":
            if self.forward_grace_timer <= 0.0:
                self.corrective_substate = "REVERSE"
                self.corrective_reverse_timer = 0.0
                self.corrective_reverse_stroke_count += 1

    def update(
        self,
        dt: float,
        current_v: float,
        commanded_v: float,
        heading_err: float = 0.0,
        active_steer: float = 0.0,
        hazard_mask: Optional[np.ndarray] = None,
        costmap_obj: Optional[Any] = None,
        cached_tofs: Optional[Dict[str, float]] = None,
        current_yaw: float = 0.0,
        target_breadcrumb_yaw: Optional[float] = None,
        **kwargs,
    ) -> Tuple[bool, float, float]:
        """
        Updates recovery state machine.
        Returns:
            (is_recovering, target_v, target_steer)
        """
        # 1. Direct physical sightline check on front 4.1° ToF sensors & per-side minimums
        d_straight_tof = TOF_MAX_RANGE_M
        min_l_tof = TOF_MAX_RANGE_M
        min_r_tof = TOF_MAX_RANGE_M
        max_any_tof = 0.0
        if cached_tofs:
            if isinstance(cached_tofs, dict):
                d_l_41 = cached_tofs.get("tof_front_left_4.1deg", TOF_MAX_RANGE_M)
                d_r_41 = cached_tofs.get("tof_front_right_4.1deg", TOF_MAX_RANGE_M)
                d_straight_tof = min(float(d_l_41), float(d_r_41))
                l_vals = [float(v) for k, v in cached_tofs.items() if "left" in k]
                r_vals = [float(v) for k, v in cached_tofs.items() if "right" in k]
                if l_vals:
                    min_l_tof = min(l_vals)
                if r_vals:
                    min_r_tof = min(r_vals)
                all_vals = [float(v) for v in cached_tofs.values()]
                if all_vals:
                    max_any_tof = max(all_vals)
            elif isinstance(cached_tofs, (list, tuple, np.ndarray)):
                d_straight_tof = min(cached_tofs) if len(cached_tofs) > 0 else TOF_MAX_RANGE_M
                half = len(cached_tofs) // 2
                if half > 0:
                    min_l_tof = float(min(cached_tofs[:half]))
                    min_r_tof = float(min(cached_tofs[half:]))
                if len(cached_tofs) > 0:
                    max_any_tof = float(max(cached_tofs))

        # 2. Evaluate Pinned State directly against the robot via Costmap Grid Cells
        # and front corner swing calculation at max steering lock
        is_l_pinned, is_r_pinned = self.evaluate_pinned_state(
            hazard_mask=hazard_mask,
            costmap_obj=costmap_obj,
            cached_tofs=cached_tofs,
        )

        # 3. Project Max Steering Lock Escape Arcs onto Costmap Hazard Mask
        d_esc_l, d_esc_r, d_esc_c = self.evaluate_costmap_escape_arcs(hazard_mask, costmap_obj)
        arc_clearance = (d_esc_l, d_esc_r, d_esc_c)
        max_forward_escape = max(d_esc_l, d_esc_r, d_esc_c)

        # 4. Check if front is blocked (< 15cm) and neither side arc is open
        front_blocked = (d_straight_tof < 0.15)
        side_free_thresh = max(0.08, self.arc_lookahead_m - 0.01)
        side_curvature_free = (d_esc_l >= side_free_thresh or d_esc_r >= side_free_thresh)
        is_front_imminent_obstacle = front_blocked and (not side_curvature_free)

        # STATE 0: NORMAL DRIVE (Shield handles recovery initiation)
        if self.state == StuckRecoveryState.NORMAL_DRIVE:
            return False, 0.0, 0.0

        # ACTIVE RECOVERY TIMERS
        self.stroke_timer += dt
        self.total_recovery_timer += dt
        if self.forward_grace_timer > 0.0:
            self.forward_grace_timer -= dt

        # Global failsafe timeout
        if self.state != StuckRecoveryState.CORRECTIVE_180:
            if self.total_recovery_timer >= RECOVERY_TOTAL_TIMEOUT_S and d_straight_tof >= self.clearance_exit_m:
                self.state = StuckRecoveryState.NORMAL_DRIVE
                self.stroke_timer = 0.0
                self.total_recovery_timer = 0.0
                return False, 0.0, 0.0

        # PHASE 1: PIVOT REVERSE ARC
        if self.state == StuckRecoveryState.PIVOT_REVERSE:
            # Stop reversing when front ToF clearance opens up (>= clearance_exit_m and not front_blocked)
            # AND vehicle is unpinned from walls (not is_l_pinned and not is_r_pinned)
            # AND at least one other ToF sensor opens up (>= any_tof_exit_m, default 0.60m)
            is_unpinned = (not is_l_pinned and not is_r_pinned)
            front_cleared = (d_straight_tof >= self.clearance_exit_m and not front_blocked and is_unpinned)
            any_tof_cleared = (max_any_tof >= self.any_tof_exit_m) or (
                cached_tofs is not None and len(cached_tofs) <= 2 and max_any_tof >= self.clearance_exit_m
            )
            has_sufficient_clearance = (
                self.stroke_timer >= RECOVERY_EXIT_MIN_TIME_S
                and front_cleared
                and any_tof_cleared
            )
            stroke_timeout = (self.stroke_timer >= RECOVERY_REVERSE_STROKE_MAX_S)

            if has_sufficient_clearance or stroke_timeout:
                self.state = StuckRecoveryState.DECEL_AND_CENTER
                self.stroke_timer = 0.0
                self.current_cmd_v = 0.0
                self.current_cmd_steer = 0.0
                return True, 0.0, 0.0

            target_v = -self.reverse_speed
            target_steer = self.compute_reverse_steer(
                min_l_tof=min_l_tof,
                min_r_tof=min_r_tof,
                current_yaw=current_yaw,
                target_yaw=target_breadcrumb_yaw,
                current_v=current_v,
                preferred_sign=self.preferred_yaw_sign,
                is_left_pinned=is_l_pinned,
                is_right_pinned=is_r_pinned,
                hazard_mask=hazard_mask,
                costmap_obj=costmap_obj,
            )

            self.current_cmd_v = target_v
            self.current_cmd_steer = target_steer

            return True, self.current_cmd_v, self.current_cmd_steer

        # PHASE 2: CORRECTIVE 180 AUTO-CORRECTION ARC (Anti-Wrong-Way)
        elif self.state == StuckRecoveryState.CORRECTIVE_180:
            d_yaw = abs(wrap_to_pi(current_yaw - self.last_corrective_yaw))
            self.corrective_accumulated_turn += d_yaw
            self.last_corrective_yaw = current_yaw

            # Check alignment with target corrective yaw
            yaw_aligned = False
            if self.target_corrective_yaw is not None:
                yaw_diff = abs(wrap_to_pi(self.target_corrective_yaw - current_yaw))
                if yaw_diff <= math.radians(30.0):
                    yaw_aligned = True

            has_turned_135 = (self.corrective_accumulated_turn >= math.radians(135.0))
            is_turn_sufficient = has_turned_135 or (yaw_aligned and self.corrective_accumulated_turn >= math.radians(75.0))

            # Global failsafe timeout for corrective 180 (3.0s with clear front, or 4.5s unconditional)
            if (self.total_recovery_timer >= 3.0 and d_straight_tof >= 0.20) or self.total_recovery_timer >= 4.5:
                self.state = StuckRecoveryState.NORMAL_DRIVE
                self.stroke_timer = 0.0
                self.total_recovery_timer = 0.0
                self.current_cmd_v = 0.0
                self.current_cmd_steer = 0.0
                return False, 0.0, 0.0

            # --- SUBSTATE A: FORWARD 180 SPIN ARC ---
            if self.corrective_substate == "FORWARD":
                # Exit condition: turned sufficiently, front ToF open (>= clearance_exit_m) and not blocked
                front_cleared = (
                    d_straight_tof >= self.clearance_exit_m
                    and not front_blocked
                    and not is_l_pinned
                    and not is_r_pinned
                )
                any_tof_cleared = (max_any_tof >= self.any_tof_exit_m) or (
                    cached_tofs is not None and len(cached_tofs) <= 2 and max_any_tof >= self.clearance_exit_m
                )
                if is_turn_sufficient and front_cleared and any_tof_cleared and not is_front_imminent_obstacle:
                    self.state = StuckRecoveryState.DECEL_AND_CENTER
                    self.stroke_timer = 0.0
                    self.current_cmd_v = 0.0
                    self.current_cmd_steer = 0.0
                    return True, 0.0, 0.0

                # Obstacle detection during forward turn:
                # Do NOT trigger stall during forward grace period while accelerating from reverse!
                is_stalled = (self.forward_grace_timer <= 0.0 and abs(current_v) < 0.10 and self.stroke_timer > 0.30)
                is_wall_collision = (
                    d_straight_tof < 0.16
                    or is_l_pinned
                    or is_r_pinned
                    or min_l_tof <= 0.035
                    or min_r_tof <= 0.035
                )

                if is_wall_collision or is_stalled:
                    # Obstacle encountered: engage reverse logic to support the 180!
                    self.engage_corrective_reverse()
                    # Fall through to execute reverse command in this cycle

            # --- SUBSTATE B: REVERSE SUPPORT PHASE ---
            if self.corrective_substate == "REVERSE":
                self.corrective_reverse_timer += dt

                target_v = -self.reverse_speed
                target_steer = self.compute_reverse_steer(
                    min_l_tof=min_l_tof,
                    min_r_tof=min_r_tof,
                    current_yaw=current_yaw,
                    target_yaw=self.target_corrective_yaw,
                    current_v=current_v,
                    preferred_sign=self.corrective_steer_dir,
                    is_left_pinned=is_l_pinned,
                    is_right_pinned=is_r_pinned,
                    hazard_mask=hazard_mask,
                    costmap_obj=costmap_obj,
                )

                # Check if reverse has created enough front clearance or reached stroke limit
                front_cleared = (
                    d_straight_tof >= self.clearance_exit_m
                    and not front_blocked
                    and not is_l_pinned
                    and not is_r_pinned
                )
                any_tof_cleared = (max_any_tof >= self.any_tof_exit_m) or (
                    cached_tofs is not None and len(cached_tofs) <= 2 and max_any_tof >= self.clearance_exit_m
                )
                has_min_reverse_time = (self.corrective_reverse_timer >= RECOVERY_EXIT_MIN_TIME_S)
                reverse_timeout = (self.corrective_reverse_timer >= RECOVERY_REVERSE_STROKE_MAX_S)

                # Did reverse pivot itself complete alignment?
                if is_turn_sufficient and front_cleared and any_tof_cleared and not is_front_imminent_obstacle:
                    self.state = StuckRecoveryState.DECEL_AND_CENTER
                    self.stroke_timer = 0.0
                    self.current_cmd_v = 0.0
                    self.current_cmd_steer = 0.0
                    return True, 0.0, 0.0

                if (front_cleared and any_tof_cleared and has_min_reverse_time) or reverse_timeout:
                    # Reverse clearance achieved! Switch back to forward 180 arc
                    self.corrective_substate = "FORWARD"
                    self.stroke_timer = 0.0
                    self.forward_grace_timer = 0.35  # Grace period to accelerate forward from negative velocity

                    # Re-evaluate open side with current arc clearances
                    diff_arc = d_esc_l - d_esc_r
                    if abs(diff_arc) > 0.04:
                        self.corrective_steer_dir = 1.0 if diff_arc > 0.0 else -1.0
                        self.preferred_yaw_sign = self.corrective_steer_dir

                    target_v = self.forward_speed
                    target_steer = self.corrective_steer_dir * self.steer_angle
                    self.current_cmd_v = target_v
                    self.current_cmd_steer = target_steer
                    return True, self.current_cmd_v, self.current_cmd_steer

                self.current_cmd_v = target_v
                self.current_cmd_steer = target_steer
                return True, self.current_cmd_v, self.current_cmd_steer

            # Default forward execution in Substate A:
            target_v = self.forward_speed
            target_steer = self.corrective_steer_dir * self.steer_angle
            self.current_cmd_v = target_v
            self.current_cmd_steer = target_steer
            return True, self.current_cmd_v, self.current_cmd_steer

        # PHASE 3: DECELERATE SPEED TO 0 WHILE CENTERING SERVO
        elif self.state == StuckRecoveryState.DECEL_AND_CENTER:
            self.stroke_timer += dt
            self.total_recovery_timer += dt
            self.current_cmd_v = 0.0

            # Slew commanded steering toward 0.0 radians at physical servo slew rate
            dsteer_step = self.servo_slew_rad_s * dt
            if abs(self.current_cmd_steer) <= dsteer_step:
                self.current_cmd_steer = 0.0
            else:
                self.current_cmd_steer -= math.copysign(dsteer_step, self.current_cmd_steer)

            # Wait until steering physically and commanded gets to 0.0
            is_centered = (abs(active_steer) <= 0.04) and (abs(self.current_cmd_steer) <= 0.02)
            phase_timeout = (self.stroke_timer >= 0.25)

            if is_centered or phase_timeout:
                self.state = StuckRecoveryState.NORMAL_DRIVE
                self.stroke_timer = 0.0
                self.total_recovery_timer = 0.0
                self.current_cmd_v = 0.0
                self.current_cmd_steer = 0.0
                return False, 0.0, 0.0

            return True, self.current_cmd_v, self.current_cmd_steer

        return False, 0.0, 0.0
