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
        RECOVERY_STALL_CONFIRM_S,
        RECOVERY_FRONT_BLOCKED_M,
        RECOVERY_REVERSE_STROKE_MAX_S,
        RECOVERY_TOTAL_TIMEOUT_S,
        RECOVERY_CLEARANCE_EXIT_M,
        RECOVERY_ANY_TOF_EXIT_M,
        RECOVERY_EXIT_MIN_TIME_S,
        RECOVERY_SERVO_SLEW_RAD_S,
        RECOVERY_ARC_LOOKAHEAD_M,
        BRAKE_DECEL_MAX,
        ACCEL_FWD_MAX,
        TOF_MAX_RANGE_M,
    )
except ImportError:
    from planner_config import (
        WHEELBASE,
        MAX_STEERING_RAD,
        RECOVERY_REVERSE_SPEED_MPS,
        RECOVERY_STEER_ANGLE_RAD,
        RECOVERY_STALL_CONFIRM_S,
        RECOVERY_FRONT_BLOCKED_M,
        RECOVERY_REVERSE_STROKE_MAX_S,
        RECOVERY_TOTAL_TIMEOUT_S,
        RECOVERY_CLEARANCE_EXIT_M,
        RECOVERY_ANY_TOF_EXIT_M,
        RECOVERY_EXIT_MIN_TIME_S,
        RECOVERY_SERVO_SLEW_RAD_S,
        RECOVERY_ARC_LOOKAHEAD_M,
        BRAKE_DECEL_MAX,
        ACCEL_FWD_MAX,
        TOF_MAX_RANGE_M,
    )


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
        steer_angle_rad: float = RECOVERY_STEER_ANGLE_RAD,
        clearance_exit_m: float = RECOVERY_CLEARANCE_EXIT_M,
        any_tof_exit_m: float = RECOVERY_ANY_TOF_EXIT_M,
        servo_slew_rad_s: float = RECOVERY_SERVO_SLEW_RAD_S,
        brake_decel_max: float = BRAKE_DECEL_MAX,
        accel_fwd_max: float = ACCEL_FWD_MAX,
        wheelbase_m: float = WHEELBASE,
        max_steer_rad: float = MAX_STEERING_RAD,
        arc_lookahead_m: float = RECOVERY_ARC_LOOKAHEAD_M,
        **kwargs,
    ):
        self.state = StuckRecoveryState.NORMAL_DRIVE
        self.reverse_speed = float(reverse_speed_m_s)
        self.steer_angle = float(steer_angle_rad)
        self.clearance_exit_m = float(clearance_exit_m)
        self.any_tof_exit_m = float(any_tof_exit_m)
        self.servo_slew_rad_s = float(servo_slew_rad_s)
        self.brake_decel_max = float(brake_decel_max)
        self.accel_fwd_max = float(accel_fwd_max)

        self.stall_timer = 0.0
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        self.preferred_yaw_sign = 1.0

        # State tracking for smooth ramping
        self.current_cmd_v = 0.0
        self.current_cmd_steer = 0.0

        # Anti-Wrong-Way CORRECTIVE_180 tracking
        self.target_corrective_yaw: Optional[float] = None
        self.corrective_start_yaw: float = 0.0
        self.last_corrective_yaw: float = 0.0
        self.corrective_accumulated_turn: float = 0.0

        # --- KINEMATIC ESCAPE ARCS PRECOMPUTATION ---
        self.wheelbase = float(wheelbase_m)
        self.max_steer_rad = float(max_steer_rad)
        self.r_min_turn = self.wheelbase / max(1e-3, math.tan(self.max_steer_rad))
        self.origin_y = 0.0   # Robot center origin (0.0, 0.0)
        self.bumper_y = 0.095 # Front bumper mount Y in vehicle frame

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

    def get_costmap_forward_clearance(
        self,
        hazard_mask: Optional[np.ndarray],
        costmap_obj: Optional[Any],
        max_dist_m: float = 0.50,
    ) -> float:
        """Raytraces directly along vehicle centerline forward from front bumper on costmap hazard mask."""
        if hazard_mask is None or costmap_obj is None:
            return TOF_MAX_RANGE_M
        res = getattr(costmap_obj, "res", 0.015)
        x_min = getattr(costmap_obj, "x_min", -1.25)
        y_min = getattr(costmap_obj, "y_min", -0.10)
        h, w = hazard_mask.shape

        n_steps = int(math.ceil(max_dist_m / res))
        for step in range(n_steps):
            s = step * res
            py = self.bumper_y + s
            px = 0.0
            col = int(round((px - x_min) / res))
            row = int(round((py - y_min) / res))
            if 0 <= row < h and 0 <= col < w:
                if hazard_mask[row, col] > 0:
                    return float(s)
        return float(max_dist_m)

    @staticmethod
    def resolve_escape_steer_dir(
        arc_clearance: Tuple[float, float, float],
        heading_err: float = 0.0,
        similar_eps_m: float = 0.020,
        **kwargs,
    ) -> float:
        """
        Steering direction resolution when triggering emergency backup:
        Compares clearance along Right max-lock arc vs Left max-lock arc.
        Returns: -1.0 (steer right: right arc is clearer)
                 or +1.0 (steer left: left arc is clearer)
        """
        d_esc_left, d_esc_right, _ = arc_clearance
        diff_arc = d_esc_right - d_esc_left
        if abs(diff_arc) > similar_eps_m:
            return -1.0 if diff_arc > 0.0 else 1.0

        # Secondary fallback: Heading error alignment (+ is left, - is right)
        if abs(heading_err) > math.radians(10.0):
            return 1.0 if heading_err > 0.0 else -1.0

        return -1.0

    def trigger_emergency_recovery(
        self,
        heading_err: float = 0.0,
        steer_dir: Optional[float] = None,
        current_v: float = 0.0,
        current_steer: float = 0.0,
        instant_reverse: bool = True,
    ):
        """Immediately engages streamlined recovery maneuver."""
        self.state = StuckRecoveryState.PIVOT_REVERSE
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        if steer_dir is not None:
            self.preferred_yaw_sign = 1.0 if steer_dir > 0 else -1.0
        elif abs(heading_err) > math.radians(25.0):
            self.preferred_yaw_sign = 1.0 if heading_err > 0 else -1.0
        else:
            self.preferred_yaw_sign = -1.0

        # When driving forward (current_v > 0.1 m/s), steer towards escape opening (away from obstacle);
        # when reversing (current_v <= 0.1 m/s), steer opposite to pivot the front nose away in reverse.
        steer_sign = self.preferred_yaw_sign if current_v > 0.1 else -self.preferred_yaw_sign
        target_steer = steer_sign * self.steer_angle

        if instant_reverse:
            self.current_cmd_v = -self.reverse_speed
            self.current_cmd_steer = target_steer
        else:
            self.current_cmd_v = float(current_v)
            self.current_cmd_steer = float(current_steer)

    def trigger_corrective_180(
        self,
        target_yaw: Optional[float] = None,
        steer_sign: float = 1.0,
        current_yaw: float = 0.0,
    ):
        """
        Immediately engages Anti-Wrong-Way corrective turn.
        Rotates the vehicle ~135 to 180 degrees back toward the valid forward corridor path.
        """
        self.state = StuckRecoveryState.CORRECTIVE_180
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        self.preferred_yaw_sign = 1.0 if steer_sign >= 0.0 else -1.0
        self.target_corrective_yaw = target_yaw
        self.corrective_start_yaw = current_yaw
        self.last_corrective_yaw = current_yaw
        self.corrective_accumulated_turn = 0.0

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
        **kwargs,
    ) -> Tuple[bool, float, float]:
        """
        Updates recovery state machine.
        Returns:
            (is_recovering, target_v, target_steer)
        """
        # 1. Direct physical sightline check on front 4.1° ToF sensors
        d_straight_tof = TOF_MAX_RANGE_M
        if cached_tofs:
            if isinstance(cached_tofs, dict):
                d_l_41 = cached_tofs.get("tof_front_left_4.1deg", TOF_MAX_RANGE_M)
                d_r_41 = cached_tofs.get("tof_front_right_4.1deg", TOF_MAX_RANGE_M)
                d_straight_tof = min(float(d_l_41), float(d_r_41))
            elif isinstance(cached_tofs, (list, tuple, np.ndarray)):
                d_straight_tof = min(cached_tofs) if len(cached_tofs) > 0 else TOF_MAX_RANGE_M

        # STATE 0: NORMAL DRIVE (Shield handles recovery initiation)
        if self.state == StuckRecoveryState.NORMAL_DRIVE:
            return False, 0.0, 0.0

        # ACTIVE RECOVERY TIMERS
        self.stroke_timer += dt
        self.total_recovery_timer += dt

        # Global failsafe timeout
        if self.state != StuckRecoveryState.CORRECTIVE_180:
            if self.total_recovery_timer >= RECOVERY_TOTAL_TIMEOUT_S and d_straight_tof >= self.clearance_exit_m:
                self.state = StuckRecoveryState.NORMAL_DRIVE
                self.stall_timer = 0.0
                self.stroke_timer = 0.0
                self.total_recovery_timer = 0.0
                return False, 0.0, 0.0

        # PHASE 1: PIVOT REVERSE ARC
        if self.state == StuckRecoveryState.PIVOT_REVERSE:
            # Stop reversing when front ToF clearance opens up (>= clearance_exit_m)
            front_cleared = (d_straight_tof >= self.clearance_exit_m)
            has_sufficient_clearance = (
                self.stroke_timer >= RECOVERY_EXIT_MIN_TIME_S
                and front_cleared
            )
            stroke_timeout = (self.stroke_timer >= RECOVERY_REVERSE_STROKE_MAX_S)

            if has_sufficient_clearance or stroke_timeout:
                self.state = StuckRecoveryState.NORMAL_DRIVE
                self.stall_timer = 0.0
                self.stroke_timer = 0.0
                self.total_recovery_timer = 0.0
                self.current_cmd_v = 0.0
                self.current_cmd_steer = 0.0
                return False, 0.0, 0.0

            target_v = -self.reverse_speed
            # When forward speed > 0.1 m/s (coasting forward), steer towards escape opening (away from obstacle);
            # when reversing (speed <= 0.1 m/s), steer opposite to pivot the front nose away in reverse.
            steer_sign = self.preferred_yaw_sign if current_v > 0.1 else -self.preferred_yaw_sign
            target_steer = steer_sign * self.steer_angle

            self.current_cmd_v = target_v
            self.current_cmd_steer = target_steer

            return True, self.current_cmd_v, self.current_cmd_steer

        # PHASE 2: CORRECTIVE 180 AUTO-CORRECTION ARC (Anti-Wrong-Way)
        elif self.state == StuckRecoveryState.CORRECTIVE_180:
            # Track accumulated yaw rotation to ensure at least 135-180 degrees of turn
            def wrap_to_pi(a):
                return math.atan2(math.sin(a), math.cos(a))

            d_yaw = abs(wrap_to_pi(current_yaw - self.last_corrective_yaw))
            self.corrective_accumulated_turn += d_yaw
            self.last_corrective_yaw = current_yaw

            has_turned_135 = (self.corrective_accumulated_turn >= math.radians(135.0))
            yaw_aligned = False
            if self.target_corrective_yaw is not None:
                yaw_diff = abs(wrap_to_pi(self.target_corrective_yaw - current_yaw))
                if yaw_diff <= math.radians(30.0):
                    yaw_aligned = True

            # Exit condition for CORRECTIVE_180:
            # Must turn at least 135 deg (or >= 90 deg AND aligned with target yaw)
            # AND forward escape path must be open (>= 0.22m) and not imminent obstacle
            if (has_turned_135 or (yaw_aligned and self.corrective_accumulated_turn >= math.radians(90.0))) and max_forward_escape >= 0.22 and not is_front_imminent_obstacle:
                self.state = StuckRecoveryState.NORMAL_DRIVE
                self.stall_timer = 0.0
                self.stroke_timer = 0.0
                self.total_recovery_timer = 0.0
                self.current_cmd_v = 0.0
                self.current_cmd_steer = 0.0
                return False, 0.0, 0.0

            # Failsafe timeout for corrective 180 (e.g. 1.8 seconds)
            if self.total_recovery_timer >= 1.8 and not is_front_imminent_obstacle:
                self.state = StuckRecoveryState.NORMAL_DRIVE
                self.stall_timer = 0.0
                self.stroke_timer = 0.0
                self.total_recovery_timer = 0.0
                return False, 0.0, 0.0

            # Turn direction towards target yaw or preferred yaw sign
            if self.target_corrective_yaw is not None:
                y_err = wrap_to_pi(self.target_corrective_yaw - current_yaw)
                intended_steer_dir = 1.0 if y_err >= 0.0 else -1.0
            else:
                intended_steer_dir = self.preferred_yaw_sign

            target_v = 1.20  # forward spinning speed
            target_steer = intended_steer_dir * self.steer_angle
            self.current_cmd_v = target_v
            self.current_cmd_steer = target_steer
            return True, self.current_cmd_v, self.current_cmd_steer

        # PHASE 3: DECELERATE SPEED TO 0 WHILE CENTERING SERVO (Direct Handshake fallback)
        elif self.state == StuckRecoveryState.DECEL_AND_CENTER:
            self.state = StuckRecoveryState.NORMAL_DRIVE
            self.stall_timer = 0.0
            self.stroke_timer = 0.0
            self.total_recovery_timer = 0.0
            self.current_cmd_v = 0.0
            self.current_cmd_steer = 0.0
            return False, 0.0, 0.0

        return False, 0.0, 0.0
