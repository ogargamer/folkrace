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
        RECOVERY_EXIT_MIN_TIME_S,
        RECOVERY_SERVO_SLEW_RAD_S,
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
        RECOVERY_EXIT_MIN_TIME_S,
        RECOVERY_SERVO_SLEW_RAD_S,
        BRAKE_DECEL_MAX,
        ACCEL_FWD_MAX,
        TOF_MAX_RANGE_M,
    )


class StuckRecoveryState:
    NORMAL_DRIVE = "DRIVE"
    PIVOT_REVERSE = "PIVOT_REVERSE"
    DECEL_AND_CENTER = "DECEL_AND_CENTER"


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
        servo_slew_rad_s: float = RECOVERY_SERVO_SLEW_RAD_S,
        brake_decel_max: float = BRAKE_DECEL_MAX,
        accel_fwd_max: float = ACCEL_FWD_MAX,
        wheelbase_m: float = WHEELBASE,
        max_steer_rad: float = MAX_STEERING_RAD,
        arc_lookahead_m: float = 0.20,
        **kwargs,
    ):
        self.state = StuckRecoveryState.NORMAL_DRIVE
        self.reverse_speed = float(reverse_speed_m_s)
        self.steer_angle = float(steer_angle_rad)
        self.clearance_exit_m = float(clearance_exit_m)
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

        # --- KINEMATIC ESCAPE ARCS PRECOMPUTATION ---
        self.wheelbase = float(wheelbase_m)
        self.max_steer_rad = float(max_steer_rad)
        self.r_min_turn = self.wheelbase / max(1e-3, math.tan(self.max_steer_rad))
        self.bumper_y = 0.095  # front bumper mount Y in vehicle frame

        self.arc_lookahead_m = float(arc_lookahead_m)
        self.arc_step_ds = 0.015  # 15mm steps
        n_pts = max(3, int(self.arc_lookahead_m / self.arc_step_ds))
        self._arc_s = np.linspace(self.arc_step_ds, self.arc_lookahead_m, n_pts, dtype=np.float32)

        # Left Turn Arc (full left lock: sweeps left -X)
        self._arc_pts_left = np.zeros((n_pts, 2), dtype=np.float32)
        self._arc_pts_left[:, 0] = -self.r_min_turn * (1.0 - np.cos(self._arc_s / self.r_min_turn))
        self._arc_pts_left[:, 1] = self.bumper_y + self.r_min_turn * np.sin(self._arc_s / self.r_min_turn)

        # Right Turn Arc (full right lock: sweeps right +X)
        self._arc_pts_right = np.zeros((n_pts, 2), dtype=np.float32)
        self._arc_pts_right[:, 0] = self.r_min_turn * (1.0 - np.cos(self._arc_s / self.r_min_turn))
        self._arc_pts_right[:, 1] = self.bumper_y + self.r_min_turn * np.sin(self._arc_s / self.r_min_turn)

        # Center Straight Sightline
        self._arc_pts_center = np.zeros((n_pts, 2), dtype=np.float32)
        self._arc_pts_center[:, 0] = 0.0
        self._arc_pts_center[:, 1] = self.bumper_y + self._arc_s

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
        y_min = getattr(costmap_obj, "y_min", 0.0)
        h, w = hazard_mask.shape

        def _trace(pts: np.ndarray) -> float:
            for i in range(len(pts)):
                px = pts[i, 0]
                py = pts[i, 1]
                col = int(round((px - x_min) / res))
                row = int(round((py - y_min) / res))
                if row < 0 or row >= h or col < 0 or col >= w:
                    return float(self._arc_s[max(0, i - 1)])
                if hazard_mask[row, col] > 0:
                    return float(self._arc_s[max(0, i - 1)])
            return float(self._arc_s[-1])

        d_left = _trace(self._arc_pts_left)
        d_right = _trace(self._arc_pts_right)
        d_center = _trace(self._arc_pts_center)
        return (d_left, d_right, d_center)

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
        Returns: +1.0 (steer right so front turns right / away from left)
                 or -1.0 (steer left so front turns left / away from right)
        """
        d_esc_left, d_esc_right, _ = arc_clearance
        diff_arc = d_esc_right - d_esc_left
        if abs(diff_arc) > similar_eps_m:
            return 1.0 if diff_arc > 0.0 else -1.0

        # Secondary fallback: Heading error alignment
        if abs(heading_err) > math.radians(10.0):
            return 1.0 if heading_err > 0.0 else -1.0

        return 1.0

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
            self.preferred_yaw_sign = 1.0

        target_steer = -self.preferred_yaw_sign * self.steer_angle
        if instant_reverse:
            self.current_cmd_v = -self.reverse_speed
            self.current_cmd_steer = target_steer
        else:
            self.current_cmd_v = float(current_v)
            self.current_cmd_steer = float(current_steer)

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
        **kwargs,
    ) -> Tuple[bool, float, float]:
        """
        Updates recovery state machine using costmap max-lock escape arc evaluations.
        Returns:
            (is_recovering, target_v, target_steer)
        """
        # Project Max Steering Lock Escape Arcs onto Costmap Hazard Mask
        d_esc_l, d_esc_r, d_esc_c = self.evaluate_costmap_escape_arcs(hazard_mask, costmap_obj)
        arc_clearance = (d_esc_l, d_esc_r, d_esc_c)

        # 1. Direct physical sightline check: minimum of front 4.1 deg ToFs (straight ahead)
        d_straight_tof = TOF_MAX_RANGE_M
        if cached_tofs:
            d_l_41 = cached_tofs.get("tof_front_left_4.1deg", TOF_MAX_RANGE_M)
            d_r_41 = cached_tofs.get("tof_front_right_4.1deg", TOF_MAX_RANGE_M)
            d_straight_tof = min(float(d_l_41), float(d_r_41))

        # 2. Check if front ToFs are blocked (< 15cm):
        # If front is blocked, check if either side escape curvature is free (>= 15cm).
        # Only trigger emergency reverse if BOTH front is blocked AND neither side curvature has a way out.
        front_blocked = (d_straight_tof < 0.15)
        side_curvature_free = (d_esc_l >= 0.15 or d_esc_r >= 0.15)
        is_front_imminent_obstacle = front_blocked and (not side_curvature_free)

        # 3. Spinout / Facing Backwards check (110 degrees)
        is_facing_backwards = abs(heading_err) > math.radians(110.0)

        max_forward_escape = max(d_esc_l, d_esc_r, d_esc_c)
        is_stalled_forward = (commanded_v > 0.30 and abs(current_v) < 0.10 and max_forward_escape < RECOVERY_FRONT_BLOCKED_M)

        # Immediate override for imminent obstacle from any state
        if is_front_imminent_obstacle and self.state != StuckRecoveryState.PIVOT_REVERSE:
            side_steer = self.resolve_escape_steer_dir(arc_clearance=arc_clearance, heading_err=heading_err)
            self.trigger_emergency_recovery(heading_err=heading_err, steer_dir=side_steer, current_v=current_v, current_steer=active_steer, instant_reverse=True)
            return True, self.current_cmd_v, self.current_cmd_steer

        # STATE 0: NORMAL DRIVE
        if self.state == StuckRecoveryState.NORMAL_DRIVE:
            if is_front_imminent_obstacle:
                side_steer = self.resolve_escape_steer_dir(arc_clearance=arc_clearance, heading_err=heading_err)
                self.trigger_emergency_recovery(heading_err=heading_err, steer_dir=side_steer, current_v=current_v, current_steer=active_steer, instant_reverse=True)
                return True, self.current_cmd_v, self.current_cmd_steer
            elif is_facing_backwards:
                self.trigger_emergency_recovery(heading_err=heading_err, current_v=current_v, current_steer=active_steer, instant_reverse=True)
                return True, self.current_cmd_v, self.current_cmd_steer
            elif is_stalled_forward:
                self.stall_timer += dt
                if self.stall_timer > RECOVERY_STALL_CONFIRM_S:
                    side_steer = self.resolve_escape_steer_dir(arc_clearance=arc_clearance, heading_err=heading_err)
                    self.trigger_emergency_recovery(heading_err=heading_err, steer_dir=side_steer, current_v=current_v, current_steer=active_steer, instant_reverse=True)
                    return True, self.current_cmd_v, self.current_cmd_steer
            else:
                self.stall_timer = max(0.0, self.stall_timer - dt * 2.0)
            return False, 0.0, 0.0

        # ACTIVE RECOVERY TIMERS
        self.stroke_timer += dt
        self.total_recovery_timer += dt

        # Global failsafe timeout: if recovery is stuck longer than timeout, force exit to drive
        if self.total_recovery_timer >= RECOVERY_TOTAL_TIMEOUT_S and not is_front_imminent_obstacle and max_forward_escape > 0.12:
            self.state = StuckRecoveryState.NORMAL_DRIVE
            self.stall_timer = 0.0
            self.stroke_timer = 0.0
            self.total_recovery_timer = 0.0
            return False, 0.0, 0.0

        # PHASE 1: PIVOT REVERSE ARC
        if self.state == StuckRecoveryState.PIVOT_REVERSE:
            target_v = -self.reverse_speed
            target_steer = -self.preferred_yaw_sign * self.steer_angle

            # Instantly apply full reverse speed and escape steering angle
            self.current_cmd_v = target_v
            self.current_cmd_steer = target_steer

            # Check stopping conditions for reverse backing:
            forward_cleared = (max_forward_escape >= self.clearance_exit_m)

            has_sufficient_clearance = (
                self.stroke_timer >= RECOVERY_EXIT_MIN_TIME_S
                and forward_cleared
                and not is_front_imminent_obstacle
            )
            is_stalled_reverse = (self.stroke_timer > 0.45 and current_v > -0.05 and not is_front_imminent_obstacle)
            stroke_timeout = (self.stroke_timer >= RECOVERY_REVERSE_STROKE_MAX_S and not is_front_imminent_obstacle)

            if has_sufficient_clearance or is_stalled_reverse or stroke_timeout:
                self.state = StuckRecoveryState.DECEL_AND_CENTER
                self.stroke_timer = 0.0

            return True, self.current_cmd_v, self.current_cmd_steer

        # PHASE 2: DECELERATE SPEED TO 0 WHILE CENTERING SERVO
        elif self.state == StuckRecoveryState.DECEL_AND_CENTER:
            # 1. Slew front steering back toward 0.0 radians
            dsteer_step = self.servo_slew_rad_s * dt
            if abs(self.current_cmd_steer) <= dsteer_step:
                self.current_cmd_steer = 0.0
            else:
                self.current_cmd_steer -= math.copysign(dsteer_step, self.current_cmd_steer)

            # 2. Synchronize deceleration to match remaining servo centering time
            t_servo_remaining = abs(self.current_cmd_steer) / max(0.001, self.servo_slew_rad_s)
            if t_servo_remaining > 0.005:
                decel_needed = abs(self.current_cmd_v) / t_servo_remaining
            else:
                decel_needed = self.brake_decel_max

            # Clamp decel rate so it never exceeds tire friction limit
            decel_rate = min(self.brake_decel_max, max(0.40 * self.brake_decel_max, decel_needed))
            dv_step = decel_rate * dt

            # Ramp speed toward 0.0 m/s
            if self.current_cmd_v < 0.0:
                self.current_cmd_v = min(0.0, self.current_cmd_v + dv_step)
            else:
                self.current_cmd_v = max(0.0, self.current_cmd_v - dv_step)

            # 3. Clean Handshake: When car is near standstill and wheels are centered, hand off to planner!
            is_stopped = abs(self.current_cmd_v) <= 0.08
            is_centered = abs(self.current_cmd_steer) <= 0.03
            phase_timeout = (self.stroke_timer >= 0.28)

            if (is_stopped and is_centered) or phase_timeout:
                self.state = StuckRecoveryState.NORMAL_DRIVE
                self.stall_timer = 0.0
                self.stroke_timer = 0.0
                self.total_recovery_timer = 0.0
                self.current_cmd_v = 0.0
                self.current_cmd_steer = 0.0
                return False, 0.0, 0.0

            return True, self.current_cmd_v, self.current_cmd_steer

        return False, 0.0, 0.0
