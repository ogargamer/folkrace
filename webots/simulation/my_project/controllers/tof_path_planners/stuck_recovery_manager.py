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

try:
    from .planner_config import (
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
    - Reverses away from obstacle along escape curvature until front clearance opens up.
    - Decelerates reverse motion smoothly to 0 m/s while concurrently centering the front servo.
    - Hands control cleanly back to the path planner with 0 steering offset and 0 yaw rate shock.
    """

    def __init__(
        self,
        reverse_speed_m_s: float = RECOVERY_REVERSE_SPEED_MPS,
        steer_angle_rad: float = RECOVERY_STEER_ANGLE_RAD,
        emergency_backup_dist_m: float = 0.09,
        clearance_exit_m: float = RECOVERY_CLEARANCE_EXIT_M,
        servo_slew_rad_s: float = RECOVERY_SERVO_SLEW_RAD_S,
        brake_decel_max: float = BRAKE_DECEL_MAX,
        accel_fwd_max: float = ACCEL_FWD_MAX,
        **kwargs,
    ):
        self.state = StuckRecoveryState.NORMAL_DRIVE
        self.reverse_speed = float(reverse_speed_m_s)
        self.steer_angle = float(steer_angle_rad)
        self.emergency_backup_dist_m = float(emergency_backup_dist_m)
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

    @staticmethod
    def resolve_escape_steer_dir(
        cached_tofs: Any,
        heading_err: float = 0.0,
        similar_eps_m: float = 0.025,
    ) -> float:
        """
        Hierarchical steering direction resolution when triggering emergency backup:
        Returns: +1.0 (steer right so front turns right / away from left)
                 or -1.0 (steer left so front turns left / away from right)
        """
        if isinstance(cached_tofs, dict):
            if "tof_front_left_4.1deg" in cached_tofs:
                # 16-Channel Front-Only Cross-Pointing Rig
                d_fl_4 = float(cached_tofs.get("tof_front_left_4.1deg", 1.80))
                d_fr_4 = float(cached_tofs.get("tof_front_right_4.1deg", 1.80))
                d_fl_15 = float(cached_tofs.get("tof_front_left_15.4deg", 1.80))
                d_fr_15 = float(cached_tofs.get("tof_front_right_15.4deg", 1.80))

                # Left flank (right bumper sensors cross-pointing left):
                d_left_side = min(
                    float(cached_tofs.get("tof_front_right_82.9deg", 1.80)),
                    float(cached_tofs.get("tof_front_right_71.6deg", 1.80)),
                    float(cached_tofs.get("tof_front_right_60.4deg", 1.80)),
                    float(cached_tofs.get("tof_front_right_49.1deg", 1.80)),
                )
                # Right flank (left bumper sensors cross-pointing right):
                d_right_side = min(
                    float(cached_tofs.get("tof_front_left_82.9deg", 1.80)),
                    float(cached_tofs.get("tof_front_left_71.6deg", 1.80)),
                    float(cached_tofs.get("tof_front_left_60.4deg", 1.80)),
                    float(cached_tofs.get("tof_front_left_49.1deg", 1.80)),
                )

                # Tier 1: 4.1 deg center sensors (d_fr_4 - d_fl_4)
                # If d_fr_4 > d_fl_4, obstacle is on the left, right has more space.
                # In forward steering, +steer turns right. In reverse steering, -steer swings the front right!
                # Returning +1.0 indicates escape toward the right.
                diff_1 = d_fr_4 - d_fl_4
                if abs(diff_1) > similar_eps_m:
                    return 1.0 if diff_1 > 0.0 else -1.0

                # Tier 2: 15.4 deg inner sensors
                diff_2 = d_fr_15 - d_fl_15
                if abs(diff_2) > similar_eps_m:
                    return 1.0 if diff_2 > 0.0 else -1.0

                # Tier 3: Lateral flank clearance
                diff_side = d_left_side - d_right_side
                if abs(diff_side) > 0.035:
                    return 1.0 if diff_side < 0.0 else -1.0

                # Tier 4: Heading error fallback
                if abs(heading_err) > math.radians(10.0):
                    return 1.0 if heading_err > 0.0 else -1.0

                return 1.0
            else:
                fl3 = float(cached_tofs.get("tof_fl_3", 1.80))
                fr3 = float(cached_tofs.get("tof_fr_3", 1.80))
                diff_1 = fr3 - fl3
                if abs(diff_1) > similar_eps_m:
                    return 1.0 if diff_1 > 0.0 else -1.0
                if abs(heading_err) > math.radians(10.0):
                    return 1.0 if heading_err > 0.0 else -1.0
                return 1.0
        else:
            tofs = list(cached_tofs)
            fl_c = float(tofs[0]) if len(tofs) > 0 else 1.80
            fr_c = float(tofs[8]) if len(tofs) > 8 else (float(tofs[7]) if len(tofs) > 7 else 1.80)
            diff_1 = fr_c - fl_c
            if abs(diff_1) > similar_eps_m:
                return 1.0 if diff_1 > 0.0 else -1.0
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
        cached_tofs: Dict[str, float],
        heading_err: float = 0.0,
        active_steer: float = 0.0,
    ) -> Tuple[bool, float, float]:
        """
        Updates recovery state machine.
        Returns:
            (is_recovering, target_v, target_steer)
        """
        if isinstance(cached_tofs, dict):
            if "tof_front_left_4.1deg" in cached_tofs:
                front_keys = [
                    "tof_front_left_4.1deg", "tof_front_right_4.1deg",
                    "tof_front_left_15.4deg", "tof_front_right_15.4deg",
                ]
                d_fl3 = cached_tofs.get("tof_front_left_4.1deg", TOF_MAX_RANGE_M)
                d_fr3 = cached_tofs.get("tof_front_right_4.1deg", TOF_MAX_RANGE_M)
            else:
                front_keys = ["tof_fl_3", "tof_fr_3", "tof_fl_2", "tof_fr_2"]
                d_fl3 = cached_tofs.get("tof_fl_3", TOF_MAX_RANGE_M)
                d_fr3 = cached_tofs.get("tof_fr_3", TOF_MAX_RANGE_M)

            min_all_dist = min(cached_tofs.values()) if cached_tofs else TOF_MAX_RANGE_M
            min_front_cone = min([cached_tofs.get(k, TOF_MAX_RANGE_M) for k in front_keys]) if front_keys else TOF_MAX_RANGE_M
        else:
            tofs_arr = list(cached_tofs)
            min_all_dist = min(tofs_arr) if tofs_arr else TOF_MAX_RANGE_M
            d_fl3 = tofs_arr[0] if len(tofs_arr) > 0 else TOF_MAX_RANGE_M
            d_fr3 = tofs_arr[8] if len(tofs_arr) > 8 else TOF_MAX_RANGE_M
            min_front_cone = min([tofs_arr[i] for i in [0, 1, 8, 9] if i < len(tofs_arr)]) if tofs_arr else TOF_MAX_RANGE_M

        is_front_imminent_obstacle = (d_fl3 <= self.emergency_backup_dist_m or d_fr3 <= self.emergency_backup_dist_m)
        is_facing_backwards = abs(heading_err) > math.radians(75.0)
        is_stalled_forward = (commanded_v > 0.30 and abs(current_v) < 0.10 and min_front_cone < RECOVERY_FRONT_BLOCKED_M)

        # Immediate override for 9cm obstacle from any state
        if is_front_imminent_obstacle and self.state != StuckRecoveryState.PIVOT_REVERSE:
            side_steer = self.resolve_escape_steer_dir(cached_tofs, heading_err=heading_err, similar_eps_m=0.025)
            self.trigger_emergency_recovery(heading_err=heading_err, steer_dir=side_steer, current_v=current_v, current_steer=active_steer, instant_reverse=True)
            return True, self.current_cmd_v, self.current_cmd_steer

        # STATE 0: NORMAL DRIVE
        if self.state == StuckRecoveryState.NORMAL_DRIVE:
            if is_front_imminent_obstacle:
                side_steer = self.resolve_escape_steer_dir(cached_tofs, heading_err=heading_err, similar_eps_m=0.025)
                self.trigger_emergency_recovery(heading_err=heading_err, steer_dir=side_steer, current_v=current_v, current_steer=active_steer, instant_reverse=True)
                return True, self.current_cmd_v, self.current_cmd_steer
            elif is_facing_backwards:
                self.trigger_emergency_recovery(heading_err=heading_err, current_v=current_v, current_steer=active_steer, instant_reverse=True)
                return True, self.current_cmd_v, self.current_cmd_steer
            elif is_stalled_forward:
                self.stall_timer += dt
                if self.stall_timer > RECOVERY_STALL_CONFIRM_S:
                    side_steer = self.resolve_escape_steer_dir(cached_tofs, heading_err=heading_err, similar_eps_m=0.025)
                    self.trigger_emergency_recovery(heading_err=heading_err, steer_dir=side_steer, current_v=current_v, current_steer=active_steer, instant_reverse=True)
                    return True, self.current_cmd_v, self.current_cmd_steer
            else:
                self.stall_timer = max(0.0, self.stall_timer - dt * 2.0)
            return False, 0.0, 0.0

        # ACTIVE RECOVERY TIMERS
        self.stroke_timer += dt
        self.total_recovery_timer += dt

        # Global failsafe timeout: if recovery is stuck longer than timeout, force exit to drive
        if self.total_recovery_timer >= RECOVERY_TOTAL_TIMEOUT_S and not is_front_imminent_obstacle and d_fl3 > 0.12 and d_fr3 > 0.12:
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
            has_sufficient_clearance = (
                self.stroke_timer >= RECOVERY_EXIT_MIN_TIME_S
                and min_front_cone >= self.clearance_exit_m
                and min_all_dist >= 0.14
                and not is_front_imminent_obstacle
                and d_fl3 > 0.15
                and d_fr3 > 0.15
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
