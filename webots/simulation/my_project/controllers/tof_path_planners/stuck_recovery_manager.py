"""
================================================================================
  INTELLIGENT AUTONOMOUS MULTI-POINT K-TURN STUCK RECOVERY MANAGER
================================================================================
"""

import math
from typing import Any, Dict, Optional, Tuple

try:
    from .planner_config import (
        RECOVERY_REVERSE_SPEED_MPS,
        RECOVERY_FORWARD_SPEED_MPS,
        RECOVERY_STEER_ANGLE_RAD,
        RECOVERY_STALL_CONFIRM_S,
        RECOVERY_FRONT_BLOCKED_M,
        RECOVERY_REVERSE_STROKE_MAX_S,
        RECOVERY_FORWARD_STROKE_MAX_S,
        RECOVERY_TOTAL_TIMEOUT_S,
        RECOVERY_CLEARANCE_EXIT_M,
        RECOVERY_EXIT_MIN_TIME_S,
        TOF_MAX_RANGE_M,
    )
except ImportError:
    from planner_config import (
        RECOVERY_REVERSE_SPEED_MPS,
        RECOVERY_FORWARD_SPEED_MPS,
        RECOVERY_STEER_ANGLE_RAD,
        RECOVERY_STALL_CONFIRM_S,
        RECOVERY_FRONT_BLOCKED_M,
        RECOVERY_REVERSE_STROKE_MAX_S,
        RECOVERY_FORWARD_STROKE_MAX_S,
        RECOVERY_TOTAL_TIMEOUT_S,
        RECOVERY_CLEARANCE_EXIT_M,
        RECOVERY_EXIT_MIN_TIME_S,
        TOF_MAX_RANGE_M,
    )


class StuckRecoveryState:
    NORMAL_DRIVE = "DRIVE"
    PIVOT_REVERSE = "PIVOT_REVERSE"
    PIVOT_FORWARD = "PIVOT_FORWARD"


class StuckRecoveryManager:
    """
    Intelligent Autonomous Multi-Point K-Turn & Clearance Recovery Controller:
    - Eliminates endless reverse backing up into rear walls or competitors.
    - If facing backwards or physically stalled, executes rapid alternating 3-point K-turns
      (Reverse Arc <-> Forward Arc) that continuously rotate the chassis toward open space.
    - Instantly exits to NORMAL_DRIVE the moment front clearance and forward alignment are restored.
    """

    def __init__(
        self,
        reverse_speed_m_s: float = RECOVERY_REVERSE_SPEED_MPS,
        forward_speed_m_s: float = RECOVERY_FORWARD_SPEED_MPS,
        steer_angle_rad: float = RECOVERY_STEER_ANGLE_RAD,
    ):
        self.state = StuckRecoveryState.NORMAL_DRIVE
        self.reverse_speed = reverse_speed_m_s
        self.forward_speed = forward_speed_m_s
        self.steer_angle = steer_angle_rad
        self.stall_timer = 0.0
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        self.preferred_yaw_sign = 1.0

    @staticmethod
    def resolve_escape_steer_dir(
        cached_tofs: Any,
        heading_err: float = 0.0,
        similar_eps_m: float = 0.040,
    ) -> float:
        """
        Hierarchical steering direction resolution when triggering emergency backup:
        Returns: +1.0 (steer right so front turns right / away from left)
                 or -1.0 (steer left so front turns left / away from right)

        Tiers in order:
        1. Primary: Compare tof_fl_3 vs tof_fr_3. If abs(fr3 - fl3) > 0.04m, steer away from closer side.
        2. Lateral rear outer: If within 4cm, compare tof_bl_0 vs tof_br_0.
        3. Lateral rear inner: If still within 4cm, compare tof_bl_1 vs tof_br_1.
        4. Track flow heading error fallback: Use heading error toward open corridor.
        """
        if isinstance(cached_tofs, dict):
            fl3 = float(cached_tofs.get("tof_fl_3", 1.80))
            fr3 = float(cached_tofs.get("tof_fr_3", 1.80))
            bl0 = float(cached_tofs.get("tof_bl_0", 1.80))
            br0 = float(cached_tofs.get("tof_br_0", 1.80))
            bl1 = float(cached_tofs.get("tof_bl_1", 1.80))
            br1 = float(cached_tofs.get("tof_br_1", 1.80))
        else:
            tofs = list(cached_tofs)
            fl3 = float(tofs[3]) if len(tofs) > 3 else 1.80
            fr3 = float(tofs[7]) if len(tofs) > 7 else 1.80
            bl0 = float(tofs[8]) if len(tofs) > 8 else 1.80
            br0 = float(tofs[12]) if len(tofs) > 12 else 1.80
            bl1 = float(tofs[9]) if len(tofs) > 9 else 1.80
            br1 = float(tofs[13]) if len(tofs) > 13 else 1.80

        # Tier 1: Primary front-center (tof_fl_3 vs tof_fr_3)
        diff_1 = fr3 - fl3
        if abs(diff_1) > similar_eps_m:
            # fl3 is closer -> obstacle on left -> steer right (+1.0)
            return 1.0 if diff_1 > 0.0 else -1.0

        # Tier 3: Lateral rear outer (tof_bl_0 vs tof_br_0)
        diff_3 = br0 - bl0
        if abs(diff_3) > similar_eps_m:
            return 1.0 if diff_3 > 0.0 else -1.0

        # Tier 4: Lateral rear inner (tof_bl_1 vs tof_br_1)
        diff_4 = br1 - bl1
        if abs(diff_4) > similar_eps_m:
            return 1.0 if diff_4 > 0.0 else -1.0

        # Tier 6: Heading error fallback
        if abs(heading_err) > math.radians(10.0):
            return 1.0 if heading_err > 0.0 else -1.0

        return 1.0  # Default right pivot

    def trigger_emergency_recovery(self, heading_err: float = 0.0, steer_dir: Optional[float] = None):
        """Immediately triggers intelligent recovery maneuver."""
        self.state = StuckRecoveryState.PIVOT_REVERSE
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        if steer_dir is not None:
            self.preferred_yaw_sign = 1.0 if steer_dir > 0 else -1.0
        elif abs(heading_err) > math.radians(25.0):
            self.preferred_yaw_sign = 1.0 if heading_err > 0 else -1.0
        else:
            self.preferred_yaw_sign = 1.0

    def update(
        self,
        dt: float,
        current_v: float,
        commanded_v: float,
        cached_tofs: Dict[str, float],
        heading_err: float = 0.0,
    ) -> Tuple[bool, float, float]:
        """
        Updates recovery state machine.
        Returns:
            (is_recovering, target_v, target_steer)
        """
        front_keys = ["tof_fl_3", "tof_fr_3", "tof_fl_2", "tof_fr_2"]
        rear_keys = ["tof_bl_3", "tof_br_3", "tof_bl_2", "tof_br_2"]
        left_keys = ["tof_fl_0", "tof_fl_1", "tof_bl_0", "tof_bl_1"]
        right_keys = ["tof_fr_0", "tof_fr_1", "tof_br_0", "tof_br_1"]

        if isinstance(cached_tofs, dict):
            min_all_dist = min(cached_tofs.values()) if cached_tofs else TOF_MAX_RANGE_M
            min_front_cone = min([cached_tofs.get(k, TOF_MAX_RANGE_M) for k in front_keys])
            min_rear_cone = min([cached_tofs.get(k, TOF_MAX_RANGE_M) for k in rear_keys])
            left_dist = min([cached_tofs.get(k, TOF_MAX_RANGE_M) for k in left_keys])
            right_dist = min([cached_tofs.get(k, TOF_MAX_RANGE_M) for k in right_keys])
            d_fl3 = cached_tofs.get("tof_fl_3", TOF_MAX_RANGE_M)
            d_fr3 = cached_tofs.get("tof_fr_3", TOF_MAX_RANGE_M)
        else:
            tofs_arr = list(cached_tofs)
            min_all_dist = min(tofs_arr) if tofs_arr else TOF_MAX_RANGE_M
            d_fl3 = tofs_arr[3] if len(tofs_arr) > 3 else TOF_MAX_RANGE_M
            d_fr3 = tofs_arr[7] if len(tofs_arr) > 7 else TOF_MAX_RANGE_M
            min_front_cone = min([tofs_arr[i] for i in [2, 3, 6, 7] if i < len(tofs_arr)]) if tofs_arr else TOF_MAX_RANGE_M
            min_rear_cone = min([tofs_arr[i] for i in [10, 11, 14, 15] if i < len(tofs_arr)]) if tofs_arr else TOF_MAX_RANGE_M
            left_dist = min([tofs_arr[i] for i in [0, 1, 8, 9] if i < len(tofs_arr)]) if tofs_arr else TOF_MAX_RANGE_M
            right_dist = min([tofs_arr[i] for i in [4, 5, 12, 13] if i < len(tofs_arr)]) if tofs_arr else TOF_MAX_RANGE_M

        # Imminent obstacle check: whenever fl_3 or fr_3 is 9cm or under (<= 0.09m)
        is_front_imminent_obstacle = (d_fl3 <= 0.09 or d_fr3 <= 0.09)

        # 1. Spinout / Wrong-Way Detection (|heading_err| > 75 deg)
        is_facing_backwards = abs(heading_err) > math.radians(75.0)

        # 2. Physical Stall Detection (Forward throttle commanded but vehicle blocked)
        is_stalled_forward = (commanded_v > 0.30 and abs(current_v) < 0.10 and min_front_cone < RECOVERY_FRONT_BLOCKED_M)

        if self.state == StuckRecoveryState.NORMAL_DRIVE:
            if is_front_imminent_obstacle:
                # Hierarchical resolution: if fl_3 and fr_3 are within 4cm, checks fl_2/fr_2, bl_0/br_0, bl_1/br_1, etc.
                side_steer = self.resolve_escape_steer_dir(cached_tofs, heading_err=heading_err, similar_eps_m=0.040)
                self.trigger_emergency_recovery(heading_err=heading_err, steer_dir=side_steer)
                self.state = StuckRecoveryState.PIVOT_REVERSE
                target_v = -self.reverse_speed
                target_steer = -self.preferred_yaw_sign * self.steer_angle
                return True, target_v, target_steer
            elif is_facing_backwards:
                self.trigger_emergency_recovery(heading_err=heading_err)
            elif is_stalled_forward:
                self.stall_timer += dt
                if self.stall_timer > RECOVERY_STALL_CONFIRM_S:
                    side_steer = self.resolve_escape_steer_dir(cached_tofs, heading_err=heading_err, similar_eps_m=0.040)
                    self.trigger_emergency_recovery(heading_err=heading_err, steer_dir=side_steer)
            else:
                self.stall_timer = max(0.0, self.stall_timer - dt * 2.0)
            return False, 0.0, 0.0

        # Active Recovery Loop
        self.stroke_timer += dt
        self.total_recovery_timer += dt

        is_aligned_forward = abs(heading_err) <= math.radians(45.0)
        has_sufficient_clearance = (min_front_cone >= RECOVERY_CLEARANCE_EXIT_M and min_all_dist >= 0.14 and not is_front_imminent_obstacle and d_fl3 > 0.10 and d_fr3 > 0.10)

        # EXIT GATE: Front clear + heading facing forward, or total timeout reached (provided front is not still blocked <= 9cm)
        if (self.total_recovery_timer >= RECOVERY_EXIT_MIN_TIME_S and has_sufficient_clearance and is_aligned_forward) or (self.total_recovery_timer >= RECOVERY_TOTAL_TIMEOUT_S and not is_front_imminent_obstacle):
            self.state = StuckRecoveryState.NORMAL_DRIVE
            self.stall_timer = 0.0
            return False, 0.0, 0.0

        # Update preferred yaw rotation if heading error is substantial
        if abs(heading_err) > math.radians(30.0):
            self.preferred_yaw_sign = 1.0 if heading_err > 0 else -1.0

        # --- STROKE 1: REVERSE ARC ---
        if self.state == StuckRecoveryState.PIVOT_REVERSE:
            target_v = -self.reverse_speed
            target_steer = -self.preferred_yaw_sign * self.steer_angle

            is_stalled_reverse = (self.stroke_timer > 0.12 and abs(current_v) < 0.08)
            stroke_timeout = (self.stroke_timer >= RECOVERY_REVERSE_STROKE_MAX_S) or (min_rear_cone < 0.10)

            if is_stalled_reverse or stroke_timeout:
                self.state = StuckRecoveryState.PIVOT_FORWARD
                self.stroke_timer = 0.0

            return True, target_v, target_steer

        # --- STROKE 2: FORWARD ARC ---
        elif self.state == StuckRecoveryState.PIVOT_FORWARD:
            target_v = self.forward_speed
            target_steer = self.preferred_yaw_sign * self.steer_angle

            front_blocked = (is_front_imminent_obstacle or (self.stroke_timer > 0.10 and min_front_cone < 0.18))
            stroke_timeout = (self.stroke_timer >= RECOVERY_FORWARD_STROKE_MAX_S)

            if front_blocked or stroke_timeout:
                self.state = StuckRecoveryState.PIVOT_REVERSE
                self.stroke_timer = 0.0
                target_v = -self.reverse_speed
                target_steer = -self.preferred_yaw_sign * self.steer_angle

            return True, target_v, target_steer

        return False, 0.0, 0.0
