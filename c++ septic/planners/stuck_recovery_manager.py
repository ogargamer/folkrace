"""
================================================================================
  INTELLIGENT AUTONOMOUS MULTI-POINT K-TURN STUCK RECOVERY MANAGER
================================================================================
"""

import math
from collections import deque
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
        AWW_HISTORY_WINDOW_S,
        AWW_LOOKAHEAD_M,
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
        AWW_HISTORY_WINDOW_S,
        AWW_LOOKAHEAD_M,
    )


def wrap_angle(angle_rad: float) -> float:
    """Wraps an angle into [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


wrap_to_pi = wrap_angle


class StuckRecoveryState:
    NORMAL_DRIVE = "DRIVE"
    PIVOT_REVERSE = "PIVOT_REVERSE"
    PIVOT_FORWARD = "PIVOT_FORWARD"
    CORRECTIVE_180 = "CORRECTIVE_180"


class StuckRecoveryManager:
    """
    Intelligent Autonomous Multi-Point K-Turn & Clearance Recovery Controller:
    - Eliminates endless reverse backing up into rear walls or competitors.
    - If facing backwards or physically stalled, executes rapid alternating 3-point K-turns
      (Reverse Arc <-> Forward Arc) that continuously rotate the chassis toward open space.
    - Supports Anti-Wrong-Way Watchdog CORRECTIVE_180 to immediately spin 180 back to valid path.
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
        self.target_corrective_yaw: Optional[float] = None
        self.past_poses: deque = deque(maxlen=600)
        self.last_logged_pos: Optional[Tuple[float, float]] = None
        self.current_time_s: float = 0.0
        self.corrective_start_yaw: float = 0.0
        self.last_corrective_yaw: float = 0.0
        self.corrective_accumulated_turn: float = 0.0


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
            flank_l6_4 = float(cached_tofs.get("tof_flank_left_6.4deg", cached_tofs.get("tof_flank_left_7", cached_tofs.get("tof_bl_0", 1.80))))
            flank_r6_4 = float(cached_tofs.get("tof_flank_right_6.4deg", cached_tofs.get("tof_flank_right_7", cached_tofs.get("tof_br_0", 1.80))))
            flank_l17_6 = float(cached_tofs.get("tof_flank_left_17.6deg", cached_tofs.get("tof_flank_left_18", cached_tofs.get("tof_bl_1", 1.80))))
            flank_r17_6 = float(cached_tofs.get("tof_flank_right_17.6deg", cached_tofs.get("tof_flank_right_18", cached_tofs.get("tof_br_1", 1.80))))
        else:
            tofs = list(cached_tofs)
            fl3 = float(tofs[3]) if len(tofs) > 3 else 1.80
            fr3 = float(tofs[7]) if len(tofs) > 7 else 1.80
            flank_l6_4 = float(tofs[8]) if len(tofs) > 8 else 1.80
            flank_r6_4 = float(tofs[12]) if len(tofs) > 12 else 1.80
            flank_l17_6 = float(tofs[9]) if len(tofs) > 9 else 1.80
            flank_r17_6 = float(tofs[13]) if len(tofs) > 13 else 1.80

        # Tier 1: Primary front-center (tof_fl_3 vs tof_fr_3)
        diff_1 = fr3 - fl3
        if abs(diff_1) > similar_eps_m:
            # fl3 is closer -> obstacle on left -> steer right (+1.0)
            return 1.0 if diff_1 > 0.0 else -1.0

        # Tier 3: Lateral flank outer (tof_flank_left_6.4deg vs tof_flank_right_6.4deg)
        diff_3 = flank_r6_4 - flank_l6_4
        if abs(diff_3) > similar_eps_m:
            return 1.0 if diff_3 > 0.0 else -1.0

        # Tier 4: Lateral flank inner (tof_flank_left_17.6deg vs tof_flank_right_17.6deg)
        diff_4 = flank_r17_6 - flank_l17_6
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
        cached_tofs: Dict[str, float],
        heading_err: float = 0.0,
        current_yaw: float = 0.0,
        current_x: float = 0.0,
        current_y: float = 0.0,
        now_s: Optional[float] = None,
    ) -> Tuple[bool, float, float]:
        """
        Updates recovery state machine.
        Returns:
            (is_recovering, target_v, target_steer)
        """
        front_keys = ["tof_fl_3", "tof_fr_3", "tof_fl_2", "tof_fr_2"]
        rear_keys = [
            "tof_flank_left_40.1deg", "tof_flank_right_40.1deg", "tof_flank_left_28.9deg", "tof_flank_right_28.9deg",
            "tof_flank_left_41", "tof_flank_right_41", "tof_flank_left_30", "tof_flank_right_30",
            "tof_bl_3", "tof_br_3", "tof_bl_2", "tof_br_2"
        ]
        left_keys = [
            "tof_fl_0", "tof_fl_1", "tof_flank_left_6.4deg", "tof_flank_left_17.6deg",
            "tof_flank_left_7", "tof_flank_left_18", "tof_bl_0", "tof_bl_1"
        ]
        right_keys = [
            "tof_fr_0", "tof_fr_1", "tof_flank_right_6.4deg", "tof_flank_right_17.6deg",
            "tof_flank_right_7", "tof_flank_right_18", "tof_br_0", "tof_br_1"
        ]

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

        if now_s is not None:
            self.current_time_s = now_s
        else:
            self.current_time_s += dt

        if self.state == StuckRecoveryState.NORMAL_DRIVE:
            # Log on-board dead-reckoned trajectory history (with timestamp)
            if self.last_logged_pos is None:
                self.past_poses.append((current_x, current_y, current_yaw, self.current_time_s))
                self.last_logged_pos = (current_x, current_y)
            else:
                d_log = math.hypot(current_x - self.last_logged_pos[0], current_y - self.last_logged_pos[1])
                if d_log >= 0.05:
                    self.past_poses.append((current_x, current_y, current_yaw, self.current_time_s))
                    self.last_logged_pos = (current_x, current_y)

            # Rolling history: prune poses older than AWW_HISTORY_WINDOW_S so closed-loop laps never collide
            while self.past_poses and (self.current_time_s - self.past_poses[0][3]) > AWW_HISTORY_WINDOW_S:
                self.past_poses.popleft()

            if is_front_imminent_obstacle:
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
        # CRITICAL: Do NOT allow generic exit gate to abort CORRECTIVE_180!
        # CORRECTIVE_180 must complete its dedicated 135-degree rotation arc before exiting!
        if self.state != StuckRecoveryState.CORRECTIVE_180:
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

            # Clearance Rule: Front tofs >= 25cm AND any one other ToF >= 40cm
            both_front_clear_25cm = (d_fl3 >= 0.25 and d_fr3 >= 0.25)
            
            # Check if any non-front-center ToF has at least 40cm of clearance
            if isinstance(cached_tofs, dict):
                other_tofs = [v for k, v in cached_tofs.items() if k not in ("tof_fl_3", "tof_fr_3")]
            else:
                other_tofs = [val for idx, val in enumerate(cached_tofs) if idx not in (3, 7)]
            
            any_other_clear_40cm = any(d >= 0.40 for d in other_tofs) if other_tofs else False
            backup_clearance_satisfied = both_front_clear_25cm and any_other_clear_40cm

            timer_timeout_600ms = (self.stroke_timer >= 0.600)
            rear_blocked = (min_rear_cone < 0.10)
            is_stalled_reverse = (self.stroke_timer > 0.15 and abs(current_v) < 0.05)

            if backup_clearance_satisfied or timer_timeout_600ms or rear_blocked or is_stalled_reverse:
                # Look configured lookahead forward along current heading using IMU dead-reckoning
                fwd_x = current_x + AWW_LOOKAHEAD_M * math.cos(current_yaw)
                fwd_y = current_y + AWW_LOOKAHEAD_M * math.sin(current_yaw)

                # Check if lookahead point matches the exact point the vehicle came from within rolling history
                facing_origin = False
                for px, py, pyaw, pt in reversed(self.past_poses):
                    if (self.current_time_s - pt) > AWW_HISTORY_WINDOW_S:
                        continue
                    d_origin = math.hypot(px - fwd_x, py - fwd_y)
                    if d_origin <= 0.28:
                        heading_diff = abs(wrap_angle(current_yaw - pyaw))
                        if heading_diff >= math.radians(110.0):
                            facing_origin = True
                            break

                if facing_origin:
                    # Point is the exact point we came from in reverse!
                    # Engage corrective turn to rotate ~135 degrees back around to forward racing direction
                    self.trigger_corrective_180(
                        target_yaw=wrap_angle(current_yaw + math.pi),
                        steer_sign=self.preferred_yaw_sign,
                        current_yaw=current_yaw,
                    )
                    return True, -self.reverse_speed, -self.preferred_yaw_sign * self.steer_angle
                elif backup_clearance_satisfied or (timer_timeout_600ms and not is_front_imminent_obstacle):
                    # Front is clear over 25cm and at least one other ToF >= 40cm (or timeout): exit immediately to forward drive!
                    self.state = StuckRecoveryState.NORMAL_DRIVE
                    self.stroke_timer = 0.0
                    self.stall_timer = 0.0
                    return False, 0.0, 0.0
                else:
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

        # --- STROKE 3: CORRECTIVE 135-DEGREE / 180-DEGREE RECOVERY ARC ---
        elif self.state == StuckRecoveryState.CORRECTIVE_180:
            # Track accumulated yaw rotation to ensure at least 135 degrees of turn
            d_yaw = abs(wrap_angle(current_yaw - self.last_corrective_yaw))
            self.corrective_accumulated_turn += d_yaw
            self.last_corrective_yaw = current_yaw

            # Turn required: at least 135 degrees (2.356 rad)
            has_turned_135 = (self.corrective_accumulated_turn >= math.radians(135.0))

            # Alignment with target yaw (if target yaw provided)
            yaw_aligned = False
            if self.target_corrective_yaw is not None:
                yaw_diff = abs(wrap_angle(self.target_corrective_yaw - current_yaw))
                if yaw_diff <= math.radians(30.0):
                    yaw_aligned = True

            # Exit condition for CORRECTIVE_180:
            # Must turn at least 135 deg (or >= 90 deg AND aligned with target yaw)
            # AND front sensors must have open clearance (>= 0.22m and fl3/fr3 not blocked)
            if (has_turned_135 or (yaw_aligned and self.corrective_accumulated_turn >= math.radians(90.0))) and min_front_cone >= 0.22 and not is_front_imminent_obstacle:
                self.state = StuckRecoveryState.NORMAL_DRIVE
                self.stall_timer = 0.0
                self.stroke_timer = 0.0
                return False, 0.0, 0.0

            # 1. Primary intended turn direction toward target yaw
            if self.target_corrective_yaw is not None:
                yaw_err = wrap_angle(self.target_corrective_yaw - current_yaw)
                intended_steer_dir = 1.0 if yaw_err > 0 else -1.0
            else:
                intended_steer_dir = self.preferred_yaw_sign

            # 2. Obstacle-aware clearance assessment:
            # For full steering turns, the outward-angled flank ToFs (flank_left_6.4deg, 17.6deg, flank_right_6.4deg, 17.6deg) 
            # and flank corner ToFs (flank_left_28.9deg, 40.1deg, flank_right_28.9deg, 40.1deg) dictate tail swing and pivot clearance.
            if isinstance(cached_tofs, dict):
                bl_lateral = min(
                    cached_tofs.get("tof_flank_left_6.4deg", cached_tofs.get("tof_flank_left_7", cached_tofs.get("tof_bl_0", 1.80))),
                    cached_tofs.get("tof_flank_left_17.6deg", cached_tofs.get("tof_flank_left_18", cached_tofs.get("tof_bl_1", 1.80)))
                )
                br_lateral = min(
                    cached_tofs.get("tof_flank_right_6.4deg", cached_tofs.get("tof_flank_right_7", cached_tofs.get("tof_br_0", 1.80))),
                    cached_tofs.get("tof_flank_right_17.6deg", cached_tofs.get("tof_flank_right_18", cached_tofs.get("tof_br_1", 1.80)))
                )
                bl_corner = min(
                    cached_tofs.get("tof_flank_left_28.9deg", cached_tofs.get("tof_flank_left_30", cached_tofs.get("tof_bl_2", 1.80))),
                    cached_tofs.get("tof_flank_left_40.1deg", cached_tofs.get("tof_flank_left_41", cached_tofs.get("tof_bl_3", 1.80)))
                )
                br_corner = min(
                    cached_tofs.get("tof_flank_right_28.9deg", cached_tofs.get("tof_flank_right_30", cached_tofs.get("tof_br_2", 1.80))),
                    cached_tofs.get("tof_flank_right_40.1deg", cached_tofs.get("tof_flank_right_41", cached_tofs.get("tof_br_3", 1.80)))
                )
            else:
                tofs_arr = list(cached_tofs)
                bl_lateral = min(tofs_arr[8], tofs_arr[9]) if len(tofs_arr) > 9 else 1.80
                br_lateral = min(tofs_arr[12], tofs_arr[13]) if len(tofs_arr) > 13 else 1.80
                bl_corner = min(tofs_arr[10], tofs_arr[11]) if len(tofs_arr) > 11 else 1.80
                br_corner = min(tofs_arr[14], tofs_arr[15]) if len(tofs_arr) > 15 else 1.80

            rear_left = min(bl_lateral, bl_corner)
            rear_right = min(br_lateral, br_corner)
            rear_diff = rear_right - rear_left

            # Front clearance check
            fl_near = min(d_fl3, cached_tofs.get("tof_fl_2", 1.80) if isinstance(cached_tofs, dict) else 1.80)
            fr_near = min(d_fr3, cached_tofs.get("tof_fr_2", 1.80) if isinstance(cached_tofs, dict) else 1.80)
            front_diff = fr_near - fl_near

            # Check if forward or reverse motion is viable
            front_blocked = (min_front_cone < 0.20 or is_front_imminent_obstacle)
            rear_blocked = (min_rear_cone < 0.12 or bl_corner < 0.10 or br_corner < 0.10)

            # Reversing stroke when front is tight or blocked, unless rear corners are pinned
            is_reverse_stroke = front_blocked and not rear_blocked

            # Alternate stroke if stalled in current direction
            if self.stroke_timer > 0.25 and abs(current_v) < 0.04:
                is_reverse_stroke = not front_blocked

            if is_reverse_stroke:
                target_v = -self.reverse_speed
                # In reverse, prioritize rear corner clearance: steer away from pinched rear side
                if abs(rear_diff) > 0.05:
                    # Rear-right has more space -> steer Right (-steer) so tail swings Right
                    steer_sign = -1.0 if rear_diff > 0.0 else 1.0
                elif abs(front_diff) > 0.05:
                    # Front-right is clearer -> swing front Right
                    steer_sign = 1.0 if front_diff > 0.0 else -1.0
                else:
                    steer_sign = intended_steer_dir
                target_steer = -steer_sign * self.steer_angle
            else:
                target_v = self.forward_speed
                # In forward gear, check if rear tail swing has clearance on the opposite flank
                if abs(rear_diff) > 0.08:
                    steer_sign = 1.0 if rear_diff > 0.0 else -1.0
                elif abs(front_diff) > 0.06:
                    steer_sign = 1.0 if front_diff > 0.0 else -1.0
                else:
                    steer_sign = intended_steer_dir
                target_steer = steer_sign * self.steer_angle

            # Timeout failsafe after 3.5s (provided front is not still blocked <= 9cm)
            if self.total_recovery_timer >= 3.5 and min_front_cone >= 0.20 and not is_front_imminent_obstacle:
                self.state = StuckRecoveryState.NORMAL_DRIVE
                self.stall_timer = 0.0
                return False, 0.0, 0.0

            return True, target_v, target_steer

