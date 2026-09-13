"""
================================================================================
  AUTONOMOUS BACKUP OBSTACLE ESCAPE CONTROLLER (50 Hz)
================================================================================
  Monitors front ToF sensors (tof_fl_3 and tof_fr_3).
  - Triggers automatic reverse when either sensor <= 0.09m (9 cm).
  - Multi-tier hierarchical direction resolution:
      1. Primary: Compare tof_fl_3 vs tof_fr_3.
      2. Secondary: If similar, compare tof_bl_0 vs tof_br_0.
      3. Tertiary: If similar, compare tof_bl_1 vs tof_br_1.
      4. Fallback: Use IMU heading history (locked_target_yaw) to steer
         backwards so the front points towards the pre-event vector.
  - Reverses blindly (all 16 ToFs point forward/laterally, 0 rear-facing).
  - Maintains reverse until BOTH tof_fl_3 and tof_fr_3 see at least 0.25m (25 cm).
  - Max duration safety timeout (2.5s).
================================================================================
"""

import math
from typing import Dict, Tuple, Optional

# Physical vehicle and traction parameters
MAX_STEER_RAD = 0.6981  # 40 deg steering limit
TRIGGER_DIST_M = 0.090  # 9 cm trigger
CLEAR_DIST_M = 0.250    # 25 cm exit hysteresis
BACKUP_SPEED_MPS = -1.05   # Fast reverse speed near tire traction limit (m/s)
MAX_BACKUP_DURATION_S = 0.600  # 600 ms maximum backup timer timeout
SIMILAR_EPS_M = 0.040   # 4 cm minimum difference threshold for sensor comparison
WHEELBASE_M = 0.1325    # Chassis wheelbase in meters
MU_FRICTION = 0.80      # Grip coefficient
GRAVITY_G = 9.81        # m/s^2


def wrap_angle(angle: float) -> float:
    """Wraps angle into [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class BackupObstacleController:
    """
    Stateful 50 Hz controller managing close-proximity reverse escape.
    """

    def __init__(
        self,
        trigger_dist_m: float = TRIGGER_DIST_M,
        clear_dist_m: float = CLEAR_DIST_M,
        backup_speed_mps: float = BACKUP_SPEED_MPS,
        max_steer_rad: float = MAX_STEER_RAD,
        similar_eps_m: float = SIMILAR_EPS_M,
        max_duration_s: float = MAX_BACKUP_DURATION_S,
        mu_friction: float = MU_FRICTION,
    ):
        self.trigger_dist = trigger_dist_m
        self.clear_dist = clear_dist_m
        self.backup_speed = backup_speed_mps
        self.max_steer = max_steer_rad
        self.similar_eps = similar_eps_m
        self.max_duration = max_duration_s
        self.mu_friction = mu_friction

        self.is_active: bool = False
        self.timer_s: float = 0.0
        self.latched_steer: float = 0.0
        self.decision_reason: str = "IDLE"

    def reset(self):
        """Resets controller state."""
        self.is_active = False
        self.timer_s = 0.0
        self.latched_steer = 0.0
        self.decision_reason = "IDLE"

    def resolve_escape_steer(
        self,
        tof_readings: Dict[str, float],
        current_yaw: float,
        locked_target_yaw: float,
    ) -> Tuple[float, str]:
        """
        Hierarchical steering resolution in reverse gear:
        In reverse, steering Left (+steer) rotates chassis clockwise (front turns Right).
        Steering Right (-steer) rotates chassis counter-clockwise (front turns Left).
        """
        is_front_rig = "tof_front_left_4.1deg" in tof_readings or "tof_front_right_4.1deg" in tof_readings

        if is_front_rig:
            fl_center = float(tof_readings.get("tof_front_left_4.1deg", 1.80))
            fr_center = float(tof_readings.get("tof_front_right_4.1deg", 1.80))
            diff_1 = fr_center - fl_center

            # Tier 1: 4.1 deg center sensors
            if abs(diff_1) > self.similar_eps:
                if diff_1 > 0.0:
                    # Obstacle on Left (fl < fr) -> open on Right -> steer Left (+steer) so front swings Right
                    return self.max_steer, "TIER1_FRONT_4_1DEG_RIGHT"
                else:
                    # Obstacle on Right (fr < fl) -> open on Left -> steer Right (-steer) so front swings Left
                    return -self.max_steer, "TIER1_FRONT_4_1DEG_LEFT"

            # Tier 2: 15.4 deg inner sensors
            fl_inner = float(tof_readings.get("tof_front_left_15.4deg", 1.80))
            fr_inner = float(tof_readings.get("tof_front_right_15.4deg", 1.80))
            diff_2 = fr_inner - fl_inner
            if abs(diff_2) > self.similar_eps:
                if diff_2 > 0.0:
                    return self.max_steer, "TIER2_FRONT_15_4DEG_RIGHT"
                else:
                    return -self.max_steer, "TIER2_FRONT_15_4DEG_LEFT"

            # Tier 3: Flank sensors (cross-pointing: right sensors probe left side, left sensors probe right side)
            d_left_side = min(
                float(tof_readings.get("tof_front_right_82.9deg", 1.80)),
                float(tof_readings.get("tof_front_right_71.6deg", 1.80)),
                float(tof_readings.get("tof_front_right_60.4deg", 1.80)),
                float(tof_readings.get("tof_front_right_49.1deg", 1.80)),
            )
            d_right_side = min(
                float(tof_readings.get("tof_front_left_82.9deg", 1.80)),
                float(tof_readings.get("tof_front_left_71.6deg", 1.80)),
                float(tof_readings.get("tof_front_left_60.4deg", 1.80)),
                float(tof_readings.get("tof_front_left_49.1deg", 1.80)),
            )
            diff_side = d_left_side - d_right_side
            if abs(diff_side) > self.similar_eps:
                if diff_side > 0.0:
                    # More room on left -> steer Right (-steer) so front swings Left
                    return -self.max_steer, "TIER3_FLANK_LEFT"
                else:
                    # More room on right -> steer Left (+steer) so front swings Right
                    return self.max_steer, "TIER3_FLANK_RIGHT"
        else:
            # Tier 1: tof_fl_3 vs tof_fr_3
            fl3 = float(tof_readings.get("tof_fl_3", 1.80))
            fr3 = float(tof_readings.get("tof_fr_3", 1.80))
            diff_1 = fr3 - fl3

            if abs(diff_1) > self.similar_eps:
                if diff_1 > 0.0:
                    # Obstacle on Left (fl3 < fr3) -> open on Right -> steer Left (+steer) so front swings Right
                    return self.max_steer, "TIER1_FL3_FR3_RIGHT"
                else:
                    # Obstacle on Right (fr3 < fl3) -> open on Left -> steer Right (-steer) so front swings Left
                    return -self.max_steer, "TIER1_FL3_FR3_LEFT"

            # Tier 2: tof_flank_left_6.4deg vs tof_flank_right_6.4deg
            flank_l6_4 = float(tof_readings.get("tof_flank_left_6.4deg", tof_readings.get("tof_flank_left_7", tof_readings.get("tof_bl_0", 1.80))))
            flank_r6_4 = float(tof_readings.get("tof_flank_right_6.4deg", tof_readings.get("tof_flank_right_7", tof_readings.get("tof_br_0", 1.80))))
            diff_2 = flank_r6_4 - flank_l6_4

            if abs(diff_2) > self.similar_eps:
                if diff_2 > 0.0:
                    return self.max_steer, "TIER2_FLANK_L6_4_R6_4_RIGHT"
                else:
                    return -self.max_steer, "TIER2_FLANK_L6_4_R6_4_LEFT"

            # Tier 3: tof_flank_left_17.6deg vs tof_flank_right_17.6deg
            flank_l17_6 = float(tof_readings.get("tof_flank_left_17.6deg", tof_readings.get("tof_flank_left_18", tof_readings.get("tof_bl_1", 1.80))))
            flank_r17_6 = float(tof_readings.get("tof_flank_right_17.6deg", tof_readings.get("tof_flank_right_18", tof_readings.get("tof_br_1", 1.80))))
            diff_3 = flank_r17_6 - flank_l17_6

            if abs(diff_3) > self.similar_eps:
                if diff_3 > 0.0:
                    return self.max_steer, "TIER3_FLANK_L17_6_R17_6_RIGHT"
                else:
                    return -self.max_steer, "TIER3_FLANK_L17_6_R17_6_LEFT"

        # Tier 4: IMU Vector History Logic
        # Steer backwards so front points towards locked_target_yaw
        heading_err = wrap_angle(locked_target_yaw - current_yaw)

        if heading_err > 0.05:
            # Target is to Left (CCW) -> in reverse, steer Right (-steer) so front swings Left
            return -self.max_steer, "TIER4_IMU_TARGET_LEFT"
        elif heading_err < -0.05:
            # Target is to Right (CW) -> in reverse, steer Left (+steer) so front swings Right
            return self.max_steer, "TIER4_IMU_TARGET_RIGHT"
        else:
            return 0.0, "TIER4_IMU_ALIGNED"

    def update(
        self,
        tof_readings: Dict[str, float],
        current_yaw: float = 0.0,
        locked_target_yaw: float = 0.0,
        contact_detected: bool = False,
        dt: float = 0.020,
    ) -> Tuple[bool, float, float]:
        """
        Executes one 50 Hz cycle.

        Args:
            tof_readings: Dictionary mapping sensor names to distance (meters).
            current_yaw: Current vehicle yaw from IMU (rad).
            locked_target_yaw: Target heading anchor from IMU history (rad).
            contact_detected: Physical contact / bumper touch flag.
            dt: Timestep (0.020s for 50 Hz).

        Returns:
            Tuple[bool, float, float]: (active, cmd_steer_rad, cmd_speed_mps)
        """
        if "tof_front_left_4.1deg" in tof_readings or "tof_front_right_4.1deg" in tof_readings:
            fl_center = float(tof_readings.get("tof_front_left_4.1deg", 1.80))
            fr_center = float(tof_readings.get("tof_front_right_4.1deg", 1.80))
            center_keys = ("tof_front_left_4.1deg", "tof_front_right_4.1deg")
        else:
            fl_center = float(tof_readings.get("tof_fl_3", 1.80))
            fr_center = float(tof_readings.get("tof_fr_3", 1.80))
            center_keys = ("tof_fl_3", "tof_fr_3")

        # 1. Trigger condition from normal forward operation
        if not self.is_active:
            if fl_center <= self.trigger_dist or fr_center <= self.trigger_dist or contact_detected:
                self.is_active = True
                self.timer_s = 0.0
                steer, reason = self.resolve_escape_steer(tof_readings, current_yaw, locked_target_yaw)
                self.latched_steer = steer
                self.decision_reason = reason if not contact_detected else f"CONTACT_{reason}"

        # 2. If active, monitor exit condition and timeout
        if self.is_active:
            self.timer_s += dt

            # Exit condition: both front sensors see at least clear_dist (25 cm) AND any other ToF sees >= 40 cm
            other_tofs = [v for k, v in tof_readings.items() if k not in center_keys]
            any_other_clear_40cm = any(d >= 0.40 for d in other_tofs) if other_tofs else False
            if fl_center >= self.clear_dist and fr_center >= self.clear_dist and any_other_clear_40cm:
                self.reset()
                return False, 0.0, 0.0

            # Safety timeout guard
            if self.timer_s >= self.max_duration:
                self.reset()
                return False, 0.0, 0.0

            # Re-evaluate escape steering every cycle dynamically
            steer, reason = self.resolve_escape_steer(tof_readings, current_yaw, locked_target_yaw)
            self.latched_steer = steer
            self.decision_reason = reason

            # Reversing blindly (0 rear-pointing ToFs):
            # Dynamic reverse speed scaled to physical tire traction limit based on steering angle
            kappa = abs(math.tan(self.latched_steer)) / WHEELBASE_M
            v_lat_max = math.sqrt((self.mu_friction * GRAVITY_G * 0.85) / max(kappa, 0.5))
            speed_cmd = -min(abs(self.backup_speed), v_lat_max)

            return True, self.latched_steer, speed_cmd

        return False, 0.0, 0.0
