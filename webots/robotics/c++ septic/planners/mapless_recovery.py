"""
================================================================================
  MAPLESS WRONG-WAY DETECTION & RECOVERY CONTROLLER (50 Hz)
================================================================================
  Tracks inertial heading anchor during healthy forward driving.
  Detects:
    1. Impact shock spikes (|a_xy| > 3.2g)
    2. Violent rotational breakaway (|yaw_rate| > 6.0 rad/s)
  Freezes pre-crash heading anchor upon event detection.
  Evaluates post-event orientation:
    - If heading error > 1.85 rad (~106 deg), the robot is facing backward.
    - Uses Ackermann-compatible realignment:
      * If blocked ahead (front ToF < 0.20m): reverses with counter-steer to
        swing nose toward anchor.
      * When clear ahead: crawls forward aligning heading toward anchor.
  Transitions smoothly through SETTLE state back to primary path planner.
================================================================================
"""

import math
from enum import IntEnum
from typing import Dict, Tuple, Optional


class RecoveryState(IntEnum):
    NORMAL_TRACKING = 0
    SPIN_OR_IMPACT_BRAKE = 1
    REALIGN_REVERSE_ARC = 2
    REALIGN_FORWARD_ALIGN = 3
    SETTLE = 4


def wrap_angle(angle: float) -> float:
    """Wraps angle into [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class MaplessRecoveryController:
    """
    50 Hz state machine for mapless collision / wrong-way recovery.
    """

    def __init__(
        self,
        impact_g_thresh: float = 3.2,       # Gs: shock collision threshold
        spin_rate_thresh: float = 6.0,      # rad/s (~343 deg/s): breakaway spin threshold
        max_controlled_yaw: float = 3.5,    # rad/s: normal maximum controlled cornering yaw rate
        inverted_angle_thresh: float = 1.85,# rad (~106 deg): backward threshold
        realign_tol_rad: float = 0.40,      # rad (~23 deg): heading tolerance to resume
        pivot_speed_mps: float = 0.55,      # m/s: realignment forward crawl speed
        reverse_speed_mps: float = -1.00,   # m/s: fast realignment reverse crawl speed near traction limit
        max_steer_rad: float = 0.6981,      # rad: maximum steering lock (+/- 40 deg)
        settle_time_s: float = 0.08,        # stabilization duration before handoff
        max_recovery_time_s: float = 3.0,   # global safety timeout for recovery
    ):
        self.impact_g_thresh = impact_g_thresh
        self.spin_rate_thresh = spin_rate_thresh
        self.max_controlled_yaw = max_controlled_yaw
        self.inverted_angle_thresh = inverted_angle_thresh
        self.realign_tol_rad = realign_tol_rad
        self.pivot_speed = pivot_speed_mps
        self.reverse_speed = reverse_speed_mps
        self.max_steer = max_steer_rad
        self.settle_time = settle_time_s
        self.max_recovery_time = max_recovery_time_s

        self.state: RecoveryState = RecoveryState.NORMAL_TRACKING
        self.locked_target_yaw: float = 0.0
        self.initialized_anchor: bool = False
        self.settle_timer: float = 0.0
        self.state_timer: float = 0.0
        self.total_recovery_timer: float = 0.0

    def reset(self, initial_yaw: Optional[float] = None):
        """Resets controller state."""
        self.state = RecoveryState.NORMAL_TRACKING
        self.settle_timer = 0.0
        self.state_timer = 0.0
        self.total_recovery_timer = 0.0
        if initial_yaw is not None:
            self.locked_target_yaw = initial_yaw
            self.initialized_anchor = True
        else:
            self.initialized_anchor = False

    def update(
        self,
        current_yaw: float,
        current_yaw_rate: float,
        accel_x_g: float,
        accel_y_g: float,
        current_speed: float,
        tof_readings: Optional[Dict[str, float]] = None,
        dt: float = 0.020,
    ) -> Tuple[bool, float, float]:
        """
        Executes one 50 Hz recovery evaluation cycle.

        Args:
            current_yaw: World yaw heading from IMU (rad).
            current_yaw_rate: Body Z-axis gyro rate (rad/s).
            accel_x_g: Lateral acceleration in Gs.
            accel_y_g: Longitudinal acceleration in Gs.
            current_speed: Estimated forward ground speed (m/s).
            tof_readings: Dictionary of ToF sensor distances (meters).
            dt: Timestep (0.020s for 50 Hz).

        Returns:
            Tuple[bool, float, float]:
                - active: True if recovery overrides planner, False otherwise.
                - out_steer: Overridden steering command (rad).
                - out_speed: Overridden velocity command (m/s).
        """
        if not self.initialized_anchor:
            self.locked_target_yaw = current_yaw
            self.initialized_anchor = True

        total_accel_g = math.hypot(accel_x_g, accel_y_g)
        abs_yaw_rate = abs(current_yaw_rate)

        # Front clearance helper
        if tof_readings is not None:
            front_clearance = min(
                float(tof_readings.get("tof_fl_3", 1.80)),
                float(tof_readings.get("tof_fr_3", 1.80)),
            )
        else:
            front_clearance = 1.0

        if self.state != RecoveryState.NORMAL_TRACKING:
            self.total_recovery_timer += dt
            self.state_timer += dt

            # Global timeout guard: return control if recovery takes too long
            if self.total_recovery_timer >= self.max_recovery_time:
                self.reset(current_yaw)
                return False, 0.0, 0.0

        # State 0: Normal Tracking
        if self.state == RecoveryState.NORMAL_TRACKING:
            is_spinout = abs_yaw_rate > self.spin_rate_thresh
            is_impact = total_accel_g > self.impact_g_thresh

            # Wrong-way recovery ONLY engages if there is a severe spinout breakaway,
            # or if an impact caused the vehicle to rotate backward (> 90 deg from anchor).
            heading_err = wrap_angle(self.locked_target_yaw - current_yaw)
            is_wrong_way = abs(heading_err) > 1.5708  # Facing backwards (> 90 deg)

            if is_spinout or (is_impact and is_wrong_way):
                # FREEZE pre-crash anchor and initiate emergency brake
                self.state = RecoveryState.SPIN_OR_IMPACT_BRAKE
                self.state_timer = 0.0
                self.total_recovery_timer = 0.0
                return True, 0.0, 0.0

            # Normal driving: update anchor smoothly
            # Tracks continuously through tight 10 cm U-turns at normal cornering yaw rates
            if current_speed > 0.20 and abs_yaw_rate < self.max_controlled_yaw:
                self.locked_target_yaw = current_yaw

            return False, 0.0, 0.0

        # State 1: Spin or Impact Brake
        if self.state == RecoveryState.SPIN_OR_IMPACT_BRAKE:
            # Dissipate rotational kinetic energy
            # Proceed when rotation settles OR after 0.30s timeout guard
            if abs_yaw_rate < 1.2 or self.state_timer >= 0.30:
                heading_err = wrap_angle(self.locked_target_yaw - current_yaw)

                if abs(heading_err) > self.inverted_angle_thresh:
                    # Vehicle is backward relative to pre-crash anchor:
                    # Reverse arc to swing nose back toward the racing line direction
                    self.state = RecoveryState.REALIGN_REVERSE_ARC
                    self.state_timer = 0.0
                else:
                    # Facing correct direction: return immediately to normal/backup control
                    self.reset(current_yaw)
                    return False, 0.0, 0.0
            else:
                return True, 0.0, 0.0

        # State 2: Ackermann Reverse Arc (creates room and pivots nose toward anchor)
        if self.state == RecoveryState.REALIGN_REVERSE_ARC:
            heading_err = wrap_angle(self.locked_target_yaw - current_yaw)

            # In reverse, steering Left (+steer) swings nose Right (CW, decreasing yaw).
            # Steering Right (-steer) swings nose Left (CCW, increasing yaw).
            # If heading_err > 0, we need yaw to increase (rotate CCW): steer Right (-steer)!
            # If heading_err < 0, we need yaw to decrease (rotate CW): steer Left (+steer)!
            turn_dir = -1.0 if heading_err > 0.0 else 1.0
            out_steer = turn_dir * self.max_steer
            out_speed = self.reverse_speed

            # Switch to forward realignment once front clearance opens or heading closes in
            if front_clearance >= 0.25 or abs(heading_err) < 1.20 or self.state_timer >= 1.50:
                self.state = RecoveryState.REALIGN_FORWARD_ALIGN
                self.state_timer = 0.0

            return True, out_steer, out_speed

        # State 3: Forward Alignment
        elif self.state == RecoveryState.REALIGN_FORWARD_ALIGN:
            heading_err = wrap_angle(self.locked_target_yaw - current_yaw)

            # When driving forward, positive steer (+steer) turns Left (CCW, increasing yaw).
            turn_dir = 1.0 if heading_err > 0.0 else -1.0
            out_steer = turn_dir * self.max_steer
            out_speed = self.pivot_speed

            # If front gets blocked again while forward realigning, reverse arc again
            if front_clearance < 0.09 and self.state_timer > 0.20:
                self.state = RecoveryState.REALIGN_REVERSE_ARC
                self.state_timer = 0.0
                return True, -turn_dir * self.max_steer, self.reverse_speed

            # Exit alignment once within tolerance
            if abs(heading_err) < self.realign_tol_rad:
                self.state = RecoveryState.SETTLE
                self.settle_timer = 0.0
                self.state_timer = 0.0

            return True, out_steer, out_speed

        # State 4: Settle Stabilization
        elif self.state == RecoveryState.SETTLE:
            self.settle_timer += dt
            heading_err = wrap_angle(self.locked_target_yaw - current_yaw)

            out_steer = max(-self.max_steer, min(heading_err * 0.8, self.max_steer))
            out_speed = self.pivot_speed * 1.2

            if self.settle_timer >= self.settle_time:
                self.reset(current_yaw)
                return False, 0.0, 0.0

            return True, out_steer, out_speed

        return False, 0.0, 0.0
