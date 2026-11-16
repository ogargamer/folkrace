"""
================================================================================
  4WD ELECTRONIC DIFFERENTIAL & ACKERMANN KINEMATICS MODULE
  Real-time geometric speed allocation and BNO08x gyro slip detection
  for Folkrace Robot in Webots.
================================================================================
"""

import math
from typing import Optional, Dict, Any

try:
    from .planner_config import (
        WHEELBASE,
        TRACK_REAR,
        TRACK_FRONT,
        TRACK_KINGPIN,
        WHEEL_RADIUS,
        MAX_SERVO_RAD,
        MAX_STEERING_RAD,
        ACKERMANN_PERCENT,
        MOTOR_DIR,
        V_MAX_WHEEL,
        SERVO_MAX_SPEED_RAD_S,
        TIRE_MU,
        GRAVITY_G,
        SLIP_GATE_SPEED_MPS,
        SLIP_THRESH_SPIN_RAD_S,
    )
except ImportError:
    from planner_config import (
        WHEELBASE,
        TRACK_REAR,
        TRACK_FRONT,
        TRACK_KINGPIN,
        WHEEL_RADIUS,
        MAX_SERVO_RAD,
        MAX_STEERING_RAD,
        ACKERMANN_PERCENT,
        MOTOR_DIR,
        V_MAX_WHEEL,
        SERVO_MAX_SPEED_RAD_S,
        TIRE_MU,
        GRAVITY_G,
        SLIP_GATE_SPEED_MPS,
        SLIP_THRESH_SPIN_RAD_S,
    )

V_SLIP_GATE = SLIP_GATE_SPEED_MPS         # m/s minimum speed to activate dynamic IMU slip detection
THRESH_SPIN = SLIP_THRESH_SPIN_RAD_S      # rad/s gross yaw rate divergence threshold for oversteer

GYRO_YAW_REF_TAU_S = 0.030      # 30 ms 1st-order tire/chassis yaw reference lag
GYRO_YAW_DOT_TAU_S = 0.020      # 20 ms 1st-order derivative low-pass filter
GYRO_DEADBAND_RAD_S = 0.20      # IMU yaw-rate noise deadband
GYRO_KP_STEER = 0.08            # Proportional gyro countersteer gain (rad / (rad/s))
GYRO_KD_STEER = 0.008           # Derivative yaw damper gain (rad / (rad/s^2))
GYRO_FULL_SPEED_MPS = 0.60      # Speed where gyro assist reaches 100% authority
SLIP_HOLD_TIME_S = 0.060        # 60 ms hysteresis hold latch for slip telemetry/interventions
DIFF_FADE_RATE = 12.0           # Fast fade-out of rear diff split on oversteer (1/s)
DIFF_RISE_RATE = 4.0            # Smooth recovery of rear diff split (1/s)
UNDERSTEER_SPEED_SCALE = 0.75   # Speed reduction ratio during front-plow understeer
OVERSTEER_SPEED_SCALE = 0.85    # Speed reduction ratio during rear-axle oversteer
SLIP_SPEED_FLOOR_MPS = 0.50     # Minimum cornering speed floor during slip throttle cut

SLIP_NORMAL = "NORMAL"
SLIP_OVERSTEER = "OVERSTEER"
SLIP_UNDERSTEER = "UNDERSTEER"


class ElectronicDifferential4WD:
    """
    Computes 45% Ackermann steering angles (35 deg inner, 30 deg outer),
    continuous grip-capped PD gyro steering assist, slip-coupled traction caps,
    and 4-wheel rotational speeds with electronic differential.
    """

    def __init__(self, wheelbase=WHEELBASE, track_rear=TRACK_REAR, track_front=TRACK_FRONT,
                 track_kingpin=TRACK_KINGPIN, wheel_radius=WHEEL_RADIUS,
                 max_steer=MAX_STEERING_RAD, max_servo_angle=MAX_SERVO_RAD,
                 ackermann_percent=ACKERMANN_PERCENT,
                 motor_dir=MOTOR_DIR, v_max=V_MAX_WHEEL,
                 thresh_spin=THRESH_SPIN, max_servo_rate=SERVO_MAX_SPEED_RAD_S):
        self.L = wheelbase
        self.W_r = track_rear
        self.W_f = track_front
        self.W_k = track_kingpin
        self.R_w = wheel_radius
        self.max_steer = max_steer
        self.max_servo_angle = max_servo_angle
        self.ackermann_percent = ackermann_percent
        self.motor_dir = motor_dir
        self.v_max = v_max
        self.thresh_spin = thresh_spin
        self.max_servo_rate = max_servo_rate
        # PTK9794 @ 8.4V: 1510 +/- 271 us over a 2000 us / 180 deg span -> +/- 24.39 deg horn throw
        self.max_horn_angle = (271.0 / 2000.0) * math.pi
        self._linkage_gain = math.sin(self.max_steer) / math.sin(self.max_horn_angle)
        self.reset()

    def reset(self) -> None:
        """Resets steering linkage, yaw reference, derivative filter, and slip state."""
        self.current_horn_rad = 0.0
        self.current_steer_rad = 0.0
        self.driver_steer_rad = 0.0
        self.omega_ref = 0.0
        self.prev_gyro_z = 0.0
        self.gyro_z_dot_filt = 0.0
        self._has_prev_gyro = False
        self.slip_state = SLIP_NORMAL
        self._slip_hold_timer = 0.0
        self.diff_split_gain = 1.0

    def apply_traction_speed_cap(self, target_v: float, current_v: float, mu: float = TIRE_MU) -> float:
        """Caps commanded velocity when active understeer or oversteer is latched."""
        safe_mu = max(0.10, float(mu))
        effective_v = float(target_v)
        if self.slip_state == SLIP_UNDERSTEER:
            tan_driver = max(0.05, abs(math.tan(self.driver_steer_rad)))
            v_corner_max = math.sqrt((safe_mu * GRAVITY_G * self.L) / tan_driver)
            v_cap = max(SLIP_SPEED_FLOOR_MPS, min(v_corner_max, UNDERSTEER_SPEED_SCALE * abs(current_v)))
            if effective_v > 0.0:
                effective_v = min(effective_v, v_cap)
            elif effective_v < 0.0:
                effective_v = max(effective_v, -v_cap)
        elif self.slip_state == SLIP_OVERSTEER:
            v_cap = max(SLIP_SPEED_FLOOR_MPS, OVERSTEER_SPEED_SCALE * abs(current_v))
            if effective_v > 0.0:
                effective_v = min(effective_v, v_cap)
            elif effective_v < 0.0:
                effective_v = max(effective_v, -v_cap)
        return effective_v

    def update(
        self,
        target_v: float,
        target_steer_rad: float,
        gyro_z_rad_s: float = 0.0,
        dt: float = 1.0 / 120.0,
        rear_alpha: float = 1.0,
        v_ground_RL: Optional[float] = None,
        v_ground_RR: Optional[float] = None,
        is_drifting: bool = False,
        mu: float = TIRE_MU,
    ):
        """
        Calculates actuator setpoints given desired longitudinal velocity and virtual steering angle.
        Mirrors ESP32-S3 ElectronicDifferential4WD continuous gyro assist and slip stabilization.
        """
        safe_dt = dt if dt > 1e-5 else (1.0 / 120.0)
        safe_mu = max(0.10, float(mu))

        # 1. Driver steering intent & grip-capped lagged yaw-rate reference
        target_delta = max(-self.max_steer, min(target_steer_rad, self.max_steer))
        max_d_driver = self.max_servo_rate * safe_dt
        driver_diff = target_delta - self.driver_steer_rad
        if abs(driver_diff) <= max_d_driver:
            self.driver_steer_rad = target_delta
        else:
            self.driver_steer_rad += math.copysign(max_d_driver, driver_diff)
        self.driver_steer_rad = max(-self.max_steer, min(self.driver_steer_rad, self.max_steer))

        omega_cmd_driver = (target_v * math.tan(self.driver_steer_rad)) / self.L
        omega_grip_max = (safe_mu * GRAVITY_G) / max(abs(target_v), 0.50)
        omega_ref_target = max(-omega_grip_max, min(omega_grip_max, omega_cmd_driver))
        alpha_ref = safe_dt / (GYRO_YAW_REF_TAU_S + safe_dt)
        self.omega_ref += alpha_ref * (omega_ref_target - self.omega_ref)

        # 2. Yaw-rate derivative filter & continuous PD gyro-assisted countersteer
        d_gyro_raw = ((gyro_z_rad_s - self.prev_gyro_z) / safe_dt) if self._has_prev_gyro else 0.0
        self.prev_gyro_z = gyro_z_rad_s
        self._has_prev_gyro = True
        alpha_d = safe_dt / (GYRO_YAW_DOT_TAU_S + safe_dt)
        self.gyro_z_dot_filt += alpha_d * (d_gyro_raw - self.gyro_z_dot_filt)

        # Continuous excess yaw error (zero in normal turn-in / understeer band [0 .. omega_ref])
        excess_yaw = 0.0
        if self.omega_ref > 1e-4:
            if gyro_z_rad_s > self.omega_ref:
                excess_yaw = gyro_z_rad_s - self.omega_ref
            elif gyro_z_rad_s < 0.0:
                excess_yaw = gyro_z_rad_s
        elif self.omega_ref < -1e-4:
            if gyro_z_rad_s < self.omega_ref:
                excess_yaw = gyro_z_rad_s - self.omega_ref
            elif gyro_z_rad_s > 0.0:
                excess_yaw = gyro_z_rad_s
        else:
            excess_yaw = gyro_z_rad_s

        speed_abs = abs(target_v)
        gyro_fade = 0.0
        if speed_abs > V_SLIP_GATE:
            gyro_fade = min(1.0, (speed_abs - V_SLIP_GATE) / max(0.05, GYRO_FULL_SPEED_MPS - V_SLIP_GATE))
        speed_sign = 1.0 if target_v >= 0.0 else -1.0

        delta_gyro = 0.0
        if gyro_fade > 0.0 and abs(excess_yaw) > GYRO_DEADBAND_RAD_S:
            excess_db = math.copysign(abs(excess_yaw) - GYRO_DEADBAND_RAD_S, excess_yaw)
            d_weight = min(1.0, abs(excess_db) / 0.25)
            pd_raw = -(GYRO_KP_STEER * excess_db) - (d_weight * GYRO_KD_STEER * self.gyro_z_dot_filt)
            if excess_yaw > 0.0:
                pd_raw = min(0.0, pd_raw)
            else:
                pd_raw = max(0.0, pd_raw)
            delta_gyro = gyro_fade * speed_sign * pd_raw

        stabilized_target = max(-self.max_steer, min(self.max_steer, target_delta + delta_gyro))

        # 3. Apply servo horn slew rate limit (22.0 rad/s) through non-linear tie-rod linkage
        sin_horn_target = max(-1.0, min(1.0, math.sin(stabilized_target) / self._linkage_gain))
        target_horn = math.asin(sin_horn_target)
        max_d_horn = self.max_servo_rate * safe_dt
        horn_diff = target_horn - self.current_horn_rad
        if abs(horn_diff) <= max_d_horn:
            self.current_horn_rad = target_horn
        else:
            self.current_horn_rad += math.copysign(max_d_horn, horn_diff)

        sin_delta = max(-1.0, min(1.0, self._linkage_gain * math.sin(self.current_horn_rad)))
        self.current_steer_rad = math.asin(sin_delta)
        delta = self.current_steer_rad

        # 4. Kinematic yaw rate command from actual physical wheel angle
        sin_d = math.sin(delta)
        cos_d = math.cos(delta)
        tan_d = math.tan(delta)
        omega_cmd = (target_v * tan_d) / self.L

        # 5. 45% Geometric Ackermann Front Steering Angles (35 deg inner, 30 deg outer)
        if abs(tan_d) > 1e-4:
            half_Wk_sin = (self.W_k * 0.5) * sin_d
            L_cos = self.L * cos_d
            L_sin = self.L * sin_d
            steer_left_true = math.atan2(L_sin, L_cos - half_Wk_sin)
            steer_right_true = math.atan2(L_sin, L_cos + half_Wk_sin)
            steer_left = delta + self.ackermann_percent * (steer_left_true - delta)
            steer_right = delta + self.ackermann_percent * (steer_right_true - delta)
            steer_left = max(-self.max_servo_angle, min(steer_left, self.max_servo_angle))
            steer_right = max(-self.max_servo_angle, min(steer_right, self.max_servo_angle))
        else:
            steer_left = delta
            steer_right = delta

        # 6. Standstill Guard
        if abs(target_v) < 0.001:
            self.omega_ref = 0.0
            self.slip_state = SLIP_NORMAL
            self._slip_hold_timer = 0.0
            self.diff_split_gain = min(1.0, self.diff_split_gain + DIFF_RISE_RATE * safe_dt)
            return {
                "steer_left": steer_left,
                "steer_right": steer_right,
                "steer_rad": delta,
                "driver_steer_rad": self.driver_steer_rad,
                "delta_gyro_rad": 0.0,
                "v_FL": 0.0, "v_FR": 0.0, "v_RL": 0.0, "v_RR": 0.0,
                "omega_FL": 0.0, "omega_FR": 0.0, "omega_RL": 0.0, "omega_RR": 0.0,
                "omega_cmd": 0.0,
                "omega_ref": 0.0,
                "scale_factor": 1.0,
                "slip_state": SLIP_NORMAL,
                "yaw_error": 0.0
            }

        # 7. Grip-Capped, Hysteretic IMU Slip Classification
        yaw_error = gyro_z_rad_s - self.omega_ref
        raw_slip = SLIP_NORMAL

        if abs(target_v) >= V_SLIP_GATE:
            dyn_thresh = max(self.thresh_spin, 0.35 * abs(self.omega_ref))

            if self.omega_ref > 0.05 and (gyro_z_rad_s - self.omega_ref) > dyn_thresh:
                raw_slip = SLIP_OVERSTEER
            elif self.omega_ref < -0.05 and (self.omega_ref - gyro_z_rad_s) > dyn_thresh:
                raw_slip = SLIP_OVERSTEER
            elif abs(gyro_z_rad_s) > max(1.00, self.thresh_spin * 2.0) and (gyro_z_rad_s * self.omega_ref <= 0.0):
                raw_slip = SLIP_OVERSTEER
            elif (abs(self.driver_steer_rad) > 0.15 and abs(self.omega_ref) > 1.00 and
                  (gyro_z_rad_s * self.omega_ref >= 0.0) and abs(gyro_z_rad_s) < (0.40 * abs(self.omega_ref))):
                raw_slip = SLIP_UNDERSTEER

        if abs(target_v) < V_SLIP_GATE:
            self.slip_state = SLIP_NORMAL
            self._slip_hold_timer = 0.0
        elif raw_slip != SLIP_NORMAL:
            self.slip_state = raw_slip
            self._slip_hold_timer = SLIP_HOLD_TIME_S
        elif self._slip_hold_timer > 0.0:
            self._slip_hold_timer = max(0.0, self._slip_hold_timer - safe_dt)
        else:
            self.slip_state = SLIP_NORMAL

        target_diff_gain = 0.0 if self.slip_state == SLIP_OVERSTEER else 1.0
        if target_diff_gain < self.diff_split_gain:
            self.diff_split_gain = max(target_diff_gain, self.diff_split_gain - DIFF_FADE_RATE * safe_dt)
        else:
            self.diff_split_gain = min(target_diff_gain, self.diff_split_gain + DIFF_RISE_RATE * safe_dt)

        # 8. 4-Wheel Linear Speed Allocation (Forward & Reverse)
        half_Wr_omega_rear = omega_cmd * (self.W_r * 0.5) * self.diff_split_gain
        half_Wf_omega = omega_cmd * (self.W_f * 0.5)

        # Rear axle speeds from differential split (faded on oversteer)
        v_diff_RL = target_v - half_Wr_omega_rear
        v_diff_RR = target_v + half_Wr_omega_rear

        # Rear axle speeds: Ground-roll clamping ONLY during drift detection
        is_drift_active = is_drifting or (rear_alpha < 0.99)
        if is_drift_active and target_v >= 0.0 and v_ground_RL is not None and v_ground_RR is not None:
            v_RL = v_ground_RL + rear_alpha * max(0.0, v_diff_RL - v_ground_RL)
            v_RR = v_ground_RR + rear_alpha * max(0.0, v_diff_RR - v_ground_RR)
        else:
            v_RL = v_diff_RL
            v_RR = v_diff_RR

        # Front axle speeds (vector projection along steered wheel orientations)
        fl_x = target_v - half_Wf_omega
        fr_x = target_v + half_Wf_omega
        front_y = self.L * omega_cmd

        # Project hub velocities onto each wheel's individual heading angle
        v_FL = fl_x * math.cos(steer_left) + front_y * math.sin(steer_left)
        v_FR = fr_x * math.cos(steer_right) + front_y * math.sin(steer_right)

        # 9. Proportional Saturation Scaling (Preserve turning radius when near motor RPM limit)
        v_peak = max(abs(v_FL), abs(v_FR), abs(v_RL), abs(v_RR))
        scale = 1.0
        if v_peak > self.v_max:
            scale = self.v_max / v_peak
            v_FL *= scale
            v_FR *= scale
            v_RL *= scale
            v_RR *= scale

        # 10. Convert Linear Velocities (m/s) to Actuator Rotational Velocities (rad/s)
        max_omega = (self.v_max / self.R_w)
        omega_FL = max(-max_omega, min(max_omega, (v_FL / self.R_w) * self.motor_dir))
        omega_FR = max(-max_omega, min(max_omega, (v_FR / self.R_w) * self.motor_dir))
        omega_RL = max(-max_omega, min(max_omega, (v_RL / self.R_w) * self.motor_dir))
        omega_RR = max(-max_omega, min(max_omega, (v_RR / self.R_w) * self.motor_dir))

        return {
            "steer_left": steer_left,
            "steer_right": steer_right,
            "steer_rad": delta,
            "driver_steer_rad": self.driver_steer_rad,
            "delta_gyro_rad": delta_gyro,
            "v_FL": v_FL, "v_FR": v_FR, "v_RL": v_RL, "v_RR": v_RR,
            "omega_FL": omega_FL, "omega_FR": omega_FR,
            "omega_RL": omega_RL, "omega_RR": omega_RR,
            "omega_cmd": omega_cmd,
            "omega_ref": self.omega_ref,
            "scale_factor": scale,
            "slip_state": self.slip_state,
            "yaw_error": yaw_error
        }
