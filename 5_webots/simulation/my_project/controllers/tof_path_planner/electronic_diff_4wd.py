"""
================================================================================
  4WD ELECTRONIC DIFFERENTIAL & ACKERMANN KINEMATICS MODULE
  1:1 mirror of 3_esp32/ESP32_Main/src/electronic_diff.cpp (5-Layer 4WD Traction
  Control with V_trq decoupling, split yaw references, and Rear Torque Vectoring)
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

# Yaw reference & gyro assist (1:1 mirror of ESP32 electronic_diff.cpp)
GYRO_YAW_REF_TAU_S = 0.055                # 55 ms chassis + servo yaw-rate buildup lag
GYRO_YAW_REF_UNWIND_TAU_S = 0.020         # 20 ms fast unwind when straightening out
TV_YAW_REF_TAU_S = 0.012                  # 12 ms short-lag uncapped yaw target for Rear TV
YAW_ACCEL_LPF_TAU_S = 0.00531             # ~30 Hz 1st-order LPF on yaw accel
GYRO_DEADBAND_RAD_S = 0.28                # IMU yaw-rate noise deadband for servo countersteer
GYRO_KP_STEER = 0.105                     # Proportional gyro countersteer gain (rad / (rad/s))
GYRO_KD_STEER = 0.004                     # Derivative yaw damper gain (rad / (rad/s^2))
GYRO_KD_RECOVERY_SCALE = 0.15             # Attenuate D-term during spin recovery
GYRO_MAX_RATE_RAD_S = 12.0                # Countersteer slew cap (rad/s)
GYRO_FULL_SPEED_MPS = 0.60                # Speed where gyro assist reaches 100% authority

# Reactive slip telemetry latch timers
UNDERSTEER_CONFIRM_TIME_S = 0.065
OVERSTEER_CONFIRM_TIME_S = 0.035
SLIP_HOLD_TIME_S = 0.035

# Steering cap & asymmetric turn-in slew
STEER_GRIP_MARGIN = 1.25                  # Extra grip margin on speed-based steer cap
PEAK_FRONT_SLIP_RAD = 0.16                # ~9.2 deg peak front slip angle allowance
STEER_SLEW_V_LOW_MPS = 1.20               # Speed below which turn-in uses full servo slew rate
STEER_SLEW_V_HIGH_MPS = 3.80              # Speed where turn-in slew reaches minimum taper
STEER_SLEW_MIN_SCALE = 0.70               # Minimum turn-in servo slew scale at high speed

# Torque-envelope TCS, SVPWM headroom & Feedforward gains (GM3506 @ 12V, 5.6 Ohm)
VTRQ_FULL_V = 6.93                        # Linear SVPWM phase-peak voltage headroom (12V / sqrt(3))
EMERGENCY_CORNER_MU = 1.30                # High-mu emergency sanity ceiling; SL-MPC owns normal corner speeds
COLLISION_RESYNC_MARGIN_MPS = 0.85        # Wide collision resync margin
ENV_ACCEL_MAX_MPS2 = 11.28                # ~1.15 g max longitudinal accel ramp
ENV_DECEL_MAX_MPS2 = 11.28                # ~1.15 g max longitudinal braking ramp
FF_ACCEL_MAX_MPS2 = 7.50                  # ~0.76 g accel FF scale ceiling
FF_DECEL_MAX_MPS2 = 7.85                  # ~0.80 g brake FF scale ceiling
FF_TRACK_TAU_S = 0.12                     # Velocity error -> longitudinal accel FF horizon
OVERSPEED_FLOOR_RAD_S = 18.0              # ~0.50 m/s safety overspeed floor above v_est
UNDERSPEED_FLOOR_RAD_S = 16.0             # ~0.44 m/s ABS underspeed floor below v_est
SLIP_CEILING_RATIO = 0.15                 # 15% high-speed longitudinal slip allowance
ABS_MIN_SPEED_MPS = 0.30                  # Minimum speed for ABS wheel-lock floor
KA_FRONT = 0.61                           # 4WD launch FF gain (V / (m/s^2))
KA_REAR = 0.61                            # 4WD launch FF gain (V / (m/s^2))

# Rear Torque Vectoring & Axle Envelope defaults
TV_DEADBAND_RAD_S = 0.20                  # +/- 0.20 rad/s deadband on yaw rate error
DEFAULT_TV_KP_PSI = 4.0                   # Rear TV proportional slip bias ((rad/s wheel) / (rad/s yaw))
DEFAULT_TV_KD_PSI = 0.08                  # Rear TV yaw acceleration damping ((rad/s wheel) / (rad/s^2))
DEFAULT_TV_MAX_VFF = 1.65                 # Max rear TV feedforward torque voltage differential (V)
DEFAULT_OVERSTEER_MU_CAP = 1.20           # Oversteer reference grip cap for Shore A22 silicone
TV_SLIP_RATIO_MAX = 0.10                  # 10% wheel speed slip ceiling
TV_SLIP_FLOOR_RAD_S = 4.0                 # Low-speed TV slip floor (rad/s)
FRONT_TURN_MIN_FRAC = 0.15                # Reserve ~85% front friction circle at full steering lock

SLIP_NORMAL = "NORMAL"
SLIP_OVERSTEER = "OVERSTEER"
SLIP_UNDERSTEER = "UNDERSTEER"


class ElectronicDifferential4WD:
    """
    Computes 45% Ackermann steering angles (35 deg inner, 30 deg outer),
    split yaw references, continuous grip-capped PD gyro steering assist,
    Option A1 torque-envelope speed ramping, Rear Torque Vectoring (TV),
    and 4-wheel rotational speeds matching ESP32-S3 electronic_diff.cpp.
    """

    def __init__(
        self,
        wheelbase=WHEELBASE,
        track_rear=TRACK_REAR,
        track_front=TRACK_FRONT,
        track_kingpin=TRACK_KINGPIN,
        wheel_radius=WHEEL_RADIUS,
        max_steer=MAX_STEERING_RAD,
        max_servo_angle=MAX_SERVO_RAD,
        ackermann_percent=ACKERMANN_PERCENT,
        motor_dir=MOTOR_DIR,
        v_max=V_MAX_WHEEL,
        thresh_spin=THRESH_SPIN,
        max_servo_rate=SERVO_MAX_SPEED_RAD_S,
        v_slip_gate=V_SLIP_GATE,
    ):
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
        self.v_slip_gate = v_slip_gate

        # PTK9794 @ 8.4V: 1510 +/- 271 us over a 2000 us / 180 deg span -> +/- 24.39 deg horn throw
        self.max_horn_angle = (271.0 / 2000.0) * math.pi
        self._linkage_gain = math.sin(self.max_steer) / math.sin(self.max_horn_angle)

        self.gyro_assist_enabled = True
        self.rear_tv_enabled = True
        self.front_env_enabled = True
        self.tv_kp_psi = DEFAULT_TV_KP_PSI
        self.tv_kd_psi = DEFAULT_TV_KD_PSI
        self.tv_max_vff = DEFAULT_TV_MAX_VFF
        self.mu_os_cap = DEFAULT_OVERSTEER_MU_CAP

        self.reset()

    def reset(self) -> None:
        """Resets steering linkage, speed envelope, yaw references, derivative filter, and slip state."""
        self.current_horn_rad = 0.0
        self.current_steer_rad = 0.0
        self.driver_steer_rad = 0.0
        self._prev_delta_gyro_rad = 0.0
        self.rate_limited_speed = 0.0
        self.omega_ref = 0.0
        self.omega_tv_target = 0.0
        self.prev_gyro_z = 0.0
        self.gyro_z_dot_filt = 0.0
        self._has_prev_gyro = False
        self.slip_state = SLIP_NORMAL
        self._slip_hold_timer = 0.0
        self._understeer_confirm_timer = 0.0
        self._oversteer_confirm_timer = 0.0

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
        v_curr: Optional[float] = None,
        bypass_speed_envelope: bool = False,
    ) -> Dict[str, Any]:
        """
        Calculates actuator setpoints given desired longitudinal velocity and virtual steering angle.
        Mirrors ESP32-S3 ElectronicDifferential4WD continuous gyro assist, Option A1 speed envelope,
        and Rear Torque Vectoring.
        """
        safe_dt = dt if dt > 1e-5 else (1.0 / 120.0)
        safe_mu = max(0.10, float(mu))
        v_curr_val = float(v_curr) if v_curr is not None else self.rate_limited_speed

        # ---------------------------------------------------------------------
        # 1. Driver Steering Intent (Speed-Capped & Asymmetric Slew)
        # ---------------------------------------------------------------------
        target_delta = max(-self.max_steer, min(float(target_steer_rad), self.max_steer))

        v_for_steer = max(abs(v_curr_val), abs(self.rate_limited_speed))
        steer_cap_rad = self.max_steer
        if v_for_steer > 0.50:
            delta_grip = math.atan((STEER_GRIP_MARGIN * safe_mu * GRAVITY_G * self.L) / (v_for_steer * v_for_steer))
            steer_cap_rad = min(self.max_steer, delta_grip + PEAK_FRONT_SLIP_RAD)
        capped_target_delta = max(-steer_cap_rad, min(target_delta, steer_cap_rad))

        is_steer_unwinding = (abs(capped_target_delta) < abs(self.driver_steer_rad)) or (
            capped_target_delta * self.driver_steer_rad < 0.0
        )
        slew_scale = 1.0
        if not is_steer_unwinding and v_for_steer > STEER_SLEW_V_LOW_MPS:
            t = min(1.0, (v_for_steer - STEER_SLEW_V_LOW_MPS) / (STEER_SLEW_V_HIGH_MPS - STEER_SLEW_V_LOW_MPS))
            slew_scale = 1.0 - t * (1.0 - STEER_SLEW_MIN_SCALE)
        max_d_driver = self.max_servo_rate * slew_scale * safe_dt

        driver_diff = capped_target_delta - self.driver_steer_rad
        if abs(driver_diff) <= max_d_driver:
            self.driver_steer_rad = capped_target_delta
        else:
            self.driver_steer_rad += math.copysign(max_d_driver, driver_diff)
        self.driver_steer_rad = max(-self.max_steer, min(self.driver_steer_rad, self.max_steer))

        # ---------------------------------------------------------------------
        # 2. Split Yaw References (Option C2) & ~30 Hz LPF on Yaw Accel
        # ---------------------------------------------------------------------
        v_yaw = v_curr_val if abs(v_curr_val) > 0.05 else self.rate_limited_speed
        omega_kin_driver = (v_yaw * math.tan(self.driver_steer_rad)) / self.L

        # 2a. Short-lag (12 ms) uncapped kinematic yaw target for Rear TV turn-in
        alpha_tv_ref = safe_dt / (TV_YAW_REF_TAU_S + safe_dt)
        self.omega_tv_target += alpha_tv_ref * (omega_kin_driver - self.omega_tv_target)

        # 2b. Grip-capped (mu_os_cap ~ 1.20) & lagged reference for oversteer & gyro assist
        omega_grip_max = (self.mu_os_cap * GRAVITY_G) / max(abs(v_yaw), 0.50)
        omega_ref_target = max(-omega_grip_max, min(omega_grip_max, omega_kin_driver))
        is_unwinding = (abs(omega_ref_target) < abs(self.omega_ref)) or (omega_ref_target * self.omega_ref < 0.0)
        tau_ref = GYRO_YAW_REF_UNWIND_TAU_S if is_unwinding else GYRO_YAW_REF_TAU_S
        alpha_ref = safe_dt / (tau_ref + safe_dt)
        self.omega_ref += alpha_ref * (omega_ref_target - self.omega_ref)

        # 2c. ~30 Hz 1st-order LPF on yaw accel
        if self._has_prev_gyro:
            d_gyro_raw = (gyro_z_rad_s - self.prev_gyro_z) / safe_dt
            alpha_d = safe_dt / (YAW_ACCEL_LPF_TAU_S + safe_dt)
            self.gyro_z_dot_filt += alpha_d * (d_gyro_raw - self.gyro_z_dot_filt)
        self.prev_gyro_z = gyro_z_rad_s
        self._has_prev_gyro = True

        # ---------------------------------------------------------------------
        # 3. Coordinated Continuous PD Gyro-Assisted Steering (Option B1)
        # ---------------------------------------------------------------------
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

        speed_abs = abs(v_yaw)
        gyro_fade = 0.0
        if speed_abs > self.v_slip_gate:
            gyro_fade = min(1.0, (speed_abs - self.v_slip_gate) / max(0.05, GYRO_FULL_SPEED_MPS - self.v_slip_gate))
        speed_sign = 1.0 if v_yaw >= 0.0 else -1.0

        raw_delta_gyro = 0.0
        if self.gyro_assist_enabled and gyro_fade > 0.0 and abs(excess_yaw) > GYRO_DEADBAND_RAD_S:
            excess_db = math.copysign(abs(excess_yaw) - GYRO_DEADBAND_RAD_S, excess_yaw)
            d_weight = min(1.0, abs(excess_db) / 0.25)
            kd_scale = 1.0 if (excess_yaw * self.gyro_z_dot_filt >= 0.0) else GYRO_KD_RECOVERY_SCALE
            pd_raw = -(GYRO_KP_STEER * excess_db) - (d_weight * kd_scale * GYRO_KD_STEER * self.gyro_z_dot_filt)
            if excess_yaw > 0.0:
                pd_raw = min(0.0, pd_raw)
            else:
                pd_raw = max(0.0, pd_raw)
            raw_delta_gyro = gyro_fade * speed_sign * pd_raw

        max_d_gyro = GYRO_MAX_RATE_RAD_S * safe_dt
        gyro_diff = raw_delta_gyro - self._prev_delta_gyro_rad
        if abs(gyro_diff) <= max_d_gyro:
            self._prev_delta_gyro_rad = raw_delta_gyro
        else:
            self._prev_delta_gyro_rad += math.copysign(max_d_gyro, gyro_diff)
        delta_gyro = self._prev_delta_gyro_rad

        # Servo horn slew rate limit (22.0 rad/s) through non-linear tie-rod linkage
        stabilized_target = max(-self.max_steer, min(self.max_steer, self.driver_steer_rad + delta_gyro))
        sin_horn_target = max(-1.0, min(1.0, math.sin(stabilized_target) / self._linkage_gain))
        target_horn = math.asin(sin_horn_target)
        max_d_horn = self.max_servo_rate * safe_dt
        horn_diff = target_horn - self.current_horn_rad
        if abs(horn_diff) <= max_d_horn:
            self.current_horn_rad = target_horn
        else:
            self.current_horn_rad += math.copysign(max_d_horn, horn_diff)

        sin_delta = max(-1.0, min(1.0, self._linkage_gain * math.sin(self.current_horn_rad)))
        self.current_steer_rad = max(-self.max_steer, min(self.max_steer, math.asin(sin_delta)))
        delta = self.current_steer_rad

        # ---------------------------------------------------------------------
        # 4. Torque-Envelope Speed Target, Overspeed Ceiling & V_trq,ff (Option A1)
        # ---------------------------------------------------------------------
        effective_target_v = float(target_v)
        if bypass_speed_envelope:
            self.rate_limited_speed = effective_target_v
        else:
            # Emergency-only high-mu sanity ceiling (1.30 g); SL-MPC owns normal corner speeds
            steer_for_cap = max(abs(self.driver_steer_rad), abs(capped_target_delta))
            tan_steer_cap = abs(math.tan(steer_for_cap))
            if tan_steer_cap > 0.05:
                v_emerg = max(0.80, math.sqrt((EMERGENCY_CORNER_MU * GRAVITY_G * self.L) / tan_steer_cap))
                if effective_target_v > 0.0:
                    effective_target_v = min(effective_target_v, v_emerg)
                elif effective_target_v < 0.0:
                    effective_target_v = max(effective_target_v, -v_emerg)

            # Directional collision resync (never snap across zero away from effective_target_v)
            if self.rate_limited_speed > (v_curr_val + COLLISION_RESYNC_MARGIN_MPS):
                self.rate_limited_speed = max(min(effective_target_v, 0.0), v_curr_val)
            elif self.rate_limited_speed < (v_curr_val - COLLISION_RESYNC_MARGIN_MPS):
                self.rate_limited_speed = min(max(effective_target_v, 0.0), v_curr_val)

            if effective_target_v <= v_curr_val:
                if self.rate_limited_speed > v_curr_val:
                    self.rate_limited_speed = max(effective_target_v, v_curr_val)
            else:
                if self.rate_limited_speed < v_curr_val:
                    self.rate_limited_speed = min(effective_target_v, v_curr_val)

            prev_v = self.rate_limited_speed
            speed_err = effective_target_v - self.rate_limited_speed
            is_accel = (self.rate_limited_speed >= 0.0 and speed_err > 0.0) or (
                self.rate_limited_speed <= 0.0 and speed_err < 0.0
            )
            max_dv = (ENV_ACCEL_MAX_MPS2 if is_accel else ENV_DECEL_MAX_MPS2) * safe_dt
            if abs(speed_err) <= max_dv:
                self.rate_limited_speed = effective_target_v
            else:
                self.rate_limited_speed += math.copysign(max_dv, speed_err)

            # Reasonable overspeed / monotonic ABS underspeed envelope
            max_overspeed = max(OVERSPEED_FLOOR_RAD_S * self.R_w, SLIP_CEILING_RATIO * abs(v_curr_val))
            max_underspeed = max(UNDERSPEED_FLOOR_RAD_S * self.R_w, SLIP_CEILING_RATIO * abs(v_curr_val))
            if self.rate_limited_speed > 0.0:
                self.rate_limited_speed = min(self.rate_limited_speed, max(0.0, v_curr_val) + max_overspeed)
            elif self.rate_limited_speed < 0.0:
                self.rate_limited_speed = max(self.rate_limited_speed, min(0.0, v_curr_val) - max_overspeed)

            if v_curr_val > ABS_MIN_SPEED_MPS and effective_target_v < v_curr_val:
                abs_floor = min(prev_v, v_curr_val - max_underspeed)
                self.rate_limited_speed = max(self.rate_limited_speed, abs_floor)
            elif v_curr_val < -ABS_MIN_SPEED_MPS and effective_target_v > v_curr_val:
                abs_ceil = max(prev_v, v_curr_val + max_underspeed)
                self.rate_limited_speed = min(self.rate_limited_speed, abs_ceil)

        # Longitudinal acceleration feedforward (V_trq,ff = Ka * a_x_cmd)
        v_ff_target = self.rate_limited_speed if (self.rate_limited_speed < -0.05 and effective_target_v >= 0.0) else effective_target_v
        v_err_long = v_ff_target - v_curr_val
        a_x_cmd = 0.0
        if abs(v_err_long) > 0.05:
            a_x_cmd = max(-FF_DECEL_MAX_MPS2, min(FF_ACCEL_MAX_MPS2, v_err_long / FF_TRACK_TAU_S))
        v_ff_front_base = KA_FRONT * a_x_cmd
        v_ff_rear_base = KA_REAR * a_x_cmd

        # ---------------------------------------------------------------------
        # 5. Kinematic Yaw Rate & 45% Geometric Ackermann Knuckle Headings
        # ---------------------------------------------------------------------
        sin_d = math.sin(delta)
        cos_d = math.cos(delta)
        tan_d = math.tan(delta)
        omega_cmd = (self.rate_limited_speed * tan_d) / self.L

        if abs(tan_d) > 1e-4:
            half_Wk_sin = (self.W_k * 0.5) * sin_d
            L_cos = self.L * cos_d
            L_sin = self.L * sin_d
            steer_left_true = math.atan2(L_sin, L_cos - half_Wk_sin)
            steer_right_true = math.atan2(L_sin, L_cos + half_Wk_sin)
            steer_left = max(-self.max_servo_angle, min(self.max_servo_angle, delta + self.ackermann_percent * (steer_left_true - delta)))
            steer_right = max(-self.max_servo_angle, min(self.max_servo_angle, delta + self.ackermann_percent * (steer_right_true - delta)))
        else:
            steer_left = delta
            steer_right = delta

        # ---------------------------------------------------------------------
        # 6. Standstill Guard
        # ---------------------------------------------------------------------
        if abs(self.rate_limited_speed) < 0.001 and abs(v_curr_val) < 0.05:
            self.omega_ref = 0.0
            self.omega_tv_target = 0.0
            self._prev_delta_gyro_rad = 0.0
            self.slip_state = SLIP_NORMAL
            self._slip_hold_timer = 0.0
            self._understeer_confirm_timer = 0.0
            self._oversteer_confirm_timer = 0.0
            return {
                "steer_left": steer_left,
                "steer_right": steer_right,
                "steer_rad": delta,
                "driver_steer_rad": self.driver_steer_rad,
                "delta_gyro_rad": 0.0,
                "rate_limited_v": 0.0,
                "v_FL": 0.0, "v_FR": 0.0, "v_RL": 0.0, "v_RR": 0.0,
                "omega_FL": 0.0, "omega_FR": 0.0, "omega_RL": 0.0, "omega_RR": 0.0,
                "vtrq_min_FL": 0.0, "vtrq_max_FL": 0.0, "vtrq_ff_FL": 0.0,
                "vtrq_min_FR": 0.0, "vtrq_max_FR": 0.0, "vtrq_ff_FR": 0.0,
                "vtrq_min_RL": 0.0, "vtrq_max_RL": 0.0, "vtrq_ff_RL": 0.0,
                "vtrq_min_RR": 0.0, "vtrq_max_RR": 0.0, "vtrq_ff_RR": 0.0,
                "omega_cmd": 0.0,
                "omega_ref": 0.0,
                "omega_tv_target": 0.0,
                "delta_omega_tv": 0.0,
                "delta_vtrq_tv": 0.0,
                "scale_factor": 1.0,
                "slip_state": SLIP_NORMAL,
                "yaw_error": 0.0,
                "is_countersteering": False,
            }

        # ---------------------------------------------------------------------
        # 7. Reverse Guard (Symmetric 4WD kinematic allocation, zero TV)
        # ---------------------------------------------------------------------
        yaw_error = gyro_z_rad_s - self.omega_ref
        if effective_target_v < -0.05 or (self.rate_limited_speed < -0.05 and v_curr_val < 0.05):
            self.slip_state = SLIP_NORMAL
            half_Wr_omega = omega_cmd * (self.W_r * 0.5)
            half_Wf_omega = omega_cmd * (self.W_f * 0.5)
            v_RL = self.rate_limited_speed - half_Wr_omega
            v_RR = self.rate_limited_speed + half_Wr_omega

            fl_x = self.rate_limited_speed - half_Wf_omega
            fr_x = self.rate_limited_speed + half_Wf_omega
            front_y = self.L * omega_cmd
            v_FL = fl_x * math.cos(steer_left) + front_y * math.sin(steer_left)
            v_FR = fr_x * math.cos(steer_right) + front_y * math.sin(steer_right)

            v_peak = max(abs(v_FL), abs(v_FR), abs(v_RL), abs(v_RR))
            scale = 1.0
            if v_peak > self.v_max:
                scale = self.v_max / v_peak
                v_FL *= scale
                v_FR *= scale
                v_RL *= scale
                v_RR *= scale

            max_omega = self.v_max / self.R_w
            omega_FL = max(-max_omega, min(max_omega, (v_FL / self.R_w) * self.motor_dir))
            omega_FR = max(-max_omega, min(max_omega, (v_FR / self.R_w) * self.motor_dir))
            omega_RL = max(-max_omega, min(max_omega, (v_RL / self.R_w) * self.motor_dir))
            omega_RR = max(-max_omega, min(max_omega, (v_RR / self.R_w) * self.motor_dir))
            v_ff_front_clamped = max(-VTRQ_FULL_V, min(VTRQ_FULL_V, v_ff_front_base))
            v_ff_rear_clamped = max(-VTRQ_FULL_V, min(VTRQ_FULL_V, v_ff_rear_base))

            return {
                "steer_left": steer_left,
                "steer_right": steer_right,
                "steer_rad": delta,
                "driver_steer_rad": self.driver_steer_rad,
                "delta_gyro_rad": delta_gyro,
                "rate_limited_v": self.rate_limited_speed,
                "v_FL": v_FL, "v_FR": v_FR, "v_RL": v_RL, "v_RR": v_RR,
                "omega_FL": omega_FL, "omega_FR": omega_FR,
                "omega_RL": omega_RL, "omega_RR": omega_RR,
                "vtrq_min_FL": -VTRQ_FULL_V, "vtrq_max_FL": VTRQ_FULL_V, "vtrq_ff_FL": v_ff_front_clamped,
                "vtrq_min_FR": -VTRQ_FULL_V, "vtrq_max_FR": VTRQ_FULL_V, "vtrq_ff_FR": v_ff_front_clamped,
                "vtrq_min_RL": -VTRQ_FULL_V, "vtrq_max_RL": VTRQ_FULL_V, "vtrq_ff_RL": v_ff_rear_clamped,
                "vtrq_min_RR": -VTRQ_FULL_V, "vtrq_max_RR": VTRQ_FULL_V, "vtrq_ff_RR": v_ff_rear_clamped,
                "omega_cmd": omega_cmd,
                "omega_ref": self.omega_ref,
                "omega_tv_target": self.omega_tv_target,
                "delta_omega_tv": 0.0,
                "delta_vtrq_tv": 0.0,
                "scale_factor": scale,
                "slip_state": SLIP_NORMAL,
                "yaw_error": yaw_error,
                "is_countersteering": False,
            }

        # ---------------------------------------------------------------------
        # 8. Cornering State Classification (5-State Model + Hysteretic Telemetry)
        # ---------------------------------------------------------------------
        os_frac = 0.0
        us_frac = 0.0
        raw_slip = SLIP_NORMAL

        if speed_abs >= self.v_slip_gate:
            os_frac = max(0.0, min(1.0, (abs(excess_yaw) - 0.25) / 0.95))

            if os_frac <= 0.05 and abs(self.driver_steer_rad) > 0.08 and abs(self.omega_tv_target) > 0.60:
                turn_sign = 1.0 if self.driver_steer_rad >= 0.0 else -1.0
                us_deficit = (self.omega_tv_target - gyro_z_rad_s) * turn_sign
                us_frac = max(0.0, min(1.0, (us_deficit - 0.20) / 1.80))

            dyn_thresh = max(max(self.thresh_spin * 2.2, 0.95), 0.40 * abs(self.omega_ref))
            if self.omega_ref > 0.10 and (gyro_z_rad_s - self.omega_ref) > dyn_thresh:
                raw_slip = SLIP_OVERSTEER
            elif self.omega_ref < -0.10 and (self.omega_ref - gyro_z_rad_s) > dyn_thresh:
                raw_slip = SLIP_OVERSTEER
            elif abs(gyro_z_rad_s) > max(1.35, self.thresh_spin * 3.0) and (gyro_z_rad_s * self.omega_ref <= 0.0):
                raw_slip = SLIP_OVERSTEER
            elif us_frac > 0.45:
                raw_slip = SLIP_UNDERSTEER

        if speed_abs < self.v_slip_gate:
            self.slip_state = SLIP_NORMAL
            self._slip_hold_timer = 0.0
            self._understeer_confirm_timer = 0.0
            self._oversteer_confirm_timer = 0.0
        elif raw_slip == SLIP_OVERSTEER:
            self._oversteer_confirm_timer += safe_dt
            self._understeer_confirm_timer = 0.0
            if self._oversteer_confirm_timer >= OVERSTEER_CONFIRM_TIME_S:
                self.slip_state = SLIP_OVERSTEER
                self._slip_hold_timer = SLIP_HOLD_TIME_S
        elif raw_slip == SLIP_UNDERSTEER:
            self._understeer_confirm_timer += safe_dt
            self._oversteer_confirm_timer = 0.0
            if self._understeer_confirm_timer >= UNDERSTEER_CONFIRM_TIME_S:
                self.slip_state = SLIP_UNDERSTEER
                self._slip_hold_timer = SLIP_HOLD_TIME_S
        else:
            self._understeer_confirm_timer = 0.0
            self._oversteer_confirm_timer = 0.0
            if self._slip_hold_timer > 0.0:
                self._slip_hold_timer = max(0.0, self._slip_hold_timer - safe_dt)
            else:
                self.slip_state = SLIP_NORMAL

        is_os = os_frac > 0.05
        is_countersteering = is_os and ((delta * gyro_z_rad_s) <= 0.03 * abs(gyro_z_rad_s))

        # ---------------------------------------------------------------------
        # 9. Front Axle Envelope (Zero L/R Vectoring + Steering-Coupled V_trq)
        # ---------------------------------------------------------------------
        steer_norm = max(0.0, min(1.0, abs(delta) / max(1e-4, self.max_steer)))
        front_steer_scale = FRONT_TURN_MIN_FRAC + (1.0 - FRONT_TURN_MIN_FRAC) * (1.0 - steer_norm) ** 2
        is_braking = (effective_target_v < v_curr_val - 0.10) and (v_curr_val > 0.20)

        v_f_max = (VTRQ_FULL_V * front_steer_scale) if self.front_env_enabled else VTRQ_FULL_V
        v_f_min = (-VTRQ_FULL_V * max(0.35, front_steer_scale)) if is_braking else 0.0
        v_f_ff = (v_ff_front_base * front_steer_scale) if (a_x_cmd > 0.0 and self.front_env_enabled) else v_ff_front_base

        if is_countersteering:
            v_f_max = VTRQ_FULL_V * (0.65 + 0.35 * os_frac)
            v_f_min = 0.0
            v_f_ff = max(v_ff_front_base, 1.5 + 2.5 * os_frac)
        elif is_os:
            v_f_max = min(v_f_max, 0.20 + 0.80 * (1.0 - os_frac))
            v_f_min = 0.0
            v_f_ff = 0.0
        elif us_frac > 0.0 and self.front_env_enabled:
            v_f_max *= (1.0 - 0.85 * us_frac)
            v_f_min = 0.0
            v_f_ff *= (1.0 - us_frac)

        v_f_ff = max(v_f_min, min(v_f_max, v_f_ff))

        # ---------------------------------------------------------------------
        # 10. Rear Torque Vectoring (Bounded Slip Bias + Coordinated Yaw Damping)
        # ---------------------------------------------------------------------
        delta_omega_tv = 0.0
        delta_vtrq_tv = 0.0
        if self.rear_tv_enabled and speed_abs >= self.v_slip_gate:
            e_psi = (-excess_yaw) if is_os else (self.omega_tv_target - gyro_z_rad_s)
            e_db = 0.0
            if abs(e_psi) > TV_DEADBAND_RAD_S:
                e_db = math.copysign(abs(e_psi) - TV_DEADBAND_RAD_S, e_psi)

            tv_kd_scale = max(0.0, 1.0 - abs(delta_gyro) / 0.06)
            delta_omega_max = max(TV_SLIP_FLOOR_RAD_S, TV_SLIP_RATIO_MAX * (speed_abs / self.R_w))
            d_term_tv = (
                (tv_kd_scale * self.tv_kd_psi * self.gyro_z_dot_filt)
                if (abs(e_psi) > 0.15 or abs(self.gyro_z_dot_filt) > 3.0)
                else 0.0
            )
            delta_omega_tv = max(-delta_omega_max, min(delta_omega_max, self.tv_kp_psi * e_db - d_term_tv))
            delta_vtrq_tv = (delta_omega_tv / max(1.0, delta_omega_max)) * self.tv_max_vff

        # ---------------------------------------------------------------------
        # 11. Asymmetric Rear V_trq Bounds Across Cornering States
        # ---------------------------------------------------------------------
        if is_os:
            spin_left = (gyro_z_rad_s > 0.0) if abs(gyro_z_rad_s) > 0.05 else (excess_yaw > 0.0)
            outer_max = VTRQ_FULL_V * max(0.0, 1.0 - 1.4 * os_frac)
            inner_max = (
                (VTRQ_FULL_V * max(0.10, 0.35 * (1.0 - 0.6 * os_frac)))
                if is_countersteering
                else (VTRQ_FULL_V * 0.45)
            )
            if spin_left:
                v_min_RR, v_max_RR, v_ff_RR = 0.0, outer_max, 0.0
                v_min_RL, v_max_RL = 0.0, inner_max
                v_ff_RL = 0.0 if is_countersteering else max(0.0, min(inner_max, v_ff_rear_base - delta_vtrq_tv))
            else:
                v_min_RL, v_max_RL, v_ff_RL = 0.0, outer_max, 0.0
                v_min_RR, v_max_RR = 0.0, inner_max
                v_ff_RR = 0.0 if is_countersteering else max(0.0, min(inner_max, v_ff_rear_base + delta_vtrq_tv))
        else:
            lat_sig = v_yaw * gyro_z_rad_s + 0.35 * v_yaw * self.omega_tv_target
            lat_load = max(0.0, min(1.0, abs(v_yaw * gyro_z_rad_s) / max(1.0, safe_mu * GRAVITY_G)))
            outer_max = VTRQ_FULL_V
            outer_min = -VTRQ_FULL_V if is_braking else -1.0
            inner_max = VTRQ_FULL_V * (1.0 - 0.60 * lat_load)
            inner_min = (-VTRQ_FULL_V * (1.0 - 0.50 * lat_load)) if is_braking else 0.0
            if lat_sig >= 0.0:
                v_min_RL, v_max_RL = inner_min, inner_max
                v_min_RR, v_max_RR = outer_min, outer_max
            else:
                v_min_RL, v_max_RL = outer_min, outer_max
                v_min_RR, v_max_RR = inner_min, inner_max
            v_ff_RL = max(v_min_RL, min(v_max_RL, v_ff_rear_base - delta_vtrq_tv))
            v_ff_RR = max(v_min_RR, min(v_max_RR, v_ff_rear_base + delta_vtrq_tv))

        # ---------------------------------------------------------------------
        # 12. 4-Wheel Speed Allocation + Bounded Rear TV Slip Bias
        # ---------------------------------------------------------------------
        half_Wr_omega_rear = omega_cmd * (self.W_r * 0.5)
        half_Wf_omega = omega_cmd * (self.W_f * 0.5)

        v_tv_bias = delta_omega_tv * self.R_w
        v_diff_RL = (self.rate_limited_speed - half_Wr_omega_rear) - v_tv_bias
        v_diff_RR = (self.rate_limited_speed + half_Wr_omega_rear) + v_tv_bias

        is_drift_active = is_drifting or (rear_alpha < 0.99)
        if is_drift_active and self.rate_limited_speed >= 0.0 and v_ground_RL is not None and v_ground_RR is not None:
            v_RL = v_ground_RL + rear_alpha * max(0.0, v_diff_RL - v_ground_RL)
            v_RR = v_ground_RR + rear_alpha * max(0.0, v_diff_RR - v_ground_RR)
        else:
            v_RL = v_diff_RL
            v_RR = v_diff_RR

        fl_x = self.rate_limited_speed - half_Wf_omega
        fr_x = self.rate_limited_speed + half_Wf_omega
        front_y = self.L * omega_cmd
        v_FL = fl_x * math.cos(steer_left) + front_y * math.sin(steer_left)
        v_FR = fr_x * math.cos(steer_right) + front_y * math.sin(steer_right)

        v_peak = max(abs(v_FL), abs(v_FR), abs(v_RL), abs(v_RR))
        scale = 1.0
        if v_peak > self.v_max:
            scale = self.v_max / v_peak
            v_FL *= scale
            v_FR *= scale
            v_RL *= scale
            v_RR *= scale

        max_omega = self.v_max / self.R_w
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
            "rate_limited_v": self.rate_limited_speed,
            "v_FL": v_FL, "v_FR": v_FR, "v_RL": v_RL, "v_RR": v_RR,
            "omega_FL": omega_FL, "omega_FR": omega_FR,
            "omega_RL": omega_RL, "omega_RR": omega_RR,
            "vtrq_min_FL": v_f_min, "vtrq_max_FL": v_f_max, "vtrq_ff_FL": v_f_ff,
            "vtrq_min_FR": v_f_min, "vtrq_max_FR": v_f_max, "vtrq_ff_FR": v_f_ff,
            "vtrq_min_RL": v_min_RL, "vtrq_max_RL": v_max_RL, "vtrq_ff_RL": v_ff_RL,
            "vtrq_min_RR": v_min_RR, "vtrq_max_RR": v_max_RR, "vtrq_ff_RR": v_ff_RR,
            "omega_cmd": omega_cmd,
            "omega_ref": self.omega_ref,
            "omega_tv_target": self.omega_tv_target,
            "delta_omega_tv": delta_omega_tv,
            "delta_vtrq_tv": delta_vtrq_tv,
            "scale_factor": scale,
            "slip_state": self.slip_state,
            "yaw_error": yaw_error,
            "is_countersteering": is_countersteering,
        }
