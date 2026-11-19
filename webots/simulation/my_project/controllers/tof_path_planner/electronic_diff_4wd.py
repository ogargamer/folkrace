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

GYRO_YAW_REF_TAU_S = 0.055      # 55 ms chassis + servo yaw-rate buildup lag
GYRO_YAW_REF_UNWIND_TAU_S = 0.020  # 20 ms fast unwind when straightening out
GYRO_YAW_DOT_TAU_S = 0.032      # 32 ms 1st-order derivative low-pass filter
GYRO_DEADBAND_RAD_S = 0.28      # IMU yaw-rate noise deadband
GYRO_KP_STEER = 0.105           # Proportional gyro countersteer gain (rad / (rad/s))
GYRO_KD_STEER = 0.004           # Derivative yaw damper gain (rad / (rad/s^2))
GYRO_KD_RECOVERY_SCALE = 0.15   # Attenuate D-term during spin recovery so it won't unwind countersteer
GYRO_MAX_RATE_RAD_S = 12.0      # Countersteer slew cap (rad/s) to reject landing/shock spikes
GYRO_FULL_SPEED_MPS = 0.60      # Speed where gyro assist reaches 100% authority
UNDERSTEER_CONFIRM_TIME_S = 0.065  # Require 65 ms sustained plow before throttling
OVERSTEER_CONFIRM_TIME_S = 0.035   # Require 35 ms sustained breakaway before latching
SLIP_HOLD_TIME_S = 0.035        # 35 ms hysteresis hold latch for slip telemetry/interventions
DIFF_FADE_RATE = 12.0           # Fast fade-out of rear diff split on oversteer (1/s)
DIFF_RISE_RATE = 4.0            # Smooth recovery of rear diff split (1/s)
UNDERSTEER_SPEED_SCALE = 0.85   # Fixed cap fraction of entry speed on sustained understeer
OVERSTEER_SPEED_SCALE = 0.85    # 15% throttle lift on oversteer (fixed entry cap, no slam braking)
OVERSTEER_ACCEL_SCALE = 0.25    # Cut forward acceleration torque during oversteer
OVERSTEER_DECEL_SCALE = 0.60    # Keep oversteer decel smooth to avoid rear weight transfer
UNDERSTEER_ACCEL_SCALE = 0.30   # Cut forward acceleration torque during front plow
SLIP_SPEED_FLOOR_MPS = 0.60     # Minimum cornering speed floor during slip throttle cap

# Proactive Traction Prevention Constants (1:1 mirror of ESP32 electronic_diff.cpp)
PROACTIVE_CORNER_MU_SCALE = 0.94  # Feedforward corner speed mu scale (~97% apex speed)
STEER_GRIP_MARGIN = 1.25          # Extra grip margin on speed-based steer cap
PEAK_FRONT_SLIP_RAD = 0.16        # ~9.2 deg peak front slip angle allowance
STEER_SLEW_V_LOW_MPS = 1.20       # Speed below which turn-in uses full servo slew rate
STEER_SLEW_V_HIGH_MPS = 3.80      # Speed where turn-in slew reaches minimum taper
STEER_SLEW_MIN_SCALE = 0.70       # Minimum turn-in servo slew scale at high speed
ACCEL_MIN_LONG_FRACTION = 0.30    # Minimum longitudinal accel fraction in hard cornering
BRAKE_MIN_LONG_FRACTION = 0.50    # Minimum longitudinal braking fraction in hard cornering
GEOM_STEER_LAT_WEIGHT = 0.35      # Baseline geometric steer utilization weight
EARLY_US_YAW_RATIO_HIGH = 0.60    # Yaw ratio where incipient understeer modulation starts
EARLY_US_YAW_RATIO_LOW = 0.38     # Yaw ratio where incipient understeer modulation saturates
EARLY_US_FILT_TAU_S = 0.030       # 30 ms LPF on incipient understeer & observed lateral accel
EARLY_US_MIN_SPEED_MPS = 0.65     # Minimum speed to evaluate incipient front saturation
EARLY_US_MIN_STEER_RAD = 0.12     # Minimum steering angle to evaluate incipient front saturation
EARLY_US_MIN_OMEGA_REF = 1.00     # Minimum reference yaw rate for incipient front saturation
EARLY_US_ACCEL_CUT = 0.30         # Max proactive accel cut (30%) at full incipient understeer
EARLY_US_SPEED_FLOOR_FRAC = 0.70  # Floor on observed-grip corner cap relative to nominal v_proactive
EARLY_OS_START_RAD_S = 0.65       # Excess yaw (with lead) where incipient oversteer starts
EARLY_OS_FULL_RAD_S = 1.25        # Excess yaw (with lead) where incipient oversteer reaches 1.0
EARLY_OS_YAW_DOT_LEAD_S = 0.025   # 25 ms yaw-acceleration lead for snap breakaway
EARLY_OS_RISE_TAU_S = 0.007       # 7 ms rise LPF to reject single-tick kerb/chatter spikes
EARLY_OS_DECAY_TAU_S = 0.025      # 25 ms smooth decay when incipient oversteer subsides
EARLY_OS_ACCEL_CUT = 0.35         # Max proactive accel cut (35%) at full incipient oversteer
EARLY_OS_DECEL_CUT = 0.35         # Max proactive decel softening (35%) to prevent rear snap
SLIP_RATIO_ACCEL_MAX = 0.15       # Peak tire longitudinal slip ratio cap (15%)
SLIP_RATIO_MIN_CORNER = 0.04      # Tightened cornering longitudinal slip ratio floor (4%)
MIN_SLIP_LEAD_MPS = 0.12          # Absolute slip lead floor (m/s) for crisp low-speed launches & exits
ABS_MIN_SPEED_MPS = 0.30          # Minimum speed for ABS wheel-lock floor

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
        self._prev_delta_gyro_rad = 0.0
        self.omega_ref = 0.0
        self.prev_gyro_z = 0.0
        self.gyro_z_dot_filt = 0.0
        self._has_prev_gyro = False
        self.slip_state = SLIP_NORMAL
        self._slip_hold_timer = 0.0
        self._understeer_confirm_timer = 0.0
        self._oversteer_confirm_timer = 0.0
        self._slip_entry_speed = 0.0
        self.diff_split_gain = 1.0
        self.early_us_filt = 0.0
        self.early_os = 0.0
        self._a_lat_obs_filt = 0.0

    def _compute_capped_target_delta(self, target_steer_rad: float, v_for_steer: float, safe_mu: float) -> float:
        """Caps driver steering command at high speed to kinematic grip limit plus peak front slip allowance."""
        target_delta = max(-self.max_steer, min(float(target_steer_rad), self.max_steer))
        steer_cap_rad = self.max_steer
        if v_for_steer > 0.50:
            delta_grip = math.atan((STEER_GRIP_MARGIN * safe_mu * GRAVITY_G * self.L) / (v_for_steer * v_for_steer))
            steer_cap_rad = min(self.max_steer, delta_grip + PEAK_FRONT_SLIP_RAD)
        return max(-steer_cap_rad, min(target_delta, steer_cap_rad))

    def apply_traction_speed_cap(
        self,
        target_v: float,
        current_v: float,
        mu: float = TIRE_MU,
        target_steer_rad: Optional[float] = None,
        v_curr: Optional[float] = None,
    ) -> float:
        """Caps commanded velocity proactively by corner grip and reactively when slip is confirmed."""
        safe_mu = max(0.10, float(mu))
        effective_v = float(target_v)
        v_for_steer = abs(float(current_v))
        if v_curr is not None:
            v_for_steer = max(abs(float(v_curr)), v_for_steer)

        steer_for_cap = abs(self.driver_steer_rad)
        if target_steer_rad is not None:
            capped_target_delta = self._compute_capped_target_delta(target_steer_rad, v_for_steer, safe_mu)
            steer_for_cap = max(steer_for_cap, abs(capped_target_delta))

        tan_steer_cap = abs(math.tan(steer_for_cap))
        v_cap_total = 1e6
        if tan_steer_cap > 0.05:
            v_proactive = math.sqrt((PROACTIVE_CORNER_MU_SCALE * safe_mu * GRAVITY_G * self.L) / tan_steer_cap)
            if self.early_us_filt > 0.0:
                v_floor = max(SLIP_SPEED_FLOOR_MPS, EARLY_US_SPEED_FLOOR_FRAC * v_proactive)
                v_obs_raw = math.sqrt((max(0.50, self._a_lat_obs_filt) * self.L) / tan_steer_cap)
                v_obs_grip = min(v_proactive, max(v_floor, v_obs_raw))
                v_proactive += self.early_us_filt * (v_obs_grip - v_proactive)
            v_cap_total = max(SLIP_SPEED_FLOOR_MPS, v_proactive)

        entry_v = self._slip_entry_speed if self._slip_entry_speed > 0.0 else abs(float(current_v))
        if self.slip_state == SLIP_UNDERSTEER:
            tan_driver = max(0.05, abs(math.tan(self.driver_steer_rad)))
            v_corner_max = math.sqrt((safe_mu * GRAVITY_G * self.L) / tan_driver)
            v_us_cap = max(SLIP_SPEED_FLOOR_MPS, min(v_corner_max, UNDERSTEER_SPEED_SCALE * entry_v))
            v_cap_total = min(v_cap_total, v_us_cap)
        elif self.slip_state == SLIP_OVERSTEER:
            v_os_cap = max(SLIP_SPEED_FLOOR_MPS, OVERSTEER_SPEED_SCALE * entry_v)
            v_cap_total = min(v_cap_total, v_os_cap)

        if v_cap_total < 1e5:
            if effective_v > 0.0:
                effective_v = min(effective_v, v_cap_total)
            elif effective_v < 0.0:
                effective_v = max(effective_v, -v_cap_total)
        return effective_v

    def compute_friction_circle_budgets(
        self,
        v_curr: float,
        rate_limited_v: float,
        gyro_z_rad_s: float,
        steer_rad: float,
        mu: float = TIRE_MU,
    ) -> tuple:
        """Returns (accel_long_frac, brake_long_frac) from true combined-slip lateral utilization."""
        safe_mu = max(0.10, float(mu))
        v_ref_lat = max(abs(float(v_curr)), abs(float(rate_limited_v)))
        a_lat_meas = abs(float(v_curr) * float(gyro_z_rad_s))
        omega_pred = (v_ref_lat * math.tan(float(steer_rad))) / self.L
        a_lat_pred = abs(v_ref_lat * omega_pred)
        a_lat_eff = max(a_lat_meas, a_lat_pred)
        mu_g = safe_mu * GRAVITY_G
        lat_util_dyn = a_lat_eff / max(0.50, mu_g)
        lat_util_geom = GEOM_STEER_LAT_WEIGHT * abs(math.sin(float(steer_rad)))
        lat_util = min(0.96, max(lat_util_dyn, lat_util_geom))
        raw_long_frac = math.sqrt(max(0.0, 1.0 - lat_util * lat_util))
        return (
            max(ACCEL_MIN_LONG_FRACTION, raw_long_frac),
            max(BRAKE_MIN_LONG_FRACTION, raw_long_frac),
        )

    def get_slip_accel_decel_scales(self) -> tuple:
        """Returns (accel_scale, decel_scale) combining proactive early-slip and reactive slip states."""
        proactive_accel = min(
            1.0 - EARLY_US_ACCEL_CUT * self.early_us_filt,
            1.0 - EARLY_OS_ACCEL_CUT * self.early_os,
        )
        proactive_decel = 1.0 - EARLY_OS_DECEL_CUT * self.early_os
        reactive_accel = 1.0
        reactive_decel = 1.0
        if self.slip_state == SLIP_OVERSTEER:
            reactive_accel = OVERSTEER_ACCEL_SCALE
            reactive_decel = OVERSTEER_DECEL_SCALE
        elif self.slip_state == SLIP_UNDERSTEER:
            reactive_accel = UNDERSTEER_ACCEL_SCALE
        return (min(proactive_accel, reactive_accel), min(proactive_decel, reactive_decel))

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
    ):
        """
        Calculates actuator setpoints given desired longitudinal velocity and virtual steering angle.
        Mirrors ESP32-S3 ElectronicDifferential4WD continuous gyro assist and slip stabilization.
        """
        safe_dt = dt if dt > 1e-5 else (1.0 / 120.0)
        safe_mu = max(0.10, float(mu))

        # 1. Driver steering intent (speed-capped & asymmetric slew) & yaw-rate reference
        v_for_steer = max(abs(target_v), abs(float(v_curr)) if v_curr is not None else 0.0)
        capped_target_delta = self._compute_capped_target_delta(target_steer_rad, v_for_steer, safe_mu)
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

        v_for_ref = target_v
        if v_curr is not None and abs(v_curr) > abs(target_v) and (v_curr * target_v >= 0.0):
            v_for_ref = math.copysign(abs(v_curr), target_v)

        omega_cmd_driver = (v_for_ref * math.tan(self.driver_steer_rad)) / self.L
        omega_grip_max = (safe_mu * GRAVITY_G) / max(abs(v_for_ref), 0.50)
        omega_ref_target = max(-omega_grip_max, min(omega_grip_max, omega_cmd_driver))
        is_unwinding = (abs(omega_ref_target) < abs(self.omega_ref)) or (omega_ref_target * self.omega_ref < 0.0)
        tau_ref = GYRO_YAW_REF_UNWIND_TAU_S if is_unwinding else GYRO_YAW_REF_TAU_S
        alpha_ref = safe_dt / (tau_ref + safe_dt)
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

        speed_abs = abs(v_for_ref)
        gyro_fade = 0.0
        if speed_abs > V_SLIP_GATE:
            gyro_fade = min(1.0, (speed_abs - V_SLIP_GATE) / max(0.05, GYRO_FULL_SPEED_MPS - V_SLIP_GATE))
        speed_sign = 1.0 if v_for_ref >= 0.0 else -1.0

        raw_delta_gyro = 0.0
        if gyro_fade > 0.0 and abs(excess_yaw) > GYRO_DEADBAND_RAD_S:
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

        # 2b. Model-free incipient tire-saturation modulation (Early US / Early OS)
        raw_early_os = 0.0
        if abs(v_for_ref) >= V_SLIP_GATE and abs(excess_yaw) > GYRO_DEADBAND_RAD_S:
            yaw_dot_lead = (
                (EARLY_OS_YAW_DOT_LEAD_S * abs(self.gyro_z_dot_filt))
                if (excess_yaw * self.gyro_z_dot_filt > 0.0)
                else 0.0
            )
            excess_lead = abs(excess_yaw) + yaw_dot_lead
            raw_early_os = max(0.0, min(1.0, (excess_lead - EARLY_OS_START_RAD_S) /
                                        (EARLY_OS_FULL_RAD_S - EARLY_OS_START_RAD_S)))
        tau_os = EARLY_OS_RISE_TAU_S if raw_early_os >= self.early_os else EARLY_OS_DECAY_TAU_S
        alpha_os = safe_dt / (tau_os + safe_dt)
        self.early_os += alpha_os * (raw_early_os - self.early_os)
        if self.early_os < 0.005:
            self.early_os = 0.0

        v_obs = float(v_curr) if v_curr is not None else v_for_ref
        a_lat_meas_now = abs(v_obs * gyro_z_rad_s)
        alpha_us = safe_dt / (EARLY_US_FILT_TAU_S + safe_dt)
        self._a_lat_obs_filt += alpha_us * (a_lat_meas_now - self._a_lat_obs_filt)

        raw_early_us = 0.0
        a_lat_req_driver = abs(v_for_ref * omega_cmd_driver)
        if (abs(v_obs) >= EARLY_US_MIN_SPEED_MPS and
                abs(self.driver_steer_rad) > EARLY_US_MIN_STEER_RAD and
                abs(self.omega_ref) > EARLY_US_MIN_OMEGA_REF and
                a_lat_req_driver > (0.35 * safe_mu * GRAVITY_G) and
                (gyro_z_rad_s * self.omega_ref >= 0.0)):
            yaw_ratio = abs(gyro_z_rad_s) / max(1e-4, abs(self.omega_ref))
            if yaw_ratio < EARLY_US_YAW_RATIO_HIGH:
                raw_early_us = max(0.0, min(1.0, (EARLY_US_YAW_RATIO_HIGH - yaw_ratio) /
                                            (EARLY_US_YAW_RATIO_HIGH - EARLY_US_YAW_RATIO_LOW)))
        self.early_us_filt += alpha_us * (raw_early_us - self.early_us_filt)
        if self.early_us_filt < 0.005:
            self.early_us_filt = 0.0

        stabilized_target = max(-self.max_steer, min(self.max_steer, self.driver_steer_rad + delta_gyro))

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
            self._prev_delta_gyro_rad = 0.0
            self.slip_state = SLIP_NORMAL
            self._slip_hold_timer = 0.0
            self._understeer_confirm_timer = 0.0
            self._oversteer_confirm_timer = 0.0
            self._slip_entry_speed = 0.0
            self.early_us_filt = 0.0
            self.early_os = 0.0
            self._a_lat_obs_filt = 0.0
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

        # 7. Grip-Capped, Confirmed & Hysteretic IMU Slip Classification
        yaw_error = gyro_z_rad_s - self.omega_ref
        raw_slip = SLIP_NORMAL

        if abs(v_for_ref) >= V_SLIP_GATE:
            dyn_thresh = max(max(self.thresh_spin * 2.2, 0.95), 0.40 * abs(self.omega_ref))
            a_lat_req = abs(v_for_ref * omega_cmd_driver)

            if self.omega_ref > 0.10 and (gyro_z_rad_s - self.omega_ref) > dyn_thresh:
                raw_slip = SLIP_OVERSTEER
            elif self.omega_ref < -0.10 and (self.omega_ref - gyro_z_rad_s) > dyn_thresh:
                raw_slip = SLIP_OVERSTEER
            elif abs(gyro_z_rad_s) > max(1.35, self.thresh_spin * 3.0) and (gyro_z_rad_s * self.omega_ref <= 0.0):
                raw_slip = SLIP_OVERSTEER
            elif (abs(self.driver_steer_rad) > 0.17 and abs(self.omega_ref) > 1.25 and
                  a_lat_req > (0.50 * safe_mu * GRAVITY_G) and
                  (gyro_z_rad_s * self.omega_ref >= 0.0) and abs(gyro_z_rad_s) < (0.40 * abs(self.omega_ref))):
                raw_slip = SLIP_UNDERSTEER

        prev_slip = self.slip_state
        if abs(v_for_ref) < V_SLIP_GATE:
            self.slip_state = SLIP_NORMAL
            self._slip_hold_timer = 0.0
            self._understeer_confirm_timer = 0.0
            self._oversteer_confirm_timer = 0.0
            self._slip_entry_speed = 0.0
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
                self._slip_entry_speed = 0.0

        if self.slip_state != SLIP_NORMAL and prev_slip != self.slip_state:
            self._slip_entry_speed = abs(v_for_ref)

        target_diff_gain = 0.0 if self.slip_state == SLIP_OVERSTEER else max(0.0, 1.0 - self.early_os)
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
