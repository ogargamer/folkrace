"""
================================================================================
  ZERO-CHEATING SENSOR FUSION: 4-WHEEL ENCODERS + BNO08X IMU (120 Hz)
  Computes true forward velocity, yaw rate, and heading without Supervisor shortcuts.
  Features:
    - 4-Wheel Differential Kinematics with 40% Ackermann Steer Projection
    - Median-of-4 Outlier Rejection (robust against single-wheel spin / lift)
    - Real-Time Wheel Slip Detection (Gyro vs Differential Cross-Check)
    - Adaptive Traction-Aware Complementary Filter
    - Standstill & Bidirectional Crash Impact Handlers
================================================================================
"""

import math
from typing import Dict, Optional, Tuple


class IMUEncoderFusion:
    """
    Fuses 4-wheel rotational encoders (PositionSensor) with BNO08x IMU
    (accelerometer, gyroscope, and inertial unit) at 120 Hz.
    """

    def __init__(
        self,
        wheel_radius: float = 0.0275,
        track_rear: float = 0.1000,
        track_front: float = 0.0822,
        wheelbase: float = 0.1325,
        dt: float = 1.0 / 120.0,
    ):
        self.wheel_radius = wheel_radius
        self.track_rear = track_rear
        self.track_front = track_front
        self.wheelbase = wheelbase
        self.dt = dt

        # Encoder state tracking (radians)
        self._prev_pos: Dict[str, Optional[float]] = {
            "FL": None,
            "FR": None,
            "RL": None,
            "RR": None,
        }

        # Filtered kinematic state
        self.v_est: float = 0.0
        self.v_wheels: float = 0.0
        self.v_rear: float = 0.0
        self.v_front: float = 0.0
        self.slip_ratio: float = 0.0
        self.yaw: float = 0.0
        self.yaw_rate: float = 0.0
        self.roll: float = 0.0
        self.pitch: float = 0.0
        self.roll_rate: float = 0.0
        self.pitch_rate: float = 0.0
        self.tilt_mag: float = 0.0
        self.a_forward: float = 0.0
        self.a_lateral: float = 0.0
        self.total_distance_m: float = 0.0

        # Hardware devices
        self.encoders: Dict[str, any] = {}
        self.imu_dev = None
        self.gyro_dev = None
        self.accel_dev = None

    def reset(self) -> None:
        """Resets fusion internal kinematic state and encoder baseline."""
        self._prev_pos = {
            "FL": None,
            "FR": None,
            "RL": None,
            "RR": None,
        }
        self.v_est = 0.0
        self.v_wheels = 0.0
        self.v_rear = 0.0
        self.v_front = 0.0
        self.slip_ratio = 0.0
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.tilt_mag = 0.0
        self.a_forward = 0.0
        self.a_lateral = 0.0
        self.total_distance_m = 0.0

    def attach_devices(
        self,
        encoders: Dict[str, any],
        imu_dev: Optional[any] = None,
        gyro_dev: Optional[any] = None,
        accel_dev: Optional[any] = None,
    ):
        """Attaches initialized Webots device handles."""
        self.encoders = encoders
        self.imu_dev = imu_dev
        self.gyro_dev = gyro_dev
        self.accel_dev = accel_dev

    @property
    def roll_deg(self) -> float:
        return math.degrees(self.roll)

    @property
    def pitch_deg(self) -> float:
        return math.degrees(self.pitch)

    @property
    def tilt_mag_deg(self) -> float:
        return math.degrees(self.tilt_mag)

    def update(
        self,
        dt: Optional[float] = None,
        steer_rad: float = 0.0,
    ) -> Tuple[float, float, float]:
        """
        Executes one 120 Hz fusion cycle.

        Args:
            dt: Timestep duration (seconds)
            steer_rad: Current active steering angle of the front wheels (radians)

        Returns:
            Tuple[float, float, float]: (v_est [m/s], yaw_rate [rad/s], yaw [rad])
        """
        step_dt = dt if (dt is not None and dt > 0.0) else self.dt

        # 1. Read 4 wheel encoders and compute rotational speeds (rad/s)
        wheel_omegas = {}
        for key in ["FL", "FR", "RL", "RR"]:
            dev = self.encoders.get(key)
            if dev is not None:
                curr_p = float(dev.getValue())
                prev_p = self._prev_pos[key]
                if prev_p is not None:
                    dp = curr_p - prev_p
                    if abs(dp) > 50.0:  # Catch rare resets or anomalies
                        dp = 0.0
                    omega = dp / step_dt
                else:
                    omega = 0.0
                self._prev_pos[key] = curr_p
                wheel_omegas[key] = omega
            else:
                wheel_omegas[key] = 0.0

        # 2. Derive linear rolling speed along wheel heading (m/s)
        # Folkrace motor direction convention: negative motor angle pushes vehicle forward
        v_rl = -wheel_omegas["RL"] * self.wheel_radius
        v_rr = -wheel_omegas["RR"] * self.wheel_radius
        v_fl = -wheel_omegas["FL"] * self.wheel_radius
        v_fr = -wheel_omegas["FR"] * self.wheel_radius

        # 3. Read IMU Gyroscope (body frame: +X Pitch, +Y Roll, +Z Yaw)
        gyro_z = 0.0
        gyro_roll = 0.0
        gyro_pitch = 0.0
        if self.gyro_dev is not None:
            raw_gyro = self.gyro_dev.getValues()
            if raw_gyro and len(raw_gyro) >= 3:
                gyro_pitch = float(raw_gyro[0])
                gyro_roll = float(raw_gyro[1])
                gyro_z = float(raw_gyro[2])
        self.yaw_rate = gyro_z
        self.roll_rate = gyro_roll
        self.pitch_rate = gyro_pitch

        # 4. Read Inertial Unit (Euler angles: +X pitch, +Y roll, +Z yaw in ENU vehicle frame)
        if self.imu_dev is not None:
            rpy = self.imu_dev.getRollPitchYaw()
            if rpy and len(rpy) >= 3:
                # In robot body (+Y Forward, +X Right, +Z Up):
                # rpy[1] is rotation around Y (longitudinal roll / bank)
                # rpy[0] is rotation around X (transverse pitch)
                # rpy[2] is rotation around Z (heading yaw)
                self.pitch = float(rpy[0])
                self.roll = float(rpy[1])
                self.yaw = float(rpy[2])
                self.tilt_mag = float(math.hypot(self.roll, self.pitch))

        # 5. Read IMU Accelerometer (local robot body: +Y is Forward, +X is Right)
        a_fwd = 0.0
        a_lat = 0.0
        if self.accel_dev is not None:
            raw_accel = self.accel_dev.getValues()
            if raw_accel and len(raw_accel) >= 3:
                a_lat = float(raw_accel[0])
                a_fwd = float(raw_accel[1])
        self.a_forward = a_fwd
        self.a_lateral = a_lat

        # 6. Kinematic 4-Wheel Speed Calculation & Slip Detection
        # Rear axle: Unsteered pure forward ground speed
        v_rear_avg = 0.5 * (v_rl + v_rr)
        self.v_rear = v_rear_avg

        # Front axle: Steered wheels projected along vehicle longitudinal body axis (+Y)
        cos_steer = math.cos(steer_rad)
        v_fl_proj = v_fl * cos_steer
        v_fr_proj = v_fr * cos_steer
        v_front_avg = 0.5 * (v_fl_proj + v_fr_proj)
        self.v_front = v_front_avg

        # Individual wheel estimates corrected for turning differential using gyro yaw rate
        # v = v_wheel -/+ (track/2) * yaw_rate
        v_rl_body = v_rl + 0.5 * self.track_rear * self.yaw_rate
        v_rr_body = v_rr - 0.5 * self.track_rear * self.yaw_rate
        v_fl_body = v_fl_proj + 0.5 * self.track_front * self.yaw_rate
        v_fr_body = v_fr_proj - 0.5 * self.track_front * self.yaw_rate

        # Yaw rate computed from rear wheel differential vs gyro
        rear_diff_yaw_rate = (v_rr - v_rl) / max(0.01, self.track_rear)
        diff_error = abs(rear_diff_yaw_rate - self.yaw_rate)

        # Axle discrepancy
        axle_discrepancy = abs(v_front_avg - v_rear_avg)

        # Robust wheel speed consensus:
        # Outlier-rejected median of all 4 wheels (rejects single-wheel spin or lift)
        all_wheel_estimates = sorted([v_rl_body, v_rr_body, v_fl_body, v_fr_body])
        v_wheels_robust = 0.5 * (all_wheel_estimates[1] + all_wheel_estimates[2])
        self.v_wheels = v_wheels_robust

        # 7. Inertial Prediction & Adaptive Slip Weighting
        v_pred = self.v_est + self.a_forward * step_dt

        # Crash Impact Check (handles both forward and reverse high-G impacts)
        is_crash_impact = (abs(self.v_wheels) < 0.20 and abs(self.a_forward) > 20.0)

        # Slip detection index [0.0, 1.0]
        wheel_accel = (v_wheels_robust - self.v_est) / max(1e-4, step_dt)
        accel_mismatch = abs(wheel_accel - self.a_forward)

        slip_indicator = min(
            1.0,
            (diff_error / 4.0) * 0.40 +
            (axle_discrepancy / 0.50) * 0.35 +
            (accel_mismatch / 8.0) * 0.25,
        )
        self.slip_ratio = float(slip_indicator)

        # Standstill lock
        if abs(self.v_wheels) < 0.02 and abs(self.a_forward) < 0.20:
            self.v_est = 0.0
        elif is_crash_impact:
            self.v_est = 0.0
        else:
            # Under clean traction, trust wheel encoders heavily (adaptive_alpha ~ 0.08)
            # Under high slip / wheelspin, automatically shift weight to inertial prediction (adaptive_alpha ~ 0.85)
            adaptive_alpha = 0.08 + 0.77 * slip_indicator

            # Bound prediction within physical traction envelope
            envelope_margin = 0.80 if slip_indicator < 0.30 else 1.50
            v_pred_clamped = min(self.v_wheels + envelope_margin, max(self.v_wheels - envelope_margin, v_pred))

            self.v_est = adaptive_alpha * v_pred_clamped + (1.0 - adaptive_alpha) * self.v_wheels

        # Distance integration
        self.total_distance_m += self.v_est * step_dt

        return self.v_est, self.yaw_rate, self.yaw
