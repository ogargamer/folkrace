"""
================================================================================
  ZERO-CHEATING SENSOR FUSION: 4-WHEEL ENCODERS + BNO08X IMU (50 Hz)
  Computes true forward velocity, yaw rate, and heading without Supervisor shortcuts.
================================================================================
"""

import math
from typing import Dict, Optional, Tuple


class IMUEncoderFusion:
    """
    Fuses 4-wheel rotational encoders (PositionSensor) with BNO08x IMU
    (accelerometer, gyroscope, and inertial unit) at 50 Hz.
    """

    def __init__(
        self,
        wheel_radius: float = 0.0275,
        dt: float = 0.02,
        alpha_accel: float = 0.80,
    ):
        self.wheel_radius = wheel_radius
        self.dt = dt
        self.alpha_accel = alpha_accel

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
        self.yaw: float = 0.0
        self.yaw_rate: float = 0.0
        self.a_forward: float = 0.0
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
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.a_forward = 0.0
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

    def update(self, dt: Optional[float] = None) -> Tuple[float, float, float]:
        """
        Executes one 50 Hz fusion cycle.

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
                    # Catch rare resets or anomalies
                    if abs(dp) > 50.0:
                        dp = 0.0
                    omega = dp / step_dt
                else:
                    omega = 0.0
                self._prev_pos[key] = curr_p
                wheel_omegas[key] = omega
            else:
                wheel_omegas[key] = 0.0

        # 2. Derive linear rolling speed from wheel encoders
        # Webots Folkrace motor direction convention: negative motor angle pushes vehicle forward
        # In our world: omega * MOTOR_DIR (-1.0) gives forward wheel speed
        v_rl = -wheel_omegas["RL"] * self.wheel_radius
        v_rr = -wheel_omegas["RR"] * self.wheel_radius
        v_fl = -wheel_omegas["FL"] * self.wheel_radius
        v_fr = -wheel_omegas["FR"] * self.wheel_radius

        # Rear axle is unsteered, providing purest forward rolling ground speed
        v_rear = 0.5 * (v_rl + v_rr)
        v_front = 0.5 * (v_fl + v_fr)
        self.v_wheels = 0.70 * v_rear + 0.30 * v_front

        # 3. Read IMU Accelerometer (local robot body: +Y is Forward, +X is Right)
        a_fwd = 0.0
        a_lat = 0.0
        if self.accel_dev is not None:
            raw_accel = self.accel_dev.getValues()
            if raw_accel and len(raw_accel) >= 3:
                a_lat = float(raw_accel[0])
                a_fwd = float(raw_accel[1])
        self.a_forward = a_fwd

        # Crash Impact Check: Physical wall collision spikes well beyond tire grip (> 14 m/s^2 / ~1.5g)
        is_crash_impact = (a_fwd < -14.0 or math.hypot(a_lat, a_fwd) > 20.0)

        # 4. Read Gyroscope (body Z-axis yaw rate)
        gyro_z = 0.0
        if self.gyro_dev is not None:
            raw_gyro = self.gyro_dev.getValues()
            if raw_gyro and len(raw_gyro) >= 3:
                gyro_z = float(raw_gyro[2])
        self.yaw_rate = gyro_z

        # 5. Read Inertial Unit (Yaw heading)
        if self.imu_dev is not None:
            rpy = self.imu_dev.getRollPitchYaw()
            if rpy and len(rpy) >= 3:
                self.yaw = float(rpy[2])

        # 6. Complementary Filter: Fuse dead-reckoned acceleration with encoder rolling speed
        # Prevents high-frequency wheel spin slip from corrupting velocity
        v_pred = self.v_est + self.a_forward * step_dt
        if is_crash_impact:
            # Crash impact instantly halts forward velocity; reset state and filter lag to 0.0
            self.v_est = 0.0
        elif abs(self.v_wheels) < 0.03 and abs(self.a_forward) < 0.15:
            # Standstill lock
            self.v_est = 0.0
        else:
            self.v_est = self.alpha_accel * v_pred + (1.0 - self.alpha_accel) * self.v_wheels

        # Distance integration
        self.total_distance_m += self.v_est * step_dt

        return self.v_est, self.yaw_rate, self.yaw
