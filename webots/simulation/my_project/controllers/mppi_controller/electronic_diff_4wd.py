"""
================================================================================
  4WD ELECTRONIC DIFFERENTIAL & TRUE ACKERMANN KINEMATICS MODULE
  Real-time geometric speed allocation and BNO08x gyro slip detection
  for Folkrace Robot in Webots.
================================================================================
"""

import math

# --- PHYSICAL VEHICLE PARAMETERS ---
WHEELBASE = 0.1325          # meters (distance between front & rear axles)
TRACK_REAR = 0.1000         # meters (lateral distance between rear wheel centers)
TRACK_FRONT = 0.0822        # meters (lateral distance between front wheel centers)
TRACK_KINGPIN = 0.0620      # meters (lateral distance between front steering pivots)
WHEEL_RADIUS = 0.0275       # meters (27.5 mm rolling radius)
MAX_STEERING_RAD = 0.6981   # +/- 40 degrees limit (0.698132 rad)
MOTOR_DIR = -1.0            # Webots motor rotational orientation (-1.0 pushes vehicle forward)
V_MAX_WHEEL = 5.5000        # m/s physical ceiling (200.0 rad/s * 0.0275 m)
V_SLIP_GATE = 0.30          # m/s minimum speed to activate dynamic IMU slip detection
THRESH_SPIN = 0.45          # rad/s gross yaw rate divergence threshold for oversteer

SLIP_NORMAL = "NORMAL"
SLIP_OVERSTEER = "OVERSTEER"
SLIP_UNDERSTEER = "UNDERSTEER"


class ElectronicDifferential4WD:
    """
    Computes exact independent Ackermann steering angles and 4-wheel rotational speeds,
    handles standstill guards, proportional saturation scaling, and IMU gross slip detection.
    """

    def __init__(self, wheelbase=WHEELBASE, track_rear=TRACK_REAR, track_front=TRACK_FRONT,
                 track_kingpin=TRACK_KINGPIN, wheel_radius=WHEEL_RADIUS,
                 max_steer=MAX_STEERING_RAD, motor_dir=MOTOR_DIR, v_max=V_MAX_WHEEL,
                 thresh_spin=THRESH_SPIN):
        self.L = wheelbase
        self.W_r = track_rear
        self.W_f = track_front
        self.W_k = track_kingpin
        self.R_w = wheel_radius
        self.max_steer = max_steer
        self.motor_dir = motor_dir
        self.v_max = v_max
        self.thresh_spin = thresh_spin

    def update(self, target_v, target_steer_rad, gyro_z_rad_s=0.0):
        """
        Calculates actuator setpoints given desired longitudinal velocity and virtual steering angle.

        Args:
            target_v (float): Desired forward linear velocity (m/s). Negative for reverse.
            target_steer_rad (float): Virtual bicycle steering angle at center (rad).
            gyro_z_rad_s (float): Measured body Z-axis yaw rate from IMU/gyro (rad/s).

        Returns:
            dict: Actuator setpoints and telemetry.
        """
        # 1. Clamp virtual center steering input to physical steering lock (40 deg = 0.698132 rad)
        delta = max(-self.max_steer, min(target_steer_rad, self.max_steer))

        # 2. Kinematic yaw rate command (zero-singularity formulation)
        sin_d = math.sin(delta)
        cos_d = math.cos(delta)
        tan_d = math.tan(delta)
        omega_cmd = (target_v * tan_d) / self.L

        # 3. 100% Exact Geometric Ackermann Front Steering Angles
        if abs(tan_d) > 1e-4:
            half_Wk_sin = (self.W_k * 0.5) * sin_d
            L_cos = self.L * cos_d
            L_sin = self.L * sin_d
            steer_left = math.atan2(L_sin, L_cos - half_Wk_sin)
            steer_right = math.atan2(L_sin, L_cos + half_Wk_sin)
            steer_left = max(-self.max_steer, min(steer_left, self.max_steer))
            steer_right = max(-self.max_steer, min(steer_right, self.max_steer))
        else:
            steer_left = delta
            steer_right = delta

        # 4. Standstill Guard
        if abs(target_v) < 0.001:
            return {
                "steer_left": steer_left,
                "steer_right": steer_right,
                "v_FL": 0.0, "v_FR": 0.0, "v_RL": 0.0, "v_RR": 0.0,
                "omega_FL": 0.0, "omega_FR": 0.0, "omega_RL": 0.0, "omega_RR": 0.0,
                "omega_cmd": 0.0,
                "scale_factor": 1.0,
                "slip_state": SLIP_NORMAL,
                "yaw_error": 0.0
            }

        # 5. 4-Wheel Linear Speed Allocation (Forward & Reverse)
        sign_v = 1.0 if target_v >= 0.0 else -1.0
        half_Wr_omega = omega_cmd * (self.W_r * 0.5)
        half_Wf_omega = omega_cmd * (self.W_f * 0.5)

        # Rear axle speeds (longitudinal heading)
        v_RL = target_v - half_Wr_omega
        v_RR = target_v + half_Wr_omega

        # Front axle speeds (vector hypotenuse along steered wheel orientation)
        fl_x = target_v - half_Wf_omega
        fr_x = target_v + half_Wf_omega
        front_y = self.L * omega_cmd

        v_FL = sign_v * math.hypot(fl_x, front_y)
        v_FR = sign_v * math.hypot(fr_x, front_y)

        # 6. Proportional Saturation Scaling (Preserve turning radius when near motor RPM limit)
        v_peak = max(abs(v_FL), abs(v_FR), abs(v_RL), abs(v_RR))
        scale = 1.0
        if v_peak > self.v_max:
            scale = self.v_max / v_peak
            v_FL *= scale
            v_FR *= scale
            v_RL *= scale
            v_RR *= scale

        # 7. Convert Linear Velocities (m/s) to Actuator Rotational Velocities (rad/s)
        max_omega = (self.v_max / self.R_w)
        omega_FL = max(-max_omega, min(max_omega, (v_FL / self.R_w) * self.motor_dir))
        omega_FR = max(-max_omega, min(max_omega, (v_FR / self.R_w) * self.motor_dir))
        omega_RL = max(-max_omega, min(max_omega, (v_RL / self.R_w) * self.motor_dir))
        omega_RR = max(-max_omega, min(max_omega, (v_RR / self.R_w) * self.motor_dir))

        # 8. Reliability-Focused IMU Slip Detection
        slip_state = SLIP_NORMAL
        yaw_error = gyro_z_rad_s - omega_cmd

        if abs(target_v) >= V_SLIP_GATE:
            # Dynamic tolerance scales with commanded yaw rate (accounts for natural tire slip compliance)
            dyn_thresh = max(self.thresh_spin, 0.40 * abs(omega_cmd))

            # Case A: True Oversteer (vehicle rotating faster than commanded in turn direction)
            if omega_cmd > 0.05 and (gyro_z_rad_s - omega_cmd) > dyn_thresh:
                slip_state = SLIP_OVERSTEER
            elif omega_cmd < -0.05 and (omega_cmd - gyro_z_rad_s) > dyn_thresh:
                slip_state = SLIP_OVERSTEER
            # Case B: Counter-steer slide or uncommanded spinout while straight/counter-steering
            elif abs(gyro_z_rad_s) > max(1.20, self.thresh_spin * 2.0) and (gyro_z_rad_s * omega_cmd <= 0.0):
                slip_state = SLIP_OVERSTEER
            # Case C: Severe Understeer / Wall Plow (Front wheels turned hard but vehicle not turning)
            else:
                omega_achievable = min(abs(omega_cmd), (0.90 * 9.81) / max(abs(target_v), 0.50))
                if abs(delta) > 0.15 and omega_achievable > 1.20 and abs(gyro_z_rad_s) < (0.35 * omega_achievable):
                    slip_state = SLIP_UNDERSTEER

        return {
            "steer_left": steer_left,
            "steer_right": steer_right,
            "v_FL": v_FL, "v_FR": v_FR, "v_RL": v_RL, "v_RR": v_RR,
            "omega_FL": omega_FL, "omega_FR": omega_FR,
            "omega_RL": omega_RL, "omega_RR": omega_RR,
            "omega_cmd": omega_cmd,
            "scale_factor": scale,
            "slip_state": slip_state,
            "yaw_error": yaw_error
        }
