"""
================================================================================
  FOLKRACE AUTONOMOUS BRAIN: POSE TRACKER & SE(2) LOOKAHEAD PROJECTION
================================================================================
  Maintains rolling dead-reckoning odometry and executes the 3-step SE(2)
  forward lookahead projection to cancel ToF sensor integration delay
  and actuator execution lag.
================================================================================
"""

import math
import collections
from typing import Tuple, List, Optional

try:
    from .planner_config import (
        WHEELBASE,
        MAX_STEERING_RAD,
        SERVO_MAX_SPEED_RAD_S,
        TOF_LATENCY_STEPS,
        ACTUATOR_LATENCY_STEPS,
        TOF_LATENCY_S,
        ACTUATOR_LATENCY_S,
    )
except ImportError:
    from planner_config import (
        WHEELBASE,
        MAX_STEERING_RAD,
        SERVO_MAX_SPEED_RAD_S,
        TOF_LATENCY_STEPS,
        ACTUATOR_LATENCY_STEPS,
        TOF_LATENCY_S,
        ACTUATOR_LATENCY_S,
    )


def wrap_to_pi(angle: float) -> float:
    """Wraps angle in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class PoseTracker:
    """
    Maintains a rolling ring buffer of fused odometry poses and projects
    the vehicle's future anchor pose in SE(2) coordinates to compensate
    for perception latency and actuator response delay.
    """

    def __init__(
        self,
        wheelbase: float = WHEELBASE,
        tof_latency_steps: int = TOF_LATENCY_STEPS,
        actuator_latency_steps: int = ACTUATOR_LATENCY_STEPS,
        max_steer_rad: float = MAX_STEERING_RAD,
        servo_max_speed: float = SERVO_MAX_SPEED_RAD_S,
        tof_latency_s: float = TOF_LATENCY_S,
        actuator_latency_s: float = ACTUATOR_LATENCY_S,
    ):
        self.wheelbase = wheelbase
        self.tof_latency_s = float(tof_latency_s)
        self.actuator_latency_s = float(actuator_latency_s)
        self.tof_latency_steps = max(1, int(tof_latency_steps))
        self.actuator_latency_steps = max(1, int(actuator_latency_steps))
        self.max_steer_rad = max_steer_rad
        self.servo_max_speed = servo_max_speed

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.total_distance = 0.0

        # Ring buffer holding past poses (x, y, yaw)
        history_len = self.tof_latency_steps + 1
        self.pose_history = collections.deque(
            [(0.0, 0.0, 0.0) for _ in range(history_len)],
            maxlen=history_len,
        )

        self.proj_anchor = (0.0, 0.0, 0.0)
        self.proj_v = 0.0
        self.proj_steer = 0.0

    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        """Resets the dead-reckoning state and initializes the history buffer."""
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.total_distance = 0.0
        history_len = self.tof_latency_steps + 1
        self.pose_history = collections.deque(
            [(self.x, self.y, self.yaw) for _ in range(history_len)],
            maxlen=history_len,
        )
        self.proj_anchor = (0.0, 0.0, 0.0)
        self.proj_v = 0.0
        self.proj_steer = 0.0

    def update_odometry(self, v_est: float, yaw_rate: float, dt: float) -> Tuple[float, float, float]:
        """
        Integrates forward velocity and yaw rate into local Cartesian coordinates.
        Robot Body convention: +Y is Forward, +X is Right, +Z is Up (turning Left is CCW / positive).
        Uses second-order midpoint heading for curvature accuracy.
        Dynamically adapts past pose history buffer to active timestep dt (time-based parity).
        """
        if math.isnan(v_est) or math.isinf(v_est):
            v_est = 0.0
        if math.isnan(yaw_rate) or math.isinf(yaw_rate):
            yaw_rate = 0.0

        # Dynamically adapt history depth based on active dt (time-based lookahead parity)
        if dt > 0.001:
            req_steps = max(1, int(round(self.tof_latency_s / dt)))
            if req_steps != self.tof_latency_steps:
                self.tof_latency_steps = req_steps
                target_len = req_steps + 1
                if self.pose_history.maxlen != target_len:
                    current_poses = list(self.pose_history)
                    while len(current_poses) < target_len:
                        current_poses.insert(0, current_poses[0] if current_poses else (self.x, self.y, self.yaw))
                    self.pose_history = collections.deque(current_poses[-target_len:], maxlen=target_len)

        yaw_mid = self.yaw + 0.5 * yaw_rate * dt
        dx = -v_est * math.sin(yaw_mid) * dt
        dy =  v_est * math.cos(yaw_mid) * dt

        self.x += dx
        self.y += dy
        self.yaw = wrap_to_pi(self.yaw + yaw_rate * dt)
        self.total_distance += abs(v_est) * dt

        self.pose_history.append((self.x, self.y, self.yaw))
        return (self.x, self.y, self.yaw)

    def compute_projected_anchor(
        self,
        v_current: float,
        steer_current: float,
        steer_target: float,
        dt: float,
        is_reversing: bool = False,
    ) -> Tuple[Tuple[float, float, float], float, float]:
        """
        Computes 3-step composite SE(2) lookahead projection in sensor frame F_sens:
          - Past Motion: step k-2 -> step k (ToF integration latency, ~16 ms)
          - Future Motion: step k -> step k+1 (actuator execution lag, ~8 ms)

        Returns:
          ((dx_anchor, dy_anchor, delta_yaw_anchor), projected_v, projected_steer)
        """
        # 1. Past Motion in sensor coordinate frame
        x_past, y_past, yaw_past = self.pose_history[0]
        x_curr, y_curr, yaw_curr = self.pose_history[-1]

        delta_yaw_past = wrap_to_pi(yaw_curr - yaw_past)
        dx_w = x_curr - x_past
        dy_w = y_curr - y_past
        cos_p = math.cos(yaw_past)
        sin_p = math.sin(yaw_past)
        dx_past = cos_p * dx_w + sin_p * dy_w
        dy_past = -sin_p * dx_w + cos_p * dy_w

        # 2. Future Motion via single-track bicycle model (time-based actuator latency)
        v_k = v_current if is_reversing else max(0.0, v_current)
        dt_act = self.actuator_latency_s if hasattr(self, "actuator_latency_s") else (self.actuator_latency_steps * dt)
        delta_yaw_future = (v_k / self.wheelbase) * math.tan(steer_current) * dt_act
        yaw_future_mid = 0.5 * delta_yaw_future
        dx_future = -v_k * math.sin(yaw_future_mid) * dt_act
        dy_future =  v_k * math.cos(yaw_future_mid) * dt_act

        # 3. Composite 3-step projected anchor pose in F_sens
        delta_yaw_3 = wrap_to_pi(delta_yaw_past + delta_yaw_future)
        cos_dp = math.cos(delta_yaw_past)
        sin_dp = math.sin(delta_yaw_past)
        dx_3 = dx_past + (cos_dp * dx_future - sin_dp * dy_future)
        dy_3 = dy_past + (sin_dp * dx_future + cos_dp * dy_future)

        # 4. Projected actuator state (speed v_3 and steering delta_3)
        # When unwinding out of a corner, project across full round-trip latency so planner
        # doesn't throttle exit acceleration on stale steering feedback.
        is_unwinding = (abs(steer_target) < abs(steer_current)) or (steer_target * steer_current < 0.0)
        dt_steer = max(dt_act, dt + dt_act) if is_unwinding else dt_act
        max_steer_delta = self.servo_max_speed * dt_steer
        delta_steer = max(-max_steer_delta, min(max_steer_delta, steer_target - steer_current))
        steer_3 = max(-self.max_steer_rad, min(self.max_steer_rad, steer_current + delta_steer))

        self.proj_anchor = (dx_3, dy_3, delta_yaw_3)
        self.proj_v = v_k
        self.proj_steer = steer_3

        return (self.proj_anchor, self.proj_v, self.proj_steer)

