"""
Real-Time 60 Hz Local Pathfinder & Controller (< 2 ms CPU Budget)
1. Adaptive Pure Pursuit with lookahead scaling
2. Dynamic Opponent Detection & Smooth Lateral Overtaking (Frenet Ribbon)
3. Anti-Wrong-Way / Track Tangent Alignment Guard
4. Reactive Emergency Reverse State Machine (< 25cm trigger)
"""

import time
import numpy as np

# Controller State Machine
STATE_RACING = "RACING"
STATE_AVOIDANCE = "AVOIDANCE"
STATE_REVERSE = "EMERGENCY_REVERSE"
STATE_REALIGN = "REALIGN"

class RealtimePathfinder:
    def __init__(self, wheelbase=0.25, max_steer_deg=25.0):
        self.wheelbase = wheelbase          # 25cm wheelbase
        self.max_steer_rad = np.radians(max_steer_deg)
        
        # Pure Pursuit Lookahead params
        self.lookahead_gain = 0.65          # L = gain * v
        self.min_lookahead = 0.50           # 50cm min
        self.max_lookahead = 1.40           # 1.4m max
        
        # State Machine
        self.state = STATE_RACING
        self.reverse_start_time = 0.0
        self.reverse_duration = 1.2         # Max reverse time (s)
        
        # Dynamic Overtaking State
        self.current_lateral_offset = 0.0   # Current smooth offset (m)
        self.target_lateral_offset = 0.0    # Target offset (m)
        self.lateral_smoothing_rate = 0.12  # Smooth transition filter

    def compute_control_step(self, robot_pose, global_racing_line, global_velocities, lidar_ranges=None, lidar_angles=None):
        """
        Main 60 Hz Controller Cycle.
        Takes:
          - robot_pose: (x, y, yaw_rad, current_v)
          - global_racing_line: Array of (N, 2) waypoints
          - global_velocities: Array of (N,) speed targets
          - lidar_ranges, lidar_angles: Optional live scan data for obstacle / opponent avoidance
        Returns:
          - velocity (m/s)
          - steering_angle_deg (degrees)
          - diagnostics dict (state, compute_time_ms, lookahead_pt)
        """
        t0 = time.perf_counter()
        
        rx, ry, ryaw, rv = robot_pose
        pts = global_racing_line
        N = len(pts)
        
        # 1. Find Closest Point on Global Racing Line
        dists = np.hypot(pts[:, 0] - rx, pts[:, 1] - ry)
        closest_idx = np.argmin(dists)
        track_dist = dists[closest_idx]
        
        # Calculate local track tangent
        next_idx = (closest_idx + 1) % N
        tangent_vec = pts[next_idx] - pts[closest_idx]
        track_tangent_yaw = np.arctan2(tangent_vec[1], tangent_vec[0])
        
        # 2. Check Emergency Reverse Proximity Guard (< 25 cm obstacle directly ahead)
        front_obstacle_detected = False
        min_front_dist = 99.0
        
        if lidar_ranges is not None and lidar_angles is not None:
            # Check 45-degree front cone (-22.5° to +22.5°)
            front_mask = np.abs(lidar_angles) < np.radians(25.0)
            front_ranges = lidar_ranges[front_mask]
            if len(front_ranges) > 0:
                min_front_dist = np.min(front_ranges)
                if min_front_dist < 0.25: # Under 25 cm
                    front_obstacle_detected = True

        # 3. State Machine Execution
        if self.state == STATE_RACING or self.state == STATE_AVOIDANCE:
            if front_obstacle_detected and rv >= 0.0:
                self.state = STATE_REVERSE
                self.reverse_start_time = time.time()

        if self.state == STATE_REVERSE:
            # Back up while steering wheels opposite to obstacle
            cmd_v = -0.40  # Backwards at 0.4 m/s
            cmd_steer_deg = 20.0 # Counter-steer
            
            elapsed = time.time() - self.reverse_start_time
            if min_front_dist > 0.55 or elapsed > self.reverse_duration:
                self.state = STATE_REALIGN
            
            dt_ms = (time.perf_counter() - t0) * 1000.0
            return cmd_v, cmd_steer_deg, {
                "state": self.state,
                "compute_ms": dt_ms,
                "front_clearance": min_front_dist
            }

        elif self.state == STATE_REALIGN:
            # Re-align heading with track flow
            yaw_diff = (track_tangent_yaw - ryaw + np.pi) % (2 * np.pi) - np.pi
            cmd_v = 0.35
            cmd_steer_deg = np.degrees(np.clip(yaw_diff * 1.5, -self.max_steer_rad, self.max_steer_rad))
            
            if abs(yaw_diff) < np.radians(15.0):
                self.state = STATE_RACING

            dt_ms = (time.perf_counter() - t0) * 1000.0
            return cmd_v, cmd_steer_deg, {
                "state": self.state,
                "compute_ms": dt_ms
            }

        # 4. Anti-Wrong-Way / Heading Alignment Guard
        yaw_error = (track_tangent_yaw - ryaw + np.pi) % (2 * np.pi) - np.pi
        if abs(yaw_error) > np.radians(110.0):
            # Facing wrong way! Slow down and steer hard in direction of track flow
            cmd_v = 0.30
            cmd_steer_deg = np.degrees(np.sign(yaw_error) * self.max_steer_rad)
            dt_ms = (time.perf_counter() - t0) * 1000.0
            return cmd_v, cmd_steer_deg, {
                "state": "RECOVERY_FLIP",
                "compute_ms": dt_ms
            }

        # 5. Dynamic Overtaking & Obstacle Avoidance (Frenet Lateral Shift)
        # Check if an opponent robot is blocking the racing line (0.4m to 1.8m ahead)
        if lidar_ranges is not None and lidar_angles is not None:
            forward_cone = (lidar_angles > -np.radians(20)) & (lidar_angles < np.radians(20))
            obstacle_distances = lidar_ranges[forward_cone]
            if len(obstacle_distances) > 0 and np.min(obstacle_distances) < 1.4:
                # Opponent blocking ahead -> shift lateral target
                self.state = STATE_AVOIDANCE
                self.target_lateral_offset = 0.35 # Shift 35cm to the side
            else:
                # Clear track -> decay back to zero (return to optimal racing line)
                self.state = STATE_RACING
                self.target_lateral_offset = 0.0

        # Smooth lateral offset transition
        self.current_lateral_offset += (self.target_lateral_offset - self.current_lateral_offset) * self.lateral_smoothing_rate

        # 6. Adaptive Lookahead Distance (scales with current velocity)
        lookahead_dist = np.clip(self.lookahead_gain * max(0.5, rv), self.min_lookahead, self.max_lookahead)

        # 7. Find Lookahead Target on Racing Line
        cum_dist = 0.0
        target_idx = closest_idx
        for step in range(1, N // 2):
            idx = (closest_idx + step) % N
            prev = (closest_idx + step - 1) % N
            cum_dist += np.hypot(pts[idx, 0] - pts[prev, 0], pts[idx, 1] - pts[prev, 1])
            if cum_dist >= lookahead_dist:
                target_idx = idx
                break

        target_point = pts[target_idx].copy()
        
        # Apply dynamic lateral offset if overtaking
        if abs(self.current_lateral_offset) > 0.01:
            # Perpendicular normal: (-dy, dx)
            dx = pts[(target_idx + 1) % N, 0] - pts[(target_idx - 1) % N, 0]
            dy = pts[(target_idx + 1) % N, 1] - pts[(target_idx - 1) % N, 1]
            mag = np.hypot(dx, dy) + 1e-6
            normal = np.array([-dy / mag, dx / mag])
            target_point += normal * self.current_lateral_offset

        # 8. Pure Pursuit Curvature & Steering Angle
        # Vector from robot to lookahead point in robot-frame
        dx_w = target_point[0] - rx
        dy_w = target_point[1] - ry
        
        # Rotate into robot frame
        cos_y = np.cos(ryaw)
        sin_y = np.sin(ryaw)
        x_rob = cos_y * dx_w + sin_y * dy_w
        y_rob = -sin_y * dx_w + cos_y * dy_w
        
        # Pure pursuit curvature: kappa = 2 * y_rob / L^2
        actual_L = np.hypot(x_rob, y_rob) + 1e-6
        curvature = (2.0 * y_rob) / (actual_L ** 2)
        
        # Steering angle: delta = atan(curvature * wheelbase)
        steer_rad = np.arctan(curvature * self.wheelbase)
        steer_rad = np.clip(steer_rad, -self.max_steer_rad, self.max_steer_rad)
        cmd_steer_deg = float(np.degrees(steer_rad))

        # 9. Velocity Target from Global Profile
        target_v = float(global_velocities[closest_idx])
        
        # Throttle down slightly if large steering angle
        steer_penalty = 1.0 - 0.45 * (abs(cmd_steer_deg) / 25.0)
        cmd_v = float(max(0.5, target_v * steer_penalty))

        dt_ms = (time.perf_counter() - t0) * 1000.0

        return cmd_v, cmd_steer_deg, {
            "state": self.state,
            "compute_ms": dt_ms,
            "lookahead_pt": target_point,
            "lateral_offset": self.current_lateral_offset,
            "target_idx": target_idx
        }
