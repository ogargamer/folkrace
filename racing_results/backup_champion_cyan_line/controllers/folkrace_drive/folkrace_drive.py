"""
================================================================================
  FOLKRACE AUTONOMOUS RACING LINE CONTROLLER
  High-Speed Pure Pursuit + Ackermann Steering + Optimal Velocity Profile Tracking
  for the Folkrace Robot in Webots.
================================================================================
"""

import os
import sys
import math

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

# Import Webots controller API
try:
    from controller import Robot, Supervisor, Keyboard
except ImportError:
    pass

# --- PHYSICAL VEHICLE PARAMETERS ---
WHEEL_RADIUS = 0.0275      # meters (27.5 mm)
WHEELBASE = 0.1325         # meters (distance between front & rear axles)
TRACK_WIDTH = 0.082        # meters (lateral distance between left & right wheels)
MAX_STEERING_RAD = 0.5236  # +/- 30 degrees limit
MOTOR_DIR = -1.0           # Invert motor rotation (-1.0 pushes vehicle forward)

# --- CONTROLLER TUNING PARAMETERS ---
LOOKAHEAD_MIN = 0.20       # meters minimum lookahead
LOOKAHEAD_GAIN = 0.20      # seconds (Ld = clamp(gain * v, min, max))
LOOKAHEAD_MAX = 0.48       # meters maximum lookahead
HEADING_P_GAIN = 0.30      # Stanley heading correction gain
CROSS_TRACK_GAIN = 1.00    # Pure pursuit curvature gain
SPEED_SCALE = 0.40         # 2x FASTER (0.40 = 40% full race pace, ~0.8 - 1.2 m/s)

TIME_STEP = 16             # ms (60 Hz control loop)


def load_trajectory():
    """Load the pre-computed optimal Cyan Champion racing trajectory."""
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    csv_candidates = [
        os.path.join(curr_dir, "cyan_champion_trajectory.csv"),
        os.path.join(curr_dir, "..", "..", "racing_results", "backup_champion_cyan_line", "cyan_champion_trajectory.csv"),
        os.path.join(curr_dir, "..", "folkrace_drive", "cyan_champion_trajectory.csv"),
        r"c:\Users\nurmi\Documents\Da Winna\racing_results\backup_champion_cyan_line\cyan_champion_trajectory.csv",
        r"c:\Users\nurmi\Documents\Da Winna\my_project\controllers\folkrace_drive\cyan_champion_trajectory.csv",
    ]
    
    found_path = None
    for p in csv_candidates:
        if os.path.exists(p):
            found_path = p
            break
            
    if not found_path:
        raise FileNotFoundError("Could not locate cyan_champion_trajectory.csv in search paths.")
        
    rows = []
    with open(found_path, "r", encoding="utf-8") as f:
        lines = f.readlines()
        for line in lines[1:]:
            parts = line.strip().split(",")
            if len(parts) >= 6:
                rows.append([float(x) for x in parts])
                
    if HAS_NUMPY:
        data = np.array(rows)
    else:
        data = rows
        
    print(f"[Folkrace Autodrive] Successfully loaded racing trajectory from: {os.path.basename(found_path)}")
    print(f"[Folkrace Autodrive] Trajectory contains {len(rows)} waypoints, length: {rows[-1][5]:.2f}m, est lap: {rows[-1][6]:.2f}s")
    return data


def main():
    print("[Folkrace Autodrive] Starting controller initialization...")
    
    # Initialize Supervisor / Robot instance
    is_supervisor = False
    robot_node = None
    try:
        robot = Supervisor()
        robot_node = robot.getSelf()
        if robot_node is not None:
            is_supervisor = True
    except Exception:
        pass
        
    if not is_supervisor:
        try:
            robot = Robot()
        except Exception as e:
            print(f"[Folkrace Autodrive] Error initializing Webots Robot: {e}")
            return

    keyboard = Keyboard()
    keyboard.enable(TIME_STEP)

    # --- ACTUATOR INITIALIZATION ---
    motor_map = {"RL": "motor RL", "RR": "motor RR", "FL": "motor FL", "FR": "motor FR"}
    drive_motors = {}
    for short_key, dev_name in motor_map.items():
        m = robot.getDevice(dev_name)
        if m:
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
            drive_motors[short_key] = m
        else:
            print(f"[Folkrace Autodrive] Warning: Actuator '{dev_name}' not found.")

    steering_servos = {}
    for name, key in [("servo left", "left"), ("servo right", "right")]:
        s = robot.getDevice(name)
        if s:
            s.setPosition(0.0)
            steering_servos[key] = s
        else:
            print(f"[Folkrace Autodrive] Warning: Actuator '{name}' not found.")

    # --- SENSORS INITIALIZATION ---
    lidar = robot.getDevice("ms200k_lidar")
    if lidar:
        lidar.enable(67)  # 15 Hz scan rate
        lidar.enablePointCloud()
        print("[Folkrace Autodrive] MS200K LiDAR enabled (15 Hz, 5m range).")

    imu = robot.getDevice("bno08x_imu")
    if imu:
        imu.enable(10)
        print("[Folkrace Autodrive] BNO08X IMU enabled.")

    gyro = robot.getDevice("gyro")
    if gyro:
        gyro.enable(10)

    accel = robot.getDevice("accelerometer")
    if accel:
        accel.enable(10)

    for k in range(12):
        dev = robot.getDevice(f"tof_{k}")
        if dev:
            dev.enable(16)  # 60 Hz sampling

    tof_lidar = robot.getDevice("vl53l8cx_tof_array")
    if tof_lidar:
        tof_lidar.enable(16)

    # --- LOAD RACING TRAJECTORY ---
    traj = load_trajectory()
    if HAS_NUMPY:
        traj_x = traj[:, 0].tolist()
        traj_y = traj[:, 1].tolist()
        traj_v = (traj[:, 2] * SPEED_SCALE).tolist()
        traj_yaw = traj[:, 3].tolist()
        traj_k = traj[:, 4].tolist()
    else:
        traj_x = [r[0] for r in traj]
        traj_y = [r[1] for r in traj]
        traj_v = [r[2] * SPEED_SCALE for r in traj]
        traj_yaw = [r[3] for r in traj]
        traj_k = [r[4] for r in traj]
        
    n_pts = len(traj_x)

    print("\n" + "=" * 70)
    print("  FOLKRACE AUTONOMOUS RACING LINE CONTROLLER READY & ACTIVE")
    print("  Status: [RUNNING AUTONOMOUSLY] (Press 'M' for Manual, 'A' for Auto)")
    print("  Controls: Up/Down speed, Left/Right steer, Space = E-STOP")
    print("=" * 70 + "\n")

    auto_mode = True
    manual_speed = 0.0
    manual_steer = 0.0

    last_idx = 0
    lap_count = 0
    lap_start_time = robot.getTime()
    first_step = True
    
    # Odometry dead reckoning fallback if not supervisor
    odom_x = 0.6
    odom_y = 0.0
    odom_yaw = 0.0

    while robot.step(TIME_STEP) != -1:
        # Handle keyboard mode switching & manual override
        key = keyboard.getKey()
        while key != -1:
            if key == ord('M') or key == ord('m'):
                auto_mode = False
                print("[MODE] Switched to MANUAL mode.")
            elif key == ord('A') or key == ord('a'):
                auto_mode = True
                print("[MODE] Switched to AUTONOMOUS RACING LINE mode.")
            elif key == Keyboard.UP:
                manual_speed = min(manual_speed + 5.0, 100.0)
            elif key == Keyboard.DOWN:
                manual_speed = max(manual_speed - 5.0, -50.0)
            elif key == Keyboard.LEFT:
                manual_steer = min(manual_steer + 0.05, MAX_STEERING_RAD)
            elif key == Keyboard.RIGHT:
                manual_steer = max(manual_steer - 0.05, -MAX_STEERING_RAD)
            elif key == ord(' '):
                manual_speed = 0.0
                manual_steer = 0.0
                auto_mode = False
                print("[E-STOP] Stopped motors.")
            key = keyboard.getKey()

        if not auto_mode:
            # Manual drive
            for m in drive_motors.values():
                m.setVelocity(manual_speed * MOTOR_DIR)
            if "left" in steering_servos:
                steering_servos["left"].setPosition(manual_steer)
            if "right" in steering_servos:
                steering_servos["right"].setPosition(manual_steer)
            continue

        # --- AUTONOMOUS POSE LOCALIZATION ---
        if is_supervisor and robot_node is not None:
            pos = robot_node.getPosition()
            rx, ry = pos[0], pos[1]
            rot = robot_node.getOrientation()
            fwd_x = rot[1]
            fwd_y = rot[4]
            current_yaw = math.atan2(fwd_y, fwd_x)
        else:
            # Fallback using IMU yaw
            if imu:
                rpy = imu.getRollPitchYaw()
                current_yaw = rpy[2]
            else:
                current_yaw = odom_yaw
            rx, ry = odom_x, odom_y

        # 1. Find closest waypoint on trajectory
        search_range = n_pts if first_step else 80
        first_step = False
        best_dist_sq = 1e9
        closest_idx = last_idx
        for offset in range(-search_range, search_range):
            idx = (last_idx + offset) % n_pts
            d_sq = (traj_x[idx] - rx)**2 + (traj_y[idx] - ry)**2
            if d_sq < best_dist_sq:
                best_dist_sq = d_sq
                closest_idx = idx

        # Lap counter trigger (crossing start line at index ~0)
        if last_idx > n_pts - 40 and closest_idx < 40:
            lap_count += 1
            now = robot.getTime()
            lap_time = now - lap_start_time
            print(f"[LAP COMPLETED] Lap #{lap_count} Time: {lap_time:.2f} s")
            lap_start_time = now
        last_idx = closest_idx

        # 2. Adaptive Lookahead Distance based on target speed
        current_target_v = traj_v[closest_idx]
        lookahead_dist = max(LOOKAHEAD_MIN, min(LOOKAHEAD_GAIN * current_target_v, LOOKAHEAD_MAX))

        # 3. Find lookahead target point
        target_idx = closest_idx
        accum_dist = 0.0
        while accum_dist < lookahead_dist:
            next_idx = (target_idx + 1) % n_pts
            seg_len = math.hypot(traj_x[next_idx] - traj_x[target_idx], traj_y[next_idx] - traj_y[target_idx])
            accum_dist += max(seg_len, 0.01)
            target_idx = next_idx

        tx, ty = traj_x[target_idx], traj_y[target_idx]
        target_v = traj_v[target_idx]
        target_yaw = traj_yaw[target_idx]
        target_k = traj_k[target_idx]

        # 4. Transform lookahead point to robot's local body frame (+Y Forward, -X Left, +X Right)
        dx = tx - rx
        dy = ty - ry

        local_fwd = dx * math.cos(current_yaw) + dy * math.sin(current_yaw)
        local_lat_left = -dx * math.sin(current_yaw) + dy * math.cos(current_yaw)

        dist_to_target = max(math.hypot(local_fwd, local_lat_left), 0.05)
        pure_pursuit_angle = math.atan2(2.0 * WHEELBASE * local_lat_left, dist_to_target**2)

        # Stanley heading alignment correction
        heading_err = (target_yaw - current_yaw + math.pi) % (2.0 * math.pi) - math.pi
        total_steer = pure_pursuit_angle + HEADING_P_GAIN * heading_err

        # Clamp steering to physical servo limits
        steer_angle = max(-MAX_STEERING_RAD, min(total_steer, MAX_STEERING_RAD))

        # 5. Ackermann Steering Geometry for Left & Right front wheels
        tan_delta = math.tan(steer_angle)
        if abs(tan_delta) > 1e-4:
            steer_left = math.atan((WHEELBASE * tan_delta) / (WHEELBASE - 0.5 * TRACK_WIDTH * tan_delta))
            steer_right = math.atan((WHEELBASE * tan_delta) / (WHEELBASE + 0.5 * TRACK_WIDTH * tan_delta))
        else:
            steer_left = steer_angle
            steer_right = steer_angle

        steer_left = max(-MAX_STEERING_RAD, min(steer_left, MAX_STEERING_RAD))
        steer_right = max(-MAX_STEERING_RAD, min(steer_right, MAX_STEERING_RAD))

        if "left" in steering_servos:
            steering_servos["left"].setPosition(steer_left)
        if "right" in steering_servos:
            steering_servos["right"].setPosition(steer_right)

        # 6. Longitudinal Wheel Velocity Actuation with Electronic Differential
        base_omega = (target_v / WHEEL_RADIUS) * MOTOR_DIR
        diff_factor = 0.5 * TRACK_WIDTH * target_k
        omega_left = base_omega * (1.0 - diff_factor)
        omega_right = base_omega * (1.0 + diff_factor)

        if "RL" in drive_motors:
            drive_motors["RL"].setVelocity(omega_left)
        if "FL" in drive_motors:
            drive_motors["FL"].setVelocity(omega_left)
        if "RR" in drive_motors:
            drive_motors["RR"].setVelocity(omega_right)
        if "FR" in drive_motors:
            drive_motors["FR"].setVelocity(omega_right)

        # Periodic live telemetry (once every 30 steps = ~0.5s)
        if int(robot.getTime() * 1000) % 500 < TIME_STEP:
            print(f"[RACE] Speed: {target_v:.2f} m/s ({target_v*3.6:.1f} km/h) | Steer: {math.degrees(steer_angle):+.1f} deg | Waypoint: {closest_idx}/{n_pts}")

        # Update simple dead reckoning
        dt_s = TIME_STEP / 1000.0
        odom_x += target_v * math.cos(current_yaw) * dt_s
        odom_y += target_v * math.sin(current_yaw) * dt_s
        odom_yaw = (odom_yaw + (target_v * math.tan(steer_angle) / WHEELBASE) * dt_s) % (2.0 * math.pi)


if __name__ == "__main__":
    main()
