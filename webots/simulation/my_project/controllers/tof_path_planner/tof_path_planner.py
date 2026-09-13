"""
================================================================================
  UNIFIED AUTONOMOUS TOF+IMU RACING CONTROLLER (50 Hz)
  Zero cheating: No racing line, no global cheating, no 360-degree lidar.
  Relies purely on:
    - 16-channel ToF sensor rig (15 mm C++ Smart Hybrid Costmap)
    - IMU (gyro, accelerometer, yaw) fused with 4-wheel encoders
    - 4WD Ackermann Electronic Differential torque allocation
================================================================================
"""

import os
import sys
import time
import math
from typing import Dict, Optional, Tuple, Any
import numpy as np

# Ensure controller directory is on sys.path
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
MY_PROJECT_DIR = os.path.dirname(CONTROLLERS_DIR)
WORKSPACE_ROOT = os.path.dirname(MY_PROJECT_DIR)

for p in [CURR_DIR, CONTROLLERS_DIR, MY_PROJECT_DIR, WORKSPACE_ROOT]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

try:
    from controller import Robot, Supervisor
    WEBOTS_AVAILABLE = True
except ImportError:
    WEBOTS_AVAILABLE = False

from electronic_diff_4wd import ElectronicDifferential4WD
from cpp_smart_hybrid_costmap import CppSmartHybridCostmap
from planner_config import (
    PLANNER_TYPE,
    TIME_STEP_MS,
    DT,
    SIM_HZ,
    PLANNER_HZ,
    DT_PLANNER,
    PLANNER_DECIMATION,
    TOF_TIME_STEP_MS,
    TOF_SENSOR_NAMES,
    TOF_MAX_RANGE_M,
    TOF_MIN_RANGE_M,
    ENCODER_NAMES,
    WHEEL_RADIUS,
    V_MIN_CRUISE,
    V_MAX_CRUISE,
    MAX_STEERING_RAD,
    WHEELBASE,
    TRACK_REAR,
    TRACK_FRONT,
    TRACK_KINGPIN,
    MOTOR_DIR,
    V_MAX_WHEEL,
    GRID_X_MIN,
    GRID_X_MAX,
    GRID_Y_MIN,
    GRID_Y_MAX,
    GRID_RES,
    R_INFLATION_M,
    MOVING_OBSTACLE_NAMES,
    MOVING_OBSTACLE_SPEED_MPS,
    MOVING_OBSTACLE_Z_ELEVATION,
    RECOVERY_REVERSE_SPEED_MPS,
    RECOVERY_STEER_ANGLE_RAD,
    RECOVERY_STALL_CONFIRM_S,
    RECOVERY_FRONT_BLOCKED_M,
    RECOVERY_REVERSE_STROKE_MAX_S,
    RECOVERY_TOTAL_TIMEOUT_S,
    RECOVERY_CLEARANCE_EXIT_M,
    RECOVERY_EXIT_MIN_TIME_S,
    ACCEL_FWD_MAX,
    BRAKE_DECEL_MAX,
    SERVO_MAX_SPEED_RAD_S,
    STRAIGHT_LOCK_DIST_M,
    STRAIGHT_LOCK_SPEED_MPS,
    EMERGENCY_BACKUP_DIST_M,
    compute_steering_aggressiveness,
)
from imu_encoder_fusion import IMUEncoderFusion
from tof_gap_scout import ToFGapScout
from planner_option2_sl_mpc import CurvilinearSLMPC
from stuck_recovery_manager import StuckRecoveryManager, StuckRecoveryState


def read_tof_multi_ray(dev: any, noise_pct: float = 0.02) -> float:
    """Reads distance from Webots Lidar/DistanceSensor node modeling ToF zone."""
    val = TOF_MAX_RANGE_M
    if hasattr(dev, "getRangeImage"):
        r_img = dev.getRangeImage()
        if r_img:
            valid_r = [v for v in r_img if not math.isinf(v) and not math.isnan(v) and v <= TOF_MAX_RANGE_M]
            if valid_r:
                val = float(min(valid_r))
    elif hasattr(dev, "getValue"):
        raw = float(dev.getValue())
        if raw > 10.0:
            raw /= 1000.0
        val = raw

    if val < (TOF_MAX_RANGE_M - 0.01) and noise_pct > 0.0:
        noise = (np.random.random() * 2.0 - 1.0) * noise_pct
        val = val * (1.0 + noise)

    return float(np.clip(val, TOF_MIN_RANGE_M, TOF_MAX_RANGE_M))


class ReverseObstacleEngine:
    """
    Interpolates obstacle positions along the track racing line in reverse
    (traveling against the robot heading at 0.50 m/s pace).
    """

    def __init__(self, csv_path: str, speed_mps: float = 0.50):
        self.speed_mps = speed_mps
        self.traj = np.zeros((0, 2))
        self.dists = np.zeros(0)
        self.total_length = 14.109

        if os.path.exists(csv_path):
            try:
                arr = np.loadtxt(csv_path, delimiter=",", skiprows=1)
                self.traj = arr[:, 0:2]  # x, y
                if arr.shape[1] >= 6:
                    self.dists = arr[:, 5]
                    self.total_length = float(self.dists[-1])
                else:
                    diffs = np.diff(self.traj, axis=0)
                    seg_dists = np.hypot(diffs[:, 0], diffs[:, 1])
                    self.dists = np.concatenate([[0.0], np.cumsum(seg_dists)])
                    self.total_length = float(self.dists[-1])
                print(f"[ToF Obstacles] Loaded racing line from {os.path.basename(csv_path)} (Length: {self.total_length:.3f} m)", flush=True)
            except Exception as e:
                print(f"[ToF Obstacles] Failed to parse trajectory CSV: {e}", flush=True)
        else:
            print(f"[ToF Obstacles] Trajectory file not found: {csv_path}", flush=True)

    def get_position_at_s(self, s: float) -> Tuple[float, float]:
        if len(self.dists) < 2:
            return 0.0, 0.0
        s_norm = s % self.total_length
        idx = int(np.searchsorted(self.dists, s_norm))
        if idx <= 0:
            return float(self.traj[0, 0]), float(self.traj[0, 1])
        if idx >= len(self.dists):
            return float(self.traj[-1, 0]), float(self.traj[-1, 1])

        s0 = self.dists[idx - 1]
        s1 = self.dists[idx]
        ratio = (s_norm - s0) / max(1e-6, s1 - s0)
        p0 = self.traj[idx - 1]
        p1 = self.traj[idx]
        x = p0[0] + ratio * (p1[0] - p0[0])
        y = p0[1] + ratio * (p1[1] - p0[1])
        return float(x), float(y)


def run_autonomous_controller(forced_planner_type: Optional[str] = None):
    """Main execution loop running at 50 Hz in Webots."""
    if not WEBOTS_AVAILABLE:
        print("[ToF Controller] Webots API not detected. Exiting.", flush=True)
        return

    robot = Supervisor()
    robot_node = robot.getSelf()

    # Persistent Planner Selection File
    planner_state_file = os.path.join(CURR_DIR, "active_planner.json")
    persisted_planner = None
    if os.path.exists(planner_state_file):
        try:
            with open(planner_state_file, "r", encoding="utf-8") as f:
                p_cfg = json.load(f)
                persisted_planner = p_cfg.get("planner")
        except Exception:
            pass

    active_planner_name = forced_planner_type or persisted_planner or os.environ.get("PLANNER_TYPE", PLANNER_TYPE)
    print("=" * 80, flush=True)
    print(f"  STARTING AUTONOMOUS TOF CONTROLLER (Multi-Rate: 120 Hz Sim/IMU/4WD, 60 Hz ToF/Planner)", flush=True)
    print(f"  Active Path Planner: {active_planner_name} (Persisted: {persisted_planner})", flush=True)
    print("=" * 80, flush=True)

    # 1. Initialize 4WD Drive Motors
    motors = {
        "FL": robot.getDevice("motor FL"),
        "FR": robot.getDevice("motor FR"),
        "RL": robot.getDevice("motor RL"),
        "RR": robot.getDevice("motor RR"),
    }
    for name, m in motors.items():
        if m:
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
        else:
            print(f"[Warning] Motor '{name}' not found!", flush=True)

    # 2. Initialize Steering Servos
    servos = {
        "left": robot.getDevice("servo left"),
        "right": robot.getDevice("servo right"),
    }
    for name, s in servos.items():
        if s:
            s.setPosition(0.0)
        else:
            print(f"[Warning] Servo '{name}' not found!", flush=True)

    # 3. Initialize Wheel Encoders (PositionSensor at 120 Hz)
    encoders = {}
    for key, enc_name in ENCODER_NAMES.items():
        enc = robot.getDevice(enc_name)
        if enc:
            enc.enable(TIME_STEP_MS)
            encoders[key] = enc
        else:
            print(f"[Warning] PositionSensor '{enc_name}' not found on chassis.", flush=True)

    # 4. Initialize IMU, Gyro & Accelerometer (at 120 Hz)
    imu_dev = robot.getDevice("bno08x_imu") or robot.getDevice("inertial unit") or robot.getDevice("imu")
    if imu_dev:
        imu_dev.enable(TIME_STEP_MS)

    gyro = robot.getDevice("gyro") or robot.getDevice("bno08x_gyro")
    if gyro:
        gyro.enable(TIME_STEP_MS)

    accel = robot.getDevice("accelerometer") or robot.getDevice("bno08x_accel")
    if accel:
        accel.enable(TIME_STEP_MS)

    # 5. Initialize 16-Channel ToF Lidar Sensor Array (at 60 Hz = 16 ms)
    tof_devices = {}
    active_tof_names = []
    is_front_only = any(robot.getDevice(n) is not None for n in TOF_SENSOR_NAMES)
    candidate_sensors = TOF_SENSOR_NAMES if is_front_only else getattr(planner_config, "LEGACY_TOF_SENSOR_NAMES", TOF_SENSOR_NAMES)

    for dev_name in candidate_sensors:
        dev = robot.getDevice(dev_name)
        if dev:
            dev.enable(TOF_TIME_STEP_MS)
            tof_devices[dev_name] = dev
            active_tof_names.append(dev_name)

    print(f"[ToF Controller] Enabled {len(tof_devices)}/16 ToF sensors ({'Front-Only 16ch' if is_front_only else 'Legacy 4-Quadrant'}) at {PLANNER_HZ} Hz (IMU & 4WD at {SIM_HZ} Hz).", flush=True)

    # 6. Initialize Modules
    costmap = CppSmartHybridCostmap(
        grid_bounds=(GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX),
        resolution=GRID_RES,
        r_inflation_m=R_INFLATION_M,
    )
    gap_scout = ToFGapScout()
    fusion = IMUEncoderFusion(wheel_radius=WHEEL_RADIUS, dt=DT)
    fusion.attach_devices(encoders, imu_dev=imu_dev, gyro_dev=gyro, accel_dev=accel)

    ediff = ElectronicDifferential4WD(
        wheelbase=WHEELBASE,
        track_rear=TRACK_REAR,
        track_front=TRACK_FRONT,
        track_kingpin=TRACK_KINGPIN,
        wheel_radius=WHEEL_RADIUS,
        max_steer=MAX_STEERING_RAD,
        motor_dir=MOTOR_DIR,
        v_max=V_MAX_WHEEL,
    )

    # Instantiate C++ Curvilinear S-L MPC planner
    active_planner_name = "OPTION2_SL_MPC"
    planner = CurvilinearSLMPC(costmap)

    # 7. Non-Blocking Stuck & Reverse Recovery Manager
    recovery_manager = StuckRecoveryManager(
        reverse_speed_m_s=RECOVERY_REVERSE_SPEED_MPS,
        steer_angle_rad=RECOVERY_STEER_ANGLE_RAD,
        emergency_backup_dist_m=EMERGENCY_BACKUP_DIST_M,
        clearance_exit_m=RECOVERY_CLEARANCE_EXIT_M,
        servo_slew_rad_s=SERVO_MAX_SPEED_RAD_S,
        brake_decel_max=BRAKE_DECEL_MAX,
        accel_fwd_max=ACCEL_FWD_MAX,
    )

    # 8. Moving Obstacles along Racing Line (Reverse motion at 0.50 m/s pace)
    obs_fields = []
    for name in MOVING_OBSTACLE_NAMES:
        node = robot.getFromDef(name)
        if node:
            field = node.getField("translation")
            if field:
                obs_fields.append(field)

    traj_candidates = [
        os.path.join(WORKSPACE_ROOT, "cyan_champion_trajectory.csv"),
        os.path.join(MY_PROJECT_DIR, "controllers", "folkrace_drive", "trajectories", "cyan_champion_trajectory.csv"),
        os.path.join(WORKSPACE_ROOT, "webots/optimization/ga_planner_optimizer", "data", "cyan_champion_trajectory.csv"),
    ]
    traj_csv = next((p for p in traj_candidates if os.path.exists(p)), traj_candidates[0])
    obs_engine = ReverseObstacleEngine(traj_csv, speed_mps=MOVING_OBSTACLE_SPEED_MPS)
    if obs_fields:
        print(f"[ToF Controller] Hooked {len(obs_fields)} moving obstacles to reverse engine at {MOVING_OBSTACLE_SPEED_MPS:.2f} m/s.", flush=True)

    step_count = 0
    cached_tofs = {k: TOF_MAX_RANGE_M for k in active_tof_names}
    active_steer = 0.0
    cached_target_steer = 0.0
    cached_target_speed = float(V_MIN_CRUISE)
    rate_limited_speed = 0.0
    was_recovering = False
    cached_v_est = 0.0
    cached_yaw_rate = 0.0
    cached_yaw = 0.0
    cached_steer_aggr = 1.0
    cached_plan_res = {"cmd_steer_rad": 0.0, "cmd_speed_mps": V_MIN_CRUISE, "status": "WARMUP"}
    cached_plan_ms = 0.0
    cached_map_ms = 0.0
    cached_xg = 0.0
    cached_yg = 1.0
    cached_width = 0.5
    cached_theta_g = 0.0
    cached_prob_map = np.zeros((1, 1), dtype=np.float32)
    cached_hazard_mask = np.zeros((1, 1), dtype=np.uint8)
    cached_front_min_m = TOF_MAX_RANGE_M
    cached_d_straight = TOF_MAX_RANGE_M

    telem_path = os.environ.get("TELEMETRY_LOG_FILE", os.path.join(WORKSPACE_ROOT, f"telemetry_{active_planner_name.lower()}.jsonl"))
    try:
        telem_file = open(telem_path, "w", encoding="utf-8")
        print(f"[ToF Controller] Logging detailed telemetry to {os.path.basename(telem_path)}", flush=True)
    except Exception as e:
        print(f"[ToF Controller] Telemetry log disabled: {e}", flush=True)
        telem_file = None

    # UDP socket for real-time GUI telemetry broadcasting (Port 9876) and command receiving (Port 9877)
    import socket
    import json
    gui_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gui_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        cmd_sock.bind(("127.0.0.1", 9877))
        cmd_sock.setblocking(False)
        print("[ToF Controller] Live GUI command receiver bound to port 9877", flush=True)
    except Exception as e:
        print(f"[ToF Controller] Warning: Could not bind command port 9877: {e}", flush=True)

    sim_paused = False
    sim_step_requested = False
    sim_speed_mult = 0.05  # Default 0.05x (20x Slow-Mo) on initial launch

    max_sim_seconds = float(os.environ.get("WEBOTS_RUN_SECONDS", "0"))

    # Main 50 Hz Simulation Control Loop
    while robot.step(TIME_STEP_MS) != -1:
        wall_start_t = time.perf_counter()

        # Check incoming live GUI commands on Port 9877
        try:
            while True:
                cmd_data, _ = cmd_sock.recvfrom(4096)
                cmd_pkt = json.loads(cmd_data.decode("utf-8"))
                c = cmd_pkt.get("cmd", "")
                if c == "switch_planner":
                    planner = CurvilinearSLMPC(costmap)
                    active_planner_name = "OPTION2_SL_MPC"
                    print(f"[ToF Controller] Active planner: {active_planner_name} (C++ S-L MPC)", flush=True)
                elif c == "pause":
                    sim_paused = True
                elif c == "play":
                    sim_paused = False
                elif c == "step":
                    sim_step_requested = True
                elif c == "set_speed":
                    sim_speed_mult = float(cmd_pkt.get("val", 1.0))
                    print(f"[ToF Controller] Live updated simulation speed multiplier: {sim_speed_mult:.2f}x", flush=True)
                elif c == "set_param":
                    p_name = cmd_pkt.get("name", "")
                    p_val = cmd_pkt.get("val", None)
                    if p_val is not None:
                        if p_name == "r_inflation_m":
                            costmap.r_inflation_m = float(p_val)
                            costmap._handle = costmap._lib.costmap_create(
                                costmap.x_min, costmap.x_max, costmap.y_min, costmap.y_max, costmap.res, costmap.r_inflation_m
                            )
                            print(f"[ToF Controller] Live updated inflation radius to: {p_val*100:.1f} cm", flush=True)
                        elif p_name == "target_speed":
                            import planner_config
                            planner_config.OPT1_TARGET_SPEED_MPS = float(p_val)
                            planner_config.OPT2_TARGET_SPEED_MPS = float(p_val)
                            planner_config.OPT3_TOP_SPEED_MPS = float(p_val)
                            print(f"[ToF Controller] Live updated target cruising speed to: {p_val:.2f} m/s", flush=True)
                        elif p_name == "alpha_peak":
                            import planner_config
                            planner_config.ALPHA_PEAK = float(p_val)
                            print(f"[ToF Controller] Live updated peak slip angle to: {math.degrees(p_val):.1f} deg", flush=True)
                        elif p_name == "alt_switch_hysteresis":
                            import planner_config
                            planner_config.ALT_SWITCH_HYSTERESIS = float(p_val)
                            if hasattr(planner, "alt_switch_hysteresis"):
                                planner.alt_switch_hysteresis = float(p_val)
                            print(f"[ToF Controller] Live updated Challenger Switch Hysteresis to: {float(p_val):.1f}", flush=True)
                        elif p_name == "alt_min_separation_m":
                            import planner_config
                            planner_config.ALT_MIN_SEPARATION_M = float(p_val)
                            if hasattr(gap_scout, "alt_min_separation_m"):
                                gap_scout.alt_min_separation_m = float(p_val)
                            if hasattr(planner, "alt_min_separation_m"):
                                planner.alt_min_separation_m = float(p_val)
                            print(f"[ToF Controller] Live updated Challenger Min Separation to: {float(p_val)*100:.1f} cm", flush=True)
        except (BlockingIOError, socket.error):
            pass
        except Exception as e:
            print(f"[ToF Controller] Cmd receive error: {e}", flush=True)

        if sim_paused and not sim_step_requested:
            for m in motors.values():
                if m:
                    m.setVelocity(0.0)
            continue
        sim_step_requested = False

        step_count += 1
        t_now = robot.getTime()

        if max_sim_seconds > 0.0 and t_now >= max_sim_seconds:
            print(f"[ToF Controller] Reached time limit {max_sim_seconds:.1f}s. Exiting cleanly.", flush=True)
            robot.simulationQuit(0)
            break

        # 0. Update Moving Obstacles along Racing Line (0.50 m/s reverse motion)
        if obs_fields and obs_engine.total_length > 0:
            n_obs = len(obs_fields)
            for k, field in enumerate(obs_fields):
                obs_s = (float(k + 1) * (obs_engine.total_length / float(n_obs + 1)) - MOVING_OBSTACLE_SPEED_MPS * t_now) % obs_engine.total_length
                ox, oy = obs_engine.get_position_at_s(obs_s)
                field.setSFVec3f([float(ox), float(oy), float(MOVING_OBSTACLE_Z_ELEVATION)])

        # Initial warmup pause (0.50 s)
        if t_now < 0.50:
            for m in motors.values():
                if m:
                    m.setVelocity(0.0)
            for s in servos.values():
                if s:
                    s.setPosition(0.0)
            continue

        # 60 Hz Control Tick (every 2nd step)
        # Fusion, 16-channel ToF sampling, Costmap, Gap Scout, Planner, Recovery Manager, Straight-Lock
        run_planner_step = (step_count % PLANNER_DECIMATION == 0)
        if run_planner_step:
            # A. Update IMU + Wheel Encoder Odometry at 60 Hz (Zero Cheating)
            v_est, yaw_rate, yaw = fusion.update(dt=DT_PLANNER)
            cached_v_est = v_est
            cached_yaw_rate = yaw_rate
            cached_yaw = yaw

            # B. Read 16 ToF Sensors (60 Hz)
            min_tof = TOF_MAX_RANGE_M
            for name in active_tof_names:
                dev = tof_devices.get(name)
                if dev:
                    val = read_tof_multi_ray(dev, noise_pct=0.02)
                    cached_tofs[name] = val
                    if val < min_tof:
                        min_tof = val

            # C. Process 15mm C++ Smart Hybrid Costmap (<0.10 ms)
            t_map_start = time.perf_counter()
            prob_map, hazard_mask, map_ms = costmap.process_tof_frame(cached_tofs)
            cached_prob_map = prob_map
            cached_hazard_mask = hazard_mask
            cached_map_ms = map_ms

            # D. Scout Best Open Gap Ahead in Visible Clear Space
            xg, yg, width, theta_g = gap_scout.scout_gap(
                cached_tofs,
                hazard_mask,
                costmap,
                target_lookahead_m=1.70,
                steer_current=active_steer,
                prob_map=prob_map,
            )
            cached_xg, cached_yg, cached_width, cached_theta_g = xg, yg, width, theta_g

            # E. Execute Active Path Planner at 60 Hz
            t_plan_start = time.perf_counter()
            plan_res = planner.plan(
                xg=xg,
                yg=yg,
                theta_g=theta_g,
                aperture_width=width,
                v_current=v_est,
                steer_current=active_steer,
                yaw_rate_current=yaw_rate,
                extra_targets=gap_scout.last_targets,
            )
            plan_ms = (time.perf_counter() - t_plan_start) * 1000.0

            cached_plan_res = plan_res
            cached_plan_ms = plan_ms
            raw_target_steer = float(plan_res.get("cmd_steer_rad", 0.0))
            raw_target_speed = float(plan_res.get("cmd_speed_mps", V_MIN_CRUISE))

            # Side obstacle guard & straight/front clearance metrics:
            if "tof_front_right_82.9deg" in cached_tofs:
                # In front-only cross-pointing rig:
                # Right sensors point to left side; Left sensors point to right side
                d_left_side = min(
                    cached_tofs.get("tof_front_right_82.9deg", TOF_MAX_RANGE_M),
                    cached_tofs.get("tof_front_right_71.6deg", TOF_MAX_RANGE_M),
                    cached_tofs.get("tof_front_right_60.4deg", TOF_MAX_RANGE_M),
                )
                d_right_side = min(
                    cached_tofs.get("tof_front_left_82.9deg", TOF_MAX_RANGE_M),
                    cached_tofs.get("tof_front_left_71.6deg", TOF_MAX_RANGE_M),
                    cached_tofs.get("tof_front_left_60.4deg", TOF_MAX_RANGE_M),
                )
                d_front_l = cached_tofs.get("tof_front_left_4.1deg", TOF_MAX_RANGE_M)
                d_front_r = cached_tofs.get("tof_front_right_4.1deg", TOF_MAX_RANGE_M)
                cached_d_straight = min(d_front_l, d_front_r)
                front_keys = [
                    "tof_front_left_4.1deg", "tof_front_right_4.1deg",
                    "tof_front_left_15.4deg", "tof_front_right_15.4deg",
                ]
            else:
                d_left_side = min(
                    cached_tofs.get("tof_fl_0", TOF_MAX_RANGE_M),
                    cached_tofs.get("tof_bl_0", TOF_MAX_RANGE_M),
                    cached_tofs.get("tof_fl_1", TOF_MAX_RANGE_M),
                )
                d_right_side = min(
                    cached_tofs.get("tof_fr_0", TOF_MAX_RANGE_M),
                    cached_tofs.get("tof_br_0", TOF_MAX_RANGE_M),
                    cached_tofs.get("tof_fr_1", TOF_MAX_RANGE_M),
                )
                d_fl3 = cached_tofs.get("tof_fl_3", TOF_MAX_RANGE_M)
                d_fr3 = cached_tofs.get("tof_fr_3", TOF_MAX_RANGE_M)
                cached_d_straight = min(d_fl3, d_fr3)
                front_keys = ["tof_fl_3", "tof_fr_3", "tof_fl_2", "tof_fr_2"]

            cached_front_min_m = min([cached_tofs.get(k, TOF_MAX_RANGE_M) for k in front_keys])

            # Non-Blocking Stuck & Reverse Recovery Manager (evaluated at 60 Hz)
            is_recovering, rec_v, rec_steer = recovery_manager.update(
                dt=DT_PLANNER,
                current_v=v_est,
                commanded_v=raw_target_speed,
                cached_tofs=cached_tofs,
                heading_err=theta_g,
                active_steer=active_steer,
            )

            if is_recovering:
                target_speed = rec_v
                target_steer = rec_steer
                active_steer = rec_steer
                steer_aggressiveness = 1.0
                was_recovering = True
            else:
                if was_recovering:
                    # Clean Handshake: recovery exited with wheels centered; hand off smoothly
                    was_recovering = False
                    active_steer = 0.0

                # Fast Straight-Line Lock (evaluated at 60 Hz, guarded by turn intent)
                if "tof_front_left_4.1deg" in cached_tofs:
                    is_straight_locked = (
                        cached_tofs.get("tof_front_left_4.1deg", 0.0) >= STRAIGHT_LOCK_DIST_M and
                        cached_tofs.get("tof_front_right_4.1deg", 0.0) >= STRAIGHT_LOCK_DIST_M and
                        abs(theta_g) < 0.08 and
                        abs(raw_target_steer) < 0.08
                    )
                else:
                    is_straight_locked = (
                        cached_tofs.get("tof_fl_3", 0.0) >= STRAIGHT_LOCK_DIST_M and
                        cached_tofs.get("tof_fr_3", 0.0) >= STRAIGHT_LOCK_DIST_M and
                        abs(theta_g) < 0.08 and
                        abs(raw_target_steer) < 0.08
                    )

                if is_straight_locked:
                    target_speed = STRAIGHT_LOCK_SPEED_MPS
                    target_steer = 0.0
                    active_steer = 0.0
                    steer_aggressiveness = compute_steering_aggressiveness(cached_d_straight, steer_error_rad=0.0)
                else:
                    # Straight-distance & small-offset adaptive steering aggressiveness (60 Hz):
                    target_speed = raw_target_speed
                    target_steer = raw_target_steer
                    steer_delta = target_steer - active_steer
                    steer_aggressiveness = compute_steering_aggressiveness(cached_d_straight, steer_error_rad=steer_delta)
                    active_steer = active_steer + steer_aggressiveness * steer_delta

            # Save command setpoints for 120 Hz 4WD Differential
            cached_target_speed = target_speed
            cached_target_steer = active_steer
            cached_steer_aggr = steer_aggressiveness

        # F. 4WD Electronic Differential Torque & Ackermann Steering Allocation at 120 Hz
        # Consumes commands every 2nd step, runs at 120 Hz with live gyro slip damping
        # Unified Friction Rate Limiter for speed command (enforces acceleration & deceleration limits)
        speed_err = cached_target_speed - rate_limited_speed
        if (rate_limited_speed >= 0.0 and speed_err > 0.0) or (rate_limited_speed <= 0.0 and speed_err < 0.0):
            max_dv = ACCEL_FWD_MAX * DT
        else:
            max_dv = BRAKE_DECEL_MAX * DT

        if abs(speed_err) <= max_dv:
            rate_limited_speed = cached_target_speed
        else:
            rate_limited_speed += math.copysign(max_dv, speed_err)

        live_gyro_z = float(gyro.getValues()[2]) if gyro else cached_yaw_rate
        actuators = ediff.update(target_v=rate_limited_speed, target_steer_rad=cached_target_steer, gyro_z_rad_s=live_gyro_z)

        # H. Command Webots Actuators (at 120 Hz)
        if servos["left"]:
            servos["left"].setPosition(actuators["steer_left"])
        if servos["right"]:
            servos["right"].setPosition(actuators["steer_right"])

        if motors["FL"]:
            motors["FL"].setVelocity(actuators["omega_FL"])
        if motors["FR"]:
            motors["FR"].setVelocity(actuators["omega_FR"])
        if motors["RL"]:
            motors["RL"].setVelocity(actuators["omega_RL"])
        if motors["RR"]:
            motors["RR"].setVelocity(actuators["omega_RR"])

        # Prepare variables for 60 Hz Telemetry & GUI broadcasting
        v_est = cached_v_est
        yaw_rate = cached_yaw_rate
        target_speed = cached_target_speed
        target_steer = cached_target_steer
        steer_aggressiveness = cached_steer_aggr
        plan_res = cached_plan_res
        plan_ms = cached_plan_ms
        map_ms = cached_map_ms
        xg, yg, width, theta_g = cached_xg, cached_yg, cached_width, cached_theta_g
        prob_map = cached_prob_map
        hazard_mask = cached_hazard_mask
        front_min_m = cached_front_min_m
        d_straight = cached_d_straight

        # H. Command Webots Actuators (at 120 Hz)
        if servos["left"]:
            servos["left"].setPosition(actuators["steer_left"])
        if servos["right"]:
            servos["right"].setPosition(actuators["steer_right"])

        if motors["FL"]:
            motors["FL"].setVelocity(actuators["omega_FL"])
        if motors["FR"]:
            motors["FR"].setVelocity(actuators["omega_FR"])
        if motors["RL"]:
            motors["RL"].setVelocity(actuators["omega_RL"])
        if motors["RR"]:
            motors["RR"].setVelocity(actuators["omega_RR"])

        # I. Log Comprehensive Telemetry (JSONL format at 25 Hz)
        if telem_file is not None and step_count % 2 == 0:
            entry = {
                "t": round(t_now, 4),
                "step": step_count,
                "planner": active_planner_name,
                "v_est": round(v_est, 3),
                "v_tgt": round(target_speed, 3),
                "steer_rad": round(target_steer, 4),
                "yaw_rate": round(yaw_rate, 4),
                "a_forward": round(fusion.a_forward, 3),
                "dist_m": round(fusion.total_distance_m, 3),
                "front_clear_m": round(front_min_m, 3),
                "d_straight": round(d_straight, 3),
                "steer_aggr": round(steer_aggressiveness, 3),
                "plan_ms": round(plan_ms, 2),
                "map_ms": round(map_ms, 2),
                "surviving": plan_res.get("surviving_count", 0),
                "recovery": recovery_manager.state,
            }
            try:
                telem_file.write(json.dumps(entry) + "\n")
                telem_file.flush()
            except Exception:
                pass

        # J. Broadcast Live Telemetry & Trajectory Lines to GUI on Port 9876 (every 2 steps = 25 Hz)
        if step_count % 2 == 0:
            try:
                # Downsample 15mm prob_map (134x167) to uint8 bytes for fast UDP transmission
                # or send compressed binary grid
                prob_down = (prob_map[::2, ::2] * 255.0).astype(np.uint8)
                hazard_down = hazard_mask[::2, ::2].astype(np.uint8)

                import zlib
                import base64
                grid_compressed = base64.b64encode(zlib.compress(prob_down.tobytes())).decode("ascii")
                hazard_compressed = base64.b64encode(zlib.compress(hazard_down.tobytes())).decode("ascii")

                def to_py(obj):
                    if isinstance(obj, (np.floating, np.float32, np.float64)):
                        return float(obj)
                    elif isinstance(obj, (np.integer, np.int32, np.int64)):
                        return int(obj)
                    elif isinstance(obj, (list, tuple)):
                        return [to_py(x) for x in obj]
                    elif isinstance(obj, dict):
                        return {k: to_py(v) for k, v in obj.items()}
                    return obj

                gui_pkt = {
                    "time": round(float(t_now), 3),
                    "planner": active_planner_name,
                    "status": plan_res.get("status", "OK"),
                    "v_est": round(float(v_est), 3),
                    "v_tgt": round(float(target_speed), 3),
                    "steer_deg": round(math.degrees(float(target_steer)), 2),
                    "yaw_rate": round(float(yaw_rate), 3),
                    "dist_m": round(float(fusion.total_distance_m), 3),
                    "gap": [round(float(xg), 3), round(float(yg), 3), round(float(width), 3)],
                    "targets": [[round(float(tx), 3), round(float(ty), 3), round(float(ap), 3)] for (tx, ty, ap) in (gap_scout.last_targets or [])],
                    "plan_ms": round(float(plan_ms), 2),
                    "map_ms": round(float(map_ms), 2),
                    "surviving": int(plan_res.get("surviving_count", 0)),
                    "tof": {k: round(float(v), 3) for k, v in cached_tofs.items()},
                    "best_line": to_py(plan_res.get("best_line", [])),
                    "candidate_lines": to_py(plan_res.get("candidate_lines", []))[:24],
                    "grid_shape": [int(s) for s in prob_down.shape],
                    "grid_bounds": [float(b) for b in [GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX]],
                    "grid_data": grid_compressed,
                    "hazard_data": hazard_compressed,
                    "r_inflation_m": float(costmap.r_inflation_m),
                }
                raw_pkt = json.dumps(gui_pkt).encode("utf-8")
                gui_sock.sendto(raw_pkt, ("127.0.0.1", 9876))
            except Exception as e:
                if step_count % 50 == 0:
                    print(f"[ToF Controller] UDP broadcast warning: {e}", flush=True)

        # Periodic Telemetry to terminal (every 10 steps = 0.20 s)
        if step_count % 10 == 0:
            print(
                f"[{active_planner_name}] T={t_now:5.2f}s | "
                f"v_est={v_est:4.2f}m/s (tgt: {target_speed:4.2f}) | "
                f"steer={math.degrees(target_steer):+5.1f} deg | "
                f"gap=({xg:+5.2f}, {yg:4.2f})m, w={width*100:4.1f}cm | "
                f"Plan={plan_ms:4.2f}ms (Map={map_ms:4.2f}ms) | "
                f"Status={plan_res.get('status', 'OK')} | "
                f"Dist={fusion.total_distance_m:5.2f}m",
                flush=True,
            )

        # K. Simulation Speed Throttling (0.05x to 100.0x)
        if sim_speed_mult < 4.9:
            target_step_dt = (TIME_STEP_MS / 1000.0) / max(0.01, sim_speed_mult)
            elapsed_wall = time.perf_counter() - wall_start_t
            if elapsed_wall < target_step_dt:
                time.sleep(target_step_dt - elapsed_wall)

    if telem_file is not None:
        telem_file.close()


if __name__ == "__main__":
    run_autonomous_controller()
