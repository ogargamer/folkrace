"""
================================================================================
  PURE MPPI AUTONOMOUS RACING CONTROLLER (Webots 60-100 Hz)
  - Dedicated MPPI Controller with 8 Vectorized Variants
  - 16-Channel Multi-Ray ToF Sensor Fusion (up to 1.80m shortest ray)
  - 20mm 60Hz Precomputed Local Occupancy Costmap
  - 3-Curve Geometric Seeding (Center QP, Left +25cm, Right -25cm)
  - 4WD Ackermann Electronic Differential Torque Vectoring
================================================================================
"""

import os
import sys
import time
import math
import json
import socket
import random
import zlib
from typing import Dict, List, Optional, Tuple, Any
import numpy as np

# Ensure parent and controller directories are on sys.path
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
FOLKRACE_DIR = os.path.join(CONTROLLERS_DIR, "folkrace_drive")
SCANNER_DIR = os.path.join(CONTROLLERS_DIR, "tof_16ch_grid_scanner")
MY_PROJECT_DIR = os.path.dirname(CONTROLLERS_DIR)
WORKSPACE_ROOT = os.path.dirname(MY_PROJECT_DIR)

for p in [CURR_DIR, FOLKRACE_DIR, SCANNER_DIR, MY_PROJECT_DIR, WORKSPACE_ROOT]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

try:
    from controller import Robot, Supervisor
    WEBOTS_AVAILABLE = True
except ImportError:
    WEBOTS_AVAILABLE = False

from electronic_diff_4wd import (
    ElectronicDifferential4WD,
    WHEELBASE,
    WHEEL_RADIUS,
    TRACK_REAR,
    TRACK_FRONT,
    TRACK_KINGPIN,
    MOTOR_DIR,
    V_MAX_WHEEL,
)
MAX_STEERING_RAD = 0.6981
from tof_pathfinders import FrenetTransformer
from mppi_folkrace_controller import MPPIController, MPPIGenome
from fast_tof_carto_costmap import FastCostmapPipeline, SENSOR_RIG, StampedPose, load_cartographer_map

TIME_STEP_MS = 10  # 100 Hz Webots physics step (10 ms)
TOF_MAX_RANGE_M = 1.80
TOF_MIN_RANGE_M = 0.001

# 16-channel sensor device names on robot chassis
TOF_SENSOR_NAMES = [
    "tof_fl_0", "tof_fl_1", "tof_fl_2", "tof_fl_3",
    "tof_fr_0", "tof_fr_1", "tof_fr_2", "tof_fr_3",
    "tof_bl_0", "tof_bl_1", "tof_bl_2", "tof_bl_3",
    "tof_br_0", "tof_br_1", "tof_br_2", "tof_br_3",
]


def load_trajectory(filename: str) -> Optional[np.ndarray]:
    """Loads a racing line CSV file from candidate directories."""
    candidate_paths = [
        os.path.join(WORKSPACE_ROOT, "racing_results", "tum_qp_obstacle_aware_25cm_offsets", filename),
        os.path.join(FOLKRACE_DIR, "trajectories", filename),
        os.path.join(FOLKRACE_DIR, filename),
        os.path.join(CURR_DIR, filename),
    ]
    for path in candidate_paths:
        if os.path.exists(path):
            try:
                arr = np.loadtxt(path, delimiter=",", skiprows=1)
                if arr.ndim == 2 and arr.shape[1] >= 6:
                    print(f"[MPPI Controller] Loaded {len(arr)} pts from {os.path.basename(path)}", flush=True)
                    return arr
            except Exception as e:
                print(f"[MPPI Controller] Warning loading {path}: {e}", flush=True)
    return None


class PureMPPIDriver:
    """
    Dedicated MPPI Autonomous Driving Agent.
    """

    def __init__(self, genome: Optional[MPPIGenome] = None):
        if genome is None:
            win_json = os.path.join(WORKSPACE_ROOT, "winning_mppi_genome.json")
            if os.path.exists(win_json):
                try:
                    with open(win_json, "r") as f:
                        data = json.load(f)
                    flat = data.get("flat_array")
                    if flat and len(flat) >= 27:
                        self.genome = MPPIGenome.from_array(flat)
                        print(f"[MPPI Controller] Loaded champion GA genome from {os.path.basename(win_json)} (fitness={data.get('fitness', 0.0):.1f})", flush=True)
                    else:
                        self.genome = MPPIGenome()
                except Exception as e:
                    print(f"[MPPI Controller] Warning reading {win_json}: {e}", flush=True)
                    self.genome = MPPIGenome()
            else:
                self.genome = MPPIGenome()
        else:
            self.genome = genome

        self.mppi = MPPIController(genome=self.genome)

        # 4WD Electronic Differential with Ackermann kinematics and dynamic torque vectoring
        self.ediff = ElectronicDifferential4WD(
            wheelbase=WHEELBASE,
            track_rear=TRACK_REAR,
            track_front=TRACK_FRONT,
            track_kingpin=TRACK_KINGPIN,
            wheel_radius=WHEEL_RADIUS,
            max_steer=MAX_STEERING_RAD,
            motor_dir=MOTOR_DIR,
            v_max=V_MAX_WHEEL,
        )

        # 60 Hz Local Occupancy Costmap Pipeline (20 mm resolution with C-space inflation)
        self.costmap_pipe = FastCostmapPipeline(
            resolution=0.02,
            enable_latency_comp=True,
            r_inflation_m=self.genome.r_inflation_m,
        )

        # 1. Load Centerline Optimal Trajectory
        center_raw = load_trajectory("cyan_champion_trajectory.csv")
        if center_raw is None:
            center_raw = load_trajectory("master_cyan_champion_optimal.csv")

        if center_raw is not None:
            self.frenet_center = FrenetTransformer(
                center_raw[:, 5], center_raw[:, 0], center_raw[:, 1], center_raw[:, 3], center_raw[:, 4]
            )
            self.traj_v = center_raw[:, 2]
        else:
            self.frenet_center = None
            self.traj_v = None

        # 2. Load Dedicated Left Midpoint Corridor Trajectory (between racing line and left wall)
        left_raw = load_trajectory("midpoint_offset_left.csv")
        if left_raw is None:
            left_raw = load_trajectory("wall_offset_left_25cm.csv")
        if left_raw is not None:
            self.frenet_left = FrenetTransformer(
                left_raw[:, 5], left_raw[:, 0], left_raw[:, 1], left_raw[:, 3], left_raw[:, 4]
            )
        else:
            self.frenet_left = None

        # 3. Load Dedicated Right Midpoint Corridor Trajectory (between racing line and right wall)
        right_raw = load_trajectory("midpoint_offset_right.csv")
        if right_raw is None:
            right_raw = load_trajectory("wall_offset_right_25cm.csv")
        if right_raw is not None:
            self.frenet_right = FrenetTransformer(
                right_raw[:, 5], right_raw[:, 0], right_raw[:, 1], right_raw[:, 3], right_raw[:, 4]
            )
        else:
            self.frenet_right = None

        # 4. Load Cartographer 15 Hz Global Baseline Occupancy Map
        carto_yaml = os.path.join(WORKSPACE_ROOT, "maps", "cyan_champion_clean_static", "cartographer_2d_slam_static_map.yaml")
        if not os.path.exists(carto_yaml):
            carto_yaml = os.path.join(WORKSPACE_ROOT, "webots/robotics/ros2_cartographer", "maps", "folkrace_track_map.yaml")

        carto_data = load_cartographer_map(carto_yaml)
        if carto_data is not None:
            self.carto_grid, self.carto_origin, self.carto_res = carto_data
            print(f"[MPPI Controller] Cartographer 15Hz Baseline Map Loaded ({os.path.basename(carto_yaml)}): {self.carto_grid.shape}, res={self.carto_res*100:.1f}cm", flush=True)
        else:
            self.carto_grid, self.carto_origin, self.carto_res = None, (0.0, 0.0), 0.02

        self.curr_station_idx = 0
        self.active_steer_rad = 0.0
        self.step_counter = 0
        self.smoothed_v = 0.0
        self.last_step_t = 0.0
        self.reversing_lock = False

    def step(
        self,
        rx: float,
        ry: float,
        ryaw: float,
        v_mps: float,
        gyro_z: float,
        tof_readings: Dict[str, float],
        timestamp_s: float,
    ) -> Tuple[Dict[str, float], Dict[str, Any], float]:
        """
        Executes one complete MPPI control cycle.
        """
        self.step_counter += 1

        # 1. Update Cartographer 15 Hz Baseline Map Patch
        if self.carto_grid is not None and (self.step_counter == 1 or self.step_counter % 6 == 0):
            pose_stamped = StampedPose(timestamp_s, rx, ry, ryaw, v_mps)
            self.costmap_pipe.update_slam_map(self.carto_grid, self.carto_origin, self.carto_res, pose_stamped)

        # 2. Update pose ring buffer and generate fused 20mm local costmap (50 Hz ToF Override)
        self.costmap_pipe.update_pose(timestamp_s, rx, ry, ryaw, v_mps)
        prob_map, hazard_mask, costmap_time_ms = self.costmap_pipe.process_tof_frame(
            tof_readings, timestamp_s
        )

        # 2. Track Frenet Station Index & Reference Speed
        cte = 0.0
        if self.frenet_center is not None:
            _, cte, self.curr_station_idx, _, _ = self.frenet_center.cartesian_to_frenet(
                rx, ry, guess_idx=self.curr_station_idx
            )
            raw_v_ref = float(self.traj_v[self.curr_station_idx % len(self.traj_v)])
            v_ref = min(3.80, raw_v_ref)
        else:
            v_ref = 2.80

        # 2.5. Front ToF Distance & Stateful Reverse Escape Evaluation
        front_keys = ['tof_fl_0', 'tof_fl_1', 'tof_fl_2', 'tof_fl_3', 'tof_fr_0', 'tof_fr_1', 'tof_fr_2', 'tof_fr_3']
        front_tofs = [float(tof_readings.get(k, 1.80)) for k in front_keys]
        front_min_cm = min(front_tofs) * 100.0 if front_tofs else 180.0

        if front_min_cm < 14.0 or (abs(v_mps) < 0.12 and front_min_cm < 24.0 and timestamp_s > 2.0):
            if not self.reversing_lock:
                print(f"[MPPI Controller] >>> ENGAGING REVERSE LOCK (Front ToF = {front_min_cm:.1f} cm) <<<", flush=True)
            self.reversing_lock = True
        elif self.reversing_lock and front_min_cm >= 25.0:
            print(f"[MPPI Controller] >>> FRONT CLEAR ({front_min_cm:.1f} cm) -> RESUMING FORWARD DRIVE <<<", flush=True)
            self.reversing_lock = False

        # 3. Vectorized MPPI Optimization
        steer_cmd, speed_cmd, diag = self.mppi.compute_control(
            rx=rx,
            ry=ry,
            ryaw=ryaw,
            v_mps=v_mps,
            active_steer_rad=self.active_steer_rad,
            prob_map=prob_map,
            hazard_mask=hazard_mask,
            frenet_helper=self.frenet_center,
            curr_idx=self.curr_station_idx,
            v_target_ref=v_ref,
            frenet_left=self.frenet_left,
            frenet_right=self.frenet_right,
        )

        # If locked in reverse, enforce reverse trajectory towards outer line until front ToFs are fully clear
        if self.reversing_lock and speed_cmd >= 0.0:
            frenet_outer = self.frenet_left if (self.frenet_left and self.frenet_right and self.frenet_left.total_length >= self.frenet_right.total_length) else (self.frenet_right or self.frenet_center)
            if frenet_outer is not None and frenet_outer.n_pts > 0:
                d_sqs_out = (frenet_outer.traj_x - rx) ** 2 + (frenet_outer.traj_y - ry) ** 2
                closest_out_idx = int(np.argmin(d_sqs_out))
                s_now = frenet_outer.traj_s[closest_out_idx]
                s_rev_tgt = (s_now - 0.50) % frenet_outer.total_length
                idx_tgt = int(np.searchsorted(frenet_outer.traj_s, s_rev_tgt)) % frenet_outer.n_pts
                dx_r = frenet_outer.traj_x[idx_tgt] - rx
                dy_r = frenet_outer.traj_y[idx_tgt] - ry
                alpha_rev = math.atan2(dy_r, dx_r) - (ryaw + math.pi)
                alpha_rev = (alpha_rev + math.pi) % (2.0 * math.pi) - math.pi
                dist_r = max(0.20, math.hypot(dx_r, dy_r))
                steer_cmd = float(np.clip(-math.atan2(2.0 * WHEELBASE * math.sin(alpha_rev), dist_r), -MAX_STEERING_RAD, MAX_STEERING_RAD))
            else:
                steer_cmd = 0.0
            speed_cmd = -abs(float(self.genome.reverse_speed))
            diag["cmd_speed_mps"] = speed_cmd
            diag["cmd_steer_rad"] = steer_cmd

        self.active_steer_rad = steer_cmd
        diag["cte_m"] = cte
        diag["v_ref"] = v_ref

        # 4. Pure Friction-Based Acceleration & Deceleration Slew Limiting (Strictly Symmetrical MU Limit)
        dt_cmd = timestamp_s - self.last_step_t if self.last_step_t > 0.0 else 0.010
        self.last_step_t = timestamp_s
        dt_cmd = max(0.005, min(0.050, dt_cmd))

        # Physical limit: a_max = mu_friction * g (symmetrical for acceleration, braking, and reverse)
        a_max_mps2 = float(self.genome.mu_friction) * 9.81
        max_dv = a_max_mps2 * dt_cmd
        dv = speed_cmd - self.smoothed_v
        self.smoothed_v += max(-max_dv, min(max_dv, dv))

        # 5. Map Virtual Bicycle Setpoint to 4WD Ackermann Actuation
        actuators = self.ediff.update(
            target_v=self.smoothed_v,
            target_steer_rad=steer_cmd,
            gyro_z_rad_s=gyro_z,
        )

        return actuators, diag, costmap_time_ms


def read_tof_multi_ray(dev: Any, noise_pct: float = 0.04) -> float:
    """
    Reads a ToF Lidar node range image with multiple rays, returns the shortest valid ray,
    and applies realistic +-4% measurement noise.
    """
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

    # Inject realistic +-4% sensor noise (only on real returns < max range)
    if val < (TOF_MAX_RANGE_M - 0.01) and noise_pct > 0.0:
        noise = (random.random() * 2.0 - 1.0) * noise_pct
        val = val * (1.0 + noise)

    return max(TOF_MIN_RANGE_M, min(TOF_MAX_RANGE_M, val))


def get_wsl_ip() -> str:
    """Discovers WSL2 virtual machine IP address for cross-VM UDP communication."""
    try:
        import subprocess
        out = subprocess.check_output(["wsl", "-e", "hostname", "-I"], text=True).strip()
        if out:
            return out.split()[0]
    except Exception:
        pass
    return "127.0.0.1"


def main():
    if not WEBOTS_AVAILABLE:
        print("[MPPI Controller] Webots environment not found. Exiting.", flush=True)
        return

    robot_mode = os.environ.get("ROBOT_MODE", "mppi").lower()
    wsl_ip = get_wsl_ip()
    port_offset = int(os.environ.get("MPPI_PORT_OFFSET", "0"))

    print("================================================================================", flush=True)
    print(f"  STARTING DEDICATED PURE MPPI CONTROLLER (WSL IP: {wsl_ip}, Port Offset: {port_offset})", flush=True)
    print("================================================================================", flush=True)

    try:
        robot = Supervisor()
        is_supervisor = True
        robot_node = robot.getSelf()
    except Exception:
        robot = Robot()
        is_supervisor = False
        robot_node = None

    # Load candidate genome from environment or file if provided
    genome_json_str = os.environ.get("MPPI_GENOME_JSON", "")
    genome_file = os.environ.get("MPPI_GENOME_FILE", "")
    active_genome = None

    if genome_json_str:
        try:
            parsed_data = json.loads(genome_json_str)
            if isinstance(parsed_data, list):
                active_genome = MPPIGenome.from_array(np.array(parsed_data, dtype=np.float64))
            elif isinstance(parsed_data, dict):
                active_genome = MPPIGenome(**parsed_data)
            print(f"[MPPI Controller] Injected candidate MPPIGenome (H={active_genome.H}, K={active_genome.K}, dt={active_genome.dt:.3f}).", flush=True)
        except Exception as e:
            print(f"[MPPI Controller] Warning: Failed to parse MPPI_GENOME_JSON: {e}", flush=True)
    elif genome_file and os.path.exists(genome_file):
        try:
            with open(genome_file, "r") as gf:
                parsed_data = json.load(gf)
            if isinstance(parsed_data, list):
                active_genome = MPPIGenome.from_array(np.array(parsed_data, dtype=np.float64))
            elif isinstance(parsed_data, dict):
                active_genome = MPPIGenome(**parsed_data)
            print(f"[MPPI Controller] Injected candidate MPPIGenome from {genome_file}.", flush=True)
        except Exception as e:
            print(f"[MPPI Controller] Warning: Failed to load MPPI_GENOME_FILE: {e}", flush=True)

    driver = PureMPPIDriver(genome=active_genome)

    # 1. Initialize 4WD Drive Motors
    motors = {
        "FL": robot.getDevice("motor FL"),
        "FR": robot.getDevice("motor FR"),
        "RL": robot.getDevice("motor RL"),
        "RR": robot.getDevice("motor RR"),
    }
    for m in motors.values():
        if m:
            m.setPosition(float("inf"))
            m.setVelocity(0.0)

    # 2. Initialize Steering Servos
    servos = {
        "left": robot.getDevice("servo left"),
        "right": robot.getDevice("servo right"),
    }
    for s in servos.values():
        if s:
            s.setPosition(0.0)

    # 3. Initialize BNO08X IMU, Gyro & Accelerometer
    imu_dev = robot.getDevice("bno08x_imu") or robot.getDevice("inertial unit") or robot.getDevice("imu")
    if imu_dev:
        imu_dev.enable(TIME_STEP_MS)

    gyro = robot.getDevice("gyro") or robot.getDevice("bno08x_gyro")
    if gyro:
        gyro.enable(TIME_STEP_MS)

    accel = robot.getDevice("accelerometer") or robot.getDevice("bno08x_accel")
    if accel:
        accel.enable(TIME_STEP_MS)

    # 4. Initialize 16 ToF Lidar Sensors (for local obstacle avoidance & moving obstacle detection)
    tof_devices = {}
    for dev_name in TOF_SENSOR_NAMES:
        dev = robot.getDevice(dev_name)
        if dev:
            dev.enable(TIME_STEP_MS)
            tof_devices[dev_name] = dev

    print(f"[MPPI Controller] Enabled {len(tof_devices)}/16 ToF Lidar sensors (max range {TOF_MAX_RANGE_M}m, 50Hz, +-4% noise).", flush=True)
    print(f"[MPPI Controller] Ground-truth Supervisor pose extraction active (zero latency, zero noise).", flush=True)

    # 5. Initialize 3 Moving Obstacles (0.50 m/s along racing line, 1/3 lap spacing)
    enable_moving_obstacle = os.environ.get("ENABLE_MOVING_OBSTACLE", "1") == "1"
    moving_obs_speed = float(os.environ.get("MOVING_OBS_SPEED", "0.50"))
    obs_start_s = float(os.environ.get("MOVING_OBS_START_S", "2.50"))
    obs_trans_fields = []
    if is_supervisor and enable_moving_obstacle:
        for def_name in ("MOVING_OBSTACLE", "MOVING_OBSTACLE_2", "MOVING_OBSTACLE_3"):
            node = robot.getFromDef(def_name)
            if node is not None:
                field = node.getField("translation")
                if field is not None:
                    obs_trans_fields.append(field)
        if obs_trans_fields:
            print(f"[MPPI Controller] Enabled {len(obs_trans_fields)} Moving Obstacles (1kg) at {moving_obs_speed:.2f} m/s along racing line.", flush=True)

    # GUI Telemetry Broadcast Socket (Port 9876 for costmap_gui.py)
    gui_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gui_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # GUI Command Listener Socket (Port 9877 from costmap_gui.py)
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        cmd_sock.bind(("127.0.0.1", 9877))
        cmd_sock.setblocking(False)
    except Exception as e:
        try:
            cmd_sock.bind(("", 9877))
            cmd_sock.setblocking(False)
        except Exception:
            pass

    # Simulation Control State (Default 1.00x real-time speed)
    sim_paused = False
    single_step = False
    speed_mult = 1.0

    # Initialize Vehicle Pose at Track Start (0.60m, 0.00m, 0.00 rad)
    rx = 0.60
    ry = 0.00
    ryaw = 0.00

    step_count = 0
    prev_pos = (rx, ry)
    prev_time = 0.0
    v_est = 0.0
    cached_tof_readings = {name: TOF_MAX_RANGE_M for name in TOF_SENSOR_NAMES}

    max_sim_seconds = float(os.environ.get("WEBOTS_RUN_SECONDS", "0"))

    # Evaluation telemetry accumulators
    distance_traveled_m = 0.0
    speeds = []
    steering_angles = []
    wall_proximity_costs = []
    collision_count = 0
    laps_completed = 0
    lap_times = []
    last_lap_time = 0.0
    prev_station_idx = 0
    stuck_start_time = None
    stuck_abort = False
    consecutive_near_wall_ticks = 0

    while True:
        wall_start_t = time.perf_counter()

        def process_cmd_packet(cmd_pkt: dict):
            nonlocal sim_paused, single_step, speed_mult, distance_traveled_m, step_count, now
            action = cmd_pkt.get("cmd")
            if action == "play":
                sim_paused = False
            elif action == "pause":
                sim_paused = True
            elif action == "step":
                sim_paused = False
                single_step = True
            elif action == "set_speed":
                speed_mult = float(cmd_pkt.get("speed_multiplier", cmd_pkt.get("val", 1.0)))
            elif action == "reset":
                if robot_trans_field is not None:
                    robot_trans_field.setSFVec3f([0.60, 0.00, 0.04])
                if robot_rot_field is not None:
                    robot_rot_field.setSFRotation([0.0, 0.0, 1.0, 0.0])
                distance_traveled_m = 0.0
                step_count = 0
                now = 0.0
            elif action in ("set_params", "set_inflation"):
                if action == "set_inflation":
                    new_params = {"r_inflation_m": float(cmd_pkt.get("r_inflation_m", 0.088))}
                else:
                    new_params = cmd_pkt.get("params", {})

                for k, v in new_params.items():
                    if hasattr(driver.genome, k):
                        orig_type = type(getattr(driver.genome, k))
                        setattr(driver.genome, k, orig_type(v))

                driver.mppi.genome = driver.genome
                driver.costmap_pipe.set_inflation_radius(float(driver.genome.r_inflation_m))

                win_json = os.path.join(WORKSPACE_ROOT, "winning_mppi_genome.json")
                try:
                    save_data = {
                        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                        "genes": driver.genome.to_dict(),
                        "flat_array": driver.genome.to_array().tolist(),
                    }
                    with open(win_json, "w") as f:
                        json.dump(save_data, f, indent=2)
                except Exception as e:
                    print(f"[MPPI Controller] Error saving genome: {e}", flush=True)
                print(f"[MPPI Controller] Hot-reloaded genome parameters: {list(new_params.keys())}", flush=True)

        # 1. Non-blocking read of GUI commands from port 9877
        try:
            while True:
                cmd_data, _ = cmd_sock.recvfrom(4096)
                cmd_pkt = json.loads(cmd_data.decode("utf-8"))
                process_cmd_packet(cmd_pkt)
        except (BlockingIOError, socket.error):
            pass
        except Exception:
            pass

        # 2. Pause Handling
        while sim_paused and not single_step:
            if motors["FL"]: motors["FL"].setVelocity(0.0)
            if motors["FR"]: motors["FR"].setVelocity(0.0)
            if motors["RL"]: motors["RL"].setVelocity(0.0)
            if motors["RR"]: motors["RR"].setVelocity(0.0)
            if robot.step(TIME_STEP_MS) == -1:
                break
            time.sleep(0.02)
            try:
                while True:
                    cmd_data, _ = cmd_sock.recvfrom(4096)
                    cmd_pkt = json.loads(cmd_data.decode("utf-8"))
                    process_cmd_packet(cmd_pkt)
            except (BlockingIOError, socket.error):
                pass
            except Exception:
                pass

        # 3. Advance Webots Physics Step (100 Hz)
        if robot.step(TIME_STEP_MS) == -1:
            break

        step_count += 1
        now = robot.getTime()

        dt_step = max(1e-4, now - prev_time) if prev_time > 0.0 else (TIME_STEP_MS / 1000.0)
        if single_step:
            sim_paused = True
            single_step = False

        # --- UPDATE 3 MOVING OBSTACLES (0.50 m/s along racing line, 1/3 lap spacing) ---
        if enable_moving_obstacle and obs_trans_fields and driver.frenet_center is not None:
            frenet = driver.frenet_center
            tot_len = frenet.total_length
            n_obs = len(obs_trans_fields)
            for k, field in enumerate(obs_trans_fields):
                obs_s = (obs_start_s + (k * tot_len / n_obs) + moving_obs_speed * now) % tot_len
                obs_idx = int(np.interp(obs_s, frenet.traj_s, np.arange(frenet.n_pts))) % frenet.n_pts
                field.setSFVec3f([float(frenet.traj_x[obs_idx]), float(frenet.traj_y[obs_idx]), 0.075])

        if max_sim_seconds > 0 and now >= max_sim_seconds:
            print(f"[MPPI Controller] Reached time limit {max_sim_seconds:.1f}s. Exiting cleanly.", flush=True)
            break

        # A. Read Direct Ground-Truth Pose & Kinematics from Webots Supervisor (Zero Latency, Zero Noise)
        carto_wz = 0.0
        if robot_node is not None:
            pos = robot_node.getPosition()
            rot = robot_node.getOrientation()
            vel = robot_node.getVelocity()
            rx = float(pos[0])
            ry = float(pos[1])
            # Robot local body axis: +Y is Forward, +X is Right
            # Webots rotation matrix columns: Col 0 is local X (rot[0], rot[3]), Col 1 is local Y (rot[1], rot[4])
            # True forward heading in world XY plane is atan2(Col1_Y, Col1_X)
            ryaw = math.atan2(float(rot[4]), float(rot[1]))
            vx = float(vel[0])
            vy = float(vel[1])
            # True signed longitudinal velocity (positive forward, negative in reverse)
            v_est = vx * math.cos(ryaw) + vy * math.sin(ryaw)
            carto_wz = float(vel[5])

        # Accumulate odometer distance
        step_dist = math.hypot(rx - prev_pos[0], ry - prev_pos[1])
        if now > 1.5:  # Only count distance after warmup hold
            distance_traveled_m += step_dist
            speeds.append(v_est)

        prev_pos = (rx, ry)
        prev_time = now

        # D. Read 16 ToF Lidar Distances
        min_all_m = TOF_MAX_RANGE_M
        for dev_name in TOF_SENSOR_NAMES:
            dev = tof_devices.get(dev_name)
            if dev:
                val = read_tof_multi_ray(dev, noise_pct=0.04)
            else:
                val = TOF_MAX_RANGE_M
            clamped_tof = max(TOF_MIN_RANGE_M, min(TOF_MAX_RANGE_M, val))
            cached_tof_readings[dev_name] = clamped_tof
            if clamped_tof < min_all_m:
                min_all_m = clamped_tof

        tof_readings = cached_tof_readings.copy()
        min_all_cm = min_all_m * 100.0

        # Collision / Proximity Tracking (wall impact logging without terminating)
        if now > 1.5:
            if min_all_cm < 3.0:
                consecutive_near_wall_ticks += 1
                if consecutive_near_wall_ticks >= 3:
                    collision_count += 1
            else:
                consecutive_near_wall_ticks = 0

        # E. Execute MPPI Step
        actuators, diag, map_time_ms = driver.step(
            rx=rx,
            ry=ry,
            ryaw=ryaw,
            v_mps=v_est,
            gyro_z=carto_wz,
            tof_readings=tof_readings,
            timestamp_s=now,
        )

        if now > 1.5:
            steering_angles.append(diag.get("cmd_steer_rad", 0.0))
            wall_proximity_costs.append(diag.get("best_cost", 0.0))

            # Lap Completion Detection (Station index loop 800+ -> <150)
            curr_st_idx = driver.curr_station_idx
            if prev_station_idx > 800 and curr_st_idx < 150:
                laps_completed += 1
                lap_t = now - last_lap_time if last_lap_time > 0.0 else now
                lap_times.append(round(lap_t, 3))
                last_lap_time = now
                print(f"[MPPI Controller] >>> LAP {laps_completed} COMPLETED! Time: {lap_t:.2f}s <<<", flush=True)
            prev_station_idx = curr_st_idx

            # Low Speed / Reverse Recovery Notice (no early abort)
            if abs(v_est) < 0.15:
                if stuck_start_time is None:
                    stuck_start_time = now
            else:
                stuck_start_time = None

        # Startup Warmup Hold: Keep vehicle stationary for first 1.5s
        WARMUP_HOLD_SECONDS = 1.5
        if now < WARMUP_HOLD_SECONDS:
            driver.smoothed_v = 0.0
            actuators = {
                "omega_FL": 0.0, "omega_FR": 0.0,
                "omega_RL": 0.0, "omega_RR": 0.0,
                "steer_left": 0.0, "steer_right": 0.0,
            }

        # F. Apply Actuators
        if motors["FL"]: motors["FL"].setVelocity(actuators["omega_FL"])
        if motors["FR"]: motors["FR"].setVelocity(actuators["omega_FR"])
        if motors["RL"]: motors["RL"].setVelocity(actuators["omega_RL"])
        if motors["RR"]: motors["RR"].setVelocity(actuators["omega_RR"])

        if servos["left"]: servos["left"].setPosition(actuators["steer_left"])
        if servos["right"]: servos["right"].setPosition(actuators["steer_right"])

        # H. Clean Telemetry Output & Live GUI Broadcast
        if step_count % 2 == 0:
            try:
                # Project global track reference lines into vehicle local frame for continuous display
                track_ref_center = []
                track_ref_left = []
                track_ref_right = []

                if driver.frenet_center is not None:
                    fc = driver.frenet_center
                    n_pts = fc.n_pts
                    s_cur = fc.traj_s[driver.curr_station_idx % n_pts]
                    s_sample = (s_cur + np.linspace(-1.5, 5.5, 60)) % fc.total_length
                    idx_sample = np.searchsorted(fc.traj_s, s_sample) % n_pts

                    cos_y = math.cos(ryaw)
                    sin_y = math.sin(ryaw)

                    gx = fc.traj_x[idx_sample]
                    gy = fc.traj_y[idx_sample]
                    dx = gx - rx
                    dy = gy - ry
                    fwd = dx * cos_y + dy * sin_y
                    lat = dx * sin_y - dy * cos_y
                    track_ref_center = [[round(float(lat[i]), 3), round(float(fwd[i]), 3)] for i in range(len(lat))]

                    # Left reference line (+25cm corridor)
                    if driver.frenet_left is not None:
                        gx_l = driver.frenet_left.traj_x[idx_sample]
                        gy_l = driver.frenet_left.traj_y[idx_sample]
                    else:
                        tyaw = fc.traj_yaw[idx_sample]
                        gx_l = gx - np.sin(tyaw) * 0.25
                        gy_l = gy + np.cos(tyaw) * 0.25
                    dx_l = gx_l - rx
                    dy_l = gy_l - ry
                    fwd_l = dx_l * cos_y + dy_l * sin_y
                    lat_l = dx_l * sin_y - dy_l * cos_y
                    track_ref_left = [[round(float(lat_l[i]), 3), round(float(fwd_l[i]), 3)] for i in range(len(lat_l))]

                    # Right reference line (-25cm corridor)
                    if driver.frenet_right is not None:
                        gx_r = driver.frenet_right.traj_x[idx_sample]
                        gy_r = driver.frenet_right.traj_y[idx_sample]
                    else:
                        tyaw = fc.traj_yaw[idx_sample]
                        gx_r = gx + np.sin(tyaw) * 0.25
                        gy_r = gy - np.cos(tyaw) * 0.25
                    dx_r = gx_r - rx
                    dy_r = gy_r - ry
                    fwd_r = dx_r * cos_y + dy_r * sin_y
                    lat_r = dx_r * sin_y - dy_r * cos_y
                    track_ref_right = [[round(float(lat_r[i]), 3), round(float(fwd_r[i]), 3)] for i in range(len(lat_r))]

                # Downsample candidate rollouts for network efficiency
                cand_trajs_sub = None
                cand_costs_sub = None
                if diag.get("cand_trajs_local") is not None:
                    raw_trajs = diag["cand_trajs_local"]
                    raw_costs = diag.get("cand_costs")
                    n_cands = len(raw_trajs)
                    step_sub = max(1, n_cands // 64)
                    cand_trajs_sub = [
                        [[round(float(p[0]), 3), round(float(p[1]), 3)] for p in raw_trajs[i]]
                        for i in range(0, n_cands, step_sub)
                    ]
                    if raw_costs is not None:
                        cand_costs_sub = [round(float(raw_costs[i]), 1) for i in range(0, n_cands, step_sub)]

                gui_pkt = {
                    "time": float(now),
                    "pos": [float(rx), float(ry)],
                    "yaw": float(ryaw),
                    "v": float(v_est),
                    "tof": {k: float(v) for k, v in tof_readings.items()},
                    "speed": float(diag.get("cmd_speed_mps", 0.0)),
                    "steer": float(diag.get("cmd_steer_rad", 0.0)),
                    "cte": float(diag.get("cte_m", 0.0)),
                    "best_cost": float(diag.get("best_cost", 0.0)),
                    "collision_rate": float(diag.get("collision_rate", 0.0)),
                    "seed_weights": diag.get("seed_weights", {}),
                    "best_traj": diag.get("best_traj_local", None),
                    "seed_center": diag.get("seed_center_local", None),
                    "seed_straight": diag.get("seed_straight_local", None),
                    "seed_left": diag.get("seed_left_local", None),
                    "seed_right": diag.get("seed_right_local", None),
                    "cand_trajs": cand_trajs_sub,
                    "cand_costs": cand_costs_sub,
                    "track_ref_center": track_ref_center,
                    "track_ref_left": track_ref_left,
                    "track_ref_right": track_ref_right,
                    "sim_state": {"paused": sim_paused, "speed_mult": speed_mult},
                }
                raw_bytes = json.dumps(gui_pkt).encode("utf-8")
                comp_bytes = zlib.compress(raw_bytes)
                gui_sock.sendto(comp_bytes, ("127.0.0.1", 9876))
            except Exception as e:
                print(f"[MPPI Telemetry Error] {e}", flush=True)

        if step_count % 10 == 0:
            w_c = diag["seed_weights"].get("center", 0.0)
            w_s = diag["seed_weights"].get("straight", 0.0)
            w_l = diag["seed_weights"].get("left", 0.0)
            w_r = diag["seed_weights"].get("right", 0.0)
            w_rev = diag["seed_weights"].get("reverse", 0.0)
            print(
                f"[MPPI] T={now:5.2f}s | v={v_est:4.2f}m/s (tgt: {diag['cmd_speed_mps']:4.2f}) | "
                f"steer={math.degrees(diag['cmd_steer_rad']):+5.1f}° | "
                f"MPPI={diag['exec_time_ms']:4.2f}ms (Map={map_time_ms:4.2f}ms) | "
                f"Cost={diag['best_cost']:6.1f} | Seeds(C/S/L/R/Rev)={w_c:.2f}/{w_s:.2f}/{w_l:.2f}/{w_r:.2f}/{w_rev:.2f} | "
                f"Pruned={diag['collision_rate']*100:4.1f}% | ToF_min={min_all_cm:3.0f}cm | "
                f"Dist={distance_traveled_m:5.1f}m | CTE={diag['cte_m']*100:+5.1f}cm",
                flush=True,
            )

        # I. Simulation Speed Throttling (0.05x to 100.0x)
        if speed_mult < 4.9:
            target_step_dt = (TIME_STEP_MS / 1000.0) / max(0.01, speed_mult)
            elapsed_wall = time.perf_counter() - wall_start_t
            if elapsed_wall < target_step_dt:
                time.sleep(target_step_dt - elapsed_wall)

    # Calculate Evaluation Result Summary
    mean_speed = float(np.mean(speeds)) if speeds else 0.0
    max_speed = float(np.max(speeds)) if speeds else 0.0
    steering_var = float(np.var(steering_angles)) if steering_angles else 0.0
    mean_wall_cost = float(np.mean(wall_proximity_costs)) if wall_proximity_costs else 0.0

    eval_result = {
        "status": "STUCK_ABORT" if stuck_abort else "COMPLETED",
        "distance_m": round(distance_traveled_m, 3),
        "mean_speed_mps": round(mean_speed, 3),
        "max_speed_mps": round(max_speed, 3),
        "laps_completed": laps_completed,
        "lap_times": lap_times,
        "best_lap_time_s": round(min(lap_times), 3) if lap_times else 999.0,
        "collision_count": collision_count,
        "mean_wall_proximity_cost": round(mean_wall_cost, 2),
        "steering_variance": round(steering_var, 5),
        "stuck_abort": stuck_abort,
        "sim_duration_s": round(robot.getTime(), 3),
    }

    eval_file = os.environ.get("MPPI_EVAL_FILE", os.path.join(WORKSPACE_ROOT, f"eval_res_{port_offset}.json"))
    try:
        with open(eval_file, "w", encoding="utf-8") as ef:
            json.dump(eval_result, ef, indent=2)
    except Exception as e:
        print(f"[MPPI Controller] Error saving eval file {eval_file}: {e}", flush=True)

    print(f"\n================================================================================", flush=True)
    print(f" [EVAL_RESULT_JSON] {json.dumps(eval_result)}", flush=True)
    print(f"================================================================================\n", flush=True)
    sys.stdout.flush()
    sys.stderr.flush()

    if is_supervisor:
        robot.simulationQuit(0)
    sys.exit(0)


if __name__ == "__main__":
    main()
