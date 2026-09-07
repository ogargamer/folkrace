"""
================================================================================
  FOLKRACE RACING-LINE-CENTRIC 12-CHANNEL ToF CONTROLLER
  Closed-Loop 4WD Electronic Differential, True Ackermann Steering,
  Frenet-Frame Optimization Suite (1D-QP, Elastic Band, Lattice, Disparity Extender)
================================================================================
"""

import os
import sys
import math
import json
import numpy as np

try:
    from controller import Robot, Supervisor, Keyboard
except ImportError:
    pass

# --- IMPORT ELECTRONIC DIFFERENTIAL & ToF PATHFINDERS ---
from electronic_diff_4wd import (
    ElectronicDifferential4WD,
    WHEELBASE,
    WHEEL_RADIUS,
    TRACK_REAR,
    TRACK_FRONT,
    TRACK_KINGPIN,
    MAX_STEERING_RAD,
    MOTOR_DIR,
    V_MAX_WHEEL,
    SLIP_NORMAL,
    SLIP_OVERSTEER,
    SLIP_UNDERSTEER
)

from tof_pathfinders import (
    NUM_TOF_ZONES,
    TOF_ZONE_ANGLES,
    TOF_MAX_RANGE_M,
    TOF_MIN_RANGE_M,
    FrenetTransformer,
    FrenetCorridorQP,
    GaussianElasticBand,
    FrenetLatticePlanner,
    BiasedDisparityExtender,
    FrenetMPCC,
    WrongWayDetector,
    StuckRecoveryManager,
    StuckRecoveryState
)

# --- IMPORT MPPI CONTROLLER & 60 Hz COSTMAP PIPELINE ---
SCANNER_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tof_16ch_grid_scanner")
if SCANNER_DIR not in sys.path and os.path.exists(SCANNER_DIR):
    sys.path.insert(0, SCANNER_DIR)

from mppi_folkrace_controller import MPPIController, MPPIGenome
try:
    from fast_tof_carto_costmap import FastCostmapPipeline, SENSOR_RIG
    FAST_COSTMAP_AVAILABLE = True
except ImportError:
    FAST_COSTMAP_AVAILABLE = False

TIME_STEP = 10             # ms (100 Hz control loop, matches Webots basicTimeStep)
CONFIG_FILE_NAME = "folkrace_config.json"
FINISH_LINE_CENTER = [0.0, 0.0]  # [X, Y] world coordinates of the finish line center


class DynamicConfig:
    """
    Manages live vehicle parameters with auto-sync from JSON file and keyboard tuning.
    """
    def __init__(self, config_path):
        self.config_path = config_path
        self.last_mtime = 0.0
        
        # High-Performance Default Parameters
        self.top_speed_m_s = 4.40           # Top linear speed ceiling (160 rad/s * 0.0275 m)
        self.accel_limit_m_s2 = 12.0        # Max longitudinal acceleration (m/s^2)
        self.brake_limit_m_s2 = 16.0        # Max braking deceleration (m/s^2)
        self.steer_rate_limit_rad_s = 10.47 # Max steering slew rate (EMAX ES08MD II @ 6.0V = 10.47 rad/s)
        self.spinout_thresh_rad_s = 0.45    # IMU slip detection threshold (rad/s)
        self.traction_cut_ratio = 0.25      # Speed cut ratio during gross oversteer
        self.race_pace_pct = 80.0           # Default Champion Race Pace (80% safe default)
        
        # Active Pathfinder: "FRENETQP", "ELASTICBAND", "LATTICE", "DISPARITYEXTENDER"
        self.active_pathfinder = "FRENETQP"
        self.reverse_speed_m_s = 1.20       # Reverse recovery backing speed
        self.lead_time_s = 0.065            # Preview steering feedforward lead time (s)
        self.front_proj_m = 0.00            # Front-axle projected cross-track lookahead (m)
        self.k_cte = 2.20                   # Proportional cross-track gain
        self.k_d = 0.00                     # Lateral drift velocity damping gain
        self.k_ug = 0.060                   # Dynamic understeer gradient compensation
        
        self.load_from_file()

    def reset_to_defaults(self):
        self.top_speed_m_s = 4.40
        self.accel_limit_m_s2 = 12.0
        self.brake_limit_m_s2 = 16.0
        self.steer_rate_limit_rad_s = 10.47
        self.spinout_thresh_rad_s = 0.45
        self.traction_cut_ratio = 0.25
        self.race_pace_pct = 85.0
        self.active_pathfinder = "FRENETQP"
        self.reverse_speed_m_s = 1.20
        self.lead_time_s = 0.065
        self.front_proj_m = 0.00
        self.k_cte = 2.20
        self.k_d = 0.00
        self.k_ug = 0.060
        self.save_to_file()
        print("[CONFIG] Reset all parameters to maximum performance defaults.", flush=True)

    def load_from_file(self):
        if not os.path.exists(self.config_path):
            self.save_to_file()
            return
        try:
            mtime = os.path.getmtime(self.config_path)
            if mtime != self.last_mtime:
                with open(self.config_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                self.top_speed_m_s = float(data.get("top_speed_m_s", self.top_speed_m_s))
                self.accel_limit_m_s2 = float(data.get("accel_limit_m_s2", self.accel_limit_m_s2))
                self.brake_limit_m_s2 = float(data.get("brake_limit_m_s2", self.brake_limit_m_s2))
                self.steer_rate_limit_rad_s = float(data.get("steer_rate_limit_rad_s", self.steer_rate_limit_rad_s))
                self.spinout_thresh_rad_s = float(data.get("spinout_thresh_rad_s", self.spinout_thresh_rad_s))
                self.traction_cut_ratio = float(data.get("traction_cut_ratio", self.traction_cut_ratio))
                self.race_pace_pct = float(data.get("race_pace_pct", self.race_pace_pct))
                self.active_pathfinder = str(data.get("active_pathfinder", self.active_pathfinder)).upper()
                self.reverse_speed_m_s = float(data.get("reverse_speed_m_s", self.reverse_speed_m_s))
                self.lead_time_s = float(data.get("lead_time_s", 0.035))
                self.front_proj_m = float(data.get("front_proj_m", 0.12))
                self.k_cte = float(data.get("k_cte", 1.40))
                self.k_d = float(data.get("k_d", 0.00))
                self.k_ug = float(data.get("k_ug", 0.040))
                self.last_mtime = mtime
                print(f"[CONFIG] Live parameters reloaded from {os.path.basename(self.config_path)}", flush=True)
        except Exception as e:
            print(f"[CONFIG] Error loading JSON config: {e}", flush=True)

    def save_to_file(self):
        data = {
            "top_speed_m_s": round(self.top_speed_m_s, 2),
            "accel_limit_m_s2": round(self.accel_limit_m_s2, 1),
            "brake_limit_m_s2": round(self.brake_limit_m_s2, 1),
            "steer_rate_limit_rad_s": round(self.steer_rate_limit_rad_s, 2),
            "spinout_thresh_rad_s": round(self.spinout_thresh_rad_s, 2),
            "traction_cut_ratio": round(self.traction_cut_ratio, 2),
            "race_pace_pct": round(self.race_pace_pct, 1),
            "active_pathfinder": self.active_pathfinder,
            "reverse_speed_m_s": round(self.reverse_speed_m_s, 2),
            "lead_time_s": round(self.lead_time_s, 3),
            "front_proj_m": round(self.front_proj_m, 3),
            "k_cte": round(self.k_cte, 2),
            "k_d": round(self.k_d, 2),
            "k_ug": round(self.k_ug, 3)
        }
        try:
            with open(self.config_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            self.last_mtime = os.path.getmtime(self.config_path)
        except Exception as e:
            print(f"[CONFIG] Error saving JSON config: {e}", flush=True)

    def print_dashboard(self):
        print("\n" + "=" * 80, flush=True)
        print("  FOLKRACE RACING-LINE-CENTRIC 12-CHANNEL ToF DASHBOARD", flush=True)
        print("=" * 80, flush=True)
        print(f"  Active Pathfinder  : [{self.active_pathfinder}]", flush=True)
        print(f"    Options: '1'=Frenet 1D-QP | '2'=Elastic Band | '3'=Lattice | '4'=Disparity Extender", flush=True)
        print(f"  [O / L] Race Pace  : {self.race_pace_pct:5.1f} %  (O = +5%, L = -5%)", flush=True)
        print(f"  [T / G] Top Speed  : {self.top_speed_m_s:5.2f} m/s ({self.top_speed_m_s*3.6:4.1f} km/h)", flush=True)
        print(f"  [Y / H] Max Accel  : {self.accel_limit_m_s2:5.1f} m/s^2 | [U / J] Max Braking: {self.brake_limit_m_s2:5.1f} m/s^2", flush=True)
        print(f"  [I / K] Steer Rate : {self.steer_rate_limit_rad_s:5.2f} rad/s ({math.degrees(self.steer_rate_limit_rad_s):3.0f}°/s)", flush=True)
        print("  ----------------------------------------------------------------------------", flush=True)
        print("  Modes: [M] Manual Drive    | [Q] Autonomous Drive | [P] Show Table | [R] Reset", flush=True)
        print("=" * 80 + "\n", flush=True)


class SlewRateLimiter:
    """Applies smooth acceleration and steering slew rate bounds."""
    def __init__(self):
        self.v_current = 0.0
        self.steer_current = 0.0

    def filter(self, target_v, target_steer, cfg, dt):
        v_diff = target_v - self.v_current
        if v_diff > 0:
            max_step = cfg.accel_limit_m_s2 * dt
            self.v_current += min(v_diff, max_step)
        else:
            max_step = cfg.brake_limit_m_s2 * dt
            self.v_current += max(v_diff, -max_step)

        self.v_current = max(-2.0, min(self.v_current, cfg.top_speed_m_s))

        steer_diff = target_steer - self.steer_current
        max_steer_step = cfg.steer_rate_limit_rad_s * dt
        self.steer_current += max(-max_steer_step, min(steer_diff, max_steer_step))
        self.steer_current = max(-MAX_STEERING_RAD, min(self.steer_current, MAX_STEERING_RAD))

        return self.v_current, self.steer_current


def load_trajectory():
    """Load the pre-computed optimal Cyan Champion racing trajectory."""
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_root = os.path.dirname(os.path.dirname(curr_dir))
    csv_candidates = [
        os.path.join(workspace_root, "racing_results", "tum_qp_obstacle_aware_25cm_offsets", "master_cyan_champion_optimal.csv"),
        os.path.join(workspace_root, "racing_results", "tum_qp_pillar_racing_lines", "master_cyan_champion_optimal.csv"),
        os.path.join(workspace_root, "racing_results", "backup_champion_latest", "cyan_champion_trajectory.csv"),
        os.path.join(curr_dir, "trajectories", "cyan_champion_trajectory.csv"),
        os.path.join(curr_dir, "cyan_champion_trajectory.csv"),
        r"c:\Users\nurmi\Documents\Da Winna\racing_results\backup_champion_cyan_line\cyan_champion_trajectory.csv",
    ]
    
    found_path = None
    for p in csv_candidates:
        if os.path.exists(p):
            found_path = p
            break
            
    if not found_path:
        print("[Folkrace Autodrive] Note: cyan_champion_trajectory.csv not found.", flush=True)
        return None
        
    rows = []
    try:
        with open(found_path, "r", encoding="utf-8") as f:
            lines = f.readlines()
            for line in lines[1:]:
                parts = line.strip().split(",")
                if len(parts) >= 6:
                    rows.append([float(x) for x in parts])
    except Exception as e:
        print(f"[Folkrace Autodrive] Error reading trajectory file: {e}", flush=True)
        return None
                
    data = np.array(rows)
    print(f"[Folkrace Autodrive] Loaded optimal racing line: {os.path.basename(found_path)} ({len(data)} pts)", flush=True)
    return data


def main():
    print("[Folkrace Autodrive] Starting controller initialization...", flush=True)
    
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
            print(f"[Folkrace Autodrive] Error initializing Webots Robot: {e}", flush=True)
            return

    keyboard = Keyboard()
    keyboard.enable(TIME_STEP)

    curr_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(curr_dir, CONFIG_FILE_NAME)
    telemetry_log_path = os.path.join(curr_dir, "controller_telemetry.log")
    try:
        log_file = open(telemetry_log_path, "w", encoding="utf-8", buffering=1)
        log_file.write(f"=== FOLKRACE CONTROLLER INITIALIZED: {robot.getName()} ===\n")
    except Exception:
        log_file = None
    try:
        cfg = DynamicConfig(config_path)
        slew_limiter = SlewRateLimiter()

        # --- INITIALIZE 4WD ELECTRONIC DIFFERENTIAL ---
        ediff = ElectronicDifferential4WD(
            wheelbase=WHEELBASE,
            track_rear=TRACK_REAR,
            track_front=TRACK_FRONT,
            track_kingpin=TRACK_KINGPIN,
            wheel_radius=WHEEL_RADIUS,
            max_steer=MAX_STEERING_RAD,
            motor_dir=MOTOR_DIR,
            v_max=cfg.top_speed_m_s,
            thresh_spin=cfg.spinout_thresh_rad_s
        )

        from predictive_shield import PredictiveSafetyShield

        # --- INITIALIZE 6 RACING-LINE-CENTRIC PATHFINDERS + PREDICTIVE SHIELD ---
        pathfinder_qp = FrenetCorridorQP()
        pathfinder_geb = GaussianElasticBand()
        pathfinder_lattice = FrenetLatticePlanner()
        pathfinder_bde = BiasedDisparityExtender()
        pathfinder_mpcc = FrenetMPCC()
        pathfinder_mppi = MPPIController(genome=MPPIGenome())
        if FAST_COSTMAP_AVAILABLE:
            costmap_pipeline = FastCostmapPipeline(resolution=0.02, enable_latency_comp=True)
        else:
            costmap_pipeline = None

        margin_mode = os.environ.get("SHIELD_MARGIN_MODE", getattr(cfg, "shield_margin_mode", "QUADRATIC"))
        d_min_val = float(os.environ.get("SHIELD_D_MIN", getattr(cfg, "shield_d_min", 0.020)))
        d_max_val = float(os.environ.get("SHIELD_D_MAX", getattr(cfg, "shield_d_max", 0.055)))
        predictive_shield = PredictiveSafetyShield(
            horizon_s=0.90, dt_step=0.045, margin_mode=margin_mode,
            d_min_m=d_min_val, d_max_m=d_max_val, v_max_m_s=cfg.top_speed_m_s
        )
        
        wrong_way_detector = WrongWayDetector()
        recovery_manager = StuckRecoveryManager(reverse_speed_m_s=cfg.reverse_speed_m_s)

        print(f"[Folkrace Autodrive] Racing-Line-Centric Pathfinder Suite Initialized (QP, GEB, Lattice, BDE, MPCC, MPPI).", flush=True)

        # --- ACTUATOR INITIALIZATION ---
        motor_map = {"RL": "motor RL", "RR": "motor RR", "FL": "motor FL", "FR": "motor FR"}
        drive_motors = {}
        for short_key, dev_name in motor_map.items():
            m = robot.getDevice(dev_name)
            if m:
                m.setPosition(float("inf"))
                m.setVelocity(0.0)
                drive_motors[short_key] = m

        steering_servos = {}
        for name, key in [("servo left", "left"), ("servo right", "right")]:
            s = robot.getDevice(name)
            if s:
                s.setPosition(0.0)
                steering_servos[key] = s

        # --- SENSORS INITIALIZATION ---
        lidar = robot.getDevice("ms200k_lidar")
        if lidar:
            lidar.enable(10)
            lidar.enablePointCloud()

        imu = robot.getDevice("bno08x_imu")
        if imu:
            imu.enable(10)

        gyro = robot.getDevice("bno08x_gyro") or robot.getDevice("gyro")
        if gyro:
            gyro.enable(10)

        accel = robot.getDevice("bno08x_accel") or robot.getDevice("accelerometer")
        if accel:
            accel.enable(10)

        encoders = {}
        for short_key in ["RL", "RR", "FL", "FR"]:
            dev = robot.getDevice(f"encoder_{short_key}")
            if dev:
                dev.enable(10)
                encoders[short_key] = dev

        # 16-channel ToF Sensor Rig for 60 Hz Costmap Pipeline (and legacy support)
        tof_16ch_names = [
            "tof_fl_0", "tof_fl_1", "tof_fl_2", "tof_fl_3",
            "tof_fr_0", "tof_fr_1", "tof_fr_2", "tof_fr_3",
            "tof_bl_0", "tof_bl_1", "tof_bl_2", "tof_bl_3",
            "tof_br_0", "tof_br_1", "tof_br_2", "tof_br_3",
        ]
        tof_16ch_devs = {}
        for dev_name in tof_16ch_names:
            dev = robot.getDevice(dev_name)
            if dev:
                dev.enable(10)
                tof_16ch_devs[dev_name] = dev

        # Fallback for any legacy tof_0..tof_11 names if present
        for k in range(NUM_TOF_ZONES):
            dev = robot.getDevice(f"tof_{k}")
            if dev and f"tof_{k}" not in tof_16ch_devs:
                dev.enable(10)
                tof_16ch_devs[f"tof_{k}"] = dev

        print(f"[Folkrace Autodrive] Enabled {len(tof_16ch_devs)} ToF multi-ray sensor nodes (max range 1.80m).", flush=True)

        # Output JSONL for Cartographer SLAM benchmark
        slam_log_path = os.path.join(curr_dir, "live_webots_slam_scans.jsonl")
        try:
            slam_file = open(slam_log_path, "w", encoding="utf-8", buffering=1)
        except Exception:
            slam_file = None

        # Helper for loading auxiliary left/right offset trajectories
        def load_aux_frenet(filename):
            offset_dir = os.path.join(r"c:\Users\nurmi\Documents\Da Winna", "racing_results", "tum_qp_obstacle_aware_25cm_offsets")
            p = os.path.join(offset_dir, filename)
            if os.path.exists(p):
                try:
                    arr = np.loadtxt(p, delimiter=",", skiprows=1)
                    return FrenetTransformer(arr[:, 5], arr[:, 0], arr[:, 1], arr[:, 3], arr[:, 4])
                except Exception:
                    pass
            return None

        frenet_left = load_aux_frenet("wall_offset_left_25cm.csv")
        frenet_right = load_aux_frenet("wall_offset_right_25cm.csv")

        # --- LOAD TRAJECTORY & INITIALIZE FRENET TRANSFORMER ---
        traj = load_trajectory()
        traj_available = traj is not None and len(traj) > 0
        if traj_available:
            traj_x = traj[:, 0]
            traj_y = traj[:, 1]
            traj_v_base = traj[:, 2]
            traj_yaw = traj[:, 3]
            traj_curv = traj[:, 4]
            traj_dist = traj[:, 5]
            n_pts = len(traj_x)
            
            frenet_helper = FrenetTransformer(
                traj_s=traj_dist,
                traj_x=traj_x,
                traj_y=traj_y,
                traj_yaw=traj_yaw,
                traj_k=traj_curv
            )
        else:
            traj_x, traj_y, traj_v_base, traj_yaw, traj_curv, traj_dist = (
                np.array([]), np.array([]), np.array([]), np.array([]), np.array([]), np.array([])
            )
            n_pts = 0
            frenet_helper = None

        cfg.print_dashboard()

        auto_mode = True
        manual_raw_speed = 0.0
        manual_raw_steer = 0.0

        last_idx = 0
        lap_count = 0
        lap_start_time = robot.getTime()
        first_step = True
        last_cfg_poll = 0.0

        odom_x = 0.6
        odom_y = 0.0
        odom_yaw = 0.0
        current_e_lat = 0.0
        step_count = 0
        crash_count = 0

        last_enc_pos = {k: 0.0 for k in ["RL", "RR", "FL", "FR"]}
        last_enc_time = None

        max_sim_seconds = float(os.environ.get("WEBOTS_RUN_SECONDS", "0.0"))
        max_bench_laps = int(os.environ.get("MAX_BENCHMARK_LAPS", "3"))

        while robot.step(TIME_STEP) != -1:
            dt_s = TIME_STEP / 1000.0
            now = robot.getTime()
            step_count += 1

            if max_sim_seconds > 0.0 and now >= max_sim_seconds:
                print(f"\n[BENCHMARK] Target duration {max_sim_seconds:.1f}s reached in Webots (steps={step_count}). Exiting...", flush=True)
                if is_supervisor:
                    robot.simulationQuit(0)
                sys.exit(0)

            # 1. Periodic Config File Hot-Reload (every 1.0s)
            if now - last_cfg_poll > 1.0:
                cfg.load_from_file()
                ediff.v_max = cfg.top_speed_m_s
                ediff.thresh_spin = cfg.spinout_thresh_rad_s
                last_cfg_poll = now

            # 2. Read Sensors (60 Hz)
            gyro_z = 0.0
            g_vals = [0.0, 0.0, 0.0]
            if gyro:
                read_g = gyro.getValues()
                if read_g:
                    g_vals = read_g
                    gyro_z = read_g[2]

            accel_data = [0.0, 0.0, 9.81]
            if accel:
                read_a = accel.getValues()
                if read_a:
                    accel_data = list(read_a)
            if accel_data[2] < 1.0:
                accel_data[2] = 9.81

            current_yaw = odom_yaw
            rpy = [0.0, 0.0, 0.0]
            if imu:
                read_rpy = imu.getRollPitchYaw()
                if read_rpy:
                    rpy = read_rpy
                    current_yaw = (rpy[2] + math.pi / 2.0 + math.pi) % (2.0 * math.pi) - math.pi

            # Read wheel encoders
            enc_omegas = {}
            for short_key in ["RL", "RR", "FL", "FR"]:
                if short_key in encoders:
                    p_val = encoders[short_key].getValue()
                    if last_enc_time is not None and dt_s > 0:
                        enc_omegas[short_key] = (p_val - last_enc_pos[short_key]) / dt_s
                    else:
                        enc_omegas[short_key] = 0.0
                    last_enc_pos[short_key] = p_val
                else:
                    enc_omegas[short_key] = 0.0
            last_enc_time = now

            # Read Ground Truth from Webots Supervisor
            if is_supervisor and robot_node:
                gt_vel = robot_node.getVelocity() # [vx, vy, vz, wx, wy, wz] in world
                gt_rot = np.array(robot_node.getOrientation()).reshape(3, 3)
                gt_v_world = np.array(gt_vel[:3])
                gt_v_local = gt_rot.T @ gt_v_world
                # Local robot coordinate: +Y is Forward, -X is Left, +Z is Up
                # In ROS base_link standard: X is Forward (+Y local), Y is Left (-X local)
                v_gt_x = float(gt_v_local[1])
                v_gt_y = float(-gt_v_local[0])
                v_gt_z = float(gt_v_local[2])
                w_gt_z = float(gt_vel[5])
                gt_pos = robot_node.getPosition()
            else:
                v_gt_x, v_gt_y, v_gt_z, w_gt_z = 0.0, 0.0, 0.0, 0.0
                gt_pos = [odom_x, odom_y, 0.02]

            # Read ms200k 360-LiDAR (450 beams) directly from Webots
            lidar_ranges = None
            if lidar:
                raw_ranges = lidar.getRangeImage()
                if raw_ranges:
                    lidar_ranges = [round(float(r), 4) for r in raw_ranges]

            # Read 16-channel ToF Distances (multi-ray Lidar nodes up to 1.80m, shortest valid ray reported)
            tof_16ch_readings = {}
            tof_16ch_distances = []
            for name in tof_16ch_names:
                dev = tof_16ch_devs.get(name)
                if dev:
                    if hasattr(dev, "getRangeImage"):
                        r_img = dev.getRangeImage()
                        valid_r = [v for v in r_img if not math.isinf(v) and not math.isnan(v) and v <= 1.80] if r_img else []
                        val = min(valid_r) if valid_r else 1.80
                    elif hasattr(dev, "getValue"):
                        val = dev.getValue()
                        if val > 10.0:
                            val /= 1000.0
                    else:
                        val = 1.80
                else:
                    val = 1.80
                val_m = max(0.02, min(1.80, float(val)))
                tof_16ch_readings[name] = val_m
                tof_16ch_distances.append(val_m)

            # Map to legacy array format if needed
            tof_distances = np.array(tof_16ch_distances[:12], dtype=np.float32)

            # 100 Hz IMU + ~15 Hz LiDAR dump for realistic physical hardware Cartographer SLAM
            # Step is 10 ms (100 Hz). IMU is recorded every step. LiDAR is sampled every 7 steps (~14.3 Hz).
            if slam_file:
                is_lidar_step = (step_count % 7 == 0) and (lidar_ranges is not None)
                curr_pos = gt_pos if (is_supervisor and robot_node) else [odom_x, odom_y, 0.02]
                curr_rpy = [round(x, 4) for x in rpy] if (imu and rpy) else [0.0, 0.0, round(current_yaw, 4)]
                curr_gyro = [round(x, 4) for x in g_vals] if (gyro and g_vals) else [0.0, 0.0, round(gyro_z, 4)]
                curr_accel = [round(x, 4) for x in accel_data] if accel_data else [0.0, 0.0, 0.0]

                w_rl_val = enc_omegas.get("RL", 0.0)
                w_rr_val = enc_omegas.get("RR", 0.0)
                w_fl_val = enc_omegas.get("FL", 0.0)
                w_fr_val = enc_omegas.get("FR", 0.0)

                v_rl_wheel = (w_rl_val * WHEEL_RADIUS) / MOTOR_DIR
                v_rr_wheel = (w_rr_val * WHEEL_RADIUS) / MOTOR_DIR
                v_wheel_calc = 0.5 * (v_rl_wheel + v_rr_wheel)
                
                packet = {
                    "time": round(now, 4),
                    "pos": [round(curr_pos[0], 4), round(curr_pos[1], 4)],
                    "yaw": round(current_yaw, 4),
                    "rpy": curr_rpy,
                    "gyro": curr_gyro,
                    "accel": curr_accel,
                    "gyro_z": curr_gyro[2],
                    "accel_x": curr_accel[0],
                    "accel_y": curr_accel[1],
                    "accel_z": curr_accel[2],
                    "omega_RL": round(w_rl_val, 4),
                    "omega_RR": round(w_rr_val, 4),
                    "omega_FL": round(w_fl_val, 4),
                    "omega_FR": round(w_fr_val, 4),
                    "w_rl": round(w_rl_val, 4),
                    "w_rr": round(w_rr_val, 4),
                    "v_wheel": round(v_wheel_calc, 4),
                    "v_gt_x": round(v_gt_x, 4),
                    "v_gt_y": round(v_gt_y, 4),
                    "v_gt_z": round(v_gt_z, 4),
                    "w_gt_z": round(w_gt_z, 4),
                    "ranges": lidar_ranges if is_lidar_step else []
                }

                # Detect finish line crossing and record landmark
                if traj_available and n_pts > 0:
                    wrap_high = int(n_pts * 0.75)
                    wrap_low = int(n_pts * 0.25)
                    if last_idx > wrap_high and closest_idx < wrap_low:
                        rot = robot_node.getOrientation() if (is_supervisor and robot_node) else None
                        if rot:
                            dx = curr_pos[0] - FINISH_LINE_CENTER[0]
                            dy = curr_pos[1] - FINISH_LINE_CENTER[1]
                            local_x = rot[0] * (-dx) + rot[3] * (-dy)
                            local_y = rot[1] * (-dx) + rot[4] * (-dy)
                        else:
                            local_x = 0.0
                            local_y = 0.0
                        packet["landmark"] = {
                            "type": "landmark",
                            "timestamp": round(now, 4),
                            "id": "finish_line",
                            "rel_x": float(local_x),
                            "rel_y": float(local_y),
                            "rel_yaw": 0.0
                        }
                slam_file.write(json.dumps(packet) + "\n")

            min_tof_left = np.min(tof_distances[0:4])
            min_tof_center = np.min(tof_distances[4:8])
            min_tof_right = np.min(tof_distances[8:12])
            min_all_tof = np.min(tof_distances)

            # 3. Handle Keyboard Controls with Debounce Protection
            key = keyboard.getKey()
            while key > 0:
                # Normalize lowercase to uppercase ASCII for letter keys
                k_norm = key - 32 if (ord('a') <= key <= ord('z')) else key

                last_press = key_cooldown_map.get(k_norm, 0.0)
                if (now - last_press) >= KEY_DEBOUNCE_S:
                    key_cooldown_map[k_norm] = now
                    if k_norm in (ord('1'), ord('2'), ord('3'), ord('4'), ord('5'), ord('6')):
                        if k_norm == ord('1'):
                            cfg.active_pathfinder = "FRENETQP"
                            print("\n[PATHFINDER] Switched to Option 1: 1D-QP Frenet Micro-Corridor", flush=True)
                        elif k_norm == ord('2'):
                            cfg.active_pathfinder = "ELASTICBAND"
                            print("\n[PATHFINDER] Switched to Option 2: Gaussian Elastic Band (GEB)", flush=True)
                        elif k_norm == ord('3'):
                            cfg.active_pathfinder = "LATTICE"
                            print("\n[PATHFINDER] Switched to Option 3: Frenet Spatiotemporal Lattice", flush=True)
                        elif k_norm == ord('4'):
                            cfg.active_pathfinder = "DISPARITYEXTENDER"
                            print("\n[PATHFINDER] Switched to Option 4: Biased Disparity Extender (BDE)", flush=True)
                        elif k_norm == ord('5'):
                            cfg.active_pathfinder = "MPCC"
                            print("\n[PATHFINDER] Switched to Option 5: Model Predictive Contouring Controller (MPCC)", flush=True)
                        elif k_norm == ord('6'):
                            cfg.active_pathfinder = "MPPI"
                            print("\n[PATHFINDER] Switched to Option 6: Model Predictive Path Integral (MPPI 60Hz)", flush=True)
                        cfg.save_to_file()
                    elif k_norm == ord('O'):
                        cfg.race_pace_pct = min(120.0, cfg.race_pace_pct + 5.0)
                        cfg.save_to_file()
                        print(f"\n[RACE PACE] Increased to {cfg.race_pace_pct:.0f}%", flush=True)
                    elif k_norm == ord('L'):
                        cfg.race_pace_pct = max(30.0, cfg.race_pace_pct - 5.0)
                        cfg.save_to_file()
                        print(f"\n[RACE PACE] Decreased to {cfg.race_pace_pct:.0f}%", flush=True)
                    elif k_norm == ord('T'):
                        cfg.top_speed_m_s = min(6.0, cfg.top_speed_m_s + 0.5)
                        cfg.save_to_file()
                        print(f"\n[TOP SPEED] Set to {cfg.top_speed_m_s:.1f} m/s", flush=True)
                    elif k_norm == ord('G'):
                        cfg.top_speed_m_s = max(1.0, cfg.top_speed_m_s - 0.5)
                        cfg.save_to_file()
                        print(f"\n[TOP SPEED] Set to {cfg.top_speed_m_s:.1f} m/s", flush=True)
                    elif k_norm == ord('Q'):
                        auto_mode = True
                        print("\n[MODE] Switched to AUTONOMOUS RACING Mode", flush=True)
                    elif k_norm == ord('M'):
                        auto_mode = False
                        print("\n[MODE] Switched to MANUAL DRIVE Mode (WASD / Arrows)", flush=True)
                    elif k_norm == ord('P'):
                        print("", flush=True)
                        cfg.print_dashboard()
                    elif k_norm == ord('R'):
                        cfg.reset_to_defaults()
                        print("\n[CONFIG] Reset to factory defaults", flush=True)
                    elif k_norm == ord(' '):
                        manual_raw_speed = 0.0
                        manual_raw_steer = 0.0
                        auto_mode = False
                        print("\n[EMERGENCY] STOP ACTIVATED", flush=True)

                # Manual driving inputs (Continuous hold allowed)
                if not auto_mode:
                    if key in (Keyboard.UP, ord('W'), ord('w')):
                        manual_raw_speed = min(cfg.top_speed_m_s, manual_raw_speed + 0.10)
                    elif key in (Keyboard.DOWN, ord('S'), ord('s')):
                        manual_raw_speed = max(-cfg.reverse_speed_m_s, manual_raw_speed - 0.10)
                    if key in (Keyboard.LEFT, ord('A'), ord('a')):
                        manual_raw_steer = min(MAX_STEERING_RAD, manual_raw_steer + math.radians(2.5))
                    elif key in (Keyboard.RIGHT, ord('D'), ord('d')):
                        manual_raw_steer = max(-MAX_STEERING_RAD, manual_raw_steer - math.radians(2.5))

                key = keyboard.getKey()

            # 4. Autonomous / Manual Decision Layer
            if not auto_mode:
                raw_cmd_v = manual_raw_speed
                raw_cmd_steer = manual_raw_steer
            else:
                # 4A. Supervisor Ground Truth Pose Tracking
                if is_supervisor and robot_node is not None:
                    pos = robot_node.getPosition()
                    rx, ry = pos[0], pos[1]
                    ori = robot_node.getOrientation()
                    # In Webots, Folkrace model forward heading is local +Y: forward_x = ori[1], forward_y = ori[4]
                    current_yaw = math.atan2(ori[4], ori[1])
                else:
                    rx, ry = odom_x, odom_y

                suggested_speed = cfg.top_speed_m_s
                suggested_steer = 0.0
                track_flow_yaw = current_yaw
                closest_idx = last_idx
                pure_pursuit_steer = 0.0

                if traj_available and frenet_helper is not None:
                    if first_step:
                        first_step = False
                        best_d_sq = 1e9
                        init_idx = 0
                        for k in range(n_pts):
                            d2 = (traj_x[k] - rx)**2 + (traj_y[k] - ry)**2
                            if d2 < best_d_sq:
                                best_d_sq = d2
                                init_idx = k
                        last_idx = init_idx
                        start_track_idx = init_idx

                    curr_s, current_e_lat, closest_idx, track_flow_yaw, track_k = frenet_helper.cartesian_to_frenet(
                        rx, ry, last_idx
                    )

                    # Lap tracking (Finish line wrap-around crossing)
                    wrap_high = int(n_pts * 0.75)
                    wrap_low = int(n_pts * 0.25)
                    if last_idx > wrap_high and closest_idx < wrap_low:
                        lap_count += 1
                        lap_time = now - lap_start_time
                        lap_msg = f"\n>>> [LAP COMPLETED] Lap #{lap_count} Time: {lap_time:.2f} s ({cfg.active_pathfinder} @ {cfg.race_pace_pct:.0f}%) <<<\n"
                        print(lap_msg, flush=True)
                        if log_file:
                            log_file.write(lap_msg)
                            log_file.flush()
                        lap_start_time = now
                        
                        max_bench_laps = int(os.environ.get("MAX_BENCHMARK_LAPS", "0"))
                        if max_bench_laps > 0 and lap_count >= max_bench_laps:
                            print(f"[BENCHMARK] Reached target {max_bench_laps} laps for {cfg.active_pathfinder}. Exiting controller.", flush=True)
                            if is_supervisor:
                                robot.simulationQuit(0)
                            sys.exit(0)

                    last_idx = closest_idx

                    # Anticipatory deceleration (Curvature-Coupled Friction Envelope with Flying Start Support)
                    curr_v_allowable = traj_v_base[closest_idx]
                    decel_lookahead_dist = max(2.80, (slew_limiter.v_current**2) / (2.0 * 2.10))
                    accum_dist = 0.0
                    scan_idx = closest_idx
                    while accum_dist < decel_lookahead_dist:
                        next_scan = (scan_idx + 1) % n_pts
                        seg_d = math.hypot(traj_x[next_scan] - traj_x[scan_idx], traj_y[next_scan] - traj_y[scan_idx])
                        accum_dist += max(seg_d, 0.01)
                        scan_idx = next_scan
                        k_scan = abs(traj_curv[scan_idx])
                        v_apex_cap = math.sqrt(max(0.42 * 0.70 * 9.81 / max(k_scan, 0.18), 0.50))
                        v_target_scan = min(traj_v_base[scan_idx], v_apex_cap)
                        v_brake_req = math.sqrt(max(0.25, v_target_scan**2 + 2.0 * 2.10 * accum_dist))
                        if v_brake_req < curr_v_allowable:
                            curr_v_allowable = v_brake_req

                    suggested_speed = curr_v_allowable * (cfg.race_pace_pct / 100.0)

                    # Lateral Alignment factor
                    yaw_error = (current_yaw - track_flow_yaw + math.pi) % (2.0 * math.pi) - math.pi
                    alignment_factor = math.cos(yaw_error)
                    if alignment_factor < 0.85:
                        suggested_speed *= max(0.40, alignment_factor)

                    # Dynamic Speed-Adaptive Pure Pursuit Lookahead
                    lookahead_dist = np.clip(0.18 + 0.10 * slew_limiter.v_current, 0.18, 0.70)
                    target_idx = closest_idx
                    accum_dist = 0.0
                    while accum_dist < lookahead_dist:
                        next_idx = (target_idx + 1) % n_pts
                        seg_len = math.hypot(traj_x[next_idx] - traj_x[target_idx], traj_y[next_idx] - traj_y[target_idx])
                        accum_dist += max(seg_len, 0.01)
                        target_idx = next_idx

                    tx, ty = traj_x[target_idx], traj_y[target_idx]
                    dx = tx - rx
                    dy = ty - ry
                    local_fwd = dx * math.cos(current_yaw) + dy * math.sin(current_yaw)
                    local_lat_left = -dx * math.sin(current_yaw) + dy * math.cos(current_yaw)

                    advance_count = 0
                    while local_fwd < 0.12 and advance_count < 25:
                        target_idx = (target_idx + 1) % n_pts
                        advance_count += 1
                        tx, ty = traj_x[target_idx], traj_y[target_idx]
                        dx = tx - rx
                        dy = ty - ry
                        local_fwd = dx * math.cos(current_yaw) + dy * math.sin(current_yaw)
                        local_lat_left = -dx * math.sin(current_yaw) + dy * math.cos(current_yaw)

                    dist_to_target = max(math.hypot(local_fwd, local_lat_left), 0.12)
                    pure_pursuit_steer = math.atan2(2.0 * WHEELBASE * local_lat_left, dist_to_target**2)

                # 4B. Check Wrong-Way Orientation (Prevent Driving Reverse Race Direction)
                is_wrong_way, heading_diff = wrong_way_detector.evaluate(current_yaw, track_flow_yaw, slew_limiter.v_current)
                if is_wrong_way:
                    suggested_steer = math.copysign(MAX_STEERING_RAD, heading_diff)

                # 4C. Check Non-Blocking Stuck & Reverse Recovery Manager
                is_recovering, rec_v, rec_steer = recovery_manager.update(
                    current_v=slew_limiter.v_current,
                    commanded_v=suggested_speed,
                    tof_distances=tof_distances,
                    dt=dt_s
                )

                if is_recovering:
                    raw_cmd_v = rec_v
                    raw_cmd_steer = rec_steer
                else:
                    # 4D. Dynamically Optimal Actuator Lead-In (Hardware-Synchronized Transit + Lag)
                    t_lead_optimal = max(0.010, min(0.080, cfg.lead_time_s))
                    lead_dist = max(0.05, abs(slew_limiter.v_current) * t_lead_optimal)
                    lead_idx = closest_idx
                    lead_accum = 0.0
                    while lead_accum < lead_dist:
                        next_l = (lead_idx + 1) % n_pts
                        lead_accum += max(math.hypot(traj_x[next_l] - traj_x[lead_idx], traj_y[next_l] - traj_y[lead_idx]), 0.01)
                        lead_idx = next_l
                    kappa_lead = traj_curv[lead_idx]
                    delta_ff_base = math.atan(WHEELBASE * kappa_lead)

                    target_d = 0.0
                    raw_cmd_v = min(suggested_speed, cfg.top_speed_m_s)

                    if cfg.active_pathfinder == "MPPI":
                        # OPTION 6: Model Predictive Path Integral (MPPI 60Hz Vectorized Engine)
                        # 1. Read 16-channel ToF sensor rig (Lidar nodes or DistanceSensors)
                        tof_16ch_readings = {}
                        for s_name, s_dev in tof_16ch_devs.items():
                            if hasattr(s_dev, "getRangeImage"):
                                r_img = s_dev.getRangeImage()
                                valid_r = [v for v in r_img if not math.isinf(v) and not math.isnan(v) and v <= 1.80] if r_img else []
                                val = min(valid_r) if valid_r else 1.80
                            elif hasattr(s_dev, "getValue"):
                                val = s_dev.getValue()
                                if val > 10.0:
                                    val /= 1000.0
                            else:
                                val = 1.80
                            tof_16ch_readings[s_name] = max(0.02, min(1.80, float(val)))

                        # 2. Build 60 Hz Local Costmap
                        if costmap_pipeline is not None:
                            costmap_pipeline.update_pose(now, rx, ry, current_yaw, slew_limiter.v_current)
                            prob_map, hazard_mask, costmap_ms = costmap_pipeline.process_tof_frame(tof_16ch_readings, now)
                        else:
                            prob_map = np.zeros((101, 101), dtype=np.float32)
                            hazard_mask = np.zeros((101, 101), dtype=bool)
                            costmap_ms = 0.0

                        # 3. Vectorized MPPI Optimization Cycle
                        mppi_steer, mppi_speed, mppi_diag = pathfinder_mppi.compute_control(
                            rx=rx,
                            ry=ry,
                            ryaw=current_yaw,
                            v_mps=slew_limiter.v_current,
                            active_steer_rad=slew_limiter.steer_current,
                            prob_map=prob_map,
                            hazard_mask=hazard_mask,
                            frenet_helper=frenet_helper,
                            curr_idx=closest_idx,
                            v_target_ref=suggested_speed,
                            frenet_left=frenet_left,
                            frenet_right=frenet_right,
                        )
                        raw_cmd_steer = mppi_steer
                        raw_cmd_v = min(mppi_speed, cfg.top_speed_m_s)

                        # 4. Rich Debugging Telemetry (logged to console & file every 10 steps = 100 ms)
                        if step_count % 10 == 0:
                            w_c = mppi_diag['seed_weights']['center']
                            w_l = mppi_diag['seed_weights']['left']
                            w_r = mppi_diag['seed_weights']['right']
                            telemetry_line = (
                                f"[MPPI Telemetry] T={now:5.2f}s | Speed={slew_limiter.v_current:4.2f}->{raw_cmd_v:4.2f}m/s | "
                                f"Steer={math.degrees(raw_cmd_steer):+5.1f}deg | Calc={mppi_diag['exec_time_ms']:4.2f}ms (Map={costmap_ms:4.2f}ms) | "
                                f"Cost={mppi_diag['best_cost']:6.1f} | Seeds(C/L/R)={w_c:.2f}/{w_l:.2f}/{w_r:.2f} | "
                                f"Pruned={mppi_diag['collision_rate']*100:4.1f}% | CTE={current_e_lat:+5.3f}m"
                            )
                            print(telemetry_line, flush=True)
                            if log_file:
                                log_file.write(telemetry_line + "\n")

                    elif cfg.active_pathfinder == "MPCC":
                        # OPTION 5: Model Predictive Contouring Control (Frenet LTV-MPC via OSQP)
                        cmd_steer, cmd_v, _ = pathfinder_mpcc.plan(
                            rx, ry, current_yaw, slew_limiter.v_current, slew_limiter.steer_current,
                            tof_distances, frenet_helper, closest_idx, target_v=suggested_speed
                        )
                        raw_cmd_steer = cmd_steer
                        raw_cmd_v = min(cmd_v, cfg.top_speed_m_s)

                    elif cfg.active_pathfinder == "FRENETQP":
                        # OPTION 1: 1D-QP Frenet Micro-Corridor (Servo-Constrained)
                        target_d, feedforward_k, _ = pathfinder_qp.plan(
                            rx, ry, current_yaw, tof_distances, frenet_helper, closest_idx,
                            current_steer_rad=slew_limiter.steer_current,
                            v_mps=slew_limiter.v_current
                        )
                        # Dynamic Understeer Gradient Feedforward
                        a_y_lead = (slew_limiter.v_current**2) * kappa_lead
                        delta_ff = math.atan(WHEELBASE * kappa_lead) + getattr(cfg, "k_ug", 0.040) * (a_y_lead / 9.81)
                        
                        # Heading and Front-Axle Projected Cross-Track Error
                        heading_err = (track_flow_yaw - current_yaw + math.pi) % (2.0 * math.pi) - math.pi
                        e_lat_total = current_e_lat - target_d + getattr(cfg, "front_proj_m", 0.0) * math.sin(heading_err)
                        
                        v_soft = 0.55 + 0.22 * max(abs(slew_limiter.v_current), 0.50)
                        stanley_steer = math.atan(getattr(cfg, "k_cte", 2.20) * (-e_lat_total) / v_soft)
                        k_heading = 0.95
                        raw_cmd_steer = delta_ff + k_heading * heading_err + stanley_steer

                    elif cfg.active_pathfinder == "ELASTICBAND":
                        # OPTION 2: Gaussian Elastic Band (GEB Potential Deformation)
                        target_d, _ = pathfinder_geb.plan(
                            rx, ry, current_yaw, tof_distances, frenet_helper, closest_idx,
                            current_steer_rad=slew_limiter.steer_current
                        )
                        e_lat_total = current_e_lat - target_d
                        v_soft = 0.70 + 0.20 * max(abs(slew_limiter.v_current), 0.50)
                        stanley_steer = math.atan(2.20 * (-e_lat_total) / v_soft)
                        raw_cmd_steer = delta_ff_base + stanley_steer

                    elif cfg.active_pathfinder == "LATTICE":
                        # OPTION 3: Frenet Spatiotemporal Lattice Planner
                        target_d, feedforward_k, _ = pathfinder_lattice.plan(
                            rx, ry, current_yaw, tof_distances, frenet_helper, closest_idx,
                            current_steer_rad=slew_limiter.steer_current,
                            v_mps=slew_limiter.v_current
                        )
                        delta_ff = math.atan(WHEELBASE * kappa_lead)
                        e_lat_total = current_e_lat - target_d
                        v_soft = 0.70 + 0.20 * max(abs(slew_limiter.v_current), 0.50)
                        stanley_steer = math.atan(2.20 * (-e_lat_total) / v_soft)
                        raw_cmd_steer = delta_ff + stanley_steer

                    else: # "DISPARITYEXTENDER"
                        # OPTION 4: Biased Disparity Extender with Line Target
                        raw_stanley_cte = math.atan2(1.20 * (-current_e_lat), max(abs(slew_limiter.v_current), 0.80))
                        nominal_line_steer = delta_ff_base + pure_pursuit_steer + np.clip(raw_stanley_cte, -math.radians(10.0), math.radians(10.0))
                        bde_steer, speed_factor = pathfinder_bde.compute(
                            tof_distances, target_line_steer=nominal_line_steer
                        )
                        raw_cmd_steer = bde_steer
                        raw_cmd_v *= speed_factor

                    # 4E. Dynamic Predictive Safety Shield (Forward Swept-Volume Rollout & TTC Governor)
                    has_risk, min_ttc, min_tof_c, evasion_d = predictive_shield.evaluate_rollout(
                        rx, ry, current_yaw, slew_limiter.v_current, raw_cmd_steer,
                        tof_distances, frenet_helper, closest_idx, kappa=kappa_lead
                    )
                    
                    if has_risk:
                        # 1. Proactive Lateral Evasion Shift
                        if abs(evasion_d) > 0.04:
                            raw_cmd_steer += math.atan2(0.85 * evasion_d, max(abs(slew_limiter.v_current), 0.80))
                        
                        # 2. Smooth Proportional TTC Deceleration
                        if min_ttc < 0.75:
                            ttc_speed_cap = max(1.20, slew_limiter.v_current * max(0.30, min_ttc / 0.75))
                            raw_cmd_v = min(raw_cmd_v, ttc_speed_cap)
                    else:
                        # Forward cone backup check
                        min_front_zone = np.min(tof_distances[4:8])
                        if min_front_zone < 0.35:
                            d_avail = max(min_front_zone - 0.08, 0.02)
                            v_tof_safe = math.sqrt(2.0 * max(cfg.brake_limit_m_s2, 12.0) * d_avail)
                            raw_cmd_v = min(raw_cmd_v, max(v_tof_safe, 1.20))

                    # 4F. Active Gyro Yaw Damping for High-Speed Stability
                    expected_yaw = (slew_limiter.v_current * math.tan(raw_cmd_steer)) / WHEELBASE
                    raw_cmd_steer += -0.03 * (gyro_z - expected_yaw)

                    # 4G. Dynamic Steering Authority Limit
                    raw_cmd_steer = max(-MAX_STEERING_RAD, min(raw_cmd_steer, MAX_STEERING_RAD))

                    # 4H. Automatic Crash Recovery & Backing-Up Controller
                    h_err_flow = (track_flow_yaw - current_yaw + math.pi) % (2.0 * math.pi) - math.pi
                    is_recovering, rec_v, rec_steer = recovery_manager.update(
                        dt_s, slew_limiter.v_current, raw_cmd_v, tof_distances, heading_err=h_err_flow
                    )
                    if is_recovering:
                        raw_cmd_v = rec_v
                        raw_cmd_steer = rec_steer

            # 5. Apply Slew Rate & Acceleration Filtering
            filtered_v, filtered_steer = slew_limiter.filter(raw_cmd_v, raw_cmd_steer, cfg, dt_s)

            # 6. Compute 4WD Electronic Differential & True Ackermann Kinematics
            act = ediff.update(filtered_v, filtered_steer, gyro_z)

            # 7. Command Steering Servos (True Independent Ackermann Angles)
            if "left" in steering_servos:
                steering_servos["left"].setPosition(act["steer_left"])
            if "right" in steering_servos:
                steering_servos["right"].setPosition(act["steer_right"])

            # 8. Command Drive Motors (Independent Wheel Angular Velocities)
            if "RL" in drive_motors:
                drive_motors["RL"].setVelocity(act["omega_RL"])
            if "RR" in drive_motors:
                drive_motors["RR"].setVelocity(act["omega_RR"])
            if "FL" in drive_motors:
                drive_motors["FL"].setVelocity(act["omega_FL"])
            if "FR" in drive_motors:
                drive_motors["FR"].setVelocity(act["omega_FR"])

            # 9. Real-Time Telemetry HUD (Output every 15 steps = ~240ms)
            if step_count % 15 == 0:
                slip_str = f"[{act['slip_state']}]" if act['slip_state'] != SLIP_NORMAL else "[NORMAL]"
                ww_str = " [WRONG-WAY!]" if is_wrong_way else ""
                rec_str = f" [{recovery_manager.state}]" if recovery_manager.state != StuckRecoveryState.NORMAL_DRIVE else ""
                
                tof_fl_str = ",".join(f"{d*100:3.0f}" for d in tof_16ch_distances[0:4])
                tof_fr_str = ",".join(f"{d*100:3.0f}" for d in tof_16ch_distances[4:8])
                tof_bl_str = ",".join(f"{d*100:3.0f}" for d in tof_16ch_distances[8:12])
                tof_br_str = ",".join(f"{d*100:3.0f}" for d in tof_16ch_distances[12:16])
                tof_matrix_str = f"ToF[16]: [FL:{tof_fl_str} | FR:{tof_fr_str} | BL:{tof_bl_str} | BR:{tof_br_str}]cm"

                rz = pos[2] if (is_supervisor and robot_node is not None) else 0.028
                hud_line = f"[{cfg.active_pathfinder}] idx={closest_idx:3d} pos=({rx:+.2f},{ry:+.2f},{rz:.3f}) v={filtered_v:.2f}m/s ({filtered_v*3.6:4.1f}km/h) | Steer={math.degrees(filtered_steer):+5.1f}° | {tof_matrix_str} | Dev={current_e_lat*100:+5.1f}cm | {slip_str}{rec_str}{ww_str}"
                print(hud_line, flush=True)
                if log_file:
                    log_file.write(hud_line + "\n")
                    log_file.flush()

            # 10. Physical Crash & Barrier Impact Detector (True Physical Collision)
            min_tof_all = np.min(tof_distances)
            a_mag = math.hypot(accel_data[0], accel_data[1]) if accel_data else 0.0
            crash_reason = None

            # Only check for new collisions when not already actively executing recovery
            if recovery_manager.state == StuckRecoveryState.NORMAL_DRIVE:
                if (a_mag > 80.0 or (a_mag > 40.0 and min_tof_all < 0.10)) and filtered_v > 0.60:
                    crash_reason = f"High-G Impact ({a_mag:.1f} m/s^2)"
                elif min_tof_all < 0.035 and filtered_v > 0.40:
                    crash_reason = f"Bumper Wall Strike ({min_tof_all*100:.1f} cm)"
                elif abs(current_e_lat) > 0.38 and step_count > 30:
                    crash_reason = f"Track Barrier Excursion (Dev={current_e_lat*100:+.1f} cm)"
                elif abs(gyro_z) > 9.0 and filtered_v > 0.80:
                    crash_reason = f"Violent Spinout (GyroZ={gyro_z:.1f} rad/s)"

            if crash_reason:
                crash_count += 1
                crash_msg = f"\n*** [CRASH DETECTED #{crash_count}] {crash_reason} at idx={closest_idx}, pos=({rx:+.2f},{ry:+.2f}), v={filtered_v:.2f}m/s ***\n"
                print(crash_msg, flush=True)
                if log_file:
                    log_file.write(crash_msg)
                    log_file.flush()
                
                # Trigger emergency backing up and recovery
                h_err_c = (track_flow_yaw - current_yaw + math.pi) % (2.0 * math.pi) - math.pi
                if abs(h_err_c) > math.radians(60.0):
                    steer_sign = +1.0 if h_err_c > 0 else -1.0
                else:
                    min_l = np.min(tof_distances[0:5])
                    min_r = np.min(tof_distances[7:12])
                    steer_sign = +1.0 if min_l < min_r else -1.0
                recovery_manager.trigger_emergency_recovery(heading_err=h_err_c, steer_dir=steer_sign)

                if os.environ.get("QUIT_ON_CRASH", "0") == "1":
                    print(f"[CRASH ABORT] Halting simulation due to collision: {crash_reason}", flush=True)
                    if is_supervisor:
                        robot.simulationQuit(1)
                    sys.exit(1)

            # Update dead reckoning
            odom_x += filtered_v * math.cos(odom_yaw) * dt_s
            odom_y += filtered_v * math.sin(odom_yaw) * dt_s
            odom_yaw = (odom_yaw + act["omega_cmd"] * dt_s) % (2.0 * math.pi)

        exit_msg = f"[CONTROLLER TERMINATION] robot.step() returned -1 at step={step_count}, sim_time={now:.2f}s, last_idx={last_idx}\n"
        print(exit_msg, flush=True)
        if log_file:
            log_file.write(exit_msg)
            log_file.flush()

    except Exception as e:
        import traceback
        err_msg = f"\n[FATAL CONTROLLER ERROR] {type(e).__name__}: {e}\n"
        print(err_msg, flush=True)
        traceback.print_exc()
        if log_file:
            log_file.write(err_msg)
            traceback.print_exc(file=log_file)
            log_file.flush()
    finally:
        if log_file:
            log_file.close()


if __name__ == "__main__":
    main()
