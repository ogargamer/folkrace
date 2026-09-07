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

TIME_STEP = 10             # ms (100 Hz control loop, matches Webots basicTimeStep)
CONFIG_FILE_NAME = "folkrace_config.json"


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
        self.race_pace_pct = 85.0           # Default Champion Race Pace (85% safe default)
        
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
        print("  FOLKRACE RACING-LINE FRENET 1D-QP DASHBOARD", flush=True)
        print("=" * 80, flush=True)
        print(f"  Active Pathfinder  : [FRENET 1D-QP CORRIDOR PLANNER]", flush=True)
        print(f"  Dynamic Margin Mode: [{getattr(self, 'shield_margin_mode', 'QUADRATIC')}] (d_min={getattr(self, 'shield_d_min', 0.020)*1000:.0f}mm, d_max={getattr(self, 'shield_d_max', 0.055)*1000:.0f}mm)", flush=True)
        print("  --- DYNAMIC MARGIN HOTKEYS (0-7) ---", flush=True)
        print("    '1'=Quadratic Std (20-55mm) | '2'=Quadratic Tight (10-35mm) | '3'=Quadratic Gen (30-75mm)", flush=True)
        print("    '4'=Linear Std    (20-55mm) | '5'=Curvature G-Force (20-55mm)| '6'=Sigmoid Std   (20-55mm)", flush=True)
        print("    '7'=Asymmetric Split        | '0'=Fixed Baseline Margin (35mm)", flush=True)
        print("  --- OBSTACLE CONTROLS (8-9) ---", flush=True)
        print("    '8'=Cycle Obstacle Speed (0.25 -> 0.50 -> 0.75 -> 1.00 -> 1.50 m/s)", flush=True)
        print("    '9'=Toggle Moving Obstacle Active (On/Off)", flush=True)
        print("  ----------------------------------------------------------------------------", flush=True)
        print(f"  [O / L] Race Pace  : {self.race_pace_pct:5.1f} %  (O = +5%, L = -5%)", flush=True)
        print(f"  [T / G] Top Speed  : {self.top_speed_m_s:5.2f} m/s ({self.top_speed_m_s*3.6:4.1f} km/h)", flush=True)
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


class PillarRouteManager:
    """
    Tiered Trajectory Hierarchy:
    - Tier 1: Master Cyan Champion Racing Line (Primary fast path)
    - Tier 2: Pillar-Specific Verified Alternative Trajectories (Pillar 2, 4, 5)
    - Tier 3: Local Reactive QP Gap Avoidance on Open Straights
    """
    def __init__(self, curr_dir):
        self.routes = {} # name -> dict with fields
        self.active_name = "MASTER"
        self.active_pillar = None
        self.switch_cooldown = 0

        # 1. Master line
        m_path = os.path.join(curr_dir, "cyan_champion_trajectory.csv")
        m_data = self._load_csv(m_path)
        if m_data is not None:
            self.routes["MASTER"] = self._create_route_dict("MASTER", m_data)
            print(f"[Folkrace Autodrive] Loaded MASTER racing line ({len(m_data)} points).", flush=True)

        # 2. Pillar Fallback lines
        loaded_alts = []
        for p in (2, 3, 4, 5, 6):
            p_path = os.path.join(curr_dir, f"alt_route_pillar_{p}.csv")
            p_data = self._load_csv(p_path)
            if p_data is not None:
                self.routes[f"PILLAR_{p}"] = self._create_route_dict(f"PILLAR_{p}", p_data)
                loaded_alts.append(f"Pillar {p}")
        if loaded_alts:
            print(f"[Folkrace Autodrive] Loaded Verified Pillar Alternate Routes: {', '.join(loaded_alts)}", flush=True)

        # Sectors where an alternative line diverges and merges
        self.pillar_sectors = {
            2: {"entry_idx": 210, "exit_idx": 395}, # Pillar 2 (East Chicane)
            4: {"entry_idx": 645, "exit_idx": 795}, # Pillar 4
            5: {"entry_idx": 780, "exit_idx": 965}, # Pillar 5 (West Chicane)
        }

    def _load_csv(self, path):
        if not os.path.exists(path):
            return None
        rows = []
        try:
            with open(path, "r", encoding="utf-8") as f:
                lines = f.readlines()
                for line in lines[1:]:
                    parts = line.strip().split(",")
                    if len(parts) >= 6:
                        rows.append([float(x) for x in parts])
            return np.array(rows) if rows else None
        except Exception:
            return None

    def _create_route_dict(self, name, data):
        tx = data[:, 0]
        ty = data[:, 1]
        tv = data[:, 2]
        tyaw = data[:, 3]
        tk = data[:, 4]
        tdist = data[:, 5]
        frenet = FrenetTransformer(tdist, tx, ty, tyaw, tk)
        return {
            "name": name,
            "data": data,
            "frenet": frenet,
            "x": tx, "y": ty, "v": tv,
            "yaw": tyaw, "curv": tk, "dist": tdist,
            "n_pts": len(tx)
        }

    def update(self, rx, ry, current_yaw, closest_idx, min_front_zone, tof_distances):
        if self.switch_cooldown > 0:
            self.switch_cooldown -= 1

        # 1. Check if past exit point of current active sector
        if self.active_pillar is not None:
            sector = self.pillar_sectors.get(self.active_pillar)
            if sector and (closest_idx >= sector["exit_idx"] or closest_idx < sector["entry_idx"] - 40):
                self.active_name = "MASTER"
                self.active_pillar = None

        # 2. Continuous dynamic evaluation inside any active pillar sector
        for p, sector in self.pillar_sectors.items():
            if sector["entry_idx"] - 35 <= closest_idx <= sector["exit_idx"]:
                route_key = f"PILLAR_{p}"
                if route_key not in self.routes:
                    continue

                # Sensor clearance evaluation for Master vs Alternate
                if p == 2:
                    clear_master = float(np.min(tof_distances[6:10]))
                    clear_alt = float(np.min(tof_distances[1:6]))
                elif p == 5:
                    clear_master = float(np.min(tof_distances[2:7]))
                    clear_alt = float(np.min(tof_distances[6:11]))
                else: # Pillar 4
                    clear_master = float(np.min(tof_distances[5:10]))
                    clear_alt = float(np.min(tof_distances[1:6]))

                diff_clear = clear_alt - clear_master

                if self.switch_cooldown == 0:
                    # Primary Criterion: Superior Clearance
                    if diff_clear > 0.10:
                        if self.active_name != route_key:
                            self.active_name = route_key
                            self.active_pillar = p
                            self.switch_cooldown = 20
                    elif diff_clear < -0.10:
                        if self.active_name != "MASTER":
                            self.active_name = "MASTER"
                            self.active_pillar = None
                            self.switch_cooldown = 20
                    else:
                        # Tie-breaker: vehicle current heading alignment
                        master_route = self.routes.get("MASTER")
                        alt_route = self.routes.get(route_key)
                        if master_route and alt_route:
                            m_idx = min(closest_idx, master_route["n_pts"] - 1)
                            a_idx = min(closest_idx, alt_route["n_pts"] - 1)
                            yaw_m = master_route["yaw"][m_idx]
                            yaw_a = alt_route["yaw"][a_idx]
                            
                            err_m = abs((current_yaw - yaw_m + math.pi) % (2.0 * math.pi) - math.pi)
                            err_a = abs((current_yaw - yaw_a + math.pi) % (2.0 * math.pi) - math.pi)
                            
                            if err_a < err_m - 0.05:
                                if self.active_name != route_key:
                                    self.active_name = route_key
                                    self.active_pillar = p
                                    self.switch_cooldown = 20
                            elif err_m < err_a - 0.05:
                                if self.active_name != "MASTER":
                                    self.active_name = "MASTER"
                                    self.active_pillar = None
                                    self.switch_cooldown = 20
                break

        return self.routes.get(self.active_name, self.routes.get("MASTER"))


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

    # --- 3 MOVING OBSTACLES SUPERVISOR TRACKING (1/3 LAP SPACING) ---
    enable_moving_obstacle = os.environ.get("ENABLE_MOVING_OBSTACLE", "1") == "1"
    moving_obs_speed = float(os.environ.get("MOVING_OBS_SPEED", "0.50")) # m/s
    obs_start_s = float(os.environ.get("MOVING_OBS_START_S", "2.50")) # meters along racing line
    obs_nodes = []
    obs_trans_fields = []
    if is_supervisor:
        for def_name in ("MOVING_OBSTACLE", "MOVING_OBSTACLE_2", "MOVING_OBSTACLE_3"):
            node = robot.getFromDef(def_name)
            if node is not None:
                obs_nodes.append(node)
                obs_trans_fields.append(node.getField("translation"))
        if obs_nodes:
            print(f"[Folkrace Autodrive] {len(obs_nodes)} Moving Obstacles active at {moving_obs_speed:.2f} m/s (spaced 1/3 lap apart).", flush=True)

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

        # --- INITIALIZE 5 RACING-LINE-CENTRIC PATHFINDERS + PREDICTIVE SHIELD ---
        pathfinder_qp = FrenetCorridorQP()
        pathfinder_geb = GaussianElasticBand()
        pathfinder_lattice = FrenetLatticePlanner()
        pathfinder_bde = BiasedDisparityExtender()
        pathfinder_mpcc = FrenetMPCC()
        margin_mode = os.environ.get("SHIELD_MARGIN_MODE", getattr(cfg, "shield_margin_mode", "QUADRATIC"))
        d_min_val = float(os.environ.get("SHIELD_D_MIN", getattr(cfg, "shield_d_min", 0.020)))
        d_max_val = float(os.environ.get("SHIELD_D_MAX", getattr(cfg, "shield_d_max", 0.055)))
        predictive_shield = PredictiveSafetyShield(
            horizon_s=0.90, dt_step=0.045, margin_mode=margin_mode,
            d_min_m=d_min_val, d_max_m=d_max_val, v_max_m_s=cfg.top_speed_m_s
        )
        
        wrong_way_detector = WrongWayDetector()
        recovery_manager = StuckRecoveryManager(reverse_speed_m_s=cfg.reverse_speed_m_s)

        print(f"[Folkrace Autodrive] Racing-Line-Centric Pathfinder Suite Initialized (QP, GEB, Lattice, BDE, MPCC).", flush=True)

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
        imu = robot.getDevice("bno08x_imu")
        if imu:
            imu.enable(10)

        gyro = robot.getDevice("gyro")
        if gyro:
            gyro.enable(10)

        accel = robot.getDevice("accelerometer")
        if accel:
            accel.enable(10)

        tof_sensors = []
        for k in range(NUM_TOF_ZONES):
            dev = robot.getDevice(f"tof_{k}")
            if dev:
                dev.enable(10)
                tof_sensors.append(dev)

        # --- INITIALIZE MULTI-ROUTE PILLAR HIERARCHY MANAGER ---
        route_manager = PillarRouteManager(curr_dir)
        active_route = route_manager.update(0.0, 0.0, 0.0, 0, 1.5, [1.5]*12)
        traj_available = active_route is not None
        if traj_available:
            traj_x = active_route["x"]
            traj_y = active_route["y"]
            traj_v_base = active_route["v"]
            traj_yaw = active_route["yaw"]
            traj_curv = active_route["curv"]
            traj_dist = active_route["dist"]
            n_pts = active_route["n_pts"]
            frenet_helper = active_route["frenet"]
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

        key_cooldown_map = {}
        KEY_DEBOUNCE_S = 0.25  # 250ms debounce cooldown for discrete key presses

        while robot.step(TIME_STEP) != -1:
            dt_s = TIME_STEP / 1000.0
            now = robot.getTime()
            step_count += 1

            # --- UPDATE 3 MOVING OBSTACLES (0.50 m/s spaced 1/3 lap apart) ---
            if enable_moving_obstacle and obs_trans_fields and traj_available:
                sim_time = step_count * dt_s
                n_obs = len(obs_trans_fields)
                c_idx = closest_idx if 'closest_idx' in locals() else 0
                for k, field in enumerate(obs_trans_fields):
                    obs_s_curr = (obs_start_s + (k * frenet_helper.total_length / n_obs) + moving_obs_speed * sim_time) % frenet_helper.total_length
                    obs_xy = frenet_helper.frenet_to_cartesian(obs_s_curr, 0.0, c_idx)
                    field.setSFVec3f([float(obs_xy[0]), float(obs_xy[1]), 0.075])

            # 1. Periodic Config File Hot-Reload (every 1.0s)
            if now - last_cfg_poll > 1.0:
                cfg.load_from_file()
                ediff.v_max = cfg.top_speed_m_s
                ediff.thresh_spin = cfg.spinout_thresh_rad_s
                last_cfg_poll = now

            # 2. Read Sensors (60 Hz)
            gyro_z = 0.0
            if gyro:
                g_vals = gyro.getValues()
                if g_vals:
                    gyro_z = g_vals[2]

            accel_data = None
            if accel:
                accel_data = accel.getValues()

            current_yaw = odom_yaw
            if imu:
                rpy = imu.getRollPitchYaw()
                if rpy:
                    current_yaw = (rpy[2] + math.pi / 2.0 + math.pi) % (2.0 * math.pi) - math.pi

            # Read 12 ToF Distances (meters, range 0.02 to 1.50m)
            tof_distances = np.full(NUM_TOF_ZONES, TOF_MAX_RANGE_M)
            if tof_sensors and len(tof_sensors) == NUM_TOF_ZONES:
                for idx in range(NUM_TOF_ZONES):
                    val = tof_sensors[idx].getValue()
                    tof_distances[idx] = TOF_MAX_RANGE_M if (math.isnan(val) or val <= 0.0) else min(val, TOF_MAX_RANGE_M)

            min_tof_left = np.min(tof_distances[0:4])
            min_tof_center = np.min(tof_distances[4:8])
            min_all_tof = np.min(tof_distances)

            # 3. Handle Keyboard Controls with Debounce Protection
            key = keyboard.getKey()
            while key > 0:
                # Normalize lowercase to uppercase ASCII for letter keys
                k_norm = key - 32 if (ord('a') <= key <= ord('z')) else key

                last_press = key_cooldown_map.get(k_norm, 0.0)
                if (now - last_press) >= KEY_DEBOUNCE_S:
                    key_cooldown_map[k_norm] = now
                    # Dynamic Margins Hotkeys (0-7)
                    if k_norm == ord('1'):
                        cfg.shield_margin_mode = "QUADRATIC"
                        cfg.shield_d_min = 0.020
                        cfg.shield_d_max = 0.055
                        predictive_shield.margin_mode = "QUADRATIC"
                        predictive_shield.d_min = 0.020
                        predictive_shield.d_max = 0.055
                        os.environ["SHIELD_MARGIN_MODE"] = "QUADRATIC"
                        os.environ["SHIELD_D_MIN"] = "0.020"
                        os.environ["SHIELD_D_MAX"] = "0.055"
                        print("\n[DYNAMIC MARGIN] Option 1: Quadratic Standard (20-55 mm)", flush=True)
                    elif k_norm == ord('2'):
                        cfg.shield_margin_mode = "QUADRATIC"
                        cfg.shield_d_min = 0.010
                        cfg.shield_d_max = 0.035
                        predictive_shield.margin_mode = "QUADRATIC"
                        predictive_shield.d_min = 0.010
                        predictive_shield.d_max = 0.035
                        os.environ["SHIELD_MARGIN_MODE"] = "QUADRATIC"
                        os.environ["SHIELD_D_MIN"] = "0.010"
                        os.environ["SHIELD_D_MAX"] = "0.035"
                        print("\n[DYNAMIC MARGIN] Option 2: Quadratic Tight (10-35 mm)", flush=True)
                    elif k_norm == ord('3'):
                        cfg.shield_margin_mode = "QUADRATIC"
                        cfg.shield_d_min = 0.030
                        cfg.shield_d_max = 0.075
                        predictive_shield.margin_mode = "QUADRATIC"
                        predictive_shield.d_min = 0.030
                        predictive_shield.d_max = 0.075
                        os.environ["SHIELD_MARGIN_MODE"] = "QUADRATIC"
                        os.environ["SHIELD_D_MIN"] = "0.030"
                        os.environ["SHIELD_D_MAX"] = "0.075"
                        print("\n[DYNAMIC MARGIN] Option 3: Quadratic Generous (30-75 mm)", flush=True)
                    elif k_norm == ord('4'):
                        cfg.shield_margin_mode = "LINEAR"
                        cfg.shield_d_min = 0.020
                        cfg.shield_d_max = 0.055
                        predictive_shield.margin_mode = "LINEAR"
                        predictive_shield.d_min = 0.020
                        predictive_shield.d_max = 0.055
                        os.environ["SHIELD_MARGIN_MODE"] = "LINEAR"
                        os.environ["SHIELD_D_MIN"] = "0.020"
                        os.environ["SHIELD_D_MAX"] = "0.055"
                        print("\n[DYNAMIC MARGIN] Option 4: Linear Standard (20-55 mm)", flush=True)
                    elif k_norm == ord('5'):
                        cfg.shield_margin_mode = "CURVATURE"
                        cfg.shield_d_min = 0.020
                        cfg.shield_d_max = 0.055
                        predictive_shield.margin_mode = "CURVATURE"
                        predictive_shield.d_min = 0.020
                        predictive_shield.d_max = 0.055
                        os.environ["SHIELD_MARGIN_MODE"] = "CURVATURE"
                        os.environ["SHIELD_D_MIN"] = "0.020"
                        os.environ["SHIELD_D_MAX"] = "0.055"
                        print("\n[DYNAMIC MARGIN] Option 5: Curvature G-Force Standard (20-55 mm)", flush=True)
                    elif k_norm == ord('6'):
                        cfg.shield_margin_mode = "SIGMOID"
                        cfg.shield_d_min = 0.020
                        cfg.shield_d_max = 0.055
                        predictive_shield.margin_mode = "SIGMOID"
                        predictive_shield.d_min = 0.020
                        predictive_shield.d_max = 0.055
                        os.environ["SHIELD_MARGIN_MODE"] = "SIGMOID"
                        os.environ["SHIELD_D_MIN"] = "0.020"
                        os.environ["SHIELD_D_MAX"] = "0.055"
                        print("\n[DYNAMIC MARGIN] Option 6: Sigmoid Standard (20-55 mm)", flush=True)
                    elif k_norm == ord('7'):
                        cfg.shield_margin_mode = "ASYMMETRIC"
                        cfg.shield_d_min = 0.020
                        cfg.shield_d_max = 0.055
                        predictive_shield.margin_mode = "ASYMMETRIC"
                        predictive_shield.d_min = 0.020
                        predictive_shield.d_max = 0.055
                        os.environ["SHIELD_MARGIN_MODE"] = "ASYMMETRIC"
                        os.environ["SHIELD_D_MIN"] = "0.020"
                        os.environ["SHIELD_D_MAX"] = "0.055"
                        print("\n[DYNAMIC MARGIN] Option 7: Asymmetric Split (20-55 mm Lat / 40-110 mm Lon)", flush=True)
                    elif k_norm == ord('0'):
                        cfg.shield_margin_mode = "FIXED"
                        cfg.shield_d_min = 0.035
                        cfg.shield_d_max = 0.035
                        predictive_shield.margin_mode = "FIXED"
                        predictive_shield.d_min = 0.035
                        predictive_shield.d_max = 0.035
                        os.environ["SHIELD_MARGIN_MODE"] = "FIXED"
                        os.environ["SHIELD_D_MIN"] = "0.035"
                        os.environ["SHIELD_D_MAX"] = "0.035"
                        print("\n[DYNAMIC MARGIN] Option 0: Fixed Baseline Margin (35 mm)", flush=True)
                    # Obstacle Controls (8-9)
                    elif k_norm == ord('8'):
                        speeds = [0.25, 0.50, 0.75, 1.00, 1.50]
                        curr_speed = moving_obs_speed
                        next_idx = (speeds.index(curr_speed) + 1) % len(speeds) if curr_speed in speeds else 0
                        moving_obs_speed = speeds[next_idx]
                        print(f"\n[MOVING OBSTACLE] Speed set to {moving_obs_speed:.2f} m/s ({moving_obs_speed*3.6:.1f} km/h)", flush=True)
                    elif k_norm == ord('9'):
                        enable_moving_obstacle = not enable_moving_obstacle
                        status_str = "ENABLED" if enable_moving_obstacle else "DISABLED"
                        print(f"\n[MOVING OBSTACLE] {status_str}", flush=True)
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
                    # Dynamically check & update Active Route from Pillar Hierarchy
                    active_route = route_manager.update(rx, ry, current_yaw, last_idx, min_all_tof, tof_distances)
                    if active_route is not None and active_route["frenet"] != frenet_helper:
                        traj_x = active_route["x"]
                        traj_y = active_route["y"]
                        traj_v_base = active_route["v"]
                        traj_yaw = active_route["yaw"]
                        traj_curv = active_route["curv"]
                        traj_dist = active_route["dist"]
                        n_pts = active_route["n_pts"]
                        frenet_helper = active_route["frenet"]

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
                    if last_idx > 800 and closest_idx < 200:
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
                    min_front_zone = float(np.min(tof_distances[4:8]))

                    if cfg.active_pathfinder == "MPCC":
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
                        
                        # Continuous Linear Stay-On-Line Weight:
                        # Scales smoothly with obstacle proximity on the racing line:
                        # - Clear track (obs >= 0.85m): k_line_weight = 1.0 (100% adherence to optimal line)
                        # - Approaching obstacle (0.20m <= obs < 0.85m): linear decay from 1.0 down to 0.0
                        # - Close / Alongside obstacle (obs < 0.20m or active gap swerve |target_d| > 0.06m): k_line_weight = 0.0
                        min_obstacle_prox = min(
                            float(np.min(tof_distances[0:4])),
                            float(np.min(tof_distances[8:12])),
                            min_front_zone
                        )
                        if abs(target_d) > 0.06 or min_obstacle_prox < 0.20:
                            k_line_weight = 0.0
                        elif min_obstacle_prox >= 0.85:
                            k_line_weight = 1.0
                        else:
                            k_line_weight = (min_obstacle_prox - 0.20) / (0.85 - 0.20)

                        v_soft = 0.55 + 0.22 * max(abs(slew_limiter.v_current), 0.50)
                        stanley_steer = math.atan(k_line_weight * getattr(cfg, "k_cte", 2.20) * (-e_lat_total) / v_soft)
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
                    
                    min_front_zone = float(np.min(tof_distances[4:8]))
                    if has_risk:
                        # 1. Proactive Lateral Evasion Shift
                        if abs(evasion_d) > 0.03:
                            raw_cmd_steer += math.atan2(0.85 * evasion_d, max(abs(slew_limiter.v_current), 0.70))
                        
                        # 2. Smooth Continuous TTC Deceleration (down to 0.50 m/s without stopping)
                        if min_ttc < 0.85:
                            v_ttc_cap = max(0.50, slew_limiter.v_current * (min_ttc / 0.85))
                            raw_cmd_v = min(raw_cmd_v, v_ttc_cap)

                    # Kinematically Guaranteed Frontal Obstacle Braking:
                    # Lookahead distance dynamically scales with v^2 / 2a + reaction margin (up to 1.1m)
                    v_curr_abs = abs(slew_limiter.v_current)
                    d_brake_req = max(0.70, (v_curr_abs**2) / (2.0 * 10.0) + 0.20)
                    
                    if min_front_zone < d_brake_req:
                        d_margin = max(0.01, min_front_zone - 0.10)
                        v_safe_front = max(0.50, math.sqrt(2.0 * 12.0 * d_margin))
                        raw_cmd_v = min(raw_cmd_v, v_safe_front)

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

            # 9. Real-Time Telemetry HUD (High-Density Output every 3 steps = 30ms / 33 Hz)
            if step_count % 3 == 0:
                slip_str = f"[{act['slip_state']}]" if act['slip_state'] != SLIP_NORMAL else "[NORMAL]"
                ww_str = " [WRONG-WAY!]" if is_wrong_way else ""
                rec_str = f" [{recovery_manager.state}]" if recovery_manager.state != StuckRecoveryState.NORMAL_DRIVE else ""
                
                tof_l_str = ",".join(f"{d*100:2.0f}" for d in tof_distances[0:4])
                tof_c_str = ",".join(f"{d*100:2.0f}" for d in tof_distances[4:8])
                tof_r_str = ",".join(f"{d*100:2.0f}" for d in tof_distances[8:12])

                h_err_deg = math.degrees(yaw_error) if 'yaw_error' in locals() else 0.0
                tgt_d_cm = target_d * 100.0 if 'target_d' in locals() else 0.0
                front_cm = min_front_zone * 100.0 if 'min_front_zone' in locals() else 150.0

                hud_line = f"[{route_manager.active_name}] #{closest_idx:3d} ({rx:+.2f},{ry:+.2f}) v={filtered_v:.2f}m/s Str={math.degrees(filtered_steer):+5.1f}° Dev={current_e_lat*100:+5.1f}cm (Tgt={tgt_d_cm:+4.1f}) Fr={front_cm:3.0f}cm | [{tof_l_str}|{tof_c_str}|{tof_r_str}] | {slip_str}{rec_str}{ww_str}"
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
