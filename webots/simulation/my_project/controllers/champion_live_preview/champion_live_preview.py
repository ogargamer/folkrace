"""
================================================================================
  LIVE VISUAL PREVIEW CONTROLLER FOR GA CHAMPION PLANNER
================================================================================
  Runs the champion path planner in real-time Webots with full 3D rendering,
  live sensor fusion, smart hybrid costmap, and moving obstacles.
  Supports switching between all fastest configurations:
    1. OPTION_2_SL_MPC (142.1 m, 7.10 m/s)
    2. OPTION_3_SEPTIC Gen 21 C++ (107.0 m, 5.35 m/s)
    3. OPTION_1_BVP (90.2 m, 4.51 m/s)
    4. OPTION_3_SEPTIC Baseline (92.7 m, 4.63 m/s)
================================================================================
"""

import os
import sys
import math
import json
import time
import socket
import zlib
import base64
from typing import Dict, List, Optional, Tuple, Any
import numpy as np

# Discover controller paths
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
GA_OPTIMIZER_DIR = os.path.dirname(CONTROLLERS_DIR)
PLANNERS_DIR = os.path.join(GA_OPTIMIZER_DIR, "planners")
DATA_DIR = os.path.join(GA_OPTIMIZER_DIR, "data")
WORKSPACE_ROOT = os.path.dirname(GA_OPTIMIZER_DIR)

for p in [GA_OPTIMIZER_DIR, PLANNERS_DIR, DATA_DIR, WORKSPACE_ROOT, CURR_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

try:
    from controller import Robot, Supervisor
    WEBOTS_AVAILABLE = True
except ImportError:
    WEBOTS_AVAILABLE = False

from electronic_diff_4wd import ElectronicDifferential4WD, V_MAX_WHEEL
from cpp_smart_hybrid_costmap import CppSmartHybridCostmap
from imu_encoder_fusion import IMUEncoderFusion
from tof_gap_scout import ToFGapScout
from stuck_recovery_manager import StuckRecoveryManager
from planner_config import (
    TIME_STEP_MS,
    DT,
    SIM_HZ,
    PLANNER_HZ,
    DT_PLANNER,
    PLANNER_DECIMATION,
    TOF_TIME_STEP_MS,
    TOF_SENSOR_NAMES,
    FRONT_TOF_SENSOR_NAMES,
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
    GRID_X_MIN,
    GRID_X_MAX,
    GRID_Y_MIN,
    GRID_Y_MAX,
    GRID_RES,
    R_INFLATION_M,
    compute_steering_aggressiveness,
)

# Planners (C++ Curvilinear S-L MPC)
from planner_option2_sl_mpc import CurvilinearSLMPC

from ga_planner_config import ROBOT_START_POS, ROBOT_START_ROT

# Track racing line origin start pose (matches Point 0 of cyan_champion_trajectory.csv)
START_POS = list(ROBOT_START_POS)
START_ROT = list(ROBOT_START_ROT)

MOVING_OBSTACLE_NAMES = ["MOVING_OBSTACLE", "MOVING_OBSTACLE_2", "MOVING_OBSTACLE_3"]
MOVING_OBSTACLE_SPEED_MPS = 0.50


class ReverseObstacleEngine:
    """Interpolates obstacle positions along the track in reverse."""

    def __init__(self, csv_path: str, speed_mps: float = 0.50):
        self.speed_mps = speed_mps
        self.traj = np.zeros((0, 2))
        self.dists = np.zeros(0)
        self.total_length = 14.109

        if os.path.exists(csv_path):
            try:
                arr = np.loadtxt(csv_path, delimiter=",", skiprows=1)
                self.traj = arr[:, 0:2]
                if arr.shape[1] >= 6:
                    self.dists = arr[:, 5]
                    self.total_length = float(self.dists[-1])
                else:
                    diffs = np.diff(self.traj, axis=0)
                    seg_dists = np.hypot(diffs[:, 0], diffs[:, 1])
                    self.dists = np.concatenate([[0.0], np.cumsum(seg_dists)])
                    self.total_length = float(self.dists[-1])
            except Exception as e:
                print(f"[ReverseObstacleEngine] Failed to parse trajectory CSV: {e}", flush=True)

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


def read_tof_device(dev: any) -> float:
    val = TOF_MAX_RANGE_M
    if hasattr(dev, "getRangeImage"):
        r_img = dev.getRangeImage()
        if r_img:
            # Filter valid ranges; if values are below 0.025m (optical saturation against barrier),
            # treat them as immediate contact (0.010m) instead of open space.
            valid_r = [v for v in r_img if not math.isinf(v) and not math.isnan(v)]
            if valid_r:
                min_v = min(valid_r)
                if min_v < 0.025:
                    val = 0.010  # Immediate bumper contact
                elif min_v <= TOF_MAX_RANGE_M:
                    val = float(min_v)
    elif hasattr(dev, "getValue"):
        raw = float(dev.getValue())
        if raw > 10.0:
            raw /= 1000.0
        if 0.0 < raw < 0.025:
            val = 0.010
        elif raw >= 0.025:
            val = raw
    return float(np.clip(val, TOF_MIN_RANGE_M, TOF_MAX_RANGE_M))


class WebotsTrackProgressTracker:
    """
    Computes true monotonic forward progress along the track racing line using Webots
    ground truth supervisor position (x, y).

    Prevents cheating from:
      - Driving back and forth (points only advance when pushing new frontiers)
      - Wiggling side to side (orthogonal projection keeps arc-length invariant)
      - Wall bouncing / shuddering
      - Reverse driving
    """

    def __init__(self, traj_csv_path: str):
        arr = np.loadtxt(traj_csv_path, delimiter=",", skiprows=1)
        self.traj_pts = arr[:, 0:2]
        self.dists = arr[:, 5]
        self.total_length = float(self.dists[-1])
        self.n_pts = len(self.dists)

        self.last_idx = 0
        self.prev_s = 0.0
        self.current_lap = 0
        self.current_track_dist = 0.0
        self.max_track_progress = 0.0
        self.prev_pos = np.array([0.0, 0.0])
        self.off_track = False

    def _project_point(self, p: np.ndarray, i0: int, i1: int):
        A = self.traj_pts[i0]
        B = self.traj_pts[i1]
        v = B - A
        v_sq = float(np.dot(v, v))
        if v_sq < 1e-9:
            return 0.0, float(np.linalg.norm(p - A)), 0.0
        t = float(np.clip(np.dot(p - A, v) / v_sq, 0.0, 1.0))
        proj = A + t * v
        d_perp = float(np.linalg.norm(p - proj))
        seg_len = math.sqrt(v_sq)
        return t, d_perp, seg_len

    def reset(self, start_pos: List[float]):
        cx, cy = float(start_pos[0]), float(start_pos[1])
        dists_sq = (self.traj_pts[:, 0] - cx) ** 2 + (self.traj_pts[:, 1] - cy) ** 2
        self.last_idx = int(np.argmin(dists_sq))

        p = np.array([cx, cy])
        i_next = (self.last_idx + 1) % self.n_pts
        t, _, seg_len = self._project_point(p, self.last_idx, i_next)
        self.prev_s = float(self.dists[self.last_idx] + t * seg_len)
        self.current_lap = 0
        self.current_track_dist = 0.0
        self.max_track_progress = 0.0
        self.prev_pos = np.array([cx, cy], dtype=float)
        self.off_track = False

    def update(self, current_pos: List[float]) -> float:
        cx, cy = float(current_pos[0]), float(current_pos[1])
        p = np.array([cx, cy])

        # Local search window: -15 to +35 waypoints around last tracked index
        search_window = [(self.last_idx + offset) % self.n_pts for offset in range(-15, 35)]
        cand_pts = self.traj_pts[search_window]
        dists_sq = (cand_pts[:, 0] - cx) ** 2 + (cand_pts[:, 1] - cy) ** 2
        best_local_idx = int(np.argmin(dists_sq))
        closest_idx = search_window[best_local_idx]

        # Test segments before and after closest waypoint
        i_prev = (closest_idx - 1) % self.n_pts
        i_next = (closest_idx + 1) % self.n_pts

        t_bwd, d_bwd, seg_bwd = self._project_point(p, i_prev, closest_idx)
        t_fwd, d_fwd, seg_fwd = self._project_point(p, closest_idx, i_next)

        if d_fwd <= d_bwd:
            seg_start = closest_idx
            t_chosen = t_fwd
            seg_len = seg_fwd
            min_d_perp = d_fwd
        else:
            seg_start = i_prev
            t_chosen = t_bwd
            seg_len = seg_bwd
            min_d_perp = d_bwd

        # Off track check (track width is 1.0m, walls at +/- 0.50m)
        if min_d_perp > 0.65:
            self.off_track = True
            return self.max_track_progress

        s_curr = float(self.dists[seg_start] + t_chosen * seg_len)
        ds = s_curr - self.prev_s

        if ds < -0.5 * self.total_length:
            ds += self.total_length
        elif ds > 0.5 * self.total_length:
            ds -= self.total_length

        # Physical ceiling guard: 5.50 m/s * 0.020s = 0.110m; clamp step jump at 0.25m
        step_euclid = float(np.hypot(cx - self.prev_pos[0], cy - self.prev_pos[1]))
        if ds > 0.25:
            ds = min(step_euclid, 0.25)

        self.current_track_dist += ds
        self.current_track_dist = max(0.0, self.current_track_dist)

        # Monotonic track progression: only advances when new forward ground is broken
        if self.current_track_dist > self.max_track_progress:
            self.max_track_progress = self.current_track_dist

        self.last_idx = closest_idx
        self.prev_s = s_curr
        self.prev_pos = np.array([cx, cy])
        self.current_lap = int(self.max_track_progress // self.total_length)
        return self.max_track_progress

    @property
    def total_progress_m(self) -> float:
        return self.max_track_progress


def load_track_walls() -> Tuple[np.ndarray, np.ndarray]:
    """Loads exact left and right wall point coordinates from Cyan_Champion_Live_Preview.wbt."""
    world_path = os.path.join(GA_OPTIMIZER_DIR, "worlds", "Cyan_Champion_Live_Preview.wbt")
    if not os.path.exists(world_path):
        world_path = os.path.join(GA_OPTIMIZER_DIR, "worlds", "Cyan_Champion_GA.wbt")
    try:
        with open(world_path, "r", encoding="utf-8", errors="ignore") as f:
            text = f.read()
        import re
        left_pos = text.find('track_left_wall')
        right_pos = text.find('track_right_wall')

        def extract_pts(start_idx: int) -> np.ndarray:
            if start_idx == -1:
                return np.zeros((0, 2))
            m = re.search(r'Coordinate\s*\{\s*point\s*\[\s*([-\d\.\s,]+)\]', text[start_idx:])
            if m:
                return np.array([float(x) for x in m.group(1).replace(',', ' ').split() if x]).reshape(-1, 3)[:, :2]
            return np.zeros((0, 2))

        left_pts = extract_pts(left_pos)
        right_pts = extract_pts(right_pos)
        return left_pts, right_pts
    except Exception as e:
        print(f"[Champion Preview] Warning: Could not load track walls: {e}", flush=True)
    return np.zeros((0, 2)), np.zeros((0, 2))


def load_active_config() -> Tuple[str, Dict[str, Any], Dict[str, Any]]:
    """
    Loads active config from active_preview_config.json first,
    or falls back to ga_checkpoint.json or best_planner_genome.json.
    Returns: (planner_type, genes, info_dict)
    """
    # 1. Primary: check active_preview_config.json
    active_path = os.path.join(GA_OPTIMIZER_DIR, "active_preview_config.json")
    if os.path.exists(active_path):
        try:
            with open(active_path, "r", encoding="utf-8") as f:
                d = json.load(f)
                p_type = d.get("planner", "OPTION_2_SL_MPC")
                genes = d.get("genes") or d.get("best_genome") or {}
                info = {
                    "source": active_path,
                    "title": d.get("title", f"Active Config ({p_type})"),
                    "fitness": d.get("fitness", d.get("best_fitness", "N/A")),
                    "metrics": d.get("metrics", d.get("best_metrics", {})),
                }
                print(f"[Champion Preview] Loaded Active Config: {info['title']} (Planner: {p_type}, Fitness: {info['fitness']})", flush=True)
                return p_type, genes, info
        except Exception as e:
            print(f"[Champion Preview] Failed to parse active_preview_config.json: {e}", flush=True)

    # 2. Check ga_checkpoint.json
    ckpt_path = os.path.join(GA_OPTIMIZER_DIR, "ga_checkpoint.json")
    if os.path.exists(ckpt_path):
        try:
            with open(ckpt_path, "r", encoding="utf-8") as f:
                d = json.load(f)
                p_type = d.get("planner", "OPTION_2_SL_MPC")
                genes = d.get("best_genome") or d.get("genes") or {}
                info = {
                    "source": ckpt_path,
                    "title": f"GA Checkpoint Gen {d.get('generation', '?')} ({p_type})",
                    "fitness": d.get("best_fitness", "N/A"),
                    "metrics": d.get("best_metrics", {}),
                }
                print(f"[Champion Preview] Loaded from GA Checkpoint {ckpt_path} (Planner: {p_type}, Fitness: {info['fitness']})", flush=True)
                return p_type, genes, info
        except Exception as e:
            print(f"[Champion Preview] Failed to parse ga_checkpoint.json: {e}", flush=True)

    # 3. Check best_planner_genome.json
    fb_path = os.path.join(GA_OPTIMIZER_DIR, "best_planner_genome.json")
    if os.path.exists(fb_path):
        try:
            with open(fb_path, "r", encoding="utf-8") as f:
                d = json.load(f)
                p_type = d.get("planner", "OPTION_2_SL_MPC")
                genes = d.get("genes") or d.get("best_genome") or {}
                info = {
                    "source": fb_path,
                    "title": f"Best Planner Genome ({p_type})",
                    "fitness": d.get("fitness", "N/A"),
                    "metrics": d.get("metrics", {}),
                }
                print(f"[Champion Preview] Loaded Best Planner Genome from {fb_path} (Planner: {p_type}, Fitness: {info['fitness']})", flush=True)
                return p_type, genes, info
        except Exception as e:
            print(f"[Champion Preview] Failed to parse best_planner_genome.json: {e}", flush=True)

    return "OPTION_2_SL_MPC", {}, {"title": "Default SL-MPC", "fitness": "N/A", "metrics": {}}


def create_planner(planner_type: str, genes: Dict[str, Any], costmap: CppSmartHybridCostmap):
    """Instantiates CurvilinearSLMPC with the given genes dictionary."""
    mu_val = float(genes.get("mu_friction", 0.817))
    print(f"[Champion Preview] Initializing OPTION_2_SL_MPC Planner (Horizon={genes.get('horizon_N', 30)})...", flush=True)
    planner = CurvilinearSLMPC(
        costmap_engine=costmap,
        horizon_N=int(genes.get("horizon_N", 30)),
        dt=float(genes.get("dt", 0.016667)),
        q_progress=float(genes.get("q_progress", 8.00)),
        q_cte=float(genes.get("q_cte", 18.00)),
        q_heading=float(genes.get("q_heading", 8.00)),
        r_steer=float(genes.get("r_steer", 5.00)),
        r_accel=float(genes.get("r_accel", 2.00)),
        r_steer_rate=float(genes.get("r_steer_rate", 10.00)),
        mu_friction=mu_val,
        v_corner_factor=float(genes.get("v_corner_factor", 0.95)),
        pure_pursuit_blend_gain=float(genes.get("pp_blend_gain", 0.35)),
        lookahead_blend_m=float(genes.get("lookahead_blend_m", 0.45)),
        v_cruise_max=float(genes.get("v_cruise_max", 5.50)),
        v_cruise_min=float(genes.get("v_cruise_min", 0.60)),
        straight_lock_dist_m=float(genes.get("straight_lock_dist", 1.70)),
        straight_lock_speed_mps=float(genes.get("straight_lock_speed", 5.50)),
        alt_switch_hysteresis=float(genes.get("alt_switch_hysteresis", 15.00)),
        alt_min_separation_m=float(genes.get("alt_min_separation_m", 0.25)),
        alt_aperture_bias=float(genes.get("alt_aperture_bias", 1.80)),
    )
    planner.costmap = costmap
    return planner


def main():
    if not WEBOTS_AVAILABLE:
        print("[Champion Preview] Webots Python API not found.", flush=True)
        return

    robot = Supervisor()
    robot_node = robot.getSelf()

    print("=" * 80, flush=True)
    print("  STARTING REAL-TIME CHAMPION PREVIEW RUN", flush=True)
    print("=" * 80, flush=True)

    # Actuators
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

    servos = {
        "left": robot.getDevice("servo left"),
        "right": robot.getDevice("servo right"),
    }
    for s in servos.values():
        if s:
            s.setPosition(0.0)

    # Sensors
    encoders = {}
    for key, name_or_list in ENCODER_NAMES.items():
        dev = None
        names = name_or_list if isinstance(name_or_list, (list, tuple)) else [name_or_list]
        for name in names:
            dev = robot.getDevice(name)
            if dev:
                break
        if dev:
            dev.enable(TIME_STEP_MS)
            encoders[key] = dev
        else:
            print(f"[Warning] PositionSensor '{key}' not found!", flush=True)

    imu_dev = robot.getDevice("bno08x_imu") or robot.getDevice("inertial unit") or robot.getDevice("imu")
    if imu_dev:
        imu_dev.enable(TIME_STEP_MS)

    gyro = robot.getDevice("gyro") or robot.getDevice("bno08x_gyro")
    if gyro:
        gyro.enable(TIME_STEP_MS)

    accel = robot.getDevice("accelerometer") or robot.getDevice("bno08x_accel")
    if accel:
        accel.enable(TIME_STEP_MS)

    tofs = {}
    for name in TOF_SENSOR_NAMES:
        dev = robot.getDevice(name)
        if dev:
            dev.enable(TOF_TIME_STEP_MS)
            tofs[name] = dev

    # Moving Obstacles
    obs_fields = []
    for name in MOVING_OBSTACLE_NAMES:
        node = robot.getFromDef(name)
        if node:
            field = node.getField("translation")
            if field:
                obs_fields.append(field)

    traj_csv = os.path.join(DATA_DIR, "cyan_champion_trajectory.csv")
    if not os.path.exists(traj_csv):
        traj_csv = os.path.join(WORKSPACE_ROOT, "cyan_champion_trajectory.csv")
    obs_engine = ReverseObstacleEngine(traj_csv, speed_mps=MOVING_OBSTACLE_SPEED_MPS)

    # Vehicle Start Pose (Lined up with Point 0 of cyan_champion_trajectory.csv)
    car_trans_field = robot_node.getField("translation") if robot_node else None
    car_rot_field = robot_node.getField("rotation") if robot_node else None

    # Track racing line start pose:
    START_POS = list(ROBOT_START_POS)
    START_ROT = list(ROBOT_START_ROT)

    # Load Active Config
    planner_type, genes, info = load_active_config()
    mu_val = float(genes.get("mu_friction", 0.817))
    r_infl = float(genes.get("r_inflation_m", 0.090))
    target_lookahead_m = float(genes.get("target_lookahead_m", 1.35))
    aggr_dist_far = float(genes.get("aggr_dist_far_m", 1.70))
    aggr_dist_close = float(genes.get("aggr_dist_close_m", 1.00))
    aggr_min_scale = float(genes.get("aggr_min_scale", 0.30))
    steer_deadband = float(genes.get("steer_deadband_rad", 0.002))
    straight_lock_dist = float(genes.get("straight_lock_dist", 1.70))
    straight_lock_speed = float(genes.get("straight_lock_speed", 5.50))

    # Electronic Differential
    ediff = ElectronicDifferential4WD(
        wheelbase=0.1325,
        track_rear=0.1000,
        track_front=0.0822,
        track_kingpin=0.0620,
        wheel_radius=0.0275,
        max_steer=0.6981,
        motor_dir=-1.0,
        v_max=V_MAX_WHEEL,
    )

    fusion = IMUEncoderFusion(wheel_radius=WHEEL_RADIUS, dt=DT_PLANNER)
    fusion.attach_devices(encoders=encoders, imu_dev=imu_dev, gyro_dev=gyro, accel_dev=accel)

    costmap = CppSmartHybridCostmap(
        grid_bounds=(GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX),
        resolution=GRID_RES,
        r_inflation_m=r_infl,
    )
    gap_scout = ToFGapScout(
        w_gap_distance=float(genes.get("w_gap_distance", 3.50)),
        w_gap_width=float(genes.get("w_gap_width", 2.00)),
        w_align_steer=float(genes.get("w_gap_steer", 0.80)),
        w_align_straight=float(genes.get("w_gap_straight", 0.40)),
        w_side_clearance=float(genes.get("w_side_clearance", 2.80)),
        gap_dist_exp_k=float(genes.get("gap_dist_exp_k", 0.60)),
        gap_progress_margin_m=float(genes.get("gap_progress_margin_m", 0.15)),
        alt_min_separation_m=float(genes.get("alt_min_separation_m", 0.25)),
        alt_aperture_bias=float(genes.get("alt_aperture_bias", 1.80)),
    )

    planner = create_planner(planner_type, genes, costmap)

    # Initialize Webots Real Ground-Truth Track Progress Tracker
    track_left_wall, track_right_wall = load_track_walls()
    track_tracker = WebotsTrackProgressTracker(traj_csv)
    track_tracker.reset(START_POS)

    if robot_node:
        try:
            robot_node.enableContactPointsTracking(TIME_STEP_MS, True)
        except Exception:
            pass

    # Initialize Intelligent Multi-Point K-Turn Recovery Manager
    recovery_manager = StuckRecoveryManager(
        reverse_speed_m_s=float(genes.get("recovery_reverse_speed", 1.30)),
        steer_angle_rad=float(genes.get("recovery_steer_angle_rad", 0.4538)),
        emergency_backup_dist_m=float(genes.get("emergency_backup_dist_m", 0.09)),
        clearance_exit_m=float(genes.get("recovery_clearance_exit_m", 0.28)),
    )

    # Initialize UDP streaming socket for Live Preview Map (Port 9876 / 9877)
    gui_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        cmd_sock.bind(("127.0.0.1", 9877))
        cmd_sock.setblocking(False)
    except Exception as e:
        print(f"[Champion Preview] Note: Could not bind cmd_sock to 9877: {e}", flush=True)

    # Enable Keyboard Control for Live User Interaction
    keyboard = robot.getKeyboard()
    if keyboard:
        try:
            keyboard.enable(TIME_STEP_MS)
        except Exception:
            pass

    sim_is_paused = False
    sim_step_one_frame = False
    sim_speed_mult = 1.0  # Default 1.0x (Real-Time Webots Playback)

    step_count = 0
    active_steer = 0.0
    v_est = 0.0
    yaw_rate = 0.0
    cached_yaw_rate = 0.0
    last_print_time = 0.0
    lap_start_time = 0.0
    current_lap = 0
    stall_counter = 0

    cached_target_steer = 0.0
    cached_target_speed = float(genes.get("v_cruise_min", V_MIN_CRUISE))
    rate_limited_speed = 0.0
    was_recovering = False
    cached_theta_g = 0.0
    cached_xg = 0.0
    cached_yg = 1.0
    cached_width = 0.50
    cached_d_straight = TOF_MAX_RANGE_M
    cached_front_min_m = TOF_MAX_RANGE_M
    cached_plan_ms = 0.0
    cached_map_ms = 0.0
    cached_tofs = {k: TOF_MAX_RANGE_M for k in TOF_SENSOR_NAMES}
    if "tof_front_left_4.1deg" in cached_tofs:
        front_tof_keys = [
            "tof_front_left_4.1deg", "tof_front_right_4.1deg",
            "tof_front_left_15.4deg", "tof_front_right_15.4deg",
        ]
    else:
        front_tof_keys = ["tof_fl_3", "tof_fr_3", "tof_fl_2", "tof_fr_2"]
    plan_res = {}

    print(f"[Champion Preview] Controller active! Running Multi-Rate (120 Hz Sim/IMU/4WD, 60 Hz ToF/Planner)", flush=True)

    while robot.step(TIME_STEP_MS) != -1:
        wall_start_t = time.perf_counter()

        # Check for keyboard commands
        if keyboard:
            try:
                k = keyboard.getKey()
                while k > 0:
                    if k in (ord('+'), ord('=')):
                        sim_speed_mult = min(5.0, round(sim_speed_mult + 0.10, 2))
                        print(f"\n>>> [KEYBOARD] SIMULATION SPEED: {sim_speed_mult:.2f}x", flush=True)
                    elif k in (ord('-'), ord('_')):
                        sim_speed_mult = max(0.05, round(sim_speed_mult - 0.10, 2))
                        print(f"\n>>> [KEYBOARD] SIMULATION SPEED: {sim_speed_mult:.2f}x", flush=True)
                    elif k in (ord('P'), ord('p'), ord(' ')):
                        sim_is_paused = not sim_is_paused
                        print(f"\n>>> [KEYBOARD] PAUSED: {sim_is_paused}", flush=True)
                    elif k in (ord('R'), ord('r')):
                        if car_trans_field:
                            car_trans_field.setSFVec3f(START_POS)
                        if car_rot_field:
                            car_rot_field.setSFRotation(START_ROT)
                        track_tracker.reset(START_POS)
                        print("\n>>> [KEYBOARD] ROBOT POSE RESET TO TRACK START", flush=True)
                    k = keyboard.getKey()
            except Exception:
                pass

        # Check for commands from GUI
        try:
            cmd_data, _ = cmd_sock.recvfrom(4096)
            if cmd_data:
                cmd_dict = json.loads(cmd_data.decode("utf-8"))
                cmd_type = cmd_dict.get("cmd", "")
                if cmd_type == "pause":
                    sim_is_paused = True
                elif cmd_type == "play":
                    sim_is_paused = False
                elif cmd_type == "step":
                    sim_is_paused = False
                    sim_step_one_frame = True
                elif cmd_type == "set_speed":
                    sim_speed_mult = float(cmd_dict.get("val", 1.0))
                    print(f"[Champion Preview] Live updated simulation speed: {sim_speed_mult:.2f}x", flush=True)
                elif cmd_type == "set_param":
                    p_name = cmd_dict.get("name")
                    p_val = float(cmd_dict.get("val", 0.0))
                    if hasattr(costmap, p_name):
                        setattr(costmap, p_name, p_val)
                    if hasattr(planner, p_name):
                        setattr(planner, p_name, p_val)
                    print(f"[Champion Preview] Live tuned {p_name} = {p_val}", flush=True)

        except (BlockingIOError, socket.error):
            pass
        except Exception as e:
            print(f"[Champion Preview] Command parse error: {e}", flush=True)

        if sim_is_paused:
            for m in motors.values():
                if m:
                    m.setVelocity(0.0)
            time.sleep(0.02)
            continue

        if sim_step_one_frame:
            sim_is_paused = True
            sim_step_one_frame = False

        step_count += 1
        sim_time = step_count * DT

        # Webots Real Ground-Truth Data
        car_pos = car_trans_field.getSFVec3f() if car_trans_field else [0.0, 0.0, 0.028]
        cx, cy, cz = float(car_pos[0]), float(car_pos[1]), float(car_pos[2])

        vel = robot_node.getVelocity() if robot_node else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        v_real = float(math.hypot(vel[0], vel[1]))

        # Real distance along track centerline
        dist = float(track_tracker.update([cx, cy]))
        calculated_lap = int(track_tracker.current_lap)

        # Update reverse moving obstacles
        if obs_fields and obs_engine.total_length > 0:
            n_obs = len(obs_fields)
            for k, field in enumerate(obs_fields):
                obs_s = (float(k + 1) * (obs_engine.total_length / float(n_obs + 1)) - MOVING_OBSTACLE_SPEED_MPS * sim_time) % obs_engine.total_length
                ox, oy = obs_engine.get_position_at_s(obs_s)
                field.setSFVec3f([float(ox), float(oy), 0.075])

        # Initial physics warmup pause (0.50 s)
        if sim_time < 0.50:
            for m in motors.values():
                if m:
                    m.setVelocity(0.0)
            for s in servos.values():
                if s:
                    s.setPosition(0.0)
            continue

        # 60 Hz Control Tick (Decimation = 2)
        # Fusion, 16-CH ToF, Costmap, Path Planner, Recovery, Straight-Lock & Quieting
        run_planner_step = (step_count % PLANNER_DECIMATION == 0)
        if run_planner_step:
            # Sensor fusion at 60 Hz (Zero Cheating)
            v_est, yaw_rate, yaw = fusion.update(dt=DT_PLANNER)
            v_est = max(0.0, v_est) if not math.isnan(v_est) else 0.0
            cached_yaw_rate = yaw_rate

            for name in TOF_SENSOR_NAMES:
                dev = tofs.get(name)
                if dev:
                    cached_tofs[name] = read_tof_device(dev)

            prob_map, hazard_mask, map_ms = costmap.process_tof_frame(cached_tofs)
            cached_map_ms = map_ms

            xg, yg, width, theta_g = gap_scout.scout_gap(
                tof_readings=cached_tofs,
                hazard_mask=hazard_mask,
                costmap_obj=costmap,
                target_lookahead_m=target_lookahead_m,
                steer_current=active_steer,
                prob_map=prob_map,
            )
            cached_xg, cached_yg, cached_width, cached_theta_g = xg, yg, width, theta_g

            t_plan_0 = time.perf_counter()
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
            cached_plan_ms = (time.perf_counter() - t_plan_0) * 1000.0

            raw_target_steer = float(plan_res.get("cmd_steer_rad", 0.0))
            raw_target_speed = float(plan_res.get("cmd_speed_mps", genes.get("v_cruise_min", V_MIN_CRUISE)))
            cached_theta_g = float(theta_g)
            if "tof_front_left_4.1deg" in cached_tofs:
                d_fl3 = cached_tofs.get("tof_front_left_4.1deg", TOF_MAX_RANGE_M)
                d_fr3 = cached_tofs.get("tof_front_right_4.1deg", TOF_MAX_RANGE_M)
                d_fl_lock = cached_tofs.get("tof_front_left_4.1deg", 0.0)
                d_fr_lock = cached_tofs.get("tof_front_right_4.1deg", 0.0)
            else:
                d_fl3 = cached_tofs.get("tof_fl_3", TOF_MAX_RANGE_M)
                d_fr3 = cached_tofs.get("tof_fr_3", TOF_MAX_RANGE_M)
                d_fl_lock = cached_tofs.get("tof_fl_3", 0.0)
                d_fr_lock = cached_tofs.get("tof_fr_3", 0.0)
            cached_d_straight = min(d_fl3, d_fr3)
            cached_front_min_m = min([cached_tofs.get(k, TOF_MAX_RANGE_M) for k in front_tof_keys])

            # Recovery Manager at 60 Hz
            is_recovering, rec_v, rec_steer = recovery_manager.update(
                dt=DT_PLANNER,
                current_v=v_est,
                commanded_v=raw_target_speed,
                cached_tofs=cached_tofs,
                heading_err=cached_theta_g,
                active_steer=active_steer,
            )

            if is_recovering:
                target_speed = rec_v
                target_steer = rec_steer
                active_steer = rec_steer
                mode_desc = f"RECOVERY_{recovery_manager.state}"
                was_recovering = True
            else:
                if was_recovering:
                    was_recovering = False
                    active_steer = 0.0

                # Fast Straight-Line Lock (60 Hz, guarded by turn intent)
                is_straight_locked = (
                    d_fl_lock >= straight_lock_dist and
                    d_fr_lock >= straight_lock_dist and
                    abs(cached_theta_g) < 0.08 and
                    abs(raw_target_steer) < 0.08
                )
                if is_straight_locked:
                    target_speed = straight_lock_speed
                    target_steer = 0.0
                    active_steer = 0.0
                    mode_desc = "STRAIGHT_LOCK"
                else:
                    target_speed = raw_target_speed
                    target_steer = raw_target_steer
                    steer_delta = target_steer - active_steer

                    # Progressive distance and small-offset steering aggressiveness
                    if cached_d_straight >= aggr_dist_far:
                        dist_scale = aggr_min_scale
                    elif cached_d_straight <= aggr_dist_close:
                        dist_scale = 1.0
                    else:
                        frac = (aggr_dist_far - cached_d_straight) / max(0.001, (aggr_dist_far - aggr_dist_close))
                        dist_scale = float(aggr_min_scale + (frac ** 1.5) * (1.0 - aggr_min_scale))

                    err_mag = abs(steer_delta)
                    if err_mag <= steer_deadband:
                        err_scale = 0.90
                    elif err_mag >= 0.05:
                        err_scale = 1.0
                    else:
                        err_frac = (err_mag - steer_deadband) / max(0.001, (0.05 - steer_deadband))
                        err_scale = float(0.90 + err_frac * (1.0 - 0.90))

                    steer_aggressiveness = float(max(0.85, min(1.0, dist_scale * err_scale)))
                    active_steer = active_steer + steer_aggressiveness * steer_delta
                    mode_desc = "OPTION_2_SL_MPC"

            cached_target_speed = target_speed
            cached_target_steer = active_steer

        # 4WD Electronic Differential & Motor Commands at 120 Hz
        # Unified Friction Rate Limiter for speed command (enforces acceleration & deceleration limits)
        speed_err = cached_target_speed - rate_limited_speed
        if (rate_limited_speed >= 0.0 and speed_err > 0.0) or (rate_limited_speed <= 0.0 and speed_err < 0.0):
            max_dv = (0.85 * mu_val * 9.81) * DT
        else:
            max_dv = (1.00 * mu_val * 9.81) * DT

        if abs(speed_err) <= max_dv:
            rate_limited_speed = cached_target_speed
        else:
            rate_limited_speed += math.copysign(max_dv, speed_err)

        live_gyro_z = float(gyro.getValues()[2]) if gyro else cached_yaw_rate
        actuators = ediff.update(target_v=rate_limited_speed, target_steer_rad=cached_target_steer, gyro_z_rad_s=live_gyro_z)

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

        if step_count % 2 == 0:
            try:
                prob_map_send = costmap.get_probability_map() if hasattr(costmap, "get_probability_map") else prob_map
                hazard_mask_send = costmap.get_hazard_mask() if hasattr(costmap, "get_hazard_mask") else hazard_mask
                if prob_map_send is not None and hazard_mask_send is not None:
                    prob_down = (prob_map_send[::2, ::2] * 255.0).astype(np.uint8)
                    hazard_down = hazard_mask_send[::2, ::2].astype(np.uint8)

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

                    b_line = to_py(plan_res.get("best_line", [])) if "plan_res" in locals() else []
                    c_lines = to_py(plan_res.get("candidate_lines", []))[:20] if "plan_res" in locals() else []
                    g_gap = [round(float(xg), 3), round(float(yg), 3), round(float(width), 3)] if "xg" in locals() else [0, 0, 0]
                    g_targets = [[round(float(tx), 3), round(float(ty), 3), round(float(ap), 3)] for (tx, ty, ap) in (gap_scout.last_targets or [])]

                    gui_pkt = {
                        "time": round(float(sim_time), 3),
                        "mode": mode_desc,
                        "status": plan_res.get("status", "OK") if "plan_res" in locals() else "OK",
                        "v_est": round(float(v_est), 3),
                        "v_cmd": round(float(cached_target_speed), 3),
                        "steer_deg": round(math.degrees(float(cached_target_steer)), 2),
                        "dist_m": round(float(dist), 3),
                        "lap": int(current_lap),
                        "gap": g_gap,
                        "targets": g_targets,
                        "plan_ms": round(float(cached_plan_ms), 2),
                        "map_ms": round(float(cached_map_ms), 2),
                        "surviving": int(plan_res.get("surviving_count", 0)) if "plan_res" in locals() else 0,
                        "candidates": int(plan_res.get("candidate_count", len(c_lines))) if "plan_res" in locals() else 0,
                        "tof": {k: round(float(v), 3) for k, v in cached_tofs.items()},
                        "best_line": b_line,
                        "candidate_lines": c_lines,
                        "grid_shape": [int(s) for s in prob_down.shape],
                        "grid_bounds": [float(GRID_X_MIN), float(GRID_X_MAX), float(GRID_Y_MIN), float(GRID_Y_MAX)],
                        "grid_data": grid_compressed,
                        "hazard_data": hazard_compressed,
                    }
                    gui_sock.sendto(json.dumps(gui_pkt).encode("utf-8"), ("127.0.0.1", 9876))
            except Exception:
                pass

        # Stall logging
        if sim_time > 0.5 and cached_target_speed >= 0.50 and v_real < 0.12:
            stall_counter += 1
            if stall_counter * DT >= 0.5 and sim_time - last_print_time >= 0.5:
                print(f"  [Vehicle Stall] Stopped against barrier (v_real={v_real:.2f}m/s).", flush=True)
                last_print_time = sim_time
        else:
            stall_counter = 0

        # Lap counting
        if calculated_lap > current_lap:
            lap_time = sim_time - lap_start_time
            print(f"  >>> LAP {calculated_lap} FINISHED in {lap_time:.2f}s! (Avg Speed: {track_tracker.total_length/lap_time:.2f} m/s) <<<", flush=True)
            current_lap = calculated_lap
            lap_start_time = sim_time

        # Live telemetry display every other step (25 Hz Webots Real Data)
        if step_count % 2 == 0:
            print(
                f"[Telemetry] Mode: {mode_desc:14s} | Time: {sim_time:5.1f}s | Lap: {current_lap:2d} | "
                f"Dist: {dist:6.1f}m | Speed: {v_real:4.2f}m/s (cmd: {cached_target_speed:4.2f}) | "
                f"Steer: {math.degrees(cached_target_steer):+5.1f} deg | Plan Latency: {cached_plan_ms:4.2f}ms",
                flush=True,
            )

        # Simulation Speed Throttling (0.05x to 100x)
        if sim_speed_mult < 4.9:
            target_step_dt = (TIME_STEP_MS / 1000.0) / max(0.01, sim_speed_mult)
            elapsed_wall = time.perf_counter() - wall_start_t
            if elapsed_wall < target_step_dt:
                time.sleep(target_step_dt - elapsed_wall)


if __name__ == "__main__":
    main()
