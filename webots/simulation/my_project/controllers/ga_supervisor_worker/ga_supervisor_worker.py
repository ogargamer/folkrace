"""
================================================================================
  HEADLESS GENETIC ALGORITHM SUPERVISOR CONTROLLER FOR AUTONOMOUS TOF RACING
================================================================================
  - Persistent Webots Supervisor Worker with instant trial reset (<2ms).
  - Reverse Moving Obstacle Engine (moves at 0.50 m/s against robot heading
    along cyan_champion_trajectory.csv).
  - 100% Autonomous ToF + IMU Path Planning (Option 1 BVP, Option 2 SL-MPC, Option 3 Septic).
  - Strictly evaluates distance traveled in 20.0 simulation seconds.
================================================================================
"""

import os
import sys
import math
import json
import time
import socket
import select
from typing import Dict, List, Optional, Tuple, Any
import numpy as np

# Discover controller paths
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
MY_PROJECT_DIR = os.path.dirname(CONTROLLERS_DIR)
WEBOTS_DIR = os.path.dirname(os.path.dirname(MY_PROJECT_DIR))
WORKSPACE_ROOT = os.path.dirname(WEBOTS_DIR)

PLANNER_DIR = os.path.join(CONTROLLERS_DIR, "tof_path_planner")
GA_OPTIMIZER_DIR = os.path.join(WEBOTS_DIR, "optimization", "ga_planner_optimizer")
DATA_DIR = os.path.join(GA_OPTIMIZER_DIR, "data")

for p in [PLANNER_DIR, CONTROLLERS_DIR, MY_PROJECT_DIR, GA_OPTIMIZER_DIR, DATA_DIR, WORKSPACE_ROOT, CURR_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

# Redirect controller stdout/stderr to disk for headless validation
try:
    log_dir = os.path.join(GA_OPTIMIZER_DIR, "logs")
    os.makedirs(log_dir, exist_ok=True)
    worker_id = os.environ.get("GA_WORKER_ID", "0")
    _ctrl_log_path = os.path.join(log_dir, f"controller_worker_{worker_id}.log")
    _log_f = open(_ctrl_log_path, "w", encoding="utf-8", buffering=1)
    class _TeeOut:
        def __init__(self, f, orig):
            self.f = f
            self.orig = orig
        def write(self, s):
            try:
                self.f.write(s)
            except Exception:
                pass
            if self.orig:
                try:
                    self.orig.write(s)
                except Exception:
                    pass
        def flush(self):
            try:
                self.f.flush()
            except Exception:
                pass
            if self.orig:
                try:
                    self.orig.flush()
                except Exception:
                    pass
    sys.stdout = _TeeOut(_log_f, sys.stdout)
    sys.stderr = _TeeOut(_log_f, sys.stderr)
except Exception:
    pass

try:
    from controller import Robot, Supervisor
    WEBOTS_AVAILABLE = True
except ImportError:
    WEBOTS_AVAILABLE = False

from ga_planner_config import (
    EVALUATION_DURATION_S,
    MOVING_OBSTACLE_SPEED_MPS,
    MOVING_OBSTACLE_NAMES,
    STALL_SPEED_THRESHOLD_MPS,
    STALL_MAX_DURATION_S,
    COLLISION_DISTANCE_MIN_M,
    compute_scalar_fitness,
    compute_efficiency_multiplier,
    get_genome_class,
    ROBOT_START_POS,
    ROBOT_START_ROT,
)

from electronic_diff_4wd import ElectronicDifferential4WD
from cpp_smart_hybrid_costmap import CppSmartHybridCostmap
from imu_encoder_fusion import IMUEncoderFusion
from tof_gap_scout import ToFGapScout
from stuck_recovery_manager import StuckRecoveryManager
from planner_option2_sl_mpc import CurvilinearSLMPC
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
    V_MAX_WHEEL,
    GRID_X_MIN,
    GRID_X_MAX,
    GRID_Y_MIN,
    GRID_Y_MAX,
    GRID_RES,
    R_INFLATION_M,
    compute_steering_aggressiveness,
)


# Track racing line origin start pose (matches Point 0 of cyan_champion_trajectory.csv)
START_POS = list(ROBOT_START_POS)
START_ROT = list(ROBOT_START_ROT)

ENCODER_NAMES = {
    "FL": ["encoder FL", "encoder_fl", "encoder_FL", "encoderFL"],
    "FR": ["encoder FR", "encoder_fr", "encoder_FR", "encoderFR"],
    "RL": ["encoder RL", "encoder_rl", "encoder_RL", "encoderRL"],
    "RR": ["encoder RR", "encoder_rr", "encoder_RR", "encoderRR"],
}


class ReverseObstacleEngine:
    """
    Interpolates obstacle positions along the track racing line in reverse
    (traveling against the robot heading at 0.50 m/s).
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


def load_track_walls() -> Tuple[np.ndarray, np.ndarray]:
    """Loads exact left and right wall point coordinates from track_ga_eval.wbt."""
    world_path = os.path.join(MY_PROJECT_DIR, "worlds", "track_ga_eval.wbt")
    if not os.path.exists(world_path):
        world_path = os.path.join(MY_PROJECT_DIR, "worlds", "track_racing.wbt")
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
        print(f"[GA Worker] Warning: Could not load track walls: {e}", flush=True)
    return np.zeros((0, 2)), np.zeros((0, 2))


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


STATIC_OBSTACLES = [
    # Static obstacles on track (Cyan Champion track has moving obstacles only)
]


def read_tof_device(dev: any) -> float:
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
    return float(np.clip(val, TOF_MIN_RANGE_M, TOF_MAX_RANGE_M))


def run_supervisor_worker():
    if not WEBOTS_AVAILABLE:
        print("[GA Supervisor] Webots API not detected.", flush=True)
        return

    robot = Supervisor()
    robot_node = robot.getSelf()

    worker_port = int(os.environ.get("GA_WORKER_PORT", 10200))
    print(f"[GA Worker] Initializing Supervisor Worker on Port {worker_port}...", flush=True)

    # 1. Hardware Actuators
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

    # 2. Encoders & IMU Sensors
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

    # 3. Moving Obstacle Solid Fields
    obs_fields = []
    for name in MOVING_OBSTACLE_NAMES:
        node = robot.getFromDef(name)
        if node:
            field = node.getField("translation")
            if field:
                obs_fields.append(field)

    # 4. Obstacle Trajectory Loader (for reverse motion at 0.5 m/s)
    traj_csv = os.path.join(DATA_DIR, "cyan_champion_trajectory.csv")
    if not os.path.exists(traj_csv):
        traj_csv = os.path.join(WORKSPACE_ROOT, "cyan_champion_trajectory.csv")
    obs_engine = ReverseObstacleEngine(traj_csv, speed_mps=MOVING_OBSTACLE_SPEED_MPS)
    track_left_wall, track_right_wall = load_track_walls()

    # 5. Vehicle Start Pose Fields (Lined up with Point 0 of cyan_champion_trajectory.csv)
    car_trans_field = robot_node.getField("translation") if robot_node else None
    car_rot_field = robot_node.getField("rotation") if robot_node else None

    # Track racing line start pose:
    START_POS = list(ROBOT_START_POS)
    START_ROT = list(ROBOT_START_ROT)

    # 6. Electronic Differential
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

    # 7. Start TCP Server for High-Throughput GA Requests
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(("127.0.0.1", worker_port))
    server_sock.listen(5)
    server_sock.setblocking(False)

    print(f"[GA Worker {worker_port}] Ready for evaluation requests.", flush=True)

    def reset_simulation_environment():
        """Resets car and physics state instantly."""
        # Stop motors
        for m in motors.values():
            if m:
                m.setVelocity(0.0)
        for s in servos.values():
            if s:
                s.setPosition(0.0)

        # Reset car pose
        if car_trans_field:
            car_trans_field.setSFVec3f(START_POS)
        if car_rot_field:
            car_rot_field.setSFRotation(START_ROT)

        # Reset moving obstacles to initial starting positions (t = 0)
        if obs_fields and obs_engine.total_length > 0:
            n_obs = len(obs_fields)
            for k, field in enumerate(obs_fields):
                obs_s0 = (float(k + 1) * (obs_engine.total_length / float(n_obs + 1))) % obs_engine.total_length
                ox0, oy0 = obs_engine.get_position_at_s(obs_s0)
                field.setSFVec3f([float(ox0), float(oy0), 0.075])

        if robot_node:
            robot_node.resetPhysics()
        robot.simulationResetPhysics()

        # Step 2 frames to flush initial physics
        for _ in range(2):
            robot.step(TIME_STEP_MS)

    def evaluate_candidate(planner_name: str, gene_dict: Dict[str, Any], duration_s: float) -> Dict[str, Any]:
        """Executes a 20s autonomous run evaluating the candidate parameters."""
        reset_simulation_environment()

        # Initialize Fusion & Costmap (Matching tof_path_planner.py)
        r_infl = float(gene_dict.get("r_inflation_m", R_INFLATION_M))
        target_lookahead_m = float(gene_dict.get("target_lookahead_m", 1.35))

        fusion = IMUEncoderFusion(wheel_radius=WHEEL_RADIUS, dt=DT_PLANNER)
        fusion.attach_devices(encoders=encoders, imu_dev=imu_dev, gyro_dev=gyro, accel_dev=accel)

        costmap = CppSmartHybridCostmap(
            grid_bounds=(GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX),
            resolution=GRID_RES,
            r_inflation_m=r_infl,
        )

        gap_scout = ToFGapScout(
            w_gap_distance=float(gene_dict.get("w_gap_distance", 3.50)),
            w_gap_width=float(gene_dict.get("w_gap_width", 2.00)),
            w_align_steer=float(gene_dict.get("w_gap_steer", 1.20)),
            w_align_straight=float(gene_dict.get("w_gap_straight", 0.50)),
            w_side_clearance=float(gene_dict.get("w_side_clearance", 2.50)),
            gap_dist_exp_k=float(gene_dict.get("gap_dist_exp_k", 0.50)),
            gap_progress_margin_m=float(gene_dict.get("gap_progress_margin_m", 0.10)),
            alt_min_separation_m=float(gene_dict.get("alt_min_separation_m", 0.25)),
            alt_aperture_bias=float(gene_dict.get("alt_aperture_bias", 1.80)),
        )
        recovery_manager = StuckRecoveryManager(
            reverse_speed_m_s=float(gene_dict.get("recovery_reverse_speed", 1.30)),
            steer_angle_rad=float(gene_dict.get("recovery_steer_angle_rad", 0.60)),
            emergency_backup_dist_m=float(gene_dict.get("emergency_backup_dist_m", 0.09)),
            clearance_exit_m=float(gene_dict.get("recovery_clearance_exit_m", 0.28)),
        )

        # Instantiate Planner
        mu_val = float(gene_dict.get("mu_friction", 0.85))
        aggr_dist_far = float(gene_dict.get("aggr_dist_far_m", 1.70))
        aggr_dist_close = float(gene_dict.get("aggr_dist_close_m", 1.35))
        aggr_min_scale = float(gene_dict.get("aggr_min_scale", 0.90))
        steer_deadband = float(gene_dict.get("steer_deadband_rad", 0.002))
        straight_lock_dist = float(gene_dict.get("straight_lock_dist", 1.70))
        straight_lock_speed = float(gene_dict.get("straight_lock_speed", 5.50))

        planner = CurvilinearSLMPC(
            costmap_engine=costmap,
            horizon_N=int(gene_dict.get("horizon_N", 30)),
            dt=float(gene_dict.get("dt", DT_PLANNER)),
            q_progress=float(gene_dict.get("q_progress", 8.00)),
            q_cte=float(gene_dict.get("q_cte", 18.00)),
            q_heading=float(gene_dict.get("q_heading", 8.00)),
            r_steer=float(gene_dict.get("r_steer", 5.00)),
            r_accel=float(gene_dict.get("r_accel", 2.00)),
            r_steer_rate=float(gene_dict.get("r_steer_rate", 10.00)),
            mu_friction=mu_val,
            v_corner_factor=float(gene_dict.get("v_corner_factor", 0.95)),
            pure_pursuit_blend_gain=float(gene_dict.get("pp_blend_gain", 0.35)),
            lookahead_blend_m=float(gene_dict.get("lookahead_blend_m", 0.45)),
            v_cruise_max=float(gene_dict.get("v_cruise_max", 5.50)),
            v_cruise_min=float(gene_dict.get("v_cruise_min", 0.60)),
            straight_lock_dist_m=straight_lock_dist,
            straight_lock_speed_mps=straight_lock_speed,
            alt_switch_hysteresis=float(gene_dict.get("alt_switch_hysteresis", 15.00)),
            alt_min_separation_m=float(gene_dict.get("alt_min_separation_m", 0.25)),
            alt_aperture_bias=float(gene_dict.get("alt_aperture_bias", 1.80)),
        )

        planner.costmap = costmap

        # Initialize Webots Real Ground-Truth Track Progress Tracker
        track_tracker = WebotsTrackProgressTracker(traj_csv)
        track_tracker.reset(START_POS)

        # Enable Webots Native Contact Points Tracking
        if robot_node:
            try:
                robot_node.enableContactPointsTracking(TIME_STEP_MS, True)
            except Exception:
                pass

        # Trial execution variables
        max_steps = int(duration_s / DT)
        active_steer = 0.0
        v_est = 0.0
        yaw_rate = 0.0
        cached_yaw_rate = 0.0
        cached_target_steer = 0.0
        cached_target_speed = float(gene_dict.get("v_cruise_min", V_MIN_CRUISE))
        rate_limited_speed = 0.0
        cached_theta_g = 0.0
        cached_d_straight = TOF_MAX_RANGE_M
        cached_front_min_m = TOF_MAX_RANGE_M
        cached_tofs = {k: TOF_MAX_RANGE_M for k in TOF_SENSOR_NAMES}
        if "tof_front_left_4.1deg" in cached_tofs:
            front_tof_keys = [
                "tof_front_left_4.1deg", "tof_front_right_4.1deg",
                "tof_front_left_15.4deg", "tof_front_right_15.4deg",
            ]
        else:
            front_tof_keys = ["tof_fl_3", "tof_fr_3", "tof_fl_2", "tof_fr_2"]

        step_count = 0
        crashed = False
        stalled = False
        stall_counter_steps = 0
        high_g_counter_steps = 0
        true_speed_history = []
        crash_reason = ""

        try:
            # Main Evaluation Physics Step Loop (120 Hz simulation)
            while step_count < max_steps:
                if robot.step(TIME_STEP_MS) == -1:
                    break
                step_count += 1
                sim_time = step_count * DT

                # --- A. UPDATE REVERSE MOVING OBSTACLES (0.50 m/s AGAINST robot heading) ---
                if obs_fields and obs_engine.total_length > 0:
                    n_obs = len(obs_fields)
                    for k, field in enumerate(obs_fields):
                        obs_s = (float(k + 1) * (obs_engine.total_length / float(n_obs + 1)) - MOVING_OBSTACLE_SPEED_MPS * sim_time) % obs_engine.total_length
                        ox, oy = obs_engine.get_position_at_s(obs_s)
                        field.setSFVec3f([float(ox), float(oy), 0.075])

                # --- C. WEBOTS REAL GROUND-TRUTH METRICS (No Sensors) ---
                car_pos = robot_node.getPosition() if robot_node else (car_trans_field.getSFVec3f() if car_trans_field else [0.0, 0.0, 0.028])
                cx, cy, cz = float(car_pos[0]), float(car_pos[1]), float(car_pos[2])

                vel = robot_node.getVelocity() if robot_node else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                v_real = float(math.hypot(vel[0], vel[1]))
                true_speed_history.append(v_real)

                # True distance progress along track centerline
                true_dist = track_tracker.update([cx, cy])

                if step_count % 50 == 0:
                    print(
                        f"[GA Worker Step] t={sim_time:5.2f}s | pos=({cx:+5.2f}, {cy:+5.2f}) | "
                        f"dist={true_dist:5.2f}m | v_real={v_real:4.2f}m/s (est: {v_est:4.2f}) | "
                        f"steer={math.degrees(active_steer):+5.1f} deg | theta_g={math.degrees(cached_theta_g):+5.1f} deg | "
                        f"d_str={cached_d_straight:4.2f}m",
                        flush=True,
                    )

                # Initial physics warmup pause (0.50 s)
                if sim_time < 0.50:
                    for m in motors.values():
                        if m:
                            m.setVelocity(0.0)
                    for s in servos.values():
                        if s:
                            s.setPosition(0.0)
                    continue

                # --- D. 60 Hz CONTROL TICK (Decimation = 2) ---
                # Fusion, 16-CH ToF, Costmap, Path Planner, Recovery, Straight-Lock & Quieting
                run_planner_step = (step_count % PLANNER_DECIMATION == 0)
                if run_planner_step:
                    # Update sensor fusion at 60 Hz (Zero Cheating)
                    v_est, yaw_rate, yaw = fusion.update(dt=DT_PLANNER)
                    v_est = max(0.0, v_est) if not math.isnan(v_est) else 0.0
                    cached_yaw_rate = yaw_rate

                    # Read 16 ToF sensors at 60 Hz
                    for name in TOF_SENSOR_NAMES:
                        dev = tofs.get(name)
                        if dev:
                            cached_tofs[name] = read_tof_device(dev)

                    prob_map, hazard_mask, _ = costmap.process_tof_frame(cached_tofs)

                    xg, yg, width, theta_g = gap_scout.scout_gap(
                        tof_readings=cached_tofs,
                        hazard_mask=hazard_mask,
                        costmap_obj=costmap,
                        target_lookahead_m=target_lookahead_m,
                        steer_current=active_steer,
                        prob_map=prob_map,
                    )

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

                    raw_target_steer = float(plan_res.get("cmd_steer_rad", 0.0))
                    raw_target_speed = float(plan_res.get("cmd_speed_mps", gene_dict.get("v_cruise_min", V_MIN_CRUISE)))
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

                    # Recovery Manager (60 Hz)
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
                    else:
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

                    cached_target_speed = target_speed
                    cached_target_steer = active_steer

                # --- E. 4WD ELECTRONIC DIFFERENTIAL & ACTUATORS AT 120 Hz ---
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

                # --- G. COMPREHENSIVE INSTANT CRASH DETECTION (Fast GA Abort) ---
                # 0. Webots Direct Physical Contact Joints (detects any chassis/body wall contact)
                if robot_node and sim_time > 0.05:
                    try:
                        contacts = robot_node.getContactPoints(True)
                        if contacts:
                            for cp in contacts:
                                pt = cp.getPoint()
                                if pt[2] > 0.038:
                                    print(f"[GA Worker] Crash: Webots physical contact joint at t={sim_time:.2f}s (pt=({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f}m), car=({cx:.3f}, {cy:.3f}))!", flush=True)
                                    crashed = True
                                    crash_reason = f"physical_contact_z_{pt[2]:.3f}"
                                    break
                    except Exception:
                        pass
                if crashed:
                    break

                # 1. Exact Left and Right Track Wall Collision (< 0.075 m from vehicle center)
                if len(track_left_wall) > 0 and len(track_right_wall) > 0:
                    min_dl_sq = np.min((track_left_wall[:, 0] - cx)**2 + (track_left_wall[:, 1] - cy)**2)
                    min_dr_sq = np.min((track_right_wall[:, 0] - cx)**2 + (track_right_wall[:, 1] - cy)**2)
                    min_wall_dist = math.sqrt(min(min_dl_sq, min_dr_sq))
                    if min_wall_dist < 0.075:
                        print(f"[GA Worker] Crash: Wall proximity collision at t={sim_time:.2f}s | wall_dist={min_wall_dist:.3f}m < 0.075m", flush=True)
                        crashed = True
                        crash_reason = f"wall_proximity_{min_wall_dist:.3f}m"
                        break

                # 2. Static Obstacle Collision (Boxes and Pillars)
                for ox, oy, orad in STATIC_OBSTACLES:
                    if math.hypot(cx - ox, cy - oy) < orad:
                        print(f"[GA Worker] Crash: Static obstacle collision at t={sim_time:.2f}s!", flush=True)
                        crashed = True
                        crash_reason = "static_obstacle"
                        break
                if crashed:
                    break

                # 3. Moving Obstacle Collision (< 0.15 m from vehicle center)
                if obs_fields:
                    for field in obs_fields:
                        opos = field.getSFVec3f()
                        if math.hypot(cx - float(opos[0]), cy - float(opos[1])) < 0.15:
                            print(f"[GA Worker] Crash: Moving obstacle collision at t={sim_time:.2f}s!", flush=True)
                            crashed = True
                            crash_reason = "moving_obstacle"
                            break
                    if crashed:
                        break

                # 4. Multi-Sensor Front ToF Physical Contact Collision (< 0.015 m)
                front_tofs = [cached_tofs.get(k, TOF_MAX_RANGE_M) for k in front_tof_keys]
                if front_tofs and min(front_tofs) <= 0.015:
                    min_tof_val = min(front_tofs)
                    print(f"[GA Worker] Crash: Front ToF contact proximity at t={sim_time:.2f}s (min_tof={min_tof_val:.3f}m <= 0.015m)", flush=True)
                    crashed = True
                    crash_reason = f"front_tof_proximity_{min_tof_val:.3f}m"
                    break

                # 5. Vehicle Flip / Wall Ride / Roll-Pitch Tumble
                rpy = imu_dev.getRollPitchYaw() if imu_dev else [0.0, 0.0, 0.0]
                roll, pitch = rpy[0], rpy[1]
                if cz > 0.055 or cz < 0.015 or abs(roll) > 0.35 or abs(pitch) > 0.35:
                    print(f"[GA Worker] Crash: Vehicle tumble/flip at t={sim_time:.2f}s (z={cz:.3f}m, roll={roll:.2f}rad)", flush=True)
                    crashed = True
                    crash_reason = f"tumble_flip_z_{cz:.3f}"
                    break

                # 6. High-G impact shock from accelerometer (> 90 m/s^2 confirmed across multiple steps)
                if accel:
                    ax, ay, _ = accel.getValues()
                    acc_mag = math.hypot(ax, ay)
                    if acc_mag > 90.0:
                        high_g_counter_steps += 1
                        if high_g_counter_steps >= 3:
                            print(f"[GA Worker] Crash: High-G impact shock at t={sim_time:.2f}s ({acc_mag:.1f} m/s^2) pos=({cx:.2f},{cy:.2f})", flush=True)
                            crashed = True
                            crash_reason = f"high_g_impact_{acc_mag:.1f}"
                            break
                    else:
                        high_g_counter_steps = 0

                # --- H. FAST WALL PIN / STALL CHECK (Using Real Webots Velocity) ---
                if sim_time > 0.60 and not is_recovering and target_speed >= 0.50 and v_real < 0.12:
                    stall_counter_steps += 1
                    if stall_counter_steps * DT >= STALL_MAX_DURATION_S:
                        print(f"[GA Worker] Crash/Stall: Pinned against barrier for >{STALL_MAX_DURATION_S}s at t={sim_time:.2f}s (v_real={v_real:.2f}m/s)!", flush=True)
                        stalled = True
                        crashed = True
                        crash_reason = f"stalled_pinned_v_{v_real:.2f}"
                        break
                else:
                    stall_counter_steps = 0

        except Exception as e:
            import traceback
            print(f"[GA Worker] Error during trial execution: {e}", flush=True)
            traceback.print_exc()
            crashed = True
            crash_reason = f"exception_{e}"

        # Stop motors at trial end
        for m in motors.values():
            if m:
                m.setVelocity(0.0)

        # Compute Trial Performance Metrics FROM WEBOTS REAL DATA (NO SENSORS)
        total_dist = float(track_tracker.total_progress_m)
        mean_speed = float(np.mean(true_speed_history)) if true_speed_history else 0.0
        
        # Sanity clamping: physical ceiling is 5.5 m/s, floor is 0.0 m/s
        if math.isnan(mean_speed) or mean_speed < 0.0:
            mean_speed = 0.0
        elif mean_speed > 5.50:
            mean_speed = 5.50

        # If zero or negligible distance, mark crashed/failed run
        if math.isnan(total_dist) or total_dist <= 0.10:
            total_dist = 0.0
            if not crashed:
                crashed = True
                crash_reason = "negligible_distance_or_nan"

        metrics = {
            "distance_m": round(total_dist, 3),
            "mean_speed_mps": round(mean_speed, 3),
            "laps_completed": int(track_tracker.current_lap),
            "duration_s": round(sim_time, 2),
            "crashed": bool(crashed),
            "crash_reason": locals().get("crash_reason", "none"),
            "stalled": bool(stalled),
        }
        metrics["compute_multiplier"] = round(compute_efficiency_multiplier(gene_dict), 4)
        metrics["fitness"] = round(compute_scalar_fitness(metrics, gene_dict=gene_dict), 2)
        return metrics

    # Main TCP Worker Server Loop
    while True:
        conn = None
        try:
            conn, _ = server_sock.accept()
        except (BlockingIOError, socket.error):
            if robot.step(TIME_STEP_MS) == -1:
                break
            time.sleep(0.002)
            continue

        if conn is None:
            continue

        conn.setblocking(True)
        conn.settimeout(60.0)
        raw_req = ""
        while "\n" not in raw_req:
            try:
                chunk = conn.recv(8192).decode("utf-8")
                if not chunk:
                    break
                raw_req += chunk
            except Exception:
                break

        if not raw_req:
            try:
                conn.close()
            except Exception:
                pass
            continue

        try:
            req = json.loads(raw_req.strip())
            cmd = req.get("cmd", "EVAL")

            if cmd == "PING":
                resp = {"status": "PONG"}
            elif cmd == "SHUTDOWN":
                resp = {"status": "SHUTDOWN_OK"}
                conn.sendall((json.dumps(resp) + "\n").encode("utf-8"))
                conn.close()
                break
            elif cmd == "EVAL":
                p_name = req.get("planner", "OPTION_1_BVP")
                genes = req.get("genes", {})
                dur = float(req.get("duration_s", EVALUATION_DURATION_S))
                metrics = evaluate_candidate(p_name, genes, dur)
                resp = {"status": "OK", "metrics": metrics}
            else:
                resp = {"status": "ERROR", "message": f"Unknown cmd: {cmd}"}

            conn.sendall((json.dumps(resp) + "\n").encode("utf-8"))
        except Exception as e:
            err_resp = {"status": "ERROR", "message": str(e)}
            try:
                conn.sendall((json.dumps(err_resp) + "\n").encode("utf-8"))
            except Exception:
                pass
        finally:
            try:
                conn.close()
            except Exception:
                pass

    server_sock.close()


if __name__ == "__main__":
    run_supervisor_worker()
