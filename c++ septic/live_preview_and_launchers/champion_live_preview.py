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

# Discover controller and c++ septic paths
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
DIR_C_SEPTIC = CURR_DIR
while DIR_C_SEPTIC and os.path.basename(DIR_C_SEPTIC) != "c++ septic":
    parent = os.path.dirname(DIR_C_SEPTIC)
    if parent == DIR_C_SEPTIC:
        break
    DIR_C_SEPTIC = parent

WORKSPACE_ROOT = os.path.dirname(DIR_C_SEPTIC) if DIR_C_SEPTIC else os.path.dirname(CURR_DIR)
GA_OPTIMIZER_DIR = os.path.join(WORKSPACE_ROOT, "ga_planner_optimizer")
DATA_DIR = os.path.join(GA_OPTIMIZER_DIR, "data")

# Priority order: c++ septic folders first!
primary_paths = [
    os.path.join(DIR_C_SEPTIC, "planners"),
    os.path.join(DIR_C_SEPTIC, "best_genomes_and_configs"),
    os.path.join(DIR_C_SEPTIC, "live_preview_and_launchers"),
    DIR_C_SEPTIC,
]
fallback_paths = [
    os.path.join(GA_OPTIMIZER_DIR, "planners"),
    DATA_DIR,
    GA_OPTIMIZER_DIR,
    WORKSPACE_ROOT,
    CURR_DIR,
]

for p in fallback_paths:
    if p and p not in sys.path and os.path.exists(p):
        sys.path.append(p)

for p in reversed(primary_paths):
    if p and os.path.exists(p):
        while p in sys.path:
            sys.path.remove(p)
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
from anti_wrong_way_watchdog import AntiWrongWayWatchdog, wrap_to_pi

def wrap_to_pi(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))
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

# Planners
from planner_option1_bvp import BVPDynamicStateLattice
from planner_option2_sl_mpc import CurvilinearSLMPC
from planner_option3_septic import SepticSplineTractionProfiler

from ga_planner_config import ROBOT_START_POS, ROBOT_START_ROT

try:
    from cpp_planner_option3_septic import CppSepticSplineTractionProfiler
except ImportError:
    CppSepticSplineTractionProfiler = None

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
    Loads active config from best_planner_genome.json first (or active_preview_config.json fallback).
    Returns: (planner_type, genes, info_dict)
    """
    # 1. Primary: load best_planner_genome.json
    fb_path = os.path.join(GA_OPTIMIZER_DIR, "best_planner_genome.json")
    if os.path.exists(fb_path):
        try:
            with open(fb_path, "r", encoding="utf-8") as f:
                d = json.load(f)
                p_type = d.get("planner", "OPTION_3_SEPTIC")
                genes = d.get("genes", {})
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

    # 2. Fallback to active_preview_config.json
    active_path = os.path.join(GA_OPTIMIZER_DIR, "active_preview_config.json")
    if os.path.exists(active_path):
        try:
            with open(active_path, "r", encoding="utf-8") as f:
                d = json.load(f)
                p_type = d.get("planner", "OPTION_3_SEPTIC")
                genes = d.get("genes", {})
                info = {
                    "source": active_path,
                    "title": d.get("title", p_type),
                    "fitness": d.get("fitness", "N/A"),
                    "metrics": d.get("metrics", {}),
                }
                print(f"[Champion Preview] Active Config: {info['title']} (Fitness: {info['fitness']})", flush=True)
                return p_type, genes, info
        except Exception as e:
            print(f"[Champion Preview] Failed to parse active_preview_config.json: {e}", flush=True)

    return "OPTION_3_SEPTIC", {}, {"title": "Default Septic", "fitness": "N/A", "metrics": {}}


def create_planner(planner_type: str, genes: Dict[str, Any], costmap: CppSmartHybridCostmap):
    """Instantiates the specified planner with its gene dictionary and links the costmap."""
    p_up = planner_type.upper()
    mu_val = float(genes.get("mu_friction", 0.817))

    if "OPTION_2" in p_up or "MPC" in p_up:
        print(f"[Champion Preview] Initializing OPTION_2_SL_MPC Planner (Horizon={genes.get('horizon_N', 17)})...", flush=True)
        planner = CurvilinearSLMPC(
            horizon_N=int(genes.get("horizon_N", 17)),
            dt=float(genes.get("dt", 0.055)),
            q_progress=float(genes.get("q_progress", 7.60)),
            q_cte=float(genes.get("q_cte", 13.92)),
            q_heading=float(genes.get("q_heading", 9.99)),
            r_steer=float(genes.get("r_steer", 5.90)),
            r_accel=float(genes.get("r_accel", 1.74)),
            r_steer_rate=float(genes.get("r_steer_rate", 16.39)),
            mu_friction=mu_val,
            v_corner_factor=float(genes.get("v_corner_factor", 0.877)),
            pure_pursuit_blend_gain=float(genes.get("pp_blend_gain", 0.341)),
            lookahead_blend_m=float(genes.get("lookahead_blend_m", 0.404)),
        )
    elif "OPTION_1" in p_up or "BVP" in p_up:
        print(f"[Champion Preview] Initializing OPTION_1_BVP State Lattice Planner...", flush=True)
        planner = BVPDynamicStateLattice(
            v_cruise_max=float(genes.get("v_cruise_max", 3.37)),
            v_cruise_min=float(genes.get("v_cruise_min", 1.43)),
            mu_friction=mu_val,
            w_target_dist=float(genes.get("w_target_dist", 6.25)),
            w_heading_align=float(genes.get("w_heading_align", 7.16)),
            w_curvature=float(genes.get("w_curvature", 1.58)),
            w_side_bias=float(genes.get("w_side_bias", 1.13)),
            w_smoothness=float(genes.get("w_smoothness", 3.08)),
            slew_rate_limit_deg=float(genes.get("slew_rate_limit_deg", 13.29)),
        )
    else:
        # Septic Spline
        blend_w0 = float(genes.get("blend_w0", 0.092))
        blend_w1 = float(genes.get("blend_w1", 0.510))
        blend_w2 = float(genes.get("blend_w2", 0.920))
        num_blends = int(genes.get("num_blend_weights", 4))

        # Ensure blends are monotonic between blend_w0 and blend_w2
        if blend_w2 < blend_w0:
            blend_w2, blend_w0 = blend_w0, blend_w2

        if num_blends == 2:
            blends = [blend_w0, blend_w2]
        elif num_blends == 3:
            mid = float(np.clip(blend_w1, blend_w0, blend_w2))
            if abs(mid - blend_w0) < 1e-3 or abs(mid - blend_w2) < 1e-3:
                mid = 0.5 * (blend_w0 + blend_w2)
            blends = [blend_w0, mid, blend_w2]
        else:
            blends = [float(x) for x in np.linspace(blend_w0, blend_w2, num_blends)]

        print(f"[Champion Preview] Configured Septic Blends ({len(blends)} levels): {[round(b, 4) for b in blends]}", flush=True)

        if CppSepticSplineTractionProfiler is not None:
            print(
                f"[Champion Preview] Initializing C++ OPTION_3_SEPTIC Planner:\n"
                f"   - v_entry_max: {genes.get('v_entry_max', 4.92):.2f} m/s | v_cruise_min: {genes.get('v_cruise_min', 2.19):.2f} m/s\n"
                f"   - blend_w0: {blend_w0:.3f} | blend_w2: {blend_w2:.3f} | steer_kp: {genes.get('steer_kp', 0.862):.3f}\n"
                f"   - pp_lookahead_gain: {genes.get('pp_lookahead_gain', 0.647):.3f} | target_lookahead: {genes.get('target_lookahead_m', 1.76):.2f} m",
                flush=True,
            )
            planner = CppSepticSplineTractionProfiler(
                costmap_engine=costmap,
                v_entry_max=float(genes.get("v_entry_max", 4.92)),
                v_cruise_min=float(genes.get("v_cruise_min", 2.19)),
                mu_friction=mu_val,
                probe_samples=int(genes.get("probe_samples", 21)),
                num_lat_offsets=int(genes.get("num_lat_offsets", 5)),
                lat_offset_max=float(genes.get("lat_offset_max", 0.1168)),
                num_head_offsets=int(genes.get("num_head_offsets", 3)),
                head_offset_max=float(genes.get("head_offset_max", 0.1566)),
                num_blend_weights=len(blends),
                blend_weights=blends,
                max_extra_targets=int(genes.get("max_extra_targets", 0)),
                w_time=float(genes.get("w_time", 3.13)),
                w_clearance=float(genes.get("w_clearance", 1.32)),
                w_curvature_smooth=float(genes.get("kappa_smooth_gain", 2.57)),
                w_chord_align=float(genes.get("w_chord_tangent", 4.48)),
                w_steer_effort=float(genes.get("w_steer_effort", 2.016)),
                slew_deg_max=float(genes.get("slew_deg_max", 12.03)),
                steer_kp=float(genes.get("steer_kp", 0.8625)),
                pp_lookahead_gain=float(genes.get("pp_lookahead_gain", 0.6474)),
            )
        else:
            print("[Champion Preview] Fallback to Python Septic Spline Engine...", flush=True)
            planner = SepticSplineTractionProfiler(
                costmap_engine=costmap,
                v_entry_max=float(genes.get("v_entry_max", 2.96)),
                v_cruise_min=float(genes.get("v_cruise_min", 1.30)),
                mu_friction=mu_val,
                w_clearance=float(genes.get("w_clearance", 3.049)),
                w_chord_tangent=float(genes.get("w_chord_tangent", 3.012)),
                kappa_smooth_gain=float(genes.get("kappa_smooth_gain", 2.084)),
                pp_lookahead_gain=float(genes.get("pp_lookahead_gain", 0.402)),
                slew_deg_max=float(genes.get("slew_deg_max", 12.03)),
                steer_kp=float(genes.get("steer_kp", 1.124)),
                blend_w0=blend_w0,
                blend_w1=blend_w1,
                blend_w2=blend_w2,
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
            dev.enable(TIME_STEP_MS)
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
    r_infl = float(genes.get("r_inflation_m", 0.0886))
    target_lookahead_m = float(genes.get("target_lookahead_m", 1.262))
    aggr_dist_far = float(genes.get("aggr_dist_far_m", 1.40))
    aggr_dist_close = float(genes.get("aggr_dist_close_m", 0.55))
    aggr_min_scale = float(genes.get("aggr_min_scale", 0.15))
    steer_deadband = float(genes.get("steer_deadband_rad", 0.030))

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

    fusion = IMUEncoderFusion(wheel_radius=WHEEL_RADIUS, dt=DT)
    fusion.attach_devices(encoders=encoders, imu_dev=imu_dev, gyro_dev=gyro, accel_dev=accel)

    costmap = CppSmartHybridCostmap(
        grid_bounds=(GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX),
        resolution=GRID_RES,
        r_inflation_m=r_infl,
    )
    gap_scout = ToFGapScout()

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
    recovery_manager = StuckRecoveryManager()

    # Initialize Anti-Wrong-Way and U-Turn Discrimination Watchdog
    aww_watchdog = AntiWrongWayWatchdog()

    # Initialize Webots Keyboard
    keyboard = robot.getKeyboard() if hasattr(robot, "getKeyboard") else None
    if keyboard:
        keyboard.enable(TIME_STEP_MS)

    # Front & Back Pillar Nodes (for interactive toggling in test worlds)
    front_pillar_node = robot.getFromDef("FRONT_PILLAR") if WEBOTS_AVAILABLE and hasattr(robot, "getFromDef") else None
    front_pillar_trans = front_pillar_node.getField("translation") if front_pillar_node else None
    front_pillar_active = True
    front_pillar_pos = [2.6, 0.4, 0.06]
    if front_pillar_trans:
        p = front_pillar_trans.getSFVec3f()
        front_pillar_pos = [float(p[0]), float(p[1]), float(p[2])]

    back_pillar_node = robot.getFromDef("BACK_PILLAR") if WEBOTS_AVAILABLE and hasattr(robot, "getFromDef") else None
    back_pillar_trans = back_pillar_node.getField("translation") if back_pillar_node else None
    back_pillar_active = True
    back_pillar_pos = [-0.2066, 0.39, 0.06]
    if back_pillar_trans:
        p = back_pillar_trans.getSFVec3f()
        back_pillar_pos = [float(p[0]), float(p[1]), float(p[2])]

    # Initialize UDP streaming socket for Live Preview Map (Port 9876 / 9877)
    gui_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gui_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        cmd_sock.bind(("127.0.0.1", 9877))
        cmd_sock.setblocking(False)
    except Exception as e:
        print(f"[Champion Preview] Note: Could not bind cmd_sock to 9877: {e}", flush=True)

    sim_is_paused = False
    sim_step_one_frame = False
    sim_speed_mult = 0.50  # Default 0.50x (Half Speed)

    step_count = 0
    active_steer = 0.0
    last_print_time = 0.0
    lap_start_time = 0.0
    current_lap = 0
    stall_counter = 0
    dr_x = 0.0
    dr_y = 0.0
    rx = 0.0
    ry = 0.0
    ryaw = 0.0
    v_est = 0.0
    yaw_rate = 0.0
    target_speed = float(genes.get("v_cruise_min", V_MIN_CRUISE))
    dist = 0.0
    mode_desc = "NORMAL_DRIVE"

    cached_target_steer = 0.0
    cached_target_speed = float(genes.get("v_cruise_min", V_MIN_CRUISE))
    cached_theta_g = 0.0
    cached_xg = 0.0
    cached_yg = 1.0
    cached_width = 0.50
    cached_d_straight = TOF_MAX_RANGE_M
    cached_front_min_m = TOF_MAX_RANGE_M
    cached_plan_ms = 0.0
    cached_map_ms = 0.0
    cached_tofs = {k: TOF_MAX_RANGE_M for k in TOF_SENSOR_NAMES}
    front_tof_keys = ["tof_fl_3", "tof_fr_3", "tof_fl_2", "tof_fr_2"]

    def send_lean_telemetry():
        try:
            curr_sim_time = float(robot.getTime())
            aww_data = aww_watchdog.get_detailed_telemetry(rx, ry, ryaw, v_est, yaw_rate, now_s=curr_sim_time)
            lean_pkt = {
                "type": "telemetry",
                "time": round(curr_sim_time, 3),
                "mode": str(mode_desc),
                "v_est": round(float(v_est), 3),
                "v_cmd": round(float(target_speed), 3),
                "steer_deg": round(math.degrees(float(active_steer)), 2),
                "dist_m": round(float(dist), 3),
                "lap": int(current_lap),
                "pose": [round(float(rx), 3), round(float(ry), 3), round(float(ryaw), 3)],
                "front_pillar": bool(front_pillar_active),
                "back_pillar": bool(back_pillar_active),
                "watchdog_mode": str(getattr(aww_watchdog.mode, "name", aww_watchdog.mode)),
                "sim_speed": round(float(sim_speed_mult), 2),
                "aww": aww_data,
                "tof": {k: round(float(v), 3) for k, v in cached_tofs.items()},
            }
            gui_sock.sendto(json.dumps(lean_pkt).encode("utf-8"), ("127.0.0.1", 9876))
        except Exception as e:
            print(f"[Champion Preview] Lean UDP send error: {e}", flush=True)

    def toggle_front_pillar():
        nonlocal front_pillar_active
        if not front_pillar_trans:
            print("\n>>> [PILLAR] FRONT_PILLAR node not found in world", flush=True)
            return
        front_pillar_active = not front_pillar_active
        if front_pillar_active:
            front_pillar_trans.setSFVec3f([front_pillar_pos[0], front_pillar_pos[1], 0.06])
            print(f"\n>>> [KEYBOARD] FRONT PILLAR: RAISED (ACTIVE BLOCKING) at (2.60, 0.40, 0.06)", flush=True)
        else:
            front_pillar_trans.setSFVec3f([front_pillar_pos[0], front_pillar_pos[1], -5.0])
            print(f"\n>>> [KEYBOARD] FRONT PILLAR: LOWERED (CLEARED) at (2.60, 0.40, -5.00)", flush=True)
        send_lean_telemetry()

    def toggle_back_pillar():
        nonlocal back_pillar_active
        if not back_pillar_trans:
            print("\n>>> [PILLAR] BACK_PILLAR node not found in world", flush=True)
            return
        back_pillar_active = not back_pillar_active
        if back_pillar_active:
            back_pillar_trans.setSFVec3f([back_pillar_pos[0], back_pillar_pos[1], 0.06])
            print(f"\n>>> [KEYBOARD] BACK PILLAR: RAISED (ACTIVE BLOCKING) at (-0.21, 0.39, 0.06)", flush=True)
        else:
            back_pillar_trans.setSFVec3f([back_pillar_pos[0], back_pillar_pos[1], -5.0])
            print(f"\n>>> [KEYBOARD] BACK PILLAR: LOWERED (CLEARED) at (-0.21, 0.39, -5.00)", flush=True)
        send_lean_telemetry()

    def set_watchdog_mode(mode_name: str):
        aww_watchdog.set_mode(mode_name)
        print(f"\n>>> [KEYBOARD] ANTI-WRONG-WAY WATCHDOG MODE SWITCHED -> {mode_name}", flush=True)
        send_lean_telemetry()

    def reset_robot_pose():
        nonlocal dr_x, dr_y, rx, ry, ryaw
        if car_trans_field and car_rot_field:
            car_trans_field.setSFVec3f(START_POS)
            car_rot_field.setSFRotation(START_ROT)
            if robot_node:
                robot_node.resetPhysics()
            aww_watchdog.reset()
            recovery_manager.state = "DRIVE"
            fusion.x_est = 0.0
            fusion.y_est = 0.0
            fusion.total_distance_m = 0.0
            dr_x = 0.0
            dr_y = 0.0
            rx = 0.0
            ry = 0.0
            ryaw = 0.0
            print("\n>>> [ROBOT RESET] TELEPORTED TO START POSE (0, 0)", flush=True)
            send_lean_telemetry()

    plan_res = {}
    prob_map = None
    hazard_mask = None

    # Send initial startup beacon to GUI
    try:
        init_aww = aww_watchdog.get_detailed_telemetry(0.0, 0.0, 0.0, 0.0, 0.0, now_s=0.0)
        beacon_pkt = {
            "time": 0.0,
            "mode": "READY",
            "status": "OK",
            "v_est": 0.0,
            "v_cmd": 0.0,
            "steer_deg": 0.0,
            "dist_m": 0.0,
            "lap": 0,
            "gap": [0, 0, 0],
            "targets": [],
            "plan_ms": 0.0,
            "map_ms": 0.0,
            "surviving": 0,
            "candidates": 0,
            "tof": cached_tofs,
            "best_line": [],
            "candidate_lines": [],
            "grid_shape": [0, 0],
            "grid_bounds": [-1.0, 1.0, -0.5, 2.0],
            "grid_data": "",
            "hazard_data": "",
            "aww": init_aww,
            "pose": [0.0, 0.0, 0.0],
        }
        gui_sock.sendto(json.dumps(beacon_pkt).encode("utf-8"), ("127.0.0.1", 9876))
    except Exception:
        pass

    print(f"[Champion Preview] Controller active! Running Multi-Rate (120 Hz Sim/IMU/4WD, 60 Hz ToF/Planner)", flush=True)

    while robot.step(TIME_STEP_MS) != -1:
        wall_start_t = time.perf_counter()

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
                    sim_speed_mult = float(cmd_dict.get("val", 0.50))
                    print(f"[Champion Preview] Live updated simulation speed: {sim_speed_mult:.2f}x", flush=True)
                    send_lean_telemetry()
                elif cmd_type == "set_param":
                    p_name = cmd_dict.get("name")
                    p_val = float(cmd_dict.get("val", 0.0))
                    if hasattr(costmap, p_name):
                        setattr(costmap, p_name, p_val)
                    if hasattr(planner, p_name):
                        setattr(planner, p_name, p_val)
                    print(f"[Champion Preview] Live tuned {p_name} = {p_val}", flush=True)
                elif cmd_type == "toggle_front_pillar":
                    toggle_front_pillar()
                elif cmd_type == "toggle_back_pillar":
                    toggle_back_pillar()
                elif cmd_type == "set_watchdog_mode":
                    set_watchdog_mode(cmd_dict.get("mode", "OPTION_COMBINED_HYBRID"))
                elif cmd_type == "reset_robot":
                    reset_robot_pose()
        except (BlockingIOError, socket.error):
            pass
        except Exception as e:
            print(f"[Champion Preview] Command parse error: {e}", flush=True)

        # Read Webots Keyboard Input
        if keyboard:
            k = keyboard.getKey()
            while k > 0:
                if k in (ord('F'), ord('f')):
                    toggle_front_pillar()
                elif k in (ord('B'), ord('b')):
                    toggle_back_pillar()
                elif k == ord('1'):
                    set_watchdog_mode("OPTION_1_BREADCRUMB")
                elif k == ord('2'):
                    set_watchdog_mode("OPTION_2_KINEMATIC_ARC")
                elif k == ord('3'):
                    set_watchdog_mode("OPTION_3_FLOW_COSTMAP")
                elif k == ord('4'):
                    set_watchdog_mode("OPTION_4_LATERAL_SEPARATION")
                elif k == ord('5'):
                    set_watchdog_mode("OPTION_COMBINED_HYBRID")
                elif k == ord('0'):
                    set_watchdog_mode("DISABLED")
                elif k in (ord('+'), ord('=')):
                    sim_speed_mult = min(5.0, round(sim_speed_mult + 0.10, 2))
                    print(f"\n>>> [KEYBOARD] SIMULATION SPEED MULTIPLIER: {sim_speed_mult:.2f}x", flush=True)
                    send_lean_telemetry()
                elif k in (ord('-'), ord('_')):
                    sim_speed_mult = max(0.05, round(sim_speed_mult - 0.10, 2))
                    print(f"\n>>> [KEYBOARD] SIMULATION SPEED MULTIPLIER: {sim_speed_mult:.2f}x", flush=True)
                    send_lean_telemetry()
                elif k == ord('['):
                    sim_speed_mult = max(0.05, round(sim_speed_mult * 0.5, 2))
                    print(f"\n>>> [KEYBOARD] SIMULATION SPEED HALVED: {sim_speed_mult:.2f}x", flush=True)
                    send_lean_telemetry()
                elif k == ord(']'):
                    sim_speed_mult = min(5.0, round(sim_speed_mult * 2.0, 2))
                    print(f"\n>>> [KEYBOARD] SIMULATION SPEED DOUBLED: {sim_speed_mult:.2f}x", flush=True)
                    send_lean_telemetry()
                elif k in (ord('R'), ord('r')):
                    reset_robot_pose()
                elif k in (ord('H'), ord('h'), ord('?')):
                    print_interactive_help()
                k = keyboard.getKey()


        if sim_is_paused:
            for m in motors.values():
                if m:
                    m.setVelocity(0.0)
            send_lean_telemetry()
            time.sleep(0.02)
            continue

        if sim_step_one_frame:
            sim_is_paused = True
            sim_step_one_frame = False

        step_count += 1
        sim_time = step_count * DT

        # Update reverse moving obstacles
        if obs_fields and obs_engine.total_length > 0:
            n_obs = len(obs_fields)
            for k, field in enumerate(obs_fields):
                obs_s = (float(k + 1) * (obs_engine.total_length / float(n_obs + 1)) - MOVING_OBSTACLE_SPEED_MPS * sim_time) % obs_engine.total_length
                ox, oy = obs_engine.get_position_at_s(obs_s)
                field.setSFVec3f([float(ox), float(oy), 0.075])

        # Sensor fusion at 120 Hz
        v_est, yaw_rate, yaw = fusion.update(DT)
        v_est = max(0.0, v_est) if not math.isnan(v_est) else 0.0

        # Webots Real Ground-Truth Data (No Sensors)
        car_pos = car_trans_field.getSFVec3f() if car_trans_field else [0.0, 0.0, 0.028]
        cx, cy, cz = float(car_pos[0]), float(car_pos[1]), float(car_pos[2])

        vel = robot_node.getVelocity() if robot_node else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        v_real = float(math.hypot(vel[0], vel[1]))

        # Real distance along track centerline
        dist = float(track_tracker.update([cx, cy]))
        calculated_lap = int(track_tracker.current_lap)

        # Initial physics warmup pause (0.35 s)
        if sim_time < 0.35:
            for m in motors.values():
                if m:
                    m.setVelocity(0.0)
            for s in servos.values():
                if s:
                    s.setPosition(0.0)
            send_lean_telemetry()
            continue

        # Run 16-CH ToF, Costmap & Path Planner at 60 Hz (Decimation = 2)
        run_planner_step = (step_count % PLANNER_DECIMATION == 0)
        if run_planner_step:
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

            cached_target_steer = float(plan_res.get("cmd_steer_rad", 0.0))
            cached_target_speed = float(plan_res.get("cmd_speed_mps", genes.get("v_cruise_min", V_MIN_CRUISE)))
            d_fl3 = cached_tofs.get("tof_fl_3", TOF_MAX_RANGE_M)
            d_fr3 = cached_tofs.get("tof_fr_3", TOF_MAX_RANGE_M)
            cached_d_straight = min(d_fl3, d_fr3)
            cached_front_min_m = min([cached_tofs.get(k, TOF_MAX_RANGE_M) for k in front_tof_keys])

        # Dead-reckoned 2D pose from BNO08x + Encoders (with safe fallback)
        if hasattr(fusion, "get_pose_2d"):
            rx, ry, ryaw = fusion.get_pose_2d()
        else:
            dr_x += v_est * math.cos(yaw) * DT
            dr_y += v_est * math.sin(yaw) * DT
            rx, ry, ryaw = dr_x, dr_y, yaw

        # Update Anti-Wrong-Way Watchdog at 120 Hz
        aww_status = aww_watchdog.update(
            x=rx,
            y=ry,
            yaw=ryaw,
            v_est=v_est,
            yaw_rate=yaw_rate,
            cached_tofs=cached_tofs,
            now_s=sim_time,
        )

        if aww_status.is_wrong_way and not recovery_manager.state.startswith("PIVOT") and recovery_manager.state != "CORRECTIVE_180":
            if aww_status.corrective_action == "CORRECTIVE_180":
                yaw_diff = wrap_to_pi(aww_status.target_recovery_yaw - ryaw) if aww_status.target_recovery_yaw is not None else 1.0
                steer_sign = 1.0 if yaw_diff >= 0 else -1.0
                recovery_manager.trigger_corrective_180(
                    target_yaw=aww_status.target_recovery_yaw,
                    steer_sign=steer_sign,
                    current_yaw=ryaw,
                )

        # Recovery Manager & Steering Aggressiveness at 120 Hz
        is_recovering, rec_v, rec_steer = recovery_manager.update(
            dt=DT,
            current_v=v_est,
            commanded_v=cached_target_speed,
            cached_tofs=cached_tofs,
            heading_err=cached_theta_g,
            current_yaw=ryaw,
            current_x=rx,
            current_y=ry,
            now_s=sim_time,
        )

        if is_recovering:
            target_speed = rec_v
            target_steer = rec_steer
            active_steer = rec_steer
            mode_desc = f"RECOVERY_{recovery_manager.state}"
        else:
            target_speed = cached_target_speed
            target_steer = cached_target_steer
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
            mode_desc = "SEPTIC_SPLINE"

        # Electronic Differential & Motor Commands at 120 Hz
        actuators = ediff.update(target_v=target_speed, target_steer_rad=active_steer, gyro_z_rad_s=yaw_rate)

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
        # 1. Ultra-lean telemetry stream (450 bytes) at 60 Hz for Watchdog Studio & real-time controls
        if step_count % 2 == 0:
            send_lean_telemetry()

        # 2. Full costmap and trajectory packet at 12 Hz for map viewers
        if step_count % 10 == 0:
            try:
                prob_map_send = costmap.get_probability_map() if hasattr(costmap, "get_probability_map") else prob_map
                hazard_mask_send = costmap.get_hazard_mask() if hasattr(costmap, "get_hazard_mask") else hazard_mask
                if prob_map_send is not None and hazard_mask_send is not None:
                    prob_down = (prob_map_send[::2, ::2] * 255.0).astype(np.uint8)
                    hazard_down = hazard_mask_send[::2, ::2].astype(np.uint8)
                    grid_compressed = base64.b64encode(zlib.compress(prob_down.tobytes())).decode("ascii")
                    hazard_compressed = base64.b64encode(zlib.compress(hazard_down.tobytes())).decode("ascii")
                    grid_shape = [int(s) for s in prob_down.shape]
                else:
                    grid_compressed = ""
                    hazard_compressed = ""
                    grid_shape = [0, 0]

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

                aww_pkt = aww_watchdog.get_detailed_telemetry(rx, ry, ryaw, v_est, yaw_rate, now_s=sim_time)
                gui_pkt = {
                    "type": "full_map",
                    "time": round(float(sim_time), 3),
                    "mode": mode_desc,
                    "status": plan_res.get("status", "OK") if "plan_res" in locals() else "OK",
                    "v_est": round(float(v_est), 3),
                    "v_cmd": round(float(target_speed), 3),
                    "steer_deg": round(math.degrees(float(active_steer)), 2),
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
                    "grid_shape": grid_shape,
                    "grid_bounds": [-1.0, 1.0, -0.5, 2.0],
                    "grid_data": grid_compressed,
                    "hazard_data": hazard_compressed,
                    "front_pillar": bool(front_pillar_active),
                    "back_pillar": bool(back_pillar_active),
                    "watchdog_mode": str(getattr(aww_watchdog.mode, "name", aww_watchdog.mode)),
                    "sim_speed": round(float(sim_speed_mult), 2),
                    "aww": aww_pkt,
                    "pose": [round(float(rx), 3), round(float(ry), 3), round(float(ryaw), 3)],
                }
                gui_sock.sendto(json.dumps(gui_pkt).encode("utf-8"), ("127.0.0.1", 9876))
            except Exception as e:
                print(f"[Champion Preview] UDP streaming error: {e}", flush=True)

        # Stall logging
        if sim_time > 0.5 and target_speed >= 0.50 and v_real < 0.12:
            stall_counter += 1
            if stall_counter * DT >= 0.5 and sim_time - last_print_time >= 0.5:
                print(f"  [Vehicle Stall] Stopped against barrier (v_real={v_real:.2f}m/s).", flush=True)
        else:
            stall_counter = 0

        # Actuators
        actuators = ediff.update(target_v=target_speed, target_steer_rad=target_steer, gyro_z_rad_s=yaw_rate)

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
                f"Dist: {dist:6.1f}m | Speed: {v_real:4.2f}m/s (cmd: {target_speed:4.2f}) | "
                f"Steer: {math.degrees(target_steer):+5.1f} deg | Plan Latency: {cached_plan_ms:4.2f}ms",
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
