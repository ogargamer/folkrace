"""
================================================================================
  UNIFIED FOLKRACE PATH PLANNER & WEBOTS CONTROLLER DISPATCHER
================================================================================
  Acts as the single authoritative Webots robot controller matching track_racing.wbt.
  Dispatches cleanly between three operational modes via WEBOTS_CONTROLLER_MODE:
    1. 'racing'    (default): Interactive real-time studio with UDP telemetry & live sliders
    2. 'preview'   : Champion genome visual replay with ground-truth progress tracking
    3. 'ga_eval'   : High-throughput headless worker for genetic algorithm optimization
================================================================================
"""

import os
import sys
import time
import math
import json
import socket
from typing import Dict, List, Optional, Tuple, Any
import numpy as np

# Ensure controller directories are on sys.path
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
MY_PROJECT_DIR = os.path.dirname(CONTROLLERS_DIR)
WEBOTS_DIR = os.path.dirname(os.path.dirname(MY_PROJECT_DIR))
GA_OPTIMIZER_DIR = os.path.join(WEBOTS_DIR, "optimization", "ga_planner_optimizer")
DATA_DIR = os.path.join(GA_OPTIMIZER_DIR, "data")
WORKSPACE_ROOT = os.path.dirname(WEBOTS_DIR)

for p in [CURR_DIR, CONTROLLERS_DIR, MY_PROJECT_DIR, GA_OPTIMIZER_DIR, DATA_DIR, WORKSPACE_ROOT]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

try:
    from controller import Robot, Supervisor
    WEBOTS_AVAILABLE = True
except ImportError:
    WEBOTS_AVAILABLE = False

from vehicle_harness import FolkraceVehicle, read_tof_multi_ray
from progress_tracker import WebotsTrackProgressTracker, load_track_walls
from obstacle_engine import ReverseObstacleEngine, MovingObstacleManager
from planner_config import (
    TIME_STEP_MS,
    DT,
    SIM_HZ,
    PLANNER_HZ,
    DT_PLANNER,
    TOF_MAX_RANGE_M,
    MOVING_OBSTACLE_NAMES,
    MOVING_OBSTACLE_SPEED_MPS,
    MOVING_OBSTACLE_Z_ELEVATION,
    V_MIN_CRUISE,
    V_MAX_CRUISE,
)


def find_trajectory_csv() -> str:
    """Finds canonical cyan champion trajectory CSV path."""
    cands = [
        os.path.join(DATA_DIR, "cyan_champion_trajectory.csv"),
        os.path.join(WORKSPACE_ROOT, "cyan_champion_trajectory.csv"),
    ]
    for c in cands:
        if os.path.exists(c):
            return c
    return ""


def to_py(obj):
    if isinstance(obj, (np.floating, float)):
        return float(obj)
    elif isinstance(obj, (np.integer, int)):
        return int(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, dict):
        return {k: to_py(v) for k, v in obj.items()}
    elif isinstance(obj, (list, tuple)):
        return [to_py(v) for v in obj]
    return obj


# ==============================================================================
# MODE 1: INTERACTIVE RACING STUDIO (Default)
# ==============================================================================
def run_racing_studio(robot: any, vehicle: FolkraceVehicle) -> None:
    print("=" * 80, flush=True)
    print("  FOLKRACE AUTONOMOUS RACING STUDIO (120 Hz Dynamics / 60 Hz Planner)", flush=True)
    print("  Zero cheating: 16-ch ToF, IMU+Encoder Odometry, Curvilinear S-L MPC", flush=True)
    print("=" * 80, flush=True)

    traj_csv = find_trajectory_csv()
    obs_engine = ReverseObstacleEngine(traj_csv, speed_mps=MOVING_OBSTACLE_SPEED_MPS)
    obs_manager = MovingObstacleManager(
        robot=robot,
        engine=obs_engine,
        obstacle_names=MOVING_OBSTACLE_NAMES,
        elevation_z=MOVING_OBSTACLE_Z_ELEVATION,
        speed_mps=MOVING_OBSTACLE_SPEED_MPS,
    )

    # UDP Telemetry Broadcaster (Port 9876) & Command Receiver (Port 9877)
    gui_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gui_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        cmd_sock.bind(("127.0.0.1", 9877))
        cmd_sock.setblocking(False)
        print("[Racing Studio] Live GUI command receiver bound to port 9877", flush=True)
    except Exception as e:
        print(f"[Racing Studio] Note: Could not bind command port 9877: {e}", flush=True)

    traj_csv = find_trajectory_csv()
    track_tracker = WebotsTrackProgressTracker(traj_csv)
    track_tracker.reset(vehicle.get_ground_truth_pose()[:2])

    sim_paused = False
    sim_step_requested = False
    sim_speed_mult = 1.0
    sim_time = 0.0
    telemetry_hz = max(1, min(60, int(os.environ.get("WEBOTS_TELEMETRY_HZ", 60))))
    last_telemetry_time = -1.0

    def to_py(obj):
        if isinstance(obj, (np.floating, float)):
            return float(obj)
        elif isinstance(obj, (np.integer, int)):
            return int(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {k: to_py(v) for k, v in obj.items()}
        elif isinstance(obj, (list, tuple)):
            return [to_py(v) for v in obj]
        return obj

    # Discover Front and Back Pillars if present in the world (e.g. test_wrong_way.wbt)
    front_pillar_node = robot.getFromDef("FRONT_PILLAR") if (vehicle.is_supervisor and hasattr(robot, "getFromDef")) else None
    front_pillar_trans = front_pillar_node.getField("translation") if front_pillar_node else None
    front_pillar_active = True
    front_pillar_pos = [2.60, 0.40]

    back_pillar_node = robot.getFromDef("BACK_PILLAR") if (vehicle.is_supervisor and hasattr(robot, "getFromDef")) else None
    back_pillar_trans = back_pillar_node.getField("translation") if back_pillar_node else None
    back_pillar_active = True
    back_pillar_pos = [-0.2066, 0.39]

    def toggle_front_pillar():
        nonlocal front_pillar_active
        if not front_pillar_trans:
            return
        front_pillar_active = not front_pillar_active
        if front_pillar_active:
            front_pillar_trans.setSFVec3f([front_pillar_pos[0], front_pillar_pos[1], 0.06])
            print(f"\n>>> [PILLAR] FRONT PILLAR: RAISED (BLOCKING) at (2.60, 0.40, 0.06)", flush=True)
        else:
            front_pillar_trans.setSFVec3f([front_pillar_pos[0], front_pillar_pos[1], -5.0])
            print(f"\n>>> [PILLAR] FRONT PILLAR: LOWERED (CLEARED) at (2.60, 0.40, -5.00)", flush=True)

    def toggle_back_pillar():
        nonlocal back_pillar_active
        if not back_pillar_trans:
            return
        back_pillar_active = not back_pillar_active
        if back_pillar_active:
            back_pillar_trans.setSFVec3f([back_pillar_pos[0], back_pillar_pos[1], 0.06])
            print(f"\n>>> [PILLAR] BACK PILLAR: RAISED (BLOCKING) at (-0.21, 0.39, 0.06)", flush=True)
        else:
            back_pillar_trans.setSFVec3f([back_pillar_pos[0], back_pillar_pos[1], -5.0])
            print(f"\n>>> [PILLAR] BACK PILLAR: LOWERED (CLEARED) at (-0.21, 0.39, -5.00)", flush=True)

    keyboard = robot.getKeyboard() if hasattr(robot, "getKeyboard") else None
    if keyboard:
        try:
            keyboard.enable(TIME_STEP_MS)
        except Exception:
            pass

    while robot.step(TIME_STEP_MS) != -1:
        wall_start_t = time.perf_counter()
        sim_time += DT

        if keyboard:
            try:
                k = keyboard.getKey()
                while k > 0:
                    if k in (ord('F'), ord('f')):
                        toggle_front_pillar()
                    elif k in (ord('B'), ord('b')):
                        toggle_back_pillar()
                    elif k == ord('1'):
                        vehicle.watchdog.set_mode("OPTION_1_BREADCRUMB")
                        print("\n>>> [KEYBOARD] AWW WATCHDOG: OPTION_1_BREADCRUMB", flush=True)
                    elif k == ord('2'):
                        vehicle.watchdog.set_mode("OPTION_2_KINEMATIC_ARC")
                        print("\n>>> [KEYBOARD] AWW WATCHDOG: OPTION_2_KINEMATIC_ARC", flush=True)
                    elif k == ord('3'):
                        vehicle.watchdog.set_mode("OPTION_3_FLOW_COSTMAP")
                        print("\n>>> [KEYBOARD] AWW WATCHDOG: OPTION_3_FLOW_COSTMAP", flush=True)
                    elif k == ord('4'):
                        vehicle.watchdog.set_mode("OPTION_4_LATERAL_SEPARATION")
                        print("\n>>> [KEYBOARD] AWW WATCHDOG: OPTION_4_LATERAL_SEPARATION", flush=True)
                    elif k == ord('5'):
                        vehicle.watchdog.set_mode("OPTION_COMBINED_HYBRID")
                        print("\n>>> [KEYBOARD] AWW WATCHDOG: OPTION_COMBINED_HYBRID", flush=True)
                    elif k == ord('0'):
                        vehicle.watchdog.set_mode("DISABLED")
                        print("\n>>> [KEYBOARD] AWW WATCHDOG: DISABLED", flush=True)
                    elif k in (ord('R'), ord('r')):
                        vehicle.reset(start_pos=[0.0, 0.0, 0.04])
                        track_tracker.reset([0.0, 0.0])
                        print("\n>>> [KEYBOARD] ROBOT POSE RESET", flush=True)
                    k = keyboard.getKey()
            except Exception:
                pass

        # 1. Check incoming live GUI commands on Port 9877
        try:
            while True:
                cmd_data, _ = cmd_sock.recvfrom(4096)
                cmd_pkt = json.loads(cmd_data.decode("utf-8"))
                c = cmd_pkt.get("cmd", "")
                if c == "pause":
                    sim_paused = True
                elif c == "play":
                    sim_paused = False
                elif c == "step":
                    sim_step_requested = True
                elif c == "set_speed":
                    sim_speed_mult = float(cmd_pkt.get("val", 1.0))
                    print(f"[Racing Studio] Live simulation speed set to: {sim_speed_mult:.2f}x", flush=True)
                    if vehicle.is_supervisor and hasattr(robot, "simulationSetMode"):
                        if sim_speed_mult > 1.05:
                            robot.simulationSetMode(Supervisor.SIMULATION_MODE_FAST)
                        else:
                            robot.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)
                elif c == "set_telemetry_hz":
                    telemetry_hz = max(1, min(60, int(cmd_pkt.get("val", 60))))
                    print(f"[Racing Studio] Telemetry streaming rate set to: {telemetry_hz} Hz", flush=True)
                elif c == "toggle_front_pillar":
                    toggle_front_pillar()
                elif c == "toggle_back_pillar":
                    toggle_back_pillar()
                elif c == "set_watchdog_mode":
                    mode_name = cmd_pkt.get("mode", "OPTION_COMBINED_HYBRID")
                    vehicle.watchdog.set_mode(mode_name)
                    print(f"[Racing Studio] Watchdog mode set to: {mode_name}", flush=True)
                elif c == "reset_robot":
                    vehicle.reset(start_pos=[0.0, 0.0, 0.04])
                    track_tracker.reset([0.0, 0.0])
                    print("[Racing Studio] Robot reset requested by GUI", flush=True)
                elif c == "set_param":
                    p_name = cmd_pkt.get("name", "")
                    p_val = cmd_pkt.get("val", None)
                    if p_val is not None:
                        if p_name == "telemetry_hz":
                            telemetry_hz = max(1, min(60, int(p_val)))
                            print(f"[Racing Studio] Telemetry streaming rate set to: {telemetry_hz} Hz", flush=True)
                        elif p_name == "r_inflation_m":
                            vehicle.costmap.r_inflation_m = float(p_val)
                            vehicle.costmap._handle = vehicle.costmap._lib.costmap_create(
                                vehicle.costmap.x_min, vehicle.costmap.x_max, vehicle.costmap.y_min, vehicle.costmap.y_max, vehicle.costmap.res, vehicle.costmap.r_inflation_m
                            )
                        elif p_name == "target_speed":
                            vehicle.planner.v_cruise_max = float(p_val)
                        elif p_name == "alpha_peak":
                            vehicle.planner.alpha_peak = float(p_val)
                        elif p_name == "alt_switch_hysteresis" and hasattr(vehicle.planner, "alt_switch_hysteresis"):
                            vehicle.planner.alt_switch_hysteresis = float(p_val)
                        elif p_name == "alt_min_separation_m":
                            if hasattr(vehicle.gap_scout, "alt_min_separation_m"):
                                vehicle.gap_scout.alt_min_separation_m = float(p_val)
                            if hasattr(vehicle.planner, "alt_min_separation_m"):
                                vehicle.planner.alt_min_separation_m = float(p_val)
        except (BlockingIOError, socket.error):
            pass
        except Exception:
            pass

        if sim_paused and not sim_step_requested:
            for m in vehicle.motors.values():
                if m:
                    m.setVelocity(0.0)
            time.sleep(0.02)
            continue

        sim_step_requested = False

        # 2. Update Dynamic Obstacles
        obs_positions = obs_manager.update(sim_time)

        # 3. Vehicle Physics & Planner Step
        telemetry, planner_ran = vehicle.step(dt=DT)

        # 4. Broadcast Telemetry & Update Webots World HUD / Terminal based on telemetry_hz
        if planner_ran:
            gx, gy, _ = vehicle.get_ground_truth_pose()
            track_dist = track_tracker.update([gx, gy])

            telem_period = 1.0 / float(telemetry_hz)
            if (sim_time - last_telemetry_time) >= (telem_period - 0.001) or last_telemetry_time < 0:
                last_telemetry_time = sim_time

                telemetry["obstacles"] = [[round(ox, 3), round(oy, 3)] for (ox, oy) in obs_positions]
                telemetry["active_planner"] = "OPTION2_SL_MPC"
                telemetry["sim_time"] = round(sim_time, 2)
                telemetry["telemetry_hz"] = telemetry_hz
                telemetry["front_pillar"] = bool(front_pillar_active)
                telemetry["back_pillar"] = bool(back_pillar_active)
                telemetry["sim_speed"] = round(float(sim_speed_mult), 2)
                try:
                    pkt_bytes = json.dumps(to_py(telemetry)).encode("utf-8")
                    gui_sock.sendto(pkt_bytes, ("127.0.0.1", 9876))
                except Exception:
                    pass

                # HUD Overlay in Webots 3D World Viewport
                if hasattr(robot, "setLabel"):
                    v_real = telemetry.get("v_real", 0.0)
                    v_cmd = telemetry.get("v_cmd", 0.0)
                    steer_deg = telemetry.get("steer_deg", 0.0)
                    mode_str = telemetry.get("mode", "DRIVE")
                    plan_ms = telemetry.get("plan_ms", 0.0)
                    lap_num = getattr(track_tracker, "current_lap", 0)

                    color_mode = 0x00FF88 if "RECOVERY" not in mode_str else 0xFF3333
                    robot.setLabel(0, f"SL-MPC {telemetry_hz}Hz | Mode: {mode_str:12s} | Time: {sim_time:5.1f}s | Lap: {lap_num}", 0.01, 0.01, 0.06, 0x00E5FF, 0.0, "Arial")
                    robot.setLabel(1, f"Speed: {v_real:4.2f} m/s (cmd: {v_cmd:4.2f}) | Steer: {steer_deg:+5.1f} deg | Plan: {plan_ms:4.2f}ms", 0.01, 0.04, 0.06, color_mode, 0.0, "Arial")
                    robot.setLabel(2, f"Track Progress: {track_dist:6.1f}m", 0.01, 0.07, 0.05, 0xFFFFFF, 0.0, "Arial")

                # Terminal Telemetry Output (throttled to telemetry_hz)
                print(
                    f"[Telemetry {telemetry_hz:2d}Hz] Mode: {telemetry['mode']:14s} | Time: {sim_time:5.2f}s | Lap: {track_tracker.current_lap:2d} | "
                    f"Dist: {track_dist:6.1f}m | Speed: {telemetry['v_real']:4.2f}m/s (cmd: {telemetry['v_cmd']:4.2f}) | "
                    f"Steer: {telemetry['steer_deg']:+5.1f} deg | Plan: {telemetry['plan_ms']:4.2f}ms",
                    flush=True,
                )

        # 5. Simulation Speed Throttling (only active when slow-motion < 1.0x is explicitly requested)
        if sim_speed_mult < 0.99:
            target_step_dt = (TIME_STEP_MS / 1000.0) / max(0.01, sim_speed_mult)
            elapsed_wall = time.perf_counter() - wall_start_t
            if elapsed_wall < target_step_dt:
                time.sleep(target_step_dt - elapsed_wall)


# ==============================================================================
# MODE 2: CHAMPION GENOME LIVE PREVIEW
# ==============================================================================
def run_champion_preview(robot: any, vehicle: FolkraceVehicle) -> None:
    print("=" * 80, flush=True)
    print("  FOLKRACE CHAMPION PREVIEW REPLAY RUN", flush=True)
    print("=" * 80, flush=True)

    # 1. Load active champion configuration
    active_path = os.path.join(GA_OPTIMIZER_DIR, "active_preview_config.json")
    ckpt_path = os.path.join(GA_OPTIMIZER_DIR, "ga_checkpoint.json")
    best_path = os.path.join(GA_OPTIMIZER_DIR, "best_planner_genome.json")
    genes: Dict[str, Any] = {}
    title = "Default SL-MPC"

    for candidate_path in [active_path, ckpt_path, best_path]:
        if os.path.exists(candidate_path):
            try:
                with open(candidate_path, "r", encoding="utf-8") as f:
                    cfg = json.load(f)
                genes = cfg.get("genes") or cfg.get("best_genome") or cfg
                title = cfg.get("title", cfg.get("meta", {}).get("title", os.path.basename(candidate_path)))
                print(f"[Champion Preview] Loaded genome from {os.path.basename(candidate_path)}: {title}", flush=True)
                break
            except Exception as e:
                print(f"[Champion Preview] Warning: Could not parse {candidate_path}: {e}", flush=True)

    vehicle.configure(genes)

    traj_csv = find_trajectory_csv()
    obs_engine = ReverseObstacleEngine(traj_csv, speed_mps=MOVING_OBSTACLE_SPEED_MPS)
    obs_manager = MovingObstacleManager(
        robot=robot,
        engine=obs_engine,
        obstacle_names=MOVING_OBSTACLE_NAMES,
        elevation_z=MOVING_OBSTACLE_Z_ELEVATION,
        speed_mps=MOVING_OBSTACLE_SPEED_MPS,
    )

    track_tracker = WebotsTrackProgressTracker(traj_csv)
    track_left_wall, track_right_wall = load_track_walls()

    from ga_planner_config import ROBOT_START_POS, ROBOT_START_ROT
    start_pos = list(ROBOT_START_POS)
    start_rot = list(ROBOT_START_ROT)
    vehicle.reset(start_pos=start_pos, start_rot=start_rot)
    track_tracker.reset(start_pos[:2])

    gui_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gui_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        cmd_sock.bind(("127.0.0.1", 9877))
        cmd_sock.setblocking(False)
    except Exception:
        pass

    keyboard = robot.getKeyboard() if hasattr(robot, "getKeyboard") else None
    if keyboard:
        try:
            keyboard.enable(TIME_STEP_MS)
        except Exception:
            pass

    sim_time = 0.0
    sim_paused = False
    sim_step_requested = False
    sim_speed_mult = 1.0
    sim_time = 0.0
    telemetry_hz = max(1, min(60, int(os.environ.get("WEBOTS_TELEMETRY_HZ", 60))))
    last_telemetry_time = -1.0

    def to_py(obj):
        if isinstance(obj, (np.floating, float)):
            return float(obj)
        elif isinstance(obj, (np.integer, int)):
            return int(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {k: to_py(v) for k, v in obj.items()}
        elif isinstance(obj, (list, tuple)):
            return [to_py(v) for v in obj]
        return obj

    while robot.step(TIME_STEP_MS) != -1:
        wall_start_t = time.perf_counter()
        sim_time += DT

        if keyboard:
            try:
                k = keyboard.getKey()
                while k > 0:
                    if k in (ord('+'), ord('=')):
                        sim_speed_mult = min(5.0, round(sim_speed_mult + 0.10, 2))
                        print(f"\n>>> [KEYBOARD] SIM SPEED: {sim_speed_mult:.2f}x", flush=True)
                        if vehicle.is_supervisor and hasattr(robot, "simulationSetMode"):
                            if sim_speed_mult > 1.05:
                                robot.simulationSetMode(Supervisor.SIMULATION_MODE_FAST)
                            else:
                                robot.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)
                    elif k in (ord('-'), ord('_')):
                        sim_speed_mult = max(0.05, round(sim_speed_mult - 0.10, 2))
                        print(f"\n>>> [KEYBOARD] SIM SPEED: {sim_speed_mult:.2f}x", flush=True)
                        if vehicle.is_supervisor and hasattr(robot, "simulationSetMode"):
                            if sim_speed_mult > 1.05:
                                robot.simulationSetMode(Supervisor.SIMULATION_MODE_FAST)
                            else:
                                robot.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)
                    elif k in (ord('P'), ord('p'), ord(' ')):
                        sim_paused = not sim_paused
                        print(f"\n>>> [KEYBOARD] PAUSED: {sim_paused}", flush=True)
                    elif k in (ord('R'), ord('r')):
                        vehicle.reset(start_pos=start_pos, start_rot=start_rot)
                        obs_manager.reset()
                        if hasattr(robot, "simulationResetPhysics"):
                            robot.simulationResetPhysics()
                        for m in vehicle.motors.values():
                            if m:
                                m.setPosition(float("inf"))
                                m.setVelocity(0.0)
                        for s in vehicle.servos.values():
                            if s:
                                s.setPosition(0.0)
                        for _ in range(3):
                            robot.step(TIME_STEP_MS)
                        track_tracker.reset(start_pos[:2])
                        print("\n>>> [KEYBOARD] ROBOT POSE RESET TO TRACK START", flush=True)
                    k = keyboard.getKey()
            except Exception:
                pass

        try:
            while True:
                cmd_data, _ = cmd_sock.recvfrom(4096)
                cmd_pkt = json.loads(cmd_data.decode("utf-8"))
                c = cmd_pkt.get("cmd", "")
                if c == "pause":
                    sim_paused = True
                elif c == "play":
                    sim_paused = False
                elif c == "step":
                    sim_step_requested = True
                elif c == "set_speed":
                    sim_speed_mult = float(cmd_pkt.get("val", 1.0))
                    print(f"[Champion Preview] Live simulation speed set to: {sim_speed_mult:.2f}x", flush=True)
                    if vehicle.is_supervisor and hasattr(robot, "simulationSetMode"):
                        if sim_speed_mult > 1.05:
                            robot.simulationSetMode(Supervisor.SIMULATION_MODE_FAST)
                        else:
                            robot.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)
                elif c == "set_telemetry_hz":
                    telemetry_hz = max(1, min(60, int(cmd_pkt.get("val", 60))))
                    print(f"[Champion Preview] Telemetry streaming rate set to: {telemetry_hz} Hz", flush=True)
                elif c == "set_param":
                    p_name = cmd_pkt.get("name", "")
                    p_val = cmd_pkt.get("val", None)
                    if p_val is not None:
                        if p_name == "telemetry_hz":
                            telemetry_hz = max(1, min(60, int(p_val)))
                            print(f"[Champion Preview] Telemetry streaming rate set to: {telemetry_hz} Hz", flush=True)
                        elif p_name == "r_inflation_m":
                            vehicle.costmap.r_inflation_m = float(p_val)
                            vehicle.costmap._handle = vehicle.costmap._lib.costmap_create(
                                vehicle.costmap.x_min, vehicle.costmap.x_max, vehicle.costmap.y_min, vehicle.costmap.y_max, vehicle.costmap.res, vehicle.costmap.r_inflation_m
                            )
                        elif p_name == "target_speed":
                            vehicle.planner.v_cruise_max = float(p_val)
                        elif p_name == "alpha_peak":
                            vehicle.planner.alpha_peak = float(p_val)
                        elif p_name == "alt_switch_hysteresis" and hasattr(vehicle.planner, "alt_switch_hysteresis"):
                            vehicle.planner.alt_switch_hysteresis = float(p_val)
                        elif p_name == "alt_min_separation_m":
                            if hasattr(vehicle.gap_scout, "alt_min_separation_m"):
                                vehicle.gap_scout.alt_min_separation_m = float(p_val)
                            if hasattr(vehicle.planner, "alt_min_separation_m"):
                                vehicle.planner.alt_min_separation_m = float(p_val)
        except (BlockingIOError, socket.error):
            pass
        except Exception:
            pass

        if sim_paused and not sim_step_requested:
            for m in vehicle.motors.values():
                if m:
                    m.setVelocity(0.0)
            time.sleep(0.02)
            continue

        sim_step_requested = False

        obs_positions = obs_manager.update(sim_time)
        telemetry, planner_ran = vehicle.step(dt=DT)

        gx, gy, _ = vehicle.get_ground_truth_pose()
        track_dist = track_tracker.update([gx, gy])

        if planner_ran:
            telem_period = 1.0 / float(telemetry_hz)
            if (sim_time - last_telemetry_time) >= (telem_period - 0.001) or last_telemetry_time < 0:
                last_telemetry_time = sim_time

                telemetry["obstacles"] = [[round(ox, 3), round(oy, 3)] for (ox, oy) in obs_positions]
                telemetry["active_planner"] = "OPTION2_SL_MPC"
                telemetry["sim_time"] = round(sim_time, 2)
                telemetry["telemetry_hz"] = telemetry_hz
                telemetry["metrics"] = {
                    "track_dist_m": round(track_dist, 2),
                    "laps": int(track_tracker.current_lap),
                    "speed_mps": round(telemetry.get("v_fused", 0.0), 2),
                    "title": title,
                }
                try:
                    pkt_bytes = json.dumps(to_py(telemetry)).encode("utf-8")
                    gui_sock.sendto(pkt_bytes, ("127.0.0.1", 9876))
                except Exception:
                    pass

                # HUD Overlay in Webots 3D World Viewport
                if hasattr(robot, "setLabel"):
                    v_real = telemetry.get("v_real", 0.0)
                    v_cmd = telemetry.get("v_cmd", 0.0)
                    steer_deg = telemetry.get("steer_deg", 0.0)
                    mode_str = telemetry.get("mode", "DRIVE")
                    plan_ms = telemetry.get("plan_ms", 0.0)
                    lap_num = getattr(track_tracker, "current_lap", 0)

                    color_mode = 0x00FF88 if "RECOVERY" not in mode_str else 0xFF3333
                    robot.setLabel(0, f"CHAMPION PREVIEW {telemetry_hz}Hz | Mode: {mode_str:12s} | Time: {sim_time:5.1f}s | Lap: {lap_num}", 0.01, 0.01, 0.06, 0x00E5FF, 0.0, "Arial")
                    robot.setLabel(1, f"Speed: {v_real:4.2f} m/s (cmd: {v_cmd:4.2f}) | Steer: {steer_deg:+5.1f} deg | Plan: {plan_ms:4.2f}ms", 0.01, 0.04, 0.06, color_mode, 0.0, "Arial")
                    robot.setLabel(2, f"Track Progress: {track_dist:6.1f}m | {title[:40]}", 0.01, 0.07, 0.05, 0xFFFFFF, 0.0, "Arial")

                # Terminal Telemetry Output (throttled to telemetry_hz)
                print(
                    f"[Telemetry {telemetry_hz:2d}Hz] Mode: {telemetry['mode']:14s} | Time: {sim_time:5.2f}s | Lap: {track_tracker.current_lap:2d} | "
                    f"Dist: {track_dist:6.1f}m | Speed: {telemetry['v_real']:4.2f}m/s (cmd: {telemetry['v_cmd']:4.2f}) | "
                    f"Steer: {telemetry['steer_deg']:+5.1f} deg | Plan: {telemetry['plan_ms']:4.2f}ms",
                    flush=True,
                )

        # Simulation Speed Throttling (only active when slow-motion < 1.0x is explicitly requested)
        if sim_speed_mult < 0.99:
            target_step_dt = (TIME_STEP_MS / 1000.0) / max(0.01, sim_speed_mult)
            elapsed_wall = time.perf_counter() - wall_start_t
            if elapsed_wall < target_step_dt:
                time.sleep(target_step_dt - elapsed_wall)


# ==============================================================================
# MODE 3: GA OPTIMIZER HEADLESS EVALUATION WORKER
# ==============================================================================
def run_ga_worker(robot: any, vehicle: FolkraceVehicle) -> None:
    from ga_planner_config import (
        BASE_WORKER_PORT,
        ROBOT_START_POS,
        ROBOT_START_ROT,
        compute_scalar_fitness,
        compute_efficiency_multiplier,
    )

    worker_port = int(os.environ.get("GA_WORKER_PORT", BASE_WORKER_PORT))
    print(f"[GA Worker] Initializing on TCP port {worker_port}...", flush=True)

    traj_csv = find_trajectory_csv()
    obs_engine = ReverseObstacleEngine(traj_csv, speed_mps=MOVING_OBSTACLE_SPEED_MPS)
    obs_manager = MovingObstacleManager(
        robot=robot,
        engine=obs_engine,
        obstacle_names=MOVING_OBSTACLE_NAMES,
        elevation_z=MOVING_OBSTACLE_Z_ELEVATION,
        speed_mps=MOVING_OBSTACLE_SPEED_MPS,
    )
    track_left_wall, track_right_wall = load_track_walls()

    start_pos = list(ROBOT_START_POS)
    start_rot = list(ROBOT_START_ROT)

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(("127.0.0.1", worker_port))
    server_sock.listen(5)
    server_sock.setblocking(False)

    print(f"[GA Worker {worker_port}] Ready for evaluation requests.", flush=True)

    def evaluate_candidate(gene_dict: Dict[str, Any], duration_s: float) -> Dict[str, Any]:
        # Reset vehicle and obstacles
        vehicle.reset(start_pos=start_pos, start_rot=start_rot)
        obs_manager.reset()
        if hasattr(robot, "simulationResetPhysics"):
            robot.simulationResetPhysics()
        vehicle.configure(gene_dict)

        # Re-assert velocity mode and standstill after simulationResetPhysics
        for m in vehicle.motors.values():
            if m:
                m.setPosition(float("inf"))
                m.setVelocity(0.0)
        for s in vehicle.servos.values():
            if s:
                s.setPosition(0.0)

        # Step 3 frames to flush physics and settle contact points
        for _ in range(3):
            robot.step(TIME_STEP_MS)

        track_tracker = WebotsTrackProgressTracker(traj_csv)
        track_tracker.reset(start_pos[:2])

        sim_time = 0.0
        crashed = False
        crash_reason = "none"
        stalled = False
        stall_duration_s = 0.0
        true_speeds = []

        total_steps = int(math.ceil(duration_s / DT))

        for _ in range(total_steps):
            if robot.step(TIME_STEP_MS) == -1:
                crashed = True
                crash_reason = "sim_terminated"
                break

            sim_time += DT
            obs_positions = obs_manager.update(sim_time)
            telemetry, _ = vehicle.step(dt=DT, full_telemetry=False)

            gx, gy, _ = vehicle.get_ground_truth_pose()
            v_cur = telemetry.get("v_fused", 0.0)
            true_speeds.append(v_cur)

            track_tracker.update([gx, gy])

            # 1. Check Wall Collisions
            if len(track_left_wall) > 0 and len(track_right_wall) > 0:
                min_dl_sq = np.min((track_left_wall[:, 0] - gx) ** 2 + (track_left_wall[:, 1] - gy) ** 2)
                min_dr_sq = np.min((track_right_wall[:, 0] - gx) ** 2 + (track_right_wall[:, 1] - gy) ** 2)
                closest_wall_m = math.sqrt(min(min_dl_sq, min_dr_sq))
                if closest_wall_m < 0.045:
                    crashed = True
                    crash_reason = "wall_collision"
                    break

            # 2. Check Dynamic Obstacle Collisions
            for ox, oy in obs_positions:
                d_obs = math.hypot(gx - ox, gy - oy)
                if d_obs < 0.12:
                    crashed = True
                    crash_reason = "obstacle_collision"
                    break
            if crashed:
                break

            # 3. Check Off-Track
            if track_tracker.off_track:
                crashed = True
                crash_reason = "off_track"
                break

            # 4. Check Stalling
            if v_cur < 0.15 and sim_time > 1.5:
                stall_duration_s += DT
                if stall_duration_s > 3.0:
                    stalled = True
                    break
            else:
                stall_duration_s = max(0.0, stall_duration_s - DT)

        for m in vehicle.motors.values():
            if m:
                m.setVelocity(0.0)

        total_dist = float(track_tracker.total_progress_m)
        mean_speed = float(np.mean(true_speeds)) if true_speeds else 0.0
        mean_speed = max(0.0, min(5.50, mean_speed))

        if math.isnan(total_dist) or total_dist <= 0.10:
            total_dist = 0.0
            if not crashed:
                crashed = True
                crash_reason = "negligible_distance"

        metrics = {
            "distance_m": round(total_dist, 3),
            "mean_speed_mps": round(mean_speed, 3),
            "laps_completed": int(track_tracker.current_lap),
            "duration_s": round(sim_time, 2),
            "crashed": bool(crashed),
            "crash_reason": crash_reason,
            "stalled": bool(stalled),
        }
        metrics["compute_multiplier"] = round(compute_efficiency_multiplier(gene_dict), 4)
        metrics["fitness"] = round(compute_scalar_fitness(metrics, gene_dict=gene_dict), 2)
        return metrics

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
                chunk = conn.recv(4096).decode("utf-8")
                if not chunk:
                    break
                raw_req += chunk
            except Exception:
                break

        if not raw_req.strip():
            conn.close()
            continue

        try:
            req_data = json.loads(raw_req.strip())
            cmd = req_data.get("cmd", "EVAL")

            if cmd == "PING":
                resp = json.dumps({"status": "PONG"}) + "\n"
                conn.sendall(resp.encode("utf-8"))
            elif cmd == "SHUTDOWN":
                resp = json.dumps({"status": "SHUTDOWN_OK"}) + "\n"
                conn.sendall(resp.encode("utf-8"))
                conn.close()
                break
            elif cmd in ("EVAL", "EVALUATE"):
                genes = req_data.get("genes", {})
                duration_s = float(req_data.get("duration_s", 20.0))
                metrics = evaluate_candidate(genes, duration_s)
                resp = json.dumps({"status": "OK", "metrics": metrics}) + "\n"
                conn.sendall(resp.encode("utf-8"))
            else:
                resp = json.dumps({"status": "ERROR", "error": f"Unknown cmd: {cmd}"}) + "\n"
                conn.sendall(resp.encode("utf-8"))
        except Exception as e:
            err_resp = json.dumps({"status": "ERROR", "error": str(e)}) + "\n"
            try:
                conn.sendall(err_resp.encode("utf-8"))
            except Exception:
                pass
        finally:
            try:
                conn.close()
            except Exception:
                pass

    server_sock.close()


# ==============================================================================
# MODE 4: STATIONARY ROBOT + DYNAMIC OBSTACLE TEST (PHASE 1)
# ==============================================================================
def run_stationary_obstacle_test(robot: any, vehicle: FolkraceVehicle) -> None:
    print("=" * 80, flush=True)
    print("  PHASE 1 TEST: STATIONARY ROBOT + DYNAMIC MOVING OBSTACLE VALIDATION", flush=True)
    print("  Evaluating 16-channel ToF blob extraction, wall rejection, velocity tracking & gating", flush=True)
    print("=" * 80, flush=True)

    # 1. Supervisor handles for obstacle and extra obstacles
    obs_node = robot.getFromDef("MOVING_OBSTACLE") if hasattr(robot, "getFromDef") else None
    obs2_node = robot.getFromDef("MOVING_OBSTACLE_2") if hasattr(robot, "getFromDef") else None
    obs3_node = robot.getFromDef("MOVING_OBSTACLE_3") if hasattr(robot, "getFromDef") else None

    # Park secondary obstacles away from sensor view
    for node in [obs2_node, obs3_node]:
        if node:
            f = node.getField("translation")
            if f:
                f.setSFVec3f([100.0, 100.0, -10.0])

    obs_field = obs_node.getField("translation") if obs_node else None
    if not obs_field:
        print("[Test Warning] MOVING_OBSTACLE not found in world via getFromDef!", flush=True)

    # Telemetry broadcast socket (port 9876)
    gui_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gui_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    # Warm up physics (0.50 s) to settle ground contact
    for _ in range(int(0.50 / (TIME_STEP_MS / 1000.0))):
        if robot.step(TIME_STEP_MS) == -1:
            return
        for m in vehicle.motors.values():
            if m:
                m.setVelocity(0.0)
        for s in vehicle.servos.values():
            if s:
                s.setPosition(0.0)

    # Get baseline stationary pose
    X0, Y0, theta0 = vehicle.get_ground_truth_pose()
    print(f"[Stationary Robot] Locked at X={X0:.3f}m, Y={Y0:.3f}m, Yaw={theta0:.3f}rad", flush=True)

    test_stages = [
        # (name, speed_mps, duration_s, x_start, y_start, vx_r, vy_r)
        ("Stage 1: Stationary Obstacle (0.0 m/s)", 0.00, 1.0, 0.00, 1.20, 0.00, 0.00),
        ("Stage 2: Sub-threshold Slow Obstacle (0.20 m/s)", 0.20, 1.5, 0.00, 1.40, 0.00, -0.20),
        ("Stage 3: Moderate Oncoming Obstacle (0.50 m/s)", 0.50, 1.5, 0.00, 1.50, 0.00, -0.50),
        ("Stage 4: Fast Oncoming Obstacle (0.80 m/s)", 0.80, 1.2, 0.00, 1.60, 0.00, -0.80),
        ("Stage 5: High-Speed Oncoming Obstacle (1.20 m/s)", 1.20, 0.8, 0.00, 1.70, 0.00, -1.20),
        ("Stage 6: Lateral Crossing Obstacle (0.50 m/s)", 0.50, 1.5, -0.45, 0.90, 0.50, 0.00),
    ]

    results_summary = []
    cos_th0 = math.cos(theta0)
    sin_th0 = math.sin(theta0)

    for stage_idx, (st_name, target_speed, duration_s, x_start, y_start, vx_r, vy_r) in enumerate(test_stages):
        print(f"\n---> Starting {st_name} ...", flush=True)
        vehicle.obstacle_tracker.reset()

        dt_step = TIME_STEP_MS / 1000.0
        n_steps = int(round(duration_s / dt_step))
        stage_speeds = []
        dynamic_detected = False

        for step in range(n_steps):
            t_stage = step * dt_step

            # Update moving obstacle position relative to stationary robot
            xr = x_start + vx_r * t_stage
            yr = y_start + vy_r * t_stage
            X_obs = float(X0 + cos_th0 * xr - sin_th0 * yr)
            Y_obs = float(Y0 + sin_th0 * xr + cos_th0 * yr)

            if obs_field:
                obs_field.setSFVec3f([X_obs, Y_obs, 0.075])

            # Lock vehicle stationary
            for m in vehicle.motors.values():
                if m:
                    m.setVelocity(0.0)
            for s in vehicle.servos.values():
                if s:
                    s.setPosition(0.0)

            if robot.step(TIME_STEP_MS) == -1:
                return

            telemetry, planner_ran = vehicle.step(dt=DT)

            # Record track data if planner ran (60 Hz)
            if planner_ran and vehicle.last_dynamic_tracks:
                for tr in vehicle.last_dynamic_tracks:
                    stage_speeds.append(tr.v_mag)
                    if tr.is_dynamic:
                        dynamic_detected = True
            elif planner_ran and vehicle.obstacle_tracker.tracks:
                # If obstacle is static, check its estimated speed
                for tr in vehicle.obstacle_tracker.tracks.values():
                    if abs(tr.x_r - xr) < 0.35 and abs(tr.y_r - yr) < 0.35:
                        if tr.consecutive_associations >= 5:
                            stage_speeds.append(tr.v_mag)

            # Send telemetry packet for live GUI visualization
            if planner_ran:
                telemetry["obstacles"] = [[round(X_obs, 3), round(Y_obs, 3)]]
                telemetry["test_stage"] = st_name
                try:
                    pkt_bytes = json.dumps(to_py(telemetry)).encode("utf-8")
                    gui_sock.sendto(pkt_bytes, ("127.0.0.1", 9876))
                except Exception:
                    pass

        mean_est_speed = float(np.mean(stage_speeds)) if stage_speeds else 0.0
        expected_dyn = (target_speed >= 0.40)
        passed = (dynamic_detected == expected_dyn)

        results_summary.append({
            "stage": st_name,
            "target_speed": target_speed,
            "est_speed": round(mean_est_speed, 3),
            "error_mps": round(abs(mean_est_speed - target_speed), 3),
            "flagged_dynamic": dynamic_detected,
            "expected_dynamic": expected_dyn,
            "status": "PASS" if passed else "FAIL",
        })

        status_str = "PASS" if passed else "FAIL"
        print(f"     Target: {target_speed:.2f} m/s | Est: {mean_est_speed:.3f} m/s | Dynamic: {dynamic_detected} (Expected: {expected_dyn}) -> [{status_str}]", flush=True)

    # Print Final Summary Table
    print("\n" + "=" * 80, flush=True)
    print("  PHASE 1 STATIONARY OBSTACLE TEST RESULTS SUMMARY", flush=True)
    print("=" * 80, flush=True)
    print(f"{'Stage / Scenario':<45} | {'Target':<8} | {'Estimated':<9} | {'Error':<7} | {'Dynamic?':<8} | {'Result'}", flush=True)
    print("-" * 95, flush=True)
    for r in results_summary:
        print(f"{r['stage']:<45} | {r['target_speed']:4.2f} m/s | {r['est_speed']:5.3f} m/s | {r['error_mps']:4.3f}m | {str(r['flagged_dynamic']):<8} | {r['status']}", flush=True)
    print("=" * 80 + "\n", flush=True)

    auto_exit = os.environ.get("WEBOTS_AUTO_QUIT", "1").lower() in ("1", "true", "yes")
    if auto_exit and hasattr(robot, "simulationQuit"):
        print("[Stationary Test] Auto-quitting simulation after test completion...", flush=True)
        robot.simulationQuit(0)


# ==============================================================================
# MODE 5: MOVING ROBOT + DYNAMIC OBSTACLE TEST (PHASE 2 EGO-MOTION COMPENSATION)
# ==============================================================================
def run_moving_obstacle_test(robot: any, vehicle: FolkraceVehicle) -> None:
    print("=" * 80, flush=True)
    print("  PHASE 2 TEST: MOVING ROBOT + EGO-MOTION COMPENSATED OBSTACLE TRACKING", flush=True)
    print("  Evaluating dynamic obstacle tracking with vehicle driving at race speeds", flush=True)
    print("=" * 80, flush=True)

    obs_node = robot.getFromDef("MOVING_OBSTACLE") if hasattr(robot, "getFromDef") else None
    obs2_node = robot.getFromDef("MOVING_OBSTACLE_2") if hasattr(robot, "getFromDef") else None
    obs3_node = robot.getFromDef("MOVING_OBSTACLE_3") if hasattr(robot, "getFromDef") else None

    # Park secondary obstacles away
    for node in [obs2_node, obs3_node]:
        if node:
            f = node.getField("translation")
            if f:
                f.setSFVec3f([100.0, 100.0, -10.0])

    obs_field = obs_node.getField("translation") if obs_node else None
    if not obs_field:
        print("[Test Warning] MOVING_OBSTACLE not found in world via getFromDef!", flush=True)

    gui_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gui_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    # Warm up physics (0.50 s)
    for _ in range(int(0.50 / (TIME_STEP_MS / 1000.0))):
        if robot.step(TIME_STEP_MS) == -1:
            return
        for m in vehicle.motors.values():
            if m:
                m.setVelocity(0.0)
        for s in vehicle.servos.values():
            if s:
                s.setPosition(0.0)

    start_pos = [0.0, 0.0, 0.0275]
    start_rot = [0.0, 0.0, 1.0, -1.570795]

    test_stages = [
        # (name, ego_v, ego_steer, target_obs_v, duration_s, x0_rel, y0_rel, vx_obs_w, vy_obs_w)
        ("Stage 1: Cruising (2.0 m/s) passing Stationary Obstacle", 2.0, 0.0, 0.00, 1.5, 0.00, 1.50, 0.00, 0.00),
        ("Stage 2: Cruising (2.5 m/s) with Oncoming Obstacle (1.0 m/s)", 2.5, 0.0, 1.00, 1.2, 0.00, 1.60, 0.00, -1.00),
        ("Stage 3: Cruising (2.0 m/s) with Leading Obstacle (1.5 m/s)", 2.0, 0.0, 1.50, 1.5, 0.00, 1.20, 0.00, 1.50),
        ("Stage 4: Turning (Steer 0.25 rad) passing Stationary Wall", 1.8, 0.25, 0.00, 1.2, 0.00, 1.40, 0.00, 0.00),
        ("Stage 5: Turning (Steer 0.25 rad) with Lateral Crossing Obstacle (0.6 m/s)", 1.8, 0.25, 0.60, 1.5, -0.40, 1.00, 0.60, 0.00),
    ]

    results_summary = []
    dt_step = TIME_STEP_MS / 1000.0

    def to_py(obj):
        if isinstance(obj, (np.floating, float)):
            return float(obj)
        elif isinstance(obj, (np.integer, int)):
            return int(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {k: to_py(v) for k, v in obj.items()}
        elif isinstance(obj, (list, tuple)):
            return [to_py(v) for v in obj]
        return obj

    for stage_idx, (st_name, ego_v, ego_steer, target_obs_v, duration_s, x0_rel, y0_rel, vx_obs_w, vy_obs_w) in enumerate(test_stages):
        print(f"\n---> Starting {st_name} ...", flush=True)
        vehicle.reset(start_pos=start_pos, start_rot=start_rot)
        vehicle.obstacle_tracker.reset()

        # Settle after reset (0.10 s)
        for _ in range(int(0.10 / dt_step)):
            if robot.step(TIME_STEP_MS) == -1:
                return

        X_ego0, Y_ego0, theta_ego0 = vehicle.get_ground_truth_pose()
        cos_th = math.cos(theta_ego0)
        sin_th = math.sin(theta_ego0)
        X_obs0 = float(X_ego0 + cos_th * x0_rel - sin_th * y0_rel)
        Y_obs0 = float(Y_ego0 + sin_th * x0_rel + cos_th * y0_rel)

        n_steps = int(round(duration_s / dt_step))
        stage_speeds = []
        dynamic_detected = False

        for step in range(n_steps):
            t_stage = step * dt_step

            # Update obstacle world position
            X_obs = X_obs0 + vx_obs_w * t_stage
            Y_obs = Y_obs0 + vy_obs_w * t_stage
            if obs_field:
                obs_field.setSFVec3f([X_obs, Y_obs, 0.075])

            # Apply motor velocity and steering
            wheel_omega = (ego_v / WHEEL_RADIUS) * MOTOR_DIR
            for m in vehicle.motors.values():
                if m:
                    m.setVelocity(wheel_omega)
            for s in vehicle.servos.values():
                if s:
                    s.setPosition(ego_steer)

            # Step vehicle planner pipeline
            telem, was_active = vehicle.step_planner(sim_time=t_stage)

            if was_active and vehicle.last_dynamic_tracks:
                for tr in vehicle.last_dynamic_tracks:
                    if tr.is_dynamic:
                        dynamic_detected = True
                        stage_speeds.append(tr.v_mag)
            elif was_active and vehicle.obstacle_tracker.tracks:
                for tr in vehicle.obstacle_tracker.tracks.values():
                    stage_speeds.append(tr.v_mag)

            if telem:
                try:
                    payload = json.dumps(to_py(telem)).encode("utf-8")
                    gui_sock.sendto(payload, ("127.0.0.1", 9876))
                except Exception:
                    pass

            if robot.step(TIME_STEP_MS) == -1:
                return

        mean_est_speed = float(np.mean(stage_speeds[-10:])) if stage_speeds else 0.0
        expected_dyn = (target_obs_v >= 0.40)
        err = abs(mean_est_speed - target_obs_v)
        passed = (dynamic_detected == expected_dyn) and (err < 0.25 or not expected_dyn)

        results_summary.append({
            "stage": st_name,
            "target_speed": target_obs_v,
            "est_speed": mean_est_speed,
            "error_mps": err,
            "flagged_dynamic": dynamic_detected,
            "status": "PASS" if passed else "FAIL",
        })

        status_str = "PASS" if passed else "FAIL"
        print(f"     Target: {target_obs_v:.2f} m/s | Est: {mean_est_speed:.3f} m/s | Dynamic: {dynamic_detected} (Expected: {expected_dyn}) -> [{status_str}]", flush=True)

    print("\n" + "=" * 80, flush=True)
    print("  PHASE 2 MOVING ROBOT OBSTACLE TEST RESULTS SUMMARY", flush=True)
    print("=" * 80, flush=True)
    print(f"{'Stage / Scenario':<55} | {'Target':<8} | {'Estimated':<9} | {'Error':<7} | {'Dynamic?':<8} | {'Result'}", flush=True)
    print("-" * 105, flush=True)
    for r in results_summary:
        print(f"{r['stage']:<55} | {r['target_speed']:4.2f} m/s | {r['est_speed']:5.3f} m/s | {r['error_mps']:4.3f}m | {str(r['flagged_dynamic']):<8} | {r['status']}", flush=True)
    print("=" * 80 + "\n", flush=True)

    auto_exit = os.environ.get("WEBOTS_AUTO_QUIT", "1").lower() in ("1", "true", "yes")
    if auto_exit and hasattr(robot, "simulationQuit"):
        print("[Moving Test] Auto-quitting simulation after test completion...", flush=True)
        robot.simulationQuit(0)


# ==============================================================================
# MODE 6: PHASE 3 LIVE WEBOTS RENDERED TEST (MOVING ROBOT & DYNAMIC OBSTACLES)
# ==============================================================================
def run_phase3_live_test(robot: any, vehicle: FolkraceVehicle) -> None:
    log_file_path = os.path.join(MY_PROJECT_DIR, "phase3_telemetry.log")
    try:
        with open(log_file_path, "w", encoding="utf-8") as f:
            f.write(f"=== PHASE 3 TELEMETRY LOG STARTED AT {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
    except Exception:
        pass

    def log_line(msg: str):
        print(msg, flush=True)
        try:
            with open(log_file_path, "a", encoding="utf-8") as f:
                f.write(msg + "\n")
        except Exception:
            pass

    log_line("=" * 80)
    log_line("  PHASE 3 LIVE WEBOTS TEST: FULLY RENDERED 3D SIMULATION")
    log_line("  Robot moves back-and-forth along X axis with ego-motion compensation.")
    log_line("  Interactive Controls (Click 3D Webots window to focus):")
    log_line("    [1] : Mode 1 - Obstacle Stationary (Test ego-motion rejection while robot oscillates)")
    log_line("    [2] : Mode 2 - Obstacle Longitudinal Movement (Towards / away at ~0.63 m/s)")
    log_line("    [3] : Mode 3 - Obstacle Lateral Crossing (Left / right across FOV at ~0.68 m/s)")
    log_line("    [4] : Mode 4 - Robot Paused, Obstacle Longitudinal Movement")
    log_line("    [5] : Mode 5 - Robot Paused, Obstacle Lateral Crossing")
    log_line("    [6] : Mode 6 - Both Robot and Obstacle Stationary (Zero baseline)")
    log_line("    [Space] : Toggle Robot back-and-forth movement")
    log_line("    [R] : Reset Robot and Obstacle poses")
    log_line("=" * 80)

    obs_node = robot.getFromDef("MOVING_OBSTACLE") if hasattr(robot, "getFromDef") else None
    obs_field = obs_node.getField("translation") if obs_node else None

    # Enable keyboard
    keyboard = robot.getKeyboard() if hasattr(robot, "getKeyboard") else None
    if keyboard:
        try:
            keyboard.enable(TIME_STEP_MS)
        except Exception:
            pass

    gui_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gui_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        cmd_sock.bind(("127.0.0.1", 9877))
        cmd_sock.setblocking(False)
        log_line("[Phase 3] Live GUI command receiver bound to port 9877")
    except Exception as e:
        log_line(f"[Phase 3] Note: Could not bind command port 9877: {e}")

    start_pos = [0.0, 0.0, 0.0275]
    start_rot = [0.0, 0.0, 1.0, -1.5707953]

    # Warm up physics (0.50 s)
    for _ in range(int(0.50 / (TIME_STEP_MS / 1000.0))):
        if robot.step(TIME_STEP_MS) == -1:
            return
        for m in vehicle.motors.values():
            if m:
                m.setVelocity(0.0)
        for s in vehicle.servos.values():
            if s:
                s.setPosition(0.0)

    # Initial mode & sweep options
    init_mode = int(os.environ.get("WEBOTS_PHASE3_MODE", "1"))
    obs_mode = init_mode if init_mode in (1, 2, 3, 4, 5, 6) else 1
    auto_sweep = os.environ.get("WEBOTS_AUTO_SWEEP", "0").lower() in ("1", "true", "yes")
    sweep_interval = float(os.environ.get("WEBOTS_SWEEP_INTERVAL", "6.0"))
    last_sweep_stage = -1

    # Inject +-5% sensor noise by default in Phase 3
    vehicle.tof_noise_pct = float(os.environ.get("TOF_NOISE_PCT", "0.05"))

    robot_moving_enabled = True
    sim_time = 0.0
    dt_step = TIME_STEP_MS / 1000.0
    last_print_time = 0.0

    mode_names = {
        1: "1-STATIC-EGO-REJECT",
        2: "2-LONGITUDINAL",
        3: "3-LATERAL-CROSS",
        4: "4-DIAGONAL-PATH",
        5: "5-ACCEL-DECEL-SURGE",
        6: "6-S-CURVE-WEAVE",
    }
    mode_targets = {
        1: 0.00,
        2: 0.63,
        3: 0.63,
        4: 0.68,
        5: 0.60,
        6: 0.65,
    }

    # Tracking metrics per mode for automated summary
    stage_metrics = {m: {"speeds": [], "dynamics": [], "leads": []} for m in range(1, 7)}

    def to_py(obj):
        if isinstance(obj, (np.floating, float)):
            return float(obj)
        elif isinstance(obj, (np.integer, int)):
            return int(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {k: to_py(v) for k, v in obj.items()}
        elif isinstance(obj, (list, tuple)):
            return [to_py(v) for v in obj]
        return obj

    def print_summary_table():
        log_line("\n" + "=" * 92)
        log_line("  PHASE 3 DYNAMIC OBSTACLE TEST RESULTS SUMMARY (+/-5% SENSOR NOISE, ROBOT ALWAYS MOVING)")
        log_line("=" * 92)
        log_line(f"{'Mode / Scenario':<26} | {'Target':<8} | {'Estimated':<9} | {'Error':<7} | {'Dynamic?':<9} | {'Lead':<7} | {'Result'}")
        log_line("-" * 92)
        for m in range(1, 7):
            m_info = stage_metrics[m]
            tgt_v = mode_targets[m]
            speeds = m_info["speeds"]
            dyns = m_info["dynamics"]
            leads = m_info["leads"]
            if speeds:
                est_v = float(np.mean(speeds[-20:])) if len(speeds) >= 20 else float(np.mean(speeds))
                any_dyn = any(dyns)
                avg_lead = float(np.mean(leads[-20:])) if leads else 0.0
                err = abs(est_v - tgt_v)
                expected_dyn = (tgt_v >= 0.40)
                overshoot = (est_v > tgt_v + 0.01) if expected_dyn else (max(speeds) >= 0.40)
                passed = (any_dyn == expected_dyn) and (not overshoot) and (err <= 0.35 or not expected_dyn)
                res_str = "PASS" if passed else "FAIL"
                log_line(f"{mode_names[m]:<26} | {tgt_v:4.2f} m/s | {est_v:5.2f} m/s | {err:4.2f} m | {str(any_dyn):<9} | {avg_lead:4.1f}cm | {res_str}")
            else:
                log_line(f"{mode_names[m]:<26} | {tgt_v:4.2f} m/s |   N/A     |   N/A   |   N/A     |   N/A   | UNTESTED")
        log_line("=" * 92 + "\n")

    while robot.step(TIME_STEP_MS) != -1:
        sim_time += dt_step

        # Check automated sweep progression
        if auto_sweep:
            curr_stage = int(sim_time // sweep_interval)
            new_mode = (curr_stage % 6) + 1
            if curr_stage != last_sweep_stage:
                last_sweep_stage = curr_stage
                obs_mode = new_mode
                vehicle.obstacle_tracker.reset()
                log_line(f">>> [Phase 3 Auto-Sweep] Advanced to Mode {obs_mode}: {mode_names[obs_mode]}")

        # Check incoming UDP GUI commands on Port 9877
        try:
            while True:
                cmd_data, _ = cmd_sock.recvfrom(4096)
                cmd_pkt = json.loads(cmd_data.decode("utf-8"))
                c = cmd_pkt.get("cmd", "")
                if c == "set_phase3_mode":
                    m_val = int(cmd_pkt.get("mode", 1))
                    if m_val in (1, 2, 3, 4, 5, 6):
                        auto_sweep = False
                        obs_mode = m_val
                        vehicle.obstacle_tracker.reset()
                        log_line(f">>> [Phase 3 GUI Cmd] Switched to Mode {obs_mode}: {mode_names[obs_mode]}")
                elif c == "toggle_sweep":
                    auto_sweep = not auto_sweep
                    log_line(f">>> [Phase 3 GUI Cmd] Auto Sweep: {'ENABLED' if auto_sweep else 'DISABLED'}")
                elif c == "toggle_robot":
                    robot_moving_enabled = not robot_moving_enabled
                    log_line(f">>> [Phase 3 GUI Cmd] Robot Movement: {'ENABLED' if robot_moving_enabled else 'PAUSED'}")
                elif c == "reset":
                    vehicle.reset(start_pos=start_pos, start_rot=start_rot)
                    vehicle.obstacle_tracker.reset()
                    sim_time = 0.0
                    last_sweep_stage = -1
                    log_line(">>> [Phase 3 GUI Cmd] Reset to Initial Pose")
        except (BlockingIOError, socket.error):
            pass
        except Exception:
            pass

        max_run_seconds = float(os.environ.get("WEBOTS_RUN_SECONDS", "0.0"))
        if max_run_seconds > 0.0 and sim_time >= max_run_seconds:
            print_summary_table()
            log_line(f"[Phase 3] Target duration reached ({sim_time:.1f}s >= {max_run_seconds:.1f}s). Exiting Webots cleanly...")
            if hasattr(robot, "simulationQuit"):
                robot.simulationQuit(0)
            return

        # Read keyboard in Webots 3D view
        if keyboard:
            key = keyboard.getKey()
            while key > 0:
                if key == ord('1'):
                    auto_sweep = False
                    obs_mode = 1
                    vehicle.obstacle_tracker.reset()
                    log_line(">>> [Phase 3] Switched to Mode 1: Obstacle Stationary (Robot Moving)")
                elif key == ord('2'):
                    auto_sweep = False
                    obs_mode = 2
                    vehicle.obstacle_tracker.reset()
                    log_line(">>> [Phase 3] Switched to Mode 2: Obstacle Longitudinal Motion")
                elif key == ord('3'):
                    auto_sweep = False
                    obs_mode = 3
                    vehicle.obstacle_tracker.reset()
                    log_line(">>> [Phase 3] Switched to Mode 3: Obstacle Lateral Crossing")
                elif key == ord('4'):
                    auto_sweep = False
                    obs_mode = 4
                    vehicle.obstacle_tracker.reset()
                    log_line(">>> [Phase 3] Switched to Mode 4: Obstacle Diagonal Path")
                elif key == ord('5'):
                    auto_sweep = False
                    obs_mode = 5
                    vehicle.obstacle_tracker.reset()
                    log_line(">>> [Phase 3] Switched to Mode 5: Obstacle Accel/Decel Surge")
                elif key == ord('6'):
                    auto_sweep = False
                    obs_mode = 6
                    vehicle.obstacle_tracker.reset()
                    log_line(">>> [Phase 3] Switched to Mode 6: Obstacle S-Curve Weave")
                elif key in (ord('A'), ord('a')):
                    auto_sweep = not auto_sweep
                    log_line(f">>> [Phase 3] Auto Sweep: {'ENABLED' if auto_sweep else 'DISABLED'}")
                elif key == ord(' '):
                    robot_moving_enabled = not robot_moving_enabled
                    state_str = "ENABLED" if robot_moving_enabled else "PAUSED"
                    log_line(f">>> [Phase 3] Robot Movement: {state_str}")
                elif key in (ord('R'), ord('r')):
                    vehicle.reset(start_pos=start_pos, start_rot=start_rot)
                    vehicle.obstacle_tracker.reset()
                    sim_time = 0.0
                    last_sweep_stage = -1
                    log_line(">>> [Phase 3] Reset to Initial Pose")
                key = keyboard.getKey()

        # Update obstacle position based on current mode
        # Mode 1: Stationary at X=1.25, Y=0.00
        # Mode 2: Longitudinal motion along X (peak ~0.63 m/s)
        # Mode 3: Lateral crossing along Y (peak ~0.63 m/s)
        # Mode 4: Diagonal path (simultaneous X and Y motion, peak ~0.68 m/s)
        # Mode 5: Accelerating & Decelerating Surge (harmonic pulse, peak ~0.60 m/s)
        # Mode 6: S-Curve / Lissajous Weave in front of vehicle (peak ~0.65 m/s)
        if obs_mode == 1:
            X_obs = 1.25
            Y_obs = 0.00
        elif obs_mode == 2:
            X_obs = 1.25 + 0.22 * math.sin(2.0 * math.pi * sim_time / 2.2)
            Y_obs = 0.00
        elif obs_mode == 3:
            X_obs = 1.25
            Y_obs = 0.22 * math.sin(2.0 * math.pi * sim_time / 2.2)
        elif obs_mode == 4:
            X_obs = 1.25 + 0.17 * math.sin(2.0 * math.pi * sim_time / 2.2)
            Y_obs = 0.17 * math.sin(2.0 * math.pi * sim_time / 2.2)
        elif obs_mode == 5:
            X_obs = 1.25 + 0.18 * math.sin(2.0 * math.pi * sim_time / 2.2) + 0.08 * math.sin(4.0 * math.pi * sim_time / 2.2)
            Y_obs = 0.00
        elif obs_mode == 6:
            X_obs = 1.25 + 0.10 * math.sin(4.0 * math.pi * sim_time / 2.2)
            Y_obs = 0.18 * math.sin(2.0 * math.pi * sim_time / 2.2)
        else:
            X_obs = 1.25
            Y_obs = 0.00

        if obs_field:
            obs_field.setSFVec3f([float(X_obs), float(Y_obs), 0.075])

        # Robot movement: ALWAYS MOVING IN PHASE 3!
        # High-dynamics slalom: peak speed 1.40 m/s with +/-0.10 rad active steering oscillating at 0.9s period
        if robot_moving_enabled:
            target_v = 1.40 * math.sin(2.0 * math.pi * sim_time / 2.0)
            target_steer = 0.10 * math.sin(2.0 * math.pi * sim_time / 0.9)
        else:
            target_v = 0.0
            target_steer = 0.0

        # Step vehicle with speed and steer overrides
        try:
            telem, was_active = vehicle.step(
                dt=dt_step,
                full_telemetry=True,
                override_speed=target_v,
                override_steer=target_steer,
            )
        except Exception as step_err:
            import traceback
            log_line(f"[Phase 3] Step Exception: {step_err}\n{traceback.format_exc()}")
            telem = None
            was_active = False

        # Extract current track metrics
        obs_v_est = 0.0
        is_dyn = False
        lead_cm = 0.0
        if vehicle.obstacle_tracker.tracks:
            for tr in vehicle.obstacle_tracker.tracks.values():
                obs_v_est = tr.v_mag
                is_dyn = tr.is_dynamic
                if is_dyn:
                    lead_cm = min(0.25, tr.v_mag * 0.20) * 100.0
                break

        # Record metrics for summary
        stage_metrics[obs_mode]["speeds"].append(obs_v_est)
        stage_metrics[obs_mode]["dynamics"].append(is_dyn)
        stage_metrics[obs_mode]["leads"].append(lead_cm)

        # Enrich and send live telemetry to PyQt5 GUI
        if telem:
            m_str = mode_names.get(obs_mode, "UNKNOWN")
            telem["mode"] = "PHASE3"
            telem["test_stage"] = f"Mode {obs_mode}: {m_str}"
            telem["obstacles"] = [[round(float(X_obs), 3), round(float(Y_obs), 3)]]
            telem["auto_sweep"] = auto_sweep
            telem["phase3_mode"] = obs_mode
            try:
                payload = json.dumps(to_py(telem)).encode("utf-8")
                gui_sock.sendto(payload, ("127.0.0.1", 9876))
            except Exception:
                pass

        # Print status at 2 Hz
        if sim_time - last_print_time >= 0.50:
            last_print_time = sim_time
            m_str = mode_names.get(obs_mode, "UNKNOWN")
            dyn_str = "DYNAMIC [YES]" if is_dyn else "STATIC  [NO ]"
            v_ego_est = getattr(vehicle, "v_est", 0.0)
            sweep_tag = "[AUTO]" if auto_sweep else "[MANUAL]"
            log_line(f"[Phase 3] {sweep_tag} Mode: {m_str:<15} | Robot v: {v_ego_est:+5.2f}m/s | Obs Speed: {obs_v_est:5.2f}m/s | {dyn_str} | Lead: {lead_cm:4.1f}cm")


# ==============================================================================
# ENTRY POINT
# ==============================================================================
def main():
    if not WEBOTS_AVAILABLE:
        print("[ToF Controller] Error: Webots Python API not found.", flush=True)
        return

    # Use Supervisor if available, falling back to Robot
    try:
        robot = Supervisor()
        is_supervisor = True
    except Exception:
        robot = Robot()
        is_supervisor = False

    vehicle = FolkraceVehicle(robot, is_supervisor=is_supervisor)

    # Determine mode: GA worker, Champion preview, Stationary test, Moving test, Phase 3 live, or Racing studio
    mode = os.environ.get("WEBOTS_CONTROLLER_MODE", "").lower()
    if os.environ.get("GA_WORKER_PORT") or mode in ("ga_eval", "eval", "ga"):
        run_ga_worker(robot, vehicle)
    elif mode in ("phase3", "phase3_live", "live_obs_test", "dynamic_obs_live"):
        run_phase3_live_test(robot, vehicle)
    elif mode in ("stationary_obs_test", "stationary_test", "obs_test"):
        run_stationary_obstacle_test(robot, vehicle)
    elif mode in ("moving_obs_test", "moving_test", "dynamic_test"):
        run_moving_obstacle_test(robot, vehicle)
    elif mode in ("preview", "champion_preview", "replay"):
        run_champion_preview(robot, vehicle)
    else:
        run_racing_studio(robot, vehicle)


if __name__ == "__main__":
    main()

