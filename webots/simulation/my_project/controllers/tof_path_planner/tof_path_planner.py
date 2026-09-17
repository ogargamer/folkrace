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

    # Determine mode: GA worker, Champion preview, or Racing studio
    mode = os.environ.get("WEBOTS_CONTROLLER_MODE", "").lower()
    if os.environ.get("GA_WORKER_PORT") or mode in ("ga_eval", "eval", "ga"):
        run_ga_worker(robot, vehicle)
    elif mode in ("preview", "champion_preview", "replay"):
        run_champion_preview(robot, vehicle)
    else:
        run_racing_studio(robot, vehicle)


if __name__ == "__main__":
    main()
