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
    MOVING_OBSTACLE_DIRECTION,
    MOVING_OBSTACLE_Z_ELEVATION,
    V_MIN_CRUISE,
    V_MAX_CRUISE,
)

STEP_MS_INT = int(round(TIME_STEP_MS))


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


def _json_default(obj):
    """Fast JSON fallback serializer for NumPy scalar/array types without recursive cloning."""
    if isinstance(obj, (np.floating, float)):
        return float(obj)
    elif isinstance(obj, (np.integer, int)):
        return int(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, bytes):
        return obj.decode("ascii", errors="replace")
    elif hasattr(obj, "tolist"):
        return obj.tolist()
    raise TypeError(f"Object of type {type(obj)} is not JSON serializable")


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
        direction=MOVING_OBSTACLE_DIRECTION,
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
            keyboard.enable(STEP_MS_INT)
        except Exception:
            pass

    while robot.step(STEP_MS_INT) != -1:
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
                    mode_name = cmd_pkt.get("mode", "OPTION_1_BREADCRUMB")
                    vehicle.watchdog.set_mode(mode_name)
                    print(f"[Racing Studio] Watchdog mode set to: {mode_name}", flush=True)
                elif c == "reset_robot":
                    vehicle.reset(start_pos=[0.0, 0.0, 0.04])
                    track_tracker.reset([0.0, 0.0])
                    print("[Racing Studio] Robot reset requested by GUI", flush=True)
                elif c == "set_obstacle_speed":
                    spd = float(cmd_pkt.get("val", 1.0))
                    obs_manager.speed_mps = spd
                    obs_engine.speed_mps = spd
                    print(f"[Racing Studio] Moving obstacle speed set to: {obs_manager.speed_mps:.2f} m/s", flush=True)
                elif c == "set_obstacle_direction":
                    d = int(cmd_pkt.get("val", 1))
                    obs_manager.set_direction(d)
                    print(f"[Racing Studio] Moving obstacle direction set to: {'Forward (+)' if obs_manager.direction > 0 else 'Reverse (-)'}", flush=True)
                elif c == "toggle_obstacle_direction":
                    obs_manager.toggle_direction()
                    print(f"[Racing Studio] Moving obstacle direction toggled to: {'Forward (+)' if obs_manager.direction > 0 else 'Reverse (-)'}", flush=True)
                elif c == "reset_obstacles":
                    obs_manager.reset()
                    print("[Racing Studio] Moving obstacles reset to initial spacing", flush=True)
                elif c == "set_param":
                    p_name = cmd_pkt.get("name", "")
                    p_val = cmd_pkt.get("val", None)
                    if p_val is not None:
                        if p_name == "telemetry_hz":
                            telemetry_hz = max(1, min(60, int(p_val)))
                            print(f"[Racing Studio] Telemetry streaming rate set to: {telemetry_hz} Hz", flush=True)
                        elif p_name in ("obstacle_speed", "obstacle_speed_mps"):
                            spd = float(p_val)
                            obs_manager.speed_mps = spd
                            obs_engine.speed_mps = spd
                            print(f"[Racing Studio] Moving obstacle speed set to: {obs_manager.speed_mps:.2f} m/s", flush=True)
                        elif p_name in ("obstacle_direction", "obs_direction"):
                            d = int(p_val)
                            obs_manager.set_direction(d)
                            print(f"[Racing Studio] Moving obstacle direction set to: {'Forward (+)' if obs_manager.direction > 0 else 'Reverse (-)'}", flush=True)
                        elif p_name == "r_inflation_m":
                            vehicle.costmap.r_inflation_m = float(p_val)
                            vehicle.costmap._handle = vehicle.costmap._lib.costmap_create(
                                vehicle.costmap.x_min, vehicle.costmap.x_max, vehicle.costmap.y_min, vehicle.costmap.y_max, vehicle.costmap.res, vehicle.costmap.r_inflation_m
                            )
                        elif p_name == "target_speed":
                            val_f = float(p_val)
                            vehicle.planner.v_cruise_max = val_f
                        elif p_name == "alpha_peak":
                            vehicle.planner.alpha_peak = float(p_val)
                        elif p_name == "alt_switch_hysteresis" and hasattr(vehicle.planner, "alt_switch_hysteresis"):
                            vehicle.planner.alt_switch_hysteresis = float(p_val)
                        elif p_name in ("w_gap_clear", "alt_lookahead_depth_weight"):
                            val_f = float(p_val)
                            if hasattr(vehicle.planner, "w_gap_clear"):
                                vehicle.planner.w_gap_clear = val_f
                            if hasattr(vehicle.planner, "alt_lookahead_depth_weight"):
                                vehicle.planner.alt_lookahead_depth_weight = val_f
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
                telemetry["obstacle_speed"] = round(float(obs_manager.speed_mps), 2)
                telemetry["obstacle_direction"] = int(obs_manager.direction)
                telemetry["active_planner"] = "SL_MPC"
                telemetry["sim_time"] = round(sim_time, 2)
                telemetry["telemetry_hz"] = telemetry_hz
                telemetry["front_pillar"] = bool(front_pillar_active)
                telemetry["back_pillar"] = bool(back_pillar_active)
                telemetry["sim_speed"] = round(float(sim_speed_mult), 2)
                try:
                    pkt_bytes = json.dumps(telemetry, default=_json_default).encode("utf-8")
                    gui_sock.sendto(pkt_bytes, ("127.0.0.1", 9876))
                except Exception:
                    pass

                # HUD Overlay in Webots 3D World Viewport
                if hasattr(robot, "setLabel"):
                    v_real = telemetry.get("v_real", 0.0)
                    v_cmd = telemetry.get("v_cmd", 0.0)
                    v_plan = telemetry.get("v_plan", telemetry.get("v_tgt", v_cmd))
                    steer_cmd = telemetry.get("steer_cmd_deg", telemetry.get("steer_deg", 0.0))
                    steer_plan = telemetry.get("steer_plan_deg", steer_cmd)
                    steer_act = telemetry.get("steer_actual_deg", steer_cmd)
                    mode_str = telemetry.get("mode", "DRIVE")
                    plan_ms = telemetry.get("plan_ms", 0.0)
                    map_ms = telemetry.get("map_ms", 0.0)
                    scout_ms = telemetry.get("scout_ms", 0.0)
                    opt_prim_ms = telemetry.get("opt_prim_ms", 0.0)
                    opt_alt_ms = telemetry.get("opt_alt_ms", 0.0)
                    opt_shield_ms = telemetry.get("opt_shield_ms", 0.0)
                    iters_prim = telemetry.get("iters_prim", telemetry.get("osqp_iters", 0))
                    iters_alt = telemetry.get("iters_alt", telemetry.get("osqp_iters_alt", 0))
                    step_ms = telemetry.get("step_ms", 0.0)
                    step_cpu_ms = telemetry.get("step_cpu_ms", 0.0)
                    lap_num = getattr(track_tracker, "current_lap", 0)

                    color_mode = 0x00FF88 if not any(x in mode_str for x in ("PIVOT", "DECEL", "SPIN")) else 0xFF3333
                    f_tof_val = telemetry.get("f_tof", 0.0)
                    robot.setLabel(0, f"SL-MPC {telemetry_hz}Hz | Mode: {mode_str:16s} | t: {sim_time:5.1f} | Plan: {plan_ms:4.1f}ms", 0.01, 0.01, 0.06, 0x00E5FF, 0.0, "Arial")
                    robot.setLabel(1, f"v: {v_real:+4.2f} (cmd: {v_cmd:+4.2f}, plan: {v_plan:+4.2f}) | Steer: {steer_act:+5.1f} (cmd: {steer_cmd:+5.1f}, plan: {steer_plan:+5.1f})", 0.01, 0.04, 0.06, color_mode, 0.0, "Arial")
                    robot.setLabel(2, f"Track: {track_dist:5.1f}m | f_tof: {f_tof_val:4.2f} | map:{map_ms:.1f} p:{opt_prim_ms:.1f}({iters_prim}) a:{opt_alt_ms:.1f}({iters_alt}) step:{step_ms:.1f}ms cpu:{step_cpu_ms:.1f}ms", 0.01, 0.07, 0.05, 0xFFFFFF, 0.0, "Arial")

                # Terminal Telemetry Output (throttled to telemetry_hz)
                v_real_val = telemetry.get("v_real", 0.0)
                v_cmd_val = telemetry.get("v_cmd", 0.0)
                v_plan_val = telemetry.get("v_plan", telemetry.get("v_tgt", v_cmd_val))
                steer_cmd_val = telemetry.get("steer_cmd_deg", telemetry.get("steer_deg", 0.0))
                steer_plan_val = telemetry.get("steer_plan_deg", steer_cmd_val)
                steer_act_val = telemetry.get("steer_actual_deg", steer_cmd_val)
                f_tof_val = telemetry.get("f_tof", 0.0)
                plan_ms = telemetry.get("plan_ms", 0.0)
                map_ms = telemetry.get("map_ms", 0.0)
                scout_ms = telemetry.get("scout_ms", 0.0)
                opt_prim_ms = telemetry.get("opt_prim_ms", 0.0)
                opt_alt_ms = telemetry.get("opt_alt_ms", 0.0)
                opt_shield_ms = telemetry.get("opt_shield_ms", 0.0)
                iters_prim = telemetry.get("iters_prim", telemetry.get("osqp_iters", 0))
                iters_alt = telemetry.get("iters_alt", telemetry.get("osqp_iters_alt", 0))
                obs_ms = telemetry.get("obs_ms", 0.0)
                telem_ms = telemetry.get("telem_ms", 0.0)
                step_ms = telemetry.get("step_ms", 0.0)
                step_cpu_ms = telemetry.get("step_cpu_ms", 0.0)
                gc_ms = telemetry.get("gc_ms", 0.0)
                gc_str = f" gc:{gc_ms:3.1f}ms" if gc_ms > 0.0 else ""
                mp = telemetry.get("map_perf", ())
                map_detail = f"[{mp[3]:.1f}/{mp[4]:.1f}]" if (len(mp) >= 5 and map_ms > 4.0) else ""
                alt_str = f" alt:{opt_alt_ms:3.1f} it:{iters_prim}/{iters_alt}" if opt_alt_ms > 0.0 else f" it:{iters_prim}"
                obs_str = f" obs:{obs_ms:3.1f}" if obs_ms > 0.0 else ""
                perf_breakdown = (
                    f"Plan: {plan_ms:4.1f}ms (map:{map_ms:3.1f}{map_detail} sc:{scout_ms:3.1f} "
                    f"prim:{opt_prim_ms:3.1f}{alt_str} "
                    f"sh:{opt_shield_ms:3.1f}{obs_str} tlm:{telem_ms:3.1f} "
                    f"step:{step_ms:4.1f}ms cpu:{step_cpu_ms:4.1f}ms{gc_str})"
                )
                print(
                    f"Mode: {telemetry['mode']:20s} | t: {sim_time:5.2f} | f_tof: {f_tof_val:4.2f} | "
                    f"v: {v_real_val:+5.2f} (cmd: {v_cmd_val:+5.2f}, plan: {v_plan_val:+5.2f}) | "
                    f"Steer: {steer_act_val:+5.1f} (cmd: {steer_cmd_val:+5.1f}, plan: {steer_plan_val:+5.1f}) | "
                    f"{perf_breakdown}",
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

    # 1. Load champion configuration directly from best_planner_genome.json
    best_path = os.path.join(GA_OPTIMIZER_DIR, "best_planner_genome.json")
    ckpt_path = os.path.join(GA_OPTIMIZER_DIR, "ga_checkpoint.json")
    genes: Dict[str, Any] = {}
    title = "Default SL-MPC"

    for candidate_path in [best_path, ckpt_path]:
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
        direction=MOVING_OBSTACLE_DIRECTION,
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
            keyboard.enable(STEP_MS_INT)
        except Exception:
            pass

    sim_time = 0.0
    sim_paused = False
    sim_step_requested = False
    sim_speed_mult = 1.0
    sim_time = 0.0
    telemetry_hz = max(1, min(60, int(os.environ.get("WEBOTS_TELEMETRY_HZ", 60))))
    last_telemetry_time = -1.0

    while robot.step(STEP_MS_INT) != -1:
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
                            robot.step(STEP_MS_INT)
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
                elif c == "set_obstacle_speed":
                    spd = float(cmd_pkt.get("val", 1.0))
                    obs_manager.speed_mps = spd
                    obs_engine.speed_mps = spd
                    print(f"[Champion Preview] Moving obstacle speed set to: {obs_manager.speed_mps:.2f} m/s", flush=True)
                elif c == "set_obstacle_direction":
                    d = int(cmd_pkt.get("val", 1))
                    obs_manager.set_direction(d)
                    print(f"[Champion Preview] Moving obstacle direction set to: {'Forward (+)' if obs_manager.direction > 0 else 'Reverse (-)'}", flush=True)
                elif c == "toggle_obstacle_direction":
                    obs_manager.toggle_direction()
                    print(f"[Champion Preview] Moving obstacle direction toggled to: {'Forward (+)' if obs_manager.direction > 0 else 'Reverse (-)'}", flush=True)
                elif c == "reset_obstacles":
                    obs_manager.reset()
                    print("[Champion Preview] Moving obstacles reset to initial spacing", flush=True)
                elif c == "set_param":
                    p_name = cmd_pkt.get("name", "")
                    p_val = cmd_pkt.get("val", None)
                    if p_val is not None:
                        if p_name == "telemetry_hz":
                            telemetry_hz = max(1, min(60, int(p_val)))
                            print(f"[Champion Preview] Telemetry streaming rate set to: {telemetry_hz} Hz", flush=True)
                        elif p_name in ("obstacle_speed", "obstacle_speed_mps"):
                            spd = float(p_val)
                            obs_manager.speed_mps = spd
                            obs_engine.speed_mps = spd
                            print(f"[Champion Preview] Moving obstacle speed set to: {obs_manager.speed_mps:.2f} m/s", flush=True)
                        elif p_name in ("obstacle_direction", "obs_direction"):
                            d = int(p_val)
                            obs_manager.set_direction(d)
                            print(f"[Champion Preview] Moving obstacle direction set to: {'Forward (+)' if obs_manager.direction > 0 else 'Reverse (-)'}", flush=True)
                        elif p_name == "r_inflation_m":
                            vehicle.costmap.r_inflation_m = float(p_val)
                            vehicle.costmap._handle = vehicle.costmap._lib.costmap_create(
                                vehicle.costmap.x_min, vehicle.costmap.x_max, vehicle.costmap.y_min, vehicle.costmap.y_max, vehicle.costmap.res, vehicle.costmap.r_inflation_m
                            )
                        elif p_name == "target_speed":
                            val_f = float(p_val)
                            vehicle.planner.v_cruise_max = val_f
                        elif p_name == "alpha_peak":
                            vehicle.planner.alpha_peak = float(p_val)
                        elif p_name == "alt_switch_hysteresis" and hasattr(vehicle.planner, "alt_switch_hysteresis"):
                            vehicle.planner.alt_switch_hysteresis = float(p_val)
                        elif p_name in ("w_gap_clear", "alt_lookahead_depth_weight"):
                            val_f = float(p_val)
                            if hasattr(vehicle.planner, "w_gap_clear"):
                                vehicle.planner.w_gap_clear = val_f
                            if hasattr(vehicle.planner, "alt_lookahead_depth_weight"):
                                vehicle.planner.alt_lookahead_depth_weight = val_f
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
                telemetry["obstacle_speed"] = round(float(obs_manager.speed_mps), 2)
                telemetry["obstacle_direction"] = int(obs_manager.direction)
                telemetry["active_planner"] = "SL_MPC"
                telemetry["sim_time"] = round(sim_time, 2)
                telemetry["telemetry_hz"] = telemetry_hz
                telemetry["sim_speed"] = round(float(sim_speed_mult), 2)
                telemetry["metrics"] = {
                    "track_dist_m": round(track_dist, 2),
                    "laps": int(track_tracker.current_lap),
                    "speed_mps": round(telemetry.get("v_fused", 0.0), 2),
                    "title": title,
                }
                try:
                    pkt_bytes = json.dumps(telemetry, default=_json_default).encode("utf-8")
                    gui_sock.sendto(pkt_bytes, ("127.0.0.1", 9876))
                except Exception:
                    pass

                # HUD Overlay in Webots 3D World Viewport
                if hasattr(robot, "setLabel"):
                    v_real = telemetry.get("v_real", 0.0)
                    v_cmd = telemetry.get("v_cmd", 0.0)
                    v_plan = telemetry.get("v_plan", telemetry.get("v_tgt", v_cmd))
                    steer_cmd = telemetry.get("steer_cmd_deg", telemetry.get("steer_deg", 0.0))
                    steer_plan = telemetry.get("steer_plan_deg", steer_cmd)
                    steer_act = telemetry.get("steer_actual_deg", steer_cmd)
                    mode_str = telemetry.get("mode", "DRIVE")
                    plan_ms = telemetry.get("plan_ms", 0.0)
                    map_ms = telemetry.get("map_ms", 0.0)
                    scout_ms = telemetry.get("scout_ms", 0.0)
                    opt_prim_ms = telemetry.get("opt_prim_ms", 0.0)
                    opt_alt_ms = telemetry.get("opt_alt_ms", 0.0)
                    opt_shield_ms = telemetry.get("opt_shield_ms", 0.0)
                    iters_prim = telemetry.get("iters_prim", telemetry.get("osqp_iters", 0))
                    iters_alt = telemetry.get("iters_alt", telemetry.get("osqp_iters_alt", 0))
                    step_ms = telemetry.get("step_ms", 0.0)
                    step_cpu_ms = telemetry.get("step_cpu_ms", 0.0)
                    lap_num = getattr(track_tracker, "current_lap", 0)

                    color_mode = 0x00FF88 if not any(x in mode_str for x in ("PIVOT", "DECEL", "SPIN")) else 0xFF3333
                    f_tof_val = telemetry.get("f_tof", 0.0)
                    robot.setLabel(0, f"CHAMPION PREVIEW {telemetry_hz}Hz | Mode: {mode_str:16s} | t: {sim_time:5.1f} | Plan: {plan_ms:4.1f}ms", 0.01, 0.01, 0.06, 0x00E5FF, 0.0, "Arial")
                    robot.setLabel(1, f"v: {v_real:+4.2f} (cmd: {v_cmd:+4.2f}, plan: {v_plan:+4.2f}) | Steer: {steer_act:+5.1f} (cmd: {steer_cmd:+5.1f}, plan: {steer_plan:+5.1f})", 0.01, 0.04, 0.06, color_mode, 0.0, "Arial")
                    robot.setLabel(2, f"Track: {track_dist:5.1f}m | map:{map_ms:.1f} p:{opt_prim_ms:.1f}({iters_prim}) a:{opt_alt_ms:.1f}({iters_alt}) step:{step_ms:.1f}ms cpu:{step_cpu_ms:.1f}ms | {title[:25]}", 0.01, 0.07, 0.05, 0xFFFFFF, 0.0, "Arial")

                # Terminal Telemetry Output (throttled to telemetry_hz)
                v_real_val = telemetry.get("v_real", 0.0)
                v_cmd_val = telemetry.get("v_cmd", 0.0)
                v_plan_val = telemetry.get("v_plan", telemetry.get("v_tgt", v_cmd_val))
                steer_cmd_val = telemetry.get("steer_cmd_deg", telemetry.get("steer_deg", 0.0))
                steer_plan_val = telemetry.get("steer_plan_deg", steer_cmd_val)
                steer_act_val = telemetry.get("steer_actual_deg", steer_cmd_val)
                f_tof_val = telemetry.get("f_tof", 0.0)
                plan_ms = telemetry.get("plan_ms", 0.0)
                map_ms = telemetry.get("map_ms", 0.0)
                scout_ms = telemetry.get("scout_ms", 0.0)
                opt_prim_ms = telemetry.get("opt_prim_ms", 0.0)
                opt_alt_ms = telemetry.get("opt_alt_ms", 0.0)
                opt_shield_ms = telemetry.get("opt_shield_ms", 0.0)
                iters_prim = telemetry.get("iters_prim", telemetry.get("osqp_iters", 0))
                iters_alt = telemetry.get("iters_alt", telemetry.get("osqp_iters_alt", 0))
                obs_ms = telemetry.get("obs_ms", 0.0)
                telem_ms = telemetry.get("telem_ms", 0.0)
                step_ms = telemetry.get("step_ms", 0.0)
                step_cpu_ms = telemetry.get("step_cpu_ms", 0.0)
                gc_ms = telemetry.get("gc_ms", 0.0)
                gc_str = f" gc:{gc_ms:3.1f}ms" if gc_ms > 0.0 else ""
                mp = telemetry.get("map_perf", ())
                map_detail = f"[{mp[3]:.1f}/{mp[4]:.1f}]" if (len(mp) >= 5 and map_ms > 4.0) else ""
                alt_str = f" alt:{opt_alt_ms:3.1f} it:{iters_prim}/{iters_alt}" if opt_alt_ms > 0.0 else f" it:{iters_prim}"
                obs_str = f" obs:{obs_ms:3.1f}" if obs_ms > 0.0 else ""
                perf_breakdown = (
                    f"Plan: {plan_ms:4.1f}ms (map:{map_ms:3.1f}{map_detail} sc:{scout_ms:3.1f} "
                    f"prim:{opt_prim_ms:3.1f}{alt_str} "
                    f"sh:{opt_shield_ms:3.1f}{obs_str} tlm:{telem_ms:3.1f} "
                    f"step:{step_ms:4.1f}ms cpu:{step_cpu_ms:4.1f}ms{gc_str})"
                )
                print(
                    f"Mode: {telemetry['mode']:20s} | t: {sim_time:5.2f} | f_tof: {f_tof_val:4.2f} | "
                    f"v: {v_real_val:+5.2f} (cmd: {v_cmd_val:+5.2f}, plan: {v_plan_val:+5.2f}) | "
                    f"Steer: {steer_act_val:+5.1f} (cmd: {steer_cmd_val:+5.1f}, plan: {steer_plan_val:+5.1f}) | "
                    f"{perf_breakdown}",
                    flush=True,
                )

        # Simulation Speed Throttling (only active when slow-motion < 1.0x is explicitly requested)
        if sim_speed_mult < 0.99:
            target_step_dt = (TIME_STEP_MS / 1000.0) / max(0.01, sim_speed_mult)
            elapsed_wall = time.perf_counter() - wall_start_t
            if elapsed_wall < target_step_dt:
                time.sleep(target_step_dt - elapsed_wall)

# ==============================================================================
# MODE 2B: FAST CHAMPION PURE DRIVE (MAX REAL-TIME PERFORMANCE)
# ==============================================================================
def run_fast_champion_drive(robot: any, vehicle: FolkraceVehicle) -> None:
    """
    Ultra-lean driving loop: full 3D simulation visualization with zero GUI/UDP/HUD overhead.
    Evaluates at maximum native speed while preserving physics and C++ SL-MPC planning.
    """
    print("=" * 80, flush=True)
    print("  FOLKRACE FAST CHAMPION PURE DRIVE (MAX PERFORMANCE)", flush=True)
    print("=" * 80, flush=True)

    # 1. Load champion configuration directly from best_planner_genome.json
    from planner_config import BEST_GENOME_SOURCE_PATH
    best_path = os.path.join(GA_OPTIMIZER_DIR, "best_planner_genome.json")
    ckpt_path = os.path.join(GA_OPTIMIZER_DIR, "ga_checkpoint.json")
    genes: Dict[str, Any] = {}
    title = "Champion SL-MPC"
    loaded_path: Optional[str] = None

    for candidate_path in [best_path, ckpt_path]:
        if os.path.exists(candidate_path):
            try:
                with open(candidate_path, "r", encoding="utf-8") as f:
                    cfg = json.load(f)
                genes = cfg.get("genes") or cfg.get("best_genome") or cfg
                title = cfg.get("title", cfg.get("meta", {}).get("title", os.path.basename(candidate_path)))
                loaded_path = candidate_path
                print(f"[Fast Drive] Loaded champion genome from {os.path.basename(candidate_path)}: {title}", flush=True)
                break
            except Exception as e:
                print(f"[Fast Drive] Warning: Could not parse {candidate_path}: {e}", flush=True)

    already_configured = (
        loaded_path is not None
        and BEST_GENOME_SOURCE_PATH is not None
        and os.path.abspath(loaded_path) == os.path.abspath(BEST_GENOME_SOURCE_PATH)
    )
    if genes and not already_configured:
        vehicle.configure(genes)

    traj_csv = find_trajectory_csv()
    try:
        obs_dir = int(os.environ.get("FOLKRACE_OBS_DIRECTION", MOVING_OBSTACLE_DIRECTION))
    except ValueError:
        obs_dir = MOVING_OBSTACLE_DIRECTION
    obs_engine = ReverseObstacleEngine(traj_csv, speed_mps=MOVING_OBSTACLE_SPEED_MPS)
    obs_manager = MovingObstacleManager(
        robot=robot,
        engine=obs_engine,
        obstacle_names=MOVING_OBSTACLE_NAMES,
        elevation_z=MOVING_OBSTACLE_Z_ELEVATION,
        speed_mps=MOVING_OBSTACLE_SPEED_MPS,
        direction=obs_dir,
    )

    from ga_planner_config import ROBOT_START_POS, ROBOT_START_ROT
    start_pos = list(ROBOT_START_POS)
    start_rot = list(ROBOT_START_ROT)
    vehicle.reset(start_pos=start_pos, start_rot=start_rot)
    obs_manager.reset()

    sim_time = 0.0
    last_print_time = 0.0

    print("[Fast Drive] Simulation started. Real-time driving active.", flush=True)

    while robot.step(STEP_MS_INT) != -1:
        sim_time += DT
        if not obs_manager.is_physical_robots:
            obs_manager.update(sim_time)
        telemetry, planner_ran = vehicle.step(dt=DT, full_telemetry=False)

        # Throttle console telemetry to 1 Hz so stdout pipe never stalls physics
        if planner_ran and (sim_time - last_print_time >= 1.0):
            last_print_time = sim_time
            f_tof_val = telemetry.get("f_tof", 0.0)
            v_real_val = vehicle.get_ground_truth_velocity()
            v_cmd_val = telemetry.get("v_cmd", 0.0)
            v_plan_val = telemetry.get("v_plan", telemetry.get("v_tgt", v_cmd_val))
            steer_cmd_val = telemetry.get("steer_cmd_deg", telemetry.get("steer_deg", 0.0))
            steer_plan_val = telemetry.get("steer_plan_deg", steer_cmd_val)
            steer_act_val = telemetry.get("steer_actual_deg", steer_cmd_val)
            plan_ms = telemetry.get("plan_ms", 0.0)
            map_ms = telemetry.get("map_ms", 0.0)
            scout_ms = telemetry.get("scout_ms", 0.0)
            opt_prim_ms = telemetry.get("opt_prim_ms", 0.0)
            opt_alt_ms = telemetry.get("opt_alt_ms", 0.0)
            opt_shield_ms = telemetry.get("opt_shield_ms", 0.0)
            iters_prim = telemetry.get("iters_prim", telemetry.get("osqp_iters", 0))
            iters_alt = telemetry.get("iters_alt", telemetry.get("osqp_iters_alt", 0))
            obs_ms = telemetry.get("obs_ms", 0.0)
            telem_ms = telemetry.get("telem_ms", 0.0)
            step_ms = telemetry.get("step_ms", 0.0)
            step_cpu_ms = telemetry.get("step_cpu_ms", 0.0)
            gc_ms = telemetry.get("gc_ms", 0.0)
            gc_str = f" gc:{gc_ms:3.1f}ms" if gc_ms > 0.0 else ""
            mp = telemetry.get("map_perf", ())
            map_detail = f"[{mp[3]:.1f}/{mp[4]:.1f}]" if (len(mp) >= 5 and map_ms > 4.0) else ""
            alt_str = f" alt:{opt_alt_ms:3.1f} it:{iters_prim}/{iters_alt}" if opt_alt_ms > 0.0 else f" it:{iters_prim}"
            obs_str = f" obs:{obs_ms:3.1f}" if obs_ms > 0.0 else ""
            perf_breakdown = (
                f"Plan: {plan_ms:4.1f}ms (map:{map_ms:3.1f}{map_detail} sc:{scout_ms:3.1f} "
                f"prim:{opt_prim_ms:3.1f}{alt_str} "
                f"sh:{opt_shield_ms:3.1f}{obs_str} tlm:{telem_ms:3.1f} "
                f"step:{step_ms:4.1f}ms cpu:{step_cpu_ms:4.1f}ms{gc_str})"
            )
            print(
                f"Mode: {telemetry['mode']:20s} | t: {sim_time:5.2f} | f_tof: {f_tof_val:4.2f} | "
                f"v: {v_real_val:+5.2f} (cmd: {v_cmd_val:+5.2f}, plan: {v_plan_val:+5.2f}) | "
                f"Steer: {steer_act_val:+5.1f} (cmd: {steer_cmd_val:+5.1f}, plan: {steer_plan_val:+5.1f}) | "
                f"{perf_breakdown}",
                flush=True,
            )



# ==============================================================================
# MODE 3B: AUTOMATED WRONG-WAY & RECOVERY EVALUATION (HEADLESS WEBOTS)
# ==============================================================================
def run_wrong_way_eval(robot: any, vehicle: FolkraceVehicle) -> None:
    """
    Runs automated wrong-way detection, discrimination, and recovery scenarios
    in real 3D Webots physics and writes a JSON report to AWW_EVAL_OUT.
    """
    world_tag = os.environ.get("AWW_EVAL_WORLD", "test_wrong_way").lower()
    req_scenarios = os.environ.get("AWW_EVAL_SCENARIOS", "all")
    out_path = os.environ.get("AWW_EVAL_OUT", "")
    params_raw = os.environ.get("AWW_EVAL_PARAMS_JSON", "").strip()
    eval_params: Dict[str, Any] = {}
    if params_raw:
        try:
            eval_params = json.loads(params_raw)
        except Exception as e:
            print(f"[WrongWayEval] Warning: Failed to parse AWW_EVAL_PARAMS_JSON: {e}", flush=True)

    def _apply_eval_params():
        if not eval_params:
            return
        wd = vehicle.watchdog
        bc = wd.breadcrumb_watchdog
        rm = vehicle.recovery_manager
        if "aww_duration_scale" in eval_params:
            wd.set_duration_scale(float(eval_params["aww_duration_scale"]))
        if "aww_max_detect_dist_m" in eval_params:
            bc.max_detect_dist_m = float(eval_params["aww_max_detect_dist_m"])
        if "aww_max_lat_xv_m" in eval_params:
            bc.max_lat_xv_m = float(eval_params["aww_max_lat_xv_m"])
        if "aww_min_confirmed" in eval_params:
            bc.min_confirmed = max(1, int(eval_params["aww_min_confirmed"]))
        if "aww_confirm_time_s" in eval_params:
            wd.confirm_time_s = float(eval_params["aww_confirm_time_s"])
        if "aww_post_cooldown_s" in eval_params:
            wd.post_recovery_cooldown_duration_s = float(eval_params["aww_post_cooldown_s"])
        if "rec_180_speed_mps" in eval_params:
            rm.corrective_forward_speed = float(eval_params["rec_180_speed_mps"])
        if "rec_180_rev_clear_m" in eval_params:
            rm.corrective_rev_clear_m = float(eval_params["rec_180_rev_clear_m"])
        if "rec_180_rev_min_s" in eval_params:
            rm.corrective_rev_min_s = float(eval_params["rec_180_rev_min_s"])
        if "rec_180_align_deg" in eval_params:
            rm.corrective_align_rad = math.radians(float(eval_params["rec_180_align_deg"]))
        if "rec_180_early_exit_turn_deg" in eval_params:
            rm.corrective_early_exit_turn_rad = math.radians(float(eval_params["rec_180_early_exit_turn_deg"]))

    traj_csv = find_trajectory_csv()
    is_track_racing = ("track_racing" in world_tag)

    obs_manager = None
    track_tracker = None
    if is_track_racing:
        obs_engine = ReverseObstacleEngine(traj_csv, speed_mps=MOVING_OBSTACLE_SPEED_MPS)
        obs_manager = MovingObstacleManager(
            robot=robot,
            engine=obs_engine,
            obstacle_names=MOVING_OBSTACLE_NAMES,
            elevation_z=MOVING_OBSTACLE_Z_ELEVATION,
            speed_mps=MOVING_OBSTACLE_SPEED_MPS,
            direction=MOVING_OBSTACLE_DIRECTION,
        )
        track_tracker = WebotsTrackProgressTracker(traj_csv)

    start_pos = [0.0, 0.0, 0.0275]
    start_rot = [0.0, 0.0, 1.0, -1.5707953071795862]

    front_pillar_node = robot.getFromDef("FRONT_PILLAR") if (vehicle.is_supervisor and hasattr(robot, "getFromDef")) else None
    front_pillar_trans = front_pillar_node.getField("translation") if front_pillar_node else None
    back_pillar_node = robot.getFromDef("BACK_PILLAR") if (vehicle.is_supervisor and hasattr(robot, "getFromDef")) else None
    back_pillar_trans = back_pillar_node.getField("translation") if back_pillar_node else None

    def _set_pillars(raised: bool):
        if front_pillar_trans:
            front_pillar_trans.setSFVec3f([2.60, 0.40, 0.06 if raised else -5.0])
        if back_pillar_trans:
            back_pillar_trans.setSFVec3f([-0.2066, 0.39, 0.06 if raised else -5.0])

    def _wrap_pi(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    def _reset_world_robot(obs_speed_mps: Optional[float] = None):
        if obs_manager is not None:
            eff_obs_speed = float(obs_speed_mps) if obs_speed_mps is not None else float(MOVING_OBSTACLE_SPEED_MPS)
            obs_engine.speed_mps = eff_obs_speed
            obs_manager.speed_mps = eff_obs_speed
        vehicle.reset(start_pos=start_pos, start_rot=start_rot)
        _apply_eval_params()
        if obs_manager is not None:
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
        for _ in range(10):
            robot.step(STEP_MS_INT)
        vehicle.fusion.reset()
        vehicle.ediff.reset()
        vehicle.pose_tracker.reset(start_pos[0], start_pos[1], 0.0)
        vehicle.v_est = 0.0
        if track_tracker is not None:
            track_tracker.reset(start_pos[:2])

    def _inject_perturbation(delta_yaw_rad: float, fwd_shove_m: float = 0.0, lat_shift_m: float = 0.0):
        gx, gy, gyaw = vehicle.get_ground_truth_pose()
        # Robot forward in Webots world: (-sin(gyaw), cos(gyaw)); Right: (cos(gyaw), sin(gyaw))
        nx = gx + (-math.sin(gyaw)) * fwd_shove_m + math.cos(gyaw) * lat_shift_m
        ny = gy + math.cos(gyaw) * fwd_shove_m + math.sin(gyaw) * lat_shift_m
        nyaw = _wrap_pi(gyaw + delta_yaw_rad)
        if vehicle._trans_field:
            vehicle._trans_field.setSFVec3f([float(nx), float(ny), 0.0275])
        if vehicle._rot_field:
            vehicle._rot_field.setSFRotation([0.0, 0.0, 1.0, float(nyaw)])
        if vehicle.robot_node:
            try:
                vehicle.robot_node.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            except Exception:
                pass
        for wn in vehicle.wheel_nodes:
            try:
                wn.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            except Exception:
                pass
        for m in vehicle.motors.values():
            if m:
                m.setVelocity(0.0)
        # Step 1 physics tick so IMU/sensors refresh at new pose without clearing breadcrumbs
        robot.step(STEP_MS_INT)
        vehicle.fusion.v_est = 0.0
        vehicle.fusion.v_wheels = 0.0
        vehicle.fusion.v_front = 0.0
        vehicle.fusion.v_rear = 0.0
        vehicle.v_est = 0.0
        return gyaw

    def _run_spin_scenario(
        name: str,
        drive_s: float,
        delta_yaw_rad: float,
        fwd_shove_m: float = 0.0,
        lat_shift_m: float = 0.0,
        max_rec_s: float = 4.5,
        raise_pillars: bool = False,
        min_post_dist_m: float = 0.40,
        align_tol_deg: float = 45.0,
    ) -> Dict[str, Any]:
        _set_pillars(raise_pillars)
        _reset_world_robot()
        sim_t = 0.0
        flipped = False
        pre_ww = 0

        steps_drive = int(round(drive_s / DT))
        for _ in range(steps_drive):
            if robot.step(STEP_MS_INT) == -1:
                break
            sim_t += DT
            if obs_manager is not None:
                obs_manager.update(sim_t)
            telem, _ = vehicle.step(dt=DT, full_telemetry=False)
            if telem.get("is_wrong_way"):
                pre_ww += 1
            if abs(vehicle.fusion.roll_deg) > 70.0 or abs(vehicle.fusion.pitch_deg) > 70.0:
                flipped = True

        pre_yaw = _inject_perturbation(delta_yaw_rad, fwd_shove_m=fwd_shove_m, lat_shift_m=lat_shift_m)
        pgx, pgy, pgyaw = vehicle.get_ground_truth_pose()
        n_bc = len(vehicle.watchdog.breadcrumb_watchdog.breadcrumbs)
        print(
            f"[WrongWayEval:{name}] Injected at gt=({pgx:.2f},{pgy:.2f},{math.degrees(pgyaw):.1f}deg) "
            f"pt=({vehicle.pose_tracker.x:.2f},{vehicle.pose_tracker.y:.2f},{math.degrees(vehicle.pose_tracker.yaw):.1f}deg) "
            f"pre_yaw={math.degrees(pre_yaw):.1f}deg bcs={n_bc}",
            flush=True,
        )
        t_spin = sim_t
        detect_lat = None
        rec_done_lat = None
        was_in_180 = False
        aligned_at_exit = False
        retriggers = 0
        post_rec_dist = 0.0
        prev_pos = vehicle.get_ground_truth_pose()[:2]

        steps_rec = int(round(max_rec_s / DT))
        for step_idx in range(steps_rec):
            if robot.step(STEP_MS_INT) == -1:
                break
            sim_t += DT
            if obs_manager is not None:
                obs_manager.update(sim_t)
            telem, planner_ran = vehicle.step(dt=DT, full_telemetry=False)
            gx, gy, gyaw = vehicle.get_ground_truth_pose()
            if abs(vehicle.fusion.roll_deg) > 70.0 or abs(vehicle.fusion.pitch_deg) > 70.0:
                flipped = True

            is_ww = bool(vehicle.last_aww_status and vehicle.last_aww_status.is_wrong_way)
            rec_state = str(vehicle.recovery_manager.state)
            in_180 = ("CORRECTIVE_180" in rec_state)

            if planner_ran and step_idx in (2, 12, 30, 45, 60, 90, 120, 150, 180):
                rays = vehicle.last_aww_status.sightline_rays if vehicle.last_aww_status else []
                n_clr_opp = sum(1 for r in rays if r.get("clear") and r.get("opposing"))
                n_opp = sum(1 for r in rays if r.get("opposing"))
                sub_s = getattr(vehicle.recovery_manager, "corrective_substate", "")
                print(
                    f"  [t+{sim_t-t_spin:.2f}s] gt=({gx:.2f},{gy:.2f},{math.degrees(gyaw):.1f}deg) pt_yaw={math.degrees(vehicle.pose_tracker.yaw):.1f}deg "
                    f"v={vehicle.v_est:.2f} vr={vehicle.get_ground_truth_velocity():.2f} f={vehicle.cached_f_tof:.2f} rec={rec_state}({sub_s}) dir={vehicle.recovery_manager.corrective_steer_dir:+.0f} "
                    f"cmd=({vehicle.recovery_manager.current_cmd_v:.2f},{vehicle.recovery_manager.current_cmd_steer:+.2f}) ww={is_ww} rays={len(rays)} opp={n_opp} clr_opp={n_clr_opp}",
                    flush=True,
                )

            if is_ww and detect_lat is None:
                detect_lat = sim_t - t_spin
            if rec_done_lat is None and was_in_180 and not in_180:
                rec_done_lat = sim_t - t_spin
                aligned_at_exit = abs(_wrap_pi(gyaw - pre_yaw)) <= math.radians(align_tol_deg)
            elif rec_done_lat is not None:
                if is_ww:
                    retriggers += 1
                post_rec_dist += math.hypot(gx - prev_pos[0], gy - prev_pos[1])
            prev_pos = (gx, gy)
            was_in_180 = in_180

        _, _, final_yaw = vehicle.get_ground_truth_pose()
        v_final = float(vehicle.v_est)
        passed = (
            (not flipped)
            and pre_ww == 0
            and detect_lat is not None and detect_lat <= 0.60
            and rec_done_lat is not None and rec_done_lat <= 3.6
            and aligned_at_exit
            and retriggers == 0
            and (post_rec_dist >= min_post_dist_m or v_final >= 0.50)
        )
        return {
            "name": name,
            "passed": bool(passed),
            "pre_ww": int(pre_ww),
            "detect_s": round(detect_lat, 3) if detect_lat is not None else None,
            "rec_done_s": round(rec_done_lat, 3) if rec_done_lat is not None else None,
            "aligned_at_exit": bool(aligned_at_exit),
            "retriggers": int(retriggers),
            "post_rec_dist_m": round(post_rec_dist, 2),
            "v_final": round(v_final, 2),
            "final_yaw_err_deg": round(math.degrees(_wrap_pi(final_yaw - pre_yaw)), 1),
            "flipped": bool(flipped),
        }

    def _run_clean_drive_scenario(
        name: str,
        duration_s: float,
        lower_pillars: bool = False,
        obs_speed_mps: Optional[float] = None,
        min_progress_m: float = 8.0,
        allow_off_track: bool = False,
    ) -> Dict[str, Any]:
        _set_pillars(not lower_pillars)
        _reset_world_robot(obs_speed_mps=obs_speed_mps)
        sim_t = 0.0
        ww_triggers = 0
        ww_fp = 0
        ww_tp = 0
        flipped = False
        dist = 0.0
        prev_pos = vehicle.get_ground_truth_pose()[:2]

        steps = int(round(duration_s / DT))
        for _ in range(steps):
            if robot.step(STEP_MS_INT) == -1:
                break
            sim_t += DT
            if obs_manager is not None:
                obs_manager.update(sim_t)
            telem, _ = vehicle.step(dt=DT, full_telemetry=False)
            gx, gy, gyaw_chk = vehicle.get_ground_truth_pose()
            dist += math.hypot(gx - prev_pos[0], gy - prev_pos[1])
            prev_pos = (gx, gy)
            if track_tracker is not None:
                track_tracker.update([gx, gy])
            if telem.get("is_wrong_way"):
                ww_triggers += 1
                is_true_pos = False
                if track_tracker is not None and track_tracker.n_pts >= 2:
                    i0 = track_tracker.last_idx
                    i1 = (i0 + 1) % track_tracker.n_pts
                    seg_dx = float(track_tracker.traj_pts[i1, 0] - track_tracker.traj_pts[i0, 0])
                    seg_dy = float(track_tracker.traj_pts[i1, 1] - track_tracker.traj_pts[i0, 1])
                    track_fwd_yaw = math.atan2(-seg_dx, seg_dy)
                    if abs(_wrap_pi(gyaw_chk - track_fwd_yaw)) >= math.radians(90.0):
                        is_true_pos = True
                if is_true_pos:
                    ww_tp += 1
                else:
                    ww_fp += 1
                if ww_triggers <= 3:
                    rays_w4 = vehicle.last_aww_status.sightline_rays if vehicle.last_aww_status else []
                    clr_opp = [r for r in rays_w4 if r.get("clear") and r.get("opposing")]
                    print(
                        f"  [W4 WW #{ww_triggers} ({'TP' if is_true_pos else 'FP'}) t={sim_t:.2f}s] gt=({gx:.2f},{gy:.2f},{math.degrees(gyaw_chk):.1f}deg) "
                        f"pt=({vehicle.pose_tracker.x:.2f},{vehicle.pose_tracker.y:.2f},{math.degrees(vehicle.pose_tracker.yaw):.1f}deg) "
                        f"clr_opp={len(clr_opp)} sample={clr_opp[0] if clr_opp else None}",
                        flush=True,
                    )
            if abs(vehicle.fusion.roll_deg) > 70.0 or abs(vehicle.fusion.pitch_deg) > 70.0:
                flipped = True

        prog = float(track_tracker.total_progress_m) if track_tracker is not None else dist
        off_track = bool(track_tracker.off_track) if track_tracker is not None else False
        passed = (ww_fp == 0) and (not flipped) and (allow_off_track or not off_track) and (prog >= min_progress_m)
        return {
            "name": name,
            "passed": bool(passed),
            "ww_fp": int(ww_fp),
            "ww_tp": int(ww_tp),
            "ww_triggers": int(ww_triggers),
            "progress_m": round(prog, 2),
            "off_track": bool(off_track),
            "flipped": bool(flipped),
        }

    if is_track_racing:
        suite = {
            "T1_clean_fwd_opponents_25s": lambda: _run_clean_drive_scenario("T1_clean_fwd_opponents_25s", 25.0, obs_speed_mps=1.0, min_progress_m=45.0),
            "T2_clean_rev_opponents_25s": lambda: _run_clean_drive_scenario("T2_clean_rev_opponents_25s", 25.0, obs_speed_mps=-1.0, min_progress_m=15.0, allow_off_track=True),
            "T3_midrace_straight_180": lambda: _run_spin_scenario("T3_midrace_straight_180", 3.5, math.pi, min_post_dist_m=0.50),
            "T4_midrace_corner_160": lambda: _run_spin_scenario("T4_midrace_corner_160", 6.5, math.radians(160.0), min_post_dist_m=0.50, align_tol_deg=60.0),
            "T5_midrace_shoved_wall_180": lambda: _run_spin_scenario("T5_midrace_shoved_wall_180", 3.5, math.pi, fwd_shove_m=0.55, lat_shift_m=0.10, min_post_dist_m=0.50),
        }
    else:
        suite = {
            "W1_straight_180": lambda: _run_spin_scenario("W1_straight_180", 1.25, math.pi),
            "W2_wall_hug_right_170": lambda: _run_spin_scenario("W2_wall_hug_right_170", 1.25, math.radians(170.0), lat_shift_m=0.22),
            "W3_wall_hug_left_170": lambda: _run_spin_scenario("W3_wall_hug_left_170", 1.25, math.radians(-170.0), lat_shift_m=-0.22),
            "W4_shoved_fwd_180": lambda: _run_spin_scenario("W4_shoved_fwd_180", 1.25, math.pi, fwd_shove_m=0.60),
            "W5_diagonal_135_spin": lambda: _run_spin_scenario("W5_diagonal_135_spin", 1.25, math.radians(135.0)),
            "W6_pillar_occluded_spin": lambda: _run_spin_scenario("W6_pillar_occluded_spin", 1.25, math.pi, max_rec_s=3.5, raise_pillars=True, min_post_dist_m=0.40),
            "W7_hairpin_clean_endurance": lambda: _run_clean_drive_scenario("W7_hairpin_clean_endurance", 15.0, lower_pillars=True, min_progress_m=18.0),
        }

    selected = list(suite.keys()) if req_scenarios == "all" else [s.strip() for s in req_scenarios.split(",") if s.strip() in suite]
    results = []
    for s_name in selected:
        print(f"[WrongWayEval] Running {s_name}...", flush=True)
        res = suite[s_name]()
        results.append(res)
        print(f"[WrongWayEval] {'PASS' if res['passed'] else 'FAIL'} {s_name}: {res}", flush=True)

    payload = {"world": world_tag, "results": results, "all_passed": all(r["passed"] for r in results)}
    if out_path:
        os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)

    if hasattr(robot, "simulationQuit"):
        robot.simulationQuit(0 if payload["all_passed"] else 1)


# ==============================================================================
# MODE 3: GA OPTIMIZER HEADLESS EVALUATION WORKER
# ==============================================================================
def run_ga_worker(robot: any, vehicle: FolkraceVehicle) -> None:
    from ga_planner_config import (
        BASE_WORKER_PORT,
        ROBOT_START_POS,
        ROBOT_START_ROT,
        TRACK1_MOVING_OBSTACLE_SPEED_MPS,
        TRACK2_MOVING_OBSTACLE_SPEED_MPS,
        USE_TRACK2,
        compute_scalar_fitness,
        compute_efficiency_multiplier,
    )

    worker_port = int(os.environ.get("GA_WORKER_PORT", BASE_WORKER_PORT))
    print(f"[GA Worker] Initializing on TCP port {worker_port}...", flush=True)

    traj_csv = find_trajectory_csv()
    obs_engine = ReverseObstacleEngine(traj_csv, speed_mps=TRACK1_MOVING_OBSTACLE_SPEED_MPS)
    obs_manager = MovingObstacleManager(
        robot=robot,
        engine=obs_engine,
        obstacle_names=MOVING_OBSTACLE_NAMES,
        elevation_z=MOVING_OBSTACLE_Z_ELEVATION,
        speed_mps=TRACK1_MOVING_OBSTACLE_SPEED_MPS,
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

    def _run_single_trial(gene_dict: Dict[str, Any], duration_s: float, obstacle_speed_mps: float) -> Dict[str, Any]:
        # Reset vehicle and obstacles with specific speed
        obs_engine.speed_mps = float(obstacle_speed_mps)
        obs_manager.speed_mps = float(obstacle_speed_mps)

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
            robot.step(STEP_MS_INT)

        track_tracker = WebotsTrackProgressTracker(traj_csv)
        track_tracker.reset(start_pos[:2])

        sim_time = 0.0
        crashed = False
        crash_reason = "none"
        stalled = False
        stall_duration_s = 0.0
        opponent_contacts = 0
        true_speeds = []

        total_steps = int(math.ceil(duration_s / DT))

        for _ in range(total_steps):
            if robot.step(STEP_MS_INT) == -1:
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

            # 2. Dynamic Opponent Contacts / Legacy Cylinder Collisions
            for ox, oy in obs_positions:
                d_obs = math.hypot(gx - ox, gy - oy)
                if d_obs < 0.18:
                    opponent_contacts += 1
                if not obs_manager.is_physical_robots and d_obs < 0.12:
                    crashed = True
                    crash_reason = "obstacle_collision"
                    break
            if crashed:
                break

            # 2b. Check Flip / Rollover
            if abs(vehicle.fusion.roll_deg) > 70.0 or abs(vehicle.fusion.pitch_deg) > 70.0:
                crashed = True
                crash_reason = "flipped"
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
        obs_manager.set_paused(True)

        total_dist = float(track_tracker.total_progress_m)
        mean_speed = float(np.mean(true_speeds)) if true_speeds else 0.0
        mean_speed = max(0.0, min(V_MAX_CRUISE, mean_speed))

        if math.isnan(total_dist) or total_dist <= 0.10:
            total_dist = 0.0
            if not crashed:
                crashed = True
                crash_reason = "negligible_distance"

        trial_metrics = {
            "distance_m": round(total_dist, 3),
            "mean_speed_mps": round(mean_speed, 3),
            "laps_completed": int(track_tracker.current_lap),
            "duration_s": round(sim_time, 2),
            "eval_duration_s": round(duration_s, 2),
            "crashed": bool(crashed),
            "crash_reason": crash_reason,
            "stalled": bool(stalled),
            "opponent_contacts": int(opponent_contacts),
            "obstacle_speed_mps": float(obstacle_speed_mps),
        }
        trial_metrics["compute_multiplier"] = round(compute_efficiency_multiplier(gene_dict), 4)
        trial_metrics["fitness"] = round(compute_scalar_fitness(trial_metrics, gene_dict=gene_dict, eval_duration_s=duration_s), 2)
        return trial_metrics

    def evaluate_candidate(gene_dict: Dict[str, Any], duration_s: float, use_track2: bool = USE_TRACK2) -> Dict[str, Any]:
        # Track 1: Primary track with moving obstacles at 1.00 m/s
        m1 = _run_single_trial(gene_dict, duration_s, obstacle_speed_mps=TRACK1_MOVING_OBSTACLE_SPEED_MPS)
        fit1 = float(m1.get("fitness", -5000.0))

        if not use_track2:
            m1_out = dict(m1)
            m1_out["track1_metrics"] = dict(m1)
            m1_out["track2_metrics"] = None
            return m1_out

        # Track 2: Secondary track with moving obstacles the other way at 1.00 m/s (reverse)
        m2 = _run_single_trial(gene_dict, duration_s, obstacle_speed_mps=TRACK2_MOVING_OBSTACLE_SPEED_MPS)
        fit2 = float(m2.get("fitness", -5000.0))

        # Fitness is lowest of those 2 so candidate must do well on both
        composite_fitness = min(fit1, fit2)

        # Composite metrics reporting conservative performance across both tracks
        crashed = m1.get("crashed", False) or m2.get("crashed", False)
        stalled = m1.get("stalled", False) or m2.get("stalled", False)
        crash_reason = m1.get("crash_reason") if m1.get("crashed", False) else (m2.get("crash_reason") if m2.get("crashed", False) else "none")

        composite_metrics = {
            "distance_m": round(min(float(m1.get("distance_m", 0.0)), float(m2.get("distance_m", 0.0))), 3),
            "mean_speed_mps": round(min(float(m1.get("mean_speed_mps", 0.0)), float(m2.get("mean_speed_mps", 0.0))), 3),
            "laps_completed": int(min(int(m1.get("laps_completed", 0)), int(m2.get("laps_completed", 0)))),
            "duration_s": round(float(m1.get("duration_s", 0.0)) + float(m2.get("duration_s", 0.0)), 2),
            "eval_duration_s": round(duration_s, 2),
            "crashed": bool(crashed),
            "crash_reason": crash_reason,
            "stalled": bool(stalled),
            "compute_multiplier": round(min(float(m1.get("compute_multiplier", 1.0)), float(m2.get("compute_multiplier", 1.0))), 4),
            "fitness": round(composite_fitness, 2),
            "track1_metrics": m1,
            "track2_metrics": m2,
        }
        return composite_metrics

    while True:
        conn = None
        try:
            conn, _ = server_sock.accept()
        except (BlockingIOError, socket.error):
            if robot.step(STEP_MS_INT) == -1:
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
                use_track2 = bool(req_data.get("use_track2", USE_TRACK2))
                metrics = evaluate_candidate(genes, duration_s, use_track2=use_track2)
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
# MODE 4: PHYSICAL OPPONENT ROBOT DRIVER (FROZEN RIVAL PROFILES)
# ==============================================================================
OPPONENT_PROFILES: Dict[str, Dict[str, Any]] = {
    "opponent_fast_light": {
        "planner_decimation": 2,  # 60 Hz
        "genes": {
            "v_cruise_max": 2.65,
            "v_cruise_min": 1.05,
            "mu_friction": 0.78,
            "center_bias": 0.15,
        },
    },
    "opponent_reactive_light": {
        "planner_decimation": 4,  # 30 Hz
        "genes": {
            "v_cruise_max": 2.15,
            "v_cruise_min": 0.95,
            "mu_friction": 0.72,
            "r_steer_rate": 8.0,
            "center_bias": 0.05,
        },
    },
    "opponent_heavy_tank": {
        "planner_decimation": 4,  # 30 Hz
        "genes": {
            "v_cruise_max": 1.25,
            "v_cruise_min": 0.65,
            "mu_friction": 0.80,
            "v_corner_factor": 0.72,
            "center_bias": 0.40,
        },
    },
}


def run_opponent_driver(robot: any, role: str) -> None:
    """Runs a frozen, lightweight autonomous controller for a physical opponent robot."""
    profile = OPPONENT_PROFILES.get(role, OPPONENT_PROFILES["opponent_reactive_light"])
    base_genes: Dict[str, Any] = {}
    best_path = os.path.join(GA_OPTIMIZER_DIR, "best_planner_genome.json")
    if os.path.exists(best_path):
        try:
            with open(best_path, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            base_genes = dict(cfg.get("genes") or cfg.get("best_genome") or {})
        except Exception:
            base_genes = {}

    frozen_genes = {**base_genes, **profile["genes"]}
    base_v_max = float(frozen_genes.get("v_cruise_max", 2.0))
    decim = int(profile.get("planner_decimation", 2))

    vehicle = FolkraceVehicle(
        robot,
        is_supervisor=False,
        genes=frozen_genes,
        planner_decimation=decim,
    )
    print(
        f"[Opponent:{role}] Active (decim={decim}, v_max={base_v_max:.2f} m/s, "
        f"mu={frozen_genes.get('mu_friction', 0.8):.2f})",
        flush=True,
    )

    last_custom = ""
    last_trial = -1
    paused = False

    while robot.step(STEP_MS_INT) != -1:
        raw_custom = ""
        if hasattr(robot, "getCustomData"):
            try:
                raw_custom = robot.getCustomData() or ""
            except Exception:
                raw_custom = ""

        if raw_custom and raw_custom != last_custom:
            last_custom = raw_custom
            try:
                msg = json.loads(raw_custom)
                paused = bool(msg.get("paused", False))
                trial_id = int(msg.get("trial", -1))
                is_rev = bool(msg.get("reverse", False))
                speed_scale = max(0.2, min(1.5, float(msg.get("speed_scale", 1.0))))
                if trial_id != last_trial:
                    last_trial = trial_id
                    vehicle.reset()
                    for m in vehicle.motors.values():
                        if m:
                            m.setPosition(float("inf"))
                            m.setVelocity(0.0)
                    for s in vehicle.servos.values():
                        if s:
                            s.setPosition(0.0)
                    scaled_v = max(0.60, min(V_MAX_CRUISE, base_v_max * speed_scale))
                    vehicle.v_cruise_max = scaled_v
                    vehicle.brain.v_cruise_max = scaled_v
                    vehicle.planner.v_cruise_max = scaled_v
                    vehicle.watchdog.set_mode("DISABLED" if is_rev else "OPTION_1_BREADCRUMB")
            except Exception:
                pass

        if paused:
            for m in vehicle.motors.values():
                if m:
                    m.setVelocity(0.0)
            continue

        vehicle.step(dt=DT, full_telemetry=False)


# ==============================================================================
# ENTRY POINT
# ==============================================================================
def main():
    if not WEBOTS_AVAILABLE:
        print("[ToF Controller] Error: Webots Python API not found.", flush=True)
        return

    # Check controllerArgs BEFORE env vars: Webots passes worker env vars to all 4 robots!
    opp_role = None
    for arg in sys.argv[1:]:
        if arg in OPPONENT_PROFILES or arg.startswith("opponent_"):
            opp_role = arg
            break
    if opp_role is not None:
        robot = Robot()
        run_opponent_driver(robot, opp_role)
        return

    # Use Supervisor if available, falling back to Robot
    try:
        robot = Supervisor()
        is_supervisor = True
    except Exception:
        robot = Robot()
        is_supervisor = False

    vehicle = FolkraceVehicle(robot, is_supervisor=is_supervisor)

    # Determine mode: GA worker, Wrong-Way Eval, Champion preview, Fast champion drive, or Racing studio
    mode = os.environ.get("WEBOTS_CONTROLLER_MODE", "").lower()
    if os.environ.get("GA_WORKER_PORT") or mode in ("ga_eval", "eval", "ga"):
        run_ga_worker(robot, vehicle)
    elif mode in ("wrong_way_eval", "aww_eval"):
        run_wrong_way_eval(robot, vehicle)
    elif mode in ("fast_drive", "pure_drive", "realtime_drive", "fast_preview"):
        run_fast_champion_drive(robot, vehicle)
    elif mode in ("preview", "champion_preview", "replay"):
        run_champion_preview(robot, vehicle)
    else:
        run_racing_studio(robot, vehicle)


if __name__ == "__main__":
    main()

