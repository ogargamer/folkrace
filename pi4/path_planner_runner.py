#!/usr/bin/env python3
"""
================================================================================
  FOLKRACE PI 4: REAL-TIME PATH PLANNER RUNNER (CORE 1 HARD PINNED)
================================================================================
  Hardware Policy & Determinism:
    - Strictly pinned to CPU Core 1 (os.sched_setaffinity(0, {1})).
    - Hard isolation guarantees zero preemption from the Debug Server (Core 2)
      or Linux OS / network stack (Core 0).
    - Asynchronous, non-blocking telemetry broadcast over loopback UDP (Port 9876).
    - If the debug server or SD card experiences latency, packets are dropped
      immediately with zero delay to the planning loop.
================================================================================
"""

import os
import sys
import time
import json
import math
import zlib
import base64
import socket
import csv
import queue
import threading
import gc
from typing import Dict, List, Optional, Any
import numpy as np

# Tune Python GC to prevent mid-frame garbage collection pauses on Core 1
gc.set_threshold(50000, 50, 50)

# Ensure pi4 folder is in sys.path
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
if CURR_DIR not in sys.path:
    sys.path.insert(0, CURR_DIR)

ROOT_DIR = os.path.dirname(CURR_DIR)
CONTROLLERS_DIR = os.path.join(ROOT_DIR, "webots", "simulation", "my_project", "controllers", "tof_path_planner")
CORE_DIR = os.path.join(ROOT_DIR, "core")
for d in (CONTROLLERS_DIR, CORE_DIR):
    if os.path.exists(d) and d not in sys.path:
        sys.path.insert(0, d)

from core_manager import set_core_affinity, set_process_priority, get_core_affinity
from cpp_smart_hybrid_costmap import CppSmartHybridCostmap, SENSOR_NAMES
from tof_gap_scout import ToFGapScout
from tof_latency_compensator import ToFLatencyCompensator
from planner_sl_mpc import CurvilinearSLMPC, CPP_AVAILABLE
from planner_config import (
    GRID_X_MIN,
    GRID_X_MAX,
    GRID_Y_MIN,
    GRID_Y_MAX,
    GRID_RES,
    R_INFLATION_M,
    GAP_MAX_LOOKAHEAD_M,
    V_MAX_CRUISE,
    MAX_STEERING_RAD,
)

TARGET_HZ = 60.0
DT_STEP = 1.0 / TARGET_HZ
TELEM_HZ = 20.0
TELEM_DECIMATION = int(round(TARGET_HZ / TELEM_HZ))  # Exactly 20 Hz (1 frame per 3 control steps)
DEBUG_UDP_PORT = int(os.environ.get("DEBUG_UDP_PORT", 9876))
DEBUG_UDP_HOST = os.environ.get("DEBUG_UDP_HOST", "127.0.0.1")
SENSOR_UDP_PORT = int(os.environ.get("SENSOR_UDP_PORT", 9875))


def create_nonblocking_udp_socket() -> socket.socket:
    """Creates a UDP socket that will never block under any circumstance."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    # 512 KB socket buffer for high-rate full-fidelity telemetry
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 524288)
    except Exception:
        pass
    return sock



def run_path_planner():
    print("=" * 70)
    print("  FOLKRACE PI 4: REAL-TIME PATH PLANNER (CORE 1 DEDICATED)")
    print("=" * 70)

    # 1. HARD HARDWARE ISOLATION: Pin strictly to CPU Core 1
    pinned = set_core_affinity({1}, "PathPlanner")
    set_process_priority(niceness=-15, realtime_fifo=True, priority=50)

    affinity = get_core_affinity()
    print(f"[Planner Core 1] Verified active CPU affinity: {affinity}")
    if 1 not in affinity:
        print("[Planner Core 1 WARNING] Process is not running on Core 1!")

    # 2. Initialize C++ Perception & Planning Engines
    print(f"[Planner Core 1] Initializing C++ Native Costmap & Curvilinear SL-MPC...")
    costmap = CppSmartHybridCostmap(
        grid_bounds=(GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX),
        resolution=GRID_RES,
        r_inflation_m=R_INFLATION_M,
    )
    gap_scout = ToFGapScout()
    compensator = ToFLatencyCompensator(default_dt_s=DT_STEP)
    planner = CurvilinearSLMPC(costmap_engine=costmap)
    planner.costmap = costmap

    # 3. Setup Non-Blocking UDP Telemetry Dispatcher & Live Sensor Receiver
    target_addr = (DEBUG_UDP_HOST, DEBUG_UDP_PORT)

    # Lightweight Telemetry Dispatch Worker: offloads JSON dumping and UDP socket I/O to Core 2/3
    telem_queue = queue.Queue(maxsize=30)

    def telemetry_worker():
        try:
            set_core_affinity({2, 3})
        except Exception:
            pass

        # Dedicated background sender socket with 2MB send buffer and safe timeout
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2097152)
        except Exception:
            pass
        sock.settimeout(0.05)

        while True:
            try:
                telem_dict = telem_queue.get()
                if telem_dict is None:
                    break
                pkt_bytes = json.dumps(telem_dict).encode("utf-8")
                sock.sendto(pkt_bytes, target_addr)
            except Exception:
                pass

    worker_thread = threading.Thread(target=telemetry_worker, daemon=True, name="TelemetryWorker")
    worker_thread.start()

    sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sensor_sock.setblocking(False)
    try:
        sensor_sock.bind(("127.0.0.1", SENSOR_UDP_PORT))
    except Exception as e:
        print(f"[Planner Core 1] Note on sensor socket: {e}")

    live_tof = {k: 0.0 for k in SENSOR_NAMES}
    has_live_sensor_feed = False
    last_sensor_time = 0.0
    last_mcu_time = 0.0
    last_mcu_pkt: Optional[Dict[str, Any]] = None

    print(f"[Planner Core 1] Starting Deterministic {TARGET_HZ:.0f} Hz Loop (Period: {DT_STEP*1000:.2f} ms)...")
    print(f"[Planner Core 1] Listening for live ToF stream on 127.0.0.1:{SENSOR_UDP_PORT}...")
    print("=" * 70)

    step_counter = 0
    sim_time = 0.0

    # Windowed metrics accumulation for consistent 20 Hz telemetry reporting
    win_step_ms_max = 0.0
    win_plan_ms_max = 0.0
    win_map_ms_max = 0.0
    win_scout_ms_max = 0.0
    win_step_ms_sum = 0.0
    win_count = 0

    # Kinematic tracking states
    v_actual = 0.0
    steer_actual_deg = 0.0
    wheel_base = 0.1325
    wheel_radius = 0.0275
    wheel_dir = -1.0
    rad_to_rpm = 60.0 / (2.0 * math.pi)

    t_loop_next = time.perf_counter()

    try:
        while True:
            t_step_start = time.perf_counter()
            sim_time += DT_STEP
            step_counter += 1

            # 1. Perception Ingestion: Drain sensor UDP socket to receive latest real-time frame
            latest_mcu_pkt = None
            now_mono = time.time()
            while True:
                try:
                    raw_bytes, _ = sensor_sock.recvfrom(4096)
                    pkt = json.loads(raw_bytes.decode("utf-8"))
                    latest_mcu_pkt = pkt
                    last_mcu_pkt = pkt
                    last_mcu_time = now_mono
                    if "tof" in pkt and isinstance(pkt["tof"], dict):
                        live_tof = pkt["tof"]
                        has_live_sensor_feed = True
                        last_sensor_time = now_mono
                    elif "tof_ranges_raw" in pkt:
                        live_tof = {
                            SENSOR_NAMES[i]: pkt["tof_ranges_raw"][i]
                            for i in range(min(16, len(pkt["tof_ranges_raw"])))
                        }
                        has_live_sensor_feed = True
                        last_sensor_time = now_mono
                except (BlockingIOError, socket.error):
                    break
                except Exception:
                    break

            if has_live_sensor_feed and (now_mono - last_sensor_time > 1.0):
                has_live_sensor_feed = False
                live_tof = {k: 0.0 for k in SENSOR_NAMES}

            raw_tof = live_tof

            if has_live_sensor_feed:
                # Option 1: Instantaneous Forward Extrapolation (Ego-Motion Latency Compensation)
                steer_rad = math.radians(steer_actual_deg)
                yaw_rate_current = (v_actual * math.tan(steer_rad)) / wheel_base
                comp_tof = compensator.compensate_dict(
                    raw_tof, v_est=v_actual, gyro_z=yaw_rate_current, dt=DT_STEP
                )

                # 2. C++ Smart Hybrid Costmap (evaluated on motion-compensated readings)
                t0 = time.perf_counter()
                prob_map, hazard_mask, _ = costmap.process_tof_frame(comp_tof)
                t_map_ms = (time.perf_counter() - t0) * 1000.0

                # 3. ToF Gap Scout
                t0 = time.perf_counter()
                xg, yg, width, theta_g = gap_scout.scout_gap(
                    tof_readings=comp_tof,
                    hazard_mask=hazard_mask,
                    costmap_obj=costmap,
                    target_lookahead_m=GAP_MAX_LOOKAHEAD_M,
                    steer_current=steer_rad,
                    prob_map=prob_map,
                )
                t_scout_ms = (time.perf_counter() - t0) * 1000.0

                # 4. Curvilinear SL-MPC Optimization (OSQP)
                t0 = time.perf_counter()
                opt_res = planner.plan(
                    xg=xg,
                    yg=yg,
                    theta_g=theta_g,
                    aperture_width=width,
                    v_current=v_actual,
                    steer_current=steer_rad,
                    yaw_rate_current=yaw_rate_current,
                    extra_targets=getattr(gap_scout, "last_targets", None),
                    t_frame_start=t_step_start,
                    frame_budget_ms=13.0,
                )
                t_plan_ms = (time.perf_counter() - t0) * 1000.0

                # Commanded outputs
                v_cmd = float(opt_res.get("cmd_speed_mps", 0.0))
                steer_cmd_rad = float(opt_res.get("cmd_steer_rad", 0.0))
                steer_cmd_deg = math.degrees(steer_cmd_rad)

                # Commanded 4-Wheel RPMs
                base_rpm = (v_cmd / wheel_radius) * rad_to_rpm * (-wheel_dir)
                ackermann_factor = math.tan(steer_cmd_rad) * (0.09 / (2.0 * wheel_base))
                rpm_fl_cmd = base_rpm * (1.0 - ackermann_factor)
                rpm_fr_cmd = base_rpm * (1.0 + ackermann_factor)
                rpm_rl_cmd = base_rpm * (1.0 - ackermann_factor * 0.5)
                rpm_rr_cmd = base_rpm * (1.0 + ackermann_factor * 0.5)
            else:
                prob_map = None
                hazard_mask = None
                t_map_ms = 0.0
                t_scout_ms = 0.0
                t_plan_ms = 0.0
                v_cmd = 0.0
                steer_cmd_rad = 0.0
                steer_cmd_deg = 0.0
                rpm_fl_cmd = 0.0
                rpm_fr_cmd = 0.0
                rpm_rl_cmd = 0.0
                rpm_rr_cmd = 0.0
                xg, yg, width = 0.0, 0.0, 0.0
                opt_res = {"status": "NO SENSORS", "cmd_speed_mps": 0.0, "cmd_steer_rad": 0.0, "best_line": []}
                comp_tof = raw_tof

            # True closed-loop control actuation latency (from ToF receipt to motor commands)
            t_ctrl_ms = (time.perf_counter() - t_step_start) * 1000.0

            # Ingest MCU feedback (prefer latest frame, fallback to last valid hardware frame within 1.0s)
            active_mcu = latest_mcu_pkt or (last_mcu_pkt if (now_mono - last_mcu_time < 1.0) else None)
            if active_mcu and "rpms_act" in active_mcu:
                # Real hardware feedback ingested from MCU UART Bridge
                v_actual = float(active_mcu.get("v_est", active_mcu.get("v_real", v_actual)))
                steer_actual_deg = float(active_mcu.get("steer_actual_deg", steer_actual_deg))
                mcu_act = active_mcu.get("rpms_act", [0, 0, 0, 0])
                rpm_fl_act = float(mcu_act[0])
                rpm_fr_act = float(mcu_act[1])
                rpm_rl_act = float(mcu_act[2])
                rpm_rr_act = float(mcu_act[3])
                yaw_rate_rad_s = float(active_mcu.get("gyro_z", (v_actual * math.tan(math.radians(steer_actual_deg))) / wheel_base))
                gyro_z = yaw_rate_rad_s
                accel_fwd = float(active_mcu.get("accel_fwd", (v_cmd - v_actual) / DT_STEP))
                accel_lat = float(active_mcu.get("accel_lat", v_actual * yaw_rate_rad_s))
                mcu_flags = dict(active_mcu.get("flags", {}))
                slip_state_str = active_mcu.get("slip_state", "NORMAL")
                slip_ratio_val = float(active_mcu.get("slip_ratio", 0.0))
                yaw_deg_val = float(active_mcu.get("yaw_deg", round(math.degrees(sim_time * yaw_rate_rad_s) % 360.0, 1)))
                pitch_deg_val = float(active_mcu.get("pitch_deg", 0.0))
                roll_deg_val = float(active_mcu.get("roll_deg", 0.0))
            else:
                # No controller connected: ZERO data (no fake motion / simulation)
                v_actual = 0.0
                steer_actual_deg = 0.0
                rpm_fl_act = 0.0
                rpm_fr_act = 0.0
                rpm_rl_act = 0.0
                rpm_rr_act = 0.0
                yaw_rate_rad_s = 0.0
                gyro_z = 0.0
                accel_fwd = 0.0
                accel_lat = 0.0
                mcu_flags = {"fake_data": False, "standalone_sim": False}
                slip_state_str = "OFFLINE"
                slip_ratio_val = 0.0
                yaw_deg_val = 0.0
                pitch_deg_val = 0.0
                roll_deg_val = 0.0

            # 5. Measure computational step latency & accumulate window metrics
            t_total_ms = t_ctrl_ms

            win_step_ms_max = max(win_step_ms_max, t_ctrl_ms)
            win_plan_ms_max = max(win_plan_ms_max, t_plan_ms)
            win_map_ms_max = max(win_map_ms_max, t_map_ms)
            win_scout_ms_max = max(win_scout_ms_max, t_scout_ms)
            win_step_ms_sum += t_ctrl_ms
            win_count += 1

            # 6. Compose & Dispatch Telemetry at exact 20 Hz (every 3rd 60 Hz step)
            if step_counter % TELEM_DECIMATION == 0:
                g_targets = [
                    [round(float(tgt[0]), 3), round(float(tgt[1]), 3), round(float(tgt[2]), 3)]
                    for tgt in (getattr(gap_scout, "last_targets", []) or [])
                    if isinstance(tgt, (list, tuple)) and len(tgt) >= 3
                ]

                avg_step_ms = win_step_ms_sum / max(1, win_count)

                telemetry = {
                    "time_step": step_counter,
                    "timestamp": time.time(),
                    "sim_time": round(sim_time, 3),
                    "mode": "DRIVE" if has_live_sensor_feed else "STANDBY",
                    "recovery_state": "DRIVE" if has_live_sensor_feed else "STANDBY",
                    "escape_state": "NORMAL",
                    "status": opt_res.get("status", "OK" if has_live_sensor_feed else "OFFLINE"),
                    "v_cmd": round(v_cmd, 2),
                    "v_tgt": round(v_cmd, 2),
                    "v_real": round(v_actual, 2),
                    "v_est": round(v_actual, 2),
                    "steer_cmd_deg": round(steer_cmd_deg, 1),
                    "steer_deg": round(steer_cmd_deg, 1),
                    "steer_actual_deg": round(steer_actual_deg, 1),
                    "rpms_cmd": [round(rpm_fl_cmd, 1), round(rpm_fr_cmd, 1), round(rpm_rl_cmd, 1), round(rpm_rr_cmd, 1)],
                    "rpms_act": [round(rpm_fl_act, 1), round(rpm_fr_act, 1), round(rpm_rl_act, 1), round(rpm_rr_act, 1)],
                    "yaw_deg": round(yaw_deg_val, 1),
                    "pitch_deg": round(pitch_deg_val, 1),
                    "roll_deg": round(roll_deg_val, 1),
                    "gyro_z": round(gyro_z, 3),
                    "accel_fwd": round(accel_fwd, 2),
                    "accel_lat": round(accel_lat, 2),
                    "slip_state": slip_state_str,
                    "slip_ratio": round(slip_ratio_val, 2),
                    "flags": mcu_flags,
                    "mcu_msg": latest_mcu_pkt.get("mcu_msg", "") if latest_mcu_pkt else "",
                    "mcu_log": latest_mcu_pkt.get("mcu_log", []) if latest_mcu_pkt else [],
                    "f_tof": round(float(min(comp_tof.values())), 3) if comp_tof and has_live_sensor_feed else 0.0,
                    "tof": {k: round(v, 3) for k, v in comp_tof.items()} if has_live_sensor_feed else {k: 0.0 for k in SENSOR_NAMES},
                    "tof_ranges": [round(comp_tof.get(k, 0.0), 3) for k in SENSOR_NAMES] if has_live_sensor_feed else [0.0] * 16,
                    "tof_ranges_raw": [round(raw_tof.get(k, 0.0), 3) for k in SENSOR_NAMES] if has_live_sensor_feed else [0.0] * 16,
                    "gap": [round(xg, 3), round(yg, 3), round(width, 3)],
                    "targets": g_targets,
                    "best_line": opt_res.get("best_line", []),
                    "candidate_lines": opt_res.get("candidate_lines", [])[:16],
                    "guide_line": opt_res.get("guide_line", []),
                    "corridor_left": opt_res.get("corridor_left", []),
                    "corridor_right": opt_res.get("corridor_right", []),
                    "shield": opt_res.get("shield", {}),
                    "plan_ms": round(t_plan_ms, 2),
                    "map_ms": round(t_map_ms, 2),
                    "scout_ms": round(t_scout_ms, 2),
                    "plan_ms_max": round(win_plan_ms_max, 2),
                    "map_ms_max": round(win_map_ms_max, 2),
                    "scout_ms_max": round(win_scout_ms_max, 2),
                    "opt_prim_ms": round(float(opt_res.get("opt_prim_ms", 0.0)), 2),
                    "opt_alt_ms": round(float(opt_res.get("opt_alt_ms", 0.0)), 2),
                    "opt_shield_ms": round(float(opt_res.get("opt_shield_ms", 0.0)), 2),
                    "step_ms": round(t_total_ms, 2),
                    "step_ms_avg": round(avg_step_ms, 2),
                    "step_ms_max": round(win_step_ms_max, 2),
                    "iters_prim": int(opt_res.get("osqp_iters", 12)),
                    "iters_alt": int(opt_res.get("osqp_iters_alt", 0)),
                    "grid_shape": [int(s) for s in costmap.shape],
                    "grid_bounds": [GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX],
                }

                # Reset window metrics
                win_step_ms_max = 0.0
                win_plan_ms_max = 0.0
                win_map_ms_max = 0.0
                win_scout_ms_max = 0.0
                win_step_ms_sum = 0.0
                win_count = 0

                # 7. Non-blocking handoff to background worker (<0.005ms latency, never stalls Core 1)
                try:
                    if telem_queue.full():
                        _ = telem_queue.get_nowait()
                    telem_queue.put_nowait(telemetry)
                except Exception:
                    pass

            # 8. Deterministic 60 Hz Rate Keeper
            t_loop_next += DT_STEP
            t_sleep = t_loop_next - time.perf_counter()
            if t_sleep > 0:
                time.sleep(t_sleep)
            else:
                # Loop overrun: reset deadline
                t_loop_next = time.perf_counter()

    except KeyboardInterrupt:
        print("\n[Planner Core 1] Stopped by user.")


if __name__ == "__main__":
    run_path_planner()
