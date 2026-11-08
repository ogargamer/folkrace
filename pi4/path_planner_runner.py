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
from typing import Dict, List, Optional, Any
import numpy as np

# Ensure pi4 folder is in sys.path
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
if CURR_DIR not in sys.path:
    sys.path.insert(0, CURR_DIR)

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
DEBUG_UDP_PORT = int(os.environ.get("DEBUG_UDP_PORT", 9876))
DEBUG_UDP_HOST = os.environ.get("DEBUG_UDP_HOST", "127.0.0.1")


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


def load_dataset_frames() -> List[Dict[str, float]]:
    """Loads recorded ToF sensor frames from local data directory."""
    candidates = [
        os.path.join(CURR_DIR, "data", "tof_readings_cube_10cm.csv"),
        os.path.join(CURR_DIR, "tof_readings_cube_10cm.csv"),
    ]
    for p in candidates:
        if os.path.exists(p):
            frames = []
            with open(p, "r", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    frame = {k: float(row[k]) for k in SENSOR_NAMES if k in row}
                    if len(frame) == 16:
                        frames.append(frame)
            if frames:
                print(f"[Planner Core 1] Loaded {len(frames)} benchmark ToF frames from {p}")
                return frames
    return []


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

    # 3. Setup Non-Blocking UDP Telemetry Dispatcher
    telem_sock = create_nonblocking_udp_socket()
    target_addr = (DEBUG_UDP_HOST, DEBUG_UDP_PORT)

    # 4. Dataset / Perception Feed
    frames = load_dataset_frames()
    if not frames:
        # Default synthetic frame if no CSV present
        frames = [{k: 1.50 for k in SENSOR_NAMES}]

    print(f"[Planner Core 1] Starting Deterministic {TARGET_HZ:.0f} Hz Loop (Period: {DT_STEP*1000:.2f} ms)...")
    print("=" * 70)

    frame_idx = 0
    total_frames = len(frames)
    step_counter = 0
    sim_time = 0.0

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

            # 1. Perception Ingestion: Get 16-channel ToF frame
            raw_tof = frames[frame_idx]
            frame_idx = (frame_idx + 1) % total_frames

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
            )
            t_plan_ms = (time.perf_counter() - t0) * 1000.0

            # Commanded outputs
            v_cmd = float(opt_res.get("cmd_speed_mps", 3.0))
            steer_cmd_rad = float(opt_res.get("cmd_steer_rad", 0.0))
            steer_cmd_deg = math.degrees(steer_cmd_rad)

            # Simulated physical dynamics response
            v_actual = 0.90 * v_actual + 0.10 * v_cmd
            steer_actual_deg = 0.85 * steer_actual_deg + 0.15 * steer_cmd_deg

            # Commanded 4-Wheel RPMs
            base_rpm = (v_cmd / wheel_radius) * rad_to_rpm * (-wheel_dir)
            ackermann_factor = math.tan(steer_cmd_rad) * (0.09 / (2.0 * wheel_base))
            rpm_fl_cmd = base_rpm * (1.0 - ackermann_factor)
            rpm_fr_cmd = base_rpm * (1.0 + ackermann_factor)
            rpm_rl_cmd = base_rpm * (1.0 - ackermann_factor * 0.5)
            rpm_rr_cmd = base_rpm * (1.0 + ackermann_factor * 0.5)

            # Actual 4-Wheel RPMs (with slight wheel slip dynamics)
            rpm_fl_act = rpm_fl_cmd * 0.98
            rpm_fr_act = rpm_fr_cmd * 0.99
            rpm_rl_act = rpm_rl_cmd * 1.01
            rpm_rr_act = rpm_rr_cmd * 1.00

            # Kinematic IMU states
            yaw_rate_rad_s = (v_actual * math.tan(math.radians(steer_actual_deg))) / wheel_base
            accel_fwd = (v_cmd - v_actual) / DT_STEP
            accel_lat = v_actual * yaw_rate_rad_s

            # 5. Full-fidelity 1cm costmap for laptop telemetry (no max-pooling distortion)
            prob_u8 = (np.clip(prob_map, 0.0, 1.0) * 255.0).astype(np.uint8)
            haz_u8 = (hazard_mask > 0).astype(np.uint8)

            grid_comp = base64.b64encode(zlib.compress(prob_u8.tobytes(), 1)).decode("ascii")
            haz_comp = base64.b64encode(zlib.compress(haz_u8.tobytes(), 1)).decode("ascii")

            g_targets = [
                [round(float(tgt[0]), 3), round(float(tgt[1]), 3), round(float(tgt[2]), 3)]
                for tgt in (getattr(gap_scout, "last_targets", []) or [])
                if isinstance(tgt, (list, tuple)) and len(tgt) >= 3
            ]

            # 6. Compose Full Telemetry Packet
            t_total_ms = (time.perf_counter() - t_step_start) * 1000.0

            telemetry = {
                "time_step": step_counter,
                "timestamp": time.time(),
                "sim_time": round(sim_time, 3),
                "mode": "DRIVE",
                "recovery_state": "DRIVE",
                "escape_state": "NORMAL",
                "status": opt_res.get("status", "OK"),
                "v_cmd": round(v_cmd, 2),
                "v_tgt": round(v_cmd, 2),
                "v_real": round(v_actual, 2),
                "v_est": round(v_actual, 2),
                "steer_cmd_deg": round(steer_cmd_deg, 1),
                "steer_deg": round(steer_cmd_deg, 1),
                "steer_actual_deg": round(steer_actual_deg, 1),
                "rpms_cmd": [round(rpm_fl_cmd, 1), round(rpm_fr_cmd, 1), round(rpm_rl_cmd, 1), round(rpm_rr_cmd, 1)],
                "rpms_act": [round(rpm_fl_act, 1), round(rpm_fr_act, 1), round(rpm_rl_act, 1), round(rpm_rr_act, 1)],
                "yaw_deg": round(math.degrees(sim_time * yaw_rate_rad_s) % 360.0, 1),
                "pitch_deg": 0.0,
                "roll_deg": 0.0,
                "gyro_z": round(yaw_rate_rad_s, 3),
                "accel_fwd": round(accel_fwd, 2),
                "accel_lat": round(accel_lat, 2),
                "slip_state": "NORMAL",
                "slip_ratio": 0.02,
                "f_tof": round(float(min(comp_tof.values())), 3),
                "tof": {k: round(v, 3) for k, v in comp_tof.items()},
                "tof_ranges": [round(comp_tof.get(k, 1.8), 3) for k in SENSOR_NAMES],
                "tof_ranges_raw": [round(raw_tof.get(k, 1.8), 3) for k in SENSOR_NAMES],
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
                "opt_prim_ms": round(float(opt_res.get("opt_prim_ms", 0.0)), 2),
                "opt_alt_ms": round(float(opt_res.get("opt_alt_ms", 0.0)), 2),
                "opt_shield_ms": round(float(opt_res.get("opt_shield_ms", 0.0)), 2),
                "step_ms": round(t_total_ms, 2),
                "iters_prim": int(opt_res.get("osqp_iters", 12)),
                "iters_alt": int(opt_res.get("osqp_iters_alt", 0)),
                "grid_shape": [int(s) for s in prob_u8.shape],
                "grid_bounds": [GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX],
                "grid_data": grid_comp,
                "hazard_data": haz_comp,
            }

            # 7. Non-blocking telemetry dispatch (NEVER pauses or stalls the planner)
            try:
                pkt_bytes = json.dumps(telemetry).encode("utf-8")
                telem_sock.sendto(pkt_bytes, target_addr)
            except (BlockingIOError, socket.error):
                # Socket buffer full: silently drop debug frame, preserve 60 Hz real-time loop!
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
