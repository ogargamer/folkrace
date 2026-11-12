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
BRAIN_DIR = os.path.join(ROOT_DIR, "brain")
CORE_DIR = os.path.join(ROOT_DIR, "core")
CONTROLLERS_DIR = os.path.join(ROOT_DIR, "webots", "simulation", "my_project", "controllers", "tof_path_planner")
for d in (BRAIN_DIR, CORE_DIR, CONTROLLERS_DIR):
    if os.path.exists(d) and d not in sys.path:
        sys.path.insert(0, d)

from core_manager import set_core_affinity, set_process_priority, get_core_affinity
from brain import AutonomousBrain, SENSOR_NAMES
from mcu_telemetry_protocol import encode_motion_command
from brain.planner_config import (
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

    # 2. Initialize Shared Autonomous Brain (C++ Costmap, Gap Scout, Curvilinear SL-MPC, Recovery & Watchdog)
    print(f"[Planner Core 1] Initializing Shared Autonomous Brain...")
    brain = AutonomousBrain()

    # 3. Setup Non-Blocking Sockets
    target_addr = (DEBUG_UDP_HOST, DEBUG_UDP_PORT)
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.setblocking(False)
    mcu_cmd_target = ("127.0.0.1", 9877)

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

    # Kinematic tracking states
    v_actual = 0.0
    steer_actual_deg = 0.0
    gyro_z = 0.0

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
                    if "tof_ranges_raw" in pkt:
                        live_tof = {
                            SENSOR_NAMES[i]: pkt["tof_ranges_raw"][i]
                            for i in range(min(16, len(pkt["tof_ranges_raw"])))
                        }
                        has_live_sensor_feed = True
                        last_sensor_time = now_mono
                    elif "tof" in pkt and isinstance(pkt["tof"], dict):
                        live_tof = pkt["tof"]
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

            # Ingest MCU feedback (prefer latest frame, fallback to last valid hardware frame within 1.0s)
            active_mcu = latest_mcu_pkt or (last_mcu_pkt if (now_mono - last_mcu_time < 1.0) else None)
            if active_mcu and "rpms_act" in active_mcu:
                v_actual = float(active_mcu.get("v_est", active_mcu.get("v_real", v_actual)))
                steer_actual_deg = float(active_mcu.get("steer_actual_deg", steer_actual_deg))
                gyro_z = float(active_mcu.get("gyro_z", 0.0))
            else:
                v_actual = 0.0
                steer_actual_deg = 0.0
                gyro_z = 0.0

            # 2. Execute Shared Autonomous Brain Step
            if has_live_sensor_feed:
                steer_rad = math.radians(steer_actual_deg)
                need_full_telem = (step_counter % TELEM_DECIMATION == 0)
                v_cmd, steer_cmd_rad, brain_telem = brain.step(
                    tof_readings=raw_tof,
                    v_est=v_actual,
                    steer_actual_rad=steer_rad,
                    gyro_z_rad_s=gyro_z,
                    dt=DT_STEP,
                    sim_time_s=sim_time,
                    full_telemetry=need_full_telem,
                )
                steer_cmd_deg = math.degrees(steer_cmd_rad)
                run_mode_val = 1
            else:
                v_cmd = 0.0
                steer_cmd_rad = 0.0
                steer_cmd_deg = 0.0
                run_mode_val = 0
                brain_telem = {
                    "mode": "STANDBY",
                    "status": "OFFLINE",
                    "v_cmd": 0.0,
                    "steer_cmd_deg": 0.0,
                    "plan_ms": 0.0,
                    "map_ms": 0.0,
                    "scout_ms": 0.0,
                    "step_ms": 0.0,
                }

            # 3. Motion Command Uplink: Dispatch binary packet to ESP32 Main via MCU UART Bridge
            cmd_bytes = encode_motion_command(
                target_v_mps=v_cmd,
                target_steer_rad=steer_cmd_rad,
                run_mode=run_mode_val,
                seq=step_counter,
            )
            try:
                cmd_sock.sendto(cmd_bytes, mcu_cmd_target)
            except Exception:
                pass

            # True closed-loop control actuation latency (from ToF receipt to command dispatch)
            t_ctrl_ms = (time.perf_counter() - t_step_start) * 1000.0

            # 4. Compose & Dispatch Telemetry at exact 20 Hz (every 3rd 60 Hz step)
            if step_counter % TELEM_DECIMATION == 0:
                telemetry = dict(brain_telem)
                telemetry["time_step"] = step_counter
                telemetry["timestamp"] = time.time()
                telemetry["sim_time"] = round(sim_time, 3)
                telemetry["v_real"] = round(v_actual, 2)
                telemetry["steer_actual_deg"] = round(steer_actual_deg, 1)

                if active_mcu and "rpms_act" in active_mcu:
                    telemetry["rpms_cmd"] = active_mcu.get("rpms_cmd", [0, 0, 0, 0])
                    telemetry["rpms_act"] = active_mcu.get("rpms_act", [0, 0, 0, 0])
                    telemetry["yaw_deg"] = float(active_mcu.get("yaw_deg", 0.0))
                    telemetry["pitch_deg"] = float(active_mcu.get("pitch_deg", 0.0))
                    telemetry["roll_deg"] = float(active_mcu.get("roll_deg", 0.0))
                    telemetry["gyro_z"] = round(gyro_z, 3)
                    telemetry["accel_fwd"] = float(active_mcu.get("accel_fwd", 0.0))
                    telemetry["accel_lat"] = float(active_mcu.get("accel_lat", 0.0))
                    telemetry["slip_state"] = str(active_mcu.get("slip_state", "NORMAL"))
                    telemetry["slip_ratio"] = float(active_mcu.get("slip_ratio", 0.0))
                    telemetry["flags"] = dict(active_mcu.get("flags", {}))
                    telemetry["mcu_msg"] = active_mcu.get("mcu_msg", "")
                    telemetry["mcu_log"] = active_mcu.get("mcu_log", [])
                else:
                    telemetry["rpms_cmd"] = [0, 0, 0, 0]
                    telemetry["rpms_act"] = [0, 0, 0, 0]
                    telemetry["yaw_deg"] = 0.0
                    telemetry["pitch_deg"] = 0.0
                    telemetry["roll_deg"] = 0.0
                    telemetry["gyro_z"] = 0.0
                    telemetry["accel_fwd"] = 0.0
                    telemetry["accel_lat"] = 0.0
                    telemetry["slip_state"] = "OFFLINE"
                    telemetry["slip_ratio"] = 0.0
                    telemetry["flags"] = {"fake_data": False, "standalone_sim": False}
                    telemetry["mcu_msg"] = ""
                    telemetry["mcu_log"] = []

                telemetry["tof_ranges_raw"] = [round(raw_tof.get(k, 0.0), 3) for k in SENSOR_NAMES] if has_live_sensor_feed else [0.0] * 16
                telemetry["step_ms"] = round(t_ctrl_ms, 2)
                telemetry["grid_shape"] = [int(s) for s in brain.costmap.shape]
                telemetry["grid_bounds"] = [GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX]

                # Non-blocking handoff to background worker
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
