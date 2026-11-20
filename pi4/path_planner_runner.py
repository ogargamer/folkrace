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
for d in (BRAIN_DIR, CORE_DIR, CONTROLLERS_DIR, os.path.join(CURR_DIR, "brain"), os.path.join(CURR_DIR, "core")):
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
LINK_TIMEOUT_S = 0.25  # 250ms link-loss failsafe (matches MCU bridge & ESP32-STM32)
BRAKE_DURATION_S = 2.0  # Active braking hold duration before disabling motor drivers
TELEM_HZ = 60.0
TELEM_DECIMATION = int(round(TARGET_HZ / TELEM_HZ))  # Full 60 Hz for flight recorder; debug_server throttles live WS to 20 Hz
DEBUG_UDP_PORT = int(os.environ.get("DEBUG_UDP_PORT", 9876))
DEBUG_UDP_HOST = os.environ.get("DEBUG_UDP_HOST", "127.0.0.1")
SENSOR_UDP_PORT = int(os.environ.get("SENSOR_UDP_PORT", 9875))
PLANNER_CMD_UDP_PORT = int(os.environ.get("PLANNER_CMD_UDP_PORT", 9878))


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
    telem_queue = queue.Queue(maxsize=90)

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

    planner_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    planner_cmd_sock.setblocking(False)
    try:
        planner_cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        planner_cmd_sock.bind(("127.0.0.1", PLANNER_CMD_UDP_PORT))
        print(f"[Planner Core 1] Listening for Start/Stop commands on 127.0.0.1:{PLANNER_CMD_UDP_PORT}...")
    except Exception as e:
        print(f"[Planner Core 1] Note on planner command socket: {e}")

    def dispatch_stop_hw():
        try:
            cmd_sock.sendto(b"ESTOP\n", mcu_cmd_target)
            cmd_sock.sendto(b"RESET\n", mcu_cmd_target)
            cmd_sock.sendto(encode_motion_command(0.0, 0.0, 0, step_counter), mcu_cmd_target)
        except Exception:
            pass

    def dispatch_start_hw():
        try:
            cmd_sock.sendto(b"E\n", mcu_cmd_target)
            cmd_sock.sendto(b"MOTORS\n", mcu_cmd_target)
        except Exception:
            pass

    planner_running = False
    was_mcu_main_connected = False
    was_driving_links_ok = False
    brake_until = 0.0

    live_tof_list: List[float] = [0.0] * 16
    has_live_sensor_feed = False
    last_sensor_time = 0.0
    last_mcu_time = 0.0
    last_mcu_pkt: Optional[Dict[str, Any]] = None
    last_tof_pkt: Optional[Dict[str, Any]] = None

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

            # 0. Drain Start/Stop Planner Commands
            while True:
                try:
                    p_cmd_bytes, _ = planner_cmd_sock.recvfrom(512)
                    p_cmd = p_cmd_bytes.decode("utf-8", errors="ignore").strip().upper()
                    if p_cmd == "START":
                        brake_until = 0.0
                        if not planner_running:
                            planner_running = True
                            dispatch_start_hw()
                            print("[Planner Core 1] ▶ STARTED (Line calc active, motor drivers enabled)")
                    elif p_cmd == "STOP":
                        if planner_running:
                            planner_running = False
                            if was_driving_links_ok or was_mcu_main_connected:
                                brake_until = time.perf_counter() + BRAKE_DURATION_S
                                print(f"[Planner Core 1] 🛑 STOPPING — Braking for {BRAKE_DURATION_S:.1f}s before disabling motor drivers...")
                            else:
                                brake_until = 0.0
                                dispatch_stop_hw()
                                print("[Planner Core 1] ⏹ STOPPED (Line calc stopped, motors disabled, servo centered)")
                        elif brake_until == 0.0:
                            dispatch_stop_hw()
                            print("[Planner Core 1] ⏹ STOPPED (Line calc stopped, motors disabled, servo centered)")
                    elif p_cmd == "TOGGLE":
                        if not planner_running:
                            brake_until = 0.0
                            planner_running = True
                            dispatch_start_hw()
                            print("[Planner Core 1] ▶ STARTED (Line calc active, motor drivers enabled)")
                        else:
                            planner_running = False
                            if was_driving_links_ok or was_mcu_main_connected:
                                brake_until = time.perf_counter() + BRAKE_DURATION_S
                                print(f"[Planner Core 1] 🛑 STOPPING — Braking for {BRAKE_DURATION_S:.1f}s before disabling motor drivers...")
                            else:
                                brake_until = 0.0
                                dispatch_stop_hw()
                                print("[Planner Core 1] ⏹ STOPPED (Line calc stopped, motors disabled, servo centered)")
                except (BlockingIOError, socket.error):
                    break
                except Exception:
                    break

            # 1. Perception Ingestion: Drain sensor UDP socket to receive latest real-time frame.
            # Coalesce raw datagrams before JSON decoding so bursty packets never waste Core 1 CPU.
            latest_mcu_pkt = None
            raw_mcu_bytes = None
            raw_tof_bytes = None
            now_mono = time.time()
            while True:
                try:
                    raw_bytes, _ = sensor_sock.recvfrom(4096)
                    if b'"rpms_act"' in raw_bytes:
                        raw_mcu_bytes = raw_bytes
                    if b'"tof_ranges_raw"' in raw_bytes or b'"tof"' in raw_bytes:
                        raw_tof_bytes = raw_bytes
                except (BlockingIOError, socket.error):
                    break
                except Exception:
                    break

            if raw_mcu_bytes is not None:
                try:
                    pkt = json.loads(raw_mcu_bytes.decode("utf-8"))
                    latest_mcu_pkt = pkt
                    last_mcu_pkt = pkt
                    last_mcu_time = now_mono
                    if raw_tof_bytes == raw_mcu_bytes:
                        raw_tof_bytes = None
                        last_tof_pkt = pkt
                        if not pkt.get("mcu_tof_connected", False) or not pkt.get("tof_front_ok", True):
                            has_live_sensor_feed = False
                            live_tof_list = [0.0] * 16
                        elif "tof_ranges_raw" in pkt:
                            r_in = pkt["tof_ranges_raw"]
                            live_tof_list = [float(r_in[i]) if i < len(r_in) else 0.0 for i in range(16)]
                            has_live_sensor_feed = True
                            last_sensor_time = now_mono
                except Exception:
                    pass

            if raw_tof_bytes is not None:
                try:
                    pkt = json.loads(raw_tof_bytes.decode("utf-8"))
                    last_tof_pkt = pkt
                    if not pkt.get("mcu_tof_connected", False) or not pkt.get("tof_front_ok", True):
                        has_live_sensor_feed = False
                        live_tof_list = [0.0] * 16
                    elif "tof_ranges_raw" in pkt:
                        r_in = pkt["tof_ranges_raw"]
                        live_tof_list = [float(r_in[i]) if i < len(r_in) else 0.0 for i in range(16)]
                        has_live_sensor_feed = True
                        last_sensor_time = now_mono
                    elif "tof" in pkt and isinstance(pkt["tof"], dict):
                        t_dict = pkt["tof"]
                        live_tof_list = [float(t_dict.get(k, 0.0)) for k in SENSOR_NAMES]
                        has_live_sensor_feed = True
                        last_sensor_time = now_mono
                except Exception:
                    pass

            if has_live_sensor_feed and (now_mono - last_sensor_time > LINK_TIMEOUT_S):
                has_live_sensor_feed = False
                live_tof_list = [0.0] * 16

            # Ingest MCU feedback (prefer latest frame, fallback to last valid hardware frame within LINK_TIMEOUT_S)
            active_mcu = latest_mcu_pkt or (last_mcu_pkt if (now_mono - last_mcu_time <= LINK_TIMEOUT_S) else None)
            if active_mcu and "rpms_act" in active_mcu:
                v_actual = float(active_mcu.get("v_est", active_mcu.get("v_real", v_actual)))
                steer_actual_deg = float(active_mcu.get("steer_actual_deg", steer_actual_deg))
                gyro_z = float(active_mcu.get("gyro_z", 0.0))
                pitch_deg = float(active_mcu.get("pitch_deg", 0.0))
                roll_deg = float(active_mcu.get("roll_deg", 0.0))
            else:
                v_actual = 0.0
                steer_actual_deg = 0.0
                gyro_z = 0.0
                pitch_deg = 0.0
                roll_deg = 0.0

            # Ensure hardware starts in safe stopped state when ESP32 Main first connects
            mcu_main_now = bool(active_mcu and active_mcu.get("mcu_main_connected", False))
            if mcu_main_now and not was_mcu_main_connected and not planner_running:
                dispatch_stop_hw()
            was_mcu_main_connected = mcu_main_now

            # 2. Execute Shared Autonomous Brain Step (only when started AND both Main & ToF links are live)
            driving_links_ok = has_live_sensor_feed and mcu_main_now
            if planner_running and driving_links_ok:
                was_driving_links_ok = True
                steer_rad = math.radians(steer_actual_deg)
                need_full_telem = (step_counter % TELEM_DECIMATION == 0)
                v_cmd, steer_cmd_rad, brain_telem = brain.step(
                    tof_readings=live_tof_list,
                    v_est=v_actual,
                    steer_actual_rad=steer_rad,
                    gyro_z_rad_s=gyro_z,
                    dt=DT_STEP,
                    sim_time_s=sim_time,
                    full_telemetry=need_full_telem,
                    pitch_rad=math.radians(pitch_deg),
                    roll_rad=math.radians(roll_deg),
                )
                steer_cmd_deg = math.degrees(steer_cmd_rad)
                run_mode_val = 1
            else:
                if planner_running and was_driving_links_ok:
                    print(f"[Planner Core 1] ⚠ FAILSAFE: MCU/ToF link lost — braking {BRAKE_DURATION_S:.1f}s before disabling drivers.")
                    if mcu_main_now:
                        brake_until = time.perf_counter() + BRAKE_DURATION_S
                    else:
                        brake_until = 0.0
                        dispatch_stop_hw()
                was_driving_links_ok = False
                v_cmd = 0.0
                steer_cmd_rad = 0.0
                steer_cmd_deg = 0.0
                if brake_until > 0.0:
                    if time.perf_counter() < brake_until:
                        run_mode_val = 1
                    else:
                        brake_until = 0.0
                        dispatch_stop_hw()
                        run_mode_val = 0
                        print("[Planner Core 1] ⏹ STOPPED (Brake complete, motor drivers disabled, servo centered)")
                else:
                    run_mode_val = 0
                brain.rate_limited_speed = 0.0
                brain.cached_target_speed = 0.0
                brain.cached_target_steer = 0.0
                brain.recovery_manager.reset()
                mode_str = "BRAKING" if run_mode_val == 1 else ("STOPPED" if not planner_running else "STANDBY")
                status_str = "BRAKING" if run_mode_val == 1 else ("STOPPED" if not planner_running else "OFFLINE")
                brain_telem = {
                    "mode": mode_str,
                    "status": status_str,
                    "v_cmd": 0.0,
                    "steer_cmd_deg": 0.0,
                    "plan_ms": 0.0,
                    "map_ms": 0.0,
                    "scout_ms": 0.0,
                    "osqp_iters": 0,
                    "osqp_iters_alt": 0,
                    "iters_prim": 0,
                    "iters_alt": 0,
                    "step_ms": 0.0,
                    "best_line": [],
                    "candidate_lines": [],
                    "guide_line": [],
                    "corridor_left": [],
                    "corridor_right": [],
                    "targets": [],
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
                telemetry["v_est"] = round(v_actual, 3)
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
                    telemetry["servo_pulse_us"] = int(active_mcu.get("servo_pulse_us", 1565))
                    telemetry["flags"] = dict(active_mcu.get("flags", {}))
                    telemetry["mcu_msg"] = active_mcu.get("mcu_msg", "")
                    telemetry["mcu_log"] = active_mcu.get("mcu_log", [])
                    telemetry["mcu_main_connected"] = bool(active_mcu.get("mcu_main_connected", False))
                    telemetry["mcu_tof_connected"] = bool(active_mcu.get("mcu_tof_connected", has_live_sensor_feed))
                    telemetry["mcu_main_port"] = str(active_mcu.get("mcu_main_port", ""))
                    telemetry["mcu_tof_port"] = str(active_mcu.get("mcu_tof_port", ""))
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
                    telemetry["servo_pulse_us"] = 1565
                    telemetry["flags"] = {"fake_data": False, "standalone_sim": False}
                    telemetry["mcu_msg"] = ""
                    telemetry["mcu_log"] = []
                    telemetry["mcu_main_connected"] = False
                    telemetry["mcu_tof_connected"] = bool(has_live_sensor_feed)
                    telemetry["mcu_main_port"] = ""
                    telemetry["mcu_tof_port"] = str(last_tof_pkt.get("mcu_tof_port", "")) if last_tof_pkt else ""

                telemetry["tof_ranges_raw"] = [round(v, 3) for v in live_tof_list] if has_live_sensor_feed else [0.0] * 16
                if has_live_sensor_feed and last_tof_pkt:
                    for tk in (
                        "tof_target0_mm",
                        "tof_target1_mm",
                        "tof_target0_status",
                        "tof_target1_status",
                        "tof_front_ok",
                        "tof_sensor_online",
                    ):
                        if tk in last_tof_pkt:
                            telemetry[tk] = last_tof_pkt[tk]
                else:
                    telemetry["tof_target0_mm"] = [0] * 16
                    telemetry["tof_target1_mm"] = [0] * 16
                    telemetry["tof_target0_status"] = [255] * 16
                    telemetry["tof_target1_status"] = [255] * 16
                telemetry["step_ms"] = round(t_ctrl_ms, 2)
                telemetry["grid_shape"] = [int(s) for s in brain.costmap.shape]
                telemetry["grid_bounds"] = [GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX]
                telemetry["planner_running"] = planner_running
                if not isinstance(telemetry.get("flags"), dict):
                    telemetry["flags"] = {}
                telemetry["flags"]["planner_running"] = planner_running

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
