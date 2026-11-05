#!/usr/bin/env python3
"""
================================================================================
  FOLKRACE PI 4: REAL-HARDWARE CURVILINEAR SL-MPC BENCHMARK
================================================================================
  Hardware Policy:
    * Strictly restricted to physical Raspberry Pi 4 Model B hardware (ARM64).
    * Execution on host workstation (Windows / x86_64) is disallowed.
    * When invoked from host workstation, it checks SSH connection to 'robot'.
      - If connected: syncs repository stack and runs benchmark remotely on Pi 4.
      - If disconnected: fails immediately and requests physical connection.
================================================================================
"""

import os
import sys
import time
import csv
import subprocess
import platform
from pathlib import Path
from typing import Dict, List, Optional, Tuple

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
if CURR_DIR not in sys.path:
    sys.path.insert(0, CURR_DIR)

WORKSPACE_ROOT = os.path.abspath(os.path.join(CURR_DIR, ".."))
if WORKSPACE_ROOT not in sys.path:
    sys.path.insert(0, WORKSPACE_ROOT)

PLANNER_DIR = os.path.join(WORKSPACE_ROOT, "webots", "simulation", "my_project", "controllers", "tof_path_planner")
if os.path.exists(PLANNER_DIR) and PLANNER_DIR not in sys.path:
    sys.path.insert(0, PLANNER_DIR)

SSH_HOST = os.environ.get("PI_SSH_HOST", "robot")
REMOTE_DIR = "/home/admin/folkrace"


def is_raspberry_pi() -> bool:
    """Verifies whether the current process is executing on physical Raspberry Pi hardware."""
    if sys.platform != "linux":
        return False

    # 1. Check device-tree model name
    for path in ("/sys/firmware/devicetree/base/model", "/proc/device-tree/model"):
        if os.path.exists(path):
            try:
                with open(path, "r", encoding="utf-8", errors="ignore") as f:
                    model = f.read()
                    if "Raspberry Pi" in model:
                        return True
            except Exception:
                pass

    # 2. Check cpuinfo hardware identifier
    if os.path.exists("/proc/cpuinfo"):
        try:
            with open("/proc/cpuinfo", "r", encoding="utf-8", errors="ignore") as f:
                content = f.read()
                if any(x in content for x in ("BCM2711", "BCM2835", "Raspberry Pi")):
                    return True
        except Exception:
            pass

    return False


def check_pi_ssh(host: str = SSH_HOST, timeout_s: int = 3) -> bool:
    """Checks if Raspberry Pi is reachable over SSH."""
    cmd = ["ssh", "-o", "BatchMode=yes", "-o", f"ConnectTimeout={timeout_s}", host, "echo ok"]
    try:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout_s + 1, text=True)
        return res.returncode == 0 and "ok" in res.stdout
    except Exception:
        return False


def run_remote_on_pi(host: str = SSH_HOST):
    """Synchronizes files and runs benchmark remotely on the Raspberry Pi 4 over SSH."""
    print("=" * 70)
    print("  [REMOTE DELEGATION] Running benchmark on Raspberry Pi 4...")
    print(f"  Target: {host} | Remote Directory: {REMOTE_DIR}")
    print("=" * 70)

    # 1. Create remote target directory
    print("[1/3] Preparing remote environment...")
    subprocess.run(["ssh", host, f"mkdir -p {REMOTE_DIR}/core {REMOTE_DIR}/data"], check=True)

    # 2. Sync files
    print("[2/3] Syncing controller scripts, core params, and dataset to Pi 4...")
    files_to_sync = [
        os.path.join(CURR_DIR, "benchmark_sl_mpc.py"),
        os.path.join(CURR_DIR, "build_so.sh"),
        os.path.join(PLANNER_DIR, "planner_config.py"),
        os.path.join(PLANNER_DIR, "planner_sl_mpc.py"),
        os.path.join(PLANNER_DIR, "cpp_smart_hybrid_costmap.py"),
        os.path.join(PLANNER_DIR, "tof_gap_scout.py"),
        os.path.join(PLANNER_DIR, "ramp_detector.py"),
    ]
    scp_cmd = ["scp"] + [f for f in files_to_sync if os.path.exists(f)] + [f"{host}:{REMOTE_DIR}/"]
    subprocess.run(scp_cmd, check=True)

    # Sync core/
    core_dir = os.path.join(WORKSPACE_ROOT, "core")
    if os.path.exists(core_dir):
        subprocess.run(["scp", "-r", f"{core_dir}/*", f"{host}:{REMOTE_DIR}/core/"], check=True)

    # Sync dataset if available
    dataset_src = os.path.join(PLANNER_DIR, "data", "datasets", "tof_readings_cube_10cm.csv")
    if os.path.exists(dataset_src):
        subprocess.run(["scp", dataset_src, f"{host}:{REMOTE_DIR}/data/"], check=True)

    # 3. Execute remote benchmark
    print("[3/3] Executing benchmark natively on Raspberry Pi 4...")
    print("-" * 70)
    remote_exec = f"cd {REMOTE_DIR} && python3 benchmark_sl_mpc.py"
    res = subprocess.run(["ssh", "-t", host, remote_exec])
    sys.exit(res.returncode)


def print_disconnected_error(host: str):
    """Prints a clear error message when Pi 4 is unreachable."""
    print("=" * 70)
    print("  [FATAL ERROR] RASPBERRY PI 4 IS NOT CONNECTED")
    print("=" * 70)
    print(f"  pi4/benchmark_sl_mpc.py is restricted to physical Raspberry Pi 4 hardware.")
    print(f"  Current machine: {sys.platform} ({platform.machine()}) -- Execution rejected.")
    print()
    print(f"  SSH Target : {host} (admin@{host})")
    print(f"  Status     : UNREACHABLE / DISCONNECTED")
    print()
    print("  ACTIONS REQUIRED:")
    print("  1. Connect Raspberry Pi 4 to power and network (Ethernet / Wi-Fi).")
    print("  2. Verify Pi 4 is accessible: ping robot or ssh robot")
    print("  3. Re-run this benchmark once the Pi 4 connection is established.")
    print("=" * 70)
    sys.exit(1)


# ==============================================================================
# PHYSICAL PI 4 BENCHMARK EXECUTION
# ==============================================================================

def load_dataset(csv_path: str, sensor_names: List[str]):
    """Loads 16-channel ToF frames from recorded CSV file."""
    frames = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame = {}
            for k in sensor_names:
                if k in row:
                    frame[k] = float(row[k])
            if len(frame) == 16:
                frames.append(frame)
    return frames


def run_physical_pi_benchmark():
    import numpy as np
    from cpp_smart_hybrid_costmap import CppSmartHybridCostmap, SENSOR_NAMES
    from tof_gap_scout import ToFGapScout
    from planner_sl_mpc import CurvilinearSLMPC, CPP_AVAILABLE
    from planner_config import (
        GRID_X_MIN,
        GRID_X_MAX,
        GRID_Y_MIN,
        GRID_Y_MAX,
        GRID_RES,
        R_INFLATION_M,
    )

    print("=" * 70)
    print("  FOLKRACE PI 4: PHYSICAL HARDWARE CURVILINEAR SL-MPC BENCHMARK")
    print(f"  Host Architecture : {platform.machine()} | Kernel: {platform.release()}")
    print(f"  C++ Native SL-MPC Engine Available: {CPP_AVAILABLE}")
    print("=" * 70)

    if not CPP_AVAILABLE:
        print("[WARNING] C++ native engine not built on Pi 4. Compiling now via build_so.sh...")
        build_sh = os.path.join(CURR_DIR, "build_so.sh")
        if os.path.exists(build_sh):
            subprocess.run(["bash", build_sh], check=True)
            print("[INFO] Build complete, restarting benchmark...")
            os.execv(sys.executable, [sys.executable] + sys.argv)
        else:
            print("[ERROR] build_so.sh not found.")
            sys.exit(1)

    dataset_candidates = [
        os.path.join(CURR_DIR, "data", "tof_readings_cube_10cm.csv"),
        os.path.join(CURR_DIR, "tof_readings_cube_10cm.csv"),
        os.path.join(PLANNER_DIR, "data", "datasets", "tof_readings_cube_10cm.csv"),
    ]
    dataset_path = None
    for cand in dataset_candidates:
        if os.path.exists(cand):
            dataset_path = cand
            break

    if not dataset_path:
        print(f"[ERR] Dataset not found in any of: {dataset_candidates}")
        sys.exit(1)

    frames = load_dataset(dataset_path, SENSOR_NAMES)
    print(f"[INFO] Loaded {len(frames)} ToF frames from {os.path.basename(dataset_path)}")

    costmap = CppSmartHybridCostmap(
        grid_bounds=(GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX),
        resolution=GRID_RES,
        r_inflation_m=R_INFLATION_M,
    )
    gap_scout = ToFGapScout()
    planner = CurvilinearSLMPC(costmap_engine=costmap)
    planner.costmap = costmap

    v_curr = 2.50
    steer_curr = 0.0
    yaw_rate_curr = 0.0
    target_lookahead = 2.40

    # Warmup phase (50 iterations)
    print("\n[INFO] Running 50 warmup iterations on BCM2711 CPU...")
    for i in range(min(50, len(frames))):
        tof = frames[i]
        prob_map, hazard_mask, _ = costmap.process_tof_frame(tof)
        xg, yg, width, theta_g = gap_scout.scout_gap(
            tof_readings=tof,
            hazard_mask=hazard_mask,
            costmap_obj=costmap,
            target_lookahead_m=target_lookahead,
            steer_current=steer_curr,
            prob_map=prob_map,
        )
        _ = planner.plan(
            xg=xg,
            yg=yg,
            theta_g=theta_g,
            aperture_width=width,
            v_current=v_curr,
            steer_current=steer_curr,
            yaw_rate_current=yaw_rate_curr,
            extra_targets=getattr(gap_scout, "last_targets", None),
        )

    # Benchmark Measurement across all frames
    print(f"[INFO] Benchmarking {len(frames)} frames...")
    costmap_times = []
    scout_times = []
    sl_mpc_times = []
    total_pipeline_times = []
    osqp_iterations = []
    cmd_speeds = []
    cmd_steers = []

    for i, tof in enumerate(frames):
        t_start_total = time.perf_counter_ns()

        t0 = time.perf_counter_ns()
        prob_map, hazard_mask, _ = costmap.process_tof_frame(tof)
        t_costmap_us = (time.perf_counter_ns() - t0) / 1000.0

        t0 = time.perf_counter_ns()
        xg, yg, width, theta_g = gap_scout.scout_gap(
            tof_readings=tof,
            hazard_mask=hazard_mask,
            costmap_obj=costmap,
            target_lookahead_m=target_lookahead,
            steer_current=steer_curr,
            prob_map=prob_map,
        )
        t_scout_us = (time.perf_counter_ns() - t0) / 1000.0

        t0 = time.perf_counter_ns()
        plan_res = planner.plan(
            xg=xg,
            yg=yg,
            theta_g=theta_g,
            aperture_width=width,
            v_current=v_curr,
            steer_current=steer_curr,
            yaw_rate_current=yaw_rate_curr,
            extra_targets=getattr(gap_scout, "last_targets", None),
        )
        t_sl_mpc_us = (time.perf_counter_ns() - t0) / 1000.0

        t_total_us = (time.perf_counter_ns() - t_start_total) / 1000.0

        costmap_times.append(t_costmap_us)
        scout_times.append(t_scout_us)
        sl_mpc_times.append(t_sl_mpc_us)
        total_pipeline_times.append(t_total_us)

        osqp_iterations.append(plan_res.get("osqp_iters", 0))
        cmd_speeds.append(plan_res.get("cmd_speed_mps", 0.0))
        cmd_steers.append(plan_res.get("cmd_steer_rad", 0.0))

        cmd_steer = plan_res.get("cmd_steer_rad", 0.0)
        steer_curr = steer_curr + 0.3 * (cmd_steer - steer_curr)
        v_curr = max(0.5, min(5.0, plan_res.get("cmd_speed_mps", v_curr)))

    def stats_ms(arr_us):
        arr_ms = np.array(arr_us) / 1000.0
        return {
            "mean": np.mean(arr_ms),
            "median": np.median(arr_ms),
            "min": np.min(arr_ms),
            "max": np.max(arr_ms),
            "p95": np.percentile(arr_ms, 95),
            "p99": np.percentile(arr_ms, 99),
        }

    c_stats = stats_ms(costmap_times)
    s_stats = stats_ms(scout_times)
    m_stats = stats_ms(sl_mpc_times)
    t_stats = stats_ms(total_pipeline_times)

    print("\n" + "=" * 70)
    print("  PHYSICAL RASPBERRY PI 4 BENCHMARK RESULTS")
    print("=" * 70)
    print(f"{'Component':<32} {'Mean':<8} {'Median':<8} {'P95':<8} {'P99':<8} {'Max':<8}")
    print("-" * 70)
    print(f"{'1. C++ Costmap Update':<32} {c_stats['mean']:>6.3f}ms {c_stats['median']:>6.3f}ms {c_stats['p95']:>6.3f}ms {c_stats['p99']:>6.3f}ms {c_stats['max']:>6.3f}ms")
    print(f"{'2. ToF Gap Scout':<32} {s_stats['mean']:>6.3f}ms {s_stats['median']:>6.3f}ms {s_stats['p95']:>6.3f}ms {s_stats['p99']:>6.3f}ms {s_stats['max']:>6.3f}ms")
    print(f"{'3. C++ Curvilinear SL-MPC (OSQP)':<32} {m_stats['mean']:>6.3f}ms {m_stats['median']:>6.3f}ms {m_stats['p95']:>6.3f}ms {m_stats['p99']:>6.3f}ms {m_stats['max']:>6.3f}ms")
    print("-" * 70)
    print(f"{'TOTAL PLANNING PIPELINE':<32} {t_stats['mean']:>6.3f}ms {t_stats['median']:>6.3f}ms {t_stats['p95']:>6.3f}ms {t_stats['p99']:>6.3f}ms {t_stats['max']:>6.3f}ms")
    print("=" * 70)

    avg_hz = 1000.0 / t_stats['mean']
    p99_hz = 1000.0 / t_stats['p99']
    min_hz = 1000.0 / t_stats['max']
    sl_mpc_only_hz = 1000.0 / m_stats['mean']

    print(f"\n[RATES & FREQUENCIES (Physical Hardware)]")
    print(f"  * Curvilinear SL-MPC Alone:     {sl_mpc_only_hz:>6.1f} Hz (mean {m_stats['mean']:.3f} ms)")
    print(f"  * Full End-to-End Pipeline:    {avg_hz:>6.1f} Hz (mean {t_stats['mean']:.3f} ms)")
    print(f"  * 99th Percentile Guaranteed:  {p99_hz:>6.1f} Hz ({t_stats['p99']:.3f} ms)")
    print(f"  * Worst-Case Single Frame:     {min_hz:>6.1f} Hz ({t_stats['max']:.3f} ms)")

    print(f"\n[SOLVER METRICS]")
    print(f"  * OSQP Mean Iterations:         {np.mean(osqp_iterations):.1f}")
    print(f"  * OSQP Max Iterations:          {np.max(osqp_iterations)}")
    print(f"  * Mean Commanded Speed:         {np.mean(cmd_speeds):.2f} m/s")
    print(f"  * Steering Output Range:        [{np.min(cmd_steers):.3f}, {np.max(cmd_steers):.3f}] rad")
    print("\n" + "=" * 70)
    print("  PHYSICAL PI 4 BENCHMARK COMPLETED SUCCESSFULLY")
    print("=" * 70)


def main():
    if is_raspberry_pi():
        # Running directly on physical Raspberry Pi hardware
        run_physical_pi_benchmark()
    else:
        # Running from host workstation: check SSH connectivity
        print(f"[pi4_benchmark] Host is {sys.platform} ({platform.machine()}). Enforcing Pi 4 execution policy...")
        if check_pi_ssh(SSH_HOST):
            run_remote_on_pi(SSH_HOST)
        else:
            print_disconnected_error(SSH_HOST)


if __name__ == "__main__":
    main()
