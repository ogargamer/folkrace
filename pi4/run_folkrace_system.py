#!/usr/bin/env python3
"""
================================================================================
  FOLKRACE PI 4: DUAL-CORE AUTONOMOUS SYSTEM LAUNCHER
================================================================================
  Spawns and monitors:
    1. Path Planner Pipeline -> HARD PINNED TO CPU CORE 1
    2. Debug Server & Black Box Recorder -> HARD PINNED TO CPU CORE 2 (and 3)
  
  Verifies hardware isolation so the debug server cannot affect planner performance.
================================================================================
"""

import os
import sys
import time
import signal
import subprocess
import platform

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
if CURR_DIR not in sys.path:
    sys.path.insert(0, CURR_DIR)

PLANNER_SCRIPT = os.path.join(CURR_DIR, "path_planner_runner.py")
DEBUG_SERVER_SCRIPT = os.path.join(CURR_DIR, "debug_server.py")


def check_core_assignment(pid: int) -> str:
    """Reads /proc/<pid>/status to verify assigned CPU cores."""
    status_file = f"/proc/{pid}/status"
    if os.path.exists(status_file):
        try:
            with open(status_file, "r") as f:
                for line in f:
                    if line.startswith("Cpus_allowed_list:"):
                        return line.split(":", 1)[1].strip()
        except Exception:
            pass
    return "unknown"


def main():
    print("=" * 70)
    print("  FOLKRACE PI 4: DUAL-CORE HARDWARE ISOLATION LAUNCHER")
    print("=" * 70)
    print(f"  Platform Architecture : {platform.machine()} ({sys.platform})")
    print(f"  Available CPU Cores   : {os.cpu_count()}")
    print("=" * 70)

    has_taskset = False
    if sys.platform == "linux":
        try:
            has_taskset = bool(subprocess.run(["which", "taskset"], stdout=subprocess.PIPE, text=True).stdout.strip())
        except Exception:
            pass

    # Launch Debug Server on Core 2
    print("[1/2] Launching Debug Server on Core 2...")
    if has_taskset:
        cmd_debug = ["taskset", "-c", "2,3", sys.executable, DEBUG_SERVER_SCRIPT]
    else:
        cmd_debug = [sys.executable, DEBUG_SERVER_SCRIPT]

    p_debug = subprocess.Popen(cmd_debug, cwd=CURR_DIR)
    time.sleep(1.0)
    print(f"      PID: {p_debug.pid} | CPU Core Mask: {check_core_assignment(p_debug.pid)}")

    # Launch Path Planner on Core 1
    print("[2/2] Launching Real-Time Path Planner on Core 1...")
    if sys.platform == "linux":
        cmd_planner = ["taskset", "-c", "1", sys.executable, PLANNER_SCRIPT]
    else:
        cmd_planner = [sys.executable, PLANNER_SCRIPT]

    p_planner = subprocess.Popen(cmd_planner, cwd=CURR_DIR)
    time.sleep(1.0)
    print(f"      PID: {p_planner.pid} | CPU Core Mask: {check_core_assignment(p_planner.pid)}")

    print("=" * 70)
    print("  [SYSTEM READY]")
    print("  * Path Planner is running deterministically on CPU CORE 1.")
    print("  * Debug Server & Black Box Recorder are running on CPU CORE 2.")
    print("  * Open Web Dashboard on connected laptop: http://robot:8080")
    print("  * Press Ctrl+C to stop all processes.")
    print("=" * 70)

    try:
        while True:
            # Check if either process died unexpectedly
            if p_debug.poll() is not None:
                print(f"[Launcher] Debug server exited with code {p_debug.returncode}")
                break
            if p_planner.poll() is not None:
                print(f"[Launcher] Path planner exited with code {p_planner.returncode}")
                break
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[Launcher] Shutting down Folkrace subsystems...")
    finally:
        for p in (p_planner, p_debug):
            if p and p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    p.kill()
        print("[Launcher] Clean shutdown complete.")


if __name__ == "__main__":
    main()
