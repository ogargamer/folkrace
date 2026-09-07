"""
Launcher for the Analytic 16-Channel ToF Costmap & MPPI Decision Debugger GUI.
Automatically launches Google Cartographer 2D SLAM and the Webots Bridge in WSL in the background.

Usage:
    python launch_analytic_debugger.py
"""
import os
import sys
import atexit
import subprocess
import time

script_dir = os.path.dirname(os.path.abspath(__file__))
scanner_dir = os.path.join(script_dir, "webots/simulation/my_project", "controllers", "tof_16ch_grid_scanner")
if scanner_dir not in sys.path:
    sys.path.insert(0, scanner_dir)


def to_wsl_path(win_path: str) -> str:
    p = os.path.abspath(win_path).replace("\\", "/")
    if len(p) >= 2 and p[1] == ":":
        drive = p[0].lower()
        p = f"/mnt/{drive}" + p[2:]
    return p


def launch_wsl_cartographer_bridge():
    """Generates and executes the background live Cartographer SLAM bridge inside WSL."""
    cfg_dir_wsl = to_wsl_path(os.path.join(script_dir, "webots/robotics/ros2_cartographer", "config"))
    bridge_py_wsl = to_wsl_path(os.path.join(script_dir, "webots/robotics/ros2_cartographer", "scripts", "webots_cartographer_bridge.py"))
    
    sh_content = f"""#!/usr/bin/env bash
source /opt/ros/humble/setup.bash

# 1. Clean up any stale instances
pkill -9 -f cartographer_node 2>/dev/null || true
pkill -9 -f cartographer_occupancy_grid_node 2>/dev/null || true
pkill -9 -f webots_cartographer_bridge.py 2>/dev/null || true
rm -rf /dev/shm/* 2>/dev/null || true
sleep 0.2

echo "[WSL] Starting Cartographer SLAM Node..."
nohup ros2 run cartographer_ros cartographer_node \\
    -configuration_directory "{cfg_dir_wsl}" \\
    -configuration_basename folkrace_2d.lua \\
    --remap scan:=/scan --remap imu:=/imu > /tmp/carto_live.log 2>&1 &

echo "[WSL] Starting Occupancy Grid Node (2cm resolution, 15 Hz map publishing at 0.066s period)..."
nohup ros2 run cartographer_ros cartographer_occupancy_grid_node \\
    -resolution 0.02 -publish_period_sec 0.066 > /tmp/grid_live.log 2>&1 &

sleep 0.8

echo "[WSL] Starting Webots <-> ROS 2 UDP Bridge..."
exec python3 "{bridge_py_wsl}"
"""
    sh_win = os.path.join(script_dir, "webots/robotics/ros2_cartographer", "scripts", "run_live_wsl_bridge.sh")
    os.makedirs(os.path.dirname(sh_win), exist_ok=True)
    with open(sh_win, "w", encoding="utf-8", newline="\n") as f:
        f.write(sh_content)
        
    sh_wsl = to_wsl_path(sh_win)
    
    print("[Launcher] Launching Google Cartographer 2D SLAM in WSL in background...", flush=True)
    try:
        proc = subprocess.Popen(
            ["wsl", "-e", "bash", sh_wsl],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        return proc
    except Exception as e:
        print(f"[Launcher] Warning: Could not auto-launch WSL Cartographer: {e}", flush=True)
        return None


def cleanup_wsl():
    """Kills background Cartographer processes in WSL on GUI exit."""
    try:
        clean_sh = os.path.join(script_dir, "webots/robotics/ros2_cartographer", "scripts", "stop_wsl_cartographer.sh")
        with open(clean_sh, "w", encoding="utf-8", newline="\n") as f:
            f.write("#!/usr/bin/env bash\npkill -9 -f cartographer_node 2>/dev/null || true\npkill -9 -f cartographer_occupancy_grid_node 2>/dev/null || true\npkill -9 -f webots_cartographer_bridge.py 2>/dev/null || true\n")
        subprocess.run(["wsl", "-e", "bash", to_wsl_path(clean_sh)], capture_output=True)
    except Exception:
        pass


if __name__ == "__main__":
    wsl_proc = launch_wsl_cartographer_bridge()
    atexit.register(cleanup_wsl)

    from analytic_costmap_gui import launch_analytic_gui
    try:
        launch_analytic_gui()
    finally:
        cleanup_wsl()
