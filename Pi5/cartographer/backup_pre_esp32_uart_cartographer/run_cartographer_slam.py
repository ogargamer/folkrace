#!/usr/bin/env python3
"""
Master Supervisor for Google Cartographer 2D SLAM System (Raspberry Pi 5)
Orchestrates LiDAR, IMU, Static TFs, Cartographer SLAM, and Web Map Server.
"""

import os
import sys
import time
import signal
import subprocess

CARTOGRAPHER_DIR = "/home/admin/cartographer"

processes = []

def cleanup(signum=None, frame=None):
    print("\n[SLAM Supervisor] Stopping all Cartographer processes...")
    for p in processes:
        try:
            p.terminate()
        except Exception:
            pass
    time.sleep(0.5)
    for p in processes:
        try:
            p.kill()
        except Exception:
            pass
    # Force kill any lingering ROS nodes
    os.system("pkill -9 -f cartographer_node 2>/dev/null || true")
    os.system("pkill -9 -f cartographer_occupancy_grid_node 2>/dev/null || true")
    os.system("pkill -9 -f ms200_laser_node.py 2>/dev/null || true")
    os.system("pkill -9 -f bno08x_imu_node.py 2>/dev/null || true")
    os.system("pkill -9 -f slam_map_server.py 2>/dev/null || true")
    print("[SLAM Supervisor] All processes terminated cleanly.")
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

def run_cmd(cmd_list, name):
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    # Source ROS 2 Jazzy environment
    bash_cmd = f"source /opt/ros/jazzy/setup.bash && {' '.join(cmd_list)}"
    p = subprocess.Popen(["bash", "-c", bash_cmd], env=env)
    processes.append(p)
    print(f"  [+] Started: {name} (PID {p.pid})")
    return p

def main():
    print("================================================================")
    print("   STARTING GOOGLE CARTOGRAPHER 2D REAL-TIME SLAM SYSTEM")
    print("================================================================")

    # 1. Start Static Transforms (colocated base_link -> imu_link for Cartographer tracking)
    run_cmd(["ros2", "run", "tf2_ros", "static_transform_publisher", "--x", "0.0", "--y", "0.0", "--z", "0.0", "--roll", "0", "--pitch", "0", "--yaw", "0", "--frame-id", "base_link", "--child-frame-id", "imu_link"], "Static TF (base_link -> imu_link)")
    run_cmd(["ros2", "run", "tf2_ros", "static_transform_publisher", "--x", "0.0", "--y", "0.0", "--z", "0.05", "--roll", "0", "--pitch", "0", "--yaw", "0", "--frame-id", "base_link", "--child-frame-id", "laser_frame"], "Static TF (base_link -> laser_frame)")
    time.sleep(1)

    # 2. Start MS200 LiDAR Publisher (/scan)
    run_cmd(["python3", f"{CARTOGRAPHER_DIR}/ms200_laser_node.py"], "MS200 LiDAR Node (/scan)")
    time.sleep(1)

    # 3. Start BNO08X IMU Publisher (/imu)
    run_cmd(["python3", f"{CARTOGRAPHER_DIR}/bno08x_imu_node.py"], "BNO08X IMU Node (/imu)")
    time.sleep(1)

    # 4. Start Google Cartographer 2D Node
    carto_cmd = [
        "ros2", "run", "cartographer_ros", "cartographer_node",
        "-configuration_directory", CARTOGRAPHER_DIR,
        "-configuration_basename", "cartographer_2d.lua",
        "--ros-args", "-r", "scan:=/scan", "-r", "imu:=/imu"
    ]
    run_cmd(carto_cmd, "Google Cartographer 2D SLAM Node")
    time.sleep(1)

    # 5. Start Cartographer Occupancy Grid Node (/map @ 2cm)
    grid_cmd = [
        "ros2", "run", "cartographer_ros", "cartographer_occupancy_grid_node",
        "-resolution", "0.02",
        "-publish_period_sec", "0.5"
    ]
    run_cmd(grid_cmd, "Occupancy Grid Map Node (/map)")
    time.sleep(1)

    # 6. Start Web Map Server (Port 8888)
    run_cmd(["python3", f"{CARTOGRAPHER_DIR}/slam_map_server.py"], "2D SLAM Web Server (Port 8888)")

    print("\n================================================================")
    print("  >> GOOGLE CARTOGRAPHER 2D SLAM ACTIVE! <<")
    print("  >> Web Dashboard: http://10.43.8.42:8888 <<")
    print("================================================================\n")

    # Monitor processes
    while True:
        for p in processes:
            if p.poll() is not None:
                print(f"[Warning] Process {p.pid} exited with code {p.returncode}")
        time.sleep(2)

if __name__ == "__main__":
    main()
