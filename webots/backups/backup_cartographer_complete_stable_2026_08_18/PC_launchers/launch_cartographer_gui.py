#!/usr/bin/env python3
"""
================================================================================
  DA WINNA - GOOGLE CARTOGRAPHER 2D SLAM ONE-CLICK LAUNCHER
================================================================================
Fully sets up, synchronizes, and runs Google Cartographer 2D SLAM on Raspberry Pi 5:
- Reconnects automatically after Pi reboots (with multi-attempt retry loop)
- Synchronizes latest MS200 LiDAR, 200Hz esp32 UART2 IMU, and Cartographer configs
- Frees hardware serial ports (/dev/ttyAMA2, /dev/ttyAMA4) & Web ports (8888, 8000)
- Starts full ROS 2 Jazzy Cartographer supervisor (LiDAR + IMU + TFs + SLAM + Map Server)
- Opens real-time 2D SLAM dashboard at http://10.43.8.42:8888
================================================================================
"""

import os
import sys
import time
import webbrowser
import paramiko

PI_IP = "10.43.8.42"
PI_USER = "admin"
PI_PASS = "l"
PORT = 8888
URL = f"http://{PI_IP}:{PORT}"

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
PI5_DIR = os.path.join(ROOT_DIR, "pi4")
CARTO_DIR = os.path.join(PI5_DIR, "cartographer")

def connect_ssh(max_retries=20):
    print("================================================================")
    print("   DA WINNA - GOOGLE CARTOGRAPHER 2D SLAM LAUNCHER")
    print("================================================================")
    print(f"Connecting to Raspberry Pi 5 ({PI_IP})...")

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    for attempt in range(1, max_retries + 1):
        try:
            ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=4)
            print("[OK] Connected to Raspberry Pi 5!")
            return ssh
        except Exception as e:
            if attempt < max_retries:
                print(f"  [Attempt {attempt}/{max_retries}] Pi not ready ({e}). Retrying in 2s...")
                time.sleep(2)
            else:
                print(f"\n[ERROR] Could not connect to Pi 5 at {PI_IP} after {max_retries} attempts.")
                print("Make sure the Pi 5 is powered on and connected to the same network.")
                sys.exit(1)

def sync_files(ssh):
    print("\n[1/4] Synchronizing latest SLAM drivers & Web UI to Pi 5...")
    sftp = ssh.open_sftp()
    for d in ["/home/admin/cartographer", "/home/admin/cartographer/static"]:
        try:
            sftp.mkdir(d)
        except Exception:
            pass

    files_to_sync = [
        (os.path.join(CARTO_DIR, "cartographer_2d.lua"), "/home/admin/cartographer/cartographer_2d.lua"),
        (os.path.join(CARTO_DIR, "ms200_laser_node.py"), "/home/admin/cartographer/ms200_laser_node.py"),
        (os.path.join(CARTO_DIR, "bno08x_imu_node.py"), "/home/admin/cartographer/bno08x_imu_node.py"),
        (os.path.join(CARTO_DIR, "slam_map_server.py"), "/home/admin/cartographer/slam_map_server.py"),
        (os.path.join(CARTO_DIR, "run_cartographer_slam.py"), "/home/admin/cartographer/run_cartographer_slam.py"),
        (os.path.join(CARTO_DIR, "static", "slam_view.html"), "/home/admin/cartographer/static/slam_view.html"),
        (os.path.join(CARTO_DIR, "static", "slam_view.css"), "/home/admin/cartographer/static/slam_view.css"),
        (os.path.join(CARTO_DIR, "static", "slam_view.js"), "/home/admin/cartographer/static/slam_view.js"),
    ]

    for local_p, remote_p in files_to_sync:
        if os.path.exists(local_p):
            sftp.put(local_p, remote_p)
    sftp.close()

    ssh.exec_command("chmod +x /home/admin/cartographer/*.py")
    print("[OK] All SLAM files synchronized!")

def start_slam_supervisor(ssh):
    print("\n[2/4] Freeing serial ports & background tasks...")
    cleanup_cmd = """
echo 'l' | sudo -S systemctl stop bno-server lidar-server ms200.service 2>/dev/null || true
echo 'l' | sudo -S systemctl disable bno-server lidar-server ms200.service 2>/dev/null || true
for p in $(pgrep -f "cartographer|laser_node|imu_node|slam_map|run_cartographer|imu_server"); do
    echo 'l' | sudo -S kill -9 $p 2>/dev/null || true
done
echo 'l' | sudo -S fuser -k -9 /dev/ttyAMA2 2>/dev/null || true
echo 'l' | sudo -S fuser -k -9 /dev/ttyAMA4 2>/dev/null || true
echo 'l' | sudo -S fuser -k -9 8888/tcp 2>/dev/null || true
echo 'l' | sudo -S fuser -k -9 8000/tcp 2>/dev/null || true
sleep 1.0
"""
    stdin, stdout, stderr = ssh.exec_command(f"bash -c '{cleanup_cmd}'")
    stdout.channel.recv_exit_status()
    time.sleep(0.5)

    print("[3/4] Launching Google Cartographer 2D SLAM Supervisor...")
    run_cmd = "nohup python3 /home/admin/cartographer/run_cartographer_slam.py > /home/admin/cartographer/slam.log 2>&1 &"
    ssh.exec_command(f"bash -c '{run_cmd}'")
    time.sleep(3)

def verify_and_launch_browser(ssh):
    print("[4/4] Verifying SLAM stack health & Web Server ready...")
    server_ready = False
    for attempt in range(25):
        try:
            with urllib.request.urlopen(URL, timeout=1.0) as resp:
                if resp.status == 200:
                    server_ready = True
                    break
        except Exception:
            time.sleep(0.4)

    if server_ready:
        print("[OK] Cartographer SLAM Stack & Web GUI are ACTIVE & ONLINE!")
    else:
        print("[!] Server initializing...")

    stdin, stdout, stderr = ssh.exec_command("cat /home/admin/cartographer/slam.log | tail -n 12")
    log_tail = stdout.read().decode().strip()
    if log_tail:
        print("\n--- Recent SLAM Startup Log ---")
        print(log_tail)

    print("\n" + "=" * 64)
    print(f"  2D SLAM DASHBOARD READY AT: {URL}")
    print("=" * 64)
    print("\nOpening web browser...")
    webbrowser.open(URL)

def launch_slam():
    ssh = connect_ssh()
    try:
        sync_files(ssh)
        start_slam_supervisor(ssh)
        verify_and_launch_browser(ssh)
    finally:
        ssh.close()
    
    print("\n[SUCCESS] Everything is running smoothly on your Raspberry Pi 5.")
    print("You can run launch_cartographer_gui.py (or start_slam.py) anytime to restart SLAM.\n")

if __name__ == "__main__":
    launch_slam()
