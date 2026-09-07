#!/usr/bin/env python3
"""
================================================================================
  DA WINNA - BNO08X IMU 3D TEST SERVER & GUI LAUNCHER (ESP32 UART -> PI 5)
================================================================================
Stops Cartographer SLAM, starts the ESP32 UART IMU 3D Visualizer Server
on the Raspberry Pi 5 (Port 8000), and opens your browser.
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
PORT = 8000
URL = f"http://{PI_IP}:{PORT}"

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PI5_DIR = os.path.join(ROOT_DIR, "Pi5")

def connect_ssh(max_retries=10):
    print(f"Connecting to Raspberry Pi 5 ({PI_IP})...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    for attempt in range(1, max_retries + 1):
        try:
            ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=8, banner_timeout=15)
            print("[OK] Connected to Raspberry Pi 5!")
            return ssh
        except Exception as e:
            if attempt < max_retries:
                print(f"  [Attempt {attempt}/{max_retries}] Connecting ({e}). Retrying in 2s...")
                time.sleep(2)
            else:
                print(f"[ERROR] Could not connect to Pi 5: {e}")
                sys.exit(1)

def launch_imu_server():
    print("=" * 64)
    print("   DA WINNA - ESP32 UART IMU 3D TEST SERVER LAUNCHER")
    print("=" * 64)
    ssh = connect_ssh()

    print("\n[1/3] Stopping Cartographer & freeing serial ports...")
    stop_cmd = """
echo 'l' | sudo -S systemctl stop bno-server lidar-server ms200.service 2>/dev/null || true
echo 'l' | sudo -S dtoverlay uart2-pi5 2>/dev/null || true
pkill -9 -f cartographer 2>/dev/null || true
pkill -9 -f laser_node 2>/dev/null || true
pkill -9 -f imu_node 2>/dev/null || true
pkill -9 -f slam_map_server 2>/dev/null || true
pkill -9 -f run_cartographer_slam 2>/dev/null || true
echo 'l' | sudo -S pkill -9 -f imu_server 2>/dev/null || true
echo 'l' | sudo -S fuser -k -9 8000/tcp 2>/dev/null || true
echo 'l' | sudo -S fuser -k -9 /dev/ttyAMA2 2>/dev/null || true
sleep 1.0
"""
    stdin, stdout, stderr = ssh.exec_command(f"bash -c '{stop_cmd}'")
    stdout.channel.recv_exit_status()
    time.sleep(0.5)

    print("[2/3] Synchronizing ESP32 UART IMU Bridge & 3D Web Dashboard files...")
    sftp = ssh.open_sftp()
    try:
        sftp.mkdir("/home/admin/static")
    except Exception:
        pass

    sftp.put(os.path.join(PI5_DIR, "imu", "imu_uart_server.py"), "/home/admin/imu_server.py")
    
    static_local = os.path.join(PI5_DIR, "static")
    if os.path.exists(static_local):
        for f in os.listdir(static_local):
            lp = os.path.join(static_local, f)
            if os.path.isfile(lp):
                sftp.put(lp, f"/home/admin/static/{f}")
    sftp.close()
    print("[OK] Files synchronized!")

    print("[3/3] Starting IMU 3D Test Server (Port 8000)...")
    run_cmd = "nohup python3 /home/admin/imu_server.py > /home/admin/imu_server.log 2>&1 &"
    ssh.exec_command(f"bash -c '{run_cmd}'")
    time.sleep(2)

    stdin, stdout, stderr = ssh.exec_command("ps aux | grep imu_server | grep -v grep")
    procs = stdout.read().decode().strip()
    if "imu_server" in procs:
        print("[OK] IMU 3D Test Server is RUNNING!")
    else:
        stdin, stdout, stderr = ssh.exec_command("cat /home/admin/imu_server.log | tail -n 10")
        print("Startup log:\n", stdout.read().decode())

    ssh.close()

    print("\n" + "=" * 64)
    print(f"  IMU 3D TEST DASHBOARD READY AT: {URL}")
    print("=" * 64)
    print("\nOpening web browser...")
    webbrowser.open(URL)
    print("\n[SUCCESS] IMU GUI is now active. When you want to return to SLAM, run start_slam.py.\n")

if __name__ == "__main__":
    launch_imu_server()
