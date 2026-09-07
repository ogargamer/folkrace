#!/usr/bin/env python3
"""
BNO08X IMU Dashboard Launcher for Windows
Connects to Raspberry Pi 5, ensures the IMU server is running, and opens the Web GUI.
"""

import time
import webbrowser
import paramiko
import os
import sys

PI_IP = "10.43.8.42"
PI_USER = "admin"
PI_PASS = "l"
PORT = 8000
URL = f"http://{PI_IP}:{PORT}"

def sync_and_launch():
    print("========================================================")
    print("   LAUNCHING BNO08X 3D MISSION DASHBOARD")
    print("========================================================")
    print(f"Connecting to Raspberry Pi 5 ({PI_IP})...")

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=4)
        print("Connected to Pi 5.")
    except Exception as e:
        print(f"[Error] Could not connect to Pi 5 at {PI_IP}: {e}")
        print("Make sure the Pi 5 is powered on and connected to the network.")
        sys.exit(1)

    sftp = ssh.open_sftp()
    
    # Create static dir if needed
    try:
        sftp.mkdir("/home/admin/static")
    except Exception:
        pass

    base_dir = os.path.dirname(os.path.abspath(__file__))
    files_to_sync = [
        ("Pi5/bno08x_i2c.py", "/home/admin/bno08x_i2c.py"),
        ("Pi5/imu_server.py", "/home/admin/imu_server.py"),
        ("Pi5/static/index.html", "/home/admin/static/index.html"),
        ("Pi5/static/style.css", "/home/admin/static/style.css"),
        ("Pi5/static/app.js", "/home/admin/static/app.js"),
    ]

    print("Synchronizing latest dashboard files to Pi 5...")
    for local_rel, remote_path in files_to_sync:
        local_path = os.path.join(base_dir, local_rel)
        if os.path.exists(local_path):
            sftp.put(local_path, remote_path)
    sftp.close()

    print("Starting IMU Telemetry Server on Pi 5 (Port 8000)...")
    # Kill any existing server instance cleanly and start fresh
    ssh.exec_command("pkill -f imu_server.py ; sleep 0.5 ; nohup python3 /home/admin/imu_server.py > /home/admin/imu_server.log 2>&1 &")

    time.sleep(1.5)
    ssh.close()

    print(f"\n>> Opening browser at: {URL} <<\n")
    webbrowser.open(URL)
    print("Dashboard launched successfully! Press Ctrl+C to exit.")

if __name__ == "__main__":
    sync_and_launch()
