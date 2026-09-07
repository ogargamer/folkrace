#!/usr/bin/env python3
"""
Deploy Cartographer 2D SLAM suite to Raspberry Pi 5
"""

import paramiko
import os
import sys

PI_IP = "10.43.8.42"
PI_USER = "admin"
PI_PASS = "l"

def deploy():
    print(f"Connecting to Raspberry Pi 5 ({PI_IP})...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=5)

    sftp = ssh.open_sftp()
    
    # Create directories
    for d in ["/home/admin/cartographer", "/home/admin/cartographer/static"]:
        try:
            sftp.mkdir(d)
        except Exception:
            pass

    base_dir = os.path.dirname(os.path.abspath(__file__))
    files = [
        ("cartographer_2d.lua", "/home/admin/cartographer/cartographer_2d.lua"),
        ("cartographer_2d.launch.py", "/home/admin/cartographer/cartographer_2d.launch.py"),
        ("ms200_laser_node.py", "/home/admin/cartographer/ms200_laser_node.py"),
        ("bno08x_imu_node.py", "/home/admin/cartographer/bno08x_imu_node.py"),
        ("slam_map_server.py", "/home/admin/cartographer/slam_map_server.py"),
        ("static/slam_view.html", "/home/admin/cartographer/static/slam_view.html"),
        ("static/slam_view.css", "/home/admin/cartographer/static/slam_view.css"),
        ("static/slam_view.js", "/home/admin/cartographer/static/slam_view.js"),
    ]

    print("Uploading Cartographer 2D files...")
    for local_name, remote_path in files:
        local_path = os.path.join(base_dir, local_name)
        if os.path.exists(local_path):
            sftp.put(local_path, remote_path)
            print(f"  -> Uploaded: {local_name}")

    sftp.close()

    # Make executable
    ssh.exec_command("chmod +x /home/admin/cartographer/*.py")
    print(">> Cartographer 2D Files Deployed Successfully! <<")
    ssh.close()

if __name__ == "__main__":
    deploy()
