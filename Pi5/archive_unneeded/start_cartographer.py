import paramiko
import os
import time

PI_IP = "10.43.8.42"
PI_USER = "admin"
PI_PASS = "l"

def start_cartographer():
    print(f"Connecting to Raspberry Pi 5 ({PI_IP})...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=5)

    # 1. Stop background servers holding the serial port & I2C
    print("Freeing serial port & I2C bus for Cartographer...")
    ssh.exec_command("echo 'l' | sudo -S systemctl stop bno-server 2>/dev/null || true")
    ssh.exec_command("pkill -9 -f cartographer 2>/dev/null || true")
    ssh.exec_command("pkill -9 -f laser_node 2>/dev/null || true")
    ssh.exec_command("pkill -9 -f imu_node 2>/dev/null || true")
    ssh.exec_command("pkill -9 -f slam_map_server 2>/dev/null || true")
    ssh.exec_command("pkill -9 -f run_cartographer_slam 2>/dev/null || true")
    ssh.exec_command("pkill -9 -f lidar_server.py 2>/dev/null || true")
    ssh.exec_command("pkill -9 -f imu_server.py 2>/dev/null || true")
    time.sleep(1.5)

    # 2. Upload latest cartographer files
    sftp = ssh.open_sftp()
    for d in ["/home/admin/cartographer", "/home/admin/cartographer/static"]:
        try:
            sftp.mkdir(d)
        except Exception:
            pass

    base_dir = os.path.dirname(os.path.abspath(__file__)) # this is in Pi5/
    carto_dir = os.path.join(base_dir, "cartographer")
    files = [
        ("cartographer_2d.lua", "/home/admin/cartographer/cartographer_2d.lua"),
        ("ms200_laser_node.py", "/home/admin/cartographer/ms200_laser_node.py"),
        ("bno08x_imu_node.py", "/home/admin/cartographer/bno08x_imu_node.py"),
        ("slam_map_server.py", "/home/admin/cartographer/slam_map_server.py"),
        ("run_cartographer_slam.py", "/home/admin/cartographer/run_cartographer_slam.py"),
        ("static/slam_view.html", "/home/admin/cartographer/static/slam_view.html"),
        ("static/slam_view.css", "/home/admin/cartographer/static/slam_view.css"),
        ("static/slam_view.js", "/home/admin/cartographer/static/slam_view.js"),
    ]
    # Also upload bno08x_i2c.py into cartographer dir
    sftp.put(os.path.join(base_dir, "bno08x_i2c.py"), "/home/admin/cartographer/bno08x_i2c.py")

    for local_name, remote_path in files:
        lp = os.path.join(carto_dir, local_name)
        if os.path.exists(lp):
            sftp.put(lp, remote_path)
    sftp.close()

    ssh.exec_command("chmod +x /home/admin/cartographer/*.py")

    # 3. Start Cartographer SLAM Stack
    print("Starting Google Cartographer 2D SLAM Supervisor...")
    cmd = "nohup python3 /home/admin/cartographer/run_cartographer_slam.py > /home/admin/cartographer/slam.log 2>&1 &"
    ssh.exec_command(f"bash -c '{cmd}'")

    time.sleep(3)
    stdin, stdout, stderr = ssh.exec_command("cat /home/admin/cartographer/slam.log")
    print("\n--- SLAM Supervisor Log ---")
    print(stdout.read().decode())

    ssh.close()

if __name__ == "__main__":
    start_cartographer()
