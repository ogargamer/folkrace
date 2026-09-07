import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=4)

cmd = """
echo 'l' | sudo -S systemctl stop bno-server lidar-server 2>/dev/null || true
echo 'l' | sudo -S systemctl disable bno-server lidar-server 2>/dev/null || true
pkill -9 -f imu_server 2>/dev/null || true
pkill -9 -f lidar_server 2>/dev/null || true
pkill -9 -f cartographer 2>/dev/null || true
pkill -9 -f laser_node 2>/dev/null || true
pkill -9 -f imu_node 2>/dev/null || true
pkill -9 -f slam_map_server 2>/dev/null || true
pkill -9 -f run_cartographer_slam 2>/dev/null || true
sleep 1.5

nohup python3 /home/admin/cartographer/run_cartographer_slam.py > /home/admin/cartographer/slam.log 2>&1 &
sleep 2.5
"""

stdin, stdout, stderr = ssh.exec_command(cmd)
stdin.close()
print(stdout.read().decode())
ssh.close()
print("Disabled old services and restarted Cartographer SLAM cleanly!")
