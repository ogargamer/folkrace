import paramiko

s = paramiko.SSHClient()
s.set_missing_host_key_policy(paramiko.AutoAddPolicy())
s.connect('10.43.8.42', port=22, username='admin', password='l', timeout=4)

stdin, stdout, stderr = s.exec_command('cat /home/admin/cartographer/slam.log | tail -n 25')
print("=== SLAM SUPERVISOR LOG ===")
print(stdout.read().decode('utf-8', errors='ignore'))

stdin, stdout, stderr = s.exec_command('ps aux | grep -E "cartographer|laser_node|imu_node|slam_map_server" | grep -v grep')
print("=== ACTIVE SLAM PROCESSES ===")
print(stdout.read().decode('utf-8', errors='ignore'))

s.close()
