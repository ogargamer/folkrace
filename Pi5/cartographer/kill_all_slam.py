import paramiko
import time

s = paramiko.SSHClient()
s.set_missing_host_key_policy(paramiko.AutoAddPolicy())
s.connect('10.43.8.42', port=22, username='admin', password='l', timeout=4)

kill_cmd = """
for p in $(pgrep -f "cartographer|laser_node|imu_node|slam_map|run_cartographer|imu_server"); do
    echo "Killing PID $p..."
    echo 'l' | sudo -S kill -9 $p 2>/dev/null || true
done
echo 'l' | sudo -S fuser -k -9 /dev/ttyAMA2 2>/dev/null || true
echo 'l' | sudo -S fuser -k -9 /dev/ttyAMA4 2>/dev/null || true
echo 'l' | sudo -S fuser -k -9 8888/tcp 2>/dev/null || true
echo 'l' | sudo -S fuser -k -9 8000/tcp 2>/dev/null || true
sleep 1.5
"""
stdin, stdout, stderr = s.exec_command(f"bash -c '{kill_cmd}'")
print(stdout.read().decode('utf-8', errors='ignore'))

stdin, stdout, stderr = s.exec_command('ps aux | grep -E "cartographer|laser_node|imu_node|slam_map_server" | grep -v grep')
print("Remaining:", stdout.read().decode('utf-8', errors='ignore'))
s.close()
