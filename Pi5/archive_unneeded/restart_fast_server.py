import paramiko
import os
import time

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=4)

sftp = ssh.open_sftp()
base_dir = os.path.dirname(os.path.abspath(__file__))
carto_dir = os.path.join(base_dir, "cartographer")

sftp.put(os.path.join(carto_dir, "slam_map_server.py"), "/home/admin/cartographer/slam_map_server.py")
sftp.put(os.path.join(carto_dir, "static", "slam_view.js"), "/home/admin/cartographer/static/slam_view.js")
sftp.put(os.path.join(carto_dir, "static", "slam_view.html"), "/home/admin/cartographer/static/slam_view.html")
sftp.close()

print("Restarting slam_map_server.py...")
ssh.exec_command("pkill -9 -f slam_map_server.py 2>/dev/null || true")
time.sleep(1)
ssh.exec_command("bash -c 'source /opt/ros/jazzy/setup.bash && nohup python3 /home/admin/cartographer/slam_map_server.py > /home/admin/cartographer/web.log 2>&1 &'")
time.sleep(2)

stdin, stdout, stderr = ssh.exec_command("cat /home/admin/cartographer/web.log")
print(stdout.read().decode())

ssh.close()
