import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=5)

service_content = """[Unit]
Description=MS200 LiDAR 2D Web Visualizer Service
After=network.target

[Service]
Type=simple
User=admin
WorkingDirectory=/home/admin
ExecStart=/usr/bin/python3 /home/admin/lidar_server.py
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
"""

# Write to /tmp/ms200.service via SFTP
sftp = ssh.open_sftp()
with sftp.file("/tmp/ms200.service", "w") as f:
    f.write(service_content)
sftp.close()

cmd = """
echo 'l' | sudo -S cp /tmp/ms200.service /etc/systemd/system/ms200.service
echo 'l' | sudo -S chmod 644 /etc/systemd/system/ms200.service
echo 'l' | sudo -S systemctl daemon-reload
echo 'l' | sudo -S systemctl enable ms200.service
echo 'l' | sudo -S systemctl restart ms200.service
sleep 1
echo 'l' | sudo -S systemctl status ms200.service --no-pager
"""

stdin, stdout, stderr = ssh.exec_command(cmd, get_pty=True)
print(stdout.read().decode())
ssh.close()
