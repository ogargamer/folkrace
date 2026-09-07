import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=5)

service_content = """[Unit]
Description=BNO08X IMU 3D Mission Dashboard Server
After=network.target

[Service]
Type=simple
User=admin
WorkingDirectory=/home/admin
ExecStart=/usr/bin/python3 /home/admin/imu_server.py
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
"""

print("Installing bno-server systemd service...")
sftp = ssh.open_sftp()
with sftp.file("/tmp/bno-server.service", "w") as f:
    f.write(service_content)
sftp.close()

stdin, stdout, stderr = ssh.exec_command("echo 'l' | sudo -S mv /tmp/bno-server.service /etc/systemd/system/bno-server.service && echo 'l' | sudo -S systemctl daemon-reload && echo 'l' | sudo -S systemctl restart bno-server && echo 'l' | sudo -S systemctl enable bno-server")
print(stdout.read().decode())
print(stderr.read().decode())

stdin, stdout, stderr = ssh.exec_command("systemctl status bno-server --no-pager")
print(stdout.read().decode())
ssh.close()
