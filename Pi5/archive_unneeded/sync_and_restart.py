import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=5)

sftp = ssh.open_sftp()
sftp.put(r"c:\Users\nurmi\Documents\Da Winna\Pi5\imu_server.py", "/home/admin/imu_server.py")
sftp.put(r"c:\Users\nurmi\Documents\Da Winna\Pi5\bno08x_i2c.py", "/home/admin/bno08x_i2c.py")
sftp.put(r"c:\Users\nurmi\Documents\Da Winna\Pi5\static\index.html", "/home/admin/static/index.html")
sftp.put(r"c:\Users\nurmi\Documents\Da Winna\Pi5\static\style.css", "/home/admin/static/style.css")
sftp.put(r"c:\Users\nurmi\Documents\Da Winna\Pi5\static\app.js", "/home/admin/static/app.js")
sftp.close()

print("Restarting bno-server systemd service...")
stdin, stdout, stderr = ssh.exec_command("echo 'l' | sudo -S systemctl restart bno-server")
print(stdout.read().decode())
print(stderr.read().decode())

ssh.close()
