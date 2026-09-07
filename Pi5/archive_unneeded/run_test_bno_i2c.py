import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=5)

sftp = ssh.open_sftp()
sftp.put(r"c:\Users\nurmi\Documents\Da Winna\Pi5\bno08x_i2c.py", "/home/admin/bno08x_i2c.py")
sftp.close()

stdin, stdout, stderr = ssh.exec_command("python3 /home/admin/bno08x_i2c.py", get_pty=True)
for line in iter(stdout.readline, ""):
    print(line, end="")

ssh.close()
