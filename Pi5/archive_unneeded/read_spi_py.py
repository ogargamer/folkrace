import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=3)

stdin, stdout, stderr = ssh.exec_command("cat /tmp/bno08x_repo/lib/spi.py")
print(stdout.read().decode('utf-8', errors='ignore'))
ssh.close()
