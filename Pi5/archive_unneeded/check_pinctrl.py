import paramiko
import sys

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=3)
stdin, stdout, stderr = ssh.exec_command("pinctrl")
for line in stdout:
    for target in [" 4:", " 5:", " 12:", " 13:", " 14:", " 15:", " 16:", " 17:"]:
        if target in line:
            print(line.strip())
ssh.close()
