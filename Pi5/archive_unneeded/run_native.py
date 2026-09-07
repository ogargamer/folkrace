import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=5)

sftp = ssh.open_sftp()
sftp.put(r"c:\Users\nurmi\Documents\Da Winna\Pi5\bno08x_native.cpp", "/home/admin/bno08x_native.cpp")
sftp.close()

print("Compiling native C++ BNO08X driver on Pi 5...")
stdin, stdout, stderr = ssh.exec_command("g++ -O3 /home/admin/bno08x_native.cpp -lgpiod -o /home/admin/bno08x_native && /home/admin/bno08x_native", get_pty=True)
for line in iter(stdout.readline, ""):
    print(line, end="")

ssh.close()
