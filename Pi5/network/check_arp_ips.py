import paramiko
import socket

ips = ["192.168.0.37", "192.168.0.54", "192.168.0.103", "192.168.0.121", "192.168.0.207", "192.168.0.2"]

for ip in ips:
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(ip, port=22, username="admin", password="l", timeout=1.5)
        print(f"FOUND PI AT: {ip}!")
        stdin, stdout, stderr = ssh.exec_command("hostname -I")
        print("Pi IP(s):", stdout.read().decode())
        ssh.close()
        break
    except Exception as e:
        print(f"{ip}: {e}")
