import socket
import concurrent.futures
import paramiko

def check_ip(ip):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        res = s.connect_ex((ip, 22))
        s.close()
        if res == 0:
            print(f"Open Port 22 on {ip}! Testing SSH auth...")
            try:
                ssh = paramiko.SSHClient()
                ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                ssh.connect(ip, port=22, username="admin", password="l", timeout=3)
                stdin, stdout, stderr = ssh.exec_command("hostname -I")
                out = stdout.read().decode().strip()
                ssh.close()
                return f"SUCCESS: Pi 5 is at {ip} (all IPs: {out})"
            except Exception as e:
                return f"Port 22 open on {ip}, but SSH error: {e}"
    except:
        pass
    return None

ips = [f"10.43.8.{i}" for i in range(1, 255)]
print(f"Scanning {len(ips)} IPs on 10.43.8.x subnet...")

with concurrent.futures.ThreadPoolExecutor(max_workers=50) as executor:
    results = executor.map(check_ip, ips)
    for r in results:
        if r:
            print(">>>", r)
