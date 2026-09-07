import socket
import concurrent.futures
import paramiko

def check_ip(ip):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.3)
        if s.connect_ex((ip, 22)) == 0:
            s.close()
            # Try SSH
            try:
                ssh = paramiko.SSHClient()
                ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                ssh.connect(ip, port=22, username="admin", password="l", timeout=2)
                stdin, stdout, stderr = ssh.exec_command("hostname -I")
                out = stdout.read().decode().strip()
                ssh.close()
                return f"FOUND PI 5 at {ip} (IPs: {out})"
            except Exception as e:
                return f"SSH open on {ip} (Auth/other: {e})"
        s.close()
    except:
        pass
    return None

ips = [f"192.168.0.{i}" for i in range(1, 255)] + [f"10.43.8.{i}" for i in range(1, 255)]

print(f"Scanning {len(ips)} IPs on local subnets for Pi 5 SSH...")
with concurrent.futures.ThreadPoolExecutor(max_workers=100) as executor:
    results = executor.map(check_ip, ips)
    for r in results:
        if r:
            print(">>>", r)
