import socket
import concurrent.futures
import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def check_ip(ip):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.4)
        if s.connect_ex((ip, 22)) == 0:
            s.close()
            # Try SSH auth
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            try:
                ssh.connect(ip, port=22, username="admin", password="l", timeout=1.5)
                ssh.close()
                return (ip, True)
            except Exception as e:
                return (ip, False)
        s.close()
    except:
        pass
    return None

def scan_all():
    print("Finding local network IPs...")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        my_ip = s.getsockname()[0]
    finally:
        s.close()
        
    print(f"Host IP: {my_ip}")
    prefix = ".".join(my_ip.split(".")[:3])
    targets = [f"{prefix}.{i}" for i in range(1, 255)]
    
    print(f"Scanning 254 IPs on {prefix}.0/24 with 50 threads...")
    found = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=50) as executor:
        results = executor.map(check_ip, targets)
        for res in results:
            if res:
                ip, is_pi = res
                print(f"Port 22 open on {ip} (Admin SSH: {is_pi})")
                if is_pi:
                    found.append(ip)
                    
    print(f"Found Pi 5 at: {found}")
    return found

if __name__ == "__main__":
    scan_all()
