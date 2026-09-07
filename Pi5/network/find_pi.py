import socket
import paramiko
import sys
import concurrent.futures

sys.stdout.reconfigure(encoding='utf-8')

candidates = ["192.168.0.13", "10.43.8.42", "robot.local", "raspberrypi.local"]

# Also scan local subnets if needed
def check_ssh(ip):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(1.0)
    try:
        s.connect((ip, 22))
        s.close()
        return ip
    except:
        return None

def find_pi():
    print("Testing known IPs/hostnames...")
    with concurrent.futures.ThreadPoolExecutor(max_workers=10) as executor:
        results = executor.map(check_ssh, candidates)
        for r in results:
            if r:
                print(f"Found active SSH port on: {r}")
                return r
    print("Could not reach known IPs directly.")
    return None

if __name__ == "__main__":
    find_pi()
