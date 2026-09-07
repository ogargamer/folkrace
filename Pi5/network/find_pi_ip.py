import socket
import sys
import paramiko

sys.stdout.reconfigure(encoding='utf-8')

def find_pi_and_connect():
    print("======================================================================")
    print("  FINDING PI 5 IP ADDRESS ACROSS HOSTNAMES & SUBNETS")
    print("======================================================================")

    # Possible hostnames
    candidates = [
        "10.43.8.42",
        "Robot.local",
        "raspberrypi.local",
        "robot",
        "raspberrypi"
    ]

    for host in candidates:
        try:
            ip = socket.gethostbyname(host)
            print(f"Candidate {host} -> Resolved IP: {ip}")
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(ip, port=22, username="admin", password="l", timeout=3)
            print(f"Connected successfully to Pi 5 at {ip} ({host})!")
            stdin, stdout, stderr = ssh.exec_command("ip a show wlan0")
            print(stdout.read().decode())
            ssh.close()
            return ip
        except Exception as e:
            print(f"Could not connect to {host}: {e}")

    # If hostnames fail, scan local subnet
    print("\nScanning local subnet for Pi 5...")
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        my_ip = s.getsockname()[0]
        s.close()
        prefix = ".".join(my_ip.split(".")[:3])
        print(f"My PC IP: {my_ip}, scanning subnet {prefix}.0/24...")

        for last in range(1, 255):
            target = f"{prefix}.{last}"
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(0.1)
                if sock.connect_ex((target, 22)) == 0:
                    print(f"Found open SSH on {target}! Testing auth...")
                    try:
                        ssh = paramiko.SSHClient()
                        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                        ssh.connect(target, port=22, username="admin", password="l", timeout=2)
                        print(f"SUCCESS! Pi 5 found at: {target}")
                        ssh.close()
                        return target
                    except:
                        pass
                sock.close()
            except:
                pass
    except Exception as e:
        print(f"Subnet scan error: {e}")

if __name__ == "__main__":
    find_pi_and_connect()
