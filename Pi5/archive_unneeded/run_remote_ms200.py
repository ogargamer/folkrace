import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def run_lidar_test():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "10.43.8.42"
    print(f"Connecting to Pi 5 at {ip}...")
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)
    print("Connected successfully!")

    # Check serial devices and permissions
    print("\n--- Listing Serial Ports on Pi 5 ---")
    stdin, stdout, stderr = ssh.exec_command("ls -l /dev/ttyAMA* /dev/serial* /dev/ttyS* 2>/dev/null")
    print(stdout.read().decode())

    # Check UART overlay / config
    stdin, stdout, stderr = ssh.exec_command("grep -E 'uart|serial' /boot/firmware/config.txt /boot/firmware/cmdline.txt 2>/dev/null || grep -E 'uart|serial' /boot/config.txt /boot/cmdline.txt 2>/dev/null")
    print("UART boot config:")
    print(stdout.read().decode())

    # Upload test_ms200.py to Pi 5
    sftp = ssh.open_sftp()
    local_file = r"c:\Users\nurmi\Documents\Da Winna\Pi5\test_ms200.py"
    remote_file = "/home/admin/test_ms200.py"
    sftp.put(local_file, remote_file)
    sftp.close()

    # Run Python test on Pi 5
    print("\n================ TESTING MS200 LIDAR ON /dev/ttyAMA0 ================")
    stdin, stdout, stderr = ssh.exec_command("python3 /home/admin/test_ms200.py /dev/ttyAMA0", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    # Also check /dev/serial0 if different
    print("\n================ TESTING MS200 LIDAR ON /dev/serial0 ================")
    stdin, stdout, stderr = ssh.exec_command("python3 /home/admin/test_ms200.py /dev/serial0", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    ssh.close()

if __name__ == "__main__":
    run_lidar_test()
