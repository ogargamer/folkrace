import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def test_imu_env():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "10.43.8.42"
    print(f"Connecting to Pi 5 at {ip}...")
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)
    print("Connected to Pi 5!")

    # Check SPI devices
    stdin, stdout, stderr = ssh.exec_command("ls -l /dev/spidev* 2>/dev/null")
    print("SPI Devices:\n", stdout.read().decode())

    # Install official Adafruit BNO08x library & Blinka for Pi
    print("Installing official Adafruit BNO08x driver from PyPI...")
    cmd_install = "pip install --break-system-packages adafruit-circuitpython-bno08x adafruit-blinka 2>&1"
    stdin, stdout, stderr = ssh.exec_command(cmd_install, get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    ssh.close()

if __name__ == "__main__":
    test_imu_env()
