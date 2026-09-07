import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def test_sensor_ranging():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)
    except Exception as e:
        print(f"SSH Connection failed: {e}")
        return

    print("======================================================================")
    print("  TESTING SWAPPED VL53L8CX SENSOR WITH ORIGINAL DRIVER")
    print("======================================================================")

    # First ensure GPIO 17 (Pin 11 / LPn) is HIGH and run official test
    script = r"""#!/bin/bash
echo l | sudo -S pinctrl set 17 op dh 2>/dev/null || true
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

# Run run_spi1_official which captures 10 frames and prints full 4x4 matrix
./run_spi1_official
"""

    stdin, stdout, stderr = ssh.exec_command(script, get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_sensor_ranging()
