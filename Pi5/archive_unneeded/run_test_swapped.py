import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "10.43.8.42"
PI_USER = "admin"
PI_PASS = "l"

def run_test():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  TESTING SWAPPED VL53L8CX SENSOR ON SPI1.0")
    print("======================================================================")

    script = r"""#!/bin/bash
# 1. Ensure LPn (Pin 11 / GPIO 17) is HIGH
echo l | sudo -S pinctrl set 17 op dh 2>/dev/null || true

# 2. Check /dev/spidev1.0 exists, if not load overlay
if [ ! -e /dev/spidev1.0 ]; then
    echo "Loading spi1-1cs overlay..."
    echo l | sudo -S dtoverlay spi1-1cs || true
fi

cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

# Run the official test binary configured for SPI1
./run_spi1_official
"""

    stdin, stdout, stderr = ssh.exec_command(script, get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    run_test()
