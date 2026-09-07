import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def build_spi_i2c_bus_scanner():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  HARDWARE DIAGNOSTIC: PROBING BOTH SPI0 AND I2C-1 BUS ADDRESSES")
    print("======================================================================")

    script = """#!/bin/bash
set -e

echo "=== 1. Checking Linux I2C Bus Detection ==="
echo l | sudo -S i2cdetect -y 1 || true

echo "=== 2. Checking Linux SPI Bus Devices ==="
ls -la /dev/spidev*

echo "=== 3. Driving LPn (GPIO 17) LOW then HIGH ==="
gpioset 4 17=0 2>/dev/null || true
sleep 0.1
gpioset 4 17=1 2>/dev/null || true
sleep 0.1

echo "=== 4. Post-Power Cycle I2C Bus Scan ==="
echo l | sudo -S i2cdetect -y 1 || true
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/hw_diag.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/hw_diag.sh && /tmp/hw_diag.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    build_spi_i2c_bus_scanner()
