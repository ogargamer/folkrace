import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def setup_spi1_and_test_lpn():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  CONFIGURING SPI1 OVERLAY & DRIVING PIN 11 (GPIO 17) HIGH FOR LPN")
    print("======================================================================")

    script = r"""#!/bin/bash
echo l | sudo -S pinctrl set 17 op dh || true

if ! grep -q "dtoverlay=spi1-1cs" /boot/firmware/config.txt; then
    echo "Adding dtoverlay=spi1-1cs to /boot/firmware/config.txt..."
    echo l | sudo -S sed -i '$a dtoverlay=spi1-1cs' /boot/firmware/config.txt
fi

echo l | sudo -S dtoverlay spi1-1cs || true

ls -la /dev/spidev*
pinctrl get 17
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/setup_spi1.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/setup_spi1.sh && /tmp/setup_spi1.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    setup_spi1_and_test_lpn()
