import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def diagnose_tof_connection():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)
    except Exception as e:
        print(f"SSH connection error: {e}")
        return

    print("======================================================================")
    print("  DIAGNOSING TOF HARDWARE & SPI STATE ON PI 5")
    print("======================================================================")

    script = r"""#!/bin/bash
echo "==> Checking SPI device node..."
ls -l /dev/spidev* || true

echo ""
echo "==> Checking GPIO 17 state..."
pinctrl get 17 || true

echo ""
echo "==> Pulse LPn (GPIO 17) LOW -> HIGH..."
gpioset 4 17=0 2>/dev/null || true
sleep 0.1
gpioset 4 17=1 2>/dev/null || true
sleep 0.2

pinctrl get 17 || true

echo ""
echo "==> Probing sensor chip ID register (0x0000) over /dev/spidev0.0..."
python3 - << 'PYEOF'
import spidev, time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 2000000
spi.mode = 0

# Read reg 0x0000 (8 bytes)
rx = spi.xfer2([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
print("Read reg 0x0000:", [f"0x{b:02X}" for b in rx])

# Read reg 0x0006 (boot status)
rx_stat = spi.xfer2([0x00, 0x06, 0x00, 0x00, 0x00, 0x00])
print("Read reg 0x0006:", [f"0x{b:02X}" for b in rx_stat])

spi.close()
PYEOF

echo ""
echo "==> Running STSW-IMG042 multi..."
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test
echo "1" | ./multi || true
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/diag_tof.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/diag_tof.sh && /tmp/diag_tof.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    diagnose_tof_connection()
