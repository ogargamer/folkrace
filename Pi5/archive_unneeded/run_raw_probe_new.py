import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def raw_spi_probe():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "192.168.0.13"
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)

    print("======================================================================")
    print("  RAW SPI DIAGNOSTIC: READ CHIP ID AT 192.168.0.13")
    print("======================================================================")

    script = r"""#!/bin/bash
python3 - << 'PYEOF'
import spidev, time

for speed in [500000, 1000000, 2000000, 3000000]:
    try:
        spi = spidev.SpiDev()
        spi.open(0, 0)
        spi.mode = 0
        spi.max_speed_hz = speed

        # Read reg 0x0000 (8 bytes)
        tx = [0x00, 0x00] + [0x00] * 6
        rx = spi.xfer2(tx)
        rx_hex = [f"0x{b:02X}" for b in rx]
        print(f"Speed {speed//1000} kHz -> rx: {rx_hex}")

        spi.close()
    except Exception as e:
        print(f"Error at {speed}: {e}")
PYEOF
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/raw_probe.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/raw_probe.sh && /tmp/raw_probe.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    raw_spi_probe()
