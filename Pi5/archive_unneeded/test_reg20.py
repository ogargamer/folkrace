import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def test_spi_reg20():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  TESTING SPI REG 0x0020 VS 0x0021 FOR FW ACCESS READY (0x04)")
    print("======================================================================")

    script = r"""#!/bin/bash
python3 - << 'PYEOF'
import spidev, time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0

def rd(reg, size=2):
    tx = [(reg >> 8) & 0x7F, reg & 0xFF] + [0x00] * size
    rx = spi.xfer2(tx)
    return rx[2:]

def wr(reg, val):
    tx = [((reg >> 8) & 0x7F) | 0x80, reg & 0xFF, val]
    spi.xfer2(tx)

# 1. Reset
wr(0x7FFF, 0x00)
wr(0x0009, 0x04)
wr(0x000F, 0x40)
wr(0x000A, 0x03)
rd(0x7FFF, 1)
wr(0x000C, 0x01)
wr(0x0101, 0x00)
wr(0x0102, 0x00)
wr(0x010A, 0x01)
wr(0x4002, 0x01)
wr(0x4002, 0x00)
wr(0x010A, 0x03)
wr(0x0103, 0x01)
wr(0x000C, 0x00)
wr(0x000F, 0x43)
time.sleep(0.001)
wr(0x000F, 0x40)
wr(0x000A, 0x01)
time.sleep(0.1)

wr(0x7FFF, 0x00)
print("Boot status 0x0006:", rd(0x0006, 2))

# 2. Enable FW access
wr(0x000E, 0x01)
wr(0x7FFF, 0x02)
wr(0x7FFF, 0x01)
wr(0x0006, 0x01)
time.sleep(0.01)

# Check both 0x0020 and 0x0021
print("Read 0x0020 (2 bytes):", [f"0x{b:02X}" for b in rd(0x0020, 2)])
print("Read 0x0021 (2 bytes):", [f"0x{b:02X}" for b in rd(0x0021, 2)])

spi.close()
PYEOF
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/test_reg20.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/test_reg20.sh && /tmp/test_reg20.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_spi_reg20()
