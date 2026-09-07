import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def test_16bit_page_latch():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  TESTING 4-BYTE / 16-BIT SPI PAGE REGISTER LATCH")
    print("======================================================================")

    script = r"""#!/bin/bash
python3 - << 'PYEOF'
import spidev, time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0

def rd(reg, size=4):
    tx = [(reg >> 8) & 0x7F, reg & 0xFF] + [0x00] * size
    rx = spi.xfer2(tx)
    return rx[2:]

def wr16(reg, val):
    tx = [((reg >> 8) & 0x7F) | 0x80, reg & 0xFF, (val >> 8) & 0xFF, val & 0xFF]
    spi.xfer2(tx)

def wr8(reg, val):
    tx = [((reg >> 8) & 0x7F) | 0x80, reg & 0xFF, 0x00, val]
    spi.xfer2(tx)

# Test wr8 and wr16 on 0x7FFF
wr8(0x7FFF, 0x00)
time.sleep(0.005)
print("After wr8(0x7FFF, 0x00) -> 0x7FFF reads:", [f"0x{b:02X}" for b in rd(0x7FFF, 2)])
print("                           0x2C00 reads:", [f"0x{b:02X}" for b in rd(0x2C00, 4)])

wr8(0x7FFF, 0x02)
time.sleep(0.005)
print("After wr8(0x7FFF, 0x02) -> 0x7FFF reads:", [f"0x{b:02X}" for b in rd(0x7FFF, 2)])
print("                           0x2C00 reads:", [f"0x{b:02X}" for b in rd(0x2C00, 4)])

spi.close()
PYEOF
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/test_page_latch.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/test_page_latch.sh && /tmp/test_page_latch.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_16bit_page_latch()
