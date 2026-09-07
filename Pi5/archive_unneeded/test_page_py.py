import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def test_page_switch_in_python():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  TESTING PAGE SWITCH ON SPI IN PYTHON DIRECTLY")
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

def wr(reg, val):
    tx = [((reg >> 8) & 0x7F) | 0x80, reg & 0xFF, val]
    spi.xfer2(tx)

# Check Page 2
wr(0x7FFF, 0x02)
time.sleep(0.001)
p2_crc = rd(0x2FFC, 4)
print("Page 2 CRC (0x2FFC):", [f"0x{b:02X}" for b in p2_crc])
p2_2c00 = rd(0x2C00, 4)
print("Page 2 @ 0x2C00:   ", [f"0x{b:02X}" for b in p2_2c00])

# Switch to Page 0
wr(0x7FFF, 0x00)
time.sleep(0.001)
p0_2c00 = rd(0x2C00, 4)
print("Page 0 @ 0x2C00:   ", [f"0x{b:02X}" for b in p0_2c00])

# Switch to Page 1
wr(0x7FFF, 0x01)
time.sleep(0.001)
p1_2c00 = rd(0x2C00, 4)
print("Page 1 @ 0x2C00:   ", [f"0x{b:02X}" for b in p1_2c00])

spi.close()
PYEOF
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/test_page_py.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/test_page_py.sh && /tmp/test_page_py.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_page_switch_in_python()
