import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def inspect_spi1_bytes():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  INSPECTING SPI READ BYTES AT 0x0006 ON SPI1.0")
    print("======================================================================")

    script = r"""#!/bin/bash
python3 - << 'PYEOF'
import spidev, time

spi = spidev.SpiDev()
spi.open(1, 0)
spi.max_speed_hz = 1000000
spi.mode = 0

def rd(reg, size=4):
    tx = [(reg >> 8) & 0x7F, reg & 0xFF] + [0x00] * size
    rx = spi.xfer2(tx)
    return rx

def wr(reg, val):
    tx = [((reg >> 8) & 0x7F) | 0x80, reg & 0xFF, val]
    spi.xfer2(tx)

wr(0x7FFF, 0x00)
for reg in [0x0000, 0x0002, 0x0004, 0x0006, 0x0008, 0x000A]:
    rx = rd(reg, 4)
    print(f"Reg 0x{reg:04X}: tx=[0x{(reg>>8)&0x7F:02X}, 0x{reg&0xFF:02X}], rx={[hex(b) for b in rx]}")

spi.close()
PYEOF
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/inspect_spi1_bytes.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/inspect_spi1_bytes.sh && /tmp/inspect_spi1_bytes.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    inspect_spi1_bytes()
