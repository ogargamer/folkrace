import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def locate_all_16_distances():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  LOCATING EXACT 16 DISTANCES IN SRAM")
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

wr(0x7FFF, 0x00)

# Dump 1KB of SRAM and find 16-element arrays
sram = []
for r in range(0, 0x400, 32):
    sram.extend(rd(r, 32))

print(f"Total SRAM Dumped: {len(sram)} bytes")

# Look for consecutive 16-element int16 arrays
print("\nScanning 16-element arrays (int16_t):")
for offset in range(0, len(sram) - 32, 2):
    vals = [(sram[offset + i*2 + 1] << 8) | sram[offset + i*2] for i in range(16)]
    # Check if reasonable distances (e.g. 50mm to 3000mm)
    valid_count = sum(1 for v in vals if 50 <= (v//4) <= 3000 or 50 <= v <= 3000)
    if valid_count >= 2:
        print(f"Candidate Offset 0x{offset:04X}: {[v//4 if v > 4000 else v for v in vals]}")

spi.close()
PYEOF
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/locate_dist.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/locate_dist.sh && /tmp/locate_dist.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    locate_all_16_distances()
