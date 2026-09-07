import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def test_spi_exact_protocol():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "192.168.0.13"
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)

    print("======================================================================")
    print("  TESTING EXACT SPI PROTOCOL & FIRMWARE UPLOAD IN PYTHON")
    print("======================================================================")

    script = r"""#!/bin/bash
python3 - << 'PYEOF'
import spidev, time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0

def rd(reg, size=4):
    # READ: 2 bytes address, data arrives at rx[2:]
    tx = [(reg >> 8) & 0x7F, reg & 0xFF] + [0x00] * size
    rx = spi.xfer2(tx)
    return rx[2:]

def wr(reg, data):
    if isinstance(data, int):
        data = [data]
    # WRITE: bit 15 set = 0x80
    tx = [((reg >> 8) & 0x7F) | 0x80, reg & 0xFF] + data
    spi.xfer2(tx)

print("1. Read 0x0000 (Chip ID):", [f"0x{b:02X}" for b in rd(0x0000, 4)])

# SW reboot sequence:
print("2. Sending SW Reboot sequence...")
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
time.sleep(0.002)
wr(0x000F, 0x40)
wr(0x000A, 0x01)
time.sleep(0.1)

print("3. Polling 0x0006 after reboot...")
wr(0x7FFF, 0x00)
for i in range(10):
    r = rd(0x0006, 2)
    print(f"   Poll {i}: {r}")
    if r[0] == 1 or r[1] == 1:
        print("   -> BOOTLOADER READY!")
        break
    time.sleep(0.01)

spi.close()
PYEOF
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/test_py_proto.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/test_py_proto.sh && /tmp/test_py_proto.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_spi_exact_protocol()
