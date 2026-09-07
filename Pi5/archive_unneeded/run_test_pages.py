import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def test_rdmulti_and_pages():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "192.168.0.13"
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)

    print("======================================================================")
    print("  TESTING RdMulti & PAGES ON 192.168.0.13")
    print("======================================================================")

    script = r"""#!/bin/bash
python3 - << 'PYEOF'
import spidev, time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.mode = 0
spi.max_speed_hz = 1000000

def rd(reg, size=1):
    tx = [(reg >> 8) & 0x7F, reg & 0xFF] + [0] * size
    rx = spi.xfer2(tx)
    return rx[2:]

def wr(reg, val):
    tx = [((reg >> 8) & 0x7F) | 0x80, reg & 0xFF, val]
    spi.xfer2(tx)

# Test 1: Page 0
wr(0x7FFF, 0x00)
time.sleep(0.01)
id_page0 = rd(0x0000, 4)
print("Page 0 @ 0x0000 (4 bytes):", [f"0x{b:02X}" for b in id_page0])

# Test 2: Page 2
wr(0x7FFF, 0x02)
time.sleep(0.01)
id_page2 = rd(0x0000, 4)
print("Page 2 @ 0x0000 (4 bytes):", [f"0x{b:02X}" for b in id_page2])

# Test 3: Check reg 0x0006 on page 0
wr(0x7FFF, 0x00)
time.sleep(0.01)
r06 = rd(0x0006, 2)
print("Page 0 @ 0x0006:", [f"0x{b:02X}" for b in r06])

spi.close()
PYEOF
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/test_pages.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/test_pages.sh && /tmp/test_pages.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_rdmulti_and_pages()
