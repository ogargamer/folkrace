import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "10.43.8.42"
PI_USER = "admin"
PI_PASS = "l"

def inspect_ranging_dci_packet():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  RAW PACKET INSPECTION OF RANGING DATA ON SPI1.0")
    print("======================================================================")

    script = r"""#!/bin/bash
python3 - << 'PYEOF'
import spidev, time

spi = spidev.SpiDev()
spi.open(1, 0)
spi.max_speed_hz = 1000000
spi.mode = 0

def rd(reg, size):
    tx = [(reg >> 8) & 0x7F, reg & 0xFF] + [0x00] * size
    rx = spi.xfer2(tx)
    return rx[2:]

def wr(reg, data):
    tx = [((reg >> 8) & 0x7F) | 0x80, reg & 0xFF] + data
    spi.xfer2(tx)

# Check Page 0 registers
wr(0x7FFF, [0x00])
reg_dump = rd(0x0000, 16)
print("Page 0 (0x0000..0x000F):", [hex(x) for x in reg_dump])

ui_status = rd(0x2C00, 8)
print("UI status (0x2C00):", [hex(x) for x in ui_status])

# Read result buffer at 0x0000 (after stream count check)
res_hdr = rd(0x0000, 32)
print("Result header (0x0000..0x001F):", [hex(x) for x in res_hdr])

spi.close()
PYEOF
"""

    stdin, stdout, stderr = ssh.exec_command(script, get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    ssh.close()

if __name__ == "__main__":
    inspect_ranging_dci_packet()
