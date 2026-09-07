import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def probe_boot_pages():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  PROBING REGISTER 0x0006 ACROSS PAGES AFTER MCU BOOT SEQUENCE")
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

def wr_buf(reg, data):
    chunk = 1024
    for i in range(0, len(data), chunk):
        sub = data[i:i+chunk]
        tx = [((reg >> 8) & 0x7F) | 0x80, reg & 0xFF] + list(sub)
        spi.xfer2(tx)

# Load 84KB Firmware binary
with open("/home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/uld-driver/src/vl53l8cx_api.c", "r") as f:
    c_code = f.read()

# Soft reset
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
print("Bootloader ready (0x06):", rd(0x06, 2))

# Enable FW access
wr(0x000E, 0x01)
wr(0x7FFF, 0x02)
wr(0x7FFF, 0x01)
wr(0x0006, 0x01)
time.sleep(0.01)
print("FW access ready (0x20):", rd(0x20, 2))

# Host access
wr(0x7FFF, 0x00)
rd(0x7FFF, 1)
wr(0x000C, 0x01)
wr(0x7FFF, 0x00)
wr(0x0101, 0x00)
wr(0x0102, 0x00)
wr(0x010A, 0x01)
wr(0x4002, 0x01)
wr(0x4002, 0x00)
wr(0x010A, 0x03)
wr(0x0103, 0x01)
wr(0x400F, 0x00)
wr(0x021A, 0x43)
wr(0x021A, 0x03)
wr(0x021A, 0x01)
wr(0x021A, 0x00)
wr(0x0219, 0x00)
wr(0x021B, 0x00)
wr(0x7FFF, 0x00)
rd(0x7FFF, 1)
wr(0x7FFF, 0x01)

# Extract firmware bytes from header/C
print("Uploading firmware via C test...")
spi.close()
PYEOF
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/test_pages_probe.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/test_pages_probe.sh && /tmp/test_pages_probe.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    probe_boot_pages()
