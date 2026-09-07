import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def test_bootloader_readiness():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  CHECKING BOOTLOADER READINESS (REG 0x06)")
    print("======================================================================")

    script = r"""#!/bin/bash
python3 - << 'PYEOF'
import spidev, time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 2000000
spi.mode = 0

def rd(reg, count=1):
    tx = [(reg >> 8) & 0x7F, reg & 0xFF] + [0] * count
    rx = spi.xfer2(tx)
    return rx[2:]

def wr(reg, val):
    tx = [((reg >> 8) & 0x7F) | 0x80, reg & 0xFF, val]
    spi.xfer2(tx)

# Select page 0
wr(0x7FFF, 0x00)
time.sleep(0.01)

# Read device ID
dev_id = rd(0x0000, 2)
print("Device ID (expect [0xF0, 0x0C]):", [f"0x{b:02X}" for b in dev_id])

# Read reg 0x0006
status_06 = rd(0x0006, 2)
print("Reg 0x0006 (0x01 = Bootloader Ready):", [f"0x{b:02X}" for b in status_06])

# If status is not 0x01, send SW reboot sequence
print("\nSending ST SW Reboot Sequence...")
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
time.sleep(0.15)

wr(0x7FFF, 0x00)
status_after_reboot = rd(0x0006, 2)
print("Reg 0x06 after SW reboot (expect 0x01):", [f"0x{b:02X}" for b in status_after_reboot])

spi.close()
PYEOF
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/test_bootloader.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/test_bootloader.sh && /tmp/test_bootloader.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_bootloader_readiness()
