import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def probe_all_status_regs():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  PROBING ALL STATUS REGISTERS (0x0000 TO 0x0020) DURING ACTIVE RANGING")
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

# Check Page 0 registers
wr(0x7FFF, 0x00)
for r in range(0, 0x24, 2):
    val = rd(r, 2)
    print(f"Reg 0x{r:04X}: [0x{val[0]:02X} 0x{val[1]:02X}]")

print("\nUI Status (0x2C00):", [f"0x{b:02X}" for b in rd(0x2C00, 4)])
print("UI Start  (0x2C04):", [f"0x{b:02X}" for b in rd(0x2C04, 8)])

# Check Page 2 registers
wr(0x7FFF, 0x02)
print("\n--- Page 2 ---")
for r in range(0, 0x20, 4):
    val = rd(r, 4)
    print(f"Page 2 Reg 0x{r:04X}: [0x{val[0]:02X} 0x{val[1]:02X} 0x{val[2]:02X} 0x{val[3]:02X}]")

spi.close()
PYEOF
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/probe_status_regs.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/probe_status_regs.sh && /tmp/probe_status_regs.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    probe_all_status_regs()
