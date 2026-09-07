import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def test_full_power_ranging_sequence():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  TESTING FULL VCSEL POWER-UP SEQUENCE & STREAMCOUNT INCREMENT")
    print("======================================================================")

    script = r"""#!/bin/bash
python3 - << 'PYEOF'
import spidev, time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0

def rd(reg, size=1):
    tx = [(reg >> 8) & 0x7F, reg & 0xFF] + [0x00] * size
    rx = spi.xfer2(tx)
    return rx[2:]

def wr(reg, val):
    tx = [((reg >> 8) & 0x7F) | 0x80, reg & 0xFF, val]
    spi.xfer2(tx)

wr(0x7FFF, 0x00)

# Check power mode registers
print("Reg 0x0009 (Power State):", hex(rd(0x0009)[0]))
print("Reg 0x000F (Sleep Mode):", hex(rd(0x000F)[0]))
print("Reg 0x0006 (MCU Status):", hex(rd(0x0006)[0]))

# Wake up sensor to active ranging mode
wr(0x000F, 0x40) # Active
wr(0x0009, 0x04) # Wakeup
time.sleep(0.01)

# Enable VCSEL laser pump
wr(0x0009, 0x05) # Ranging
time.sleep(0.01)

print("\nAfter Wakeup:")
print("Reg 0x0009:", hex(rd(0x0009)[0]))
print("Reg 0x000F:", hex(rd(0x000F)[0]))
print("Reg 0x0006:", hex(rd(0x0006)[0]))

# Check streamcount over 10 ticks
print("\nStreamcount over 10 ticks:")
for i in range(10):
    time.sleep(0.07)
    sc = (rd(0x0020)[0] << 8) | rd(0x0021)[0]
    print(f"Tick {i+1}: {sc}")

spi.close()
PYEOF
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/test_power.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/test_power.sh && /tmp/test_power.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_full_power_ranging_sequence()
