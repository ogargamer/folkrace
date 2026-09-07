import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def test_mosi_continuity():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  TESTING MOSI HARDWARE WRITE REACHABILITY")
    print("======================================================================")

    script = r"""#!/bin/bash
python3 - << 'PYEOF'
import spidev, time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0

def rd(reg):
    tx = [(reg >> 8) & 0x7F, reg & 0xFF, 0x00]
    rx = spi.xfer2(tx)
    return rx[2]

def wr(reg, val):
    tx = [((reg >> 8) & 0x7F) | 0x80, reg & 0xFF, val]
    spi.xfer2(tx)

print("Reading reg 0x7FFF (default):", hex(rd(0x7FFF)))

print("Writing reg 0x7FFF = 0x02...")
wr(0x7FFF, 0x02)
time.sleep(0.01)
val1 = rd(0x7FFF)
print("Reading reg 0x7FFF:", hex(val1), "(expect 0x02 if MOSI is connected)")

print("Writing reg 0x7FFF = 0x00...")
wr(0x7FFF, 0x00)
time.sleep(0.01)
val2 = rd(0x7FFF)
print("Reading reg 0x7FFF:", hex(val2), "(expect 0x00)")

if val1 == 0x02:
    print("\nSUCCESS: MOSI write line is working properly!")
else:
    print("\nFAILURE: MOSI line is disconnected or not transmitting data to the sensor.")

spi.close()
PYEOF
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/test_mosi.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/test_mosi.sh && /tmp/test_mosi.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_mosi_continuity()
