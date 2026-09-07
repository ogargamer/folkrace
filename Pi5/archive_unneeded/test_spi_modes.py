import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def test_spi_modes_and_write_registers():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  TESTING ALL SPI MODES & WRITE REGISTERS ON PI 5")
    print("======================================================================")

    script = r"""#!/bin/bash
python3 - << 'PYEOF'
import spidev, time

for mode in [0, 1, 2, 3]:
    for speed in [1000000, 2000000, 3000000]:
        try:
            spi = spidev.SpiDev()
            spi.open(0, 0)
            spi.mode = mode
            spi.max_speed_hz = speed

            # Read chip ID
            rx_id = spi.xfer2([0x00, 0x00, 0x00, 0x00])
            id_val = (rx_id[2] << 8) | rx_id[3] if len(rx_id) >= 4 else 0

            # Test write
            # Try write to 0x7FFF = 0x02
            tx_wr = [0xFF, 0xFF, 0x02] # (0x7FFF >> 8 | 0x80) = 0xFF, 0xFF, 0x02
            spi.xfer2(tx_wr)
            time.sleep(0.001)

            # Read back 0x7FFF
            rx_rd = spi.xfer2([0x7F, 0xFF, 0x00])
            val = rx_rd[2]

            print(f"Mode {mode} @ {speed//1000000}MHz -> ID: 0x{id_val:04X} | Readback 0x7FFF: 0x{val:02X}")
            spi.close()
        except Exception as e:
            print(f"Mode {mode} error: {e}")
PYEOF
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/test_modes.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/test_modes.sh && /tmp/test_modes.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_spi_modes_and_write_registers()
