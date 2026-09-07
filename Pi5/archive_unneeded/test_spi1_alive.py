import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def test_gpio17_and_spi1_alive():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  ASSERTING GPIO 17 HIGH (LPN) & TESTING SENSOR IS_ALIVE ON SPI1.0")
    print("======================================================================")

    script = r"""#!/bin/bash
python3 - << 'PYEOF'
import spidev, time

# Drive GPIO 17 (Pin 11) HIGH using libgpiod / gpiod
try:
    import gpiod
    # On Pi 5, RP1 GPIO chip is typically gpiochip4
    for chip_num in [4, 0, 1, 2]:
        try:
            chip = gpiod.Chip(f"gpiochip{chip_num}")
            line = chip.get_line(17)
            line.request(consumer="vl53l8_lpn", type=gpiod.LINE_REQ_DIR_OUT)
            line.set_value(1)
            print(f"GPIO 17 driven HIGH on gpiochip{chip_num}!")
            break
        except Exception as e:
            continue
except Exception as e:
    print("gpiod exception:", e)

time.sleep(0.05)

# Test SPI1.0 communication
try:
    spi = spidev.SpiDev()
    spi.open(1, 0)
    spi.max_speed_hz = 1000000
    spi.mode = 0

    # Write Page 0
    tx = [0x80 | 0x7F, 0xFF, 0x00]
    spi.xfer2(tx)

    # Read Device ID at 0x0000
    tx = [0x00, 0x00, 0x00, 0x00]
    rx = spi.xfer2(tx)
    print(f"SPI1.0 Read at 0x0000: [0x{rx[2]:02X} 0x{rx[3]:02X}]")
    if rx[2] == 0xF0 and rx[3] == 0x0C:
        print("--> SUCCESS: VL53L8CX (0xF00C) DETECTED ON SPI1.0!")
    else:
        print(f"--> Received: 0x{rx[2]:02X} 0x{rx[3]:02X}")
    spi.close()
except Exception as e:
    print("SPI1 Error:", e)
PYEOF
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/test_spi1_alive.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/test_spi1_alive.sh && /tmp/test_spi1_alive.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_gpio17_and_spi1_alive()
