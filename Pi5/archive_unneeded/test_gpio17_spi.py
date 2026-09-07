import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def test_gpio17_high_and_read_06():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  SETTING GPIO 17 HIGH VIA PYTHON LIBGPIOD / SYSFS AND CHECKING 0x06")
    print("======================================================================")

    script = r"""#!/bin/bash
python3 - << 'PYEOF'
import spidev, time, os, fcntl, struct

# First, open gpiochip4 and hold line 17 HIGH persistently in background
# struct gpiohandle_request: 64 bytes total
# lineoffsets (64*4), flags (4), default_values (64*1), consumer_label (32*1), lines (4), fd (4)
# Using direct ioctl to gpiochip4:
GPIO_GET_LINEHANDLE_IOCTL = 0xC16CB403
GPIOHANDLE_SET_LINE_VALUES_IOCTL = 0xC040B409

chip = os.open("/dev/gpiochip4", os.O_RDWR)
# Request line 17 as output
# struct layout: lineoffsets[64]=uint32, flags=uint32, default_values[64]=uint8, consumer[32]=char, lines=uint32, fd=int32
req = bytearray(64*4 + 4 + 64*1 + 32*1 + 4 + 4)
struct.pack_into("I", req, 0, 17) # line 17
struct.pack_into("I", req, 64*4, 0x02) # GPIOHANDLE_REQUEST_OUTPUT (0x02)
req[64*4 + 4] = 0 # start LOW
struct.pack_into("I", req, 64*4 + 4 + 64 + 32, 1) # 1 line

fcntl.ioctl(chip, GPIO_GET_LINEHANDLE_IOCTL, req)
line_fd = struct.unpack_from("i", req, 64*4 + 4 + 64 + 32 + 4)[0]
print(f"GPIO 17 handle acquired! (fd={line_fd})")

# Pulse LOW for 100ms
time.sleep(0.1)

# Set line 17 to HIGH (1)
val = bytearray(64)
val[0] = 1
fcntl.ioctl(line_fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, val)
print("GPIO 17 is now driven firmly HIGH (3.3V)!")
time.sleep(0.2)

# Now check SPI
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

wr(0x7FFF, 0x00)
time.sleep(0.01)
dev_id = rd(0x0000, 2)
print("Device ID:", [f"0x{b:02X}" for b in dev_id])

status_06 = rd(0x0006, 2)
print("Reg 0x0006:", [f"0x{b:02X}" for b in status_06])

# Run SW Reboot while LPn is held HIGH
print("\nRunning SW Reboot sequence...")
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
status_after = rd(0x0006, 2)
print("Reg 0x0006 after reboot (expect 0x01):", [f"0x{b:02X}" for b in status_after])

spi.close()
os.close(line_fd)
os.close(chip)
PYEOF
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/test_gpio17_spi.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/test_gpio17_spi.sh && /tmp/test_gpio17_spi.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_gpio17_high_and_read_06()
