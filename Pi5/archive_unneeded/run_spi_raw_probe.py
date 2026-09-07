import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def check_miso_mosi_and_raw_register():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  RAW SPI DIAGNOSTIC: READ CHIP ID REGISTER + MISO/MOSI VERIFICATION")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e

cat << 'EOF' > /tmp/spi_raw_probe.c
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

int main() {
    int fd = open("/dev/spidev0.0", O_RDWR);
    uint8_t mode = SPI_MODE_0;
    uint32_t speed = 1000000;
    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    printf("=== Raw SPI Read Probe - VL53L8CX ===\n");
    printf("Reading chip ID registers at multiple offsets:\n\n");

    // Try reading reg 0x0000 (device ID page 0) - 3 byte transfer
    uint8_t tx[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx[8] = {0};
    struct spi_ioc_transfer tr = { .tx_buf = (unsigned long)tx, .rx_buf = (unsigned long)rx, .len = 8, .speed_hz = 1000000 };
    ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    printf("READ reg=0x0000 (8 bytes): rx=[");
    for (int i = 0; i < 8; i++) printf("0x%02X%s", rx[i], i < 7 ? " " : "]\n");

    // Write page select: 0x7FFF = 0x00
    uint8_t wr1[3] = {0xFF, 0xFF, 0x00};  // write: reg 0x7FFF, val 0x00
    struct spi_ioc_transfer tr2 = { .tx_buf = (unsigned long)wr1, .len = 3, .speed_hz = 1000000 };
    ioctl(fd, SPI_IOC_MESSAGE(1), &tr2);
    printf("WROTE page select: reg=0x7FFF val=0x00\n");
    usleep(1000);

    // Now read reg 0x0000 after page select (8 bytes to see all offsets)
    uint8_t tx3[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx3[8] = {0};
    struct spi_ioc_transfer tr3 = { .tx_buf = (unsigned long)tx3, .rx_buf = (unsigned long)rx3, .len = 8, .speed_hz = 1000000 };
    ioctl(fd, SPI_IOC_MESSAGE(1), &tr3);
    printf("READ reg=0x0000 after page=0 (8 bytes): rx=[");
    for (int i = 0; i < 8; i++) printf("0x%02X%s", rx3[i], i < 7 ? " " : "]\n");

    // Read the status register 0x0006 (8 bytes)
    uint8_t tx4[10] = {0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx4[10] = {0};
    struct spi_ioc_transfer tr4 = { .tx_buf = (unsigned long)tx4, .rx_buf = (unsigned long)rx4, .len = 10, .speed_hz = 1000000 };
    ioctl(fd, SPI_IOC_MESSAGE(1), &tr4);
    printf("READ reg=0x0006 (10 bytes): rx=[");
    for (int i = 0; i < 10; i++) printf("0x%02X%s", rx4[i], i < 9 ? " " : "]\n");

    // Write page select: reg 0x7FFF = 0x02 (firmware page)
    uint8_t wr5[3] = {0xFF, 0xFF, 0x02};
    struct spi_ioc_transfer tr5 = { .tx_buf = (unsigned long)wr5, .len = 3, .speed_hz = 1000000 };
    ioctl(fd, SPI_IOC_MESSAGE(1), &tr5);
    printf("WROTE page select: reg=0x7FFF val=0x02 (firmware page)\n");
    usleep(1000);

    // Read reg 0x0000 on firmware page (8 bytes)
    uint8_t tx6[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx6[8] = {0};
    struct spi_ioc_transfer tr6 = { .tx_buf = (unsigned long)tx6, .rx_buf = (unsigned long)rx6, .len = 8, .speed_hz = 1000000 };
    ioctl(fd, SPI_IOC_MESSAGE(1), &tr6);
    printf("READ reg=0x0000 page=2 (8 bytes): rx=[");
    for (int i = 0; i < 8; i++) printf("0x%02X%s", rx6[i], i < 7 ? " " : "]\n");

    printf("\n=== All rx bytes at every offset tell us where data actually arrives ===\n");
    printf("If MISO=MOSI swapped: all rx bytes = 0x00 or echo back tx\n");
    printf("If correct: rx[2] or rx[3] should have real sensor data\n");

    return 0;
}
EOF

gcc /tmp/spi_raw_probe.c -o /tmp/spi_raw_probe
/tmp/spi_raw_probe
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_spi_raw_probe.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_spi_raw_probe.sh && /tmp/run_spi_raw_probe.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    check_miso_mosi_and_raw_register()
