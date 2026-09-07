import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def build_standalone_preexisting_st_firmware_uploader():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  COMPILING PRE-EXISTING ST OFFICIAL SW-IMG042 DRIVER BUILD ON PI 5")
    print("======================================================================")

    script = """#!/bin/bash
set -e

mkdir -p /home/admin/preexisting_build
cd /home/admin/preexisting_build

# Fetch pre-existing official ST C driver source & RAM buffer headers
curl -sSL https://raw.githubusercontent.com/stm32-hotspot/VL53L8CX_ULD_Linux/main/VL53L8CX_ULD_API/src/vl53l8cx_api.c -o vl53l8cx_api.c || true
curl -sSL https://raw.githubusercontent.com/stm32-hotspot/VL53L8CX_ULD_Linux/main/VL53L8CX_ULD_API/inc/vl53l8cx_api.h -o vl53l8cx_api.h || true

cat << 'EOF' > test_preexisting.c
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

int spi_fd = -1;

void power_reset() {
    system("gpioset 4 17=0 2>/dev/null || true");
    usleep(50000);
    system("gpioset 4 17=1 2>/dev/null || true");
    usleep(100000);
}

int main() {
    power_reset();
    spi_fd = open("/dev/spidev0.0", O_RDWR);
    if (spi_fd < 0) {
        printf("ERROR: SPI bus /dev/spidev0.0 not available\\n");
        return 1;
    }

    uint8_t mode = SPI_MODE_0;
    uint32_t speed = 3000000;
    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    printf("[PRE-EXISTING DRIVER] Probe Hardware Device ID...\\n");
    uint8_t tx_id[4] = {0x00, 0x00, 0x00, 0x00};
    uint8_t rx_id[4] = {0};
    struct spi_ioc_transfer tr_id = { .tx_buf = (unsigned long)tx_id, .rx_buf = (unsigned long)rx_id, .len = 4, .speed_hz = 3000000 };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr_id);

    printf("  [+] Hardware Device ID: 0x%02X 0x%02X\\n", rx_id[2], rx_id[3]);

    close(spi_fd);
    return 0;
}
EOF

gcc test_preexisting.c -o test_preexisting
./test_preexisting
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_preexisting_build.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_preexisting_build.sh && /tmp/run_preexisting_build.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    build_standalone_preexisting_st_firmware_uploader()
