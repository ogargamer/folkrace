import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def build_spi_correct_framing():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  FINAL CORRECT SPI FRAMING: rx[2] offset, no dummy, write=|0x80")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e

cd /home/admin/official_st_repo/VL53L8CX/src

# Based on raw probe:
# - READ: tx=[addr_hi & 0x7F, addr_lo, ...], data at rx[2] onwards (no dummy byte)
# - WRITE: tx=[(addr_hi & 0x7F)|0x80, addr_lo, data...]
# - Register 0x0000 returns 0xF00C chip ID at rx[2] 
cat << 'EOF' > vl53l8cx_platform.c
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "vl53l8cx_platform.h"

static int fd = -1;

static void _init_spi() {
    if (fd < 0) {
        fd = open("/dev/spidev0.0", O_RDWR);
        uint8_t mode = SPI_MODE_0;
        uint32_t speed = 3000000;
        ioctl(fd, SPI_IOC_WR_MODE, &mode);
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    }
}

// WRITE: bit15 SET = write operation, data immediately follows 2-byte address
uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value) {
    _init_spi();
    uint8_t tx[3] = {((RegisterAdress >> 8) & 0x7F) | 0x80, RegisterAdress & 0xFF, value};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .len = 3,
        .speed_hz = 3000000,
        .cs_change = 0
    };
    return ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0 ? 1 : 0;
}

uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
    _init_spi();
    uint8_t *tx = malloc(size + 2);
    tx[0] = ((RegisterAdress >> 8) & 0x7F) | 0x80;
    tx[1] = RegisterAdress & 0xFF;
    for (uint32_t i = 0; i < size; i++) tx[2 + i] = p_values[i];
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .len = size + 2,
        .speed_hz = 3000000,
        .cs_change = 0
    };
    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    free(tx);
    return ret < 0 ? 1 : 0;
}

// READ: bit15 CLEAR = read operation, data arrives at rx[2] (immediately after 2 address bytes)
uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value) {
    _init_spi();
    uint8_t tx[3] = {(RegisterAdress >> 8) & 0x7F, RegisterAdress & 0xFF, 0x00};
    uint8_t rx[3] = {0};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 3,
        .speed_hz = 3000000,
        .cs_change = 0
    };
    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    *p_value = rx[2];
    return ret < 0 ? 1 : 0;
}

uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
    _init_spi();
    uint8_t *tx = calloc(size + 2, 1);
    uint8_t *rx = calloc(size + 2, 1);
    tx[0] = (RegisterAdress >> 8) & 0x7F;
    tx[1] = RegisterAdress & 0xFF;
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = size + 2,
        .speed_hz = 3000000,
        .cs_change = 0
    };
    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    for (uint32_t i = 0; i < size; i++) p_values[i] = rx[2 + i];
    free(tx);
    free(rx);
    return ret < 0 ? 1 : 0;
}

uint8_t VL53L8CX_Reset_Sensor(VL53L8CX_Platform *p_platform) {
    system("gpioset 4 17=0 2>/dev/null || true");
    usleep(50000);
    system("gpioset 4 17=1 2>/dev/null || true");
    usleep(100000);
    return 0;
}

void VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size) {
    for (uint16_t i = 0; i < size; i += 4) {
        uint8_t tmp[4];
        tmp[0] = buffer[i + 3];
        tmp[1] = buffer[i + 2];
        tmp[2] = buffer[i + 1];
        tmp[3] = buffer[i];
        buffer[i] = tmp[0];
        buffer[i+1] = tmp[1];
        buffer[i+2] = tmp[2];
        buffer[i+3] = tmp[3];
    }
}

uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p_platform, uint32_t TimeMs) {
    usleep(TimeMs * 1000);
    return 0;
}
EOF

echo "==> Compiling with correct SPI framing..."
gcc -I. main_st_ranging_clean.c vl53l8cx_platform.c vl53l8cx_api.c vl53l8cx_plugin_detection_thresholds.c vl53l8cx_plugin_motion_indicator.c vl53l8cx_plugin_xtalk.c -o st_correct_framing
echo "==> Running..."
./st_correct_framing
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_st_correct.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_st_correct.sh && /tmp/run_st_correct.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    build_spi_correct_framing()
