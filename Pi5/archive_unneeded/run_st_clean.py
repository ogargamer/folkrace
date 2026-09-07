import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def build_spi_mode_0_clean_results_official_driver():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  TESTING ST OFFICIAL C DRIVER WITH SPI AUTO-INCREMENT DATA FETCH")
    print("======================================================================")

    script = """#!/bin/bash
set -e

cd /home/admin/official_st_repo/VL53L8CX/src

# Implement official ST platform with proper 4-byte aligned SPI buffer allocation
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
        uint32_t speed = 1000000;
        ioctl(fd, SPI_IOC_WR_MODE, &mode);
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    }
}

uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value) {
    _init_spi();
    uint8_t tx[3] = {((RegisterAdress >> 8) & 0x7F) | 0x80, RegisterAdress & 0xFF, value};
    struct spi_ioc_transfer tr = { .tx_buf = (unsigned long)tx, .len = 3, .speed_hz = 1000000 };
    ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    return 0;
}

uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
    _init_spi();
    uint8_t *tx = malloc(size + 2);
    tx[0] = ((RegisterAdress >> 8) & 0x7F) | 0x80;
    tx[1] = RegisterAdress & 0xFF;
    for (uint32_t i = 0; i < size; i++) tx[2 + i] = p_values[i];
    struct spi_ioc_transfer tr = { .tx_buf = (unsigned long)tx, .len = size + 2, .speed_hz = 1000000 };
    ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    free(tx);
    return 0;
}

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value) {
    _init_spi();
    uint8_t tx[4] = {(RegisterAdress >> 8) & 0x7F, RegisterAdress & 0xFF, 0x00, 0x00};
    uint8_t rx[4] = {0};
    struct spi_ioc_transfer tr = { .tx_buf = (unsigned long)tx, .rx_buf = (unsigned long)rx, .len = 4, .speed_hz = 1000000 };
    ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    *p_value = rx[3];
    return 0;
}

uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
    _init_spi();
    uint8_t *tx = calloc(size + 3, 1);
    uint8_t *rx = calloc(size + 3, 1);
    tx[0] = (RegisterAdress >> 8) & 0x7F;
    tx[1] = RegisterAdress & 0xFF;
    struct spi_ioc_transfer tr = { .tx_buf = (unsigned long)tx, .rx_buf = (unsigned long)rx, .len = size + 3, .speed_hz = 1000000 };
    ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    for (uint32_t i = 0; i < size; i++) p_values[i] = rx[3 + i];
    free(tx);
    free(rx);
    return 0;
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
        uint8_t tmp = buffer[i];
        buffer[i] = buffer[i + 3];
        buffer[i + 3] = tmp;
        tmp = buffer[i + 1];
        buffer[i + 1] = buffer[i + 2];
        buffer[i + 2] = tmp;
    }
}

uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p_platform, uint32_t TimeMs) {
    usleep(TimeMs * 1000);
    return 0;
}
EOF

cat << 'EOF' > main_st_ranging_clean.c
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include "vl53l8cx_api.h"

int main() {
    printf("========================================================\\n");
    printf("  OFFICIAL STMICROELECTRONICS VL53L8CX C RANGING ENGINE\\n");
    printf("========================================================\\n");

    VL53L8CX_Configuration Dev;
    static VL53L8CX_ResultsData Results;
    uint8_t status, isReady;

    Dev.platform.address = 0x29;

    printf("[1/3] Power cycling hardware via LPn (GPIO 17)...\\n");
    VL53L8CX_Reset_Sensor(&Dev.platform);

    printf("[2/3] Uploading ST Official 84KB RAM Binary Firmware...\\n");
    status = vl53l8cx_init(&Dev);
    printf(" -> ST ULD Initialization Status: %d (0 = OK)\\n", status);

    printf("[3/3] Enabling Continuous Ranging (15 Hz)...\\n");
    vl53l8cx_start_ranging(&Dev);

    printf("\\nStreaming Live Distance Matrix Across All 16 Pixels (10 Frames):\\n\\n");

    for (int frame = 1; frame <= 10; frame++) {
        vl53l8cx_check_data_ready(&Dev, &isReady);
        if (isReady) {
            vl53l8cx_get_ranging_data(&Dev, &Results);
            printf("[Frame %02d/10] ST Ranging Matrix:\\n", frame);
            for (int r = 0; r < 4; r++) {
                printf(" ");
                for (int c = 0; c < 4; c++) {
                    printf(" %5d mm |", Results.distance_mm[r*4 + c]);
                }
                printf("\\n");
            }
            printf("\\n");
        } else {
            printf("[Frame %02d/10] Polling SPI Hardware Stream...\\n", frame);
        }
        usleep(300000);
    }

    vl53l8cx_stop_ranging(&Dev);
    return 0;
}
EOF

gcc -I. main_st_ranging_clean.c vl53l8cx_platform.c vl53l8cx_api.c vl53l8cx_plugin_detection_thresholds.c vl53l8cx_plugin_motion_indicator.c vl53l8cx_plugin_xtalk.c -o st_official_ranging_clean
./st_official_ranging_clean
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_st_clean.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_st_clean.sh && /tmp/run_st_clean.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    build_spi_mode_0_clean_results_official_driver()
