import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def build_perfect_platform_and_stream():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "192.168.0.13"
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)

    print("======================================================================")
    print("  BUILDING SOLID SPI PLATFORM & STREAMING LIVE DISTANCE MATRIX")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/platform

cat << 'EOF' > platform.c
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include "platform.h"
#include "types.h"
#include "vl53l8cx_api.h"

#define VL53L8CX_SPI_MODE      SPI_MODE_0
#define VL53L8CX_SPI_SPEED_HZ  1000000
#define VL53L8CX_SPI_NB_BITS   8
#define VL53L8_SPIDEV_CHUNK_SIZE 4094

static int32_t SPI_Xfer(int fd, uint8_t *tx, uint8_t *rx, uint32_t len, uint32_t speed) {
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = len,
        .speed_hz = speed,
        .bits_per_word = 8,
        .cs_change = 0
    };
    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    return ret < 0 ? -1 : 0;
}

static int32_t Linux_SPI_Read_16M(int fd, uint16_t index, uint8_t *read_data, uint32_t read_size, uint32_t speed) {
    // For single byte reads on SPI word registers (like reg 0x06), read 2 bytes so low byte arrives
    uint32_t actual_read = (read_size == 1) ? 2 : read_size;
    uint32_t total_len = actual_read + 2;

    uint8_t *tx = calloc(total_len, 1);
    uint8_t *rx = calloc(total_len, 1);

    index = index & 0x7FFF;
    tx[0] = (uint8_t)((index >> 8) & 0x7F);
    tx[1] = (uint8_t)(index & 0xFF);

    int status = SPI_Xfer(fd, tx, rx, total_len, speed);
    if (status == 0) {
        if (read_size == 1) {
            // If high byte is 0, return low byte (rx[3]), else rx[2]
            read_data[0] = (rx[2] != 0) ? rx[2] : rx[3];
        } else {
            memcpy(read_data, rx + 2, read_size);
        }
    }

    free(tx);
    free(rx);
    return status;
}

static int32_t Linux_SPI_Write_16M(int fd, uint16_t index, uint8_t *write_data, uint32_t write_size, uint32_t speed) {
    uint32_t total_len = write_size + 2;
    uint8_t *tx = malloc(total_len);
    uint8_t *rx = malloc(total_len);

    tx[0] = (uint8_t)(((index >> 8) & 0x7F) | 0x80);
    tx[1] = (uint8_t)(index & 0xFF);
    memcpy(tx + 2, write_data, write_size);

    int status = SPI_Xfer(fd, tx, rx, total_len, speed);
    free(tx);
    free(rx);
    return status;
}

int32_t vl53l8cx_comms_init(VL53L8CX_Platform *p_platform) {
    char devname[32];
    sprintf(devname, "/dev/spidev%d.%d", p_platform->spi_num, p_platform->spi_cs);
    p_platform->fd = open(devname, O_RDWR);
    if (p_platform->fd < 0) return -1;

    uint8_t mode = VL53L8CX_SPI_MODE;
    uint8_t bits = VL53L8CX_SPI_NB_BITS;
    uint32_t speed = VL53L8CX_SPI_SPEED_HZ;

    ioctl(p_platform->fd, SPI_IOC_WR_MODE, &mode);
    ioctl(p_platform->fd, SPI_IOC_RD_MODE, &mode);
    ioctl(p_platform->fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(p_platform->fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    ioctl(p_platform->fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    ioctl(p_platform->fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    return 0;
}

int32_t vl53l8cx_comms_close(VL53L8CX_Platform *p_platform) {
    if (p_platform->fd >= 0) close(p_platform->fd);
    return 0;
}

int32_t write_read_multi(int fd, uint16_t i2c_addr, uint16_t reg_addr, uint8_t *pdata, uint32_t count, int write_not_read) {
    uint32_t pos = 0;
    while (pos < count) {
        uint32_t chunk = (count - pos > VL53L8_SPIDEV_CHUNK_SIZE) ? VL53L8_SPIDEV_CHUNK_SIZE : (count - pos);
        if (write_not_read) {
            if (Linux_SPI_Write_16M(fd, reg_addr + pos, pdata + pos, chunk, VL53L8CX_SPI_SPEED_HZ) != 0) return -1;
        } else {
            if (Linux_SPI_Read_16M(fd, reg_addr + pos, pdata + pos, chunk, VL53L8CX_SPI_SPEED_HZ) != 0) return -1;
        }
        pos += chunk;
    }
    return 0;
}

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t reg, uint8_t *val) {
    return (uint8_t)write_read_multi(p_platform->fd, 0, reg, val, 1, 0);
}

uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t reg, uint8_t val) {
    return (uint8_t)write_read_multi(p_platform->fd, 0, reg, &val, 1, 1);
}

uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t reg, uint8_t *vals, uint32_t size) {
    return (uint8_t)write_read_multi(p_platform->fd, 0, reg, vals, size, 0);
}

uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform, uint16_t reg, uint8_t *vals, uint32_t size) {
    return (uint8_t)write_read_multi(p_platform->fd, 0, reg, vals, size, 1);
}

void VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size) {
    for (uint16_t i = 0; i < size; i += 4) {
        uint32_t tmp = (buffer[i]<<24) | (buffer[i+1]<<16) | (buffer[i+2]<<8) | buffer[i+3];
        memcpy(&buffer[i], &tmp, 4);
    }
}

uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p_platform, uint32_t time_ms) {
    usleep(time_ms * 1000);
    return 0;
}

uint8_t VL53L8CX_wait_for_dataready(VL53L8CX_Platform *p_platform) {
    VL53L8CX_Configuration *p_dev = (VL53L8CX_Configuration *)((uint8_t *)p_platform - offsetof(VL53L8CX_Configuration, platform));
    uint8_t isReady = 0;
    do {
        VL53L8CX_WaitMs(p_platform, 5);
        vl53l8cx_check_data_ready(p_dev, &isReady);
    } while (isReady == 0);
    return 1;
}
EOF

cd ../test
make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI" PLTF_DIR=../platform

cat << 'EOF' > live_4x4_stream.c
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "vl53l8cx_api.h"

int main() {
    VL53L8CX_Configuration Dev;
    VL53L8CX_ResultsData Results;
    uint8_t isAlive = 0, isReady = 0;

    memset(&Dev, 0, sizeof(Dev));
    Dev.platform.spi_num = 0;
    Dev.platform.spi_cs = 0;

    vl53l8cx_comms_init(&Dev.platform);

    vl53l8cx_is_alive(&Dev, &isAlive);
    printf("is_alive: %d\n", isAlive);

    printf("Initializing VL53L8CX 84KB Firmware...\n");
    int status = vl53l8cx_init(&Dev);
    printf("vl53l8cx_init: status = %d\n", status);

    if (status == 0) {
        vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
        vl53l8cx_set_ranging_frequency_hz(&Dev, 15);
        vl53l8cx_start_ranging(&Dev);

        printf("\n=======================================================\n");
        printf("  LIVE VL53L8CX 4x4 ToF MATRIX STREAM (15 Hz)\n");
        printf("=======================================================\n\n");

        for (int frame = 1; frame <= 15; frame++) {
            isReady = VL53L8CX_wait_for_dataready(&Dev.platform);
            if (isReady) {
                vl53l8cx_get_ranging_data(&Dev, &Results);
                printf("--- Frame #%02d (Stream Count: %u) ---\n", frame, Dev.streamcount);
                for (int r = 0; r < 4; r++) {
                    for (int c = 0; c < 4; c++) {
                        int idx = r * 4 + c;
                        int16_t dist = Results.distance_mm[idx];
                        uint8_t stat = Results.target_status[idx];
                        if (stat == 5 || stat == 6 || stat == 9 || stat == 10) {
                            printf("%5d mm  ", dist);
                        } else {
                            printf("   ----   ");
                        }
                    }
                    printf("\n");
                }
                printf("\n");
                fflush(stdout);
            }
        }
        vl53l8cx_stop_ranging(&Dev);
    }

    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o live_4x4_stream live_4x4_stream.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./live_4x4_stream
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_final_live.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_final_live.sh && /tmp/run_final_live.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    build_perfect_platform_and_stream()
