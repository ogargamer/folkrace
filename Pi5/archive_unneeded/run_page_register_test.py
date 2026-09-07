import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def test_page_register_16bit_write():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  TESTING 16-BIT PAGE REGISTER WRITES & LIVE STREAM")
    print("======================================================================")

    platform_c = r"""#include <fcntl.h>
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
#define VL53L8_SPIDEV_CHUNK_SIZE 1024

static int32_t SPI_Xfer(int fd, uint8_t *tx, uint8_t *rx, uint32_t len, uint32_t speed) {
    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = len;
    tr.speed_hz = speed;
    tr.bits_per_word = 8;
    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    return ret < 0 ? -1 : 0;
}

static int32_t Linux_SPI_Read_16M(int fd, uint16_t index, uint8_t *read_data, uint32_t read_size, uint32_t speed) {
    uint16_t is_odd = index & 1;
    uint16_t aligned_index = (index & ~1) & 0x7FFF;
    uint32_t total_len = read_size + 2 + is_odd;
    if (total_len < 4) total_len = 4;

    uint8_t *tx = calloc(total_len, 1);
    uint8_t *rx = calloc(total_len, 1);

    tx[0] = (uint8_t)((aligned_index >> 8) & 0x7F);
    tx[1] = (uint8_t)(aligned_index & 0xFF);

    int status = SPI_Xfer(fd, tx, rx, total_len, speed);
    if (status == 0) {
        if (read_size == 1) {
            read_data[0] = is_odd ? rx[3] : (rx[2] != 0 ? rx[2] : rx[3]);
        } else {
            memcpy(read_data, rx + 2 + is_odd, read_size);
        }
    }
    free(tx);
    free(rx);
    return status;
}

static int32_t Linux_SPI_Write_16M(int fd, uint16_t index, uint8_t *write_data, uint32_t write_size, uint32_t speed) {
    if (index == 0x7FFF) {
        uint8_t tx[4] = {0xFF, 0xFE, 0x00, write_data[0]};
        uint8_t rx[4] = {0};
        return SPI_Xfer(fd, tx, rx, 4, speed);
    }

    uint32_t total_len = write_size + 2;
    uint8_t *tx = malloc(total_len);
    uint8_t *rx = malloc(total_len);

    uint16_t aligned_index = index & 0x7FFF;
    tx[0] = (uint8_t)(((aligned_index >> 8) & 0x7F) | 0x80);
    tx[1] = (uint8_t)(aligned_index & 0xFF);
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
"""

    sftp = ssh.open_sftp()
    with sftp.file('/home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/platform/platform.c', 'w') as f:
        f.write(platform_c)

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI -Wno-error=unused-result -Wno-error=implicit-function-declaration" PLTF_DIR=../platform

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o live_matrix_stream live_matrix_stream.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./live_matrix_stream
"""

    with sftp.file('/tmp/run_page_test.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_page_test.sh && /tmp/run_page_test.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_page_register_16bit_write()
