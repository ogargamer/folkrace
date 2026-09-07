import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def deploy_complete_platform_and_run_spi1():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  DEPLOYING CLEAN PLATFORM & EXECUTING OFFICIAL ST DRIVER ON SPI1.0")
    print("======================================================================")

    platform_c_content = r"""/**
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  */

#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#ifdef SPI
#include <linux/spi/spidev.h>
#endif
#include <sys/ioctl.h>

#include "platform.h"
#include "types.h"
#include "vl53l8cx_api.h"

#define VL53L8CX_ERROR_GPIO_SET_FAIL	-1
#define VL53L8CX_COMMS_ERROR		-2
#define VL53L8CX_ERROR_TIME_OUT		-3

#define VL53L8CX_COMMS_CHUNK_SIZE  1024

#ifdef SPI
#define VL53L8_SPIDEV_CHUNK_SIZE   1024
#define VL53L8CX_SPI_MODE  SPI_MODE_0
#define VL53L8CX_SPI_SPEED_HZ  1000000
#define VL53L8CX_SPI_NB_BITS   8
#endif

#define LOG printf

#ifdef SPI
static int32_t Linux_SPI_Read_16M(int fd, uint16_t index, uint8_t* read_data, uint32_t read_size, uint32_t speed_hz);
static int32_t Linux_SPI_Write_16M(int fd, uint16_t index, uint8_t* write_data, uint32_t write_size, uint32_t speed_hz);
#endif

int32_t vl53l8cx_comms_init(VL53L8CX_Platform * p_platform)
{
#ifdef SPI
	uint8_t spi_mode = VL53L8CX_SPI_MODE;
	uint8_t bits = VL53L8CX_SPI_NB_BITS;
	uint32_t speed = VL53L8CX_SPI_SPEED_HZ;
	char devname[32];

	sprintf(devname, "/dev/spidev%d.%d", p_platform->spi_num, p_platform->spi_cs);
	p_platform->fd = open(devname, O_RDWR);
	if (p_platform->fd == -1) {
		LOG("Failed to open %s\n", devname);
		return VL53L8CX_COMMS_ERROR;
	}
	else
		LOG("Opened SPI %s (fd=%d)\n", devname, p_platform->fd);

	ioctl(p_platform->fd, SPI_IOC_WR_MODE, &spi_mode);
	ioctl(p_platform->fd, SPI_IOC_RD_MODE, &spi_mode);
	ioctl(p_platform->fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	ioctl(p_platform->fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	ioctl(p_platform->fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	ioctl(p_platform->fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
#else
	p_platform->address = 0x52;
	p_platform->fd = open("/dev/i2c-1", O_RDONLY);
	if (p_platform->fd == -1) {
		LOG("Failed to open /dev/i2c-1\n");
		return VL53L8CX_COMMS_ERROR;
	}
	ioctl(p_platform->fd, I2C_SLAVE, p_platform->address);
#endif
	return 0;
}

int32_t vl53l8cx_comms_close(VL53L8CX_Platform * p_platform)
{
	close(p_platform->fd);
	return 0;
}

static int32_t write_read_multi(
		int fd,
		uint16_t i2c_address,
		uint16_t reg_address,
		uint8_t *pdata,
		uint32_t count,
		int write_not_read)
{
#ifdef SPI
	uint32_t position = 0;
	uint32_t data_size = 0;

	if (write_not_read) {
		do {
			data_size = (count - position) > VL53L8_SPIDEV_CHUNK_SIZE ? VL53L8_SPIDEV_CHUNK_SIZE : (count - position);
			if (Linux_SPI_Write_16M(fd, reg_address + position, pdata + position, data_size, VL53L8CX_SPI_SPEED_HZ) != 0)
				return VL53L8CX_COMMS_ERROR;
			position += data_size;
		} while (position < count);
	} else {
		do {
			data_size = (count - position) > VL53L8_SPIDEV_CHUNK_SIZE ? VL53L8_SPIDEV_CHUNK_SIZE : (count - position);
			if (Linux_SPI_Read_16M(fd, reg_address + position, pdata + position, data_size, VL53L8CX_SPI_SPEED_HZ) != 0)
				return VL53L8CX_COMMS_ERROR;
			position += data_size;
		} while (position < count);
	}
#else
	struct i2c_msg messages[2];
	struct i2c_rdwr_ioctl_data packets;
	uint32_t position = 0;
	uint32_t data_size = 0;
	static uint8_t i2c_buffer[VL53L8CX_COMMS_CHUNK_SIZE];

	if (write_not_read) {
		do {
			data_size = (count - position) > (VL53L8CX_COMMS_CHUNK_SIZE - 2) ? (VL53L8CX_COMMS_CHUNK_SIZE - 2) : (count - position);
			i2c_buffer[0] = (reg_address + position) >> 8;
			i2c_buffer[1] = (reg_address + position) & 0xFF;
			memcpy(&i2c_buffer[2], pdata + position, data_size);

			messages[0].addr = i2c_address >> 1;
			messages[0].flags = 0;
			messages[0].len = data_size + 2;
			messages[0].buf = i2c_buffer;

			packets.msgs = messages;
			packets.nmsgs = 1;
			if (ioctl(fd, I2C_RDWR, &packets) < 0) return VL53L8CX_COMMS_ERROR;
			position += data_size;
		} while (position < count);
	} else {
		do {
			data_size = (count - position) > VL53L8CX_COMMS_CHUNK_SIZE ? VL53L8CX_COMMS_CHUNK_SIZE : (count - position);
			i2c_buffer[0] = (reg_address + position) >> 8;
			i2c_buffer[1] = (reg_address + position) & 0xFF;

			messages[0].addr = i2c_address >> 1;
			messages[0].flags = 0;
			messages[0].len = 2;
			messages[0].buf = i2c_buffer;

			messages[1].addr = i2c_address >> 1;
			messages[1].flags = I2C_M_RD;
			messages[1].len = data_size;
			messages[1].buf = pdata + position;

			packets.msgs = messages;
			packets.nmsgs = 2;
			if (ioctl(fd, I2C_RDWR, &packets) < 0) return VL53L8CX_COMMS_ERROR;
			position += data_size;
		} while (position < count);
	}
#endif
	return 0;
}

int32_t write_multi(int fd, uint16_t i2c_address, uint16_t reg_address, uint8_t *pdata, uint32_t count)
{
	return write_read_multi(fd, i2c_address, reg_address, pdata, count, 1);
}

int32_t read_multi(int fd, uint16_t i2c_address, uint16_t reg_address, uint8_t *pdata, uint32_t count)
{
	return write_read_multi(fd, i2c_address, reg_address, pdata, count, 0);
}

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform * p_platform, uint16_t reg_address, uint8_t *p_value)
{
	return read_multi(p_platform->fd, p_platform->address, reg_address, p_value, 1);
}

uint8_t VL53L8CX_WrByte(VL53L8CX_Platform * p_platform, uint16_t reg_address, uint8_t value)
{
	return write_multi(p_platform->fd, p_platform->address, reg_address, &value, 1);
}

uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform * p_platform, uint16_t reg_address, uint8_t *p_values, uint32_t size)
{
	return read_multi(p_platform->fd, p_platform->address, reg_address, p_values, size);
}

uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform * p_platform, uint16_t reg_address, uint8_t *p_values, uint32_t size)
{
	return write_multi(p_platform->fd, p_platform->address, reg_address, p_values, size);
}

void VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size)
{
	uint32_t i, tmp;
	for (i = 0; i < size; i += 4) {
		tmp = (buffer[i]<<24) | (buffer[i+1]<<16) | (buffer[i+2]<<8) | buffer[i+3];
		memcpy(&(buffer[i]), &tmp, 4);
	}
}

uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform * p_platform, uint32_t time_ms)
{
	usleep(time_ms * 1000);
	return 0;
}

#ifdef SPI
static int32_t SPI_Xfer(int device_handle, uint8_t* write_data, uint8_t* read_data, uint32_t len, uint32_t speed_hz)
{
	struct spi_ioc_transfer message;
	memset(&message, 0, sizeof(message));

	message.tx_buf = (unsigned long)write_data;
	message.rx_buf = (unsigned long)read_data;
	message.len = len;
	message.delay_usecs = 0;
	message.speed_hz = VL53L8CX_SPI_SPEED_HZ;
	message.bits_per_word = VL53L8CX_SPI_NB_BITS;

	int error = ioctl(device_handle, SPI_IOC_MESSAGE(1), &message);
	return (error < 0) ? error : 0;
}

static int32_t Linux_SPI_Read_16M(int fd, uint16_t index, uint8_t* read_data, uint32_t read_size, uint32_t speed_hz)
{
	uint8_t is_odd = index & 1;
	uint16_t aligned_index = (index & ~1) & 0x7FFF;
	uint32_t total_len = (read_size == 1) ? 4 : (read_size + 2 + is_odd);

	uint8_t* read_buffer = malloc(total_len);
	uint8_t* write_buffer = malloc(total_len);
	memset(write_buffer, 0, total_len);

	write_buffer[0] = (uint8_t)((aligned_index >> 8) & 0x7F);
	write_buffer[1] = (uint8_t)(aligned_index & 0xFF);

	int32_t status = SPI_Xfer(fd, write_buffer, read_buffer, total_len, speed_hz);

	if (status == 0) {
		if (read_size == 1) {
			*read_data = is_odd ? read_buffer[3] : read_buffer[2];
		} else {
			memcpy(read_data, read_buffer + 2 + is_odd, read_size);
		}
	}

	free(read_buffer);
	free(write_buffer);
	return status;
}

static int32_t Linux_SPI_Write_16M(int fd, uint16_t index, uint8_t* write_data, uint32_t write_size, uint32_t speed_hz)
{
	uint8_t* read_buffer = malloc(write_size + 2);
	uint8_t* write_buffer = malloc(write_size + 2);

	write_buffer[0] = (uint8_t)(((index & 0xFF00) >> 8) | 0x80);
	write_buffer[1] = (uint8_t)(index & 0xFF);
	memcpy(write_buffer + 2, write_data, write_size);

	int32_t status = SPI_Xfer(fd, write_buffer, read_buffer, write_size + 2, speed_hz);

	free(read_buffer);
	free(write_buffer);
	return status;
}
#endif

uint8_t VL53L8CX_wait_for_dataready(VL53L8CX_Platform *p_platform)
{
	VL53L8CX_Configuration * p_dev = (VL53L8CX_Configuration *)((uint8_t *)p_platform - offsetof(VL53L8CX_Configuration, platform));
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
        f.write(platform_c_content)
    sftp.close()

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

gcc -Wall -Wno-missing-braces -Os -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o run_spi1_official run_spi1_official.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./run_spi1_official
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_spi1.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_spi1.sh && /tmp/run_spi1.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    deploy_complete_platform_and_run_spi1()
