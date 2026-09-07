import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def build_i2c_new_sensor_driver():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  TESTING BRAND NEW VL53L8CX SENSOR WITH I2C BUS (/dev/i2c-1)")
    print("======================================================================")

    script = """#!/bin/bash
set -e

cd /home/admin/official_st_repo/VL53L8CX/src

# Set LPn LOW then HIGH to select standard I2C mode on start
gpioset 4 17=0 2>/dev/null || true
python3 -c "import time; time.sleep(0.05)"
gpioset 4 17=1 2>/dev/null || true
python3 -c "import time; time.sleep(0.1)"

cat << 'EOF' > vl53l8cx_platform.c
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "vl53l8cx_platform.h"

static int fd = -1;

static void _init_i2c(uint16_t addr) {
    if (fd < 0) {
        fd = open("/dev/i2c-1", O_RDWR);
    }
    ioctl(fd, I2C_SLAVE, addr >> 1);
}

uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value) {
    _init_i2c(p_platform->address);
    uint8_t buf[3] = {(RegisterAdress >> 8) & 0xFF, RegisterAdress & 0xFF, value};
    return write(fd, buf, 3) == 3 ? 0 : 1;
}

uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
    _init_i2c(p_platform->address);
    uint8_t *buf = malloc(size + 2);
    buf[0] = (RegisterAdress >> 8) & 0xFF;
    buf[1] = RegisterAdress & 0xFF;
    for (uint32_t i = 0; i < size; i++) buf[2 + i] = p_values[i];
    int ret = write(fd, buf, size + 2);
    free(buf);
    return ret == (int)(size + 2) ? 0 : 1;
}

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value) {
    _init_i2c(p_platform->address);
    uint8_t buf[2] = {(RegisterAdress >> 8) & 0xFF, RegisterAdress & 0xFF};
    if (write(fd, buf, 2) != 2) return 1;
    return read(fd, p_value, 1) == 1 ? 0 : 1;
}

uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
    _init_i2c(p_platform->address);
    uint8_t buf[2] = {(RegisterAdress >> 8) & 0xFF, RegisterAdress & 0xFF};
    if (write(fd, buf, 2) != 2) return 1;
    return read(fd, p_values, size) == (int)size ? 0 : 1;
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

gcc -I. main_st_ranging_clean.c vl53l8cx_platform.c vl53l8cx_api.c vl53l8cx_plugin_detection_thresholds.c vl53l8cx_plugin_motion_indicator.c vl53l8cx_plugin_xtalk.c -o st_official_ranging_i2c
./st_official_ranging_i2c
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_st_i2c.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_st_i2c.sh && /tmp/run_st_i2c.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    build_i2c_new_sensor_driver()
