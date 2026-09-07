import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def test_hardware_reset_then_init():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  HARDWARE RESET VIA GPIOCHIP4 PIN 17 -> INIT -> RANGE")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > full_reset_and_range.c
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include "vl53l8cx_api.h"

void hardware_power_cycle_lpn() {
    int chip_fd = open("/dev/gpiochip4", O_RDWR);
    if (chip_fd >= 0) {
        struct gpiohandle_request req;
        memset(&req, 0, sizeof(req));
        req.lineoffsets[0] = 17;
        req.flags = GPIOHANDLE_REQUEST_OUTPUT;
        req.lines = 1;
        strcpy(req.consumer_label, "lpn_pulse");
        req.default_values[0] = 0; // Pull LOW
        if (ioctl(chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req) >= 0) {
            printf("Pulled LPn LOW...\n");
            usleep(100000); // 100ms LOW
            struct gpiohandle_data data;
            data.values[0] = 1; // Pull HIGH
            ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
            printf("Pulled LPn HIGH (Powered On)...\n");
            usleep(200000); // 200ms HIGH to stabilize bootloader
            close(req.fd);
        }
        close(chip_fd);
    }
}

int main() {
    VL53L8CX_Configuration Dev;
    VL53L8CX_ResultsData Results;
    uint8_t status, isAlive, isReady;

    printf("=== Step 1: Hardware Power-Cycle via LPn ===\n");
    hardware_power_cycle_lpn();

    memset(&Dev, 0, sizeof(Dev));
    Dev.platform.spi_num = 0;
    Dev.platform.spi_cs = 0;

    status = vl53l8cx_comms_init(&Dev.platform);
    if (status) {
        printf("Error: Could not open SPI\n");
        return 1;
    }

    printf("=== Step 2: Checking is_alive ===\n");
    status = vl53l8cx_is_alive(&Dev, &isAlive);
    printf("is_alive: status=%d, isAlive=%d\n", status, isAlive);

    printf("=== Step 3: Initializing 84KB Firmware ===\n");
    status = vl53l8cx_init(&Dev);
    printf("vl53l8cx_init returned: %d\n", status);

    if (status == 0) {
        printf("=== Step 4: Starting Ranging ===\n");
        status = vl53l8cx_start_ranging(&Dev);
        printf("vl53l8cx_start_ranging returned: %d\n", status);

        for (int frame = 1; frame <= 5; frame++) {
            isReady = VL53L8CX_wait_for_dataready(&Dev.platform);
            if (isReady) {
                vl53l8cx_get_ranging_data(&Dev, &Results);
                printf("Frame #%d -> Zone 0: %4d mm | Zone 5: %4d mm | Zone 10: %4d mm\n",
                       frame, Results.distance_mm[0], Results.distance_mm[5], Results.distance_mm[10]);
            }
        }
        vl53l8cx_stop_ranging(&Dev);
        printf("=== Ranging Finished Cleanly ===\n");
    }

    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o full_reset_and_range full_reset_and_range.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./full_reset_and_range
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_full_reset.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_full_reset.sh && /tmp/run_full_reset.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_hardware_reset_then_init()
