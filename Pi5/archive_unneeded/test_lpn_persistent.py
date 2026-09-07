import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def test_lpn_persistent_hold():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  PERSISTENT LPn HOLD (PULSE LOW -> HOLD HIGH THROUGHOUT RUN)")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > persistent_hold_range.c
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include "vl53l8cx_api.h"

static int chip_fd = -1;
static int line_fd = -1;

void lpn_reset_and_hold_high() {
    chip_fd = open("/dev/gpiochip4", O_RDWR);
    if (chip_fd >= 0) {
        struct gpiohandle_request req;
        memset(&req, 0, sizeof(req));
        req.lineoffsets[0] = 17;
        req.flags = GPIOHANDLE_REQUEST_OUTPUT;
        req.lines = 1;
        strcpy(req.consumer_label, "lpn_active");
        req.default_values[0] = 0; // Start LOW

        if (ioctl(chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req) >= 0) {
            line_fd = req.fd;
            printf("LPn: Reset pulse LOW (50ms)...\n");
            usleep(50000);

            struct gpiohandle_data data;
            data.values[0] = 1; // Pull and HOLD HIGH
            ioctl(line_fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
            printf("LPn: Held HIGH (Active)...\n");
            usleep(150000); // 150ms bootloader startup
        }
    }
}

void lpn_cleanup() {
    if (line_fd >= 0) close(line_fd);
    if (chip_fd >= 0) close(chip_fd);
}

int main() {
    VL53L8CX_Configuration Dev;
    VL53L8CX_ResultsData Results;
    uint8_t status, isAlive, isReady;

    lpn_reset_and_hold_high();

    memset(&Dev, 0, sizeof(Dev));
    Dev.platform.spi_num = 0;
    Dev.platform.spi_cs = 0;

    status = vl53l8cx_comms_init(&Dev.platform);
    if (status) {
        printf("Error: Could not open SPI\n");
        lpn_cleanup();
        return 1;
    }

    status = vl53l8cx_is_alive(&Dev, &isAlive);
    printf("is_alive: status=%d, isAlive=%d\n", status, isAlive);

    printf("Uploading 84KB Firmware (with LPn held firmly HIGH)...\n");
    status = vl53l8cx_init(&Dev);
    printf("vl53l8cx_init status: %d (0 = SUCCESS)\n", status);

    if (status == 0) {
        status = vl53l8cx_start_ranging(&Dev);
        printf("vl53l8cx_start_ranging status: %d\n\n", status);

        printf("=== Streaming 10 Live Frames ===\n");
        for (int frame = 1; frame <= 10; frame++) {
            isReady = VL53L8CX_wait_for_dataready(&Dev.platform);
            if (isReady) {
                vl53l8cx_get_ranging_data(&Dev, &Results);
                printf("Frame #%02d | Center: %4d mm (stat %d) | TopLeft: %4d mm | BottomRight: %4d mm\n",
                       frame,
                       Results.distance_mm[5], Results.target_status[5],
                       Results.distance_mm[0],
                       Results.distance_mm[15]);
            }
        }
        vl53l8cx_stop_ranging(&Dev);
        printf("\nRanging finished cleanly!\n");
    }

    vl53l8cx_comms_close(&Dev.platform);
    lpn_cleanup();
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o persistent_hold_range persistent_hold_range.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./persistent_hold_range
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_persistent.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_persistent.sh && /tmp/run_persistent.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_lpn_persistent_hold()
