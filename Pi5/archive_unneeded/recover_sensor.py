import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def recover_and_run_sensor():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  CLEANING SENSOR STATE & RECOVERING TOF STREAM")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > recover_stream.c
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "vl53l8cx_api.h"

int main() {
    VL53L8CX_Configuration Dev;
    VL53L8CX_ResultsData Results;
    uint8_t status, isAlive, isReady;

    memset(&Dev, 0, sizeof(Dev));
    Dev.platform.spi_num = 0;
    Dev.platform.spi_cs = 0;

    status = vl53l8cx_comms_init(&Dev.platform);
    if (status) {
        printf("Error: Could not open SPI\n");
        return 1;
    }

    // Try to stop any hanging stream first
    vl53l8cx_stop_ranging(&Dev);
    usleep(50000);

    // Check if alive
    status = vl53l8cx_is_alive(&Dev, &isAlive);
    printf("is_alive: status=%d isAlive=%d\n", status, isAlive);

    // If alive == 0, we can re-init
    printf("Initializing firmware...\n");
    status = vl53l8cx_init(&Dev);
    printf("vl53l8cx_init: status=%d\n", status);

    if (status != 0) {
        printf("Init failed. Retrying with stop first...\n");
        vl53l8cx_stop_ranging(&Dev);
        usleep(50000);
        status = vl53l8cx_init(&Dev);
        printf("Retry init: status=%d\n", status);
    }

    if (status == 0) {
        status = vl53l8cx_start_ranging(&Dev);
        printf("Start ranging: status=%d\n", status);

        for (int frame = 0; frame < 5; frame++) {
            isReady = VL53L8CX_wait_for_dataready(&Dev.platform);
            if (isReady) {
                vl53l8cx_get_ranging_data(&Dev, &Results);
                printf("Frame %d | Center Zone 5: %d mm (status %d), Zone 6: %d mm (status %d)\n",
                       frame, Results.distance_mm[5], Results.target_status[5],
                       Results.distance_mm[6], Results.target_status[6]);
            }
        }
        vl53l8cx_stop_ranging(&Dev);
    }

    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o recover_stream recover_stream.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./recover_stream
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_recover.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_recover.sh && /tmp/run_recover.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    recover_and_run_sensor()
