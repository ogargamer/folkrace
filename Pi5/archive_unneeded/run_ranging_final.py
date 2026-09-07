import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def run_ranging_clean():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "192.168.0.13"
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)

    print("======================================================================")
    print("  RUNNING VL53L8CX RANGING DEMO ON PI 5 (192.168.0.13)")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > live_ranging_demo.c
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "vl53l8cx_api.h"

int main() {
    VL53L8CX_Configuration Dev;
    VL53L8CX_ResultsData Results;
    uint8_t status, isReady;

    memset(&Dev, 0, sizeof(Dev));
    Dev.platform.spi_num = 0;
    Dev.platform.spi_cs = 0;

    status = vl53l8cx_comms_init(&Dev.platform);
    if (status) {
        printf("Error: Could not open SPI device\n");
        return 1;
    }

    printf("--> Initializing VL53L8CX (84KB Firmware Upload)...\n");
    status = vl53l8cx_init(&Dev);
    if (status != 0) {
        printf("Error: vl53l8cx_init failed with status %d\n", status);
        vl53l8cx_comms_close(&Dev.platform);
        return 1;
    }
    printf("--> VL53L8CX Initialization SUCCESSFUL!\n");

    printf("--> Setting 4x4 resolution (16 zones) & 15 Hz ranging frequency...\n");
    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_frequency_hz(&Dev, 15);

    printf("--> Starting Continuous Ranging...\n\n");
    status = vl53l8cx_start_ranging(&Dev);
    if (status != 0) {
        printf("Error starting ranging: %d\n", status);
        vl53l8cx_comms_close(&Dev.platform);
        return 1;
    }

    printf("=========================================================\n");
    printf("  STREAMING LIVE 4x4 DISTANCE MATRIX (15 Hz)\n");
    printf("=========================================================\n\n");

    for (int frame = 1; frame <= 20; frame++) {
        isReady = VL53L8CX_wait_for_dataready(&Dev.platform);
        if (isReady) {
            vl53l8cx_get_ranging_data(&Dev, &Results);
            printf("--- Frame #%02d ---\n", frame);
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
    vl53l8cx_comms_close(&Dev.platform);
    printf("--> Ranging completed cleanly!\n");
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o live_ranging_demo live_ranging_demo.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./live_ranging_demo
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_clean.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_clean.sh && /tmp/run_clean.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    run_ranging_clean()
