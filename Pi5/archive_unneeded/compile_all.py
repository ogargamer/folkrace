import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def compile_all_st_examples():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  COMPILING ALL OFFICIAL ST BINARIES (menu, multi, live_matrix)")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI -Wno-error=unused-result -Wno-error=implicit-function-declaration" PLTF_DIR=../platform

# Also create the interactive continuous visualizer
cat << 'EOF' > live_viewer.c
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include "vl53l8cx_api.h"

static volatile int running = 1;
void int_handler(int sig) { running = 0; }

int main() {
    signal(SIGINT, int_handler);
    VL53L8CX_Configuration Dev;
    VL53L8CX_ResultsData Results;
    uint8_t isAlive = 0;

    memset(&Dev, 0, sizeof(Dev));
    Dev.platform.spi_num = 0;
    Dev.platform.spi_cs = 0;

    vl53l8cx_comms_init(&Dev.platform);
    vl53l8cx_is_alive(&Dev, &isAlive);
    if (!isAlive) {
        printf("Error: VL53L8CX not detected on SPI0.0!\n");
        return 1;
    }

    printf("Booting VL53L8CX...\n");
    if (vl53l8cx_init(&Dev) != 0) {
        printf("Error initializing sensor!\n");
        return 1;
    }

    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_frequency_hz(&Dev, 15);
    vl53l8cx_start_ranging(&Dev);

    printf("\033[2J\033[H"); // Clear screen
    printf("========================================================\n");
    printf("  VL53L8CX LIVE 4x4 MATRIX CONTINUOUS STREAM (Press Ctrl+C to stop)\n");
    printf("========================================================\n\n");

    while (running) {
        uint8_t isReady = 0;
        for (int p = 0; p < 100 && running; p++) {
            VL53L8CX_WaitMs(&Dev.platform, 5);
            vl53l8cx_check_data_ready(&Dev, &isReady);
            if (isReady) break;
        }

        if (isReady && running) {
            vl53l8cx_get_ranging_data(&Dev, &Results);
            printf("\033[4;1H"); // Move cursor to top of matrix
            printf("Stream Count: %-5u | Measurement Rate: 15 Hz\n\n", Dev.streamcount);
            for (int r = 0; r < 4; r++) {
                printf("  [ ");
                for (int c = 0; c < 4; c++) {
                    int idx = r * 4 + c;
                    int16_t dist = Results.distance_mm[idx];
                    uint8_t stat = Results.target_status[idx];
                    if (stat == 5 || stat == 6 || stat == 9 || stat == 10 || (dist > 0 && dist < 4000)) {
                        printf("%5d mm  ", dist);
                    } else {
                        printf("   ----   ");
                    }
                }
                printf("]\n");
            }
            printf("\n  Status: Active & Ranging\n");
            fflush(stdout);
        }
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    printf("\n--> Stopped cleanly.\n");
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o live_viewer live_viewer.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

ls -la live_viewer menu multi
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/build_all.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/build_all.sh && /tmp/build_all.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    compile_all_st_examples()
