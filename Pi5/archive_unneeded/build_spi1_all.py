import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def compile_all_spi1_binaries():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  COMPILING ALL ST BINARIES FOR SPI1.0 (menu, multi, live_viewer)")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

# Update menu.c and multi_ranging.c to use spi_num = 1
sed -i 's/Dev\.platform\.spi_num = 0;/Dev.platform.spi_num = 1;/g' menu.c multi_ranging.c || true

make clean || true
make CFLAGS_RELEASE="-Os -DSPI -Wno-error=unused-result -Wno-error=implicit-function-declaration" PLTF_DIR=../platform

cat << 'EOF' > live_viewer.c
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include "vl53l8cx_api.h"

static volatile int running = 1;
void sig_handler(int s) { running = 0; }

int main() {
    signal(SIGINT, sig_handler);

    VL53L8CX_Configuration Dev;
    VL53L8CX_ResultsData Results;
    uint8_t isAlive = 0;

    memset(&Dev, 0, sizeof(Dev));
    memset(&Results, 0, sizeof(Results));
    Dev.platform.spi_num = 1;
    Dev.platform.spi_cs = 0;

    vl53l8cx_comms_init(&Dev.platform);
    vl53l8cx_is_alive(&Dev, &isAlive);
    if (!isAlive) {
        printf("Error: VL53L8CX not detected on SPI1.0!\n");
        return 1;
    }

    printf("Booting VL53L8CX on SPI1.0...\n");
    if (vl53l8cx_init(&Dev) != 0) {
        printf("Error initializing sensor firmware!\n");
        return 1;
    }

    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_frequency_hz(&Dev, 15);
    vl53l8cx_start_ranging(&Dev);

    printf("\033[2J\033[H"); // Clear screen

    uint32_t frame_count = 0;

    while (running) {
        uint8_t isReady = 0;
        for (int p = 0; p < 100 && running; p++) {
            VL53L8CX_WaitMs(&Dev.platform, 5);
            vl53l8cx_check_data_ready(&Dev, &isReady);
            if (isReady) break;
        }

        if (isReady && running) {
            vl53l8cx_get_ranging_data(&Dev, &Results);
            frame_count++;

            printf("\033[H"); // Cursor home
            printf("=================================================================\n");
            printf("  VL53L8CX LIVE 4x4 MATRIX ON SPI1 (15 Hz Stream)\n");
            printf("=================================================================\n\n");
            printf("  Frame: #%-6u | Stream Count: %-3u | Port: /dev/spidev1.0\n\n",
                   frame_count, Dev.streamcount);

            printf("  +-------------------------------------------------------------+\n");
            for (int r = 0; r < 4; r++) {
                printf("  | ");
                for (int c = 0; c < 4; c++) {
                    int idx = r * 4 + c;
                    int16_t dist = Results.distance_mm[idx];
                    uint8_t stat = Results.target_status[idx];
                    if (stat == 5 || stat == 6 || stat == 9 || stat == 10 || (dist > 0 && dist < 4000)) {
                        printf("  %4d mm (Z%02d) ", dist, idx);
                    } else {
                        printf("   ----  (Z%02d) ", idx);
                    }
                }
                printf(" |\n");
                if (r < 3) printf("  |                                                             |\n");
            }
            printf("  +-------------------------------------------------------------+\n\n");
            printf("  Press Ctrl+C to stop continuous streaming.\n");
            fflush(stdout);
        }
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    printf("\n--> Sensor stopped cleanly.\n");
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o live_viewer live_viewer.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

ls -la live_viewer menu multi
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/build_spi1.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/build_spi1.sh && /tmp/build_spi1.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    compile_all_spi1_binaries()
