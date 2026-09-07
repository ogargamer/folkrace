import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def build_and_run_matrix_visualizer():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  CREATING REAL-TIME 4x4 ASCII MATRIX VISUALIZER FOR VL53L8CX")
    print("======================================================================")

    c_source = r"""#include <stdio.h>
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

    // Reset sensor via LPn (GPIO 17)
    system("gpioset 4 17=0 2>/dev/null || true");
    usleep(50000);
    system("gpioset 4 17=1 2>/dev/null || true");
    usleep(100000);

    status = vl53l8cx_comms_init(&Dev.platform);
    if (status) {
        printf("Error: Could not open /dev/spidev0.0\n");
        return 1;
    }

    status = vl53l8cx_is_alive(&Dev, &isAlive);
    if (!isAlive || status) {
        printf("Error: VL53L8CX not responding (isAlive=%d, status=%d)\n", isAlive, status);
        return 1;
    }

    printf("Initializing VL53L8CX 84KB Firmware via SPI...\n");
    status = vl53l8cx_init(&Dev);
    if (status) {
        printf("Error: ULD initialization failed with code %d\n", status);
        return 1;
    }

    // Set 4x4 resolution (16 zones) and 15Hz ranging rate
    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_frequency_hz(&Dev, 15);
    vl53l8cx_set_ranging_mode(&Dev, VL53L8CX_RANGING_MODE_CONTINUOUS);

    status = vl53l8cx_start_ranging(&Dev);
    if (status) {
        printf("Error starting ranging: %d\n", status);
        return 1;
    }

    printf("\033[2J\033[H"); // Clear screen
    printf("=================================================================\n");
    printf("  VL53L8CX REAL-TIME 4x4 ToF DISTANCE MATRIX (15 Hz) [SPI]\n");
    printf("=================================================================\n\n");

    for (int frame = 1; frame <= 15; frame++) {
        isReady = VL53L8CX_wait_for_dataready(&Dev.platform);
        if (isReady) {
            vl53l8cx_get_ranging_data(&Dev, &Results);

            printf("\033[4;1H"); // Move cursor to top of matrix area
            printf("Frame #%03d | Stream Count: %5u\n", frame, Dev.streamcount);
            printf("+---------------------------------------+\n");

            for (int r = 0; r < 4; r++) {
                printf("|");
                for (int c = 0; c < 4; c++) {
                    int idx = r * 4 + c;
                    int16_t dist = Results.distance_mm[idx];
                    uint8_t stat = Results.target_status[idx];

                    // Status 5 or 6 or 9 or 10 indicates valid targets
                    if (stat == 5 || stat == 6 || stat == 9 || stat == 10) {
                        if (dist < 200) {
                            printf(" \033[1;31m%4dmm\033[0m |", dist); // Red (close)
                        } else if (dist < 500) {
                            printf(" \033[1;33m%4dmm\033[0m |", dist); // Yellow (mid)
                        } else {
                            printf(" \033[1;32m%4dmm\033[0m |", dist); // Green (far)
                        }
                    } else {
                        printf(" \033[90m ---- \033[0m |"); // Dim grey for invalid / out of range
                    }
                }
                printf("\n+---------------------------------------+\n");
            }
            fflush(stdout);
        }
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    printf("\nRanging complete!\n");
    return 0;
}
"""

    script = f"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > matrix_visualizer.c
{c_source}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o matrix_visualizer matrix_visualizer.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./matrix_visualizer
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_visualizer.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_visualizer.sh && /tmp/run_visualizer.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    build_and_run_matrix_visualizer()
