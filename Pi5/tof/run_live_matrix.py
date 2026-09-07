import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def stream_live_matrix():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  VL53L8CX REAL-TIME 4x4 ToF DISTANCE MATRIX STREAM (15 Hz)")
    print("======================================================================")

    # Compile a continuous streaming visualizer (loops 30 frames)
    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > live_stream.c
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

    status = vl53l8cx_start_ranging(&Dev);
    if (status) {
        printf("Error starting ranging: %d\n", status);
        return 1;
    }

    printf("\n=== Real-Time 4x4 Distance Grid (mm) ===\n\n");

    for (int frame = 1; frame <= 30; frame++) {
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
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o live_stream live_stream.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./live_stream
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_live_stream.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_live_stream.sh && /tmp/run_live_stream.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    stream_live_matrix()
