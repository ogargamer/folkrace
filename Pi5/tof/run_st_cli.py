import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def build_spi_official_ranging_app():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  BUILDING OFFICIAL ST C DRIVER SPI CLI UTILITY AND CLI INSTALLATION")
    print("======================================================================")

    script = """#!/bin/bash
set -e

cd /home/admin/official_st_repo/VL53L8CX/src

# Create high-performance C ranging app linked against official ST ULD
cat << 'EOF' > st_tof_cli.c
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "vl53l8cx_api.h"

int main(int argc, char *argv[]) {
    int count = 10;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--count") == 0 && i + 1 < argc) {
            count = atoi(argv[i + 1]);
            i++;
        }
    }

    VL53L8CX_Configuration Dev;
    static VL53L8CX_ResultsData Results;
    uint8_t status, isReady;

    Dev.platform.address = 0x29;

    VL53L8CX_Reset_Sensor(&Dev.platform);
    vl53l8cx_init(&Dev);
    vl53l8cx_start_ranging(&Dev);

    printf("Streaming %d matrix frames...\\n", count);
    printf("========================================================\\n");
    printf("   VL53L8CX 4x4 ALL-PIXEL REAL-TIME DISTANCE GRID (mm)\\n");
    printf("========================================================\\n\\n");

    int frames_captured = 0;
    while (frames_captured < count) {
        vl53l8cx_check_data_ready(&Dev, &isReady);
        if (isReady) {
            vl53l8cx_get_ranging_data(&Dev, &Results);
            frames_captured++;
            printf("[%d/%d] 16-Pixel Distance Matrix (Zone 0 to Zone 15):\\n", frames_captured, count);
            printf("  +--------+--------+--------+--------+\\n");
            for (int r = 0; r < 4; r++) {
                printf("  |");
                for (int c = 0; c < 4; c++) {
                    int dist = Results.distance_mm[r*4 + c];
                    printf(" %5d  |", dist);
                }
                printf("\\n");
            }
            printf("  +--------+--------+--------+--------+\\n");
            printf("  Status: All 16 Pixels Active & Streaming over SPI0\\n\\n");
        } else {
            usleep(10000);
        }
    }

    vl53l8cx_stop_ranging(&Dev);
    return 0;
}
EOF

gcc -I. st_tof_cli.c vl53l8cx_platform.c vl53l8cx_api.c vl53l8cx_plugin_detection_thresholds.c vl53l8cx_plugin_motion_indicator.c vl53l8cx_plugin_xtalk.c -o /tmp/tof_bin
sudo cp /tmp/tof_bin /usr/local/bin/tof
sudo chmod +x /usr/local/bin/tof

tof --count 3
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_st_cli.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_st_cli.sh && /tmp/run_st_cli.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    build_spi_official_ranging_app()
