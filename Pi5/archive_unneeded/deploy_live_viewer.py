import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def deploy_perfect_16_zone_visualizer():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  DEPLOYING FULL 16-ZONE 4X4 DISTANCE MATRIX VISUALIZER ON PI 5")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

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
    memset(&Dev, 0, sizeof(Dev));
    Dev.platform.spi_num = 0;
    Dev.platform.spi_cs = 0;

    vl53l8cx_comms_init(&Dev.platform);
    uint8_t isAlive = 0;
    vl53l8cx_is_alive(&Dev, &isAlive);
    if (!isAlive) {
        printf("Error: VL53L8CX not found on SPI0.0!\n");
        return 1;
    }

    printf("Booting sensor with 84KB firmware in RAM...\n");
    if (vl53l8cx_init(&Dev) != 0) {
        printf("Error initializing sensor firmware!\n");
        return 1;
    }

    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_frequency_hz(&Dev, 15);

    VL53L8CX_WrByte(&Dev.platform, 0x7fff, 0x00);
    vl53l8cx_start_ranging(&Dev);

    printf("\033[2J\033[H"); // Clear screen

    uint32_t frame_count = 0;

    while (running) {
        uint8_t stat_reg = 0;
        for (int p = 0; p < 100 && running; p++) {
            VL53L8CX_WaitMs(&Dev.platform, 5);
            VL53L8CX_RdByte(&Dev.platform, 0x0006, &stat_reg);
            if (stat_reg == 0x01) break;
        }

        uint8_t sram[0x400] = {0};
        VL53L8CX_RdMulti(&Dev.platform, 0x0000, sram, 0x400);

        frame_count++;
        uint8_t streamcount = sram[0x0020 + 1];

        // Locate active distance block
        int offset = 0x0340;

        printf("\033[H"); // Move cursor home
        printf("=================================================================\n");
        printf("  VL53L8CX REAL-TIME 4x4 DISTANCE MATRIX (15 Hz Stream)\n");
        printf("=================================================================\n\n");
        printf("  Frame: #%-6u | Stream Count: %-3u | Status: ACTIVE RANGING\n\n", frame_count, streamcount);

        printf("  +-------------------------------------------------------------+\n");
        for (int r = 0; r < 4; r++) {
            printf("  | ");
            for (int c = 0; c < 4; c++) {
                int z = r * 4 + c;
                uint16_t raw_val = (sram[offset + z*2 + 1] << 8) | sram[offset + z*2];
                int16_t dist_mm = (raw_val > 4000) ? (raw_val / 4) : raw_val;

                if (dist_mm > 0 && dist_mm < 4000) {
                    printf("  %4d mm (Z%02d) ", dist_mm, z);
                } else {
                    printf("   ----  (Z%02d) ", z);
                }
            }
            printf(" |\n");
            if (r < 3) printf("  |                                                             |\n");
        }
        printf("  +-------------------------------------------------------------+\n\n");
        printf("  Press Ctrl+C to stop continuous streaming.\n");
        fflush(stdout);
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    printf("\n--> Sensor stopped cleanly.\n");
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

ls -la live_viewer
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/deploy_live.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/deploy_live.sh && /tmp/deploy_live.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    deploy_perfect_16_zone_visualizer()
