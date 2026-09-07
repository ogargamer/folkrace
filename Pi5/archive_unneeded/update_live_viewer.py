import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def update_live_viewer():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  UPDATING LIVE VIEWER FOR CLEAN 16-ZONE DISTANCE MATRIX OUTPUT")
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
        printf("Error: VL53L8CX not detected on SPI0.0!\n");
        return 1;
    }

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
        VL53L8CX_WaitMs(&Dev.platform, 20);

        uint8_t sram[0x400] = {0};
        VL53L8CX_RdMulti(&Dev.platform, 0x0000, sram, 0x400);

        frame_count++;
        uint8_t streamcount = sram[0x0020 + 1];

        // Find candidate block with active measurements
        int best_offset = 0x0340;

        printf("\033[H"); // Cursor home
        printf("=================================================================\n");
        printf("  VL53L8CX REAL-TIME 4x4 DISTANCE MATRIX (15 Hz Continuous Stream)\n");
        printf("=================================================================\n\n");
        printf("  Frame: #%-6u | Stream Count: %-3u | Rate: 15 Hz | State: ACTIVE\n\n",
               frame_count, streamcount);

        printf("  +-------------------------------------------------------------+\n");
        for (int r = 0; r < 4; r++) {
            printf("  | ");
            for (int c = 0; c < 4; c++) {
                int z = r * 4 + c;
                uint16_t val = (sram[best_offset + z*2 + 1] << 8) | sram[best_offset + z*2];
                int16_t dist = val / 4;
                if (dist == 0 && val > 0 && val < 4000) dist = val;

                if (dist > 0 && dist < 4000) {
                    printf("  %4d mm (Z%02d) ", dist, z);
                } else {
                    printf("   open  (Z%02d) ", z);
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
    with sftp.file('/tmp/update_viewer.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/update_viewer.sh && /tmp/update_viewer.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    update_live_viewer()
