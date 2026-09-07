import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def track_16_zones_live_across_frames():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  TRACKING REAL 16-ZONE DISTANCES ACROSS 5 CONSECUTIVE FRAMES")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > track_16_live.c
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "vl53l8cx_api.h"

int main() {
    VL53L8CX_Configuration Dev;
    memset(&Dev, 0, sizeof(Dev));
    Dev.platform.spi_num = 0;
    Dev.platform.spi_cs = 0;

    vl53l8cx_comms_init(&Dev.platform);
    uint8_t isAlive = 0;
    vl53l8cx_is_alive(&Dev, &isAlive);
    vl53l8cx_init(&Dev);

    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_frequency_hz(&Dev, 15);
    vl53l8cx_start_ranging(&Dev);

    printf("\nCapturing 5 frames of all 16 zones:\n");

    for (int frame = 1; frame <= 5; frame++) {
        VL53L8CX_WaitMs(&Dev.platform, 70);

        uint8_t sram[0x400] = {0};
        VL53L8CX_RdMulti(&Dev.platform, 0x0000, sram, 0x400);

        uint16_t streamcount = (sram[0x0020] << 8) | sram[0x0021];
        printf("\n==================== FRAME #%02d (Stream: %u) ====================\n",
               frame, streamcount);

        int offset = 0x0340;
        for (int r = 0; r < 4; r++) {
            printf("  [ ");
            for (int c = 0; c < 4; c++) {
                int z = r * 4 + c;
                int16_t dist = (int16_t)((sram[offset + z*2] << 8) | sram[offset + z*2 + 1]);
                if (dist > 4000) dist /= 4;
                printf("%5d mm  ", dist);
            }
            printf("]\n");
        }
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o track_16_live track_16_live.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./track_16_live
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_track16.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_track16.sh && /tmp/run_track16.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    track_16_zones_live_across_frames()
