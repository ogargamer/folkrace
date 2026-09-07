import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def test_full_548_byte_frame():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  READING FULL 548-BYTE FRAME TO EXTRACT ALL 16 ZONES")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > test_548.c
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

    // Read full 548 bytes
    uint32_t full_size = 548;
    Dev.data_read_size = full_size;

    printf("\nWaiting for frame with full 548 bytes...\n");
    for (int frame = 1; frame <= 5; frame++) {
        uint8_t isReady = 0;
        for (int p = 0; p < 100; p++) {
            VL53L8CX_WaitMs(&Dev.platform, 5);
            vl53l8cx_check_data_ready(&Dev, &isReady);
            if (isReady) break;
        }

        if (isReady) {
            VL53L8CX_RdMulti(&Dev.platform, 0x0, Dev.temp_buffer, full_size);

            printf("\n=======================================================\n");
            printf("  ALL 16 ZONES DISTANCE MATRIX & STATUS (Frame %d)\n", frame);
            printf("=======================================================\n");

            // Look for distance and status headers in the 548-byte buffer
            int dist_idx = -1;
            int stat_idx = -1;

            for (uint32_t i = 16; i < full_size - 4; i += 4) {
                uint32_t w = (Dev.temp_buffer[i]<<24) | (Dev.temp_buffer[i+1]<<16) | (Dev.temp_buffer[i+2]<<8) | Dev.temp_buffer[i+3];
                uint16_t bh_idx = (w >> 16) & 0xFFFF;
                if (bh_idx == 0x5440 || bh_idx == 0x5430) {
                    dist_idx = i + 4;
                }
                if (bh_idx == 0x5480 || bh_idx == 0x5490 || bh_idx == 0x5460) {
                    stat_idx = i + 4;
                }
            }

            printf("Block offsets: Dist = %d, Stat = %d\n\n", dist_idx, stat_idx);

            printf("--- 4x4 DISTANCE MATRIX (mm) ---\n");
            for (int r = 0; r < 4; r++) {
                printf("  [ ");
                for (int c = 0; c < 4; c++) {
                    int z = r * 4 + c;
                    int16_t d = 0;
                    if (dist_idx > 0 && dist_idx + z*2 + 1 < full_size) {
                        d = (int16_t)((Dev.temp_buffer[dist_idx + z*2] << 8) | Dev.temp_buffer[dist_idx + z*2 + 1]);
                        d /= 4; // 14.2 fixed point
                    }
                    printf("%5d mm  ", d);
                }
                printf("]\n");
            }

            printf("\n--- 4x4 TARGET STATUS (5/6/9/10 = Valid) ---\n");
            for (int r = 0; r < 4; r++) {
                printf("  [ ");
                for (int c = 0; c < 4; c++) {
                    int z = r * 4 + c;
                    uint8_t s = (stat_idx > 0 && stat_idx + z < full_size) ? Dev.temp_buffer[stat_idx + z] : 0;
                    printf("  0x%02X   ", s);
                }
                printf("]\n");
            }
        }
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o test_548 test_548.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./test_548
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_548.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_548.sh && /tmp/run_548.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_full_548_byte_frame()
