import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def test_unscrambled_16_zones():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  TESTING DIRECT UNSCRAMBLED 16-ZONE DISTANCE MATRIX")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > test_unscrambled.c
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

    printf("\nWaiting for clean frame...\n");
    for (int frame = 1; frame <= 5; frame++) {
        uint8_t isReady = 0;
        for (int p = 0; p < 100; p++) {
            VL53L8CX_WaitMs(&Dev.platform, 5);
            vl53l8cx_check_data_ready(&Dev, &isReady);
            if (isReady) break;
        }

        if (isReady) {
            VL53L8CX_RdMulti(&Dev.platform, 0x0, Dev.temp_buffer, Dev.data_read_size);

            printf("\n=======================================================\n");
            printf("  RAW 4x4 MATRIX DISTANCES & TARGET STATUS (Frame %d)\n", frame);
            printf("=======================================================\n");

            // Look for Distance Block Header (0x5440) in un-swapped buffer
            int dist_offset = -1;
            int status_offset = -1;

            for (int i = 16; i < Dev.data_read_size - 4; i += 4) {
                uint16_t idx = (Dev.temp_buffer[i+2] << 8) | Dev.temp_buffer[i+3];
                uint16_t idx_le = (Dev.temp_buffer[i+1] << 8) | Dev.temp_buffer[i];

                if (idx == 0x5440 || idx_le == 0x5440) {
                    dist_offset = i + 4;
                }
                if (idx == 0x5490 || idx_le == 0x5490 || idx == 0x5480 || idx_le == 0x5480) {
                    status_offset = i + 4;
                }
            }

            printf("Distance Offset: 0x%04X, Status Offset: 0x%04X\n\n", dist_offset, status_offset);

            // Print 4x4 Distance Grid (16 zones)
            printf("--- 4x4 DISTANCE MATRIX (mm) ---\n");
            for (int r = 0; r < 4; r++) {
                printf("  [ ");
                for (int c = 0; c < 4; c++) {
                    int z = r * 4 + c;
                    int16_t d_raw = 0;
                    if (dist_offset > 0) {
                        d_raw = (int16_t)((Dev.temp_buffer[dist_offset + z*2 + 1] << 8) | Dev.temp_buffer[dist_offset + z*2]);
                        d_raw /= 4; // 14.2 fixed point
                    }
                    printf("%5d mm  ", d_raw);
                }
                printf("]\n");
            }

            printf("\n--- 4x4 TARGET STATUS (5/6/9/10 = Valid) ---\n");
            for (int r = 0; r < 4; r++) {
                printf("  [ ");
                for (int c = 0; c < 4; c++) {
                    int z = r * 4 + c;
                    uint8_t st = (status_offset > 0) ? Dev.temp_buffer[status_offset + z] : 0;
                    printf("  0x%02X   ", st);
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
    -o test_unscrambled test_unscrambled.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./test_unscrambled
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_unscrambled.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_unscrambled.sh && /tmp/run_unscrambled.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_unscrambled_16_zones()
