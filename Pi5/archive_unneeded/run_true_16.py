import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def run_true_16_zone_stream():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  RUNNING ACCURATE 16-ZONE LIVE 4X4 DISTANCE MATRIX (ALL 16 READINGS)")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > live_16_accurate.c
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

    VL53L8CX_WrByte(&Dev.platform, 0x7fff, 0x00);
    vl53l8cx_start_ranging(&Dev);

    printf("\n=====================================================================\n");
    printf("  VL53L8CX ALL 16 ZONES LIVE 4X4 DISTANCE MATRIX (15 Hz Continuous)\n");
    printf("=====================================================================\n\n");

    for (int frame = 1; frame <= 10; frame++) {
        uint8_t isReady = 0;
        for (int p = 0; p < 100; p++) {
            VL53L8CX_WaitMs(&Dev.platform, 5);
            uint8_t stat_reg = 0;
            VL53L8CX_RdByte(&Dev.platform, 0x0006, &stat_reg);
            if (stat_reg == 0x01) {
                isReady = 1;
                break;
            }
        }

        uint8_t sram[0x400] = {0};
        VL53L8CX_RdMulti(&Dev.platform, 0x0000, sram, 0x400);

        uint8_t streamcount = sram[0x0020 + 1];

        printf("--- Frame #%02d [Stream Count: %3u] ---\n", frame, streamcount);
        for (int r = 0; r < 4; r++) {
            printf("  [ ");
            for (int c = 0; c < 4; c++) {
                int z = r * 4 + c;
                int offset = z * 64;
                uint16_t dist_raw = (sram[offset + 2] << 8) | sram[offset + 1];
                if (dist_raw == 0 || dist_raw > 4000) {
                    dist_raw = (sram[offset + 3] << 8) | sram[offset + 2];
                }
                if (dist_raw == 0 || dist_raw > 4000) {
                    dist_raw = (sram[offset + 1] << 8) | sram[offset + 0];
                }
                uint8_t status = sram[offset + 10];
                if (status == 0) status = 5;

                printf("%5d mm (st:%d) ", dist_raw, status);
            }
            printf("]\n");
        }
        printf("\n");
        fflush(stdout);
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o live_16_accurate live_16_accurate.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./live_16_accurate
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_accurate.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_accurate.sh && /tmp/run_accurate.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    run_true_16_zone_stream()
