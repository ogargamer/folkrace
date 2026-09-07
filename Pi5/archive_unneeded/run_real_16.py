import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def test_page0_start_command():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  TRIGGERING START RANGING ON PAGE 0 & READING ACTIVE 16-ZONE STREAM")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > test_real_16.c
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

    // Make sure Page 0 is selected for UI command mailbox
    VL53L8CX_WrByte(&Dev.platform, 0x7fff, 0x00);
    vl53l8cx_start_ranging(&Dev);

    printf("\nCapturing 5 active ranging frames across all 16 zones:\n");

    for (int frame = 1; frame <= 5; frame++) {
        uint8_t isReady = 0;
        for (int p = 0; p < 100; p++) {
            VL53L8CX_WaitMs(&Dev.platform, 5);
            vl53l8cx_check_data_ready(&Dev, &isReady);
            if (isReady) break;
        }

        printf("\n--- FRAME #%02d (isReady = %d) ---\n", frame, isReady);
        uint8_t buf[600] = {0};
        VL53L8CX_RdMulti(&Dev.platform, 0x0000, buf, 548);

        // Find distance block (idx 0x5440 or 0x5430)
        int d_offset = -1;
        int s_offset = -1;

        for (int i = 16; i < 540; i += 4) {
            uint16_t idx = (buf[i+2] << 8) | buf[i+3];
            uint16_t idx_le = (buf[i+1] << 8) | buf[i];
            if (idx == 0x5440 || idx_le == 0x5440) d_offset = i + 4;
            if (idx == 0x5480 || idx_le == 0x5480) s_offset = i + 4;
        }

        printf("Offsets: Distance = %d, Status = %d\n", d_offset, s_offset);
        printf("16-Zone Distances (mm):\n");
        for (int r = 0; r < 4; r++) {
            printf("  [ ");
            for (int c = 0; c < 4; c++) {
                int z = r * 4 + c;
                int16_t dist = 0;
                if (d_offset > 0) {
                    dist = (int16_t)((buf[d_offset + z*2] << 8) | buf[d_offset + z*2 + 1]);
                    dist /= 4;
                }
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
    -o test_real_16 test_real_16.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./test_real_16
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_real_16.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_real_16.sh && /tmp/run_real_16.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_page0_start_command()
