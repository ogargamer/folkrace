import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def test_reading_from_2c04():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  TESTING MEASUREMENT STREAM FROM 0x2C04 (ALL 16 ZONES)")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > test_2c04.c
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

    printf("Starting 5 frames capture from 0x2C04...\n");

    for (int frame = 1; frame <= 5; frame++) {
        uint8_t isReady = 0;
        for (int p = 0; p < 100; p++) {
            VL53L8CX_WaitMs(&Dev.platform, 5);
            vl53l8cx_check_data_ready(&Dev, &isReady);
            if (isReady) break;
        }

        if (isReady) {
            uint8_t buf[600] = {0};
            // Read from 0x2C04
            VL53L8CX_RdMulti(&Dev.platform, 0x2C04, buf, 548);

            printf("\n=================== FRAME %d @ 0x2C04 ===================\n", frame);
            printf("First 32 bytes:\n");
            for (int b = 0; b < 32; b++) {
                printf("%02X ", buf[b]);
                if ((b+1)%16 == 0) printf("\n");
            }
            printf("\n");

            // Look for Distance header 0x5440 or 0x5430
            for (int i = 0; i < 500; i += 4) {
                uint16_t idx = (buf[i+2] << 8) | buf[i+3];
                uint16_t idx_le = (buf[i+1] << 8) | buf[i];
                if (idx == 0x5440 || idx_le == 0x5440) {
                    printf("Found 0x5440 Distance Block at offset %d!\n", i);
                    printf("16-Zone Distances (mm):\n");
                    for (int r = 0; r < 4; r++) {
                        printf("  [ ");
                        for (int c = 0; c < 4; c++) {
                            int z = r * 4 + c;
                            int16_t d = (int16_t)((buf[i + 4 + z*2 + 1] << 8) | buf[i + 4 + z*2]);
                            d /= 4;
                            printf("%5d mm  ", d);
                        }
                        printf("]\n");
                    }
                }
            }
        }
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o test_2c04 test_2c04.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./test_2c04
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_2c04.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_2c04.sh && /tmp/run_2c04.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_reading_from_2c04()
