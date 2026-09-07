import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def test_fixed_struct():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "192.168.0.13"
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)

    print("======================================================================")
    print("  TESTING FIXED STRUCT LAYOUT & RUNNING RANGING IN C")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0

cat << 'EOF' > user/platform/platform.h
#ifndef PLATFORM_H_
#define PLATFORM_H_
#include <stdint.h>
#include <string.h>

typedef struct {
    int fd;
    uint16_t address;
    uint8_t spi_num;
    uint8_t spi_cs;
} VL53L8CX_Platform;

#define VL53L8CX_NB_TARGET_PER_ZONE 1U

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t reg_address, uint8_t *p_value);
uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t reg_address, uint8_t value);
uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t reg_address, uint8_t *p_values, uint32_t size);
uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform, uint16_t reg_address, uint8_t *p_values, uint32_t size);
void VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p_platform, uint32_t time_ms);
uint8_t VL53L8CX_wait_for_dataready(VL53L8CX_Platform *p_platform);
int32_t vl53l8cx_comms_init(VL53L8CX_Platform *p_platform);
int32_t vl53l8cx_comms_close(VL53L8CX_Platform *p_platform);

#endif
EOF

cd user/test
make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI" PLTF_DIR=../platform

cat << 'EOF' > test_solid_range.c
#include <stdio.h>
#include <string.h>
#include "vl53l8cx_api.h"

int main() {
    VL53L8CX_Configuration Dev;
    memset(&Dev, 0, sizeof(Dev));
    Dev.platform.spi_num = 0;
    Dev.platform.spi_cs = 0;

    int status = vl53l8cx_comms_init(&Dev.platform);
    printf("SPI fd opened: %d\n", Dev.platform.fd);

    uint8_t isAlive = 0;
    vl53l8cx_is_alive(&Dev, &isAlive);
    printf("vl53l8cx_is_alive: isAlive = %d\n", isAlive);

    printf("Uploading 84KB Firmware...\n");
    status = vl53l8cx_init(&Dev);
    printf("vl53l8cx_init: status = %d (0 = SUCCESS)\n", status);

    if (status == 0) {
        printf("\n=======================================================\n");
        printf("  STREAMING LIVE DISTANCE MATRIX FROM VL53L8CX\n");
        printf("=======================================================\n\n");

        vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
        vl53l8cx_set_ranging_frequency_hz(&Dev, 15);
        vl53l8cx_start_ranging(&Dev);

        VL53L8CX_ResultsData Results;
        for (int frame = 1; frame <= 10; frame++) {
            if (VL53L8CX_wait_for_dataready(&Dev.platform)) {
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
    }

    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o test_solid_range test_solid_range.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./test_solid_range
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_solid.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_solid.sh && /tmp/run_solid.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_fixed_struct()
