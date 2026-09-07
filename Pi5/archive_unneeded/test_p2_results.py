import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "10.43.8.42"
PI_USER = "admin"
PI_PASS = "l"

def test_page2_results():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  TESTING PAGE 2 (0x7FFF=0x02) RANGING BUFFER")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > test_p2.c
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "vl53l8cx_api.h"

int main() {
    VL53L8CX_Configuration Dev;
    memset(&Dev, 0, sizeof(Dev));
    Dev.platform.spi_num = 1;
    Dev.platform.spi_cs = 0;

    vl53l8cx_comms_init(&Dev.platform);
    vl53l8cx_init(&Dev);
    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_frequency_hz(&Dev, 15);
    vl53l8cx_start_ranging(&Dev);

    VL53L8CX_WaitMs(&Dev.platform, 200);

    for(int f = 0; f < 3; f++) {
        uint8_t p0_hdr[16], p2_hdr[16];
        
        // Read Page 0
        VL53L8CX_WrByte(&Dev.platform, 0x7fff, 0x00);
        VL53L8CX_RdMulti(&Dev.platform, 0x0, p0_hdr, 16);
        
        // Read Page 2
        VL53L8CX_WrByte(&Dev.platform, 0x7fff, 0x02);
        VL53L8CX_RdMulti(&Dev.platform, 0x0, p2_hdr, 16);

        printf("Frame %d Page 0 [0x00..0x0F]: ", f);
        for(int k=0; k<16; k++) printf("%02X ", p0_hdr[k]);
        printf("\n");

        printf("Frame %d Page 2 [0x00..0x0F]: ", f);
        for(int k=0; k<16; k++) printf("%02X ", p2_hdr[k]);
        printf("\n\n");

        VL53L8CX_WaitMs(&Dev.platform, 70);
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o test_p2 test_p2.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./test_p2
"""

    stdin, stdout, stderr = ssh.exec_command(script, get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    ssh.close()

if __name__ == "__main__":
    test_page2_results()
