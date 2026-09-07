import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def run_official_st_driver_spi1():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  RUNNING ORIGINAL ST OFFICIAL DRIVER ON SPI1.0")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > run_spi1_official.c
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "vl53l8cx_api.h"

int main() {
    VL53L8CX_Configuration Dev;
    VL53L8CX_ResultsData Results;
    uint8_t isAlive = 0;

    memset(&Dev, 0, sizeof(Dev));
    memset(&Results, 0, sizeof(Results));
    
    // Set to SPI1.0 (/dev/spidev1.0)
    Dev.platform.spi_num = 1;
    Dev.platform.spi_cs = 0;

    printf("1. Initializing comms on /dev/spidev1.0...\n");
    int32_t comms_status = vl53l8cx_comms_init(&Dev.platform);
    if (comms_status != 0) {
        printf("Failed to open /dev/spidev1.0!\n");
        return 1;
    }

    printf("2. Checking sensor is_alive...\n");
    uint8_t status = vl53l8cx_is_alive(&Dev, &isAlive);
    printf("   is_alive: %d (status: %d)\n", isAlive, status);

    if (!isAlive) {
        printf("Error: Sensor not alive on SPI1.0!\n");
        vl53l8cx_comms_close(&Dev.platform);
        return 1;
    }

    printf("3. Initializing VL53L8CX 84KB firmware in RAM (Official ST init)...\n");
    status = vl53l8cx_init(&Dev);
    printf("   vl53l8cx_init status: %d (0 = Success)\n", status);

    if (status == 0) {
        printf("4. Starting ranging at 15 Hz...\n");
        vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
        vl53l8cx_set_ranging_frequency_hz(&Dev, 15);
        vl53l8cx_start_ranging(&Dev);

        printf("\n=================================================================\n");
        printf("  OFFICIAL ST RANGING CAPTURE ON SPI1 (10 FRAMES)\n");
        printf("=================================================================\n\n");

        for (int frame = 1; frame <= 10; frame++) {
            uint8_t isReady = 0;
            for (int p = 0; p < 100; p++) {
                VL53L8CX_WaitMs(&Dev.platform, 5);
                vl53l8cx_check_data_ready(&Dev, &isReady);
                if (isReady) break;
            }

            if (isReady) {
                vl53l8cx_get_ranging_data(&Dev, &Results);
                printf("--- Frame #%02d [Stream Count: %3u] ---\n", frame, Dev.streamcount);
                for (int r = 0; r < 4; r++) {
                    printf("  [ ");
                    for (int c = 0; c < 4; c++) {
                        int idx = r * 4 + c;
                        printf("%5d mm (st:%2u) ", Results.distance_mm[idx], Results.target_status[idx]);
                    }
                    printf("]\n");
                }
                printf("\n");
                fflush(stdout);
            } else {
                printf("Frame #%02d: data_ready waiting...\n", frame);
            }
        }

        vl53l8cx_stop_ranging(&Dev);
        printf("--> Ranging stopped cleanly.\n");
    }

    vl53l8cx_comms_close(&Dev.platform);
    return status;
}
EOF

gcc -Wall -Wno-missing-braces -Os -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o run_spi1_official run_spi1_official.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./run_spi1_official
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_spi1_official.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_spi1_official.sh && /tmp/run_spi1_official.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    run_official_st_driver_spi1()
