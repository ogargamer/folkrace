import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def sample_live_sensor_readings():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  SAMPLING LIVE 4X4 DISTANCE READINGS DIRECTLY FROM SENSOR")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > live_sample.c
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
    Dev.platform.spi_num = 0;
    Dev.platform.spi_cs = 0;

    vl53l8cx_comms_init(&Dev.platform);
    vl53l8cx_is_alive(&Dev, &isAlive);
    if (!isAlive) {
        printf("Sensor is_alive check failed!\n");
        return 1;
    }

    if (vl53l8cx_init(&Dev) != 0) {
        printf("Sensor init failed!\n");
        return 1;
    }

    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_frequency_hz(&Dev, 15);
    vl53l8cx_start_ranging(&Dev);

    printf("\n--- CAPTURING 10 LIVE 4x4 MATRIX FRAMES ---\n\n");

    int valid_frames = 0;
    for (int attempt = 1; attempt <= 30 && valid_frames < 10; attempt++) {
        uint8_t isReady = 0;
        for (int p = 0; p < 100; p++) {
            VL53L8CX_WaitMs(&Dev.platform, 5);
            vl53l8cx_check_data_ready(&Dev, &isReady);
            if (isReady) break;
        }

        if (isReady) {
            valid_frames++;
            vl53l8cx_get_ranging_data(&Dev, &Results);
            printf("Frame #%02d (Stream count: %u, Rate: 15 Hz):\n", valid_frames, Dev.streamcount);
            for (int r = 0; r < 4; r++) {
                printf("  [ ");
                for (int c = 0; c < 4; c++) {
                    int idx = r * 4 + c;
                    int16_t dist = Results.distance_mm[idx];
                    uint8_t stat = Results.target_status[idx];
                    if (stat == 5 || stat == 6 || stat == 9 || stat == 10 || (dist > 0 && dist < 4000)) {
                        printf("%5d mm  ", dist);
                    } else {
                        printf("   ----   ");
                    }
                }
                printf("]\n");
            }
            printf("\n");
            fflush(stdout);
        }
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    printf("--> Sample completed cleanly.\n");
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o live_sample live_sample.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./live_sample
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_sample.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_sample.sh && /tmp/run_sample.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    sample_live_sensor_readings()
