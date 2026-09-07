import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def test_true_ranging_without_swap_corruption():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  FIXING SwapBuffer IN vl53l8cx_get_ranging_data & TESTING ALL 16 ZONES")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/uld-driver/src

# Python script to fix vl53l8cx_get_ranging_data
python3 - << 'PYEOF'
with open("vl53l8cx_api.c", "r") as f:
    text = f.read()

# In vl53l8cx_get_ranging_data, comment out the SwapBuffer call
old_swap = "VL53L8CX_SwapBuffer(p_dev->temp_buffer, (uint16_t)p_dev->data_read_size);"
new_swap = "/* VL53L8CX_SwapBuffer not needed on Little-Endian ARM64 SPI */"

if old_swap in text:
    text = text.replace(old_swap, new_swap)
    print("Disabled SwapBuffer in get_ranging_data.")

with open("vl53l8cx_api.c", "w") as f:
    f.write(text)
PYEOF

cd ../../test
make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI -Wno-error=unused-result -Wno-error=implicit-function-declaration" PLTF_DIR=../platform

cat << 'EOF' > run_all_16.c
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
    Dev.platform.spi_num = 0;
    Dev.platform.spi_cs = 0;

    vl53l8cx_comms_init(&Dev.platform);
    vl53l8cx_is_alive(&Dev, &isAlive);
    vl53l8cx_init(&Dev);

    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_frequency_hz(&Dev, 15);
    vl53l8cx_start_ranging(&Dev);

    printf("\n===================================================================\n");
    printf("  ALL 16 ZONES LIVE MATRIX STREAM (15 Hz Continuous Ranging)\n");
    printf("===================================================================\n\n");

    for (int frame = 1; frame <= 10; frame++) {
        uint8_t isReady = 0;
        for (int p = 0; p < 100; p++) {
            VL53L8CX_WaitMs(&Dev.platform, 5);
            vl53l8cx_check_data_ready(&Dev, &isReady);
            if (isReady) break;
        }

        if (isReady) {
            vl53l8cx_get_ranging_data(&Dev, &Results);
            printf("--- Frame #%02d (Stream Count: %u) ---\n", frame, Dev.streamcount);
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
        }
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o run_all_16 run_all_16.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./run_all_16
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_unswapped.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_unswapped.sh && /tmp/run_unswapped.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_true_ranging_without_swap_corruption()
