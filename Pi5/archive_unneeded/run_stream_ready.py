import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def test_live_stream_frame_ready():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  UPDATING vl53l8cx_check_data_ready FOR MM1.8 & PRINTING LIVE DISTANCES")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/uld-driver/src

# Python script to update vl53l8cx_check_data_ready
python3 - << 'PYEOF'
with open("vl53l8cx_api.c", "r") as f:
    text = f.read()

target = "if((p_dev->temp_buffer[0] != p_dev->streamcount)"
idx = text.find(target)
if idx >= 0:
    end_idx = text.find("{\n\t\t*p_isReady = (uint8_t)1;", idx)
    replacement = "if((p_dev->temp_buffer[3] & 0x01) || (p_dev->temp_buffer[0] != p_dev->streamcount))\n\t"
    text = text[:idx] + replacement + text[end_idx:]
    with open("vl53l8cx_api.c", "w") as f:
        f.write(text)
    print("Patched vl53l8cx_check_data_ready successfully.")
else:
    print("Pattern already modified.")
PYEOF

cd ../../test
make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI -Wno-error=unused-result -Wno-error=implicit-function-declaration" PLTF_DIR=../platform

cat << 'EOF' > live_ranging_stream.c
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
    printf("1. Check is_alive: %d (1 = Connected)\n", isAlive);

    printf("2. Initializing VL53L8CX 84KB Firmware in RAM...\n");
    int status = vl53l8cx_init(&Dev);
    printf("3. vl53l8cx_init status: %d (0 = SUCCESS!)\n\n", status);

    if (status == 0) {
        vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
        vl53l8cx_set_ranging_frequency_hz(&Dev, 15);
        vl53l8cx_start_ranging(&Dev);

        printf("=================================================================\n");
        printf("  VL53L8CX LIVE 4x4 DISTANCE MATRIX STREAM (15 Hz Continuous)\n");
        printf("=================================================================\n\n");

        for (int frame = 1; frame <= 25; frame++) {
            uint8_t isReady = 0;
            for (int poll = 0; poll < 100; poll++) {
                VL53L8CX_WaitMs(&Dev.platform, 5);
                vl53l8cx_check_data_ready(&Dev, &isReady);
                if (isReady) break;
            }

            if (isReady) {
                vl53l8cx_get_ranging_data(&Dev, &Results);
                printf("--- Frame #%02d [Stream Count: %u] ---\n", frame, Dev.streamcount);
                for (int r = 0; r < 4; r++) {
                    for (int c = 0; c < 4; c++) {
                        int idx = r * 4 + c;
                        int16_t dist = Results.distance_mm[idx];
                        uint8_t stat = Results.target_status[idx];
                        if (stat == 5 || stat == 6 || stat == 9 || stat == 10 || dist > 0) {
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
        printf("--> Live ranging session completed successfully!\n");
    }

    vl53l8cx_comms_close(&Dev.platform);
    return status;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o live_ranging_stream live_ranging_stream.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./live_ranging_stream
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_stream_ready.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_stream_ready.sh && /tmp/run_stream_ready.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_live_stream_frame_ready()
