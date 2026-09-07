import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def inspect_all_16_zones():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  INSPECTING ALL 16 ZONES & TARGET STATUS CODES")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > inspect_16.c
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
    vl53l8cx_init(&Dev);

    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_frequency_hz(&Dev, 15);
    vl53l8cx_start_ranging(&Dev);

    printf("\nWaiting for 3 frames...\n");
    for (int f = 1; f <= 5; f++) {
        uint8_t isReady = 0;
        for (int p = 0; p < 100; p++) {
            VL53L8CX_WaitMs(&Dev.platform, 5);
            vl53l8cx_check_data_ready(&Dev, &isReady);
            if (isReady) break;
        }

        if (isReady) {
            vl53l8cx_get_ranging_data(&Dev, &Results);
            printf("\n--- FRAME %d ---\n", f);
            printf("Zone | Distance (mm) | Target Status | Signal Rate (kcps/SPAD) | Ambient (kcps/SPAD)\n");
            printf("-----------------------------------------------------------------------------\n");
            for (int z = 0; z < 16; z++) {
                printf(" %2d  | %10d mm |      0x%02X     |        %8u         |       %8u\n",
                       z, Results.distance_mm[z], Results.target_status[z],
                       Results.signal_per_spad[z], Results.ambient_per_spad[z]);
            }
        }
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o inspect_16 inspect_16.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./inspect_16
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_inspect.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_inspect.sh && /tmp/run_inspect.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    inspect_all_16_zones()
