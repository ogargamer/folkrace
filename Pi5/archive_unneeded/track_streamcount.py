import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def track_page2_streamcount():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  VERIFYING PAGE 2 STREAMCOUNT AUTO-INCREMENT & RANGING FRAMES")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > track_page2.c
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

    // Explicitly set Page 2 for continuous ranging stream
    VL53L8CX_WrByte(&Dev.platform, 0x7fff, 0x02);

    printf("\nPolling Page 2 @ 0x0000 for 10 distinct frames:\n");
    uint8_t last_count = 255;
    int frames_captured = 0;

    for (int i = 0; i < 50 && frames_captured < 10; i++) {
        VL53L8CX_WaitMs(&Dev.platform, 30);
        uint8_t hdr[4] = {0};
        VL53L8CX_RdMulti(&Dev.platform, 0x0000, hdr, 4);

        if (hdr[0] != last_count && hdr[0] != 0xFF) {
            frames_captured++;
            printf("  Frame #%02d -> Streamcount: %3u (Hex: 0x%02X), Header Bytes: [0x%02X 0x%02X 0x%02X 0x%02X]\n",
                   frames_captured, hdr[0], hdr[0], hdr[0], hdr[1], hdr[2], hdr[3]);
            last_count = hdr[0];
        }
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o track_page2 track_page2.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./track_page2
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_track_page2.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_track_page2.sh && /tmp/run_track_page2.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    track_page2_streamcount()
