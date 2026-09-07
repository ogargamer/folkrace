import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def test_aligned_start_command():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  TESTING DIRECT 16-BIT ALIGNED START COMMAND AT 0x2FFC")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > test_aligned_start.c
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

    // Write start command directly to 0x2FFC on Page 0
    VL53L8CX_WrByte(&Dev.platform, 0x7fff, 0x00);
    uint8_t start_cmd[4] = {0x00, 0x03, 0x00, 0x00};
    VL53L8CX_WrMulti(&Dev.platform, 0x2FFC, start_cmd, 4);

    // Poll for status 0x03 on 0x2C00
    uint8_t status_reply[4] = {0};
    for (int p = 0; p < 50; p++) {
        VL53L8CX_WaitMs(&Dev.platform, 5);
        VL53L8CX_RdMulti(&Dev.platform, 0x2C00, status_reply, 4);
        if (status_reply[0] == 0x03) break;
    }
    printf("Status reply from 0x2C00: [0x%02X 0x%02X 0x%02X 0x%02X]\n",
           status_reply[0], status_reply[1], status_reply[2], status_reply[3]);

    printf("\nPolling Stream Count over 2 seconds (15 Hz):\n");
    for (int i = 1; i <= 10; i++) {
        VL53L8CX_WaitMs(&Dev.platform, 100);
        uint8_t sram[0x400] = {0};
        VL53L8CX_RdMulti(&Dev.platform, 0x0000, sram, 0x400);

        uint16_t sc = (sram[0x0020] << 8) | sram[0x0021];
        printf("Tick %02d: Streamcount = %3u (Hex: 0x%04X)\n", i, sc, sc);
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o test_aligned_start test_aligned_start.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./test_aligned_start
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_aligned.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_aligned.sh && /tmp/run_aligned.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_aligned_start_command()
