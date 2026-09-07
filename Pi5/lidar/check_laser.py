import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def check_laser_safety_status():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  CHECKING SENSOR LASER SAFETY & STREAMCOUNT AUTO-INCREMENT")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > check_laser.c
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

    // Read laser safety register 0xE0C4
    uint8_t safety_buf[8] = {0};
    vl53l8cx_dci_read_data(&Dev, safety_buf, 0xE0C4, 8);
    printf("Laser Safety Register (0xE0C4): [0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X]\n",
           safety_buf[0], safety_buf[1], safety_buf[2], safety_buf[3],
           safety_buf[4], safety_buf[5], safety_buf[6], safety_buf[7]);

    printf("\nPolling Stream Count over 2 seconds (expecting incrementing values):\n");
    for (int i = 0; i < 20; i++) {
        VL53L8CX_WaitMs(&Dev.platform, 100);
        uint8_t hdr[4] = {0};
        VL53L8CX_RdMulti(&Dev.platform, 0x0000, hdr, 4);
        printf("  Tick %02d: Streamcount = %3u (0x%02X), Status = [0x%02X 0x%02X 0x%02X]\n",
               i, hdr[0], hdr[0], hdr[1], hdr[2], hdr[3]);
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o check_laser check_laser.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./check_laser
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_laser.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_laser.sh && /tmp/run_laser.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    check_laser_safety_status()
