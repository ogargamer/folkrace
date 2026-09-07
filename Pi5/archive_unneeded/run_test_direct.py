import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def test_rdbyte_directly():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "192.168.0.13"
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)

    print("======================================================================")
    print("  TESTING C VL53L8CX_RdByte AND is_alive DIRECTLY")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > test_alive_direct.c
#include <stdio.h>
#include <string.h>
#include "vl53l8cx_api.h"

int main() {
    VL53L8CX_Configuration Dev;
    memset(&Dev, 0, sizeof(Dev));
    Dev.platform.spi_num = 0;
    Dev.platform.spi_cs = 0;

    int status = vl53l8cx_comms_init(&Dev.platform);
    printf("Comms init fd: %d\n", Dev.platform.fd);

    uint8_t dev_id = 0, rev_id = 0, isAlive = 0;

    VL53L8CX_WrByte(&Dev.platform, 0x7fff, 0x00);
    VL53L8CX_RdByte(&Dev.platform, 0x0000, &dev_id);
    VL53L8CX_RdByte(&Dev.platform, 0x0001, &rev_id);
    printf("VL53L8CX_RdByte -> dev_id: 0x%02X, rev_id: 0x%02X\n", dev_id, rev_id);

    status = vl53l8cx_is_alive(&Dev, &isAlive);
    printf("vl53l8cx_is_alive returned: %d, isAlive: %d\n", status, isAlive);

    if (isAlive) {
        printf("Sensor is ALIVE! Initializing ULD...\n");
        status = vl53l8cx_init(&Dev);
        printf("vl53l8cx_init status: %d\n", status);

        if (status == 0) {
            printf("Starting ranging...\n");
            vl53l8cx_start_ranging(&Dev);
            VL53L8CX_ResultsData res;
            for (int i = 0; i < 5; i++) {
                if (VL53L8CX_wait_for_dataready(&Dev.platform)) {
                    vl53l8cx_get_ranging_data(&Dev, &res);
                    printf("Frame %d -> Center Distance: %d mm (status %d)\n",
                           i, res.distance_mm[5], res.target_status[5]);
                }
            }
            vl53l8cx_stop_ranging(&Dev);
        }
    }

    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o test_alive_direct test_alive_direct.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./test_alive_direct
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/test_direct.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/test_direct.sh && /tmp/test_direct.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_rdbyte_directly()
