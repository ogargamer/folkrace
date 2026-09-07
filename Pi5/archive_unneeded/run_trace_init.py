import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def trace_init_detailed():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  TRACING EXACT INIT REGISTER FAILURE")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > trace_init.c
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

    uint8_t dev_id = 0, rev_id = 0;
    VL53L8CX_WrByte(&Dev.platform, 0x7fff, 0x00);
    VL53L8CX_RdByte(&Dev.platform, 0x0000, &dev_id);
    VL53L8CX_RdByte(&Dev.platform, 0x0001, &rev_id);
    printf("Read ID: dev=0x%02X rev=0x%02X\n", dev_id, rev_id);

    uint8_t stat_06 = 0;
    VL53L8CX_RdByte(&Dev.platform, 0x0006, &stat_06);
    printf("Read Reg 0x0006: 0x%02X\n", stat_06);

    uint8_t stat_21 = 0;
    VL53L8CX_WrByte(&Dev.platform, 0x7fff, 0x01);
    VL53L8CX_RdByte(&Dev.platform, 0x0021, &stat_21);
    printf("Read Reg 0x0021 (page 1): 0x%02X\n", stat_21);

    uint8_t stat_cmd = 0;
    VL53L8CX_WrByte(&Dev.platform, 0x7fff, 0x00);
    VL53L8CX_RdByte(&Dev.platform, 0x2C00, &stat_cmd);
    printf("Read Reg 0x2C00 (UI cmd status): 0x%02X\n", stat_cmd);

    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o trace_init trace_init.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./trace_init
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_trace_init.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_trace_init.sh && /tmp/run_trace_init.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    trace_init_detailed()
