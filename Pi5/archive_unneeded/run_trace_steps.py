import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def trace_init_line_by_line():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "192.168.0.13"
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)

    print("======================================================================")
    print("  TRACING vl53l8cx_init STEP-BY-STEP IN C")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > trace_steps.c
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

    uint8_t tmp = 0, status = 0;
    uint8_t buf[4] = {0};

    printf("Step 1: Check ID...\n");
    VL53L8CX_WrByte(&Dev.platform, 0x7fff, 0x00);
    VL53L8CX_RdMulti(&Dev.platform, 0x0000, buf, 4);
    printf("  ID: [0]=0x%02X [1]=0x%02X [2]=0x%02X [3]=0x%02X\n", buf[0], buf[1], buf[2], buf[3]);

    printf("Step 2: SW Reboot Sequence...\n");
    VL53L8CX_WrByte(&Dev.platform, 0x7fff, 0x00);
    VL53L8CX_WrByte(&Dev.platform, 0x0009, 0x04);
    VL53L8CX_WrByte(&Dev.platform, 0x000F, 0x40);
    VL53L8CX_WrByte(&Dev.platform, 0x000A, 0x03);
    VL53L8CX_RdByte(&Dev.platform, 0x7FFF, &tmp);
    VL53L8CX_WrByte(&Dev.platform, 0x000C, 0x01);
    VL53L8CX_WrByte(&Dev.platform, 0x0101, 0x00);
    VL53L8CX_WrByte(&Dev.platform, 0x0102, 0x00);
    VL53L8CX_WrByte(&Dev.platform, 0x010A, 0x01);
    VL53L8CX_WrByte(&Dev.platform, 0x4002, 0x01);
    VL53L8CX_WrByte(&Dev.platform, 0x4002, 0x00);
    VL53L8CX_WrByte(&Dev.platform, 0x010A, 0x03);
    VL53L8CX_WrByte(&Dev.platform, 0x0103, 0x01);
    VL53L8CX_WrByte(&Dev.platform, 0x000C, 0x00);
    VL53L8CX_WrByte(&Dev.platform, 0x000F, 0x43);
    VL53L8CX_WaitMs(&Dev.platform, 1);
    VL53L8CX_WrByte(&Dev.platform, 0x000F, 0x40);
    VL53L8CX_WrByte(&Dev.platform, 0x000A, 0x01);
    VL53L8CX_WaitMs(&Dev.platform, 100);

    printf("Step 3: Polling reg 0x0006 for bootloader ready...\n");
    VL53L8CX_WrByte(&Dev.platform, 0x7fff, 0x00);
    for (int i = 0; i < 20; i++) {
        VL53L8CX_RdMulti(&Dev.platform, 0x0006, buf, 2);
        printf("  Poll %d -> buf: [0]=0x%02X [1]=0x%02X\n", i, buf[0], buf[1]);
        if (buf[0] == 0x01 || buf[1] == 0x01) {
            printf("  Bootloader READY!\n");
            break;
        }
        VL53L8CX_WaitMs(&Dev.platform, 10);
    }

    printf("Step 4: Enable FW Access...\n");
    VL53L8CX_WrByte(&Dev.platform, 0x000E, 0x01);
    VL53L8CX_WrByte(&Dev.platform, 0x7fff, 0x02);
    VL53L8CX_WrByte(&Dev.platform, 0x7fff, 0x01);
    VL53L8CX_WrByte(&Dev.platform, 0x06, 0x01);

    printf("Step 5: Polling reg 0x21...\n");
    for (int i = 0; i < 20; i++) {
        VL53L8CX_RdMulti(&Dev.platform, 0x20, buf, 2);
        printf("  Poll %d -> buf: [0]=0x%02X [1]=0x%02X\n", i, buf[0], buf[1]);
        if (buf[0] == 0x04 || buf[1] == 0x04) {
            printf("  FW access READY (0x04)!\n");
            break;
        }
        VL53L8CX_WaitMs(&Dev.platform, 10);
    }

    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o trace_steps trace_steps.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./trace_steps
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_trace_steps.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_trace_steps.sh && /tmp/run_trace_steps.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    trace_init_line_by_line()
