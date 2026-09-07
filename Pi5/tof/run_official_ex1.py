import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def run_official_example_1():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  RUNNING OFFICIAL ST EXAMPLE 1 (ALL 16 ZONES PRINTED)")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > run_example1.c
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "vl53l8cx_api.h"

extern int example1(VL53L8CX_Configuration *p_dev);

int main() {
    VL53L8CX_Configuration Dev;
    memset(&Dev, 0, sizeof(Dev));
    Dev.platform.spi_num = 0;
    Dev.platform.spi_cs = 0;

    vl53l8cx_comms_init(&Dev.platform);

    printf("Executing Official ST Example 1...\n");
    int ret = example1(&Dev);

    vl53l8cx_comms_close(&Dev.platform);
    return ret;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o run_example1 run_example1.c \
    ../examples/Example_1_ranging_basic.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./run_example1
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_ex1.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_ex1.sh && /tmp/run_ex1.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    run_official_example_1()
