import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def test_st_example_pure():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  RUNNING UNMODIFIED ST EXAMPLE 1 (RANGING BASIC) VIA USERSPACE SPI")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > main_example1.c
#include <stdio.h>
#include <string.h>
#include "vl53l8cx_api.h"
#include "examples.h"

int main() {
    VL53L8CX_Configuration Dev;
    memset(&Dev, 0, sizeof(Dev));
    Dev.platform.spi_num = 0;
    Dev.platform.spi_cs = 0;

    int status = vl53l8cx_comms_init(&Dev.platform);
    if (status) {
        printf("Comms init failed\n");
        return 1;
    }

    printf("Running ST Example 1 (Basic Ranging)...\n");
    status = example1(&Dev);
    printf("Example 1 returned: %d\n", status);

    vl53l8cx_comms_close(&Dev.platform);
    return status;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o test_example1 main_example1.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c \
    ../examples/Example_1_ranging_basic.c

./test_example1
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_example1.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_example1.sh && /tmp/run_example1.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_st_example_pure()
