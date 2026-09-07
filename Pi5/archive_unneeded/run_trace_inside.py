import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def trace_init_line_by_line():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  TRACING EXACT LINE INSIDE vl53l8cx_init")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > trace_inside_init.c
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
    printf("is_alive: %d\n", isAlive);

    int status = vl53l8cx_init(&Dev);
    printf("vl53l8cx_init returned: %d\n", status);

    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

# Add printf to each section of vl53l8cx_init
python3 - << 'PYEOF'
with open("../uld-driver/src/vl53l8cx_api.c", "r") as f:
    text = f.read()

# Add printf before every return / goto / status assignment
text = text.replace("if(status != (uint8_t)0){\n\t\tgoto exit;\n\t}",
                    "if(status != (uint8_t)0){\n\t\tprintf(\"  FAIL AT LINE %d (status=%d)\\n\", __LINE__, status);\n\t\tgoto exit;\n\t}")
text = text.replace("status |= (uint8_t)VL53L8CX_STATUS_TIMEOUT_ERROR;",
                    "printf(\"  POLL TIMEOUT AT LINE %d!\\n\", __LINE__); status |= (uint8_t)VL53L8CX_STATUS_TIMEOUT_ERROR;")
text = text.replace("status |= VL53L8CX_MCU_ERROR;",
                    "printf(\"  POLL MCU ERROR AT LINE %d!\\n\", __LINE__); status |= VL53L8CX_MCU_ERROR;")
text = text.replace("status |= VL53L8CX_STATUS_FW_CHECKSUM_FAIL;",
                    "printf(\"  FW CHECKSUM FAIL AT LINE %d!\\n\", __LINE__); status |= VL53L8CX_STATUS_FW_CHECKSUM_FAIL;")

with open("../uld-driver/src/vl53l8cx_api.c", "w") as f:
    f.write(text)
print("Instrumented vl53l8cx_api.c with line numbers.")
PYEOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o trace_inside_init trace_inside_init.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./trace_inside_init
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_trace_inside.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_trace_inside.sh && /tmp/run_trace_inside.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    trace_init_line_by_line()
