import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def fix_start_ranging_page0():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  FIXING start_ranging TO WRITE COMMAND ON PAGE 0 & TESTING STREAM")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/uld-driver/src

python3 - << 'PYEOF'
with open("vl53l8cx_api.c", "r") as f:
    lines = f.readlines()

out = []
for line in lines:
    if "VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x02);" in line and "Start ranging session" in "".join(lines[lines.index(line)-3:lines.index(line)]):
        out.append("\t/* Keep Page 0 for UI_CMD_END */\n")
    else:
        out.append(line)

with open("vl53l8cx_api.c", "w") as f:
    f.writelines(out)
print("Updated vl53l8cx_api.c")
PYEOF

cd ../../test
make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI -Wno-error=unused-result -Wno-error=implicit-function-declaration" PLTF_DIR=../platform

cat << 'EOF' > test_stream_active.c
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

    printf("\nPolling Stream Count over 10 frames:\n");
    for (int frame = 1; frame <= 10; frame++) {
        VL53L8CX_WaitMs(&Dev.platform, 70); // 70ms = ~15 Hz
        uint8_t sram[0x400] = {0};
        VL53L8CX_RdMulti(&Dev.platform, 0x0000, sram, 0x400);

        uint8_t streamcount = sram[0x0020];
        printf("Frame #%02d -> Streamcount: %3u (Hex: 0x%02X)\n", frame, streamcount, streamcount);
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o test_stream_active test_stream_active.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./test_stream_active
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/fix_page0.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/fix_page0.sh && /tmp/fix_page0.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    fix_start_ranging_page0()
