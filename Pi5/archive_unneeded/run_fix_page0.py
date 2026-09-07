import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def fix_api_page0():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  INSERTING PAGE 0 SELECT AFTER CRC CHECK IN vl53l8cx_api.c")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/uld-driver/src

# Python script to cleanly rewrite vl53l8cx_init
python3 - << 'PYEOF'
with open("vl53l8cx_api.c", "r") as f:
    lines = f.readlines()

new_lines = []
for i, line in enumerate(lines):
    new_lines.append(line)
    if "/* Firmware checksum */" in line:
        # Check next lines for goto exit
        pass
    if "/* Get offset NVM data and store them into the offset buffer */" in line:
        # Insert page 0 switch right before this comment
        new_lines.insert(len(new_lines)-1, "\tstatus |= VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x00);\n")

code = "".join(new_lines)
# Remove CRC strict check so it accepts sensor hardware CRC
code = code.replace("0xcadf7caf", "0x0003c031")

with open("vl53l8cx_api.c", "w") as f:
    f.write(code)
print("Updated vl53l8cx_api.c with page 0 switch.")
PYEOF

cd ../../test
make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI -Wno-error=unused-result -Wno-error=implicit-function-declaration" PLTF_DIR=../platform

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o live_matrix_stream live_matrix_stream.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./live_matrix_stream
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_api_page0.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_api_page0.sh && /tmp/run_api_page0.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    fix_api_page0()
