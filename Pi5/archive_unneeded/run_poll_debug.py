import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def print_poll_details():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  PRINTING EXACT BYTES RETURNED IN _vl53l8cx_poll_for_answer")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/uld-driver/src

# Print poll details inside _vl53l8cx_poll_for_answer using sed
sed -i 's/status |= (uint8_t)VL53L8CX_STATUS_TIMEOUT_ERROR;/printf("  TIMEOUT addr=0x%04X, exp=0x%02X, got=[0x%02X 0x%02X 0x%02X 0x%02X]\\n", address, expected_value, p_dev->temp_buffer[0], p_dev->temp_buffer[1], p_dev->temp_buffer[2], p_dev->temp_buffer[3]); status |= (uint8_t)VL53L8CX_STATUS_TIMEOUT_ERROR;/g' vl53l8cx_api.c

cd ../../test
make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI -Wno-error=unused-result -Wno-error=implicit-function-declaration" PLTF_DIR=../platform

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o live_ranging_clean live_ranging_clean.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./live_ranging_clean
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_poll_print.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_poll_print.sh && /tmp/run_poll_print.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    print_poll_details()
