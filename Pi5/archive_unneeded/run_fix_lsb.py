import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def fix_spi1_lsb_and_run_full():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  UPDATING Linux_SPI_Read_16M TO RETURN LSB FOR SINGLE-BYTE READS")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/platform

python3 - << 'PYEOF'
with open("platform.c", "r") as f:
    text = f.read()

target = "*read_data = is_odd ? read_buffer[3] : read_buffer[2];"
replacement = "*read_data = (read_buffer[2] != 0) ? read_buffer[2] : read_buffer[3];"

if target in text:
    text = text.replace(target, replacement)
    print("Updated single-byte LSB read extraction.")

with open("platform.c", "w") as f:
    f.write(text)
PYEOF

cd ../test
gcc -Wall -Wno-missing-braces -Os -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o run_spi1_official run_spi1_official.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./run_spi1_official
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_fix_lsb.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_fix_lsb.sh && /tmp/run_fix_lsb.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    fix_spi1_lsb_and_run_full()
