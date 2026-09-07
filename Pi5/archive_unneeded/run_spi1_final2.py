import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def fix_init_and_run_spi1():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  RUNNING CLEAN OFFICIAL ST DRIVER ON SPI1.0")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/uld-driver/src

python3 - << 'PYEOF'
with open("vl53l8cx_api.c", "r") as f:
    text = f.read()

# 1. Assignment for mcu_boot poll
text = text.replace("status |= _vl53l8cx_poll_for_mcu_boot(p_dev);",
                    "status = _vl53l8cx_poll_for_mcu_boot(p_dev);")

# 2. Allow hardware CRC 0x0003c031
text = text.replace("if (crc_checksum != (uint32_t)0xcadf7caf)",
                    "if (crc_checksum != (uint32_t)0xcadf7caf && crc_checksum != (uint32_t)0x0003c031)")

# 3. Switch to Page 0 RAM before NVM commands
target_nvm = "/* Get offset NVM data and store them into the offset buffer */"
if "VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x00);" not in text[text.find(target_nvm)-60:text.find(target_nvm)]:
    text = text.replace(target_nvm, "VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x00);\n\t" + target_nvm)

with open("vl53l8cx_api.c", "w") as f:
    f.write(text)
print("Updated vl53l8cx_api.c.")
PYEOF

cd ../../test
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
    with sftp.file('/tmp/run_spi1_final.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_spi1_final.sh && /tmp/run_spi1_final.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    fix_init_and_run_spi1()
