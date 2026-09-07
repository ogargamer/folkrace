import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def debug_init_steps_spi1():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  DEBUGGING vl53l8cx_init STEPS ON SPI1.0")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/uld-driver/src

python3 - << 'PYEOF'
with open("vl53l8cx_api.c", "r") as f:
    text = f.read()

# Add step prints in vl53l8cx_init
text = text.replace("status |= _vl53l8cx_poll_for_mcu_boot(p_dev);",
                    "status |= _vl53l8cx_poll_for_mcu_boot(p_dev); printf(\"  mcu_boot status: %d\\n\", status);")
text = text.replace("if (crc_checksum != (uint32_t)0xcadf7caf",
                    "printf(\"  crc_checksum: 0x%08X\\n\", crc_checksum);\n\tif (crc_checksum != (uint32_t)0xcadf7caf")
text = text.replace("status |= _vl53l8cx_send_offset_data(p_dev, VL53L8CX_RESOLUTION_4X4);",
                    "status |= _vl53l8cx_send_offset_data(p_dev, VL53L8CX_RESOLUTION_4X4); printf(\"  send_offset status: %d\\n\", status);")
text = text.replace("status |= _vl53l8cx_send_xtalk_data(p_dev, VL53L8CX_RESOLUTION_4X4);",
                    "status |= _vl53l8cx_send_xtalk_data(p_dev, VL53L8CX_RESOLUTION_4X4); printf(\"  send_xtalk status: %d\\n\", status);")
text = text.replace("status |= _vl53l8cx_poll_for_answer(p_dev, 4, 1,\n\t\tVL53L8CX_UI_CMD_STATUS, 0xff, 0x03);",
                    "status |= _vl53l8cx_poll_for_answer(p_dev, 4, 1, VL53L8CX_UI_CMD_STATUS, 0xff, 0x03); printf(\"  default_config status: %d\\n\", status);")

with open("vl53l8cx_api.c", "w") as f:
    f.write(text)
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
    with sftp.file('/tmp/debug_init.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/debug_init.sh && /tmp/debug_init.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    debug_init_steps_spi1()
