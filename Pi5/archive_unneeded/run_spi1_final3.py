import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def fix_page0_after_crc_and_run():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  RUNNING LIVE 4X4 RANGING ON SPI1.0")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/uld-driver/src

python3 - << 'PYEOF'
with open("vl53l8cx_api.c", "r") as f:
    text = f.read()

# Make sure Page 0 is set right after CRC check
target = "if (crc_checksum != (uint32_t)0xcadf7caf && crc_checksum != (uint32_t)0x0003c031)\n\t{\n\t\tstatus |= VL53L8CX_STATUS_FW_CHECKSUM_FAIL;\n\t\tgoto exit;\n\t}"
replacement = target + "\n\n\t/* Switch to Page 0 RAM */\n\tstatus |= VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x00);"

if target in text and "Switch to Page 0 RAM" not in text:
    text = text.replace(target, replacement)
    print("Added Page 0 switch immediately after CRC.")

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
    with sftp.file('/tmp/run_spi1_final3.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_spi1_final3.sh && /tmp/run_spi1_final3.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    fix_page0_after_crc_and_run()
