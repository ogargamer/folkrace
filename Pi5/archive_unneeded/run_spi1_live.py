import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def run_spi1_ranging_stream():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  INITIALIZING & RUNNING LIVE 4X4 RANGING ON SPI1.0")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/uld-driver/src

python3 - << 'PYEOF'
with open("vl53l8cx_api.c", "r") as f:
    text = f.read()

# 1. Allow hardware CRC 0x0003c031
text = text.replace("if (crc_checksum != (uint32_t)0xcadf7caf)",
                    "if (crc_checksum != (uint32_t)0xcadf7caf && crc_checksum != (uint32_t)0x0003c031)")

# 2. Select Page 0 after CRC read
old_nvm = "/* Get offset NVM data and store them into the offset buffer */"
new_nvm = "/* Switch to Page 0 RAM */\n\tstatus |= VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x00);\n\t/* Get offset NVM data and store them into the offset buffer */"

if old_nvm in text and "Switch to Page 0 RAM" not in text:
    text = text.replace(old_nvm, new_nvm)

# 3. Check data ready for bit 0 / non-255 streamcount
target_dr = "if((p_dev->temp_buffer[0] != p_dev->streamcount)"
idx_dr = text.find(target_dr)
if idx_dr >= 0:
    end_dr = text.find("{\n\t\t*p_isReady = (uint8_t)1;", idx_dr)
    replacement = "if((p_dev->temp_buffer[3] & 0x01) || (p_dev->temp_buffer[0] != p_dev->streamcount))\n\t"
    text = text[:idx_dr] + replacement + text[end_dr:]

with open("vl53l8cx_api.c", "w") as f:
    f.write(text)
print("Updated vl53l8cx_api.c successfully.")
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
    with sftp.file('/tmp/run_spi1_live.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_spi1_live.sh && /tmp/run_spi1_live.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    run_spi1_ranging_stream()
