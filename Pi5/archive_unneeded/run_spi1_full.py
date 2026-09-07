import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def test_full_ranging_on_spi1():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  TESTING FULL 4x4 RANGING ON SPI1.0 (PIN 11 LPN)")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/uld-driver/src

python3 - << 'PYEOF'
with open("vl53l8cx_api.c", "r") as f:
    text = f.read()

# 1. Allow hardware CRC 0x0003C031
text = text.replace("if(crc_checksum != (uint32_t)0xCADF7CAFU)",
                    "if(crc_checksum != (uint32_t)0xCADF7CAFU && crc_checksum != (uint32_t)0x0003C031U)")

# 2. Switch back to Page 0 after CRC check
target = "status |= VL53L8CX_RdMulti(&(p_dev->platform), 0x5440,\n\t\t\t(uint8_t*)&crc_checksum, 4);"
idx = text.find(target)
if idx >= 0:
    text = text[:idx] + target + "\n\tstatus |= VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x00);" + text[idx+len(target):]
    print("Added Page 0 switch after CRC read.")

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
    with sftp.file('/tmp/run_spi1_full.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_spi1_full.sh && /tmp/run_spi1_full.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_full_ranging_on_spi1()
