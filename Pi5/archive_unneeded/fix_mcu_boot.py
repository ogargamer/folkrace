import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def fix_mcu_boot_poll():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  FIXING _vl53l8cx_poll_for_mcu_boot ON SPI1.0")
    print("======================================================================")

    c_replacement = r"""static uint8_t _vl53l8cx_poll_for_mcu_boot(
              VL53L8CX_Configuration      *p_dev)
{
   uint8_t buf[4] = {0};
   uint16_t timeout = 0;

   do {
		VL53L8CX_RdMulti(&(p_dev->platform), 0x06, buf, 2);
		if((buf[1] & 0x01) || (buf[0] & 0x80))
		{
			return VL53L8CX_STATUS_OK;
		}
		(void)VL53L8CX_WaitMs(&(p_dev->platform), 1);
		timeout++;
	} while (timeout < (uint16_t)500);

   return (buf[1] & 0x01) ? VL53L8CX_STATUS_OK : VL53L8CX_STATUS_TIMEOUT_ERROR;
}"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/new_boot.c', 'w') as f:
        f.write(c_replacement)
    sftp.close()

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/uld-driver/src

python3 - << 'PYEOF'
with open("vl53l8cx_api.c", "r") as f:
    text = f.read()

with open("/tmp/new_boot.c", "r") as f:
    new_boot = f.read()

header = "static uint8_t _vl53l8cx_poll_for_mcu_boot("
idx = text.find(header)
if idx >= 0:
    end_idx = text.find("return status;\n}", idx) + len("return status;\n}")
    text = text[:idx] + new_boot + text[end_idx:]
    with open("vl53l8cx_api.c", "w") as f:
        f.write(text)
    print("Replaced _vl53l8cx_poll_for_mcu_boot successfully.")
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
    with sftp.file('/tmp/fix_boot.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/fix_boot.sh && /tmp/fix_boot.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    fix_mcu_boot_poll()
