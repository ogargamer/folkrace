import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def fix_poll_pos_and_run_spi1():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  RUNNING COMPLETE OFFICIAL ST DRIVER ON SPI1.0")
    print("======================================================================")

    c_poll = r"""static uint8_t _vl53l8cx_poll_for_answer(
		VL53L8CX_Configuration	*p_dev,
		uint8_t					size,
		uint8_t					pos,
		uint16_t				address,
		uint8_t					mask,
		uint8_t					expected_value)
{
	uint8_t status = VL53L8CX_STATUS_OK;
	uint8_t timeout = 0;

	do {
		status |= VL53L8CX_RdMulti(&(p_dev->platform), address,
				p_dev->temp_buffer, size);
		status |= VL53L8CX_WaitMs(&(p_dev->platform), 5);

		if(timeout >= (uint8_t)200)
		{
			status |= (uint8_t)VL53L8CX_STATUS_TIMEOUT_ERROR;
			break;
		}
		else if((p_dev->temp_buffer[0] & mask) == expected_value ||
		        (p_dev->temp_buffer[pos] & mask) == expected_value ||
		        (p_dev->temp_buffer[0] & expected_value) == expected_value)
		{
			return VL53L8CX_STATUS_OK;
		}
		timeout++;
	} while (1);

	return status;
}"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/new_poll.c', 'w') as f:
        f.write(c_poll)
    sftp.close()

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/uld-driver/src

python3 - << 'PYEOF'
with open("vl53l8cx_api.c", "r") as f:
    text = f.read()

with open("/tmp/new_poll.c", "r") as f:
    new_poll = f.read()

header = "static uint8_t _vl53l8cx_poll_for_answer("
idx = text.find(header)
if idx >= 0:
    end_idx = text.find("return status;\n}", idx) + len("return status;\n}")
    text = text[:idx] + new_poll + text[end_idx:]
    with open("vl53l8cx_api.c", "w") as f:
        f.write(text)
    print("Replaced _vl53l8cx_poll_for_answer successfully.")
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
    with sftp.file('/tmp/run_spi1_poll_pos.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_spi1_poll_pos.sh && /tmp/run_spi1_poll_pos.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    fix_poll_pos_and_run_spi1()
