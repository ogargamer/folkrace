import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def test_spi1_aligned_read():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  TESTING 16-BIT ALIGNED SPI READS ON SPI1.0")
    print("======================================================================")

    c_patch = r"""static int32_t Linux_SPI_Read_16M(int fd, uint16_t index, uint8_t* read_data, uint32_t read_size, uint32_t speed_hz)
{
	int32_t status = 0;
	uint8_t is_odd = index & 1;
	uint16_t aligned_index = (index & ~1) & 0x7FFF;
	uint32_t total_len = (read_size == 1) ? 4 : (read_size + 2 + is_odd);

	uint8_t* read_buffer = malloc(total_len);
	uint8_t* write_buffer = malloc(total_len);
	memset(write_buffer, 0, total_len);

	write_buffer[0] = (uint8_t)((aligned_index >> 8) & 0x7F);
	write_buffer[1] = (uint8_t)(aligned_index & 0xFF);

	status = SPI_Xfer(fd, write_buffer, read_buffer, total_len, speed_hz);

	if (status == 0) {
		if (read_size == 1) {
			*read_data = is_odd ? read_buffer[3] : read_buffer[2];
		} else {
			memcpy(read_data, read_buffer + 2 + is_odd, read_size);
		}
	}

	free(read_buffer);
	free(write_buffer);

	return status;
}"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/new_read.c', 'w') as f:
        f.write(c_patch)
    sftp.close()

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/platform

python3 - << 'PYEOF'
with open("platform.c", "r") as f:
    text = f.read()

with open("/tmp/new_read.c", "r") as f:
    new_read = f.read()

old_read = "static int32_t Linux_SPI_Read_16M(int fd, uint16_t index, uint8_t* read_data, uint32_t read_size, uint32_t speed_hz)"
idx = text.find(old_read)
if idx >= 0:
    end_idx = text.find("return status;\n}", idx) + len("return status;\n}")
    text = text[:idx] + new_read + text[end_idx:]
    with open("platform.c", "w") as f:
        f.write(text)
    print("Patched Linux_SPI_Read_16M successfully.")
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
    with sftp.file('/tmp/patch_spi1.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/patch_spi1.sh && /tmp/patch_spi1.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_spi1_aligned_read()
