import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def test_spi_aligned_is_alive():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "192.168.0.13"
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)

    print("======================================================================")
    print("  TESTING 16-BIT ALIGNED is_alive & FIRMWARE INITIALIZATION")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/uld-driver/src

# Replace vl53l8cx_is_alive in vl53l8cx_api.c with 2-byte read at 0x0000
sed -i 's/VL53L8CX_RdByte(&(p_dev->platform), 0, &device_id);/VL53L8CX_RdMulti(&(p_dev->platform), 0x0000, \&device_id, 2); revision_id = *(\&device_id + 1);/g' vl53l8cx_api.c
sed -i 's/VL53L8CX_RdByte(&(p_dev->platform), 1, &revision_id);/\/\/ byte 1 read/g' vl53l8cx_api.c

cd ../../test
make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI" PLTF_DIR=../platform

./test_c_zero
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_patch.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_patch.sh && /tmp/run_patch.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_spi_aligned_is_alive()
