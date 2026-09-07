import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def test_aligned_chunk_size():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  TESTING 1024-BYTE (32-BIT ALIGNED) CHUNK SIZE & CRC MATCH")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/platform

# Set chunk size to 1024 (perfect 32-bit word multiple)
sed -i 's/#define VL53L8_SPIDEV_CHUNK_SIZE.*/#define VL53L8_SPIDEV_CHUNK_SIZE 1024/g' platform.c

cd ../test
make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI -Wno-error=unused-result" PLTF_DIR=../platform

./test_crc
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_aligned_chunk.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_aligned_chunk.sh && /tmp/run_aligned_chunk.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_aligned_chunk_size()
