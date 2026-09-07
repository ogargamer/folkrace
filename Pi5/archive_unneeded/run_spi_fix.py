import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def test_spi_aligned_poll():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "192.168.0.13"
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)

    print("======================================================================")
    print("  APPLYING SPI WORD-ALIGNMENT FIX TO vl53l8cx_api.c AND TESTING")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/uld-driver/src

# Fix poll for answer on reg 0x06 (boot check) and reg 0x21
sed -i 's/_vl53l8cx_poll_for_answer(p_dev, 1, 0, 0x06, 0xff, 1);/_vl53l8cx_poll_for_answer(p_dev, 2, 1, 0x06, 0x01, 1);/g' vl53l8cx_api.c
sed -i 's/_vl53l8cx_poll_for_answer(p_dev, 1, 0, 0x21, 0xFF, 0x4);/_vl53l8cx_poll_for_answer(p_dev, 2, 1, 0x20, 0xFF, 0x4);/g' vl53l8cx_api.c

cd ../../test
make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI" PLTF_DIR=../platform

echo "==> Running live_ranging_demo..."
./live_ranging_demo
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_spi_fix.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_spi_fix.sh && /tmp/run_spi_fix.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_spi_aligned_poll()
