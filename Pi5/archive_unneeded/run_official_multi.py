import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def build_and_run_official_multi():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  RUNNING OFFICIAL STSW-IMG042 'multi' BINARY ON PI 5")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/platform

# 1. Tune SPI speed to 1MHz in stock platform.c
sed -i 's/#define VL53L8CX_SPI_SPEED_HZ  20000000/#define VL53L8CX_SPI_SPEED_HZ  1000000/g' platform.c
sed -i 's/#define VL53L8CX_SPI_SPEED_HZ  2000000/#define VL53L8CX_SPI_SPEED_HZ  1000000/g' platform.c

# 2. Fix the pointer arithmetic in wait_for_dataready
sed -i 's/(p_platform - offsetof(VL53L8CX_Configuration, platform))/((uint8_t *)p_platform - offsetof(VL53L8CX_Configuration, platform))/g' platform.c

cd ../test
make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI -Wno-error=unused-result" PLTF_DIR=../platform

echo ""
echo "==> Running official ST multi binary for Device #0..."
echo "1" | timeout 10s ./multi || true
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_official_multi.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_official_multi.sh && /tmp/run_official_multi.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    build_and_run_official_multi()
