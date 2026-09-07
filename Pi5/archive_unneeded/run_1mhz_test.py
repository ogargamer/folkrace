import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def test_stsw_at_1mhz():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  CONFIGURING STSW-IMG042 FOR 1 MHz SPI CLOCK & TESTING RANGING")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/platform

# Update SPI clock to 1MHz in platform.c
sed -i 's/#define VL53L8CX_SPI_SPEED_HZ.*/#define VL53L8CX_SPI_SPEED_HZ 1000000/' platform.c

cd ../test
make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI" PLTF_DIR=../platform

echo "==> Running STSW-IMG042 multi at 1 MHz..."
echo "1" | ./multi
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_1mhz.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_1mhz.sh && /tmp/run_1mhz.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_stsw_at_1mhz()
