import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def build_and_run_official():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  BUILDING & RUNNING OFFICIAL STSW-IMG042 USERSPACE SPI DRIVER")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

# Hardware reset on GPIO 17
gpioset 4 17=0 2>/dev/null || true
sleep 0.05
gpioset 4 17=1 2>/dev/null || true
sleep 0.1

# Fix Makefile to not treat unused-result warning as error
sed -i 's/-Werror//g' Makefile

# Build both menu and multi
make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI" PLTF_DIR=../platform

echo ""
echo "==> Build successful! Running multi ranging test on spidev0.0..."
# Pre-pipe choice '1' (first device #0 ranging) into multi
echo "1" | ./multi
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/build_stsw.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/build_stsw.sh && /tmp/build_stsw.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    build_and_run_official()
