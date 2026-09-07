import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def build_sparkfun_preexisting_driver():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  FETCHING & BUILDING PRE-EXISTING SPARKFUN / ST ULD C DRIVER FOR PI 5")
    print("======================================================================")

    script = """#!/bin/bash
set -e

mkdir -p /home/admin/preexisting_st_driver
cd /home/admin/preexisting_st_driver

# Clone official pre-existing Linux ULD C library repository from SparkFun / ST
if [ ! -d "SparkFun_VL53L5CX_Linux_Library" ]; then
    git clone https://github.com/sparkfun/SparkFun_VL53L5CX_Linux_Library.git || true
fi

echo "=== Checking Pre-existing ST Library Files ==="
ls -la /home/admin/preexisting_st_driver
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/fetch_preexisting.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/fetch_preexisting.sh && /tmp/fetch_preexisting.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nFetch Exit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    build_sparkfun_preexisting_driver()
