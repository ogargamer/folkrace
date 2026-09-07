import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def build_full_stsw_img040_driver():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  [GOAL] COMPILING FULL ST STSW-IMG040 C DRIVER + RAM FIRMWARE UPLOAD")
    print("======================================================================")

    setup_script = """#!/bin/bash
set -e

mkdir -p /home/admin/st_driver_build
cd /home/admin/st_driver_build

# Clone official ST driver repo if missing
if [ ! -d "VL53L8CX_ULD_Linux" ]; then
    git clone https://github.com/stm32-hotspot/VL53L8CX_ULD_Linux.git
fi

echo "=== ST Driver Source Available ==="
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/build_st_full.sh', 'w') as f:
        f.write(setup_script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/build_st_full.sh && /tmp/build_st_full.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nFetch Exit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    build_full_stsw_img040_driver()
