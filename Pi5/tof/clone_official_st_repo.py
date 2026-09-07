import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def clone_official_stm32duino_vl53l8cx():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  CLONING OFFICIAL STMICROELECTRONICS REPOSITORY: stm32duino/VL53L8CX")
    print("======================================================================")

    script = """#!/bin/bash
set -e

mkdir -p /home/admin/official_st_repo
cd /home/admin/official_st_repo

if [ ! -d "VL53L8CX" ]; then
    git clone https://github.com/stm32duino/VL53L8CX.git
fi

cd VL53L8CX/src
echo "=== ST Official VL53L8CX Library Files Available ==="
ls -la
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/clone_official_st.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/clone_official_st.sh && /tmp/clone_official_st.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nClone Exit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    clone_official_stm32duino_vl53l8cx()
