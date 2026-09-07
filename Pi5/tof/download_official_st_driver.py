import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def download_and_install_official_st_driver():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  DOWNLOADING & COMPILING OFFICIAL STMICROELECTRONICS DRIVER (STSW-IMG042)")
    print("======================================================================")

    script = """#!/bin/bash
set -e

mkdir -p /home/admin/official_st_driver
cd /home/admin/official_st_driver

# Download official ST driver repository release zip
curl -sSL https://github.com/stm32-hotspot/VL53L8CX_ULD_Linux/archive/refs/heads/main.zip -o st_driver.zip
unzip -o st_driver.zip

cd VL53L8CX_ULD_Linux-main

echo "=== Official ST Driver Files Downloaded ==="
ls -la VL53L8CX_ULD_API/src
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/download_st_official.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/download_st_official.sh && /tmp/download_st_official.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nDownload Exit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    download_and_install_official_st_driver()
