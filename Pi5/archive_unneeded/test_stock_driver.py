import os
import tarfile
import tempfile
import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"
LOCAL_DRIVER_PATH = r"C:\Users\nurmi\Downloads\STSW-IMG042\VL53L8CX_Linux_driver_2.1.0"

def deploy_and_test_clean_st_driver():
    print("======================================================================")
    print("  DEPLOYING UNTOUCHED STSW-IMG042 DRIVER FROM DOWNLOADS TO PI 5")
    print("======================================================================")

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    print(f"Connecting to Pi 5 at {PI_IP}...")
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)
    print("Connected successfully.")

    # Archive local untouched driver directory into a tar.gz
    with tempfile.NamedTemporaryFile(suffix='.tar.gz', delete=False) as tmp_tar:
        tar_path = tmp_tar.name

    print(f"Packaging local driver from {LOCAL_DRIVER_PATH}...")
    with tarfile.open(tar_path, "w:gz") as tar:
        tar.add(LOCAL_DRIVER_PATH, arcname="VL53L8CX_Linux_driver_2.1.0")

    print("Uploading clean driver archive to Pi 5...")
    sftp = ssh.open_sftp()
    remote_tar = "/tmp/VL53L8CX_clean.tar.gz"
    sftp.put(tar_path, remote_tar)
    sftp.close()
    os.remove(tar_path)

    # Shell script on Pi to clean and compile untouched driver
    build_script = r"""#!/bin/bash
set -e

echo "==> Cleaning old directories and extracting fresh untouched ST driver..."
rm -rf /home/admin/STSW-IMG042
mkdir -p /home/admin/STSW-IMG042
tar -xzf /tmp/VL53L8CX_clean.tar.gz -C /home/admin/STSW-IMG042/

cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

echo "==> Compiling untouched ST driver (SPI mode)..."
make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI" PLTF_DIR=../platform

echo "==> Testing untouched ST multi binary (Option 1: Device #0 basic ranging)..."
echo "1" | ./multi
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_stock_st.sh', 'w') as f:
        f.write(build_script)
    sftp.close()

    print("Executing stock ST driver build and test on Pi 5...\n")
    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_stock_st.sh && /tmp/run_stock_st.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExecution finished with exit code: {exit_code}")
    ssh.close()
    return exit_code

if __name__ == "__main__":
    deploy_and_test_clean_st_driver()
