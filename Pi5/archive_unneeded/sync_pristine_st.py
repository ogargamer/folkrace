import os
import tarfile
import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

SRC_DIR = r"C:\Users\nurmi\Downloads\STSW-IMG042\VL53L8CX_Linux_driver_2.1.0"
TAR_PATH = r"C:\Users\nurmi\Downloads\STSW-IMG042\pristine_driver.tar.gz"

def sync_pristine_st_driver():
    print("======================================================================")
    print("  CREATING CLEAN ARCHIVE OF PRISTINE ST OFFICIAL DRIVER")
    print("======================================================================")

    with tarfile.open(TAR_PATH, "w:gz") as tar:
        tar.add(SRC_DIR, arcname="VL53L8CX_Linux_driver_2.1.0")

    print(f"Archive created: {TAR_PATH} ({os.path.getsize(TAR_PATH)} bytes)")

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("Uploading pristine driver to Pi 5...")
    sftp = ssh.open_sftp()
    sftp.put(TAR_PATH, "/tmp/pristine_driver.tar.gz")
    sftp.close()

    print("Extracting clean driver to /home/admin/STSW-IMG042/...")
    cmd = """
    mkdir -p /home/admin/STSW-IMG042
    rm -rf /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0
    tar -xzf /tmp/pristine_driver.tar.gz -C /home/admin/STSW-IMG042/
    cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test
    make clean || true
    make CFLAGS_RELEASE="-Os -DSPI -Wno-error=unused-result -Wno-error=implicit-function-declaration" PLTF_DIR=../platform
    ls -la menu multi
    """

    stdin, stdout, stderr = ssh.exec_command(cmd, get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    sync_pristine_st_driver()
