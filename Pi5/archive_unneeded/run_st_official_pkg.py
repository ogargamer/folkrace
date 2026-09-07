import os
import sys
import paramiko

sys.stdout.reconfigure(encoding='utf-8')

def upload_and_run_official_st_driver():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)
    sftp = ssh.open_sftp()

    local_base = r"C:\Users\nurmi\Downloads\STSW-IMG042\VL53L8CX_Linux_driver_2.1.0"
    remote_base = "/home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0"

    print("======================================================================")
    print("  UPLOADING OFFICIAL STSW-IMG042 LINUX DRIVER TO RASPBERRY PI 5")
    print("======================================================================")

    def sftp_mkdirs(remote_dir):
        dirs = remote_dir.split('/')
        cur = ''
        for d in dirs:
            if not d:
                continue
            cur += '/' + d
            try:
                sftp.mkdir(cur)
            except:
                pass

    for root, dirs, files in os.walk(local_base):
        rel_dir = os.path.relpath(root, local_base).replace('\\', '/')
        if rel_dir == '.':
            rem_dir = remote_base
        else:
            rem_dir = f"{remote_base}/{rel_dir}"
        sftp_mkdirs(rem_dir)
        for f in files:
            lpath = os.path.join(root, f)
            rpath = f"{rem_dir}/{f}"
            print(f"Uploading {rpath}...")
            sftp.put(lpath, rpath)

    sftp.close()
    print("\nUpload complete! Now building user-space SPI binary...")

    build_and_run_script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

# Clean previous build
make clean || true

# Compile with SPI enabled, without STMVL53L8CX_KERNEL (pure user-space /dev/spidev0.0)
make CFLAGS_RELEASE="-Os -g0 -DSPI" PLTF_DIR=../platform

echo "==> Build successful! Running official ST multi_ranging test for spidev0.0..."
# Pre-configure device 0 on spidev0.0 and LPn reset
./multi << 'EOF'
1
EOF
"""

    stdin, stdout, stderr = ssh.exec_command(build_and_run_script, get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    upload_and_run_official_st_driver()
