import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def build_and_run_stock_st():
    print("======================================================================")
    print("  RUNNING UNTOUCHED STSW-IMG042 DRIVER ON PI 5")
    print("======================================================================")

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    build_script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

echo "==> Building stock ST multi and menu (with -Wno-error=unused-result)..."
make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI -Wno-error=unused-result" PLTF_DIR=../platform

echo "==> Executing stock ST multi binary..."
echo "1" | ./multi
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_stock_clean.sh', 'w') as f:
        f.write(build_script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_stock_clean.sh && /tmp/run_stock_clean.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExecution finished with exit code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    build_and_run_stock_st()
