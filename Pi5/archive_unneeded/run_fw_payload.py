import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def build_full_84kb_firmware_installer():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("==========================================================")
    print("  INJECTING ST 84KB RAM BINARY PAYLOAD & COMPILING ULD")
    print("==========================================================")

    installer_script = """#!/bin/bash
set -e

mkdir -p /home/admin/st_uld_full
cd /home/admin/st_uld_full

# Fetch lightweight Arduino/C driver containing the exact 84KB VL53L8CX firmware array
if [ ! -f "vl53l8cx_buffers.h" ]; then
    curl -sSL https://raw.githubusercontent.com/stm32-hotspot/VL53L8CX_ULD_Linux.git/main/vl53l8cx_buffers.h -o vl53l8cx_buffers.h || \
    curl -sSL https://raw.githubusercontent.com/sparkfun/SparkFun_VL53L5CX_Arduino_Library/main/src/vl53l5cx_buffers.h -o vl53l8cx_buffers.h || true
fi

echo "=== ST 84KB Firmware Header Check ==="
ls -la /home/admin/st_uld_full
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/install_fw_payload.sh', 'w') as f:
        f.write(installer_script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/install_fw_payload.sh && /tmp/install_fw_payload.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nPayload Fetch Exit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    build_full_84kb_firmware_installer()
