import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def build_pip_vl53l8cx_installer():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("Installing official VL53L5CX / VL53L8CX Python library via pip with --break-system-packages...")

    script = """#!/bin/bash
set -e

echo l | sudo -S pip install --break-system-packages vl53l5cx || echo l | sudo -S pip install --break-system-packages vl53l5cx-python || true

echo "=== Pip Package Installation Check ==="
python3 -c "import vl53l5cx; print('VL53L5CX Module Successfully Imported!')" || true
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/install_pip_driver.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/install_pip_driver.sh && /tmp/install_pip_driver.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nPip Driver Install Exit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    build_pip_vl53l8cx_installer()
