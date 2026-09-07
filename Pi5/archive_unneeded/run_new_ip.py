import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def test_connection_new_ip():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "192.168.0.13"
    print(f"Connecting to Pi 5 at {ip}...")
    try:
        ssh.connect(ip, port=22, username="admin", password="l", timeout=5)
        print("SSH Connection successful!")
    except Exception as e:
        print(f"SSH Failed: {e}")
        return

    print("======================================================================")
    print("  TESTING TOF HARDWARE COMMUNICATION ON NEW IP (192.168.0.13)")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e

echo "==> Testing SPI device node..."
ls -l /dev/spidev* || true

echo ""
echo "==> Testing STSW-IMG042 userspace SPI ranging..."
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

# Build if needed
make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI" PLTF_DIR=../platform

echo "1" | ./multi
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/test_new_ip.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/test_new_ip.sh && /tmp/test_new_ip.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_connection_new_ip()
