import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def run_official_ranging_session():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  RUNNING OFFICIAL STSW-IMG042 RANGING SESSION ON PI 5")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

# Make sure GPIO 17 is HIGH (LPn active)
gpioset 4 17=0 2>/dev/null || true
sleep 0.05
gpioset 4 17=1 2>/dev/null || true
sleep 0.1

echo "1" | ./multi
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_st_session.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_st_session.sh && /tmp/run_st_session.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    run_official_ranging_session()
