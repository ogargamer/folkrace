import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def build_and_test_official_st_cli_fast():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  COMPILING & RUNNING OFFICIAL ST C DRIVER (/home/admin/st_tof_cli_official)")
    print("======================================================================")

    script = """#!/bin/bash
set -e

cd /home/admin/official_st_repo/VL53L8CX/src

gcc -I. st_tof_cli_official.c vl53l8cx_platform.c vl53l8cx_api.c vl53l8cx_plugin_detection_thresholds.c vl53l8cx_plugin_motion_indicator.c vl53l8cx_plugin_xtalk.c -o /home/admin/st_tof_cli_official
/home/admin/st_tof_cli_official --count 3
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/fast_st_cli.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/fast_st_cli.sh && /tmp/fast_st_cli.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    build_and_test_official_st_cli_fast()
