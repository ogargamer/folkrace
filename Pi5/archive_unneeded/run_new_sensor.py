import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def test_new_sensor_official_st_driver():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  TESTING BRAND NEW VL53L8CX SENSOR WITH OFFICIAL ST C ULD DRIVER")
    print("======================================================================")

    script = """#!/bin/bash
set -e

cd /home/admin/official_st_repo/VL53L8CX/src

# Run C ranging CLI utility on the newly connected VL53L8CX sensor
/home/admin/st_tof_cli_official --count 10
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_new_sensor.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_new_sensor.sh && /tmp/run_new_sensor.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_new_sensor_official_st_driver()
