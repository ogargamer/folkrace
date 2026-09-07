import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "10.43.8.42"
PI_USER = "admin"
PI_PASS = "l"

def run_ranging_test():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  RUNNING ORIGINAL DRIVER RANGING TEST (POINTED TO FLOOR)")
    print("======================================================================")

    script = r"""#!/bin/bash
echo l | sudo -S pinctrl set 17 op dh 2>/dev/null || true
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

./run_spi1_official
"""

    stdin, stdout, stderr = ssh.exec_command(script, get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    run_ranging_test()
