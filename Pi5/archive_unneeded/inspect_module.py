import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def test_pip_st_driver_ranging_v2():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    py_app = """#!/usr/bin/env python3
import time
import os
import sys
import vl53l5cx

print("[1] Inspecting imported vl53l5cx python module attributes:")
print(dir(vl53l5cx))
"""

    sftp = ssh.open_sftp()
    with sftp.file('/home/admin/inspect_module.py', 'w') as f:
        f.write(py_app)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("python3 /home/admin/inspect_module.py", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_pip_st_driver_ranging_v2()
