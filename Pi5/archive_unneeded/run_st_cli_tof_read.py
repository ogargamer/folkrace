import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def test_official_st_cli_tof_read():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  RUNNING SYSTEM UTILITY: tof read --count 5")
    print("======================================================================")

    stdin, stdout, stderr = ssh.exec_command("tof read --count 5", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_official_st_cli_tof_read()
