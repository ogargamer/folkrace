import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def test_st_c_driver_on_new_sensor():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  RUNNING OFFICIAL ST C ULD RANGING ENGINE ON BRAND NEW SENSOR")
    print("======================================================================")

    stdin, stdout, stderr = ssh.exec_command("/home/admin/official_st_repo/VL53L8CX/src/st_official_ranging_clean", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_st_c_driver_on_new_sensor()
