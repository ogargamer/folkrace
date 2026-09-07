import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def run_imu_test():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "10.43.8.42"
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)

    sftp = ssh.open_sftp()
    local_script = r"c:\Users\nurmi\Documents\Da Winna\Pi5\test_bno08x_spi.py"
    sftp.put(local_script, "/home/admin/test_bno08x_spi.py")
    sftp.close()

    print("Running BNO08X IMU SPI test on Raspberry Pi 5...")
    stdin, stdout, stderr = ssh.exec_command("python3 /home/admin/test_bno08x_spi.py", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    ssh.close()

if __name__ == "__main__":
    run_imu_test()
