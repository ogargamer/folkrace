import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def list_spi_overlays():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    cmd = "ls /boot/firmware/overlays/spi*.dtbo"
    stdin, stdout, stderr = ssh.exec_command(cmd)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    ssh.close()

if __name__ == "__main__":
    list_spi_overlays()
