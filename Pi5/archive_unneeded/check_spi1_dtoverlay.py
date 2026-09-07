import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def check_spi1_overlay():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    cmd = "dtoverlay -h spi1-1cs || true; sudo dtoverlay spi1-1cs; ls -la /dev/spidev*"
    stdin, stdout, stderr = ssh.exec_command(cmd)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    ssh.close()

if __name__ == "__main__":
    check_spi1_overlay()
