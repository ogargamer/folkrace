import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def sample_spi1_live_viewer():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  SAMPLING LIVE VIEWER ON SPI1.0")
    print("======================================================================")

    stdin, stdout, stderr = ssh.exec_command(
        "cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test && timeout 4 ./live_viewer || true",
        get_pty=True
    )

    for line in iter(stdout.readline, ""):
        print(line, end="")

    ssh.close()

if __name__ == "__main__":
    sample_spi1_live_viewer()
