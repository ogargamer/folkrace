import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def check_wifi_status():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)
    except Exception as e:
        print(f"Failed to connect via SSH: {e}")
        return

    print("======================================================================")
    print("  CHECKING WI-FI CONNECTION STATUS")
    print("======================================================================")

    command = "iwconfig wlan0 2>/dev/null || wpa_cli status 2>/dev/null || ip a show wlan0"
    stdin, stdout, stderr = ssh.exec_command(command)
    print(stdout.read().decode('utf-8', errors='ignore'))

    ssh.close()

if __name__ == "__main__":
    check_wifi_status()
