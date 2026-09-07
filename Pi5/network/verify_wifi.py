import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def verify_both_networks():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)
    except Exception as e:
        print(f"Failed to connect via SSH: {e}")
        return

    print("======================================================================")
    print("  VERIFYING WPA_SUPPLICANT NETWORKS ON PI 5")
    print("======================================================================")

    command = "echo 'l' | sudo -S wpa_cli -i wlan0 list_networks && echo 'l' | sudo -S cat /etc/wpa_supplicant/wpa_supplicant.conf"
    stdin, stdout, stderr = ssh.exec_command(command)
    print(stdout.read().decode('utf-8', errors='ignore'))

    ssh.close()

if __name__ == "__main__":
    verify_both_networks()
