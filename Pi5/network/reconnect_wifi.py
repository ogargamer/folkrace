import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def trigger_wifi_reconnect():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)
    except Exception as e:
        print(f"Failed to connect via SSH: {e}")
        return

    print("======================================================================")
    print("  TRIGGERING WPA_SUPPLICANT RECONNECT TO KULLESE 5")
    print("======================================================================")

    script = r"""#!/bin/bash
echo "==> Listing configured wpa_supplicant networks..."
sudo wpa_cli -i wlan0 list_networks

echo ""
echo "==> Reassociating wpa_supplicant..."
sudo wpa_cli -i wlan0 reassociate
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/reconnect_wifi.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/reconnect_wifi.sh && echo 'l' | sudo -S /tmp/reconnect_wifi.sh", get_pty=True)
    print(stdout.read().decode('utf-8', errors='ignore'))

    ssh.close()

if __name__ == "__main__":
    trigger_wifi_reconnect()
