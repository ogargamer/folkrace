import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def add_wifi_wpa_cli():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)
    except Exception as e:
        print(f"Failed to connect via SSH: {e}")
        return

    print("======================================================================")
    print("  ADDING NETWORK VIA WPA_CLI INTERACTIVELY")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e

echo "==> Adding new network via wpa_cli..."
NET_ID=$(sudo wpa_cli -i wlan0 add_network | tail -n 1)
echo "Created network ID: $NET_ID"

sudo wpa_cli -i wlan0 set_network $NET_ID ssid '"Kullese 5"'
sudo wpa_cli -i wlan0 set_network $NET_ID psk '"Nurmikud"'
sudo wpa_cli -i wlan0 enable_network $NET_ID
sudo wpa_cli -i wlan0 save_config

echo ""
echo "==> Current configured networks:"
sudo wpa_cli -i wlan0 list_networks
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/add_wifi_cli.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/add_wifi_cli.sh && echo 'l' | sudo -S /tmp/add_wifi_cli.sh", get_pty=True)
    print(stdout.read().decode('utf-8', errors='ignore'))

    ssh.close()

if __name__ == "__main__":
    add_wifi_wpa_cli()
