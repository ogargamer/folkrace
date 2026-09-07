import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def add_wifi_wpa_supplicant():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)
    except Exception as e:
        print(f"Failed to connect via SSH: {e}")
        return

    print("======================================================================")
    print("  ADDING WIFI NETWORK via wpa_supplicant / ncli / wpa_cli")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e

SSID="Kullese 5"
PASS="Nurmikud"

echo "==> Testing available network tool..."

if command -v ncli &> /dev/null; then
    echo "Found ncli..."
    ncli dev wifi connect "$SSID" password "$PASS"
elif command -v wpa_passphrase &> /dev/null; then
    echo "Appending network to /etc/wpa_supplicant/wpa_supplicant.conf..."
    wpa_passphrase "$SSID" "$PASS" | sudo tee -a /etc/wpa_supplicant/wpa_supplicant.conf
    echo "Reconfiguring wpa_supplicant..."
    sudo wpa_cli -i wlan0 reconfigure || true
else
    echo "Neither nmcli nor wpa_passphrase found. Checking active network managers..."
    ip a
fi
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/add_wifi.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/add_wifi.sh && echo 'l' | sudo -S /tmp/add_wifi.sh", get_pty=True)
    out = stdout.read().decode('utf-8', errors='ignore')
    print(out)

    ssh.close()

if __name__ == "__main__":
    add_wifi_wpa_supplicant()
