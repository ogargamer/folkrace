import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def set_wifi_permissions_and_save():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)
    except Exception as e:
        print(f"Failed to connect via SSH: {e}")
        return

    print("======================================================================")
    print("  VERIFYING WPA_SUPPLICANT CONFIG & SAVING")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e

echo "==> Current /etc/wpa_supplicant/wpa_supplicant.conf content:"
sudo cat /etc/wpa_supplicant/wpa_supplicant.conf

echo ""
echo "==> Ensuring proper header in wpa_supplicant.conf..."
cat << 'EOF' | sudo tee /etc/wpa_supplicant/wpa_supplicant.conf
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=EE

network={
	ssid="S22 Ultra"
	psk="Nurmikud"
	priority=1
}

network={
	ssid="Kullese 5"
	psk="Nurmikud"
	priority=2
}
EOF

echo ""
echo "==> Reconfiguring wpa_supplicant..."
sudo wpa_cli -i wlan0 reconfigure

echo ""
echo "==> Listing configured networks in wpa_cli:"
sudo wpa_cli -i wlan0 list_networks
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/save_wifi.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/save_wifi.sh && echo 'l' | sudo -S /tmp/save_wifi.sh", get_pty=True)
    print(stdout.read().decode('utf-8', errors='ignore'))

    ssh.close()

if __name__ == "__main__":
    set_wifi_permissions_and_save()
