import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def set_spidev_bufsiz_cmdline_and_reboot():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  SETTING spidev.bufsiz=100000 IN /boot/firmware/cmdline.txt & REBOOTING")
    print("======================================================================")

    script = """#!/bin/bash
set -e

echo "==> Current cmdline.txt:"
cat /boot/firmware/cmdline.txt

echo ""
echo "==> Checking if spidev.bufsiz already set..."
if grep -q "spidev.bufsiz" /boot/firmware/cmdline.txt; then
    echo "Already set. Updating to 100000..."
    sudo sed -i 's/spidev.bufsiz=[0-9]*/spidev.bufsiz=100000/' /boot/firmware/cmdline.txt
else
    echo "Not set. Appending spidev.bufsiz=100000..."
    sudo sed -i 's/$/ spidev.bufsiz=100000/' /boot/firmware/cmdline.txt
fi

echo ""
echo "==> New cmdline.txt:"
cat /boot/firmware/cmdline.txt

echo ""
echo "==> Rebooting Pi 5 now... (reconnect in ~30 seconds)"
sudo reboot
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/set_bufsiz.sh', 'w') as f:
        f.write(script)
    sftp.close()

    ssh.exec_command("chmod +x /tmp/set_bufsiz.sh && echo 'l' | sudo -S /tmp/set_bufsiz.sh", get_pty=True)
    time.sleep(3)
    print("Pi 5 is rebooting... waiting 35 seconds for it to come back online.")
    ssh.close()

    time.sleep(35)

    print("\n==> Reconnecting to Pi 5 after reboot...")
    ssh2 = paramiko.SSHClient()
    ssh2.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    for attempt in range(6):
        try:
            ssh2.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)
            print("Connected!")
            break
        except Exception as e:
            print(f"Attempt {attempt+1}/6: {e} - retrying in 10s...")
            time.sleep(10)

    print("\n==> Verifying new bufsiz after reboot:")
    stdin, stdout, stderr = ssh2.exec_command("cat /sys/module/spidev/parameters/bufsiz")
    print("bufsiz =", stdout.read().decode().strip())

    print("\n==> Running official ST C driver with expanded SPI buffer...")
    stdin2, stdout2, stderr2 = ssh2.exec_command(
        "cd /home/admin/official_st_repo/VL53L8CX/src && ./st_official_ranging_bufsiz", get_pty=True
    )
    for line in iter(stdout2.readline, ""):
        print(line, end="")

    exit_code = stdout2.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh2.close()

if __name__ == "__main__":
    set_spidev_bufsiz_cmdline_and_reboot()
