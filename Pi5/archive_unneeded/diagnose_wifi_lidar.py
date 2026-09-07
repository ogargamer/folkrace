import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def inspect_pi():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "10.43.8.42"
    print(f"Connecting to Pi 5 at {ip}...")
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)
    print("Connected to Pi 5!")

    # 1. System logs for Wi-Fi / brcmfmac / SDIO / throttling / under-voltage
    print("\n================ 1. DMESG / KERNEL ERRORS ================")
    stdin, stdout, stderr = ssh.exec_command("echo 'l' | sudo -S dmesg -T | grep -iE 'voltage|throttle|brcm|wlan|wifi|mmc|ttyAMA|serial|fail|reset' | tail -n 40", get_pty=True)
    print(stdout.read().decode('utf-8', errors='ignore'))

    # 2. Check wpa_supplicant / NetworkManager logs
    print("\n================ 2. WIFI DISCONNECTION LOGS ================")
    stdin, stdout, stderr = ssh.exec_command("echo 'l' | sudo -S journalctl -u wpa_supplicant -n 30 --no-pager 2>/dev/null", get_pty=True)
    print(stdout.read().decode('utf-8', errors='ignore'))

    # 3. Check vcgencmd get_throttled
    print("\n================ 3. POWER & THROTTLING STATUS ================")
    stdin, stdout, stderr = ssh.exec_command("vcgencmd get_throttled 2>/dev/null || vcgencmd measure_temp 2>/dev/null", get_pty=True)
    print(stdout.read().decode('utf-8', errors='ignore'))

    # 4. Check serial console in cmdline.txt
    print("\n================ 4. CMDLINE & SERIAL GETTY ================")
    stdin, stdout, stderr = ssh.exec_command("cat /boot/firmware/cmdline.txt 2>/dev/null || cat /boot/cmdline.txt", get_pty=True)
    print("cmdline.txt:", stdout.read().decode('utf-8', errors='ignore'))
    stdin, stdout, stderr = ssh.exec_command("systemctl status serial-getty@ttyAMA0.service 2>/dev/null | head -n 15", get_pty=True)
    print("serial-getty status:\n", stdout.read().decode('utf-8', errors='ignore'))

    ssh.close()

if __name__ == "__main__":
    inspect_pi()
