import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def setup_and_test():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "10.43.8.42"
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)

    print("--- 1. Disabling serial console service on ttyAMA0 ---")
    setup_cmd = """
    echo 'l' | sudo -S systemctl stop serial-getty@ttyAMA0.service 2>/dev/null || true
    echo 'l' | sudo -S systemctl disable serial-getty@ttyAMA0.service 2>/dev/null || true
    echo 'l' | sudo -S systemctl stop serial-getty@serial0.service 2>/dev/null || true
    echo 'l' | sudo -S systemctl disable serial-getty@serial0.service 2>/dev/null || true
    
    # Fix cmdline.txt to remove serial console if present
    if grep -q "console=serial0" /boot/firmware/cmdline.txt; then
        echo "Removing console=serial0 from /boot/firmware/cmdline.txt..."
        echo 'l' | sudo -S sed -i 's/console=serial0,[0-9]* //g' /boot/firmware/cmdline.txt
    fi
    if grep -q "console=ttyAMA0" /boot/firmware/cmdline.txt; then
        echo "Removing console=ttyAMA0 from /boot/firmware/cmdline.txt..."
        echo 'l' | sudo -S sed -i 's/console=ttyAMA0,[0-9]* //g' /boot/firmware/cmdline.txt
    fi

    # Ensure admin is in dialout & grant chmod 666 /dev/ttyAMA0 immediately
    echo 'l' | sudo -S usermod -aG dialout,tty admin
    echo 'l' | sudo -S chmod 666 /dev/ttyAMA0
    """
    stdin, stdout, stderr = ssh.exec_command(setup_cmd, get_pty=True)
    print(stdout.read().decode())

    print("\n--- 2. Checking /dev/ttyAMA0 permissions now ---")
    stdin, stdout, stderr = ssh.exec_command("ls -la /dev/ttyAMA*", get_pty=True)
    print(stdout.read().decode())

    print("\n--- 3. Running MS200 LiDAR Test on /dev/ttyAMA0 with Sudo ---")
    stdin, stdout, stderr = ssh.exec_command("sudo python3 /home/admin/test_ms200.py /dev/ttyAMA0", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    ssh.close()

if __name__ == "__main__":
    setup_and_test()
