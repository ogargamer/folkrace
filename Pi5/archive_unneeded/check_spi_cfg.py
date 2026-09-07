import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=3)

cmd = """python3 - << 'EOF'
import subprocess

print("=== /boot/firmware/config.txt SPI and Overlay settings ===")
try:
    with open('/boot/firmware/config.txt') as f:
        for line in f:
            if 'spi' in line.lower() or 'dtoverlay' in line.lower():
                print(line.strip())
except Exception as e:
    print("Error reading config.txt:", e)

print("\n=== SPI devices in /sys/class/spidev/ ===")
res = subprocess.getoutput("ls -la /sys/class/spidev/")
print(res)

print("\n=== Kernel messages related to spi ===")
res = subprocess.getoutput("dmesg | grep -i spi")
print(res[:1000])
EOF
"""

stdin, stdout, stderr = ssh.exec_command(cmd)
print(stdout.read().decode('utf-8', errors='ignore'))
ssh.close()
