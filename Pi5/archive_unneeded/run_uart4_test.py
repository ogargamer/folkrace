import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def test_uart4():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "10.43.8.42"
    print(f"Connecting to Pi 5 at {ip}...")
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)
    print("Connected successfully!")

    # 1. Load uart4-pi5 dynamically if not loaded
    print("\n--- 1. Loading uart4-pi5 overlay ---")
    cmd_overlay = """
    echo 'l' | sudo -S dtoverlay uart4-pi5 2>/dev/null || true
    echo 'l' | sudo -S dtoverlay -l
    """
    stdin, stdout, stderr = ssh.exec_command(cmd_overlay, get_pty=True)
    print(stdout.read().decode())

    # 2. Check available serial devices
    print("\n--- 2. Checking available ttyAMA* / serial devices ---")
    stdin, stdout, stderr = ssh.exec_command("ls -la /dev/ttyAMA* /dev/serial* 2>/dev/null", get_pty=True)
    print(stdout.read().decode())

    # 3. Grant permissions
    ssh.exec_command("echo 'l' | sudo -S chmod 666 /dev/ttyAMA* /dev/serial* 2>/dev/null", get_pty=True)

    # 4. Upload updated test script
    sftp = ssh.open_sftp()
    local_file = r"c:\Users\nurmi\Documents\Da Winna\Pi5\test_ms200.py"
    remote_file = "/home/admin/test_ms200.py"
    sftp.put(local_file, remote_file)
    sftp.close()

    # 5. Run test on /dev/ttyAMA4 (or all available ttyAMA ports)
    print("\n================ 3. TESTING MS200 LIDAR STREAM ================")
    # First check raw byte stream
    test_cmd = """
    python3 - << 'EOF'
import glob
import os
import serial
import time

ports = sorted(glob.glob("/dev/ttyAMA*"))
print(f"Detected ports: {ports}")

for p in ports:
    print(f"\n--- Probing {p} at 230400 baud for 2 seconds ---")
    try:
        ser = serial.Serial(p, 230400, timeout=1.0)
        start = time.time()
        raw = b""
        while time.time() - start < 2.0:
            data = ser.read(ser.in_waiting or 1)
            if data:
                raw += data
                if len(raw) > 500:
                    break
        ser.close()
        print(f"  -> Received {len(raw)} bytes from {p}")
        if raw:
            print(f"  -> Sample Hex: {raw[:32].hex()}")
            if b'\x54\x2c' in raw:
                print("  ===> MATCH! MS200 LiDAR Header (0x54 0x2C) FOUND on " + p + "!")
    except Exception as e:
        print(f"  -> Error opening {p}: {e}")

EOF
    """
    stdin, stdout, stderr = ssh.exec_command(test_cmd, get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    # Now run the full MS200 parser on the active port (e.g. /dev/ttyAMA4)
    print("\n================ 4. RUNNING FULL MS200 PARSER ================")
    stdin, stdout, stderr = ssh.exec_command("python3 /home/admin/test_ms200.py /dev/ttyAMA4", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    ssh.close()

if __name__ == "__main__":
    test_uart4()
