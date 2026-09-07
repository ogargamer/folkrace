import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=4)

cmd = """
echo 'l' | sudo -S chmod 666 /dev/ttyAMA4 2>/dev/null || true
python3 - << 'EOF'
import serial
import time

try:
    ser = serial.Serial('/dev/ttyAMA4', 230400, timeout=1.0)
    print("Opened /dev/ttyAMA4 successfully!")
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < 3.0:
        n = ser.in_waiting
        if n > 0:
            chunk = ser.read(n)
            buf.extend(chunk)
            if len(buf) >= 47:
                print(f"Received {len(buf)} bytes! First bytes: {[hex(x) for x in buf[:12]]}")
                break
        time.sleep(0.01)
    else:
        print(f"No bytes received after 3s (read {len(buf)} bytes)")
    ser.close()
except Exception as e:
    print("Error:", e)
EOF
"""

stdin, stdout, stderr = ssh.exec_command(cmd)
stdin.close()
print(stdout.read().decode())
ssh.close()
