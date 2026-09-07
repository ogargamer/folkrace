import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=4)

cmd = """
source /opt/ros/jazzy/setup.bash
python3 - << 'EOF'
import serial
import struct
import time

def xor_check(data):
    chk = 0
    for b in data: chk ^= b
    return chk

def send_speed(ser, hz=10):
    header = struct.pack("<HBBB", 0xF5A5, 0xA1, 0xC1, 2)
    payload = struct.pack("<H", hz)
    chk = xor_check(header + payload)
    tail = bytes([chk, 0x31, 0xF2])
    ser.write(header + payload + tail)
    ser.flush()

for port in ['/dev/ttyAMA4', '/dev/ttyAMA0']:
    try:
        s = serial.Serial(port, 230400, timeout=1.0)
        print(f"Testing {port}...")
        send_speed(s, 10)
        time.sleep(0.5)
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < 3.0:
            c = s.read(s.in_waiting or 1)
            if c: buf.extend(c)
            if len(buf) > 100:
                print(f"  SUCCESS! Received {len(buf)} bytes from {port}: {[hex(x) for x in buf[:10]]}")
                break
        else:
            print(f"  Received {len(buf)} bytes (no frames).")
        s.close()
    except Exception as e:
        print(f"  Port {port} error: {e}")
EOF
"""

stdin, stdout, stderr = ssh.exec_command(cmd)
stdin.close()
print(stdout.read().decode())
ssh.close()
