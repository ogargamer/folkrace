import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=4)

cmd = """python3 - << 'EOF'
import serial
import struct
import time

def xor_check(data):
    chk = 0
    for b in data:
        chk ^= b
    return chk

def set_speed(ser, hz):
    header = struct.pack("<HBBB", 0xF5A5, 0xA1, 0xC1, 2)
    payload = struct.pack("<H", hz)
    chk = xor_check(header + payload)
    tail = bytes([chk, 0x31, 0xF2])
    pkt = header + payload + tail
    ser.write(pkt)
    ser.flush()

for p in ['/dev/ttyAMA4', '/dev/ttyAMA0', '/dev/serial0']:
    try:
        ser = serial.Serial(p, 230400, timeout=1.0)
        set_speed(ser, 10)
        time.sleep(0.5)
        raw = ser.read(100)
        ser.close()
        print(f"Port {p}: read {len(raw)} bytes -> {[hex(x) for x in raw[:12]]}")
    except Exception as e:
        print(f"Port {p} error: {e}")
EOF
"""

stdin, stdout, stderr = ssh.exec_command(cmd)
stdin.close()
print(stdout.read().decode())
ssh.close()
