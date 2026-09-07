import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=4)

cmd = """python3 - << 'EOF'
import serial
import glob

for port in glob.glob('/dev/ttyAMA*') + glob.glob('/dev/ttyUSB*') + glob.glob('/dev/serial*'):
    try:
        s = serial.Serial(port, 230400, timeout=0.5)
        data = s.read(100)
        s.close()
        print(f"Port {port}: read {len(data)} bytes -> {[hex(x) for x in data[:8]]}")
    except Exception as e:
        print(f"Port {port} error: {e}")
EOF
"""

stdin, stdout, stderr = ssh.exec_command(cmd)
stdin.close()
print(stdout.read().decode())
ssh.close()
