import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=3)

cmd = """python3 - << 'EOF'
import inspect
import adafruit_bno08x
import adafruit_bno08x.spi

print("=== BNO08X_SPI ===")
print(inspect.getsource(adafruit_bno08x.spi.BNO08X_SPI))

print("=== hard_reset ===")
print(inspect.getsource(adafruit_bno08x.BNO08X.hard_reset))

print("=== _wait_for_packet_type ===")
print(inspect.getsource(adafruit_bno08x.BNO08X._wait_for_packet_type))
EOF
"""

stdin, stdout, stderr = ssh.exec_command(cmd, get_pty=True)
print(stdout.read().decode())
ssh.close()
