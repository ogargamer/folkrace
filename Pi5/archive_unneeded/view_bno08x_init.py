import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=3)

cmd = """python3 - << 'EOF'
with open('/tmp/bno08x_repo/lib/bno08x.py') as f:
    lines = f.readlines()
for i in range(min(120, len(lines))):
    print(lines[i], end='')
EOF
"""

stdin, stdout, stderr = ssh.exec_command(cmd)
print(stdout.read().decode('utf-8', errors='ignore'))
ssh.close()
