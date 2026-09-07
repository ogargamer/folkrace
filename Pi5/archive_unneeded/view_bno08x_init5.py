import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=3)

cmd = """python3 - << 'EOF'
with open('/tmp/bno08x_repo/lib/bno08x.py') as f:
    text = f.read()

idx = text.find("def __init__")
if idx != -1:
    print(text[idx:idx+2000])
EOF
"""

stdin, stdout, stderr = ssh.exec_command(cmd)
print(stdout.read().decode('utf-8', errors='ignore'))
ssh.close()
