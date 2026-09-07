import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=3)
stdin, stdout, stderr = ssh.exec_command("python3 -m pip list")
pkgs = stdout.read().decode()
for name in ["websockets", "aiohttp", "tornado", "flask", "fastapi", "serial", "pyserial"]:
    print(f"{name}: {'FOUND' if name in pkgs.lower() else 'NOT FOUND'}")
ssh.close()
