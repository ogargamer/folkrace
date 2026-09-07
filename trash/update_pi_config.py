import paramiko

s = paramiko.SSHClient()
s.set_missing_host_key_policy(paramiko.AutoAddPolicy())
s.connect('10.43.8.42', port=22, username='admin', password='l', timeout=4)

cmd = "echo l | sudo -S bash -c \"grep -q 'dtoverlay=uart2-pi5' /boot/firmware/config.txt || echo 'dtoverlay=uart2-pi5' >> /boot/firmware/config.txt\""
stdin, stdout, stderr = s.exec_command(cmd)
stdout.read()

stdin, stdout, stderr = s.exec_command('tail -n 6 /boot/firmware/config.txt')
print("Current config.txt tail:\n", stdout.read().decode('utf-8', errors='ignore'))
s.close()
