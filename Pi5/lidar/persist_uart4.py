import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=4)

cmd = """
echo 'l' | sudo -S sed -i '/dtoverlay=uart4/d' /boot/firmware/config.txt
echo 'l' | sudo -S sed -i '/dtoverlay=uart4-pi5/d' /boot/firmware/config.txt
echo 'dtoverlay=uart4-pi5' | sudo tee -a /boot/firmware/config.txt
echo 'l' | sudo -S chmod 666 /dev/ttyAMA*
"""

stdin, stdout, stderr = ssh.exec_command(cmd)
stdin.close()
print(stdout.read().decode())
ssh.close()
print("Persisted uart4-pi5 in config.txt!")
