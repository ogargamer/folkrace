import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=4)

code = """
import gpiod
chip = gpiod.Chip('gpiochip4')
line = chip.get_line(26)
line.request(consumer='bno_test', type=gpiod.LINE_REQ_DIR_IN)
val = line.get_value()
line.release()
print(f'gpiod GPIO 26 on gpiochip4 read success! Val={val}')
"""

stdin, stdout, stderr = ssh.exec_command(f"python3 -c \"{code}\"")
print("STDOUT:", stdout.read().decode())
print("STDERR:", stderr.read().decode())
ssh.close()
