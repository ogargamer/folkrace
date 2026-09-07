import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=4)

code = """
import lgpio
h = lgpio.gpiochip_open(4) # Pi 5 RP1 chip is 4!
lgpio.gpio_claim_input(h, 26, lgpio.SET_PULL_UP)
val = lgpio.gpio_read(h, 26)
print(f'lgpio GPIO 26 on gpiochip4 read success! Val={val}')
lgpio.gpio_free(h, 26)
lgpio.gpiochip_close(h)
"""

stdin, stdout, stderr = ssh.exec_command(f"python3 -c \"{code}\"")
print("STDOUT:", stdout.read().decode())
print("STDERR:", stderr.read().decode())
ssh.close()
