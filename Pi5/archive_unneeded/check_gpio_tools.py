import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def find_gpio_tools():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  CHECKING GPIO TOOLS AND CONTROLLING LPn (GPIO 17)")
    print("======================================================================")

    script = r"""#!/bin/bash
echo "==> Looking for available GPIO tools..."
which gpioset || true
which pinctrl || true
which raspi-gpio || true
which gpiod || true

echo ""
echo "==> Listing gpiochips..."
ls -l /dev/gpiochip* || true

echo ""
echo "==> Testing python gpiod / RPi.GPIO to drive GPIO 17 HIGH..."
python3 - << 'PYEOF'
try:
    import gpiod
    print("gpiod module available!")
    # On Pi 5, RP1 GPIOs are on gpiochip4
    chip = gpiod.Chip('gpiochip4')
    line = chip.get_line(17)
    line.request(consumer="tof_reset", type=gpiod.LINE_REQ_DIR_OUT)
    line.set_value(0)
    time.sleep(0.05)
    line.set_value(1)
    time.sleep(0.1)
    print("GPIO 17 successfully pulsed LOW -> HIGH via gpiod!")
    line.release()
    chip.close()
except Exception as e:
    print(f"gpiod error: {e}")
PYEOF
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/check_gpio.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/check_gpio.sh && /tmp/check_gpio.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    find_gpio_tools()
