import subprocess

res = subprocess.run(["gpioget", "--help"], capture_output=True, text=True)
print(res.stdout[:200])

res2 = subprocess.run(["ls", "-la", "/dev/gpiochip*"], capture_output=True, text=True)
print("GPIO Chips:\n", res2.stdout or res2.stderr)
