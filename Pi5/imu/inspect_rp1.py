import subprocess

p = subprocess.Popen(["sudo", "-S", "cat", "/sys/kernel/debug/gpio"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
out, err = p.communicate(input="l\n")

for line in out.split("\n"):
    if "gpiochip571" in line or any(f"gpio-{571+i}" in line for i in range(28)):
        print(line)
