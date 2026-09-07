import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def test_pinctrl_and_run():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  CONFIGURING GPIO 17 PERMANENTLY HIGH VIA PINCTRL & RUNNING RANGING")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e

echo "==> Configuring GPIO 17 as Output Drive High (dh)..."
sudo pinctrl set 17 op dh || sudo raspi-gpio set 17 op dh || true
sleep 0.1

echo "==> GPIO 17 state:"
sudo pinctrl get 17 || sudo raspi-gpio get 17 || true

echo ""
echo "==> Running STSW-IMG042 multi..."
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

# Run option 1 (basic ranging)
echo "1" | ./multi
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_pinctrl.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_pinctrl.sh && echo 'l' | sudo -S /tmp/run_pinctrl.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_pinctrl_and_run()
