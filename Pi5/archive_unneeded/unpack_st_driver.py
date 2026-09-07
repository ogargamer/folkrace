import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def download_and_install_official_st_driver_python():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  UNPACKING ST DRIVER ZIP VIA PYTHON & BUILDING OFFICIAL ST C DRIVER")
    print("======================================================================")

    script = """#!/usr/bin/env python3
import urllib.request
import zipfile
import os
import subprocess

target_dir = "/home/admin/official_st_driver"
os.makedirs(target_dir, exist_ok=True)
zip_path = os.path.join(target_dir, "st_driver.zip")

print("[1] Downloading STMicroelectronics VL53L8CX Official Linux Archive...")
url = "https://github.com/stm32-hotspot/VL53L8CX_ULD_Linux/archive/refs/heads/main.zip"
urllib.request.urlretrieve(url, zip_path)

print("[2] Extracting archive...")
with zipfile.ZipFile(zip_path, 'r') as zip_ref:
    zip_ref.extractall(target_dir)

extracted_path = os.path.join(target_dir, "VL53L8CX_ULD_Linux-main")
print(f"[3] Extracted ST Driver contents at: {extracted_path}")
print("    Source files:", os.listdir(os.path.join(extracted_path, "VL53L8CX_ULD_API", "src")))
"""

    sftp = ssh.open_sftp()
    with sftp.file('/home/admin/unpack_st.py', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("python3 /home/admin/unpack_st.py", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nUnpack Exit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    download_and_install_official_st_driver_python()
