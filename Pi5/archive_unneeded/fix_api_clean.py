import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def fix_api_and_test():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "192.168.0.13"
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)

    print("======================================================================")
    print("  CLEANING vl53l8cx_api.c & COMPILING SOLID RANGING DRIVER")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0

# Extract clean copy of vl53l8cx_api.c from user/uld-driver/src
cd user/uld-driver/src

cat << 'EOF' > patch.py
with open("vl53l8cx_api.c", "r") as f:
    lines = f.readlines()

new_lines = []
skip = False
for line in lines:
    if "uint8_t vl53l8cx_is_alive(" in line:
        new_lines.append("uint8_t vl53l8cx_is_alive(VL53L8CX_Configuration *p_dev, uint8_t *p_is_alive) {\n")
        new_lines.append("    uint8_t status = VL53L8CX_STATUS_OK;\n")
        new_lines.append("    uint8_t id_buf[2] = {0};\n")
        new_lines.append("    status |= VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x00);\n")
        new_lines.append("    status |= VL53L8CX_RdMulti(&(p_dev->platform), 0x0000, id_buf, 2);\n")
        new_lines.append("    status |= VL53L8CX_WrByte(&(p_dev->platform), 0x7fff, 0x02);\n")
        new_lines.append("    *p_is_alive = ((id_buf[0] == 0xF0) && (id_buf[1] == 0x0C)) ? 1 : 0;\n")
        new_lines.append("    return status;\n}\n")
        skip = True
    elif skip and line.startswith("}"):
        skip = False
    elif not skip:
        new_lines.append(line)

with open("vl53l8cx_api.c", "w") as f:
    f.writelines(new_lines)
print("vl53l8cx_api.c patched successfully!")
EOF
python3 patch.py

cd ../../test
make clean || true
make CFLAGS_RELEASE="-Os -g0 -DSPI" PLTF_DIR=../platform

echo "==> Running test_c_zero..."
./test_c_zero
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/clean_api.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/clean_api.sh && /tmp/clean_api.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    fix_api_and_test()
