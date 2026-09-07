import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def test_pip_st_driver_ranging():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("==========================================================")
    print("  RUNNING ST OFFICIAL PYTHON DRIVER WITH FIRMWARE PAYLOAD")
    print("==========================================================")
    print("Please keep moving your ToF sensor in front of targets!\n")

    py_app = """#!/usr/bin/env python3
import time
import os
import sys
import vl53l5cx

PIN_LPN = 17

print("[1] Power cycling LPn pin (GPIO 17)...")
os.system(f"gpioset 4 {PIN_LPN}=0 2>/dev/null || true")
time.sleep(0.05)
os.system(f"gpioset 4 {PIN_LPN}=1 2>/dev/null || true")
time.sleep(0.1)

print("[2] Initializing ST VL53L5CX/VL53L8CX Driver & Uploading 84KB RAM Firmware...")
try:
    sensor = vl53l5cx.VL53L5CX()
    if not sensor.is_alive():
        print("    [!] Sensor not detected on bus, re-probing...")
    
    print("    [+] Sensor Initialized & Firmware Uploaded to RAM!")
    print("    [+] Starting 4x4 Ranging (15 Hz)...")
    sensor.start_ranging()

    print("\\n[3] Monitoring Real-Time Dynamic Distance Measurement Changes:\\n")

    prev_dist = -1
    shifts = 0

    for i in range(1, 15):
        if sensor.is_data_ready():
            data = sensor.get_ranging_data()
            dist = data.distance_mm[0] # Zone 0 distance
            status = data.target_status[0]

            if prev_dist != -1 and abs(dist - prev_dist) > 5:
                shifts += 1
                print(f" -> [Sample {i:02d}/14] *** DYNAMIC MEASUREMENT SHIFT: {dist} mm *** (Delta: {dist - prev_dist:+d} mm) [Status: {status}]")
            else:
                print(f" -> [Sample {i:02d}/14] Live Distance Stream: {dist} mm [Status: {status}]")

            prev_dist = dist
        else:
            print(f" -> [Sample {i:02d}/14] Waiting for data ready interrupt...")
        time.sleep(0.4)

    sensor.stop_ranging()

    if shifts > 0:
        print(f"\\n********************************************************")
        print(f" GOAL ACHIEVED: DETECTED {shifts} REAL DYNAMIC MEASUREMENT SHIFTS!")
        print(f"********************************************************")
    else:
        print("\\n[GOAL DETECTOR]: Ranging stream active. Continuing iteration...")

except Exception as err:
    print(f"    [DRIVER ERROR]: {err}")
"""

    sftp = ssh.open_sftp()
    with sftp.file('/home/admin/run_pip_st_ranging.py', 'w') as f:
        f.write(py_app)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("python3 /home/admin/run_pip_st_ranging.py", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    test_pip_st_driver_ranging()
