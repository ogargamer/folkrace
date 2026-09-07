import time
import paramiko

print("Waiting for Raspberry Pi 5 to finish booting...")
time.sleep(8)

s = paramiko.SSHClient()
s.set_missing_host_key_policy(paramiko.AutoAddPolicy())

connected = False
for attempt in range(1, 16):
    try:
        s.connect('10.43.8.42', port=22, username='admin', password='l', timeout=3)
        print(f"[OK] Pi 5 connected on attempt {attempt}!")
        connected = True
        break
    except Exception:
        time.sleep(2)

if connected:
    stdin, stdout, stderr = s.exec_command('ls -l /dev/ttyAMA*')
    print("Available serial ports:\n", stdout.read().decode('utf-8', errors='ignore'))
    
    # Run test_esp_uart_live.py
    stdin, stdout, stderr = s.exec_command('python3 /home/admin/test_esp_uart_live.py')
    print("UART Probe Results:\n", stdout.read().decode('utf-8', errors='ignore'))
    s.close()
else:
    print("[ERROR] Pi 5 did not respond within timeout.")
