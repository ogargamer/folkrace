import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def test_lidar():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    # Try known IP
    ip = "10.43.8.42"
    print(f"Connecting to Pi 5 at {ip}...")
    try:
        ssh.connect(ip, port=22, username="admin", password="l", timeout=5)
    except Exception as e:
        print(f"Failed to connect to {ip}: {e}")
        return

    print("Connected to Pi 5 successfully!")
    
    # Check serial devices and kernel messages
    cmd = """
    python3 - << 'EOF'
import serial
import time
import os
import glob

print("Available serial ports:")
ports = glob.glob("/dev/ttyAMA*") + glob.glob("/dev/serial*") + glob.glob("/dev/ttyS*")
print(ports)

# Check /dev/ttyAMA0
target_ports = ['/dev/ttyAMA0', '/dev/serial0', '/dev/ttyAMA10']
for p in target_ports:
    if os.path.exists(p):
        print(f"\n--- Testing port {p} at 230400 baud ---")
        try:
            ser = serial.Serial(p, 230400, timeout=1.5)
            # Read for 2 seconds
            start = time.time()
            total_bytes = 0
            sample_data = b""
            while time.time() - start < 2.0:
                raw = ser.read(100)
                if raw:
                    total_bytes += len(raw)
                    if len(sample_data) < 128:
                        sample_data += raw
            ser.close()
            print(f"Result for {p}: Received {total_bytes} bytes in 2 seconds.")
            if sample_data:
                print(f"Sample hex data (first {len(sample_data)} bytes):")
                print(sample_data.hex())
                # Check for MS200/LD06 packet header 0x54
                if b'\x54' in sample_data:
                    print("--> Found 0x54 packet header (characteristic of MS200 / LD06 / DToF LiDAR)!")
        except Exception as e:
            print(f"Error accessing {p}: {e}")

EOF
    """
    
    stdin, stdout, stderr = ssh.exec_command(cmd, get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")
        
    ssh.close()

if __name__ == "__main__":
    test_lidar()
