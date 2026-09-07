import time
import os
import glob

# Try importing serial
try:
    import serial
except ImportError:
    import sys
    sys.exit(1)

ports = glob.glob("/dev/ttyAMA*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*") + glob.glob("/dev/serial*")
ports = list(set(ports))
print(f"Scanning available serial devices on Pi 5: {ports}")

bauds = [921600, 115200, 2000000, 460800]

for port in sorted(ports):
    # Skip LiDAR port on ttyAMA4 if occupied
    for baud in bauds:
        try:
            ser = serial.Serial(port, baud, timeout=0.5)
            # Drain old buffer
            ser.reset_input_buffer()
            t0 = time.time()
            received = b""
            while time.time() - t0 < 0.8:
                if ser.in_waiting > 0:
                    received += ser.read(ser.in_waiting)
                time.sleep(0.05)
            ser.close()

            if len(received) > 0:
                print(f"\n[FOUND ACTIVITY] Port: {port} @ {baud} Baud! Received {len(received)} bytes:")
                try:
                    print("  Text representation:\n", received.decode('utf-8', errors='ignore')[:300])
                except Exception:
                    pass
                print("  Hex representation:", received[:40].hex(' '))
        except Exception as e:
            # Port might be busy or inaccessible
            pass
