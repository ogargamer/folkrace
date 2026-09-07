import serial
import time
import os
import glob

print("Scanning all Pi 5 UART / Serial ports for live ESP32 incoming data:")

# Make sure uart2 overlay is active
ports = ["/dev/ttyAMA2", "/dev/ttyAMA0", "/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyAMA10"]
available_ports = [p for p in ports if os.path.exists(p)]
print(f"Available ports: {available_ports}")

bauds = [921600, 115200, 2000000, 460800]

found_any = False

for port in available_ports:
    for baud in bauds:
        try:
            ser = serial.Serial(port, baud, timeout=0.5)
            ser.reset_input_buffer()
            t0 = time.time()
            data = b""
            while time.time() - t0 < 0.6:
                if ser.in_waiting > 0:
                    data += ser.read(ser.in_waiting)
                time.sleep(0.02)
            ser.close()

            if len(data) > 0:
                # Check if this is LiDAR (starts with 0x54 0x2C)
                if len(data) >= 2 and data[0] == 0x54 and data[1] == 0x2C:
                    print(f"  [LiDAR Port] {port} @ {baud} Baud (received {len(data)} LiDAR bytes)")
                else:
                    found_any = True
                    print(f"\n========================================================")
                    print(f"  [ACTIVE DATA DETECTED!] Port: {port} @ {baud} Baud")
                    print(f"  Received: {len(data)} bytes in 0.6s ({len(data)/0.6:.1f} bytes/sec)")
                    print(f"  Raw Hex : {data[:64].hex(' ')}")
                    try:
                        print(f"  Text    : {data.decode('utf-8', errors='ignore')[:300]}")
                    except Exception:
                        pass
                    print(f"========================================================\n")
        except Exception as e:
            pass

if not found_any:
    print("\n[RESULT] No data detected on /dev/ttyAMA0, /dev/ttyAMA2, /dev/ttyACM0, /dev/ttyUSB0.")
    print("Checking if GPIO pins or overlay need configuration...")
