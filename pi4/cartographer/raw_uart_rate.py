import time
import serial

try:
    ser = serial.Serial('/dev/ttyAMA2', 921600, timeout=0.1)
    ser.write(b'+')
    ser.flush()
    time.sleep(0.2)
    ser.reset_input_buffer()

    t0 = time.time()
    total_bytes = 0
    packets = 0

    while time.time() - t0 < 3.0:
        n = ser.in_waiting or 42
        data = ser.read(n)
        if data:
            total_bytes += len(data)
            for i in range(len(data) - 1):
                if data[i] == 0xAA and data[i+1] == 0x55:
                    packets += 1

    dt = time.time() - t0
    print(f"Raw /dev/ttyAMA2 throughput: {total_bytes} bytes in {dt:.2f}s ({total_bytes/dt:.1f} bytes/s)")
    print(f"Packets detected: {packets} ({packets/dt:.1f} Hz)")
    ser.close()
except Exception as e:
    print(f"Error: {e}")
