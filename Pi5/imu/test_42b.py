import serial
import struct
import time

ser = serial.Serial('/dev/ttyAMA2', 921600, timeout=0.1)
print("Listening for 42-byte binary telemetry on /dev/ttyAMA2...")

t0 = time.time()
count = 0
while time.time() - t0 < 2.0:
    raw = ser.read(ser.in_waiting or 42)
    if not raw:
        continue
    idx = 0
    while idx <= len(raw) - 42:
        if raw[idx] == 0xAA and raw[idx+1] == 0x55:
            qw, qx, qy, qz, ax, ay, az, gx, gy, gz = struct.unpack_from('<ffffffffff', raw, idx + 2)
            if count < 5:
                print(f"  [FRAME {count+1}] Quat(W:{qw:.3f} X:{qx:.3f} Y:{qy:.3f} Z:{qz:.3f}) | Accel({ax:.2f}, {ay:.2f}, {az:.2f}) m/s² | Gyro({gx:.2f}, {gy:.2f}, {gz:.2f}) rad/s")
            count += 1
            idx += 42
        else:
            idx += 1

print(f"\n[SUMMARY] Total 42-byte frames in 2s: {count} (~{count/2.0:.1f} Hz)")
ser.close()
