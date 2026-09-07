import serial
import time
import struct

print("Testing /dev/ttyAMA2 @ 921600 Baud for 3 seconds...")
try:
    ser = serial.Serial('/dev/ttyAMA2', 921600, timeout=0.5)
    ser.reset_input_buffer()
    t0 = time.time()
    total_bytes = 0
    frames_decoded = 0

    while time.time() - t0 < 3.0:
        raw = ser.read(ser.in_waiting or 30)
        if raw:
            total_bytes += len(raw)
            idx = 0
            while idx < len(raw) - 1:
                if raw[idx] == 0xAA and raw[idx+1] == 0x55 and idx + 30 <= len(raw):
                    qw, qx, qy, qz, ax, ay, az = struct.unpack_from("<fffffff", raw, idx + 2)
                    if frames_decoded < 5:
                        print(f"  [FRAME {frames_decoded+1}] Quat(W:{qw:.3f} X:{qx:.3f} Y:{qy:.3f} Z:{qz:.3f}) Accel({ax:.2f}, {ay:.2f}, {az:.2f})")
                    frames_decoded += 1
                    idx += 30
                else:
                    idx += 1
        time.sleep(0.01)
    ser.close()

    print(f"\n[SUMMARY] Total bytes received: {total_bytes} bytes in 3s ({total_bytes/3.0:.1f} B/s). Frames decoded: {frames_decoded}")
except Exception as e:
    print(f"[ERROR] {e}")
