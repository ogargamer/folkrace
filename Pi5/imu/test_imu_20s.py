import time
import math
from bno08x_i2c import BNO08xI2C

print("Testing BNO08X IMU at 100Hz...")
imu = BNO08xI2C(bus_id=1, address=0x4B, rst_pin=21, int_pin=26)
imu.reset()

# Request 100Hz (10,000 us)
imu.enable_feature(BNO08xI2C.REPORT_ARVR_STABILIZED_GRV, 10000)
imu.enable_feature(BNO08xI2C.REPORT_ACCELEROMETER, 10000)
imu.enable_feature(BNO08xI2C.REPORT_GYROSCOPE, 10000)

print("\n--- Monitoring 20 seconds of IMU Data ---")
start = time.time()
last_print = 0
readings = []

while time.time() - start < 20.0:
    if imu.update():
        now = time.time()
        d = imu.data
        if now - last_print >= 1.0:
            last_print = now
            readings.append(d['yaw'])
            print(f"[{now-start:4.1f}s] R: {d['roll']:+5.1f}° | P: {d['pitch']:+5.1f}° | Yaw: {d['yaw']:+6.1f}° | Accel: ({d['accel_x']:+4.1f}, {d['accel_y']:+4.1f}, {d['accel_z']:+4.1f}) | GyroZ: {d['gyro_z']:+4.2f}")
    time.sleep(0.001)

yaw_range = max(readings) - min(readings) if readings else 0
print(f"\nTotal Yaw Range over 20s: {yaw_range:.2f}°")
