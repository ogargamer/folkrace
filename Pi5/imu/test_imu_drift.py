import time
import math
from bno08x_i2c import BNO08xI2C

print("Initializing BNO08X on I2C...")
imu = BNO08xI2C(bus_id=1, address=0x4B, rst_pin=21, int_pin=26)
imu.reset()

# Enable Game Rotation Vector & ARVR Stabilized
imu.enable_feature(BNO08xI2C.REPORT_GAME_ROTATION_VECTOR, 10000) # 100Hz
imu.enable_feature(0x28, 10000) # ARVR Stabilized GRV
imu.enable_feature(BNO08xI2C.REPORT_ACCELEROMETER, 10000)
imu.enable_feature(BNO08xI2C.REPORT_GYROSCOPE, 10000)

print("Measuring IMU Yaw for 10 seconds (Sensor should be STATIONARY)...")
start = time.time()
last_print = 0

initial_yaw = None

while time.time() - start < 10.0:
    if imu.update():
        now = time.time()
        if now - last_print >= 1.0:
            last_print = now
            d = imu.data
            if initial_yaw is None:
                initial_yaw = d['yaw']
            drift = d['yaw'] - initial_yaw
            print(f"Time: {now - start:4.1f}s | Roll: {d['roll']:+6.2f}° | Pitch: {d['pitch']:+6.2f}° | Yaw: {d['yaw']:+6.2f}° | Drift: {drift:+6.2f}° | GyroZ: {d['gyro_z']:+6.2f}")
    time.sleep(0.001)
