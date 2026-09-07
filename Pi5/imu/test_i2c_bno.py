import time
import board
import busio
import digitalio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE
)
import math

def test_i2c_100k():
    print("==================================================================")
    print("  BNO08X I2C 100kHz / 50kHz TEST ON RASPBERRY PI 5")
    print("==================================================================")

    # 100kHz I2C bus
    i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
    rst_pin = digitalio.DigitalInOut(board.D21)

    print("Connecting at 100kHz...")
    try:
        bno = BNO08X_I2C(i2c, address=0x4B, reset=rst_pin)
        print("\n>> SUCCESS: BNO08X CONNECTED OVER I2C (100kHz)! <<\n")

        bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
        bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        bno.enable_feature(BNO_REPORT_GYROSCOPE)
        time.sleep(0.1)

        print("Streaming live 9-DOF IMU Telemetry:")
        print("-" * 80)
        for i in range(25):
            qi, qj, qk, qr = bno.game_quaternion
            ax, ay, az = bno.acceleration
            gx, gy, gz = bno.gyro

            sinr_cosp = 2.0 * (qr * qi + qj * qk)
            cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj)
            roll = math.atan2(sinr_cosp, cosr_cosp) * (180.0 / math.pi)

            sinp = 2.0 * (qr * qj - qk * qi)
            pitch = math.asin(max(-1.0, min(1.0, sinp))) * (180.0 / math.pi)

            siny_cosp = 2.0 * (qr * qk + qi * qj)
            cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk)
            yaw = math.atan2(siny_cosp, cosy_cosp) * (180.0 / math.pi)

            print(f"[{i+1:02d}] ROLL:{roll:+6.2f}° | PITCH:{pitch:+6.2f}° | YAW:{yaw:+6.2f}° | Quat:({qr:+.3f},{qi:+.3f},{qj:+.3f},{qk:+.3f}) | Accel:({ax:+5.2f},{ay:+5.2f},{az:+5.2f})")
            time.sleep(0.1)

        print("-" * 80)
        print(">> TEST PASSED! <<")
    except Exception as e:
        print("[100kHz Error]:", e)

if __name__ == "__main__":
    test_i2c_100k()
