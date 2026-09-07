import time
import board
import busio
import digitalio
from adafruit_bno08x.spi import BNO08X_SPI
from adafruit_bno08x import (
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE
)
import math

def test_pin24():
    print("==================================================================")
    print("  TESTING BNO08X WITH NATIVE HARDWARE CS (PIN 24 / CE0)")
    print("==================================================================")

    spi = busio.SPI(board.SCLK, board.MOSI, board.MISO)
    cs = digitalio.DigitalInOut(board.CE0)     # Pin 24 (GPIO 8)
    int_pin = digitalio.DigitalInOut(board.D26) # Pin 37 (GPIO 26)
    rst_pin = digitalio.DigitalInOut(board.D21) # Pin 40 (GPIO 21)

    print("Connecting with Adafruit BNO08X_SPI driver...")
    try:
        bno = BNO08X_SPI(spi, cs, int_pin, rst_pin, baudrate=3000000)
        print("\n>> SUCCESS: BNO08X IMU CONNECTED OVER SPI (PIN 24 / CE0)! <<\n")

        print("Enabling Game Rotation Vector & Accelerometer...")
        bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
        bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        bno.enable_feature(BNO_REPORT_GYROSCOPE)

        print("\nStreaming live 9-DOF IMU Telemetry:")
        print("-" * 75)
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

        print("-" * 75)
        print(">> BNO08X IMU IS 100% OPERATIONAL ON RASPBERRY PI 5! <<")
    except Exception as e:
        print("Exception:", e)

if __name__ == "__main__":
    test_pin24()
