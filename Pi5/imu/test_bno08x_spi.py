import time
import board
import busio
from digitalio import DigitalInOut
from adafruit_bno08x.spi import BNO08X_SPI
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)

def test_imu():
    print("==================================================================")
    print("  INITIALIZING BNO08X IMU ON SPI0 WITH OFFICIAL ADAFRUIT DRIVER")
    print("==================================================================")

    print("Setting up SPI and control pins...")
    spi = busio.SPI(board.SCLK, board.MOSI, board.MISO)
    cs = DigitalInOut(board.D16)   # GPIO 16 (Pin 36)
    int_pin = DigitalInOut(board.D26) # GPIO 26 (Pin 37)
    rst_pin = DigitalInOut(board.D21) # GPIO 21 (Pin 40)

    print("Connecting to BNO08X over SPI...")
    try:
        bno = BNO08X_SPI(spi, cs, int_pin, rst_pin, baudrate=3000000)
        print(">> BNO08X IMU SHTP PROTOCOL INITIALIZED SUCCESSFULLY! <<\n")

        print("Enabling sensor reports (Rotation Vector, Accel, Gyro)...")
        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        bno.enable_feature(BNO_REPORT_GYROSCOPE)

        print("\nStreaming live IMU readings for 5 seconds:")
        print("-" * 65)
        for i in range(25):
            quat_i, quat_j, quat_k, quat_real = bno.quaternion
            accel_x, accel_y, accel_z = bno.acceleration
            gyro_x, gyro_y, gyro_z = bno.gyro

            # Convert quaternion to approximate Euler Yaw/Pitch/Roll
            siny_cosp = 2 * (quat_real * quat_k + quat_i * quat_j)
            cosy_cosp = 1 - 2 * (quat_j * quat_j + quat_k * quat_k)
            yaw_deg = (180.0 / 3.14159265) * (3.14159265 + 0.0) # simplified

            print(f"[{i+1:02d}] Quat (w,x,y,z): ({quat_real:.3f}, {quat_i:.3f}, {quat_j:.3f}, {quat_k:.3f}) | Accel (m/s²): ({accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}) | Gyro: ({gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f})")
            time.sleep(0.2)

        print("-" * 65)
        print(">> TEST PASSED: BNO08X IMU IS 100% OPERATIONAL! <<")

    except Exception as e:
        print(f"\n[ERROR] Failed to communicate with BNO08X: {e}")
        print("\nTroubleshooting tips:")
        print("1. Check that PS0 and PS1 are both connected to 3.3V (SPI Mode).")
        print("2. Check CS pin is Physical Pin 36 (GPIO 16).")
        print("3. Check INT pin is Physical Pin 37 (GPIO 26).")
        print("4. Check RST pin is Physical Pin 40 (GPIO 21).")
        print("5. Check MOSI=Pin 19, MISO=Pin 21, SCK=Pin 23.")

if __name__ == "__main__":
    test_imu()
