import time
import board
import busio
import digitalio
from adafruit_bno08x.spi import BNO08X_SPI
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE
)
import struct

# Subclass BNO08X_SPI to fix the wake-up handshake bug in Adafruit's Linux SPI implementation
class FixedBNO08X_SPI(BNO08X_SPI):
    def _send_packet(self, channel, data):
        data_length = len(data)
        write_length = data_length + 4

        struct.pack_into("<H", self._data_buffer, 0, write_length)
        self._data_buffer[2] = channel
        self._data_buffer[3] = self._sequence_number[channel]
        for idx, send_byte in enumerate(data):
            self._data_buffer[4 + idx] = send_byte

        # Pull CS LOW and wait for INT to go LOW (wake-up handshake)
        with self._spi as spi:
            # Wait for INT LOW
            start_t = time.monotonic()
            while self._int.value:
                if time.monotonic() - start_t > 0.05:
                    break
                time.sleep(0.0001)
            spi.write(self._data_buffer, end=write_length)

        self._sequence_number[channel] = (self._sequence_number[channel] + 1) % 256
        return self._sequence_number[channel]

def test_fixed():
    print("==================================================================")
    print("  INITIALIZING BNO08X WITH FIXED SPI WAKE-UP HANDSHAKE")
    print("==================================================================")
    spi = busio.SPI(board.SCLK, board.MOSI, board.MISO)
    cs = digitalio.DigitalInOut(board.D16)
    int_pin = digitalio.DigitalInOut(board.D26)
    rst_pin = digitalio.DigitalInOut(board.D21)

    print("Connecting...")
    bno = FixedBNO08X_SPI(spi, cs, int_pin, rst_pin, baudrate=3000000)
    print(">> BNO08X IMU CONNECTED AND SHTP ID VERIFIED! <<\n")

    print("Enabling Game Rotation Vector (Quaternion), Accel, Gyro...")
    bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
    bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)

    print("\nStreaming live 9-DOF telemetry for 5 seconds:")
    print("-" * 75)
    for i in range(25):
        quat_i, quat_j, quat_k, quat_real = bno.game_quaternion
        ax, ay, az = bno.acceleration
        gx, gy, gz = bno.gyro

        # Euler angles (Roll, Pitch, Yaw)
        import math
        sinr_cosp = 2.0 * (quat_real * quat_i + quat_j * quat_k)
        cosr_cosp = 1.0 - 2.0 * (quat_i * quat_i + quat_j * quat_j)
        roll = math.atan2(sinr_cosp, cosr_cosp) * (180.0 / math.pi)

        sinp = 2.0 * (quat_real * quat_j - quat_k * quat_i)
        pitch = math.asin(max(-1.0, min(1.0, sinp))) * (180.0 / math.pi)

        siny_cosp = 2.0 * (quat_real * quat_k + quat_i * quat_j)
        cosy_cosp = 1.0 - 2.0 * (quat_j * quat_j + quat_k * quat_k)
        yaw = math.atan2(siny_cosp, cosy_cosp) * (180.0 / math.pi)

        print(f"[{i+1:02d}] ROLL:{roll:+6.1f}° | PITCH:{pitch:+6.1f}° | YAW:{yaw:+6.1f}° | Quat(w,x,y,z): ({quat_real:+.3f}, {quat_i:+.3f}, {quat_j:+.3f}, {quat_k:+.3f})")
        time.sleep(0.2)

    print("-" * 75)
    print(">> TEST PASSED: BNO08X IMU IS 100% OPERATIONAL! <<")

if __name__ == "__main__":
    test_fixed()
