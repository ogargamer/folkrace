import time
import board
import busio
import digitalio
from adafruit_bno08x.spi import BNO08X_SPI
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE
)

def test():
    spi = busio.SPI(board.SCLK, board.MOSI, board.MISO)
    cs = digitalio.DigitalInOut(board.D16)
    int_pin = digitalio.DigitalInOut(board.D26)
    rst_pin = digitalio.DigitalInOut(board.D21)

    print("Attempting to connect with BNO08X_SPI (debug=True)...")
    try:
        bno = BNO08X_SPI(spi, cs, int_pin, rst_pin, baudrate=1000000, debug=True)
        print("Connected successfully!")
        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        for i in range(10):
            print("Quat:", bno.quaternion)
            time.sleep(0.1)
    except Exception as e:
        print("Exception:", e)

if __name__ == "__main__":
    test()
