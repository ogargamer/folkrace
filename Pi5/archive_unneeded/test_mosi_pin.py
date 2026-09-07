import time
import spidev
import board
import digitalio

def check_pins():
    print("Testing GPIO lines...")
    # Open SPI
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 1000000
    spi.mode = 3

    # Send 1000 bytes pattern on MOSI
    pattern = [0xAA, 0x55] * 50
    spi.xfer2(pattern)
    print("MOSI clocked 100 bytes successfully via spidev.")
    spi.close()

if __name__ == "__main__":
    check_pins()
