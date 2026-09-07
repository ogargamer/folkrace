import time
import smbus2

def test_raw_i2c():
    bus = smbus2.SMBus(1)
    addr = 0x4B
    print(f"Testing raw I2C read from address 0x{addr:02X}...")
    try:
        # Read 4-byte SHTP header
        msg = smbus2.i2c_msg.read(addr, 4)
        bus.i2c_rdwr(msg)
        hdr = list(msg)
        print("Read SHTP Header:", [hex(x) for x in hdr])
        plen = (hdr[0] | (hdr[1] << 8)) & ~0x8000
        print(f"Packet Length: {plen}, Channel: {hdr[2]}, Seq: {hdr[3]}")
    except Exception as e:
        print("I2C read exception:", e)
    bus.close()

if __name__ == "__main__":
    test_raw_i2c()
