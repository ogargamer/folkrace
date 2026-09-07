import time
import spidev
import board
import digitalio

def test_wake_handshake():
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 1000000
    spi.mode = 3

    cs = digitalio.DigitalInOut(board.D16)
    cs.direction = digitalio.Direction.OUTPUT
    cs.value = True

    rst = digitalio.DigitalInOut(board.D21)
    rst.direction = digitalio.Direction.OUTPUT
    rst.value = True

    int_pin = digitalio.DigitalInOut(board.D26)
    int_pin.direction = digitalio.Direction.INPUT
    int_pin.pull = digitalio.Pull.UP

    def is_int():
        return not int_pin.value

    # Reset
    print("Resetting...")
    rst.value = False
    time.sleep(0.02)
    rst.value = True
    time.sleep(0.05)

    # Drain 3 boot packets
    for i in range(3):
        start = time.time()
        while not is_int() and (time.time() - start < 1.0):
            time.sleep(0.001)
        if is_int():
            cs.value = False
            time.sleep(0.00001)
            hdr = spi.xfer2([0x00, 0x00, 0x00, 0x00])
            plen = (hdr[0] | (hdr[1] << 8)) & ~0x8000
            if plen > 4:
                payload = spi.xfer2([0x00] * (plen - 4))
            cs.value = True
            time.sleep(0.001)
            print(f"Drained packet {i+1}: Chan {hdr[2]}, Len {plen}")

    time.sleep(0.05)
    print(f"INT pin state after boot drain: {'LOW (active)' if is_int() else 'HIGH (idle)'}")

    # Now test Host Wake: Pull CS LOW and observe INT
    print("Asserting CS LOW to request Host Write...")
    cs.value = False
    start_wake = time.time()
    int_went_low = False
    while time.time() - start_wake < 0.2:
        if is_int():
            int_went_low = True
            print(f"SUCCESS: BNO080 pulled INT LOW in {round((time.time() - start_wake)*1000, 2)} ms!")
            break
        time.sleep(0.0001)

    if not int_went_low:
        print("BNO080 did NOT pull INT LOW when CS went LOW.")

    # Send Product ID Request
    print("Sending Product ID Request [0x06, 0x00, 0x02, 0x00, 0xF9, 0x00]...")
    req = [0x06, 0x00, 0x02, 0x00, 0xF9, 0x00]
    spi.xfer2(req)
    cs.value = True
    time.sleep(0.005)

    print("Waiting for INT response...")
    start_resp = time.time()
    while time.time() - start_resp < 0.5:
        if is_int():
            cs.value = False
            time.sleep(0.00001)
            hdr = spi.xfer2([0x00, 0x00, 0x00, 0x00])
            plen = (hdr[0] | (hdr[1] << 8)) & ~0x8000
            payload = []
            if plen > 4:
                payload = spi.xfer2([0x00] * (plen - 4))
            cs.value = True
            print(f"RESPONSE RECEIVED! Chan={hdr[2]}, Len={plen}, Payload={[hex(x) for x in payload]}")
            break
        time.sleep(0.001)

    spi.close()

if __name__ == "__main__":
    test_wake_handshake()
