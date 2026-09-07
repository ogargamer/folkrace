import time
import spidev
import board
import digitalio
import struct

def debug_shtp():
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 3000000
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
    print("Resetting BNO08X...")
    rst.value = False
    time.sleep(0.02)
    rst.value = True
    time.sleep(0.05)

    start = time.time()
    while not is_int() and (time.time() - start < 2.0):
        time.sleep(0.001)
    print(f"INT asserted in {round((time.time()-start)*1000, 1)} ms")

    # Read helper
    def read_pkt():
        if not is_int():
            return None
        cs.value = False
        time.sleep(0.0001)
        hdr = spi.xfer2([0x00, 0x00, 0x00, 0x00])
        pkt_len = (hdr[0] | (hdr[1] << 8)) & ~0x8000
        chan = hdr[2]
        seq = hdr[3]
        payload = []
        if 4 < pkt_len < 1024:
            payload = spi.xfer2([0x00] * (pkt_len - 4))
        cs.value = True
        time.sleep(0.0005)
        return chan, seq, pkt_len, payload

    # Read boot packets
    while True:
        pkt = read_pkt()
        if not pkt:
            time.sleep(0.05)
            if not is_int():
                break
        else:
            print(f"Boot SHTP: Chan={pkt[0]}, Len={pkt[2]}, Seq={pkt[1]}, Data={[hex(x) for x in pkt[3][:8]]}")

    # Let's send a command: Soft Reset / Check Executable
    print("\nSending Executable Channel Command (Reset)...")
    cmd_exe = [0x05, 0x00, 0x01, 0x00, 0x01] # Len 5, Chan 1 (EXE), Seq 0, Payload: 0x01 (Reset)
    cs.value = False
    start_w = time.time()
    while not is_int() and (time.time() - start_w < 0.05):
        time.sleep(0.0001)
    spi.xfer2(cmd_exe)
    cs.value = True
    time.sleep(0.01)

    print("Waiting for response to Executable Reset...")
    for _ in range(5):
        start_r = time.time()
        while not is_int() and (time.time() - start_r < 0.5):
            time.sleep(0.001)
        pkt = read_pkt()
        if pkt:
            print(f"Response: Chan={pkt[0]}, Len={pkt[2]}, Data={[hex(x) for x in pkt[3][:8]]}")

    spi.close()

if __name__ == "__main__":
    debug_shtp()
