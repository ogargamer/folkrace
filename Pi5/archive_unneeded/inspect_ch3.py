import time
import spidev
import board
import digitalio
import struct

def inspect_channel3():
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

    def set_cs(active):
        cs.value = not active

    # Reset
    rst.value = False
    time.sleep(0.02)
    rst.value = True
    time.sleep(0.05)

    start = time.time()
    while not is_int() and (time.time() - start < 2.0):
        time.sleep(0.001)

    def read_packet(timeout=0.2):
        start_t = time.time()
        while not is_int():
            if time.time() - start_t > timeout:
                return None
            time.sleep(0.0002)

        set_cs(True)
        time.sleep(0.0001)
        hdr = spi.xfer2([0x00, 0x00, 0x00, 0x00])
        pkt_len = (hdr[0] | (hdr[1] << 8)) & ~0x8000
        channel = hdr[2]
        seq = hdr[3]

        payload = []
        if 4 < pkt_len < 1024:
            payload = spi.xfer2([0x00] * (pkt_len - 4))
        set_cs(False)
        time.sleep(0.0005)
        return channel, seq, pkt_len, payload

    def send_packet(channel, payload):
        pkt_len = len(payload) + 4
        full_pkt = [pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, channel, 0] + list(payload)
        set_cs(True)
        start_t = time.time()
        while not is_int() and (time.time() - start_t < 0.1):
            time.sleep(0.0002)
        spi.xfer2(full_pkt)
        set_cs(False)
        time.sleep(0.001)

    # Drain boot packets
    while True:
        pkt = read_packet(timeout=0.08)
        if not pkt:
            break

    # Enable Rotation Vector (0x05) at 50Hz (20ms = 20000us)
    # Hillcrest SH2 Set Feature command format:
    # 0xFD (1B), Feature Report ID (1B), Flags (1B), Change Sensitivity (2B), Report Interval (4B us), Batch Interval (4B), Sensor Specific (4B)
    cmd_rv = bytearray(17)
    cmd_rv[0] = 0xFD
    cmd_rv[1] = 0x05 # Rotation Vector
    struct.pack_into("<I", cmd_rv, 5, 20000)
    send_packet(2, cmd_rv)
    time.sleep(0.01)

    # Enable Accelerometer (0x01)
    cmd_acc = bytearray(17)
    cmd_acc[0] = 0xFD
    cmd_acc[1] = 0x01
    struct.pack_into("<I", cmd_acc, 5, 20000)
    send_packet(2, cmd_acc)
    time.sleep(0.01)

    print("Listening for Sensor Reports...")
    count = 0
    start = time.time()
    while count < 20 and (time.time() - start < 4.0):
        pkt = read_packet(timeout=0.1)
        if pkt:
            chan, seq, plen, pdata = pkt
            print(f"Chan: {chan} | Len: {plen} | Seq: {seq} | Raw: {[hex(x) for x in pdata]}")
            count += 1

    spi.close()

if __name__ == "__main__":
    inspect_channel3()
