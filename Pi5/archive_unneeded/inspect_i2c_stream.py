import time
import smbus2
import struct
import math
import board
import digitalio

def inspect_stream():
    bus = smbus2.SMBus(1)
    addr = 0x4B

    rst_pin = digitalio.DigitalInOut(board.D21)
    rst_pin.direction = digitalio.Direction.OUTPUT
    rst_pin.value = True

    int_pin = digitalio.DigitalInOut(board.D26)
    int_pin.direction = digitalio.Direction.INPUT
    int_pin.pull = digitalio.Pull.UP

    def is_int():
        return not int_pin.value

    # Reset
    rst_pin.value = False
    time.sleep(0.02)
    rst_pin.value = True
    time.sleep(0.05)

    start = time.time()
    while not is_int() and (time.time() - start < 2.0):
        time.sleep(0.001)

    def read_packet():
        try:
            msg_hdr = smbus2.i2c_msg.read(addr, 4)
            bus.i2c_rdwr(msg_hdr)
            hdr = list(msg_hdr)
            plen = (hdr[0] | (hdr[1] << 8)) & ~0x8000
            chan = hdr[2]
            seq = hdr[3]

            payload = []
            if 4 < plen < 1024:
                msg_data = smbus2.i2c_msg.read(addr, plen)
                bus.i2c_rdwr(msg_data)
                full = list(msg_data)
                payload = full[4:]
            return chan, seq, plen, payload
        except Exception:
            return None

    seq_tx = [0] * 6
    def write_packet(channel, payload):
        pkt_len = len(payload) + 4
        seq = seq_tx[channel]
        seq_tx[channel] = (seq_tx[channel] + 1) % 256
        full_pkt = [pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, channel, seq] + list(payload)
        msg = smbus2.i2c_msg.write(addr, full_pkt)
        bus.i2c_rdwr(msg)
        time.sleep(0.001)

    # Drain boot
    while is_int():
        read_packet()
        time.sleep(0.005)

    # Enable Rotation Vector (0x05) @ 50Hz (20,000 us = 0x4E20)
    cmd = bytearray(17)
    cmd[0] = 0xFD
    cmd[1] = 0x05 # Rotation Vector
    struct.pack_into("<I", cmd, 5, 20000)
    write_packet(2, cmd)
    time.sleep(0.01)

    print("Reading Channel 3 Packets:")
    for _ in range(10):
        start = time.time()
        while not is_int() and (time.time() - start < 0.5):
            time.sleep(0.001)
        pkt = read_packet()
        if pkt:
            c, s, l, p = pkt
            print(f"Chan: {c} | Len: {l} | Seq: {s} | Raw: {[hex(x) for x in p]}")
            if c == 3 and len(p) >= 19:
                # SHTP Input report structure:
                # p[0:4] = timestamp
                # p[4] = report_id (0x05 or 0xFB)
                print("  --> Decoded Report ID:", hex(p[4]) if len(p) > 4 else "N/A")

    bus.close()

if __name__ == "__main__":
    inspect_stream()
