import time
import spidev
import board
import digitalio
import struct
import math

def test_working_spi():
    print("==================================================================")
    print("  BNO08X IMU WORKING CONTINUOUS SPI TEST (RASPBERRY PI 5)")
    print("==================================================================")

    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 3000000
    spi.mode = 3 # SPI Mode 3

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
    print("1. Resetting BNO08X...")
    rst.value = False
    time.sleep(0.02)
    rst.value = True
    time.sleep(0.05)

    start = time.time()
    while not is_int() and (time.time() - start < 2.0):
        time.sleep(0.001)
    print(f"   -> INT asserted in {round((time.time()-start)*1000, 1)} ms")

    # Read Packet (CS held LOW across header and payload)
    def read_shtp():
        if not is_int():
            return None

        cs.value = False
        time.sleep(0.00001)

        # Read 4-byte header
        hdr = spi.xfer2([0x00, 0x00, 0x00, 0x00])
        raw_len = hdr[0] | (hdr[1] << 8)
        pkt_len = raw_len & ~0x8000
        chan = hdr[2]
        seq = hdr[3]

        payload = []
        if 4 < pkt_len < 1024:
            # CS is STILL LOW -> read payload immediately
            payload = spi.xfer2([0x00] * (pkt_len - 4))

        # Deassert CS after full transfer
        cs.value = True
        time.sleep(0.0001)
        return chan, seq, pkt_len, payload

    seq_tx = [0] * 6
    def write_shtp(chan, payload):
        # Drain reads
        while is_int():
            read_shtp()

        pkt_len = len(payload) + 4
        seq = seq_tx[chan]
        seq_tx[chan] = (seq_tx[chan] + 1) % 256
        full_pkt = [pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, chan, seq] + list(payload)

        cs.value = False
        time.sleep(0.00001)
        spi.xfer2(full_pkt)
        cs.value = True
        time.sleep(0.001)

    # 2. Drain boot packets
    print("2. Draining Startup Packets...")
    while True:
        pkt = read_shtp()
        if not pkt:
            time.sleep(0.05)
            if not is_int():
                break
        else:
            print(f"   -> Boot Packet: Chan={pkt[0]}, Len={pkt[2]}, Seq={pkt[1]}")

    # 3. Request Product ID
    print("3. Requesting Product ID (Report 0xF9 on Channel 2)...")
    write_shtp(2, [0xF9, 0x00])

    for _ in range(10):
        start_w = time.time()
        while not is_int() and (time.time() - start_w < 0.2):
            time.sleep(0.0005)
        pkt = read_shtp()
        if pkt:
            c, s, l, p = pkt
            print(f"   -> Response: Chan={c}, Len={l}, Seq={s}, Data={[hex(x) for x in p[:8]]}")
            if len(p) > 0 and p[0] == 0xF8:
                print(f"   ===> PRODUCT ID CONFIRMED! Firmware v{p[2]}.{p[3]}")
                break

    # 4. Enable Game Rotation Vector @ 100Hz (10,000 us) & Accelerometer @ 100Hz
    print("\n4. Enabling 100Hz Game Rotation Vector & Accelerometer...")
    def enable_report(rep_id, interval_us):
        cmd = bytearray(17)
        cmd[0] = 0xFD
        cmd[1] = rep_id
        struct.pack_into("<I", cmd, 5, interval_us)
        write_shtp(2, cmd)
        time.sleep(0.01)

    enable_report(0x08, 10000) # Game Rotation Vector
    enable_report(0x01, 10000) # Accelerometer

    print("\n5. Streaming Live 9-DOF Orientation & Motion Telemetry:")
    print("-" * 80)

    count = 0
    start_t = time.time()
    roll, pitch, yaw = 0.0, 0.0, 0.0
    ax, ay, az = 0.0, 0.0, 0.0

    while count < 40 and (time.time() - start_t < 6.0):
        if is_int():
            pkt = read_shtp()
            if pkt:
                c, s, l, p = pkt
                if c == 3 and len(p) >= 14: # Channel 3 = Reports
                    idx = 0
                    while idx < len(p):
                        if idx + 5 > len(p):
                            break
                        rep_id = p[idx + 4]
                        
                        if rep_id == 0x08 and idx + 19 <= len(p): # Game Rotation Vector
                            qi, qj, qk, qr = struct.unpack_from("<hhhh", bytes(p), idx + 7)
                            q_scale = 1.0 / (1 << 14)
                            qx, qy, qz, qw = qi * q_scale, qj * q_scale, qk * q_scale, qr * q_scale

                            sinr_cosp = 2.0 * (qw * qx + qy * qz)
                            cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
                            roll = math.atan2(sinr_cosp, cosr_cosp) * (180.0 / math.pi)

                            sinp = 2.0 * (qw * qy - qz * qx)
                            pitch = math.asin(max(-1.0, min(1.0, sinp))) * (180.0 / math.pi)

                            siny_cosp = 2.0 * (qw * qz + qx * qy)
                            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                            yaw = math.atan2(siny_cosp, cosy_cosp) * (180.0 / math.pi)

                            count += 1
                            print(f"[{count:03d}] ROLL:{roll:+6.2f}° | PITCH:{pitch:+6.2f}° | YAW:{yaw:+6.2f}° | Quat: ({qw:+.3f}, {qx:+.3f}, {qy:+.3f}, {qz:+.3f}) | Accel: ({ax:+5.2f}, {ay:+5.2f}, {az:+5.2f})")
                            idx += 19

                        elif rep_id == 0x01 and idx + 15 <= len(p): # Accelerometer
                            raw_ax, raw_ay, raw_az = struct.unpack_from("<hhh", bytes(p), idx + 7)
                            acc_scale = 1.0 / (1 << 8)
                            ax, ay, az = raw_ax * acc_scale, raw_ay * acc_scale, raw_az * acc_scale
                            idx += 15
                        else:
                            idx += 1
        time.sleep(0.0005)

    print("-" * 80)
    print(">> TEST COMPLETE! <<")
    spi.close()

if __name__ == "__main__":
    test_working_spi()
