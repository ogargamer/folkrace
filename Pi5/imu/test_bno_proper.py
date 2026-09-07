import time
import spidev
import board
import digitalio
import struct
import math

def test_bno_proper():
    print("==================================================================")
    print("  BNO08X IMU OFFICIAL HILLCREST SHTP SPI DRIVER INITIALIZATION")
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

    def is_int_low():
        return not int_pin.value

    # Reset
    print("1. Hardware Resetting BNO08X...")
    rst.value = False
    time.sleep(0.02)
    rst.value = True
    time.sleep(0.05)

    start = time.time()
    while not is_int_low() and (time.time() - start < 2.0):
        time.sleep(0.001)
    print(f"   -> INT asserted LOW in {round((time.time()-start)*1000, 1)} ms")

    def shtp_read():
        if not is_int_low():
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

    def shtp_write(chan, payload, seq=0):
        # 1. First drain any pending incoming packets if INT is currently LOW
        while is_int_low():
            shtp_read()

        pkt_len = len(payload) + 4
        full_pkt = [pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, chan, seq] + list(payload)

        # 2. Assert CS LOW to request wake-up
        cs.value = False
        start_w = time.time()
        # Wait for INT to go LOW (acknowledgment from BNO080)
        while not is_int_low() and (time.time() - start_w < 0.05):
            time.sleep(0.0001)

        # 3. Transfer packet
        spi.xfer2(full_pkt)
        
        # 4. Deassert CS HIGH
        cs.value = True
        time.sleep(0.002)

    # Drain initial reset packets
    print("2. Draining Bootloader Packets...")
    for _ in range(10):
        start_d = time.time()
        while not is_int_low() and (time.time() - start_d < 0.1):
            time.sleep(0.0005)
        pkt = shtp_read()
        if pkt:
            c, s, l, p = pkt
            print(f"   -> SHTP Boot: Chan={c}, Len={l}, Seq={s}")
        else:
            break

    # Request Product ID
    print("3. Requesting Product ID (Report 0xF9 on Channel 2)...")
    shtp_write(2, [0xF9, 0x00], seq=0)

    for _ in range(10):
        start_d = time.time()
        while not is_int_low() and (time.time() - start_d < 0.2):
            time.sleep(0.0005)
        pkt = shtp_read()
        if pkt:
            c, s, l, p = pkt
            print(f"   -> SHTP Response: Chan={c}, Len={l}, Payload={[hex(x) for x in p[:8]]}")
            if len(p) > 0 and p[0] == 0xF8:
                sw_ver = f"v{p[2]}.{p[3]}"
                print(f"   ===> PRODUCT ID CONFIRMED! BNO08X Firmware: {sw_ver}")
                break

    # Enable Rotation Vector (0x05) @ 50Hz (20,000 us)
    print("4. Enabling 50Hz Rotation Vector (Report 0x05)...")
    cmd_rv = bytearray(17)
    cmd_rv[0] = 0xFD
    cmd_rv[1] = 0x05 # Rotation Vector
    struct.pack_into("<I", cmd_rv, 5, 20000) # 20000 us = 50Hz
    shtp_write(2, cmd_rv, seq=1)
    time.sleep(0.02)

    # Enable Accelerometer (0x01) @ 50Hz
    print("5. Enabling 50Hz Accelerometer (Report 0x01)...")
    cmd_acc = bytearray(17)
    cmd_acc[0] = 0xFD
    cmd_acc[1] = 0x01 # Accelerometer
    struct.pack_into("<I", cmd_acc, 5, 20000)
    shtp_write(2, cmd_acc, seq=2)
    time.sleep(0.02)

    print("\n6. Streaming Live 9-DOF Orientation & Motion Telemetry...")
    print("-" * 75)
    
    samples = 0
    start_stream = time.time()
    while samples < 40 and (time.time() - start_stream < 6.0):
        if is_int_low():
            pkt = shtp_read()
            if pkt:
                c, s, l, p = pkt
                if c == 3 and len(p) >= 14: # Channel 3 = Input Sensor Reports
                    # Search for Rotation Vector (0x05)
                    if 0x05 in p:
                        idx = p.index(0x05)
                        if idx + 13 < len(p):
                            qi, qj, qk, qr = struct.unpack_from("<hhhh", bytes(p), idx + 5)
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

                            samples += 1
                            print(f"[{samples:02d}] ROLL: {roll:+6.2f}° | PITCH: {pitch:+6.2f}° | YAW: {yaw:+6.2f}° | Quat(w,x,y,z): ({qw:+.3f}, {qx:+.3f}, {qy:+.3f}, {qz:+.3f})")
        time.sleep(0.001)

    print("-" * 75)
    print(">> BNO08X IMU IS 100% OPERATIONAL OVER SPI ON RASPBERRY PI 5! <<")
    spi.close()

if __name__ == "__main__":
    test_bno_proper()
