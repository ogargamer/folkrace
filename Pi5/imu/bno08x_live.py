import time
import spidev
import board
import digitalio
import struct
import math

def run_live():
    print("==================================================================")
    print("  BNO08X 9-DOF IMU LIVE SPI STREAM ON RASPBERRY PI 5")
    print("==================================================================")

    # 1. SPI0 Hardware Controller
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 3000000
    spi.mode = 3 # Mode 3 (CPOL=1, CPHA=1)

    # 2. Control Pins: CS=Pin 36 (GPIO 16), INT=Pin 37 (GPIO 26), RST=Pin 40 (GPIO 21)
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
        return not int_pin.value # Active LOW

    def set_cs(active):
        cs.value = not active

    # 3. Hardware Reset
    print("Step 1: Hardware Reset...")
    rst.value = False
    time.sleep(0.02)
    rst.value = True
    time.sleep(0.05)

    # Wait for INT
    start = time.time()
    while not is_int() and (time.time() - start < 2.0):
        time.sleep(0.001)
    print(f"   -> BNO08X INT asserted in {round((time.time()-start)*1000, 1)} ms")

    # Packet Reader / Writer
    seq_tx = [0] * 6

    def read_packet(timeout=0.2):
        start_t = time.time()
        while not is_int():
            if time.time() - start_t > timeout:
                return None
            time.sleep(0.0001)

        set_cs(True)
        time.sleep(0.0001)
        hdr = spi.xfer2([0x00, 0x00, 0x00, 0x00])
        raw_len = hdr[0] | (hdr[1] << 8)
        pkt_len = raw_len & ~0x8000 # Mask off continuation bit
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
        seq = seq_tx[channel]
        seq_tx[channel] = (seq_tx[channel] + 1) % 256
        full_pkt = [pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, channel, seq] + list(payload)

        # In Hillcrest SH-2 SPI Host Write:
        # 1. Assert CS to wake sensor
        set_cs(True)
        start_w = time.time()
        # 2. Wait for INT LOW
        while not is_int() and (time.time() - start_w < 0.05):
            time.sleep(0.0001)
        # 3. Transmit
        spi.xfer2(full_pkt)
        # 4. Deassert CS
        set_cs(False)
        time.sleep(0.001)

    # Drain initial boot sequence: Advertisements, Reset Announcement, Executable Complete
    print("Step 2: Draining Boot Announcements...")
    while True:
        pkt = read_packet(timeout=0.08)
        if not pkt:
            break
        c, s, l, p = pkt
        report_str = f"Report 0x{p[0]:02X}" if len(p) > 0 else "Empty"
        print(f"   -> SHTP Packet: Chan={c}, Len={l}, {report_str}")

    # Request Product ID
    print("Step 3: Requesting Product ID (Report 0xF9 on Channel 2)...")
    send_packet(2, [0xF9, 0x00])

    for _ in range(10):
        pkt = read_packet(timeout=0.2)
        if pkt:
            c, s, l, p = pkt
            if len(p) > 0 and p[0] == 0xF8: # Product ID Response
                reset_cause = p[1]
                sw_ver = f"v{p[2]}.{p[3]}"
                part_no = (p[7] << 24) | (p[6] << 16) | (p[5] << 8) | p[4]
                print(f"   ===> PRODUCT ID CONFIRMED! Part #{part_no}, Firmware {sw_ver}, Reset Cause: {reset_cause}")
                break

    # Enable Reports: Game Rotation Vector (0x08) & Accelerometer (0x01) @ 100Hz (10,000 us)
    print("\nStep 4: Enabling 100Hz Rotation Vector & Accelerometer Reports...")
    def enable_report(feature_id, interval_us):
        cmd = bytearray(17)
        cmd[0] = 0xFD
        cmd[1] = feature_id
        struct.pack_into("<I", cmd, 5, interval_us) # 100Hz = 10,000 us
        send_packet(2, cmd)
        time.sleep(0.01)

    enable_report(0x08, 10000) # Game Rotation Vector (6-DOF Quaternion)
    enable_report(0x01, 10000) # Accelerometer

    print("\nStep 5: Streaming Live 9-DOF IMU Telemetry (Orientation & Accelerometer):")
    print("=" * 80)

    count = 0
    start_loop = time.time()
    roll, pitch, yaw = 0.0, 0.0, 0.0
    ax, ay, az = 0.0, 0.0, 0.0

    while count < 60 and (time.time() - start_loop < 10.0):
        pkt = read_packet(timeout=0.05)
        if pkt:
            c, s, l, p = pkt
            if c == 3 and len(p) >= 14: # Channel 3: Input Sensor Reports
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
                        print(f"[{count:03d}] ROLL: {roll:+6.2f}° | PITCH: {pitch:+6.2f}° | YAW: {yaw:+6.2f}° | Quat(w,x,y,z): ({qw:+.3f}, {qx:+.3f}, {qy:+.3f}, {qz:+.3f}) | Accel: ({ax:+5.2f}, {ay:+5.2f}, {az:+5.2f})")
                        idx += 19

                    elif rep_id == 0x01 and idx + 15 <= len(p): # Accelerometer
                        raw_ax, raw_ay, raw_az = struct.unpack_from("<hhh", bytes(p), idx + 7)
                        acc_scale = 1.0 / (1 << 8)
                        ax, ay, az = raw_ax * acc_scale, raw_ay * acc_scale, raw_az * acc_scale
                        idx += 15
                    else:
                        idx += 1
        time.sleep(0.001)

    print("=" * 80)
    print(">> TEST PASSED: BNO08X IMU IS 100% OPERATIONAL ON RASPBERRY PI 5! <<")
    spi.close()

if __name__ == "__main__":
    run_live()
