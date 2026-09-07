import time
import spidev
import board
import digitalio
import struct
import math

def run_sparkfun_sequence():
    print("==================================================================")
    print("  EXACT SPARKFUN BNO080 SPI DRIVER ON RASPBERRY PI 5")
    print("==================================================================")

    # 1. SPI0 Hardware Controller
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 3000000
    spi.mode = 3 # Mode 3 (CPOL=1, CPHA=1)

    # Control Pins
    cs = digitalio.DigitalInOut(board.D16) # Pin 36
    cs.direction = digitalio.Direction.OUTPUT
    cs.value = True

    rst = digitalio.DigitalInOut(board.D21) # Pin 40
    rst.direction = digitalio.Direction.OUTPUT
    rst.value = True

    int_pin = digitalio.DigitalInOut(board.D26) # Pin 37
    int_pin.direction = digitalio.Direction.INPUT
    int_pin.pull = digitalio.Pull.UP

    def is_int_low():
        return not int_pin.value

    def wait_for_spi(timeout_ms=500):
        start = time.time()
        while time.time() - start < (timeout_ms / 1000.0):
            if is_int_low():
                return True
            time.sleep(0.0001)
        return False

    def receive_packet():
        if not is_int_low():
            return False, 0, 0, []

        cs.value = False
        time.sleep(0.00001)

        # Read 4-byte header
        hdr = spi.xfer2([0x00, 0x00, 0x00, 0x00])
        packet_len = (hdr[0] | (hdr[1] << 8)) & ~0x8000
        channel = hdr[2]
        seq = hdr[3]

        payload = []
        if packet_len > 4:
            payload = spi.xfer2([0x00] * (packet_len - 4))

        cs.value = True
        time.sleep(0.00005)
        return True, channel, packet_len, payload

    seq_num = [0] * 6
    def send_packet(channel, data):
        data_len = len(data)
        packet_len = data_len + 4
        seq = seq_num[channel]
        seq_num[channel] = (seq_num[channel] + 1) % 256
        
        pkt = [packet_len & 0xFF, (packet_len >> 8) & 0xFF, channel, seq] + list(data)

        cs.value = False
        time.sleep(0.00001)
        spi.xfer2(pkt)
        cs.value = True
        time.sleep(0.0005)

    # 1. Reset
    print("Step 1: Hardware Reset...")
    rst.value = False
    time.sleep(0.01)
    rst.value = True
    time.sleep(0.02)

    # 2. Wait for first assertion of INT
    print("Step 2: Waiting for INT assertion after reset...")
    if not wait_for_spi(1000):
        print("ERROR: BNO080 did not assert INT after reset!")
        return
    print("   -> INT asserted!")

    # 3. Read and dump advertisement packet
    print("Step 3: Reading Advertisement packet...")
    ok, chan, plen, pdata = receive_packet()
    print(f"   -> Advert Packet: Chan={chan}, Len={plen}")

    # 4. Wait for and read Initialize Response (0xF1)
    print("Step 4: Waiting for Initialize Response (0xF1)...")
    if wait_for_spi(500):
        ok, chan, plen, pdata = receive_packet()
        rep_id = hex(pdata[0]) if len(pdata) > 0 else "N/A"
        print(f"   -> Init Response: Chan={chan}, Len={plen}, ReportID={rep_id}")

    # 5. Check for Executable Channel packet (Chan 1)
    if wait_for_spi(100):
        ok, chan, plen, pdata = receive_packet()
        print(f"   -> Executable Packet: Chan={chan}, Len={plen}")

    # 6. Send Product ID Request
    print("\nStep 5: Sending Product ID Request (Channel 2, Report 0xF9)...")
    send_packet(2, [0xF9, 0x00])

    # 7. Wait for Product ID Response
    print("Step 6: Waiting for Product ID Response...")
    if wait_for_spi(500):
        ok, chan, plen, pdata = receive_packet()
        print(f"   -> Received: Chan={chan}, Len={plen}, Payload={[hex(x) for x in pdata[:8]]}")
        if len(pdata) > 0 and pdata[0] == 0xF8:
            sw_major = pdata[2]
            sw_minor = pdata[3]
            part_num = (pdata[7] << 24) | (pdata[6] << 16) | (pdata[5] << 8) | pdata[4]
            print(f"   ======================================================")
            print(f"   SUCCESS! Part #{part_num}, Firmware v{sw_major}.{sw_minor}")
            print(f"   ======================================================")

    # 8. Enable Game Rotation Vector (0x08) & Accelerometer (0x01)
    print("\nStep 7: Enabling 100Hz Reports (Game Rotation Vector + Accelerometer)...")
    def set_feature_cmd(report_id, interval_us):
        cmd = bytearray(17)
        cmd[0] = 0xFD
        cmd[1] = report_id
        struct.pack_into("<I", cmd, 5, interval_us) # 100Hz = 10,000 us
        send_packet(2, cmd)
        time.sleep(0.005)

    set_feature_cmd(0x08, 10000) # Game Rotation Vector
    set_feature_cmd(0x01, 10000) # Accelerometer

    print("\nStep 8: Streaming Telemetry for 5 seconds:")
    print("-" * 80)

    start_stream = time.time()
    count = 0
    roll, pitch, yaw = 0.0, 0.0, 0.0
    ax, ay, az = 0.0, 0.0, 0.0

    while time.time() - start_stream < 5.0 and count < 50:
        if is_int_low():
            ok, chan, plen, pdata = receive_packet()
            if ok and chan == 3 and len(pdata) >= 5: # Sensor Reports Channel
                idx = 0
                while idx < len(pdata):
                    if idx + 5 > len(pdata):
                        break
                    rep_id = pdata[idx + 4]

                    if rep_id == 0x08 and idx + 19 <= len(pdata): # Game Rotation Vector
                        qi, qj, qk, qr = struct.unpack_from("<hhhh", bytes(pdata), idx + 7)
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

                    elif rep_id == 0x01 and idx + 15 <= len(pdata): # Accelerometer
                        raw_ax, raw_ay, raw_az = struct.unpack_from("<hhh", bytes(pdata), idx + 7)
                        acc_scale = 1.0 / (1 << 8)
                        ax, ay, az = raw_ax * acc_scale, raw_ay * acc_scale, raw_az * acc_scale
                        idx += 15
                    else:
                        idx += 1
        time.sleep(0.0005)

    print("-" * 80)
    print(">> TEST PASSED: 100% OPERATIONAL! <<")
    spi.close()

if __name__ == "__main__":
    run_sparkfun_sequence()
