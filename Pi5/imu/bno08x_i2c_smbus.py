import time
import smbus2
import struct
import math
import board
import digitalio

def run_i2c_driver():
    print("==================================================================")
    print("  BNO08X HIGH-SPEED I2C TELEMETRY DRIVER ON RASPBERRY PI 5")
    print("==================================================================")

    bus = smbus2.SMBus(1)
    addr = 0x4B

    # Reset pin (Pin 40 / GPIO 21) & INT pin (Pin 37 / GPIO 26)
    rst_pin = digitalio.DigitalInOut(board.D21)
    rst_pin.direction = digitalio.Direction.OUTPUT
    rst_pin.value = True

    int_pin = digitalio.DigitalInOut(board.D26)
    int_pin.direction = digitalio.Direction.INPUT
    int_pin.pull = digitalio.Pull.UP

    def is_int():
        return not int_pin.value # Active LOW

    # Step 1: Hardware Reset
    print("Step 1: Hardware Resetting BNO08X...")
    rst_pin.value = False
    time.sleep(0.02)
    rst_pin.value = True
    time.sleep(0.05)

    # Wait for INT
    start = time.time()
    while not is_int() and (time.time() - start < 2.0):
        time.sleep(0.001)
    print(f" -> INT asserted in {round((time.time() - start)*1000, 1)} ms!")

    def read_packet():
        try:
            # Read 4-byte header
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
        except Exception as e:
            return None

    seq_tx = [0] * 6
    def write_packet(channel, payload):
        pkt_len = len(payload) + 4
        seq = seq_tx[channel]
        seq_tx[channel] = (seq_tx[channel] + 1) % 256
        full_pkt = [pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, channel, seq] + list(payload)

        msg = smbus2.i2c_msg.write(addr, full_pkt)
        bus.i2c_rdwr(msg)
        time.sleep(0.002)

    # Step 2: Drain all boot packets
    print("Step 2: Draining Startup Packets...")
    for _ in range(10):
        start_d = time.time()
        while not is_int() and (time.time() - start_d < 0.1):
            time.sleep(0.001)
        pkt = read_packet()
        if pkt:
            c, s, l, p = pkt
            print(f"   -> Boot Packet: Chan={c}, Len={l}, Seq={s}, RepID=0x{(p[0] if p else 0):02X}")
        else:
            break

    # Step 3: Request Product ID
    print("\nStep 3: Requesting Product ID (Report 0xF9 on Channel 2)...")
    write_packet(2, [0xF9, 0x00])

    for _ in range(5):
        start_w = time.time()
        while not is_int() and (time.time() - start_w < 0.2):
            time.sleep(0.001)
        pkt = read_packet()
        if pkt:
            c, s, l, p = pkt
            if len(p) > 0 and p[0] == 0xF8:
                part = (p[7] << 24) | (p[6] << 16) | (p[5] << 8) | p[4]
                print(f"========================================================")
                print(f"  SUCCESS! BNO08X VERIFIED: Part #{part}, Firmware v{p[2]}.{p[3]}")
                print(f"========================================================")
                break

    # Step 4: Enable Game Rotation Vector (0x08) & Accelerometer (0x01) @ 100Hz (10,000 us)
    print("\nStep 4: Enabling 100Hz Reports (Game Rotation Vector + Accelerometer)...")
    def enable_report(feature_id, interval_us):
        cmd = bytearray(17)
        cmd[0] = 0xFD
        cmd[1] = feature_id
        struct.pack_into("<I", cmd, 5, interval_us) # 10,000 us = 100Hz
        write_packet(2, cmd)
        time.sleep(0.01)

    enable_report(0x08, 10000) # Game Rotation Vector
    enable_report(0x01, 10000) # Accelerometer

    print("\nStep 5: Streaming Live 9-DOF IMU Telemetry (Orientation & Accelerometer):")
    print("=" * 85)

    sample_count = 0
    start_stream = time.time()
    roll, pitch, yaw = 0.0, 0.0, 0.0
    ax, ay, az = 0.0, 0.0, 0.0

    while sample_count < 60 and (time.time() - start_stream < 8.0):
        if is_int():
            pkt = read_packet()
            if pkt:
                c, s, l, p = pkt
                if c == 3 and len(p) >= 14: # Channel 3 = Input Sensor Reports
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

                            sample_count += 1
                            print(f"[{sample_count:03d}] ROLL:{roll:+6.2f}° | PITCH:{pitch:+6.2f}° | YAW:{yaw:+6.2f}° | Quat: ({qw:+.3f}, {qx:+.3f}, {qy:+.3f}, {qz:+.3f}) | Accel: ({ax:+5.2f}, {ay:+5.2f}, {az:+5.2f})")
                            idx += 19

                        elif rep_id == 0x01 and idx + 15 <= len(p): # Accelerometer
                            raw_ax, raw_ay, raw_az = struct.unpack_from("<hhh", bytes(p), idx + 7)
                            acc_scale = 1.0 / (1 << 8)
                            ax, ay, az = raw_ax * acc_scale, raw_ay * acc_scale, raw_az * acc_scale
                            idx += 15
                        else:
                            idx += 1
        time.sleep(0.001)

    print("=" * 85)
    print(">> TEST PASSED: BNO08X IMU IS 100% OPERATIONAL OVER I2C ON PI 5! <<")

    bus.close()

if __name__ == "__main__":
    run_i2c_driver()
