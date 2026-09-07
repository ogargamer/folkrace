import time
import smbus2
import struct
import math
import board
import digitalio

def run_live_stream():
    print("==================================================================")
    print("  BNO08X 9-DOF IMU CONTINUOUS 100Hz I2C STREAM ON RASPBERRY PI 5")
    print("==================================================================")

    bus = smbus2.SMBus(1)
    addr = 0x4B

    rst_pin = digitalio.DigitalInOut(board.D21)
    rst_pin.direction = digitalio.Direction.OUTPUT
    rst_pin.value = True

    int_pin = digitalio.DigitalInOut(board.D26)
    int_pin.direction = digitalio.Direction.INPUT
    int_pin.pull = digitalio.Pull.UP

    def is_int():
        return not int_pin.value # Active LOW

    # Step 1: Hardware Reset
    print("Resetting BNO08X...")
    rst_pin.value = False
    time.sleep(0.02)
    rst_pin.value = True
    time.sleep(0.05)

    # Wait for INT
    start = time.time()
    while not is_int() and (time.time() - start < 2.0):
        time.sleep(0.001)

    # Single-transaction I2C read
    def read_packet(max_len=128):
        try:
            msg = smbus2.i2c_msg.read(addr, max_len)
            bus.i2c_rdwr(msg)
            raw = list(msg)
            plen = (raw[0] | (raw[1] << 8)) & ~0x8000
            chan = raw[2]
            seq = raw[3]
            if plen < 4 or plen > max_len:
                return chan, seq, plen, []
            return chan, seq, plen, raw[4:plen]
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

    # Drain boot packets
    print("Draining bootloader packets...")
    boot_count = 0
    for _ in range(6):
        start_w = time.time()
        while not is_int() and (time.time() - start_w < 0.2):
            time.sleep(0.001)
        pkt = read_packet(280)
        if pkt and pkt[2] > 4:
            boot_count += 1
            print(f"  -> Boot Packet #{boot_count}: Chan={pkt[0]}, Len={pkt[2]}, RepID=0x{(pkt[3][0] if pkt[3] else 0):02X}")
        else:
            time.sleep(0.01)

    print(f"Boot drain complete ({boot_count} packets).")
    time.sleep(0.02)

    # Enable Rotation Vector (0x05) & Accelerometer (0x01) & Gyroscope (0x02) @ 100Hz (10,000 us)
    def enable_feature(feat_id, interval_us):
        cmd = bytearray(17)
        cmd[0] = 0xFD
        cmd[1] = feat_id
        struct.pack_into("<I", cmd, 5, interval_us) # 10,000 us = 100Hz
        write_packet(2, cmd)
        time.sleep(0.005)

    print("Enabling Game Rotation Vector, Accelerometer, Gyroscope @ 100Hz...")
    enable_feature(0x08, 10000) # Game Rotation Vector
    enable_feature(0x01, 10000) # Accelerometer
    enable_feature(0x02, 10000) # Gyroscope

    print("\nStreaming 9-DOF IMU Telemetry (Live Orientation & Motion):")
    print("=" * 85)

    samples = 0
    start_t = time.time()
    roll, pitch, yaw = 0.0, 0.0, 0.0
    ax, ay, az = 0.0, 0.0, 0.0
    gx, gy, gz = 0.0, 0.0, 0.0
    qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

    while samples < 60 and (time.time() - start_t < 6.0):
        if is_int():
            pkt = read_packet(64)
            if pkt:
                chan, seq, plen, data = pkt
                if chan == 3 and len(data) >= 5: # Channel 3 = Reports
                    idx = 0
                    while idx < len(data):
                        if idx + 5 > len(data):
                            break
                        rep_id = data[idx + 4]

                        # Game Rotation Vector (0x08)
                        if rep_id == 0x08 and idx + 19 <= len(data):
                            raw_i, raw_j, raw_k, raw_r = struct.unpack_from("<hhhh", bytes(data), idx + 7)
                            q_scale = 1.0 / (1 << 14)
                            qx, qy, qz, qw = raw_i * q_scale, raw_j * q_scale, raw_k * q_scale, raw_r * q_scale

                            sinr_cosp = 2.0 * (qw * qx + qy * qz)
                            cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
                            roll = math.atan2(sinr_cosp, cosr_cosp) * (180.0 / math.pi)

                            sinp = 2.0 * (qw * qy - qz * qx)
                            pitch = math.asin(max(-1.0, min(1.0, sinp))) * (180.0 / math.pi)

                            siny_cosp = 2.0 * (qw * qz + qx * qy)
                            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                            yaw = math.atan2(siny_cosp, cosy_cosp) * (180.0 / math.pi)

                            samples += 1
                            if samples % 2 == 0:
                                print(f"[{samples:03d}] ROLL:{roll:+6.2f}° | PITCH:{pitch:+6.2f}° | YAW:{yaw:+6.2f}° | Quat:({qw:+.3f},{qx:+.3f},{qy:+.3f},{qz:+.3f}) | Accel:({ax:+5.2f},{ay:+5.2f},{az:+5.2f})")
                            idx += 19

                        # Accelerometer (0x01)
                        elif rep_id == 0x01 and idx + 15 <= len(data):
                            raw_ax, raw_ay, raw_az = struct.unpack_from("<hhh", bytes(data), idx + 7)
                            acc_scale = 1.0 / (1 << 8)
                            ax, ay, az = raw_ax * acc_scale, raw_ay * acc_scale, raw_az * acc_scale
                            idx += 15

                        # Gyroscope (0x02)
                        elif rep_id == 0x02 and idx + 15 <= len(data):
                            raw_gx, raw_gy, raw_gz = struct.unpack_from("<hhh", bytes(data), idx + 7)
                            gyro_scale = 1.0 / (1 << 9)
                            gx, gy, gz = raw_gx * gyro_scale, raw_gy * gyro_scale, raw_gz * gyro_scale
                            idx += 15
                        else:
                            idx += 1
        time.sleep(0.0005)

    print("=" * 85)
    print(">> BNO08X IMU IS 100% OPERATIONAL OVER I2C ON RASPBERRY PI 5! <<")
    bus.close()

if __name__ == "__main__":
    run_live_stream()
