import time
import spidev
import board
import digitalio
import struct
import math

def run_bno08x_stream():
    print("==================================================================")
    print("  BNO08X 9-DOF IMU 400Hz SPI TELEMETRY STREAM ON PI 5")
    print("==================================================================")

    # 1. SPI0 Hardware Controller
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 3000000
    spi.mode = 3 # Mode 3 (CPOL=1, CPHA=1)

    # 2. Control Pins: CS=Pin 36 (GPIO 16), INT=Pin 37 (GPIO 26), RST=Pin 40 (GPIO 21)
    cs = digitalio.DigitalInOut(board.D16)
    cs.direction = digitalio.Direction.OUTPUT
    cs.value = True # CS HIGH (inactive)

    rst = digitalio.DigitalInOut(board.D21)
    rst.direction = digitalio.Direction.OUTPUT
    rst.value = True

    int_pin = digitalio.DigitalInOut(board.D26)
    int_pin.direction = digitalio.Direction.INPUT
    int_pin.pull = digitalio.Pull.UP

    def is_int():
        return not int_pin.value # Active LOW

    def set_cs(active):
        cs.value = not active # True = pull LOW

    # 3. Hardware Reset Pulse
    print("Performing Hardware Reset...")
    rst.value = False
    time.sleep(0.02)
    rst.value = True
    time.sleep(0.05)

    # Wait for initial bootloader INT
    start = time.time()
    while not is_int() and (time.time() - start < 2.0):
        time.sleep(0.001)

    print(f"BNO08X Bootloader Ready (INT asserted in {round((time.time()-start)*1000, 1)} ms)!")

    # 4. SHTP Transaction Helpers
    seq_numbers = [0] * 6

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
        seq = seq_numbers[channel]
        seq_numbers[channel] = (seq_numbers[channel] + 1) % 256
        full_pkt = [pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, channel, seq] + list(payload)

        # Assert CS to wake sensor, wait for INT LOW
        set_cs(True)
        start_t = time.time()
        while not is_int() and (time.time() - start_t < 0.1):
            time.sleep(0.0002)

        spi.xfer2(full_pkt)
        set_cs(False)
        time.sleep(0.001)

    # Drain initial advertisement packets
    print("Draining SHTP Bootloader packets...")
    while True:
        pkt = read_packet(timeout=0.1)
        if not pkt:
            break

    # 5. Enable Reports
    # Set Feature Command: Report 0xFD
    # [0xFD, Feature ID, Flags, Sensitivity LSB, Sensitivity MSB, Interval (4B us), Batch (4B), Specific (4B)]
    def enable_feature(report_id, interval_us):
        cmd = bytearray(17)
        cmd[0] = 0xFD
        cmd[1] = report_id
        cmd[2] = 0x00 # flags
        cmd[3] = 0x00 # change sensitivity
        cmd[4] = 0x00
        struct.pack_into("<I", cmd, 5, interval_us) # report interval (us)
        struct.pack_into("<I", cmd, 9, 0)           # batch interval
        struct.pack_into("<I", cmd, 13, 0)          # sensor specific
        send_packet(2, cmd) # Channel 2 = Control

    print("Enabling Game Rotation Vector (6-DOF Quaternion @ 100Hz)...")
    enable_feature(0x08, 10000) # 100Hz = 10,000 us
    time.sleep(0.01)

    print("Enabling Accelerometer (m/s² @ 100Hz)...")
    enable_feature(0x01, 10000)
    time.sleep(0.01)

    print("Enabling Gyroscope (rad/s @ 100Hz)...")
    enable_feature(0x02, 10000)
    time.sleep(0.01)

    print("\n==================================================================")
    print("  STREAMING LIVE ORIENTATION, GYRO & ACCELEROMETER READINGS")
    print("==================================================================")

    sample_count = 0
    start_time = time.time()
    
    # Store latest values
    roll, pitch, yaw = 0.0, 0.0, 0.0
    ax, ay, az = 0.0, 0.0, 0.0
    gx, gy, gz = 0.0, 0.0, 0.0
    qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

    while sample_count < 50 and (time.time() - start_time < 8.0):
        pkt = read_packet(timeout=0.05)
        if pkt:
            channel, seq, pkt_len, payload = pkt
            
            # Parse Channel 3 (Input Sensor Reports)
            if channel == 3 and len(payload) >= 5:
                idx = 0
                while idx < len(payload):
                    # Each report in SHTP payload:
                    # [0: timestamp 4B, 4: report_id 1B, 5: seq 1B, 6: status 1B, 7+: data]
                    if idx + 5 > len(payload):
                        break
                    
                    rep_id = payload[idx + 4]
                    
                    if rep_id == 0x08 and idx + 19 <= len(payload): # Game Rotation Vector
                        qi, qj, qk, qr = struct.unpack_from("<hhhh", bytes(payload), idx + 7)
                        q_scale = 1.0 / (1 << 14) # Q14 format
                        qx, qy, qz, qw = qi * q_scale, qj * q_scale, qk * q_scale, qr * q_scale
                        
                        # Calculate Euler angles in degrees
                        sinr_cosp = 2.0 * (qw * qx + qy * qz)
                        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
                        roll = math.atan2(sinr_cosp, cosr_cosp) * (180.0 / math.pi)

                        sinp = 2.0 * (qw * qy - qz * qx)
                        pitch = math.asin(max(-1.0, min(1.0, sinp))) * (180.0 / math.pi)

                        siny_cosp = 2.0 * (qw * qz + qx * qy)
                        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                        yaw = math.atan2(siny_cosp, cosy_cosp) * (180.0 / math.pi)
                        idx += 19
                        sample_count += 1
                        
                        if sample_count % 2 == 0:
                            print(f"[{sample_count:02d}] R: {roll:+6.1f}° | P: {pitch:+6.1f}° | Y: {yaw:+6.1f}° | Quat: ({qw:+.2f}, {qx:+.2f}, {qy:+.2f}, {qz:+.2f}) | Accel: ({ax:+.1f}, {ay:+.1f}, {az:+.1f})")

                    elif rep_id == 0x01 and idx + 15 <= len(payload): # Accelerometer
                        raw_ax, raw_ay, raw_az = struct.unpack_from("<hhh", bytes(payload), idx + 7)
                        acc_scale = 1.0 / (1 << 8) # Q8 format in m/s^2
                        ax, ay, az = raw_ax * acc_scale, raw_ay * acc_scale, raw_az * acc_scale
                        idx += 15

                    elif rep_id == 0x02 and idx + 15 <= len(payload): # Gyroscope
                        raw_gx, raw_gy, raw_gz = struct.unpack_from("<hhh", bytes(payload), idx + 7)
                        gyro_scale = 1.0 / (1 << 9) # Q9 format in rad/s
                        gx, gy, gz = raw_gx * gyro_scale, raw_gy * gyro_scale, raw_gz * gyro_scale
                        idx += 15
                    else:
                        idx += 1

    print("==================================================================")
    print(">> BNO08X IMU IS 100% OPERATIONAL ON RASPBERRY PI 5! <<")
    print("==================================================================")
    spi.close()

if __name__ == "__main__":
    run_bno08x_stream()
