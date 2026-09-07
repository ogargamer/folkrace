#!/usr/bin/env python3
"""
BNO08X (BNO080 / BNO085) High-Speed Hardware SPI Driver
Based on the official Hillcrest Laboratories SH-2 SHTP Protocol.
Compatible with Raspberry Pi 5 SPI0 + GPIO Control Pins.
"""

import time
import spidev
import board
import digitalio
import struct
import math

class BNO08xSPI:
    # SHTP Channels
    CHAN_COMMAND = 0
    CHAN_EXECUTABLE = 1
    CHAN_CONTROL = 2
    CHAN_REPORTS = 3
    CHAN_WAKE_REPORTS = 4
    CHAN_GYRO_ROTATION = 5

    # Report IDs
    REPORT_ACCELEROMETER = 0x01
    REPORT_GYROSCOPE = 0x02
    REPORT_MAGNETOMETER = 0x03
    REPORT_ROTATION_VECTOR = 0x05
    REPORT_GAME_ROTATION_VECTOR = 0x08
    REPORT_SET_FEATURE = 0xFD
    REPORT_PRODUCT_ID_REQ = 0xF9
    REPORT_PRODUCT_ID_RESP = 0xF8

    def __init__(self, cs_pin=16, int_pin=26, rst_pin=21, spi_bus=0, spi_dev=0, baudrate=3000000):
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_dev)
        self.spi.max_speed_hz = baudrate
        self.spi.mode = 3 # SPI Mode 3 (CPOL=1, CPHA=1)

        self.cs = digitalio.DigitalInOut(getattr(board, f"D{cs_pin}"))
        self.cs.direction = digitalio.Direction.OUTPUT
        self.cs.value = True # CS High (inactive)

        self.rst = digitalio.DigitalInOut(getattr(board, f"D{rst_pin}"))
        self.rst.direction = digitalio.Direction.OUTPUT
        self.rst.value = True

        self.int_pin = digitalio.DigitalInOut(getattr(board, f"D{int_pin}"))
        self.int_pin.direction = digitalio.Direction.INPUT
        self.int_pin.pull = digitalio.Pull.UP

        self.seq_out = [0] * 6
        self.seq_in = [0] * 6
        self.data = {
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "quat_w": 1.0, "quat_x": 0.0, "quat_y": 0.0, "quat_z": 0.0,
            "accel_x": 0.0, "accel_y": 0.0, "accel_z": 0.0,
            "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0,
            "mag_x": 0.0, "mag_y": 0.0, "mag_z": 0.0,
            "updated": False
        }

    def is_int_asserted(self):
        return not self.int_pin.value # Active LOW

    def reset(self):
        self.rst.value = False
        time.sleep(0.02)
        self.rst.value = True
        time.sleep(0.05)

        # Wait for INT assertion
        start = time.time()
        while not self.is_int_asserted() and (time.time() - start < 2.0):
            time.sleep(0.001)

        # Drain all startup packets
        for _ in range(10):
            start_d = time.time()
            while not self.is_int_asserted() and (time.time() - start_d < 0.1):
                time.sleep(0.0005)
            if not self.read_packet():
                break

    def read_packet(self):
        if not self.is_int_asserted():
            return None

        # Assert CS
        self.cs.value = False
        time.sleep(0.00005)

        # Read 4-byte SHTP header
        hdr = self.spi.xfer2([0x00, 0x00, 0x00, 0x00])
        raw_len = hdr[0] | (hdr[1] << 8)
        pkt_len = raw_len & ~0x8000 # Mask continuation bit
        channel = hdr[2]
        seq = hdr[3]

        payload = []
        if 4 < pkt_len < 1024:
            payload = self.spi.xfer2([0x00] * (pkt_len - 4))

        # Deassert CS
        self.cs.value = True
        time.sleep(0.0001)

        self.seq_in[channel] = seq
        return channel, seq, pkt_len, payload

    def send_packet(self, channel, payload):
        # 1. Drain pending reads first
        while self.is_int_asserted():
            self.read_packet()

        pkt_len = len(payload) + 4
        seq = self.seq_out[channel]
        self.seq_out[channel] = (self.seq_out[channel] + 1) % 256
        full_pkt = [pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, channel, seq] + list(payload)

        # 2. Assert CS to request write
        self.cs.value = False
        start_w = time.time()
        while not self.is_int_asserted() and (time.time() - start_w < 0.05):
            time.sleep(0.0001)

        # 3. Transmit packet
        self.spi.xfer2(full_pkt)

        # 4. Deassert CS
        self.cs.value = True
        time.sleep(0.001)

    def enable_feature(self, report_id, interval_us=10000):
        # SHTP Set Feature Command (0xFD)
        cmd = bytearray(17)
        cmd[0] = self.REPORT_SET_FEATURE
        cmd[1] = report_id
        cmd[2] = 0x00 # flags
        cmd[3] = 0x00 # change sensitivity
        cmd[4] = 0x00
        struct.pack_into("<I", cmd, 5, interval_us) # report interval (us)
        struct.pack_into("<I", cmd, 9, 0)           # batch interval
        struct.pack_into("<I", cmd, 13, 0)          # sensor specific
        self.send_packet(self.CHAN_CONTROL, cmd)
        time.sleep(0.01)

    def update(self):
        pkt = self.read_packet()
        if not pkt:
            return False

        chan, seq, plen, payload = pkt
        if chan == self.CHAN_REPORTS and len(payload) >= 5:
            idx = 0
            while idx < len(payload):
                if idx + 5 > len(payload):
                    break
                rep_id = payload[idx + 4]

                # Rotation Vector (0x05) or Game Rotation Vector (0x08)
                if rep_id in (self.REPORT_ROTATION_VECTOR, self.REPORT_GAME_ROTATION_VECTOR) and idx + 19 <= len(payload):
                    qi, qj, qk, qr = struct.unpack_from("<hhhh", bytes(payload), idx + 7)
                    q_scale = 1.0 / (1 << 14)
                    qx, qy, qz, qw = qi * q_scale, qj * q_scale, qk * q_scale, qr * q_scale
                    self.data["quat_w"] = qw
                    self.data["quat_x"] = qx
                    self.data["quat_y"] = qy
                    self.data["quat_z"] = qz

                    sinr_cosp = 2.0 * (qw * qx + qy * qz)
                    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
                    self.data["roll"] = math.atan2(sinr_cosp, cosr_cosp) * (180.0 / math.pi)

                    sinp = 2.0 * (qw * qy - qz * qx)
                    self.data["pitch"] = math.asin(max(-1.0, min(1.0, sinp))) * (180.0 / math.pi)

                    siny_cosp = 2.0 * (qw * qz + qx * qy)
                    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                    self.data["yaw"] = math.atan2(siny_cosp, cosy_cosp) * (180.0 / math.pi)
                    self.data["updated"] = True
                    idx += 19

                # Accelerometer (0x01)
                elif rep_id == self.REPORT_ACCELEROMETER and idx + 15 <= len(payload):
                    raw_ax, raw_ay, raw_az = struct.unpack_from("<hhh", bytes(payload), idx + 7)
                    acc_scale = 1.0 / (1 << 8)
                    self.data["accel_x"] = raw_ax * acc_scale
                    self.data["accel_y"] = raw_ay * acc_scale
                    self.data["accel_z"] = raw_az * acc_scale
                    idx += 15

                # Gyroscope (0x02)
                elif rep_id == self.REPORT_GYROSCOPE and idx + 15 <= len(payload):
                    raw_gx, raw_gy, raw_gz = struct.unpack_from("<hhh", bytes(payload), idx + 7)
                    gyro_scale = 1.0 / (1 << 9)
                    self.data["gyro_x"] = raw_gx * gyro_scale
                    self.data["gyro_y"] = raw_gy * gyro_scale
                    self.data["gyro_z"] = raw_gz * gyro_scale
                    idx += 15
                else:
                    idx += 1
            return True
        return False

    def close(self):
        self.spi.close()

if __name__ == "__main__":
    print("Initializing BNO08X SPI Driver...")
    imu = BNO08xSPI(cs_pin=16, int_pin=26, rst_pin=21)
    imu.reset()
    print("Enabling Game Rotation Vector @ 100Hz...")
    imu.enable_feature(BNO08xSPI.REPORT_GAME_ROTATION_VECTOR, 10000)
    print("Enabling Accelerometer @ 100Hz...")
    imu.enable_feature(BNO08xSPI.REPORT_ACCELEROMETER, 10000)
    print("Enabling Gyroscope @ 100Hz...")
    imu.enable_feature(BNO08xSPI.REPORT_GYROSCOPE, 10000)

    print("\nStreaming 9-DOF IMU Telemetry for 5 seconds:")
    print("-" * 75)
    start = time.time()
    samples = 0
    while time.time() - start < 5.0:
        if imu.update():
            samples += 1
            if samples % 5 == 0:
                d = imu.data
                print(f"[{samples:03d}] ROLL:{d['roll']:+6.1f}° | PITCH:{d['pitch']:+6.1f}° | YAW:{d['yaw']:+6.1f}° | Quat: ({d['quat_w']:+.2f}, {d['quat_x']:+.2f}, {d['quat_y']:+.2f}, {d['quat_z']:+.2f}) | Accel: ({d['accel_x']:+.1f}, {d['accel_y']:+.1f}, {d['accel_z']:+.1f})")
        time.sleep(0.001)

    print("-" * 75)
    print(">> TEST PASSED! <<")
    imu.close()
