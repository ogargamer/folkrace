#!/usr/bin/env python3
"""
BNO08X (BNO080 / BNO085) I2C Driver for Raspberry Pi 5
Standard I2C address: 0x4B (or 0x4A)
Thread-safe, Zero-Latency FIFO Draining.
"""

import time
import smbus2
import struct
import math
import threading
import board
import digitalio

class BNO08xI2C:
    # SHTP Channels
    CHANNEL_COMMAND = 0
    CHANNEL_EXE = 1
    CHANNEL_CONTROL = 2
    CHANNEL_REPORTS = 3

    # Reports
    REPORT_ACCELEROMETER = 0x01
    REPORT_GYROSCOPE = 0x02
    REPORT_MAGNETOMETER = 0x03
    REPORT_LINEAR_ACCEL = 0x04
    REPORT_ROTATION_VECTOR = 0x05
    REPORT_GAME_ROTATION_VECTOR = 0x08
    REPORT_ARVR_STABILIZED_GRV = 0x28
    REPORT_ARVR_STABILIZED_RV = 0x29
    REPORT_SET_FEATURE = 0xFD
    REPORT_PRODUCT_ID_REQ = 0xF9
    REPORT_PRODUCT_ID_RESP = 0xF8

    def __init__(self, bus_id=1, address=0x4B, rst_pin=21, int_pin=26):
        self.bus_id = bus_id
        self.address = address
        self.bus = smbus2.SMBus(bus_id)
        self.lock = threading.RLock()

        # Control pins
        self.rst_pin = digitalio.DigitalInOut(getattr(board, f"D{rst_pin}"))
        self.rst_pin.direction = digitalio.Direction.OUTPUT
        self.rst_pin.value = True

        self.int_pin = digitalio.DigitalInOut(getattr(board, f"D{int_pin}"))
        self.int_pin.direction = digitalio.Direction.INPUT
        self.int_pin.pull = digitalio.Pull.UP

        self.seq_tx = [0] * 6
        self.data = {
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "quat_w": 1.0, "quat_x": 0.0, "quat_y": 0.0, "quat_z": 0.0,
            "accel_x": 0.0, "accel_y": 0.0, "accel_z": 0.0,
            "lin_accel_x": 0.0, "lin_accel_y": 0.0, "lin_accel_z": 0.0,
            "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0,
            "updated": False,
            "timestamp": 0.0
        }

    def is_int(self):
        return not self.int_pin.value

    def reset(self):
        with self.lock:
            self.rst_pin.value = False
            time.sleep(0.02)
            self.rst_pin.value = True
            time.sleep(0.05)

            start = time.time()
            while not self.is_int() and (time.time() - start < 2.0):
                time.sleep(0.001)

            # Drain boot packets
            for _ in range(6):
                start_w = time.time()
                while not self.is_int() and (time.time() - start_w < 0.15):
                    time.sleep(0.001)
                pkt = self._read_packet(128)
                if not pkt or pkt[2] <= 4:
                    time.sleep(0.01)

    def _read_packet(self, max_len=128):
        with self.lock:
            try:
                msg = smbus2.i2c_msg.read(self.address, max_len)
                self.bus.i2c_rdwr(msg)
                raw = list(msg)
                plen = (raw[0] | (raw[1] << 8)) & ~0x8000
                chan = raw[2]
                seq = raw[3]
                if plen < 4 or plen > max_len:
                    return chan, seq, plen, []
                return chan, seq, plen, raw[4:plen]
            except Exception:
                return None

    def _write_packet(self, channel, payload):
        with self.lock:
            try:
                pkt_len = len(payload) + 4
                seq = self.seq_tx[channel]
                self.seq_tx[channel] = (self.seq_tx[channel] + 1) % 256
                full_pkt = [pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, channel, seq] + list(payload)
                msg = smbus2.i2c_msg.write(self.address, full_pkt)
                self.bus.i2c_rdwr(msg)
                time.sleep(0.001)
            except Exception:
                pass

    def enable_feature(self, feature_id, interval_us=10000):
        with self.lock:
            cmd = bytearray(17)
            cmd[0] = self.REPORT_SET_FEATURE # 0xFD
            cmd[1] = feature_id
            cmd[2] = 0 # Flags
            cmd[3] = 0 # Change sensitivity LSB
            cmd[4] = 0 # Change sensitivity MSB
            struct.pack_into("<I", cmd, 5, interval_us) # Report interval in µs
            struct.pack_into("<I", cmd, 9, 0)           # Batch interval = 0 (Instant stream, no batching!)
            struct.pack_into("<I", cmd, 13, 0)          # Sensor-specific = 0
            self._write_packet(self.CHANNEL_CONTROL, cmd)
            time.sleep(0.015)

    def update(self):
        has_update = False
        loop_count = 0

        # Drain all available packets in hardware FIFO immediately
        while self.is_int() and loop_count < 10:
            loop_count += 1
            pkt = self._read_packet(128)
            if not pkt:
                break

            chan, seq, plen, data = pkt
            if chan == self.CHANNEL_REPORTS and len(data) >= 5:
                offset = 0
                while offset < len(data):
                    rep_id = data[offset]

                    # 1. Base Timestamp (0xFB) - 5 bytes
                    if rep_id == 0xFB:
                        offset += 5

                    # 2. Game Rotation Vector (0x08) & AR/VR Stabilized GRV (0x28) - 12 bytes
                    elif rep_id in (self.REPORT_GAME_ROTATION_VECTOR, self.REPORT_ARVR_STABILIZED_GRV):
                        if offset + 12 <= len(data):
                            raw_i, raw_j, raw_k, raw_r = struct.unpack_from("<hhhh", bytes(data), offset + 4)
                            q_scale = 1.0 / (1 << 14)
                            qx, qy, qz, qw = raw_i * q_scale, raw_j * q_scale, raw_k * q_scale, raw_r * q_scale
                            
                            mag = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
                            if mag > 0.0001:
                                qw, qx, qy, qz = qw/mag, qx/mag, qy/mag, qz/mag

                            self.data["quat_w"] = qw
                            self.data["quat_x"] = qx
                            self.data["quat_y"] = qy
                            self.data["quat_z"] = qz

                            sinr_cosp = 2.0 * (qw * qx + qy * qz)
                            cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
                            self.data["roll"] = math.atan2(sinr_cosp, cosr_cosp) * (180.0 / math.pi)

                            sinp = 2.0 * (qw * qy - qz * qx)
                            pitch_clamped = max(-1.0, min(1.0, sinp))
                            self.data["pitch"] = math.asin(pitch_clamped) * (180.0 / math.pi)

                            siny_cosp = 2.0 * (qw * qz + qx * qy)
                            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                            self.data["yaw"] = math.atan2(siny_cosp, cosy_cosp) * (180.0 / math.pi)
                            
                            has_update = True
                            offset += 12
                        else:
                            break

                    # 3. Absolute Rotation Vector (0x05) & AR/VR Stabilized RV (0x29) - 14 bytes
                    elif rep_id in (self.REPORT_ROTATION_VECTOR, self.REPORT_ARVR_STABILIZED_RV):
                        if offset + 14 <= len(data):
                            raw_i, raw_j, raw_k, raw_r = struct.unpack_from("<hhhh", bytes(data), offset + 4)
                            q_scale = 1.0 / (1 << 14)
                            qx, qy, qz, qw = raw_i * q_scale, raw_j * q_scale, raw_k * q_scale, raw_r * q_scale
                            
                            mag = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
                            if mag > 0.0001:
                                qw, qx, qy, qz = qw/mag, qx/mag, qy/mag, qz/mag

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
                            
                            has_update = True
                            offset += 14
                        else:
                            break

                    # 4. Total Accelerometer (0x01) - 10 bytes
                    elif rep_id == self.REPORT_ACCELEROMETER:
                        if offset + 10 <= len(data):
                            raw_ax, raw_ay, raw_az = struct.unpack_from("<hhh", bytes(data), offset + 4)
                            acc_scale = 1.0 / (1 << 8)
                            self.data["accel_x"] = raw_ax * acc_scale
                            self.data["accel_y"] = raw_ay * acc_scale
                            self.data["accel_z"] = raw_az * acc_scale
                            has_update = True
                            offset += 10
                        else:
                            break

                    # 5. Gyroscope (0x02) - 10 bytes
                    elif rep_id == self.REPORT_GYROSCOPE:
                        if offset + 10 <= len(data):
                            raw_gx, raw_gy, raw_gz = struct.unpack_from("<hhh", bytes(data), offset + 4)
                            gyro_scale = 1.0 / (1 << 9)
                            self.data["gyro_x"] = raw_gx * gyro_scale
                            self.data["gyro_y"] = raw_gy * gyro_scale
                            self.data["gyro_z"] = raw_gz * gyro_scale
                            has_update = True
                            offset += 10
                        else:
                            break

                    # 6. Linear Acceleration (0x04) - 10 bytes
                    elif rep_id == self.REPORT_LINEAR_ACCEL:
                        if offset + 10 <= len(data):
                            raw_lax, raw_lay, raw_laz = struct.unpack_from("<hhh", bytes(data), offset + 4)
                            acc_scale = 1.0 / (1 << 8)
                            self.data["lin_accel_x"] = raw_lax * acc_scale
                            self.data["lin_accel_y"] = raw_lay * acc_scale
                            self.data["lin_accel_z"] = raw_laz * acc_scale
                            has_update = True
                            offset += 10
                        else:
                            break

                    # 7. Magnetometer (0x03) - 10 bytes
                    elif rep_id == self.REPORT_MAGNETOMETER:
                        offset += 10 if offset + 10 <= len(data) else len(data)

                    else:
                        offset += 1
            else:
                break

        if has_update:
            self.data["updated"] = True
            self.data["timestamp"] = time.time()
            return True
        return False

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass
