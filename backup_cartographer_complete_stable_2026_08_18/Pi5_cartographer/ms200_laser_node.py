#!/usr/bin/env python3
"""
MS200 LiDAR ROS 2 LaserScan Publisher (Raspberry Pi 5)
Direct OS TTY Kernel Buffer Streamer.
Publishes sensor_msgs/LaserScan on topic /scan @ 10Hz
"""

import os
import math
import time
import struct
import threading
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

SERIAL_PORT = "/dev/ttyAMA4" # MS200 UART4 (Pins 32 & 33)
BAUDRATE = 230400

CRC_TABLE = [
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
]

def cal_crc8(data):
    crc = 0
    for byte in data:
        crc = CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc

def xor_check(data):
    chk = 0
    for b in data:
        chk ^= b
    return chk

def send_set_speed(ser, hz=10):
    header = struct.pack("<HBBB", 0xF5A5, 0xA1, 0xC1, 2)
    payload = struct.pack("<H", hz)
    chk = xor_check(header + payload)
    tail = bytes([chk, 0x31, 0xF2])
    pkt = header + payload + tail
    try:
        ser.write(pkt)
        ser.flush()
    except Exception:
        pass

class MS200LaserNode(Node):
    def __init__(self):
        super().__init__('ms200_laser_node')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.declare_parameter('port', SERIAL_PORT)
        self.declare_parameter('baudrate', BAUDRATE)
        self.declare_parameter('frame_id', 'laser_frame')

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value

        self.running = True
        self.reader_thread = threading.Thread(target=self._serial_worker, daemon=True)
        self.reader_thread.start()

    def _serial_worker(self):
        self.get_logger().info(f'Starting MS200 LiDAR Direct Worker on {self.port} @ {self.baudrate}...')
        while self.running:
            try:
                ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
                send_set_speed(ser, 10)
                fd = ser.fileno()
                self.get_logger().info(f'Direct TTY handle acquired for MS200 LiDAR ({self.port})!')

                buffer = bytearray()
                current_points = []
                last_start_angle = 0.0

                while self.running:
                    try:
                        chunk = os.read(fd, 4096)
                    except Exception:
                        chunk = None

                    if chunk:
                        buffer.extend(chunk)

                    while len(buffer) >= 47:
                        if buffer[0] == 0x54 and buffer[1] == 0x2C:
                            frame = buffer[:47]
                            if cal_crc8(frame[:46]) == frame[46]:
                                speed, start_angle = struct.unpack_from("<HH", frame, 2)
                                end_angle, timestamp = struct.unpack_from("<HH", frame, 42)

                                start_deg = start_angle / 100.0
                                end_deg = end_angle / 100.0
                                step_deg = (end_deg - start_deg) / 11.0 if end_deg >= start_deg else (end_deg + 360.0 - start_deg) / 11.0

                                for i in range(12):
                                    dist_mm, conf = struct.unpack_from("<HB", frame, 6 + i * 3)
                                    angle_deg = (start_deg + step_deg * i) % 360.0
                                    if dist_mm > 0:
                                        current_points.append((angle_deg, dist_mm / 1000.0, float(conf)))

                                if start_deg < last_start_angle and len(current_points) > 60:
                                    self.publish_laserscan(current_points, speed)
                                    current_points = []

                                last_start_angle = start_deg
                                buffer = buffer[47:]
                            else:
                                buffer.pop(0)
                        else:
                            buffer.pop(0)

                ser.close()
            except Exception as e:
                self.get_logger().warn(f'Serial retry in 1s ({e})')
                time.sleep(1.0)

    def publish_laserscan(self, points, speed_raw):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        NUM_BINS = 360
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = (2.0 * math.pi) / NUM_BINS
        scan_hz = max(1.0, speed_raw / 360.0) if speed_raw > 0 else 10.0
        msg.scan_time = 1.0 / scan_hz
        msg.time_increment = msg.scan_time / NUM_BINS
        msg.range_min = 0.05
        msg.range_max = 4.0

        ranges = [float('inf')] * NUM_BINS
        intensities = [0.0] * NUM_BINS

        for angle_deg, dist_m, conf in points:
            if 0.05 <= dist_m <= 4.0:
                rad = math.radians(angle_deg)
                if rad > math.pi:
                    rad -= 2.0 * math.pi
                rad = -rad # Counter-clockwise ROS standard
                
                bin_idx = int((rad - msg.angle_min) / msg.angle_increment)
                if 0 <= bin_idx < NUM_BINS:
                    if dist_m < ranges[bin_idx]:
                        ranges[bin_idx] = dist_m
                        intensities[bin_idx] = conf

        msg.ranges = ranges
        msg.intensities = intensities
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.running = False
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MS200LaserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
