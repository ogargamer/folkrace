#!/usr/bin/env python3
"""
BNO08X High-Speed IMU ROS 2 Publisher Node (Raspberry Pi 5)
Reads 200 Hz 42-Byte Binary Telemetry from ESP32-S3 over Hardware UART2 (/dev/ttyAMA2 @ 921600 Baud)
Publishes sensor_msgs/Imu on topic /imu for Google Cartographer 2D SLAM.
"""

import os
import sys
import time
import math
import struct
import threading
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class BNO08xUartImuNode(Node):
    def __init__(self):
        super().__init__('bno08x_imu_node')
        self.publisher_ = self.create_publisher(Imu, '/imu', 100)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('port', '/dev/ttyAMA2')
        self.declare_parameter('baud', 921600)

        self.frame_id = self.get_parameter('frame_id').value
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value

        # Auto-fallback if /dev/ttyAMA2 does not exist
        if not os.path.exists(self.port) and os.path.exists('/dev/ttyAMA0'):
            self.port = '/dev/ttyAMA0'

        self.get_logger().info(f'Starting BNO08X UART IMU Bridge on {self.port} @ {self.baud} Baud (Target: 200Hz)...')

        self.running = True
        self.packet_count = 0
        self.last_stat_time = time.time()

        # Dedicated High-Priority Serial Worker Thread
        self.worker_thread = threading.Thread(target=self.serial_reader_loop, daemon=True)
        self.worker_thread.start()

    def serial_reader_loop(self):
        while self.running:
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.1)
                ser.reset_input_buffer()
                self.get_logger().info(f'Connected to ESP32 IMU UART stream on {self.port}!')

                buf = bytearray()
                while self.running:
                    chunk = ser.read(ser.in_waiting or 42)
                    if chunk:
                        buf.extend(chunk)

                    while len(buf) >= 42:
                        if buf[0] == 0xAA and buf[1] == 0x55:
                            frame = buf[:42]
                            try:
                                qw, qx, qy, qz, ax, ay, az, gx, gy, gz = struct.unpack_from("<ffffffffff", frame, 2)

                                # Construct ROS 2 Imu Message
                                msg = Imu()
                                msg.header.stamp = self.get_clock().now().to_msg()
                                msg.header.frame_id = self.frame_id

                                # Orientation (Unit Quaternion)
                                msg.orientation.w = float(qw)
                                msg.orientation.x = float(qx)
                                msg.orientation.y = float(qy)
                                msg.orientation.z = float(qz)

                                # Angular Velocity (rad/s)
                                msg.angular_velocity.x = float(gx)
                                msg.angular_velocity.y = float(gy)
                                msg.angular_velocity.z = float(gz)

                                # Linear Acceleration (m/s²)
                                msg.linear_acceleration.x = float(ax)
                                msg.linear_acceleration.y = float(ay)
                                msg.linear_acceleration.z = float(az)

                                # High-Confidence Covariances for Cartographer
                                msg.orientation_covariance = [
                                    0.001, 0.0,   0.0,
                                    0.0,   0.001, 0.0,
                                    0.0,   0.0,   0.001
                                ]
                                msg.angular_velocity_covariance = [
                                    0.0005, 0.0,    0.0,
                                    0.0,    0.0005, 0.0,
                                    0.0,    0.0,    0.0005
                                ]
                                msg.linear_acceleration_covariance = [
                                    0.01, 0.0,  0.0,
                                    0.0,  0.01, 0.0,
                                    0.0,  0.0,  0.01
                                ]

                                self.publisher_.publish(msg)
                                self.packet_count += 1

                            except Exception:
                                pass

                            del buf[:42]
                        else:
                            del buf[0]

                    # Log throughput every 5 seconds
                    now = time.time()
                    if now - self.last_stat_time >= 5.0:
                        fps = self.packet_count / (now - self.last_stat_time)
                        self.get_logger().info(f'IMU Publishing Rate: {fps:.1f} Hz (Total: {self.packet_count} frames)')
                        self.packet_count = 0
                        self.last_stat_time = now

            except Exception as e:
                self.get_logger().warn(f'IMU Serial connection error: {e}. Reconnecting in 1.5s...')
                time.sleep(1.5)

    def destroy_node(self):
        self.running = False
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BNO08xUartImuNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
