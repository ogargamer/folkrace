#!/usr/bin/env python3
"""
BNO08X IMU ROS 2 Publisher Node (Raspberry Pi 5)
Publishes sensor_msgs/Imu on topic /imu @ 100Hz
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Import our BNO08X driver
from bno08x_i2c import BNO08xI2C

class BNO08xImuNode(Node):
    def __init__(self):
        super().__init__('bno08x_imu_node')
        self.publisher_ = self.create_publisher(Imu, '/imu', 10)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate_hz', 100)

        self.frame_id = self.get_parameter('frame_id').value
        rate_hz = self.get_parameter('rate_hz').value

        self.get_logger().info('Initializing BNO08X IMU on I2C bus 1 (Address 0x4B)...')
        try:
            self.imu = BNO08xI2C(bus_id=1, address=0x4B, rst_pin=21, int_pin=26)
            self.imu.reset()
            interval_us = int(1000000 / rate_hz)
            # Use Hillcrest AR/VR Stabilized GRV (0x28) to eliminate stationary gyro/mag drift
            self.imu.enable_feature(BNO08xI2C.REPORT_ARVR_STABILIZED_GRV, interval_us)
            self.imu.enable_feature(BNO08xI2C.REPORT_ACCELEROMETER, interval_us)
            self.imu.enable_feature(BNO08xI2C.REPORT_GYROSCOPE, interval_us)
            self.get_logger().info(f'BNO08X IMU Initialized with Zero-Drift AR/VR Fusion @ {rate_hz}Hz!')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize BNO08X: {e}')

        # High-rate polling timer (1ms)
        self.timer = self.create_timer(0.001, self.poll_imu)

    def poll_imu(self):
        if not hasattr(self, 'imu'):
            return

        try:
            if self.imu.update():
                d = self.imu.data
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id

                # Orientation (Quaternion)
                msg.orientation.w = float(d['quat_w'])
                msg.orientation.x = float(d['quat_x'])
                msg.orientation.y = float(d['quat_y'])
                msg.orientation.z = float(d['quat_z'])

                # Angular Velocity (rad/s)
                msg.angular_velocity.x = float(d['gyro_x'])
                msg.angular_velocity.y = float(d['gyro_y'])
                msg.angular_velocity.z = float(d['gyro_z'])

                # Linear Acceleration (m/s²)
                msg.linear_acceleration.x = float(d['accel_x'])
                msg.linear_acceleration.y = float(d['accel_y'])
                msg.linear_acceleration.z = float(d['accel_z'])

                # Covariances
                msg.orientation_covariance[0] = 0.01
                msg.orientation_covariance[4] = 0.01
                msg.orientation_covariance[8] = 0.01

                msg.angular_velocity_covariance[0] = 0.001
                msg.angular_velocity_covariance[4] = 0.001
                msg.angular_velocity_covariance[8] = 0.001

                msg.linear_acceleration_covariance[0] = 0.05
                msg.linear_acceleration_covariance[4] = 0.05
                msg.linear_acceleration_covariance[8] = 0.05

                self.publisher_.publish(msg)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = BNO08xImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
