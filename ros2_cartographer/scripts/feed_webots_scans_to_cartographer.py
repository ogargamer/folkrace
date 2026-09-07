#!/usr/bin/env python3
"""
================================================================================
  FEED LIVE WEBOTS SCANS INTO ROS 2 CARTOGRAPHER
  Uses exact Webots simulation timestamps and ROS REP-105 standard coordinate frames.
================================================================================
"""

import sys
import os
import json
import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time as MsgTime
import tf2_ros

class WebotsScanFeeder(Node):
    def __init__(self, jsonl_path):
        super().__init__('webots_scan_feeder')
        self.jsonl_path = jsonl_path
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 50)
        self.imu_pub = self.create_publisher(Imu, '/imu', 100)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        self._broadcast_static_transforms()
        self.get_logger().info(f"Initialized Webots Scan Feeder from {jsonl_path}")

    def _broadcast_static_transforms(self):
        t_zero = MsgTime(sec=0, nanosec=0)
        
        # 1. Base link to laser
        # In Webots, the robot forward direction is local +Y.
        # In ROS (REP-105), the robot forward direction is +X.
        # Therefore, the base_link coordinate system in ROS has +X forward, +Y left, +Z up.
        # Webots ms200k_lidar beam index 0 is along the robot lateral axis (-X in Webots, which is -Y in ROS).
        # We define tf_laser to represent the physical sensor mount orientation on base_link:
        tf_laser = TransformStamped()
        tf_laser.header.stamp = t_zero
        tf_laser.header.frame_id = "base_link"
        tf_laser.child_frame_id = "laser"
        tf_laser.transform.translation.x = 0.01   # forward offset
        tf_laser.transform.translation.y = 0.0    # lateral offset
        tf_laser.transform.translation.z = 0.0725 # elevation
        tf_laser.transform.rotation.w = 1.0

        # 2. Base link to IMU
        tf_imu = TransformStamped()
        tf_imu.header.stamp = t_zero
        tf_imu.header.frame_id = "base_link"
        tf_imu.child_frame_id = "imu_link"
        tf_imu.transform.translation.x = -0.01
        tf_imu.transform.translation.y = 0.0
        tf_imu.transform.translation.z = 0.015
        tf_imu.transform.rotation.w = 1.0

        self.tf_static_broadcaster.sendTransform([tf_laser, tf_imu])

    def run_feed(self):
        with open(self.jsonl_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        self.get_logger().info(f"Feeding {len(lines)} Webots scans with exact simulation timestamps...")

        # Base timestamp: starting at epoch 1000.0s so sec/nanosec are clean positive integers
        base_epoch = 1000.0

        for i, line in enumerate(lines):
            packet = json.loads(line)
            sim_time = float(packet.get("time", i * 0.02))
            abs_time = base_epoch + sim_time
            sec = int(abs_time)
            nanosec = int((abs_time - sec) * 1e9)
            stamp = MsgTime(sec=sec, nanosec=nanosec)

            # 1. Publish IMU with exact simulation timestamp
            # Robot yaw in Webots ENU: 0 rad = East (+X), pi/2 = North (+Y).
            # When Folkrace faces East, yaw = 0.
            yaw = float(packet.get("yaw", 0.0))
            gyro = packet.get("gyro", [0.0, 0.0, 0.0])
            accel = packet.get("accel", [0.0, 0.0, 0.0])

            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = "imu_link"

            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            imu.orientation.w = cy
            imu.orientation.x = 0.0
            imu.orientation.y = 0.0
            imu.orientation.z = sy

            # Gyro Z in Webots: positive = CCW rotation
            imu.angular_velocity.x = float(gyro[0])
            imu.angular_velocity.y = float(gyro[1])
            imu.angular_velocity.z = float(gyro[2])

            # Accel in Webots: [lateral, forward, vertical]
            imu.linear_acceleration.x = float(accel[1]) # forward accel (+X in ROS base_link)
            imu.linear_acceleration.y = float(accel[0]) # lateral accel (+Y in ROS base_link)
            imu.linear_acceleration.z = float(accel[2]) # vertical accel (+Z)

            imu.orientation_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
            imu.angular_velocity_covariance = [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001]
            imu.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            self.imu_pub.publish(imu)

            # 2. Publish LaserScan with exact simulation timestamp
            ranges = packet.get("ranges", [])
            if ranges:
                n_samples = len(ranges)
                scan = LaserScan()
                scan.header.stamp = stamp
                scan.header.frame_id = "laser"
                scan.angle_min = -math.pi
                scan.angle_max = math.pi - (2.0 * math.pi / float(n_samples))
                scan.angle_increment = (2.0 * math.pi) / float(n_samples)
                scan.time_increment = 0.0
                scan.scan_time = 0.02
                scan.range_min = 0.12
                scan.range_max = 4.50

                # In Webots: beam 0 is along -X (right/lateral).
                # To align with ROS standard (0 rad = straight ahead +X, -pi = behind):
                # We shift the 450-beam array by 112.5 beams (+90 degrees).
                shift = int(round(n_samples / 4.0)) # 112 beams
                shifted_ranges = ranges[shift:] + ranges[:shift]

                clean_ranges = []
                for r in shifted_ranges:
                    # Anything above 4.5m or infinity is ignored (set to inf so Cartographer doesn't register a false hit)
                    if math.isinf(r) or math.isnan(r) or r > 4.50 or r < 0.12:
                        clean_ranges.append(float('inf'))
                    else:
                        clean_ranges.append(float(r))
                scan.ranges = clean_ranges
                self.scan_pub.publish(scan)

            time.sleep(0.005) # Feed fast to ROS 2

        self.get_logger().info("Finished feeding all Webots scans.")

def main():
    rclpy.init()
    jsonl_path = sys.argv[1] if len(sys.argv) > 1 else "/mnt/c/Users/nurmi/Documents/Da Winna/my_project/controllers/folkrace_drive/live_webots_slam_scans.jsonl"
    node = WebotsScanFeeder(jsonl_path)
    node.run_feed()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
