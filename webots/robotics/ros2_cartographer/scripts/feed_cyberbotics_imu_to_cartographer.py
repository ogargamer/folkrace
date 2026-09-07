#!/usr/bin/env python3
"""
================================================================================
  CYBERBOTICS NATIVE FEEDER WITH SENSOR_MSGS/IMU SUPPORT
  Publishes:
    1. /scan (sensor_msgs/LaserScan, angle_min=+pi, angle_max=-pi, range < 4.5m)
    2. /imu (sensor_msgs/Imu, ideal ground-truth gyro & accel, tracking_frame="imu_link")
    3. /odom (nav_msgs/Odometry)
    4. TF tree: odom -> base_link -> laser & imu_link
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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time as MsgTime
import tf2_ros

class CyberboticsImuFeeder(Node):
    def __init__(self, jsonl_path):
        super().__init__('cyberbotics_imu_feeder')
        self.jsonl_path = jsonl_path
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 50)
        self.imu_pub = self.create_publisher(Imu, '/imu', 100)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 100)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        self._broadcast_static_transforms()
        self.get_logger().info(f"Initialized Cyberbotics IMU+Lidar Feeder from {jsonl_path}")

    def _broadcast_static_transforms(self):
        t_zero = MsgTime(sec=0, nanosec=0)
        
        # 1. Laser mount on robot chassis
        tf_laser = TransformStamped()
        tf_laser.header.stamp = t_zero
        tf_laser.header.frame_id = "base_link"
        tf_laser.child_frame_id = "laser"
        tf_laser.transform.translation.x = 0.01
        tf_laser.transform.translation.y = 0.0
        tf_laser.transform.translation.z = 0.0725
        tf_laser.transform.rotation.w = 1.0

        # 2. IMU mount on robot chassis
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

        self.get_logger().info(f"Feeding {len(lines)} Webots scans with high-reliability IMU telemetry...")
        base_epoch = 1000.0

        for i, line in enumerate(lines):
            packet = json.loads(line)
            sim_time = float(packet.get("time", i * 0.02))
            abs_time = base_epoch + sim_time
            sec = int(abs_time)
            nanosec = int((abs_time - sec) * 1e9)
            stamp = MsgTime(sec=sec, nanosec=nanosec)

            pos = packet.get("pos", [0.0, 0.0])
            yaw = float(packet.get("yaw", 0.0))
            rx, ry = float(pos[0]), float(pos[1])
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)

            # 1. Odometry
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = rx
            odom.pose.pose.position.y = ry
            odom.pose.pose.position.z = 0.02
            odom.pose.pose.orientation.w = cy
            odom.pose.pose.orientation.z = sy
            odom.pose.covariance = [0.001]*36
            self.odom_pub.publish(odom)

            # 2. Dynamic TF: odom -> base_link
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = "odom"
            tf_msg.child_frame_id = "base_link"
            tf_msg.transform.translation.x = rx
            tf_msg.transform.translation.y = ry
            tf_msg.transform.translation.z = 0.02
            tf_msg.transform.rotation.w = cy
            tf_msg.transform.rotation.z = sy
            self.tf_broadcaster.sendTransform(tf_msg)

            # 3. High-reliability IMU message
            gyro_data = packet.get("gyro", [0.0, 0.0, 0.0])
            accel_data = packet.get("accel", [0.0, 0.0, 9.81])
            
            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = "imu_link"
            imu_msg.orientation.w = cy
            imu_msg.orientation.z = sy
            imu_msg.orientation_covariance = [0.0001]*9
            
            imu_msg.angular_velocity.x = float(gyro_data[0])
            imu_msg.angular_velocity.y = float(gyro_data[1])
            imu_msg.angular_velocity.z = float(gyro_data[2])
            imu_msg.angular_velocity_covariance = [0.0001]*9
            
            imu_msg.linear_acceleration.x = float(accel_data[0])
            imu_msg.linear_acceleration.y = float(accel_data[1])
            imu_msg.linear_acceleration.z = float(accel_data[2])
            imu_msg.linear_acceleration_covariance = [0.0001]*9
            self.imu_pub.publish(imu_msg)

            # 4. LaserScan
            ranges = packet.get("ranges", [])
            if ranges:
                resolution = len(ranges)
                fov = 2.0 * math.pi
                scan = LaserScan()
                scan.header.stamp = stamp
                scan.header.frame_id = "laser"
                scan.angle_min = fov / 2.0
                scan.angle_max = -fov / 2.0
                scan.angle_increment = -fov / float(resolution - 1)
                scan.time_increment = 0.02 / float(resolution)
                scan.scan_time = 0.02
                scan.range_min = 0.12
                scan.range_max = 4.50

                clean_ranges = []
                for r in ranges:
                    if math.isinf(r) or math.isnan(r) or r > 4.50 or r < 0.12:
                        clean_ranges.append(float('inf'))
                    else:
                        clean_ranges.append(float(r))
                scan.ranges = clean_ranges
                self.scan_pub.publish(scan)

            time.sleep(0.003)

        self.get_logger().info("Finished feeding all Webots scans and IMU telemetry.")

def main():
    rclpy.init()
    jsonl_path = sys.argv[1] if len(sys.argv) > 1 else "/mnt/c/Users/nurmi/Documents/Da Winna/webots/simulation/my_project/controllers/folkrace_drive/live_webots_slam_scans.jsonl"
    node = CyberboticsImuFeeder(jsonl_path)
    node.run_feed()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
