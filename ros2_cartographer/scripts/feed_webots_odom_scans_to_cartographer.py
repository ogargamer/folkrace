#!/usr/bin/env python3
"""
================================================================================
  FEED LIVE WEBOTS SCANS + ODOMETRY INTO ROS 2 CARTOGRAPHER (TURTLEBOT3 PATTERN)
================================================================================
"""

import sys
import os
import json
import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time as MsgTime
import tf2_ros

class WebotsOdomScanFeeder(Node):
    def __init__(self, jsonl_path):
        super().__init__('webots_odom_scan_feeder')
        self.jsonl_path = jsonl_path
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 50)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 100)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        self._broadcast_static_transforms()
        self.get_logger().info(f"Initialized Webots Odom+Scan Feeder from {jsonl_path}")

    def _broadcast_static_transforms(self):
        t_zero = MsgTime(sec=0, nanosec=0)
        tf_laser = TransformStamped()
        tf_laser.header.stamp = t_zero
        tf_laser.header.frame_id = "base_link"
        tf_laser.child_frame_id = "laser"
        tf_laser.transform.translation.x = 0.01
        tf_laser.transform.translation.y = 0.0
        tf_laser.transform.translation.z = 0.0725
        tf_laser.transform.rotation.w = 1.0
        self.tf_static_broadcaster.sendTransform([tf_laser])

    def run_feed(self):
        with open(self.jsonl_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        self.get_logger().info(f"Feeding {len(lines)} Webots scans with ground truth odometry prior...")
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

            # 1. Publish Odometry message on /odom
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = rx
            odom.pose.pose.position.y = ry
            odom.pose.pose.position.z = 0.02

            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            odom.pose.pose.orientation.w = cy
            odom.pose.pose.orientation.z = sy
            odom.pose.covariance = [0.001]*36

            self.odom_pub.publish(odom)

            # 2. Broadcast odom -> base_link TF transform
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

            # 3. Publish LaserScan with exact simulation timestamp (<4.5m limit)
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

                shift = int(round(n_samples / 4.0)) # 112 beams
                shifted_ranges = ranges[shift:] + ranges[:shift]

                clean_ranges = []
                for r in shifted_ranges:
                    if math.isinf(r) or math.isnan(r) or r > 4.50 or r < 0.12:
                        clean_ranges.append(float('inf'))
                    else:
                        clean_ranges.append(float(r))
                scan.ranges = clean_ranges
                self.scan_pub.publish(scan)

            time.sleep(0.005)

        self.get_logger().info("Finished feeding all Webots scans.")

def main():
    rclpy.init()
    jsonl_path = sys.argv[1] if len(sys.argv) > 1 else "/mnt/c/Users/nurmi/Documents/Da Winna/my_project/controllers/folkrace_drive/live_webots_slam_scans.jsonl"
    node = WebotsOdomScanFeeder(jsonl_path)
    node.run_feed()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
