#!/usr/bin/env python3
"""
Direct ROS 2 Cartographer Live Diagnostics Node (Pi 5)
Subscribes directly to /tf, /scan, /imu, and /map to analyze Cartographer SLAM health and drift.
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage

class CartoDiagnosticsNode(Node):
    def __init__(self):
        super().__init__('carto_diagnostics_node')
        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_cb, 50)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_cb, 50)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 1)

        self.poses = []
        self.scan_count = 0
        self.imu_count = 0
        self.map_count = 0
        self.start_time = time.time()
        self.last_scan = None
        self.last_imu = None

        self.timer = self.create_timer(1.0, self.timer_cb)
        self.get_logger().info('Cartographer Live Diagnostics running for 10 seconds...')

    def tf_cb(self, msg: TFMessage):
        for t in msg.transforms:
            if t.header.frame_id in ['map', 'odom'] and t.child_frame_id == 'base_link':
                pos = t.transform.translation
                rot = t.transform.rotation
                siny_cosp = 2.0 * (rot.w * rot.z + rot.x * rot.y)
                cosy_cosp = 1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                self.poses.append((time.time(), pos.x, pos.y, yaw))

    def scan_cb(self, msg: LaserScan):
        self.scan_count += 1
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        self.last_scan = {
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'frame_id': msg.header.frame_id,
            'total_pts': len(msg.ranges),
            'valid_pts': len(valid_ranges),
            'min_r': min(valid_ranges) if valid_ranges else 0.0,
            'max_r': max(valid_ranges) if valid_ranges else 0.0
        }

    def imu_cb(self, msg: Imu):
        self.imu_count += 1
        q = msg.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp) * 57.2958

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.copysign(90.0, sinp) if abs(sinp) >= 1 else math.asin(sinp) * 57.2958

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp) * 57.2958

        self.last_imu = {
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'frame_id': msg.header.frame_id,
            'r': roll, 'p': pitch, 'y': yaw,
            'ax': msg.linear_acceleration.x,
            'ay': msg.linear_acceleration.y,
            'az': msg.linear_acceleration.z,
            'gx': msg.angular_velocity.x * 57.2958,
            'gy': msg.angular_velocity.y * 57.2958,
            'gz': msg.angular_velocity.z * 57.2958
        }

    def map_cb(self, msg: OccupancyGrid):
        self.map_count += 1

    def timer_cb(self):
        elapsed = time.time() - self.start_time
        if elapsed >= 10.0:
            self.print_report()
            rclpy.shutdown()

    def print_report(self):
        dt = time.time() - self.start_time
        print("\n" + "=" * 64)
        print("          CARTOGRAPHER ROS 2 REAL-TIME DIAGNOSTIC REPORT")
        print("=" * 64)
        print(f"Sample Duration: {dt:.1f}s")
        print(f"LiDAR /scan:  {self.scan_count} msgs ({self.scan_count/dt:.1f} Hz)")
        if self.last_scan:
            print(f"  Frame: {self.last_scan['frame_id']} | Pts: {self.last_scan['valid_pts']}/{self.last_scan['total_pts']} | Range: {self.last_scan['min_r']:.2f}m - {self.last_scan['max_r']:.2f}m")
        print(f"IMU /imu:     {self.imu_count} msgs ({self.imu_count/dt:.1f} Hz)")
        if self.last_imu:
            print(f"  Frame: {self.last_imu['frame_id']} | RPY: ({self.last_imu['r']:.1f}°, {self.last_imu['p']:.1f}°, {self.last_imu['y']:.1f}°)")
            print(f"  Accel: ({self.last_imu['ax']:.2f}, {self.last_imu['ay']:.2f}, {self.last_imu['az']:.2f}) m/s²")
            print(f"  Gyro:  ({self.last_imu['gx']:.1f}, {self.last_imu['gy']:.1f}, {self.last_imu['gz']:.1f}) °/s")
        print(f"Map Updates:  {self.map_count}")

        print("\n--- ROBOT POSE DRIFT ANALYSIS (Cartographer map -> base_link) ---")
        if not self.poses:
            print("[CRITICAL ERROR] No /tf transforms published for base_link!")
        else:
            p0 = self.poses[0]
            p1 = self.poses[-1]
            dx = p1[1] - p0[1]
            dy = p1[2] - p0[2]
            dyaw = math.degrees(p1[3] - p0[3])
            dist = math.hypot(dx, dy)
            speed = dist / (p1[0] - p0[0] or 1.0)
            xs = [p[1] for p in self.poses]
            ys = [p[2] for p in self.poses]
            yaws = [math.degrees(p[3]) for p in self.poses]

            print(f"Total Pose Updates: {len(self.poses)} ({len(self.poses)/dt:.1f} Hz)")
            print(f"Start Pose: X = {p0[1]:.4f} m, Y = {p0[2]:.4f} m, Yaw = {math.degrees(p0[3]):.2f}°")
            print(f"End Pose:   X = {p1[1]:.4f} m, Y = {p1[2]:.4f} m, Yaw = {math.degrees(p1[3]):.2f}°")
            print(f"Net Drift:  dX = {dx:.4f} m, dY = {dy:.4f} m, Distance = {dist:.4f} m")
            print(f"Drift Rate: {speed*100:.2f} cm/s ({speed*1000:.1f} mm/s)")
            print(f"Bounding:   X span = {max(xs)-min(xs):.4f} m, Y span = {max(ys)-min(ys):.4f} m, Yaw span = {max(yaws)-min(yaws):.2f}°")

            if speed > 0.05:
                print(f"\n>> STATUS: [FAIL] High drift ({speed*100:.1f} cm/s) <<")
            elif speed > 0.01:
                print(f"\n>> STATUS: [WARN] Moderate drift ({speed*100:.2f} cm/s) <<")
            else:
                print(f"\n>> STATUS: [PASS] Perfectly stationary (< {speed*100:.2f} cm/s) <<")
        print("=" * 64 + "\n")

def main():
    rclpy.init()
    node = CartoDiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
