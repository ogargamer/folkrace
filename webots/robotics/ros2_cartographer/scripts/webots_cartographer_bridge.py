"""
================================================================================
  WEBOTS TO ROS 2 GOOGLE CARTOGRAPHER BRIDGE
  Listens on UDP (port 9876) from live Webots folkrace_drive controller
  Publishes ms200k 360-LiDAR (/scan), BNO08X IMU (/imu), and Static TF Frames
================================================================================
"""

import sys
import math
import json
import socket
import select
import numpy as np

import base64
import zlib

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, Imu
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import TransformStamped, PoseStamped
    import tf2_ros
except ImportError:
    print("[ERROR] ROS 2 Python packages (rclpy, sensor_msgs, nav_msgs, tf2_ros) not installed in current environment.")
    print("Please run within your ROS 2 WSL2 environment.")


import struct

def get_windows_host_ip():
    try:
        with open("/proc/net/route", "r") as f:
            for line in f:
                fields = line.strip().split()
                if len(fields) >= 3 and fields[1] == "00000000":
                    return socket.inet_ntoa(struct.pack("<L", int(fields[2], 16)))
    except Exception:
        pass
    return "127.0.0.1"


class WebotsCartographerBridgeNode(Node):
    def __init__(self, udp_port=9876):
        super().__init__('webots_cartographer_bridge')
        
        # Publishers for Cartographer 2D SLAM
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 50)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Subscriber for Cartographer's estimated robot pose (SensorData QoS matches BestEffort publisher)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/tracked_pose', self._tracked_pose_cb, qos_profile_sensor_data
        )

        # Subscriber for Cartographer's live OccupancyGrid map
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, 1
        )

        self._broadcast_static_transforms()

        # UDP Receiver for live Webots sensor stream
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', udp_port))
        self.sock.setblocking(False)

        # Windows Host IP for cross-VM networking
        self.win_host_ip = get_windows_host_ip()
        self.pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.mppi_pose_port = 9877
        self.map_port = 9875

        # Timer to pump UDP packets at 100 Hz
        self.create_timer(0.005, self._poll_udp)
        self.get_logger().info(f"Webots -> Cartographer ROS 2 Bridge Node initialized (Listening on UDP :{udp_port}, streaming to Windows Host {self.win_host_ip}:{self.mppi_pose_port}).")

    def _tracked_pose_cb(self, msg: PoseStamped):
        """Broadcasts the real-time filtered Google Cartographer SLAM pose directly to MPPI."""
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Filtered linear velocity and yaw rate directly from Cartographer pose trajectory
        if hasattr(self, "_prev_pose_t") and (t - self._prev_pose_t) > 1e-4:
            dt = t - self._prev_pose_t
            dx = x - self._prev_pose_x
            dy = y - self._prev_pose_y
            dyaw = (yaw - self._prev_pose_yaw + math.pi) % (2.0 * math.pi) - math.pi
            v = math.hypot(dx, dy) / dt
            wz = dyaw / dt
        else:
            v = 0.0
            wz = 0.0

        self._prev_pose_t = t
        self._prev_pose_x = x
        self._prev_pose_y = y
        self._prev_pose_yaw = yaw

        pkt = {
            "x": round(x, 4),
            "y": round(y, 4),
            "yaw": round(yaw, 4),
            "v": round(v, 3),
            "wz": round(wz, 4),
            "time": round(t, 4),
        }
        payload = json.dumps(pkt).encode("utf-8")
        try:
            self.pose_sock.sendto(payload, ('127.0.0.1', self.mppi_pose_port))
            if self.win_host_ip != '127.0.0.1':
                self.pose_sock.sendto(payload, (self.win_host_ip, self.mppi_pose_port))
        except Exception:
            pass

    def _map_cb(self, msg: OccupancyGrid):
        """Broadcasts live Cartographer occupancy grid to MPPI & Costmap visualizer."""
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        raw_bytes = bytes(msg.data)
        compressed_data = zlib.compress(raw_bytes)

        pkt = {
            "w": w,
            "h": h,
            "res": res,
            "origin": [round(origin_x, 4), round(origin_y, 4)],
            "data_z64": base64.b64encode(compressed_data).decode("ascii"),
            "time": msg.info.map_load_time.sec + msg.info.map_load_time.nanosec * 1e-9,
        }
        payload = json.dumps(pkt).encode("utf-8")
        try:
            self.pose_sock.sendto(payload, ('127.0.0.1', self.map_port))
            if self.win_host_ip != '127.0.0.1':
                self.pose_sock.sendto(payload, (self.win_host_ip, self.map_port))
        except Exception:
            pass

    def _broadcast_static_transforms(self):
        """Broadcasts static robot transforms matching Webots robot dimensions."""
        now = self.get_clock().now().to_msg()

        # 1. Base Link -> Laser Frame (Lidar mount: z = +0.0725 m, y = +0.01 m)
        tf_laser = TransformStamped()
        tf_laser.header.stamp = now
        tf_laser.header.frame_id = "base_link"
        tf_laser.child_frame_id = "laser"
        tf_laser.transform.translation.x = 0.0
        tf_laser.transform.translation.y = 0.01
        tf_laser.transform.translation.z = 0.0725
        tf_laser.transform.rotation.w = 1.0

        # 2. Base Link -> IMU Frame (BNO08X: z = +0.015 m, y = -0.01 m)
        tf_imu = TransformStamped()
        tf_imu.header.stamp = now
        tf_imu.header.frame_id = "base_link"
        tf_imu.child_frame_id = "imu_link"
        tf_imu.transform.translation.x = 0.0
        tf_imu.transform.translation.y = -0.01
        tf_imu.transform.translation.z = 0.015
        tf_imu.transform.rotation.w = 1.0

        self.tf_static_broadcaster.sendTransform([tf_laser, tf_imu])

    def _reset_cartographer(self):
        """Cleans and resets Cartographer SLAM on Webots world reset."""
        self.get_logger().info("[RESET] Simulation reset detected. Resetting Cartographer SLAM...")
        self._prev_pose_t = None
        self._prev_pose_x = None
        self._prev_pose_y = None
        self._prev_pose_yaw = None

        import subprocess
        try:
            subprocess.run("pkill -9 -f cartographer_node; pkill -9 -f cartographer_occupancy_grid_node; rm -rf /dev/shm/*", shell=True)
            cfg_dir = "/mnt/c/Users/nurmi/Documents/Folkrace/webots/robotics/ros2_cartographer/config"
            subprocess.Popen(f"nohup ros2 run cartographer_ros cartographer_node -configuration_directory '{cfg_dir}' -configuration_basename folkrace_2d.lua --remap scan:=/scan --remap imu:=/imu > /tmp/carto_live.log 2>&1 &", shell=True)
            subprocess.Popen("nohup ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.02 -publish_period_sec 0.066 > /tmp/grid_live.log 2>&1 &", shell=True)
        except Exception as e:
            self.get_logger().warn(f"Error resetting Cartographer: {e}")

    def _poll_udp(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
                if not data:
                    break
                packet = json.loads(data.decode('utf-8'))

                if packet.get("reset", False):
                    self._reset_cartographer()
                    break

                t_pkt = packet.get("time", 0.0)
                if hasattr(self, "_last_sim_time") and (t_pkt < self._last_sim_time - 0.5):
                    self._reset_cartographer()
                self._last_sim_time = t_pkt

                ranges = packet.get("ranges", [])
                rpy = packet.get("rpy", [0.0, 0.0, 0.0])
                gyro = packet.get("gyro", [0.0, 0.0, 0.0])
                accel = packet.get("accel", [0.0, 0.0, 0.0])
                
                if ranges:
                    self.publish_webots_lidar(ranges)
                if rpy:
                    self.publish_webots_imu(rpy, accel, gyro)
            except (BlockingIOError, socket.error):
                break
            except Exception as e:
                self.get_logger().warn(f"Error decoding live Webots UDP packet: {e}")
                break

    def publish_webots_lidar(self, range_image, fov_rad=6.2831853, min_range=0.12, max_range=5.0):
        """Publishes 360-degree LiDAR range array as LaserScan."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser"

        n_samples = len(range_image)
        if n_samples == 0:
            return

        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / float(n_samples)
        scan.time_increment = 0.0
        scan.scan_time = 0.02
        scan.range_min = float(min_range)
        scan.range_max = float(max_range)
        # Webots LiDAR sweeps clockwise from +pi to -pi; reverse array to make it CCW from -pi to +pi for ROS 2
        scan.ranges = [float(r) for r in reversed(range_image)]

        self.scan_pub.publish(scan)

    def publish_webots_imu(self, rpy, accel_xyz, gyro_xyz):
        """Publishes orientation, linear acceleration, and angular velocity to /imu."""
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "imu_link"

        # Convert Roll/Pitch/Yaw to Quaternion
        roll, pitch, yaw = rpy[0], rpy[1], rpy[2]
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        imu.orientation.w = cr * cp * cy + sr * sp * sy
        imu.orientation.x = sr * cp * cy - cr * sp * sy
        imu.orientation.y = cr * sp * cy + sr * cp * sy
        imu.orientation.z = cr * cp * sy - sr * sp * cy

        # IMU Gyro (rad/s)
        imu.angular_velocity.x = float(gyro_xyz[0])
        imu.angular_velocity.y = float(gyro_xyz[1])
        imu.angular_velocity.z = float(gyro_xyz[2])

        # Accelerometer (m/s^2)
        imu.linear_acceleration.x = float(accel_xyz[0])
        imu.linear_acceleration.y = float(accel_xyz[1])
        imu.linear_acceleration.z = float(accel_xyz[2])

        # Covariance matrices (-1 in orientation disables uncalibrated quaternion drift in Cartographer)
        imu.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        imu.angular_velocity_covariance = [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001]
        imu.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = WebotsCartographerBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
