 #!/usr/bin/env python3

import os

import sys

import json

import math

import time

import rclpy

from rclpy.node import Node

from rosgraph_msgs.msg import Clock

from sensor_msgs.msg import LaserScan, Imu

from nav_msgs.msg import Odometry

from geometry_msgs.msg import TransformStamped

from builtin_interfaces.msg import Time as MsgTime

from tf2_ros import TransformBroadcaster

from cartographer_ros_msgs.msg import LandmarkList, LandmarkEntry


WHEEL_RADIUS = 0.0275

MOTOR_DIR = -1.0

SIM_START_TIME = 100.0


def extract_timestamp(data_dict):

    for key in ['timestamp', 'time', 'sim_time', 't', 'stamp']:

        if key in data_dict:

            try:

                return float(data_dict[key])

            except (ValueError, TypeError):

                continue

    return None


class WebotsDataFeeder(Node):

    def __init__(self):

        super().__init__('webots_carto_feeder')

        self.use_odom = os.environ.get('USE_ODOMETRY', '0') == '1'


        self.clock_pub = self.create_publisher(Clock, '/clock', 20)

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 20)

        self.imu_pub = self.create_publisher(Imu, '/imu', 20)

        self.landmark_pub = self.create_publisher(LandmarkList, '/landmark', 20)


        if self.use_odom:

            self.odom_pub = self.create_publisher(Odometry, '/odom', 20)

            self.tf_broadcaster = TransformBroadcaster(self)


        self.x = 0.0

        self.y = 0.0

        self.yaw = 0.0

        self.last_sim_time = None

        self.landmarks_sent = 0


    def feed_data(self, jsonl_path):

        if not os.path.exists(jsonl_path):

            self.get_logger().error(f"Scan file not found: {jsonl_path}")

            return


        valid_records = []

        with open(jsonl_path, 'r', encoding='utf-8') as f:

            for line in f:

                line_str = line.strip()

                if not line_str:

                    continue

                try:

                    obj = json.loads(line_str)

                    t_val = extract_timestamp(obj)

                    if t_val is not None:

                        obj['_t'] = t_val

                        valid_records.append(obj)

                except json.JSONDecodeError:

                    continue


        if not valid_records:

            self.get_logger().error("No valid scan records found!")

            return


        total_records = len(valid_records)

        t_raw_start = valid_records[0]['_t']

        t_raw_end = valid_records[-1]['_t']

        self.get_logger().info(f"Replaying {total_records} scans ({t_raw_end - t_raw_start:.1f}s sim duration)...")


        wall_start = time.time()

        current_time = SIM_START_TIME


        for data in valid_records:

            t_sim_raw = data['_t']

            dt_sim = t_sim_raw - t_raw_start

            current_time = SIM_START_TIME + dt_sim


            # Paced at 2.0x real-time

            wall_elapsed = (time.time() - wall_start) * 2.0

            if dt_sim > wall_elapsed:

                time.sleep((dt_sim - wall_elapsed) / 2.0)


            sec = int(current_time)

            nanosec = int((current_time - sec) * 1e9)

            stamp = MsgTime(sec=sec, nanosec=nanosec)


            # 1. Clock

            clk = Clock()

            clk.clock = stamp

            self.clock_pub.publish(clk)


            # 2. IMU
            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = "base_link"
            imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            imu_msg.angular_velocity.z = float(data.get('gyro_z', data.get('yaw_rate', 0.0)))
            imu_msg.linear_acceleration.x = float(data.get('accel_x', 0.0))
            imu_msg.linear_acceleration.y = float(data.get('accel_y', 0.0))
            imu_msg.linear_acceleration.z = float(data.get('accel_z', 9.81))
            self.imu_pub.publish(imu_msg)


            # 3. Odometry (if enabled)

            if self.use_odom:

                w_rl = float(data.get('omega_RL', data.get('w_rl', 0.0)))

                w_rr = float(data.get('omega_RR', data.get('w_rr', 0.0)))

                gyro_z = float(data.get('gyro_z', data.get('yaw_rate', 0.0)))


                if self.last_sim_time is not None:

                    dt = t_sim_raw - self.last_sim_time

                    if 0.0 < dt < 0.5:

                        v_x = ((w_rl + w_rr) * WHEEL_RADIUS) / (2.0 * MOTOR_DIR)

                        d_yaw = gyro_z * dt

                        self.yaw += d_yaw

                        mid_yaw = self.yaw - (d_yaw * 0.5)

                        self.x += v_x * math.cos(mid_yaw) * dt

                        self.y += v_x * math.sin(mid_yaw) * dt


                        qz = math.sin(self.yaw * 0.5)

                        qw = math.cos(self.yaw * 0.5)


                        odom = Odometry()

                        odom.header.stamp = stamp

                        odom.header.frame_id = "odom"

                        odom.child_frame_id = "base_link"

                        odom.pose.pose.position.x = self.x

                        odom.pose.pose.position.y = self.y

                        odom.pose.pose.orientation.z = qz

                        odom.pose.pose.orientation.w = qw

                        odom.twist.twist.linear.x = v_x

                        odom.twist.twist.angular.z = gyro_z

                        self.odom_pub.publish(odom)


                        tf_msg = TransformStamped()

                        tf_msg.header.stamp = stamp

                        tf_msg.header.frame_id = "odom"

                        tf_msg.child_frame_id = "base_link"

                        tf_msg.transform.translation.x = self.x

                        tf_msg.transform.translation.y = self.y

                        tf_msg.transform.rotation.z = qz

                        tf_msg.transform.rotation.w = qw

                        self.tf_broadcaster.sendTransform(tf_msg)


            self.last_sim_time = t_sim_raw


            # 4. LaserScan
            ranges_data = data.get('ranges', data.get('scan', None))
            if ranges_data and isinstance(ranges_data, list) and len(ranges_data) > 0:
                fov = 2.0 * math.pi
                num_pts = len(ranges_data)
                scan = LaserScan()
                scan.header.stamp = stamp
                scan.header.frame_id = "base_link"
                scan.angle_min = fov / 2.0
                scan.angle_max = -fov / 2.0
                scan.angle_increment = -fov / float(num_pts - 1)
                scan.range_min = 0.08
                scan.range_max = 3.5
                scan.ranges = [float(r) for r in ranges_data]
                self.scan_pub.publish(scan)

            # 5. Landmark (Start/Finish Line Crossing)
            is_landmark = ("landmark" in data) or data.get('landmark_cross', False) or (data.get('type') == 'landmark') or data.get('finish_line', False)
            if is_landmark:
                lm_obj = data.get("landmark", data)
                lm = LandmarkEntry()
                lm.id = str(lm_obj.get("id", "start_finish_gate"))
                lm.tracking_from_landmark_transform.position.x = float(lm_obj.get("rel_x", 0.0))
                lm.tracking_from_landmark_transform.position.y = float(lm_obj.get("rel_y", 0.0))
                lm.tracking_from_landmark_transform.position.z = 0.0
                lm.tracking_from_landmark_transform.orientation.w = 1.0
                lm.translation_weight = 1e6
                lm.rotation_weight = 1e6

                l_msg = LandmarkList()
                l_msg.header.stamp = stamp
                l_msg.header.frame_id = "base_link"
                l_msg.landmarks.append(lm)
                self.landmark_pub.publish(l_msg)
                self.landmarks_sent += 1


            rclpy.spin_once(self, timeout_sec=0.0)


        self.get_logger().info(f"Scan feed complete (Landmarks Published: {self.landmarks_sent}). Finalizing pose graph...")

        for _ in range(150):

            current_time += 0.02

            sec = int(current_time)

            nanosec = int((current_time - sec) * 1e9)

            stamp = MsgTime(sec=sec, nanosec=nanosec)

           

            clk = Clock()

            clk.clock = stamp

            self.clock_pub.publish(clk)

           

            rclpy.spin_once(self, timeout_sec=0.0)

            time.sleep(0.01)


def main():

    rclpy.init()

    feeder = WebotsDataFeeder()

    jsonl_path = sys.argv[1] if len(sys.argv) > 1 else "/mnt/c/Users/nurmi/Documents/Da Winna/my_project/controllers/folkrace_drive/live_webots_slam_scans.jsonl"

    feeder.feed_data(jsonl_path)

    rclpy.shutdown()


if __name__ == '__main__':

    main() 