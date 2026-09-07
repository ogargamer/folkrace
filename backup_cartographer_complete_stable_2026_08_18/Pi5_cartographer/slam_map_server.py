#!/usr/bin/env python3
"""
High-Performance Real-Time 2D SLAM Map & Sensor Diagnostics Server
Streams OccupancyGrid, Robot Pose, LiDAR Scan, and Live IMU/LiDAR Health Telemetry.
"""

import os
import json
import time
import base64
import threading
import math
import struct
import tornado.ioloop
import tornado.web
import tornado.websocket
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, Imu
from tf2_msgs.msg import TFMessage

# Global State
map_data = {
    "width": 0,
    "height": 0,
    "resolution": 0.02,
    "origin_x": 0.0,
    "origin_y": 0.0,
    "grid_b64": "",
    "updated": False
}

robot_pose = {
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0,
    "path": []
}

latest_scan = {
    "points": []
}

# Sensor Diagnostics & Health State
sensor_diag = {
    "lidar": {
        "status": "offline",
        "hz": 0.0,
        "points": 0,
        "min_dist": 0.0,
        "max_dist": 0.0,
        "last_seen_ts": 0.0,
        "_pkt_count": 0,
        "_last_calc_ts": time.time(),
        "_last_calc_count": 0
    },
    "imu": {
        "status": "offline",
        "hz": 0.0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "ax": 0.0,
        "ay": 0.0,
        "az": 0.0,
        "gx": 0.0,
        "gy": 0.0,
        "gz": 0.0,
        "last_seen_ts": 0.0,
        "_pkt_count": 0,
        "_last_calc_ts": time.time(),
        "_last_calc_count": 0
    }
}

active_clients = set()
main_ioloop = None
state_lock = threading.Lock()

class SLAMMapROSNode(Node):
    def __init__(self):
        super().__init__('slam_map_bridge_node')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 20)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 100)
        self.get_logger().info('SLAM Map Bridge Node started with Full LiDAR & IMU Telemetry Diagnostics!')

    def map_callback(self, msg: OccupancyGrid):
        global map_data, active_clients, main_ioloop
        raw_bytes = bytes([b & 0xFF for b in msg.data])
        b64_str = base64.b64encode(raw_bytes).decode('ascii')

        with state_lock:
            map_data["width"] = msg.info.width
            map_data["height"] = msg.info.height
            map_data["resolution"] = round(msg.info.resolution, 3)
            map_data["origin_x"] = round(msg.info.origin.position.x, 3)
            map_data["origin_y"] = round(msg.info.origin.position.y, 3)
            map_data["grid_b64"] = b64_str
            map_data["updated"] = True

        if active_clients and main_ioloop:
            payload = {
                "type": "map_b64",
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": msg.info.resolution,
                "origin_x": msg.info.origin.position.x,
                "origin_y": msg.info.origin.position.y,
                "grid_b64": b64_str
            }
            msg_str = json.dumps(payload)
            for c in list(active_clients):
                try:
                    main_ioloop.add_callback(c.write_message, msg_str)
                except Exception:
                    pass

    def scan_callback(self, msg: LaserScan):
        global latest_scan, sensor_diag, active_clients, main_ioloop, robot_pose
        now = time.time()
        pts = []
        valid_dists = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max and not math.isinf(r) and not math.isnan(r):
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                pts.append([round(lx, 2), round(ly, 2)])
                valid_dists.append(r)
            angle += msg.angle_increment

        with state_lock:
            latest_scan["points"] = pts
            sensor_diag["lidar"]["points"] = len(pts)
            sensor_diag["lidar"]["min_dist"] = round(min(valid_dists), 2) if valid_dists else 0.0
            sensor_diag["lidar"]["max_dist"] = round(max(valid_dists), 2) if valid_dists else 0.0
            sensor_diag["lidar"]["last_seen_ts"] = now
            sensor_diag["lidar"]["_pkt_count"] += 1

        if active_clients and main_ioloop and pts:
            payload = {
                "type": "scan",
                "points": pts
            }
            msg_str = json.dumps(payload)
            for c in list(active_clients):
                try:
                    main_ioloop.add_callback(c.write_message, msg_str)
                except Exception:
                    pass

    def imu_callback(self, msg: Imu):
        global sensor_diag
        now = time.time()
        q = msg.orientation
        
        # Quaternion to Euler (Roll, Pitch, Yaw in degrees)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll_deg = math.atan2(sinr_cosp, cosr_cosp) * 57.2958

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch_deg = math.copysign(90.0, sinp)
        else:
            pitch_deg = math.asin(sinp) * 57.2958

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg = math.atan2(siny_cosp, cosy_cosp) * 57.2958

        with state_lock:
            sensor_diag["imu"]["roll"] = round(roll_deg, 1)
            sensor_diag["imu"]["pitch"] = round(pitch_deg, 1)
            sensor_diag["imu"]["yaw"] = round(yaw_deg, 1)
            sensor_diag["imu"]["ax"] = round(msg.linear_acceleration.x, 2)
            sensor_diag["imu"]["ay"] = round(msg.linear_acceleration.y, 2)
            sensor_diag["imu"]["az"] = round(msg.linear_acceleration.z, 2)
            # rad/s to deg/s
            sensor_diag["imu"]["gx"] = round(msg.angular_velocity.x * 57.2958, 1)
            sensor_diag["imu"]["gy"] = round(msg.angular_velocity.y * 57.2958, 1)
            sensor_diag["imu"]["gz"] = round(msg.angular_velocity.z * 57.2958, 1)
            sensor_diag["imu"]["last_seen_ts"] = now
            sensor_diag["imu"]["_pkt_count"] += 1

    def tf_callback(self, msg: TFMessage):
        global robot_pose, active_clients, main_ioloop
        for transform in msg.transforms:
            if transform.header.frame_id in ['map', 'odom'] and transform.child_frame_id == 'base_link':
                t = transform.transform.translation
                r = transform.transform.rotation
                
                siny_cosp = 2.0 * (r.w * r.z + r.x * r.y)
                cosy_cosp = 1.0 - 2.0 * (r.y * r.y + r.z * r.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)

                with state_lock:
                    robot_pose["x"] = round(t.x, 3)
                    robot_pose["y"] = round(t.y, 3)
                    robot_pose["yaw"] = round(yaw, 3)
                    
                    if not robot_pose["path"] or math.hypot(t.x - robot_pose["path"][-1][0], t.y - robot_pose["path"][-1][1]) > 0.03:
                        robot_pose["path"].append([round(t.x, 3), round(t.y, 3)])
                        if len(robot_pose["path"]) > 2000:
                            robot_pose["path"].pop(0)

                if active_clients and main_ioloop:
                    payload = {
                        "type": "pose",
                        "x": robot_pose["x"],
                        "y": robot_pose["y"],
                        "yaw": robot_pose["yaw"],
                        "path": robot_pose["path"]
                    }
                    msg_str = json.dumps(payload)
                    for c in list(active_clients):
                        try:
                            main_ioloop.add_callback(c.write_message, msg_str)
                        except Exception:
                            pass

def broadcast_telemetry():
    global sensor_diag, active_clients, main_ioloop
    if not active_clients or not main_ioloop:
        return

    now = time.time()
    with state_lock:
        # Calculate LiDAR rate
        dt_lidar = now - sensor_diag["lidar"]["_last_calc_ts"]
        if dt_lidar >= 0.5:
            d_count = sensor_diag["lidar"]["_pkt_count"] - sensor_diag["lidar"]["_last_calc_count"]
            sensor_diag["lidar"]["hz"] = round(d_count / dt_lidar, 1)
            sensor_diag["lidar"]["_last_calc_ts"] = now
            sensor_diag["lidar"]["_last_calc_count"] = sensor_diag["lidar"]["_pkt_count"]

        # Calculate IMU rate with smooth EMA filter
        dt_imu = now - sensor_diag["imu"]["_last_calc_ts"]
        if dt_imu >= 0.5:
            d_count_imu = sensor_diag["imu"]["_pkt_count"] - sensor_diag["imu"]["_last_calc_count"]
            inst_hz = d_count_imu / dt_imu
            if sensor_diag["imu"]["hz"] == 0.0:
                sensor_diag["imu"]["hz"] = round(inst_hz, 1)
            else:
                sensor_diag["imu"]["hz"] = round(0.7 * sensor_diag["imu"]["hz"] + 0.3 * inst_hz, 1)
            sensor_diag["imu"]["_last_calc_ts"] = now
            sensor_diag["imu"]["_last_calc_count"] = sensor_diag["imu"]["_pkt_count"]

        # Determine health status
        lidar_age = now - sensor_diag["lidar"]["last_seen_ts"] if sensor_diag["lidar"]["last_seen_ts"] > 0 else 999.0
        imu_age = now - sensor_diag["imu"]["last_seen_ts"] if sensor_diag["imu"]["last_seen_ts"] > 0 else 999.0

        lidar_status = "OK" if lidar_age < 1.0 else ("TIMEOUT" if sensor_diag["lidar"]["last_seen_ts"] > 0 else "OFFLINE")
        imu_status = "OK" if imu_age < 0.5 else ("TIMEOUT" if sensor_diag["imu"]["last_seen_ts"] > 0 else "OFFLINE")

        payload = {
            "type": "telemetry",
            "lidar": {
                "status": lidar_status,
                "hz": sensor_diag["lidar"]["hz"],
                "points": sensor_diag["lidar"]["points"],
                "min_dist": sensor_diag["lidar"]["min_dist"],
                "max_dist": sensor_diag["lidar"]["max_dist"],
                "age_s": round(lidar_age, 1) if lidar_age < 900 else None
            },
            "imu": {
                "status": imu_status,
                "hz": sensor_diag["imu"]["hz"],
                "roll": sensor_diag["imu"]["roll"],
                "pitch": sensor_diag["imu"]["pitch"],
                "yaw": sensor_diag["imu"]["yaw"],
                "ax": sensor_diag["imu"]["ax"],
                "ay": sensor_diag["imu"]["ay"],
                "az": sensor_diag["imu"]["az"],
                "gx": sensor_diag["imu"]["gx"],
                "gy": sensor_diag["imu"]["gy"],
                "gz": sensor_diag["imu"]["gz"],
                "age_s": round(imu_age, 1) if imu_age < 900 else None
            }
        }

    msg_str = json.dumps(payload)
    for c in list(active_clients):
        try:
            main_ioloop.add_callback(c.write_message, msg_str)
        except Exception:
            pass

def ros_spin_worker():
    rclpy.init()
    node = SLAMMapROSNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except Exception:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def restart_cartographer_nodes():
    os.system("pkill -9 -f cartographer_node 2>/dev/null || true")
    os.system("pkill -9 -f cartographer_occupancy_grid_node 2>/dev/null || true")
    time.sleep(0.4)
    cmd1 = "source /opt/ros/jazzy/setup.bash && ros2 run cartographer_ros cartographer_node -configuration_directory /home/admin/cartographer -configuration_basename cartographer_2d.lua --ros-args -r scan:=/scan -r imu:=/imu"
    cmd2 = "source /opt/ros/jazzy/setup.bash && ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.02 -publish_period_sec 0.5"
    import subprocess
    subprocess.Popen(["bash", "-c", cmd1])
    subprocess.Popen(["bash", "-c", cmd2])

class ResetHandler(tornado.web.RequestHandler):
    def set_default_headers(self):
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Methods", "POST, GET, OPTIONS")
        self.set_header("Access-Control-Allow-Headers", "Content-Type")

    def options(self):
        self.set_status(204)
        self.finish()

    def post(self):
        global map_data, robot_pose, latest_scan, active_clients, main_ioloop
        with state_lock:
            map_data["width"] = 0
            map_data["height"] = 0
            map_data["grid_b64"] = ""
            map_data["updated"] = False
            robot_pose["x"] = 0.0
            robot_pose["y"] = 0.0
            robot_pose["yaw"] = 0.0
            robot_pose["path"] = []
            latest_scan["points"] = []

        if active_clients and main_ioloop:
            msg_str = json.dumps({"type": "reset"})
            for c in list(active_clients):
                try:
                    main_ioloop.add_callback(c.write_message, msg_str)
                except Exception:
                    pass

        threading.Thread(target=restart_cartographer_nodes, daemon=True).start()
        self.write(json.dumps({"status": "ok", "message": "Cartographer 2D map reset successfully"}))

class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("static/slam_view.html")

class MapWebSocketHandler(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        active_clients.add(self)
        self.set_nodelay(True)
        with state_lock:
            if map_data["updated"]:
                self.write_message(json.dumps({
                    "type": "map_b64",
                    "width": map_data["width"],
                    "height": map_data["height"],
                    "resolution": map_data["resolution"],
                    "origin_x": map_data["origin_x"],
                    "origin_y": map_data["origin_y"],
                    "grid_b64": map_data["grid_b64"]
                }))
            self.write_message(json.dumps({
                "type": "pose",
                "x": robot_pose["x"],
                "y": robot_pose["y"],
                "yaw": robot_pose["yaw"],
                "path": robot_pose["path"]
            }))

    def on_message(self, message):
        try:
            d = json.loads(message)
            if d.get("action") == "reset_map":
                ResetHandler.post(self)
        except Exception:
            pass

    def on_close(self):
        active_clients.discard(self)

def make_app():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    static_dir = os.path.join(base_dir, "static")
    return tornado.web.Application([
        (r"/", MainHandler),
        (r"/api/reset_map", ResetHandler),
        (r"/ws", MapWebSocketHandler),
        (r"/static/(.*)", tornado.web.StaticFileHandler, {"path": static_dir}),
    ], template_path=base_dir, websocket_max_message_size=10485760)

if __name__ == "__main__":
    PORT = 8888
    app = make_app()
    app.listen(PORT, address="0.0.0.0")
    main_ioloop = tornado.ioloop.IOLoop.current()

    # Telemetry periodic broadcaster (5Hz)
    tornado.ioloop.PeriodicCallback(broadcast_telemetry, 200).start()

    # Start ROS 2 listener thread
    threading.Thread(target=ros_spin_worker, daemon=True).start()

    print(f"================================================================")
    print(f"  GOOGLE CARTOGRAPHER 2D SLAM VIEWER AT: http://10.43.8.42:{PORT}")
    print(f"================================================================")
    main_ioloop.start()
