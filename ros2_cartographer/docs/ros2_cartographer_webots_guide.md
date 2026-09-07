# Comprehensive Setup Guide: ROS 2 + Google Cartographer + Webots LiDAR & IMU Integration

This guide provides the complete, end-to-end setup instructions to install ROS 2 and Google Cartographer in WSL2 (Ubuntu 22.04 LTS / Jammy or Ubuntu 24.04 LTS / Noble), build the ROS 2 Cartographer integration packages, and bridge Webots LiDAR and IMU telemetry to Cartographer for SLAM mapping and localization.

---

## 1. WSL 2 Ubuntu Installation & Initialization

From your Windows PowerShell terminal:

```powershell
# Install Ubuntu 22.04 LTS (Recommended for ROS 2 Humble) or Ubuntu 24.04 LTS (ROS 2 Jazzy)
wsl --install -d Ubuntu-22.04

# Launch Ubuntu in WSL2
wsl -d Ubuntu-22.04
```

---

## 2. ROS 2 Installation (Inside WSL2 Ubuntu)

Run the following commands inside your WSL2 terminal:

```bash
# Update package index and set locale
sudo apt update && sudo apt install -y locales curl gnupg lsb-release
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 GPG key and official repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install ROS 2 Desktop and dev tools
sudo apt update
# For Ubuntu 22.04: ROS 2 Humble | For Ubuntu 24.04: ROS 2 Jazzy
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update

# Add ROS environment setup to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

---

## 3. Google Cartographer Installation

Install Google Cartographer and the ROS 2 Cartographer packages directly via apt:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-nav2-map-server \
  ros-humble-tf2-tools \
  ros-humble-robot-state-publisher
```

---

## 4. Webots to ROS 2 Cartographer Architecture

The Webots robot has:
- **LiDAR**: `ms200k_lidar` (360 FOV, 450 points, 10-60 Hz)
- **IMU**: `bno08x_imu` (Orientation/Quaternion) + `accelerometer` + `gyro`
- **Frame Tree**: `map` -> `odom` -> `base_link` -> `laser` & `imu_link`

### Communication Pipeline:
1. **Webots Python Controller Bridge**: Connects to Webots controller API, reads `ms200k_lidar`, `gyro`, `accelerometer`, and `bno08x_imu`, and publishes ROS 2 standard topics:
   - `/scan` (`sensor_msgs/msg/LaserScan`)
   - `/imu` (`sensor_msgs/msg/Imu`)
   - `/odom` (`nav_msgs/msg/Odometry` or via TF odometry)
   - `/tf` (`tf2_msgs/msg/TFMessage`) for `base_link` -> `laser` & `imu_link`
2. **Google Cartographer Node**: Subscribes to `/scan` and `/imu`, performs 2D submap scan matching and loop closure, and outputs `/map` and optimized poses.

---

## 5. Cartographer Configuration (`folkrace_2d.lua`)

Create your Cartographer configuration file inside your ROS 2 workspace:

```lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",           -- IMU frame for high-frequency inertial tracking
  published_frame = "odom",              -- Cartographer publishes map -> odom transform
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = false,                  -- Can be enabled if wheel encoders are published
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- Real-time 2D SLAM parameters tailored for fast RC folkrace car
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 5.00
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.55

return options
```

---

## 6. Webots ROS 2 Publisher Node (`webots_ros2_cartographer_bridge.py`)

A clean bridge running in ROS 2 / Webots Python controller:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import time

class WebotsCartographerBridge(Node):
    def __init__(self):
        super().__init__('webots_cartographer_bridge')
        
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 50)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        self.publish_static_transforms()
        self.get_logger().info("Webots ROS 2 Cartographer Bridge Initialized.")

    def publish_static_transforms(self):
        # Base Link -> Laser Transform
        t_laser = TransformStamped()
        t_laser.header.stamp = self.get_clock().now().to_msg()
        t_laser.header.frame_id = "base_link"
        t_laser.child_frame_id = "laser"
        t_laser.transform.translation.x = 0.0
        t_laser.transform.translation.y = 0.01
        t_laser.transform.translation.z = 0.0725
        t_laser.transform.rotation.w = 1.0

        # Base Link -> IMU Link Transform
        t_imu = TransformStamped()
        t_imu.header.stamp = self.get_clock().now().to_msg()
        t_imu.header.frame_id = "base_link"
        t_imu.child_frame_id = "imu_link"
        t_imu.transform.translation.x = 0.0
        t_imu.transform.translation.y = -0.01
        t_imu.transform.translation.z = 0.015
        t_imu.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([t_laser, t_imu])

    def publish_lidar_scan(self, range_image, fov_rad, min_range, max_range, stamp_sec):
        scan_msg = LaserScan()
        scan_msg.header.stamp.sec = int(stamp_sec)
        scan_msg.header.stamp.nanosec = int((stamp_sec - int(stamp_sec)) * 1e9)
        scan_msg.header.frame_id = "laser"
        
        num_readings = len(range_image)
        scan_msg.angle_min = -fov_rad / 2.0
        scan_msg.angle_max = fov_rad / 2.0
        scan_msg.angle_increment = fov_rad / num_readings
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.02
        scan_msg.range_min = float(min_range)
        scan_msg.range_max = float(max_range)
        scan_msg.ranges = [float(r) for r in range_image]
        
        self.scan_pub.publish(scan_msg)

    def publish_imu(self, accel_xyz, gyro_xyz, quat_xyzw, stamp_sec):
        imu_msg = Imu()
        imu_msg.header.stamp.sec = int(stamp_sec)
        imu_msg.header.stamp.nanosec = int((stamp_sec - int(stamp_sec)) * 1e9)
        imu_msg.header.frame_id = "imu_link"
        
        imu_msg.orientation.x = float(quat_xyzw[0])
        imu_msg.orientation.y = float(quat_xyzw[1])
        imu_msg.orientation.z = float(quat_xyzw[2])
        imu_msg.orientation.w = float(quat_xyzw[3])
        
        imu_msg.angular_velocity.x = float(gyro_xyz[0])
        imu_msg.angular_velocity.y = float(gyro_xyz[1])
        imu_msg.angular_velocity.z = float(gyro_xyz[2])
        
        imu_msg.linear_acceleration.x = float(accel_xyz[0])
        imu_msg.linear_acceleration.y = float(accel_xyz[1])
        imu_msg.linear_acceleration.z = float(accel_xyz[2])
        
        self.imu_pub.publish(imu_msg)
```

---

## 7. Running SLAM with Cartographer

1. **Launch Cartographer Node**:
   ```bash
   ros2 launch cartographer_ros cartographer.launch.py \
     cartographer_config_dir:=/path/to/config \
     configuration_basename:=folkrace_2d.lua
   ```

2. **Launch Occupancy Grid Node (for standard `/map` visualization in RViz2)**:
   ```bash
   ros2 launch cartographer_ros occupancy_grid.launch.py \
     resolution:=0.05 \
     publish_period_sec:=1.0
   ```

3. **Launch RViz2**:
   ```bash
   rviz2
   ```
   - Set Fixed Frame to `map`.
   - Add **Map** display (`/map`).
   - Add **LaserScan** display (`/scan`).
   - Add **TF** display.
