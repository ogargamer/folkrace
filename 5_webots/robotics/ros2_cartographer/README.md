# ROS 2 & Google Cartographer SLAM (Experimental / Simulation Mapping)

> [!NOTE]
> **Hardware Context**: This module serves as an experimental simulation reference for 2D SLAM and offline track map generation. The physical Folkrace racing vehicle operates with **zero LiDAR**, relying exclusively on a solid-state quad VL53L8CX Time-of-Flight array and Curvilinear SL-MPC.

This package provides an integration between Webots simulation, ROS 2 Humble, and Google Cartographer 2D SLAM.

---

## Directory Structure

```
5_webots/robotics/ros2_cartographer/
├── config/
│   └── folkrace_2d.lua                  # Cartographer 2D configuration (20mm resolution, 6x6m arena)
├── docs/
│   └── 5_webots/robotics/ros2_cartographer_webots_guide.md# Complete installation, topic mappings & architecture guide
├── maps/
│   ├── folkrace_track_map.pgm           # 20mm Occupancy grid map generated from Track JSON
│   └── folkrace_track_map.yaml          # Nav2 Map Server metadata
├── scripts/
│   ├── install_webots/robotics/ros2_cartographer_wsl.sh # One-click setup script for ROS 2 & Cartographer in WSL2
│   ├── launch_cartographer.sh           # Cartographer SLAM launcher script
│   ├── webots_cartographer_bridge.py    # Python bridge streaming /scan, /imu & static TF
│   ├── convert_track_json_to_ros_map.py # JSON track layout to ROS Occupancy Grid generator
│   └── track_lidar_simulator.py         # Virtual LiDAR raycast engine
└── sim_validation/
    ├── run_solid_cartographer_test.py   # Vectorized SLAM engine with 100% opaque solid walls
    ├── run_fast_cartographer_test.py    # High-speed SLAM reconstructor
    └── run_cartographer_on_backup_champion.py # SLAM runner for backup champion tracks
```

---

## Quick Usage

### 1. Launch Cartographer in WSL2:
```bash
~/folkrace_cartographer_ws/launch_cartographer.sh
```

### 2. Stream LiDAR & IMU from Webots:
Run the bridge script or link `WebotsCartographerBridgeNode` directly in your Webots Python controller.

### 3. Generate a Map from any Track JSON:
```bash
python 5_webots/robotics/ros2_cartographer/scripts/convert_track_json_to_ros_map.py "<path_to_track.json>" "5_webots/robotics/ros2_cartographer/maps"
```
