# BACKUP: Pre-ESP32-UART Google Cartographer 2D Stack

**Backup Date**: 2026-08-18  
**Previous IMU Architecture**: Direct Raspberry Pi 5 I2C Bus 1 (Address 0x4B) using Adafruit/smbus BNO08X driver.  
**LiDAR Architecture**: MS200 LiDAR over `/dev/ttyAMA4` (Pin 32 TX / Pin 33 RX) @ 115200 baud.  
**Cartographer Config**: 2D Occupancy Grid @ 2cm resolution, 2 threads, IMU tracking frame `imu_link`.

### Files Backed Up:
- `bno08x_imu_node.py`: ROS 2 Node reading direct I2C BNO08X.
- `cartographer_2d.lua`: 2D Cartographer tuning file.
- `ms200_laser_node.py`: MS200 LiDAR binary decoder & ROS 2 LaserScan publisher.
- `run_cartographer_slam.py`: Master supervisor process launcher.
- `slam_map_server.py`: WebSocket map & telemetry bridge on Port 8888.
- `static/`: HTML/CSS/JS web interface.
