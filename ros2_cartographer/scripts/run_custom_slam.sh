#!/usr/bin/env bash
source /opt/ros/humble/setup.bash

mkdir -p /home/gold/folkrace_cartographer_ws/custom_run
cp "/mnt/c/Users/nurmi/Documents/Da Winna/ros2_cartographer/config/custom_run_carto.lua" /home/gold/folkrace_cartographer_ws/custom_run/custom_run_carto.lua

pkill -9 -f cartographer_node 2>/dev/null || true
pkill -9 -f cartographer_occupancy_grid_node 2>/dev/null || true
rm -rf /dev/shm/* 2>/dev/null || true
sleep 0.5

nohup ros2 run cartographer_ros cartographer_node -configuration_directory /home/gold/folkrace_cartographer_ws/custom_run -configuration_basename custom_run_carto.lua --ros-args --remap scan:=/scan --remap imu:=/imu > /tmp/carto_custom.log 2>&1 &
nohup ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.02 -publish_period_sec 0.5 > /tmp/grid_custom.log 2>&1 &
sleep 1.5

echo '[FEEDER] Streaming pure LiDAR + IMU sensor data...'
python3 "/mnt/c/Users/nurmi/Documents/Da Winna/ros2_cartographer/scripts/feed_pure_lidar_imu.py"
sleep 1.0

echo '[RENDERER] Exporting map to PNG...'
python3 "/mnt/c/Users/nurmi/Documents/Da Winna/ros2_cartographer/scripts/export_cartographer_png.py" "/mnt/c/Users/nurmi/Documents/Da Winna/maps/cyan_champion_clean_static/carto_slam_result.png"

pkill -9 -f cartographer_node 2>/dev/null || true
pkill -9 -f cartographer_occupancy_grid_node 2>/dev/null || true
