#!/usr/bin/env bash
source /opt/ros/humble/setup.bash

# 1. Clean up any stale instances
pkill -9 -f cartographer_node 2>/dev/null || true
pkill -9 -f cartographer_occupancy_grid_node 2>/dev/null || true
pkill -9 -f webots_cartographer_bridge.py 2>/dev/null || true
rm -rf /dev/shm/* 2>/dev/null || true
sleep 0.2

echo "[WSL] Starting Cartographer SLAM Node..."
nohup ros2 run cartographer_ros cartographer_node \
    -configuration_directory "/mnt/c/Users/nurmi/Documents/Da Winna/webots/robotics/ros2_cartographer/config" \
    -configuration_basename folkrace_2d.lua \
    --remap scan:=/scan --remap imu:=/imu > /tmp/carto_live.log 2>&1 &

echo "[WSL] Starting Occupancy Grid Node (2cm resolution, 15 Hz map publishing at 0.066s period)..."
nohup ros2 run cartographer_ros cartographer_occupancy_grid_node \
    -resolution 0.02 -publish_period_sec 0.066 > /tmp/grid_live.log 2>&1 &

sleep 0.8

echo "[WSL] Starting Webots <-> ROS 2 UDP Bridge..."
exec python3 "/mnt/c/Users/nurmi/Documents/Da Winna/webots/robotics/ros2_cartographer/scripts/webots_cartographer_bridge.py"
