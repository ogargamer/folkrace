#!/usr/bin/env bash
source /opt/ros/humble/setup.bash

mkdir -p /home/gold/folkrace_cartographer_ws/sweep_run
cp "/mnt/c/Users/nurmi/Documents/Da Winna/webots/robotics/ros2_cartographer/config/sweep_temp.lua" /home/gold/folkrace_cartographer_ws/sweep_run/sweep_temp.lua

pkill -9 -f cartographer_node 2>/dev/null || true
pkill -9 -f cartographer_occupancy_grid_node 2>/dev/null || true
rm -rf /dev/shm/* 2>/dev/null || true
sleep 0.3

nohup ros2 run cartographer_ros cartographer_node -configuration_directory /home/gold/folkrace_cartographer_ws/sweep_run -configuration_basename sweep_temp.lua --ros-args -p use_sim_time:=true --remap scan:=/scan --remap imu:=/imu --remap landmark:=/landmark > /tmp/carto_sweep.log 2>&1 &
nohup ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.02 -publish_period_sec 0.2 --ros-args -p use_sim_time:=true > /tmp/grid_sweep.log 2>&1 &
sleep 1.2

export USE_ODOMETRY=0

python3 "/mnt/c/Users/nurmi/Documents/Da Winna/webots/robotics/ros2_cartographer/scripts/feed_pure_lidar_imu.py" "/mnt/c/Users/nurmi/Documents/Da Winna/webots/simulation/my_project/controllers/folkrace_drive/live_webots_slam_scans.jsonl"

ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}" 2>/dev/null || true

python3 "/mnt/c/Users/nurmi/Documents/Da Winna/webots/robotics/ros2_cartographer/scripts/export_cartographer_png.py" "/mnt/c/Users/nurmi/Documents/Da Winna/slam_results/map_01_15mm_motDist20mm_rotW30_loop036.png"

pkill -9 -f cartographer_node 2>/dev/null || true
pkill -9 -f cartographer_occupancy_grid_node 2>/dev/null || true
