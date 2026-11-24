#!/usr/bin/env bash
# ==============================================================================
# Launch Google Cartographer SLAM Node with Webots live UDP Bridge
# ==============================================================================
source /opt/ros/humble/setup.bash

CONFIG_DIR="${HOME}/folkrace_cartographer_ws/config"
SCRIPTS_DIR="${HOME}/folkrace_cartographer_ws/scripts"

echo "=============================================================================="
echo "  STARTING GOOGLE CARTOGRAPHER 2D SLAM (ROS 2 HUMBLE) + WEBOTS LIVE BRIDGE"
echo "=============================================================================="

# 1. Start Webots Live UDP -> ROS 2 Bridge in background
python3 "${SCRIPTS_DIR}/webots_cartographer_bridge.py" &
BRIDGE_PID=$!
echo "[LAUNCH] Webots UDP Bridge started (PID: ${BRIDGE_PID})."

# 2. Start Cartographer Node
ros2 run cartographer_ros cartographer_node \
    -configuration_directory "${CONFIG_DIR}" \
    -configuration_basename folkrace_2d.lua \
    --ros-args \
    --remap scan:=/scan \
    --remap imu:=/imu &
CARTO_PID=$!
echo "[LAUNCH] Cartographer Node started (PID: ${CARTO_PID})."

# 3. Start Cartographer Occupancy Grid Node
ros2 run cartographer_ros cartographer_occupancy_grid_node \
    -resolution 0.02 \
    -publish_period_sec 0.5 &
GRID_PID=$!
echo "[LAUNCH] Occupancy Grid Node started (PID: ${GRID_PID})."

echo "[LAUNCH] All Cartographer nodes running. Press Ctrl+C to stop."
wait
