#!/usr/bin/env bash
# ==============================================================================
# Save Live Cartographer SLAM Map to PGM and YAML
# ==============================================================================
source /opt/ros/humble/setup.bash

OUT_DIR="${1:-${HOME}/folkrace_cartographer_ws/maps}"
mkdir -p "${OUT_DIR}"

echo "[MAP SAVER] Requesting Cartographer map from /map topic..."
ros2 run nav2_map_server map_saver_cli -f "${OUT_DIR}/cartographer_live_map" --fmt png --occ 0.65 --free 0.25

echo "[MAP SAVER] Map saved to ${OUT_DIR}/cartographer_live_map.pgm / .yaml / .png"
