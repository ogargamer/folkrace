#!/usr/bin/env python3
import sys
import os
import subprocess
import time

cfg_id = "r5_cfg30_native_aligned_variant"
lua_filename = f"{cfg_id}.lua"
results_dir = "/mnt/c/Users/nurmi/Documents/Da Winna/maps/cyan_champion_clean_static/sweep_results"
png_out_path = f"{results_dir}/{cfg_id}.png"

subprocess.run("pkill -9 -f cartographer; rm -rf /dev/shm/* 2>/dev/null; sleep 0.5", shell=True)

carto_cmd = f"nohup bash -c 'source /opt/ros/humble/local_setup.bash && ros2 run cartographer_ros cartographer_node -configuration_directory /home/gold/folkrace_cartographer_ws/param_sweep -configuration_basename {lua_filename} --ros-args --remap scan:=/scan --remap odom:=/odom --remap imu:=/imu' > /tmp/c30.log 2>&1 &"
grid_cmd = f"nohup bash -c 'source /opt/ros/humble/local_setup.bash && ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.02 -publish_period_sec 0.5' > /tmp/c30_grid.log 2>&1 &"

subprocess.run(carto_cmd, shell=True)
subprocess.run(grid_cmd, shell=True)
time.sleep(1.5)

feed_cmd = "bash -c 'source /opt/ros/humble/local_setup.bash && python3 /home/gold/folkrace_cartographer_ws/scripts/feed_cyberbotics_rigid_imu.py'"
subprocess.run(feed_cmd, shell=True)
time.sleep(1.0)

export_cmd = f"bash -c 'source /opt/ros/humble/local_setup.bash && python3 /home/gold/folkrace_cartographer_ws/scripts/export_cartographer_png.py \"{png_out_path}\"'"
subprocess.run(export_cmd, shell=True)

subprocess.run("pkill -9 -f cartographer; rm -rf /dev/shm/* 2>/dev/null", shell=True)
print("[DONE] cfg30 complete")
