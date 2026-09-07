#!/usr/bin/env python3
import sys
import os
import numpy as np

if "WEBOTS_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["WEBOTS_HOME"], "lib", "controller", "python"))

from controller import Supervisor

def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    lidar = robot.getDevice("ms200k_lidar")
    lidar.enable(int(1000 / 12))
    lidar.enablePointCloud()

    # Step a few times to get a scan at the start position (0, 0)
    for _ in range(10):
        robot.step(timestep)

    robot_node = robot.getSelf()
    pos = np.array(robot_node.getPosition())
    rot_mat = np.array(robot_node.getOrientation()).reshape(3, 3)

    cloud = lidar.getPointCloud()
    ranges = lidar.getRangeImage()

    out_file = os.path.join(os.path.dirname(__file__), "debug_scan_at_start.txt")
    with open(out_file, "w") as f:
        f.write(f"Robot Position in Webots: {pos}\n")
        f.write(f"Robot Orientation Matrix:\n{rot_mat}\n")
        f.write(f"Total cloud points: {len(cloud)}\n")
        f.write(f"Total range entries: {len(ranges)}\n\n")
        
        for i in range(0, len(cloud), 45): # Every 36 degrees
            p = cloud[i]
            p_local = np.array([p.x, p.y, p.z])
            p_global = rot_mat @ p_local + pos
            f.write(f"Ray {i:3d} (Range {ranges[i]:.3f}m): Local=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) | Global=({p_global[0]:.3f}, {p_global[2]:.3f})\n")

    robot.simulationQuit(0)

if __name__ == "__main__":
    main()
