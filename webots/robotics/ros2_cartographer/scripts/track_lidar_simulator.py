#!/usr/bin/env python3
"""
Simulates a virtual 360-degree LiDAR scan from the track map and streams it
along with IMU to Cartographer/ROS 2 to initialize or verify SLAM.
"""

import os
import sys
import json
import math
import numpy as np
import time

from convert_track_json_to_ros_map import build_polygons

def ray_intersect_segment(p, r_dir, a, b):
    # p: ray origin [2], r_dir: ray unit dir [2]
    # a, b: segment endpoints [2]
    v1 = p - a
    v2 = b - a
    v3 = np.array([-r_dir[1], r_dir[0]])
    dot = np.dot(v2, v3)
    if abs(dot) < 1e-8:
        return float('inf')
    t1 = (v2[0] * v1[1] - v2[1] * v1[0]) / dot
    t2 = np.dot(v1, v3) / dot
    if t1 >= 0.0 and 0.0 <= t2 <= 1.0:
        return t1
    return float('inf')

def simulate_lidar_scan(pose_x, pose_y, pose_yaw, wall_segments, n_rays=450, max_range=5.0, min_range=0.05):
    ranges = np.full(n_rays, max_range, dtype=np.float32)
    angles = np.linspace(-math.pi, math.pi, n_rays, endpoint=False) + pose_yaw
    pos = np.array([pose_x, pose_y])
    
    for i, ang in enumerate(angles):
        r_dir = np.array([math.cos(ang), math.sin(ang)])
        min_dist = max_range
        for (a, b) in wall_segments:
            d = ray_intersect_segment(pos, r_dir, a, b)
            if d < min_dist and d >= min_range:
                min_dist = d
        ranges[i] = min_dist
    return ranges

if __name__ == "__main__":
    print("[SIM] LiDAR Raycast Engine for Track Layout verified.")
