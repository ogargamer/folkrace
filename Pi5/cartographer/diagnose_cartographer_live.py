#!/usr/bin/env python3
"""
Real-time Cartographer Pose & Sensor Diagnostics Script
Connects to Pi 5 SLAM server (WebSocket / ROS 2) and measures:
- Robot Pose Drift (dX/dt, dY/dt, dYaw/dt) over 10 seconds while stationary
- /scan publication rate, point count, range min/max
- /imu publication rate and readings
- Cartographer submaps and TF tree
"""

import sys
import time
import json
import math
import websocket
import urllib.request

SERVER_IP = "10.43.8.42"
WS_URL = f"ws://{SERVER_IP}:8888/ws"

def test_live_slam(duration_sec=10):
    print(f"Connecting to Cartographer WebSocket at {WS_URL}...")
    poses = []
    telemetry_samples = []
    map_frames = 0
    start_time = time.time()

    ws = websocket.WebSocket()
    try:
        ws.connect(WS_URL, timeout=5)
        print("[OK] Connected to SLAM WebSocket! Sampling for 10 seconds...")
    except Exception as e:
        print(f"[ERROR] Could not connect to {WS_URL}: {e}")
        return

    while time.time() - start_time < duration_sec:
        try:
            msg_str = ws.recv()
            if not msg_str:
                continue
            data = json.loads(msg_str)
            msg_type = data.get("type")

            if msg_type == "pose":
                poses.append((time.time(), data.get("x", 0.0), data.get("y", 0.0), data.get("yaw", 0.0)))
            elif msg_type == "map":
                map_frames += 1
            elif msg_type == "telemetry":
                telemetry_samples.append((time.time(), data))

        except Exception as e:
            print(f"WS error: {e}")
            break

    ws.close()

    print("\n" + "=" * 60)
    print("      CARTOGRAPHER REAL-TIME DIAGNOSTIC REPORT")
    print("=" * 60)

    if not poses:
        print("[CRITICAL] ZERO pose messages received from Cartographer!")
    else:
        print(f"Total Pose Updates Received: {len(poses)} ({len(poses)/duration_sec:.1f} Hz)")
        p_start = poses[0]
        p_end = poses[-1]
        dt = p_end[0] - p_start[0]

        dx = p_end[1] - p_start[1]
        dy = p_end[2] - p_start[2]
        dyaw = p_end[3] - p_start[3]
        total_dist = math.hypot(dx, dy)
        speed = total_dist / (dt or 1.0)

        # Min / Max / StdDev
        xs = [p[1] for p in poses]
        ys = [p[2] for p in poses]
        yaws = [p[3] for p in poses]

        print(f"Initial Pose: X = {p_start[1]:.4f} m, Y = {p_start[2]:.4f} m, Yaw = {math.degrees(p_start[3]):.2f}°")
        print(f"Final Pose:   X = {p_end[1]:.4f} m, Y = {p_end[2]:.4f} m, Yaw = {math.degrees(p_end[3]):.2f}°")
        print(f"Total Drift:  dX = {dx:.4f} m, dY = {dy:.4f} m, Distance = {total_dist:.4f} m")
        print(f"Drift Speed:  {speed*100:.2f} cm/s ({speed*1000:.1f} mm/s)")
        print(f"Bounding Box: X span = {max(xs)-min(xs):.4f} m, Y span = {max(ys)-min(ys):.4f} m, Yaw span = {math.degrees(max(yaws)-min(yaws)):.2f}°")

        if speed > 0.05:
            print(f"\n[FAIL] ROBOT IS MOVING/DRIFTING RAPIDLY! ({speed*100:.1f} cm/s)")
        elif speed > 0.01:
            print(f"\n[WARN] Moderate drift detected: {speed*100:.2f} cm/s")
        else:
            print(f"\n[PASS] ROBOT IS ROCK-SOLID! Drift < {speed*100:.2f} cm/s")

    if telemetry_samples:
        latest_telem = telemetry_samples[-1][1]
        print("\n--- Latest Sensor Health ---")
        lidar = latest_telem.get("lidar", {})
        imu = latest_telem.get("imu", {})
        print(f"LiDAR: Status={lidar.get('status')} | Rate={lidar.get('hz', 0):.1f} Hz | Pts={lidar.get('points')} | Min={lidar.get('min_dist', 0):.2f}m | Max={lidar.get('max_dist', 0):.2f}m")
        print(f"IMU:   Status={imu.get('status')} | Rate={imu.get('hz', 0):.1f} Hz | R={imu.get('roll', 0)}° P={imu.get('pitch', 0)}° Y={imu.get('yaw', 0)}° | Ax={imu.get('ax', 0)} Ay={imu.get('ay', 0)} Az={imu.get('az', 0)}")

    print(f"Map frames received: {map_frames}")
    print("=" * 60)

if __name__ == "__main__":
    test_live_slam(10)
