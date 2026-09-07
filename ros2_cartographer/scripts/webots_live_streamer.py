"""
================================================================================
  WEBOTS LIVE ROS 2 SENSORS STREAMER
  Reads Webots ms200k_lidar and bno08x_imu and streams them via UDP to WSL2 ROS 2
================================================================================
"""

import json
import socket
import struct

class WebotsLiveStreamer:
    def __init__(self, host="127.0.0.1", port=9876):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target = (host, port)
        self.enabled = True

    def send_scan_and_imu(self, timestamp, ranges, rpy, gyro, accel):
        if not self.enabled or not ranges:
            return
        try:
            payload = {
                "t": round(float(timestamp), 4),
                "ranges": [round(float(r), 4) for r in ranges],
                "rpy": [round(float(x), 4) for x in rpy],
                "gyro": [round(float(x), 4) for x in gyro],
                "accel": [round(float(x), 4) for x in accel]
            }
            data = json.dumps(payload).encode("utf-8")
            self.sock.sendto(data, self.target)
        except Exception:
            pass
