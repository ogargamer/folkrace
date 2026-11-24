#!/usr/bin/env python3
"""
================================================================================
  LAUNCHER: ESP32 BNO086 9-DOF IMU 3D MISSION DASHBOARD
================================================================================
  Connects over USB serial to the ESP32-S3 Master running the BNO086 SPI driver.
  Features:
    - 3D Attitude Orientation (Three.js SLERP filtered)
    - Artificial Horizon & Heading Compass Gauges
    - G-Force Vector Accelerometer Gauge
    - Live 9-DOF Euler / Quaternion / Accel / Gyro telemetry matrix
    - Real-Time Rolling Waveform Oscilloscope (Roll, Pitch, Yaw)
    - Hardware Tare / Zero Yaw calibration

  Usage:
    python launchers/7_esp32_imu_gui.py
================================================================================
"""

import os
import sys
import subprocess

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMU_GUI_DIR = os.path.join(ROOT_DIR, "ESP32", "ESP32_Main", "imu_gui")
IMU_SERVER_SCRIPT = os.path.join(IMU_GUI_DIR, "imu_server.py")

if __name__ == "__main__":
    if not os.path.exists(IMU_SERVER_SCRIPT):
        print(f"[Error] IMU Server script not found at: {IMU_SERVER_SCRIPT}")
        sys.exit(1)

    cmd = [sys.executable, IMU_SERVER_SCRIPT] + sys.argv[1:]
    sys.exit(subprocess.run(cmd, cwd=IMU_GUI_DIR).returncode)

