"""
================================================================================
  16. LAUNCHER: ESP32 4x VL53L8CX LIVE BINARY STREAM VIEWER GUI
================================================================================
  Connects over USB serial (921600 baud) to the ESP32 4x VL53L8CX sensor rig,
  decodes the 20-byte binary telemetry packets (`ToF_Binary_Packet`), and
  displays live 1x4 2D LiDAR ray distance strips, closest targets, and FPS/Hz
  for all 4 ToF sensors simultaneously.

  Usage:
    python launchers/16_esp32_4tof_gui.py
================================================================================
"""

import os
import sys
import subprocess

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ESP32_DIR = os.path.join(ROOT_DIR, "esp32", "ESP32_ToF")
TOF_4X_GUI_SCRIPT = os.path.join(ESP32_DIR, "tof_4x_gui.py")

if __name__ == "__main__":
    if not os.path.exists(TOF_4X_GUI_SCRIPT):
        print(f"[Error] 4x ToF GUI script not found at: {TOF_4X_GUI_SCRIPT}")
        sys.exit(1)

    cmd = [sys.executable, TOF_4X_GUI_SCRIPT] + sys.argv[1:]
    sys.exit(subprocess.run(cmd, cwd=ESP32_DIR).returncode)

