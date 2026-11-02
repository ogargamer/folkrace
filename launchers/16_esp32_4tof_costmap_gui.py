"""
================================================================================
  16. LAUNCHER: ESP32 QUAD VL53L8CX 2D COSTMAP & DUAL-TARGET RADAR GUI
================================================================================
  Connects over USB serial to the ESP32 quad ToF sensor rig, visualizes live
  Webots-identical 2D occupancy grid costmap:
    - Multi-Target Obstacle Stamping:
      When a zone reports 2 targets (T1 and T2), BOTH become obstacles.
    - Guarded Ray-Carving:
      Free-space clearing rays travel from the sensor origin ONLY up to the
      closest target (min(T1, T2)), protecting the shadow space behind T1.
    - C-Space Safety Margin:
      Dilation of obstacles into a real-time hazard mask.
    - Live 4-Sensor HUD and Dual-Target Zone Inspector (Real Hardware Telemetry).

  Usage:
    python 16_esp32_4tof_costmap_gui.py
================================================================================
"""

import os
import sys
import subprocess

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ESP32_DIR = os.path.join(ROOT_DIR, "ESP32", "ESP32_ToF")
TOF_COSTMAP_GUI_SCRIPT = os.path.join(ESP32_DIR, "tof_costmap_gui.py")

if __name__ == "__main__":
    if not os.path.exists(TOF_COSTMAP_GUI_SCRIPT):
        print(f"[Error] ToF Costmap GUI script not found at: {TOF_COSTMAP_GUI_SCRIPT}")
        sys.exit(1)

    cmd = [sys.executable, TOF_COSTMAP_GUI_SCRIPT] + sys.argv[1:]
    sys.exit(subprocess.run(cmd, cwd=ESP32_DIR).returncode)
