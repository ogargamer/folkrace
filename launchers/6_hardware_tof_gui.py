"""
================================================================================
  6. LAUNCHER: ESP32 VL53L8CX 16-CHANNEL TOF LIVE VIEWER & TUNING GUI
================================================================================
  Connects over USB serial to the ESP32 sensor rig, visualizes live 16-channel
  multizone distance matrices, targets, thresholds, and sensor health.

  Usage:
    python 6_hardware_tof_gui.py
================================================================================
"""

import os
import sys
import subprocess

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ESP32_DIR = os.path.join(ROOT_DIR, "esp32")
TOF_GUI_SCRIPT = os.path.join(ESP32_DIR, "tof_gui.py")

if __name__ == "__main__":
    if not os.path.exists(TOF_GUI_SCRIPT):
        print(f"[Error] ToF GUI script not found at: {TOF_GUI_SCRIPT}")
        sys.exit(1)

    cmd = [sys.executable, TOF_GUI_SCRIPT] + sys.argv[1:]
    sys.exit(subprocess.run(cmd, cwd=ESP32_DIR).returncode)
