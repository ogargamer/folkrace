"""
================================================================================
  LAUNCHER: ESP32 QUAD VL53L8CX 4-ZONE MULTI-TARGET TELEMETRY GUI
================================================================================
  Connects over USB serial (921600 baud) to the ESP32 ToF Hub, displaying
  complete multi-target telemetry on one screen for the 4 dedicated zones
  streamed to Raspberry Pi 4:
    - Main Target (Target 0) Distance + ST Table 4 Target Status
    - 2nd Target  (Target 1) Distance + ST Table 4 Target Status
    - Active SPAD Count & Saturation / Lens Contact Alert (1024 SPADs)
    - Resolved Navigation Distance + Target Selection Badge
    - Live 2D Vehicle Tactical Radar with 16 Ray Projections

  Usage:
    python launchers/6_esp32_tof_gui.py
================================================================================
"""

import os
import sys
import subprocess

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ESP32_DIR = os.path.join(ROOT_DIR, "ESP32", "ESP32_ToF")
TOF_GUI_SCRIPT = os.path.join(ESP32_DIR, "tof_gui.py")

if __name__ == "__main__":
    if not os.path.exists(TOF_GUI_SCRIPT):
        print(f"[Error] ToF GUI script not found at: {TOF_GUI_SCRIPT}")
        sys.exit(1)

    cmd = [sys.executable, TOF_GUI_SCRIPT] + sys.argv[1:]
    sys.exit(subprocess.run(cmd, cwd=ESP32_DIR).returncode)
