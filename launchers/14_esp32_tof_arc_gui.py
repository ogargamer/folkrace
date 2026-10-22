"""
================================================================================
  14. LAUNCHER: ESP32 DUAL VL53L8CX 8-ZONE 90° TOP-DOWN 2D RADAR GUI
================================================================================
  Connects over USB serial to the ESP32 dual ToF sensor rig, visualizes live
  fused 8-zone top-down 2D radar field-of-view across 90° (-45° to +45°):
    - 11.25° angular sectors (8 zones total)
    - ToF 1 (Left):  Column 2, Row 4 (Leftmost, Z0) to Row 1 (Center-Left, Z3)
    - ToF 2 (Right): Column 3, Row 1 (Center-Right, Z4) to Row 4 (Rightmost, Z7)
    - Proximity blind spot (<50mm -> 0mm) and open space fallback (-> 3.0m)
    - Max range: 3000 mm (3.0 m)

  Usage:
    python 14_esp32_tof_arc_gui.py
================================================================================
"""

import os
import sys
import subprocess

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ESP32_DIR = os.path.join(ROOT_DIR, "esp32")
TOF_ARC_GUI_SCRIPT = os.path.join(ESP32_DIR, "tof_arc_gui.py")

if __name__ == "__main__":
    if not os.path.exists(TOF_ARC_GUI_SCRIPT):
        print(f"[Error] ToF 8-Zone Arc GUI script not found at: {TOF_ARC_GUI_SCRIPT}")
        sys.exit(1)

    cmd = [sys.executable, TOF_ARC_GUI_SCRIPT] + sys.argv[1:]
    sys.exit(subprocess.run(cmd, cwd=ESP32_DIR).returncode)
