"""
================================================================================
  4. LAUNCHER: ESP32 MT6816 ENCODER & MOTOR CLOSED-LOOP CALIBRATION BENCH GUI
================================================================================
  Connects over USB serial to the ESP32 SimpleFOC controller to calibrate the
  MT6816 magnetic angle encoder, tune motor control, monitor KV, phase currents,
  and velocity response.

  Usage:
    python 4_esp32_encoder_motor_gui.py
================================================================================
"""

import os
import sys
import subprocess

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ESP32_SUITE = os.path.join(ROOT_DIR, "esp32", "simplefoc_mt6816_suite", "test_mt6818_ui.py")
WORK_DIR = os.path.join(ROOT_DIR, "esp32", "simplefoc_mt6816_suite")

if __name__ == "__main__":
    if not os.path.exists(ESP32_SUITE):
        print(f"[Error] ESP32 encoder test bench UI not found at {ESP32_SUITE}")
        sys.exit(1)

    cmd = [sys.executable, ESP32_SUITE] + sys.argv[1:]
    sys.exit(subprocess.run(cmd, cwd=WORK_DIR).returncode)
