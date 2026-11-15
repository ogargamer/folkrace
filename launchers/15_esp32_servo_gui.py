"""
================================================================================
  15. LAUNCHER: ESP32 HIGH-RATE 333Hz DIGITAL SERVO TEST BENCH GUI
================================================================================
  Connects over USB serial to the ESP32D N4 running the 333Hz servo firmware.
  Features:
    - Dedicated 333 Hz / 3.0 ms cycle timing
    - 1229 µs to 1771 µs pulse range with 1500 µs neutral calibration
    - 2 µs deadband fine-nudge testing
    - Real-time 180° visual servo dial & horn simulation
    - Automated sweep & burn-in endurance test
    - PWM power disable/enable safety control

  Usage:
    python 15_esp32_servo_gui.py
================================================================================
"""

import os
import sys
import subprocess

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ESP32_DIR = os.path.join(ROOT_DIR, "ESP32", "ESP32_Main")
SERVO_GUI_SCRIPT = os.path.join(ESP32_DIR, "servo_test_gui.py")

if __name__ == "__main__":
    if not os.path.exists(SERVO_GUI_SCRIPT):
        print(f"[Error] Servo GUI script not found at: {SERVO_GUI_SCRIPT}")
        sys.exit(1)

    cmd = [sys.executable, SERVO_GUI_SCRIPT] + sys.argv[1:]
    sys.exit(subprocess.run(cmd, cwd=ESP32_DIR).returncode)
