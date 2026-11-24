"""
================================================================================
  LAUNCHER: STM32G474 DUAL-MOTOR 20.0 kHz SIMPLEFOC BENCH & LIVE GUI
================================================================================
  Connects over USB CDC serial (921600 baud) to the STM32G474 dual-motor FOC
  controller with MT6816 dual magnetic angle encoders on shared SPI1.
  Features real-time 26-byte binary telemetry, 14-bit angle visualizer dials,
  stepped 128-point LUT calibration, live parameter tuning, and global E-stop.

  Usage:
    python launchers/9_stm32_encoder_motor_gui.py
================================================================================
"""

import os
import sys
import subprocess

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GUI_APP = os.path.join(ROOT_DIR, "STM32", "gui", "app.py")
WORK_DIR = os.path.join(ROOT_DIR, "STM32", "gui")

if __name__ == "__main__":
    if not os.path.exists(GUI_APP):
        # Case fallback for Linux/macOS filesystems
        GUI_APP = os.path.join(ROOT_DIR, "stm32", "gui", "app.py")
        WORK_DIR = os.path.join(ROOT_DIR, "stm32", "gui")

    if not os.path.exists(GUI_APP):
        print(f"[Error] STM32 dual-motor live GUI not found at {GUI_APP}")
        sys.exit(1)

    cmd = [sys.executable, GUI_APP] + sys.argv[1:]
    sys.exit(subprocess.run(cmd, cwd=WORK_DIR).returncode)
