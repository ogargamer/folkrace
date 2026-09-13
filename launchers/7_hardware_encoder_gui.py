"""
================================================================================
  7. LAUNCHER: MT6816 SPI ENCODER & MOTOR CLOSED-LOOP CALIBRATION BENCH GUI
================================================================================
  Connects over USB serial to calibrate the MT6816 magnetic angle encoder,
  tune SimpleFOC motor control, monitor KV, phase currents, and velocity response.

  Usage:
    python 7_hardware_encoder_gui.py             (Default: ESP32 SimpleFOC suite)
    python 7_hardware_encoder_gui.py --stm32     (Launch STM32 SimpleFOC suite)
================================================================================
"""

import os
import sys
import subprocess

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ESP32_SUITE = os.path.join(ROOT_DIR, "esp32", "simplefoc_mt6816_suite", "test_mt6818_ui.py")
STM32_SUITE = os.path.join(ROOT_DIR, "stm32", "test_mt6818_ui.py")

if __name__ == "__main__":
    if "--stm32" in sys.argv:
        sys.argv.remove("--stm32")
        script = STM32_SUITE
        work_dir = os.path.join(ROOT_DIR, "stm32")
    else:
        script = ESP32_SUITE
        work_dir = os.path.join(ROOT_DIR, "esp32", "simplefoc_mt6816_suite")

    if not os.path.exists(script):
        # Fallback if other exists
        if os.path.exists(ESP32_SUITE):
            script = ESP32_SUITE
            work_dir = os.path.join(ROOT_DIR, "esp32", "simplefoc_mt6816_suite")
        elif os.path.exists(STM32_SUITE):
            script = STM32_SUITE
            work_dir = os.path.join(ROOT_DIR, "stm32")
        else:
            print(f"[Error] Encoder test bench UI not found at {script}")
            sys.exit(1)

    cmd = [sys.executable, script] + sys.argv[1:]
    sys.exit(subprocess.run(cmd, cwd=work_dir).returncode)
