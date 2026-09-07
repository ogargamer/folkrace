"""
Launch VL53L8CX Multizone Time-of-Flight Live Engine & Tuning Suite GUI
"""
import os
import sys
import subprocess

if __name__ == "__main__":
    script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ESP32", "tof_gui.py")
    subprocess.run([sys.executable, script_path] + sys.argv[1:])
