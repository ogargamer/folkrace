"""
Launch MT6818 SPI Encoder & Motor 1 Test Bench GUI
"""
import os
import sys
import subprocess

if __name__ == "__main__":
    script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "STM32", "test_mt6818_ui.py")
    subprocess.run([sys.executable, script_path] + sys.argv[1:])
