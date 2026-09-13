"""Quick launcher for C++ Septic Spline Live Preview Map & Real-Time Tuner."""
import os
import sys
import importlib.util

DIR = os.path.dirname(os.path.abspath(__file__))
TARGET = os.path.join(DIR, "live_preview_and_launchers", "launch_septic_live_map.py")

spec = importlib.util.spec_from_file_location("septic_live_map_launcher", TARGET)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

def launch_septic_live_gui():
    mod.launch_septic_live_gui()

if __name__ == "__main__":
    launch_septic_live_gui()
