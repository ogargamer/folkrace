"""Quick launcher for Anti-Wrong-Way Watchdog & 180 Auto-Correction Studio."""
import os
import sys
import importlib.util

DIR = os.path.dirname(os.path.abspath(__file__))
TARGET = os.path.join(DIR, "live_preview_and_launchers", "launch_wrong_way_studio.py")

spec = importlib.util.spec_from_file_location("studio_launcher", TARGET)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

def launch_wrong_way_studio():
    mod.launch_wrong_way_studio()

if __name__ == "__main__":
    launch_wrong_way_studio()
