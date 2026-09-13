"""Quick launcher for ToF Path Planners Interactive Studio & Tuner GUI."""
import os
import sys
import importlib.util

DIR = os.path.dirname(os.path.abspath(__file__))
TARGET = os.path.join(DIR, "live_preview_and_launchers", "launch_planner_gui.py")

spec = importlib.util.spec_from_file_location("planner_gui_launcher", TARGET)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

def launch_gui():
    mod.launch_gui()

if __name__ == "__main__":
    launch_gui()
