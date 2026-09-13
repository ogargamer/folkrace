"""
================================================================================
  LAUNCHER FOR TOF PATH PLANNERS INTERACTIVE STUDIO & TUNER GUI
================================================================================
Usage:
    python launch_planner_gui.py
"""

import os
import sys

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
WEBOTS_DIR = os.path.dirname(CURR_DIR)
WORKSPACE_ROOT = os.path.dirname(WEBOTS_DIR)
MY_PROJECT_DIR = os.path.join(WEBOTS_DIR, "simulation", "my_project")
PLANNERS_DIR = os.path.join(MY_PROJECT_DIR, "controllers", "tof_path_planner")

for p in [WORKSPACE_ROOT, WEBOTS_DIR, PLANNERS_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from planner_gui import launch_gui

if __name__ == "__main__":
    launch_gui()
