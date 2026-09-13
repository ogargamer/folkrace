"""
================================================================================
  3. LAUNCHER: STANDALONE TOF PATH PLANNER & TELEMETRY TUNER GUI
================================================================================
  Launches the PyQt5 2D Telemetry & Costmap GUI listening for live UDP packets
  on port 9876. Can be launched before, during, or independently of Webots.

  Usage:
    python 3_planner_gui.py
================================================================================
"""

import os
import sys

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MY_PROJECT_DIR = os.path.join(ROOT_DIR, "webots", "simulation", "my_project")
PLANNERS_DIR = os.path.join(MY_PROJECT_DIR, "controllers", "tof_path_planners")
SCANNER_DIR = os.path.join(MY_PROJECT_DIR, "controllers", "tof_16ch_grid_scanner")

for p in [ROOT_DIR, MY_PROJECT_DIR, PLANNERS_DIR, SCANNER_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from planner_gui import launch_gui

if __name__ == "__main__":
    launch_gui()
