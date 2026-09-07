"""
================================================================================
  LAUNCHER FOR TOF PATH PLANNERS INTERACTIVE STUDIO & TUNER GUI
================================================================================
Usage:
    python launch_planner_gui.py
"""

import os
import sys

WORKSPACE_ROOT = os.path.dirname(os.path.abspath(__file__))
PLANNERS_DIR = os.path.join(WORKSPACE_ROOT, "webots/simulation/my_project", "controllers", "tof_path_planners")
SCANNER_DIR = os.path.join(WORKSPACE_ROOT, "webots/simulation/my_project", "controllers", "tof_16ch_grid_scanner")

for p in [WORKSPACE_ROOT, PLANNERS_DIR, SCANNER_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from planner_gui import launch_gui

if __name__ == "__main__":
    launch_gui()
