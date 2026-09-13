"""
================================================================================
  LAUNCHER FOR TOF PATH PLANNERS INTERACTIVE STUDIO & TUNER GUI
================================================================================
Usage:
    python "c++ septic/live_preview_and_launchers/launch_planner_gui.py"
"""

import os
import sys

DIR = os.path.dirname(os.path.abspath(__file__))
DIR_C_SEPTIC = DIR
while DIR_C_SEPTIC and os.path.basename(DIR_C_SEPTIC) != "c++ septic":
    parent = os.path.dirname(DIR_C_SEPTIC)
    if parent == DIR_C_SEPTIC:
        break
    DIR_C_SEPTIC = parent
WORKSPACE_ROOT = os.path.dirname(DIR_C_SEPTIC) if DIR_C_SEPTIC else os.path.dirname(DIR)

for p in [
    os.path.join(DIR_C_SEPTIC, "planners"),
    os.path.join(DIR_C_SEPTIC, "live_preview_and_launchers"),
    DIR_C_SEPTIC,
    WORKSPACE_ROOT,
    os.path.join(WORKSPACE_ROOT, "webots/simulation/my_project", "controllers", "tof_path_planners"),
]:
    if p and os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)

from planner_gui import launch_gui

if __name__ == "__main__":
    launch_gui()
