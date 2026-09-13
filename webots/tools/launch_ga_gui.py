"""
================================================================================
  1-CLICK LAUNCHER FOR HEADLESS GENETIC ALGORITHM DASHBOARD GUI
================================================================================
  Launches the real-time PyQt5 GA Dashboard GUI with live Matplotlib plots,
  pause/resume controls, and multi-planner tracking.
================================================================================
"""

import os
import sys
import subprocess

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
GA_DIR = os.path.join(os.path.dirname(CURR_DIR), "optimization", "ga_planner_optimizer")
GUI_SCRIPT = os.path.join(GA_DIR, "ga_dashboard_gui.py")

if __name__ == "__main__":
    cmd = [sys.executable, GUI_SCRIPT] + sys.argv[1:]
    sys.exit(subprocess.run(cmd, cwd=GA_DIR).returncode)
