"""
================================================================================
  2. LAUNCHER: REAL-TIME GENETIC ALGORITHM PERFORMANCE DASHBOARD
================================================================================
  Launches the real-time PyQt5 GA Dashboard GUI with live Matplotlib plots,
  pause/resume controls, and multi-generation tracking.

  Usage:
    python 2_ga_dashboard.py
================================================================================
"""

import os
import sys
import subprocess

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GA_DIR = os.path.join(ROOT_DIR, "webots", "optimization", "ga_planner_optimizer")
GUI_SCRIPT = os.path.join(GA_DIR, "ga_dashboard_gui.py")

if __name__ == "__main__":
    cmd = [sys.executable, GUI_SCRIPT] + sys.argv[1:]
    sys.exit(subprocess.run(cmd, cwd=GA_DIR).returncode)
