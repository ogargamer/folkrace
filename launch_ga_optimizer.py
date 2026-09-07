"""
================================================================================
  1-CLICK LAUNCHER FOR HEADLESS GENETIC ALGORITHM PATH PLANNER OPTIMIZATION
================================================================================
  Root-level shortcut pointing to ga_planner_optimizer suite.
================================================================================
"""

import os
import sys
import subprocess

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
GA_DIR = os.path.join(CURR_DIR, "ga_planner_optimizer")
LAUNCHER_SCRIPT = os.path.join(GA_DIR, "launch_ga_optimization.py")

if __name__ == "__main__":
    cmd = [sys.executable, LAUNCHER_SCRIPT] + sys.argv[1:]
    sys.exit(subprocess.run(cmd, cwd=GA_DIR).returncode)
