"""
================================================================================
  5. LAUNCHER: HEADLESS GENETIC ALGORITHM PATH PLANNER OPTIMIZER
================================================================================
  Starts headless parallel GA / CMA-ES evolution for C++ SL-MPC path planners,
  evaluating candidate genomes in Webots batch mode.

  Usage:
    python 5_ga_optimizer.py
    python 5_ga_optimizer.py --algorithm CMA_ES --generations 200
================================================================================
"""

import os
import sys
import subprocess

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GA_DIR = os.path.join(ROOT_DIR, "webots", "optimization", "ga_planner_optimizer")
LAUNCHER_SCRIPT = os.path.join(GA_DIR, "launch_ga_optimization.py")

if __name__ == "__main__":
    cmd = [sys.executable, LAUNCHER_SCRIPT] + sys.argv[1:]
    sys.exit(subprocess.run(cmd, cwd=GA_DIR).returncode)
