"""
================================================================================
  STANDALONE CONTROLLER: OPTION 1 - TARGET-GUIDED BVP DYNAMIC STATE LATTICE
================================================================================
"""

import os
import sys

# Add sibling controller directories
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
MASTER_DIR = os.path.join(CONTROLLERS_DIR, "tof_path_planner")
if MASTER_DIR not in sys.path:
    sys.path.insert(0, MASTER_DIR)

from tof_path_planner import run_autonomous_controller

if __name__ == "__main__":
    run_autonomous_controller(forced_planner_type="OPTION1_BVP")
