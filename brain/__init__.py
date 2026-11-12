"""
================================================================================
  FOLKRACE AUTONOMOUS BRAIN PACKAGE
================================================================================
  Shared single-source-of-truth perception, planning, and safety library.
  Imported identically by Webots simulation and physical Raspberry Pi 4 runner.
================================================================================
"""

from .autonomous_brain import AutonomousBrain
from .pose_tracker import PoseTracker
from .cpp_smart_hybrid_costmap import CppSmartHybridCostmap, SENSOR_NAMES
from .tof_gap_scout import ToFGapScout
from .planner_sl_mpc import CurvilinearSLMPC
from .stuck_recovery_manager import StuckRecoveryManager, StuckRecoveryState
from .anti_wrong_way_watchdog import AntiWrongWayWatchdog
from .ramp_detector import RampDetector

__all__ = [
    "AutonomousBrain",
    "PoseTracker",
    "CppSmartHybridCostmap",
    "SENSOR_NAMES",
    "ToFGapScout",
    "CurvilinearSLMPC",
    "StuckRecoveryManager",
    "StuckRecoveryState",
    "AntiWrongWayWatchdog",
    "RampDetector",
]

