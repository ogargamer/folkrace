"""
================================================================================
  FOLKRACE AUTONOMOUS BRAIN PACKAGE
================================================================================
  Shared single-source-of-truth vehicle config, perception, planning, and safety.
  Imported identically by Webots simulation and physical Raspberry Pi 4 runner.
================================================================================
"""

from importlib import import_module

_EXPORTS = {
    "AutonomousBrain": (".autonomous_brain", "AutonomousBrain"),
    "PoseTracker": (".pose_tracker", "PoseTracker"),
    "CppSmartHybridCostmap": (".cpp_smart_hybrid_costmap", "CppSmartHybridCostmap"),
    "SENSOR_NAMES": (".cpp_smart_hybrid_costmap", "SENSOR_NAMES"),
    "ToFGapScout": (".tof_gap_scout", "ToFGapScout"),
    "CurvilinearSLMPC": (".planner_sl_mpc", "CurvilinearSLMPC"),
    "StuckRecoveryManager": (".stuck_recovery_manager", "StuckRecoveryManager"),
    "StuckRecoveryState": (".stuck_recovery_manager", "StuckRecoveryState"),
    "AntiWrongWayWatchdog": (".anti_wrong_way_watchdog", "AntiWrongWayWatchdog"),
    "RampDetector": (".ramp_detector", "RampDetector"),
}

__all__ = list(_EXPORTS.keys())


def __getattr__(name: str):
    if name in _EXPORTS:
        mod_name, attr_name = _EXPORTS[name]
        mod = import_module(mod_name, __name__)
        val = getattr(mod, attr_name)
        globals()[name] = val
        return val
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
