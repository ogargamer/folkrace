"""
================================================================================
  OPTION 2: CURVILINEAR SLACK-CONSTRAINED SL-MPC (PURE C++ NATIVE ENGINE)
  - Native AVX2 C++ implementation via embedded OSQP solver
  - Dual-Trajectory Planning: Primary Racing Line + Challenger Line
  - Sub-millisecond compute budget (~2.5 ms single / ~8.0 ms dual vs ~27 ms Python)
  - Pure Python implementation has been permanently deleted
================================================================================
"""

try:
    from cpp_planner_option2_sl_mpc import CppCurvilinearSLMPC, CPP_AVAILABLE
except ImportError:
    from .cpp_planner_option2_sl_mpc import CppCurvilinearSLMPC, CPP_AVAILABLE

if not CPP_AVAILABLE:
    raise RuntimeError(
        "C++ Curvilinear SL-MPC DLL (curvilinear_sl_mpc.dll) is missing or failed to load. "
        "Python Option 2 has been deleted. "
        "Please compile or supply curvilinear_sl_mpc.dll."
    )

# CurvilinearSLMPC is strictly the native C++ engine with Dual-Solve support
CurvilinearSLMPC = CppCurvilinearSLMPC

__all__ = [
    "CurvilinearSLMPC",
    "CppCurvilinearSLMPC",
]
