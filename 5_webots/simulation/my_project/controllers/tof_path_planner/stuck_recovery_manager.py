"""
Single-source-of-truth re-export shim for brain.stuck_recovery_manager.
"""

import os
import sys

_CURR_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_CURR_DIR, "..", "..", "..", "..", ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from brain.stuck_recovery_manager import (
    StuckRecoveryManager,
    StuckRecoveryState,
    TofSummary,
    wrap_to_pi,
)

__all__ = [
    "StuckRecoveryManager",
    "StuckRecoveryState",
    "TofSummary",
    "wrap_to_pi",
]
