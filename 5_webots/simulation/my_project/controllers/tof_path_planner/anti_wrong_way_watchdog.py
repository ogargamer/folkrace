"""
Single-source-of-truth re-export shim for brain.anti_wrong_way_watchdog.
"""

import os
import sys

_CURR_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_CURR_DIR, "..", "..", "..", "..", ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from brain.anti_wrong_way_watchdog import (
    ANTI_WRONG_WAY_MODE,
    AWW_BREADCRUMB_DURATION_SCALE,
    AWW_BREADCRUMB_LOOKBACK_M,
    AWW_BREADCRUMB_MAX_TIMEOUT_S,
    AWW_BREADCRUMB_SPACING_M,
    AWW_CONFIRM_TIME_S,
    AWW_EARLY_EXIT_CONE_DEG,
    AWW_EARLY_EXIT_CORRIDOR_HALF_WIDTH_M,
    AWW_EARLY_EXIT_MIN_BC_DIST_M,
    AWW_EARLY_EXIT_MIN_GAP_M,
    AWW_EARLY_EXIT_MIN_HALF_WIDTH_M,
    AWW_EARLY_EXIT_OPPOSING_DEG,
    AWW_MAX_DETECT_DIST_M,
    AWW_MIN_CONFIRMED_BREADCRUMBS,
    AWW_POST_RECOVERY_COOLDOWN_S,
    AntiWrongWayWatchdog,
    Breadcrumb,
    BreadcrumbWatchdog,
    WrongWayOptionMode,
    WrongWayStatus,
    wrap_to_pi,
)

__all__ = [
    "ANTI_WRONG_WAY_MODE",
    "AWW_BREADCRUMB_DURATION_SCALE",
    "AWW_BREADCRUMB_LOOKBACK_M",
    "AWW_BREADCRUMB_MAX_TIMEOUT_S",
    "AWW_BREADCRUMB_SPACING_M",
    "AWW_CONFIRM_TIME_S",
    "AWW_EARLY_EXIT_CONE_DEG",
    "AWW_EARLY_EXIT_CORRIDOR_HALF_WIDTH_M",
    "AWW_EARLY_EXIT_MIN_BC_DIST_M",
    "AWW_EARLY_EXIT_MIN_GAP_M",
    "AWW_EARLY_EXIT_MIN_HALF_WIDTH_M",
    "AWW_EARLY_EXIT_OPPOSING_DEG",
    "AWW_MAX_DETECT_DIST_M",
    "AWW_MIN_CONFIRMED_BREADCRUMBS",
    "AWW_POST_RECOVERY_COOLDOWN_S",
    "AntiWrongWayWatchdog",
    "Breadcrumb",
    "BreadcrumbWatchdog",
    "WrongWayOptionMode",
    "WrongWayStatus",
    "wrap_to_pi",
]
