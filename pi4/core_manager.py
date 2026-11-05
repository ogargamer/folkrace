"""
================================================================================
  FOLKRACE PI 4: MULTI-CORE CPU AFFINITY & TASK SCHEDULING MANAGER
================================================================================
  Provides hard hardware isolation across the 4 Cortex-A72 cores of the BCM2711:
    - Core 0: Linux OS kernel, network stack, Wi-Fi, SSH, fan daemon.
    - Core 1: Deterministic Real-Time Path Planning Pipeline (C++ Costmap + SL-MPC).
    - Core 2: Telemetry Gathering, SQLite Flight Recorder & Web Debug Server.
    - Core 3: Auxiliary / I/O thread pool (available for disk sync / web compression).
================================================================================
"""

import os
import sys
import time
from typing import Set, Optional, Dict, Any


def set_core_affinity(core_ids: Set[int], process_name: str = "Process") -> bool:
    """
    Pins the current process strictly to the specified CPU core(s).
    Uses os.sched_setaffinity on Linux (available on Raspberry Pi 4).
    """
    if hasattr(os, "sched_setaffinity"):
        try:
            pid = 0  # 0 indicates calling process
            os.sched_setaffinity(pid, core_ids)
            current = os.sched_getaffinity(pid)
            print(f"[CoreManager] Successfully pinned {process_name} to CPU core(s): {current}")
            return True
        except Exception as e:
            print(f"[CoreManager] Warning: Failed to set affinity {core_ids} for {process_name}: {e}")
            return False
    else:
        # Non-Linux host (e.g. Windows workstation simulation)
        print(f"[CoreManager] Note: CPU affinity pinning not supported on {sys.platform} (simulated run)")
        return False


def get_core_affinity() -> Set[int]:
    """Returns the set of CPU cores the current process is allowed to run on."""
    if hasattr(os, "sched_getaffinity"):
        try:
            return os.sched_getaffinity(0)
        except Exception:
            pass
    return set(range(os.cpu_count() or 1))


def set_process_priority(niceness: int = -10, realtime_fifo: bool = False, priority: int = 50) -> bool:
    """
    Elevates scheduling priority of the calling process:
      - Real-time FIFO policy (SCHED_FIFO) if root / CAP_SYS_NICE permitted.
      - Standard Linux nice value (negative = higher priority).
    """
    if realtime_fifo and hasattr(os, "sched_setscheduler"):
        try:
            param = os.sched_param(priority)
            os.sched_setscheduler(0, os.SCHED_FIFO, param)
            print(f"[CoreManager] Process elevated to SCHED_FIFO with priority {priority}")
            return True
        except PermissionError:
            # Requires root or sudo or cap_sys_nice
            pass
        except Exception as e:
            print(f"[CoreManager] SCHED_FIFO configuration note: {e}")

    if hasattr(os, "nice"):
        try:
            os.nice(niceness)
            print(f"[CoreManager] Process niceness set to {niceness}")
            return True
        except Exception:
            pass

    return False


def get_system_cpu_stats() -> Dict[str, Any]:
    """
    Reads per-core CPU statistics and Pi 4 thermal readings.
    Parses /proc/stat and /sys/class/thermal directly for minimal overhead (no psutil dependency).
    """
    stats: Dict[str, Any] = {
        "cores": {},
        "temp_c": 0.0,
        "throttled": "0x0",
    }

    # 1. Thermal reading
    thermal_path = "/sys/class/thermal/thermal_zone0/temp"
    if os.path.exists(thermal_path):
        try:
            with open(thermal_path, "r", encoding="utf-8") as f:
                stats["temp_c"] = round(int(f.read().strip()) / 1000.0, 1)
        except Exception:
            pass

    # 2. Per-core CPU utilization via /proc/stat
    stat_path = "/proc/stat"
    if os.path.exists(stat_path):
        try:
            with open(stat_path, "r", encoding="utf-8") as f:
                lines = f.readlines()
            for line in lines:
                parts = line.split()
                if not parts:
                    continue
                name = parts[0]
                if name.startswith("cpu") and name != "cpu":
                    core_idx = int(name[3:])
                    user = int(parts[1])
                    nice = int(parts[2])
                    system = int(parts[3])
                    idle = int(parts[4])
                    iowait = int(parts[5]) if len(parts) > 5 else 0
                    irq = int(parts[6]) if len(parts) > 6 else 0
                    softirq = int(parts[7]) if len(parts) > 7 else 0
                    total = user + nice + system + idle + iowait + irq + softirq
                    busy = total - idle - iowait
                    stats["cores"][core_idx] = {
                        "total": total,
                        "busy": busy,
                        "idle": idle,
                    }
        except Exception:
            pass

    return stats

