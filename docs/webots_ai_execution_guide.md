# Webots Execution & Automation Guide for AI Agents

---

## 1. The Core Issue: Background Headless Crash (`Exit Code 3221225477`)

### Symptom:
When invoking `webots.exe` directly in headless / background batch mode on Windows, the process immediately terminates with:
* **Exit Code**: `3221225477` (`0xC0000005` - `STATUS_ACCESS_VIOLATION`)

### Root Cause:
* Webots on Windows relies on the **hardware NVIDIA/DirectX OpenGL rendering context** to initialize its physics engine, robot sensor rendering pipelines (LiDAR `ms200k_lidar`, RangeFinders, 12-channel ToF sensor rays), and controller IPC bridges.
* Non-interactive background service shells and headless CLI daemon processes lack an active Windows desktop display context (DWM / Desktop Window Manager), causing OpenGL initialization to fail instantly with a memory access violation.

---

## 2. The Solution: How AI Agents Must Launch Webots

### A. Golden Rule for CLI Invocations:
1. **Never** attempt pure headless flags without an active display session.
2. Launch Webots using the standard desktop-attached process runner in fast/realtime mode:
   ```pwsh
   webots.exe --mode=fast --batch "path/to/world.wbt"
   ```
   * `--mode=fast`: Runs physics and sensor simulation as fast as CPU/GPU permits (several times faster than real-time).
   * `--batch`: Disables blocking modal dialog prompts (e.g. crash recovery or save prompts).

---

## 3. Automated Benchmark Execution Pattern (Python Wrapper)

To run automated N-second or multi-lap test sweeps autonomously from AI tool runners, use a Python supervisor execution wrapper:

```python
import os
import subprocess
import sys

# Set environment variables for the run
env = os.environ.copy()
env["WEBOTS_RUN_SECONDS"] = "15.0"
env["ENABLE_MOVING_OBSTACLE"] = "1"
env["MOVING_OBS_SPEED"] = "0.50"
env["ACTIVE_PATHFINDER"] = "MPPI"

# Webots binary path
webots_bin = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
world_path = os.path.abspath(r"webots/simulation/my_project\worlds\track_racing.wbt")

cmd = [
    webots_bin,
    "--mode=fast",
    "--batch",
    world_path
]

# Execute synchronously with interactive display pipe
process = subprocess.Popen(
    cmd,
    env=env,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    encoding="utf-8",
    errors="replace"
)

for line in process.stdout:
    print(line, end="", flush=True)

process.wait()
```

---

## 4. Supervisor-Controlled Simulation Quitting

When writing Webots controllers for automated benchmarks, use the Webots `Supervisor` API to quit cleanly once the benchmark criteria (duration or lap count) are met:

```python
from controller import Supervisor

robot = Supervisor()
# ... controller loop ...

if now >= target_duration_seconds:
    print("[BENCHMARK COMPLETE] Exiting cleanly...")
    robot.simulationQuit(0)  # Exits Webots cleanly with code 0
    sys.exit(0)
```

---

## 5. Quick Troubleshooting Checklist for AI Agents

| Issue | Cause | Corrective Action |
|---|---|---|
| **Exit Code 3221225477** | Headless/daemon execution without OpenGL context | Launch in interactive desktop session with `--mode=fast --batch`. |
| **Controller hangs on start** | Webots waiting for controller connection | Verify controller name in `.wbt` matches directory name under `controllers/`. |
| **Telemetry logs empty** | Buffered file writes on sudden exit | Use `flush=True` in `print()` and open log files with `buffering=1` (line-buffered). |
| **Moving obstacles not moving** | DEF names missing in world file | Ensure `.wbt` contains `DEF MOVING_OBSTACLE Solid { ... }`. |
