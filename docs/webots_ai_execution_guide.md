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

---

## 6. Dynamic Obstacle Avoidance & Ego-Compensation Test Suite

### Summary of Testing Phases:

| Phase | Script | Execution Type | Target / Objective |
|---|---|---|---|
| **Phase 1** | `launchers/10_stationary_obstacle_test.py` | Analytical / 3D Webots | Stationary robot baseline; tests 5 oncoming/lateral obstacle speeds with zero false alarms. |
| **Phase 2** | `launchers/11_moving_robot_obstacle_test.py` | Analytical / 3D Webots | Ego-robot cruising (up to 2.5 m/s) and turning (yaw rate 1.5 rad/s) past stationary walls and oncoming/lateral obstacles. |
| **Phase 3** | `launchers/12_phase3_live_webots.py` | Analytical / 3D Webots + GUI | Robot oscillating continuously with live interactive mode switching across 6 dynamic scenarios. |
| **Track Race** | `launchers/1_champion_preview.py` | Real-time 3D Webots + GUI | High-speed multi-car race on `track_racing.wbt` with 3 dynamic opponents. |

### Phase 3 Commands:
* **Fast Analytical Benchmark (Headless, ~1 second)**:
  ```pwsh
  python launchers/12_phase3_live_webots.py --benchmark
  ```
* **Interactive Live Simulation (Webots 3D + PyQt5 2D Radar GUI)**:
  ```pwsh
  python launchers/12_phase3_live_webots.py
  ```
* **Automated Mode Sweep (Cycles modes 1 to 6 automatically)**:
  ```pwsh
  python launchers/12_phase3_live_webots.py --auto-sweep --sweep-interval 6.0
  ```
* **Start Directly in Longitudinal or Lateral Mode**:
  ```pwsh
  python launchers/12_phase3_live_webots.py --mode 2    # Longitudinal Motion
  python launchers/12_phase3_live_webots.py --mode 3    # Lateral Crossing
  ```

### Full Track Integration & Race Deployment:
To evaluate the full autonomous racing pipeline (ToF Gap Scout + Curvilinear SL-MPC + Anti-Wrong-Way Watchdog + Dynamic Obstacle Tracker) on the full walled circuit with 3 moving opponent cars:
```pwsh
python launchers/1_champion_preview.py
```

---

## 7. Webots Coordinate Systems & Steering Sign Conventions

All controllers, C++ planners, MPC solvers, and recovery managers in this project strictly follow the standardized coordinate frame and sign conventions below:

### A. Robot Body Frame (Vehicle Local Frame)
* **$+Y$ Axis**: **Forward** (Longitudinal direction of motion)
* **$-Y$ Axis**: **Backward** (Reverse motion)
* **$+X$ Axis**: **Right** (Lateral displacement to the right of the vehicle)
* **$-X$ Axis**: **Left** (Lateral displacement to the left of the vehicle)
* **$+Z$ Axis**: **Up** (Normal to track surface)

### B. Steering Angle Sign Convention (`cmd_steer_rad`, `steer_deg`)
* **Positive ($+$)**: **Steer LEFT** (Counter-Clockwise / CCW yaw turn)
* **Negative ($-$)**: **Steer RIGHT** (Clockwise / CW yaw turn)
* **Steering Limits**: $\approx \pm 37.65^\circ$ ($\pm 0.657\text{ rad}$) virtual center limit yielding $\pm 39.9^\circ$ ($\pm 0.696\text{ rad}$) inside servo limit at 40% Ackermann.

### C. Yaw & Angular Velocity (`yaw`, `gyro_z`, `theta_g`)
* **Positive ($+$)**: **Turning Left** (CCW)
* **Negative ($-$)**: **Turning Right** (CW)

### D. Recovery Steering & Escape Arc Rules
* **Clearance Opening on the LEFT ($X < 0$)**:
  * **Forward Motion ($v > 0.1\text{ m/s}$)**: Steer **$+37.2^\circ$ (Left)** towards open space.
  * **Reverse Backing ($v \le 0.1\text{ m/s}$)**: Steer **$-37.2^\circ$ (Right)** to swing the vehicle's front nose to the Left.
* **Clearance Opening on the RIGHT ($X > 0$)**:
  * **Forward Motion ($v > 0.1\text{ m/s}$)**: Steer **$-37.2^\circ$ (Right)** towards open space.
  * **Reverse Backing ($v \le 0.1\text{ m/s}$)**: Steer **$+37.2^\circ$ (Left)** to swing the vehicle's front nose to the Right.

