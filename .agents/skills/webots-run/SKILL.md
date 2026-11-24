---
name: webots-run
description: Run and automate Webots simulations on Windows without OpenGL headless crashes, plus vehicle coordinate frame and steering sign conventions.
---

# Webots Execution & Conventions

## 1. Launching Webots on Windows (Avoid `Exit Code 3221225477`)
Webots on Windows requires an active desktop OpenGL context (`0xC0000005` `STATUS_ACCESS_VIOLATION` occurs if invoked in a pure headless/background session without DWM).

- **Always** launch in a desktop-attached session with `--mode=fast --batch`:
  ```powershell
  "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" --mode=fast --batch "5_webots/simulation/my_project/worlds/track_racing.wbt"
  ```
- **Standard launchers**:
  - `python 1_launchers/1_champion_preview.py` (3D Webots + live 2D telemetry GUI)
  - `python 1_launchers/2_fast_champion_drive.py` (ultra-lean 3D drive without GUI/UDP overhead)

## 2. Automated Benchmarks & Troubleshooting
- **Clean exit**: Use the Webots `Supervisor` API (`robot.simulationQuit(0)`) when a target duration or lap count completes.
- **Controller hangs on start**: Verify the controller name in `.wbt` matches the directory name under `5_webots/simulation/my_project/controllers/`.
- **Empty logs on exit**: Use `flush=True` in `print()` and line-buffered file opens (`buffering=1`).
- **Moving obstacles not moving**: Ensure `.wbt` defines `DEF MOVING_OBSTACLE Solid { ... }`.

## 3. Coordinate Frame & Steering Conventions
- **Robot Body Frame**: `+Y` Forward, `-Y` Backward, `+X` Right, `-X` Left, `+Z` Up.
- **Steering (`cmd_steer_rad`, `steer_deg`) & Yaw (`yaw`, `gyro_z`)**:
  - Positive (`+`) = **Steer Left / CCW**
  - Negative (`-`) = **Steer Right / CW**
  - Steering limits: ~±32.22° (±0.562 rad) virtual center (35.0° inner / 30.0° outer at 45% Ackermann; 1294 us left, 1565 us center, 1836 us right).
- **Recovery Steering Rules**:
  - Opening on **Left (`X < 0`)**: forward (`v > 0.1 m/s`) steer `+37.2°` (Left); reverse (`v <= 0.1 m/s`) steer `-37.2°` (Right).
  - Opening on **Right (`X > 0`)**: forward (`v > 0.1 m/s`) steer `-37.2°` (Right); reverse (`v <= 0.1 m/s`) steer `+37.2°` (Left).
