"""
================================================================================
  10. LAUNCHER: STATIONARY ROBOT + DYNAMIC MOVING OBSTACLE TEST BENCH (PHASE 1)
================================================================================
  Evaluates 16-channel ToF blob extraction, wall rejection, rolling 5-frame
  nearest-neighbor velocity tracking, and conservative vector enlargement.

  Usage:
    python 10_stationary_obstacle_test.py                (Analytical ToF raycast benchmark sweep)
    python 10_stationary_obstacle_test.py --webots       (Launch 3D Webots simulation in fast mode)
    python 10_stationary_obstacle_test.py --gui          (Launch 3D Webots + Live Telemetry GUI)
    python 10_stationary_obstacle_test.py --realtime     (Launch 3D Webots in real-time mode)
================================================================================
"""

import argparse
import atexit
import math
import os
import subprocess
import sys
import time
from typing import Dict, List, Tuple

import numpy as np

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEBOTS_DIR = os.path.join(ROOT_DIR, "webots")
MY_PROJECT_DIR = os.path.join(WEBOTS_DIR, "simulation", "my_project")
WORLD_PATH = os.path.join(MY_PROJECT_DIR, "worlds", "track_racing.wbt")
PLANNERS_DIR = os.path.join(MY_PROJECT_DIR, "controllers", "tof_path_planner")
GA_OPTIMIZER_DIR = os.path.join(WEBOTS_DIR, "optimization", "ga_planner_optimizer")
WEBOTS_BIN = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

# Add planner module path
for p in [PLANNERS_DIR, MY_PROJECT_DIR, ROOT_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from dynamic_obstacle_tracker import DynamicObstacleTracker
from cpp_smart_hybrid_costmap import CppSmartHybridCostmap, SENSOR_NAMES

# 16-channel sensor rig coordinates and headings (from only_front_tofs.proto)
SENSOR_CONFIG = [
    # FL 0..7 (X = -0.070, Y = +0.095, cross-pointing Right)
    (-0.070, 0.095, 1.4987),
    (-0.070, 0.095, 1.3024),
    (-0.070, 0.095, 1.1060),
    (-0.070, 0.095, 0.9097),
    (-0.070, 0.095, 0.7134),
    (-0.070, 0.095, 0.5170),
    (-0.070, 0.095, 0.3207),
    (-0.070, 0.095, 0.1243),
    # FR 8..15 (X = +0.070, Y = +0.095, cross-pointing Left)
    ( 0.070, 0.095, 1.6428),
    ( 0.070, 0.095, 1.8391),
    ( 0.070, 0.095, 2.0356),
    ( 0.070, 0.095, 2.2318),
    ( 0.070, 0.095, 2.4282),
    ( 0.070, 0.095, 2.6245),
    ( 0.070, 0.095, 2.8208),
    ( 0.070, 0.095, 3.0172),
]


def _intersect_ray_cylinder(x0: float, y0: float, heading: float, cx: float, cy: float, radius: float) -> float:
    """Computes exact analytical ray-cylinder intersection distance."""
    dx = math.cos(heading)
    dy = math.sin(heading)
    fx = x0 - cx
    fy = y0 - cy
    a = dx * dx + dy * dy
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return 1.80
    sqrt_disc = math.sqrt(disc)
    t1 = (-b - sqrt_disc) / (2.0 * a)
    t2 = (-b + sqrt_disc) / (2.0 * a)
    hits = [t for t in (t1, t2) if 0.02 <= t <= 1.80]
    return min(hits) if hits else 1.80


def _intersect_ray_walls(x0: float, y0: float, heading: float, half_w: float = 0.60) -> float:
    """Computes distance to track corridor walls at x = -half_w and x = +half_w."""
    dx = math.cos(heading)
    t_min = 1.80
    if abs(dx) > 1e-4:
        t_left = (-half_w - x0) / dx
        if 0.02 <= t_left < t_min:
            t_min = t_left
        t_right = (half_w - x0) / dx
        if 0.02 <= t_right < t_min:
            t_min = t_right
    return t_min


def generate_readings(obs_x: float, obs_y: float, obs_r: float = 0.075) -> Dict[str, float]:
    """Generates synthetic 16-channel ToF readings matching physical Webots simulation."""
    readings = {}
    for name, (x0, y0, heading) in zip(SENSOR_NAMES, SENSOR_CONFIG):
        d_obs = _intersect_ray_cylinder(x0, y0, heading, obs_x, obs_y, obs_r)
        d_wall = _intersect_ray_walls(x0, y0, heading, half_w=0.60)
        readings[name] = min(d_obs, d_wall, 1.80)
    return readings


# ==============================================================================
# ANALYTICAL BENCHMARK SUITE (Headless / Self-Contained)
# ==============================================================================
def run_analytical_benchmark() -> bool:
    print("=" * 80)
    print("  PHASE 1: STATIONARY ROBOT + DYNAMIC MOVING OBSTACLE BENCHMARK")
    print("  Pipeline: Blob Extraction -> Wall Rejection -> 5-Frame Window -> Velocity Gate")
    print("=" * 80)

    cm = CppSmartHybridCostmap(grid_bounds=(-1.0, 1.0, 0.0, 2.0), resolution=0.015, r_inflation_m=0.088)
    tracker = DynamicObstacleTracker(grid_bounds=(-1.0, 1.0, 0.0, 2.0), resolution=0.015)

    test_stages = [
        # (name, speed, duration_s, x0, y0, vx, vy)
        ("Stage 1: Stationary Obstacle (0.0 m/s)", 0.00, 1.0, 0.00, 1.20, 0.00, 0.00),
        ("Stage 2: Slow Oncoming Obstacle (0.20 m/s)", 0.20, 1.5, 0.00, 1.40, 0.00, -0.20),
        ("Stage 3: Moderate Oncoming Obstacle (0.50 m/s)", 0.50, 1.5, 0.00, 1.50, 0.00, -0.50),
        ("Stage 4: Fast Oncoming Obstacle (0.80 m/s)", 0.80, 1.2, 0.00, 1.60, 0.00, -0.80),
        ("Stage 5: High-Speed Oncoming Obstacle (1.20 m/s)", 1.20, 0.8, 0.00, 1.70, 0.00, -1.20),
        ("Stage 6: Lateral Crossing Obstacle (0.50 m/s)", 0.50, 1.5, -0.45, 0.90, 0.50, 0.00),
    ]

    robot_pose = (0.0, 0.0, 0.0)
    dt = 1.0 / 60.0
    all_passed = True
    summary_results = []

    for name, target_v, duration_s, x0, y0, vx, vy in test_stages:
        tracker.reset()
        n_frames = int(round(duration_s / dt))

        obs_speeds = []
        is_dynamic_flags = []
        wall_blob_counts = []
        lead_distances = []

        for frame in range(n_frames):
            now_s = frame * dt
            cx = x0 + vx * now_s
            cy = y0 + vy * now_s

            readings = generate_readings(cx, cy, obs_r=0.075)
            prob_map, hazard_mask = cm.process_frame(readings)

            enlarged, dyn_tracks, telem = tracker.process_frame(
                prob_map=prob_map,
                robot_pose=robot_pose,
                now_s=now_s,
                base_hazard_mask=hazard_mask,
            )

            # Check obstacle track by proximity
            for tr in tracker.tracks.values():
                if math.hypot(tr.x_r - cx, tr.y_r - cy) < 0.25:
                    if tr.consecutive_associations >= 5:
                        obs_speeds.append(tr.v_mag)
                        is_dynamic_flags.append(tr.is_dynamic)
                        if tr.is_dynamic:
                            lead_distances.append(min(0.25, tr.v_mag * 0.20))

        mean_est_v = float(np.mean(obs_speeds)) if obs_speeds else 0.0
        expected_dynamic = (target_v >= 0.40)
        actual_dynamic = any(is_dynamic_flags) if is_dynamic_flags else False
        mean_lead_cm = float(np.mean(lead_distances) * 100.0) if lead_distances else 0.0

        err = abs(mean_est_v - target_v)
        gate_ok = (actual_dynamic == expected_dynamic)
        speed_ok = (err < 0.15) if target_v > 0.0 else True
        passed = gate_ok and speed_ok

        if not passed:
            all_passed = False

        summary_results.append({
            "name": name,
            "target_v": target_v,
            "est_v": mean_est_v,
            "err": err,
            "is_dynamic": actual_dynamic,
            "expected_dyn": expected_dynamic,
            "lead_cm": mean_lead_cm,
            "status": "PASS" if passed else "FAIL",
        })

        print(f"\n---> {name}")
        print(f"     Target Speed:   {target_v:.2f} m/s")
        print(f"     Estimated Speed:{mean_est_v:.3f} m/s (Error: {err:.3f} m/s)")
        print(f"     Flagged Dynamic:{actual_dynamic} (Expected: {expected_dynamic})")
        print(f"     Mean Lead:      {mean_lead_cm:.1f} cm (Conservative Forward Expansion)")
        print(f"     Result:         [{'PASS' if passed else 'FAIL'}]")

    print("\n" + "=" * 80)
    print("  PHASE 1 STATIONARY OBSTACLE TEST BENCH - SUMMARY TABLE")
    print("=" * 80)
    print(f"{'Scenario / Stage':<45} | {'Target':<7} | {'Estimated':<9} | {'Error':<7} | {'Dynamic?':<8} | {'Lead':<6} | {'Result'}")
    print("-" * 98)
    for r in summary_results:
        lead_str = f"{r['lead_cm']:4.1f}cm" if r['lead_cm'] > 0 else "   -  "
        print(f"{r['name']:<45} | {r['target_v']:4.2f}m/s | {r['est_v']:5.3f}m/s | {r['err']:4.3f}m | {str(r['is_dynamic']):<8} | {lead_str} | {r['status']}")
    print("=" * 80 + "\n")

    return all_passed


# ==============================================================================
# WEBOTS 3D PROCESS RUNNER
# ==============================================================================
def _kill_proc(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        if sys.platform == "win32":
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=4)
        else:
            proc.terminate()
            proc.wait(timeout=3)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


def run_webots(realtime: bool = False, launch_gui: bool = False, auto_quit: bool = True) -> int:
    if not os.path.exists(WEBOTS_BIN):
        print(f"[Error] Webots binary not found at: {WEBOTS_BIN}")
        return 1

    if not os.path.exists(WORLD_PATH):
        print(f"[Error] World file not found at: {WORLD_PATH}")
        return 1

    env = os.environ.copy()
    webots_home = r"C:\Program Files\Webots"
    webots_bin_dir = os.path.dirname(WEBOTS_BIN)
    py_dir = os.path.dirname(sys.executable)
    py_scripts = os.path.join(py_dir, "Scripts")

    env["WEBOTS_HOME"] = webots_home
    env["PATH"] = webots_bin_dir + os.pathsep + py_dir + os.pathsep + py_scripts + os.pathsep + env.get("PATH", "")
    env["PYTHONPATH"] = os.pathsep.join([
        PLANNERS_DIR,
        os.path.join(GA_OPTIMIZER_DIR, "data"),
        GA_OPTIMIZER_DIR,
        MY_PROJECT_DIR,
        ROOT_DIR,
    ])
    env["WEBOTS_CONTROLLER_MODE"] = "stationary_obs_test"
    env["WEBOTS_TELEMETRY_HZ"] = "60"
    env["WEBOTS_AUTO_QUIT"] = "1" if auto_quit else "0"
    env["PYTHONUNBUFFERED"] = "1"

    gui_proc = None
    if launch_gui:
        gui_script = os.path.join(PLANNERS_DIR, "planner_gui.py")
        if os.path.exists(gui_script):
            print("[1/2] Spawning Live Telemetry Map GUI (Port 9876, 60 Hz)...", flush=True)
            gui_proc = subprocess.Popen([sys.executable, gui_script, "--telemetry-hz", "60"], cwd=PLANNERS_DIR, env=env)
            time.sleep(0.6)

    mode_flag = "--mode=realtime" if (realtime or launch_gui) else "--mode=fast"
    cmd = [
        WEBOTS_BIN,
        "--stdout",
        "--stderr",
        "--batch",
        mode_flag,
        WORLD_PATH,
    ]

    print("=" * 80)
    print("  LAUNCHING 3D WEBOTS SIMULATION: STATIONARY ROBOT TEST")
    print(f"  Mode:   {mode_flag}")
    print(f"  World:  {WORLD_PATH}")
    print("=" * 80)

    webots_proc = subprocess.Popen(
        cmd,
        env=env,
        cwd=MY_PROJECT_DIR,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    def cleanup():
        if webots_proc and webots_proc.poll() is None:
            _kill_proc(webots_proc)
        if gui_proc and gui_proc.poll() is None:
            _kill_proc(gui_proc)

    atexit.register(cleanup)

    try:
        for line in webots_proc.stdout:
            print(line, end="", flush=True)
        webots_proc.wait()
    except KeyboardInterrupt:
        print("\n[Launcher] User interrupted. Exiting...", flush=True)
    finally:
        cleanup()

    return webots_proc.returncode if webots_proc else 0


def main():
    parser = argparse.ArgumentParser(description="Phase 1: Stationary Robot + Dynamic Moving Obstacle Test Bench")
    parser.add_argument("--webots", action="store_true", help="Launch in 3D Webots physics simulation")
    parser.add_argument("--gui", action="store_true", help="Launch 3D Webots + Live Telemetry GUI")
    parser.add_argument("--realtime", action="store_true", help="Launch 3D Webots in real-time mode")
    parser.add_argument("--no-quit", action="store_true", help="Keep Webots simulation open after test completion")

    args = parser.parse_args()

    if args.webots or args.gui or args.realtime:
        ret = run_webots(realtime=args.realtime, launch_gui=args.gui, auto_quit=not args.no_quit)
        sys.exit(ret)
    else:
        success = run_analytical_benchmark()
        sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
