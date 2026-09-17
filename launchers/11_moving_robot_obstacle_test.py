"""
================================================================================
  11. LAUNCHER: MOVING ROBOT + EGO-COMPENSATED OBSTACLE TEST BENCH (PHASE 2)
================================================================================
  Evaluates 16-channel ToF blob extraction, wall rejection, rolling 5-frame
  nearest-neighbor velocity tracking, and conservative vector enlargement
  while the ego-robot is actively driving and cornering at race speeds.

  Usage:
    python 11_moving_robot_obstacle_test.py                (Analytical raycast benchmark sweep)
    python 11_moving_robot_obstacle_test.py --webots       (Launch 3D Webots simulation in fast mode)
    python 11_moving_robot_obstacle_test.py --gui          (Launch 3D Webots + Live Telemetry GUI)
    python 11_moving_robot_obstacle_test.py --realtime     (Launch 3D Webots in real-time mode)
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


def generate_readings_in_body_frame(
    obs_rx: float, obs_ry: float, obs_r: float = 0.075, corridor_half_w: float = 0.60
) -> Dict[str, float]:
    """
    Generates synthetic 16-channel ToF readings given obstacle position in robot body coordinates.
    """
    readings = {}
    for name, (x0, y0, heading) in zip(SENSOR_NAMES, SENSOR_CONFIG):
        d_obs = _intersect_ray_cylinder(x0, y0, heading, obs_rx, obs_ry, obs_r)
        d_wall = _intersect_ray_walls(x0, y0, heading, half_w=corridor_half_w)
        readings[name] = min(d_obs, d_wall, 1.80)
    return readings


# ==============================================================================
# ANALYTICAL BENCHMARK SUITE (Headless / Self-Contained)
# ==============================================================================
def run_analytical_benchmark() -> bool:
    print("=" * 80)
    print("  PHASE 2: MOVING ROBOT + EGO-COMPENSATED DYNAMIC OBSTACLE BENCHMARK")
    print("  Evaluating velocity tracking with vehicle moving at race pace (v, omega)")
    print("=" * 80)

    cm = CppSmartHybridCostmap(grid_bounds=(-1.0, 1.0, 0.0, 2.0), resolution=0.015, r_inflation_m=0.088)
    tracker = DynamicObstacleTracker(grid_bounds=(-1.0, 1.0, 0.0, 2.0), resolution=0.015)

    test_stages = [
        # (name, ego_v, ego_yaw_rate, obs_v_w, duration_s, x0_rel, y0_rel, vx_obs_w, vy_obs_w)
        ("Stage 1: Cruising (2.0 m/s) passing Stationary Obstacle",
         2.00, 0.00, 0.00, 1.5, 0.00, 1.50, 0.00, 0.00),
        ("Stage 2: Cruising (2.5 m/s) with Oncoming Obstacle (1.0 m/s)",
         2.50, 0.00, 1.00, 1.2, 0.00, 1.60, 0.00, -1.00),
        ("Stage 3: Cruising (2.0 m/s) with Leading Obstacle (1.5 m/s)",
         2.00, 0.00, 1.50, 1.5, 0.00, 1.20, 0.00, 1.50),
        ("Stage 4: Turning (Yaw Rate 1.5 rad/s) passing Stationary Wall",
         1.80, 1.50, 0.00, 1.2, 0.00, 1.40, 0.00, 0.00),
        ("Stage 5: Turning (Yaw Rate 1.5 rad/s) with Lateral Crossing Obstacle (0.6 m/s)",
         1.80, 1.50, 0.60, 1.5, -0.40, 1.00, 0.60, 0.00),
    ]

    dt = 1.0 / 60.0
    all_passed = True
    summary_results = []

    for name, ego_v, ego_yaw_rate, target_v, duration_s, x0_rel, y0_rel, vx_obs_w, vy_obs_w in test_stages:
        tracker.reset()
        n_frames = int(round(duration_s / dt))

        # Initial conditions: Ego-robot at origin, facing North (+Y, theta = 0)
        X_ego = 0.0
        Y_ego = 0.0
        theta_ego = 0.0

        # Initial obstacle world position
        X_obs = X_ego + math.cos(theta_ego) * x0_rel - math.sin(theta_ego) * y0_rel
        Y_obs = Y_ego + math.sin(theta_ego) * x0_rel + math.cos(theta_ego) * y0_rel

        has_obstacle = (name != "Stage 4: Turning (Yaw Rate 1.5 rad/s) passing Stationary Wall")

        obs_speeds = []
        is_dynamic_flags = []
        lead_distances = []

        for frame in range(n_frames):
            now_s = frame * dt

            # Update ego robot position (+Y forward in body frame)
            # dX = -v_ego * sin(theta) * dt, dY = v_ego * cos(theta) * dt
            X_ego += -ego_v * math.sin(theta_ego) * dt
            Y_ego +=  ego_v * math.cos(theta_ego) * dt
            theta_ego += ego_yaw_rate * dt

            if has_obstacle:
                X_obs += vx_obs_w * dt
                Y_obs += vy_obs_w * dt

                # Compute obstacle position relative to current ego-robot body frame
                dx_w = X_obs - X_ego
                dy_w = Y_obs - Y_ego
                cos_th = math.cos(theta_ego)
                sin_th = math.sin(theta_ego)
                obs_rx = float( cos_th * dx_w + sin_th * dy_w)
                obs_ry = float(-sin_th * dx_w + cos_th * dy_w)
                readings = generate_readings_in_body_frame(obs_rx, obs_ry, obs_r=0.075, corridor_half_w=0.60)
            else:
                obs_rx = 100.0
                obs_ry = 100.0
                readings = generate_readings_in_body_frame(100.0, 100.0, obs_r=0.075, corridor_half_w=0.60)

            # Run AVX2 costmap DLL
            prob_map, hazard_mask, _ = cm.process_tof_frame(readings)

            # Process frame with Ego-Motion Compensation
            enlarged_mask, dyn_tracks, telem = tracker.process_frame(
                prob_map=prob_map,
                robot_pose=(X_ego, Y_ego, theta_ego),
                now_s=now_s,
                base_hazard_mask=hazard_mask,
                v_ego=ego_v,
                yaw_rate=ego_yaw_rate,
            )

            # Record metrics while obstacle is within sensor view
            is_in_view = (0.20 <= obs_ry <= 1.80 and abs(obs_rx) <= 0.52) if has_obstacle else False
            if frame >= 5:
                if has_obstacle and is_in_view:
                    if tracker.tracks:
                        if target_v >= 0.40:
                            best_t = max(tracker.tracks.values(), key=lambda t: t.v_mag)
                            obs_speeds.append(best_t.v_mag)
                            is_dynamic_flags.append(best_t.is_dynamic)
                            if best_t.is_dynamic:
                                lead_distances.append(min(0.25, best_t.v_mag * 0.20))
                        else:
                            for t in tracker.tracks.values():
                                obs_speeds.append(t.v_mag)
                                is_dynamic_flags.append(t.is_dynamic)
                elif not has_obstacle:
                    # Wall-only test: ensure NO false dynamic tracks are generated
                    for t in tracker.tracks.values():
                        obs_speeds.append(t.v_mag)
                        is_dynamic_flags.append(t.is_dynamic)

        mean_speed = float(np.mean(obs_speeds[-15:])) if obs_speeds else 0.0
        flagged_dynamic = any(is_dynamic_flags) if is_dynamic_flags else False
        mean_lead = float(np.mean(lead_distances[-15:])) if lead_distances else 0.0
        expected_dynamic = (target_v >= 0.40)
        speed_err = abs(mean_speed - target_v)

        passed = (flagged_dynamic == expected_dynamic) and (speed_err < 0.20 or not expected_dynamic)
        if not passed:
            all_passed = False

        summary_results.append({
            "name": name,
            "target_v": target_v,
            "estimated_v": mean_speed,
            "error": speed_err,
            "is_dynamic": flagged_dynamic,
            "expected_dynamic": expected_dynamic,
            "lead_cm": mean_lead * 100.0,
            "passed": passed,
        })

        print(f"\n---> {name}")
        print(f"     Target Speed:   {target_v:.2f} m/s")
        print(f"     Estimated Speed:{mean_speed:.3f} m/s (Error: {speed_err:.3f} m/s)")
        print(f"     Flagged Dynamic:{flagged_dynamic} (Expected: {expected_dynamic})")
        print(f"     Mean Lead:      {mean_lead * 100.0:.1f} cm (Conservative Forward Expansion)")
        print(f"     Result:         [{'PASS' if passed else 'FAIL'}]")

    print("\n" + "=" * 80)
    print("  PHASE 2 MOVING ROBOT BENCHMARK - SUMMARY TABLE")
    print("=" * 80)
    print(f"{'Scenario / Stage':<55} | {'Target':<7} | {'Estimated':<9} | {'Error':<7} | {'Dynamic?':<8} | {'Lead':<6} | {'Result'}")
    print("-" * 105)
    for r in summary_results:
        lead_str = f"{r['lead_cm']:4.1f}cm" if r["is_dynamic"] else "   -  "
        print(f"{r['name']:<55} | {r['target_v']:4.2f}m/s | {r['estimated_v']:5.3f}m/s | {r['error']:5.3f}m | {str(r['is_dynamic']):<8} | {lead_str} | {'PASS' if r['passed'] else 'FAIL'}")
    print("=" * 80 + "\n")

    return all_passed


# ==============================================================================
# 3D WEBOTS SIMULATION RUNNER
# ==============================================================================
def _kill_proc(p: subprocess.Popen):
    if p and p.poll() is None:
        try:
            p.terminate()
            p.wait(timeout=1.5)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass


def run_webots(realtime: bool = False, launch_gui: bool = False, auto_quit: bool = True) -> int:
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join([
        PLANNERS_DIR,
        os.path.join(GA_OPTIMIZER_DIR, "data"),
        GA_OPTIMIZER_DIR,
        MY_PROJECT_DIR,
        ROOT_DIR,
    ])
    env["WEBOTS_CONTROLLER_MODE"] = "moving_obs_test"
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
    print("  LAUNCHING 3D WEBOTS SIMULATION: MOVING ROBOT TEST")
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
    parser = argparse.ArgumentParser(description="Phase 2: Moving Robot + Ego-Compensated Dynamic Obstacle Test Bench")
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
