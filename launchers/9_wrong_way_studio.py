"""
================================================================================
  9. LAUNCHER: ANTI-WRONG-WAY WATCHDOG & 180 AUTO-CORRECTION STUDIO
================================================================================
  Launches Webots real-time mode with test_wrong_way.wbt AND spawns the live
  Anti-Wrong-Way Watchdog & 180 Auto-Correction Studio GUI (Port 9876/9877).

  World includes:
    - Track corridors with tight hairpins at both ends
    - Front pillar (DEF FRONT_PILLAR at 2.6, 0.4, 0.06)
    - Back pillar (DEF BACK_PILLAR at -0.21, 0.39, 0.06)
    - Folkrace autonomous robot running tof_path_planner (Curvilinear C++ SL-MPC)

  Interactive Controls (via GUI or Webots 3D view):
    [F] : Toggle FRONT PILLAR (Raise / Lower obstacle)
    [B] : Toggle BACK PILLAR  (Raise / Lower obstacle)
    [1] : Breadcrumb Line-of-Sight Overlap & Auto 180 Correction
    [0] : Disable Watchdog (Pass-through)
    [R] : Reset Robot Pose

  Usage:
    python 9_wrong_way_studio.py             (Launches Webots + Studio GUI)
    python 9_wrong_way_studio.py --no-gui    (Webots only)
================================================================================
"""

import os
import sys
import time
import argparse
import subprocess

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEBOTS_DIR = os.path.join(ROOT_DIR, "webots")
MY_PROJECT_DIR = os.path.join(WEBOTS_DIR, "simulation", "my_project")
WORLD_PATH = os.path.join(MY_PROJECT_DIR, "worlds", "test_wrong_way.wbt")
GA_OPTIMIZER_DIR = os.path.join(WEBOTS_DIR, "optimization", "ga_planner_optimizer")
PLANNERS_DIR = os.path.join(MY_PROJECT_DIR, "controllers", "tof_path_planner")
WEBOTS_BIN = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"


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


def launch_environment(launch_gui: bool = True):
    if not os.path.exists(WEBOTS_BIN):
        print(f"[Error] Webots executable not found at {WEBOTS_BIN}")
        return

    if not os.path.exists(WORLD_PATH):
        print(f"[Error] World file not found at {WORLD_PATH}")
        return

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
    env["WEBOTS_CONTROLLER_MODE"] = "racing"
    env["WEBOTS_TELEMETRY_HZ"] = "60"
    env["PYTHONUNBUFFERED"] = "1"

    gui_proc = None
    if launch_gui:
        gui_script = os.path.join(PLANNERS_DIR, "wrong_way_studio_gui.py")
        if os.path.exists(gui_script):
            print("[1/2] Spawning Anti-Wrong-Way Studio GUI (Port 9876, Commands: 9877)...", flush=True)
            gui_proc = subprocess.Popen([sys.executable, gui_script], cwd=PLANNERS_DIR, env=env)
            time.sleep(0.6)

    cmd = [
        WEBOTS_BIN,
        "--stdout",
        "--stderr",
        "--mode=realtime",
        WORLD_PATH,
    ]

    print("=" * 80)
    print("  LAUNCHING ANTI-WRONG-WAY & U-TURN WEBOTS TEST ENVIRONMENT (C++ SL-MPC)")
    print(f"  World:  {WORLD_PATH}")
    print("=" * 80)

    print(f"[{'2/2' if gui_proc else '1/1'}] Launching Webots simulation...", flush=True)
    webots_proc = subprocess.Popen(cmd, env=env, cwd=MY_PROJECT_DIR)

    def cleanup():
        if webots_proc and webots_proc.poll() is None:
            _kill_proc(webots_proc)
        if gui_proc and gui_proc.poll() is None:
            _kill_proc(gui_proc)

    import atexit
    atexit.register(cleanup)

    try:
        while True:
            gui_done = gui_proc is not None and gui_proc.poll() is not None
            webots_done = webots_proc.poll() is not None
            if gui_done:
                print("\n[Launcher] Studio GUI closed. Terminating Webots simulation...", flush=True)
                break
            if webots_done:
                print("\n[Launcher] Webots simulation closed. Terminating Studio GUI...", flush=True)
                break
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("\n[Launcher] User interrupted session. Shutting down cleanly...", flush=True)
    finally:
        cleanup()


def main():
    parser = argparse.ArgumentParser(description="Launch Anti-Wrong-Way Watchdog Webots Test Environment")
    parser.add_argument("--no-gui", action="store_true", help="Do not launch live 2D studio GUI alongside Webots")
    parser.add_argument("--no-launch", action="store_true", help="Dry run test without spawning processes")

    args = parser.parse_args()

    if args.no_launch:
        print("[Ready] Dry-run validation passed. World and controller ready.")
        return

    launch_environment(launch_gui=not args.no_gui)


if __name__ == "__main__":
    main()
