"""
================================================================================
  3. LAUNCHER: TOF PATH PLANNER & LIVE TELEMETRY TUNER STUDIO (GUI + WEBOTS)
================================================================================
  Launches both the PyQt5 2D Telemetry & Costmap GUI and the Webots simulation
  world (track_racing.wbt) simultaneously.
  When the GUI window is closed, Webots is cleanly terminated.

  Usage:
    python 3_planner_gui.py             (Launches GUI + Webots)
    python 3_planner_gui.py --no-webots (Standalone GUI only)
================================================================================
"""

import os
import sys
import subprocess
import time
import atexit
import argparse

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MY_PROJECT_DIR = os.path.join(ROOT_DIR, "webots", "simulation", "my_project")
WORLD_PATH = os.path.join(MY_PROJECT_DIR, "worlds", "track_racing.wbt")
PLANNERS_DIR = os.path.join(MY_PROJECT_DIR, "controllers", "tof_path_planner")
GUI_SCRIPT = os.path.join(PLANNERS_DIR, "planner_gui.py")
WEBOTS_BIN = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

for p in [ROOT_DIR, MY_PROJECT_DIR, PLANNERS_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)


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


def main():
    parser = argparse.ArgumentParser(description="Launch PyQt5 Telemetry GUI and Webots Simulation Simultaneously")
    parser.add_argument("--no-webots", action="store_true", help="Launch standalone GUI only without Webots")
    args = parser.parse_args()

    env = os.environ.copy()
    webots_home = r"C:\Program Files\Webots"
    webots_bin_dir = os.path.dirname(WEBOTS_BIN)
    py_dir = os.path.dirname(sys.executable)
    py_scripts = os.path.join(py_dir, "Scripts")

    env["WEBOTS_HOME"] = webots_home
    env["PATH"] = webots_bin_dir + os.pathsep + py_dir + os.pathsep + py_scripts + os.pathsep + env.get("PATH", "")
    env["PYTHONPATH"] = os.pathsep.join([PLANNERS_DIR, MY_PROJECT_DIR, ROOT_DIR])
    env["PLANNER_TYPE"] = "OPTION2_SL_MPC"
    env["WEBOTS_CONTROLLER_MODE"] = "racing"
    env["PYTHONUNBUFFERED"] = "1"

    print("=" * 80)
    print("  LAUNCHING LIVE TELEMETRY TUNER STUDIO + 3D WEBOTS SIMULATION")
    print(f"  World: {WORLD_PATH}")
    print("=" * 80)

    # 1. Spawning GUI
    print("[1/2] Spawning 2D Telemetry & Costmap GUI (Port 9876)...", flush=True)
    gui_proc = subprocess.Popen([sys.executable, GUI_SCRIPT], cwd=PLANNERS_DIR, env=env)
    time.sleep(0.6)

    webots_proc = None
    if not args.no_webots:
        if os.path.exists(WEBOTS_BIN) and os.path.exists(WORLD_PATH):
            webots_cmd = [
                WEBOTS_BIN,
                "--stdout",
                "--stderr",
                "--mode=realtime",
                WORLD_PATH,
            ]
            print(f"[2/2] Spawning Webots simulation ({os.path.basename(WORLD_PATH)})...", flush=True)
            webots_proc = subprocess.Popen(webots_cmd, env=env, cwd=MY_PROJECT_DIR)
        else:
            print("[Launcher 3] Note: Webots binary or world not found. Running standalone GUI.", flush=True)

    def cleanup():
        if webots_proc and webots_proc.poll() is None:
            _kill_proc(webots_proc)
        if gui_proc and gui_proc.poll() is None:
            _kill_proc(gui_proc)

    atexit.register(cleanup)

    try:
        while True:
            gui_done = gui_proc.poll() is not None
            webots_done = webots_proc is not None and webots_proc.poll() is not None
            if gui_done:
                print("\n[Launcher 3] Telemetry GUI closed. Terminating Webots...", flush=True)
                break
            if webots_done:
                print("\n[Launcher 3] Webots simulation closed. Terminating GUI...", flush=True)
                break
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("\n[Launcher 3] Session stopped by user. Cleaning up...", flush=True)
    finally:
        cleanup()


if __name__ == "__main__":
    main()
