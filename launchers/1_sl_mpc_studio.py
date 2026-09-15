"""
================================================================================
  1. LAUNCHER: C++ SL-MPC WEBOTS SIMULATION STUDIO + LIVE TELEMETRY GUI
================================================================================
  1. Confirms C++ SL-MPC DLL (curvilinear_sl_mpc.dll) is compiled & ready.
  2. Launches Webots in real-time mode with track_racing.wbt.
  3. Spawns the PyQt5 Planner GUI alongside it (listening on UDP port 9876).

  Usage:
    python 1_sl_mpc_studio.py
================================================================================
"""

import os
import sys
import json
import time
import subprocess
import atexit

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MY_PROJECT_DIR = os.path.join(ROOT_DIR, "webots", "simulation", "my_project")
WORLD_PATH = os.path.join(MY_PROJECT_DIR, "worlds", "track_racing.wbt")
CONTROLLER_DIR = os.path.join(MY_PROJECT_DIR, "controllers", "tof_path_planner")
PLANNERS_DIR = CONTROLLER_DIR
ACTIVE_PLANNER_JSON = os.path.join(CONTROLLER_DIR, "active_planner.json")
GUI_SCRIPT = os.path.join(CONTROLLER_DIR, "planner_gui.py")
WEBOTS_BIN = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
SL_MPC_DLL = os.path.join(CONTROLLER_DIR, "curvilinear_sl_mpc.dll")


def ensure_c_plus_plus_sl_mpc():
    """Confirms C++ DLL exists and sets active_planner.json to OPTION2_SL_MPC."""
    if not os.path.exists(SL_MPC_DLL):
        print(f"[Warning] {SL_MPC_DLL} not found! Attempting compilation...", flush=True)
        compile_script = os.path.join(PLANNERS_DIR, "cpp", "build_dlls.py")
        if os.path.exists(compile_script):
            subprocess.run([sys.executable, compile_script], check=True)

    os.makedirs(CONTROLLER_DIR, exist_ok=True)
    with open(ACTIVE_PLANNER_JSON, "w", encoding="utf-8") as f:
        json.dump({"planner": "OPTION2_SL_MPC"}, f, indent=2)
    print(f"[Ready] Set {ACTIVE_PLANNER_JSON} to OPTION2_SL_MPC.", flush=True)


def main():
    print("=" * 80)
    print("  LAUNCHING C++ SL-MPC WEBOTS WORLD & LIVE TELEMETRY STUDIO")
    print("=" * 80)

    if not os.path.exists(WEBOTS_BIN):
        print(f"[Error] Webots binary not found at: {WEBOTS_BIN}")
        sys.exit(1)

    if not os.path.exists(WORLD_PATH):
        print(f"[Error] World file not found at: {WORLD_PATH}")
        sys.exit(1)

    ensure_c_plus_plus_sl_mpc()

    webots_home = r"C:\Program Files\Webots"
    webots_bin_dir = os.path.dirname(WEBOTS_BIN)
    py_dir = os.path.dirname(sys.executable)
    py_scripts = os.path.join(py_dir, "Scripts")

    env = os.environ.copy()
    env["WEBOTS_HOME"] = webots_home
    env["PATH"] = webots_bin_dir + os.pathsep + py_dir + os.pathsep + py_scripts + os.pathsep + env.get("PATH", "")
    env["PYTHONPATH"] = os.pathsep.join([
        PLANNERS_DIR,
        MY_PROJECT_DIR,
        ROOT_DIR,
    ])
    env["PLANNER_TYPE"] = "OPTION2_SL_MPC"
    env["WEBOTS_CONTROLLER_MODE"] = "racing"
    env["PYTHONUNBUFFERED"] = "1"

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

    # 1. Launch PyQt5 Planner GUI in background subprocess
    print("[1/2] Spawning Planner GUI (UDP Telemetry listener on port 9876)...", flush=True)
    gui_proc = subprocess.Popen(
        [sys.executable, GUI_SCRIPT],
        cwd=PLANNERS_DIR,
        env=env,
    )
    time.sleep(0.6)

    # 2. Launch Webots with track_racing.wbt
    print(f"[2/2] Launching Webots Real-Time Simulation with C++ SL-MPC Controller...", flush=True)
    print(f"      World: {WORLD_PATH}")
    webots_cmd = [
        WEBOTS_BIN,
        "--stdout",
        "--stderr",
        "--mode=realtime",
        WORLD_PATH,
    ]
    webots_proc = subprocess.Popen(webots_cmd, env=env, cwd=MY_PROJECT_DIR)

    def cleanup():
        if webots_proc and webots_proc.poll() is None:
            _kill_proc(webots_proc)
        if gui_proc and gui_proc.poll() is None:
            _kill_proc(gui_proc)

    atexit.register(cleanup)

    try:
        while True:
            gui_done = gui_proc.poll() is not None
            webots_done = webots_proc.poll() is not None
            if gui_done:
                print("\n[Studio] Telemetry GUI closed. Terminating Webots...", flush=True)
                break
            if webots_done:
                print("\n[Studio] Webots simulation closed. Terminating GUI...", flush=True)
                break
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("\n[Studio] Session stopped by user. Cleaning up...", flush=True)
    finally:
        cleanup()


if __name__ == "__main__":
    main()
