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
        compile_script = os.path.join(PLANNERS_DIR, "compile_sl_mpc_dll.py")
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

    env = os.environ.copy()
    env["PLANNER_TYPE"] = "OPTION2_SL_MPC"
    env["PYTHONUNBUFFERED"] = "1"

    # 1. Launch PyQt5 Planner GUI in background subprocess
    print("[1/2] Spawning Planner GUI (UDP Telemetry listener on port 9876)...", flush=True)
    gui_proc = subprocess.Popen(
        [sys.executable, GUI_SCRIPT],
        cwd=PLANNERS_DIR,
        env=env,
    )

    def cleanup():
        if gui_proc.poll() is None:
            print("\n[Cleanup] Closing Planner GUI...", flush=True)
            gui_proc.terminate()

    atexit.register(cleanup)
    time.sleep(1.0)

    # 2. Launch Webots with track_racing.wbt
    print(f"[2/2] Launching Webots Real-Time Simulation with C++ SL-MPC Controller...", flush=True)
    print(f"      World: {WORLD_PATH}")
    webots_cmd = [
        WEBOTS_BIN,
        "--mode=realtime",
        WORLD_PATH,
    ]

    try:
        subprocess.run(webots_cmd, env=env, cwd=MY_PROJECT_DIR)
    except KeyboardInterrupt:
        print("\n[Session] Shutting down simulation studio.", flush=True)


if __name__ == "__main__":
    main()
