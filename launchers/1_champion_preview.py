"""
================================================================================
  LAUNCHER: C++ SL-MPC CHAMPION 3D REPLAY PREVIEW (DYNAMIC OBSTACLES)
================================================================================
  Launches Webots real-time mode with track_racing.wbt AND spawns the live
  PyQt5 2D Telemetry GUI alongside it by default.
  Loads the champion CMA-ES genome, enables 3 dynamic moving obstacles, and
  evaluates lap times with full 3D visual preview.

  Usage:
    python launchers/1_champion_preview.py             (Launches Webots + Live GUI)
    python launchers/1_champion_preview.py --no-gui    (Webots only)
================================================================================
"""

import os
import sys
import time
import json
import argparse
import subprocess

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEBOTS_DIR = os.path.join(ROOT_DIR, "webots")
MY_PROJECT_DIR = os.path.join(WEBOTS_DIR, "simulation", "my_project")
WORLD_PATH = os.path.join(MY_PROJECT_DIR, "worlds", "track_racing.wbt")
GA_OPTIMIZER_DIR = os.path.join(WEBOTS_DIR, "optimization", "ga_planner_optimizer")
PLANNERS_DIR = os.path.join(MY_PROJECT_DIR, "controllers", "tof_path_planner")
WEBOTS_BIN = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

BEST_GENOME_PATH = os.path.join(GA_OPTIMIZER_DIR, "best_planner_genome.json")
ACTIVE_CHAMPION_INFO = {}


def activate_champion_config() -> bool:
    global ACTIVE_CHAMPION_INFO

    if not os.path.isfile(BEST_GENOME_PATH):
        print(f"[Error] Champion file does not exist: {BEST_GENOME_PATH}")
        return False

    try:
        with open(BEST_GENOME_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)

        genes = data.get("genes") or data.get("best_genome")
        if not genes:
            print(f"[Error] No 'genes' or 'best_genome' found in {BEST_GENOME_PATH}")
            return False

        fitness = data.get("fitness", data.get("best_fitness", 0.0))
        metrics = data.get("metrics", data.get("best_metrics", {}))
        planner = data.get("planner", "OPTION_2_SL_MPC")
        title = data.get("title", f"{planner} Best Champion (Fitness {fitness})")

        dist = metrics.get("distance_m", "N/A")
        speed = metrics.get("mean_speed_mps", "N/A")
        laps = metrics.get("laps_completed", "N/A")

        print(f"[Ready] Activated best GA champion genome:")
        print(f"        Source:   {os.path.basename(BEST_GENOME_PATH)}")
        print(f"        Title:    {title}")
        print(f"        Fitness:  {fitness}")
        print(f"        Stats:    Dist: {dist}m | Speed: {speed}m/s | Laps: {laps}")
        return True
    except Exception as e:
        print(f"[Error] Failed to activate configuration: {e}")
        return False


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


def launch_webots(launch_gui: bool = True, telemetry_hz: int = 60):
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
    env["WEBOTS_CONTROLLER_MODE"] = "preview"
    env["WEBOTS_TELEMETRY_HZ"] = str(telemetry_hz)
    env["PYTHONUNBUFFERED"] = "1"

    gui_proc = None
    if launch_gui:
        gui_script = os.path.join(PLANNERS_DIR, "planner_gui.py")
        if os.path.exists(gui_script):
            print(f"[1/2] Spawning Live Telemetry Map GUI (Port 9876, Stream: {telemetry_hz} Hz)...", flush=True)
            gui_proc = subprocess.Popen([sys.executable, gui_script, "--telemetry-hz", str(telemetry_hz), "--local-only"], cwd=PLANNERS_DIR, env=env)
            time.sleep(0.6)

    cmd = [
        WEBOTS_BIN,
        "--stdout",
        "--stderr",
        "--mode=realtime",
        WORLD_PATH,
    ]

    champ_title = ACTIVE_CHAMPION_INFO.get("title", "Best GA Champion")
    print("=" * 80)
    print(f"  LAUNCHING REAL-TIME 3D WEBOTS CHAMPION PREVIEW + LIVE TELEMETRY GUI ({telemetry_hz} Hz)")
    print(f"  World:        {WORLD_PATH}")
    print(f"  Champion:     {champ_title}")
    print(f"  Telemetry Hz: {telemetry_hz} Hz (Adjustable live via GUI slider 1-60 Hz)")
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
                print("\n[Launcher] Telemetry GUI closed. Terminating Webots simulation...", flush=True)
                break
            if webots_done:
                print("\n[Launcher] Webots simulation closed. Terminating Telemetry GUI...", flush=True)
                break
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("\n[Launcher] User interrupted session. Shutting down cleanly...", flush=True)
    finally:
        cleanup()


def main():
    parser = argparse.ArgumentParser(description="Launch Webots Real-Time Preview with C++ SL-MPC Champion Config")
    parser.add_argument("--no-gui", action="store_true", help="Do not launch live 2D telemetry map GUI alongside Webots")
    parser.add_argument("--no-launch", action="store_true", help="Only set active config without launching Webots")
    parser.add_argument("--telemetry-hz", "--hz", type=int, default=60, help="Initial streaming frequency for GUI and Webots terminal telemetry (1-60 Hz, default: 60)")

    args = parser.parse_args()
    telem_hz = max(1, min(60, args.telemetry_hz))

    if not activate_champion_config():
        return

    if not args.no_launch:
        launch_webots(launch_gui=not args.no_gui, telemetry_hz=telem_hz)


if __name__ == "__main__":
    main()
