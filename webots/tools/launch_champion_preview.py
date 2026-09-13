"""
================================================================================
  ONE-CLICK LAUNCHER & RUNNER FOR CHAMPION C++ SL-MPC WEBOTS PREVIEW
================================================================================
  Launches Webots in real-time mode with track_preview.wbt, 3 moving dynamic
  obstacles, and the C++ Curvilinear SL-MPC Champion controller.

  Usage:
    python launch_champion_preview.py           (Launch champion preview)
    python launch_champion_preview.py --gui     (Launch with live 2D telemetry GUI)
================================================================================
"""

import os
import sys
import json
import argparse
import subprocess

CURRENT_FILE_DIR = os.path.dirname(os.path.abspath(__file__))
WEBOTS_DIR = os.path.abspath(os.path.join(CURRENT_FILE_DIR, ".."))
WORKSPACE_ROOT = os.path.dirname(WEBOTS_DIR)

MY_PROJECT_DIR = os.path.join(WEBOTS_DIR, "simulation", "my_project")
WORLD_PATH = os.path.join(MY_PROJECT_DIR, "worlds", "track_preview.wbt")
GA_OPTIMIZER_DIR = os.path.join(WEBOTS_DIR, "optimization", "ga_planner_optimizer")
CHAMPION_DIR = os.path.join(MY_PROJECT_DIR, "controllers", "champion_live_preview")
ACTIVE_CONFIG_PATH = os.path.join(CHAMPION_DIR, "active_preview_config.json")
PLANNERS_DIR = os.path.join(MY_PROJECT_DIR, "controllers", "tof_path_planner")
WEBOTS_BIN = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

CHAMPION_CONFIG = {
    "id": "sl_mpc",
    "title": "OPTION 2: Curvilinear SL-MPC Champion (CMA-ES Gen 195, Fitness 7170.03)",
    "file": os.path.join(GA_OPTIMIZER_DIR, "best_genome_option_2_sl_mpc.json"),
    "speed": "2.61 m/s",
    "dist": "48.87 m",
    "laps": "3 laps in 20.0s",
    "status": "Convex quadratic Curvilinear SL-MPC over Frenet lattice with embedded OSQP solver.",
}


def activate_champion_config() -> bool:
    src_file = CHAMPION_CONFIG["file"]
    if not os.path.exists(src_file):
        print(f"Error: Genome file does not exist: {src_file}")
        return False

    try:
        with open(src_file, "r", encoding="utf-8") as f:
            data = json.load(f)

        data["config_id"] = CHAMPION_CONFIG["id"]
        data["title"] = CHAMPION_CONFIG["title"]

        os.makedirs(os.path.dirname(ACTIVE_CONFIG_PATH), exist_ok=True)
        with open(ACTIVE_CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

        print(f"[Launcher] Activated configuration: {CHAMPION_CONFIG['title']}")
        print(f"[Launcher] Written to: {ACTIVE_CONFIG_PATH}")
        return True
    except Exception as e:
        print(f"[Launcher] Failed to activate configuration: {e}")
        return False


def launch_webots(launch_gui: bool = False):
    if not os.path.exists(WEBOTS_BIN):
        print(f"Error: Webots executable not found at {WEBOTS_BIN}")
        return

    if not os.path.exists(WORLD_PATH):
        print(f"Error: World file not found at {WORLD_PATH}")
        return

    gui_proc = None
    if launch_gui:
        gui_script = os.path.join(PLANNERS_DIR, "planner_gui.py")
        if os.path.exists(gui_script):
            print("[Launcher] Spawning Live Telemetry Map GUI (Port 9876)...", flush=True)
            gui_proc = subprocess.Popen([sys.executable, gui_script], cwd=PLANNERS_DIR)

    env = os.environ.copy()
    webots_home = r"C:\Program Files\Webots"
    webots_bin_dir = os.path.dirname(WEBOTS_BIN)
    py_dir = os.path.dirname(sys.executable)
    py_scripts = os.path.join(py_dir, "Scripts")

    env["WEBOTS_HOME"] = webots_home
    env["PATH"] = webots_bin_dir + os.pathsep + py_dir + os.pathsep + py_scripts + os.pathsep + env.get("PATH", "")
    env["PYTHONPATH"] = os.pathsep.join([
        PLANNERS_DIR,
        CHAMPION_DIR,
        os.path.join(GA_OPTIMIZER_DIR, "data"),
        GA_OPTIMIZER_DIR,
        MY_PROJECT_DIR,
        WORKSPACE_ROOT,
    ])
    env["PYTHONUNBUFFERED"] = "1"

    cmd = [
        WEBOTS_BIN,
        "--mode=realtime",
        WORLD_PATH,
    ]

    print("=" * 80)
    print("  LAUNCHING REAL-TIME 3D WEBOTS CHAMPION PREVIEW")
    print(f"  World: {WORLD_PATH}")
    print(f"  Champion: {CHAMPION_CONFIG['title']}")
    print("=" * 80)

    try:
        subprocess.run(cmd, env=env, cwd=MY_PROJECT_DIR)
    finally:
        if gui_proc is not None and gui_proc.poll() is None:
            gui_proc.terminate()


def main():
    parser = argparse.ArgumentParser(description="Launch Webots Real-Time Preview with C++ SL-MPC Champion Config")
    parser.add_argument("--gui", "-g", action="store_true", help="Launch live 2D telemetry map GUI alongside Webots")
    parser.add_argument("--no-launch", action="store_true", help="Only set active config without launching Webots")

    args = parser.parse_args()

    if not activate_champion_config():
        return

    if not args.no_launch:
        launch_webots(launch_gui=args.gui)


if __name__ == "__main__":
    main()
