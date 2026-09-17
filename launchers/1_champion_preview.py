"""
================================================================================
  1. LAUNCHER: C++ SL-MPC CHAMPION 3D REPLAY PREVIEW (DYNAMIC OBSTACLES)
================================================================================
  Launches Webots real-time mode with track_racing.wbt AND spawns the live
  PyQt5 2D Telemetry GUI alongside it by default.
  Loads the champion CMA-ES genome, enables 3 dynamic moving obstacles, and
  evaluates lap times with full 3D visual preview.

  Usage:
    python 1_champion_preview.py             (Launches Webots + Live GUI)
    python 1_champion_preview.py --no-gui    (Webots only)
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
ACTIVE_CONFIG_PATH = os.path.join(GA_OPTIMIZER_DIR, "active_preview_config.json")
PLANNERS_DIR = os.path.join(MY_PROJECT_DIR, "controllers", "tof_path_planner")
WEBOTS_BIN = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

def find_best_genome() -> dict:
    """
    Scans for all available GA/CMA-ES champion and checkpoint files, parses them,
    and returns the genome configuration with the highest fitness.
    """
    candidates = []

    # Potential champion/checkpoint files in the optimizer directory
    candidate_patterns = [
        os.path.join(GA_OPTIMIZER_DIR, "best_planner_genome.json"),
        os.path.join(GA_OPTIMIZER_DIR, "ga_checkpoint.json"),
    ]

    # Also include any planner-specific champion or checkpoint files
    if os.path.isdir(GA_OPTIMIZER_DIR):
        for fname in os.listdir(GA_OPTIMIZER_DIR):
            if (fname.startswith("best_genome") or fname.startswith("ga_checkpoint")) and fname.endswith(".json"):
                full_p = os.path.join(GA_OPTIMIZER_DIR, fname)
                if full_p not in candidate_patterns:
                    candidate_patterns.append(full_p)

    for filepath in candidate_patterns:
        if not os.path.isfile(filepath):
            continue
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)

            genes = data.get("genes") or data.get("best_genome")
            if not genes:
                continue

            # Resolve fitness score
            fitness = None
            if "fitness" in data and isinstance(data["fitness"], (int, float)):
                fitness = float(data["fitness"])
            elif "best_fitness" in data and isinstance(data["best_fitness"], (int, float)):
                fitness = float(data["best_fitness"])
            elif isinstance(data.get("metrics"), dict) and "fitness" in data["metrics"]:
                fitness = float(data["metrics"]["fitness"])
            elif isinstance(data.get("best_metrics"), dict) and "fitness" in data["best_metrics"]:
                fitness = float(data["best_metrics"]["fitness"])

            if fitness is None:
                continue

            metrics = data.get("metrics") or data.get("best_metrics") or {}
            planner = data.get("planner", "OPTION_2_SL_MPC")
            gen = data.get("generation", metrics.get("generation", "N/A"))
            algo = data.get("algorithm", "CMA-ES")

            title = data.get("title")
            if not title:
                title = f"{planner} Champion ({algo} Gen {gen}, Fitness {fitness:.2f})"

            candidates.append({
                "file": filepath,
                "fitness": fitness,
                "genes": genes,
                "metrics": metrics,
                "planner": planner,
                "generation": gen,
                "algorithm": algo,
                "title": title,
                "raw_data": data,
            })
        except Exception as e:
            print(f"[Warning] Failed parsing {filepath}: {e}")

    if not candidates:
        return {}

    # Sort descending by fitness
    candidates.sort(key=lambda c: c["fitness"], reverse=True)
    return candidates[0]


ACTIVE_CHAMPION_INFO = {}


def activate_champion_config() -> bool:
    global ACTIVE_CHAMPION_INFO
    best_candidate = find_best_genome()

    if not best_candidate:
        print("[Error] No valid GA genome or checkpoint files found in optimization directory.")
        return False

    ACTIVE_CHAMPION_INFO = best_candidate
    src_file = best_candidate["file"]
    raw_data = best_candidate["raw_data"]

    try:
        data = dict(raw_data)
        data["genes"] = best_candidate["genes"]
        data["title"] = best_candidate["title"]
        data["fitness"] = best_candidate["fitness"]
        data["metrics"] = best_candidate["metrics"]
        data["source_file"] = os.path.basename(src_file)

        os.makedirs(os.path.dirname(ACTIVE_CONFIG_PATH), exist_ok=True)
        with open(ACTIVE_CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

        m = best_candidate["metrics"]
        dist = m.get("distance_m", "N/A")
        speed = m.get("mean_speed_mps", "N/A")
        laps = m.get("laps_completed", "N/A")

        print(f"[Ready] Activated best GA champion genome:")
        print(f"        Source:   {os.path.basename(src_file)}")
        print(f"        Title:    {best_candidate['title']}")
        print(f"        Fitness:  {best_candidate['fitness']:.2f}")
        print(f"        Stats:    Dist: {dist}m | Speed: {speed}m/s | Laps: {laps}")
        print(f"[Ready] Written to: {ACTIVE_CONFIG_PATH}")
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


def launch_webots(launch_gui: bool = True):
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
    env["PYTHONUNBUFFERED"] = "1"

    gui_proc = None
    if launch_gui:
        gui_script = os.path.join(PLANNERS_DIR, "planner_gui.py")
        if os.path.exists(gui_script):
            print("[1/2] Spawning Live Telemetry Map GUI (Port 9876)...", flush=True)
            gui_proc = subprocess.Popen([sys.executable, gui_script], cwd=PLANNERS_DIR, env=env)
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
    print("  LAUNCHING REAL-TIME 3D WEBOTS CHAMPION PREVIEW + LIVE TELEMETRY GUI")
    print(f"  World:    {WORLD_PATH}")
    print(f"  Champion: {champ_title}")
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

    args = parser.parse_args()

    if not activate_champion_config():
        return

    if not args.no_launch:
        launch_webots(launch_gui=not args.no_gui)


if __name__ == "__main__":
    main()
