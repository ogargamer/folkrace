"""
================================================================================
  LAUNCHER: FAST CHAMPION PURE 3D DRIVE (MAX REAL-TIME PERFORMANCE)
================================================================================
  Launches Webots real-time mode with track_racing.wbt for direct 3D visual
  driving evaluation with dynamic moving obstacles.

  Stripped of all background GUI processes, high-rate UDP broadcast streaming,
  viewport HUD overlay drawing, and stdout terminal flooding for maximum native
  physics and real-time driving performance.

  Usage:
    python 1_launchers/2_fast_champion_drive.py           (Track 2 default: Reverse opponents)
    python 1_launchers/2_fast_champion_drive.py --track 1 (Track 1: Forward opponents)
    python 1_launchers/2_fast_champion_drive.py --reverse (Track 2: Reverse opponents)
================================================================================
"""

import argparse
import os
import sys
import time
import json
import subprocess

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEBOTS_DIR = os.path.join(ROOT_DIR, "5_webots")
MY_PROJECT_DIR = os.path.join(WEBOTS_DIR, "simulation", "my_project")
WORLD_PATH = os.path.join(MY_PROJECT_DIR, "worlds", "track_racing.wbt")
GA_OPTIMIZER_DIR = os.path.join(WEBOTS_DIR, "optimization", "ga_planner_optimizer")
BEST_GENOME_PATH = os.path.join(GA_OPTIMIZER_DIR, "best_planner_genome.json")
WEBOTS_BIN = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"


def activate_champion_config(track: int = 2) -> bool:
    if not os.path.isfile(BEST_GENOME_PATH):
        print(f"[Error] Champion genome file does not exist: {BEST_GENOME_PATH}")
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
        track_label = "Track 2 (Reverse Opponents)" if track == 2 else "Track 1 (Forward Opponents)"

        print("=" * 80)
        print("  FAST CHAMPION PURE 3D DRIVE - MAX REAL-TIME PERFORMANCE")
        print("=" * 80)
        print(f"[Ready] Activated Champion: {title}")
        print(f"        Fitness: {fitness} | Dist: {dist}m | Speed: {speed}m/s | Laps: {laps}")
        print(f"        World:   {os.path.basename(WORLD_PATH)} | {track_label}")
        print("=" * 80)
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


def parse_args():
    parser = argparse.ArgumentParser(description="Fast Champion Pure 3D Drive Launcher")
    parser.add_argument(
        "--track",
        type=int,
        choices=(1, 2),
        default=2,
        help="Track mode: 1 = Forward opponents, 2 = Reverse opponents (default)",
    )
    parser.add_argument(
        "--reverse",
        action="store_true",
        help="Shortcut for --track 2 (reverse opponents)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    track = 2 if args.reverse else args.track

    if not activate_champion_config(track=track):
        sys.exit(1)

    webots_cmd = WEBOTS_BIN if os.path.isfile(WEBOTS_BIN) else "webots"

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    env["WEBOTS_CONTROLLER_MODE"] = "fast_drive"
    env["FOLKRACE_OBS_DIRECTION"] = "-1" if track == 2 else "1"

    cmd = [
        webots_cmd,
        "--stdout",
        "--stderr",
        "--mode=realtime",
        WORLD_PATH,
    ]

    print(f"[Launcher] Launching Webots in realtime 3D visualization mode...")
    webots_proc = subprocess.Popen(cmd, env=env, cwd=MY_PROJECT_DIR)

    import atexit
    atexit.register(_kill_proc, webots_proc)

    try:
        while True:
            if webots_proc.poll() is not None:
                print("\n[Launcher] Webots closed.")
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[Launcher] Interrupt caught. Terminating Webots...")
    finally:
        _kill_proc(webots_proc)
        print("[Launcher] Clean exit.")


if __name__ == "__main__":
    main()
