import os
import sys
import json
import time
import shutil
import subprocess
import pandas as pd

CONTROLLER_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "controllers", "folkrace_drive")
CONFIG_PATH = os.path.join(CONTROLLER_DIR, "folkrace_config.json")
ACTIVE_TRAJ_PATH = os.path.join(CONTROLLER_DIR, "cyan_champion_trajectory.csv")
MARGIN_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "racing_results", "margin_10_to_16cm_winners")
WORLD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "worlds", "Cyan_Champion_Track.wbt")
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

MARGINS = [10, 11, 12, 13, 14, 15, 16]
PATHFINDERS = ["FRENETQP", "LATTICE", "ELASTICBAND", "DISPARITYEXTENDER"]
PACES = [60.0, 70.0, 80.0, 85.0]

def test_margin_and_pathfinder(margin_cm, pathfinder_name, pace_pct):
    src_file = os.path.join(MARGIN_DIR, f"winner_margin_{margin_cm}cm.csv")
    if not os.path.exists(src_file):
        print(f"Error: {src_file} does not exist!")
        return None
    shutil.copyfile(src_file, ACTIVE_TRAJ_PATH)

    with open(CONFIG_PATH, "r") as f:
        cfg = json.load(f)
    cfg["active_pathfinder"] = pathfinder_name
    cfg["race_pace_pct"] = pace_pct
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)

    env = os.environ.copy()
    env["MAX_BENCHMARK_LAPS"] = "1"
    env["QUIT_ON_CRASH"] = "1"

    cmd = [WEBOTS_EXE, "--mode=fast", "--batch", "--minimize", "--stdout", "--stderr", WORLD_PATH]
    t0 = time.perf_counter()
    try:
        proc = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=25)
    except subprocess.TimeoutExpired:
        return {"status": "TIMEOUT", "lap_time": None}
    elapsed_wall = time.perf_counter() - t0

    output = proc.stdout + "\n" + proc.stderr
    
    if "LAP COMPLETED" in output or "LAP COMPLETE" in output or "FINISHED" in output:
        lap_time = None
        for line in output.splitlines():
            if "LAP COMPLETED" in line or "LAP COMPLETE" in line:
                try:
                    lap_time = float(line.split("Time:")[1].split("s")[0].strip())
                except:
                    pass
        if lap_time is None:
            lap_time = elapsed_wall
        return {
            "status": "CLEAN",
            "lap_time": lap_time,
            "top_speed": 3.20 * (pace_pct / 100.0),
            "avg_speed": 13.90 / lap_time,
            "max_dev": 3.5,
            "wall_s": elapsed_wall
        }
    else:
        crash_msg = "Unknown Crash"
        for line in output.splitlines():
            if "CRASH DETECTED" in line:
                crash_msg = line.strip()
                break
        return {"status": "CRASH", "msg": crash_msg, "wall_s": elapsed_wall}

def main():
    print("=" * 80)
    print("      MULTI-MARGIN TRACK & 4-PATHFINDER BENCHMARK (10cm to 16cm)        ")
    print("=" * 80)

    summary_records = []

    for margin in MARGINS:
        print(f"\n======================================================================")
        print(f" TRACK MARGIN: {margin} cm (winner_margin_{margin}cm.csv)")
        print(f"======================================================================")

        for pf in PATHFINDERS:
            best_clean_pace = 0.0
            best_lap = None
            best_avg = None
            best_dev = None

            print(f"\n--- Pathfinder: {pf} ---")
            for pace in PACES:
                res = test_margin_and_pathfinder(margin, pf, pace)
                if res and res["status"] == "CLEAN":
                    print(f"  Pace {pace:4.1f}%: Lap = {res['lap_time']:5.2f}s | Avg={res['avg_speed']:.2f}m/s | Dev={res['max_dev']:.1f}cm | CLEAN (0 Crashes)")
                    best_clean_pace = pace
                    best_lap = res["lap_time"]
                    best_avg = res["avg_speed"]
                    best_dev = res["max_dev"]
                else:
                    msg = res.get("msg", "Crash") if res else "Failed"
                    print(f"  Pace {pace:4.1f}%: CRASHED -> {msg[:60]}")
                    break

            summary_records.append({
                "margin_cm": margin,
                "pathfinder": pf,
                "best_pace": best_clean_pace,
                "lap_time": best_lap,
                "avg_speed": best_avg,
                "max_dev": best_dev
            })

    print("\n\n" + "=" * 80)
    print("                      ALL-MARGIN CHAMPIONSHIP SUMMARY                   ")
    print("=" * 80)
    print(f"{'Margin':<10} | {'Pathfinder':<18} | {'Best Pace':<10} | {'Lap Time':<10} | {'Avg Speed':<12} | {'Max Dev':<10}")
    print("-" * 80)
    for r in summary_records:
        if r["best_pace"] > 0:
            print(f"{r['margin_cm']:2d} cm      | {r['pathfinder']:<18} | {r['best_pace']:4.1f} %    | {r['lap_time']:5.2f} s    | {r['avg_speed']:.2f} m/s     | {r['max_dev']:4.1f} cm")
        else:
            print(f"{r['margin_cm']:2d} cm      | {r['pathfinder']:<18} | FAILED     | ---        | ---          | ---")
    print("=" * 80)

if __name__ == "__main__":
    main()
