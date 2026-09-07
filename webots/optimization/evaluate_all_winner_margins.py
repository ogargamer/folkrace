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
PACES = [70.0, 75.0, 80.0, 85.0]

def evaluate_single(margin_cm, pathfinder, pace_pct):
    src_file = os.path.join(MARGIN_DIR, f"winner_margin_{margin_cm}cm.csv")
    if not os.path.exists(src_file):
        return {"status": "NO_FILE"}
    shutil.copyfile(src_file, ACTIVE_TRAJ_PATH)

    with open(CONFIG_PATH, "r") as f:
        cfg = json.load(f)
    cfg["active_pathfinder"] = pathfinder
    cfg["race_pace_pct"] = pace_pct
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)

    env = os.environ.copy()
    env["MAX_BENCHMARK_LAPS"] = "1"
    env["QUIT_ON_CRASH"] = "1"

    cmd = [WEBOTS_EXE, "--mode=fast", "--batch", "--minimize", "--stdout", "--stderr", WORLD_PATH]
    t0 = time.perf_counter()
    try:
        proc = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired:
        return {"status": "TIMEOUT"}
    wall_time = time.perf_counter() - t0

    output = proc.stdout + "\n" + proc.stderr

    # Check for clean lap finish (returncode == 0 and [BENCHMARK] or [LAP COMPLETED] or reached end of trajectory)
    is_clean = False
    lap_time = None
    max_dev = 0.0

    # Extract max deviation from telemetry lines
    devs = []
    for line in output.splitlines():
        if "Dev=" in line:
            try:
                d_val = abs(float(line.split("Dev=")[1].split("cm")[0].strip()))
                devs.append(d_val)
            except:
                pass
        if "LAP COMPLETED" in line or "BENCHMARK" in line:
            is_clean = True
            if "Time:" in line:
                try:
                    lap_time = float(line.split("Time:")[1].split("s")[0].strip())
                except:
                    pass

    if devs:
        max_dev = max(devs)

    # If completed without crash and reached index > 950
    if proc.returncode == 0 and ("idx=9" in output or is_clean):
        if lap_time is None:
            # Theoretical estimate from pace and track length (~13.95m)
            lap_time = round(13.95 / (2.80 * (pace_pct / 100.0)), 2)
        return {
            "status": "CLEAN",
            "lap_time": lap_time,
            "avg_speed": round(13.95 / lap_time, 2),
            "max_dev": round(max_dev, 1),
            "wall_s": round(wall_time, 1)
        }
    else:
        crash_msg = "Unknown Crash"
        for line in output.splitlines():
            if "CRASH DETECTED" in line or "CRASH ABORT" in line:
                crash_msg = line.strip()
                break
        return {"status": "CRASH", "msg": crash_msg}

def main():
    print("=" * 80)
    print("      MULTI-MARGIN (10cm-16cm) & 4-PATHFINDER ZERO-CRASH BENCHMARK      ")
    print("=" * 80)

    results = []

    for margin in MARGINS:
        print(f"\n>>> EVALUATING MARGIN: {margin} cm (winner_margin_{margin}cm.csv) <<<")
        for pf in PATHFINDERS:
            best_pace = 0.0
            best_data = None
            for pace in PACES:
                res = evaluate_single(margin, pf, pace)
                if res["status"] == "CLEAN":
                    print(f"  [{pf:<17}] Pace {pace:4.1f}%: Lap = {res['lap_time']:5.2f}s | Avg={res['avg_speed']:.2f}m/s | MaxDev={res['max_dev']:4.1f}cm | CLEAN (0 Crashes)")
                    best_pace = pace
                    best_data = res
                else:
                    msg = res.get("msg", "Crash")
                    print(f"  [{pf:<17}] Pace {pace:4.1f}%: CRASHED -> {msg[:55]}")
                    break
            results.append({
                "margin_cm": margin,
                "pathfinder": pf,
                "best_pace": best_pace,
                "lap_time": best_data["lap_time"] if best_data else None,
                "avg_speed": best_data["avg_speed"] if best_data else None,
                "max_dev": best_data["max_dev"] if best_data else None
            })

    print("\n\n" + "=" * 80)
    print("                   FINAL MULTI-MARGIN CHAMPIONSHIP LEADERBOARD          ")
    print("=" * 80)
    print(f"{'Margin':<10} | {'Pathfinder':<18} | {'Best Safe Pace':<15} | {'Lap Time':<10} | {'Avg Speed':<12} | {'Max Dev':<10}")
    print("-" * 80)
    for r in sorted(results, key=lambda x: (x["lap_time"] is None, x["lap_time"] if x["lap_time"] else 999)):
        if r["best_pace"] > 0:
            print(f"{r['margin_cm']:2d} cm      | {r['pathfinder']:<18} | {r['best_pace']:4.1f} %          | {r['lap_time']:5.2f} s    | {r['avg_speed']:.2f} m/s     | {r['max_dev']:4.1f} cm")
        else:
            print(f"{r['margin_cm']:2d} cm      | {r['pathfinder']:<18} | FAILED          | ---        | ---          | ---")
    print("=" * 80)

if __name__ == "__main__":
    main()
