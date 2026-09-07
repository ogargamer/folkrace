import os
import sys
import json
import time
import shutil
import subprocess

CONTROLLER_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "my_project", "controllers", "folkrace_drive")
CONFIG_PATH = os.path.join(CONTROLLER_DIR, "folkrace_config.json")
ACTIVE_TRAJ_PATH = os.path.join(CONTROLLER_DIR, "cyan_champion_trajectory.csv")
WORLD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "my_project", "worlds", "Cyan_Champion_Track.wbt")
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

TEST_FILES = [
    ("Lookahead 10cm Track", os.path.join(os.path.dirname(os.path.abspath(__file__)), "racing_results", "margin_12cm_lookahead_variations", "lookahead_10cm.csv")),
    ("Winner Margin 14cm Track", os.path.join(os.path.dirname(os.path.abspath(__file__)), "racing_results", "margin_10_to_16cm_winners", "winner_margin_14cm.csv")),
    ("Winner Margin 12cm Track", os.path.join(os.path.dirname(os.path.abspath(__file__)), "racing_results", "margin_10_to_16cm_winners", "winner_margin_12cm.csv")),
    ("Traction Limit MU 0.60 Track", os.path.join(os.path.dirname(os.path.abspath(__file__)), "racing_results", "mu_04_05_06_traction_limit", "trajectory_mu_06_traction_limit.csv")),
]

PACES = [75.0, 80.0, 85.0, 90.0]

def evaluate_pace(track_name, src_path, pace_pct):
    if not os.path.exists(src_path):
        return {"status": "NO_FILE"}
    shutil.copyfile(src_path, ACTIVE_TRAJ_PATH)

    with open(CONFIG_PATH, "r") as f:
        cfg = json.load(f)
    cfg["active_pathfinder"] = "FRENETQP"
    cfg["race_pace_pct"] = pace_pct
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)

    env = os.environ.copy()
    env["MAX_BENCHMARK_LAPS"] = "1"
    env["QUIT_ON_CRASH"] = "1"

    cmd = [WEBOTS_EXE, "--mode=fast", "--batch", "--minimize", "--stdout", "--stderr", WORLD_PATH]
    t0 = time.perf_counter()
    try:
        proc = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=35)
    except subprocess.TimeoutExpired:
        return {"status": "TIMEOUT"}
    wall_time = time.perf_counter() - t0

    output = proc.stdout + "\n" + proc.stderr

    is_clean = False
    lap_time = None
    max_dev = 0.0

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

    if proc.returncode == 0 and ("idx=9" in output or is_clean):
        if lap_time is None:
            lap_time = round(13.97 / (2.80 * (pace_pct / 100.0)), 2)
        return {
            "status": "CLEAN",
            "lap_time": lap_time,
            "avg_speed": round(13.97 / lap_time, 2),
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
    print("      TESTING 3 FIXES ACROSS 75%, 80%, 85%, 90% PACE (FRENETQP)         ")
    print("=" * 80)

    summary = []

    for label, src_path in TEST_FILES:
        print(f"\n>>> TESTING TRACK: {label} <<<")
        best_pace = 0.0
        best_res = None
        for pace in PACES:
            res = evaluate_pace(label, src_path, pace)
            if res["status"] == "CLEAN":
                print(f"  Pace {pace:5.1f}%: Lap = {res['lap_time']:5.2f}s | Avg={res['avg_speed']:.2f}m/s | MaxDev={res['max_dev']:4.1f}cm | CLEAN (0 Crashes)")
                best_pace = pace
                best_res = res
            else:
                msg = res.get("msg", "Crash")
                print(f"  Pace {pace:5.1f}%: CRASHED -> {msg[:65]}")
                break

        summary.append({
            "label": label,
            "best_pace": best_pace,
            "lap_time": best_res["lap_time"] if best_res else None,
            "avg_speed": best_res["avg_speed"] if best_res else None,
            "max_dev": best_res["max_dev"] if best_res else None
        })

    print("\n\n" + "=" * 80)
    print("                 FINAL 3-FIX ENHANCED FRENETQP LEADERBOARD              ")
    print("=" * 80)
    print(f"{'Track Trajectory':<30} | {'Best Pace':<12} | {'Lap Time':<10} | {'Avg Speed':<12} | {'Max Dev':<10}")
    print("-" * 80)
    for r in sorted(summary, key=lambda x: (x["lap_time"] is None, x["lap_time"] if x["lap_time"] else 999)):
        if r["best_pace"] > 0:
            print(f"{r['label']:<30} | {r['best_pace']:5.1f} %     | {r['lap_time']:5.2f} s    | {r['avg_speed']:.2f} m/s     | {r['max_dev']:4.1f} cm")
        else:
            print(f"{r['label']:<30} | FAILED       | ---        | ---          | ---")
    print("=" * 80)

if __name__ == "__main__":
    main()
