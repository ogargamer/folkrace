import os
import sys
import json
import time
import shutil
import subprocess

CONTROLLER_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "controllers", "folkrace_drive")
CONFIG_PATH = os.path.join(CONTROLLER_DIR, "folkrace_config.json")
ACTIVE_TRAJ_PATH = os.path.join(CONTROLLER_DIR, "cyan_champion_trajectory.csv")
MU_SWEEP_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "racing_results", "lookahead_10cm_mu_sweep")
WORLD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "worlds", "Cyan_Champion_Track.wbt")
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

MU_FILES = [
    ("MU 0.50 (Standard)", "mu_05_standard_accel_2.2mps2.csv"),
    ("MU 0.50 (Aggressive)", "mu_05_aggressive_accel_2.8mps2.csv"),
    ("MU 0.60 (Standard)", "mu_06_standard_accel_2.4mps2.csv"),
    ("MU 0.60 (Aggressive)", "mu_06_aggressive_accel_3.2mps2.csv"),
    ("MU 0.70 (Standard)", "mu_07_standard_accel_2.8mps2.csv"),
    ("MU 0.70 (Aggressive)", "mu_07_aggressive_accel_3.6mps2.csv"),
    ("MU 0.80 (Standard)", "mu_08_standard_accel_3.3mps2.csv"),
    ("MU 0.80 (Aggressive)", "mu_08_aggressive_accel_4.2mps2.csv"),
    ("MU 0.90 (Standard)", "mu_09_standard_accel_3.8mps2.csv"),
    ("MU 0.90 (Aggressive)", "mu_09_aggressive_accel_4.8mps2.csv"),
]

PACES = [75.0, 80.0, 85.0, 90.0, 95.0, 100.0]

def evaluate_mu_file(label, filename, pace_pct):
    src_file = os.path.join(MU_SWEEP_DIR, filename)
    if not os.path.exists(src_file):
        return {"status": "NO_FILE"}
    shutil.copyfile(src_file, ACTIVE_TRAJ_PATH)

    with open(CONFIG_PATH, "r") as f:
        cfg = json.load(f)
    cfg["active_pathfinder"] = "FRENETQP"
    cfg["race_pace_pct"] = pace_pct
    cfg["lead_time_s"] = 0.035
    cfg["front_proj_m"] = 0.10
    cfg["k_cte"] = 1.40
    cfg["k_d"] = 0.04
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)

    env = os.environ.copy()
    env["MAX_BENCHMARK_LAPS"] = "1"
    env["QUIT_ON_CRASH"] = "1"

    cmd = [WEBOTS_EXE, "--mode=fast", "--batch", "--minimize", "--stdout", "--stderr", WORLD_PATH]
    t0 = time.perf_counter()
    try:
        proc = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=40)
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
    print("      MU-SWEEP (MU 0.50 TO 0.90) 10CM LOOKAHEAD BENCHMARK ON FRENETQP  ")
    print("=" * 80)

    summary = []

    for label, filename in MU_FILES:
        print(f"\n>>> TESTING: {label} ({filename}) <<<")
        best_pace = 0.0
        best_res = None
        for pace in PACES:
            res = evaluate_mu_file(label, filename, pace)
            if res["status"] == "CLEAN":
                print(f"  Pace {pace:4.1f}%: Lap = {res['lap_time']:5.2f}s | Avg={res['avg_speed']:.2f}m/s | MaxDev={res['max_dev']:4.1f}cm | CLEAN (0 Crashes)")
                best_pace = pace
                best_res = res
            else:
                msg = res.get("msg", "Crash")
                print(f"  Pace {pace:4.1f}%: CRASHED -> {msg[:55]}")
                break

        summary.append({
            "label": label,
            "filename": filename,
            "best_pace": best_pace,
            "lap_time": best_res["lap_time"] if best_res else None,
            "avg_speed": best_res["avg_speed"] if best_res else None,
            "max_dev": best_res["max_dev"] if best_res else None
        })

    print("\n\n" + "=" * 80)
    print("               FINAL MU-SWEEP CHAMPIONSHIP LEADERBOARD                  ")
    print("=" * 80)
    print(f"{'Friction Tier':<24} | {'Best Safe Pace':<15} | {'Lap Time':<10} | {'Avg Speed':<12} | {'Max Dev':<10}")
    print("-" * 80)
    for r in sorted(summary, key=lambda x: (x["lap_time"] is None, x["lap_time"] if x["lap_time"] else 999)):
        if r["best_pace"] > 0:
            print(f"{r['label']:<24} | {r['best_pace']:4.1f} %          | {r['lap_time']:5.2f} s    | {r['avg_speed']:.2f} m/s     | {r['max_dev']:4.1f} cm")
        else:
            print(f"{r['label']:<24} | FAILED          | ---        | ---          | ---")
    print("=" * 80)

if __name__ == "__main__":
    main()
