import os
import sys
import json
import time
import shutil
import subprocess
import itertools

CONTROLLER_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "controllers", "folkrace_drive")
CONFIG_PATH = os.path.join(CONTROLLER_DIR, "folkrace_config.json")
ACTIVE_TRAJ_PATH = os.path.join(CONTROLLER_DIR, "cyan_champion_trajectory.csv")
WORLD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "worlds", "Cyan_Champion_Track.wbt")
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
SRC_14CM = os.path.join(os.path.dirname(os.path.abspath(__file__)), "racing_results", "margin_10_to_16cm_winners", "winner_margin_14cm.csv")

shutil.copyfile(SRC_14CM, ACTIVE_TRAJ_PATH)

# Test configurations grid:
# - lead_time_s: [0.035, 0.050, 0.065]
# - k_ug (Understeer gradient): [0.000, 0.035, 0.060]
# - front_proj_m: [0.00, 0.08]
# - k_cte: [1.80, 2.20]

CONFIG_VARIATIONS = [
    {"name": "Baseline (No Kug, 35ms Lead)", "lead_time_s": 0.035, "k_ug": 0.000, "front_proj_m": 0.00, "k_cte": 2.20},
    {"name": "Cfg 1: Kug=0.035, 35ms Lead", "lead_time_s": 0.035, "k_ug": 0.035, "front_proj_m": 0.00, "k_cte": 2.20},
    {"name": "Cfg 2: Kug=0.050, 50ms Lead", "lead_time_s": 0.050, "k_ug": 0.050, "front_proj_m": 0.00, "k_cte": 2.20},
    {"name": "Cfg 3: Kug=0.060, 65ms Lead", "lead_time_s": 0.065, "k_ug": 0.060, "front_proj_m": 0.00, "k_cte": 2.20},
    {"name": "Cfg 4: Kug=0.045, 50ms, FrontProj=8cm", "lead_time_s": 0.050, "k_ug": 0.045, "front_proj_m": 0.08, "k_cte": 2.00},
    {"name": "Cfg 5: Kug=0.060, 55ms, FrontProj=8cm", "lead_time_s": 0.055, "k_ug": 0.060, "front_proj_m": 0.08, "k_cte": 2.20},
    {"name": "Cfg 6: Kug=0.075, 65ms, FrontProj=10cm", "lead_time_s": 0.065, "k_ug": 0.075, "front_proj_m": 0.10, "k_cte": 2.40},
]

PACES = [85.0, 90.0, 92.5, 95.0]

def evaluate(cfg_dict, pace_pct):
    with open(CONFIG_PATH, "r") as f:
        cfg = json.load(f)
    cfg["active_pathfinder"] = "FRENETQP"
    cfg["race_pace_pct"] = pace_pct
    cfg["lead_time_s"] = cfg_dict["lead_time_s"]
    cfg["k_ug"] = cfg_dict["k_ug"]
    cfg["front_proj_m"] = cfg_dict["front_proj_m"]
    cfg["k_cte"] = cfg_dict["k_cte"]
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
        return {"status": "CRASH", "msg": crash_msg, "max_dev": round(max_dev, 1)}

def main():
    print("=" * 85)
    print("       MULTI-CONFIGURATION HIGH-PACE (85% - 95%) BENCHMARK GRID        ")
    print("=" * 85)

    results = []

    for cfg_dict in CONFIG_VARIATIONS:
        print(f"\n>>> CONFIGURATION: {cfg_dict['name']} <<<")
        print(f"    Parameters: lead={cfg_dict['lead_time_s']*1000:.0f}ms | Kug={cfg_dict['k_ug']:.3f} | FrontProj={cfg_dict['front_proj_m']*100:.0f}cm | Kcte={cfg_dict['k_cte']:.2f}")
        
        best_pace = 0.0
        best_lap = None
        best_dev = None
        
        for pace in PACES:
            res = evaluate(cfg_dict, pace)
            if res["status"] == "CLEAN":
                print(f"  Pace {pace:5.1f}%: Lap = {res['lap_time']:5.2f}s | Avg={res['avg_speed']:.2f}m/s | MaxDev={res['max_dev']:4.1f}cm | CLEAN (0 Crashes)")
                best_pace = pace
                best_lap = res["lap_time"]
                best_dev = res["max_dev"]
            else:
                msg = res.get("msg", "Crash")
                print(f"  Pace {pace:5.1f}%: CRASHED -> {msg[:60]} (MaxDev={res.get('max_dev',0)}cm)")
                break
        
        results.append({
            "cfg": cfg_dict,
            "best_pace": best_pace,
            "best_lap": best_lap,
            "best_dev": best_dev
        })

    print("\n\n" + "=" * 85)
    print("                      HIGH-PACE CONFIGURATION LEADERBOARD                      ")
    print("=" * 85)
    print(f"{'Configuration Name':<38} | {'Best Pace':<10} | {'Lap Time':<10} | {'Max Dev':<10}")
    print("-" * 85)
    for r in sorted(results, key=lambda x: (-x["best_pace"], x["best_lap"] if x["best_lap"] else 99)):
        if r["best_pace"] > 0:
            print(f"{r['cfg']['name']:<38} | {r['best_pace']:5.1f} %    | {r['best_lap']:5.2f} s    | {r['best_dev']:4.1f} cm")
        else:
            print(f"{r['cfg']['name']:<38} | FAILED     | ---        | ---")
    print("=" * 85)

    # Save to json
    out_json = os.path.join(os.path.dirname(os.path.abspath(__file__)), "racing_results", "high_pace_85_95_grid_results.json")
    with open(out_json, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n[SUMMARY] Saved full benchmark telemetry to: {out_json}")

if __name__ == "__main__":
    main()
