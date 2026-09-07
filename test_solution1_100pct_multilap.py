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
SRC_14CM = os.path.join(os.path.dirname(os.path.abspath(__file__)), "racing_results", "margin_10_to_16cm_winners", "winner_margin_14cm.csv")

shutil.copyfile(SRC_14CM, ACTIVE_TRAJ_PATH)

PACES = [85.0, 90.0, 92.5, 95.0, 97.5, 100.0]
TARGET_LAPS = 3

def evaluate_multi_lap(pace_pct, target_laps=3):
    with open(CONFIG_PATH, "r") as f:
        cfg = json.load(f)
    cfg["active_pathfinder"] = "FRENETQP"
    cfg["race_pace_pct"] = pace_pct
    cfg["lead_time_s"] = 0.065
    cfg["k_ug"] = 0.060
    cfg["front_proj_m"] = 0.00
    cfg["k_cte"] = 2.20
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)

    env = os.environ.copy()
    env["MAX_BENCHMARK_LAPS"] = str(target_laps + 1)
    env["QUIT_ON_CRASH"] = "1"

    cmd = [WEBOTS_EXE, "--mode=fast", "--batch", "--minimize", "--stdout", "--stderr", WORLD_PATH]
    t0 = time.perf_counter()
    timeout_s = 20 + target_laps * 15
    try:
        proc = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=timeout_s)
    except subprocess.TimeoutExpired:
        return {"status": "TIMEOUT", "laps": []}
    wall_time = time.perf_counter() - t0

    output = proc.stdout + "\n" + proc.stderr

    lap_times = []
    max_dev = 0.0
    devs = []

    for line in output.splitlines():
        if "Dev=" in line:
            try:
                d_val = abs(float(line.split("Dev=")[1].split("cm")[0].strip()))
                devs.append(d_val)
            except:
                pass
        if "LAP COMPLETED" in line and "Time:" in line:
            try:
                lt = float(line.split("Time:")[1].split("s")[0].strip())
                lap_times.append(lt)
            except:
                pass

    if devs:
        max_dev = max(devs)

    is_clean = len(lap_times) >= target_laps
    if proc.returncode == 0 and is_clean:
        return {
            "status": "PASS",
            "laps": lap_times[:target_laps],
            "max_dev": round(max_dev, 1),
            "wall_s": round(wall_time, 1)
        }
    else:
        crash_msg = "Unknown crash or premature termination"
        for line in output.splitlines():
            if "CRASH DETECTED" in line or "CRASH ABORT" in line or "Bumper" in line:
                crash_msg = line.strip()
                break
        return {
            "status": "CRASH",
            "msg": crash_msg,
            "laps_done": len(lap_times),
            "laps": lap_times,
            "max_dev": round(max_dev, 1)
        }

def main():
    print("=" * 85)
    print("      SOLUTION 1 (FRENETQP + PREDICTIVE SHIELD) UP TO 100% PACE (3-LAP BENCHMARK)       ")
    print("=" * 85)

    results = []
    for pace in PACES:
        print(f"\n>>> TESTING PACE: {pace:5.1f} % ({TARGET_LAPS} Continuous Laps) <<<")
        res = evaluate_multi_lap(pace, target_laps=TARGET_LAPS)
        
        if res["status"] == "PASS":
            laps_str = " | ".join(f"Lap {i+1}={t:.2f}s" for i, t in enumerate(res["laps"]))
            print(f"  Result: PASS {TARGET_LAPS} LAPS! | {laps_str} | MaxDev = {res['max_dev']:4.1f}cm | (0 Crashes)")
        else:
            msg = res.get("msg", "Crash")
            done = res.get("laps_done", 0)
            print(f"  Result: CRASHED on Lap #{done+1} -> {msg[:65]} (MaxDev={res.get('max_dev',0)}cm)")
        
        results.append({"pace": pace, "res": res})

    print("\n\n" + "=" * 85)
    print("                MULTI-LAP ENDURANCE LEADERBOARD (UP TO 100% PACE)               ")
    print("=" * 85)
    print(f"{'Race Pace':<12} | {'Status':<12} | {'Lap 1':<8} | {'Lap 2':<8} | {'Lap 3':<8} | {'Max Dev':<8}")
    print("-" * 85)
    for r in results:
        p = r["pace"]
        res = r["res"]
        laps = res.get("laps", [])
        l1 = f"{laps[0]:.2f}s" if len(laps) > 0 else "---"
        l2 = f"{laps[1]:.2f}s" if len(laps) > 1 else "---"
        l3 = f"{laps[2]:.2f}s" if len(laps) > 2 else "---"
        status_str = f"PASS {len(laps)} LAPS" if res["status"] == "PASS" else f"CRASH (L{res.get('laps_done',0)+1})"
        print(f"{p:5.1f} %      | {status_str:<12} | {l1:<8} | {l2:<8} | {l3:<8} | {res.get('max_dev',0):4.1f} cm")
    print("=" * 85)

if __name__ == "__main__":
    main()
