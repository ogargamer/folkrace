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

# Test 4 distinct predictive pathfinder architectures at 90.0% pace across 2 continuous laps:
# Solution 1: FRENETQP with Predictive Safety Shield (Forward Swept-Volume Rollout)
# Solution 2: Frenet Lattice Planner (105-Candidate Spatiotemporal Trajectories)
# Solution 3: Gaussian Elastic Band (GEB Potential Field Deformation)
# Solution 4: Biased Disparity Extender (BDE Gap-Following)

SOLUTIONS = [
    {"name": "Solution 1: FRENETQP + Predictive Swept-Volume Shield", "pathfinder": "FRENETQP"},
    {"name": "Solution 2: Frenet Spatiotemporal Lattice Planner", "pathfinder": "LATTICE"},
    {"name": "Solution 3: Multi-Kernel Gaussian Elastic Band", "pathfinder": "ELASTICBAND"},
    {"name": "Solution 4: Biased Disparity Extender (BDE)", "pathfinder": "DISPARITYEXTENDER"},
]

def evaluate_solution(sol_dict, pace_pct=90.0):
    with open(CONFIG_PATH, "r") as f:
        cfg = json.load(f)
    cfg["active_pathfinder"] = sol_dict["pathfinder"]
    cfg["race_pace_pct"] = pace_pct
    cfg["lead_time_s"] = 0.065
    cfg["k_ug"] = 0.060
    cfg["front_proj_m"] = 0.00
    cfg["k_cte"] = 2.20
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)

    env = os.environ.copy()
    env["MAX_BENCHMARK_LAPS"] = "3"
    env["QUIT_ON_CRASH"] = "1"

    cmd = [WEBOTS_EXE, "--mode=fast", "--batch", "--minimize", "--stdout", "--stderr", WORLD_PATH]
    t0 = time.perf_counter()
    try:
        proc = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=55)
    except subprocess.TimeoutExpired:
        return {"status": "TIMEOUT"}
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

    if proc.returncode == 0 and len(lap_times) >= 2:
        return {
            "status": "PASS_2_LAPS",
            "lap_1": lap_times[0],
            "lap_2": lap_times[1],
            "max_dev": round(max_dev, 1),
            "wall_s": round(wall_time, 1)
        }
    else:
        crash_msg = "Failed before Lap 2"
        for line in output.splitlines():
            if "CRASH DETECTED" in line or "CRASH ABORT" in line or "Bumper" in line:
                crash_msg = line.strip()
                break
        return {"status": "CRASH", "msg": crash_msg, "laps_done": len(lap_times), "max_dev": round(max_dev, 1)}

def main():
    print("=" * 85)
    print("      MULTI-SOLUTION PREDICTIVE BENCHMARK AT 90.0% PACE (2 FULL LAPS)           ")
    print("=" * 85)

    results = []
    for sol in SOLUTIONS:
        print(f"\n>>> TESTING: {sol['name']} <<<")
        res = evaluate_solution(sol, pace_pct=90.0)
        if res["status"] == "PASS_2_LAPS":
            print(f"  Result: PASS 2 LAPS | Lap 1 = {res['lap_1']:5.2f}s | Lap 2 (Flying) = {res['lap_2']:5.2f}s | MaxDev = {res['max_dev']:4.1f}cm | (0 Crashes)")
        else:
            msg = res.get("msg", "Crash")
            print(f"  Result: CRASHED on Lap #{res.get('laps_done',0)+1} -> {msg[:65]} (MaxDev={res.get('max_dev',0)}cm)")
        results.append({"sol": sol, "res": res})

    print("\n\n" + "=" * 85)
    print("                 PREDICTIVE ARCHITECTURE 90% 2-LAP LEADERBOARD                 ")
    print("=" * 85)
    print(f"{'Pathfinder Architecture':<48} | {'Status':<12} | {'Lap 1':<8} | {'Lap 2':<8} | {'Max Dev':<8}")
    print("-" * 85)
    for r in results:
        res = r["res"]
        name = r["sol"]["name"]
        if res["status"] == "PASS_2_LAPS":
            print(f"{name:<48} | PASS 2 LAPS  | {res['lap_1']:5.2f}s | {res['lap_2']:5.2f}s | {res['max_dev']:4.1f} cm")
        else:
            print(f"{name:<48} | CRASHED      | ---     | ---     | {res.get('max_dev', 0):4.1f} cm")
    print("=" * 85)

if __name__ == "__main__":
    main()
