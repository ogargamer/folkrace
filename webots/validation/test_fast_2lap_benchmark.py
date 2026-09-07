import os
import sys
import json
import time
import shutil
import subprocess

CONTROLLER_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "controllers", "folkrace_drive")
CONFIG_PATH = os.path.join(CONTROLLER_DIR, "folkrace_config.json")
ACTIVE_TRAJ_PATH = os.path.join(CONTROLLER_DIR, "cyan_champion_trajectory.csv")
WORLD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "worlds", "Cyan_Champion_Track.wbt")
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
SRC_14CM = os.path.join(os.path.dirname(os.path.abspath(__file__)), "racing_results", "margin_10_to_16cm_winners", "winner_margin_14cm.csv")

shutil.copyfile(SRC_14CM, ACTIVE_TRAJ_PATH)

PACES = [85.0, 90.0, 95.0, 100.0]

def evaluate_pace(pace_pct):
    with open(CONFIG_PATH, "r") as f:
        cfg = json.load(f)
    cfg["active_pathfinder"] = "FRENETQP"
    cfg["race_pace_pct"] = pace_pct
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)

    env = os.environ.copy()
    env["MAX_BENCHMARK_LAPS"] = "2"
    env["QUIT_ON_CRASH"] = "1"

    cmd = [WEBOTS_EXE, "--mode=fast", "--batch", "--minimize", "--stdout", "--stderr", WORLD_PATH]
    t0 = time.perf_counter()
    try:
        proc = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=30)
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

    is_clean = len(lap_times) >= 2
    if proc.returncode == 0 and is_clean:
        return {
            "status": "PASS",
            "lap_1": lap_times[0],
            "lap_2": lap_times[1],
            "max_dev": round(max_dev, 1),
            "wall_s": round(wall_time, 1)
        }
    else:
        crash_msg = "Crash"
        for line in output.splitlines():
            if "CRASH DETECTED" in line or "CRASH ABORT" in line or "Bumper" in line:
                crash_msg = line.strip()
                break
        return {
            "status": "CRASH",
            "msg": crash_msg,
            "laps_done": len(lap_times),
            "laps": lap_times,
            "max_dev": round(max_dev, 1),
            "wall_s": round(wall_time, 1)
        }

def main():
    print("=" * 80)
    print("      2-LAP BENCHMARK (5% STEPS: 85%, 90%, 95%, 100% PACE) IN FAST MODE       ")
    print("=" * 80)

    for pace in PACES:
        res = evaluate_pace(pace)
        if res["status"] == "PASS":
            print(f"  Pace {pace:5.1f}%: Lap 1 = {res['lap_1']:5.2f}s | Lap 2 = {res['lap_2']:5.2f}s | MaxDev = {res['max_dev']:4.1f}cm | PASS (0 Crashes) [Sim in {res['wall_s']:.1f}s]")
        else:
            msg = res.get("msg", "Crash")
            done = res.get("laps_done", 0)
            print(f"  Pace {pace:5.1f}%: CRASHED on Lap #{done+1} -> {msg[:60]} (MaxDev={res.get('max_dev',0)}cm) [in {res.get('wall_s',0):.1f}s]")

if __name__ == "__main__":
    main()
