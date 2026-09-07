import subprocess
import os
import json
import time
import re

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "my_project", "controllers", "folkrace_drive", "folkrace_config.json")
LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "my_project", "controllers", "folkrace_drive", "controller_telemetry.log")
WORLD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "my_project", "worlds", "Cyan_Champion_Track.wbt")
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

def run_single_lap(pathfinder_name, pace_pct=70.0, timeout_s=65):
    with open(CONFIG_PATH, "r") as f:
        cfg = json.load(f)
    cfg["active_pathfinder"] = pathfinder_name
    cfg["race_pace_pct"] = pace_pct
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)

    if os.path.exists(LOG_PATH):
        try:
            os.remove(LOG_PATH)
        except Exception:
            pass

    env = os.environ.copy()
    env["MAX_BENCHMARK_LAPS"] = "1"

    print(f"\n=======================================================", flush=True)
    print(f" BENCHMARK: {pathfinder_name} @ {pace_pct:.0f}% PACE", flush=True)
    print(f"=======================================================", flush=True)

    t0 = time.time()
    cmd = [WEBOTS_EXE, "--mode=fast", "--batch", "--minimize", "--stdout", "--stderr", WORLD_PATH]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    try:
        p.wait(timeout=timeout_s)
        print(f"Completed and exited cleanly in {time.time() - t0:.1f}s wall time.", flush=True)
    except subprocess.TimeoutExpired:
        p.kill()
        print(f"[TIMEOUT] Reached {timeout_s}s timeout limit.", flush=True)

    elapsed_wall = time.time() - t0

    laps = []
    max_dev = 0.0
    speeds = []
    if os.path.exists(LOG_PATH):
        with open(LOG_PATH, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                l = line.strip()
                if "LAP COMPLETED" in l:
                    laps.append(l)
                    print(f"  >>> {l} <<<", flush=True)
                elif "v=" in l:
                    m_v = re.search(r"v=([\d\.]+)m/s", l)
                    if m_v:
                        speeds.append(float(m_v.group(1)))
                    m_dev = re.search(r"Dev=([\+\-]?[\d\.]+)cm", l)
                    if m_dev:
                        max_dev = max(max_dev, abs(float(m_dev.group(1))))

    lap_time = None
    for l in laps:
        m = re.search(r"Time:\s*([\d\.]+)\s*s", l)
        if m:
            lap_time = float(m.group(1))

    avg_speed = sum(speeds) / max(len(speeds), 1)
    max_speed = max(speeds) if speeds else 0.0
    print(f"Result: Lap Time = {lap_time if lap_time else 'Incomplete'} | Avg Speed = {avg_speed:.2f} m/s | Max Speed = {max_speed:.2f} m/s | Max Dev = {max_dev:.1f} cm\n", flush=True)
    return {
        "lap_time": lap_time,
        "avg_speed": avg_speed,
        "max_speed": max_speed,
        "max_dev": max_dev,
        "wall_time": elapsed_wall
    }

if __name__ == "__main__":
    results = {}
    for pf in ["ELASTICBAND", "FRENETQP", "LATTICE", "DISPARITYEXTENDER"]:
        res = run_single_lap(pf, pace_pct=70.0, timeout_s=65)
        results[pf] = res

    print("\n\n================================================================================", flush=True)
    print("           FOLKRACE 4-PATHFINDER OFFICIAL 1-LAP CIRCUIT BENCHMARK               ", flush=True)
    print("================================================================================", flush=True)
    print(f" {'Algorithm':<18} | {'Lap Time':<10} | {'Avg Speed':<14} | {'Top Speed':<14} | {'Max Dev':<10}", flush=True)
    print("--------------------------------------------------------------------------------", flush=True)
    for pf, r in results.items():
        lap_str = f"{r['lap_time']:.2f} s" if r['lap_time'] is not None else "Incomplete"
        print(f" {pf:<18} | {lap_str:<10} | {r['avg_speed']:.2f} m/s ({r['avg_speed']*3.6:4.1f}km/h) | {r['max_speed']:.2f} m/s ({r['max_speed']*3.6:4.1f}km/h) | {r['max_dev']:>5.1f} cm", flush=True)
    print("================================================================================\n", flush=True)
