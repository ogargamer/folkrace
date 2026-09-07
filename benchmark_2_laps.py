import subprocess
import os
import json
import time
import re

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "my_project", "controllers", "folkrace_drive", "folkrace_config.json")
LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "my_project", "controllers", "folkrace_drive", "controller_telemetry.log")
WORLD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "my_project", "worlds", "Cyan_Champion_Track.wbt")
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

PATHFINDERS = ["ELASTICBAND", "FRENETQP", "LATTICE", "DISPARITYEXTENDER"]

def run_2_laps(pathfinder_name, pace_pct=75.0, timeout_s=120):
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
    env["MAX_BENCHMARK_LAPS"] = "2"

    print(f"\n==================================================================", flush=True)
    print(f" BENCHMARKING 2 FULL LAPS: {pathfinder_name} @ {pace_pct:.0f}% PACE", flush=True)
    print(f"==================================================================", flush=True)

    t0 = time.time()
    cmd = [WEBOTS_EXE, "--mode=fast", "--batch", "--minimize", "--stdout", "--stderr", WORLD_PATH]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    try:
        p.wait(timeout=timeout_s)
        print(f"Completed 2 laps in {time.time() - t0:.1f}s wall time!", flush=True)
    except subprocess.TimeoutExpired:
        p.kill()
        print(f"[TIMEOUT] Simulation timed out after {timeout_s}s", flush=True)

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
                    print(f"  {l}", flush=True)
                elif "v=" in l:
                    m_v = re.search(r"v=([\d\.]+)m/s", l)
                    if m_v:
                        speeds.append(float(m_v.group(1)))
                    m_dev = re.search(r"Dev=([\+\-]?[\d\.]+)cm", l)
                    if m_dev:
                        max_dev = max(max_dev, abs(float(m_dev.group(1))))

    lap_times = []
    for l in laps:
        m = re.search(r"Time:\s*([\d\.]+)\s*s", l)
        if m:
            lap_times.append(float(m.group(1)))

    best_lap = min(lap_times) if lap_times else None
    avg_speed = sum(speeds) / max(len(speeds), 1)
    max_speed = max(speeds) if speeds else 0.0

    print(f"Summary for {pathfinder_name}:", flush=True)
    print(f"  Laps Completed: {len(lap_times)} / 2", flush=True)
    if lap_times:
        print(f"  Lap Times     : {lap_times} (Best: {best_lap:.2f} s)", flush=True)
    print(f"  Avg Speed     : {avg_speed:.2f} m/s ({avg_speed*3.6:.1f} km/h)", flush=True)
    print(f"  Max Dev       : {max_dev:.1f} cm", flush=True)

    return {
        "laps_completed": len(lap_times),
        "lap_times": lap_times,
        "best_lap": best_lap,
        "avg_speed": avg_speed,
        "max_speed": max_speed,
        "max_dev": max_dev,
        "wall_time": elapsed_wall
    }

if __name__ == "__main__":
    results = {}
    for pf in PATHFINDERS:
        res = run_2_laps(pf, pace_pct=75.0, timeout_s=120)
        results[pf] = res

    print("\n\n================================================================================", flush=True)
    print("                2-LAP HEAD-TO-HEAD BENCHMARK RESULTS                            ", flush=True)
    print("================================================================================", flush=True)
    print(f" {'Algorithm':<18} | {'Laps':<5} | {'Lap 1 Time':<12} | {'Lap 2 Time':<12} | {'Max Dev':<10}", flush=True)
    print("--------------------------------------------------------------------------------", flush=True)
    
    best_algorithm = None
    best_time = 1e9

    for pf, r in results.items():
        l1_str = f"{r['lap_times'][0]:.2f} s" if len(r['lap_times']) >= 1 else "Incomplete"
        l2_str = f"{r['lap_times'][1]:.2f} s" if len(r['lap_times']) >= 2 else "Incomplete"
        print(f" {pf:<18} | {r['laps_completed']:<5} | {l1_str:<12} | {l2_str:<12} | {r['max_dev']:>5.1f} cm", flush=True)

        if r["best_lap"] is not None and r["best_lap"] < best_time:
            best_time = r["best_lap"]
            best_algorithm = pf

    print("================================================================================", flush=True)

    if best_algorithm:
        print(f"\n>>> WINNING PATHFINDER: {best_algorithm} (Best Lap: {best_time:.2f} s) <<<", flush=True)
        print(f"Setting {best_algorithm} as default in folkrace_config.json...", flush=True)
        with open(CONFIG_PATH, "r") as f:
            cfg = json.load(f)
        cfg["active_pathfinder"] = best_algorithm
        cfg["race_pace_pct"] = 75.0
        with open(CONFIG_PATH, "w") as f:
            json.dump(cfg, f, indent=2)
        print("folkrace_config.json updated successfully!\n", flush=True)
