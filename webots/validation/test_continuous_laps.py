import subprocess
import os
import json
import time
import re

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "controllers", "folkrace_drive", "folkrace_config.json")
LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "controllers", "folkrace_drive", "controller_telemetry.log")
WORLD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "worlds", "Cyan_Champion_Track.wbt")
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

def run_test(pathfinder_name, pace_pct=70.0, target_laps=3, timeout_s=45):
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
    env["MAX_BENCHMARK_LAPS"] = str(target_laps)

    print(f"\n=======================================================", flush=True)
    print(f" TESTING: {pathfinder_name} @ {pace_pct:.0f}% PACE (Target: {target_laps} Laps)", flush=True)
    print(f"=======================================================", flush=True)

    t0 = time.time()
    cmd = [WEBOTS_EXE, "--mode=fast", "--batch", "--minimize", "--stdout", "--stderr", WORLD_PATH]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    try:
        p.wait(timeout=timeout_s)
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

    avg_speed = sum(speeds) / max(len(speeds), 1)
    max_speed = max(speeds) if speeds else 0.0
    print(f"\nSummary for {pathfinder_name}:", flush=True)
    print(f"  Laps Completed : {len(lap_times)} / {target_laps}", flush=True)
    if lap_times:
        print(f"  Lap Times      : {lap_times} (Best: {min(lap_times):.2f} s)", flush=True)
    print(f"  Speed Profile  : Avg={avg_speed:.2f} m/s ({avg_speed*3.6:.1f} km/h), Max={max_speed:.2f} m/s ({max_speed*3.6:.1f} km/h)", flush=True)
    print(f"  Max Deviation  : {max_dev:.1f} cm", flush=True)
    print(f"  Wall Time      : {elapsed_wall:.1f} s", flush=True)
    return {
        "laps": len(lap_times),
        "lap_times": lap_times,
        "best_lap": min(lap_times) if lap_times else None,
        "avg_speed": avg_speed,
        "max_speed": max_speed,
        "max_dev": max_dev
    }

if __name__ == "__main__":
    results = {}
    for pf in ["ELASTICBAND", "FRENETQP", "LATTICE", "DISPARITYEXTENDER"]:
        res = run_test(pf, pace_pct=70.0, target_laps=3, timeout_s=40)
        results[pf] = res

    print("\n\n================================================================================", flush=True)
    print("                FINAL 3-LAP MULTI-PATHFINDER BENCHMARK                          ", flush=True)
    print("================================================================================", flush=True)
    print(f" {'Algorithm':<18} | {'Laps':<5} | {'Best Lap':<10} | {'Avg Speed':<12} | {'Max Dev':<10}", flush=True)
    print("--------------------------------------------------------------------------------", flush=True)
    for pf, r in results.items():
        best_str = f"{r['best_lap']:.2f} s" if r['best_lap'] is not None else "Incomplete"
        print(f" {pf:<18} | {r['laps']:<5} | {best_str:<10} | {r['avg_speed']:.2f} m/s ({r['avg_speed']*3.6:.1f} km/h) | {r['max_dev']:>5.1f} cm", flush=True)
    print("================================================================================\n", flush=True)
