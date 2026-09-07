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

def run_pace_test(pathfinder_name, pace_pct=85.0, run_duration_s=22):
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
    print(f" TESTING: {pathfinder_name} @ {pace_pct:.0f}% PACE (Target: Sub 6.0s Lap)", flush=True)
    print(f"==================================================================", flush=True)

    cmd = [WEBOTS_EXE, "--mode=fast", "--batch", "--minimize", "--stdout", "--stderr", WORLD_PATH]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    try:
        p.wait(timeout=run_duration_s)
    except subprocess.TimeoutExpired:
        p.kill()

    laps = []
    speeds = []
    devs = []
    slip_states = []

    if os.path.exists(LOG_PATH):
        with open(LOG_PATH, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                l = line.strip()
                if "LAP COMPLETED" in l:
                    laps.append(l)
                    print(f"  >>> {l} <<<", flush=True)
                elif "idx=" in l:
                    m_v = re.search(r"v=([\d\.]+)m/s", l)
                    if m_v:
                        speeds.append(float(m_v.group(1)))
                    m_dev = re.search(r"Dev=([\+\-]?[\d\.]+)cm", l)
                    if m_dev:
                        devs.append(abs(float(m_dev.group(1))))
                    m_slip = re.search(r"\[(NORMAL|UNDERSTEER|OVERSTEER|STUCK)\]", l)
                    if m_slip:
                        slip_states.append(m_slip.group(1))

    lap_times = []
    for l in laps:
        m = re.search(r"Time:\s*([\d\.]+)\s*s", l)
        if m:
            lap_times.append(float(m.group(1)))

    best_lap = min(lap_times) if lap_times else None
    avg_v = (sum(speeds) / len(speeds)) if speeds else 0.0
    max_v = max(speeds) if speeds else 0.0
    max_dev = max(devs) if devs else 0.0
    normal_pct = (slip_states.count("NORMAL") / len(slip_states) * 100.0) if slip_states else 0.0

    print(f"Summary for {pathfinder_name} @ {pace_pct:.0f}% Pace:", flush=True)
    print(f"  Laps Completed : {len(lap_times)}", flush=True)
    if lap_times:
        print(f"  Lap Times      : {lap_times} (Best: {best_lap:.2f} s)", flush=True)
    print(f"  Speed Profile  : Avg={avg_v:.2f} m/s ({avg_v*3.6:.1f} km/h), Top={max_v:.2f} m/s ({max_v*3.6:.1f} km/h)", flush=True)
    print(f"  Max Deviation  : {max_dev:.1f} cm", flush=True)
    print(f"  Traction Grip  : {normal_pct:.0f}% Normal\n", flush=True)

    return {
        "pathfinder": pathfinder_name,
        "pace_pct": pace_pct,
        "laps": len(lap_times),
        "best_lap": best_lap,
        "avg_v": avg_v,
        "max_v": max_v,
        "max_dev": max_dev,
        "normal_pct": normal_pct
    }

if __name__ == "__main__":
    results = {}
    for pf in PATHFINDERS:
        res = run_pace_test(pf, pace_pct=85.0, run_duration_s=22)
        results[pf] = res

    print("\n\n================================================================================", flush=True)
    print("                 SUB-6s RACING BENCHMARK COMPARISON                             ", flush=True)
    print("================================================================================", flush=True)
    print(f" {'Algorithm':<18} | {'Pace':<6} | {'Avg Speed':<14} | {'Top Speed':<14} | {'Max Dev':<10} | {'Traction':<10}", flush=True)
    print("--------------------------------------------------------------------------------", flush=True)
    
    best_pf = None
    best_score = -1e9

    for pf, r in results.items():
        print(f" {pf:<18} | {r['pace_pct']:>3.0f} % | {r['avg_v']:.2f} m/s ({r['avg_v']*3.6:4.1f}km/h) | {r['max_v']:.2f} m/s ({r['max_v']*3.6:4.1f}km/h) | {r['max_dev']:>5.1f} cm  | {r['normal_pct']:>5.0f} %", flush=True)
        # Score = higher speed and higher traction with lower deviation
        score = r['avg_v'] * 10.0 + (r['normal_pct'] * 0.1) - (r['max_dev'] * 0.5)
        if score > best_score:
            best_score = score
            best_pf = pf

    print("================================================================================", flush=True)
    print(f"\n>>> WINNING SUB-6s PATHFINDER: {best_pf} <<<", flush=True)
    print(f"Applying {best_pf} as default in folkrace_config.json...", flush=True)

    with open(CONFIG_PATH, "r") as f:
        cfg = json.load(f)
    cfg["active_pathfinder"] = best_pf
    cfg["race_pace_pct"] = 85.0
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)
    print("folkrace_config.json updated successfully!\n", flush=True)
