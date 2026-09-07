import subprocess
import os
import json
import time
import re

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "controllers", "folkrace_drive", "folkrace_config.json")
LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "controllers", "folkrace_drive", "controller_telemetry.log")
WORLD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "worlds", "Cyan_Champion_Track.wbt")
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

PATHFINDERS = ["ELASTICBAND", "FRENETQP", "LATTICE", "DISPARITYEXTENDER"]

def run_20s(pathfinder_name, pace_pct=75.0, run_duration_s=20):
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

    print(f"\n=======================================================", flush=True)
    print(f" BENCHMARK: {pathfinder_name} (20s Run @ {pace_pct:.0f}% Pace)", flush=True)
    print(f"=======================================================", flush=True)

    cmd = [WEBOTS_EXE, "--mode=fast", "--batch", "--minimize", "--stdout", "--stderr", WORLD_PATH]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    try:
        p.wait(timeout=run_duration_s)
    except subprocess.TimeoutExpired:
        p.kill()

    indices = []
    speeds = []
    devs = []
    slip_states = []

    if os.path.exists(LOG_PATH):
        with open(LOG_PATH, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                l = line.strip()
                if "idx=" in l:
                    m_idx = re.search(r"idx=\s*(\d+)", l)
                    if m_idx:
                        indices.append(int(m_idx.group(1)))
                    m_v = re.search(r"v=([\d\.]+)m/s", l)
                    if m_v:
                        speeds.append(float(m_v.group(1)))
                    m_dev = re.search(r"Dev=([\+\-]?[\d\.]+)cm", l)
                    if m_dev:
                        devs.append(abs(float(m_dev.group(1))))
                    m_slip = re.search(r"\[(NORMAL|UNDERSTEER|OVERSTEER|STUCK)\]", l)
                    if m_slip:
                        slip_states.append(m_slip.group(1))

    start_idx = indices[0] if indices else 0
    end_idx = indices[-1] if indices else 0
    progress_wps = (end_idx - start_idx) if (end_idx >= start_idx) else (1000 - start_idx + end_idx)

    avg_v = (sum(speeds) / len(speeds)) if speeds else 0.0
    max_v = max(speeds) if speeds else 0.0
    avg_dev = (sum(devs) / len(devs)) if devs else 0.0
    max_dev = max(devs) if devs else 0.0
    normal_pct = (slip_states.count("NORMAL") / len(slip_states) * 100.0) if slip_states else 0.0

    print(f"Results for {pathfinder_name}:", flush=True)
    print(f"  Waypoints Traversed : {progress_wps} wps (idx {start_idx} -> {end_idx})", flush=True)
    print(f"  Speed Profile       : Avg={avg_v:.2f} m/s ({avg_v*3.6:.1f} km/h), Top={max_v:.2f} m/s ({max_v*3.6:.1f} km/h)", flush=True)
    print(f"  Lateral Deviation   : Avg={avg_dev:.1f} cm, Max={max_dev:.1f} cm", flush=True)
    print(f"  Grip Stability      : {normal_pct:.0f}% Normal Traction\n", flush=True)

    # Score: higher progress is good, lower deviation is good, higher stability is good
    composite_score = progress_wps * 1.0 - max_dev * 2.0 + (normal_pct * 0.5)

    return {
        "pathfinder": pathfinder_name,
        "progress_wps": progress_wps,
        "avg_v": avg_v,
        "max_v": max_v,
        "avg_dev": avg_dev,
        "max_dev": max_dev,
        "normal_pct": normal_pct,
        "score": composite_score
    }

if __name__ == "__main__":
    results = {}
    for pf in PATHFINDERS:
        res = run_20s(pf, pace_pct=75.0, run_duration_s=20)
        results[pf] = res

    print("\n\n================================================================================", flush=True)
    print("                 20-SECOND PATHFINDER BENCHMARK SUMMARY                         ", flush=True)
    print("================================================================================", flush=True)
    print(f" {'Algorithm':<18} | {'Progress':<12} | {'Avg Speed':<14} | {'Max Dev':<10} | {'Stability':<10}", flush=True)
    print("--------------------------------------------------------------------------------", flush=True)
    
    best_pf = None
    best_score = -1e9

    for pf, r in results.items():
        print(f" {pf:<18} | {r['progress_wps']:>3d} waypoints | {r['avg_v']:.2f} m/s ({r['avg_v']*3.6:4.1f}km/h) | {r['max_dev']:>5.1f} cm  | {r['normal_pct']:>5.0f} %", flush=True)
        if r["score"] > best_score:
            best_score = r["score"]
            best_pf = pf

    print("================================================================================", flush=True)
    print(f"\n>>> BEST PATHFINDER: {best_pf} (Highest Progress & Cleanest Tracking) <<<", flush=True)
    print(f"Setting {best_pf} as default in folkrace_config.json...", flush=True)
    
    with open(CONFIG_PATH, "r") as f:
        cfg = json.load(f)
    cfg["active_pathfinder"] = best_pf
    cfg["race_pace_pct"] = 75.0
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)
    print("folkrace_config.json successfully updated with winning pathfinder!\n", flush=True)
