import subprocess
import os
import json
import time
import re

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "controllers", "folkrace_drive", "folkrace_config.json")
LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "controllers", "folkrace_drive", "controller_telemetry.log")
WORLD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "worlds", "Cyan_Champion_Track.wbt")
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

PATHFINDERS = ["FRENETQP"]
PACES = [70.0, 75.0, 80.0, 85.0]

def run_single_pace(pathfinder_name, pace_pct):
    with open(CONFIG_PATH, "r") as f:
        cfg = json.load(f)
    cfg["active_pathfinder"] = pathfinder_name
    cfg["race_pace_pct"] = pace_pct
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)

    env = os.environ.copy()
    env["MAX_BENCHMARK_LAPS"] = "1"
    env["QUIT_ON_CRASH"] = "1"

    cmd = [WEBOTS_EXE, "--mode=fast", "--batch", "--minimize", "--stdout", "--stderr", WORLD_PATH]
    t0 = time.time()
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    try:
        stdout_data, _ = p.communicate(timeout=50)
    except subprocess.TimeoutExpired:
        p.kill()
        stdout_data, _ = p.communicate()
    elapsed_wall = time.time() - t0

    # Parse results from log
    lap_time = None
    crashes = []
    speeds = []
    devs = []

    if os.path.exists(LOG_PATH):
        try:
            with open(LOG_PATH, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    if "LAP COMPLETED" in line:
                        m_lap = re.search(r"Time:\s*([\d\.]+)\s*s", line)
                        if m_lap:
                            lap_time = float(m_lap.group(1))
                    elif "CRASH DETECTED" in line:
                        crashes.append(line.strip())
                    elif "idx=" in line:
                        m_v = re.search(r"v=([\d\.]+)m/s", line)
                        if m_v: speeds.append(float(m_v.group(1)))
                        m_dev = re.search(r"Dev=([\+\-]?[\d\.]+)cm", line)
                        if m_dev: devs.append(abs(float(m_dev.group(1))))
        except Exception:
            pass

    avg_v = (sum(speeds) / len(speeds)) if speeds else 0.0
    max_v = max(speeds) if speeds else 0.0
    max_dev = max(devs) if devs else 0.0

    return {
        "pathfinder": pathfinder_name,
        "pace_pct": pace_pct,
        "lap_time": lap_time,
        "crashed": len(crashes) > 0 or p.returncode == 1,
        "crash_info": crashes[0] if crashes else ("Crash exit code 1" if p.returncode == 1 else "None"),
        "avg_v": avg_v,
        "max_v": max_v,
        "max_dev": max_dev,
        "wall_time": elapsed_wall
    }

if __name__ == "__main__":
    print("\n================================================================================", flush=True)
    print("      FAST ZERO-CRASH PROGRESSIVE PACE LADDER & BENCHMARK                       ", flush=True)
    print("================================================================================\n", flush=True)

    ladder_summary = {}

    for pf in PATHFINDERS:
        print(f"----------------------------------------------------------------------", flush=True)
        print(f" EVALUATING: {pf}", flush=True)
        print(f"----------------------------------------------------------------------", flush=True)
        best_clean_for_pf = None

        for pace in PACES:
            res = run_single_pace(pf, pace)
            if not res["crashed"] and res["lap_time"] is not None:
                print(f"  Pace {pace:>2.0f}%: Lap Time = {res['lap_time']:.2f} s | Avg={res['avg_v']:.2f} m/s ({res['avg_v']*3.6:4.1f}km/h), Top={res['max_v']:.2f} m/s, Dev={res['max_dev']:>4.1f}cm | CLEAN (0 Crashes) [{res['wall_time']:.1f}s wall]", flush=True)
                best_clean_for_pf = res
            else:
                print(f"  Pace {pace:>2.0f}%: CRASHED -> {res['crash_info']} [{res['wall_time']:.1f}s wall]", flush=True)
                print(f"       --> Halting ladder for {pf} at {pace:.0f}% pace limit.", flush=True)
                break

        ladder_summary[pf] = best_clean_for_pf
        print("", flush=True)

    print("\n================================================================================", flush=True)
    print("                 FINAL ZERO-CRASH CHAMPIONSHIP SUMMARY                          ", flush=True)
    print("================================================================================", flush=True)
    print(f" {'Algorithm':<18} | {'Best Safe Pace':<15} | {'Lap Time':<12} | {'Avg Speed':<14} | {'Max Dev':<10}", flush=True)
    print("--------------------------------------------------------------------------------", flush=True)

    best_overall = None
    best_time = 1e9

    for pf, r in ladder_summary.items():
        if r is not None:
            print(f" {pf:<18} | {r['pace_pct']:>3.0f} % Pace      | {r['lap_time']:>5.2f} s      | {r['avg_v']:.2f} m/s ({r['avg_v']*3.6:4.1f}km/h) | {r['max_dev']:>5.1f} cm", flush=True)
            if r["lap_time"] < best_time:
                best_time = r["lap_time"]
                best_overall = r
        else:
            print(f" {pf:<18} | FAILED (Crashed at all paces)                                     ", flush=True)

    print("================================================================================\n", flush=True)

    if best_overall:
        win_pf = best_overall["pathfinder"]
        win_pace = best_overall["pace_pct"]
        print(f">>> OFFICIAL ZERO-CRASH CHAMPION: {win_pf} @ {win_pace:.0f}% Pace <<<", flush=True)
        print(f"Lap Time: {best_overall['lap_time']:.2f} s | Avg Speed: {best_overall['avg_v']:.2f} m/s ({best_overall['avg_v']*3.6:.1f} km/h) | Max Dev: {best_overall['max_dev']:.1f} cm | 0 Crashes", flush=True)
        print(f"Applying {win_pf} @ {win_pace:.0f}% as default in folkrace_config.json...", flush=True)
        with open(CONFIG_PATH, "r") as f:
            cfg = json.load(f)
        cfg["active_pathfinder"] = win_pf
        cfg["race_pace_pct"] = win_pace
        with open(CONFIG_PATH, "w") as f:
            json.dump(cfg, f, indent=2)
        print("folkrace_config.json updated successfully!\n", flush=True)
