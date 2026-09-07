import subprocess
import os
import json
import time
import re

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "my_project", "controllers", "folkrace_drive", "folkrace_config.json")
LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "my_project", "controllers", "folkrace_drive", "controller_telemetry.log")
WORLD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "my_project", "worlds", "Cyan_Champion_Track.wbt")
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

PATHFINDERS = ["FRENETQP", "ELASTICBAND", "LATTICE", "DISPARITYEXTENDER"]
PACES = [60.0, 70.0, 80.0, 88.0, 92.0]

def test_algorithm_pace(pathfinder_name, pace_pct, duration_s=18):
    with open(CONFIG_PATH, "r") as f:
        cfg = json.load(f)
    cfg["active_pathfinder"] = pathfinder_name
    cfg["race_pace_pct"] = pace_pct
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)

    cmd = [WEBOTS_EXE, "--mode=fast", "--batch", "--minimize", "--stdout", "--stderr", WORLD_PATH]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        stdout_data, _ = p.communicate(timeout=duration_s)
    except subprocess.TimeoutExpired:
        p.kill()
        stdout_data, _ = p.communicate()

    time.sleep(0.3)

    log_content = stdout_data
    if os.path.exists(LOG_PATH):
        try:
            with open(LOG_PATH, "r", encoding="utf-8", errors="ignore") as f:
                log_content += "\n" + f.read()
        except Exception:
            pass

    speeds, devs, crashes, slips = [], [], [], []
    for line in log_content.splitlines():
        l = line.strip()
        if "CRASH DETECTED" in l:
            crashes.append(l)
        elif "idx=" in l:
            m_v = re.search(r"v=([\d\.]+)m/s", l)
            if m_v: speeds.append(float(m_v.group(1)))
            m_dev = re.search(r"Dev=([\+\-]?[\d\.]+)cm", l)
            if m_dev: devs.append(abs(float(m_dev.group(1))))
            m_s = re.search(r"\[(NORMAL|UNDERSTEER|OVERSTEER)\]", l)
            if m_s: slips.append(m_s.group(1))

    avg_v = sum(speeds) / max(len(speeds), 1)
    max_v = max(speeds) if speeds else 0.0
    max_dev = max(devs) if devs else 0.0
    norm_pct = slips.count("NORMAL") / max(len(slips), 1) * 100.0
    theo_lap = 13.94 / max(avg_v, 0.1)

    return {
        "pathfinder": pathfinder_name,
        "pace_pct": pace_pct,
        "crashes": len(crashes),
        "crash_samples": crashes[:2],
        "avg_v": avg_v,
        "max_v": max_v,
        "max_dev": max_dev,
        "norm_pct": norm_pct,
        "theo_lap": theo_lap
    }

if __name__ == "__main__":
    print("\n================================================================================", flush=True)
    print("      AUTOMATED PROGRESSIVE PACE LADDER & ZERO-CRASH BENCHMARK                  ", flush=True)
    print("================================================================================\n", flush=True)

    summary_table = {}

    for pf in PATHFINDERS:
        print(f"\n----------------------------------------------------------------------", flush=True)
        print(f" EVALUATING: {pf}", flush=True)
        print(f"----------------------------------------------------------------------", flush=True)
        best_clean = None

        for pace in PACES:
            res = test_algorithm_pace(pf, pace, duration_s=18)
            status = f"CLEAN (0 Crashes)" if res["crashes"] == 0 else f"CRASHED ({res['crashes']} impacts)"
            print(f"  Pace {pace:>2.0f}%: Avg={res['avg_v']:.2f} m/s ({res['avg_v']*3.6:4.1f}km/h), Top={res['max_v']:.2f} m/s, Dev={res['max_dev']:>4.1f}cm, Lap={res['theo_lap']:.2f}s | {status}", flush=True)
            if res["crashes"] == 0:
                best_clean = res
            else:
                for c in res["crash_samples"]:
                    print(f"       -> {c.strip()}", flush=True)
                print(f"    --> Stopped ladder for {pf} at {pace:.0f}% pace.", flush=True)
                break

        summary_table[pf] = best_clean

    print("\n\n================================================================================", flush=True)
    print("               FINAL ZERO-CRASH PERFORMANCE RESULTS                             ", flush=True)
    print("================================================================================", flush=True)
    print(f" {'Algorithm':<18} | {'Max Clean Pace':<15} | {'Avg Speed':<14} | {'Top Speed':<14} | {'Zero-Crash Lap':<15}", flush=True)
    print("--------------------------------------------------------------------------------", flush=True)

    overall_champion = None
    fastest_clean_time = 1e9

    for pf, r in summary_table.items():
        if r is not None:
            print(f" {pf:<18} | {r['pace_pct']:>3.0f} % Pace      | {r['avg_v']:.2f} m/s ({r['avg_v']*3.6:4.1f}km/h) | {r['max_v']:.2f} m/s ({r['max_v']*3.6:4.1f}km/h) | {r['theo_lap']:.2f} s (0 Crashes)", flush=True)
            if r["theo_lap"] < fastest_clean_time:
                fastest_clean_time = r["theo_lap"]
                overall_champion = r
        else:
            print(f" {pf:<18} | FAILED (Crashed at all paces)                                     ", flush=True)

    print("================================================================================\n", flush=True)

    if overall_champion:
        win_pf = overall_champion["pathfinder"]
        win_pace = overall_champion["pace_pct"]
        print(f">>> ZERO-CRASH CHAMPION: {win_pf} @ {win_pace:.0f}% Pace <<<", flush=True)
        print(f"Lap Time: {overall_champion['theo_lap']:.2f} s | Avg Speed: {overall_champion['avg_v']:.2f} m/s ({overall_champion['avg_v']*3.6:.1f} km/h) | Max Dev: {overall_champion['max_dev']:.1f} cm", flush=True)
        print(f"Applying {win_pf} @ {win_pace:.0f}% pace as default in folkrace_config.json...", flush=True)
        with open(CONFIG_PATH, "r") as f:
            cfg = json.load(f)
        cfg["active_pathfinder"] = win_pf
        cfg["race_pace_pct"] = win_pace
        with open(CONFIG_PATH, "w") as f:
            json.dump(cfg, f, indent=2)
        print("folkrace_config.json updated successfully!\n", flush=True)
