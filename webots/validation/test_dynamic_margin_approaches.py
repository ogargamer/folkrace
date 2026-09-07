import os
import sys
import json
import time
import subprocess

WORLD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "worlds", "Cyan_Champion_Track.wbt")
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "controllers", "folkrace_drive", "folkrace_config.json")

APPROACHES = [
    ("QUADRATIC", "Quadratic Kinetic Energy Envelope"),
    ("LINEAR", "Linear Latency & Slew Envelope"),
    ("CURVATURE", "Curvature G-Force Scaling Envelope"),
    ("SIGMOID", "Sigmoidal S-Curve Saturation Envelope"),
    ("ASYMMETRIC", "Asymmetric Super-Elliptical Envelope")
]

VARIATIONS = [
    ("Tight (1.5cm - 4.5cm)", 0.015, 0.045),
    ("Standard (2.0cm - 5.5cm)", 0.020, 0.055),
    ("Generous (2.5cm - 6.5cm)", 0.025, 0.065)
]

def run_variation(mode_name, desc, var_name, d_min, d_max):
    # Ensure config has 100% pace
    with open(CONFIG_PATH, "r") as f:
        cfg = json.load(f)
    cfg["active_pathfinder"] = "FRENETQP"
    cfg["race_pace_pct"] = 100.0
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)

    env = os.environ.copy()
    env["MAX_BENCHMARK_LAPS"] = "1"
    env["QUIT_ON_CRASH"] = "1"
    env["ENABLE_MOVING_OBSTACLE"] = "1"
    env["MOVING_OBS_SPEED"] = "1.00"
    env["MOVING_OBS_START_S"] = "2.50"
    env["SHIELD_MARGIN_MODE"] = mode_name
    env["SHIELD_D_MIN"] = str(d_min)
    env["SHIELD_D_MAX"] = str(d_max)

    telemetry_path = os.path.join(os.path.dirname(CONFIG_PATH), "controller_telemetry.log")
    if os.path.exists(telemetry_path):
        try:
            os.remove(telemetry_path)
        except Exception:
            pass

    t0 = time.perf_counter()
    try:
        proc = subprocess.run([WEBOTS_EXE, "--mode=fast", "--batch", "--minimize", "--stdout", "--stderr", WORLD_PATH], env=env, capture_output=True, text=True, timeout=45)
    except subprocess.TimeoutExpired:
        return {"status": "TIMEOUT", "lap_time": None, "max_dev": 0.0, "wall_s": 45.0, "laps_done": 0}
    wall_s = time.perf_counter() - t0

    output = ""
    if os.path.exists(telemetry_path):
        with open(telemetry_path, "r", encoding="utf-8") as f:
            output = f.read()
    else:
        output = proc.stdout + "\n" + proc.stderr

    lap_times = []
    max_dev = 0.0
    devs = []
    crash_count = 0

    for line in output.splitlines():
        if "CRASH DETECTED" in line:
            crash_count += 1
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

    is_pass = len(lap_times) >= 1 and crash_count == 0
    return {
        "status": "PASS (0 Crashes)" if is_pass else f"CRASH ({crash_count})",
        "lap_time": lap_times[0] if len(lap_times) >= 1 else None,
        "max_dev": round(max_dev, 1),
        "wall_s": round(wall_s, 1),
        "laps_done": len(lap_times)
    }

def main():
    print("=" * 85)
    print("      CONTINUOUS DYNAMIC SAFETY MARGIN BENCHMARK (5 APPROACHES x 3 VARIATIONS)      ")
    print("=" * 85)

    results = []

    for mode, desc in APPROACHES:
        print(f"\n>>> TESTING APPROACH: {mode} ({desc}) <<<")
        for var_label, d_min, d_max in VARIATIONS:
            res = run_variation(mode, desc, var_label, d_min, d_max)
            res["mode"] = mode
            res["desc"] = desc
            res["variation"] = var_label
            res["d_min"] = d_min
            res["d_max"] = d_max
            results.append(res)

            if "PASS" in res["status"]:
                print(f"  [{var_label}]: Lap Time = {res['lap_time']:.2f}s | MaxDev = {res['max_dev']:.1f}cm | {res['status']} [Sim: {res['wall_s']:.1f}s]")
            else:
                print(f"  [{var_label}]: {res['status']} (Laps={res['laps_done']}) | MaxDev = {res['max_dev']:.1f}cm [Sim: {res['wall_s']:.1f}s]")

    # Save summary table
    out_json = os.path.join(os.path.dirname(os.path.abspath(__file__)), "racing_results", "dynamic_margin_benchmark_results.json")
    os.makedirs(os.path.dirname(out_json), exist_ok=True)
    with open(out_json, "w") as fp:
        json.dump(results, fp, indent=2)
    print("\nBenchmark results saved to:", out_json)

if __name__ == "__main__":
    main()
