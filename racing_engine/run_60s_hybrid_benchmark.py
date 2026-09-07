import os, sys, math, time, json, subprocess
import numpy as np

webots_bin = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
world_file = os.path.abspath(r"my_project/worlds/Cyan_Champion_Track.wbt")
cfg_file = os.path.abspath(r"my_project/controllers/folkrace_drive/folkrace_config.json")
log_path = os.path.abspath(r"my_project/controllers/folkrace_drive/controller_telemetry.log")

experiments = [
    {
        "id": "TIER1_PLUS_WERLING_13",
        "title": "Tier 1 (FrenetQP) + Tier 2.A (Werling-13 Lattice)",
        "pathfinder": "WERLING_LATTICE",
        "pace": 90.0,
        "top_speed": 3.20,
        "env": {
            "WEBOTS_RUN_SECONDS": "60.0",
            "ENABLE_MOVING_OBSTACLE": "1",
            "MOVING_OBS_SPEED": "0.50",
            "WERLING_MU": "0.75"
        }
    },
    {
        "id": "TIER1_PLUS_MPPI_14",
        "title": "Tier 1 (FrenetQP) + Tier 2.B (MPPI-14 Stochastic)",
        "pathfinder": "MPPI",
        "pace": 90.0,
        "top_speed": 3.20,
        "env": {
            "WEBOTS_RUN_SECONDS": "60.0",
            "ENABLE_MOVING_OBSTACLE": "1",
            "MOVING_OBS_SPEED": "0.50",
            "MPPI_K": "700",
            "MPPI_N": "18",
            "MPPI_TEMP": "8.0",
            "MPPI_SIG_STEER": "0.18",
            "MPPI_SIG_ACCEL": "3.40"
        }
    },
    {
        "id": "TIER1_PLUS_MPCC_02",
        "title": "Tier 1 (FrenetQP) + Tier 2.C (MPCC-02 Contouring)",
        "pathfinder": "MPCC",
        "pace": 90.0,
        "top_speed": 3.20,
        "env": {
            "WEBOTS_RUN_SECONDS": "60.0",
            "ENABLE_MOVING_OBSTACLE": "1",
            "MOVING_OBS_SPEED": "0.50",
            "MPCC_N": "10",
            "MPCC_MU": "0.85"
        }
    },
    {
        "id": "TIER1_PURE_FRENETQP",
        "title": "Tier 1 (Pure FrenetQP Baseline Control)",
        "pathfinder": "FRENETQP",
        "pace": 90.0,
        "top_speed": 3.20,
        "env": {
            "WEBOTS_RUN_SECONDS": "60.0",
            "ENABLE_MOVING_OBSTACLE": "1",
            "MOVING_OBS_SPEED": "0.50"
        }
    }
]

def run_60s_test(exp):
    print(f"\n" + "="*80)
    print(f"  LAUNCHING 60.0s WEBOTS RUN: {exp['title']}")
    print(f"  Pathfinder: {exp['pathfinder']} | Top Speed: {exp['top_speed']} m/s | Pace: {exp['pace']}%")
    print(f"  Environment: {exp['env']}")
    print("="*80)

    # 1. Update config file
    with open(cfg_file, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    cfg["active_pathfinder"] = exp["pathfinder"]
    cfg["race_pace_pct"] = exp["pace"]
    cfg["top_speed_m_s"] = exp["top_speed"]
    with open(cfg_file, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)

    # 2. Clear previous log
    if os.path.exists(log_path):
        try:
            os.remove(log_path)
        except Exception:
            pass

    # 3. Launch Webots Process
    env = os.environ.copy()
    env.update(exp["env"])

    cmd = [
        webots_bin,
        "--batch",
        "--mode=fast",
        "--minimize",
        "--stdout",
        "--stderr",
        world_file
    ]

    t0 = time.perf_counter()
    try:
        proc = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=120)
        cli_out = proc.stdout + "\n" + proc.stderr
    except subprocess.TimeoutExpired:
        print("[TIMEOUT] 60s Webots run exceeded timeout.")
        cli_out = ""

    wall_time = time.perf_counter() - t0

    # 4. Parse Telemetry Log
    log_content = ""
    if os.path.exists(log_path):
        try:
            with open(log_path, "r", encoding="utf-8") as f:
                log_content = f.read()
        except Exception as e:
            print(f"[LOGGER] Error reading telemetry log: {e}")

    lines = log_content.splitlines() if log_content else cli_out.splitlines()

    speeds = []
    lateral_devs = []
    steer_angles = []
    min_fronts = []
    normal_count = 0
    understeer_count = 0
    oversteer_count = 0
    crash_count = 0
    recovery_count = 0
    laps_completed = 0
    total_steps = 0
    last_position = (0.0, 0.0)

    for line in lines:
        l_str = line.strip()
        if "Dev=" in l_str and "v=" in l_str:
            total_steps += 1
            try:
                v_val = float(l_str.split("v=")[1].split("m/s")[0])
                speeds.append(v_val)
            except Exception:
                pass
            try:
                dev_val = float(l_str.split("Dev=")[1].split("cm")[0].replace("+", "").strip())
                lateral_devs.append(abs(dev_val))
            except Exception:
                pass
            try:
                str_val = float(l_str.split("Str=")[1].split("°")[0].replace("+", "").strip())
                steer_angles.append(abs(str_val))
            except Exception:
                pass
            try:
                fr_val = float(l_str.split("Fr=")[1].split("cm")[0].strip())
                min_fronts.append(fr_val)
            except Exception:
                pass
            try:
                pos_str = l_str.split("(")[1].split(")")[0]
                p_parts = pos_str.split(",")
                last_position = (float(p_parts[0]), float(p_parts[1]))
            except Exception:
                pass

            if "[NORMAL]" in l_str:
                normal_count += 1
            elif "[UNDERSTEER]" in l_str:
                understeer_count += 1
            elif "[OVERSTEER]" in l_str:
                oversteer_count += 1

            if "[PIVOT_REVERSE]" in l_str or "[RECOVERY]" in l_str:
                recovery_count += 1

        elif "[CRASH DETECTED" in l_str:
            crash_count += 1
        elif "Lap " in l_str and "Time:" in l_str:
            laps_completed += 1

    total_samples = len(speeds)
    mean_v = float(np.mean(speeds)) if speeds else 0.0
    max_v = float(np.max(speeds)) if speeds else 0.0
    mean_dev = float(np.mean(lateral_devs)) if lateral_devs else 0.0
    max_dev = float(np.max(lateral_devs)) if lateral_devs else 0.0
    mean_steer = float(np.mean(steer_angles)) if steer_angles else 0.0
    min_fr = float(np.min(min_fronts)) if min_fronts else 150.0
    normal_pct = round((normal_count / max(1, total_samples)) * 100, 1)
    est_distance = round(mean_v * 60.0, 2)

    res = {
        "id": exp["id"],
        "title": exp["title"],
        "pathfinder": exp["pathfinder"],
        "wall_time_s": round(wall_time, 2),
        "steps_simulated": total_steps,
        "est_distance_m": est_distance,
        "mean_v_mps": round(mean_v, 2),
        "max_v_mps": round(max_v, 2),
        "mean_dev_cm": round(mean_dev, 1),
        "max_dev_cm": round(max_dev, 1),
        "mean_steer_deg": round(mean_steer, 1),
        "min_front_cm": round(min_fr, 1),
        "normal_grip_pct": normal_pct,
        "understeer_events": understeer_count,
        "oversteer_events": oversteer_count,
        "crash_count": crash_count,
        "recovery_events": recovery_count,
        "laps_completed": laps_completed,
        "last_pos": last_position
    }

    print(f"\n  === 60.0s WEBOTS RUN RESULT: {exp['id']} ===")
    print(f"  Distance Covered Across 60s: {res['est_distance_m']} m (Mean Speed: {res['mean_v_mps']} m/s, Max: {res['max_v_mps']} m/s)")
    print(f"  Lateral Tracking Accuracy  : Mean Dev = {res['mean_dev_cm']} cm | Max Dev = {res['max_dev_cm']} cm")
    print(f"  Tire Grip & Stability      : Normal Grip = {res['normal_grip_pct']}% | Understeer = {res['understeer_events']} | Oversteer = {res['oversteer_events']}")
    print(f"  Obstacle Clearances        : Min Front Clearance = {res['min_front_cm']} cm")
    print(f"  Crashes / Recoveries       : Crashes = {res['crash_count']} | Recovery Activations = {res['recovery_events']}")
    print(f"  Laps Completed             : {res['laps_completed']}")

    return res

print("=== STARTING 60.0s LIVE WEBOTS HYBRID TIER 1 + TIER 2 RUNS ===")

all_results = []
for exp in experiments:
    r = run_60s_test(exp)
    all_results.append(r)

os.makedirs("racing_results", exist_ok=True)
with open(r"racing_results/webots_60s_hybrid_benchmark_results.json", "w", encoding="utf-8") as f:
    json.dump(all_results, f, indent=2)

print("\n" + "="*80)
print("  ALL 60.0s WEBOTS RUNS COMPLETED SUCCESSFULLY")
print("="*80)
