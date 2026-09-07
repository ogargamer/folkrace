import os, sys, math, time, json, subprocess
import numpy as np

webots_bin = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
world_file = os.path.abspath(r"my_project/worlds/Cyan_Champion_Track.wbt")
cfg_file = os.path.abspath(r"my_project/controllers/folkrace_drive/folkrace_config.json")
log_path = os.path.abspath(r"my_project/controllers/folkrace_drive/controller_telemetry.log")

configs = [
    # --- WERLING LATTICE WITH DYNAMIC SAFETY MARGINS ---
    {
        "id": "WERLING_DYNAMIC_MODERATE",
        "title": "Tier 1 (FrenetQP) + Werling-13 (Dynamic Margin, Moderate Gap)",
        "pathfinder": "WERLING_LATTICE",
        "pace": 85.0,
        "top_speed": 2.80,
        "env": {
            "WEBOTS_RUN_SECONDS": "15.0",
            "ENABLE_MOVING_OBSTACLE": "1",
            "MOVING_OBS_SPEED": "0.50",
            "WERLING_MU": "0.75",
            "EVADE_FRONT_CLEAR": "0.75",
            "EVADE_FRONT_THRESH": "0.55",
            "EVADE_FLANK_THRESH": "0.22"
        }
    },
    {
        "id": "WERLING_DYNAMIC_TIGHT",
        "title": "Tier 1 (FrenetQP) + Werling-13 (Dynamic Margin, Tight Gap)",
        "pathfinder": "WERLING_LATTICE",
        "pace": 85.0,
        "top_speed": 2.80,
        "env": {
            "WEBOTS_RUN_SECONDS": "15.0",
            "ENABLE_MOVING_OBSTACLE": "1",
            "MOVING_OBS_SPEED": "0.50",
            "WERLING_MU": "0.75",
            "EVADE_FRONT_CLEAR": "0.65",
            "EVADE_FRONT_THRESH": "0.45",
            "EVADE_FLANK_THRESH": "0.18"
        }
    },

    # --- MPPI WITH DYNAMIC GAUSSIAN POTENTIAL ---
    {
        "id": "MPPI_DYNAMIC_MODERATE",
        "title": "Tier 1 (FrenetQP) + MPPI-14 (Dynamic Sigma, Moderate Gap)",
        "pathfinder": "MPPI",
        "pace": 85.0,
        "top_speed": 2.80,
        "env": {
            "WEBOTS_RUN_SECONDS": "15.0",
            "ENABLE_MOVING_OBSTACLE": "1",
            "MOVING_OBS_SPEED": "0.50",
            "MPPI_K": "700",
            "MPPI_N": "18",
            "MPPI_TEMP": "8.0",
            "MPPI_SIG_STEER": "0.18",
            "MPPI_SIG_ACCEL": "3.40",
            "EVADE_FRONT_CLEAR": "0.75",
            "EVADE_FRONT_THRESH": "0.55",
            "EVADE_FLANK_THRESH": "0.22"
        }
    },
    {
        "id": "MPPI_DYNAMIC_TIGHT",
        "title": "Tier 1 (FrenetQP) + MPPI-14 (Dynamic Sigma, Tight Gap)",
        "pathfinder": "MPPI",
        "pace": 85.0,
        "top_speed": 2.80,
        "env": {
            "WEBOTS_RUN_SECONDS": "15.0",
            "ENABLE_MOVING_OBSTACLE": "1",
            "MOVING_OBS_SPEED": "0.50",
            "MPPI_K": "700",
            "MPPI_N": "18",
            "MPPI_TEMP": "8.0",
            "MPPI_SIG_STEER": "0.18",
            "MPPI_SIG_ACCEL": "3.40",
            "EVADE_FRONT_CLEAR": "0.65",
            "EVADE_FRONT_THRESH": "0.45",
            "EVADE_FLANK_THRESH": "0.18"
        }
    },

    # --- MPCC WITH DYNAMIC CORRIDOR SHIFT ---
    {
        "id": "MPCC_DYNAMIC_MODERATE",
        "title": "Tier 1 (FrenetQP) + MPCC-02 (Dynamic Shift, Moderate Gap)",
        "pathfinder": "MPCC",
        "pace": 85.0,
        "top_speed": 2.80,
        "env": {
            "WEBOTS_RUN_SECONDS": "15.0",
            "ENABLE_MOVING_OBSTACLE": "1",
            "MOVING_OBS_SPEED": "0.50",
            "MPCC_N": "10",
            "MPCC_MU": "0.85",
            "EVADE_FRONT_CLEAR": "0.75",
            "EVADE_FRONT_THRESH": "0.55",
            "EVADE_FLANK_THRESH": "0.22"
        }
    },
    {
        "id": "MPCC_DYNAMIC_TIGHT",
        "title": "Tier 1 (FrenetQP) + MPCC-02 (Dynamic Shift, Tight Gap)",
        "pathfinder": "MPCC",
        "pace": 85.0,
        "top_speed": 2.80,
        "env": {
            "WEBOTS_RUN_SECONDS": "15.0",
            "ENABLE_MOVING_OBSTACLE": "1",
            "MOVING_OBS_SPEED": "0.50",
            "MPCC_N": "10",
            "MPCC_MU": "0.85",
            "EVADE_FRONT_CLEAR": "0.65",
            "EVADE_FRONT_THRESH": "0.45",
            "EVADE_FLANK_THRESH": "0.18"
        }
    },

    # --- FRENETQP CONTROL BASELINE ---
    {
        "id": "FRENETQP_BASELINE",
        "title": "Tier 1 (FrenetQP Baseline)",
        "pathfinder": "FRENETQP",
        "pace": 85.0,
        "top_speed": 2.80,
        "env": {
            "WEBOTS_RUN_SECONDS": "15.0",
            "ENABLE_MOVING_OBSTACLE": "1",
            "MOVING_OBS_SPEED": "0.50",
            "EVADE_FRONT_CLEAR": "0.75",
            "EVADE_FRONT_THRESH": "0.55",
            "EVADE_FLANK_THRESH": "0.22"
        }
    }
]

def run_test(exp):
    print(f"\n" + "="*80)
    print(f"  RUNNING 15.0s WEBOTS TEST: {exp['title']}")
    print(f"  Pathfinder: {exp['pathfinder']} | Top Speed: {exp['top_speed']} m/s | Pace: {exp['pace']}%")
    print(f"  Env Parameters: {exp['env']}")
    print("="*80)

    # 1. Update config
    with open(cfg_file, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    cfg["active_pathfinder"] = exp["pathfinder"]
    cfg["race_pace_pct"] = exp["pace"]
    cfg["top_speed_m_s"] = exp["top_speed"]
    with open(cfg_file, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)

    # 2. Clear log
    if os.path.exists(log_path):
        try:
            os.remove(log_path)
        except Exception:
            pass

    # 3. Launch Webots
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
        proc = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=40)
        cli_out = proc.stdout + "\n" + proc.stderr
    except subprocess.TimeoutExpired:
        print("[TIMEOUT] 15s Webots run exceeded timeout.")
        cli_out = ""

    wall_time = time.perf_counter() - t0

    # 4. Parse Telemetry
    log_content = ""
    if os.path.exists(log_path):
        try:
            with open(log_path, "r", encoding="utf-8") as f:
                log_content = f.read()
        except Exception as e:
            print(f"[LOGGER] Error: {e}")

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

    total_samples = len(speeds)
    mean_v = float(np.mean(speeds)) if speeds else 0.0
    max_v = float(np.max(speeds)) if speeds else 0.0
    mean_dev = float(np.mean(lateral_devs)) if lateral_devs else 0.0
    max_dev = float(np.max(lateral_devs)) if lateral_devs else 0.0
    mean_steer = float(np.mean(steer_angles)) if steer_angles else 0.0
    min_fr = float(np.min(min_fronts)) if min_fronts else 150.0
    normal_pct = round((normal_count / max(1, total_samples)) * 100, 1)
    est_distance = round(mean_v * 15.0, 2)

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
        "last_pos": last_position
    }

    print(f"\n  === 15.0s RESULT: {exp['id']} ===")
    print(f"  Distance Covered: {res['est_distance_m']} m (Mean Speed: {res['mean_v_mps']} m/s, Max: {res['max_v_mps']} m/s)")
    print(f"  Lateral Deviation: Mean = {res['mean_dev_cm']} cm | Max = {res['max_dev_cm']} cm")
    print(f"  Tire Grip & Stability: Normal = {res['normal_grip_pct']}% | Understeer = {res['understeer_events']} | Oversteer = {res['oversteer_events']}")
    print(f"  Clearances & Collisions: Min Front = {res['min_front_cm']} cm | Crashes = {res['crash_count']} | Recoveries = {res['recovery_events']}")

    return res

all_results = []
for exp in configs:
    r = run_test(exp)
    all_results.append(r)

os.makedirs("racing_results", exist_ok=True)
with open(r"racing_results/dynamic_gap_15s_benchmark_results.json", "w", encoding="utf-8") as f:
    json.dump(all_results, f, indent=2)

print("\n" + "="*80)
print("  DYNAMIC GAP & ACTIVATION CORRIDOR BENCHMARK COMPLETE")
print("="*80)
