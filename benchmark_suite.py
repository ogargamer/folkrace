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
RESULTS = {}

def set_config(pathfinder_name, pace_pct=75.0):
    with open(CONFIG_PATH, "r") as f:
        cfg = json.load(f)
    cfg["active_pathfinder"] = pathfinder_name
    cfg["race_pace_pct"] = pace_pct
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)
    print(f"\n=======================================================", flush=True)
    print(f"  BENCHMARK CONFIG: {pathfinder_name} @ {pace_pct:.0f}% PACE", flush=True)
    print(f"=======================================================", flush=True)

def run_simulation(duration_s=50):
    # Clear telemetry log
    if os.path.exists(LOG_PATH):
        try:
            os.remove(LOG_PATH)
        except Exception:
            pass

    env = os.environ.copy()
    env["MAX_BENCHMARK_LAPS"] = "2"

    cmd = [WEBOTS_EXE, "--mode=fast", "--batch", "--stdout", "--stderr", "--minimize", WORLD_PATH]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    try:
        p.wait(timeout=duration_s)
    except subprocess.TimeoutExpired:
        p.kill()

    # Read telemetry log
    laps = []
    telemetry_lines = []
    max_dev = 0.0
    if os.path.exists(LOG_PATH):
        with open(LOG_PATH, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                l = line.strip()
                if "LAP COMPLETED" in l:
                    laps.append(l)
                elif "v=" in l:
                    telemetry_lines.append(l)
                    m_dev = re.search(r"Dev=\s*([\+\-]?[\d\.]+)\s*cm", l)
                    if m_dev:
                        max_dev = max(max_dev, abs(float(m_dev.group(1))))

    return laps, telemetry_lines, max_dev

print("Starting Folkrace 4-Pathfinder Automated Benchmark Suite...", flush=True)

# Measure CPU Latency for each algorithm (1,000 synthetic iterations)
import sys
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "my_project", "controllers", "folkrace_drive"))
import numpy as np
import pandas as pd
from tof_pathfinders import FrenetTransformer, FrenetCorridorQP, GaussianElasticBand, FrenetLatticePlanner, BiasedDisparityExtender

df = pd.read_csv(os.path.join(os.path.dirname(os.path.abspath(__file__)), "my_project", "controllers", "folkrace_drive", "cyan_champion_trajectory.csv"))
frenet = FrenetTransformer(df['dist_m'].values, df['x_m'].values, df['y_m'].values, df['yaw_rad'].values, df['curvature_inv_m'].values)
qp = FrenetCorridorQP()
geb = GaussianElasticBand()
lat = FrenetLatticePlanner()
bde = BiasedDisparityExtender()

sample_tofs = np.full(12, 1.20)
sample_tofs[4] = 0.35
sample_tofs[5] = 0.30

latencies = {}
# QP Latency
t0 = time.perf_counter()
for _ in range(1000):
    qp.plan(0.6, 0.0, 0.0, sample_tofs, frenet, 34)
latencies["FRENETQP"] = (time.perf_counter() - t0) # ms per call is * 1000 / 1000 = ms

# GEB Latency
t0 = time.perf_counter()
for _ in range(1000):
    geb.plan(0.6, 0.0, 0.0, sample_tofs, frenet, 34)
latencies["ELASTICBAND"] = (time.perf_counter() - t0)

# Lattice Latency
t0 = time.perf_counter()
for _ in range(1000):
    lat.plan(0.6, 0.0, 0.0, sample_tofs, frenet, 34)
latencies["LATTICE"] = (time.perf_counter() - t0)

# BDE Latency
t0 = time.perf_counter()
for _ in range(1000):
    bde.compute(sample_tofs, 0.05)
latencies["DISPARITYEXTENDER"] = (time.perf_counter() - t0)

for pf in PATHFINDERS:
    set_config(pf, pace_pct=75.0)
    laps, telems, max_dev = run_simulation(duration_s=45)
    
    lap_times = []
    for l in laps:
        m = re.search(r"Time:\s*([\d\.]+)\s*s", l)
        if m:
            lap_times.append(float(m.group(1)))

    print(f"Results for {pf}: {len(lap_times)} laps completed. Times: {lap_times} | Max Dev: {max_dev:.1f} cm", flush=True)
    RESULTS[pf] = {
        "laps_completed": len(lap_times),
        "lap_times": lap_times,
        "best_lap_s": min(lap_times) if lap_times else None,
        "max_dev_cm": max_dev,
        "latency_ms": latencies.get(pf, 0.0)
    }

print("\n\n================================================================================", flush=True)
print("             FOLKRACE 4-PATHFINDER COMPREHENSIVE BENCHMARK                      ", flush=True)
print("================================================================================", flush=True)
print(f" {'Algorithm':<20} | {'Laps':<5} | {'Best Lap':<10} | {'Max Dev':<10} | {'Latency':<10}", flush=True)
print("--------------------------------------------------------------------------------", flush=True)
for pf, res in RESULTS.items():
    best_str = f"{res['best_lap_s']:.2f} s" if res['best_lap_s'] is not None else "Incomplete"
    print(f" {pf:<20} | {res['laps_completed']:<5} | {best_str:<10} | {res['max_dev_cm']:>5.1f} cm   | {res['latency_ms']:>6.3f} ms", flush=True)
print("================================================================================\n", flush=True)

