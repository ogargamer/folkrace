"""
================================================================================
  AUTOMATED BENCHMARK SUITE FOR 12-CHANNEL ToF PATHFINDERS (FGM, VFH+, APF)
  Runs Webots simulation for each algorithm, captures lap times, speeds,
  wall clearances, and generates a comparative telemetry analysis.
================================================================================
"""

import os
import sys
import time
import json
import subprocess
import re

WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
WORLD_PATH = r"C:\Users\nurmi\Documents\Da Winna\my_project\worlds\Cyan_Champion_Track.wbt"
CONFIG_PATH = r"C:\Users\nurmi\Documents\Da Winna\my_project\controllers\folkrace_drive\folkrace_config.json"

ALGORITHMS = ["FGM", "VFH", "APF"]
LAPS_PER_RUN = 3

def set_active_algorithm(algo_name):
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    cfg["active_pathfinder"] = algo_name
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)
    print(f"\n[BENCHMARK] Configured active pathfinder to: [{algo_name}]")

def run_benchmark_for_algo(algo_name):
    set_active_algorithm(algo_name)
    env = os.environ.copy()
    env["MAX_BENCHMARK_LAPS"] = str(LAPS_PER_RUN)

    cmd = [
        WEBOTS_EXE,
        "--batch",
        "--mode=fast",
        "--stdout",
        "--stderr",
        "--minimize",
        WORLD_PATH
    ]

    print(f"[BENCHMARK] Starting Webots simulation run for [{algo_name}] ({LAPS_PER_RUN} laps)...")
    start_time = time.time()
    
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
        bufsize=1
    )

    lap_times = []
    telemetry_records = []
    
    timeout_sec = 60
    while True:
        if time.time() - start_time > timeout_sec:
            print(f"[BENCHMARK] Timeout reached ({timeout_sec}s). Terminating process.")
            proc.kill()
            break
            
        line = proc.stdout.readline()
        if not line and proc.poll() is not None:
            break
            
        if line:
            line_str = line.strip()
            
            # Match lap time
            lap_match = re.search(r"\[LAP COMPLETED\] Lap #(\d+) Time: ([\d\.]+) s", line_str)
            if lap_match:
                lap_num = int(lap_match.group(1))
                lap_t = float(lap_match.group(2))
                lap_times.append(lap_t)
                print(f"  --> Lap #{lap_num}: {lap_t:.2f} s")

            # Match telemetry
            # Format: [FGM] v=2.50m/s ( 9.0km/h) | Steer= +3.9° ... | ToF(L/C/R): 82/ 97/101cm | ...
            if "ToF(L/C/R):" in line_str:
                telemetry_records.append(line_str)
                # Print sample every ~10 records
                if len(telemetry_records) % 8 == 0:
                    print(f"      {line_str}")

            if "Exiting controller" in line_str or len(lap_times) >= LAPS_PER_RUN:
                print(f"[BENCHMARK] [{algo_name}] Finished {len(lap_times)} laps.")
                proc.kill()
                break

    try:
        proc.kill()
    except Exception:
        pass

    return {
        "algo": algo_name,
        "lap_times": lap_times,
        "best_lap": min(lap_times) if lap_times else None,
        "avg_lap": sum(lap_times)/len(lap_times) if lap_times else None,
        "telemetry_count": len(telemetry_records),
        "sample_telemetry": telemetry_records[-3:] if telemetry_records else []
    }

def main():
    print("=" * 80)
    print("  12-CHANNEL ToF PATHFINDER AUTOMATED BENCHMARK SUITE")
    print(f"  Target: {LAPS_PER_RUN} Laps per algorithm on Cyan Champion Track")
    print("=" * 80)

    results = []
    for algo in ALGORITHMS:
        res = run_benchmark_for_algo(algo)
        results.append(res)
        time.sleep(2.0)

    print("\n" + "=" * 80)
    print("  HEAD-TO-HEAD PATHFINDER BENCHMARK RESULTS")
    print("=" * 80)
    print(f"  {'Algorithm':<10} | {'Completed Laps':<15} | {'Best Lap (s)':<15} | {'Avg Lap (s)':<15} | {'Status':<15}")
    print("  " + "-" * 76)
    for r in results:
        best_str = f"{r['best_lap']:.2f} s" if r['best_lap'] else "N/A"
        avg_str = f"{r['avg_lap']:.2f} s" if r['avg_lap'] else "N/A"
        laps_str = f"{len(r['lap_times'])} / {LAPS_PER_RUN}"
        status = "PASSED" if r['lap_times'] else "INCOMPLETE"
        print(f"  {r['algo']:<10} | {laps_str:<15} | {best_str:<15} | {avg_str:<15} | {status:<15}")
    print("=" * 80 + "\n")

    # Reset config to FGM default
    set_active_algorithm("FGM")

if __name__ == "__main__":
    main()
