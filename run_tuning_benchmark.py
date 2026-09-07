"""
================================================================================
  RUN TUNING BENCHMARK (STREAMING REALTIME OUTPUT)
  Runs Webots simulation for all 3 planners, records telemetry, and prints analysis.
================================================================================
"""

import os
import sys
import subprocess
import time

WORKSPACE_ROOT = r"c:\Users\nurmi\Documents\Da Winna"
WEBOTS_BIN = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
WORLD_PATH = os.path.join(WORKSPACE_ROOT, "my_project", "worlds", "Cyan_Champion_ToF_Track.wbt")
ANALYZER_SCRIPT = os.path.join(WORKSPACE_ROOT, "my_project", "controllers", "tof_path_planners", "analyze_planner_telemetry.py")

PLANNERS = [
    ("OPTION1_BVP", "telemetry_option1_bvp.jsonl"),
    ("OPTION2_SL_MPC", "telemetry_option2_sl_mpc.jsonl"),
    ("OPTION3_SEPTIC", "telemetry_option3_septic.jsonl"),
]


def run_benchmark(duration_s: float = 10.0):
    print("=" * 80)
    print(f"  LAUNCHING WEBOTS TUNING BENCHMARK ({duration_s:.1f}s per planner)")
    print("=" * 80)

    for p_type, telem_name in PLANNERS:
        telem_abs = os.path.join(WORKSPACE_ROOT, telem_name)
        if os.path.exists(telem_abs):
            try:
                os.remove(telem_abs)
            except Exception:
                pass

        print(f"\n[RUNNING] {p_type} for {duration_s:.1f}s...", flush=True)
        env = os.environ.copy()
        env["PLANNER_TYPE"] = p_type
        env["WEBOTS_RUN_SECONDS"] = str(duration_s)
        env["TELEMETRY_LOG_FILE"] = telem_abs
        env["PYTHONUNBUFFERED"] = "1"

        cmd = [
            WEBOTS_BIN,
            "--mode=fast",
            "--batch",
            "--stdout",
            "--stderr",
            WORLD_PATH,
        ]

        t0 = time.time()
        proc = subprocess.Popen(
            cmd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
        )

        for line in proc.stdout:
            if "Dist=" in line and "T=" in line:
                print("  " + line.strip(), flush=True)

        proc.wait(timeout=45)
        elapsed = time.time() - t0
        print(f"[COMPLETED] {p_type} in {elapsed:.1f}s wall time.\n", flush=True)

    # Run Analysis
    print("\n" + "#" * 80)
    print("  RUNNING COMPREHENSIVE TELEMETRY ANALYSIS")
    print("#" * 80)
    subprocess.run([sys.executable, ANALYZER_SCRIPT], check=False)


if __name__ == "__main__":
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    run_benchmark(dur)
