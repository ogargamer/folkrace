"""
================================================================================
  WEBOTS AUTOMATED VERIFICATION HARNESS FOR ALL 3 PATH PLANNERS
  Runs Cyan_Champion_ToF_Track.wbt with:
    1. Option 1: BVP Dynamic State Lattice
    2. Option 2: Curvilinear SL-MPC (OSQP)
    3. Option 3: G^3 Septic Spline + Traction Profiler
================================================================================
"""

import os
import sys
import subprocess
import time

WORKSPACE_ROOT = r"c:\Users\nurmi\Documents\Da Winna"
WEBOTS_BIN = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
WORLD_PATH = os.path.join(WORKSPACE_ROOT, "my_project", "worlds", "Cyan_Champion_ToF_Track.wbt")

PLANNERS_TO_TEST = [
    ("OPTION1_BVP", "tof_bvp_lattice"),
    ("OPTION2_SL_MPC", "tof_sl_mpc"),
    ("OPTION3_SEPTIC", "tof_septic_profiler"),
]


def test_planner_in_webots(planner_type: str, controller_name: str, run_duration_s: float = 3.5):
    print("\n" + "=" * 80)
    print(f"  RUNNING WEBOTS TEST: {planner_type} (Controller: {controller_name})")
    print("=" * 80, flush=True)

    # Temporary update to world file to test specific controller if desired
    # Or inject via ROBOT_MODE / PLANNER_TYPE environment variable
    env = os.environ.copy()
    env["PLANNER_TYPE"] = planner_type
    env["WEBOTS_RUN_SECONDS"] = str(run_duration_s)
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
    try:
        proc = subprocess.Popen(
            cmd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
        )

        output_lines = []
        max_dist = 0.0
        max_speed = 0.0
        has_error = False

        for line in proc.stdout:
            print(line, end="", flush=True)
            output_lines.append(line)
            if "Error" in line or "Traceback" in line:
                has_error = True
            if "Dist=" in line:
                try:
                    parts = line.split("Dist=")
                    dist_val = float(parts[1].split("m")[0].strip())
                    if dist_val > max_dist:
                        max_dist = dist_val
                except Exception:
                    pass
            if "v_est=" in line:
                try:
                    parts = line.split("v_est=")
                    v_val = float(parts[1].split("m/s")[0].strip())
                    if v_val > max_speed:
                        max_speed = v_val
                except Exception:
                    pass

        proc.wait(timeout=30)
        elapsed = time.time() - t0
        print(f"\n>>> TEST RESULT FOR {planner_type} <<<")
        print(f"Exit Code: {proc.returncode}")
        print(f"Elapsed Wall Time: {elapsed:.2f} s")
        print(f"Max Speed Achieved: {max_speed:.2f} m/s")
        print(f"Distance Traveled: {max_dist:.2f} m")
        print(f"Errors Detected: {has_error}")
        return proc.returncode == 0 and not has_error and max_dist > 0.05
    except Exception as e:
        print(f"[Exception running Webots] {e}")
        return False


if __name__ == "__main__":
    results = {}
    for p_type, c_name in PLANNERS_TO_TEST:
        success = test_planner_in_webots(p_type, c_name, run_duration_s=3.0)
        results[p_type] = success

    print("\n" + "=" * 80)
    print("  FINAL SUMMARY OF WEBOTS EXECUTION TESTS:")
    print("=" * 80)
    for p_type, success in results.items():
        print(f"  {p_type:20s}: {'SUCCESS (Moved without errors)' if success else 'FAILED'}")
    print("=" * 80)
