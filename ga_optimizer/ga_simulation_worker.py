"""
================================================================================
  GENETIC ALGORITHM SIMULATION EVALUATOR FOR FOLKRACE MPPI
================================================================================
  Executes an isolated headless Webots simulation instance, evaluates a candidate
  MPPIGenome over a specified duration (e.g., 20.0 sim seconds), captures
  structured JSON telemetry, and computes the fitness score.
================================================================================
"""

import os
import sys
import json
import time
import math
import subprocess
from typing import Dict, Tuple, Optional, Any, Union

import numpy as np

# Ensure controllers are on sys.path
WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FOLKRACE_DIR = os.path.join(WORKSPACE_ROOT, "my_project", "controllers", "folkrace_drive")
MPPI_DIR = os.path.join(WORKSPACE_ROOT, "my_project", "controllers", "mppi_controller")

for p in [WORKSPACE_ROOT, FOLKRACE_DIR, MPPI_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from mppi_folkrace_controller import MPPIGenome

# Webots binary and world defaults
DEFAULT_WEBOTS_BIN = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
DEFAULT_WORLD_PATH = os.path.join(WORKSPACE_ROOT, "my_project", "worlds", "Cyan_Champion_Track_GA.wbt")


def compute_fitness(metrics: Dict[str, Any]) -> float:
    """
    Computes a scalar optimization objective from Webots trial metrics.
    Higher score is strictly better.
    
    Structure:
      - Clean Surviving Trial (collisions == 0 and not stuck):
        + Distance Traveled (200 pts per meter)
        + Mean Speed Reward (100 pts per m/s)
        + Lap Completion Bonus (2000 pts per lap)
        - Wall Proximity Hazard (0.5 pts per unit cost)
        - Steering Jerk Penalty (10 pts per rad^2 variance)
        -> Positive scores (+500 to +20,000 pts)
      
      - Crashed or Stalled Trial:
        - Severe Disqualification Base (-50,000 pts)
        + Distance Progress credit (10 pts per meter up to crash)
        -> Negative scores (-50,000 to -48,000 pts)
    """
    dist_m = float(metrics.get("distance_m", 0.0))
    mean_speed = float(metrics.get("mean_speed_mps", 0.0))
    laps = int(metrics.get("laps_completed", 0))
    collisions = int(metrics.get("collision_count", 0))
    wall_cost = float(metrics.get("mean_wall_proximity_cost", 0.0))
    steering_var = float(metrics.get("steering_variance", 0.0))
    stuck = bool(metrics.get("stuck_abort", False))

    if collisions > 0 or stuck:
        # Crashed or stalled run
        fitness = -50000.0 + (dist_m * 10.0) - (collisions * 1000.0)
    else:
        # Clean surviving run
        dist_reward = dist_m * 200.0
        speed_reward = mean_speed * 100.0
        lap_reward = laps * 2000.0
        wall_penalty = wall_cost * 0.5
        jerk_penalty = steering_var * 10.0
        fitness = dist_reward + speed_reward + lap_reward - wall_penalty - jerk_penalty

    return float(fitness)


def evaluate_genome(
    genome_or_array: Union[MPPIGenome, np.ndarray, list],
    worker_id: int = 0,
    sim_duration_s: float = 20.0,
    timeout_wall_s: float = 45.0,
    webots_bin: str = DEFAULT_WEBOTS_BIN,
    world_path: str = DEFAULT_WORLD_PATH,
    verbose: bool = False,
) -> Tuple[float, Dict[str, Any]]:
    """
    Runs an isolated headless Webots simulation instance for a candidate genome.
    
    Args:
        genome_or_array: MPPIGenome instance or 23-element array.
        worker_id: Unique integer (0..N-1) for port and resource isolation.
        sim_duration_s: Simulation time limit (default: 20.0 seconds).
        timeout_wall_s: Real-world timeout to kill hung processes (default: 45.0s).
        webots_bin: Path to webots.exe.
        world_path: Path to .wbt world file.
        verbose: If True, prints simulation output lines.
        
    Returns:
        (fitness_score, raw_metrics_dictionary)
    """
    if hasattr(genome_or_array, "to_array"):
        genome_arr = genome_or_array.to_array()
        genome_obj = genome_or_array
    elif isinstance(genome_or_array, (np.ndarray, list)):
        genome_arr = np.array(genome_or_array, dtype=np.float64)
        genome_obj = MPPIGenome.from_array(genome_arr)
    else:
        raise ValueError(f"Unsupported genome type: {type(genome_or_array)}")

    port_offset = worker_id * 20
    eval_file = os.path.join(WORKSPACE_ROOT, f"eval_res_worker_{worker_id}.json")
    if os.path.exists(eval_file):
        try:
            os.remove(eval_file)
        except Exception:
            pass

    env = os.environ.copy()
    env["WEBOTS_RUN_SECONDS"] = str(float(sim_duration_s))
    env["MPPI_PORT_OFFSET"] = str(port_offset)
    env["MPPI_EVAL_FILE"] = eval_file
    env["MPPI_GENOME_JSON"] = json.dumps(genome_arr.tolist())

    # Build Webots command line for fast headless execution
    cmd = [
        webots_bin,
        "--mode=fast",
        "--batch",
        "--minimize",
        "--stdout",
        "--stderr",
        world_path,
    ]

    t0_wall = time.perf_counter()
    metrics = None
    raw_output_lines = []

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

        # Stream and parse output
        for line in proc.stdout:
            raw_output_lines.append(line)
            if verbose:
                print(f"[Worker {worker_id}] {line.rstrip()}", flush=True)

            if "[EVAL_RESULT_JSON]" in line:
                try:
                    json_str = line.split("[EVAL_RESULT_JSON]")[1].strip()
                    metrics = json.loads(json_str)
                except Exception as e:
                    if verbose:
                        print(f"[Worker {worker_id}] JSON parse error: {e}", flush=True)

        try:
            proc.wait(timeout=max(5.0, timeout_wall_s - (time.perf_counter() - t0_wall)))
        except subprocess.TimeoutExpired:
            proc.kill()
            if verbose:
                print(f"[Worker {worker_id}] Subprocess timed out after {timeout_wall_s:.1f}s. Killed.", flush=True)

    except Exception as e:
        if verbose:
            print(f"[Worker {worker_id}] Subprocess execution exception: {e}", flush=True)

    # Fallback to eval file if stdout didn't capture metrics
    if metrics is None and os.path.exists(eval_file):
        try:
            with open(eval_file, "r", encoding="utf-8") as ef:
                metrics = json.load(ef)
            if verbose:
                print(f"[Worker {worker_id}] Successfully retrieved metrics from {eval_file}", flush=True)
        except Exception as e:
            if verbose:
                print(f"[Worker {worker_id}] Error reading {eval_file}: {e}", flush=True)

    wall_elapsed_s = time.perf_counter() - t0_wall

    if metrics is None:
        # Fallback if Webots crashed or exited unexpectedly
        metrics = {
            "status": "PROCESS_ERROR",
            "distance_m": 0.0,
            "mean_speed_mps": 0.0,
            "max_speed_mps": 0.0,
            "laps_completed": 0,
            "lap_times": [],
            "best_lap_time_s": 999.0,
            "collision_count": 5,
            "mean_wall_proximity_cost": 50.0,
            "steering_variance": 0.5,
            "stuck_abort": True,
            "sim_duration_s": 0.0,
            "wall_elapsed_s": round(wall_elapsed_s, 2),
        }
    else:
        metrics["wall_elapsed_s"] = round(wall_elapsed_s, 2)

    fitness = compute_fitness(metrics)
    metrics["fitness"] = fitness
    metrics["genome"] = genome_arr.tolist()

    return fitness, metrics


if __name__ == "__main__":
    print("Testing single candidate evaluation on Worker 0 for 5.0 sim seconds...")
    test_genome = MPPIGenome()
    fit, met = evaluate_genome(test_genome, worker_id=0, sim_duration_s=5.0, verbose=True)
    print("\n--- Test Result ---")
    print(f"Fitness Score : {fit:.2f}")
    print(f"Distance (m)  : {met.get('distance_m', 0.0):.2f}")
    print(f"Mean Speed    : {met.get('mean_speed_mps', 0.0):.2f} m/s")
    print(f"Status        : {met.get('status')}")
    print(f"Wall Time     : {met.get('wall_elapsed_s')} s")
