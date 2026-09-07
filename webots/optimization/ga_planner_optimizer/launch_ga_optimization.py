"""
================================================================================
  1-CLICK LAUNCHER FOR HEADLESS GENETIC ALGORITHM PATH PLANNER OPTIMIZATION
================================================================================
  Usage:
    python launch_ga_optimization.py --planner option1
    python launch_ga_optimization.py --planner option2
    python launch_ga_optimization.py --planner option3 --generations 50 --cores 6
================================================================================
"""

import os
import sys
import argparse
import subprocess

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
GA_OPTIMIZER_DIR = CURR_DIR
WORKSPACE_ROOT = os.path.dirname(GA_OPTIMIZER_DIR)

for p in [GA_OPTIMIZER_DIR, WORKSPACE_ROOT]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from ga_planner_config import (
    NUM_CPU_CORES,
    POPULATION_SIZE,
    NUM_GENERATIONS,
    EVALUATION_DURATION_S,
    ACTIVE_PLANNER,
    ALGORITHM,
)


def main():
    parser = argparse.ArgumentParser(description="Launch Headless Optimizer for ToF Path Planners")
    parser.add_argument("--planner", type=str, default=ACTIVE_PLANNER, choices=["all", "ALL", "option1", "option2", "option3", "OPTION_1_BVP", "OPTION_2_SL_MPC", "OPTION_3_SEPTIC"], help="Path planner to optimize (or 'all' for all 3)")
    parser.add_argument("--algorithm", type=str, default=ALGORITHM, choices=["CMA_ES", "cma_es", "cma", "GA", "ga", "genetic_algorithm"], help="Optimization engine (CMA_ES or GA)")
    parser.add_argument("--cores", type=int, default=NUM_CPU_CORES, help="Number of CPU cores / Webots instances")
    parser.add_argument("--pop-size", type=int, default=POPULATION_SIZE, help="Population size")
    parser.add_argument("--generations", type=int, default=NUM_GENERATIONS, help="Generations")
    parser.add_argument("--duration", type=float, default=EVALUATION_DURATION_S, help="Trial evaluation duration in sim seconds (default: 20s)")
    parser.add_argument("--resume", action="store_true", help="Resume from last checkpoint")

    args = parser.parse_args()

    # Normalize planner argument
    p_map = {
        "all": "ALL",
        "option1": "OPTION_1_BVP",
        "option2": "OPTION_2_SL_MPC",
        "option3": "OPTION_3_SEPTIC",
    }
    planner_key = p_map.get(args.planner.lower(), args.planner.upper())

    cmd = [
        sys.executable,
        os.path.join(GA_OPTIMIZER_DIR, "webots/optimization/ga_optimizer.py"),
        "--planner", planner_key,
        "--algorithm", args.algorithm,
        "--cores", str(args.cores),
        "--pop-size", str(args.pop_size),
        "--generations", str(args.generations),
        "--duration", str(args.duration),
    ]
    if args.resume:
        cmd.append("--resume")

    print("=" * 80, flush=True)
    print("  LAUNCHING HEADLESS GA OPTIMIZER", flush=True)
    print(f"  Command: {' '.join(cmd)}", flush=True)
    print("=" * 80, flush=True)

    subprocess.run(cmd, cwd=GA_OPTIMIZER_DIR)


if __name__ == "__main__":
    main()
