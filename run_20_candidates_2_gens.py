"""
================================================================================
  20 PROMISING CANDIDATES 2-GENERATION EVALUATION & COMPARATIVE ANALYSIS
================================================================================
"""

import os
import sys
import time
import json
import csv
import math
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Dict, List, Tuple, Any
import numpy as np

WORKSPACE_ROOT = os.path.dirname(os.path.abspath(__file__))
FOLKRACE_DIR = os.path.join(WORKSPACE_ROOT, "my_project", "controllers", "folkrace_drive")
MPPI_DIR = os.path.join(WORKSPACE_ROOT, "my_project", "controllers", "mppi_controller")
TOOLS_DIR = os.path.join(WORKSPACE_ROOT, "tools")

for p in [WORKSPACE_ROOT, FOLKRACE_DIR, MPPI_DIR, TOOLS_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from mppi_folkrace_controller import MPPIGenome
from ga_simulation_worker import evaluate_genome, compute_fitness, PersistentWorkerPool


def get_20_promising_candidates() -> List[Tuple[str, MPPIGenome]]:
    candidates = []

    # 1. High-Speed Curvature Sprinter
    candidates.append(("1. Speed Sprinter", MPPIGenome(
        H=22, K=512, dt=0.050, lambda_temp=0.25, ar1_alpha=0.82, gamma_discount=0.96, mu_friction=0.72,
        w_cte=12.0, w_heading=6.0, w_speed=2.0, w_hazard=10000.0, w_prob_costmap=2.0, w_unknown=2.5,
        w_steer_rate=0.60, w_accel_rate=0.30, w_terminal=22.0, sigma_steer=0.08, sigma_speed=0.30,
        lognorm_shape=0.30, ratio_center=0.45, ratio_straight=0.20, ratio_left=0.20, pure_pursuit_lookahead=0.35,
        r_inflation_m=0.088, w_reverse=2500.0, reverse_speed=1.5, t_rev_steps=2
    )))

    # 2. Smooth Gaussian Cruiser
    candidates.append(("2. Smooth Cruiser", MPPIGenome(
        H=24, K=512, dt=0.055, lambda_temp=0.55, ar1_alpha=0.85, gamma_discount=0.97, mu_friction=0.65,
        w_cte=8.0, w_heading=4.0, w_speed=1.2, w_hazard=10000.0, w_prob_costmap=2.5, w_unknown=3.0,
        w_steer_rate=1.20, w_accel_rate=0.40, w_terminal=18.0, sigma_steer=0.09, sigma_speed=0.25,
        lognorm_shape=0.25, ratio_center=0.40, ratio_straight=0.20, ratio_left=0.20, pure_pursuit_lookahead=0.45,
        r_inflation_m=0.092, w_reverse=2500.0, reverse_speed=1.4, t_rev_steps=2
    )))

    # 3. Agile Chicane Weaver
    candidates.append(("3. Chicane Weaver", MPPIGenome(
        H=20, K=512, dt=0.048, lambda_temp=0.40, ar1_alpha=0.50, gamma_discount=0.95, mu_friction=0.68,
        w_cte=9.0, w_heading=5.0, w_speed=1.5, w_hazard=10000.0, w_prob_costmap=2.0, w_unknown=2.0,
        w_steer_rate=0.50, w_accel_rate=0.35, w_terminal=20.0, sigma_steer=0.14, sigma_speed=0.35,
        lognorm_shape=0.30, ratio_center=0.30, ratio_straight=0.30, ratio_left=0.20, pure_pursuit_lookahead=0.32,
        r_inflation_m=0.085, w_reverse=2200.0, reverse_speed=1.5, t_rev_steps=2
    )))

    # 4. High-Traction Grip Maximizer
    candidates.append(("4. Grip Maximizer", MPPIGenome(
        H=22, K=512, dt=0.052, lambda_temp=0.35, ar1_alpha=0.75, gamma_discount=0.97, mu_friction=0.78,
        w_cte=14.0, w_heading=7.0, w_speed=1.8, w_hazard=10000.0, w_prob_costmap=2.0, w_unknown=3.0,
        w_steer_rate=0.70, w_accel_rate=0.30, w_terminal=25.0, sigma_steer=0.10, sigma_speed=0.30,
        lognorm_shape=0.28, ratio_center=0.45, ratio_straight=0.20, ratio_left=0.20, pure_pursuit_lookahead=0.38,
        r_inflation_m=0.088, w_reverse=2500.0, reverse_speed=1.5, t_rev_steps=2
    )))

    # 5. Conservative Safety Runner
    candidates.append(("5. Safety Runner", MPPIGenome(
        H=25, K=512, dt=0.058, lambda_temp=0.50, ar1_alpha=0.88, gamma_discount=0.97, mu_friction=0.60,
        w_cte=15.0, w_heading=6.0, w_speed=1.0, w_hazard=10000.0, w_prob_costmap=4.0, w_unknown=5.0,
        w_steer_rate=1.20, w_accel_rate=0.50, w_terminal=20.0, sigma_steer=0.07, sigma_speed=0.20,
        lognorm_shape=0.20, ratio_center=0.35, ratio_straight=0.15, ratio_left=0.25, pure_pursuit_lookahead=0.48,
        r_inflation_m=0.098, w_reverse=2800.0, reverse_speed=1.2, t_rev_steps=3
    )))

    # 6. Outer Corridor Dodger
    candidates.append(("6. Corridor Dodger", MPPIGenome(
        H=22, K=512, dt=0.052, lambda_temp=0.45, ar1_alpha=0.70, gamma_discount=0.96, mu_friction=0.66,
        w_cte=6.0, w_heading=4.0, w_speed=1.4, w_hazard=10000.0, w_prob_costmap=2.0, w_unknown=2.5,
        w_steer_rate=0.80, w_accel_rate=0.35, w_terminal=16.0, sigma_steer=0.12, sigma_speed=0.30,
        lognorm_shape=0.28, ratio_center=0.25, ratio_straight=0.25, ratio_left=0.35, pure_pursuit_lookahead=0.40,
        r_inflation_m=0.090, w_reverse=2400.0, reverse_speed=1.5, t_rev_steps=2
    )))

    # 7. Deep Predictive Planner
    candidates.append(("7. Deep Planner", MPPIGenome(
        H=30, K=480, dt=0.065, lambda_temp=0.40, ar1_alpha=0.86, gamma_discount=0.98, mu_friction=0.66,
        w_cte=10.0, w_heading=5.0, w_speed=1.5, w_hazard=10000.0, w_prob_costmap=2.5, w_unknown=3.0,
        w_steer_rate=0.90, w_accel_rate=0.30, w_terminal=15.0, sigma_steer=0.08, sigma_speed=0.22,
        lognorm_shape=0.25, ratio_center=0.40, ratio_straight=0.20, ratio_left=0.20, pure_pursuit_lookahead=0.52,
        r_inflation_m=0.092, w_reverse=2600.0, reverse_speed=1.4, t_rev_steps=3
    )))

    # 8. Fast Reflex Reactor
    candidates.append(("8. Fast Reflex", MPPIGenome(
        H=18, K=512, dt=0.042, lambda_temp=0.30, ar1_alpha=0.60, gamma_discount=0.94, mu_friction=0.70,
        w_cte=10.0, w_heading=6.0, w_speed=1.6, w_hazard=10000.0, w_prob_costmap=2.0, w_unknown=2.0,
        w_steer_rate=0.60, w_accel_rate=0.30, w_terminal=20.0, sigma_steer=0.12, sigma_speed=0.35,
        lognorm_shape=0.32, ratio_center=0.40, ratio_straight=0.25, ratio_left=0.20, pure_pursuit_lookahead=0.28,
        r_inflation_m=0.086, w_reverse=2200.0, reverse_speed=1.5, t_rev_steps=2
    )))

    # 9. Apex Hugger Purist
    candidates.append(("9. Apex Hugger", MPPIGenome(
        H=22, K=512, dt=0.048, lambda_temp=0.20, ar1_alpha=0.80, gamma_discount=0.95, mu_friction=0.72,
        w_cte=16.0, w_heading=8.0, w_speed=1.8, w_hazard=10000.0, w_prob_costmap=2.0, w_unknown=2.5,
        w_steer_rate=0.80, w_accel_rate=0.35, w_terminal=24.0, sigma_steer=0.08, sigma_speed=0.25,
        lognorm_shape=0.28, ratio_center=0.55, ratio_straight=0.15, ratio_left=0.15, pure_pursuit_lookahead=0.30,
        r_inflation_m=0.085, w_reverse=2600.0, reverse_speed=1.5, t_rev_steps=2
    )))

    # 10. Equal 4-Corridor Explorer
    candidates.append(("10. Equal Explorer", MPPIGenome(
        H=22, K=512, dt=0.052, lambda_temp=0.45, ar1_alpha=0.72, gamma_discount=0.96, mu_friction=0.66,
        w_cte=8.0, w_heading=4.5, w_speed=1.4, w_hazard=10000.0, w_prob_costmap=2.0, w_unknown=2.5,
        w_steer_rate=0.80, w_accel_rate=0.35, w_terminal=18.0, sigma_steer=0.10, sigma_speed=0.28,
        lognorm_shape=0.28, ratio_center=0.25, ratio_straight=0.25, ratio_left=0.25, pure_pursuit_lookahead=0.38,
        r_inflation_m=0.090, w_reverse=2400.0, reverse_speed=1.5, t_rev_steps=2
    )))

    # 11. High-Temp Soft Blender
    candidates.append(("11. Soft Blender", MPPIGenome(
        H=24, K=512, dt=0.055, lambda_temp=0.85, ar1_alpha=0.78, gamma_discount=0.97, mu_friction=0.65,
        w_cte=7.0, w_heading=3.5, w_speed=1.3, w_hazard=10000.0, w_prob_costmap=2.0, w_unknown=2.0,
        w_steer_rate=1.40, w_accel_rate=0.45, w_terminal=16.0, sigma_steer=0.09, sigma_speed=0.25,
        lognorm_shape=0.22, ratio_center=0.35, ratio_straight=0.20, ratio_left=0.25, pure_pursuit_lookahead=0.44,
        r_inflation_m=0.092, w_reverse=2500.0, reverse_speed=1.4, t_rev_steps=2
    )))

    # 12. Greedy Winner-Takes-All
    candidates.append(("12. Greedy Winner", MPPIGenome(
        H=20, K=512, dt=0.046, lambda_temp=0.12, ar1_alpha=0.75, gamma_discount=0.95, mu_friction=0.70,
        w_cte=12.0, w_heading=6.0, w_speed=1.8, w_hazard=10000.0, w_prob_costmap=2.0, w_unknown=2.0,
        w_steer_rate=0.70, w_accel_rate=0.30, w_terminal=22.0, sigma_steer=0.10, sigma_speed=0.30,
        lognorm_shape=0.30, ratio_center=0.40, ratio_straight=0.25, ratio_left=0.20, pure_pursuit_lookahead=0.32,
        r_inflation_m=0.086, w_reverse=2400.0, reverse_speed=1.5, t_rev_steps=2
    )))

    # 13. Reversing Specialist
    candidates.append(("13. Reverse Specialist", MPPIGenome(
        H=22, K=512, dt=0.052, lambda_temp=0.40, ar1_alpha=0.70, gamma_discount=0.96, mu_friction=0.66,
        w_cte=10.0, w_heading=5.0, w_speed=1.5, w_hazard=10000.0, w_prob_costmap=2.0, w_unknown=3.0,
        w_steer_rate=0.80, w_accel_rate=0.35, w_terminal=20.0, sigma_steer=0.10, sigma_speed=0.30,
        lognorm_shape=0.30, ratio_center=0.40, ratio_straight=0.20, ratio_left=0.20, pure_pursuit_lookahead=0.38,
        r_inflation_m=0.090, w_reverse=1600.0, reverse_speed=1.5, t_rev_steps=2
    )))

    # 14. Deep Wall Barrier Protect
    candidates.append(("14. Barrier Protect", MPPIGenome(
        H=24, K=512, dt=0.054, lambda_temp=0.45, ar1_alpha=0.82, gamma_discount=0.97, mu_friction=0.64,
        w_cte=11.0, w_heading=5.5, w_speed=1.2, w_hazard=10000.0, w_prob_costmap=5.0, w_unknown=4.0,
        w_steer_rate=1.00, w_accel_rate=0.40, w_terminal=20.0, sigma_steer=0.08, sigma_speed=0.22,
        lognorm_shape=0.25, ratio_center=0.35, ratio_straight=0.20, ratio_left=0.25, pure_pursuit_lookahead=0.42,
        r_inflation_m=0.095, w_reverse=3000.0, reverse_speed=1.3, t_rev_steps=3
    )))

    # 15. Wide Perturbation Explorer
    candidates.append(("15. Wide Explorer", MPPIGenome(
        H=20, K=512, dt=0.050, lambda_temp=0.50, ar1_alpha=0.60, gamma_discount=0.96, mu_friction=0.68,
        w_cte=8.0, w_heading=4.0, w_speed=1.5, w_hazard=10000.0, w_prob_costmap=2.0, w_unknown=2.0,
        w_steer_rate=0.60, w_accel_rate=0.30, w_terminal=18.0, sigma_steer=0.18, sigma_speed=0.40,
        lognorm_shape=0.35, ratio_center=0.35, ratio_straight=0.25, ratio_left=0.20, pure_pursuit_lookahead=0.35,
        r_inflation_m=0.088, w_reverse=2200.0, reverse_speed=1.5, t_rev_steps=2
    )))

    # 16. Low Noise Precision Rail
    candidates.append(("16. Precision Rail", MPPIGenome(
        H=24, K=512, dt=0.055, lambda_temp=0.30, ar1_alpha=0.90, gamma_discount=0.97, mu_friction=0.70,
        w_cte=14.0, w_heading=7.0, w_speed=1.6, w_hazard=10000.0, w_prob_costmap=2.0, w_unknown=2.5,
        w_steer_rate=1.00, w_accel_rate=0.35, w_terminal=22.0, sigma_steer=0.05, sigma_speed=0.15,
        lognorm_shape=0.18, ratio_center=0.50, ratio_straight=0.15, ratio_left=0.20, pure_pursuit_lookahead=0.40,
        r_inflation_m=0.088, w_reverse=2600.0, reverse_speed=1.4, t_rev_steps=2
    )))

    # 17. Tight Lookahead Lead
    candidates.append(("17. Tight Lookahead", MPPIGenome(
        H=20, K=512, dt=0.048, lambda_temp=0.35, ar1_alpha=0.70, gamma_discount=0.95, mu_friction=0.68,
        w_cte=12.0, w_heading=6.0, w_speed=1.6, w_hazard=10000.0, w_prob_costmap=2.0, w_unknown=2.0,
        w_steer_rate=0.70, w_accel_rate=0.30, w_terminal=20.0, sigma_steer=0.10, sigma_speed=0.30,
        lognorm_shape=0.28, ratio_center=0.40, ratio_straight=0.20, ratio_left=0.20, pure_pursuit_lookahead=0.22,
        r_inflation_m=0.088, w_reverse=2400.0, reverse_speed=1.5, t_rev_steps=2
    )))

    # 18. Long Lookahead Sweeper
    candidates.append(("18. Long Sweeper", MPPIGenome(
        H=26, K=512, dt=0.058, lambda_temp=0.45, ar1_alpha=0.88, gamma_discount=0.97, mu_friction=0.65,
        w_cte=8.0, w_heading=4.0, w_speed=1.3, w_hazard=10000.0, w_prob_costmap=2.5, w_unknown=3.0,
        w_steer_rate=1.10, w_accel_rate=0.40, w_terminal=15.0, sigma_steer=0.08, sigma_speed=0.22,
        lognorm_shape=0.22, ratio_center=0.35, ratio_straight=0.20, ratio_left=0.25, pure_pursuit_lookahead=0.58,
        r_inflation_m=0.092, w_reverse=2600.0, reverse_speed=1.3, t_rev_steps=3
    )))

    # 19. Unknown-Space Avoider
    candidates.append(("19. Unknown Avoider", MPPIGenome(
        H=22, K=512, dt=0.052, lambda_temp=0.40, ar1_alpha=0.75, gamma_discount=0.96, mu_friction=0.66,
        w_cte=10.0, w_heading=5.0, w_speed=1.4, w_hazard=10000.0, w_prob_costmap=3.0, w_unknown=7.0,
        w_steer_rate=0.90, w_accel_rate=0.35, w_terminal=20.0, sigma_steer=0.09, sigma_speed=0.28,
        lognorm_shape=0.28, ratio_center=0.40, ratio_straight=0.20, ratio_left=0.20, pure_pursuit_lookahead=0.38,
        r_inflation_m=0.090, w_reverse=2500.0, reverse_speed=1.4, t_rev_steps=2
    )))

    # 20. Balanced Champion Baseline (User Tuned)
    candidates.append(("20. Baseline Champion", MPPIGenome(
        H=20, K=512, dt=0.055, lambda_temp=0.60, ar1_alpha=0.60, gamma_discount=0.97, mu_friction=0.60,
        w_cte=10.0, w_heading=12.0, w_speed=1.0, w_hazard=10000.0, w_prob_costmap=2.0, w_unknown=3.0,
        w_steer_rate=1.00, w_accel_rate=0.50, w_terminal=20.0, sigma_steer=0.10, sigma_speed=0.30,
        lognorm_shape=0.30, ratio_center=0.40, ratio_straight=0.20, ratio_left=0.20, pure_pursuit_lookahead=0.5239,
        r_inflation_m=0.0923, w_reverse=2500.0, reverse_speed=1.5, t_rev_steps=2
    )))

    return candidates


def breed_generation_1(gen0_results: List[Tuple[str, MPPIGenome, float, Dict[str, Any]]]) -> List[Tuple[str, MPPIGenome]]:
    """Selects top performers from Gen 0 and breeds 20 refined Gen 1 candidates."""
    sorted_res = sorted(gen0_results, key=lambda x: x[2], reverse=True)
    top_5 = sorted_res[:5]

    gen1_candidates = []

    # Elite retention (top 5 kept directly)
    for i, (name, genome, fit, met) in enumerate(top_5):
        clean_name = name.split("(")[0].strip()
        gen1_candidates.append((f"Elite #{i+1}: {clean_name}", genome))

    # Crossover and mutations among top performers
    specs = MPPIGenome.GENE_SPECS
    for i in range(15):
        p1 = top_5[np.random.randint(0, len(top_5))][1].to_array()
        p2 = top_5[np.random.randint(0, len(top_5))][1].to_array()

        mask = np.random.rand(len(p1)) < 0.5
        child_arr = np.where(mask, p1, p2)

        for g_idx in range(len(child_arr)):
            if np.random.rand() < 0.35:
                low, high = specs[g_idx][1], specs[g_idx][2]
                sigma = (high - low) * 0.08
                child_arr[g_idx] += np.random.normal(0, sigma)
                child_arr[g_idx] = max(low, min(high, child_arr[g_idx]))

        child_genome = MPPIGenome.from_array(child_arr)
        gen1_candidates.append((f"Gen1 Hybrid #{i+1}", child_genome))

    return gen1_candidates


def evaluate_candidates_parallel(
    candidates: List[Tuple[str, MPPIGenome]],
    max_workers: int = 4,
    sim_duration_s: float = 6.0
) -> List[Tuple[str, MPPIGenome, float, Dict[str, Any]]]:
    """Evaluates candidates in parallel using robust single-shot instances."""
    results = [None] * len(candidates)

    def eval_job(idx: int, name: str, genome: MPPIGenome):
        worker_id = idx % max_workers
        fit, met = evaluate_genome(genome, worker_id=worker_id, sim_duration_s=sim_duration_s)
        return idx, name, genome, fit, met

    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = {
            executor.submit(eval_job, i, name, genome): i
            for i, (name, genome) in enumerate(candidates)
        }
        for fut in as_completed(futures):
            idx, name, genome, fit, met = fut.result()
            results[idx] = (name, genome, fit, met)
            dist = met.get("distance_m", 0.0)
            spd = met.get("mean_speed_mps", 0.0)
            laps = met.get("laps_completed", 0)
            colls = met.get("collision_count", 0)
            status = met.get("status", "OK")
            print(f" {name:<24} | {fit:>9.1f} | {dist:>9.2f} | {spd:>11.2f} | {laps:>5d} | {colls:>10d} | {status:<10}", flush=True)

    return [r for r in results if r is not None]


def main():
    print("================================================================================", flush=True)
    print("      EVALUATING 20 PROMISING CANDIDATES ACROSS 2 GENERATIONS IN WEBOTS        ", flush=True)
    print("================================================================================\n", flush=True)

    max_workers = 4
    sim_duration_s = 6.0  # 6.0 sim seconds per trial

    all_gen_results = {}

    # ==========================================================================
    # GENERATION 0
    # ==========================================================================
    print("\n" + "=" * 80, flush=True)
    print("                        STARTING GENERATION 0 (20 CANDIDATES)                   ", flush=True)
    print("=" * 80, flush=True)
    print(f" {'ID':<24} | {'Fitness':>9} | {'Dist (m)':>9} | {'Speed (m/s)':>11} | {'Laps':>5} | {'Collisions':>10} | {'Status':<10}", flush=True)
    print("-" * 90, flush=True)

    gen0_candidates = get_20_promising_candidates()
    gen0_evals = evaluate_candidates_parallel(gen0_candidates, max_workers=max_workers, sim_duration_s=sim_duration_s)
    all_gen_results["gen0"] = gen0_evals

    # ==========================================================================
    # GENERATION 1
    # ==========================================================================
    print("\n" + "=" * 80, flush=True)
    print("                        STARTING GENERATION 1 (20 CANDIDATES)                   ", flush=True)
    print("=" * 80, flush=True)
    print(f" {'ID':<24} | {'Fitness':>9} | {'Dist (m)':>9} | {'Speed (m/s)':>11} | {'Laps':>5} | {'Collisions':>10} | {'Status':<10}", flush=True)
    print("-" * 90, flush=True)

    gen1_candidates = breed_generation_1(gen0_evals)
    gen1_evals = evaluate_candidates_parallel(gen1_candidates, max_workers=max_workers, sim_duration_s=sim_duration_s)
    all_gen_results["gen1"] = gen1_evals

    # Save summary report
    summary_path = os.path.join(WORKSPACE_ROOT, "20_candidates_2_gens_results.json")
    json_export = {}
    for gen_key, evals in all_gen_results.items():
        json_export[gen_key] = [
            {
                "name": name,
                "fitness": round(fit, 2),
                "metrics": met,
                "genes": genome.to_dict()
            }
            for name, genome, fit, met in evals
        ]
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(json_export, f, indent=2)

    print(f"\nResults saved to: {summary_path}")


if __name__ == "__main__":
    main()
