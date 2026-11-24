"""
================================================================================
  TARGETED SINGLE-GENERATION 84-POPULATION GA EVALUATOR AROUND 10.2k+ REGIME
================================================================================
  1. Loads known 10k+ champions (10.874k all-time champion, 10.259k CMA xbest).
  2. Constructs a population of 84 individuals tightly clustered around this regime:
     - Exact 10.874k champion and 10.259k CMA best
     - Convex interpolations between 10.8k and 10.2k
     - Micro-perturbations (1% - 3% span) around both peaks
  3. Evaluates all 84 candidates across 14 headless Webots workers in parallel.
  4. Records metrics, reports any candidates beating 10.2k and the champion, and
     updates the champion file if a new record is set.
================================================================================
"""

import os
import sys
import time
import json
import csv
import pickle
import numpy as np

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
if CURR_DIR not in sys.path:
    sys.path.insert(0, CURR_DIR)

from ga_planner_config import (
    NUM_CPU_CORES,
    EVALUATION_DURATION_S,
    ACTIVE_PLANNER,
    Option2SLMPCGenome,
    OPTION_2_SL_MPC_GENE_SPECS,
)
from ga_evaluator import ParallelWorkerPool


def build_tight_population_84() -> list:
    """Builds 84 individuals clustered tightly around the >= 10.2k parameters."""
    # 1. Load 10.874k champion
    champ_path = os.path.join(CURR_DIR, "best_planner_genome.json")
    with open(champ_path, "r", encoding="utf-8") as f:
        champ_json = json.load(f)
    champ_genes = champ_json["genes"]

    # 2. Load 10.259k CMA-ES best
    cma_path = os.path.join(CURR_DIR, "cma_state.pkl")
    with open(cma_path, "rb") as f:
        cma_data = pickle.load(f)
    es = cma_data["es"]

    g_dummy = Option2SLMPCGenome()
    lower, upper = g_dummy.get_bounds()
    span = upper - lower

    u_best = es.result.xbest
    x_cma = lower + u_best * span
    g_cma = Option2SLMPCGenome()
    g_cma.from_array(x_cma)
    cma_genes = g_cma.to_dict()

    pop = []

    # Candidate 0: Exact 10.874k Champion
    pop.append(Option2SLMPCGenome(champ_genes))
    # Candidate 1: Exact 10.259k CMA Best
    pop.append(Option2SLMPCGenome(cma_genes))

    arr_champ = Option2SLMPCGenome(champ_genes).to_array()
    arr_cma = Option2SLMPCGenome(cma_genes).to_array()

    # Candidates 2-15: Direct linear interpolations between 10.8k and 10.2k
    for alpha in np.linspace(0.05, 0.95, 14):
        interp = (1.0 - alpha) * arr_champ + alpha * arr_cma
        g = Option2SLMPCGenome()
        g.from_array(interp)
        pop.append(g)

    # Candidates 16-49: Ultra-tight Gaussian mutations around 10.8k Champion (sigma 0.5% - 2.5% span)
    np.random.seed(42)
    for sig in [0.005, 0.010, 0.015, 0.020, 0.025]:
        for _ in range(7):
            noise = np.random.normal(0.0, sig, size=span.shape) * span
            cand = np.clip(arr_champ + noise, lower, upper)
            g = Option2SLMPCGenome()
            g.from_array(cand)
            pop.append(g)

    # Candidates 50-83: Ultra-tight Gaussian mutations around 10.2k CMA Best (sigma 0.5% - 2.5% span)
    for sig in [0.005, 0.010, 0.015, 0.020, 0.025]:
        for _ in range(7):
            noise = np.random.normal(0.0, sig, size=span.shape) * span
            cand = np.clip(arr_cma + noise, lower, upper)
            g = Option2SLMPCGenome()
            g.from_array(cand)
            pop.append(g)

    # Truncate or pad to exactly 84
    while len(pop) < 84:
        noise = np.random.normal(0.0, 0.01, size=span.shape) * span
        cand = np.clip(arr_champ + noise, lower, upper)
        g = Option2SLMPCGenome()
        g.from_array(cand)
        pop.append(g)

    return pop[:84]


def main():
    print("=" * 80)
    print("  STARTING SINGLE-GENERATION 84-POPULATION GA EVALUATION")
    print(f"  Target: Clustered tightly around >= 10.2k parameter space")
    print(f"  Workers: {NUM_CPU_CORES} CPU Cores | Eval Duration: {EVALUATION_DURATION_S}s per candidate")
    print("=" * 80, flush=True)

    pop = build_tight_population_84()
    print(f"[Population] Constructed {len(pop)} candidates clustered around 10k+ peaks.\n", flush=True)

    pool = ParallelWorkerPool(num_workers=NUM_CPU_CORES)
    t0 = time.time()

    try:
        results = pool.evaluate_population(
            planner_name=ACTIVE_PLANNER,
            population_genomes=pop,
            duration_s=EVALUATION_DURATION_S,
        )
    finally:
        pool.shutdown()

    elapsed = time.time() - t0
    fitnesses = [r[0] for r in results]
    metrics_list = [r[1] for r in results]

    print("\n" + "=" * 80)
    print(f"  EVALUATION COMPLETE ({elapsed:.1f}s elapsed for 84 candidates)")
    print("=" * 80, flush=True)

    # Parse and sort results
    indexed_results = []
    for idx, (fit, met) in enumerate(results):
        indexed_results.append({
            "idx": idx,
            "fitness": fit,
            "metrics": met,
            "genome": pop[idx],
        })

    indexed_results.sort(key=lambda x: x["fitness"], reverse=True)

    # Count > 10k and > 10.2k
    count_10k = sum(1 for r in indexed_results if r["fitness"] >= 10000.0)
    count_10_2k = sum(1 for r in indexed_results if r["fitness"] >= 10200.0)

    print(f"\n[Summary Statistics]")
    print(f"  Total Evaluated:        {len(indexed_results)}")
    print(f"  Candidates >= 10,000:   {count_10k} / {len(indexed_results)} ({count_10k/len(indexed_results)*100:.1f}%)")
    print(f"  Candidates >= 10,200:   {count_10_2k} / {len(indexed_results)} ({count_10_2k/len(indexed_results)*100:.1f}%)")
    print(f"  Max Fitness Achieved:   {indexed_results[0]['fitness']:.2f}")
    print(f"  Mean Fitness:           {np.mean(fitnesses):.2f}")

    # Display all candidates with fitness >= 10,200
    print("\n" + "=" * 80)
    print("  ALL CANDIDATES WITH FITNESS >= 10,200:")
    print("=" * 80)
    print(f"{'Rank':<5} {'Cand#':<7} {'Fitness':<12} {'Dist (m)':<10} {'Speed (m/s)':<12} {'Laps':<6} {'Crashed'}")
    print("-" * 80)

    high_candidates = [r for r in indexed_results if r["fitness"] >= 10200.0]
    for rank, r in enumerate(high_candidates, 1):
        m = r["metrics"]
        print(f"{rank:<5} #{r['idx']:<6} {r['fitness']:<12.2f} {m.get('distance_m', 0.0):<10.2f} {m.get('mean_speed_mps', 0.0):<12.2f} {m.get('laps_completed', 0):<6} {m.get('crashed', False)}")

    # Check if we beat the all-time champion
    best_cand = indexed_results[0]
    champ_path = os.path.join(CURR_DIR, "best_planner_genome.json")
    with open(champ_path, "r", encoding="utf-8") as f:
        curr_champ = json.load(f)

    curr_best_fit = curr_champ.get("fitness", 0.0)
    print("\n" + "-" * 80)
    print(f"Current Best Champion Fitness: {curr_best_fit:.2f}")
    print(f"Generation Top Candidate Fitness: {best_cand['fitness']:.2f}")

    if best_cand["fitness"] > curr_best_fit:
        print(f"  *** NEW ALL-TIME CHAMPION FOUND! Updating {champ_path} ***")
        with open(champ_path, "w", encoding="utf-8") as f:
            json.dump({
                "planner": ACTIVE_PLANNER,
                "fitness": round(best_cand["fitness"], 2),
                "metrics": best_cand["metrics"],
                "genes": best_cand["genome"].to_dict(),
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            }, f, indent=2)
    else:
        print("  Existing champion remains unbeaten.")

    # Save detailed evaluation log to CSV
    data_dir = os.path.join(CURR_DIR, "data")
    os.makedirs(data_dir, exist_ok=True)
    csv_out = os.path.join(data_dir, "ga_tight_84_run_results.csv")
    with open(csv_out, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["Rank", "Candidate_Idx", "Fitness", "Distance_m", "Mean_Speed_mps", "Laps", "Crashed", "Crash_Reason"])
        for rank, r in enumerate(indexed_results, 1):
            m = r["metrics"]
            writer.writerow([
                rank,
                r["idx"],
                round(r["fitness"], 2),
                round(m.get("distance_m", 0.0), 3),
                round(m.get("mean_speed_mps", 0.0), 3),
                m.get("laps_completed", 0),
                m.get("crashed", False),
                m.get("crash_reason", ""),
            ])
    print(f"\n[Artifact] Full results saved to: {csv_out}")


if __name__ == "__main__":
    main()
