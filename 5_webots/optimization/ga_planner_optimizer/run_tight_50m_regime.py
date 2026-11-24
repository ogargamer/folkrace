"""
================================================================================
  TARGETED 2-GENERATION TIGHT EXPLORATION (SIGMA = 0.05)
  - Gen 1: Explicitly includes the 32 historical >50m candidate genomes + 10 tight mutations (Pop=42)
  - Gen 2: Follow-up CMA-ES generation with tight sigma = 0.05 around top performers
================================================================================
"""

import os
import sys
import time
import json
import csv
import numpy as np

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
if CURR_DIR not in sys.path:
    sys.path.insert(0, CURR_DIR)

from ga_planner_config import (
    NUM_CPU_CORES,
    POPULATION_SIZE,
    EVALUATION_DURATION_S,
    ACTIVE_PLANNER,
    Option2SLMPCGenome,
    load_best_planner_genome,
)
from ga_evaluator import ParallelWorkerPool
from ga_optimizer import CMAESOptimizerAdapter


def build_gen1_population(target_pop_size: int = 42) -> list:
    """Builds Gen 1 population with the 32 historical candidate genomes + tight mutations."""
    champ_genome, champ_fit, champ_metrics = load_best_planner_genome()
    lower, upper = champ_genome.get_bounds()
    span = upper - lower
    span[span <= 1e-9] = 1.0

    # Load checkpoint population
    ckpt_path = os.path.join(CURR_DIR, "ga_checkpoint.json")
    pop_dicts = []
    if os.path.exists(ckpt_path):
        with open(ckpt_path, "r", encoding="utf-8") as f:
            ckpt = json.load(f)
            pop_dicts = ckpt.get("population", [])

    pop = []
    # Candidate 0: Champion
    pop.append(champ_genome)

    # Candidates 1..31: Distinct individuals from the checkpoint population
    for ind_dict in pop_dicts:
        if len(pop) >= 32:
            break
        # Avoid duplicate champion
        g = Option2SLMPCGenome(ind_dict)
        if len(pop) == 1 or np.max(np.abs(g.to_array() - champ_genome.to_array())) > 1e-4:
            pop.append(g)

    # If we need more up to 32, create tight micro-interpolations around champion
    arr_champ = champ_genome.to_array()
    np.random.seed(42)
    while len(pop) < 32:
        noise = np.random.normal(0.0, 0.02, size=span.shape) * span
        cand_arr = np.clip(arr_champ + noise, lower, upper)
        g = Option2SLMPCGenome()
        g.from_array(cand_arr)
        pop.append(g)

    # Pad remaining 10 candidates with tight sigma=0.05 mutations to reach pop_size=42
    while len(pop) < target_pop_size:
        noise = np.random.normal(0.0, 0.05, size=span.shape) * span
        cand_arr = np.clip(arr_champ + noise, lower, upper)
        g = Option2SLMPCGenome()
        g.from_array(cand_arr)
        pop.append(g)

    return pop[:target_pop_size]


def run_tight_exploration(num_generations: int = 2, sigma: float = 0.05, pop_size: int = 42):
    print("=" * 80)
    print("  TARGETED TIGHT 2-GENERATION GA EXPLORATION (SIGMA = 0.05)")
    print(f"  - Gen 1: Explicitly contains all 32 >50m historical genomes + tight variations")
    print(f"  - Gen 2: Follow-up CMA-ES tight exploration (sigma = {sigma:.3f})")
    print(f"  Workers: {NUM_CPU_CORES} Webots Instances | Eval Duration: {EVALUATION_DURATION_S}s")
    print("=" * 80, flush=True)

    champ_genome, champ_fit, champ_metrics = load_best_planner_genome()
    print(f"[Champion] Seed champion: Fitness = {champ_fit:.2f}, Distance = {champ_metrics.get('distance_m', 0.0):.2f}m\n", flush=True)

    gen1_pop = build_gen1_population(target_pop_size=pop_size)
    print(f"[Population] Gen 1 loaded with {len(gen1_pop)} candidates (including all 32 >50m candidate genomes).\n", flush=True)

    adapter = CMAESOptimizerAdapter(
        genome_cls=Option2SLMPCGenome,
        pop_size=pop_size,
        sigma0=sigma,
        initial_genome=champ_genome,
    )
    # Align adapter to Gen 1
    adapter.set_current_population(gen1_pop)

    pool = ParallelWorkerPool(num_workers=NUM_CPU_CORES)
    t_start = time.time()

    all_eval_results = []
    curr_best_fit = champ_fit
    curr_best_genome = champ_genome
    curr_best_metrics = champ_metrics

    try:
        # ----------------------------------------------------------------------
        # GENERATION 1: Evaluate the 32 >50m candidates + variations
        # ----------------------------------------------------------------------
        print(f"================================================================================")
        print(f"  GENERATION 1 / {num_generations}  [Evaluating 32 >50m Candidates + Variations]")
        print(f"================================================================================", flush=True)
        print(f"--- Evaluating {len(gen1_pop)} Candidates on {NUM_CPU_CORES} Parallel Workers ---", flush=True)

        t_gen_start = time.time()
        results_gen1 = pool.evaluate_population(
            planner_name=ACTIVE_PLANNER,
            population_genomes=gen1_pop,
            duration_s=EVALUATION_DURATION_S,
        )
        t_gen_elapsed = time.time() - t_gen_start

        fitnesses_gen1 = [r[0] for r in results_gen1]
        metrics_gen1 = [r[1] for r in results_gen1]

        best_idx1 = int(np.argmax(fitnesses_gen1))
        gen1_best_fit = float(fitnesses_gen1[best_idx1])
        gen1_mean_fit = float(np.mean(fitnesses_gen1))
        gen1_best_metrics = metrics_gen1[best_idx1]
        gen1_crashes = sum(1 for m in metrics_gen1 if m.get("crashed", False))

        print(
            f"[Gen 01 Results] Best Fit: {gen1_best_fit:7.1f} | Mean Fit: {gen1_mean_fit:7.1f} | "
            f"Dist: {gen1_best_metrics.get('distance_m', 0.0):5.2f}m | "
            f"Speed: {gen1_best_metrics.get('mean_speed_mps', 0.0):4.2f}m/s | "
            f"Laps: {gen1_best_metrics.get('laps_completed', 0)} | "
            f"Crashes: {gen1_crashes:2d}/{pop_size} | Time: {t_gen_elapsed:4.1f}s",
            flush=True,
        )

        for idx, (fit, met) in enumerate(results_gen1):
            all_eval_results.append({
                "generation": 1,
                "candidate_idx": idx,
                "fitness": fit,
                "metrics": met,
                "genome": gen1_pop[idx],
            })

        if gen1_best_fit > curr_best_fit:
            curr_best_fit = gen1_best_fit
            curr_best_genome = gen1_pop[best_idx1]
            curr_best_metrics = gen1_best_metrics
            champ_path = os.path.join(CURR_DIR, "best_planner_genome.json")
            print(f"  *** NEW ALL-TIME CHAMPION FOUND! Fitness: {curr_best_fit:.2f}, Dist: {curr_best_metrics.get('distance_m', 0.0):.2f}m ***", flush=True)
            with open(champ_path, "w", encoding="utf-8") as f:
                json.dump({
                    "planner": ACTIVE_PLANNER,
                    "fitness": round(curr_best_fit, 2),
                    "metrics": curr_best_metrics,
                    "genes": curr_best_genome.to_dict(),
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                }, f, indent=2)

        # Update CMA-ES with Gen 1 results
        adapter.tell(fitnesses_gen1)

        # ----------------------------------------------------------------------
        # GENERATION 2: Tight CMA-ES exploration around best performers (Sigma = 0.05)
        # ----------------------------------------------------------------------
        print(f"\n================================================================================")
        print(f"  GENERATION 2 / {num_generations}  [Tight CMA-ES Exploration (Sigma = {sigma:.3f})]")
        print(f"================================================================================", flush=True)

        gen2_pop = adapter.ask(inject_baseline=True)
        print(f"--- Evaluating {len(gen2_pop)} Candidates on {NUM_CPU_CORES} Parallel Workers ---", flush=True)

        t_gen_start = time.time()
        results_gen2 = pool.evaluate_population(
            planner_name=ACTIVE_PLANNER,
            population_genomes=gen2_pop,
            duration_s=EVALUATION_DURATION_S,
        )
        t_gen_elapsed = time.time() - t_gen_start

        fitnesses_gen2 = [r[0] for r in results_gen2]
        metrics_gen2 = [r[1] for r in results_gen2]

        best_idx2 = int(np.argmax(fitnesses_gen2))
        gen2_best_fit = float(fitnesses_gen2[best_idx2])
        gen2_mean_fit = float(np.mean(fitnesses_gen2))
        gen2_best_metrics = metrics_gen2[best_idx2]
        gen2_crashes = sum(1 for m in metrics_gen2 if m.get("crashed", False))

        print(
            f"[Gen 02 Results] Best Fit: {gen2_best_fit:7.1f} | Mean Fit: {gen2_mean_fit:7.1f} | "
            f"Dist: {gen2_best_metrics.get('distance_m', 0.0):5.2f}m | "
            f"Speed: {gen2_best_metrics.get('mean_speed_mps', 0.0):4.2f}m/s | "
            f"Laps: {gen2_best_metrics.get('laps_completed', 0)} | "
            f"Crashes: {gen2_crashes:2d}/{pop_size} | Time: {t_gen_elapsed:4.1f}s",
            flush=True,
        )

        for idx, (fit, met) in enumerate(results_gen2):
            all_eval_results.append({
                "generation": 2,
                "candidate_idx": idx,
                "fitness": fit,
                "metrics": met,
                "genome": gen2_pop[idx],
            })

        if gen2_best_fit > curr_best_fit:
            curr_best_fit = gen2_best_fit
            curr_best_genome = gen2_pop[best_idx2]
            curr_best_metrics = gen2_best_metrics
            champ_path = os.path.join(CURR_DIR, "best_planner_genome.json")
            print(f"  *** NEW ALL-TIME CHAMPION FOUND! Fitness: {curr_best_fit:.2f}, Dist: {curr_best_metrics.get('distance_m', 0.0):.2f}m ***", flush=True)
            with open(champ_path, "w", encoding="utf-8") as f:
                json.dump({
                    "planner": ACTIVE_PLANNER,
                    "fitness": round(curr_best_fit, 2),
                    "metrics": curr_best_metrics,
                    "genes": curr_best_genome.to_dict(),
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                }, f, indent=2)

        adapter.tell(fitnesses_gen2)

    finally:
        pool.shutdown()

    total_elapsed = time.time() - t_start
    print("\n" + "=" * 80)
    print(f"  TIGHT EXPLORATION RUN COMPLETE ({total_elapsed:.1f}s total)")
    print("=" * 80, flush=True)

    # Summary of Top Candidates
    all_eval_results.sort(key=lambda x: x["fitness"], reverse=True)
    print("\n=== TOP 10 CANDIDATES FROM TIGHT EXPLORATION RUN ===")
    print(f"{'Rank':<5} {'Gen':<5} {'Cand#':<7} {'Fitness':<12} {'Dist (m)':<10} {'Speed (m/s)':<12} {'Laps':<6} {'Crashed'}")
    print("-" * 80)
    for rank, r in enumerate(all_eval_results[:10], 1):
        m = r["metrics"]
        print(f"{rank:<5} {r['generation']:<5} #{r['candidate_idx']:<6} {r['fitness']:<12.2f} {m.get('distance_m', 0.0):<10.2f} {m.get('mean_speed_mps', 0.0):<12.2f} {m.get('laps_completed', 0):<6} {m.get('crashed', False)}")

    # Save detailed CSV
    data_dir = os.path.join(CURR_DIR, "data")
    os.makedirs(data_dir, exist_ok=True)
    out_csv = os.path.join(data_dir, "tight_005_exploration_results.csv")
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["Rank", "Generation", "Candidate_Idx", "Fitness", "Distance_m", "Mean_Speed_mps", "Laps", "Crashed", "Crash_Reason"])
        for rank, r in enumerate(all_eval_results, 1):
            m = r["metrics"]
            writer.writerow([
                rank,
                r["generation"],
                r["candidate_idx"],
                round(r["fitness"], 2),
                round(m.get("distance_m", 0.0), 3),
                round(m.get("mean_speed_mps", 0.0), 3),
                m.get("laps_completed", 0),
                m.get("crashed", False),
                m.get("crash_reason", ""),
            ])
    print(f"\n[Artifact] Full results saved to: {out_csv}")


if __name__ == "__main__":
    run_tight_exploration(num_generations=2, sigma=0.05, pop_size=42)
