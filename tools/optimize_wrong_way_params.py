#!/usr/bin/env python3
"""
================================================================================
  PARALLEL HEADLESS WEBOTS WRONG-WAY & RECOVERY PARAMETER OPTIMIZER
================================================================================
  Uses an elitist (mu + lambda) evolutionary strategy across 4 parallel
  headless Webots workers to optimize AntiWrongWayWatchdog & StuckRecoveryManager
  parameters across all 12 scenarios in test_wrong_way.wbt and track_racing.wbt.
================================================================================
"""

import argparse
import concurrent.futures
import json
import os
import random
import sys
import time
from typing import Any, Dict, List, Tuple

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)

from run_webots_wrong_way_tests import run_world_eval

BOUNDS: Dict[str, Tuple[float, float, bool]] = {
    "aww_duration_scale": (0.25, 0.50, False),
    "aww_max_detect_dist_m": (1.80, 2.35, False),
    "aww_max_lat_xv_m": (0.42, 0.65, False),
    "aww_min_confirmed": (3, 4, True),
    "aww_confirm_time_s": (0.032, 0.085, False),
    "aww_post_cooldown_s": (0.20, 0.65, False),
    "rec_180_speed_mps": (0.60, 1.05, False),
    "rec_180_rev_clear_m": (0.32, 0.55, False),
    "rec_180_rev_min_s": (0.12, 0.40, False),
    "rec_180_align_deg": (24.0, 38.0, False),
}

BASELINE: Dict[str, Any] = {
    "aww_duration_scale": 0.35,
    "aww_max_detect_dist_m": 2.20,
    "aww_max_lat_xv_m": 0.55,
    "aww_min_confirmed": 3,
    "aww_confirm_time_s": 0.048,
    "aww_post_cooldown_s": 0.35,
    "rec_180_speed_mps": 0.85,
    "rec_180_rev_clear_m": 0.38,
    "rec_180_rev_min_s": 0.18,
    "rec_180_align_deg": 30.0,
}


def clamp_params(p: Dict[str, Any]) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    for k, (lo, hi, is_int) in BOUNDS.items():
        val = max(lo, min(hi, float(p[k])))
        out[k] = int(round(val)) if is_int else round(val, 4)
    return out


def mutate_params(parent: Dict[str, Any], sigma: float, rng: random.Random) -> Dict[str, Any]:
    child: Dict[str, Any] = {}
    for k, (lo, hi, is_int) in BOUNDS.items():
        span = hi - lo
        if is_int:
            if rng.random() < 0.35:
                child[k] = parent[k] + rng.choice([-1, 1])
            else:
                child[k] = parent[k]
        else:
            child[k] = float(parent[k]) + rng.gauss(0.0, sigma * span)
    return clamp_params(child)


def score_results(res_ww: dict, res_tr: dict) -> Tuple[float, int, int, List[str]]:
    all_r = list(res_ww.get("results", [])) + list(res_tr.get("results", []))
    total = len(all_r)
    passed_cnt = sum(1 for r in all_r if r.get("passed"))
    failed_names = [r.get("name", "?") for r in all_r if not r.get("passed")]

    fitness = 1000.0
    det_times: List[float] = []
    rec_times: List[float] = []
    post_dists: List[float] = []
    clean_prog = 0.0

    for r in all_r:
        if r.get("flipped"):
            fitness -= 5000.0
        if r.get("off_track") and r.get("name") != "T2_clean_rev_opponents_25s":
            fitness -= 5000.0
        if "ww_triggers" in r:
            # Clean scenario
            ww = int(r.get("ww_fp", r.get("ww_triggers", 0)))
            fitness -= 10000.0 * ww
            clean_prog += float(r.get("progress_m", 0.0))
            if not r.get("passed"):
                fitness -= 2000.0
        else:
            # Spin scenario
            fitness -= 10000.0 * int(r.get("pre_ww", 0))
            fitness -= 1000.0 * int(r.get("retriggers", 0))
            if not r.get("passed"):
                fitness -= 2000.0
            if r.get("detect_s") is not None:
                det_times.append(float(r["detect_s"]))
            else:
                det_times.append(2.0)
            if r.get("rec_done_s") is not None:
                rec_times.append(float(r["rec_done_s"]))
            else:
                rec_times.append(4.5)
            post_dists.append(float(r.get("post_rec_dist_m", 0.0)))

    if det_times:
        fitness -= 250.0 * (sum(det_times) / len(det_times))
    if rec_times:
        fitness -= 150.0 * (sum(rec_times) / len(rec_times))
        fitness -= 80.0 * max(rec_times)
    if post_dists:
        fitness += 20.0 * (sum(post_dists) / len(post_dists))
    fitness += 3.0 * clean_prog

    return round(fitness, 2), passed_cnt, total, failed_names


def evaluate_candidate(cand_id: int, params: Dict[str, Any], worker_slot: int) -> Tuple[float, int, int, List[str], Dict[str, Any]]:
    p_json = json.dumps(params)
    port_base = 10400 + worker_slot * 10
    res_ww = run_world_eval("test_wrong_way", "all", False, 180.0, params_json=p_json, port=port_base)
    res_tr = run_world_eval("track_racing", "all", False, 420.0, params_json=p_json, port=port_base + 1)
    fit, p_cnt, tot, failed = score_results(res_ww, res_tr)
    return fit, p_cnt, tot, failed, params


def main() -> int:
    parser = argparse.ArgumentParser(description="Optimize wrong-way parameters in parallel headless Webots.")
    parser.add_argument("--gens", type=int, default=4, help="Number of generations")
    parser.add_argument("--pop", type=int, default=6, help="Candidates per generation")
    parser.add_argument("--workers", type=int, default=2, help="Parallel Webots workers")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    rng = random.Random(args.seed)
    t_start = time.perf_counter()

    population: List[Dict[str, Any]] = [dict(BASELINE)]
    while len(population) < args.pop:
        population.append(mutate_params(BASELINE, sigma=0.22, rng=rng))

    best_ever = None  # (fit, p_cnt, tot, failed, params)

    print("=" * 80)
    print(f"  PARALLEL WEBOTS WRONG-WAY OPTIMIZER (gens={args.gens}, pop={args.pop}, workers={args.workers})")
    print("=" * 80)

    for gen in range(args.gens):
        t_gen = time.perf_counter()
        scored: List[Tuple[float, int, int, List[str], Dict[str, Any]]] = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
            futs = [
                pool.submit(evaluate_candidate, idx, cand, idx % args.workers)
                for idx, cand in enumerate(population)
            ]
            for fut in concurrent.futures.as_completed(futs):
                scored.append(fut.result())

        scored.sort(key=lambda x: x[0], reverse=True)
        top = scored[0]
        if best_ever is None or top[0] > best_ever[0]:
            best_ever = top

        print(
            f"[Gen {gen+1}/{args.gens}] Best={top[0]:.2f} ({top[1]}/{top[2]} PASS, fail={top[3]}) "
            f"| EverBest={best_ever[0]:.2f} ({best_ever[1]}/{best_ever[2]} PASS) "
            f"| {time.perf_counter()-t_gen:.1f}s",
            flush=True,
        )

        # Elitist selection (top 2) + mutated offspring
        elites = [scored[0][4], scored[min(1, len(scored) - 1)][4]]
        sigma = max(0.08, 0.22 * (0.75 ** (gen + 1)))
        next_pop = [dict(elites[0])]
        while len(next_pop) < args.pop:
            parent = rng.choice(elites)
            next_pop.append(mutate_params(parent, sigma=sigma, rng=rng))
        population = next_pop

    print("\n" + "=" * 80)
    print(f"  OPTIMIZATION COMPLETE in {time.perf_counter()-t_start:.1f}s")
    print(f"  Best Fitness: {best_ever[0]:.2f} ({best_ever[1]}/{best_ever[2]} PASS, failed={best_ever[3]})")
    print("  Best Params JSON:")
    print(json.dumps(best_ever[4], indent=2))
    print("=" * 80)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
