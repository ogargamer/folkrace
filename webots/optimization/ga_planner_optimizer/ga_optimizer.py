"""
================================================================================
  HIGH-THROUGHPUT PARALLEL GENETIC ALGORITHM (GA) OPTIMIZER
================================================================================
  Optimizes autonomous ToF path planner parameters in headless Webots.
  Features:
    - Multi-core parallel simulation across CPU cores (20s duration)
    - Moving obstacles traveling against robot heading at 0.50 m/s
    - Tournament selection, arithmetic crossover, Gaussian mutation, elitism
    - Real-time checkpointing (ga_checkpoint.json, ga_history.csv, best_genome.json)
================================================================================
"""

import os
import sys
import time
import json
import csv
import signal
import pickle
import argparse
from typing import Dict, List, Tuple, Optional, Any
import numpy as np

try:
    import cma
except ImportError:
    cma = None

CURR_DIR = os.path.dirname(os.path.abspath(__file__))

def _find_optimizer_dir(start_dir: str) -> str:
    if os.path.exists(os.path.join(start_dir, "worlds", "Cyan_Champion_GA.wbt")):
        return start_dir
    p = start_dir
    for _ in range(6):
        cand = os.path.join(p, "webots", "optimization", "ga_planner_optimizer")
        if os.path.exists(os.path.join(cand, "worlds", "Cyan_Champion_GA.wbt")):
            return cand
        cand2 = os.path.join(p, "optimization", "ga_planner_optimizer")
        if os.path.exists(os.path.join(cand2, "worlds", "Cyan_Champion_GA.wbt")):
            return cand2
        parent = os.path.dirname(p)
        if parent == p:
            break
        p = parent
    return start_dir

OPTIMIZER_PROJECT_DIR = CURR_DIR
GA_OPTIMIZER_DIR = OPTIMIZER_PROJECT_DIR
WEBOTS_DIR = os.path.dirname(os.path.dirname(GA_OPTIMIZER_DIR))
MY_PROJECT_DIR = os.path.join(WEBOTS_DIR, "simulation", "my_project")
WORKSPACE_ROOT = os.path.dirname(WEBOTS_DIR)

for p in [
    CURR_DIR,
    OPTIMIZER_PROJECT_DIR,
    os.path.join(MY_PROJECT_DIR, "controllers", "tof_path_planner"),
    os.path.join(OPTIMIZER_PROJECT_DIR, "data"),
    WORKSPACE_ROOT,
]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from ga_planner_config import (
    NUM_CPU_CORES,
    POPULATION_SIZE,
    NUM_GENERATIONS,
    EVALUATION_DURATION_S,
    CROSSOVER_RATE,
    MUTATION_RATE,
    MUTATION_SCALE,
    ELITISM_COUNT,
    TOURNAMENT_SIZE,
    ACTIVE_PLANNER,
    ALGORITHM,
    CMA_SIGMA0,
    CMA_RESTART_ON_CONVERGENCE,
    SEED_FROM_BEST_GENOME_DEFAULT,
    BEST_GENOME_PATH,
    get_genome_class,
    generate_logical_initial_population,
    load_best_planner_genome,
    BasePlannerGenome,
)
from ga_evaluator import ParallelWorkerPool, run_rendered_preview, BASE_WORKER_PORT

CHECKPOINT_FILE = os.path.join(CURR_DIR, "ga_checkpoint.json")
HISTORY_CSV_FILE = os.path.join(CURR_DIR, "ga_history.csv")
BEST_GENOME_FILE = os.path.join(CURR_DIR, "best_planner_genome.json")


class CMAESOptimizerAdapter:
    """
    Adapter bridging Covariance Matrix Adaptation Evolution Strategy (CMA-ES)
    with bounded, mixed-type path planner genome parameter vectors.

    - Maps bounded real/int genes into normalized hypercube [0, 1]^D
    - Samples population of candidates via es.ask()
    - Quantizes, rounds integers, and clamps back to physical gene ranges
    - Translates maximized Webots scalar fitness into minimization loss for es.tell()
    - Supports full serialization (covariance matrix, mean vector, step size) via pickle
    - Handles automatic re-centering / restart if covariance matrix degenerates
    """

    def __init__(
        self,
        genome_cls,
        pop_size: int,
        sigma0: float = CMA_SIGMA0,
        initial_genome: Optional[BasePlannerGenome] = None,
    ):
        if cma is None:
            raise ImportError("The 'cma' package is required for CMA-ES optimization. Install via: pip install cma")

        self.genome_cls = genome_cls
        self.pop_size = pop_size
        self.sigma0 = float(sigma0)

        sample = genome_cls()
        self.lower, self.upper = sample.get_bounds()
        self.span = self.upper - self.lower
        # Guard against zero span
        self.span[self.span <= 1e-9] = 1.0
        self.D = len(sample.specs)

        self.initial_genome = initial_genome

        # Baseline mean: start from provided genome or expert baseline archetype
        if initial_genome is not None:
            x0 = initial_genome.to_array()
        else:
            seeds = generate_logical_initial_population(genome_cls, 1)
            x0 = seeds[0].to_array() if seeds else sample.to_array()

        u0 = (x0 - self.lower) / self.span
        u0 = np.clip(u0, 0.05, 0.95)

        self.opts = {
            "bounds": [[0.0] * self.D, [1.0] * self.D],
            "popsize": self.pop_size,
            "verbose": -9,
        }
        self.es = cma.CMAEvolutionStrategy(u0.tolist(), self.sigma0, self.opts)
        self.last_candidates_u: Optional[List[List[float]]] = None

    def ask(self, inject_baseline: bool = False) -> List[BasePlannerGenome]:
        """Samples next population of candidate genomes from CMA-ES distribution."""
        self.last_candidates_u = self.es.ask()
        if inject_baseline and len(self.last_candidates_u) > 0:
            # Candidate 0 is GUARANTEED to be the active all-time champion (Elitism guarantee)
            if self.initial_genome is not None:
                x0 = self.initial_genome.to_array()
                u0 = np.clip((x0 - self.lower) / self.span, 0.0, 1.0).tolist()
                self.last_candidates_u[0] = u0
            else:
                seeds = generate_logical_initial_population(self.genome_cls, 1)
                if seeds:
                    x0 = seeds[0].to_array()
                    u0 = np.clip((x0 - self.lower) / self.span, 0.0, 1.0).tolist()
                    self.last_candidates_u[0] = u0

        pop: List[BasePlannerGenome] = []
        for u in self.last_candidates_u:
            x = self.lower + np.array(u) * self.span
            g = self.genome_cls()
            g.from_array(x)
            pop.append(g)
        return pop

    def tell(self, fitnesses: List[float]):
        """Feeds scalar fitness scores back to CMA-ES (minimizing negative fitness)."""
        if self.last_candidates_u is None or len(self.last_candidates_u) != len(fitnesses):
            return

        losses = [-float(f) for f in fitnesses]
        try:
            self.es.tell(self.last_candidates_u, losses)
        except RuntimeError as e:
            if "tell can currently only be called once" in str(e):
                print(f"[CMA-ES] Warning: {e}. Re-synchronizing iteration cycle...", flush=True)
                _ = self.es.ask()
                self.es.tell(self.last_candidates_u, losses)
            else:
                raise

        if CMA_RESTART_ON_CONVERGENCE and bool(self.es.stop()):
            print(
                f"[CMA-ES] Stopping criteria triggered: {self.es.stop()}. "
                f"Auto-restarting search distribution around top solution with sigma0={self.sigma0:.2f}...",
                flush=True,
            )
            # Re-center around champion genome if available, otherwise xbest or xmean
            if self.initial_genome is not None:
                x_anchor = self.initial_genome.to_array()
                anchor_u = np.clip((x_anchor - self.lower) / self.span, 0.05, 0.95).tolist()
            elif self.es.result.xbest is not None:
                anchor_u = self.es.result.xbest
            else:
                anchor_u = self.es.mean
            self.es = cma.CMAEvolutionStrategy(anchor_u, self.sigma0, self.opts)
            self.last_candidates_u = None

    def restart_around_champion(self, champion_genome: BasePlannerGenome, sigma_scale: float = 0.2):
        """Forces immediate re-centering of CMA-ES around the known best champion with a tight neighborhood."""
        self.initial_genome = champion_genome
        x0 = champion_genome.to_array()
        u0 = np.clip((x0 - self.lower) / self.span, 0.05, 0.95)
        new_sigma = max(0.02, float(self.sigma0 * sigma_scale))
        print(f"[CMA-ES] Forcing distribution re-seed around champion with tight sigma={new_sigma:.3f}...", flush=True)
        self.es = cma.CMAEvolutionStrategy(u0.tolist(), new_sigma, self.opts)
        self.last_candidates_u = None

    def set_current_population(self, pop: List[BasePlannerGenome]):
        """Projects an existing population into normalized space for resume operations."""
        self.last_candidates_u = [
            np.clip((g.to_array() - self.lower) / self.span, 0.0, 1.0).tolist()
            for g in pop
        ]

    def save_state(self, filepath: str):
        """Pickles the full covariance matrix adaptation state."""
        try:
            with open(filepath, "wb") as f:
                pickle.dump({
                    "es": self.es,
                    "last_candidates_u": self.last_candidates_u,
                }, f)
        except Exception as e:
            print(f"[CMA-ES] Failed to save state to {filepath}: {e}", flush=True)

    def load_state(self, filepath: str) -> bool:
        """Restores covariance matrix adaptation state from pickle."""
        if not os.path.exists(filepath):
            return False
        try:
            with open(filepath, "rb") as f:
                data = pickle.load(f)
                loaded_es = data["es"]
                if hasattr(loaded_es, "N") and loaded_es.N != self.D:
                    print(
                        f"[CMA-ES] Pickled state dimension ({loaded_es.N}) does not match current genome specs ({self.D}). "
                        f"Re-initializing fresh CMA-ES search from champion...",
                        flush=True,
                    )
                    return False
                self.es = loaded_es
                self.last_candidates_u = data.get("last_candidates_u", None)
            return True
        except Exception as e:
            print(f"[CMA-ES] Failed to load state from {filepath}: {e}", flush=True)
            return False


class GAController:
    """Thread-safe controller to pause, resume, stop-and-save, or abort GA execution with callbacks."""

    def __init__(self):
        import threading
        self.pause_event = threading.Event()
        self.pause_event.set()  # Unpaused by default
        self.stop_requested = False
        self.abort_requested = False
        self._pool = None  # Active ParallelWorkerPool
        self._preview_worker = None  # Active rendered preview worker
        self.on_generation_done = None  # callback(gen, planner, metrics, genome, fitnesses, elapsed_s)
        self.on_status_change = None    # callback(status_str)

    def pause(self):
        self.pause_event.clear()
        if self.on_status_change:
            self.on_status_change("PAUSED")

    def resume(self):
        self.pause_event.set()
        if self.on_status_change:
            self.on_status_change("RUNNING")

    def stop_and_save(self):
        """Finishes current generation/evaluations smoothly, saves checkpoints & champions, and stops."""
        self.stop_requested = True
        self.pause_event.set()  # Unblock if paused to allow smooth completion
        if self.on_status_change:
            self.on_status_change("STOPPING (Finishing Run)...")

    def stop(self):
        """Alias for stop_and_save."""
        self.stop_and_save()

    def abort(self):
        """Immediately halts everything, kills all Webots processes, does NOT save, and exits."""
        self.abort_requested = True
        self.stop_requested = True
        self.pause_event.set()  # Unblock if paused
        if self._preview_worker is not None:
            try:
                self._preview_worker.stop_process()
            except Exception as e:
                print(f"[GAController] Error stopping preview worker on abort: {e}", flush=True)
            self._preview_worker = None
        if self._pool is not None:
            try:
                self._pool.shutdown()
            except Exception as e:
                print(f"[GAController] Error shutting down pool on abort: {e}", flush=True)
            self._pool = None
        if self.on_status_change:
            self.on_status_change("ABORTED")

    @property
    def is_paused(self) -> bool:
        return not self.pause_event.is_set()


def initialize_population(genome_cls, pop_size: int, seed_genome: Optional[Any] = None) -> List[BasePlannerGenome]:
    """Generates initial population using physically logical archetypes and stratified sampling."""
    return generate_logical_initial_population(genome_cls, pop_size, seed_genome=seed_genome)


def tournament_select(pop: List[BasePlannerGenome], fitnesses: List[float], k: int = TOURNAMENT_SIZE) -> BasePlannerGenome:
    """Selects best parent from k randomly chosen candidates."""
    idxs = np.random.choice(len(pop), size=k, replace=False)
    best_idx = idxs[0]
    best_fit = fitnesses[best_idx]
    for idx in idxs[1:]:
        if fitnesses[idx] > best_fit:
            best_fit = fitnesses[idx]
            best_idx = idx

    selected = pop[best_idx].__class__()
    selected.from_array(pop[best_idx].to_array())
    return selected


def crossover_arithmetic(p1: BasePlannerGenome, p2: BasePlannerGenome, cr: float = CROSSOVER_RATE) -> Tuple[BasePlannerGenome, BasePlannerGenome]:
    """Arithmetic blend crossover between two parent chromosomes."""
    a1 = p1.to_array()
    a2 = p2.to_array()

    if np.random.random() < cr:
        alpha = np.random.uniform(0.1, 0.9, size=a1.shape)
        c1_arr = alpha * a1 + (1.0 - alpha) * a2
        c2_arr = (1.0 - alpha) * a1 + alpha * a2
    else:
        c1_arr = np.copy(a1)
        c2_arr = np.copy(a2)

    c1 = p1.__class__()
    c1.from_array(c1_arr)
    c2 = p2.__class__()
    c2.from_array(c2_arr)
    return c1, c2


def mutate_gaussian(genome: BasePlannerGenome, mr: float = MUTATION_RATE, scale: float = MUTATION_SCALE) -> BasePlannerGenome:
    """Applies Gaussian mutation within gene boundary spans."""
    arr = genome.to_array()
    lower, upper = genome.get_bounds()
    span = upper - lower

    for i in range(len(arr)):
        if np.random.random() < mr:
            delta = np.random.normal(0.0, scale) * span[i]
            arr[i] = np.clip(arr[i] + delta, lower[i], upper[i])

    genome.from_array(arr)
    return genome


def run_genetic_algorithm(
    planner_name: str = ACTIVE_PLANNER,
    num_cores: int = NUM_CPU_CORES,
    pop_size: int = POPULATION_SIZE,
    num_generations: int = NUM_GENERATIONS,
    eval_duration_s: float = EVALUATION_DURATION_S,
    resume: bool = False,
    preview_rendered: bool = False,
    controller: Optional[GAController] = None,
    algorithm: str = ALGORITHM,
    use_best_genome: bool = SEED_FROM_BEST_GENOME_DEFAULT,
    seed_genome_file: Optional[str] = None,
):
    """
    Executes the optimization pipeline (Single Planner or All Planners Interleaved).
    Supports either CMA-ES (Covariance Matrix Adaptation Evolution Strategy) or Standard GA.
    """
    planners_to_run = ["OPTION_2_SL_MPC"]
    is_cma = str(algorithm).upper().startswith("CMA")
    is_multi = False

    # --------------------------------------------------------------------------
    # Optional 20-Second Rendered Visual Preview (Real-Time Webots Window)
    # --------------------------------------------------------------------------
    if preview_rendered:
        print("=" * 80, flush=True)
        print("  LAUNCHING 20.0s RENDERED VISUAL PREVIEW RUNS (Real-Time Webots Window)", flush=True)
        print("=" * 80, flush=True)
        for p in planners_to_run:
            if controller and (controller.abort_requested or controller.stop_requested):
                print("[GA Optimizer] Abort/Stop requested during preview. Exiting.", flush=True)
                return
            if controller and controller.on_status_change:
                controller.on_status_change(f"PREVIEW: {p} (20s Rendered)")

            # Use best available champion genome from checkpoint or best genome file, else Candidate 0
            genome_cls = get_genome_class(p)
            preview_genes = None
            ckpt_path = os.path.join(GA_OPTIMIZER_DIR, f"ga_checkpoint_{p.lower()}.json") if is_multi else CHECKPOINT_FILE
            if os.path.exists(ckpt_path):
                try:
                    with open(ckpt_path, "r", encoding="utf-8") as f:
                        preview_genes = json.load(f).get("best_genome")
                except Exception:
                    pass

            if preview_genes is None:
                target_bg = seed_genome_file or (os.path.join(GA_OPTIMIZER_DIR, f"best_genome_{p.lower()}.json") if is_multi else BEST_GENOME_FILE)
                if os.path.exists(target_bg):
                    try:
                        with open(target_bg, "r", encoding="utf-8") as f:
                            d = json.load(f)
                            preview_genes = d.get("genes") or d.get("best_genome")
                    except Exception:
                        pass

            if preview_genes is None:
                seeds = generate_logical_initial_population(genome_cls, 1)
                preview_genes = seeds[0].to_dict()

            fit_prev, metrics_prev = run_rendered_preview(
                planner_name=p,
                gene_dict=preview_genes,
                duration_s=20.0,
                port=BASE_WORKER_PORT + 99,
                controller=controller,
            )
            print(f"[Preview Complete: {p}] Fitness: {fit_prev:.1f} | Dist: {metrics_prev.get('distance_m', 0.0):.2f}m | Speed: {metrics_prev.get('mean_speed_mps', 0.0):.2f}m/s | Crashed: {metrics_prev.get('crashed', False)}", flush=True)

        if controller and (controller.abort_requested or controller.stop_requested):
            print("[GA Optimizer] Abort/Stop requested after preview. Exiting.", flush=True)
            return

    if controller and (controller.abort_requested or controller.stop_requested):
        print("[GA Optimizer] Abort/Stop requested before worker pool initialization. Exiting.", flush=True)
        return

    if controller and controller.on_status_change:
        controller.on_status_change("RUNNING")

    pool = ParallelWorkerPool(num_workers=num_cores, controller=controller)

    # Wire pool into controller
    if controller is not None:
        controller._pool = pool

    # Clean shutdown on Ctrl+C (only if in main thread)
    try:
        import threading
        if threading.current_thread() is threading.main_thread():
            def sig_handler(sig, frame):
                print("\n[Optimizer] Interrupted by user. Shutting down worker pool...", flush=True)
                pool.shutdown()
                sys.exit(0)

            signal.signal(signal.SIGINT, sig_handler)
            signal.signal(signal.SIGTERM, sig_handler)
    except Exception:
        pass

    # State per planner
    populations: Dict[str, List[BasePlannerGenome]] = {}
    best_fitnesses: Dict[str, float] = {}
    best_genomes: Dict[str, Optional[BasePlannerGenome]] = {}
    best_metrics_map: Dict[str, Dict[str, Any]] = {}
    start_gens: Dict[str, int] = {}
    cma_adapters: Dict[str, Optional[CMAESOptimizerAdapter]] = {}
    consecutive_crashes: Dict[str, int] = {}

    for p in planners_to_run:
        genome_cls = get_genome_class(p)
        populations[p] = []
        best_fitnesses[p] = -1e9
        best_genomes[p] = None
        best_metrics_map[p] = {}
        start_gens[p] = 0
        cma_adapters[p] = None
        consecutive_crashes[p] = 0

        # Checkpoint file specific to planner
        planner_ckpt = os.path.join(CURR_DIR, f"ga_checkpoint_{p.lower()}.json")
        ckpt_path = planner_ckpt if (is_multi or os.path.exists(planner_ckpt)) else CHECKPOINT_FILE
        planner_cma = os.path.join(CURR_DIR, f"cma_state_{p.lower()}.pkl")
        cma_path = planner_cma if (is_multi or os.path.exists(planner_cma)) else os.path.join(CURR_DIR, "cma_state.pkl")

        if resume and os.path.exists(ckpt_path):
            try:
                with open(ckpt_path, "r", encoding="utf-8") as f:
                    ckpt = json.load(f)
                    ckpt_p = ckpt.get("planner", "")
                    if ckpt_p and ckpt_p != p:
                        print(f"[Optimizer] Checkpoint in {ckpt_path} is for {ckpt_p}, not {p}. Starting fresh for {p}.", flush=True)
                        populations[p] = []
                    else:
                        start_gens[p] = ckpt.get("generation", 0) + 1
                        pop_dicts = ckpt.get("population", [])
                        populations[p] = [genome_cls(d) for d in pop_dicts]
                        best_fitnesses[p] = float(ckpt.get("best_fitness", -1e9))
                        best_genomes[p] = genome_cls(ckpt.get("best_genome", {}))
                        best_metrics_map[p] = ckpt.get("best_metrics", {})
                        print(f"[Optimizer] Resuming {p} from Generation {start_gens[p]} (Best Fitness: {best_fitnesses[p]:.1f}).", flush=True)
            except Exception as e:
                print(f"[Optimizer] Checkpoint load failed for {p} ({e}), starting fresh.", flush=True)
                populations[p] = []

        if not populations[p] and (use_best_genome or seed_genome_file):
            target_seed_path = seed_genome_file or (os.path.join(CURR_DIR, f"best_genome_{p.lower()}.json") if is_multi else BEST_GENOME_FILE)
            loaded_g, loaded_fit, loaded_met = load_best_planner_genome(target_seed_path, genome_cls)
            if loaded_g is not None:
                best_genomes[p] = loaded_g
                best_fitnesses[p] = loaded_fit
                best_metrics_map[p] = loaded_met
                print(f"[Optimizer] Seeding {p} from best genome in {os.path.basename(target_seed_path)} (Fitness: {loaded_fit:.1f}) instead of defaults.", flush=True)

        if is_cma:
            init_g = best_genomes[p]
            if init_g is None:
                seeds = generate_logical_initial_population(genome_cls, 1)
                init_g = seeds[0] if seeds else genome_cls()

            adapter = CMAESOptimizerAdapter(
                genome_cls=genome_cls,
                pop_size=pop_size,
                sigma0=CMA_SIGMA0,
                initial_genome=init_g,
            )
            cma_loaded = False
            if resume and os.path.exists(cma_path):
                cma_loaded = adapter.load_state(cma_path)
                if cma_loaded:
                    print(f"[CMA-ES] Loaded covariance matrix state from {cma_path}", flush=True)

            if not populations[p] or not cma_loaded:
                populations[p] = adapter.ask(inject_baseline=True)
            elif cma_loaded and not populations[p]:
                populations[p] = adapter.ask(inject_baseline=True)
            elif cma_loaded and adapter.last_candidates_u is None and populations[p]:
                adapter.set_current_population(populations[p])

            cma_adapters[p] = adapter
        else:
            if not populations[p]:
                populations[p] = initialize_population(genome_cls, pop_size, seed_genome=best_genomes[p])

    # Initialize CSV Log
    if not resume or not os.path.exists(HISTORY_CSV_FILE):
        with open(HISTORY_CSV_FILE, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "Generation",
                "Planner",
                "Best_Fitness",
                "Mean_Fitness",
                "Best_Distance_m",
                "Best_Speed_mps",
                "Best_Laps",
                "Crash_Count",
                "Gen_Time_s",
            ])

    algo_name = "CMA-ES (Covariance Matrix Adaptation)" if is_cma else "GENETIC ALGORITHM"
    print("=" * 80, flush=True)
    print(f"  STARTING {algo_name} OPTIMIZATION", flush=True)
    print(f"  - Planners: {', '.join(planners_to_run)}")
    print(f"  - Population: {pop_size} candidates per planner | Generations: {num_generations}")
    print(f"  - Workers: {num_cores} CPU Cores | Eval Timeframe: {eval_duration_s:.1f} s")
    seed_str = f"Best Genome ({os.path.basename(seed_genome_file or BEST_GENOME_FILE)})" if (use_best_genome or seed_genome_file) else "Planner Config Defaults"
    print(f"  - Seed Baseline: {seed_str}")
    if is_cma:
        print(f"  - CMA-ES Sigma0: {CMA_SIGMA0:.2f} | Restart on Convergence: {CMA_RESTART_ON_CONVERGENCE}")
    else:
        print(f"  - Elitism: {ELITISM_COUNT} | Crossover: {CROSSOVER_RATE} | Mutation: {MUTATION_RATE}")
    print("=" * 80, flush=True)

    min_start_gen = min(start_gens.values())

    try:
        for gen in range(min_start_gen, num_generations):
            if controller and controller.abort_requested:
                print("[Optimizer] Abort requested. Skipping saving and exiting immediately.", flush=True)
                break
            if controller and controller.stop_requested:
                print("[Optimizer] Stop requested. Previous generation saved. Closing smoothly...", flush=True)
                break

            print(f"\n================================================================================", flush=True)
            print(f"  GENERATION {gen + 1} / {num_generations}  [{algo_name}]", flush=True)
            print(f"================================================================================", flush=True)

            for p in planners_to_run:
                if controller and controller.abort_requested:
                    print("[Optimizer] Abort requested. Exiting without save.", flush=True)
                    break
                if controller and controller.stop_requested:
                    print("[Optimizer] Stop requested before starting next planner.", flush=True)
                    break

                # Handle Pause Event
                if controller:
                    controller.pause_event.wait()

                if gen < start_gens[p]:
                    continue

                genome_cls = get_genome_class(p)
                t_p_start = time.time()
                print(f"\n--- [{p}] Evaluating {pop_size} Candidates on {num_cores} Workers ---", flush=True)

                # 1. Parallel Evaluation of Generation
                results = pool.evaluate_population(
                    planner_name=p,
                    population_genomes=populations[p],
                    duration_s=eval_duration_s,
                    controller=controller,
                )

                if controller and controller.abort_requested:
                    print("[Optimizer] Abort requested during evaluation. Skipping save.", flush=True)
                    break

                fitnesses = [r[0] for r in results]
                metrics_list = [r[1] for r in results]

                # 2. Analyze Generation Performance
                best_idx = int(np.argmax(fitnesses))
                gen_best_fit = float(fitnesses[best_idx])
                gen_mean_fit = float(np.mean(fitnesses))
                gen_best_metrics = metrics_list[best_idx]
                gen_crashes = sum(1 for m in metrics_list if m.get("crashed", False))
                t_p_elapsed = time.time() - t_p_start

                print(
                    f"[{p} Gen {gen + 1:02d}] Best Fit: {gen_best_fit:7.1f} | Mean Fit: {gen_mean_fit:7.1f} | "
                    f"Dist: {gen_best_metrics.get('distance_m', 0.0):5.2f}m | "
                    f"Speed: {gen_best_metrics.get('mean_speed_mps', 0.0):4.2f}m/s | "
                    f"Laps: {gen_best_metrics.get('laps_completed', 0)} | "
                    f"Crashes: {gen_crashes:2d}/{pop_size} | Time: {t_p_elapsed:4.1f}s",
                    flush=True,
                )

                # Update Planner Champion
                if gen_best_fit > best_fitnesses[p]:
                    best_fitnesses[p] = gen_best_fit
                    best_genomes[p] = populations[p][best_idx]
                    best_metrics_map[p] = gen_best_metrics
                    # Keep CMA-ES adapter locked to the active all-time champion
                    if is_cma and cma_adapters[p] is not None:
                        cma_adapters[p].initial_genome = best_genomes[p]
                    print(f"  *** NEW CHAMPION FOR {p}! Dist: {best_metrics_map[p].get('distance_m', 0.0):.2f}m, Fit: {gen_best_fit:.1f} ***", flush=True)

                    # Save Planner Champion JSON
                    champion_file = os.path.join(CURR_DIR, f"best_genome_{p.lower()}.json") if is_multi else BEST_GENOME_FILE
                    with open(champion_file, "w", encoding="utf-8") as f:
                        json.dump({
                            "planner": p,
                            "fitness": round(best_fitnesses[p], 2),
                            "metrics": best_metrics_map[p],
                            "genes": best_genomes[p].to_dict(),
                            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                        }, f, indent=2)

                # Append to History CSV
                with open(HISTORY_CSV_FILE, "a", newline="", encoding="utf-8") as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        gen + 1,
                        p,
                        round(gen_best_fit, 2),
                        round(gen_mean_fit, 2),
                        gen_best_metrics.get("distance_m", 0.0),
                        gen_best_metrics.get("mean_speed_mps", 0.0),
                        gen_best_metrics.get("laps_completed", 0),
                        gen_crashes,
                        round(t_p_elapsed, 2),
                    ])

                # Trigger GUI / Listener Callback
                if controller and controller.on_generation_done:
                    try:
                        controller.on_generation_done(
                            gen + 1,
                            p,
                            gen_best_metrics,
                            best_genomes[p].to_dict() if best_genomes[p] else {},
                            fitnesses,
                            t_p_elapsed,
                            gen_crashes,
                        )
                    except Exception as cb_err:
                        print(f"[Optimizer] Callback error: {cb_err}", flush=True)

                # 3. Form Next Generation
                if is_cma and cma_adapters[p] is not None:
                    adapter = cma_adapters[p]
                    adapter.tell(fitnesses)

                    # Watchdog: detect complete generation collapse (e.g. 100% crashes / zero gradient)
                    if gen_crashes >= pop_size:
                        consecutive_crashes[p] = consecutive_crashes.get(p, 0) + 1
                    else:
                        consecutive_crashes[p] = 0

                    if consecutive_crashes.get(p, 0) >= 3 and best_genomes[p] is not None:
                        print(
                            f"\n[Optimizer Watchdog] Detected {consecutive_crashes[p]} consecutive 100% crash generations for {p}!\n"
                            f"  -> Rescuing CMA distribution: force-restarting around top champion (Best Fit: {best_fitnesses[p]:.1f}) with tight sigma.",
                            flush=True,
                        )
                        adapter.restart_around_champion(best_genomes[p], sigma_scale=0.2)
                        consecutive_crashes[p] = 0

                    if gen < num_generations - 1:
                        populations[p] = adapter.ask(inject_baseline=True)
                    # Save CMA-ES state (after ask so that iteration counter and candidates are aligned)
                    cma_file = os.path.join(CURR_DIR, f"cma_state_{p.lower()}.pkl") if is_multi else os.path.join(CURR_DIR, "cma_state.pkl")
                    adapter.save_state(cma_file)
                else:
                    if gen < num_generations - 1:
                        sorted_indices = np.argsort(fitnesses)[::-1]
                        next_population: List[BasePlannerGenome] = []

                        # Elitism: retain top N candidates
                        for e in range(min(ELITISM_COUNT, pop_size)):
                            elite_idx = sorted_indices[e]
                            elite_copy = genome_cls()
                            if gen_crashes >= pop_size and best_genomes[p] is not None and e == 0:
                                # When whole generation crashed, inject known champion as elite 0
                                elite_copy.from_array(best_genomes[p].to_array())
                            else:
                                elite_copy.from_array(populations[p][elite_idx].to_array())
                            next_population.append(elite_copy)

                        # Fill remainder via Tournament Selection + Crossover + Mutation
                        while len(next_population) < pop_size:
                            p1 = tournament_select(populations[p], fitnesses)
                            p2 = tournament_select(populations[p], fitnesses)
                            c1, c2 = crossover_arithmetic(p1, p2)
                            c1 = mutate_gaussian(c1)
                            c2 = mutate_gaussian(c2)

                            next_population.append(c1)
                            if len(next_population) < pop_size:
                                next_population.append(c2)

                        populations[p] = next_population

                # 4. Save Planner Checkpoint for Next Generation
                ckpt_file = os.path.join(CURR_DIR, f"ga_checkpoint_{p.lower()}.json") if is_multi else CHECKPOINT_FILE
                with open(ckpt_file, "w", encoding="utf-8") as f:
                    json.dump({
                        "generation": gen,
                        "planner": p,
                        "algorithm": "CMA_ES" if is_cma else "GA",
                        "best_fitness": round(best_fitnesses[p], 2),
                        "best_metrics": best_metrics_map[p],
                        "best_genome": best_genomes[p].to_dict() if best_genomes[p] else {},
                        "population": [g.to_dict() for g in populations[p]],
                    }, f, indent=2)

    finally:
        # Shut down pool unconditionally
        if pool is not None:
            pool.shutdown()
        if controller is not None:
            controller._pool = None

    if controller and controller.abort_requested:
        print("\n[Optimizer] Optimization aborted by user. Nothing saved.", flush=True)
    else:
        print("\n" + "=" * 80, flush=True)
        print(f"  ALL OPTIMIZATIONS COMPLETE / FINISHED! [{algo_name}]", flush=True)
        for p in planners_to_run:
            print(f"  [{p}] Best Fit: {best_fitnesses[p]:.1f} | Dist: {best_metrics_map[p].get('distance_m', 0.0):.2f} m | Speed: {best_metrics_map[p].get('mean_speed_mps', 0.0):.2f} m/s", flush=True)
        print("=" * 80, flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Headless Optimizer for ToF Path Planners")
    parser.add_argument("--planner", type=str, default=ACTIVE_PLANNER, help="Target planner (default: OPTION_2_SL_MPC)")
    parser.add_argument("--algorithm", type=str, default=ALGORITHM, choices=["CMA_ES", "cma_es", "cma", "GA", "ga", "genetic_algorithm"], help="Optimization algorithm: CMA_ES or GA")
    parser.add_argument("--cores", type=int, default=NUM_CPU_CORES, help="Number of parallel Webots worker processes")
    parser.add_argument("--pop-size", type=int, default=POPULATION_SIZE, help="Population size per generation")
    parser.add_argument("--generations", type=int, default=NUM_GENERATIONS, help="Total number of generations")
    parser.add_argument("--duration", type=float, default=EVALUATION_DURATION_S, help="Simulation evaluation duration (s)")
    parser.add_argument("--resume", action="store_true", help="Resume from last checkpoint if available")
    parser.add_argument("--preview", action="store_true", help="Run 1 rendered 20s preview per planner in real-time Webots GUI before optimization")
    parser.add_argument("--use-best-genome", "--use-best", "--seed-from-best", dest="use_best_genome", action="store_true", help="Seed optimizer using webots/optimization/ga_planner_optimizer/best_planner_genome.json instead of planner defaults")
    parser.add_argument("--seed-file", "--genome-file", dest="seed_file", type=str, default=None, help="Custom path to best genome JSON file to seed from")

    args = parser.parse_args()

    run_genetic_algorithm(
        planner_name=args.planner,
        num_cores=args.cores,
        pop_size=args.pop_size,
        num_generations=args.generations,
        eval_duration_s=args.duration,
        resume=args.resume,
        preview_rendered=args.preview,
        algorithm=args.algorithm,
        use_best_genome=args.use_best_genome,
        seed_genome_file=args.seed_file,
    )
