"""
================================================================================
  HIGH-THROUGHPUT PARALLEL MPPI GENETIC ALGORITHM OPTIMIZER
================================================================================
  Optimizes the 23-gene 0-hidden-layer MPPIGenome for the 4WD Folkrace car.
  Runs multi-threaded parallel Webots instances across CPU cores on Windows.
  
  Algorithms:
    - CMA-ES (Covariance Matrix Adaptation Evolution Strategy) [Default]
    - DE (Differential Evolution: DE/best/1/bin)
    - GA (Tournament Selection + SBX Crossover + Polynomial Mutation)
================================================================================
"""

import os
import sys
import time
import json
import csv
import math
import argparse
import signal
import subprocess
from typing import Dict, List, Tuple, Optional, Any
from concurrent.futures import ThreadPoolExecutor, as_completed

import numpy as np

# Ensure controller and tools paths are available
WORKSPACE_ROOT = os.path.dirname(os.path.abspath(__file__))
FOLKRACE_DIR = os.path.join(WORKSPACE_ROOT, "my_project", "controllers", "folkrace_drive")
MPPI_DIR = os.path.join(WORKSPACE_ROOT, "my_project", "controllers", "mppi_controller")
TOOLS_DIR = os.path.join(WORKSPACE_ROOT, "tools")

for p in [WORKSPACE_ROOT, FOLKRACE_DIR, MPPI_DIR, TOOLS_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from mppi_folkrace_controller import MPPIGenome
from ga_simulation_worker import evaluate_genome, compute_fitness, PersistentWorkerPool

# Try importing CMA-ES libraries
try:
    import cmaes
    CMAES_AVAILABLE = True
except ImportError:
    CMAES_AVAILABLE = False

try:
    import cma
    PYCMA_AVAILABLE = True
except ImportError:
    PYCMA_AVAILABLE = False


CHECKPOINT_FILE = os.path.join(WORKSPACE_ROOT, "ga_checkpoint.json")
HISTORY_CSV_FILE = os.path.join(WORKSPACE_ROOT, "ga_history.csv")
WINNING_GENOME_FILE = os.path.join(WORKSPACE_ROOT, "winning_mppi_genome.json")
GEN_SUMMARY_FILE = os.path.join(WORKSPACE_ROOT, "ga_generation_summary.json")


def normalize_genome(arr: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    """Normalizes physical genome array to [0.0, 1.0] unit cube."""
    return np.clip((arr - lower) / (upper - lower + 1e-12), 0.0, 1.0)


def denormalize_genome(norm_arr: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    """Denormalizes [0.0, 1.0] vector back to physical gene bounds and repairs constraints."""
    scaled = lower + np.clip(norm_arr, 0.0, 1.0) * (upper - lower)
    return repair_and_clamp_genome(scaled)


def repair_and_clamp_genome(arr: np.ndarray) -> np.ndarray:
    """
    Clamps all 27 genes to valid ranges, rounds integer genes,
    and enforces the strict compute ceiling constraint: H * K <= 15420.
    """
    repaired = np.copy(arr)
    for idx, (name, val_min, val_max, val_type) in enumerate(MPPIGenome.GENE_SPECS):
        val = repaired[idx]
        val = max(val_min, min(val_max, val))
        if val_type is int:
            val = float(int(round(val)))
        repaired[idx] = val

    # Enforce compute budget coupling: K * H <= 15420
    h_val = int(repaired[0])
    k_val = int(repaired[1])
    max_budget = 15420
    if h_val * k_val > max_budget:
        k_val = max(128, max_budget // max(1, h_val))
        repaired[1] = float(k_val)

    # Ensure dt stays in [0.040, 0.080] range for deep lookahead (0.8s to 2.4s)
    repaired[2] = max(0.040, min(0.080, repaired[2]))
    # Ensure lambda_temp stays in [0.10, 1.50] range for sharp apexing vs smooth blending
    repaired[3] = max(0.10, min(1.50, repaired[3]))

    # Ensure mu_friction stays in strict [0.55, 0.80] range for high stability
    repaired[6] = max(0.55, min(0.80, repaired[6]))
    # Ensure r_inflation_m stays in [0.080, 0.105] range (0.5cm - 3.0cm physical clearance)
    repaired[23] = max(0.080, min(0.105, repaired[23]))

    # Ensure forward seed ratios leave at least 10% for right lane:
    sum_r = repaired[19] + repaired[20] + repaired[21]
    if sum_r > 0.90:
        scale = 0.90 / sum_r
        repaired[19] *= scale
        repaired[20] *= scale
        repaired[21] *= scale

    # Reversing parameters bounds enforcement
    repaired[24] = max(50.0, min(800.0, repaired[24]))   # w_reverse
    repaired[25] = max(0.40, min(1.50, repaired[25]))    # reverse_speed
    repaired[26] = float(int(round(max(2.0, min(7.0, repaired[26]))))) # t_rev_steps

    return repaired


def get_conservative_parent_archetypes() -> List[MPPIGenome]:
    """
    Constructs 16 distinct, high-performing parent archetypes covering a wide variety
    of driving styles, reversing dynamics, deep lookahead horizons (0.8s - 2.4s), and flexible clearance margins (0.080 - 0.105m).
    """
    archetypes = []

    # 1. Ultra-Safe High-Clearance Scout (Anchor Baseline, dt=0.055 -> 1.54s horizon)
    archetypes.append(MPPIGenome(
        H=28, K=512, dt=0.055,
        lambda_temp=0.45, ar1_alpha=0.88, gamma_discount=0.96, mu_friction=0.68,
        w_cte=8.0, w_heading=4.0, w_speed=1.5, w_hazard=300000.0,
        w_prob_costmap=65.0, w_unknown=10.0, w_steer_rate=1.20, w_accel_rate=0.40,
        w_terminal=8.0, sigma_steer=0.08, sigma_speed=0.20, lognorm_shape=0.15,
        ratio_center=0.35, ratio_straight=0.20, ratio_left=0.25, pure_pursuit_lookahead=0.45,
        r_inflation_m=0.095, w_reverse=250.0, reverse_speed=0.80, t_rev_steps=3
    ))

    # 2. Dynamic Obstacle Dodger (Wide Multi-Lane Corridors, dt=0.050 -> 1.30s horizon)
    archetypes.append(MPPIGenome(
        H=26, K=512, dt=0.050,
        lambda_temp=0.50, ar1_alpha=0.90, gamma_discount=0.95, mu_friction=0.65,
        w_cte=4.0, w_heading=3.0, w_speed=1.6, w_hazard=350000.0,
        w_prob_costmap=70.0, w_unknown=10.0, w_steer_rate=0.85, w_accel_rate=0.35,
        w_terminal=8.0, sigma_steer=0.10, sigma_speed=0.22, lognorm_shape=0.15,
        ratio_center=0.20, ratio_straight=0.25, ratio_left=0.35, pure_pursuit_lookahead=0.42,
        r_inflation_m=0.088, w_reverse=180.0, reverse_speed=0.95, t_rev_steps=3
    ))

    # 3. Deep Horizon Predictive Brake (Long Lookahead ~2.0s Window)
    archetypes.append(MPPIGenome(
        H=32, K=480, dt=0.065,
        lambda_temp=0.35, ar1_alpha=0.86, gamma_discount=0.97, mu_friction=0.66,
        w_cte=10.0, w_heading=5.0, w_speed=1.8, w_hazard=250000.0,
        w_prob_costmap=60.0, w_unknown=10.0, w_steer_rate=0.90, w_accel_rate=0.30,
        w_terminal=12.0, sigma_steer=0.08, sigma_speed=0.20, lognorm_shape=0.18,
        ratio_center=0.35, ratio_straight=0.20, ratio_left=0.25, pure_pursuit_lookahead=0.55,
        r_inflation_m=0.092, w_reverse=300.0, reverse_speed=0.70, t_rev_steps=4
    ))

    # 4. Agile 3-Point Cusp Specialist (Quick Snap Escape, dt=0.045 -> 1.12s horizon)
    archetypes.append(MPPIGenome(
        H=25, K=512, dt=0.045,
        lambda_temp=0.30, ar1_alpha=0.85, gamma_discount=0.94, mu_friction=0.70,
        w_cte=7.0, w_heading=3.5, w_speed=1.6, w_hazard=350000.0,
        w_prob_costmap=65.0, w_unknown=10.0, w_steer_rate=0.80, w_accel_rate=0.35,
        w_terminal=7.0, sigma_steer=0.10, sigma_speed=0.25, lognorm_shape=0.18,
        ratio_center=0.30, ratio_straight=0.25, ratio_left=0.25, pure_pursuit_lookahead=0.38,
        r_inflation_m=0.085, w_reverse=120.0, reverse_speed=1.20, t_rev_steps=2
    ))

    # 5. Deep-Clearance Backing Strategist (Conservative Unsticker, dt=0.055)
    archetypes.append(MPPIGenome(
        H=28, K=512, dt=0.055,
        lambda_temp=0.45, ar1_alpha=0.88, gamma_discount=0.95, mu_friction=0.64,
        w_cte=9.0, w_heading=4.5, w_speed=1.5, w_hazard=400000.0,
        w_prob_costmap=70.0, w_unknown=12.0, w_steer_rate=1.10, w_accel_rate=0.40,
        w_terminal=9.0, sigma_steer=0.08, sigma_speed=0.18, lognorm_shape=0.14,
        ratio_center=0.35, ratio_straight=0.15, ratio_left=0.25, pure_pursuit_lookahead=0.45,
        r_inflation_m=0.100, w_reverse=380.0, reverse_speed=0.65, t_rev_steps=6
    ))

    # 6. Straight Gap Puncher (Chicane Shooter, dt=0.048)
    archetypes.append(MPPIGenome(
        H=24, K=512, dt=0.048,
        lambda_temp=0.40, ar1_alpha=0.86, gamma_discount=0.94, mu_friction=0.68,
        w_cte=5.0, w_heading=3.0, w_speed=1.7, w_hazard=300000.0,
        w_prob_costmap=60.0, w_unknown=10.0, w_steer_rate=0.80, w_accel_rate=0.30,
        w_terminal=6.0, sigma_steer=0.10, sigma_speed=0.24, lognorm_shape=0.20,
        ratio_center=0.25, ratio_straight=0.35, ratio_left=0.20, pure_pursuit_lookahead=0.40,
        r_inflation_m=0.084, w_reverse=200.0, reverse_speed=0.85, t_rev_steps=3
    ))

    # 7. Apex Hugger Racing Purist (Curvature Optimal Line & Sharp Apex, dt=0.046)
    archetypes.append(MPPIGenome(
        H=26, K=512, dt=0.046,
        lambda_temp=0.18, ar1_alpha=0.88, gamma_discount=0.94, mu_friction=0.72,
        w_cte=16.0, w_heading=6.0, w_speed=1.8, w_hazard=400000.0,
        w_prob_costmap=55.0, w_unknown=10.0, w_steer_rate=1.20, w_accel_rate=0.40,
        w_terminal=8.0, sigma_steer=0.07, sigma_speed=0.18, lognorm_shape=0.15,
        ratio_center=0.55, ratio_straight=0.15, ratio_left=0.15, pure_pursuit_lookahead=0.28,
        r_inflation_m=0.082, w_reverse=320.0, reverse_speed=0.75, t_rev_steps=3
    ))

    # 8. Greedy Winner-Takes-All Sprinter (Zero Hesitation Apex, dt=0.044)
    archetypes.append(MPPIGenome(
        H=25, K=512, dt=0.044,
        lambda_temp=0.15, ar1_alpha=0.85, gamma_discount=0.93, mu_friction=0.70,
        w_cte=8.0, w_heading=4.0, w_speed=1.8, w_hazard=350000.0,
        w_prob_costmap=60.0, w_unknown=10.0, w_steer_rate=0.90, w_accel_rate=0.35,
        w_terminal=7.0, sigma_steer=0.09, sigma_speed=0.22, lognorm_shape=0.20,
        ratio_center=0.35, ratio_straight=0.25, ratio_left=0.20, pure_pursuit_lookahead=0.35,
        r_inflation_m=0.085, w_reverse=160.0, reverse_speed=1.05, t_rev_steps=3
    ))

    # 9. Soft-Blending Fluid Cruiser (Ultra-Smooth Momentum, dt=0.058)
    archetypes.append(MPPIGenome(
        H=27, K=512, dt=0.058,
        lambda_temp=0.75, ar1_alpha=0.92, gamma_discount=0.96, mu_friction=0.64,
        w_cte=6.0, w_heading=3.5, w_speed=1.5, w_hazard=300000.0,
        w_prob_costmap=70.0, w_unknown=12.0, w_steer_rate=1.60, w_accel_rate=0.50,
        w_terminal=8.0, sigma_steer=0.06, sigma_speed=0.16, lognorm_shape=0.12,
        ratio_center=0.35, ratio_straight=0.20, ratio_left=0.25, pure_pursuit_lookahead=0.48,
        r_inflation_m=0.095, w_reverse=260.0, reverse_speed=0.70, t_rev_steps=3
    ))

    # 10. High-Traction Aggressive Tracker (High Grip Limit, dt=0.045)
    archetypes.append(MPPIGenome(
        H=25, K=512, dt=0.045,
        lambda_temp=0.30, ar1_alpha=0.82, gamma_discount=0.94, mu_friction=0.78,
        w_cte=10.0, w_heading=4.5, w_speed=2.0, w_hazard=350000.0,
        w_prob_costmap=55.0, w_unknown=10.0, w_steer_rate=0.80, w_accel_rate=0.30,
        w_terminal=7.0, sigma_steer=0.11, sigma_speed=0.30, lognorm_shape=0.25,
        ratio_center=0.40, ratio_straight=0.20, ratio_left=0.20, pure_pursuit_lookahead=0.35,
        r_inflation_m=0.082, w_reverse=220.0, reverse_speed=0.95, t_rev_steps=3
    ))

    # 11. Low-Traction Conservative Runner (Wet/Low-Grip Safety, dt=0.060)
    archetypes.append(MPPIGenome(
        H=28, K=512, dt=0.060,
        lambda_temp=0.45, ar1_alpha=0.88, gamma_discount=0.95, mu_friction=0.58,
        w_cte=12.0, w_heading=5.5, w_speed=1.3, w_hazard=450000.0,
        w_prob_costmap=80.0, w_unknown=12.0, w_steer_rate=1.40, w_accel_rate=0.45,
        w_terminal=9.0, sigma_steer=0.07, sigma_speed=0.15, lognorm_shape=0.12,
        ratio_center=0.35, ratio_straight=0.15, ratio_left=0.25, pure_pursuit_lookahead=0.45,
        r_inflation_m=0.102, w_reverse=320.0, reverse_speed=0.60, t_rev_steps=4
    ))

    # 12. Symmetric 4-Corridor Equal Allocator (Equal Exploration, dt=0.052)
    archetypes.append(MPPIGenome(
        H=26, K=512, dt=0.052,
        lambda_temp=0.45, ar1_alpha=0.85, gamma_discount=0.94, mu_friction=0.66,
        w_cte=6.0, w_heading=3.5, w_speed=1.6, w_hazard=300000.0,
        w_prob_costmap=65.0, w_unknown=10.0, w_steer_rate=0.90, w_accel_rate=0.35,
        w_terminal=7.0, sigma_steer=0.10, sigma_speed=0.22, lognorm_shape=0.18,
        ratio_center=0.25, ratio_straight=0.25, ratio_left=0.25, pure_pursuit_lookahead=0.40,
        r_inflation_m=0.090, w_reverse=240.0, reverse_speed=0.80, t_rev_steps=3
    ))

    # 13. Cautious Reversing Turtle (Zero-Panic Backer, dt=0.055)
    archetypes.append(MPPIGenome(
        H=27, K=512, dt=0.055,
        lambda_temp=0.35, ar1_alpha=0.88, gamma_discount=0.95, mu_friction=0.65,
        w_cte=12.0, w_heading=5.0, w_speed=1.5, w_hazard=400000.0,
        w_prob_costmap=70.0, w_unknown=10.0, w_steer_rate=1.20, w_accel_rate=0.40,
        w_terminal=8.0, sigma_steer=0.08, sigma_speed=0.18, lognorm_shape=0.14,
        ratio_center=0.35, ratio_straight=0.20, ratio_left=0.25, pure_pursuit_lookahead=0.42,
        r_inflation_m=0.098, w_reverse=550.0, reverse_speed=0.55, t_rev_steps=4
    ))

    # 14. Eager Reverse Rebounder (Hair-Trigger Backer, dt=0.045)
    archetypes.append(MPPIGenome(
        H=24, K=512, dt=0.045,
        lambda_temp=0.35, ar1_alpha=0.84, gamma_discount=0.93, mu_friction=0.68,
        w_cte=5.5, w_heading=3.0, w_speed=1.7, w_hazard=350000.0,
        w_prob_costmap=60.0, w_unknown=10.0, w_steer_rate=0.75, w_accel_rate=0.30,
        w_terminal=6.0, sigma_steer=0.10, sigma_speed=0.26, lognorm_shape=0.22,
        ratio_center=0.30, ratio_straight=0.25, ratio_left=0.25, pure_pursuit_lookahead=0.36,
        r_inflation_m=0.086, w_reverse=80.0, reverse_speed=1.30, t_rev_steps=2
    ))

    # 15. Long-Horizon Momentum Glider (Deep Planning ~2.1s Window)
    archetypes.append(MPPIGenome(
        H=30, K=500, dt=0.070,
        lambda_temp=0.50, ar1_alpha=0.91, gamma_discount=0.985, mu_friction=0.66,
        w_cte=7.0, w_heading=4.0, w_speed=1.6, w_hazard=280000.0,
        w_prob_costmap=60.0, w_unknown=10.0, w_steer_rate=1.10, w_accel_rate=0.35,
        w_terminal=10.0, sigma_steer=0.08, sigma_speed=0.20, lognorm_shape=0.16,
        ratio_center=0.35, ratio_straight=0.20, ratio_left=0.25, pure_pursuit_lookahead=0.50,
        r_inflation_m=0.092, w_reverse=270.0, reverse_speed=0.75, t_rev_steps=4
    ))

    # 16. Master Balanced Champion (Gold Standard Baseline, dt=0.052 -> 1.35s horizon)
    archetypes.append(MPPIGenome(
        H=26, K=512, dt=0.052,
        lambda_temp=0.35, ar1_alpha=0.85, gamma_discount=0.94, mu_friction=0.68,
        w_cte=9.0, w_heading=4.0, w_speed=1.6, w_hazard=300000.0,
        w_prob_costmap=60.0, w_unknown=10.0, w_steer_rate=0.80, w_accel_rate=0.30,
        w_terminal=7.0, sigma_steer=0.09, sigma_speed=0.22, lognorm_shape=0.18,
        ratio_center=0.35, ratio_straight=0.20, ratio_left=0.25, pure_pursuit_lookahead=0.40,
        r_inflation_m=0.088, w_reverse=220.0, reverse_speed=0.85, t_rev_steps=3
    ))

    return archetypes


class EvolutionaryOptimizer:
    """
    Master Evolutionary Optimization Engine managing parallel simulation evaluations,
    CMA-ES / DE / GA algorithm transitions, and state persistence.
    """

    def __init__(
        self,
        algorithm: str = "cmaes",
        num_workers: int = 6,
        population_size: int = 24,
        sim_duration_s: float = 20.0,
        max_duration_hours: float = 9.0,
        resume: bool = False,
    ):
        self.algorithm = algorithm.lower()
        self.num_workers = num_workers
        self.pop_size = population_size
        self.sim_duration_s = sim_duration_s
        self.max_duration_s = max_duration_hours * 3600.0
        self.resume = resume

        self.start_wall_time = time.time()
        self.current_generation = 0
        self.total_evaluations = 0

        self.best_fitness = -1e9
        self.best_genome_arr: Optional[np.ndarray] = None
        self.best_metrics: Dict[str, Any] = {}

        self.shutdown_requested = False
        self.history_records: List[Dict[str, Any]] = []

        # Gene bounds matrices
        self.lower_bounds = np.array([spec[1] for spec in MPPIGenome.GENE_SPECS], dtype=np.float64)
        self.upper_bounds = np.array([spec[2] for spec in MPPIGenome.GENE_SPECS], dtype=np.float64)
        self.gene_names = [spec[0] for spec in MPPIGenome.GENE_SPECS]

        # Baseline seed chromosome & conservative archetypes
        self.conservative_archetypes = get_conservative_parent_archetypes()
        self.baseline_arr = self.conservative_archetypes[0].to_array()

        # Build initial conservative population
        self.initial_population = self._create_initial_population(self.pop_size)

        # Initialize Persistent Webots Worker Pool
        self.worker_pool = PersistentWorkerPool(num_workers=self.num_workers)

        # Initialize Optimizer backend
        self._init_optimizer()

        if not self.resume:
            # Clean start: archive old history if present and start completely fresh
            if os.path.exists(HISTORY_CSV_FILE):
                archive_name = f"ga_history_backup_{int(time.time())}.csv"
                try:
                    os.rename(HISTORY_CSV_FILE, os.path.join(WORKSPACE_ROOT, archive_name))
                    print(f"[Optimizer] Clean Start: Archived old history to {archive_name}", flush=True)
                except Exception:
                    pass
            if os.path.exists(GEN_SUMMARY_FILE):
                try:
                    os.remove(GEN_SUMMARY_FILE)
                except Exception:
                    pass
            if os.path.exists(CHECKPOINT_FILE):
                try:
                    os.remove(CHECKPOINT_FILE)
                except Exception:
                    pass
        elif self.resume and os.path.exists(CHECKPOINT_FILE):
            self.load_checkpoint()

    def _init_optimizer(self):
        """Initializes the algorithmic sampling engine in normalized [0, 1] unit space."""
        if self.algorithm == "cmaes" and CMAES_AVAILABLE:
            mean_norm = normalize_genome(self.baseline_arr, self.lower_bounds, self.upper_bounds)
            sigma = 0.15
            unit_bounds = np.column_stack([np.zeros(len(self.lower_bounds)), np.ones(len(self.upper_bounds))])
            self.cma_optimizer = cmaes.CMA(
                mean=mean_norm,
                sigma=sigma,
                bounds=unit_bounds,
                population_size=self.pop_size,
            )
            print(f"[Optimizer] Initialized cmaes.CMA in normalized [0, 1] space (pop={self.pop_size}, sigma={sigma:.2f})", flush=True)
        elif self.algorithm == "cmaes" and PYCMA_AVAILABLE:
            mean_norm = normalize_genome(self.baseline_arr, self.lower_bounds, self.upper_bounds)
            inopts = {
                "bounds": [np.zeros(len(self.lower_bounds)).tolist(), np.ones(len(self.upper_bounds)).tolist()],
                "popsize": self.pop_size,
                "verb_log": 0,
                "verb_disp": 0,
            }
            self.cma_optimizer = cma.CMAEvolutionStrategy(mean_norm.tolist(), 0.15, inopts)
            print(f"[Optimizer] Initialized pycma.CMAEvolutionStrategy in normalized [0, 1] space (pop={self.pop_size})", flush=True)
        else:
            # Fallback to Custom Differential Evolution (DE/best/1/bin)
            self.algorithm = "de"
            self.de_population = np.copy(self.initial_population)
            self.de_fitnesses = np.full(self.pop_size, -1e9)
            print(f"[Optimizer] Initialized Differential Evolution (DE) Engine with popsize={self.pop_size}", flush=True)

    def _create_initial_population(self, size: int) -> np.ndarray:
        """
        Generates initial population heavily seeded with conservative, crash-preventing
        parent archetypes and fine Gaussian mutations around them.
        """
        pop = np.zeros((size, len(self.baseline_arr)), dtype=np.float64)
        n_arch = len(self.conservative_archetypes)

        # 1. Fill leading slots with the 6 curated parent archetypes
        for i in range(min(size, n_arch)):
            pop[i] = self.conservative_archetypes[i].to_array()

        # 2. Fill remaining slots with gentle mutations around the archetypes (sigma ~ 5-8%)
        for i in range(n_arch, size):
            parent_idx = i % n_arch
            parent_arr = self.conservative_archetypes[parent_idx].to_array()
            noise_scale = np.random.uniform(0.03, 0.08)
            noise = np.random.normal(0, noise_scale, size=len(parent_arr)) * (self.upper_bounds - self.lower_bounds)
            cand = parent_arr + noise

            # Enforce conservative minimums on obstacle inflation and wall clearances
            cand[11] = max(40.0, cand[11])   # w_prob_costmap >= 40.0
            cand[23] = max(0.088, cand[23])  # r_inflation_m >= 8.8cm (guaranteed safety margin)

            pop[i] = repair_and_clamp_genome(cand)

        return pop

    def ask_candidates(self) -> List[np.ndarray]:
        """Requests candidate genome vectors for the current generation."""
        # In Generation 0, always evaluate the curated conservative parent population
        if self.current_generation == 0:
            return [np.copy(ind) for ind in self.initial_population]

        if self.algorithm == "cmaes" and CMAES_AVAILABLE:
            candidates = []
            for _ in range(self.pop_size):
                raw_norm = self.cma_optimizer.ask()
                physical = denormalize_genome(raw_norm, self.lower_bounds, self.upper_bounds)
                candidates.append(physical)
            return candidates

        elif self.algorithm == "cmaes" and PYCMA_AVAILABLE:
            raw_cands = self.cma_optimizer.ask()
            return [denormalize_genome(np.array(c, dtype=np.float64), self.lower_bounds, self.upper_bounds) for c in raw_cands]

        else:
            # Differential Evolution trial generation (DE/best/1/bin)
            trial_cands = []
            best_idx = int(np.argmax(self.de_fitnesses)) if self.total_evaluations > 0 else 0
            best_ind = self.de_population[best_idx]
            F = 0.80  # Mutation factor
            CR = 0.70  # Crossover probability

            for i in range(self.pop_size):
                idxs = [idx for idx in range(self.pop_size) if idx != i and idx != best_idx]
                r1, r2 = np.random.choice(idxs, size=2, replace=False)

                mutant = best_ind + F * (self.de_population[r1] - self.de_population[r2])

                cross_mask = np.random.rand(len(self.baseline_arr)) < CR
                cross_mask[np.random.randint(0, len(self.baseline_arr))] = True
                trial = np.where(cross_mask, mutant, self.de_population[i])

                trial_cands.append(repair_and_clamp_genome(trial))

            return trial_cands

    def tell_results(self, candidate_results: List[Tuple[np.ndarray, float, Dict[str, Any]]]):
        """Feeds evaluation results back into optimizer."""
        # candidate_results is a list of (candidate_arr, fitness, metrics)
        if self.algorithm == "cmaes" and CMAES_AVAILABLE:
            tell_data = []
            for cand, fit, _ in candidate_results:
                norm_cand = normalize_genome(cand, self.lower_bounds, self.upper_bounds)
                # CMA-ES minimizes loss -> loss = -fitness
                tell_data.append((norm_cand, -float(fit)))
            try:
                self.cma_optimizer.tell(tell_data)
            except Exception as e:
                print(f"[Optimizer] CMA tell notice: {e}", flush=True)

        elif self.algorithm == "cmaes" and PYCMA_AVAILABLE:
            cands_list = [normalize_genome(cand, self.lower_bounds, self.upper_bounds).tolist() for cand, _, _ in candidate_results]
            losses = [-float(fit) for _, fit, _ in candidate_results]
            self.cma_optimizer.tell(cands_list, losses)

        else:
            # Differential Evolution selection step
            if self.current_generation == 0:
                for idx, (_, fit, _) in enumerate(candidate_results):
                    self.de_fitnesses[idx] = fit
            else:
                for idx, (trial_arr, trial_fit, _) in enumerate(candidate_results):
                    if trial_fit >= self.de_fitnesses[idx]:
                        self.de_population[idx] = np.copy(trial_arr)
                        self.de_fitnesses[idx] = trial_fit

    def evaluate_population_parallel(self, candidates: List[np.ndarray]) -> List[Tuple[np.ndarray, float, Dict[str, Any]]]:
        """
        Dispatches candidate genomes across the persistent worker pool in parallel.
        """
        results: List[Optional[Tuple[np.ndarray, float, Dict[str, Any]]]] = [None] * len(candidates)

        def eval_task(cand_idx: int, cand_arr: np.ndarray):
            fit, met = self.worker_pool.evaluate_genome(
                genome_arr=cand_arr,
                sim_duration_s=self.sim_duration_s,
            )
            return cand_idx, cand_arr, fit, met

        with ThreadPoolExecutor(max_workers=self.num_workers) as executor:
            future_map = {
                executor.submit(eval_task, i, cand): i
                for i, cand in enumerate(candidates)
            }

            for fut in as_completed(future_map):
                if self.shutdown_requested:
                    break
                try:
                    c_idx, c_arr, fit, met = fut.result()
                    results[c_idx] = (c_arr, fit, met)
                    self.total_evaluations += 1

                    # Check for new global champion
                    if fit > self.best_fitness:
                        self.best_fitness = fit
                        self.best_genome_arr = np.copy(c_arr)
                        self.best_metrics = met
                        self.save_champion(c_arr, met)

                    # Log to CSV
                    self._log_history_record(self.current_generation, self.total_evaluations, fit, met, c_arr)

                except Exception as e:
                    print(f"[Optimizer] Worker execution failed on candidate: {e}", flush=True)

        return [r for r in results if r is not None]

    def _log_history_record(self, gen: int, eval_id: int, fit: float, met: Dict[str, Any], arr: np.ndarray):
        """Appends evaluation telemetry to history CSV."""
        record = {
            "generation": gen,
            "eval_id": eval_id,
            "fitness": round(fit, 2),
            "distance_m": met.get("distance_m", 0.0),
            "mean_speed_mps": met.get("mean_speed_mps", 0.0),
            "laps": met.get("laps_completed", 0),
            "best_lap_time": met.get("best_lap_time_s", 999.0),
            "collisions": met.get("collision_count", 0),
            "wall_cost": met.get("mean_wall_proximity_cost", 0.0),
            "jerk": met.get("steering_variance", 0.0),
            "stuck": met.get("stuck_abort", False),
            "all_time_best": round(self.best_fitness, 2),
        }
        for idx, name in enumerate(self.gene_names):
            record[name] = round(arr[idx], 4)

        file_exists = os.path.exists(HISTORY_CSV_FILE)
        with open(HISTORY_CSV_FILE, "a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(record.keys()))
            if not file_exists:
                writer.writeheader()
            writer.writerow(record)

    def save_champion(self, arr: np.ndarray, metrics: Dict[str, Any]):
        """Persists winning MPPIGenome to JSON."""
        genome = MPPIGenome.from_array(arr)
        champ_dict = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "fitness": round(metrics.get("fitness", 0.0), 2),
            "distance_m": metrics.get("distance_m", 0.0),
            "mean_speed_mps": metrics.get("mean_speed_mps", 0.0),
            "max_speed_mps": metrics.get("max_speed_mps", 0.0),
            "laps_completed": metrics.get("laps_completed", 0),
            "best_lap_time_s": metrics.get("best_lap_time_s", 999.0),
            "collision_count": metrics.get("collision_count", 0),
            "genes": {name: getattr(genome, name) for name in self.gene_names},
            "flat_array": arr.tolist(),
        }
        with open(WINNING_GENOME_FILE, "w", encoding="utf-8") as f:
            json.dump(champ_dict, f, indent=2)

    def save_checkpoint(self):
        """Saves optimizer state for resuming."""
        ckpt = {
            "generation": self.current_generation,
            "total_evaluations": self.total_evaluations,
            "elapsed_seconds": round(time.time() - self.start_wall_time, 1),
            "algorithm": self.algorithm,
            "best_fitness": self.best_fitness,
            "best_genome": self.best_genome_arr.tolist() if self.best_genome_arr is not None else None,
            "best_metrics": self.best_metrics,
        }
        with open(CHECKPOINT_FILE, "w", encoding="utf-8") as f:
            json.dump(ckpt, f, indent=2)

    def save_generation_summary(self, gen_idx: int, results: List[Tuple[np.ndarray, float, Dict[str, Any]]]):
        """Exports detailed per-generation summary JSON for the live GUI dashboard."""
        try:
            gen_fits = [r[1] for r in results]
            survived = sum(1 for r in results if r[2].get("collision_count", 0) == 0 and not r[2].get("stuck_abort", False))
            summary = {
                "generation": gen_idx,
                "evaluations": self.total_evaluations,
                "elapsed_seconds": round(time.time() - self.start_wall_time, 1),
                "algorithm": self.algorithm,
                "gen_best_fitness": round(float(max(gen_fits)), 2),
                "gen_mean_fitness": round(float(np.mean(gen_fits)), 2),
                "gen_median_fitness": round(float(np.median(gen_fits)), 2),
                "all_time_best_fitness": round(float(self.best_fitness), 2),
                "survived_count": survived,
                "total_in_gen": len(results),
                "best_trial_metrics": self.best_metrics,
                "best_genome_genes": {name: float(self.best_genome_arr[i]) for i, name in enumerate(self.gene_names)} if self.best_genome_arr is not None else {},
                "candidates": [
                    {
                        "rank": 0,
                        "cand_idx": i,
                        "fitness": round(float(r[1]), 2),
                        "distance_m": round(float(r[2].get("distance_m", 0.0)), 2),
                        "mean_speed_mps": round(float(r[2].get("mean_speed_mps", 0.0)), 2),
                        "max_speed_mps": round(float(r[2].get("max_speed_mps", 0.0)), 2),
                        "best_lap_time": round(float(r[2].get("best_lap_time_s", 999.0)), 2),
                        "collisions": int(r[2].get("collision_count", 0)),
                        "status": "STUCK" if r[2].get("stuck_abort") else ("CRASH" if r[2].get("collision_count", 0) > 0 else "SURVIVED"),
                    }
                    for i, r in enumerate(results)
                ]
            }
            summary["candidates"].sort(key=lambda c: c["fitness"], reverse=True)
            for rank, c in enumerate(summary["candidates"]):
                c["rank"] = rank + 1

            with open(GEN_SUMMARY_FILE, "w", encoding="utf-8") as f:
                json.dump(summary, f, indent=2)
        except Exception as e:
            print(f"[Optimizer] Failed to export generation summary: {e}", flush=True)

    def load_checkpoint(self):
        """Restores optimizer state from checkpoint."""
        try:
            with open(CHECKPOINT_FILE, "r", encoding="utf-8") as f:
                ckpt = json.load(f)
            self.current_generation = int(ckpt.get("generation", 0))
            self.total_evaluations = int(ckpt.get("total_evaluations", 0))
            self.best_fitness = float(ckpt.get("best_fitness", -1e9))
            if ckpt.get("best_genome") is not None:
                self.best_genome_arr = np.array(ckpt["best_genome"], dtype=np.float64)
            self.best_metrics = ckpt.get("best_metrics", {})
            print(f"[Optimizer] Restored checkpoint from Gen {self.current_generation} (Evals: {self.total_evaluations}, Best: {self.best_fitness:.1f})", flush=True)
        except Exception as e:
            print(f"[Optimizer] Could not load checkpoint: {e}", flush=True)

    def run(self, max_generations: Optional[int] = None):
        """Main training loop across generations until target hours reached."""
        print("\n================================================================================", flush=True)
        print(f"  STARTING PARALLEL MPPI GENETIC OPTIMIZER ({self.algorithm.upper()})", flush=True)
        print(f"  Workers: {self.num_workers} | PopSize: {self.pop_size} | SimDuration: {self.sim_duration_s:.1f}s", flush=True)
        print(f"  Max Session Duration: {self.max_duration_s / 3600.0:.1f} hours", flush=True)
        print("================================================================================\n", flush=True)

        header = f" {'Gen':<5} | {'Evals':<6} | {'Gen Best':<10} | {'All Best':<10} | {'Dist(m)':<8} | {'Speed(m/s)':<10} | {'Coll':<5} | {'Elapsed':<8} | {'ETA':<8}"
        print(header, flush=True)
        print("-" * len(header), flush=True)

        try:
            while not self.shutdown_requested:
                gen_start_t = time.time()
                elapsed_total = time.time() - self.start_wall_time

                # Check time budget limit
                if elapsed_total >= self.max_duration_s:
                    print(f"\n[Optimizer] Target training duration of {self.max_duration_s / 3600.0:.1f} hours completed. Concluding.", flush=True)
                    break

                if max_generations is not None and self.current_generation >= max_generations:
                    print(f"\n[Optimizer] Reached maximum generation count ({max_generations}). Concluding.", flush=True)
                    break

                # 1. Ask candidate genomes
                candidates = self.ask_candidates()

                # 2. Evaluate in parallel across worker pool
                results = self.evaluate_population_parallel(candidates)
                if not results or self.shutdown_requested:
                    break

                # 3. Tell optimizer results
                self.tell_results(results)

                # 4. Telemetry aggregation
                gen_fitnesses = [r[1] for r in results]
                gen_best_fit = max(gen_fitnesses)
                best_in_gen_idx = int(np.argmax(gen_fitnesses))
                best_in_gen_res = results[best_in_gen_idx]
                best_gen_met = best_in_gen_res[2]

                elapsed_h = elapsed_total / 3600.0
                remaining_s = max(0.0, self.max_duration_s - elapsed_total)
                eta_h = remaining_s / 3600.0

                dist_m = best_gen_met.get("distance_m", 0.0)
                spd = best_gen_met.get("mean_speed_mps", 0.0)
                colls = best_gen_met.get("collision_count", 0)

                print(
                    f" {self.current_generation:<5d} | "
                    f"{self.total_evaluations:<6d} | "
                    f"{gen_best_fit:>10.1f} | "
                    f"{self.best_fitness:>10.1f} | "
                    f"{dist_m:>8.2f} | "
                    f"{spd:>10.2f} | "
                    f"{colls:>5d} | "
                    f"{elapsed_h:>7.2f}h | "
                    f"{eta_h:>7.2f}h",
                    flush=True,
                )

                # 5. Export generation summary and save checkpoint
                self.save_generation_summary(self.current_generation, results)
                self.current_generation += 1
                self.save_checkpoint()

        finally:
            if hasattr(self, "worker_pool"):
                self.worker_pool.shutdown()

        print("\n================================================================================", flush=True)
        print("                         OPTIMIZATION SESSION COMPLETED                         ", flush=True)
        print("================================================================================", flush=True)
        print(f" Total Generations   : {self.current_generation}")
        print(f" Total Evaluations   : {self.total_evaluations}")
        print(f" All-Time Best Score : {self.best_fitness:.2f}")
        if self.best_metrics:
            print(f" Best Distance (20s) : {self.best_metrics.get('distance_m', 0.0):.2f} m")
            print(f" Best Average Speed  : {self.best_metrics.get('mean_speed_mps', 0.0):.2f} m/s")
            print(f" Best Lap Time       : {self.best_metrics.get('best_lap_time_s', 999.0):.2f} s")
            print(f" Collision Count     : {self.best_metrics.get('collision_count', 0)}")
        print(f" Champion Saved To   : {WINNING_GENOME_FILE}")
        print(f" Telemetry History   : {HISTORY_CSV_FILE}")
        print("================================================================================\n", flush=True)


def parse_args():
    parser = argparse.ArgumentParser(description="High-Throughput Parallel MPPI GA Optimizer")
    parser.add_argument("--workers", type=int, default=10, help="Number of parallel Webots instances (default: 10)")
    parser.add_argument("--pop-size", type=int, default=24, help="Population size per generation (default: 24)")
    parser.add_argument("--algo", "--algorithm", dest="algo", type=str, default="cmaes", choices=["cmaes", "de"], help="Optimizer algorithm (default: cmaes)")
    parser.add_argument("--duration-hours", type=float, default=9.0, help="Total optimization run time in hours (default: 9.0)")
    parser.add_argument("--duration-sim", type=float, default=20.0, help="Simulation evaluation seconds per trial (default: 20.0)")
    parser.add_argument("--max-generations", type=int, default=None, help="Optional max generations cutoff")
    parser.add_argument("--resume", action="store_true", help="Resume from ga_checkpoint.json if present")
    parser.add_argument("--test-single", action="store_true", help="Run 1 single trial smoke test and exit")
    parser.add_argument("--gui", action="store_true", help="Launch live per-generation GUI telemetry window")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    if args.test_single:
        print("[Smoke Test] Evaluating default baseline genome on Worker 0 for 5.0 sim-seconds...")
        fit, met = evaluate_genome(MPPIGenome(), worker_id=0, sim_duration_s=5.0, verbose=True)
        print(f"\nSmoke Test Completed! Fitness: {fit:.2f}, Distance: {met.get('distance_m', 0.0):.2f}m, Status: {met.get('status')}")
        sys.exit(0)

    if args.gui:
        gui_script = os.path.join(WORKSPACE_ROOT, "ga_live_gui.py")
        print(f"[Optimizer] Spawning Live Generation Telemetry GUI: {gui_script}...", flush=True)
        subprocess.Popen([sys.executable, gui_script])

    opt = EvolutionaryOptimizer(
        algorithm=args.algo,
        num_workers=args.workers,
        population_size=args.pop_size,
        sim_duration_s=args.duration_sim,
        max_duration_hours=args.duration_hours,
        resume=args.resume,
    )

    def handle_sigint(signum, frame):
        print("\n[Signal] Interrupt received. Finishing in-flight evaluations and saving checkpoint...", flush=True)
        opt.shutdown_requested = True

    signal.signal(signal.SIGINT, handle_sigint)

    opt.run(max_generations=args.max_generations)
