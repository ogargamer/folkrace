"""
================================================================================
  HEADLESS GENETIC ALGORITHM (GA) CONFIGURATION & GENE SPECIFICATIONS
================================================================================
  Central control file for tuning the C++ Curvilinear SL-MPC planner in Webots.
  Exposes all GA parameters, CPU worker settings, fitness parameters,
  and individual gene tables for the C++ SL-MPC architecture.
================================================================================
"""

import os
import sys
import math
import json
from typing import Dict, List, Tuple, Any, Optional
import numpy as np

# ==============================================================================
# 1. GA OPTIMIZER & WORKER POOL HYPERPARAMETERS
# ==============================================================================

# Number of parallel headless Webots workers / CPU cores to utilize (8 physical / 16 SMT threads -> 10 optimal)
NUM_CPU_CORES: int = 10

# Population size per generation (40 candidates: 4 full 10-worker batches)
POPULATION_SIZE: int = 40

# Maximum number of generations to run
NUM_GENERATIONS: int = 9999

# Base evaluation duration per candidate in simulation seconds (starting at 25.0s)
EVALUATION_DURATION_S: float = 25.0

# Automatic Linear Evaluation Duration Scaling (+2.5s per wall-clock hour = +5.0s per 2 hours continuously)
AUTO_SCALE_EVAL_DURATION: bool = True
EVAL_DURATION_HOURLY_GROWTH_S: float = 2.50  # Linear seconds added per wall-clock hour (+5s every 2h)
EVAL_DURATION_MAX_S: float = 60.0            # Maximum simulation duration ceiling

# Number of evaluation runs per candidate to average (1 for deterministic, >1 if noisy)
RUNS_PER_EVALUATION: int = 1

# Optimization Algorithm: 'CMA_ES' (Covariance Matrix Adaptation), 'GA' (Genetic Algorithm)
ALGORITHM: str = "CMA_ES"

# Seed initial search distribution/population from best_planner_genome.json instead of defaults
SEED_FROM_BEST_GENOME_DEFAULT: bool = True
BEST_GENOME_PATH: str = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best_planner_genome.json")

# CMA-ES Hyperparameters (Wide exploration sigma = 0.30 across normalized [0, 1] parameter space)
CMA_SIGMA0: float = 0.25             # Initial search step size in normalized [0, 1] parameter space
CMA_RESTART_ON_CONVERGENCE: bool = True  # Automatically re-seed if search distribution collapses

# Genetic Operator Rates (Used when ALGORITHM == 'GA')
CROSSOVER_RATE: float = 0.85
MUTATION_RATE: float = 0.25
MUTATION_SCALE: float = 0.22     # Gaussian mutation sigma as a fraction of gene span
ELITISM_COUNT: int = 4           # Top candidates guaranteed to survive untouched
TOURNAMENT_SIZE: int = 3         # Selection pressure for GA tournament selection

# Active Path Planner to Optimize (Exclusively C++ Curvilinear S-L MPC)
ACTIVE_PLANNER: str = "OPTION_2_SL_MPC"

# Headless Webots Simulation Settings
WEBOTS_BINARY: str = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
BASE_WORKER_PORT: int = 10200
TIMEOUT_PER_TRIAL_WALL_S: float = 200.0

# Moving Obstacle Parameters (Track 1 = 1.00 m/s forward, Track 2 = 1.00 m/s reverse/opposite)
ENABLE_MOVING_OBSTACLES: bool = True
USE_TRACK2: bool = True  # Whether to evaluate candidate on 2nd track (-1.00 m/s opposite direction) alongside Track 1 (1.00 m/s)
MOVING_OBSTACLE_SPEED_MPS: float = 1.00
TRACK1_MOVING_OBSTACLE_SPEED_MPS: float = 1.00
TRACK2_MOVING_OBSTACLE_SPEED_MPS: float = -1.00
MOVING_OBSTACLE_NAMES: List[str] = [
    "MOVING_OBSTACLE",
    "MOVING_OBSTACLE_2",
    "MOVING_OBSTACLE_3",
]

# Track Origin Start Pose:
ROBOT_START_POS: List[float] = [0.0, 0.0, 0.0275]
ROBOT_START_ROT: List[float] = [0.0, 0.0, 1.0, -1.5707953071795862]


# ==============================================================================
# 2. FITNESS & DISQUALIFICATION REWARD WEIGHTS
# ==============================================================================

FITNESS_PROGRESS_WEIGHT: float = 150.0
FITNESS_SPEED_WEIGHT: float = 30.0
FITNESS_LAP_BONUS: float = 150.0
FITNESS_COLLISION_PENALTY: float = 450.0
FITNESS_STALL_PENALTY: float = 900.0

STALL_SPEED_THRESHOLD_MPS: float = 0.15
STALL_MAX_DURATION_S: float = 1.50

# Duration Normalization & Survival Bonus Hyperparameters
FITNESS_BASE_DURATION_S: float = 25.0        # Reference baseline duration (s) for standard ~10k fitness scaling
FITNESS_DURATION_BONUS_MAX: float = 0.10     # Max bonus fraction (+10%) for running longer sim without crashing



# ==============================================================================
# 3. GENE SPECIFICATIONS FOR C++ CURVILINEAR SL-MPC
# ==============================================================================
# Format: (gene_name, min_val, max_val, type, default_val, description)

OPTION_2_SL_MPC_GENE_SPECS: List[Tuple[str, float, float, type, float, str]] = [
    # Speeds & Limits (Real-world physical wheel ceiling = 3.84 m/s from vehicle_config.json)
    ("v_cruise_max",         3.84,   3.84,   float,  3.84,   "Max straight clearing speed (m/s, real-world physical limit)"),
    ("v_cruise_min",         1.223,   1.223,   float,  1.223,  "Min corner crawl speed (m/s)"),
    # Horizon & Discretization (Fixed at N=30, dt=1/60s to match native C++ OSQP codegen & 60 Hz Pi/ToF loop)
    ("horizon_N",           30.0,   30.0,    int,   30.0,    "MPC prediction horizon steps (fixed at 30 in native QP tables)"),
    ("dt",                   0.016667, 0.016667, float, 0.016667, "MPC prediction discretization timestep (s)"),
    # Tire Friction & Costmap Inflation (Real-world MU = 0.80, real-world inflation = 0.090 m)
    ("mu_friction",          0.80,   0.80,   float,  0.80,   "Tire friction coefficient governing steer vs brake loads"),
    ("r_inflation_m",        0.090,  0.090,  float,  0.090,  "ToF obstacle expansion / inflation radius (m, real-world costmap)"),
    # MPC Objective Weights (Defaulted to best_planner_genome.json)
    ("q_progress",           0.50,  40.00,   float, 25.143,  "Longitudinal progress reward weight"),
    ("q_cte",                1.00,  60.00,   float, 59.998,  "Cross-track error penalty weight"),
    ("q_heading",            2.00,  25.00,   float,  2.788,  "Heading error penalty weight"),
    ("r_steer",              1.00,  15.00,   float,  3.631,  "Steering angle effort penalty weight"),
    ("r_accel",              0.01,   6.00,   float,  1.722,  "Acceleration/braking effort penalty weight"),
    ("r_steer_rate",         0.50,  35.00,   float,  2.020,  "Steering rate slew penalty weight"),
    ("v_corner_factor",      0.60,   0.98,   float,  0.677,  "Friction circle speed scaling for curvature"),
    ("center_bias",          0.00,   1.00,   float,  0.000,  "Corridor centering blend (0=guide line, 1=corridor midpoint)"),
    # Gap Scout Parameters (Defaulted to best_planner_genome.json)
    ("target_lookahead_m",   1.72,   1.72,   float,  1.72,   "Scout lookahead target projection distance (m)"),
    ("w_gap_distance",       1.00,  10.00,   float,  8.198,  "Gap scout distance reward weight"),
    ("w_gap_width",          0.20,   4.00,   float,  0.542,  "Gap scout aperture width reward weight"),
    ("w_gap_steer",          0.20,   4.00,   float,  1.805,  "Gap scout steering alignment penalty weight"),
    ("w_gap_straight",       0.05,   2.00,   float,  0.589,  "Gap scout straight heading alignment reward weight"),
    ("w_side_clearance",     0.00,   6.00,   float,  1.913,  "Gap scout side asymmetry clearance weight"),
    ("gap_dist_exp_k",       0.20,   1.50,   float,  0.837,  "Gap scout exponential distance reward factor"),
    ("gap_dist_thresh_m",    0.80,   1.30,   float,  1.292,  "Gap scout distance threshold (m) to engage exponential reward"),
    ("scout_obs_width_m",    0.05,   0.45,   float,  0.176,  "Effective obstacle width (m) for angular shadow projection"),
    ("scout_straight_pen_k", 0.50,   5.00,   float,  0.637,  "Base straight obstacle penalty scaling factor"),
    ("scout_straight_exp_k", 0.10,   3.00,   float,  1.922,  "Exponential curvature factor peaking at dead center (0 deg)"),
    ("gap_latch_hysteresis", 0.20,   3.50,   float,  2.345,  "Gap scout corridor latch score margin"),
    ("gap_latch_dist_m",     0.12,   0.60,   float,  0.253,  "Gap scout corridor latch spatial association radius (m)"),
    # Dual-Path Line Choice & Challenger Selection (Primary vs Alternative)
    ("alt_min_separation_m", 0.05,   0.40,   float,  0.116,  "Min lateral separation (m) between primary and challenger lines"),
    ("alt_aperture_bias",    1.00,   4.00,   float,  3.309,  "Challenger aperture width bias multiplier"),
    ("alt_switch_hysteresis",1.00,  50.00,   float, 18.330,  "MPC cost reduction threshold needed to switch lines"),
    ("w_gap_clear",          0.00,  50.00,   float, 13.997,  "Lookahead projection depth credit and clear gap weight in dual-line arbitration"),
    # Straight Quieting & Anti-Oscillation Adaptive Steering Response
    ("aggr_dist_far_m",      1.72,   1.72,   float,  1.72,   "Forward clearance distance (m) to engage max straight quieting"),
    ("aggr_dist_close_m",    0.80,   1.30,   float,  1.042,  "Emergency distance threshold (m) below which 100% steering authority is engaged"),
    ("aggr_min_scale",       0.70,   1.00,   float,  0.719,  "Minimal steering authority scale on open straights"),
    ("steer_deadband_rad",   0.000,  0.005,  float,  0.002,  "Steering error deadband (rad) for sensor noise quieting"),
    # Stuck & Emergency Recovery
    ("recovery_clearance_exit_m", 0.25, 0.25, float, 0.25,   "Front clearance threshold (m) to conclude reverse and center steering"),
    ("recovery_steer_angle_rad",0.5623, 0.5623, float, 0.5623, "Recovery pivot steering angle (rad)"),
    ("recovery_arc_lookahead_m",0.10, 0.15,  float,  0.139,  "Kinematic escape arc lookahead length (m) on costmap"),
]


# ==============================================================================
# 4. CHROMOSOME CODEC & REPAIR ENGINE
# ==============================================================================

class BasePlannerGenome:
    """Generic genome wrapper providing serialization, clamping, and repair."""

    def __init__(self, specs: List[Tuple[str, float, float, type, float, str]], values: Optional[Dict[str, Any]] = None):
        self.specs = specs
        self.values: Dict[str, Any] = {}
        for name, min_v, max_v, vtype, def_v, _ in specs:
            if values and name in values:
                self.values[name] = vtype(values[name])
            elif values and name == "w_gap_clear" and "alt_lookahead_depth_weight" in values:
                self.values[name] = vtype(values["alt_lookahead_depth_weight"])
            else:
                self.values[name] = vtype(def_v)
        self.clamp_and_repair()

    def to_dict(self) -> Dict[str, Any]:
        return dict(self.values)

    def to_array(self) -> np.ndarray:
        arr = []
        for name, _, _, _, _, _ in self.specs:
            arr.append(float(self.values[name]))
        return np.array(arr, dtype=np.float64)

    def from_array(self, arr: np.ndarray):
        for idx, (name, min_v, max_v, vtype, _, _) in enumerate(self.specs):
            if idx < len(arr):
                val = arr[idx]
                if vtype is int:
                    val = int(round(val))
                self.values[name] = vtype(val)
        self.clamp_and_repair()

    def clamp_and_repair(self):
        for name, min_v, max_v, vtype, _, _ in self.specs:
            val = self.values[name]
            val = max(min_v, min(max_v, val))
            if vtype is int:
                val = int(round(val))
            elif vtype is float:
                val = round(float(val), 3)
            self.values[name] = vtype(val)

    def get_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        lower = np.array([s[1] for s in self.specs], dtype=np.float64)
        upper = np.array([s[2] for s in self.specs], dtype=np.float64)
        return lower, upper


class Option2SLMPCGenome(BasePlannerGenome):
    def __init__(self, values: Optional[Dict[str, Any]] = None):
        super().__init__(OPTION_2_SL_MPC_GENE_SPECS, values)


def get_genome_class(planner_name: Optional[str] = None):
    """Factory returning genome class (exclusively C++ SL-MPC)."""
    return Option2SLMPCGenome


def load_best_planner_genome(
    filepath: Optional[str] = None,
    genome_cls: Optional[Any] = None,
) -> Tuple[Optional[BasePlannerGenome], float, Dict[str, Any]]:
    """
    Loads chromosome and metrics from best_planner_genome.json (or specified filepath).
    Returns (genome_instance, fitness, metrics_dict).
    """
    path = filepath or BEST_GENOME_PATH
    if not os.path.exists(path):
        return None, -1e9, {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        genes = data.get("genes") or data.get("best_genome") or data
        fitness = float(data.get("fitness", data.get("best_fitness", -1e9)))
        metrics = data.get("metrics", data.get("best_metrics", {}))
        cls = genome_cls or get_genome_class()
        genome = cls(genes)
        return genome, fitness, metrics
    except Exception as e:
        print(f"[Genome Loader] Warning: Failed to load genome from {path}: {e}", flush=True)
        return None, -1e9, {}


def generate_logical_initial_population(
    genome_cls,
    pop_size: int,
    seed_genome: Optional[Any] = None,
) -> List[BasePlannerGenome]:
    """
    Generates a structured, physically logical initial population (Gen 0)
    combining distinct driving archetypes with stratified Latin-hypercube sampling.
    If seed_genome is provided, it is injected as Archetype 0 and perturbations center on it.
    """
    sample_inst = genome_cls()
    specs = sample_inst.specs
    lower, upper = sample_inst.get_bounds()
    span = upper - lower

    archetypes = [
        # 0. Cyan Champion Calibrated Baseline
        {
            "v_cruise_max": 5.50, "v_cruise_min": 1.223,
            "horizon_N": 30, "dt": 0.016667,
            "mu_friction": 0.85, "r_inflation_m": 0.075,
            "q_progress": 15.0, "q_cte": 18.0, "q_heading": 8.0,
            "r_steer": 5.0, "r_accel": 2.0, "r_steer_rate": 10.0,
            "v_corner_factor": 0.95,
            "center_bias": 0.0,
            "target_lookahead_m": 1.70, "w_gap_distance": 3.00, "w_gap_width": 1.50,
            "w_gap_steer": 1.00, "w_gap_straight": 0.50,
            "w_side_clearance": 2.80,
            "gap_dist_exp_k": 0.60, "gap_dist_thresh_m": 0.95,
            "gap_latch_hysteresis": 1.25, "gap_latch_dist_m": 0.28,
            "alt_min_separation_m": 0.15, "alt_aperture_bias": 1.80, "alt_switch_hysteresis": 8.00,
            "w_gap_clear": 5.00,
            "aggr_dist_far_m": 1.70, "aggr_dist_close_m": 1.00, "aggr_min_scale": 0.30,
            "steer_deadband_rad": 0.002,
            "recovery_clearance_exit_m": 0.28, "recovery_steer_angle_rad": 0.5623,
            "recovery_arc_lookahead_m": 0.13,
        },
        # 1. High-Momentum Apex Striker
        {
            "v_cruise_max": 5.50, "v_cruise_min": 1.50,
            "horizon_N": 30, "dt": 0.016667,
            "mu_friction": 0.88, "r_inflation_m": 0.075,
            "q_progress": 18.0, "q_cte": 14.0, "q_heading": 6.0,
            "r_steer": 3.5, "r_accel": 1.5, "r_steer_rate": 8.0,
            "v_corner_factor": 0.95,
            "center_bias": 0.0,
            "target_lookahead_m": 1.60, "w_gap_distance": 4.00, "w_gap_width": 1.80,
            "w_gap_steer": 0.80, "w_gap_straight": 0.50,
            "w_side_clearance": 2.50,
            "gap_dist_exp_k": 0.70, "gap_dist_thresh_m": 0.95,
            "gap_latch_hysteresis": 1.10, "gap_latch_dist_m": 0.25,
            "alt_min_separation_m": 0.15, "alt_aperture_bias": 1.80, "alt_switch_hysteresis": 7.00,
            "w_gap_clear": 6.00,
            "aggr_dist_far_m": 1.68, "aggr_dist_close_m": 1.10, "aggr_min_scale": 0.35,
            "steer_deadband_rad": 0.002,
            "recovery_clearance_exit_m": 0.30, "recovery_steer_angle_rad": 0.5623,
            "recovery_arc_lookahead_m": 0.12,
        },
        # 2. Safe High-Clearance Sentinel
        {
            "v_cruise_max": 4.80, "v_cruise_min": 1.15,
            "horizon_N": 30, "dt": 0.016667,
            "mu_friction": 0.82, "r_inflation_m": 0.080,
            "q_progress": 10.0, "q_cte": 22.0, "q_heading": 10.0,
            "r_steer": 6.0, "r_accel": 2.5, "r_steer_rate": 12.0,
            "v_corner_factor": 0.90,
            "center_bias": 0.40,
            "target_lookahead_m": 1.50, "w_gap_distance": 3.00, "w_gap_width": 2.20,
            "w_gap_steer": 1.10, "w_gap_straight": 0.40,
            "w_side_clearance": 3.20,
            "gap_dist_exp_k": 0.50, "gap_dist_thresh_m": 0.98,
            "gap_latch_hysteresis": 1.50, "gap_latch_dist_m": 0.32,
            "alt_min_separation_m": 0.20, "alt_aperture_bias": 2.00, "alt_switch_hysteresis": 12.00,
            "w_gap_clear": 4.00,
            "aggr_dist_far_m": 1.75, "aggr_dist_close_m": 1.20, "aggr_min_scale": 0.40,
            "steer_deadband_rad": 0.003,
            "recovery_clearance_exit_m": 0.25, "recovery_steer_angle_rad": 0.550,
            "recovery_arc_lookahead_m": 0.15,
        },
    ]

    # Inject seed genome as primary archetype if provided
    if seed_genome is not None:
        if isinstance(seed_genome, BasePlannerGenome):
            archetypes[0] = seed_genome.to_dict()
        elif isinstance(seed_genome, dict):
            genes_dict = seed_genome.get("genes") or seed_genome.get("best_genome") or seed_genome
            arch = dict(genes_dict)
            if "alt_lookahead_depth_weight" in arch and "w_gap_clear" not in arch:
                arch["w_gap_clear"] = arch["alt_lookahead_depth_weight"]
            archetypes[0] = arch

    population: List[BasePlannerGenome] = []
    for arch in archetypes:
        if len(population) < pop_size:
            population.append(genome_cls(arch))

    n_remain = pop_size - len(population)
    if n_remain > 0:
        base_arr = genome_cls(archetypes[0]).to_array()
        for k in range(n_remain):
            grid_frac = (k + 0.5) / n_remain
            noise = (np.random.uniform(-0.35, 0.35, size=base_arr.shape) + (grid_frac - 0.5) * 0.50) * span
            cand_arr = np.clip(base_arr + noise, lower, upper)
            cand = genome_cls()
            cand.from_array(cand_arr)
            if "v_cruise_min" in cand.values and "v_cruise_max" in cand.values:
                cand.values["v_cruise_min"] = min(cand.values["v_cruise_min"], cand.values["v_cruise_max"] - 0.30)
            cand.clamp_and_repair()
            population.append(cand)

    return population


def compute_efficiency_multiplier(gene_dict: Optional[Dict[str, Any]]) -> float:
    """SL-MPC uses constant-time native C++ OSQP solves; multiplier is normalized."""
    return 1.0


SPEED_PHYSICAL_MAX_MPS: float = 3.84
DISTANCE_VALID_MIN_M: float = 0.10


def compute_scalar_fitness(
    metrics: Dict[str, Any],
    gene_dict: Optional[Dict[str, Any]] = None,
    eval_duration_s: Optional[float] = None,
) -> float:
    """Computes duration-normalized scalar optimization fitness from Webots trial metrics.

    Normalizes distance and lap metrics relative to FITNESS_BASE_DURATION_S (25.0s) so that
    longer simulation runs (e.g. 50s-60s) do not inflate the fitness score. When a candidate
    survives longer runs without crashing or stalling, a modest bonus (up to +10% at
    EVAL_DURATION_MAX_S) is applied to reward extended collision-free survival.
    """
    dist_m = float(metrics.get("distance_m", 0.0))
    mean_speed = float(metrics.get("mean_speed_mps", 0.0))
    laps = int(metrics.get("laps_completed", 0))
    crashed = bool(metrics.get("crashed", False))
    stuck = bool(metrics.get("stalled", False))

    if math.isnan(dist_m) or dist_m <= DISTANCE_VALID_MIN_M:
        return -5000.0

    if math.isnan(mean_speed) or mean_speed < 0.0:
        mean_speed = 0.0
    elif mean_speed > SPEED_PHYSICAL_MAX_MPS:
        mean_speed = SPEED_PHYSICAL_MAX_MPS

    # Determine planned trial evaluation duration for normalization
    trial_dur = eval_duration_s
    if trial_dur is None:
        trial_dur = metrics.get("eval_duration_s") or metrics.get("target_duration_s")
    if trial_dur is None:
        # Fallback to duration_s; if crashed/stuck early, guard against small sim_time inflating distance
        raw_dur = float(metrics.get("duration_s", FITNESS_BASE_DURATION_S))
        trial_dur = max(FITNESS_BASE_DURATION_S, raw_dur) if (crashed or stuck) else raw_dur
    trial_dur = max(1.0, float(trial_dur))

    # Duration normalization: scale distance and laps to baseline 25.0s equivalent
    norm_scale = FITNESS_BASE_DURATION_S / trial_dur
    effective_dist_m = dist_m * norm_scale
    effective_laps = laps * norm_scale

    base_score = (
        (effective_dist_m * FITNESS_PROGRESS_WEIGHT)
        + (mean_speed * FITNESS_SPEED_WEIGHT)
        + (effective_laps * FITNESS_LAP_BONUS)
    )

    if crashed:
        return float(base_score - FITNESS_COLLISION_PENALTY)
    elif stuck:
        return float(base_score - FITNESS_STALL_PENALTY)
    else:
        # Survival bonus: up to +10% for completing longer evaluations without crashing
        dur_excess = max(0.0, trial_dur - FITNESS_BASE_DURATION_S)
        dur_span = max(1.0, EVAL_DURATION_MAX_S - FITNESS_BASE_DURATION_S)
        bonus_factor = min(1.0, dur_excess / dur_span)
        duration_bonus_mult = 1.0 + (FITNESS_DURATION_BONUS_MAX * bonus_factor)
        return float(base_score * duration_bonus_mult)

