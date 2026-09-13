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
from typing import Dict, List, Tuple, Any, Optional
import numpy as np

# ==============================================================================
# 1. GA OPTIMIZER & WORKER POOL HYPERPARAMETERS
# ==============================================================================

# Number of parallel headless Webots workers / CPU cores to utilize
NUM_CPU_CORES: int = 14

# Population size per generation
POPULATION_SIZE: int = 42

# Maximum number of generations to run
NUM_GENERATIONS: int = 9999

# Evaluation duration per candidate in simulation seconds (strictly 20s)
EVALUATION_DURATION_S: float = 20.0

# Number of evaluation runs per candidate to average (1 for deterministic, >1 if noisy)
RUNS_PER_EVALUATION: int = 1

# Optimization Algorithm: 'CMA_ES' (Covariance Matrix Adaptation), 'GA' (Genetic Algorithm)
ALGORITHM: str = "CMA_ES"

# CMA-ES Hyperparameters
CMA_SIGMA0: float = 0.20             # Initial search step size in normalized [0, 1] parameter space
CMA_RESTART_ON_CONVERGENCE: bool = True  # Automatically re-seed if search distribution collapses

# Genetic Operator Rates (Used when ALGORITHM == 'GA')
CROSSOVER_RATE: float = 0.80
MUTATION_RATE: float = 0.18
MUTATION_SCALE: float = 0.12     # Gaussian mutation sigma as a fraction of gene span
ELITISM_COUNT: int = 2           # Top candidates guaranteed to survive untouched
TOURNAMENT_SIZE: int = 3         # Selection pressure for GA tournament selection

# Active Path Planner to Optimize (Exclusively C++ Curvilinear S-L MPC)
ACTIVE_PLANNER: str = "OPTION_2_SL_MPC"

# Headless Webots Simulation Settings
WEBOTS_BINARY: str = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
BASE_WORKER_PORT: int = 10200
TIMEOUT_PER_TRIAL_WALL_S: float = 90.0

# Moving Obstacle Parameters (moves AGAINST robot heading at 0.50 m/s along racing line)
ENABLE_MOVING_OBSTACLES: bool = True
MOVING_OBSTACLE_SPEED_MPS: float = 0.50
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
FITNESS_COLLISION_PENALTY: float = 300.0
FITNESS_STALL_PENALTY: float = 3000.0

STALL_SPEED_THRESHOLD_MPS: float = 0.15
STALL_MAX_DURATION_S: float = 1.50
COLLISION_DISTANCE_MIN_M: float = 0.020


# ==============================================================================
# 3. GENE SPECIFICATIONS FOR C++ CURVILINEAR SL-MPC
# ==============================================================================
# Format: (gene_name, min_val, max_val, type, default_val, description)

OPTION_2_SL_MPC_GENE_SPECS: List[Tuple[str, float, float, type, float, str]] = [
    # Speeds & Limits
    ("v_cruise_max",         1.80,   5.50,   float,  5.50,   "Max straight clearing speed (m/s)"),
    ("v_cruise_min",         1.20,   2.50,   float,  1.20,   "Min corner crawl speed (m/s)"),
    # Horizon & Discretization (Fixed at N=30, dt=1/60s to match native C++ OSQP codegen)
    ("horizon_N",           30.0,   30.0,    int,   30.0,    "MPC prediction horizon steps (fixed at 30 in native QP tables)"),
    ("dt",                   0.016667, 0.016667, float, 0.016667, "MPC prediction discretization timestep (s)"),
    # Tire Friction & Costmap Inflation
    ("mu_friction",          0.70,   0.90,   float,  0.85,   "Tire friction coefficient governing steer vs brake loads"),
    ("r_inflation_m",        0.075,  0.075,  float,  0.075,  "ToF obstacle expansion / inflation radius (0.075m - 0.120m)"),
    # MPC Objective Weights
    ("q_progress",           2.00,  25.00,   float,  8.00,   "Longitudinal progress reward weight"),
    ("q_cte",                1.00,  45.00,   float, 18.00,   "Cross-track error penalty weight"),
    ("q_heading",            2.00,  20.00,   float,  8.00,   "Heading error penalty weight"),
    ("r_steer",              1.00,  15.00,   float,  5.00,   "Steering angle effort penalty weight"),
    ("r_accel",              0.50,   8.00,   float,  2.00,   "Acceleration/braking effort penalty weight"),
    ("r_steer_rate",         2.00,  30.00,   float, 10.00,   "Steering rate slew penalty weight"),
    ("v_corner_factor",      0.60,   0.98,   float,  0.95,   "Friction circle speed scaling for curvature"),
    ("pp_blend_gain",        0.00,   0.65,   float,  0.35,   "Pure pursuit lookahead blend ratio with MPC"),
    ("lookahead_blend_m",    0.25,   0.75,   float,  0.45,   "Pure pursuit blend lookahead distance (m)"),
    # Gap Scout Parameters
    ("target_lookahead_m",   0.80,   1.80,   float,  1.70,   "Scout lookahead target projection distance (m)"),
    ("w_gap_distance",       1.00,   8.00,   float,  3.00,   "Gap scout distance reward weight"),
    ("w_gap_width",          0.50,   5.00,   float,  1.50,   "Gap scout aperture width reward weight"),
    ("w_gap_steer",          0.20,   3.00,   float,  1.00,   "Gap scout steering alignment penalty weight"),
    ("w_gap_straight",       0.10,   2.00,   float,  0.50,   "Gap scout straight heading alignment reward weight"),
    ("w_side_clearance",     0.00,   6.00,   float,  2.80,   "Gap scout side asymmetry clearance weight"),
    ("gap_dist_exp_k",       0.20,   1.50,   float,  0.60,   "Gap scout exponential distance reward factor"),
    ("gap_progress_margin_m",0.05,   0.35,   float,  0.15,   "Gap scout corridor margin over 1.0m track width (m)"),
    # Dual-Path Line Choice & Challenger Selection (Primary vs Alternative)
    ("alt_min_separation_m", 0.10,   0.50,   float,  0.15,   "Min lateral separation (m) between primary and challenger lines"),
    ("alt_aperture_bias",    0.50,   4.00,   float,  1.80,   "Challenger aperture width bias multiplier"),
    ("alt_switch_hysteresis",2.00,  80.00,   float, 30.00,   "MPC cost reduction threshold needed to switch lines"),
    # Fast Straight-Line Lock
    ("straight_lock_dist",   1.70,   1.70,   float,  1.70,   "Forward clearance (m) to lock straight line acceleration"),
    ("straight_lock_speed",  5.50,   5.50,   float,  5.50,   "Speed ceiling (m/s) when straight-line locked"),
    # Straight Quieting & Anti-Oscillation Adaptive Steering Response
    ("aggr_dist_far_m",      1.40,   1.80,   float,  1.70,   "Forward clearance distance (m) to engage max straight quieting"),
    ("aggr_dist_close_m",    0.60,   1.60,   float,  1.00,   "Emergency distance threshold (m) below which 100% steering authority is engaged"),
    ("aggr_min_scale",       0.20,   0.95,   float,  0.30,   "Minimal steering authority scale on open straights"),
    ("steer_deadband_rad",   0.000,  0.005,  float,  0.002,  "Steering error deadband (rad) for sensor noise quieting"),
    # Stuck & Emergency Recovery
    ("emergency_backup_dist_m", 0.08, 0.12,  float,  0.09,   "Distance threshold (m) below which emergency reverse is triggered"),
    ("recovery_reverse_speed",  0.80, 2.00,  float,  1.30,   "Emergency reverse backing speed (m/s)"),
    ("recovery_clearance_exit_m", 0.18, 0.45, float, 0.28,   "Front clearance threshold (m) to conclude reverse and center steering"),
    ("recovery_steer_angle_rad",0.30, 0.698, float,  0.600,  "Recovery pivot steering angle (rad)"),
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


def generate_logical_initial_population(genome_cls, pop_size: int) -> List[BasePlannerGenome]:
    """
    Generates a structured, physically logical initial population (Gen 0)
    combining distinct driving archetypes with stratified Latin-hypercube sampling.
    """
    sample_inst = genome_cls()
    specs = sample_inst.specs
    lower, upper = sample_inst.get_bounds()
    span = upper - lower

    archetypes = [
        # 0. Cyan Champion Calibrated Baseline
        {
            "v_cruise_max": 5.50, "v_cruise_min": 1.20,
            "horizon_N": 30, "dt": 0.016667,
            "mu_friction": 0.85, "r_inflation_m": 0.075,
            "q_progress": 8.0, "q_cte": 18.0, "q_heading": 8.0,
            "r_steer": 5.0, "r_accel": 2.0, "r_steer_rate": 10.0,
            "v_corner_factor": 0.95, "pp_blend_gain": 0.35, "lookahead_blend_m": 0.45,
            "target_lookahead_m": 1.70, "w_gap_distance": 3.00, "w_gap_width": 1.50,
            "w_gap_steer": 1.00, "w_gap_straight": 0.50,
            "w_side_clearance": 2.80,
            "gap_dist_exp_k": 0.60, "gap_progress_margin_m": 0.15,
            "alt_min_separation_m": 0.15, "alt_aperture_bias": 1.80, "alt_switch_hysteresis": 30.00,
            "straight_lock_dist": 1.70, "straight_lock_speed": 5.50,
            "aggr_dist_far_m": 1.70, "aggr_dist_close_m": 1.00, "aggr_min_scale": 0.30,
            "steer_deadband_rad": 0.002,
            "emergency_backup_dist_m": 0.09, "recovery_reverse_speed": 1.30,
            "recovery_clearance_exit_m": 0.28, "recovery_steer_angle_rad": 0.600,
        },
        # 1. High-Momentum Apex Striker
        {
            "v_cruise_max": 5.50, "v_cruise_min": 1.50,
            "horizon_N": 30, "dt": 0.016667,
            "mu_friction": 0.88, "r_inflation_m": 0.075,
            "q_progress": 12.0, "q_cte": 14.0, "q_heading": 6.0,
            "r_steer": 3.5, "r_accel": 1.5, "r_steer_rate": 8.0,
            "v_corner_factor": 0.95, "pp_blend_gain": 0.25, "lookahead_blend_m": 0.40,
            "target_lookahead_m": 1.60, "w_gap_distance": 4.00, "w_gap_width": 1.80,
            "w_gap_steer": 0.80, "w_gap_straight": 0.50,
            "w_side_clearance": 2.50,
            "gap_dist_exp_k": 0.70, "gap_progress_margin_m": 0.12,
            "alt_min_separation_m": 0.15, "alt_aperture_bias": 1.80, "alt_switch_hysteresis": 25.00,
            "straight_lock_dist": 1.65, "straight_lock_speed": 5.50,
            "aggr_dist_far_m": 1.68, "aggr_dist_close_m": 1.10, "aggr_min_scale": 0.35,
            "steer_deadband_rad": 0.002,
            "emergency_backup_dist_m": 0.08, "recovery_reverse_speed": 1.40,
            "recovery_clearance_exit_m": 0.30, "recovery_steer_angle_rad": 0.600,
        },
        # 2. Safe High-Clearance Sentinel
        {
            "v_cruise_max": 4.80, "v_cruise_min": 1.10,
            "horizon_N": 30, "dt": 0.016667,
            "mu_friction": 0.82, "r_inflation_m": 0.080,
            "q_progress": 6.0, "q_cte": 22.0, "q_heading": 10.0,
            "r_steer": 6.0, "r_accel": 2.5, "r_steer_rate": 12.0,
            "v_corner_factor": 0.90, "pp_blend_gain": 0.45, "lookahead_blend_m": 0.50,
            "target_lookahead_m": 1.50, "w_gap_distance": 3.00, "w_gap_width": 2.20,
            "w_gap_steer": 1.10, "w_gap_straight": 0.40,
            "w_side_clearance": 3.20,
            "gap_dist_exp_k": 0.50, "gap_progress_margin_m": 0.18,
            "alt_min_separation_m": 0.20, "alt_aperture_bias": 2.00, "alt_switch_hysteresis": 35.00,
            "straight_lock_dist": 1.75, "straight_lock_speed": 5.00,
            "aggr_dist_far_m": 1.75, "aggr_dist_close_m": 1.20, "aggr_min_scale": 0.40,
            "steer_deadband_rad": 0.003,
            "emergency_backup_dist_m": 0.10, "recovery_reverse_speed": 1.20,
            "recovery_clearance_exit_m": 0.25, "recovery_steer_angle_rad": 0.550,
        },
    ]

    population: List[BasePlannerGenome] = []
    for arch in archetypes:
        if len(population) < pop_size:
            population.append(genome_cls(arch))

    n_remain = pop_size - len(population)
    if n_remain > 0:
        base_arr = genome_cls(archetypes[0]).to_array()
        for k in range(n_remain):
            grid_frac = (k + 0.5) / n_remain
            noise = (np.random.uniform(-0.18, 0.18, size=base_arr.shape) + (grid_frac - 0.5) * 0.40) * span
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


SPEED_PHYSICAL_MAX_MPS: float = 5.50
DISTANCE_VALID_MIN_M: float = 0.10


def compute_scalar_fitness(metrics: Dict[str, Any], gene_dict: Optional[Dict[str, Any]] = None) -> float:
    """Computes scalar optimization fitness from Webots trial metrics."""
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

    base_score = (dist_m * FITNESS_PROGRESS_WEIGHT) + (mean_speed * FITNESS_SPEED_WEIGHT) + (laps * FITNESS_LAP_BONUS)

    if crashed:
        return float(base_score - 50.0)
    elif stuck:
        return float(base_score - FITNESS_STALL_PENALTY * 0.05)
    else:
        return float(base_score)
