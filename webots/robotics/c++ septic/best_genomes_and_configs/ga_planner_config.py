"""
================================================================================
  HEADLESS GENETIC ALGORITHM (GA) CONFIGURATION & GENE SPECIFICATIONS
================================================================================
  Central control file for tuning autonomous ToF path planners in Webots.
  Exposes all GA parameters, CPU worker settings, fitness parameters,
  and individual gene tables for each path planner option.
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
# (Default: 4 background workers for smooth multi-core throughput without high system load)
NUM_CPU_CORES: int = 12

# Population size per generation (e.g. 16, 24, 32)
POPULATION_SIZE: int = 36

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

# Active Path Planner to Optimize:
#   'ALL'             : Interleaved round-robin optimization for ALL 3 planners
#   'OPTION_1_BVP'    : Boundary Value Problem Dynamic State Lattice (Clothoid/Arc)
#   'OPTION_2_SL_MPC' : Curvilinear Frenet SL-MPC with active corner entry speed
#   'OPTION_3_SEPTIC' : 7th-Order Septic Spline with friction-ellipse speed governing
ACTIVE_PLANNER: str = "OPTION_3_SEPTIC"

# Headless Webots Simulation Settings
WEBOTS_BINARY: str = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
BASE_WORKER_PORT: int = 10200
TIMEOUT_PER_TRIAL_WALL_S: float = 90.0

# Moving Obstacle Parameters (moves AGAINST robot heading at 0.50 m/s along racing line)
ENABLE_MOVING_OBSTACLES: bool = True
MOVING_OBSTACLE_SPEED_MPS: float = 0.50  # Speed in m/s (negative arc progression)
MOVING_OBSTACLE_NAMES: List[str] = [
    "MOVING_OBSTACLE",
    "MOVING_OBSTACLE_2",
    "MOVING_OBSTACLE_3",
]

# Track Origin Start Pose (Matches Cyan_Champion_ToF_Track.wbt Folkrace default):
# translation 0 0 0.0275, rotation 0 0 1 -1.5707953071795862
ROBOT_START_POS: List[float] = [0.0, 0.0, 0.0275]
ROBOT_START_ROT: List[float] = [0.0, 0.0, 1.0, -1.5707953071795862]


# ==============================================================================
# 2. FITNESS & DISQUALIFICATION REWARD WEIGHTS
# ==============================================================================

# Reward per meter of forward track distance traveled within 20s
FITNESS_PROGRESS_WEIGHT: float = 150.0

# Reward per m/s average forward velocity maintained
FITNESS_SPEED_WEIGHT: float = 30.0

# Bonus reward per full lap completed (~14.1 meters per lap)
FITNESS_LAP_BONUS: float = 150.0

# Fatal collision penalty (immediate trial abort upon wall or obstacle contact)
FITNESS_COLLISION_PENALTY: float = 300.0

# Stall penalty (if vehicle velocity < 0.15 m/s for > 1.5 s)
FITNESS_STALL_PENALTY: float = 3000.0

# Early abort conditions
STALL_SPEED_THRESHOLD_MPS: float = 0.15
STALL_MAX_DURATION_S: float = 1.50
COLLISION_DISTANCE_MIN_M: float = 0.020  # Fatal obstacle proximity threshold (2.0 cm from ToF lens)


# ==============================================================================
# 3. GENE SPECIFICATIONS FOR EACH PATH PLANNER OPTION
# ==============================================================================
# Format: (gene_name, min_val, max_val, type, default_val, description)

# ------------------------------------------------------------------------------
# OPTION 1: BVP Dynamic State Lattice Genes
# ------------------------------------------------------------------------------
OPTION_1_BVP_GENE_SPECS: List[Tuple[str, float, float, type, float, str]] = [
    ("v_cruise_max",         1.80,   5.50,   float,  3.50,   "Max cruise speed along straight clearings (m/s)"),
    ("v_cruise_min",         0.70,   2.20,   float,  1.20,   "Min cornering entry crawl speed (m/s)"),
    ("mu_friction",          0.50,   0.90,   float,  0.70,   "Tire friction coefficient governing steer vs brake loads"),
    ("r_inflation_m",        0.075,  0.120,  float,  0.090,  "ToF obstacle expansion / inflation radius (0.075m - 0.120m)"),
    ("w_target_dist",        1.00,  15.00,   float,  6.00,   "Weight rewarding deep forward progress towards gap"),
    ("w_heading_align",      1.00,  12.00,   float,  4.50,   "Weight penalizing heading misalignment"),
    ("w_curvature",          0.50,   8.00,   float,  2.00,   "Weight penalizing extreme path curvature"),
    ("w_side_bias",          0.00,   5.00,   float,  1.00,   "Weight penalizing large lateral offsets"),
    ("w_smoothness",         0.50,   6.00,   float,  2.50,   "Weight penalizing steering delta from current"),
    ("slew_rate_limit_deg",  4.00,  18.00,   float, 10.00,   "Max steering angle change per 20ms step (deg)"),
    ("target_lookahead_m",   0.80,   1.80,   float,  1.35,   "Scout lookahead target projection distance (m)"),
    ("w_costmap_clearance",  1.00,  10.00,   float,  3.50,   "Weight rewarding distance from inflated obstacles"),
]

# ------------------------------------------------------------------------------
# OPTION 2: Curvilinear Frenet SL-MPC Genes
# ------------------------------------------------------------------------------
OPTION_2_SL_MPC_GENE_SPECS: List[Tuple[str, float, float, type, float, str]] = [
    # Speeds & Limits
    ("v_cruise_max",         1.80,   5.50,   float,  5.50,   "Max straight clearing speed (m/s)"),
    ("v_cruise_min",         1.20,   2.50,   float,  1.20,   "Min corner crawl speed (m/s)"),
    # Horizon & Discretization
    ("horizon_N",           10.0,   24.0,    int,   18.0,    "MPC prediction horizon steps"),
    ("dt",                   0.015,  0.060,  float,  0.020,  "MPC prediction discretization timestep (s)"),
    # Tire Friction & Costmap Inflation
    ("mu_friction",          0.7,   0.85,   float,  0.85,   "Tire friction coefficient governing steer vs brake loads"),
    ("r_inflation_m",        0.075,  0.075,  float,  0.075,  "ToF obstacle expansion / inflation radius (0.075m - 0.120m)"),
    # MPC Objective Weights
    ("q_progress",           2.00,  25.00,   float,  8.00,   "Longitudinal progress reward weight"),
    ("q_cte",                1.00,  45.00,   float, 18.00,   "Cross-track error penalty weight"),
    ("q_heading",            2.00,  20.00,   float,  8.00,   "Heading error penalty weight"),
    ("r_steer",              1.00,  15.00,   float,  5.00,   "Steering angle effort penalty weight"),
    ("r_accel",              0.50,   8.00,   float,  2.00,   "Acceleration/braking effort penalty weight"),
    ("r_steer_rate",         2.00,  30.00,   float, 10.00,   "Steering rate slew penalty weight"),
    ("v_corner_factor",      0.60,   0.95,   float,  0.82,   "Friction circle speed scaling for curvature"),
    ("pp_blend_gain",        0.00,   0.65,   float,  0.35,   "Pure pursuit lookahead blend ratio with MPC"),
    ("lookahead_blend_m",    0.25,   0.75,   float,  0.45,   "Pure pursuit blend lookahead distance (m)"),
    # Gap Scout Parameters
    ("target_lookahead_m",   0.80,   1.80,   float,  1.35,   "Scout lookahead target projection distance (m)"),
    ("w_gap_distance",       1.00,   8.00,   float,  3.50,   "Gap scout distance reward weight"),
    ("w_gap_width",          0.50,   5.00,   float,  2.00,   "Gap scout aperture width reward weight"),
    ("w_gap_steer",          0.20,   3.00,   float,  0.80,   "Gap scout steering alignment penalty weight"),
    ("w_gap_straight",       0.10,   2.00,   float,  0.40,   "Gap scout straight heading alignment reward weight"),
    ("w_side_clearance",     0.00,   6.00,   float,  2.80,   "Gap scout side asymmetry clearance weight (lower allows tighter apexing)"),
    ("gap_dist_exp_k",       0.20,   1.50,   float,  0.60,   "Gap scout exponential distance reward factor (d > 1m+margin)"),
    ("gap_progress_margin_m",0.05,   0.35,   float,  0.15,   "Gap scout corridor margin over 1.0m track width (m)"),
    # Dual-Path Line Choice & Challenger Selection (Primary vs Alternative)
    ("alt_min_separation_m", 0.15,   0.60,   float,  0.25,   "Min lateral separation (m) between primary and challenger lines"),
    ("alt_aperture_bias",    0.50,   4.00,   float,  1.80,   "Challenger aperture width bias multiplier (safety/opening priority)"),
    ("alt_switch_hysteresis",2.00,  80.00,   float, 15.00,   "MPC cost reduction threshold needed for challenger to overthrow primary line"),
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
    ("recovery_steer_angle_rad",0.30, 0.698, float,  0.454,  "Recovery pivot steering angle (rad)"),
]

# ------------------------------------------------------------------------------
# OPTION 3: 7th-Order Septic Spline Genes
# ------------------------------------------------------------------------------
OPTION_3_SEPTIC_GENE_SPECS: List[Tuple[str, float, float, type, float, str]] = [
    ("v_entry_max",          1.80,   5.50,   float,  5.50,   "Max straight clearing entry speed (m/s)"),
    ("v_cruise_min",         1.20,   2.50,   float,  1.50,   "Min corner apex crawl speed (m/s)"),
    ("mu_friction",          0.70,   0.90,   float,  0.85,   "Tire friction coefficient governing steer vs brake loads"),
    ("r_inflation_m",        0.075,  0.100,  float,  0.090,  "ToF obstacle expansion / inflation radius (0.075m - 0.120m)"),
    ("w_clearance",          0.50,  10.00,   float,  2.50,   "Weight rewarding obstacle clearance along spline"),
    ("w_chord_tangent",      0.50,   8.00,   float,  1.50,   "Weight aligning spline tangents to target chord"),
    ("kappa_smooth_gain",    0.05,   5.00,   float,  0.20,   "Curvature profile smoothness penalty weight"),
    ("pp_lookahead_gain",    0.15,   0.75,   float,  0.35,   "Pure pursuit lookahead fraction along spline"),
    ("slew_deg_max",         12.03,  12.03,  float, 12.03,   "Max steering slew per 20ms physics step (deg, 10.5 rad/s servo limit)"),
    ("steer_kp",             0.40,   1.80,   float,  0.75,   "Steering proportional tracking gain"),
    ("blend_w0",             0.02,   0.25,   float,  0.08,   "Initial heading blend weight for candidate 0"),
    ("blend_w1",             0.10,   0.45,   float,  0.20,   "Initial heading blend weight for candidate 1"),
    ("blend_w2",             0.25,   0.75,   float,  0.40,   "Initial heading blend weight for candidate 2"),
    ("target_lookahead_m",   1.10,   1.80,   float,  1.35,   "Scout lookahead target projection distance (m)"),
    # Scalable compute & candidate exploration parameters (GA decider)
    ("probe_samples",       20.0,   50.0,    int,   36.0,    "Compute: Arc-length point resolution per spline (20 - 50)"),
    ("num_lat_offsets",      3.0,    9.0,    int,    5.0,    "Compute: Number of lateral exploration lines across gap (3 - 9)"),
    ("lat_offset_max",       0.06,   0.20,   float,  0.16,   "Compute: Lateral offset search half-width in meters"),
    ("num_head_offsets",     1.0,    5.0,    int,    1.0,    "Compute: Number of exit heading exploration angles (1 - 5)"),
    ("head_offset_max",      0.00,   0.35,   float,  0.00,   "Compute: Exit heading exploration half-range in rad"),
    ("num_blend_weights",    2.0,    5.0,    int,    3.0,    "Compute: Number of turn-in heading blend candidates (2 - 5)"),
    ("max_extra_targets",    0.0,    3.0,    int,    2.0,    "Compute: Multi-target gap exploration depth (0 - 3)"),
    ("w_time",               0.50,   5.00,   float,  1.00,   "Cost: Traversal duration minimization weight"),
    ("w_steer_effort",       0.20,   4.00,   float,  0.80,   "Cost: Steering angle change / twitch penalty weight"),
    # Straight Quieting & Proximity-Adaptive Steering Response (Anti-Oscillation - Conservative Authority)
    ("aggr_dist_far_m",      1.40,   1.80,   float,  1.70,   "Forward clearance distance (m) to engage max straight quieting (min tof_fl_3, tof_fr_3)"),
    ("aggr_dist_close_m",    0.60,   1.60,   float,  1.0,   "Emergency distance threshold (m) below which 100% steering authority is engaged (conservative)"),
    ("aggr_min_scale",       0.2,   0.9,   float,  0.3,   "Minimal steering authority scale (0.75-1.00) on open straights (preserves turn authority)"),
    ("steer_deadband_rad",   0.000,  0.005,  float,  0.002,  "Steering error deadband (rad): sub-degree deadband for sensor noise (~0.11 deg)"),
    ("straight_lock_dist",   1.40,   1.725,   float,  1.70,   "Forward clearance to lock straight line acceleration"),
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


class Option1BVPGenome(BasePlannerGenome):
    def __init__(self, values: Optional[Dict[str, Any]] = None):
        super().__init__(OPTION_1_BVP_GENE_SPECS, values)


class Option2SLMPCGenome(BasePlannerGenome):
    def __init__(self, values: Optional[Dict[str, Any]] = None):
        super().__init__(OPTION_2_SL_MPC_GENE_SPECS, values)


class Option3SepticGenome(BasePlannerGenome):
    def __init__(self, values: Optional[Dict[str, Any]] = None):
        super().__init__(OPTION_3_SEPTIC_GENE_SPECS, values)


def get_genome_class(planner_name: str):
    """Factory returning genome instance for chosen planner."""
    p = planner_name.upper()
    if "OPTION_1" in p or "BVP" in p:
        return Option1BVPGenome
    elif "OPTION_2" in p or "MPC" in p:
        return Option2SLMPCGenome
    elif "OPTION_3" in p or "SEPTIC" in p:
        return Option3SepticGenome
    else:
        raise ValueError(f"Unknown planner name: {planner_name}")


def generate_logical_initial_population(genome_cls, pop_size: int) -> List[BasePlannerGenome]:
    """
    Generates a structured, physically logical initial population (Gen 0)
    combining distinct driving archetypes with stratified Latin-hypercube sampling.
    """
    sample_inst = genome_cls()
    specs = sample_inst.specs
    names = [s[0] for s in specs]
    lower, upper = sample_inst.get_bounds()
    span = upper - lower

    archetypes: List[Dict[str, float]] = []

    # Check which planner this genome represents
    if isinstance(sample_inst, Option1BVPGenome):
        archetypes = [
            # 1. Baseline Calibrated
            {"v_cruise_max": 2.80, "v_cruise_min": 1.20, "mu_friction": 0.70, "r_inflation_m": 0.090, "w_target_dist": 6.0, "w_heading_align": 4.5, "w_curvature": 2.0, "w_side_bias": 1.0, "w_smoothness": 2.5, "slew_rate_limit_deg": 10.0, "target_lookahead_m": 1.35, "w_costmap_clearance": 3.5},
            # 2. High-Speed Momentum
            {"v_cruise_max": 3.40, "v_cruise_min": 1.45, "mu_friction": 0.82, "r_inflation_m": 0.082, "w_target_dist": 9.5, "w_heading_align": 3.8, "w_curvature": 1.5, "w_side_bias": 0.7, "w_smoothness": 3.2, "slew_rate_limit_deg": 13.5, "target_lookahead_m": 1.50, "w_costmap_clearance": 2.8},
            # 3. Safe High-Clearance
            {"v_cruise_max": 2.40, "v_cruise_min": 1.00, "mu_friction": 0.65, "r_inflation_m": 0.110, "w_target_dist": 4.5, "w_heading_align": 6.0, "w_curvature": 3.2, "w_side_bias": 1.4, "w_smoothness": 2.0, "slew_rate_limit_deg": 8.0, "target_lookahead_m": 1.20, "w_costmap_clearance": 5.5},
            # 4. Deep Scout Flow
            {"v_cruise_max": 3.00, "v_cruise_min": 1.30, "mu_friction": 0.75, "r_inflation_m": 0.088, "w_target_dist": 8.0, "w_heading_align": 5.5, "w_curvature": 2.0, "w_side_bias": 0.5, "w_smoothness": 3.0, "slew_rate_limit_deg": 11.0, "target_lookahead_m": 1.70, "w_costmap_clearance": 4.0},
            # 5. Responsive Agile
            {"v_cruise_max": 2.90, "v_cruise_min": 1.25, "mu_friction": 0.78, "r_inflation_m": 0.085, "w_target_dist": 7.0, "w_heading_align": 3.6, "w_curvature": 1.8, "w_side_bias": 1.2, "w_smoothness": 1.8, "slew_rate_limit_deg": 15.0, "target_lookahead_m": 1.05, "w_costmap_clearance": 4.2},
            # 6. High Grip Carver
            {"v_cruise_max": 3.30, "v_cruise_min": 1.55, "mu_friction": 0.88, "r_inflation_m": 0.078, "w_target_dist": 9.0, "w_heading_align": 4.2, "w_curvature": 1.3, "w_side_bias": 0.8, "w_smoothness": 3.8, "slew_rate_limit_deg": 15.5, "target_lookahead_m": 1.40, "w_costmap_clearance": 3.0},
        ]
    elif isinstance(sample_inst, Option2SLMPCGenome):
        archetypes = [
            # 0. Cyan Champion Calibrated Baseline
            {
                "v_cruise_max": 5.50, "v_cruise_min": 1.20,
                "horizon_N": 18, "dt": 0.020,
                "mu_friction": 0.85, "r_inflation_m": 0.090,
                "q_progress": 8.0, "q_cte": 18.0, "q_heading": 8.0,
                "r_steer": 5.0, "r_accel": 2.0, "r_steer_rate": 10.0,
                "v_corner_factor": 0.82, "pp_blend_gain": 0.35, "lookahead_blend_m": 0.45,
                "target_lookahead_m": 1.35, "w_gap_distance": 3.50, "w_gap_width": 2.00,
                "w_gap_steer": 0.80, "w_gap_straight": 0.40,
                "gap_dist_exp_k": 0.60, "gap_progress_margin_m": 0.15,
                "straight_lock_dist": 1.70, "straight_lock_speed": 5.50,
                "aggr_dist_far_m": 1.70, "aggr_dist_close_m": 1.00, "aggr_min_scale": 0.30,
                "steer_deadband_rad": 0.002,
                "emergency_backup_dist_m": 0.09, "recovery_reverse_speed": 1.30,
                "recovery_clearance_exit_m": 0.28, "recovery_steer_angle_rad": 0.454,
            },
            # 1. High-Momentum Apex Striker
            {
                "v_cruise_max": 5.50, "v_cruise_min": 1.50,
                "horizon_N": 16, "dt": 0.025,
                "mu_friction": 0.88, "r_inflation_m": 0.082,
                "q_progress": 12.0, "q_cte": 14.0, "q_heading": 6.0,
                "r_steer": 3.5, "r_accel": 1.5, "r_steer_rate": 8.0,
                "v_corner_factor": 0.88, "pp_blend_gain": 0.25, "lookahead_blend_m": 0.40,
                "target_lookahead_m": 1.45, "w_gap_distance": 4.50, "w_gap_width": 2.20,
                "w_gap_steer": 0.60, "w_gap_straight": 0.50,
                "gap_dist_exp_k": 0.70, "gap_progress_margin_m": 0.12,
                "straight_lock_dist": 1.65, "straight_lock_speed": 5.50,
                "aggr_dist_far_m": 1.68, "aggr_dist_close_m": 1.10, "aggr_min_scale": 0.35,
                "steer_deadband_rad": 0.002,
                "emergency_backup_dist_m": 0.08, "recovery_reverse_speed": 1.40,
                "recovery_clearance_exit_m": 0.30, "recovery_steer_angle_rad": 0.480,
            },
            # 2. Safe High-Clearance Sentinel
            {
                "v_cruise_max": 4.50, "v_cruise_min": 1.00,
                "horizon_N": 20, "dt": 0.025,
                "mu_friction": 0.75, "r_inflation_m": 0.105,
                "q_progress": 6.0, "q_cte": 25.0, "q_heading": 12.0,
                "r_steer": 7.0, "r_accel": 3.0, "r_steer_rate": 15.0,
                "v_corner_factor": 0.75, "pp_blend_gain": 0.45, "lookahead_blend_m": 0.55,
                "target_lookahead_m": 1.25, "w_gap_distance": 3.00, "w_gap_width": 2.80,
                "w_gap_steer": 1.10, "w_gap_straight": 0.30,
                "gap_dist_exp_k": 0.50, "gap_progress_margin_m": 0.20,
                "straight_lock_dist": 1.75, "straight_lock_speed": 5.00,
                "aggr_dist_far_m": 1.75, "aggr_dist_close_m": 1.20, "aggr_min_scale": 0.40,
                "steer_deadband_rad": 0.003,
                "emergency_backup_dist_m": 0.10, "recovery_reverse_speed": 1.20,
                "recovery_clearance_exit_m": 0.25, "recovery_steer_angle_rad": 0.454,
            },
            # 3. Pure-Pursuit Damped Flow
            {
                "v_cruise_max": 5.00, "v_cruise_min": 1.30,
                "horizon_N": 18, "dt": 0.020,
                "mu_friction": 0.82, "r_inflation_m": 0.088,
                "q_progress": 9.0, "q_cte": 20.0, "q_heading": 9.0,
                "r_steer": 4.5, "r_accel": 1.8, "r_steer_rate": 9.0,
                "v_corner_factor": 0.82, "pp_blend_gain": 0.50, "lookahead_blend_m": 0.45,
                "target_lookahead_m": 1.40, "w_gap_distance": 3.80, "w_gap_width": 2.00,
                "w_gap_steer": 0.75, "w_gap_straight": 0.45,
                "gap_dist_exp_k": 0.65, "gap_progress_margin_m": 0.14,
                "straight_lock_dist": 1.70, "straight_lock_speed": 5.50,
                "aggr_dist_far_m": 1.70, "aggr_dist_close_m": 1.05, "aggr_min_scale": 0.32,
                "steer_deadband_rad": 0.002,
                "emergency_backup_dist_m": 0.09, "recovery_reverse_speed": 1.30,
                "recovery_clearance_exit_m": 0.28, "recovery_steer_angle_rad": 0.454,
            },
        ]
    elif isinstance(sample_inst, Option3SepticGenome):
        archetypes = [
            # 0. Proven Cyan Champion ToF Track Baseline (Index 0: Candidate shown in 20s real-time rendered preview run)
            {
                "v_entry_max": 5.50, "v_cruise_min": 1.50, "mu_friction": 0.85, "r_inflation_m": 0.090,
                "w_clearance": 2.50, "w_chord_tangent": 1.50, "kappa_smooth_gain": 0.20, "pp_lookahead_gain": 0.35,
                "slew_deg_max": 12.03, "steer_kp": 0.75, "blend_w0": 0.08, "blend_w1": 0.20, "blend_w2": 0.40,
                "target_lookahead_m": 1.35, "probe_samples": 36, "num_lat_offsets": 5, "lat_offset_max": 0.16,
                "num_head_offsets": 1, "head_offset_max": 0.00, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 1.00, "w_steer_effort": 0.80,
                "aggr_dist_far_m": 1.70, "aggr_dist_close_m": 1.35, "aggr_min_scale": 0.90,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.70,
            },
            # 1. Cautious High-Clearance Sentinel
            {
                "v_entry_max": 2.40, "v_cruise_min": 1.00, "mu_friction": 0.70, "r_inflation_m": 0.105,
                "w_clearance": 6.50, "w_chord_tangent": 2.00, "kappa_smooth_gain": 2.40, "pp_lookahead_gain": 0.38,
                "slew_deg_max": 11.50, "steer_kp": 0.80, "blend_w0": 0.06, "blend_w1": 0.20, "blend_w2": 0.46,
                "target_lookahead_m": 1.20, "probe_samples": 36, "num_lat_offsets": 7, "lat_offset_max": 0.15,
                "num_head_offsets": 3, "head_offset_max": 0.20, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 0.80, "w_steer_effort": 1.20,
                "aggr_dist_far_m": 1.75, "aggr_dist_close_m": 1.45, "aggr_min_scale": 0.96,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.75,
            },
            # 2. Smooth Flow Apex Carver
            {
                "v_entry_max": 2.90, "v_cruise_min": 1.20, "mu_friction": 0.78, "r_inflation_m": 0.092,
                "w_clearance": 4.80, "w_chord_tangent": 2.60, "kappa_smooth_gain": 2.50, "pp_lookahead_gain": 0.42,
                "slew_deg_max": 12.03, "steer_kp": 0.88, "blend_w0": 0.07, "blend_w1": 0.24, "blend_w2": 0.52,
                "target_lookahead_m": 1.35, "probe_samples": 36, "num_lat_offsets": 5, "lat_offset_max": 0.12,
                "num_head_offsets": 3, "head_offset_max": 0.24, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 1.10, "w_steer_effort": 1.40,
                "aggr_dist_far_m": 1.70, "aggr_dist_close_m": 1.40, "aggr_min_scale": 0.94,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.65,
            },
            # 3. Deep Horizon Multi-Target Scout
            {
                "v_entry_max": 2.80, "v_cruise_min": 1.15, "mu_friction": 0.74, "r_inflation_m": 0.095,
                "w_clearance": 5.00, "w_chord_tangent": 2.40, "kappa_smooth_gain": 2.20, "pp_lookahead_gain": 0.45,
                "slew_deg_max": 12.00, "steer_kp": 0.82, "blend_w0": 0.08, "blend_w1": 0.22, "blend_w2": 0.48,
                "target_lookahead_m": 1.50, "probe_samples": 40, "num_lat_offsets": 7, "lat_offset_max": 0.14,
                "num_head_offsets": 3, "head_offset_max": 0.22, "num_blend_weights": 3, "max_extra_targets": 3,
                "w_time": 1.00, "w_steer_effort": 1.10,
                "aggr_dist_far_m": 1.72, "aggr_dist_close_m": 1.42, "aggr_min_scale": 0.95,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.70,
            },
            # 4. Agile Corridor Striker
            {
                "v_entry_max": 3.10, "v_cruise_min": 1.25, "mu_friction": 0.80, "r_inflation_m": 0.090,
                "w_clearance": 4.20, "w_chord_tangent": 2.80, "kappa_smooth_gain": 1.80, "pp_lookahead_gain": 0.44,
                "slew_deg_max": 12.03, "steer_kp": 0.92, "blend_w0": 0.09, "blend_w1": 0.26, "blend_w2": 0.55,
                "target_lookahead_m": 1.30, "probe_samples": 36, "num_lat_offsets": 7, "lat_offset_max": 0.15,
                "num_head_offsets": 3, "head_offset_max": 0.25, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 1.25, "w_steer_effort": 0.90,
                "aggr_dist_far_m": 1.70, "aggr_dist_close_m": 1.38, "aggr_min_scale": 0.92,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.65,
            },
            # 5. Compute-Lean Fast Evaluator (~0.25ms C++ profiler time)
            {
                "v_entry_max": 2.70, "v_cruise_min": 1.15, "mu_friction": 0.74, "r_inflation_m": 0.094,
                "w_clearance": 5.20, "w_chord_tangent": 2.30, "kappa_smooth_gain": 2.00, "pp_lookahead_gain": 0.40,
                "slew_deg_max": 12.00, "steer_kp": 0.85, "blend_w0": 0.08, "blend_w1": 0.24, "blend_w2": 0.50,
                "target_lookahead_m": 1.25, "probe_samples": 24, "num_lat_offsets": 5, "lat_offset_max": 0.12,
                "num_head_offsets": 3, "head_offset_max": 0.20, "num_blend_weights": 2, "max_extra_targets": 1,
                "w_time": 0.95, "w_steer_effort": 1.00,
                "aggr_dist_far_m": 1.75, "aggr_dist_close_m": 1.45, "aggr_min_scale": 0.95,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.70,
            },
            # 6. Moderate Track Pacer
            {
                "v_entry_max": 3.00, "v_cruise_min": 1.20, "mu_friction": 0.76, "r_inflation_m": 0.092,
                "w_clearance": 4.60, "w_chord_tangent": 2.50, "kappa_smooth_gain": 2.00, "pp_lookahead_gain": 0.42,
                "slew_deg_max": 12.03, "steer_kp": 0.88, "blend_w0": 0.08, "blend_w1": 0.24, "blend_w2": 0.52,
                "target_lookahead_m": 1.32, "probe_samples": 36, "num_lat_offsets": 7, "lat_offset_max": 0.13,
                "num_head_offsets": 3, "head_offset_max": 0.22, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 1.05, "w_steer_effort": 1.05,
                "aggr_dist_far_m": 1.70, "aggr_dist_close_m": 1.40, "aggr_min_scale": 0.94,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.68,
            },
            # 7. Maximum Traction Cushion (Friction safety reserve)
            {
                "v_entry_max": 3.20, "v_cruise_min": 1.30, "mu_friction": 0.82, "r_inflation_m": 0.088,
                "w_clearance": 4.10, "w_chord_tangent": 3.00, "kappa_smooth_gain": 1.70, "pp_lookahead_gain": 0.44,
                "slew_deg_max": 12.03, "steer_kp": 0.90, "blend_w0": 0.07, "blend_w1": 0.22, "blend_w2": 0.48,
                "target_lookahead_m": 1.38, "probe_samples": 36, "num_lat_offsets": 7, "lat_offset_max": 0.13,
                "num_head_offsets": 3, "head_offset_max": 0.24, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 1.30, "w_steer_effort": 0.95,
                "aggr_dist_far_m": 1.68, "aggr_dist_close_m": 1.38, "aggr_min_scale": 0.92,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.65,
            },
            # 8. Wide Angle Turn-In Specialist
            {
                "v_entry_max": 2.75, "v_cruise_min": 1.10, "mu_friction": 0.74, "r_inflation_m": 0.096,
                "w_clearance": 5.00, "w_chord_tangent": 2.40, "kappa_smooth_gain": 2.10, "pp_lookahead_gain": 0.40,
                "slew_deg_max": 12.00, "steer_kp": 0.86, "blend_w0": 0.08, "blend_w1": 0.24, "blend_w2": 0.52,
                "target_lookahead_m": 1.25, "probe_samples": 36, "num_lat_offsets": 7, "lat_offset_max": 0.14,
                "num_head_offsets": 5, "head_offset_max": 0.30, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 0.95, "w_steer_effort": 1.15,
                "aggr_dist_far_m": 1.72, "aggr_dist_close_m": 1.42, "aggr_min_scale": 0.95,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.70,
            },
            # 9. Conservative Trail-Braker
            {
                "v_entry_max": 2.95, "v_cruise_min": 1.05, "mu_friction": 0.72, "r_inflation_m": 0.100,
                "w_clearance": 5.30, "w_chord_tangent": 2.20, "kappa_smooth_gain": 2.30, "pp_lookahead_gain": 0.38,
                "slew_deg_max": 11.50, "steer_kp": 0.82, "blend_w0": 0.06, "blend_w1": 0.20, "blend_w2": 0.46,
                "target_lookahead_m": 1.22, "probe_samples": 36, "num_lat_offsets": 7, "lat_offset_max": 0.14,
                "num_head_offsets": 3, "head_offset_max": 0.20, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 0.85, "w_steer_effort": 1.25,
                "aggr_dist_far_m": 1.76, "aggr_dist_close_m": 1.46, "aggr_min_scale": 0.96,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.75,
            },
            # 10. Dense Lateral Gap Explorer
            {
                "v_entry_max": 2.65, "v_cruise_min": 1.15, "mu_friction": 0.75, "r_inflation_m": 0.095,
                "w_clearance": 5.40, "w_chord_tangent": 2.30, "kappa_smooth_gain": 2.00, "pp_lookahead_gain": 0.40,
                "slew_deg_max": 12.03, "steer_kp": 0.85, "blend_w0": 0.08, "blend_w1": 0.22, "blend_w2": 0.50,
                "target_lookahead_m": 1.28, "probe_samples": 36, "num_lat_offsets": 9, "lat_offset_max": 0.15,
                "num_head_offsets": 3, "head_offset_max": 0.22, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 0.90, "w_steer_effort": 1.00,
                "aggr_dist_far_m": 1.74, "aggr_dist_close_m": 1.44, "aggr_min_scale": 0.95,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.70,
            },
            # 11. Minimal-Jerk Arc Glider (High curvature smoothness)
            {
                "v_entry_max": 2.85, "v_cruise_min": 1.20, "mu_friction": 0.76, "r_inflation_m": 0.094,
                "w_clearance": 4.80, "w_chord_tangent": 2.10, "kappa_smooth_gain": 3.20, "pp_lookahead_gain": 0.44,
                "slew_deg_max": 11.00, "steer_kp": 0.78, "blend_w0": 0.07, "blend_w1": 0.20, "blend_w2": 0.45,
                "target_lookahead_m": 1.32, "probe_samples": 36, "num_lat_offsets": 5, "lat_offset_max": 0.12,
                "num_head_offsets": 3, "head_offset_max": 0.20, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 0.90, "w_steer_effort": 2.00,
                "aggr_dist_far_m": 1.72, "aggr_dist_close_m": 1.42, "aggr_min_scale": 0.94,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.70,
            },
            # 12. High-Speed Boundary Prober
            {
                "v_entry_max": 3.40, "v_cruise_min": 1.35, "mu_friction": 0.84, "r_inflation_m": 0.086,
                "w_clearance": 3.80, "w_chord_tangent": 3.20, "kappa_smooth_gain": 1.60, "pp_lookahead_gain": 0.45,
                "slew_deg_max": 12.03, "steer_kp": 0.95, "blend_w0": 0.09, "blend_w1": 0.25, "blend_w2": 0.55,
                "target_lookahead_m": 1.42, "probe_samples": 36, "num_lat_offsets": 7, "lat_offset_max": 0.13,
                "num_head_offsets": 3, "head_offset_max": 0.24, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 1.45, "w_steer_effort": 0.85,
                "aggr_dist_far_m": 1.68, "aggr_dist_close_m": 1.38, "aggr_min_scale": 0.92,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.65,
            },
            # 13. Balanced Pure-Pursuit Damped
            {
                "v_entry_max": 2.75, "v_cruise_min": 1.15, "mu_friction": 0.75, "r_inflation_m": 0.096,
                "w_clearance": 4.80, "w_chord_tangent": 2.40, "kappa_smooth_gain": 2.20, "pp_lookahead_gain": 0.52,
                "slew_deg_max": 11.50, "steer_kp": 0.80, "blend_w0": 0.08, "blend_w1": 0.22, "blend_w2": 0.48,
                "target_lookahead_m": 1.35, "probe_samples": 36, "num_lat_offsets": 7, "lat_offset_max": 0.13,
                "num_head_offsets": 3, "head_offset_max": 0.22, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 0.95, "w_steer_effort": 1.30,
                "aggr_dist_far_m": 1.74, "aggr_dist_close_m": 1.42, "aggr_min_scale": 0.95,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.70,
            },
            # 14. Precision Chicane Weaver
            {
                "v_entry_max": 3.05, "v_cruise_min": 1.25, "mu_friction": 0.78, "r_inflation_m": 0.090,
                "w_clearance": 4.40, "w_chord_tangent": 2.70, "kappa_smooth_gain": 1.90, "pp_lookahead_gain": 0.42,
                "slew_deg_max": 12.03, "steer_kp": 0.90, "blend_w0": 0.06, "blend_w1": 0.18, "blend_w2": 0.42,
                "target_lookahead_m": 1.30, "probe_samples": 36, "num_lat_offsets": 7, "lat_offset_max": 0.14,
                "num_head_offsets": 3, "head_offset_max": 0.24, "num_blend_weights": 4, "max_extra_targets": 2,
                "w_time": 1.15, "w_steer_effort": 1.00,
                "aggr_dist_far_m": 1.70, "aggr_dist_close_m": 1.40, "aggr_min_scale": 0.94,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.68,
            },
            # 15. Oversteer Guard Navigator (Safe high clearance)
            {
                "v_entry_max": 2.55, "v_cruise_min": 1.05, "mu_friction": 0.72, "r_inflation_m": 0.102,
                "w_clearance": 5.80, "w_chord_tangent": 2.00, "kappa_smooth_gain": 2.40, "pp_lookahead_gain": 0.38,
                "slew_deg_max": 11.00, "steer_kp": 0.75, "blend_w0": 0.07, "blend_w1": 0.20, "blend_w2": 0.45,
                "target_lookahead_m": 1.22, "probe_samples": 36, "num_lat_offsets": 7, "lat_offset_max": 0.14,
                "num_head_offsets": 3, "head_offset_max": 0.20, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 0.80, "w_steer_effort": 2.20,
                "aggr_dist_far_m": 1.76, "aggr_dist_close_m": 1.46, "aggr_min_scale": 0.96,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.75,
            },
            # 16. Dynamic S-Bend Scout
            {
                "v_entry_max": 2.90, "v_cruise_min": 1.20, "mu_friction": 0.76, "r_inflation_m": 0.094,
                "w_clearance": 4.60, "w_chord_tangent": 2.50, "kappa_smooth_gain": 2.00, "pp_lookahead_gain": 0.42,
                "slew_deg_max": 12.03, "steer_kp": 0.86, "blend_w0": 0.08, "blend_w1": 0.22, "blend_w2": 0.50,
                "target_lookahead_m": 1.40, "probe_samples": 36, "num_lat_offsets": 7, "lat_offset_max": 0.13,
                "num_head_offsets": 3, "head_offset_max": 0.22, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 1.05, "w_steer_effort": 1.10,
                "aggr_dist_far_m": 1.72, "aggr_dist_close_m": 1.40, "aggr_min_scale": 0.94,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.68,
            },
            # 17. Refined Apex Cutter
            {
                "v_entry_max": 3.15, "v_cruise_min": 1.28, "mu_friction": 0.80, "r_inflation_m": 0.088,
                "w_clearance": 4.00, "w_chord_tangent": 3.20, "kappa_smooth_gain": 1.75, "pp_lookahead_gain": 0.43,
                "slew_deg_max": 12.03, "steer_kp": 0.90, "blend_w0": 0.08, "blend_w1": 0.24, "blend_w2": 0.52,
                "target_lookahead_m": 1.35, "probe_samples": 36, "num_lat_offsets": 7, "lat_offset_max": 0.13,
                "num_head_offsets": 3, "head_offset_max": 0.24, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 1.25, "w_steer_effort": 0.95,
                "aggr_dist_far_m": 1.68, "aggr_dist_close_m": 1.38, "aggr_min_scale": 0.92,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.65,
            },
            # 18. Safe Perimeter Stroller
            {
                "v_entry_max": 2.45, "v_cruise_min": 0.95, "mu_friction": 0.68, "r_inflation_m": 0.108,
                "w_clearance": 7.00, "w_chord_tangent": 1.80, "kappa_smooth_gain": 2.60, "pp_lookahead_gain": 0.36,
                "slew_deg_max": 11.00, "steer_kp": 0.74, "blend_w0": 0.06, "blend_w1": 0.18, "blend_w2": 0.44,
                "target_lookahead_m": 1.18, "probe_samples": 36, "num_lat_offsets": 7, "lat_offset_max": 0.15,
                "num_head_offsets": 3, "head_offset_max": 0.18, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 0.75, "w_steer_effort": 1.50,
                "aggr_dist_far_m": 1.78, "aggr_dist_close_m": 1.48, "aggr_min_scale": 0.97,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.78,
            },
            # 19. Pareto Optimal All-Rounder
            {
                "v_entry_max": 2.80, "v_cruise_min": 1.18, "mu_friction": 0.76, "r_inflation_m": 0.095,
                "w_clearance": 4.80, "w_chord_tangent": 2.50, "kappa_smooth_gain": 2.00, "pp_lookahead_gain": 0.41,
                "slew_deg_max": 12.00, "steer_kp": 0.85, "blend_w0": 0.08, "blend_w1": 0.22, "blend_w2": 0.50,
                "target_lookahead_m": 1.30, "probe_samples": 36, "num_lat_offsets": 7, "lat_offset_max": 0.13,
                "num_head_offsets": 3, "head_offset_max": 0.22, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 1.00, "w_steer_effort": 1.10,
                "aggr_dist_far_m": 1.72, "aggr_dist_close_m": 1.42, "aggr_min_scale": 0.95,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.70,
            },
            # 20. Momentum Carrier
            {
                "v_entry_max": 3.25, "v_cruise_min": 1.32, "mu_friction": 0.80, "r_inflation_m": 0.088,
                "w_clearance": 4.00, "w_chord_tangent": 2.90, "kappa_smooth_gain": 1.70, "pp_lookahead_gain": 0.43,
                "slew_deg_max": 12.03, "steer_kp": 0.90, "blend_w0": 0.08, "blend_w1": 0.24, "blend_w2": 0.52,
                "target_lookahead_m": 1.38, "probe_samples": 36, "num_lat_offsets": 7, "lat_offset_max": 0.13,
                "num_head_offsets": 3, "head_offset_max": 0.24, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 1.30, "w_steer_effort": 0.90,
                "aggr_dist_far_m": 1.68, "aggr_dist_close_m": 1.38, "aggr_min_scale": 0.92,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.65,
            },
            # 21. Fine Mesh Explorer (44 probe samples)
            {
                "v_entry_max": 2.75, "v_cruise_min": 1.15, "mu_friction": 0.75, "r_inflation_m": 0.096,
                "w_clearance": 5.00, "w_chord_tangent": 2.40, "kappa_smooth_gain": 2.10, "pp_lookahead_gain": 0.41,
                "slew_deg_max": 12.00, "steer_kp": 0.85, "blend_w0": 0.08, "blend_w1": 0.22, "blend_w2": 0.50,
                "target_lookahead_m": 1.30, "probe_samples": 44, "num_lat_offsets": 7, "lat_offset_max": 0.13,
                "num_head_offsets": 3, "head_offset_max": 0.22, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 1.00, "w_steer_effort": 1.10,
                "aggr_dist_far_m": 1.74, "aggr_dist_close_m": 1.44, "aggr_min_scale": 0.95,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.70,
            },
            # 22. Gentle Transition Cruiser
            {
                "v_entry_max": 2.65, "v_cruise_min": 1.12, "mu_friction": 0.74, "r_inflation_m": 0.098,
                "w_clearance": 5.20, "w_chord_tangent": 2.20, "kappa_smooth_gain": 2.60, "pp_lookahead_gain": 0.40,
                "slew_deg_max": 11.50, "steer_kp": 0.80, "blend_w0": 0.07, "blend_w1": 0.20, "blend_w2": 0.46,
                "target_lookahead_m": 1.25, "probe_samples": 36, "num_lat_offsets": 7, "lat_offset_max": 0.13,
                "num_head_offsets": 3, "head_offset_max": 0.20, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 0.88, "w_steer_effort": 1.60,
                "aggr_dist_far_m": 1.75, "aggr_dist_close_m": 1.45, "aggr_min_scale": 0.96,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.72,
            },
            # 23. Ultra-Responsive Agile Scout
            {
                "v_entry_max": 3.00, "v_cruise_min": 1.22, "mu_friction": 0.78, "r_inflation_m": 0.092,
                "w_clearance": 4.50, "w_chord_tangent": 2.70, "kappa_smooth_gain": 1.90, "pp_lookahead_gain": 0.38,
                "slew_deg_max": 12.03, "steer_kp": 0.95, "blend_w0": 0.08, "blend_w1": 0.24, "blend_w2": 0.52,
                "target_lookahead_m": 1.28, "probe_samples": 36, "num_lat_offsets": 7, "lat_offset_max": 0.14,
                "num_head_offsets": 3, "head_offset_max": 0.24, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 1.15, "w_steer_effort": 0.95,
                "aggr_dist_far_m": 1.70, "aggr_dist_close_m": 1.40, "aggr_min_scale": 0.94,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.68,
            },
            # 24. Straightaway Launcher
            {
                "v_entry_max": 3.50, "v_cruise_min": 1.30, "mu_friction": 0.82, "r_inflation_m": 0.088,
                "w_clearance": 4.20, "w_chord_tangent": 3.00, "kappa_smooth_gain": 1.70, "pp_lookahead_gain": 0.45,
                "slew_deg_max": 12.03, "steer_kp": 0.90, "blend_w0": 0.08, "blend_w1": 0.22, "blend_w2": 0.50,
                "target_lookahead_m": 1.45, "probe_samples": 36, "num_lat_offsets": 7, "lat_offset_max": 0.13,
                "num_head_offsets": 3, "head_offset_max": 0.22, "num_blend_weights": 3, "max_extra_targets": 2,
                "w_time": 1.50, "w_steer_effort": 0.95,
                "aggr_dist_far_m": 1.70, "aggr_dist_close_m": 1.40, "aggr_min_scale": 0.92,
                "steer_deadband_rad": 0.002, "straight_lock_dist": 1.75,
            },
        ]

    population: List[BasePlannerGenome] = []

    # Insert defined archetypes
    for arch in archetypes:
        if len(population) < pop_size:
            g = genome_cls(arch)
            population.append(g)

    # Fill remainder of pop_size using Latin Hypercube Sampling stratified around the archetypes
    n_remain = pop_size - len(population)
    if n_remain > 0:
        base_archetype = archetypes[0] if archetypes else sample_inst.to_dict()
        base_arr = genome_cls(base_archetype).to_array()
        # Stratified grid noise
        for k in range(n_remain):
            # Sample across intervals
            grid_frac = (k + 0.5) / n_remain
            noise = (np.random.uniform(-0.18, 0.18, size=base_arr.shape) + (grid_frac - 0.5) * 0.40) * span
            cand_arr = np.clip(base_arr + noise, lower, upper)
            cand = genome_cls()
            cand.from_array(cand_arr)
            # Enforce logical relations if present
            if "v_cruise_min" in cand.values and "v_cruise_max" in cand.values:
                cand.values["v_cruise_min"] = min(cand.values["v_cruise_min"], cand.values["v_cruise_max"] - 0.30)
            if "blend_w0" in cand.values and "blend_w1" in cand.values and "blend_w2" in cand.values:
                cand.values["blend_w0"] = min(cand.values["blend_w0"], cand.values["blend_w1"] - 0.10)
                cand.values["blend_w2"] = max(cand.values["blend_w2"], cand.values["blend_w1"] + 0.10)
            cand.clamp_and_repair()
            population.append(cand)

    return population


def compute_efficiency_multiplier(gene_dict: Optional[Dict[str, Any]]) -> float:
    """
    Computes a conservative efficiency multiplier in [0.50, 1.00].
    Penalizes configurations that consume excessive CPU time when simpler,
    faster parameterizations achieve identical physical driving performance.

    Total candidate spline evaluations per cycle:
      N_eval = (1 + max_extra_targets) * num_lat_offsets * num_head_offsets * num_blend_weights * probe_samples
    Lightest compute (min parameters): 360 points (~0.12 ms) -> multiplier = 1.000
    Nominal compute (default baseline): 11,340 points (~3.3 ms) -> multiplier = 0.912
    Heaviest compute (max parameters): 63,000 points (~18.5 ms) -> multiplier = 0.500
    """
    if not gene_dict:
        return 1.0

    probe_samples = float(gene_dict.get("probe_samples", 36))
    num_lat = float(gene_dict.get("num_lat_offsets", 7))
    num_head = float(gene_dict.get("num_head_offsets", 5))
    num_blends = float(gene_dict.get("num_blend_weights", 3))
    extra_targets = float(gene_dict.get("max_extra_targets", 2))

    total_eval_points = (1.0 + extra_targets) * num_lat * num_head * num_blends * probe_samples

    min_points = 1.0 * 3.0 * 3.0 * 2.0 * 20.0       # 360 points
    max_points = 4.0 * 9.0 * 7.0 * 5.0 * 50.0      # 63,000 points

    norm_load = (total_eval_points - min_points) / max(1.0, (max_points - min_points))
    norm_load = max(0.0, min(1.0, norm_load))
    multiplier = 1.00 - (0.50 * norm_load)
    return float(multiplier)


# Maximum physically achievable vehicle speed in m/s
SPEED_PHYSICAL_MAX_MPS: float = 5.50
# Minimum threshold of distance to be considered a valid run
DISTANCE_VALID_MIN_M: float = 0.10


def compute_scalar_fitness(metrics: Dict[str, Any], gene_dict: Optional[Dict[str, Any]] = None) -> float:
    """
    Computes scalar optimization objective from Webots trial metrics.
    Progress (distance traveled in 20s) is the primary driver.

    Applies conservative [0.50, 1.00] compute efficiency multiplier:
    With 2 identical performances, the one with less compute scores better.

    Safety / Sanity Guarantees:
    - 0 distance covered (or <= 0.10m) is treated as a failed run (-5000.0 flat fitness).
    - Speed is strictly clamped to [0.0, 5.5 m/s]. Speeds exceeding 5.5 m/s indicate a physics glitch.
    - Crash: penalty of -50.0 pts (real penalty is lost run time to accumulate distance).
    - Stall: moderate stall penalty.
    """
    dist_m = float(metrics.get("distance_m", 0.0))
    mean_speed = float(metrics.get("mean_speed_mps", 0.0))
    laps = int(metrics.get("laps_completed", 0))
    crashed = bool(metrics.get("crashed", False))
    stuck = bool(metrics.get("stalled", False))

    # Sanity Check 1: 0 or negative/negligible distance covered is a failed run
    if math.isnan(dist_m) or dist_m <= DISTANCE_VALID_MIN_M:
        return -5000.0

    # Sanity Check 2: Physical speed ceiling. Max speed physically achievable is 5.5 m/s.
    if math.isnan(mean_speed) or mean_speed < 0.0:
        mean_speed = 0.0
    elif mean_speed > SPEED_PHYSICAL_MAX_MPS:
        mean_speed = SPEED_PHYSICAL_MAX_MPS

    # Base score: reward true distance, speed, laps
    base_score = (dist_m * FITNESS_PROGRESS_WEIGHT) + (mean_speed * FITNESS_SPEED_WEIGHT) + (laps * FITNESS_LAP_BONUS)

    if crashed:
        # Flat deduction: real loss comes from less time to accumulate distance.
        raw_fitness = float(base_score - 50.0)
    elif stuck:
        # Moderate stall penalty: car went nowhere and wasted evaluation window.
        raw_fitness = float(base_score - FITNESS_STALL_PENALTY * 0.05)
    else:
        raw_fitness = float(base_score)

    compute_mult = compute_efficiency_multiplier(gene_dict)
    if raw_fitness >= 0.0:
        return float(raw_fitness * compute_mult)
    else:
        return float(raw_fitness / compute_mult)


