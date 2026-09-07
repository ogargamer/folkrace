"""
================================================================================
  HIGH-PERFORMANCE MULTI-MODAL MPPI CONTROLLER SUITE FOR FOLKRACE 4WD
  Target: Raspberry Pi 5 CPU @ 60 Hz (< 8.0 ms execution budget)
================================================================================
  Variants & Optimizations Integrated:
    1. Biased / Multi-Modal Lane Seeding (Warmstart Center, Left +0.25m, Right -0.25m)
    2. Smooth-MPPI: First-Order Autoregressive (AR-1) Colored Noise Filter
    3. Log-MPPI: Normal Log-Normal Velocity Sampling (+vx biased forward exploration)
    4. Shield-MPPI: Early-Exit Costmap Collision Pruning & Memory Rejection
    5. CBF / Friction Circle Traction Constraint: sqrt(ax^2 + ay^2) <= mu * g
    6. Actuator Slew Rate Clamping (|d_delta/dt| <= 10.47 rad/s) & Latency Compensation
    7. Exponential Time Discounting: gamma^t across horizon stage costs
    8. Trajectory Warm-Starting with 60 Hz rolling horizon preservation
    9. Flat Genetic Chromosome Architecture (MPPIGenome) - No Hidden Layers
================================================================================
"""

import os
import sys
import math
import time
import json
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Tuple, Any

import numpy as np

# Fast JIT SIMD MPPI Engine
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "mppi_controller"))
from fast_mppi_core import evaluate_mppi_rollouts_jit

# --- VEHICLE PHYSICAL KINEMATICS CONSTANTS ---
WHEELBASE = 0.1325              # meters (axle-to-axle distance)
MAX_STEERING_RAD = 0.6981       # +/- 40 degrees maximum steering limit (0.698132 rad)
MAX_STEER_RATE_RAD_S = 10.47    # EMAX ES08MD II servo limit @ 6.0V (rad/s)
MAX_SPEED_MPS = 4.40            # Top physical wheel speed (160 rad/s * 0.0275 m)
GRAVITY_MPS2 = 9.81             # Gravitational acceleration (m/s^2)
TRANSPORT_LATENCY_S = 0.0166    # 1 control tick latency compensation (16.6 ms)

# Costmap Grid Bounds (Matching FastToFLUTCostmap with 0.5m rear buffer)
COSTMAP_X_MIN = -1.5
COSTMAP_X_MAX = 1.5
COSTMAP_Y_MIN = -0.5
COSTMAP_Y_MAX = 2.4
COSTMAP_RES = 0.02

# Real-time Compute Budget (~15k compute ceiling: K * H <= 15420, e.g., 512 * 28 or 480 * 32)
MAX_COMPUTE_BUDGET_KH = 15420


# ==============================================================================
# 1. FLAT GENETIC GENOME ARCHITECTURE (NO HIDDEN LAYERS)
# ==============================================================================
@dataclass
class MPPIGenome:
    """
    Transparent, flat parameter chromosome designed for direct genetic algorithm
    tuning (GA, CMA-ES, Differential Evolution) without neural network hidden layers.
    
    Includes dynamic compute budget coupling (K * H <= MAX_COMPUTE_BUDGET_KH)
    so the GA can trade off prediction horizon depth against sample count density.
    """
    # ==========================================================================
    # 1. HORIZON & TEMPORAL RESOLUTION
    # ==========================================================================
    # Total Lookahead Time = H * dt (e.g. 26 * 0.02128s = ~0.55 seconds into future)
    H: int = 20                     
    # Gene 0: Prediction Horizon Steps (10 to 35 steps).
    # Controls how far into the future MPPI plans. Higher H lets the car anticipate
    # sharp chicanes and oncoming obstacles earlier, but increases compute cost.

    K: int = 512                   
    # Gene 1: Trajectory Sample Count (derived/clamped: K <= 15420 // H).
    # Number of parallel rollouts evaluated per cycle. Higher K gives denser exploration.

    dt: float = 0.055            
    # Gene 2: Rollout Step Time in seconds (0.040 to 0.080s).
    # Time delta between simulated trajectory nodes. Deep lookahead (0.8s to 2.4s) for high-speed tracking.

    # ==========================================================================
    # 2. MPPI STOCHASTIC & TEMPERATURE PARAMETERS
    # ==========================================================================
    lambda_temp: float = 0.4     
    # Gene 3: Softmax Temperature / Cost Sensitivity (0.01 to 2.0).
    # Controls how MPPI blends candidate trajectories into the final control command:
    # - Low lambda (0.05 to 0.35): "Winner-takes-all" greedy behavior. The single best
    #   lowest-cost trajectory dominates ~90%+ of the output weight. Prevents indecision
    #   around obstacles (avoids splitting/averaging between left and right).
    # - High lambda (0.80 to 2.00): Soft blending across many paths. Produces smoother
    #   controls in open track, but risks averaging two opposing avoidance routes.

    ar1_alpha: float = 0.7095       
    # Gene 4: AR-1 Colored Noise Temporal Correlation (0.00 to 0.95).
    # Controls trajectory smoothness over time across the prediction horizon:
    # - alpha = 0.0: Pure white noise (twitchy, jagged random steering changes at each step).
    # - alpha = 0.70 - 0.85: Smooth colored noise (produces realistic, continuous curving arcs
    #   resembling human racing driver inputs).

    gamma_discount: float = 0.90 
    # Gene 5: Exponential Horizon Discount Factor (0.70 to 1.00).
    # Weights immediate risks higher than distant future risks (cost at step t is multiplied
    # by gamma^t). Ensures immediate collision avoidance takes priority over far-away turns.

    mu_friction: float = 0.5896     
    # Gene 6: Tire-Road Friction Coefficient [0.55 to 0.90] (A22 silicone tires).
    # Represents maximum grip. Used by the Control Barrier Function (CBF) traction circle
    # to clamp lateral acceleration (a_lat = v^2 / R <= mu * g) so the car never breaks traction.

    # ==========================================================================
    # 3. GUIDANCE & PATH-TRACKING COST WEIGHTS
    # ==========================================================================
    w_cte: float = 12.057           
    # Gene 7: Cross-Track Error Penalty (0.1 to 50.0).
    # Penalizes lateral distance away from the optimal global racing line when the track is clear.

    w_heading: float = 2.754        
    # Gene 8: Heading Alignment Penalty (0.1 to 30.0).
    # Penalizes angular deviation between the vehicle's yaw and the track tangent vector.

    w_speed: float = 0.759          
    # Gene 9: Target Speed Tracking Penalty (0.1 to 20.0).
    # Penalizes driving slower (or faster) than the curvature-optimal reference speed.

    w_hazard: float = 5000000.0     
    # Gene 10: Hazard Cell Penetration Penalty (1e3 to 5e6).
    # Massive lethal barrier penalty added when any rollout point touches the red C-space
    # inflated obstacle envelope. Instantly prunes dangerous crash trajectories.

    w_prob_costmap: float = 50.0  
    # Gene 11: Continuous Log-Odds / Wall Proximity Penalty (1.0 to 200.0).
    # Soft clearance cost that gently repels the vehicle away from walls and moving cars
    # even when outside the hard lethal barrier.

    w_unknown: float = 200.0       
    # Gene 12: Unknown / Unobserved Space Penalty (0.0 to 200.0).
    # Discourages driving into gray unmapped regions of the grid where obstacles might hide.

    w_steer_rate: float = 0.550     
    # Gene 13: Steering Rate Variation Penalty (0.01 to 10.0) [Prev: 0.60].
    # Penalizes sudden steering wheel jerks to prevent high-speed fishtailing and oscillation.

    w_accel_rate: float = 0.351     
    # Gene 14: Longitudinal Jerk / Accel Penalty (0.01 to 10.0) [Prev: 0.20].
    # Penalizes rapid throttle-to-brake transitions for smooth motor torque delivery.

    w_terminal: float = 9.424       
    # Gene 15: Terminal Horizon State Multiplier (1.0 to 20.0) [Prev: 6.0].
    # Extra reward/penalty on the very last step t=H. Encourages the car to finish its
    # lookahead window pointing straight down the track exit rather than drifting sideways.

    # ==========================================================================
    # 4. PERTURBATION SAMPLING VARIANCES
    # ==========================================================================
    sigma_steer: float = 0.1041     
    # Gene 16: Steering Noise Std Dev (0.02 to 0.35 rad, ~6.0 deg).
    # Amplitude of steering exploratory noise added around nominal seed paths.

    sigma_speed: float = 0.5172     
    # Gene 17: Speed Noise Std Dev (0.10 to 2.00 m/s).
    # Amplitude of longitudinal velocity exploration.

    lognorm_shape: float = 0.1874   
    # Gene 18: Forward Velocity Log-Normal Skewness (0.10 to 1.50).
    # Shapes the speed noise distribution to be asymmetric (biases exploration toward
    # accelerating forward rather than unnecessary braking).

    # ==========================================================================
    # 5. MULTI-MODAL SVG SEEDING ALLOCATION (Center 40%, Straight 20%, Left 20%, Right 20%)
    # ==========================================================================
    ratio_center: float = 0.40    
    # Gene 19: Center / Racing Line Seed Ratio (40% of Stage 1 forward budget).
    # Follows the global curvature-optimal trajectory.

    ratio_straight: float = 0.20  
    # Gene 20: Straight-Ahead Gap Shooter Seed Ratio (20% of Stage 1 forward budget).
    # Maintains zero steering (delta = 0.0) to punch through straight gaps and chicanes.

    ratio_left: float = 0.20      
    # Gene 21: Left Seed Ratio (+25cm offset) (20% of Stage 1 forward budget).
    # Tracks the left overtaking corridor.
    # Note: Right seed ratio is derived automatically as (1.0 - ratio_center - ratio_straight - ratio_left).

    pure_pursuit_lookahead: float = 0.30 
    # Gene 22: Pure Pursuit Seeding Lookahead Distance in meters (0.15 to 0.80 m).
    # Lookahead anchor used by the geometric seed generator to establish the baseline guide paths.

    r_inflation_m: float = 0.090
    # Gene 23: Obstacle C-Space Inflation Radius in meters (0.080 to 0.105 m).
    # Expands obstacles in the costmap. For a 15 cm wide robot chassis (7.5 cm half-width),
    # provides 0.5 cm to 3.0 cm physical clearance margin without blocking narrow 60 cm corridors.

    w_reverse: float = 250.0
    # Gene 24: Reverse Cost Bias Penalty (50.0 to 800.0).
    # Penalty added to backward trajectories. Lower values make vehicle reverse more eagerly;
    # higher values ensure reverse is only chosen when forward paths are blocked.

    reverse_speed: float = 0.75
    # Gene 25: Commanded Reverse Speed Magnitude (0.40 to 1.50 m/s).
    # Backing velocity applied during reverse recovery cusps and escapes.

    t_rev_steps: int = 3
    # Gene 26: Reverse Backup Duration in Steps (2 to 7 steps, ~0.05s to 0.20s).
    # Duration the vehicle rolls backward before swinging forward downstream.
    
    # Compute budget ceiling
    max_budget_kh: int = MAX_COMPUTE_BUDGET_KH
    auto_derive_k: bool = False     # If True, K is calculated as max_budget_kh // H

    # Static metadata describing gene bounds for GA optimizers (27 genes)
    GENE_SPECS = [
        ("H", 18, 35, int),                  # 0: Prediction horizon (0.45s - 1.2s window)
        ("K", 128, 512, int),                # 1: Trajectory sample count (K * H <= 15420)
        ("dt", 0.040, 0.080, float),         # 2: Step time (extended lookahead 0.8s - 2.4s)
        ("lambda_temp", 0.10, 1.50, float),  # 3: Softmax temperature (sharp apexing to smooth blending)
        ("ar1_alpha", 0.40, 0.95, float),    # 4: Colored noise smoothness
        ("gamma_discount", 0.80, 1.00, float),# 5: Time discount
        ("mu_friction", 0.55, 0.80, float),  # 6: Strictly bounded to [0.55, 0.80] for stable cornering
        ("w_cte", 2.0, 30.0, float),         # 7: Cross-track penalty
        ("w_heading", 1.0, 20.0, float),     # 8: Heading penalty
        ("w_speed", 0.5, 15.0, float),       # 9: Speed tracking penalty
        ("w_hazard", 20000.0, 500000.0, float), # 10: Fatal obstacle penalty
        ("w_prob_costmap", 20.0, 150.0, float), # 11: Wall proximity clearance
        ("w_unknown", 0.0, 30.0, float),     # 12: Unobserved space penalty
        ("w_steer_rate", 0.10, 3.0, float),  # 13: Steer slew penalty
        ("w_accel_rate", 0.05, 3.0, float),  # 14: Jerk penalty
        ("w_terminal", 2.0, 15.0, float),    # 15: Terminal horizon cost
        ("sigma_steer", 0.04, 0.25, float),  # 16: Steer noise std
        ("sigma_speed", 0.10, 0.80, float),  # 17: Speed noise std
        ("lognorm_shape", 0.10, 0.60, float),# 18: Forward skewness
        ("ratio_center", 0.15, 0.60, float), # 19: Center seed ratio
        ("ratio_straight", 0.05, 0.35, float),# 20: Straight gap shooter ratio
        ("ratio_left", 0.10, 0.40, float),   # 21: Left seed ratio
        ("pure_pursuit_lookahead", 0.20, 0.65, float), # 22: Lookahead distance
        ("r_inflation_m", 0.080, 0.105, float), # 23: C-Space expansion (0.5cm - 3.0cm physical clearance)
        ("w_reverse", 50.0, 800.0, float),   # 24: Reverse bias penalty
        ("reverse_speed", 0.40, 1.50, float),# 25: Commanded reverse speed magnitude
        ("t_rev_steps", 2, 7, int),          # 26: Reverse duration in steps
    ]

    def __post_init__(self):
        # Enforce compute budget coupling
        if self.auto_derive_k:
            self.K = max(64, min(512, self.max_budget_kh // max(1, self.H)))
        else:
            if self.H * self.K > self.max_budget_kh:
                self.K = max(64, self.max_budget_kh // max(1, self.H))
        # Ensure mu_friction stays in strict [0.55, 0.85] range
        self.mu_friction = max(0.55, min(0.85, self.mu_friction))
        # Ensure r_inflation_m stays in [0.08, 0.13] range
        self.r_inflation_m = max(0.08, min(0.13, self.r_inflation_m))
        # Ensure ratio sum does not exceed 0.90 so right seed gets at least 10%
        sum_r = self.ratio_center + self.ratio_straight + self.ratio_left
        if sum_r > 0.90:
            scale = 0.90 / sum_r
            self.ratio_center *= scale
            self.ratio_straight *= scale
            self.ratio_left *= scale

    def to_array(self) -> np.ndarray:
        """Converts genome parameters into a flat 1D float array (27 genes)."""
        return np.array([
            float(self.H), float(self.K), float(self.dt),
            float(self.lambda_temp), float(self.ar1_alpha), float(self.gamma_discount),
            float(self.mu_friction), float(self.w_cte), float(self.w_heading),
            float(self.w_speed), float(self.w_hazard), float(self.w_prob_costmap),
            float(self.w_unknown), float(self.w_steer_rate), float(self.w_accel_rate),
            float(self.w_terminal), float(self.sigma_steer), float(self.sigma_speed),
            float(self.lognorm_shape), float(self.ratio_center), float(self.ratio_straight),
            float(self.ratio_left), float(self.pure_pursuit_lookahead), float(self.r_inflation_m),
            float(self.w_reverse), float(self.reverse_speed), float(self.t_rev_steps)
        ], dtype=np.float64)

    @classmethod
    def from_array(cls, arr: np.ndarray, enforce_budget: bool = True) -> "MPPIGenome":
        """Reconstructs MPPIGenome from a flat 1D array with type/bound/budget enforcement."""
        g = cls()
        for idx, (name, val_min, val_max, val_type) in enumerate(cls.GENE_SPECS):
            raw_val = arr[idx]
            clamped_val = max(val_min, min(val_max, raw_val))
            setattr(g, name, val_type(round(clamped_val) if val_type is int else clamped_val))
        if enforce_budget and (g.H * g.K > g.max_budget_kh):
            g.K = max(64, g.max_budget_kh // max(1, g.H))
        return g

    def to_dict(self, only_genes: bool = False) -> Dict[str, Any]:
        """Serializes genome to a JSON-ready dictionary."""
        d = asdict(self)
        if only_genes:
            gene_names = [spec[0] for spec in self.GENE_SPECS]
            return {k: d[k] for k in gene_names if k in d}
        return d

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "MPPIGenome":
        """Instantiates genome from dictionary."""
        filtered = {k: v for k, v in d.items() if hasattr(cls, k)}
        return cls(**filtered)

    def save_json(self, filepath: str):
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load_json(cls, filepath: str) -> "MPPIGenome":
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        return cls.from_dict(data)

    @classmethod
    def get_bounds(cls) -> List[Tuple[float, float]]:
        """Returns list of (min, max) bounds for each gene index."""
        return [(spec[1], spec[2]) for spec in cls.GENE_SPECS]


# ==============================================================================
# 2. VECTORIZED MULTI-MODAL MPPI ENGINE
# ==============================================================================
class MPPIController:
    """
    Ultra-Fast Vectorized MPPI Controller executing in < 8.0 ms on Raspberry Pi 5.
    
    Seamlessly integrates:
      - Multi-Modal Seed Paths (Warmstart, Left Lane +0.25m, Right Lane -0.25m)
      - Smooth-MPPI (AR-1 Colored Noise)
      - Log-MPPI (Normal Log-Normal Sampling for forward speed exploration)
      - Shield-MPPI (Early-Exit Costmap Collision Pruning)
      - CBF Friction Circle Traction Clamping (sqrt(ax^2 + ay^2) <= mu * g)
      - Physical Servo Slew Rate Limiting & Latency Compensation
      - Exponential Gamma-Discounting
    """

    def __init__(
        self,
        genome: Optional[MPPIGenome] = None,
        wheelbase: float = WHEELBASE,
        max_steer: float = MAX_STEERING_RAD,
        max_speed: float = MAX_SPEED_MPS,
        steer_rate_max: float = MAX_STEER_RATE_RAD_S,
    ):
        self.genome = genome if genome is not None else MPPIGenome()
        self.L = wheelbase
        self.max_steer = max_steer
        self.max_speed = max_speed
        self.steer_rate_max = steer_rate_max
        self.max_reverse_speed = 1.5  # Max reverse speed (m/s)

        # Cache dimensions
        self.H = self.genome.H
        self.K = self.genome.K
        self.dt = self.genome.dt

        # Control sequences & memory buffers
        # Nominal control sequence: shape (H, 2) -> [steering_rad, speed_mps]
        self.u_nominal = np.zeros((self.H, 2), dtype=np.float32)
        self.u_nominal[:, 1] = 2.5  # Baseline initial forward speed
        
        # Previous AR-1 noise states: shape (K, 2)
        self.noise_prev = np.zeros((self.K, 2), dtype=np.float32)
        
        # Precomputed discount vector: shape (H,) -> gamma^0, gamma^1, ..., gamma^(H-1)
        self._update_discount_cache()

        # Telemetry & diagnostics cache
        self.last_exec_time_ms = 0.0
        self.last_best_cost = 0.0
        self.last_collision_rate = 0.0
        self.last_seed_weights = {"center": 0.0, "left": 0.0, "right": 0.0}
        self.last_rollouts_cache: Optional[np.ndarray] = None  # (K, H+1, 2)

    def _update_discount_cache(self):
        """Precomputes gamma discount multipliers across the horizon."""
        gamma = self.genome.gamma_discount
        self.gamma_factors = np.array([gamma ** t for t in range(self.H)], dtype=np.float32)
        self.gamma_terminal = np.float32(gamma ** self.H)

    def set_genome(self, genome: MPPIGenome):
        """Updates controller hyperparameters and resizes buffers if H or K changed."""
        dim_changed = (genome.H != self.H or genome.K != self.K or genome.dt != self.dt)
        self.genome = genome
        self.H = genome.H
        self.K = genome.K
        self.dt = genome.dt

        if dim_changed:
            self.u_nominal = np.zeros((self.H, 2), dtype=np.float32)
            self.u_nominal[:, 1] = 2.5
            self.noise_prev = np.zeros((self.K, 2), dtype=np.float32)
            self._update_discount_cache()

    @staticmethod
    def _project_batch(
        frenet_helper: Any, xs: np.ndarray, ys: np.ndarray, curr_idx: int, search_window: int = 80
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Vectorized batch projection of (x, y) points onto trajectory in < 0.05 ms.
        Returns: (d_arr, t_yaw_arr)
        """
        n_pts = frenet_helper.n_pts
        offsets = np.arange(-search_window // 4, search_window)
        indices = (curr_idx + offsets) % n_pts

        tx_sub = frenet_helper.traj_x[indices, None]
        ty_sub = frenet_helper.traj_y[indices, None]

        d_sq = (tx_sub - xs[None, :]) ** 2 + (ty_sub - ys[None, :]) ** 2
        best_local = np.argmin(d_sq, axis=0)
        best_indices = indices[best_local]

        tx = frenet_helper.traj_x[best_indices]
        ty = frenet_helper.traj_y[best_indices]
        t_yaw = frenet_helper.traj_yaw[best_indices]

        dx = xs - tx
        dy = ys - ty
        d_arr = -dx * np.sin(t_yaw) + dy * np.cos(t_yaw)
        return d_arr, t_yaw

    # --------------------------------------------------------------------------
    # A. MULTI-MODAL SEED TRAJECTORY GENERATION (3 MODES)
    # --------------------------------------------------------------------------
    def _generate_seed_controls(
        self,
        rx: float,
        ry: float,
        ryaw: float,
        v_mps: float,
        frenet_helper: Any,
        curr_idx: int,
        frenet_left: Optional[Any] = None,
        frenet_right: Optional[Any] = None,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Computes 3 geometric nominal control sequences [delta_rad, v_mps] via Pure Pursuit:
          1. Center Seed: Warm-started optimal trajectory (or nominal centerline d=0)
          2. Left Seed: Pure Pursuit tracking left lane offset d = +0.25 m (or dedicated left line)
          3. Right Seed: Pure Pursuit tracking right lane offset d = -0.25 m (or dedicated right line)
        """
        H, dt, L = self.H, self.dt, self.L
        lookahead = self.genome.pure_pursuit_lookahead

        u_center = self.u_nominal.copy()
        u_left = np.zeros((H, 2), dtype=np.float32)
        u_right = np.zeros((H, 2), dtype=np.float32)

        # Baseline speeds
        target_v = max(1.5, min(self.max_speed, abs(v_mps) + 0.5))
        u_left[:, 1] = target_v
        u_right[:, 1] = target_v

        # If frenet_helper is available, compute analytic Pure Pursuit seeds
        if frenet_helper is not None:
            n_pts = frenet_helper.n_pts
            traj_s = frenet_helper.traj_s
            traj_x = frenet_helper.traj_x
            traj_y = frenet_helper.traj_y
            traj_yaw = frenet_helper.traj_yaw

            curr_s = traj_s[curr_idx % n_pts]
            
            for t in range(H):
                s_ahead = curr_s + max(0.15, (t + 1) * target_v * dt + lookahead)
                s_ahead_mod = s_ahead % frenet_helper.total_length
                
                # Closest station index for lookahead
                idx_ahead = int(np.searchsorted(traj_s, s_ahead_mod)) % n_pts
                tx = traj_x[idx_ahead]
                ty = traj_y[idx_ahead]
                tyaw = traj_yaw[idx_ahead]

                # Center Pure Pursuit
                dx_c = tx - rx
                dy_c = ty - ry
                alpha_c = math.atan2(dy_c, dx_c) - ryaw
                alpha_c = (alpha_c + math.pi) % (2.0 * math.pi) - math.pi
                dist_c = max(0.10, math.hypot(dx_c, dy_c))
                steer_c = math.atan2(2.0 * L * math.sin(alpha_c), dist_c)

                # Left Seed (+0.25m) - from dedicated left line or normal vector offset
                if frenet_left is not None:
                    idx_l = int(np.searchsorted(frenet_left.traj_s, s_ahead_mod % frenet_left.total_length)) % frenet_left.n_pts
                    tx_l = frenet_left.traj_x[idx_l]
                    ty_l = frenet_left.traj_y[idx_l]
                else:
                    nx = -math.sin(tyaw)
                    ny = math.cos(tyaw)
                    tx_l = tx + nx * 0.25
                    ty_l = ty + ny * 0.25

                dx_l = tx_l - rx
                dy_l = ty_l - ry
                alpha_l = math.atan2(dy_l, dx_l) - ryaw
                alpha_l = (alpha_l + math.pi) % (2.0 * math.pi) - math.pi
                dist_l = max(0.10, math.hypot(dx_l, dy_l))
                steer_l = math.atan2(2.0 * L * math.sin(alpha_l), dist_l)

                # Right Seed (-0.25m) - from dedicated right line or normal vector offset
                if frenet_right is not None:
                    idx_r = int(np.searchsorted(frenet_right.traj_s, s_ahead_mod % frenet_right.total_length)) % frenet_right.n_pts
                    tx_r = frenet_right.traj_x[idx_r]
                    ty_r = frenet_right.traj_y[idx_r]
                else:
                    nx = -math.sin(tyaw)
                    ny = math.cos(tyaw)
                    tx_r = tx - nx * 0.25
                    ty_r = ty - ny * 0.25

                dx_r = tx_r - rx
                dy_r = ty_r - ry
                alpha_r = math.atan2(dy_r, dx_r) - ryaw
                alpha_r = (alpha_r + math.pi) % (2.0 * math.pi) - math.pi
                dist_r = max(0.10, math.hypot(dx_r, dy_r))
                steer_r = math.atan2(2.0 * L * math.sin(alpha_r), dist_r)

                u_left[t, 0] = np.clip(steer_l, -self.max_steer, self.max_steer)
                u_right[t, 0] = np.clip(steer_r, -self.max_steer, self.max_steer)
                
                # If warmstart is zero/uninitialized, populate center seed with Pure Pursuit
                if abs(u_center[t, 0]) < 1e-4 and abs(u_center[t, 1]) < 0.1:
                    u_center[t, 0] = np.clip(steer_c, -self.max_steer, self.max_steer)
                    u_center[t, 1] = target_v
        else:
            # Fallback when no global racing line is supplied
            u_left[:, 0] = 0.15
            u_right[:, 0] = -0.15

        # Straight-Ahead Gap Shooter Seed (delta = 0.0 rad, constant forward race speed)
        u_straight = np.zeros((H, 2), dtype=np.float32)
        u_straight[:, 0] = 0.0  # Zero steering: punch straight ahead through gaps
        u_straight[:, 1] = target_v

        # Multi-Point Turn & Bidirectional Recovery Seeds (Dynamic Track-Oriented Cusps):
        if frenet_helper is not None:
            n_pts = frenet_helper.n_pts
            track_yaw_now = frenet_helper.traj_yaw[curr_idx % n_pts]
            yaw_err = (ryaw - track_yaw_now + math.pi) % (2.0 * math.pi) - math.pi
        else:
            yaw_err = 0.0

        t_rev = max(2, min(max(3, H // 3), int(self.genome.t_rev_steps)))
        rev_spd = -abs(float(self.genome.reverse_speed))
        fwd_rebound_speed = max(1.4, min(self.max_speed, abs(target_v) * 0.95))

        # Dynamic Track-Oriented 3-Point Turn Cusps:
        # Reversing swings the nose toward track tangent; forward phase accelerates downstream along optimal line
        u_cusp_left = np.zeros((H, 2), dtype=np.float32)
        u_cusp_left[:t_rev, 0] = self.max_steer
        u_cusp_left[:t_rev, 1] = rev_spd
        u_cusp_left[t_rev:, 0] = u_center[t_rev:, 0] if yaw_err <= 0 else -self.max_steer
        u_cusp_left[t_rev:, 1] = fwd_rebound_speed

        u_cusp_right = np.zeros((H, 2), dtype=np.float32)
        u_cusp_right[:t_rev, 0] = -self.max_steer
        u_cusp_right[:t_rev, 1] = rev_spd
        u_cusp_right[t_rev:, 0] = u_center[t_rev:, 0] if yaw_err >= 0 else self.max_steer
        u_cusp_right[t_rev:, 1] = fwd_rebound_speed

        # Pure Reverse Seeds (Short escape)
        u_rev_left = np.zeros((H, 2), dtype=np.float32)
        u_rev_left[:t_rev, 0] = self.max_steer
        u_rev_left[:t_rev, 1] = rev_spd

        u_rev_right = np.zeros((H, 2), dtype=np.float32)
        u_rev_right[:t_rev, 0] = -self.max_steer
        u_rev_right[:t_rev, 1] = rev_spd

        return u_center, u_straight, u_left, u_right, u_rev_left, u_rev_right, u_cusp_left, u_cusp_right

    # --------------------------------------------------------------------------
    # B. SMOOTH-MPPI (AR-1 NOISE) & LOG-MPPI (LOG-NORMAL VELOCITY SAMPLING)
    # --------------------------------------------------------------------------
    def _sample_perturbations(
        self,
        K_count: Optional[int] = None,
        scale_steer: float = 1.0,
        scale_speed: float = 1.0
    ) -> np.ndarray:
        """
        Generates temporally correlated colored noise perturbations across (K_count, H, 2).
        
        - Steering: AR-1 filtered Gaussian white noise
        - Velocity: AR-1 filtered Normal Log-Normal noise (+vx biased heavy-tailed exploration)
        """
        K = K_count if K_count is not None else self.K
        H = self.H
        alpha = self.genome.ar1_alpha
        beta = math.sqrt(max(0.0, 1.0 - alpha * alpha))
        
        sigma_steer = self.genome.sigma_steer * scale_steer
        sigma_speed = self.genome.sigma_speed * scale_speed
        shape_log = self.genome.lognorm_shape

        # 1. White noise innovations for steering: (K, H)
        xi_steer = np.random.normal(0.0, sigma_steer, size=(K, H)).astype(np.float32)

        # 2. Log-Normal forward innovations for speed: (K, H)
        z_speed = np.random.normal(0.0, shape_log, size=(K, H)).astype(np.float32)
        xi_speed = sigma_speed * (np.exp(z_speed - 0.5 * (shape_log ** 2)) - 1.0)

        # 3. Vectorized AR-1 Temporal Autoregressive Filtering
        eps = np.zeros((K, H, 2), dtype=np.float32)
        
        # Initial step t=0
        if self.noise_prev.shape[0] == K:
            eps[:, 0, 0] = alpha * self.noise_prev[:, 0] + beta * xi_steer[:, 0]
            eps[:, 0, 1] = alpha * self.noise_prev[:, 1] + beta * xi_speed[:, 0]
        else:
            eps[:, 0, 0] = beta * xi_steer[:, 0]
            eps[:, 0, 1] = beta * xi_speed[:, 0]

        for t in range(1, H):
            eps[:, t, 0] = alpha * eps[:, t - 1, 0] + beta * xi_steer[:, t]
            eps[:, t, 1] = alpha * eps[:, t - 1, 1] + beta * xi_speed[:, t]

        if K == self.K:
            self.noise_prev = eps[:, -1, :].copy()

        return eps

    def _apply_actuator_limits(
        self,
        u_samples: np.ndarray,
        active_steer_rad: float,
        slew_limit_step: float
    ):
        """Applies realistic physical servo slew rate clamping and steering/velocity saturation."""
        K_cnt, H, _ = u_samples.shape
        is_rev_0 = (u_samples[:, 0, 1] < -0.05)
        u_samples[~is_rev_0, 0, 0] = np.clip(
            u_samples[~is_rev_0, 0, 0],
            active_steer_rad - slew_limit_step,
            active_steer_rad + slew_limit_step
        )
        u_samples[:, 0, 0] = np.clip(u_samples[:, 0, 0], -self.max_steer, self.max_steer)
        u_samples[:, 0, 1] = np.clip(u_samples[:, 0, 1], -self.max_reverse_speed, self.max_speed)

        for t in range(1, H):
            is_rev_t = (u_samples[:, t, 1] < -0.05)
            u_samples[~is_rev_t, t, 0] = np.clip(
                u_samples[~is_rev_t, t, 0],
                u_samples[~is_rev_t, t - 1, 0] - slew_limit_step,
                u_samples[~is_rev_t, t - 1, 0] + slew_limit_step
            )
            u_samples[:, t, 0] = np.clip(u_samples[:, t, 0], -self.max_steer, self.max_steer)
            u_samples[:, t, 1] = np.clip(u_samples[:, t, 1], -self.max_reverse_speed, self.max_speed)

    # --------------------------------------------------------------------------
    # C. MAIN MPPI OPTIMIZATION CYCLE
    # --------------------------------------------------------------------------
    def compute_control(
        self,
        rx: float,
        ry: float,
        ryaw: float,
        v_mps: float,
        active_steer_rad: float,
        prob_map: np.ndarray,
        hazard_mask: np.ndarray,
        frenet_helper: Optional[Any] = None,
        curr_idx: int = 0,
        v_target_ref: float = 3.8,
        frenet_left: Optional[Any] = None,
        frenet_right: Optional[Any] = None,
    ) -> Tuple[float, float, Dict[str, Any]]:
        """
        Executes one full 60 Hz MPPI optimization cycle.
        
        Args:
            rx, ry: Robot world Cartesian position (m)
            ryaw: Robot world heading angle (rad)
            v_mps: Current linear velocity (m/s)
            active_steer_rad: Current active servo steering angle (rad)
            prob_map: 20 mm 101x101 local occupancy probability array [0.0, 1.0]
            hazard_mask: 20 mm 101x101 boolean instant collision array
            frenet_helper: Trajectory Frenet frame transformer for center line (optional)
            curr_idx: Closest waypoint index on nominal racing line
            v_target_ref: Nominal target race speed (m/s)
            frenet_left: Dedicated left offset (+25cm) trajectory helper (optional)
            frenet_right: Dedicated right offset (-25cm) trajectory helper (optional)
            
        Returns:
            optimal_steer_rad (float): Commanded Ackermann steering angle (rad)
            optimal_speed_mps (float): Commanded forward target speed (m/s)
            diagnostics (dict): Execution performance and telemetry metrics
        """
        t_start = time.perf_counter()
        K, H, dt, L = self.K, self.H, self.dt, self.L
        mu_g = self.genome.mu_friction * GRAVITY_MPS2
        slew_limit_step = self.steer_rate_max * dt

        # ----------------------------------------------------------------------
        # 1. 1-Tick Latency Compensation: Forward Project Initial State
        # ----------------------------------------------------------------------
        dt_lat = TRANSPORT_LATENCY_S
        x0 = rx + v_mps * math.cos(ryaw) * dt_lat
        y0 = ry + v_mps * math.sin(ryaw) * dt_lat
        yaw0 = ryaw + (v_mps / L) * math.tan(active_steer_rad) * dt_lat
        v0 = v_mps

        # ----------------------------------------------------------------------
        # 2. Multi-Modal Lane & Bidirectional Recovery Seeding Pool
        # ----------------------------------------------------------------------
        u_center, u_straight, u_left, u_right, u_rev_l, u_rev_r, u_cusp_l, u_cusp_r = self._generate_seed_controls(
            x0, y0, yaw0, v0, frenet_helper, curr_idx, frenet_left, frenet_right
        )

        # Check front bumper contact zone directly in local costmap (X: [-12cm, +12cm], Y: [0cm, 10cm])
        c_min = int(round((-0.12 - COSTMAP_X_MIN) / COSTMAP_RES))
        c_max = int(round((0.12 - COSTMAP_X_MIN) / COSTMAP_RES))
        r_min = int(round((0.00 - COSTMAP_Y_MIN) / COSTMAP_RES))
        r_max = int(round((0.10 - COSTMAP_Y_MIN) / COSTMAP_RES))
        bumper_contact = False
        if 0 <= r_min < r_max <= hazard_mask.shape[0] and 0 <= c_min < c_max <= hazard_mask.shape[1]:
            bumper_contact = bool(np.any(hazard_mask[r_min:r_max, c_min:c_max] > 0))

        # Dynamic Multi-Modal Seeding:
        # Scale recovery sample lines dynamically with the pruning / collision rate
        K_scout = max(96, K // 3)
        col_rate = float(self.last_collision_rate)
        if col_rate > 0.20:
            # As forward paths get pruned, dynamically expand reverse & cusp budget up to 45% of Scout
            rec_ratio = 0.05 + 0.40 * min(1.0, ((col_rate - 0.20) / 0.70) ** 1.5)
            n_rev = max(4, int(round(K_scout * rec_ratio * 0.5)))
            n_cusp = max(4, int(round(K_scout * rec_ratio * 0.5)))
        else:
            n_rev = 3
            n_cusp = 3
        n_recovery_total = 2 * n_rev + 2 * n_cusp

        # ======================================================================
        # 3. Reference Path Preparation for SIMD JIT
        # ======================================================================
        if frenet_helper is not None:
            s_cur = frenet_helper.traj_s[curr_idx % frenet_helper.n_pts]
            s_ahead = (s_cur + np.linspace(0.0, 3.0, 30)) % frenet_helper.total_length
            idx_ahead = np.searchsorted(frenet_helper.traj_s, s_ahead) % frenet_helper.n_pts
            ref_path = np.column_stack([
                frenet_helper.traj_x[idx_ahead],
                frenet_helper.traj_y[idx_ahead],
                frenet_helper.traj_yaw[idx_ahead],
            ]).astype(np.float32)
        else:
            ref_path = np.zeros((0, 3), dtype=np.float32)

        # ======================================================================
        # 4. ADAPTIVE 2-STAGE MPPI: Stage 1 (Multi-Modal Scout Phase)
        # ======================================================================
        k_fwd_scout = max(32, K_scout - n_recovery_total)
        n_c_scout = int(round(k_fwd_scout * self.genome.ratio_center))     # 40% Center
        n_s_scout = int(round(k_fwd_scout * self.genome.ratio_straight))   # 20% Straight
        n_l_scout = int(round(k_fwd_scout * self.genome.ratio_left))       # 20% Left
        n_r_scout = max(0, k_fwd_scout - n_c_scout - n_s_scout - n_l_scout) # 20% Right

        u_scout_base = np.zeros((K_scout, H, 2), dtype=np.float32)
        idx = 0
        if n_c_scout > 0:
            u_scout_base[idx:idx + n_c_scout] = u_center
            idx += n_c_scout
        if n_s_scout > 0:
            u_scout_base[idx:idx + n_s_scout] = u_straight
            idx += n_s_scout
        if n_l_scout > 0:
            u_scout_base[idx:idx + n_l_scout] = u_left
            idx += n_l_scout
        if n_r_scout > 0:
            u_scout_base[idx:idx + n_r_scout] = u_right
            idx += n_r_scout
        if n_rev > 0:
            u_scout_base[idx:idx + n_rev] = u_rev_l
            idx += n_rev
            u_scout_base[idx:idx + n_rev] = u_rev_r
            idx += n_rev
        if n_cusp > 0:
            u_scout_base[idx:idx + n_cusp] = u_cusp_l
            idx += n_cusp
            rem = K_scout - idx
            if rem > 0:
                u_scout_base[idx:idx + rem] = u_cusp_r

        # Sample colored exploratory perturbations for Stage 1 Scout
        eps_scout = self._sample_perturbations(K_count=K_scout, scale_steer=1.0, scale_speed=1.0)
        u_scout = u_scout_base + eps_scout
        self._apply_actuator_limits(u_scout, active_steer_rad, slew_limit_step)

        # Evaluate Stage 1 Scout in SIMD JIT
        _, _, _, _, _, costs_s, _, _, _ = evaluate_mppi_rollouts_jit(
            float(x0), float(y0), float(yaw0), float(v0),
            float(active_steer_rad), float(self.L), float(self.dt),
            float(self.max_steer), float(self.max_speed), float(mu_g),
            u_scout.astype(np.float32),
            prob_map.astype(np.float32),
            hazard_mask.astype(np.uint8),
            float(COSTMAP_X_MIN), float(COSTMAP_Y_MIN), float(COSTMAP_RES),
            ref_path, float(v_target_ref),
            self.gamma_factors.astype(np.float32),
            float(self.genome.w_cte), float(self.genome.w_heading),
            float(self.genome.w_speed), float(self.genome.w_hazard),
            float(self.genome.w_prob_costmap), float(self.genome.w_unknown),
            float(self.genome.w_steer_rate), float(self.genome.w_accel_rate),
            float(self.genome.w_terminal), float(self.gamma_terminal),
            float(self.genome.lambda_temp),
            float(self.genome.w_reverse),
            float(-abs(self.genome.reverse_speed))
        )

        # ======================================================================
        # 5. ADAPTIVE 2-STAGE MPPI: Stage 2 (Dynamic Apex & Recovery Refinement)
        # ======================================================================
        haz_thresh = self.genome.w_hazard * 0.80
        i_c0, i_c1 = 0, n_c_scout
        i_s0, i_s1 = i_c1, i_c1 + n_s_scout
        i_l0, i_l1 = i_s1, i_s1 + n_l_scout
        i_r0, i_r1 = i_l1, k_fwd_scout
        i_rev0, i_rev1 = k_fwd_scout, K_scout

        surv_c = float(np.mean(costs_s[i_c0:i_c1] < haz_thresh)) if n_c_scout > 0 else 0.0
        surv_s = float(np.mean(costs_s[i_s0:i_s1] < haz_thresh)) if n_s_scout > 0 else 0.0
        surv_l = float(np.mean(costs_s[i_l0:i_l1] < haz_thresh)) if n_l_scout > 0 else 0.0
        surv_r = float(np.mean(costs_s[i_r0:i_r1] < haz_thresh)) if n_r_scout > 0 else 0.0
        surv_rev = float(np.mean(costs_s[i_rev0:i_rev1] < haz_thresh)) if i_rev1 > i_rev0 else 0.0

        best_cost_c = float(np.min(costs_s[i_c0:i_c1])) if n_c_scout > 0 else 1e9
        best_cost_s = float(np.min(costs_s[i_s0:i_s1])) if n_s_scout > 0 else 1e9
        best_cost_l = float(np.min(costs_s[i_l0:i_l1])) if n_l_scout > 0 else 1e9
        best_cost_r = float(np.min(costs_s[i_r0:i_r1])) if n_r_scout > 0 else 1e9
        best_cost_rev = float(np.min(costs_s[i_rev0:i_rev1])) if i_rev1 > i_rev0 else 1e9

        best_u_c = u_scout[i_c0 + int(np.argmin(costs_s[i_c0:i_c1]))] if n_c_scout > 0 else u_center
        best_u_s = u_scout[i_s0 + int(np.argmin(costs_s[i_s0:i_s1]))] if n_s_scout > 0 else u_straight
        best_u_l = u_scout[i_l0 + int(np.argmin(costs_s[i_l0:i_l1]))] if n_l_scout > 0 else u_left
        best_u_r = u_scout[i_r0 + int(np.argmin(costs_s[i_r0:i_r1]))] if n_r_scout > 0 else u_right
        best_u_rev = u_scout[i_rev0 + int(np.argmin(costs_s[i_rev0:i_rev1]))] if i_rev1 > i_rev0 else u_cusp_l

        # Dynamic channel preference scoring across all channels
        score_c = (surv_c ** 2) / max(100.0, best_cost_c) if surv_c >= 0.15 else 0.0
        score_s = (surv_s ** 2) / max(100.0, best_cost_s) if surv_s >= 0.15 else 0.0
        score_l = (surv_l ** 2) / max(100.0, best_cost_l) if surv_l >= 0.15 else 0.0
        score_r = (surv_r ** 2) / max(100.0, best_cost_r) if surv_r >= 0.15 else 0.0
        tot_score_fwd = score_c + score_s + score_l + score_r

        K_refine = max(64, K - K_scout)
        u_refine_base = np.zeros((K_refine, H, 2), dtype=np.float32)

        if tot_score_fwd > 1e-9:
            # At least one forward channel is open -> refine forward channels
            w_c_ref = score_c / tot_score_fwd
            w_s_ref = score_s / tot_score_fwd
            w_l_ref = score_l / tot_score_fwd
            w_r_ref = score_r / tot_score_fwd
            n_c_ref = int(round(K_refine * w_c_ref))
            n_s_ref = int(round(K_refine * w_s_ref))
            n_l_ref = int(round(K_refine * w_l_ref))
            n_r_ref = max(0, K_refine - n_c_ref - n_s_ref - n_l_ref)
            n_rev_ref = 0

            idx_r = 0
            if n_c_ref > 0:
                u_refine_base[idx_r:idx_r + n_c_ref] = best_u_c
                idx_r += n_c_ref
            if n_s_ref > 0:
                u_refine_base[idx_r:idx_r + n_s_ref] = best_u_s
                idx_r += n_s_ref
            if n_l_ref > 0:
                u_refine_base[idx_r:idx_r + n_l_ref] = best_u_l
                idx_r += n_l_ref
            if n_r_ref > 0:
                u_refine_base[idx_r:idx_r + n_r_ref] = best_u_r
                idx_r += n_r_ref
        else:
            # ALL forward channels are blocked by walls -> spend 100% of Stage 2 on reverse escape!
            n_c_ref, n_s_ref, n_l_ref, n_r_ref = 0, 0, 0, 0
            n_rev_ref = K_refine
            u_refine_base[:] = best_u_rev

        # High-density fine-grain perturbations around surviving apex lines (tight noise)
        eps_refine = self._sample_perturbations(K_count=K_refine, scale_steer=0.50, scale_speed=0.55)
        u_refine = u_refine_base + eps_refine
        self._apply_actuator_limits(u_refine, active_steer_rad, slew_limit_step)

        # Combine Stage 1 (scout) and Stage 2 (dense refinement) into complete K-sample pool
        u_samples = np.vstack([u_scout, u_refine])

        # ----------------------------------------------------------------------
        # 6. Final Unified Native LLVM SIMD Multi-Core Rollouts (<0.5 ms)
        # ----------------------------------------------------------------------
        u_opt, xs, ys, yaws, vs, costs, weights, best_k, collision_rate = evaluate_mppi_rollouts_jit(
            float(x0), float(y0), float(yaw0), float(v0),
            float(active_steer_rad), float(self.L), float(self.dt),
            float(self.max_steer), float(self.max_speed), float(mu_g),
            u_samples.astype(np.float32),
            prob_map.astype(np.float32),
            hazard_mask.astype(np.uint8),
            float(COSTMAP_X_MIN), float(COSTMAP_Y_MIN), float(COSTMAP_RES),
            ref_path, float(v_target_ref),
            self.gamma_factors.astype(np.float32),
            float(self.genome.w_cte), float(self.genome.w_heading),
            float(self.genome.w_speed), float(self.genome.w_hazard),
            float(self.genome.w_prob_costmap), float(self.genome.w_unknown),
            float(self.genome.w_steer_rate), float(self.genome.w_accel_rate),
            float(self.genome.w_terminal), float(self.gamma_terminal),
            float(self.genome.lambda_temp),
            float(self.genome.w_reverse),
            float(-abs(self.genome.reverse_speed))
        )

        min_cost = costs[best_k]
        is_active = (costs < (self.genome.w_hazard * 0.9))

        # ----------------------------------------------------------------------
        # 7. Trajectory Warm-Starting for Next Frame (60 Hz Continuity)
        # ----------------------------------------------------------------------
        self.u_nominal[:-1, :] = u_opt[1:, :].copy()
        self.u_nominal[-1, :] = u_opt[-1, :].copy()

        # Commanded immediate actions for step t=0 (supports reverse recovery)
        cmd_steer = float(np.clip(u_opt[0, 0], -self.max_steer, self.max_steer))
        cmd_speed = float(np.clip(u_opt[0, 1], -self.max_reverse_speed, self.max_speed))

        # Diagnostics & Profiling
        exec_ms = (time.perf_counter() - t_start) * 1000.0
        self.last_exec_time_ms = exec_ms
        self.last_best_cost = float(min_cost)
        
        collision_count = int(np.sum(~is_active))
        self.last_collision_rate = float(collision_count) / float(K)

        # Seed pool importance weight allocation across Scout and Refinement pools
        ref_off_c = K_scout
        ref_off_s = ref_off_c + n_c_ref
        ref_off_l = ref_off_s + n_s_ref
        ref_off_r = ref_off_l + n_l_ref

        w_center = float(np.sum(weights[i_c0:i_c1]) + (np.sum(weights[ref_off_c:ref_off_s]) if n_c_ref > 0 else 0.0))
        w_straight = float(np.sum(weights[i_s0:i_s1]) + (np.sum(weights[ref_off_s:ref_off_l]) if n_s_ref > 0 else 0.0))
        w_left = float(np.sum(weights[i_l0:i_l1]) + (np.sum(weights[ref_off_l:ref_off_r]) if n_l_ref > 0 else 0.0))
        w_right = float(np.sum(weights[i_r0:i_r1]) + (np.sum(weights[ref_off_r:]) if n_r_ref > 0 else 0.0))
        w_reverse = float(np.sum(weights[k_fwd_scout:K_scout])) if K_scout > k_fwd_scout else 0.0
        self.last_seed_weights = {
            "center": w_center,
            "straight": w_straight,
            "left": w_left,
            "right": w_right,
            "reverse": w_reverse
        }

        # Cache rollout preview for visual debugging / supervisor inspection
        self.last_rollouts_cache = np.stack([xs, ys], axis=-1)  # (K, H+1, 2)

        # Convert best trajectory, nominal seeds, and sample rollouts to vehicle local frame for GUI overlay
        cos_yaw0 = math.cos(ryaw)
        sin_yaw0 = math.sin(ryaw)

        # 1. Best Selected Trajectory
        best_idx = int(np.argmin(costs))
        bx = xs[best_idx] - x0
        by = ys[best_idx] - y0
        b_fwd = bx * cos_yaw0 + by * sin_yaw0
        b_lat = bx * sin_yaw0 - by * cos_yaw0
        best_traj_local = [[round(float(b_lat[i]), 3), round(float(b_fwd[i]), 3)] for i in range(H + 1)]

        # 2. Distinct SVG-MPPI Nominal Seed Lines (Center, Straight, Left +25cm, Right -25cm)
        # Center Seed (Racing Line)
        c_x = xs[0] - x0
        c_y = ys[0] - y0
        c_fwd = c_x * cos_yaw0 + c_y * sin_yaw0
        c_lat = c_x * sin_yaw0 - c_y * cos_yaw0
        seed_center_local = [[round(float(c_lat[i]), 3), round(float(c_fwd[i]), 3)] for i in range(H + 1)]

        # Straight Seed (Gap Shooter delta = 0.0)
        s_idx = min(i_s0, K - 1)
        s_x = xs[s_idx] - x0
        s_y = ys[s_idx] - y0
        s_fwd = s_x * cos_yaw0 + s_y * sin_yaw0
        s_lat = s_x * sin_yaw0 - s_y * cos_yaw0
        seed_straight_local = [[round(float(s_lat[i]), 3), round(float(s_fwd[i]), 3)] for i in range(H + 1)]

        # Left Seed (+0.25m offset)
        l_idx = min(i_l0, K - 1)
        l_x = xs[l_idx] - x0
        l_y = ys[l_idx] - y0
        l_fwd = l_x * cos_yaw0 + l_y * sin_yaw0
        l_lat = l_x * sin_yaw0 - l_y * cos_yaw0
        seed_left_local = [[round(float(l_lat[i]), 3), round(float(l_fwd[i]), 3)] for i in range(H + 1)]

        # Right Seed (-0.25m offset)
        r_idx = min(i_r0, K - 1)
        r_x = xs[r_idx] - x0
        r_y = ys[r_idx] - y0
        r_fwd = r_x * cos_yaw0 + r_y * sin_yaw0
        r_lat = r_x * sin_yaw0 - r_y * cos_yaw0
        seed_right_local = [[round(float(r_lat[i]), 3), round(float(r_fwd[i]), 3)] for i in range(H + 1)]

        # 3. Sample 20 candidate rollouts across all seeds for exploration fan visualization
        sample_indices = np.linspace(0, K - 1, min(20, K), dtype=int)
        cand_trajs_local = []
        for s_idx in sample_indices:
            cx = xs[s_idx] - x0
            cy = ys[s_idx] - y0
            c_f = cx * cos_yaw0 + cy * sin_yaw0
            c_l = cx * sin_yaw0 - cy * cos_yaw0
            cand_trajs_local.append([[round(float(c_l[i]), 3), round(float(c_f[i]), 3)] for i in range(H + 1)])

        diagnostics = {
            "exec_time_ms": exec_ms,
            "best_cost": float(min_cost),
            "collision_rate": self.last_collision_rate,
            "seed_weights": self.last_seed_weights,
            "cmd_steer_rad": cmd_steer,
            "cmd_speed_mps": cmd_speed,
            "active_samples": int(np.sum(is_active)),
            "best_traj_local": best_traj_local,
            "seed_center_local": seed_center_local,
            "seed_straight_local": seed_straight_local,
            "seed_left_local": seed_left_local,
            "seed_right_local": seed_right_local,
            "cand_trajs_local": cand_trajs_local,
        }

        return cmd_steer, cmd_speed, diagnostics

    def reset(self):
        """Resets nominal trajectory and noise buffers."""
        self.u_nominal = np.zeros((self.H, 2), dtype=np.float32)
        self.u_nominal[:, 1] = 2.5
        self.noise_prev = np.zeros((self.K, 2), dtype=np.float32)
