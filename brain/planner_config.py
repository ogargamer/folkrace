"""
================================================================================
  AUTONOMOUS TOF PATH PLANNERS: UNIFIED CONFIGURATION & TUNING PARAMETERS
================================================================================
  Single source of truth for:
    1. Physical Vehicle Dimensions, Kinematics & Actuator Limits
    2. Tire Friction & Vehicle Dynamic Traction Envelopes
    3. Sensors, Costmap & Timing (60 Hz ToF & Pi Planning Loop)
    4. Gap Scouting, Aperture Sizing & Selection Weights
    5. Curvilinear Slack-Constrained SL-MPC (Native C++ / OSQP)
    6. Non-Blocking Stuck Detection & Reverse Recovery Manager
    7. Anti-Wrong-Way Watchdog & Early Gap Exit Sentry
================================================================================
"""

import os
import sys
import json
import math
from typing import Any, Dict, List, Optional, Tuple

# Ensure repository root is on sys.path for brain.vehicle_params access
_CURR_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_CURR_DIR, ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from brain.vehicle_params import (
    REAR_AXLE_OFFSET_M,
    FRONT_AXLE_OFFSET_M,
    WHEELBASE,
    TRACK_REAR,
    TRACK_FRONT,
    TRACK_KINGPIN,
    WHEEL_RADIUS,
    VEHICLE_MASS,
    ACKERMANN_PERCENT,
    MAX_SERVO_RAD,
    MAX_STEERING_RAD,
    SERVO_MAX_SPEED_RAD_S,
    MOTOR_DIR,
    V_MAX_WHEEL,
    V_MAX_CRUISE,
    TIRE_MU,
    GRAVITY_G,
    A_MAX_BASE,
    ACCEL_FWD_RATIO,
    BRAKE_DECEL_RATIO,
    LAT_ACCEL_RATIO,
    ACCEL_FWD_MAX,
    BRAKE_DECEL_MAX,
    LAT_ACCEL_MAX,
    R_MIN_TURN,
    V_MIN_CRUISE,
    SLIP_GATE_SPEED_MPS,
    SLIP_THRESH_SPIN_RAD_S,
    TOF_HZ,
    PI_LOOP_HZ,
    DT_PI_LOOP,
    CONTROL_LOOP_FREQ_HZ,
    CONTROL_LOOP_DT_S,
    SIM_TIME_STEP_MS,
    UART_BAUD,
    GRID_X_MAX,
    GRID_X_MIN,
    GRID_Y_MIN,
    GRID_Y_MAX,
    GRID_RES,
    R_INFLATION_M,
)

# ==============================================================================
# ACTIVE PLANNER CONFIGURATION
# ==============================================================================
PLANNER_TYPE: str = "SL_MPC"


# ==============================================================================
# 1. PHYSICAL VEHICLE, SENSORS & TIMING
# (Unified and synchronized from brain/vehicle_config.json via brain.vehicle_params)
# ==============================================================================

# Simulation Physics & Actuator Step Timing
TIME_STEP_MS: float = float(SIM_TIME_STEP_MS)  # Webots basicTimeStep in ms (matches world file: 8.333333 ms = 120.0 Hz)
DT: float = TIME_STEP_MS / 1000.0              # Physics & fast actuator/IMU step duration (0.00833333 s = 1/120 s)
SIM_HZ: float = 1000.0 / TIME_STEP_MS          # Simulation physics, 4WD motors, steering, and IMU rate (120.0 Hz)

PLANNER_DECIMATION: int = 1                    # Run planner every physics step in simulation
TOF_TIME_STEP_MS: float = TIME_STEP_MS * PLANNER_DECIMATION  # Sim ToF sampling interval in ms
DT_PLANNER: float = DT_PI_LOOP                 # Canonical planner cycle duration (0.0166667 s = 60.0 Hz)
PLANNER_HZ: float = PI_LOOP_HZ                 # ToF scan processing & trajectory planning rate (60.0 Hz)

# Realistic Hardware Latency & Lookahead Compensation (Time-Based)
TOF_LATENCY_S: float = 0.01667          # 16.67 ms (nominal 1 frame @ 60 Hz ToF integration & transport delay)
ACTUATOR_LATENCY_S: float = 0.00800     # 8.0 ms (UART transmission + servo physical response lag)
TOTAL_LATENCY_S: float = TOF_LATENCY_S + ACTUATOR_LATENCY_S  # 24.67 ms total latency

# Discrete step counts at current DT (120 Hz in simulation = 8.333 ms):
TOF_LATENCY_STEPS: int = max(1, int(round(TOF_LATENCY_S / DT)))        # 2 steps = 16.67 ms
ACTUATOR_LATENCY_STEPS: int = max(1, int(round(ACTUATOR_LATENCY_S / DT))) # 1 step = 8.33 ms
TOTAL_LATENCY_STEPS: int = TOF_LATENCY_STEPS + ACTUATOR_LATENCY_STEPS      # 3 steps = 25.0 ms

# 3.3 Sensor Device Names & Measurement Limits (16-Channel Front Rig: only_front_tofs.proto)
TOF_SENSOR_NAMES: List[str] = [
    # Front-Left Array (cross-pointing Right, 8 sensors)
    "tof_front_left_4.1deg",
    "tof_front_left_15.4deg",
    "tof_front_left_26.6deg",
    "tof_front_left_37.9deg",
    "tof_front_left_49.1deg",
    "tof_front_left_60.4deg",
    "tof_front_left_71.6deg",
    "tof_front_left_82.9deg",
    # Front-Right Array (cross-pointing Left, 8 sensors)
    "tof_front_right_4.1deg",
    "tof_front_right_15.4deg",
    "tof_front_right_26.6deg",
    "tof_front_right_37.9deg",
    "tof_front_right_49.1deg",
    "tof_front_right_60.4deg",
    "tof_front_right_71.6deg",
    "tof_front_right_82.9deg",
]
TOF_MAX_RANGE_M: float = 2.40          # meters maximum valid ToF range
TOF_MIN_RANGE_M: float = 0.001         # meters minimum valid ToF range

# Wheel Encoders
ENCODER_NAMES: Dict[str, str] = {
    "FL": "encoder FL",
    "FR": "encoder FR",
    "RL": "encoder RL",
    "RR": "encoder RR",
}

# 3.4 Dynamic Moving Obstacles (On Racing Line)
MOVING_OBSTACLE_NAMES: List[str] = ["MOVING_OBSTACLE", "MOVING_OBSTACLE_2", "MOVING_OBSTACLE_3"]
MOVING_OBSTACLE_SPEED_MPS: float = 1.00     # m/s pace along racing line (default: 1.00 m/s)
MOVING_OBSTACLE_DIRECTION: int = 1          # Direction along trajectory: +1 (Forward) or -1 (Reverse)
MOVING_OBSTACLE_Z_ELEVATION: float = 0.075  # meters elevation above track ground


# ==============================================================================
# 4. GAP SCOUTING & SELECTION WEIGHTS
# ==============================================================================

# 4.1 Gap Aperture & Lookahead Constraints
GAP_MIN_WIDTH_M: float = 0.0           # meters minimum passable aperture width
GAP_MAX_LOOKAHEAD_M: float = 2.30      # meters maximum lookahead distance for gap target
GAP_MIN_LOOKAHEAD_M: float = 0.20      # meters minimum lookahead distance

# 4.2 Candidate Evaluation Weights (GA Champion)
# Score = W_DIST * dist_factor + W_WIDTH * (width/max_w) - W_DEV_STEER * |theta - steer| - W_DEV_STRAIGHT * |theta|
WEIGHT_GAP_DISTANCE: float = 7.026     # Preference for longer/further clear gaps (GA Champion)
WEIGHT_GAP_WIDTH: float = 2.047        # Preference for wider open apertures
WEIGHT_ALIGN_STEER: float = 1.681      # Preference for turning toward where servo is already aiming
WEIGHT_ALIGN_STRAIGHT: float = 0.998   # Preference for going straight when openings are equal
WEIGHT_SIDE_CLEARANCE: float = 2.547   # Preference for steering away from close walls / side asymmetry

# 4.3 Exponential Distance Preference (1.0m Walled Track Geometry)
# Sightlines exceeding GAP_DIST_THRESH_M receive exponential distance reward
TRACK_CORRIDOR_WIDTH_M: float = 1.00   # meters physical track corridor width
GAP_DIST_THRESH_M: float = 0.95        # meters threshold distance to engage exponential gap reward
GAP_DIST_EXP_K: float = 0.717          # Exponential growth scaling factor for distance beyond threshold

# 4.4 Dual-Path Candidate Selection & Smart Challenger Arbitration
ALT_MIN_SEPARATION_M: float = 0.052    # meters minimum lateral separation between primary and challenger targets
ALT_APERTURE_BIAS: float = 1.553       # Multiplier on aperture width reward for challenger target
COSTMAP_PERSISTENCE_FRAMES: int = 0    # Body-frame obstacle persistence frames (0 = zero ghost smear)
GAP_USE_TRUE_APERTURE: int = 1         # 1 = real 180-ray angular gap width, 0 = legacy 0.30*dist
WEIGHT_GAP_CENTER: float = 0.832       # Corridor edge-clearance centering weight in gap scout


# ==============================================================================
# 5. CURVILINEAR SLACK-CONSTRAINED SL-MPC (OSQP / C++ NATIVE)
# ==============================================================================

# 5.1 Prediction Horizon & Timestep
MPC_HORIZON_N: int = 30                # Prediction horizon steps (fixed at 30 in native QP sparsity table)
MPC_DT: float = 1.0 / 60.0             # seconds per horizon step (exact 60.0 Hz planning loop: ~0.016667 s)
MPC_MAX_ITER: int = 25                 # Primary OSQP iteration cap (converged, ~0.47 ms mean latency)

# 5.2 Cost Function Weights (Quadratic & Linear OSQP Penalties, GA Champion)
MPC_Q_PROGRESS: float = 23.108         # Weight encouraging forward arc-length progress
MPC_Q_CTE: float = 54.342              # Weight penalizing cross-track deviation from guide line
MPC_Q_HEADING: float = 9.685           # Weight penalizing heading error
MPC_R_STEER: float = 4.944             # Weight penalizing steering angle magnitude
MPC_R_ACCEL: float = 3.507             # Weight penalizing longitudinal acceleration/braking effort
MPC_R_STEER_RATE: float = 12.310       # Weight penalizing steering rate slew
MPC_CENTER_BIAS: float = 0.0           # Corridor centering blend in [0.0, 1.0] (0=guide line, 1=corridor midpoint)

# 5.3 Traction Circle & Dynamic Speeds
MPC_MU_FRICTION: float = TIRE_MU       # Tire friction coefficient governing grip envelope (0.86)
MPC_V_CORNER_FACTOR: float = 0.843     # Friction circle cornering speed scaling factor
MPC_V_CRUISE_MAX: float = V_MAX_CRUISE # m/s top cruising speed ceiling (5.0 m/s)
MPC_V_CRUISE_MIN: float = V_MIN_CRUISE # m/s cornering / crawl speed floor (~1.20 m/s)

# 5.4 Challenger Aliases
MPC_ALT_SWITCH_HYSTERESIS: float = 24.484                     # Cost reduction needed for challenger overthrow
MPC_ALT_MIN_SEPARATION_M: float = ALT_MIN_SEPARATION_M        # Minimum lateral separation between paths (m)
MPC_ALT_APERTURE_BIAS: float = ALT_APERTURE_BIAS              # Challenger aperture width bias multiplier
MPC_W_GAP_CLEAR: float = 10.0                                 # Lookahead depth credit & clear horizon weight (m)

# 5.5 Track-Aware Cornering & Corridor Flow Parameters (Neutral Defaults = Baseline)
MPC_USE_CORRIDOR_FLOW: int = 0                                # 1 = corridor-aware blockage & narrow-gap flow, 0 = baseline
MPC_TRACK_WIDTH_CAP_M: float = 1.25                           # One-sided track-width corridor clamp (m; >=1.25 off)
MPC_CORNER_OUTER_BIAS: float = 0.0                            # Soft outside-entry bias inside free corridor on turns
MPC_INNER_CORNER_MARGIN_M: float = 0.0                        # Inner-apex kinematic turn-radius clearance margin (m)
MPC_BLIND_TURN_KAPPA: float = 0.0                             # Blind 90-deg corner curvature floor (1/m)
MPC_BRAKE_DECEL_SCALE: float = 1.0                            # Backward braking deceleration multiplier
MPC_NARROW_CORRIDOR_V_GAIN: float = 0.0                       # Speed moderation in narrow (<0.48m) corridors


# ==============================================================================
# 6. NON-BLOCKING STUCK & REVERSE RECOVERY MANAGER
# ==============================================================================

# 6.1 Recovery Kinematics & Speed Limits
RECOVERY_STEER_ANGLE_RAD: float = MAX_STEERING_RAD     # radians (~32.22 deg) steering lock during reverse arc
RECOVERY_REVERSE_SPEED_MPS: float = V_MIN_CRUISE       # m/s dynamic cornering speed floor (~1.33 m/s)
RECOVERY_CORRECTIVE_180_SPEED_MPS: float = 0.9349     # m/s controlled forward U-turn speed well below TIRE_MU*g
RECOVERY_SERVO_SLEW_RAD_S: float = SERVO_MAX_SPEED_RAD_S  # rad/s centering slew rate (15.4 rad/s at 8.4V)
RECOVERY_ARC_LOOKAHEAD_M: float = 0.146                # meters length of kinematic escape arcs projected on costmap

# 6.2 Stall Detection Triggers & Failsafe Timeouts
RECOVERY_CLEARANCE_EXIT_M: float = 0.25      # meters (25 cm) minimum forward clearance to conclude reverse
RECOVERY_REVERSE_STROKE_MAX_S: float = 1.0  # seconds maximum reverse stroke failsafe
RECOVERY_TOTAL_TIMEOUT_S: float = 1.2       # seconds maximum total recovery duration before forced exit
RECOVERY_EXIT_MIN_TIME_S: float = 0.0        # seconds minimum recovery stroke time before clearance check
RECOVERY_180_TIMEOUT_S: float = 2.5          # seconds timeout for forward-only corrective 180 turn
RECOVERY_180_TIMEOUT_WITH_REV_S: float = 3.5 # seconds timeout for corrective 180 when reverse support engaged
RECOVERY_180_REV_CLEAR_M: float = 0.4208     # meters forward clearance required to exit CORRECTIVE_180[REVERSE]
RECOVERY_180_REV_MIN_S: float = 0.1361       # seconds minimum reverse duration in CORRECTIVE_180[REVERSE]
RECOVERY_180_ALIGN_DEG: float = 30.7898      # degrees target alignment tolerance for CORRECTIVE_180 exit
RECOVERY_180_EARLY_EXIT_TURN_DEG: float = 95.0  # degrees minimum net turn before allowing early exit

# 6.3 Multi-Criteria Exit Conditions
RECOVERY_ANY_TOF_EXIT_M: float = 0.60        # meters (60 cm) minimum clearance on at least one ToF to conclude reverse


# ==============================================================================
# 7. ANTI-WRONG-WAY WATCHDOG & EARLY GAP SENTRY
# ==============================================================================

# 7.1 Watchdog Mode & Breadcrumb History
ANTI_WRONG_WAY_MODE: str = "OPTION_1_BREADCRUMB"
AWW_BREADCRUMB_LOOKBACK_M: float = 8.0         # meters baseline historical path retained
AWW_BREADCRUMB_DURATION_SCALE: float = 0.35    # scale factor for breadcrumb fade (0.35x * 8.0m = 2.8m effective window)
AWW_BREADCRUMB_SPACING_M: float = 0.05         # meters minimum spatial spacing between breadcrumb nodes
AWW_BREADCRUMB_MAX_TIMEOUT_S: float = 6.0      # seconds maximum timeout before breadcrumb history expires
AWW_CONFIRM_TIME_S: float = 0.0543             # seconds continuous confirmation (~3-4 frames @ 60 Hz)
AWW_POST_RECOVERY_COOLDOWN_S: float = 0.3703   # seconds cooldown after CORRECTIVE_180 before re-arming
AWW_MIN_CONFIRMED_BREADCRUMBS: int = 4         # minimum confirmed opposing LOS breadcrumbs required
AWW_MAX_DETECT_DIST_M: float = 2.223           # meters forward LOS search envelope (within 2.4m ToF / 2.6m costmap)
AWW_MAX_LAT_XV_M: float = 0.538                # meters lateral in-corridor bound |lat_bc| for wrong-way LOS matching

# 7.2 Narrow-Cone Early Gap Exit Sentry
AWW_EARLY_EXIT_MIN_GAP_M: float = 1.4          # meters minimum scouted gap distance to qualify as long-path early exit
AWW_EARLY_EXIT_CONE_DEG: float = 30.0          # degrees total narrow cone width (±15 deg) oriented along candidate gap vector
AWW_EARLY_EXIT_OPPOSING_DEG: float = 110.0     # degrees heading divergence threshold for opposing breadcrumbs in gap corridor
AWW_EARLY_EXIT_MIN_BC_DIST_M: float = 0.15     # meters minimum distance from robot to evaluate opposing breadcrumbs in gap corridor (starts 15cm out)
AWW_EARLY_EXIT_CORRIDOR_HALF_WIDTH_M: float = 0.35  # meters max lateral corridor half-width for early-exit capsule
AWW_EARLY_EXIT_MIN_HALF_WIDTH_M: float = 0.08       # meters min close-range lateral half-width for early-exit capsule


# ==============================================================================
# 8. DRIFT & TRACTION CONTROL (REAR OPTICAL FLOW + ACTIVE FRONT STEERING)
# ==============================================================================

ENABLE_OPTICAL_FLOW_SENSOR: bool = False
ENABLE_TRACTION_DRIFT_CONTROL: bool = False

# 8.1 Speed Activation Gates & Hysteresis (m/s)
# Armed at or above dynamic cornering floor V_MIN_CRUISE (~1.204 m/s)
V_ARM_DRIFT_MPS: float = V_MIN_CRUISE          # ~1.204 m/s entry threshold
V_DISARM_CRAWL_MPS: float = 0.60               # Disarm only if speed drops below this AND tires have gripped
V_CRAWL_FLOOR_MPS: float = 0.40                # Floor of the linear fade ramp (zero countersteer at or below)

# 8.2 Slip Angle Thresholds (Radians)
BETA_SCRUB_RAD: float = math.radians(3.5)      # ~0.061 rad (linear tire limit)
BETA_DRIFT_RAD: float = math.radians(7.5)      # ~0.131 rad (saturated drift entry)
BETA_SPIN_RAD: float = math.radians(18.0)      # ~0.314 rad (critical spinout danger)

# 8.3 Active Countersteering & Anti-Snap Parameters
K_BETA: float = 1.10                           # Proportional countersteer gain on rear slip angle
K_YAW_RATE: float = 0.06                       # Damping against measured body yaw rate (rad/s -> rad steer)
K_YAW_DOT: float = 0.015                       # Derivative damping against yaw snap acceleration
K_RECOVERY_RELAX: float = 0.45                 # Damping relaxation when slip angle is collapsing (0.45 * K_BETA)
ANTI_SNAP_HOLD_TIME_S: float = 0.080           # 80 ms stateful hold latch for Schmitt trigger
D_BETA_LP_ALPHA: float = 0.43                  # Low-pass filter smoothing factor on slip rate (~15 Hz cutoff at 125 Hz)

# 8.4 Front-Axle Braking Lateral Reserve
FRONT_LATERAL_RESERVE_MIN: float = 0.40        # Minimum lateral friction fraction reserved for front steering during rear coast

# 8.5 Dynamic MU Scaling (Tire Kinetic Friction Reduction)
ENABLE_DYNAMIC_MU_SCALING: bool = False
MU_SLIDE_RATIO: float = 0.85                   # Minimum friction ratio (1.0 - 0.15 = 0.85 at max slip)
MU_RECOVERY_RATE: float = 2.0                  # Rate at which effective MU recovers back to nominal (1/s)

# 8.6 Continuous Rear Axle Drive Alpha (0.0 = pure ground-roll coasting, 1.0 = full 4WD drive)
ALPHA_SLEW_DROP_RATE: float = 4.0              # Max alpha decay rate (units/s) to prevent friction-circle shock
ALPHA_SLEW_RISE_RATE: float = 2.5              # Max alpha recovery rate (units/s) to prevent breaking traction again


# ==============================================================================
# 9. RAMP DETECTION & SPEED GOVERNOR
# ==============================================================================

ENABLE_RAMP_DETECTION: bool = True
RAMP_SPEED_LIMIT_MPS: float = 2.5       # Cruising speed limit (m/s) when robot is on a ramp
RAMP_ENTER_PITCH_DEG: float = 8.0       # Pitch angle (deg) threshold to trigger ramp mode
RAMP_ENTER_TILT_DEG: float = 9.0        # Combined tilt hypot(roll, pitch) (deg) threshold
RAMP_EXIT_PITCH_DEG: float = 5.0        # Pitch hysteresis exit floor (deg)
RAMP_EXIT_TILT_DEG: float = 6.0         # Tilt hysteresis exit floor (deg)
RAMP_HOLD_TIME_S: float = 0.30          # Hold latch duration (s) across ramp apex / transition


# ==============================================================================
# 10. BEST GENOME AUTO-LOADING (SHARED BY PI 4, BRAIN & WEBOTS PLANNERS)
# ==============================================================================

BEST_GENOME_SOURCE_PATH: Optional[str] = None


def _find_best_genome_path() -> Optional[str]:
    """Searches local and repository paths for best_planner_genome.json."""
    candidates: List[str] = [
        os.path.join(_CURR_DIR, "best_planner_genome.json"),
        os.path.join(os.path.dirname(_CURR_DIR), "best_planner_genome.json"),
    ]
    probe = _CURR_DIR
    for _ in range(6):
        candidates.append(
            os.path.join(probe, "5_webots", "optimization", "ga_planner_optimizer", "best_planner_genome.json")
        )
        parent = os.path.dirname(probe)
        if parent == probe:
            break
        probe = parent

    for path in candidates:
        if os.path.isfile(path):
            return path
    return None


def load_best_genome_genes() -> Tuple[Dict[str, Any], Optional[str]]:
    """Loads the 'genes' dictionary from best_planner_genome.json if present."""
    path = _find_best_genome_path()
    if not path:
        return {}, None
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        genes = data.get("genes") or data.get("best_genome") or {}
        if isinstance(genes, dict) and genes:
            return genes, path
    except Exception as e:
        print(f"[planner_config] Warning: Failed to load {path}: {e}", flush=True)
    return {}, None


def _apply_best_genome_overrides() -> None:
    """Overrides static planner defaults with values from best_planner_genome.json."""
    global BEST_GENOME_SOURCE_PATH
    genes, path = load_best_genome_genes()
    if not genes:
        return
    BEST_GENOME_SOURCE_PATH = path

    mapping = {
        "r_inflation_m": ("R_INFLATION_M", float),
        "target_lookahead_m": ("GAP_MAX_LOOKAHEAD_M", float),
        "w_gap_distance": ("WEIGHT_GAP_DISTANCE", float),
        "w_gap_width": ("WEIGHT_GAP_WIDTH", float),
        "w_gap_steer": ("WEIGHT_ALIGN_STEER", float),
        "w_gap_straight": ("WEIGHT_ALIGN_STRAIGHT", float),
        "w_side_clearance": ("WEIGHT_SIDE_CLEARANCE", float),
        "gap_dist_thresh_m": ("GAP_DIST_THRESH_M", float),
        "gap_dist_exp_k": ("GAP_DIST_EXP_K", float),
        "alt_min_separation_m": ("ALT_MIN_SEPARATION_M", float),
        "alt_aperture_bias": ("ALT_APERTURE_BIAS", float),
        "costmap_persistence": ("COSTMAP_PERSISTENCE_FRAMES", int),
        "use_true_aperture": ("GAP_USE_TRUE_APERTURE", int),
        "w_gap_center": ("WEIGHT_GAP_CENTER", float),
        "mpc_max_iter": ("MPC_MAX_ITER", int),
        "horizon_N": ("MPC_HORIZON_N", int),
        "dt": ("MPC_DT", float),
        "q_progress": ("MPC_Q_PROGRESS", float),
        "q_cte": ("MPC_Q_CTE", float),
        "q_heading": ("MPC_Q_HEADING", float),
        "r_steer": ("MPC_R_STEER", float),
        "r_accel": ("MPC_R_ACCEL", float),
        "r_steer_rate": ("MPC_R_STEER_RATE", float),
        "center_bias": ("MPC_CENTER_BIAS", float),
        "mu_friction": ("MPC_MU_FRICTION", float),
        "v_corner_factor": ("MPC_V_CORNER_FACTOR", float),
        "v_cruise_max": ("MPC_V_CRUISE_MAX", float),
        "v_cruise_min": ("MPC_V_CRUISE_MIN", float),
        "alt_switch_hysteresis": ("MPC_ALT_SWITCH_HYSTERESIS", float),
        "w_gap_clear": ("MPC_W_GAP_CLEAR", float),
        "use_corridor_flow": ("MPC_USE_CORRIDOR_FLOW", int),
        "track_width_cap_m": ("MPC_TRACK_WIDTH_CAP_M", float),
        "corner_outer_bias": ("MPC_CORNER_OUTER_BIAS", float),
        "inner_corner_margin_m": ("MPC_INNER_CORNER_MARGIN_M", float),
        "blind_turn_kappa": ("MPC_BLIND_TURN_KAPPA", float),
        "brake_decel_scale": ("MPC_BRAKE_DECEL_SCALE", float),
        "narrow_corridor_v_gain": ("MPC_NARROW_CORRIDOR_V_GAIN", float),
        "recovery_steer_angle_rad": ("RECOVERY_STEER_ANGLE_RAD", float),
        "recovery_clearance_exit_m": ("RECOVERY_CLEARANCE_EXIT_M", float),
        "recovery_arc_lookahead_m": ("RECOVERY_ARC_LOOKAHEAD_M", float),
    }

    g = globals()
    for gene_key, (const_name, cast) in mapping.items():
        if gene_key in genes:
            g[const_name] = cast(genes[gene_key])

    # Keep alias constants synchronized
    g["MPC_ALT_MIN_SEPARATION_M"] = g["ALT_MIN_SEPARATION_M"]
    g["MPC_ALT_APERTURE_BIAS"] = g["ALT_APERTURE_BIAS"]


_apply_best_genome_overrides()
