"""
================================================================================
  AUTONOMOUS TOF PATH PLANNERS: UNIFIED CONFIGURATION & TUNING PARAMETERS
================================================================================
  Single source of truth for:
    1. Physical Vehicle Dimensions, Kinematics & Actuator Limits
    2. Tire Friction & Vehicle Dynamic Traction Envelopes
    3. Sensors, Costmap & Multi-Rate Loop Timing (8 ms Sim / 16 ms Planner)
    4. Gap Scouting, Aperture Sizing & Selection Weights
    5. Curvilinear Slack-Constrained SL-MPC (Native C++ / OSQP)
    6. Non-Blocking Stuck Detection & Reverse Recovery Manager
    7. Anti-Wrong-Way Watchdog & Early Gap Exit Sentry
================================================================================
"""

import math
from typing import Dict, List

# ==============================================================================
# ACTIVE PLANNER CONFIGURATION
# ==============================================================================
PLANNER_TYPE: str = "SL_MPC"


# ==============================================================================
# 1. PHYSICAL VEHICLE & ACTUATOR SPECIFICATIONS
# ==============================================================================

# 1.1 Chassis Dimensions & Mass Properties
WHEELBASE: float = 0.1325              # meters (L: distance between front and rear axles)
TRACK_REAR: float = 0.1000             # meters (lateral distance between rear wheel contact centers)
TRACK_FRONT: float = 0.0822            # meters (lateral distance between front wheel contact centers)
TRACK_KINGPIN: float = 0.0620          # meters (distance between front steering kingpin pivots)
WHEEL_RADIUS: float = 0.0275           # meters (27.5 mm rolling radius)
VEHICLE_MASS: float = 1.000            # kg total vehicle mass

# 1.2 Steering Kinematics (40% Ackermann Geometry & Servo Endstops)
ACKERMANN_PERCENT: float = 0.40        # 40% Ackermann steering geometry (0.0 = parallel, 1.0 = true Ackermann)
MAX_SERVO_RAD: float = 0.696386        # radians (+- 39.9 deg servo limit, 0.1 deg inside Webots 40.0 deg physical hard stops)
MAX_STEERING_RAD: float = 0.657137     # radians (+- 37.65 deg virtual center angle producing 39.9 deg at inside servo)
MAX_ACKERMANN_RAD: float = MAX_STEERING_RAD  # Backward-compatible alias

# 1.3 Powertrain & Velocity Ceilings
MOTOR_DIR: float = -1.0                # Webots rotational sign (-1.0 applies forward thrust)
V_MAX_WHEEL: float = 5.0               # m/s governed wheel velocity ceiling
V_MAX_CRUISE: float = V_MAX_WHEEL      # m/s maximum straight-line cruising speed ceiling


# ==============================================================================
# 2. TIRE & VEHICLE DYNAMICS LIMITS
# ==============================================================================

# 2.1 Coulomb Friction & Gravitational Reference
TIRE_MU: float = 0.86                  # Dimensionless tire-ground Coulomb friction coefficient (GA champion)
GRAVITY_G: float = 9.81                # m/s^2 gravitational acceleration

# 2.2 Traction-Derived Acceleration & Deceleration Envelopes
A_MAX_BASE: float = TIRE_MU * GRAVITY_G  # ~8.437 m/s^2 nominal friction limit (mu * g)
ACCEL_FWD_RATIO: float = 0.95            # 95% of mu*g for smooth forward motor acceleration without wheel spin
BRAKE_DECEL_RATIO: float = 1.00          # 100% of mu*g for maximum braking / stopping deceleration
LAT_ACCEL_RATIO: float = 1.00            # 100% of mu*g for dynamic cornering grip

ACCEL_FWD_MAX: float = ACCEL_FWD_RATIO * A_MAX_BASE      # ~8.015 m/s^2 maximum forward motor acceleration
BRAKE_DECEL_MAX: float = BRAKE_DECEL_RATIO * A_MAX_BASE  # ~8.437 m/s^2 maximum braking deceleration
LAT_ACCEL_MAX: float = LAT_ACCEL_RATIO * A_MAX_BASE      # ~8.437 m/s^2 maximum cornering lateral acceleration

# 2.3 Turning Geometry & Friction-Limited Crawl Speed
# Kinematic minimum turning radius at inside wheel lock (37.65 deg virtual center): R_min = L / tan(delta_max)
R_MIN_TURN: float = WHEELBASE / math.tan(MAX_STEERING_RAD)  # ~0.1717 m
V_MIN_CRUISE: float = math.sqrt(LAT_ACCEL_MAX * R_MIN_TURN)  # ~1.204 m/s dynamic cornering floor

# 2.4 Digital Servo Dynamic Limits (PTK 9497 MG-D at 8.4V HV)
SERVO_MAX_SPEED_RAD_S: float = 15.4    # rad/s maximum steering servo slew rate at 8.4V (~0.068 s / 60 deg)


# ==============================================================================
# 3. SENSORS, COSTMAP & TIMING (Multi-Rate: 125 Hz Sim / 62.5 Hz Planner)
# ==============================================================================

# 3.1 Timing & Frequency Decimation
TIME_STEP_MS: int = 8                  # Webots basicTimeStep in ms (matches world file)
DT: float = TIME_STEP_MS / 1000.0      # Physics & fast actuator/IMU step duration (0.008 s = 8 ms)
SIM_HZ: float = 1000.0 / TIME_STEP_MS  # Simulation physics, 4WD motors, steering, and IMU rate (125.0 Hz)

PLANNER_DECIMATION: int = 2            # Run planner every 2 physics steps (125 Hz / 2 = 62.5 Hz)
TOF_TIME_STEP_MS: int = TIME_STEP_MS * PLANNER_DECIMATION  # ToF sensor sampling interval in ms (16 ms)
DT_PLANNER: float = TOF_TIME_STEP_MS / 1000.0              # Planner cycle duration (0.016 s = 16 ms)
PLANNER_HZ: float = 1000.0 / TOF_TIME_STEP_MS              # ToF scan processing & trajectory planning rate (62.5 Hz)

# 3.2 C++ Smart Hybrid Costmap Bounds (Local Vehicle Coordinates, +Y Forward, +X Right)
GRID_X_MIN: float = -1.25              # meters (lateral left boundary)
GRID_X_MAX: float = 1.25               # meters (lateral right boundary)
GRID_Y_MIN: float = 0.00               # meters (starting from robot center)
GRID_Y_MAX: float = 1.90               # meters (forward preview range up to 1.9m)
GRID_RES: float = 0.010                # meters (10 mm grid resolution)
R_INFLATION_M: float = 0.075           # meters (7.5 cm inflation: half chassis width, GA-optimized)

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
FRONT_TOF_SENSOR_NAMES: List[str] = TOF_SENSOR_NAMES  # Backward-compatible alias
TOF_MAX_RANGE_M: float = 1.80          # meters maximum valid ToF range
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
MOVING_OBSTACLE_SPEED_MPS: float = 1.00     # m/s backwards pace along racing line (Track 1 = 1.00 m/s, Track 2 = 0.50 m/s)
MOVING_OBSTACLE_Z_ELEVATION: float = 0.075  # meters elevation above track ground


# ==============================================================================
# 4. GAP SCOUTING & SELECTION WEIGHTS
# ==============================================================================

# 4.1 Gap Aperture & Lookahead Constraints
GAP_MIN_WIDTH_M: float = 0.0           # meters minimum passable aperture width
GAP_MAX_LOOKAHEAD_M: float = 1.70      # meters maximum lookahead distance for gap target
GAP_MIN_LOOKAHEAD_M: float = 0.20      # meters minimum lookahead distance

# 4.2 Candidate Evaluation Weights (GA Champion)
# Score = W_DIST * dist_factor + W_WIDTH * (width/max_w) - W_DEV_STEER * |theta - steer| - W_DEV_STRAIGHT * |theta|
WEIGHT_GAP_DISTANCE: float = 7.026     # Preference for longer/further clear gaps (GA Champion)
WEIGHT_GAP_WIDTH: float = 2.047        # Preference for wider open apertures
WEIGHT_ALIGN_STEER: float = 1.681      # Preference for turning toward where servo is already aiming
WEIGHT_ALIGN_STRAIGHT: float = 0.998   # Preference for going straight when openings are equal
WEIGHT_SIDE_CLEARANCE: float = 2.547   # Preference for steering away from close walls / side asymmetry

# 4.3 Exponential Distance Preference (1.0m Walled Track Geometry)
# Any sightline > 1.0m + margin is definitively the open racing corridor
TRACK_CORRIDOR_WIDTH_M: float = 1.00   # meters standard walled race corridor width
GAP_PROGRESS_MARGIN_M: float = 0.164   # meters (1.0m + 0.164m = 1.164m threshold for definitive open corridor)
GAP_DIST_EXP_K: float = 0.717          # Exponential growth scaling factor for distance beyond threshold

# 4.4 Fast Straight-Line Lock
STRAIGHT_LOCK_DIST_M: float = 1.70     # meters forward clearance on both 4.1 deg ToFs to lock full speed straight
STRAIGHT_LOCK_SPEED_MPS: float = V_MAX_CRUISE  # m/s top cruising speed when straight-line locked (5.0 m/s)

# 4.5 Dual-Path Candidate Selection & Smart Challenger Arbitration
ALT_MIN_SEPARATION_M: float = 0.052    # meters minimum lateral separation between primary and challenger targets
ALT_APERTURE_BIAS: float = 1.553       # Multiplier on aperture width reward for challenger target


# ==============================================================================
# 5. CURVILINEAR SLACK-CONSTRAINED SL-MPC (OSQP / C++ NATIVE)
# ==============================================================================

# 5.1 Prediction Horizon & Timestep
MPC_HORIZON_N: int = 30                # Prediction horizon steps (fixed at 30 in native QP sparsity table)
MPC_DT: float = 0.017                  # seconds per horizon step (~60 Hz planning loop)

# 5.2 Cost Function Weights (Quadratic & Linear OSQP Penalties, GA Champion)
MPC_Q_PROGRESS: float = 23.108         # Weight encouraging forward arc-length progress
MPC_Q_CTE: float = 54.342              # Weight penalizing cross-track deviation from guide line
MPC_Q_HEADING: float = 9.685           # Weight penalizing heading error
MPC_R_STEER: float = 4.944             # Weight penalizing steering angle magnitude
MPC_R_ACCEL: float = 3.507             # Weight penalizing longitudinal acceleration/braking effort
MPC_R_STEER_RATE: float = 12.310       # Weight penalizing steering rate slew

# 5.3 Traction Circle & Dynamic Speeds
MPC_MU_FRICTION: float = TIRE_MU       # Tire friction coefficient governing grip envelope (0.86)
MPC_V_CORNER_FACTOR: float = 0.843     # Friction circle cornering speed scaling factor
MPC_V_CRUISE_MAX: float = V_MAX_CRUISE # m/s top cruising speed ceiling (5.0 m/s)
MPC_V_CRUISE_MIN: float = V_MIN_CRUISE # m/s cornering / crawl speed floor (~1.20 m/s)

# 5.4 Straight-Line Lock & Challenger Aliases
MPC_STRAIGHT_LOCK_DIST_M: float = STRAIGHT_LOCK_DIST_M        # Forward clearance to lock straight (1.70 m)
MPC_STRAIGHT_LOCK_SPEED_MPS: float = STRAIGHT_LOCK_SPEED_MPS  # Top speed when straight locked (5.0 m/s)
MPC_ALT_SWITCH_HYSTERESIS: float = 24.484                     # Cost reduction needed for challenger overthrow
MPC_ALT_MIN_SEPARATION_M: float = ALT_MIN_SEPARATION_M        # Minimum lateral separation between paths (m)
MPC_ALT_APERTURE_BIAS: float = ALT_APERTURE_BIAS              # Challenger aperture width bias multiplier


# ==============================================================================
# 6. NON-BLOCKING STUCK & REVERSE RECOVERY MANAGER
# ==============================================================================

# 6.1 Recovery Kinematics & Speed Limits
RECOVERY_STEER_ANGLE_RAD: float = MAX_STEERING_RAD     # radians (~37.65 deg) steering lock during reverse arc
RECOVERY_REVERSE_SPEED_MPS: float = V_MIN_CRUISE       # m/s dynamic cornering speed floor (~1.20 m/s)
RECOVERY_SERVO_SLEW_RAD_S: float = SERVO_MAX_SPEED_RAD_S  # rad/s centering slew rate (15.4 rad/s at 8.4V)
RECOVERY_ARC_LOOKAHEAD_M: float = 0.146                # meters length of kinematic escape arcs projected on costmap

# 6.2 Stall Detection Triggers & Failsafe Timeouts
RECOVERY_FRONT_BLOCKED_M: float = 0.25       # meters front clearance threshold to detect forward stall
RECOVERY_REVERSE_STROKE_MAX_S: float = 1.0  # seconds maximum reverse stroke failsafe
RECOVERY_TOTAL_TIMEOUT_S: float = 1.2       # seconds maximum total recovery duration before forced exit
RECOVERY_EXIT_MIN_TIME_S: float = 0.0        # seconds minimum recovery stroke time before clearance check

# 6.3 Multi-Criteria Exit Conditions
RECOVERY_CLEARANCE_EXIT_M: float = 0.25      # meters (25 cm) minimum forward clearance to conclude reverse
RECOVERY_ANY_TOF_EXIT_M: float = 0.60        # meters (60 cm) minimum clearance on at least one ToF to conclude reverse


# ==============================================================================
# 7. ANTI-WRONG-WAY WATCHDOG & EARLY GAP SENTRY
# ==============================================================================

# 7.1 Watchdog Mode & Breadcrumb History
ANTI_WRONG_WAY_MODE: str = "OPTION_1_BREADCRUMB"
AWW_BREADCRUMB_LOOKBACK_M: float = 8.0         # meters baseline historical path retained
AWW_BREADCRUMB_DURATION_SCALE: float = 0.6     # scale factor for breadcrumb fade (0.6x = disappears 1.67x faster / 60% retention)
AWW_BREADCRUMB_SPACING_M: float = 0.05         # meters minimum spatial spacing between breadcrumb nodes
AWW_BREADCRUMB_MAX_TIMEOUT_S: float = 6.0      # seconds maximum timeout before breadcrumb history expires
AWW_HISTORY_WINDOW_S: float = AWW_BREADCRUMB_MAX_TIMEOUT_S  # Backward-compatible alias (6.0 s)

# 7.2 Narrow-Cone Early Gap Exit Sentry
AWW_EARLY_EXIT_MIN_GAP_M: float = 1.0          # meters minimum scouted gap distance to qualify as long-path early exit
AWW_EARLY_EXIT_CONE_DEG: float = 20.0          # degrees total narrow cone width (±10 deg) oriented along candidate gap vector
AWW_EARLY_EXIT_OPPOSING_DEG: float = 110.0     # degrees heading divergence threshold for opposing breadcrumbs in gap corridor
