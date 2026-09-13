"""
================================================================================
  AUTONOMOUS TOF PATH PLANNERS: UNIFIED CONFIGURATION & TUNING PARAMETERS
  Exposes all physical, kinematic, dynamic, costmap, and solver settings.
================================================================================
"""

import math

# ==============================================================================
# ACTIVE PLANNER SELECTION
# Option 2: Curvilinear Slack-Constrained SL-MPC (OSQP / C++ Native)
# ==============================================================================
PLANNER_TYPE = "OPTION2_SL_MPC"

# ==============================================================================
# 1. PHYSICAL VEHICLE & ACTUATOR SPECIFICATIONS
# ==============================================================================
WHEELBASE = 0.1325             # meters (L: distance between front and rear axles)
TRACK_REAR = 0.1000            # meters (lateral distance between rear wheel centers)
TRACK_FRONT = 0.0822           # meters (lateral distance between front wheel centers)
TRACK_KINGPIN = 0.0620         # meters (distance between front steering kingpin pivots)
WHEEL_RADIUS = 0.0275          # meters (27.5 mm rolling radius)
VEHICLE_MASS = 0.600           # kg total vehicle mass
IZ = 0.0025                    # kg * m^2 vehicle moment of inertia around yaw axis

# Center of Gravity distribution
LF = 0.06625                   # meters (front axle to CoG, approx 50/50 split)
LR = 0.06625                   # meters (rear axle to CoG)

# Actuator Limits (Virtual Bicycle Model)
MAX_STEERING_RAD = 0.698132     # radians (+- 40.0 deg steering limit matching Webots servo limits)
MAX_SERVO_RATE_RAD_S = 10.5    # rad/s maximum steering slew rate
MOTOR_DIR = -1.0               # Webots motor sign (-1.0 pushes vehicle forward)
V_MAX_WHEEL = 5.500            # m/s physical maximum wheel velocity (200 rad/s * 0.0275m)
V_MIN_CRUISE = 1.2           # m/s minimum driving speed (prevents zero-speed stall)
V_MAX_CRUISE = 5.50            # m/s maximum straight-line cruising speed (matches 5.5m/s motor ceiling)

# ==============================================================================
# 2. TIRE & VEHICLE DYNAMICS LIMITS
# ==============================================================================
TIRE_MU = 0.85                 # dimensionless tire-ground Coulomb friction coefficient
GRAVITY_G = 9.81               # m/s^2 gravitational acceleration
ALPHA_PEAK = 0.220             # radians (~12.6 degrees dynamic peak tire slip angle before severe washout)

# --- UNIFIED FRICTION-DERIVED ACCELERATION & SERVO LIMITS ---
# Master Traction Limit: Base Acceleration = TIRE_MU * GRAVITY_G (~8.34 m/s^2 @ mu=0.85)
A_MAX_BASE = TIRE_MU * GRAVITY_G
ACCEL_FWD_RATIO = 0.9                  # 85% of mu*g for smooth forward motor acceleration without wheel spin
BRAKE_DECEL_RATIO = 1.00                # 100% of mu*g for maximum braking / stopping deceleration
LAT_ACCEL_RATIO = 0.95                  # 95% of mu*g for dynamic cornering limit

ACCEL_FWD_MAX = ACCEL_FWD_RATIO * A_MAX_BASE      # ~7.09 m/s^2 forward motor acceleration limit
BRAKE_DECEL_MAX = BRAKE_DECEL_RATIO * A_MAX_BASE  # ~8.34 m/s^2 braking / deceleration limit
LAT_ACCEL_MAX = LAT_ACCEL_RATIO * A_MAX_BASE      # ~7.92 m/s^2 cornering lateral acceleration limit

# PTK 9497 MG-D Digital Servo Specifications (at 8.4V HV)
# Spec: ~0.068 s / 60 deg -> (pi/3) / 0.068 = ~15.4 rad/s slew rate
SERVO_MAX_SPEED_RAD_S = 15.4            # rad/s maximum steering servo slew rate at 8.4V

# Linear cornering stiffness per axle (N/rad)
C_AF = 34.0                    # Front cornering stiffness (N/rad)
C_AR = 38.0                    # Rear cornering stiffness (N/rad)

# Understeer gradient K_us (rad / (m/s^2))
UNDERSTEER_K_US = (VEHICLE_MASS / WHEELBASE) * (LR / C_AF - LF / C_AR)

# ==============================================================================
# 3. SENSORS, COSTMAP & TIMING (Multi-Rate: 120 Hz Sim/IMU/4WD, 60 Hz ToF/Planner)
# ==============================================================================
SIM_HZ = 120                   # Simulation physics, 4WD motors, steering, and IMU rate (120 Hz)
TIME_STEP_MS = 8               # Webots basicTimeStep in ms (1000 / 120 ~ 8.33 ms -> 8 ms)
DT = 1.0 / 120.0               # Physics & fast actuator/IMU step duration in seconds (~0.00833 s)
PLANNER_HZ = 60                # ToF scan processing & trajectory planning rate (60 Hz)
DT_PLANNER = 1.0 / 60.0        # Planner cycle duration in seconds (~0.01667 s)
PLANNER_DECIMATION = 2         # Run planner every 2 physics steps (120 Hz / 2 = 60 Hz)
TOF_TIME_STEP_MS = 16          # ToF sensor sampling interval in ms (60 Hz)

# 15mm C++ Smart Hybrid Costmap bounds (local vehicle coordinates)
# +Y is Forward, +X is Right
GRID_X_MIN = -1.25             # meters (left boundary)
GRID_X_MAX = 1.25              # meters (right boundary)
GRID_Y_MIN = -0.10             # meters (10 cm chassis clearance behind origin)
GRID_Y_MAX = 2.00              # meters (forward preview range)
GRID_RES = 0.015               # meters (15 mm grid resolution)
R_INFLATION_M = 0.090          # meters (9.0 cm default dilation: bare minimum is 7.5cm for 15cm chassis width)

# Sensor device names: 16-Channel Front-Only ToF rig (only_front_tofs.proto)
TOF_SENSOR_NAMES = [
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
FRONT_TOF_SENSOR_NAMES = TOF_SENSOR_NAMES
TOF_MAX_RANGE_M = 1.80         # meters max range
TOF_MIN_RANGE_M = 0.001        # meters min range

# Moving Obstacles on Racing Line
MOVING_OBSTACLE_NAMES = ["MOVING_OBSTACLE", "MOVING_OBSTACLE_2", "MOVING_OBSTACLE_3"]
MOVING_OBSTACLE_SPEED_MPS = 0.50   # m/s backwards pace along racing line
MOVING_OBSTACLE_Z_ELEVATION = 0.075 # m elevation above track ground

ENCODER_NAMES = {
    "FL": "encoder FL",
    "FR": "encoder FR",
    "RL": "encoder RL",
    "RR": "encoder RR",
}

# ==============================================================================
# 4. GAP SCOUTING & SELECTION WEIGHTS
# ==============================================================================
GAP_MIN_WIDTH_M = 0.0         # meters minimum passable aperture width
GAP_MAX_LOOKAHEAD_M = 1.70     # meters maximum lookahead distance for gap target
GAP_MIN_LOOKAHEAD_M = 0.35     # meters minimum lookahead distance
GAP_OFFSETS = [-0.10, -0.08, -0.05, -0.03, -0.01, 0.00, 0.01, 0.03, 0.05, 0.08, 0.10]  # meters lateral exploration offsets

# Candidate Evaluation Weights:
# Score = W_DIST * dist_weight_factor + W_WIDTH * (width/max_w) - W_DEV_STEER * |theta - current_steer| - W_DEV_STRAIGHT * |theta|
WEIGHT_GAP_DISTANCE = 3.0      # Preference for longer/further clear gaps
WEIGHT_GAP_WIDTH = 1.5         # Preference for wider open apertures
WEIGHT_ALIGN_STEER = 1.0       # Preference for turning toward where servo is already aiming
WEIGHT_ALIGN_STRAIGHT = 0.5    # Preference for going straight when openings are equal
WEIGHT_SIDE_CLEARANCE = 2.8    # Preference for steering away from close walls / side asymmetry


# Exponential Distance Preference (1.0m Walled Track Geometry):
# Since the track is walled off and 1.0m wide, any sightline > 1.0m + margin is certainly the path towards progress.
TRACK_CORRIDOR_WIDTH_M = 1.00  # meters standard walled race corridor width
GAP_PROGRESS_MARGIN_M = 0.15   # meters: 1.0m + 0.15m = 1.15m threshold for definitive open corridor
GAP_DIST_EXP_K = 0.60          # slight exponential growth scaling factor for distance beyond threshold (~48% boost at 1.80m)
STRAIGHT_LOCK_DIST_M = 1.70    # meters: forward clearance on both 4.1 deg ToFs to lock full speed straight
STRAIGHT_LOCK_SPEED_MPS = 5.50 # m/s: top speed cruising when straight locked
EMERGENCY_BACKUP_DIST_M = 0.09 # meters (9cm): emergency threshold on 4.1 deg ToFs to trigger reverse

# Dual-Path Candidate Selection & Smart Challenger Arbitration (Option 2 SL-MPC)
ALT_MIN_SEPARATION_M = 0.15      # meters: minimum lateral separation between primary and challenger target lines
ALT_APERTURE_BIAS = 1.80         # multiplier on aperture width reward for challenger target (safety/opening priority)
ALT_SWITCH_HYSTERESIS = 30.0     # objective cost reduction threshold needed for challenger to overthrow primary line


# ==============================================================================
# 5. OPTION 2: CURVILINEAR SLACK-CONSTRAINED SL-MPC (OSQP / C++ NATIVE)
# ==============================================================================
# Prediction horizon & timestep
OPT2_HORIZON_N = 30                 # prediction horizon steps (fixed at 30 in QP sparsity table)
OPT2_DT = DT_PLANNER                # seconds per horizon step (matches 60 Hz loop ~0.01667 s)

# Cost function weights (Quadratic & Linear OSQP penalties)
OPT2_Q_PROGRESS = 8.0               # weight encouraging forward arc-length progress
OPT2_Q_CTE = 18.0                   # weight penalizing lateral / cross-track deviation from guide line
OPT2_Q_HEADING = 8.0                # weight penalizing heading error
OPT2_R_STEER = 5.0                  # weight penalizing steering angle magnitude
OPT2_R_ACCEL = 2.0                  # weight penalizing acceleration / deceleration effort
OPT2_R_STEER_RATE = 10.0            # weight penalizing steering rate (slew)

# Traction, cornering & guidance
OPT2_MU_FRICTION = TIRE_MU          # tire friction coefficient governing grip envelope (0.85)
OPT2_V_CORNER_FACTOR = 0.95         # friction circle cornering speed scaling factor
OPT2_PP_BLEND_GAIN = 0.35           # pure pursuit entry blend gain
OPT2_LOOKAHEAD_BLEND_M = 0.45       # pure pursuit lookahead blend distance (m)
OPT2_V_CRUISE_MAX = V_MAX_CRUISE    # m/s top cruising speed limit (5.50 m/s)
OPT2_V_CRUISE_MIN = V_MIN_CRUISE    # m/s cornering / crawl speed floor (0.60 m/s)

# Fast straight-line lock & dual-path challenger arbitration
OPT2_STRAIGHT_LOCK_DIST_M = STRAIGHT_LOCK_DIST_M        # forward clearance to lock straight (1.70 m)
OPT2_STRAIGHT_LOCK_SPEED_MPS = STRAIGHT_LOCK_SPEED_MPS  # top speed when straight-line locked (5.50 m/s)
OPT2_ALT_SWITCH_HYSTERESIS = ALT_SWITCH_HYSTERESIS      # cost reduction needed for challenger overthrow (15.0)
OPT2_ALT_MIN_SEPARATION_M = ALT_MIN_SEPARATION_M        # min lateral separation between paths (0.25 m)
OPT2_ALT_APERTURE_BIAS = ALT_APERTURE_BIAS              # challenger aperture width bias multiplier (1.80)

# Target speed alias
OPT2_TARGET_SPEED_MPS = OPT2_V_CRUISE_MAX

# ==============================================================================
# 6. ADAPTIVE STEERING AGGRESSIVENESS & SMOOTHING
# ==============================================================================
STEER_AGGR_DIST_FAR_M = 1.70      # meters: distance where steering quieting begins on open straights
STEER_AGGR_DIST_CLOSE_M = 1.35    # meters: distance where full 100% steering aggressiveness is guaranteed
STEER_AGGR_MIN_SCALE = 0.90       # conservative aggressiveness scale on straights (90% response preserves turning authority)
STEER_AGGR_MAX_SCALE = 1.00       # full aggressiveness scale (100% response)

# Small Target Offset Steering Smoothing (tunes micro-adjustments on straights)
STEER_ERROR_DEADBAND_RAD = 0.002       # radians (~0.11 deg): sub-degree deadband for sensor noise
STEER_SMALL_ERROR_THRESHOLD_RAD = 0.05 # radians (~2.86 deg): angle threshold above which full response is engaged
STEER_SMALL_ERROR_MIN_GAIN = 0.90      # minimum gain applied near deadband (never freezes steering)


def compute_steering_aggressiveness(d_straight: float, steer_error_rad: float = 0.0) -> float:
    """
    Computes conservative compound steering aggressiveness factor:
    1. Distance ahead: min(front ToFs) scaling from 0.90 (far >= 1.70m) to 1.00 (close <= 1.35m)
    2. Small steering error attenuation: keeps response >= 0.90 at all times so robot never stalls turning.
    """
    # 1. Distance-based scale (Progressive curve)
    if d_straight >= STEER_AGGR_DIST_FAR_M:
        dist_scale = STEER_AGGR_MIN_SCALE
    elif d_straight <= STEER_AGGR_DIST_CLOSE_M:
        dist_scale = STEER_AGGR_MAX_SCALE
    else:
        frac = (STEER_AGGR_DIST_FAR_M - d_straight) / max(0.001, (STEER_AGGR_DIST_FAR_M - STEER_AGGR_DIST_CLOSE_M))
        dist_scale = float(STEER_AGGR_MIN_SCALE + (frac ** 1.5) * (STEER_AGGR_MAX_SCALE - STEER_AGGR_MIN_SCALE))

    # 2. Target offset / steering error attenuation (conservative: minimum 0.90 authority)
    err_mag = abs(steer_error_rad)
    if err_mag <= STEER_ERROR_DEADBAND_RAD:
        err_scale = STEER_SMALL_ERROR_MIN_GAIN
    elif err_mag >= STEER_SMALL_ERROR_THRESHOLD_RAD:
        err_scale = 1.0
    else:
        err_frac = (err_mag - STEER_ERROR_DEADBAND_RAD) / max(0.001, (STEER_SMALL_ERROR_THRESHOLD_RAD - STEER_ERROR_DEADBAND_RAD))
        err_scale = float(STEER_SMALL_ERROR_MIN_GAIN + err_frac * (1.0 - STEER_SMALL_ERROR_MIN_GAIN))

    return float(max(0.85, min(1.0, dist_scale * err_scale)))


# ==============================================================================
# 7. NON-BLOCKING STUCK & REVERSE RECOVERY MANAGER
# ==============================================================================
RECOVERY_REVERSE_SPEED_MPS = 1.30    # m/s reverse backing velocity during recovery arc
RECOVERY_STEER_ANGLE_RAD = 0.6    # radians (26.0 deg) steering lock during reverse arc
RECOVERY_STALL_CONFIRM_S = 0.15      # seconds of low forward speed before engaging recovery
RECOVERY_FRONT_BLOCKED_M = 0.25      # meters front clearance threshold to detect forward stall
RECOVERY_REVERSE_STROKE_MAX_S = 0.6 # seconds maximum reverse stroke failsafe
RECOVERY_TOTAL_TIMEOUT_S = 0.8      # seconds maximum total recovery duration before forced exit
RECOVERY_CLEARANCE_EXIT_M = 0.28     # meters minimum front clearance to conclude reverse and center
RECOVERY_EXIT_MIN_TIME_S = 0.08      # seconds minimum recovery stroke time before clearance check
RECOVERY_SERVO_SLEW_RAD_S = SERVO_MAX_SPEED_RAD_S  # rad/s PTK 9497 8.4V centering slew rate (15.4 rad/s)
