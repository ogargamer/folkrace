"""
================================================================================
   DA WINNA - PERMANENT BACKUP: CHILD 05 (300 SUB-ITERATIONS PRESET)
================================================================================
Profile Characteristics:
  - High-Efficiency Real-Time Mode (~1,140 ms solve time, 2.3x faster than baseline)
  - 7 Progression Stages with 300 sub-iterations per stage
  - Maximum cornering apex velocity (2.85 - 2.94 m/s peak turn speed)
  - Balanced Grand Prix blend with sprint path tension (Wk=3.20, Wp=0.055, Wj=0.38)
================================================================================
"""

# Mathematical Weighting Matrix
WEIGHT_CURVATURE = 3.20
WEIGHT_PATH_SMOOTHNESS = 0.055
WEIGHT_JERK_REGULARIZATION = 0.38

# Track Bounds & Obstacle Safety
ROBOT_SAFETY_MARGIN = 0.10      # 100mm (10cm)
TRACK_WIDTH_NOMINAL = 1.00      # 1000mm (1.0m)
MAX_BOUNDARY_SLOPE = 0.0        # Unconstrained boundary tangent

# Numerical Continuation & Solver Config
NUM_PROGRESSION_STAGES = 7
MAX_SOLVER_SUB_ITERS = 300
SOLVER_FTOL = 1e-8
SOLVER_GTOL = 1e-5

# Differential Geometry & Filtering
SPLINE_S = 0.015
MACRO_SIGMA = 4.0

# Vehicle Dynamics Defaults
FRICTION_COEFF_NOMINAL = 0.90   # Tire grip factor (mu)
MAX_LATERAL_ACCEL = 8.829       # mu * g (m/s^2)
MAX_LONG_ACCEL = 2.00           # m/s^2
MAX_BRAKE_ACCEL = 3.00          # m/s^2
MAX_SPEED_LIMIT = 4.00          # m/s (straightaway cap)
