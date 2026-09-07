"""
================================================================================
   DA WINNA - PERMANENT BACKUP: MAGENTA PRESET (ULTRA CURVATURE ARC)
================================================================================
Profile Characteristics:
  - Maximum turn radius R and highest apex cornering speed (2.36 m/s average)
  - 10 Progression Stages with 800 sub-iterations
  - Heavy curvature pull (Wk = 5.00) utilizing maximum outer track envelope
================================================================================
"""

# Mathematical Weighting Matrix
WEIGHT_CURVATURE = 5.00
WEIGHT_PATH_SMOOTHNESS = 0.030
WEIGHT_JERK_REGULARIZATION = 0.35

# Track Bounds & Obstacle Safety
ROBOT_SAFETY_MARGIN = 0.10      # 100mm (10cm)
TRACK_WIDTH_NOMINAL = 1.00      # 1000mm (1.0m)
MAX_BOUNDARY_SLOPE = 0.0        # Unconstrained boundary tangent

# Numerical Continuation & Solver Config
NUM_PROGRESSION_STAGES = 10
MAX_SOLVER_SUB_ITERS = 800
SOLVER_FTOL = 1e-8
SOLVER_GTOL = 1e-5

# Differential Geometry & Filtering
SPLINE_S = 0.015
MACRO_SIGMA = 4.0
