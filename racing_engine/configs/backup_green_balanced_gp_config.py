"""
================================================================================
   DA WINNA - PERMANENT BACKUP: GREEN PRESET (BALANCED GRAND PRIX)
================================================================================
Profile Characteristics:
  - Fastest solve time (~2,137 ms across 10 tracks, 1.25x faster than baseline)
  - 9 Progression Stages with 700 sub-iterations
  - Balanced Grand Prix line with excellent path tension & cornering apex speed
================================================================================
"""

# Mathematical Weighting Matrix
WEIGHT_CURVATURE = 2.80
WEIGHT_PATH_SMOOTHNESS = 0.060
WEIGHT_JERK_REGULARIZATION = 0.40

# Track Bounds & Obstacle Safety
ROBOT_SAFETY_MARGIN = 0.10      # 100mm (10cm)
TRACK_WIDTH_NOMINAL = 1.00      # 1000mm (1.0m)
MAX_BOUNDARY_SLOPE = 0.0        # Unconstrained boundary tangent

# Numerical Continuation & Solver Config
NUM_PROGRESSION_STAGES = 9
MAX_SOLVER_SUB_ITERS = 700
SOLVER_FTOL = 1e-8
SOLVER_GTOL = 1e-5

# Differential Geometry & Filtering
SPLINE_S = 0.015
MACRO_SIGMA = 4.0
