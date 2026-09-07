"""
================================================================================
   DA WINNA - BACKUP CONFIGURATION: MAX APEX SPEED (10CM MARGIN)
================================================================================
Archived: 2026-08-19
Configuration Profile: Max Apex Speed (10cm Margin)
Description:
  - Aggressive curvature minimization with wide corner entries & exits
  - Smooth C2 periodic reference spline
  - Fixed 10cm safety margin from walls & obstacles
================================================================================
"""

MAX_APEX_10CM_CONFIG = {
    "profile_name": "Max Apex Speed (10cm Margin)",
    "corner_entry_apex_aggressiveness": 1.00,
    "corner_exit_width_aggressiveness": 1.00,
    "num_progression_stages": 10,
    "max_solver_sub_iters": 800,
    "robot_safety_margin": 0.10,
    "weight_curvature": 3.50,
    "weight_path_smoothness": 0.03,
    "weight_jerk_regularization": 0.35,
    "macro_curvature_sigma": 4.0,
    "spline_smoothing_s": 0.015,
    "max_boundary_slope": 0.0,
    "default_max_speed": 4.0,
    "default_friction_coeff": 0.954,
    "max_acceleration": 2.4,
    "max_deceleration": 3.8
}
