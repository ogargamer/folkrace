"""
Predefined Tuning Profiles for TUM / F1TENTH Racing Line Optimization:

1. Max Apex Speed (Aggressive, Maximum Radius)
2. Balanced Grand Prix (Standard Default TUM)
3. Chicane Slicer (Taut Line, Shortest Path Length)
4. Fast Embedded CPU (Ultra-Low CPU, Real-Time Embedded)
5. Precision CAD HQ (Maximum Numerical Precision)
"""

TUNING_PROFILES = {
    "Profile A: Max Apex Speed": {
        "description": "Aggressive maximum corner radius & apex speed (8cm safety margin)",
        "weight_curvature": 3.50,
        "weight_path_smoothness": 0.03,
        "weight_jerk": 0.35,
        "robot_safety_margin": 0.08,
        "num_progression_stages": 10,
        "max_solver_sub_iters": 800,
        "color": "#ff0055",
        "category": "High Speed"
    },
    "Profile B: Balanced Grand Prix": {
        "description": "Standard balanced TUM / F1TENTH racing line (10cm safety margin)",
        "weight_curvature": 2.00,
        "weight_path_smoothness": 0.10,
        "weight_jerk": 0.50,
        "robot_safety_margin": 0.10,
        "num_progression_stages": 8,
        "max_solver_sub_iters": 600,
        "color": "#00ff88",
        "category": "Balanced"
    },
    "Profile C: Chicane Slicer": {
        "description": "Taut path, cuts straighter through S-bends with minimum distance",
        "weight_curvature": 1.20,
        "weight_path_smoothness": 0.35,
        "weight_jerk": 0.60,
        "robot_safety_margin": 0.12,
        "num_progression_stages": 8,
        "max_solver_sub_iters": 600,
        "color": "#00b4d8",
        "category": "Taut / Low Drag"
    },
    "Profile D: Fast Embedded CPU": {
        "description": "Ultra-low compute budget (4 stages, max 120 iters) for Pi 5 / realtime",
        "weight_curvature": 2.00,
        "weight_path_smoothness": 0.12,
        "weight_jerk": 0.40,
        "robot_safety_margin": 0.10,
        "num_progression_stages": 4,
        "max_solver_sub_iters": 120,
        "color": "#ffb800",
        "category": "Low CPU"
    },
    "Profile E: Precision CAD HQ": {
        "description": "Maximum numerical precision (12 stages, 1200 sub-iters) for offline CAD",
        "weight_curvature": 2.50,
        "weight_path_smoothness": 0.08,
        "weight_jerk": 0.60,
        "robot_safety_margin": 0.10,
        "num_progression_stages": 12,
        "max_solver_sub_iters": 1200,
        "color": "#b5179e",
        "category": "High Precision"
    }
}
