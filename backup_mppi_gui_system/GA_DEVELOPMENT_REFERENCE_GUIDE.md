# Genetic Algorithm (GA) Development Reference Guide for Da Winna MPPI

This document serves as the authoritative architectural specification and codebase reference for any AI agent developing the automated Genetic Algorithm (GA / CMA-ES / Differential Evolution) to optimize the autonomous 4WD Folkrace racing vehicle.

---

## 1. System Overview & Evaluation Architecture

```
+-----------------------------------------------------------------------------------+
|                            GENETIC ALGORITHM OPTIMIZER                            |
|             (Population Management -> Crossover -> Mutation -> Selection)          |
+-----------------------------------------+-----------------------------------------+
                                          | Passes candidate MPPIGenome (23 genes)
                                          v
+-----------------------------------------------------------------------------------+
|                             WEBOTS SIMULATION RUNNER                              |
|           - Headless or GUI Webots Supervisor instance                            |
|           - Resets car to start position (x=0.60, y=0.00, yaw=0.00)               |
|           - Runs vehicle for 1-2 laps (or 15-20 sim seconds)                      |
|           - Collects telemetry: Lap Time, Max Speed, Collisions, CTE, Jerk        |
+-----------------------------------------+-----------------------------------------+
                                          | Real-time Step Loop (100 Hz physics)
                                          v
+-----------------------------------------------------------------------------------+
|                        HIGH-PERFORMANCE MPPI CONTROLLER                           |
|  - fast_mppi_core.py: LLVM SIMD JIT Kernel (<0.5 ms for 512 rollouts)             |
|  - fast_tof_carto_costmap.py: 60 Hz 20mm Bayesian Occupancy Grid + 16 ToFs       |
|  - mppi_folkrace_controller.py: Multi-Modal Seeding (Center, Left, Right)        |
+-----------------------------------------------------------------------------------+
```

---

## 2. Key Codebase File Locations

| Component | Absolute Path | Description |
| :--- | :--- | :--- |
| **MPPI Controller Suite** | `my_project/controllers/folkrace_drive/mppi_folkrace_controller.py` | Contains `MPPIGenome` (23 genes), `MPPIController`, and `to_array()` / `from_array()`. |
| **JIT SIMD Rollout Engine** | `my_project/controllers/mppi_controller/fast_mppi_core.py` | Native AVX2/NEON machine-code kernel (`evaluate_mppi_rollouts_jit`) executing 12,800 rollouts in 0.47 ms. |
| **Active Webots Controller** | `my_project/controllers/mppi_controller/mppi_controller.py` | Webots Supervisor controller connecting motors, servos, IMU, 16 ToFs, and LiDAR. |
| **Costmap Pipeline** | `my_project/controllers/tof_16ch_grid_scanner/fast_tof_carto_costmap.py` | Fuses 15 Hz Cartographer OccupancyGrid and 50 Hz 16 ToFs into 20mm rolling log-odds grid. |
| **GUI & WSL SLAM Launcher** | `launch_costmap_gui.py` | 1-click launcher starting WSL2 ROS 2 Cartographer daemon and PyQt6 30 FPS visualizer. |
| **Cartographer 2D Config** | `ros2_cartographer/config/folkrace_2d.lua` | Google Cartographer SLAM tuning parameters for 6x6m indoor arena. |
| **Optimal Racing Lines** | `racing_results/tum_qp_obstacle_aware_25cm_offsets/` | Contains `master_cyan_champion_optimal.csv`, `wall_offset_left_25cm.csv`, `wall_offset_right_25cm.csv`. |

---

## 3. The 23-Gene Chromosome Specification (`MPPIGenome`)

Each candidate solution in the GA population is represented by `MPPIGenome` (or a flat 1D NumPy array of length 23):

```python
from my_project.controllers.folkrace_drive.mppi_folkrace_controller import MPPIGenome

# Serialization
genome = MPPIGenome()
arr = genome.to_array()           # Flat float64 array of shape (23,)
genome_rec = MPPIGenome.from_array(arr) # Clamped & budget-enforced reconstruction
```

### Gene Table & Search Bounds

| Index | Gene Name | Type | Min | Max | Default | Description |
| :---: | :--- | :---: | :---: | :---: | :---: | :--- |
| **0** | `H` | `int` | `10` | `35` | `25` | Prediction horizon steps |
| **1** | `K` | `int` | `64` | `512` | `512` | Sample trajectory count |
| **2** | `dt` | `float` | `0.010` | `0.050` | `0.025` | Rollout step duration (seconds) |
| **3** | `lambda_temp` | `float` | `0.01` | `2.0` | `0.35` | Softmax temperature / cost sensitivity |
| **4** | `ar1_alpha` | `float` | `0.00` | `0.95` | `0.80` | AR-1 temporal noise correlation |
| **5** | `gamma_discount` | `float` | `0.70` | `1.00` | `0.94` | Exponential stage discount factor |
| **6** | `mu_friction` | `float` | `0.60` | `0.90` | `0.85` | Tire friction coefficient (strict bound) |
| **7** | `w_cte` | `float` | `0.1` | `50.0` | `8.0` | Cross-track error penalty |
| **8** | `w_heading` | `float` | `0.1` | `30.0` | `4.0` | Heading alignment penalty |
| **9** | `w_speed` | `float` | `0.1` | `20.0` | `2.5` | Target speed tracking penalty |
| **10** | `w_hazard` | `float` | `1000.0` | `500000.0` | `100000.0` | Fatal hazard cell penalty |
| **11** | `w_prob_costmap`| `float` | `1.0` | `200.0` | `45.0` | Continuous log-odds wall clearance penalty |
| **12** | `w_unknown` | `float` | `0.0` | `50.0` | `12.0` | Unobserved / unknown space penalty |
| **13** | `w_steer_rate` | `float` | `0.01` | `10.0` | `0.60` | Steering slew rate penalty (servo protection) |
| **14** | `w_accel_rate` | `float` | `0.01` | `10.0` | `0.20` | Longitudinal jerk penalty |
| **15** | `w_terminal` | `float` | `1.0` | `20.0` | `6.0` | Terminal horizon state multiplier |
| **16** | `sigma_steer` | `float` | `0.02` | `0.35` | `0.14` | Steering innovation standard deviation (rad) |
| **17** | `sigma_speed` | `float` | `0.10` | `2.00` | `0.50` | Speed innovation standard deviation (m/s) |
| **18** | `lognorm_shape`| `float` | `0.10` | `1.50` | `0.40` | Log-normal forward velocity exploration skew |
| **19** | `ratio_center` | `float` | `0.20` | `0.80` | `0.50` | Centerline seed allocation ratio |
| **20** | `ratio_left` | `float` | `0.10` | `0.40` | `0.25` | Left apex (+25cm) seed allocation ratio |
| **21** | `pure_pursuit_lookahead` | `float` | `0.15` | `0.80` | `0.35` | Pure pursuit seeding lookahead (m) |
| **22** | `r_inflation_m`| `float` | `0.08` | `0.13` | `0.090` | C-space obstacle expansion radius (m) |

### Compute Budget Coupling Constraint
To prevent exceeding real-time compute limits, enforce:
```python
if H * K > 15420:
    K = 15420 // H
```

---

## 4. Designing the Multi-Objective Fitness Function

When evaluating a candidate chromosome over a simulation run of `T_eval` seconds (e.g. 15.0 s), compute:

```python
def compute_fitness(metrics: dict) -> float:
    # 1. Lap Completion & Progress (Positive Reward)
    # Laps completed (e.g. 1.0 = 1 full lap ~18m track length)
    progress_reward = metrics["distance_traveled_m"] * 100.0
    
    # 2. Lap Time Efficiency
    if metrics["laps_completed"] >= 1:
        time_bonus = max(0.0, (10.0 - metrics["best_lap_time_s"])) * 250.0
    else:
        time_bonus = 0.0

    # 3. Speed Maintenance
    avg_speed_reward = metrics["mean_speed_mps"] * 50.0

    # 4. Penalties (Hard Constraints)
    collision_penalty = metrics["collision_count"] * 5000.0
    wall_proximity_penalty = metrics["mean_wall_proximity_cost"] * 2.0
    steering_jerk_penalty = metrics["steering_variance"] * 20.0

    # Overall Fitness Score (Maximize)
    fitness = progress_reward + time_bonus + avg_speed_reward - collision_penalty - wall_proximity_penalty - steering_jerk_penalty
    return float(fitness)
```

---

## 5. Webots Simulation Automation Guidelines

1. **Headless Execution**:
   Run Webots in batch/headless mode for maximum training speed:
   `webots --batch --mode=fast --no-rendering "my_project/worlds/my_world.wbt"`
2. **Supervisor Reset Mechanism**:
   In `mppi_controller.py`, the controller listens for reset commands on UDP port `9878` (`{"cmd": "reset"}`). When received, it calls `robot.simulationReset()`, rewinds state, and notifies Cartographer.
3. **Early Evaluation Termination**:
   If a candidate crashes or gets stuck (`v < 0.2 m/s` for `> 1.5 s`), immediately abort the trial, assign maximum penalty, and reset to the next chromosome.

---

## 6. Recommended Optimization Algorithms

1. **CMA-ES (Covariance Matrix Adaptation Evolution Strategy)**:
   - Best for continuous parameters with complex inter-gene correlations (`cmaes` or `pycma` library).
2. **Differential Evolution (DE)**:
   - Robust global search with mutation (`CR=0.7`, `F=0.8`).
3. **NSGA-II (Non-dominated Sorting Genetic Algorithm)**:
   - Best for Pareto-optimal trade-offs between **Lap Time** vs **Collision Risk**.
