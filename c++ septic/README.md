# Backup: C++ Septic Path Planner & Best Parameters (2026-09-06)

This directory contains a dedicated standalone backup of the C++ Septic trajectory planner, C++ friction/acceleration profilers, champion GA parameters, live visualization interfaces, and recovery controllers.

---

## 1. Directory Structure

- **`planners/`**:
  - `cpp_planner_option3_septic.py`: Python wrapper and controller interface for the C++ Septic Spline planner.
  - `septic_traction_profiler.cpp` & `.dll` (v1-v5): High-speed C++ G^3 continuous septic polynomial spline generation and bidirectional traction profiler.
  - `smart_hybrid_costmap_15mm.cpp` & `.dll`: 15mm high-resolution hybrid costmap generator.
  - `stuck_recovery_manager.py`: Autonomous recovery manager featuring 9cm emergency trigger and hierarchical multi-tier resolution (Tier 1 -> Tier 3 -> Tier 4 -> Tier 6).
  - `tof_gap_scout.py`: 16-channel ToF spatial gap and obstacle clearance scout.
  - `imu_encoder_fusion.py`: Real-time state estimator and fusion filter.
  - `electronic_diff_4wd.py`: 4WD torque and wheel speed vectoring.
  - `planner_config.py`: Septic planner configuration constants and thresholds.

- **`best_genomes_and_configs/`**:
  - `best_planner_genome.json`: Top non-crashed Generation 175 champion genome (fitness: 5214.71, 3 laps, 47.943 m, 2.594 m/s average speed).
  - `best_genome_option_3_septic.json`: Septic planner specific chromosome.
  - `ga_checkpoint.json`: Full GA checkpoint containing all evaluated generations and population states.
  - `ga_history.csv`: Generation-by-generation evolution metrics.
  - `ga_planner_config.py`: Parameter bounds, weights, and genetic hyperparameters.
  - `ga_optimizer.py`, `ga_evaluator.py`, `ga_dashboard_gui.py`: Genetic optimizer and evaluation pipeline.

- **`live_preview_and_launchers/`**:
  - `champion_live_preview.py`: Webots robot controller for previewing the champion genome in real time.
  - `ga_supervisor_worker.py`: Webots supervisor worker for automated chromosome evaluation.
  - `launch_septic_live_map.py`: Real-time 2D canvas live preview GUI with UDP telemetry.
  - `launch_planner_gui.py`: Interactive planner visualizer.
  - `launch_champion_preview.py`: Headless/interactive Webots launcher.

---

## 2. Key Champion Parameters Summary (Generation 175)

- **mu_nominal**: 1.25
- **max_lateral_accel**: 20.27 m/s^2
- **max_longitudinal_accel**: 8.27 m/s^2
- **max_longitudinal_decel**: 14.50 m/s^2
- **base_lookahead_m**: 0.90 m
- **max_speed_cap_mps**: 4.00 m/s
- **speed_lookahead_gain**: 0.28 s
- **spline_tension**: 0.44
- **safety_margin_m**: 0.08 m
- **k_p_steer**: 1.40
- **k_d_steer**: 0.06

---

## 3. Emergency Stuck Recovery Logic

- **Immediate Backup Trigger**: Whenever `tof_fl_3` or `tof_fr_3` <= 0.09 m (9 cm).
- **Escape Direction Priority**:
  - Tier 1: Primary front-center `abs(fr3 - fl3) > 0.04 m` -> steer away from closer side.
  - Tier 3: Lateral rear-outer `abs(br0 - bl0) > 0.04 m` -> steer toward side with more rear clearance.
  - Tier 4: Lateral rear-inner `abs(br1 - bl1) > 0.04 m` -> steer toward side with more inner clearance.
  - Tier 6: Track centerline heading error -> steer in direction of forward track flow.
