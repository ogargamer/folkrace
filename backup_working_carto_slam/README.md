# Working Google Cartographer 2D SLAM Pipeline Backup

This self-contained folder contains all files, configurations, sensor logs, and controller scripts required to reproduce the clean Google Cartographer 2D SLAM map on `Cyan_Champion_Track_Static.wbt`.

---

## Directory Structure

```
backup_working_carto_slam/
├── run_carto_slam.py                  # Master runner script for Webots and WSL Cartographer
├── ros2_scripts/
│   ├── feed_pure_lidar_imu.py         # ROS2 node streaming 360° LiDAR & IMU to Cartographer
│   ├── export_cartographer_png.py     # Subscribes to /map and saves PNG, PGM, and YAML
│   └── run_temp_sweep.sh              # WSL execution wrapper
├── config/
│   ├── folkrace_2d.lua                # Production Cartographer 2D configuration
│   └── sweep_temp.lua                 # Active parameter configuration
├── controller/
│   ├── folkrace_drive.py              # 1D-QP Frenet corridor vehicle controller
│   ├── folkrace_config.json           # Vehicle settings (80% race pace locked)
│   ├── electronic_diff_4wd.py         # 4WD Ackermann kinematics & slip regulation
│   ├── tof_pathfinders.py             # Frenet transformation and QP pathfinders
│   ├── predictive_shield.py           # Safety shield and TTC governor
│   └── live_webots_slam_scans.jsonl   # Pre-recorded 3-lap sensor stream (450-ray LiDAR + IMU)
├── world/
│   └── Cyan_Champion_Track_Static.wbt # Clean static benchmark world file
└── results/
    ├── map_01_15mm_motDist20mm_rotW30_loop036.png # High-contrast visual SLAM render
    ├── cartographer_2d_slam_static_map.yaml       # Standard ROS2 map metadata
    └── cartographer_2d_slam_static_map.pgm        # 2.0 cm discrete occupancy grid
```

---

## Key Configurations & Settings

- **Vehicle Driver**: `folkrace_drive` running the `FRENETQP` 1D-Corridor pathfinder at `80.0%` race pace.
- **LiDAR Sensor**: 360° `ms200k_lidar` (450 rays, `max_range = 3.90 m`).
  - Angle min: `+3.1416 rad (+pi)`
  - Angle max: `-3.1416 rad (-pi)`
  - Angle increment: `-0.013994 rad (-2*pi / 449)` (Clockwise Cyberbotics convention).
- **IMU Sensor**: 6-axis gyro & accelerometer with `orientation_covariance[0] = -1.0`.
- **Cartographer Settings**:
  - `submaps_num_range_data = 36`
  - `ceres_occupied_space_weight = 52.5`
  - `hit_probability = 0.78`, `miss_probability = 0.38`
  - `loop_closure_min_score = 0.36`
  - `motion_filter_max_dist_m = 0.02`

---

## How to Run

1. **Replay Existing Sensor Scans through Cartographer**:
   In `run_carto_slam.py`, set `"re_run_webots": False` and run:
   ```bash
   python run_carto_slam.py
   ```

2. **Re-run Full Simulation + SLAM**:
   In `run_carto_slam.py`, set `"re_run_webots": True` and run:
   ```bash
   python run_carto_slam.py
   ```
