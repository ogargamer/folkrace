# Webots Simulation & Control Architecture

This directory contains the Webots physics simulation environments, robot models, autonomous controllers, C++ Curvilinear SL-MPC trajectory optimizers, genetic algorithm optimization pipelines, and validation test benches for the Folkrace vehicle.

---

## 1. Coordinate Systems & Sign Conventions

All Webots controllers, MPC algorithms, sensor fusion models, and recovery managers adhere strictly to the following conventions:

### A. Robot Body Frame (Vehicle Local Frame)
* **+Y Axis**: **Forward** (Longitudinal vehicle motion)
* **-Y Axis**: **Backward** (Reverse motion)
* **+X Axis**: **Right** (Lateral displacement to the right of the vehicle)
* **-X Axis**: **Left** (Lateral displacement to the left of the vehicle)
* **+Z Axis**: **Up** (Normal to track surface)

### B. Steering Angle Convention (`cmd_steer_rad`, `steer_deg`)
* **Positive (+)**: **Steer LEFT** (Counter-Clockwise / CCW yaw turn)
* **Negative (-)**: **Steer RIGHT** (Clockwise / CW yaw turn)

### C. Yaw & Angular Rates (`yaw`, `gyro_z`, `theta_g`)
* **Positive (+)**: **Turning Left** (CCW)
* **Negative (-)**: **Turning Right** (CW)

---

## 2. Simulation Architecture & Controller Suite

The Webots simulation stack models the distributed physical architecture (Raspberry Pi 4 brain, ESP32 dynamics, and quad ToF perception hub):

* **`simulation/my_project/controllers/tof_path_planner/`**:
  * **Curvilinear SL-MPC (`planner_sl_mpc.py` & `cpp/curvilinear_sl_mpc.cpp`)**: High-performance Frenet-frame trajectory planner formulated as a Quadratic Program (QP) and solved via OSQP in compiled C++ (`curvilinear_sl_mpc.dll`). Matches the deployment on the Raspberry Pi 4.
  * **Quad ToF Perception Model (`cpp_smart_hybrid_costmap.py`)**: Simulates the 4x VL53L8CX multizone ToF array (16 distance readings), performing closest-obstacle extraction and dynamic costmap updates.
  * **4WD Electronic Differential (`electronic_diff_4wd.py`)**: Replicates the ESP32 Main dynamics engine, computing individual wheel velocities for all 4 direct-drive motors under true Ackermann geometry.
  * **Anti-Wrong-Way Watchdog (`anti_wrong_way_watchdog.py`)**: Monitors track heading progress and commands immediate 180-degree recovery spins if inverted.
  * **Stuck Recovery Manager (`stuck_recovery_manager.py`)**: Multi-stage escape state machine executing reverse maneuvers and clearance scans if the vehicle stalls.
* **`optimization/ga_planner_optimizer/`**:
  * Multi-core headless Genetic Algorithm tuning pipeline optimizing MPC weights, lookahead horizons, and speed profiles.
* **`worlds/`**:
  * Active Webots simulation worlds (e.g. `track_racing.wbt` with modular 1200mm track walls and dynamic obstacles).

---

## 3. Directory Layout

```
webots/
├── simulation/
│   └── my_project/
│       ├── controllers/
│       │   └── tof_path_planner/     # Main autonomous driving controller suite
│       │       ├── cpp/              # C++ SL-MPC & OSQP solver source code
│       │       └── data/             # Track reference curves and champion genomes
│       ├── worlds/                   # Active simulation worlds (track_racing.wbt)
│       └── protos/                   # Robot model and dynamic obstacle protos
├── optimization/
│   └── ga_planner_optimizer/         # Multi-core genetic algorithm parameter optimizer
├── robotics/
│   └── ros2_cartographer/            # Legacy / experimental mapping reference (Note: Physical car uses ToF, no LiDAR)
└── assets/                           # CAD models, 3D meshes, and track geometries
```

---

## 4. Quick Usage

Launch the full simulation stack with live telemetry from the repository root:

```bash
# Launch champion drive with 3 dynamic obstacles and live telemetry GUI
python launchers/1_champion_preview.py

# Launch ultra-lean champion drive without GUI overhead
python launchers/2_fast_champion_drive.py

# Launch headless Genetic Algorithm optimizer dashboard
python launchers/3_ga_dashboard.py
```
