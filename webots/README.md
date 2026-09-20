# Webots Simulation & Control Architecture

This directory contains the Webots simulation environments, robot models, controllers, C++ MPC trajectory optimizers, genetic optimization pipelines, and validation benches for the Folkrace robot.

---

## 1. Coordinate Systems & Sign Conventions

All Webots controllers, MPC algorithms, sensor fusion models, and recovery managers adhere strictly to the following conventions:

### A. Robot Body Frame (Vehicle Local Frame)
* **$+Y$ Axis**: **Forward** (Longitudinal vehicle motion)
* **$-Y$ Axis**: **Backward** (Reverse motion)
* **$+X$ Axis**: **Right** (Lateral displacement to the right of the vehicle)
* **$-X$ Axis**: **Left** (Lateral displacement to the left of the vehicle)
* **$+Z$ Axis**: **Up** (Normal to track surface)

### B. Steering Angle Convention (`cmd_steer_rad`, `steer_deg`)
* **Positive ($+$)**: **Steer LEFT** (Counter-Clockwise / CCW yaw turn)
* **Negative ($-$)**: **Steer RIGHT** (Clockwise / CW yaw turn)

### C. Yaw & Angular Rates (`yaw`, `gyro_z`, `theta_g`)
* **Positive ($+$)**: **Turning Left** (CCW)
* **Negative ($-$)**: **Turning Right** (CW)

---

## 2. Directory Structure

* **`simulation/my_project/`**:
  * `controllers/tof_path_planner/`: Main autonomous driving controller suite (Curvilinear SL-MPC, ToF Gap Scout, Dynamic Obstacle Tracker, Stuck Recovery Manager, Electronic Differential 4WD, Sensor Fusion).
  * `controllers/tof_path_planner/cpp/`: High-performance C++ implementation of Curvilinear SL-MPC and collision safety shield.
  * `worlds/`: Active Webots simulation worlds (e.g. `track_racing.wbt`).
  * `protos/`: Robot and obstacle proto definitions.
* **`optimization/`**:
  * `ga_planner_optimizer/`: High-throughput genetic algorithm optimizer tuning MPC weights, lookahead, and control parameters.
* **`robotics/`**:
  * ROS2 and Cartographer mapping configurations.
* **`assets/`**:
  * 3D meshes, textures, track definitions, and CAD exports.
