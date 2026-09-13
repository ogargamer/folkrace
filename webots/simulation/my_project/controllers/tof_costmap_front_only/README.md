# Front-Only ToF Costmap Suite (Path Planner Integration)

Optimized C++ & Python Occupancy Costmap engines for the **16-channel front-only ToF sensor rig** defined in [`only_front_tofs.proto`](file:///c:/Users/nurmi/Documents/Folkrace.worktrees/repo-restructure-tiered-folders/webots/simulation/my_project/protos/only_front_tofs.proto).

---

## 1. Files in This Folder

This folder contains only the runtime engines and assets required by the path planners:

```
tof_costmap_front_only/
├── cpp_smart_hybrid_costmap.py    # Python ctypes bridge (import CppSmartHybridCostmap)
├── smart_hybrid_costmap_front.dll # Compiled AVX2 C++ shared library (sub-1.5 ms / ~650 Hz)
├── smart_hybrid_costmap_front.cpp # C++ Costmap Maker source (16 front ToFs, 64 cross-pairs)
├── compile_dll.py                 # Convenience script to recompile DLL using workspace g++
├── fast_tof_lut_costmap.py        # Vectorized Python LUT costmap (0.17 ms / ~5,800 Hz)
├── generate_tof_lut.py            # Script to regenerate front_tofs_lut_20mm.npz
├── front_tofs_lut_20mm.npz        # Precomputed compressed polar beam lookup table
└── README.md                      # This documentation
```

---

## 2. Sensor Layout (`only_front_tofs.proto`)

All 16 sensors are mounted on the chassis front edge at $Y = +0.095\text{ m}$ and $Z = 0.0275\text{ m}$:
- **8 Front-Left Sensors** at $X = -0.070\text{ m}$ (cross-pointing Right):
  - `tof_front_left_4.1deg` ($\theta = 1.4987$ rad, 85.87°)
  - `tof_front_left_15.4deg` ($\theta = 1.3024$ rad, 74.62°)
  - `tof_front_left_26.6deg` ($\theta = 1.1060$ rad, 63.37°)
  - `tof_front_left_37.9deg` ($\theta = 0.9097$ rad, 52.12°)
  - `tof_front_left_49.1deg` ($\theta = 0.7134$ rad, 40.87°)
  - `tof_front_left_60.4deg` ($\theta = 0.5170$ rad, 29.62°)
  - `tof_front_left_71.6deg` ($\theta = 0.3207$ rad, 18.37°)
  - `tof_front_left_82.9deg` ($\theta = 0.1243$ rad, 7.12°)
- **8 Front-Right Sensors** at $X = +0.070\text{ m}$ (cross-pointing Left):
  - `tof_front_right_4.1deg` ($\theta = 1.6428$ rad, 94.13°)
  - `tof_front_right_15.4deg` ($\theta = 1.8391$ rad, 105.37°)
  - `tof_front_right_26.6deg` ($\theta = 2.0356$ rad, 116.63°)
  - `tof_front_right_37.9deg` ($\theta = 2.2318$ rad, 127.87°)
  - `tof_front_right_49.1deg` ($\theta = 2.4282$ rad, 139.12°)
  - `tof_front_right_60.4deg` ($\theta = 2.6245$ rad, 150.37°)
  - `tof_front_right_71.6deg` ($\theta = 2.8208$ rad, 161.62°)
  - `tof_front_right_82.9deg` ($\theta = 3.0172$ rad, 172.87°)

---

## 3. Coordinate Frame & Robot World Orientation

- **Robot Body Frame**:
  - $+Y_{\text{body}}$: Forward (longitudinal driving direction)
  - $+X_{\text{body}}$: Right (lateral axis)
  - $-X_{\text{body}}$: Left
  - $+Z_{\text{body}}$: Up
- **Webots World Frame (90° Clockwise / Right Import Rotation)**:
  - In `only_front_tofs.proto`: `rotation 0 0 1 -1.5707953071795862`.
  - The vehicle is angled $90^\circ$ to the right in the world (facing $+X_{\text{world}}$).
  - Transformation from local body coordinates $(x_b, y_b, z_b)$ to world coordinates $(X_w, Y_w, Z_w)$:
    $$X_w = X_{\text{robot}} + y_b, \quad Y_w = Y_{\text{robot}} - x_b, \quad Z_w = Z_{\text{robot}} + z_b$$

---

## 4. Usage with Path Planners

```python
import os
import sys

# Add costmap directory to sys.path
COSTMAP_DIR = r"c:\Users\nurmi\Documents\Folkrace.worktrees\repo-restructure-tiered-folders\webots\simulation\my_project\controllers\tof_costmap_front_only"
if COSTMAP_DIR not in sys.path:
    sys.path.insert(0, COSTMAP_DIR)

from cpp_smart_hybrid_costmap import CppSmartHybridCostmap, SENSOR_NAMES

# Initialize 15mm C++ Costmap engine
costmap = CppSmartHybridCostmap(
    grid_bounds=(-1.25, 1.25, 0.0, 2.0),
    resolution=0.015,
    r_inflation_m=0.088,
)

# In planner control loop:
prob_map, hazard_mask = costmap.process_frame(tof_readings)

# Fast collision checking for a candidate trajectory
is_collision_free = costmap.probe_path(trajectory_xs, trajectory_ys)
```
