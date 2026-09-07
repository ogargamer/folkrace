# MPPI & Costmap GUI System Backup

This directory contains a self-contained backup of the complete MPPI control, perception, and live visualization suite.

---

## Included Components

1. **Launcher & Visualizer**:
   - `launch_costmap_gui.py`: Main 1-click launcher handling automatic WSL Cartographer daemon lifecycle and PyQt6 GUI.
   - `webots/simulation/my_project/controllers/tof_16ch_grid_scanner/costmap_gui.py`: 30 FPS PyQt6 real-time rolling costmap visualizer.

2. **Perception & SLAM**:
   - `webots/simulation/my_project/controllers/tof_16ch_grid_scanner/fast_tof_carto_costmap.py`: 60 Hz 20mm Bayesian log-odds costmap fusion engine.
   - `webots/robotics/ros2_cartographer/scripts/webots_cartographer_bridge.py`: Dynamic cross-VM UDP bridge node streaming /scan, /imu, /map, and /tracked_pose.
   - `webots/robotics/ros2_cartographer/scripts/run_live_wsl_bridge.sh`: Background WSL shell launcher.
   - `webots/robotics/ros2_cartographer/config/folkrace_2d.lua`: 2D Cartographer SLAM configuration.

3. **Controllers & Acceleration Core**:
   - `webots/simulation/my_project/controllers/mppi_controller/mppi_controller.py`: Webots supervisor vehicle controller (50 Hz synchronized telemetry).
   - `webots/simulation/my_project/controllers/mppi_controller/fast_mppi_core.py`: Native LLVM SIMD JIT MPPI rollout kernel (<0.5 ms for 512 samples).
   - `webots/simulation/my_project/controllers/folkrace_drive/mppi_folkrace_controller.py`: Multi-modal lane seeding (Center, Left, Right) & 23-gene MPPIGenome.
   - `webots/simulation/my_project/controllers/folkrace_drive/tof_pathfinders.py`: Frenet coordinate transformer.

4. **GA Reference**:
   - `GA_DEVELOPMENT_REFERENCE_GUIDE.md`: Full architectural specification for automated Genetic Algorithm tuning.

---

## Quick Launch
```powershell
python launch_costmap_gui.py
```
