# DA WINNA Launcher Suite

This directory contains consolidated, one-click launchers for simulation, genetic algorithm optimization, physical hardware diagnostics, and robotics SLAM deployment.

## Overview of Launchers

| # | Launcher Script | Batch (1-Click) | Purpose |
|---|----------------|-----------------|---------|
| **1** | [`1_sl_mpc_studio.py`](./1_sl_mpc_studio.py) | `1_sl_mpc_studio.bat` | Launches Webots simulation (`track_racing.wbt`) with C++ Curvilinear S-L MPC and spawns the live 2D telemetry GUI studio. |
| **2** | [`2_champion_preview.py`](./2_champion_preview.py) | `2_champion_preview.bat` | Visual 3D replay preview (`track_preview.wbt`) with 3 dynamic moving obstacles and the champion CMA-ES genome. |
| **3** | [`3_planner_gui.py`](./3_planner_gui.py) | `3_planner_gui.bat` | Standalone PyQt5 Live Telemetry & Costmap Tuner GUI listening on UDP port 9876. |
| **4** | [`4_ga_dashboard.py`](./4_ga_dashboard.py) | `4_ga_dashboard.bat` | Standalone real-time Genetic Algorithm / CMA-ES convergence & fitness monitor dashboard. |
| **5** | [`5_ga_optimizer.py`](./5_ga_optimizer.py) | `5_ga_optimizer.bat` | Headless Genetic Algorithm optimization runner for training new champion genomes. |
| **6** | [`6_hardware_tof_gui.py`](./6_hardware_tof_gui.py) | `6_hardware_tof_gui.bat` | Physical ESP32 VL53L8CX 16-channel ToF sensor live distance matrix visualizer and calibration GUI over serial. |
| **7** | [`7_hardware_encoder_gui.py`](./7_hardware_encoder_gui.py) | `7_hardware_encoder_gui.bat` | MT6816 SPI magnetic angle encoder & brushless motor closed-loop bench calibration GUI (SimpleFOC). |
| **8** | [`8_pi_cartographer_slam.py`](./8_pi_cartographer_slam.py) | `8_pi_cartographer_slam.bat` | 1-click Raspberry Pi 5 SSH deployment for Google Cartographer 2D SLAM (LiDAR + IMU + Web dashboard). |
| **9** | [`9_track_studio.py`](./9_track_studio.py) | `9_track_studio.bat` | Modular 1200mm Race Track Designer Studio (Local Web UI on port 8088). |
