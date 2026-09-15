# DA WINNA Launcher Suite

This directory contains consolidated, one-click launchers for simulation, genetic algorithm optimization, physical microcontroller hardware diagnostics (ESP32 & STM32), and Raspberry Pi 5 robotics deployment.

## Hardware & Target Conventions
All hardware-dependent launchers explicitly identify their target platform in the file name:
- **`esp32`**: ESP32-S3 microcontroller firmware / sensors / motor suites
- **`stm32`**: STM32 microcontroller firmware / motor suites
- **`pi`**: Raspberry Pi 5 onboard companion computer stacks

---

## Overview of Launchers

| # | Target | Launcher Script | Description |
|---|--------|----------------|-------------|
| **1** | Simulation | [`1_sl_mpc_studio.py`](./1_sl_mpc_studio.py) | Launches Webots simulation (`track_racing.wbt`) with C++ Curvilinear S-L MPC and spawns the live 2D telemetry GUI studio. |
| **2** | Simulation | [`2_champion_preview.py`](./2_champion_preview.py) | Launches Webots 3D champion preview (`track_racing.wbt`) with 3 dynamic obstacles AND live 2D telemetry GUI simultaneously. |
| **3** | Simulation & Telemetry | [`3_planner_gui.py`](./3_planner_gui.py) | Launches Webots simulation (`track_racing.wbt`) AND PyQt5 Live Telemetry & Costmap Tuner GUI simultaneously. |
| **4** | GA Training | [`4_ga_dashboard.py`](./4_ga_dashboard.py) | Standalone real-time Genetic Algorithm / CMA-ES convergence & fitness monitor dashboard. |
| **5** | GA Training | [`5_ga_optimizer.py`](./5_ga_optimizer.py) | Headless Genetic Algorithm optimization runner for training new champion genomes. |
| **6** | **ESP32** | [`6_esp32_tof_gui.py`](./6_esp32_tof_gui.py) | Physical ESP32 VL53L8CX 16-channel ToF sensor live distance matrix visualizer and calibration GUI over USB serial. |
| **7** | **ESP32** | [`7_esp32_encoder_motor_gui.py`](./7_esp32_encoder_motor_gui.py) | ESP32 MT6816 SPI magnetic angle encoder & motor closed-loop bench calibration GUI (SimpleFOC). |
| **8** | **STM32** | [`8_stm32_encoder_motor_gui.py`](./8_stm32_encoder_motor_gui.py) | STM32 MT6816 SPI magnetic angle encoder & motor closed-loop bench calibration GUI (SimpleFOC). |
| **9** | **Pi 5** | [`9_pi_cartographer_slam.py`](./9_pi_cartographer_slam.py) | 1-click Raspberry Pi 5 SSH deployment for Google Cartographer 2D SLAM (LiDAR + IMU + Web dashboard). |
| **10** | **Pi 5** | [`10_pi_imu_server.py`](./10_pi_imu_server.py) | ESP32 UART BNO08X IMU 3D visualizer server on Raspberry Pi 5 (Port 8000). |
| **11** | Tool | [`11_track_studio.py`](./11_track_studio.py) | Modular 1200mm Race Track Designer Studio (Local Web UI on port 8088). |
