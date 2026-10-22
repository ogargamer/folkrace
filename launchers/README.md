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
| **1** | Simulation | [`1_champion_preview.py`](./1_champion_preview.py) | Launches Webots 3D champion preview (`track_racing.wbt`) with 3 dynamic obstacles AND live 2D telemetry GUI simultaneously. |
| **2** | GA Training | [`2_ga_dashboard.py`](./2_ga_dashboard.py) | Standalone real-time Genetic Algorithm / CMA-ES convergence & fitness monitor dashboard. |
| **3** | **ESP32** | [`3_esp32_tof_gui.py`](./3_esp32_tof_gui.py) | Physical ESP32 VL53L8CX 16-channel ToF sensor live distance matrix visualizer and calibration GUI over USB serial. |
| **4** | **ESP32** | [`4_esp32_encoder_motor_gui.py`](./4_esp32_encoder_motor_gui.py) | ESP32 MT6816 SPI magnetic angle encoder & motor closed-loop bench calibration GUI (SimpleFOC). |
| **5** | **STM32** | [`5_stm32_encoder_motor_gui.py`](./5_stm32_encoder_motor_gui.py) | STM32 MT6816 SPI magnetic angle encoder & motor closed-loop bench calibration GUI (SimpleFOC). |
| **6** | **Pi 5** | [`6_pi_cartographer_slam.py`](./6_pi_cartographer_slam.py) | 1-click Raspberry Pi 5 SSH deployment for Google Cartographer 2D SLAM (LiDAR + IMU + Web dashboard). |
| **7** | **Pi 5** | [`7_pi_imu_server.py`](./7_pi_imu_server.py) | ESP32 UART BNO08X IMU 3D visualizer server on Raspberry Pi 5 (Port 8000). |
| **8** | Tool | [`8_track_studio.py`](./8_track_studio.py) | Modular 1200mm Race Track Designer Studio (Local Web UI on port 8088). |
| **9** | Simulation | [`9_wrong_way_studio.py`](./9_wrong_way_studio.py) | Anti-Wrong-Way Watchdog & 180 Auto-Correction Studio (`test_wrong_way.wbt` + 2D Tactical Radar & Thought Matrix GUI). |
| **10** | Simulation | [`10_stationary_obstacle_test.py`](./10_stationary_obstacle_test.py) | Phase 1 Stationary Robot Test Bench: Dynamic moving obstacle detection, wall rejection, velocity tracking & vector enlargement. |
| **11** | Simulation | [`11_moving_robot_obstacle_test.py`](./11_moving_robot_obstacle_test.py) | Phase 2 Moving Robot Test Bench: Ego-motion compensated dynamic obstacle detection & vector enlargement at race speeds. |
| **12** | Simulation | [`12_phase3_live_webots.py`](./12_phase3_live_webots.py) | Phase 3 Live Webots Rendered Test: Fully rendered 3D simulation with moving robot, interactive dynamic obstacle modes, and live 2D radar GUI. |
| **13** | Simulation | [`13_fast_champion_drive.py`](./13_fast_champion_drive.py) | Ultra-lean champion 3D drive launcher for maximum real-time rendering & physics performance (zero GUI, zero UDP/HUD overhead). |
| **14** | **ESP32** | [`14_esp32_tof_arc_gui.py`](./14_esp32_tof_arc_gui.py) | Physical ESP32 dual VL53L8CX 8-zone 90° top-down 2D radar visualizer with proximity blind-spot / open-space fallback. |

