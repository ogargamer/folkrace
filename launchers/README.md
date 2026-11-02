# Folkrace Launcher Suite

This directory contains consolidated, one-click launchers for 3D physics simulation, Genetic Algorithm parameter optimization, physical microcontroller bench testing (ESP32 and STM32), track design, and high-rate sensor calibration.

> [!NOTE]
> All runnable scripts, dashboards, and hardware test interfaces must reside exclusively in this directory per project file hygiene rules.

---

## 1. Hardware & Platform Conventions

Hardware-dependent diagnostic launchers explicitly denote their target hardware platform in their filename:
* **`esp32`**: ESP32-S3 microcontroller firmware / sensors (ESP32 Main dynamics, servo, IMU, and ESP32 ToF Hub quad VL53L8CX sensors).
* **`stm32`**: STM32G474 microcontroller firmware / motor suites (dual-motor direct-drive FOC & MT6816 magnetic encoders).
* **`simulation` / `tool`**: Workstation or companion computer (Raspberry Pi 4) simulation and path planning utilities.

---

## 2. Master Launcher Catalog

| # | Category | Launcher Script | Description | Primary Target |
| :---: | :--- | :--- | :--- | :--- |
| **1** | Simulation | [`1_champion_preview.py`](file:///c:/Users/nurmi/Documents/Folkrace/launchers/1_champion_preview.py) | Launches Webots 3D champion preview (`track_racing.wbt`) with dynamic obstacle avoidance AND live 2D telemetry radar simultaneously. | Webots / Python |
| **2** | GA Training | [`2_ga_dashboard.py`](file:///c:/Users/nurmi/Documents/Folkrace/launchers/2_ga_dashboard.py) | Standalone real-time Genetic Algorithm / CMA-ES convergence, fitness progress, and generational metric dashboard. | Workstation |
| **3** | **ESP32** | [`3_esp32_tof_gui.py`](file:///c:/Users/nurmi/Documents/Folkrace/launchers/3_esp32_tof_gui.py) | Physical ESP32 VL53L8CX 16-channel ToF sensor live distance matrix visualizer and calibration GUI over USB serial. | ESP32-S3 (ToF Hub) |
| **4** | **ESP32** | [`4_esp32_encoder_motor_gui.py`](file:///c:/Users/nurmi/Documents/Folkrace/launchers/4_esp32_encoder_motor_gui.py) | ESP32 MT6816 SPI magnetic angle encoder & motor closed-loop bench calibration GUI (SimpleFOC single-motor bench). | ESP32-S3 |
| **5** | **STM32** | [`5_stm32_encoder_motor_gui.py`](file:///c:/Users/nurmi/Documents/Folkrace/launchers/5_stm32_encoder_motor_gui.py) | STM32G474 MT6816 SPI magnetic angle encoder & motor closed-loop bench calibration GUI (Dual-motor FOC tuning). | STM32G474 (Axle MCUs) |
| **8** | Tool | [`8_track_studio.py`](file:///c:/Users/nurmi/Documents/Folkrace/launchers/8_track_studio.py) | Modular 1200mm Race Track Designer Studio (Local Web UI on port 8088) for designing standard Folkrace courses. | Browser / Flask |
| **9** | Simulation | [`9_wrong_way_studio.py`](file:///c:/Users/nurmi/Documents/Folkrace/launchers/9_wrong_way_studio.py) | Anti-Wrong-Way Watchdog & 180-Degree Auto-Correction Studio (`test_wrong_way.wbt` + 2D Tactical Radar & Thought Matrix GUI). | Webots / Python |
| **13** | Simulation | [`13_fast_champion_drive.py`](file:///c:/Users/nurmi/Documents/Folkrace/launchers/13_fast_champion_drive.py) | Ultra-lean champion 3D drive launcher for maximum real-time rendering & physics performance (zero GUI, zero UDP/HUD overhead). | Webots |
| **14** | **ESP32** | [`14_esp32_tof_arc_gui.py`](file:///c:/Users/nurmi/Documents/Folkrace/launchers/14_esp32_tof_arc_gui.py) | Physical ESP32 dual VL53L8CX 8-zone 90-degree top-down 2D radar visualizer with proximity blind-spot / open-space fallback. | ESP32-S3 (ToF Hub) |
| **15** | **ESP32** | [`15_esp32_servo_gui.py`](file:///c:/Users/nurmi/Documents/Folkrace/launchers/15_esp32_servo_gui.py) | Physical ESP32 333 Hz high-rate digital steering servo test bench, center trimming, and dynamic frequency sweep GUI. | ESP32-S3 (Main) |
| **16** | **ESP32** | [`16_esp32_4tof_costmap_gui.py`](file:///c:/Users/nurmi/Documents/Folkrace/launchers/16_esp32_4tof_costmap_gui.py) | Physical ESP32 quad VL53L8CX 2D costmap & multi-target radar GUI with guarded ray clearing and dual-target obstacle stamping. | ESP32-S3 (ToF Hub) |

---

## 3. Quick Usage Examples

### Benchmarking Physical STM32 Direct-Drive Motors:
```bash
python launchers/5_stm32_encoder_motor_gui.py
```
* Connects over native USB CDC or UART.
* Visualizes real-time MT6816 magnetic angle readout, electrical angle offset, and RPM response.
* Runs automated 128-point LUT calibration to eliminate motor cogging torque.

### Benchmarking Quad VL53L8CX ToF Array (16-Zone Costmap):
```bash
python launchers/16_esp32_4tof_costmap_gui.py
```
* Connects to the ESP32 ToF Hub over USB serial (921,600 baud).
* Displays polar and Cartesian obstacle radar reflecting all 4 Time-of-Flight sensors simultaneously.
* Validates noise rejection and closest-target identification before feeding data into the path planner.

### High-Rate Digital Steering Servo Calibration:
```bash
python launchers/15_esp32_servo_gui.py
```
* Connects to ESP32 Main over USB serial.
* Allows fine-tuning center trim, maximum deflection limits (plus or minus 30 degrees), and high-frequency (333 Hz) response dynamics.

### Running 3D Simulation with Curvilinear SL-MPC:
```bash
python launchers/1_champion_preview.py
```
* Automatically verifies and builds `curvilinear_sl_mpc.dll` if needed.
* Launches Webots with the dynamic obstacle world (`track_racing.wbt`).
* Starts the companion 2D tactical radar and thought matrix telemetry viewer.
