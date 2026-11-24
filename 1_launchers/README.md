# Folkrace Launcher Suite

This directory contains consolidated, human-facing one-click launchers for 3D physics simulation, Genetic Algorithm parameter optimization, Raspberry Pi 4 flight operations, and physical microcontroller bench testing (ESP32 and STM32).

> [!NOTE]
> All human-facing runnable GUIs and hardware test launchers reside in this directory per project file hygiene rules. AI/automation CLI utilities live in [`tools/`](file:///c:/Users/nurmi/Documents/Folkrace/tools/) (e.g., [`tools/ai_debug_suite.py`](file:///c:/Users/nurmi/Documents/Folkrace/tools/ai_debug_suite.py)).

---

## 1. Ordering & Future Renumbering

Launchers use leading numeric prefixes (`1_`–`9_`) to preserve workflow order in file explorers and terminals:
* **`1–3` — Simulation & Training**: Webots 3D previews and GA / CMA-ES optimization dashboard.
* **`4–5` — Pi 4 Operations & Replay**: Live Tactical Mission Control and offline 60 Hz flight log replay.
* **`6–8` — ESP32 Benches**: Quad VL53L8CX ToF Hub, BNO086 9-DOF IMU dashboard, and 333 Hz digital steering servo.
* **`9` — STM32 Bench**: Dual-motor direct-drive FOC & MT6816 magnetic encoders.

Internal cross-imports (e.g., [`5_local_flight_replay.py`](file:///c:/Users/nurmi/Documents/Folkrace/1_launchers/5_local_flight_replay.py) and [`tools/ai_debug_suite.py`](file:///c:/Users/nurmi/Documents/Folkrace/tools/ai_debug_suite.py)) discover `*pi4_debug_studio.py` dynamically, so reordering numeric prefixes in the future will not break Python imports.

---

## 2. Master Launcher Catalog

| # | Category | Launcher Script | Description | Primary Target |
| :---: | :--- | :--- | :--- | :--- |
| **1** | Simulation | [`1_champion_preview.py`](file:///c:/Users/nurmi/Documents/Folkrace/1_launchers/1_champion_preview.py) | Launches Webots 3D champion preview (`track_racing.wbt`) with dynamic obstacle avoidance AND live 2D telemetry radar simultaneously. | Webots / Python |
| **2** | Simulation | [`2_fast_champion_drive.py`](file:///c:/Users/nurmi/Documents/Folkrace/1_launchers/2_fast_champion_drive.py) | Ultra-lean champion 3D drive launcher for maximum real-time rendering & physics performance (zero GUI, zero UDP/HUD overhead). | Webots |
| **3** | GA Training | [`3_ga_dashboard.py`](file:///c:/Users/nurmi/Documents/Folkrace/1_launchers/3_ga_dashboard.py) | Standalone real-time Genetic Algorithm / CMA-ES convergence, fitness progress, and generational metric dashboard. | Workstation |
| **4** | **Pi 4** | [`4_pi4_debug_studio.py`](file:///c:/Users/nurmi/Documents/Folkrace/1_launchers/4_pi4_debug_studio.py) | Raspberry Pi 4 Dual-Core Debug Studio: Web Tactical Mission Control & SQLite Black Box with frame-by-frame time-travel scrubbing. | Pi 4 / Browser |
| **5** | **Pi 4** | [`5_local_flight_replay.py`](file:///c:/Users/nurmi/Documents/Folkrace/1_launchers/5_local_flight_replay.py) | Offline 60 Hz SQLite flight log replay viewer (plus `--digest` and `--export` analysis modes). | Workstation / Browser |
| **6** | **ESP32** | [`6_esp32_tof_gui.py`](file:///c:/Users/nurmi/Documents/Folkrace/1_launchers/6_esp32_tof_gui.py) | Physical ESP32 quad VL53L8CX 4-zone multi-target telemetry visualizer, SPAD saturation alerts, and live 2D tactical radar over USB serial. | ESP32-S3 (ToF Hub) |
| **7** | **ESP32** | [`7_esp32_imu_gui.py`](file:///c:/Users/nurmi/Documents/Folkrace/1_launchers/7_esp32_imu_gui.py) | Physical ESP32 BNO086 9-DOF IMU 3D mission dashboard, artificial horizon, compass, G-force, and real-time waveform GUI. | ESP32-S3 (Main) |
| **8** | **ESP32** | [`8_esp32_servo_gui.py`](file:///c:/Users/nurmi/Documents/Folkrace/1_launchers/8_esp32_servo_gui.py) | Physical ESP32 333 Hz high-rate digital steering servo test bench, center trimming, and dynamic frequency sweep GUI. | ESP32-S3 (Main) |
| **9** | **STM32** | [`9_stm32_encoder_motor_gui.py`](file:///c:/Users/nurmi/Documents/Folkrace/1_launchers/9_stm32_encoder_motor_gui.py) | STM32G474 MT6816 SPI magnetic angle encoder & motor closed-loop bench calibration GUI (Dual-motor FOC tuning). | STM32G474 (Axle MCUs) |
| **10** | Tool | [`10_track_studio.py`](file:///c:/Users/nurmi/Documents/Folkrace/1_launchers/10_track_studio.py) | Modular 1200mm Race Track Designer Studio (Local Web UI on port 8088) for designing standard Folkrace courses, exporting ToF Webots `.wbt` worlds, and saving local `data/maps/` bundles. | Browser / http.server |

---

## 3. Quick Usage Examples

### Running 3D Simulation with Curvilinear SL-MPC:
```bash
python 1_launchers/1_champion_preview.py
```
* Launches Webots with the dynamic obstacle world (`track_racing.wbt`) and companion 2D telemetry radar GUI.
* Use `python 1_launchers/2_fast_champion_drive.py` for maximum real-time FPS without GUI/UDP overhead.

### Raspberry Pi 4 Dual-Core Debug Studio & Offline Replay:
```bash
python 1_launchers/4_pi4_debug_studio.py
python 1_launchers/5_local_flight_replay.py
```
* Auto-detects the Pi 4 (`robot`), opens Web Tactical Mission Control (`http://<pi>:8080`), and supports flight log downloads and offline 60 Hz scrubbing.

### Benchmarking Quad VL53L8CX ToF Array:
```bash
python 1_launchers/6_esp32_tof_gui.py
```
* Connects to the ESP32 ToF Hub over USB serial (921,600 baud) and visualizes all 4 ToF sensors (16 ray projections, multi-target status, and SPAD counts).

### High-Rate Digital Steering Servo Calibration:
```bash
python 1_launchers/8_esp32_servo_gui.py
```
* Connects to ESP32 Main over USB serial to tune center trim, deflection limits, and 333 Hz response dynamics.

### Benchmarking Physical STM32 Direct-Drive Motors:
```bash
python 1_launchers/9_stm32_encoder_motor_gui.py
```
* Connects over native USB CDC or UART, visualizes MT6816 magnetic angle and RPM response, and runs automated 128-point LUT calibration.
