=== BUNDLE: folkrace_1of7_README.md_to_2_pi4.txt (27 files) ===
  - README.md
  - platformio.ini
  - .agents/rules/file_hygiene.md
  - .agents/rules/formatting.md
  - .agents/rules/pi_connectivity.md
  - .agents/skills/ai-flight-debug/SKILL.md
  - .agents/skills/esp32-pi-flash/SKILL.md
  - .agents/skills/pi-ssh/SKILL.md
  - .agents/skills/stm32-dfu/SKILL.md
  - .agents/skills/stm32-dfu/scripts/enter_dfu.py
  - .agents/skills/webots-run/SKILL.md
  - brain/__init__.py
  - brain/anti_wrong_way_watchdog.py
  - brain/autonomous_brain.py
  - brain/cpp_smart_hybrid_costmap.py
  - brain/mcu_telemetry_protocol.h
  - brain/planner_config.py
  - brain/planner_sl_mpc.py
  - brain/pose_tracker.py
  - brain/ramp_detector.py
  - brain/stuck_recovery_manager.py
  - brain/tof_gap_scout.py
  - brain/vehicle_config.json
  - brain/vehicle_params.py
  - 2_pi4/README.md
  - 2_pi4/auto_servo_center.py
  - 2_pi4/benchmark_sl_mpc.py

=== FILE: README.md ===
# Folkrace Autonomous High-Speed Racing Platform

An autonomous, 1:20-scale high-speed racing vehicle powered by a distributed multi-processor architecture: a **Raspberry Pi 4** companion computer running Curvilinear SL-MPC path planning, an **ESP32-S3 Main** chassis controller handling high-rate vehicle dynamics, a dedicated **ESP32-S3 ToF Hub** filtering quad multizone Time-of-Flight sensors, and **two STM32G474** microcontrollers driving 4 independent direct-drive brushless motors via hardware Field Oriented Control (FOC).

> [!IMPORTANT]
> **No LiDAR is used in this vehicle.** Perception is 100% solid-state, utilizing a quad VL53L8CX multizone Time-of-Flight array on a shared SPI bus that streams 16 distance zones directly to the Raspberry Pi 4.

---

## 1. System Hardware Architecture

```mermaid
flowchart TD
    subgraph PI4["Raspberry Pi 4 (High-Level Brain)"]
        direction TB
        MPC["Core 0/1: Curvilinear SL-MPC Path Planner<br/>(Frenet OSQP Solver @ 20-50 Hz)"]
        DBG["Core 2/3: Telemetry & Debug Server<br/>(Live Radar, HUD, WebSocket / HTTP)"]
    end

    subgraph ESP_TOF["ESP32-S3 ToF Perception Hub"]
        direction TB
        TOF_FILT["Quad VL53L8CX SPI Driver & Filter<br/>(Outlier Rejection, Multi-Target Parsing)"]
    end

    subgraph TOF_SENSORS["Solid-State Perception Array"]
        T1["Front-Left VL53L8CX"]
        T2["Front-Right VL53L8CX"]
        T3["Side-Left VL53L8CX"]
        T4["Side-Right VL53L8CX"]
    end

    subgraph ESP_MAIN["ESP32-S3 Main (Chassis & Dynamics Master)"]
        direction TB
        DYN["400 Hz Vehicle Dynamics Engine<br/>Ackermann Steering + 4WD Electronic Diff"]
        SLIP["IMU Slip Detection & Traction Control"]
        SERVO_DRV["333 Hz Digital Steering Servo PWM"]
    end

    subgraph SENSORS_ACTUATORS["Chassis I/O"]
        IMU["BNO08x 9-DOF IMU (SPI, 200 Hz)"]
        SERVO["Front Steering Servo"]
    end

    subgraph STM_FRONT["STM32G474 (Front Axle Controller)"]
        direction TB
        FOC_F["Dual-Motor FOC (SimpleFOC @ 20 kHz PWM)<br/>4 kHz Closed-Loop Velocity PID"]
        M1["M1: Front-Left GM3506 Direct-Drive"]
        M2["M2: Front-Right GM3506 Direct-Drive"]
        E1["ENC1: MT6816 SPI Magnetic Encoder"]
        E2["ENC2: MT6816 SPI Magnetic Encoder"]
    end

    subgraph STM_REAR["STM32G474 (Rear Axle Controller)"]
        direction TB
        FOC_R["Dual-Motor FOC (SimpleFOC @ 20 kHz PWM)<br/>4 kHz Closed-Loop Velocity PID"]
        M3["M3: Rear-Left GM3506 Direct-Drive"]
        M4["M4: Rear-Right GM3506 Direct-Drive"]
        E3["ENC3: MT6816 SPI Magnetic Encoder"]
        E4["ENC4: MT6816 SPI Magnetic Encoder"]
    end

    %% Sensor to ToF Hub
    T1 & T2 & T3 & T4 -->|"Shared SPI Bus + Dedicated MISO"| TOF_FILT

    %% ToF Hub to Pi 4
    TOF_FILT -->|"16 Clean Distance Readings (UART @ 921,600)"| MPC

    %% Pi 4 to ESP32 Main
    MPC -->|"Target Velocity v & Steering Delta (UART @ 921,600)"| DYN
    DYN -->|"IMU & Telemetry Feedback (100 Hz UART)"| DBG

    %% ESP32 Main I/O
    IMU -->|"Zero-Drift Quaternions & Rates (SPI)"| SLIP
    SERVO_DRV -->|"High-Rate PWM"| SERVO

    %% ESP32 Main to STM32 Axle Controllers
    DYN -->|"Front Left/Right RPM Targets (UART @ 921,600)"| FOC_F
    DYN -->|"Rear Left/Right RPM Targets (UART @ 921,600)"| FOC_R
    FOC_F -->|"Shaft RPM Telemetry"| DYN
    FOC_R -->|"Shaft RPM Telemetry"| DYN

    %% STM32 Motor Actuation
    FOC_F --> M1 & M2
    E1 & E2 --> FOC_F
    FOC_R --> M3 & M4
    E3 & E4 --> FOC_R
```

---

## 2. Distributed Subsystems Summary

| Subsystem | Hardware Platform | Primary Responsibilities | Key Communication |
| :--- | :--- | :--- | :--- |
| **Brain** | [Raspberry Pi 4](file:///c:/Users/nurmi/Documents/Folkrace/2_pi4/README.md) | High-level path planning via Curvilinear SL-MPC on Core 0/1; live telemetry and debugging server on Core 2/3. No LiDAR. | UART from ESP32 ToF (16 zones), UART with ESP32 Main |
| **Chassis Dynamics** | [ESP32 Main](file:///c:/Users/nurmi/Documents/Folkrace/3_esp32/README.md) | 400 Hz control loop: BNO08x 9-DOF IMU reading, steering servo control, 4WD electronic differential, slip/spinout watchdog. | High-speed UART to Pi 4; dual UART links to 2x STM32 |
| **Perception Hub** | [ESP32 ToF Hub](file:///c:/Users/nurmi/Documents/Folkrace/3_esp32/README.md) | Manages 4x VL53L8CX multizone ToFs on one shared SPI bus with dedicated MISO lines; filters data into 16 distance zones. | UART output to Pi 4 (921,600 baud) |
| **Front Axle Motors** | [STM32 Front](file:///c:/Users/nurmi/Documents/Folkrace/4_stm32/README.md) | Controls 2x direct-drive GM3506 BLDC motors (FL, FR) with 2x MT6816 SPI magnetic encoders via hardware SimpleFOC at 20 kHz. | UART link with ESP32 Main; SPI1/SPI2 for encoders |
| **Rear Axle Motors** | [STM32 Rear](file:///c:/Users/nurmi/Documents/Folkrace/4_stm32/README.md) | Controls 2x direct-drive GM3506 BLDC motors (RL, RR) with 2x MT6816 SPI magnetic encoders via hardware SimpleFOC at 20 kHz. | UART link with ESP32 Main; SPI1/SPI2 for encoders |

---

## 3. Directory Structure

* **[1_launchers/](file:///c:/Users/nurmi/Documents/Folkrace/1_launchers/README.md)**: Consolidated one-click launcher scripts for simulation, genetic algorithm optimization, and hardware test benches.
* **[2_pi4/](file:///c:/Users/nurmi/Documents/Folkrace/2_pi4/README.md)**: Raspberry Pi 4 companion stack, SL-MPC planner configuration, debug server notes, and pin mapping.
* **[3_esp32/](file:///c:/Users/nurmi/Documents/Folkrace/3_esp32/README.md)**:
  * **`ESP32_Main/`**: Real-time vehicle dynamics, electronic differential, BNO08x IMU driver, steering servo actuation, and STM32 UART link.
  * **`ESP32_ToF/`**: Quad VL53L8CX multi-zone Time-of-Flight driver suite, dedicated MISO bus architecture, and 16-zone distance matrix stream.
* **[4_stm32/](file:///c:/Users/nurmi/Documents/Folkrace/4_stm32/README.md)**: Dual-motor Field Oriented Control (SimpleFOC), MT6816 SPI magnetic encoder drivers, 128-point LUT calibration, and hardware timer configurations.
* **[5_webots/](file:///c:/Users/nurmi/Documents/Folkrace/5_webots/README.md)**: Webots physics simulation, Curvilinear SL-MPC C++ solvers (`curvilinear_sl_mpc.dll`), genetic algorithm tuning benches, and virtual race tracks.

---

## 4. Hardware Pinout Reference (STM32G474 LQFP48 Dual-Motor FOC)

Each STM32 board controls two direct-drive brushless motors and two MT6816 magnetic angle encoders using dedicated hardware timers and SPI peripherals:

| Pin | Function / Role | Peripheral | Notes |
| :--- | :--- | :--- | :--- |
| **PA1** | Motor Phase B (M1) | TIM2_CH2 | Center-aligned 25 kHz PWM |
| **PA2** | Motor Phase C (M1) | TIM2_CH3 | Center-aligned 25 kHz PWM |
| **PA3** | Motor 1 Driver Enable | GPIO Out | Active-High driver enable |
| **PA4** | Motor Phase B (M2) | TIM3_CH2 | Center-aligned 25 kHz PWM |
| **PA5** | Motor Phase A (M1) | TIM2_CH1 | Center-aligned 25 kHz PWM |
| **PA6** | Motor Phase A (M2) | TIM3_CH1 | Center-aligned 25 kHz PWM |
| **PA7** | Motor 2 Driver Enable | GPIO Out | Active-High driver enable |
| **PA9** | Telemetry TX | USART1_TX | Communication link |
| **PA10** | Telemetry RX | USART1_RX | Communication link |
| **PA11** | USB D- | Native USB | DFU Bootloader / CDC Serial |
| **PA12** | USB D+ | Native USB | DFU Bootloader / CDC Serial |
| **PA13** | SWDIO | SWD SYS | Hardware debug data |
| **PA14** | SWCLK | SWD SYS | Hardware debug clock |
| **PA15** | Motor 1 Encoder CS | GPIO Out | SPI1 Chip Select for MT6816 #1 |
| **PB0** | Motor Phase C (M2) | TIM3_CH3 | Center-aligned 25 kHz PWM |
| **PB3** | Encoder 1 SCK | SPI1_SCK | MT6816 #1 SPI Clock |
| **PB4** | Encoder 1 MISO | SPI1_MISO | MT6816 #1 SPI Data In |
| **PB5** | Encoder 1 MOSI | SPI1_MOSI | MT6816 #1 SPI Data Out |
| **PB11** | Motor 2 Encoder CS | GPIO Out | SPI2 Chip Select for MT6816 #2 |
| **PB13** | Encoder 2 SCK | SPI2_SCK | MT6816 #2 SPI Clock (AF5) |
| **PB14** | Encoder 2 MISO | SPI2_MISO | MT6816 #2 SPI Data In (AF5) |
| **PB15** | Encoder 2 MOSI | SPI2_MOSI | MT6816 #2 SPI Data Out (AF5) |
| **PC13** | Onboard Status LED | GPIO Out | Active-Low user status LED |

---

## 5. Firmware Quick Start (PlatformIO)

The root [`platformio.ini`](file:///c:/Users/nurmi/Documents/Folkrace/platformio.ini) manages all microcontroller firmware targets:

```bash
# Build & flash STM32 Dual-FOC controller over USB DFU
pio run -e stm32_dual_foc -t upload

# Build & flash ESP32 Main (Chassis & Dynamics Master)
pio run -e esp32_main -t upload

# Build & flash ESP32 ToF Hub (Quad VL53L8CX Driver)
pio run -e esp32_tof -t upload
```

---

## 6. Verification & Test Launchers

All diagnostic GUIs and simulation suites are centralized in [`1_launchers/`](file:///c:/Users/nurmi/Documents/Folkrace/1_launchers/README.md):

```bash
# Launch physical quad VL53L8CX ToF multi-target visualizer
python 1_launchers/6_esp32_tof_gui.py

# Launch physical high-rate steering servo calibration bench
python 1_launchers/8_esp32_servo_gui.py

# Launch physical STM32 motor closed-loop FOC & encoder GUI
python 1_launchers/9_stm32_encoder_motor_gui.py

# Launch Webots 3D simulation with Curvilinear SL-MPC & dynamic obstacles
python 1_launchers/1_champion_preview.py
```


=== FILE: platformio.ini ===
[platformio]
default_envs = stm32_dual_foc, esp32_main, esp32_tof
src_dir = .
boards_dir = 4_stm32/boards

; ==============================================================================
; ESP32 MAIN: Real-Time Motion, IMU (BNO08x), Servos & STM32 FOC Master Link
; ==============================================================================
[env:esp32_main]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
upload_port = COM5
upload_speed = 921600
monitor_port = COM5
monitor_speed = 921600
extra_scripts = pre:tools/sync_params.py
build_src_filter =
    -<*>
    +<3_esp32/ESP32_Main/src/*.cpp>
build_flags =
    -I 3_esp32/ESP32_Main/include
    -DARDUINO_USB_MODE=0
    -DARDUINO_USB_CDC_ON_BOOT=0
    -DCORE_DEBUG_LEVEL=0
lib_deps =
    Wire
    SPI
    sparkfun/SparkFun BNO080 Cortex Based IMU @ ^1.1.12

; ==============================================================================
; ESP32 TOF: Dedicated Dual-SPI Perception Hub (4x VL53L8CX Multizone ToFs)
; ==============================================================================
[env:esp32_tof]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
upload_port = COM7
monitor_port = COM7
monitor_speed = 921600
build_src_filter =
    -<*>
    +<3_esp32/ESP32_ToF/src/*.cpp>
build_flags =
    -I 3_esp32/ESP32_ToF/include
    -I 3_esp32/ESP32_ToF/VL53L8CX/src
    -I 3_esp32/ESP32_ToF/src
    -D VL53L8CX_NB_TARGET_PER_ZONE=2U
    -D VL53L8CX_DISABLE_SIGNAL_PER_SPAD
    -D VL53L8CX_DISABLE_RANGE_SIGMA_MM
    -D VL53L8CX_DISABLE_AMBIENT_PER_SPAD
    -D VL53L8CX_DISABLE_REFLECTANCE_PERCENT
    -D VL53L8CX_DISABLE_MOTION_INDICATOR
    -D ARDUINO_USB_MODE=0
    -D ARDUINO_USB_CDC_ON_BOOT=0
lib_deps =
    Wire
    SPI
    3_esp32/ESP32_ToF/VL53L8CX

; Legacy Alias for backwards compatibility
[env:esp32_tof_diag]
extends = env:esp32_tof

[env:esp32s3_suite]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 921600
build_src_filter =
    -<*>
    +<3_esp32/ESP32_Main/simplefoc_mt6816_suite/src/*.cpp>
build_flags =
    -DARDUINO_USB_MODE=0
    -DARDUINO_USB_CDC_ON_BOOT=0
    -DCORE_DEBUG_LEVEL=0
lib_deps =
    askuric/Simple FOC @ 2.3.3
    Wire
    SPI

[env:esp32s3_servo_test]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
upload_port = COM5
upload_speed = 921600
monitor_port = COM5
monitor_speed = 921600
extra_scripts = pre:tools/sync_params.py
build_src_filter =
    -<*>
    +<3_esp32/ESP32_Main/servo_test/*.cpp>
build_flags =
    -I 3_esp32/ESP32_Main/include
    -DARDUINO_USB_MODE=0
    -DARDUINO_USB_CDC_ON_BOOT=0

; ==============================================================================
; STM32G474: Dual-Motor FOC Controller & MT6816 Shared SPI Suite
; ==============================================================================
[stm32_base]
platform = ststm32
board = genericSTM32G474CB
framework = arduino
lib_archive = false
lib_deps =
    askuric/Simple FOC @ ^2.3.2
monitor_speed = 921600
monitor_eol = LF
build_flags =
    -DHAL_OPAMP_MODULE_ENABLED
    -DHAL_CORDIC_MODULE_ENABLED
    -DFLASH_BANK_NUMBER=FLASH_BANK_1
    -DSIMPLEFOC_DISABLE_DEBUG
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard

[env:stm32_dual_foc]
extends = stm32_base
build_src_filter =
    -<*>
    +<4_stm32/src/*.cpp>
    +<4_stm32/src/*.c>
build_flags =
    ${stm32_base.build_flags}
    -DUSBCON
    -DUSBD_USE_CDC
    -DSERIAL_RX_BUFFER_SIZE=512
    -DSERIAL_TX_BUFFER_SIZE=512
    -Os
    -I 4_stm32/src
    -Wl,--defsym=_Min_Stack_Size=0x2000
upload_protocol = dfu

[env:stm32_pwm_test]
extends = stm32_base
build_src_filter =
    -<*>
    +<4_stm32/test_pwm/src/*.cpp>
build_flags =
    ${stm32_base.build_flags}
    -DUSBCON
    -DUSBD_USE_CDC
    -I 4_stm32/src
upload_protocol = dfu


=== FILE: .agents/rules/file_hygiene.md ===
---
trigger: always_on
description: Workspace file hygiene and directory boundaries.
---

# File Hygiene

- **Edit in place:** Never create `*_v2`, `*_old`, `*_test2`, `*.bak`, `*.dll.old`, or extra root folders/docs (`docs/`). Put agent guides in `.agents/skills/`.
- **Temp & data:** Put one-off scripts in `scratch/` (delete after use) and datasets/CSVs/`.npz`/`.db` in `data/` or `scratch/`, never source folders.
- **Directory map:**
  - `1_launchers/`: Human-facing GUIs & simulation launchers only.
  - `2_pi4/`, `3_esp32/`, `4_stm32/`: Target runtime/firmware and hardware benches.
  - `5_webots/`: Simulation (`controllers/` 1:1 with `worlds/`, no helper clutter in `controllers/`), GA, assets.
  - `brain/`: Shared planner, `vehicle_config.json`, and synced `vehicle_params.py` (sync via `python tools/sync_params.py`).
  - `tools/`: CLI/AI utilities and unit tests.


=== FILE: .agents/rules/formatting.md ===
---
trigger: always_on
description: Chat formatting rules: strictly ban LaTeX and dollar sign math delimiters in chat text.
---

No LaTeX

=== FILE: .agents/rules/pi_connectivity.md ===
# Raspberry Pi Connectivity Rule

- Test Pi connectivity with a single quick check (e.g. `ssh -o ConnectTimeout=2 robot ...` or `Test-Connection -Count 1`).
- If the Pi is unreachable or the connection times out:
  - DO NOT retry in a loop.
  - DO NOT run network/subnet sweeps or repeated ping loops.
  - Report immediately that the Pi is unreachable (noting common hardware causes: brownout/undervoltage from weak USB power, powered off, or disconnected Wi-Fi) and ask the user how they would like to proceed.



=== FILE: .agents/skills/ai-flight-debug/SKILL.md ===
---
name: ai-flight-debug
description: Analyze Folkrace Pi 4 SQLite flight recordings (.db) for crashes, anomalies, planner/ToF/MCU diagnostics, A/B run comparisons, and ML/AI data export.
---

# AI Flight Debugging Suite

Use `tools/ai_debug_suite.py` instead of writing ad-hoc SQLite/zlib scripts (supports both v1 and v2 `.db` files in `Downloads/` and `data/recordings/`).

## Workflow
1. Run `digest` first as a triage index (diagnostics, KPIs, top episodes, deduplicated MCU logs).
2. Drill into incidents with `episodes`, then `window` starting **1–2s before** the episode to see upstream cause, then `frame` for full sensors + 2D ASCII ToF/MPC radar.
3. Use `compare` for A/B run regressions, `query` for custom filters, `tag` to label frame ranges in the `.db`, and `export` for `report.md`/`digest.json`/`frames.csv`/`arrays.npz`.

## Commands
```powershell
python tools/ai_debug_suite.py list
python tools/ai_debug_suite.py digest [db]                 # omit [db] for latest
python tools/ai_debug_suite.py episodes [db] [--category SHIELD|STALL|LINK_DROP]
python tools/ai_debug_suite.py window [db] <start> <end> [--by-time] [--stride N]
python tools/ai_debug_suite.py frame [db] <frame_id>       # full state + ASCII radar
python tools/ai_debug_suite.py query [db] --shield --max-tof 0.25 --limit 20
python tools/ai_debug_suite.py compare <db_a> <db_b>
python tools/ai_debug_suite.py export [db] [--formats md,json,csv,npz]
python tools/ai_debug_suite.py tag [db] <start> <end> <tag> --note "..."
python tools/ai_debug_suite.py pull --digest               # download from Pi 4 + digest
```

## Folkrace Telemetry & Verification Guardrails
- **Cross-eyed ToF pods**: `tof_front_left_*` (ch 0..7) points right; `tof_front_right_*` (ch 8..15) points left (`0.0` = invalid/offline, `>=1.79` = open/saturated). ASCII radar is ~`5x9.5 cm` quantized and omits `candidate_lines`/costmap inflation; use `frame --json` for exact coordinates.
- **Do not trust `digest` blindly**:
  - Check **wheel tracking MAE (`cmd - act`)**, not just raw RPM lag (track turn direction creates Ackermann speed differences).
  - Check **`override_active`** and **`mcu_logs`** before calling `LINK_DROP`/`EMERGENCY` a crash—manual GUI/bench stops also trigger them.
  - Check `pitch_deg` directly if `is_on_ramp` stays high (an IMU pitch offset `>=5 deg` bypasses the flat-floor warning).
  - Silent tuning issues (slow lap, mild oscillation, poor line choice) may trigger zero anomalies—verify with `window` and `compare`.


=== FILE: .agents/skills/esp32-pi-flash/SKILL.md ===
---
name: esp32-pi-flash
description: >-
  Build ESP32-S3 firmware locally and flash ESP32_Main or ESP32_ToF over USB via Raspberry Pi 4.
---

# ESP32 Flashing via Pi 4

## Device Mapping
- `esp32_main`: `/dev/ttyACM0` (USB serial `5B90128539`, MAC `ac:a7:04:1d:a4:fc`)
- `esp32_tof`: `/dev/ttyACM1` (USB serial `5B8E139240`)
- Verify if ports swap: `ssh robot 'cd /home/admin/folkrace && python3 -c "import mcu_uart_bridge as m; print(m.probe_port_role(\"/dev/ttyACM0\"), m.probe_port_role(\"/dev/ttyACM1\"))"'`

## Build & Flash (`<env>` = `esp32_main` | `esp32_tof`, `<port>` = `/dev/ttyACM0` | `/dev/ttyACM1`)
```powershell
pio run -e <env>
scp .pio/build/<env>/bootloader.bin .pio/build/<env>/partitions.bin .pio/build/<env>/firmware.bin admin@robot:/home/admin/folkrace/esp_flash/
ssh robot 'echo l | sudo -S systemctl stop folkrace && cd /home/admin/folkrace/esp_flash && python3 -m esptool --chip esp32s3 --port <port> --baud 921600 write-flash 0x0 bootloader.bin 0x8000 partitions.bin 0x10000 firmware.bin && echo l | sudo -S systemctl start folkrace'
```


=== FILE: .agents/skills/pi-ssh/SKILL.md ===
---
name: pi-ssh
description: SSH connection, command execution, and code deployment for Raspberry Pi 4 (pi4).
---

# Raspberry Pi 4 SSH

- **SSH Target (Old Wi-Fi)**: `robot` (`admin@robot`, `~/.ssh/config` hardcodes `192.168.20.120`, uses `~/.ssh/id_ed25519`, sudo password: `l`)
- **SSH Target (Phone Hotspot)**: Dynamic IP (e.g., `10.88.202.59`). Because `~/.ssh/config` hardcodes `192.168.20.120` for `robot` and `robot.local` mDNS may not resolve on hotspots, connect with `ssh admin@<ip>` or `ssh -o HostName=<ip> robot`.
- **Finding Hotspot IP (No Subnet Scan)**: Check the local ARP table (`arp -a`) for the Pi's MAC OUI (`dc-a6-32-...`, e.g. `dc-a6-32-f0-ce-c9`).
- **Local Source**: `2_pi4/`
- **Remote Target**: `/home/admin/folkrace/`

## Connection Guidelines
- Use a short timeout (`-o ConnectTimeout=2`) when checking Pi availability.
- If the Pi does not respond or times out, **stop immediately**: do NOT retry in a loop and do NOT scan the subnet. Inform the user right away that the Pi is unreachable (e.g. insufficient USB power/brownout, power off, or network loss).

## Quick Commands
- **Run command**: `ssh robot "<command>"`
- **Sync pi4 files**: `scp -r 2_pi4/* admin@robot:/home/admin/folkrace/`
- **Sync planner stack**: `scp 5_webots/simulation/my_project/controllers/tof_path_planner/planner_config.py 5_webots/simulation/my_project/controllers/tof_path_planner/planner_sl_mpc.py 5_webots/simulation/my_project/controllers/tof_path_planner/cpp_smart_hybrid_costmap.py 5_webots/simulation/my_project/controllers/tof_path_planner/tof_gap_scout.py 5_webots/simulation/my_project/controllers/tof_path_planner/ramp_detector.py admin@robot:/home/admin/folkrace/`
- **Check USB/Serial**: `ssh robot "ls -l /dev/ttyUSB* /dev/ttyACM*"`


=== FILE: .agents/skills/stm32-dfu/SKILL.md ===
---
name: stm32-dfu
description: >-
  Put STM32 into System ROM DFU bootloader and flash firmware over USB hands-free.
---

# STM32 DFU Flashing

Firmware jumps to ST System ROM DFU when receiving `DFU\n` over USB CDC (VID `0x0483`, PID `0x5740`, 921600 baud).

## Commands

- **One-step DFU reboot & flash:**
  ```powershell
  python .agents/skills/stm32-dfu/scripts/enter_dfu.py --upload
  ```

- **DFU reboot only:**
  ```powershell
  python .agents/skills/stm32-dfu/scripts/enter_dfu.py
  ```

- **Inline CLI:**
  ```powershell
  python -c "import serial, time; s = serial.Serial('COM11', 921600); s.write(b'DFU\n'); s.flush(); time.sleep(0.15); s.close()"; Start-Sleep -Milliseconds 1500; pio run -e stm32_dual_foc -t upload
  ```

## Notes
- If device is already in DFU mode (`0483:df11`), run `pio run -e stm32_dual_foc -t upload` directly.
- Ignore Windows Bluetooth ports (`COM9`, `COM10`).


=== FILE: .agents/skills/stm32-dfu/scripts/enter_dfu.py ===
#!/usr/bin/env python3
"""
Automatically puts STM32 into System ROM DFU mode without physical buttons.
Identifies STMicroelectronics USB CDC port (VID 0x0483, PID 0x5740),
sends 'DFU\\n' to trigger firmware software jump to bootloader, and optionally
executes PlatformIO upload.
"""

import os
import sys
import time
import argparse
import subprocess
import serial
import serial.tools.list_ports

def find_stm32_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x0483 and p.pid == 0x5740:
            return p.device
        if "STM" in (p.description or "") or "STMicroelectronics" in (p.manufacturer or ""):
            return p.device
    return None

def trigger_dfu(port=None):
    if not port:
        port = find_stm32_port()
    if not port:
        print("[ERR] No active STM32 USB CDC port found (VID: 0x0483, PID: 0x5740).")
        print("      Device may already be in DFU mode or disconnected.")
        return False

    print(f"[INFO] Found STM32 on {port}. Sending DFU command...")
    try:
        ser = serial.Serial(port, baudrate=921600, timeout=1.0)
        ser.dtr = True
        ser.rts = True
        time.sleep(0.05)
        ser.write(b"\n\nDFU\n")
        ser.flush()
        time.sleep(0.2)
        ser.close()
        print(f"[OK] DFU trigger sent. Device on {port} is rebooting to ST System DFU Bootloader...")
        time.sleep(3.0)
        return True
    except Exception as e:
        print(f"[WARN] Error sending DFU command to {port}: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Reboot STM32 into DFU mode and optionally upload firmware.")
    parser.add_argument("--port", "-p", help="Explicit COM port (e.g. COM11). Auto-detects if omitted.")
    parser.add_argument("--upload", "-u", action="store_true", help="Run 'pio run -e stm32_dual_foc -t upload' after rebooting to DFU.")
    parser.add_argument("--env", "-e", default="stm32_dual_foc", help="PlatformIO environment name (default: stm32_dual_foc)")
    args = parser.parse_args()

    success = trigger_dfu(args.port)

    if args.upload:
        if os.path.exists("platformio.ini"):
            cmd = ["pio", "run", "-e", args.env, "-t", "upload"]
        else:
            cmd = ["pio", "run", "-d", "4_stm32", "-e", args.env, "-t", "upload"]
        print(f"\n[RUN] Running: {' '.join(cmd)}")
        ret = subprocess.run(cmd)
        sys.exit(ret.returncode)

if __name__ == "__main__":
    main()


=== FILE: .agents/skills/webots-run/SKILL.md ===
---
name: webots-run
description: Run and automate Webots simulations on Windows without OpenGL headless crashes, plus vehicle coordinate frame and steering sign conventions.
---

# Webots Execution & Conventions

## 1. Launching Webots on Windows (Avoid `Exit Code 3221225477`)
Webots on Windows requires an active desktop OpenGL context (`0xC0000005` `STATUS_ACCESS_VIOLATION` occurs if invoked in a pure headless/background session without DWM).

- **Always** launch in a desktop-attached session with `--mode=fast --batch`:
  ```powershell
  "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" --mode=fast --batch "5_webots/simulation/my_project/worlds/track_racing.wbt"
  ```
- **Standard launchers**:
  - `python 1_launchers/1_champion_preview.py` (3D Webots + live 2D telemetry GUI)
  - `python 1_launchers/2_fast_champion_drive.py` (ultra-lean 3D drive without GUI/UDP overhead)

## 2. Automated Benchmarks & Troubleshooting
- **Clean exit**: Use the Webots `Supervisor` API (`robot.simulationQuit(0)`) when a target duration or lap count completes.
- **Controller hangs on start**: Verify the controller name in `.wbt` matches the directory name under `5_webots/simulation/my_project/controllers/`.
- **Empty logs on exit**: Use `flush=True` in `print()` and line-buffered file opens (`buffering=1`).
- **Moving obstacles not moving**: Ensure `.wbt` defines `DEF MOVING_OBSTACLE Solid { ... }`.

## 3. Coordinate Frame & Steering Conventions
- **Robot Body Frame**: `+Y` Forward, `-Y` Backward, `+X` Right, `-X` Left, `+Z` Up.
- **Steering (`cmd_steer_rad`, `steer_deg`) & Yaw (`yaw`, `gyro_z`)**:
  - Positive (`+`) = **Steer Left / CCW**
  - Negative (`-`) = **Steer Right / CW**
  - Steering limits: ~±32.22° (±0.562 rad) virtual center (35.0° inner / 30.0° outer at 45% Ackermann; 1294 us left, 1565 us center, 1836 us right).
- **Recovery Steering Rules**:
  - Opening on **Left (`X < 0`)**: forward (`v > 0.1 m/s`) steer `+37.2°` (Left); reverse (`v <= 0.1 m/s`) steer `-37.2°` (Right).
  - Opening on **Right (`X > 0`)**: forward (`v > 0.1 m/s`) steer `-37.2°` (Right); reverse (`v <= 0.1 m/s`) steer `+37.2°` (Left).


=== FILE: brain/__init__.py ===
"""
================================================================================
  FOLKRACE AUTONOMOUS BRAIN PACKAGE
================================================================================
  Shared single-source-of-truth vehicle config, perception, planning, and safety.
  Imported identically by Webots simulation and physical Raspberry Pi 4 runner.
================================================================================
"""

from importlib import import_module

_EXPORTS = {
    "AutonomousBrain": (".autonomous_brain", "AutonomousBrain"),
    "PoseTracker": (".pose_tracker", "PoseTracker"),
    "CppSmartHybridCostmap": (".cpp_smart_hybrid_costmap", "CppSmartHybridCostmap"),
    "SENSOR_NAMES": (".cpp_smart_hybrid_costmap", "SENSOR_NAMES"),
    "ToFGapScout": (".tof_gap_scout", "ToFGapScout"),
    "CurvilinearSLMPC": (".planner_sl_mpc", "CurvilinearSLMPC"),
    "StuckRecoveryManager": (".stuck_recovery_manager", "StuckRecoveryManager"),
    "StuckRecoveryState": (".stuck_recovery_manager", "StuckRecoveryState"),
    "AntiWrongWayWatchdog": (".anti_wrong_way_watchdog", "AntiWrongWayWatchdog"),
    "RampDetector": (".ramp_detector", "RampDetector"),
}

__all__ = list(_EXPORTS.keys())


def __getattr__(name: str):
    if name in _EXPORTS:
        mod_name, attr_name = _EXPORTS[name]
        mod = import_module(mod_name, __name__)
        val = getattr(mod, attr_name)
        globals()[name] = val
        return val
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


=== FILE: brain/anti_wrong_way_watchdog.py ===
"""
================================================================================
  ANTI-WRONG-WAY & U-TURN DISCRIMINATION WATCHDOG (BREADCRUMB & EARLY GAP EXIT)
================================================================================
  Distinguishes legitimate track hairpins (U-turns into adjacent corridors separated
  by walls) from in-lane wrong-way reversals (spinning 180 within the same corridor).

  Utilizes:
    - OPTION_1_BREADCRUMB: Line-of-Sight (LOS) 2D Bresenham raycasting against
      the inflated costmap (hazard_mask) to confirm in-corridor wrong-way reversals.
    - Narrow-Cone Early Gap Exit Sentry: Evaluates candidate scouted gaps for
      safe forward paths, preventing false positives and enabling swift exit from
      corrective maneuvers.
================================================================================
"""

import math
import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    from .planner_config import (
        ANTI_WRONG_WAY_MODE,
        AWW_BREADCRUMB_LOOKBACK_M,
        AWW_BREADCRUMB_DURATION_SCALE,
        AWW_BREADCRUMB_SPACING_M,
        AWW_BREADCRUMB_MAX_TIMEOUT_S,
        AWW_CONFIRM_TIME_S,
        AWW_POST_RECOVERY_COOLDOWN_S,
        AWW_MIN_CONFIRMED_BREADCRUMBS,
        AWW_MAX_DETECT_DIST_M,
        AWW_MAX_LAT_XV_M,
        AWW_EARLY_EXIT_MIN_GAP_M,
        AWW_EARLY_EXIT_CONE_DEG,
        AWW_EARLY_EXIT_OPPOSING_DEG,
        AWW_EARLY_EXIT_MIN_BC_DIST_M,
        AWW_EARLY_EXIT_CORRIDOR_HALF_WIDTH_M,
        AWW_EARLY_EXIT_MIN_HALF_WIDTH_M,
        GRID_X_MIN,
        GRID_Y_MIN,
        GRID_RES,
        V_MIN_CRUISE,
        V_MAX_CRUISE,
    )
except ImportError:
    try:
        from planner_config import (
            ANTI_WRONG_WAY_MODE,
            AWW_BREADCRUMB_LOOKBACK_M,
            AWW_BREADCRUMB_DURATION_SCALE,
            AWW_BREADCRUMB_SPACING_M,
            AWW_BREADCRUMB_MAX_TIMEOUT_S,
            AWW_CONFIRM_TIME_S,
            AWW_POST_RECOVERY_COOLDOWN_S,
            AWW_MIN_CONFIRMED_BREADCRUMBS,
            AWW_MAX_DETECT_DIST_M,
            AWW_MAX_LAT_XV_M,
            AWW_EARLY_EXIT_MIN_GAP_M,
            AWW_EARLY_EXIT_CONE_DEG,
            AWW_EARLY_EXIT_OPPOSING_DEG,
            AWW_EARLY_EXIT_MIN_BC_DIST_M,
            AWW_EARLY_EXIT_CORRIDOR_HALF_WIDTH_M,
            AWW_EARLY_EXIT_MIN_HALF_WIDTH_M,
            GRID_X_MIN,
            GRID_Y_MIN,
            GRID_RES,
            V_MIN_CRUISE,
            V_MAX_CRUISE,
        )
    except ImportError:
        ANTI_WRONG_WAY_MODE = "OPTION_1_BREADCRUMB"
        AWW_BREADCRUMB_LOOKBACK_M = 8.0
        AWW_BREADCRUMB_DURATION_SCALE = 0.35
        AWW_BREADCRUMB_SPACING_M = 0.05
        AWW_BREADCRUMB_MAX_TIMEOUT_S = 6.0
        AWW_CONFIRM_TIME_S = 0.080
        AWW_POST_RECOVERY_COOLDOWN_S = 0.35
        AWW_MIN_CONFIRMED_BREADCRUMBS = 3
        AWW_MAX_DETECT_DIST_M = 2.20
        AWW_MAX_LAT_XV_M = 0.55
        AWW_EARLY_EXIT_MIN_GAP_M = 1.4
        AWW_EARLY_EXIT_CONE_DEG = 30.0
        AWW_EARLY_EXIT_OPPOSING_DEG = 110.0
        AWW_EARLY_EXIT_MIN_BC_DIST_M = 0.15
        AWW_EARLY_EXIT_CORRIDOR_HALF_WIDTH_M = 0.35
        AWW_EARLY_EXIT_MIN_HALF_WIDTH_M = 0.08
        GRID_X_MIN = -1.25
        GRID_Y_MIN = 0.00
        GRID_RES = 0.015
        V_MIN_CRUISE = 1.284
        V_MAX_CRUISE = 3.84


class WrongWayOptionMode(str, Enum):
    DISABLED = "DISABLED"
    OPTION_1_BREADCRUMB = "OPTION_1_BREADCRUMB"


def wrap_to_pi(angle_rad: float) -> float:
    """Wraps an angle into [-pi, pi]."""
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


@dataclass
class Breadcrumb:
    x: float
    y: float
    yaw: float
    v: float
    timestamp: float
    s_dist: float = 0.0


@dataclass
class WrongWayStatus:
    is_wrong_way: bool = False
    confidence: float = 0.0
    detected_by_option: str = "NONE"
    corrective_action: str = "NONE"
    target_recovery_yaw: Optional[float] = None
    turning_radius_m: float = 999.0
    lateral_separation_m: float = 0.0
    overlap_score: float = 0.0
    sightline_rays: List[Dict[str, Any]] = field(default_factory=list)
    early_exit_gap: Optional[Tuple[float, float]] = None
    early_exit_found: bool = False
    pending_wrong_way: bool = False


# ==============================================================================
# BREADCRUMB REVERSE-PATH OVERLAP & NARROW-CONE EARLY GAP EXIT SENTRY
# ==============================================================================
class BreadcrumbWatchdog:
    """
    Maintains a dead-reckoned trail of recent vehicle positions.
    Evaluates Line-of-Sight (LOS) 2D Bresenham raycasting against the inflated costmap
    (hazard_mask) to confirm in-corridor wrong-way reversals while rejecting adjacent-lane
    track hairpins separated by walls.
    """

    # Speed limits anchored to vehicle operational envelope from planner_config:
    _V_MIN_MPS: float = float(V_MIN_CRUISE)   # m/s dynamic cornering floor (~1.17 m/s)
    _V_MAX_MPS: float = float(V_MAX_CRUISE)   # m/s maximum straight-line cruising speed (5.00 m/s)

    def __init__(
        self,
        lookback_m: float = AWW_BREADCRUMB_LOOKBACK_M,
        spacing_m: float = AWW_BREADCRUMB_SPACING_M,
        max_history_s: float = AWW_BREADCRUMB_MAX_TIMEOUT_S,
        cone_angle_deg: float = 65.0,
        duration_scale: float = AWW_BREADCRUMB_DURATION_SCALE,
        max_timeout_s: float = AWW_BREADCRUMB_MAX_TIMEOUT_S,
        min_confirmed: int = AWW_MIN_CONFIRMED_BREADCRUMBS,
        max_detect_dist_m: float = AWW_MAX_DETECT_DIST_M,
        max_lat_xv_m: float = AWW_MAX_LAT_XV_M,
        **kwargs,
    ):
        self.lookback_m = float(lookback_m)
        self.duration_scale = float(kwargs.get("duration_scale", kwargs.get("lifetime_scale", duration_scale)))
        self.spacing_m = spacing_m
        self.max_timeout_s = float(kwargs.get("max_timeout_s", kwargs.get("max_history_s", max_timeout_s)))
        self.max_history_s = self.max_timeout_s
        self.cone_angle_rad = math.radians(cone_angle_deg)
        self.min_confirmed = max(1, int(min_confirmed))
        self.max_detect_dist_m = float(max_detect_dist_m)
        self.max_lat_xv_m = float(max_lat_xv_m)

        self.breadcrumbs: deque[Breadcrumb] = deque()
        self.cumulative_dist_m = 0.0
        self.last_x: Optional[float] = None
        self.last_y: Optional[float] = None
        self.last_yaw: Optional[float] = None
        self.last_in_lane_opposing_count: int = 0
        self._last_add_time_s: Optional[float] = None
        self._was_frozen: bool = False

    @property
    def effective_lookback_m(self) -> float:
        """
        Effective spatial lookback distance accounting for duration_scale (how fast breadcrumbs disappear).
        Enforces a sensible minimum of 1.5m to ensure sufficient points for detection.
        """
        return max(1.5, self.lookback_m * self.duration_scale)

    def set_duration_scale(self, scale: float):
        """Dynamically updates how fast breadcrumbs disappear."""
        self.duration_scale = max(0.05, float(scale))

    def set_max_timeout(self, timeout_s: float):
        """Dynamically updates maximum breadcrumb history timeout."""
        self.max_timeout_s = max(0.5, float(timeout_s))
        self.max_history_s = self.max_timeout_s

    def reset(self):
        self.breadcrumbs.clear()
        self.cumulative_dist_m = 0.0
        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.last_in_lane_opposing_count = 0
        self._last_add_time_s = None
        self._was_frozen = False

    def add_point(
        self,
        x: float,
        y: float,
        yaw: float,
        v: float,
        now_s: float,
        state: str = "DRIVE",
        yaw_rate: float = 0.0,
        freeze_recording: bool = False,
        freeze_old_state: Optional[bool] = None,
        dt: Optional[float] = None,
    ):
        state_str = getattr(state, "value", getattr(state, "name", str(state)))
        in_recovery = state_str not in ("DRIVE", "NORMAL_DRIVE")
        is_reversing = (v < 0.0)
        is_spinning = (abs(yaw_rate) > 3.5)

        if dt is not None and dt >= 0.0:
            dt_step = float(dt)
        elif self._last_add_time_s is not None:
            dt_step = max(0.0, min(0.25, float(now_s) - self._last_add_time_s))
        else:
            dt_step = 0.0
        self._last_add_time_s = float(now_s)

        if freeze_old_state is None:
            eff_freeze_old = in_recovery or is_reversing or is_spinning or bool(freeze_recording)
        else:
            eff_freeze_old = bool(freeze_old_state) or in_recovery or is_reversing or is_spinning

        # Freeze both pruning and recording during recovery, reversing, spinning, or occlusion
        if eff_freeze_old:
            if dt_step > 0.0 and self.breadcrumbs:
                for bc in self.breadcrumbs:
                    bc.timestamp += dt_step
            self._was_frozen = True
            return

        if self._was_frozen:
            self._was_frozen = False
            if self.last_x is not None and self.last_y is not None:
                dx_u = x - self.last_x
                dy_u = y - self.last_y
                fwd_u = -math.sin(yaw) * dx_u + math.cos(yaw) * dy_u
                lat_u = abs(math.cos(yaw) * dx_u + math.sin(yaw) * dy_u)
                yaw_chg = abs(wrap_to_pi(yaw - self.last_yaw)) if self.last_yaw is not None else math.pi
                # Re-anchor without distance jump if heading changed (U-turn/bypass), laterally shifted,
                # or moved ahead of the last recorded breadcrumb. If simply backed straight up behind it,
                # keep last_x/last_y so forward motion re-engages smoothly once past the old tip.
                if yaw_chg > math.radians(45.0) or lat_u > 0.30 or fwd_u > 0.0:
                    self.last_x = x
                    self.last_y = y
                    self.last_yaw = yaw

        # Effective spatial lookback distance accounting for duration scale
        effective_lookback = self.effective_lookback_m
        # Max history timeout (default 6.0s from planner_config prevents premature queue wipe during maneuvers)
        self.max_history_s = self.max_timeout_s

        # Always prune expired points during DRIVE even when stationary (v < 0.25)
        while self.breadcrumbs and (
            (now_s - self.breadcrumbs[0].timestamp) > self.max_history_s
            or (self.cumulative_dist_m - self.breadcrumbs[0].s_dist) > effective_lookback
        ):
            self.breadcrumbs.popleft()

        # Anti-Contamination Gating:
        # Breadcrumbs are only recorded during valid forward racing (v >= 0.25 m/s,
        # |yaw_rate| <= 3.5 rad/s, and not frozen by opposing in-lane occlusion).
        if freeze_recording or v < 0.25 or abs(yaw_rate) > 3.5:
            return

        if self.last_x is None:
            self.breadcrumbs.append(Breadcrumb(x, y, yaw, v, now_s, s_dist=self.cumulative_dist_m))
            self.last_x = x
            self.last_y = y
            self.last_yaw = yaw
            return

        dx = x - self.last_x
        dy = y - self.last_y
        # Require forward progress along current heading (+Y forward = (-sin(yaw), cos(yaw)))
        if (-math.sin(yaw) * dx + math.cos(yaw) * dy) <= 0.0:
            return

        dist = math.hypot(dx, dy)

        if dist >= self.spacing_m:
            self.cumulative_dist_m += dist
            self.breadcrumbs.append(Breadcrumb(x, y, yaw, v, now_s, s_dist=self.cumulative_dist_m))
            self.last_x = x
            self.last_y = y
            self.last_yaw = yaw

            while self.breadcrumbs and (
                (now_s - self.breadcrumbs[0].timestamp) > self.max_history_s
                or (self.cumulative_dist_m - self.breadcrumbs[0].s_dist) > effective_lookback
            ):
                self.breadcrumbs.popleft()

    @staticmethod
    def _bresenham_raycast_clear(
        r0: int,
        c0: int,
        r1: int,
        c1: int,
        hazard_mask: np.ndarray,
        unknown_mask: Optional[np.ndarray] = None,
    ) -> bool:
        """
        Discrete 2D Bresenham raycast from (r0, c0) to (r1, c1) across hazard_mask
        (and optional unknown_mask).
        Allows a 2-pixel tolerance at the start to clear any noise on origin cell, and
        tolerates up to 3 consecutive unknown cells along the ray to bridge discrete ToF seams
        while blocking sightlines through unmapped walls or blind zones.
        """
        h, w = hazard_mask.shape
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dc - dr
        r, c = r0, c0
        step = 0
        unobs_streak = 0

        if unknown_mask is not None and (not (0 <= r1 < h and 0 <= c1 < w) or unknown_mask[r1, c1] > 0):
            return False

        while True:
            if 0 <= r < h and 0 <= c < w:
                if step > 2:
                    if hazard_mask[r, c] > 0:
                        return False
                    if unknown_mask is not None:
                        if unknown_mask[r, c] > 0:
                            unobs_streak += 1
                            if unobs_streak > 3:
                                return False
                        else:
                            unobs_streak = 0
            else:
                return False

            if r == r1 and c == c1:
                break

            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                c += sc
            if e2 < dc:
                err += dc
                r += sr
            step += 1

        return True

    def check_wrong_way(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        hazard_mask: Optional[np.ndarray] = None,
        now_s: Optional[float] = None,
        costmap_obj: Optional[Any] = None,
        unknown_mask: Optional[np.ndarray] = None,
        min_confirmed: Optional[int] = None,
        max_detect_dist_m: Optional[float] = None,
        max_lat_xv_m: Optional[float] = None,
        freeze_expiry: bool = False,
        **kwargs,
    ) -> Tuple[bool, float, float, List[Dict[str, Any]]]:
        """
        Line-of-Sight Raycasting on Inflated Costmap (hazard_mask & unknown_mask).
        Evaluates candidate breadcrumbs:
          - Odometry gate: Delta S in [0.80 m, effective_lookback] excludes points under chassis and far laps.
          - Spatial search envelope: Euclidean distance d in [0.40 m, max_detect_dist_m], y_v in [0.40 m, max_detect_dist_m],
            forward cone |angle| <= cone_angle_rad (default 65 deg).
          - Direct 2D Bresenham raycast across hazard_mask (and unknown_mask if provided).
          - Heading opposition check: diff >= 110 deg.
        Returns: (is_wrong_way, confidence, target_recovery_yaw, sightline_rays)
        """
        self.last_in_lane_opposing_count = 0
        if len(self.breadcrumbs) < 5:
            return False, 0.0, 0.0, []

        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)

        x_min = float(getattr(costmap_obj, "x_min", GRID_X_MIN)) if costmap_obj is not None else float(GRID_X_MIN)
        y_min = float(getattr(costmap_obj, "y_min", GRID_Y_MIN)) if costmap_obj is not None else float(GRID_Y_MIN)
        res = float(getattr(costmap_obj, "res", GRID_RES)) if costmap_obj is not None else float(GRID_RES)

        # Origin cell in local costmap grid (cell centers at origin + (idx + 0.5) * res)
        col_0 = int(math.floor((0.0 - x_min) / res))
        row_0 = int(math.floor((0.0 - y_min) / res))

        sightline_rays: List[Dict[str, Any]] = []
        confirmed_candidates: List[Breadcrumb] = []
        in_lane_opp = 0

        eff_min_confirmed = max(1, int(min_confirmed if min_confirmed is not None else self.min_confirmed))
        eff_max_dist = float(max_detect_dist_m if max_detect_dist_m is not None else self.max_detect_dist_m)
        eff_max_lat = float(max_lat_xv_m if max_lat_xv_m is not None else self.max_lat_xv_m)

        effective_lookback = self.effective_lookback_m
        for bc in reversed(self.breadcrumbs):
            if now_s is not None and not freeze_expiry and (now_s - bc.timestamp) > self.max_timeout_s:
                continue

            # 1. Odometry gate: Delta S in [0.80m, effective_lookback]
            delta_s = self.cumulative_dist_m - bc.s_dist
            if not (0.80 <= delta_s <= effective_lookback):
                continue

            # 2. World to vehicle frame: (+Y forward, +X right)
            dx = bc.x - current_x
            dy = bc.y - current_y
            xv = cos_yaw * dx + sin_yaw * dy
            yv = -sin_yaw * dx + cos_yaw * dy

            # Euclidean distance from robot origin
            d = math.hypot(xv, yv)

            # Lateral offset of car relative to the breadcrumb's own travel lane (+X right in bc frame)
            lat_bc = math.cos(bc.yaw) * (-dx) + math.sin(bc.yaw) * (-dy)

            # Heading opposition
            heading_diff = abs(wrap_to_pi(current_yaw - bc.yaw))
            if heading_diff >= math.radians(120.0) and abs(lat_bc) <= 0.40 and 0.40 <= yv <= 2.00:
                in_lane_opp += 1

            # 3. Spatial search envelope (same-corridor lateral bound |lat_bc| <= eff_max_lat prevents hairpin mouth FPs)
            if d < 0.40 or d > eff_max_dist or yv < 0.40 or yv > eff_max_dist or abs(lat_bc) > eff_max_lat:
                continue

            angle_rad = math.atan2(xv, yv)
            if abs(angle_rad) > self.cone_angle_rad:
                continue

            # 4. Heading opposition: diff >= 110 deg
            is_opposing = (heading_diff >= math.radians(110.0))

            # 5. Raycast cell target in costmap
            col_tgt = int(math.floor((xv - x_min) / res))
            row_tgt = int(math.floor((yv - y_min) / res))

            if hazard_mask is not None:
                h_m, w_m = hazard_mask.shape
                if 0 <= row_tgt < h_m and 0 <= col_tgt < w_m:
                    is_clear = self._bresenham_raycast_clear(
                        row_0, col_0, row_tgt, col_tgt, hazard_mask, unknown_mask=unknown_mask
                    )
                else:
                    is_clear = False
            else:
                is_clear = True

            ray_info = {
                "x": float(bc.x),
                "y": float(bc.y),
                "xv": float(xv),
                "yv": float(yv),
                "dist": float(d),
                "delta_s": float(delta_s),
                "clear": bool(is_clear),
                "opposing": bool(is_opposing),
                "heading_diff_deg": float(math.degrees(heading_diff)),
                "target_yaw": float(bc.yaw),
            }
            sightline_rays.append(ray_info)

            if is_clear and is_opposing:
                confirmed_candidates.append(bc)

        self.last_in_lane_opposing_count = in_lane_opp

        if len(confirmed_candidates) >= eff_min_confirmed:
            count_confirmed = len(confirmed_candidates)
            confidence = min(1.0, 0.70 + 0.10 * count_confirmed)
            # Circular mean of confirmed breadcrumb yaws for noise-robust target recovery heading
            sum_sin = sum(math.sin(bc.yaw) for bc in confirmed_candidates)
            sum_cos = sum(math.cos(bc.yaw) for bc in confirmed_candidates)
            target_recovery_yaw = math.atan2(sum_sin, sum_cos)
            return True, confidence, target_recovery_yaw, sightline_rays

        return False, 0.0, 0.0, sightline_rays

    def get_reverse_breadcrumb_yaw(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        target_dist_m: float = 0.45,
        min_dist_m: float = 0.25,
        max_dist_m: float = 0.70,
    ) -> Optional[float]:
        """Returns a target yaw that steers the rear toward the recent breadcrumb closest to ~0.45m."""
        if not self.breadcrumbs:
            return None

        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        best_bc: Optional[Tuple[float, float]] = None
        best_err = float("inf")

        for bc in reversed(self.breadcrumbs):
            delta_s = self.cumulative_dist_m - bc.s_dist
            if delta_s > (max_dist_m + 0.30):
                break

            dx = bc.x - current_x
            dy = bc.y - current_y
            xv = cos_yaw * dx + sin_yaw * dy
            yv = -sin_yaw * dx + cos_yaw * dy
            d = math.hypot(xv, yv)

            if d < min_dist_m or d > max_dist_m or yv > 0.15:
                continue

            dist_err = abs(d - target_dist_m)
            if dist_err < best_err:
                best_err = dist_err
                best_bc = (xv, yv)

        if best_bc is None:
            return None

        xv, yv = best_bc
        # Reverse-pursuit bearing: if bc is behind-left (xv < 0, yv < 0),
        # heading_err < 0 -> compute_reverse_steer steers LEFT (+steer),
        # swinging the rear toward the breadcrumb and the nose away from it.
        return wrap_to_pi(current_yaw + math.atan2(xv, -yv))

    def check_gap_channel(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        gap_xg: float,
        gap_yg: float,
        cone_deg: float = AWW_EARLY_EXIT_CONE_DEG,
        opposing_deg: float = AWW_EARLY_EXIT_OPPOSING_DEG,
        min_gap_dist: float = AWW_EARLY_EXIT_MIN_GAP_M,
        min_bc_dist: float = AWW_EARLY_EXIT_MIN_BC_DIST_M,
        max_reach_margin_m: float = 0.80,
        now_s: Optional[float] = None,
        freeze_expiry: bool = False,
        corridor_half_width_m: float = AWW_EARLY_EXIT_CORRIDOR_HALF_WIDTH_M,
        min_close_half_width_m: float = AWW_EARLY_EXIT_MIN_HALF_WIDTH_M,
    ) -> Tuple[bool, int, List[Dict[str, Any]]]:
        """
        Narrow-Cone Early Gap Exit Sentry (Spatial Channel Filter).
        Evaluates whether a corridor capsule/cone centered along the scouted gap vector (gap_xg, gap_yg)
        contains historical breadcrumbs whose heading opposes the GAP's world heading (not just chassis yaw).
        """
        if len(self.breadcrumbs) < 5:
            return False, 0, []

        gap_dist = math.hypot(gap_xg, gap_yg)
        if gap_dist < 1e-6:
            return False, 0, []
        gap_angle_v = math.atan2(gap_xg, gap_yg)
        gap_world_yaw = wrap_to_pi(current_yaw - gap_angle_v)
        ux = gap_xg / gap_dist
        uy = gap_yg / gap_dist
        half_cone_rad = math.radians(cone_deg / 2.0)
        tan_half_cone = math.tan(half_cone_rad)
        eff_min_close_w = min(float(min_close_half_width_m), 0.15) if cone_deg < 25.0 else float(min_close_half_width_m)
        opposing_rad = math.radians(opposing_deg)

        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)

        # Breadcrumbs are evaluated up to the gap distance plus corridor reach margin
        max_eval_dist = max(gap_dist + max_reach_margin_m, 2.50)

        matching_bcs: List[Dict[str, Any]] = []

        for bc in reversed(self.breadcrumbs):
            if now_s is not None and not freeze_expiry and (now_s - bc.timestamp) > self.max_timeout_s:
                continue

            # 1. Odometry gate: Delta S >= min_bc_dist (0.15m) excludes points under chassis center
            delta_s = self.cumulative_dist_m - bc.s_dist
            if delta_s < min_bc_dist:
                continue

            # 2. Transform breadcrumb to vehicle frame (+Y forward, +X right)
            dx = bc.x - current_x
            dy = bc.y - current_y
            xv = cos_yaw * dx + sin_yaw * dy
            yv = -sin_yaw * dx + cos_yaw * dy

            d_bc = math.hypot(xv, yv)
            if d_bc < min_bc_dist or d_bc > max_eval_dist:
                continue

            # 3. Hybrid capsule-cone lateral check relative to candidate gap vector
            proj = xv * ux + yv * uy
            if proj < min_bc_dist or proj > max_eval_dist:
                continue
            lat = abs(xv * uy - yv * ux)
            eff_half_w = min(float(corridor_half_width_m), max(eff_min_close_w, proj * tan_half_cone))
            if lat > eff_half_w:
                continue

            bc_angle_v = math.atan2(xv, yv)
            angle_diff_from_gap = abs(wrap_to_pi(bc_angle_v - gap_angle_v))

            # 4. Heading opposition check against gap's world heading
            heading_diff = abs(wrap_to_pi(gap_world_yaw - bc.yaw))
            if heading_diff >= opposing_rad:
                matching_bcs.append({
                    "x": float(bc.x),
                    "y": float(bc.y),
                    "xv": float(xv),
                    "yv": float(yv),
                    "dist": float(d_bc),
                    "gap_angle_deg": float(math.degrees(gap_angle_v)),
                    "bc_angle_deg": float(math.degrees(bc_angle_v)),
                    "angle_diff_deg": float(math.degrees(angle_diff_from_gap)),
                    "heading_diff_deg": float(math.degrees(heading_diff)),
                    "delta_s": float(delta_s),
                })

        has_opposing = len(matching_bcs) > 0
        return has_opposing, len(matching_bcs), matching_bcs

    def is_gap_early_exit(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        gap_xg: float,
        gap_yg: float,
        min_gap_dist: float = AWW_EARLY_EXIT_MIN_GAP_M,
        cone_deg: float = AWW_EARLY_EXIT_CONE_DEG,
        opposing_deg: float = AWW_EARLY_EXIT_OPPOSING_DEG,
        min_bc_dist: float = AWW_EARLY_EXIT_MIN_BC_DIST_M,
        now_s: Optional[float] = None,
        freeze_expiry: bool = False,
    ) -> Tuple[bool, Dict[str, Any]]:
        """
        Determines if candidate gap qualifies as a valid early exit.
        Condition: gap distance >= min_gap_dist AND narrow corridor towards gap
        does not contain historical opposing breadcrumbs starting from min_bc_dist.
        """
        gap_dist = math.hypot(gap_xg, gap_yg)
        if gap_dist < min_gap_dist:
            return False, {
                "is_exit": False,
                "reason": "GAP_TOO_SHORT",
                "gap_dist": float(gap_dist),
                "min_gap_dist": float(min_gap_dist),
                "opposing_count": 0,
            }

        has_opposing, count, bcs = self.check_gap_channel(
            current_x=current_x,
            current_y=current_y,
            current_yaw=current_yaw,
            gap_xg=gap_xg,
            gap_yg=gap_yg,
            cone_deg=cone_deg,
            opposing_deg=opposing_deg,
            min_gap_dist=min_gap_dist,
            min_bc_dist=min_bc_dist,
            now_s=now_s,
            freeze_expiry=freeze_expiry,
        )

        if has_opposing:
            return False, {
                "is_exit": False,
                "reason": "OPPOSING_BREADCRUMBS_IN_CHANNEL",
                "gap_dist": float(gap_dist),
                "opposing_count": int(count),
                "breadcrumbs": bcs,
            }

        return True, {
            "is_exit": True,
            "reason": "SAFE_EARLY_EXIT",
            "gap_dist": float(gap_dist),
            "opposing_count": 0,
            "breadcrumbs": [],
        }


# ==============================================================================
# UNIFIED ANTI-WRONG-WAY WATCHDOG COORDINATOR
# ==============================================================================
class AntiWrongWayWatchdog:
    """
    Unified manager coordinating wrong-way protection via Breadcrumb LOS Raycasting.
    Supported modes:
      - DISABLED
      - OPTION_1_BREADCRUMB
    """

    CONFIRM_STEPS_THRESHOLD = 5  # Require >= 5 consecutive steps (~83 ms @ 60 Hz)

    def __init__(
        self,
        mode: str = ANTI_WRONG_WAY_MODE,
        lookback_m: float = AWW_BREADCRUMB_LOOKBACK_M,
        duration_scale: float = AWW_BREADCRUMB_DURATION_SCALE,
        max_timeout_s: float = AWW_BREADCRUMB_MAX_TIMEOUT_S,
        confirm_time_s: float = AWW_CONFIRM_TIME_S,
        post_recovery_cooldown_s: float = AWW_POST_RECOVERY_COOLDOWN_S,
        **kwargs,
    ):
        self.mode = mode
        self.breadcrumb_watchdog = BreadcrumbWatchdog(
            lookback_m=lookback_m,
            duration_scale=duration_scale,
            max_timeout_s=max_timeout_s,
            **kwargs,
        )
        self.latest_status = WrongWayStatus()
        self.consecutive_ww_steps = 0
        self.confirm_time_s = float(confirm_time_s)
        self.ww_confirm_s = 0.0
        self.post_recovery_cooldown_duration_s = float(post_recovery_cooldown_s)
        self.post_recovery_cooldown_s = 0.0
        self._last_update_s: Optional[float] = None
        self._was_in_rec_180: bool = False
        self._last_frozen: bool = False

    def set_duration_scale(self, scale: float):
        """Dynamically updates how fast breadcrumbs disappear."""
        self.breadcrumb_watchdog.set_duration_scale(scale)

    def set_max_timeout(self, timeout_s: float):
        """Dynamically updates maximum breadcrumb history timeout."""
        self.breadcrumb_watchdog.set_max_timeout(timeout_s)

    def set_mode(self, mode: str):
        self.mode = mode
        self.reset()

    def reset(self):
        self.breadcrumb_watchdog.reset()
        self.latest_status = WrongWayStatus()
        self.consecutive_ww_steps = 0
        self.ww_confirm_s = 0.0
        self.post_recovery_cooldown_s = 0.0
        self._last_update_s = None
        self._was_in_rec_180 = False
        self._last_frozen = False

    def record_breadcrumb(
        self,
        x: float,
        y: float,
        yaw: float,
        v_est: float,
        yaw_rate: float,
        now_s: float,
        state: str = "DRIVE",
        freeze_recording: bool = False,
        freeze_old_state: Optional[bool] = None,
        dt: Optional[float] = None,
    ) -> None:
        """Records or freezes breadcrumb state for the current frame."""
        if self.mode == WrongWayOptionMode.DISABLED or self.mode == "DISABLED":
            self._last_frozen = False
            return
        in_lane_occ = self.breadcrumb_watchdog.last_in_lane_opposing_count
        eff_freeze_rec = bool(freeze_recording) or (in_lane_occ >= AWW_MIN_CONFIRMED_BREADCRUMBS)
        state_str = getattr(state, "value", getattr(state, "name", str(state)))
        in_recovery = state_str not in ("DRIVE", "NORMAL_DRIVE")
        if freeze_old_state is None:
            eff_freeze_old = in_recovery or (v_est < 0.0) or (abs(yaw_rate) > 3.5) or eff_freeze_rec
        else:
            eff_freeze_old = bool(freeze_old_state) or in_recovery or (v_est < 0.0) or (abs(yaw_rate) > 3.5) or eff_freeze_rec
        self._last_frozen = eff_freeze_old
        self.breadcrumb_watchdog.add_point(
            x,
            y,
            yaw,
            v_est,
            now_s,
            state=state,
            yaw_rate=yaw_rate,
            freeze_recording=eff_freeze_rec,
            freeze_old_state=eff_freeze_old,
            dt=dt,
        )

    def update(
        self,
        x: float,
        y: float,
        yaw: float,
        v_est: float,
        yaw_rate: float,
        cached_tofs: Optional[Any] = None,
        now_s: Optional[float] = None,
        hazard_mask: Optional[np.ndarray] = None,
        state: str = "DRIVE",
        candidate_gaps: Optional[List[Any]] = None,
        costmap_obj: Optional[Any] = None,
        unknown_mask: Optional[np.ndarray] = None,
        dt: Optional[float] = None,
        record_breadcrumbs: bool = True,
        **kwargs,
    ) -> WrongWayStatus:
        """
        Executes one update step of the active anti-wrong-way detection mode.
        """
        if now_s is None:
            now_s = time.time()

        if dt is not None and dt > 0.0:
            dt_step = float(dt)
        elif self._last_update_s is not None:
            dt_step = max(0.0, min(0.1, float(now_s) - self._last_update_s))
        else:
            dt_step = 1.0 / 60.0
        self._last_update_s = float(now_s)

        if self.mode == WrongWayOptionMode.DISABLED or self.mode == "DISABLED":
            self.latest_status = WrongWayStatus()
            self.consecutive_ww_steps = 0
            self.ww_confirm_s = 0.0
            self.post_recovery_cooldown_s = 0.0
            self._was_in_rec_180 = False
            return self.latest_status

        is_rec_180 = (
            state == "CORRECTIVE_180"
            or getattr(state, "name", "") == "CORRECTIVE_180"
            or getattr(state, "value", "") == "CORRECTIVE_180"
        )
        if is_rec_180 != self._was_in_rec_180:
            self.consecutive_ww_steps = 0
            self.ww_confirm_s = 0.0
            if self._was_in_rec_180 and not is_rec_180:
                self.post_recovery_cooldown_s = self.post_recovery_cooldown_duration_s
            self._was_in_rec_180 = is_rec_180

        # 1. Check Line-of-Sight Breadcrumb Overlap BEFORE recording new breadcrumbs
        #    so we can detect occluded opposing in-lane trails and prevent overwrite.
        status = WrongWayStatus()
        state_str = getattr(state, "value", getattr(state, "name", str(state)))
        in_recovery = state_str not in ("DRIVE", "NORMAL_DRIVE")
        is_ww, score, target_yaw, rays = self.breadcrumb_watchdog.check_wrong_way(
            x,
            y,
            yaw,
            hazard_mask=hazard_mask,
            now_s=now_s,
            costmap_obj=costmap_obj,
            unknown_mask=unknown_mask,
            freeze_expiry=(is_rec_180 or in_recovery or self._last_frozen),
        )
        in_lane_occ = self.breadcrumb_watchdog.last_in_lane_opposing_count

        # 2. Breadcrumb logging with anti-contamination & occlusion anti-overwrite gating
        if record_breadcrumbs:
            freeze_rec = (in_lane_occ >= AWW_MIN_CONFIRMED_BREADCRUMBS)
            self.record_breadcrumb(
                x=x,
                y=y,
                yaw=yaw,
                v_est=v_est,
                yaw_rate=yaw_rate,
                now_s=now_s,
                state=state,
                freeze_recording=freeze_rec,
                dt=dt_step,
            )

        status.sightline_rays = rays
        status.overlap_score = score
        if is_ww:
            status.is_wrong_way = True
            status.confidence = score
            status.detected_by_option = "OPTION_1_BREADCRUMB"
            status.target_recovery_yaw = target_yaw
            status.corrective_action = "CORRECTIVE_180"

        # 3. Check Candidate Gaps for Long-Path Early Exits (Narrow-Cone Sentry)
        # Early exit is ONLY valid during U-turn recovery (CORRECTIVE_180) to terminate the turn early,
        # and ONLY when the vehicle has turned enough that forward Line-of-Sight is NO LONGER facing
        # confirmed opposing breadcrumbs (is_ww is False).
        if is_rec_180 and not is_ww:
            c_gaps = candidate_gaps or kwargs.get("candidate_gaps", None)
            if c_gaps:
                for c_gap in c_gaps:
                    if isinstance(c_gap, (list, tuple)) and len(c_gap) >= 2:
                        gx, gy = float(c_gap[0]), float(c_gap[1])
                        is_exit, _ = self.is_gap_early_exit(
                            x, y, yaw, gx, gy, now_s=now_s, freeze_expiry=True
                        )
                        if is_exit:
                            status.early_exit_found = True
                            status.early_exit_gap = (gx, gy)
                            break

        # 4. Post-recovery cooldown & bounded multi-step/time confirmation filter
        if is_rec_180:
            self.consecutive_ww_steps = 0
            self.ww_confirm_s = 0.0
        elif self.post_recovery_cooldown_s > 0.0:
            self.post_recovery_cooldown_s = max(0.0, self.post_recovery_cooldown_s - dt_step)
            status.is_wrong_way = False
            status.corrective_action = "NONE"
            self.consecutive_ww_steps = 0
            self.ww_confirm_s = 0.0
        elif status.is_wrong_way:
            self.consecutive_ww_steps = min(self.CONFIRM_STEPS_THRESHOLD, self.consecutive_ww_steps + 1)
            self.ww_confirm_s = min(self.confirm_time_s, self.ww_confirm_s + dt_step)
            if self.consecutive_ww_steps < self.CONFIRM_STEPS_THRESHOLD and self.ww_confirm_s < self.confirm_time_s:
                status.pending_wrong_way = True
                status.is_wrong_way = False
                status.corrective_action = "NONE"
        else:
            self.consecutive_ww_steps = max(0, self.consecutive_ww_steps - 1)
            self.ww_confirm_s = max(0.0, self.ww_confirm_s - dt_step)

        self.latest_status = status
        return status

    def check_gap_channel(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        gap_xg: float,
        gap_yg: float,
        cone_deg: float = AWW_EARLY_EXIT_CONE_DEG,
        opposing_deg: float = AWW_EARLY_EXIT_OPPOSING_DEG,
        min_gap_dist: float = AWW_EARLY_EXIT_MIN_GAP_M,
        min_bc_dist: float = AWW_EARLY_EXIT_MIN_BC_DIST_M,
        now_s: Optional[float] = None,
        freeze_expiry: bool = False,
    ) -> Tuple[bool, int, List[Dict[str, Any]]]:
        """Exposes Narrow-Cone Early Gap Exit Sentry from BreadcrumbWatchdog."""
        return self.breadcrumb_watchdog.check_gap_channel(
            current_x=current_x,
            current_y=current_y,
            current_yaw=current_yaw,
            gap_xg=gap_xg,
            gap_yg=gap_yg,
            cone_deg=cone_deg,
            opposing_deg=opposing_deg,
            min_gap_dist=min_gap_dist,
            min_bc_dist=min_bc_dist,
            now_s=now_s,
            freeze_expiry=freeze_expiry,
        )

    def is_gap_early_exit(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        gap_xg: float,
        gap_yg: float,
        min_gap_dist: float = AWW_EARLY_EXIT_MIN_GAP_M,
        cone_deg: float = AWW_EARLY_EXIT_CONE_DEG,
        opposing_deg: float = AWW_EARLY_EXIT_OPPOSING_DEG,
        min_bc_dist: float = AWW_EARLY_EXIT_MIN_BC_DIST_M,
        now_s: Optional[float] = None,
        freeze_expiry: bool = False,
    ) -> Tuple[bool, Dict[str, Any]]:
        """Evaluates whether candidate gap qualifies as a safe long-path early exit."""
        return self.breadcrumb_watchdog.is_gap_early_exit(
            current_x=current_x,
            current_y=current_y,
            current_yaw=current_yaw,
            gap_xg=gap_xg,
            gap_yg=gap_yg,
            min_gap_dist=min_gap_dist,
            cone_deg=cone_deg,
            opposing_deg=opposing_deg,
            min_bc_dist=min_bc_dist,
            now_s=now_s,
            freeze_expiry=freeze_expiry,
        )

    def get_reverse_breadcrumb_yaw(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        target_dist_m: float = 0.45,
        min_dist_m: float = 0.25,
        max_dist_m: float = 0.70,
    ) -> Optional[float]:
        """Returns a reverse-pursuit target yaw toward the recent breadcrumb closest to ~0.45m."""
        if self.mode == WrongWayOptionMode.DISABLED or self.mode == "DISABLED":
            return None
        return self.breadcrumb_watchdog.get_reverse_breadcrumb_yaw(
            current_x=current_x,
            current_y=current_y,
            current_yaw=current_yaw,
            target_dist_m=target_dist_m,
            min_dist_m=min_dist_m,
            max_dist_m=max_dist_m,
        )

    def get_detailed_telemetry(
        self,
        x: float,
        y: float,
        yaw: float,
        v_est: float,
        yaw_rate: float,
        now_s: Optional[float] = None,
        hazard_mask: Optional[np.ndarray] = None,
        costmap_obj: Optional[Any] = None,
        unknown_mask: Optional[np.ndarray] = None,
        **kwargs,
    ) -> Dict[str, Any]:
        """
        Generates rich visual and narrative telemetry to stream to the Thought Process GUI.
        """
        if now_s is None:
            now_s = time.time()

        # Breadcrumb LOS raycast
        ww_bc, score_bc, target_yaw_bc, rays_bc = self.breadcrumb_watchdog.check_wrong_way(
            x,
            y,
            yaw,
            hazard_mask=hazard_mask,
            now_s=now_s,
            costmap_obj=costmap_obj,
            unknown_mask=unknown_mask,
            freeze_expiry=self._last_frozen,
        )
        n_bc = len(self.breadcrumb_watchdog.breadcrumbs)
        eff_lookback = self.breadcrumb_watchdog.effective_lookback_m
        dur_scale = self.breadcrumb_watchdog.duration_scale
        thought_bc = (
            f"Trail has {n_bc} points ({eff_lookback:.1f}m window @ {dur_scale:.2f}x). Unobstructed line-of-sight to opposing trail confirmed (score: {score_bc:.2f}) -> IN-LANE WRONG-WAY DETECTED! Action: Spin 180° back."
            if ww_bc else
            f"Trail has {n_bc} points ({eff_lookback:.1f}m window @ {dur_scale:.2f}x). Raycast scan: forward corridor is clear or adjacent lane blocked by wall. Heading towards open track."
        )

        trail_pts = [[round(p.x, 3), round(p.y, 3)] for p in list(self.breadcrumb_watchdog.breadcrumbs)[::2]]

        return {
            "mode": self.mode.name if hasattr(self.mode, "name") else str(self.mode),
            "pose": [round(float(x), 3), round(float(y), 3), round(float(yaw), 3)],
            "is_wrong_way": self.latest_status.is_wrong_way,
            "detected_by": self.latest_status.detected_by_option,
            "action": self.latest_status.corrective_action,
            "target_yaw": round(float(self.latest_status.target_recovery_yaw or 0.0), 3),
            "opt1": {
                "name": "Option 1: Breadcrumb Trail Overlap & LOS Raycast",
                "score": round(float(score_bc), 3),
                "is_tripped": bool(ww_bc),
                "trail_count": n_bc,
                "target_yaw": round(float(target_yaw_bc), 3),
                "sightlines": rays_bc,
                "duration_scale": round(float(dur_scale), 2),
                "effective_lookback_m": round(float(eff_lookback), 2),
                "thought": thought_bc,
            },
            "opt2": {
                "name": "Option 2: Kinematic Turn Radius Filter (Deprecated)",
                "radius_m": 0.50,
                "min_radius_m": 0.32,
                "is_tripped": False,
                "v_est": round(float(v_est), 3),
                "yaw_rate": round(float(yaw_rate), 3),
                "thought": "Option 2 retired in favor of Breadcrumb LOS Raycast.",
            },
            "opt3": {
                "name": "Option 3: Directional Flow Costmap (Deprecated)",
                "dot_product": 1.0,
                "is_tripped": False,
                "cells_count": 0,
                "thought": "Option 3 retired in favor of Breadcrumb LOS Raycast.",
            },
            "opt4": {
                "name": "Option 4: Lateral Corridor Separation (Deprecated)",
                "in_turn": False,
                "turn_deg": 0.0,
                "lateral_m": 0.0,
                "min_lateral_m": 0.40,
                "is_tripped": False,
                "thought": "Option 4 retired in favor of Breadcrumb LOS Raycast.",
            },
            "early_exit": {
                "found": bool(self.latest_status.early_exit_found),
                "gap": [round(float(v), 3) for v in self.latest_status.early_exit_gap] if self.latest_status.early_exit_gap else None,
            },
            "trail": trail_pts[-60:] if trail_pts else [],
        }


=== FILE: brain/autonomous_brain.py ===
"""
================================================================================
  FOLKRACE AUTONOMOUS BRAIN: MASTER COORDINATOR (FRAMEWORK-AGNOSTIC)
================================================================================
  Single authoritative autonomous perception and trajectory optimization brain.
  Executed identically in:
    1. Webots Simulation (120 Hz / 60 Hz harness)
    2. Raspberry Pi 4 Real-World Runner (Core 1 dedicated 60 Hz loop)

  Encapsulates:
    - 30 mm minimum floor ToF perception pre-processor
    - C++ Smart Hybrid Costmap (native AVX2 / ARM64 .so)
    - ToF Gap Scout & Aperture Evaluator
    - Curvilinear Slack-Constrained SL-MPC (Native OSQP solver)
    - SE(2) Forward Lookahead Anchor Pose Projector (cancels ToF & actuator lag)
    - Stuck Recovery State Machine & 5-Step Costmap Escape Arc Evaluator
    - Anti-Wrong-Way Watchdog Sentry & Early Gap Exit Evaluator
================================================================================
"""

import os
import sys
import time
import math
from typing import Dict, List, Optional, Tuple, Any, Union
import numpy as np

# Ensure brain directory is on sys.path
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
if CURR_DIR not in sys.path:
    sys.path.insert(0, CURR_DIR)

try:
    from .pose_tracker import PoseTracker, wrap_to_pi
    from .cpp_smart_hybrid_costmap import CppSmartHybridCostmap, SENSOR_NAMES
    from .tof_gap_scout import ToFGapScout
    from .planner_sl_mpc import CurvilinearSLMPC
    from .stuck_recovery_manager import StuckRecoveryManager, StuckRecoveryState
    from .anti_wrong_way_watchdog import AntiWrongWayWatchdog
    from .ramp_detector import RampDetector
except ImportError:
    from pose_tracker import PoseTracker, wrap_to_pi
    from cpp_smart_hybrid_costmap import CppSmartHybridCostmap, SENSOR_NAMES
    from tof_gap_scout import ToFGapScout
    from planner_sl_mpc import CurvilinearSLMPC
    from stuck_recovery_manager import StuckRecoveryManager, StuckRecoveryState
    from anti_wrong_way_watchdog import AntiWrongWayWatchdog
    from ramp_detector import RampDetector

try:
    from .planner_config import (
        GRID_X_MIN,
        GRID_X_MAX,
        GRID_Y_MIN,
        GRID_Y_MAX,
        GRID_RES,
        R_INFLATION_M,
        V_MIN_CRUISE,
        V_MAX_CRUISE,
        MAX_STEERING_RAD,
        SERVO_MAX_SPEED_RAD_S,
        RECOVERY_REVERSE_SPEED_MPS,
        RECOVERY_CORRECTIVE_180_SPEED_MPS,
        RECOVERY_STEER_ANGLE_RAD,
        RECOVERY_CLEARANCE_EXIT_M,
        RECOVERY_ANY_TOF_EXIT_M,
        RECOVERY_ARC_LOOKAHEAD_M,
        BRAKE_DECEL_MAX,
        ACCEL_FWD_MAX,
        ACCEL_FWD_RATIO,
        BRAKE_DECEL_RATIO,
        GRAVITY_G,
        MPC_MU_FRICTION,
        MPC_CENTER_BIAS,
        MPC_MAX_ITER,
        COSTMAP_PERSISTENCE_FRAMES,
        GAP_USE_TRUE_APERTURE,
        WEIGHT_GAP_CENTER,
        TOF_MAX_RANGE_M,
        WHEELBASE,
    )
except ImportError:
    from planner_config import (
        GRID_X_MIN,
        GRID_X_MAX,
        GRID_Y_MIN,
        GRID_Y_MAX,
        GRID_RES,
        R_INFLATION_M,
        V_MIN_CRUISE,
        V_MAX_CRUISE,
        MAX_STEERING_RAD,
        SERVO_MAX_SPEED_RAD_S,
        RECOVERY_REVERSE_SPEED_MPS,
        RECOVERY_CORRECTIVE_180_SPEED_MPS,
        RECOVERY_STEER_ANGLE_RAD,
        RECOVERY_CLEARANCE_EXIT_M,
        RECOVERY_ANY_TOF_EXIT_M,
        RECOVERY_ARC_LOOKAHEAD_M,
        BRAKE_DECEL_MAX,
        ACCEL_FWD_MAX,
        ACCEL_FWD_RATIO,
        BRAKE_DECEL_RATIO,
        GRAVITY_G,
        MPC_MU_FRICTION,
        MPC_CENTER_BIAS,
        MPC_MAX_ITER,
        COSTMAP_PERSISTENCE_FRAMES,
        GAP_USE_TRUE_APERTURE,
        WEIGHT_GAP_CENTER,
        TOF_MAX_RANGE_M,
        WHEELBASE,
    )

MIN_TOF_PHYSICAL_M = 0.030  # 30 mm minimum floor matching physical VL53L8CX blind zone


class AutonomousBrain:
    """
    Unified, framework-agnostic Autonomous Driving Brain for Folkrace.
    Consumes clean physical sensor readings and produces motion targets.
    """

    def __init__(self, genes: Optional[Dict[str, Any]] = None, disable_alt_solver: bool = False):
        self.disable_alt_solver = bool(disable_alt_solver)
        # 1. Perception & Navigation Engines
        self.r_infl = R_INFLATION_M
        self.costmap = CppSmartHybridCostmap(
            grid_bounds=(GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX),
            resolution=GRID_RES,
            r_inflation_m=self.r_infl,
        )
        self.gap_scout = ToFGapScout()
        self.planner = CurvilinearSLMPC(costmap_engine=self.costmap)
        self.planner.costmap = self.costmap

        self.pose_tracker = PoseTracker()

        # 2. Safety State Machines
        self.recovery_manager = StuckRecoveryManager(
            reverse_speed_m_s=RECOVERY_REVERSE_SPEED_MPS,
            forward_speed_m_s=V_MIN_CRUISE,
            corrective_forward_speed_m_s=RECOVERY_CORRECTIVE_180_SPEED_MPS,
            steer_angle_rad=RECOVERY_STEER_ANGLE_RAD,
            clearance_exit_m=RECOVERY_CLEARANCE_EXIT_M,
            any_tof_exit_m=RECOVERY_ANY_TOF_EXIT_M,
            servo_slew_rad_s=SERVO_MAX_SPEED_RAD_S,
            brake_decel_max=BRAKE_DECEL_MAX,
            accel_fwd_max=ACCEL_FWD_MAX,
            arc_lookahead_m=RECOVERY_ARC_LOOKAHEAD_M,
        )
        self.watchdog = AntiWrongWayWatchdog()
        self.ramp_detector = RampDetector()

        # 3. Dynamic Tuning Parameters
        self.base_mu = MPC_MU_FRICTION
        self.mu_val = self.base_mu
        self.target_lookahead_m = 2.40
        self.aggr_dist_far = 2.40
        self.aggr_dist_close = 1.00
        self.aggr_min_scale = 0.30
        self.steer_deadband = 0.002
        self.v_cruise_min = V_MIN_CRUISE
        self.v_cruise_max = V_MAX_CRUISE

        # 4. Internal State Memories
        self.step_counter = 0
        self.active_steer = 0.0
        self.cached_target_steer = 0.0
        self.cached_target_speed = 0.0
        self.rate_limited_speed = 0.0
        self.was_recovering = False
        self.mode_desc = "SL_MPC"
        self.last_aww_status = None
        self.last_gap_info = (0.0, 1.0, 0.50, 0.0)
        self.last_plan_res: Dict[str, Any] = {}
        self.cached_f_tof = TOF_MAX_RANGE_M
        self._clean_tofs: Dict[str, float] = {name: TOF_MAX_RANGE_M for name in SENSOR_NAMES}

        if genes:
            self.configure(genes)
        else:
            try:
                from .planner_config import load_best_genome_genes
            except ImportError:
                from planner_config import load_best_genome_genes
            best_genes, best_path = load_best_genome_genes()
            if best_genes:
                print(f"[AutonomousBrain] Auto-loaded best genome from {best_path}", flush=True)
                self.configure(best_genes)

    def configure(self, genes: Dict[str, Any]) -> None:
        """Applies dynamic tuning parameters or genome weights."""
        if not genes:
            return

        self.base_mu = float(genes.get("mu_friction", MPC_MU_FRICTION))
        self.mu_val = self.base_mu
        self.r_infl = float(genes.get("r_inflation_m", R_INFLATION_M))
        self.target_lookahead_m = float(genes.get("target_lookahead_m", 2.40))
        self.aggr_dist_far = float(genes.get("aggr_dist_far_m", 2.40))
        self.aggr_dist_close = float(genes.get("aggr_dist_close_m", 1.00))
        self.aggr_min_scale = float(genes.get("aggr_min_scale", 0.30))
        self.steer_deadband = float(genes.get("steer_deadband_rad", 0.002))
        self.v_cruise_min = float(genes.get("v_cruise_min", V_MIN_CRUISE))
        self.v_cruise_max = float(genes.get("v_cruise_max", V_MAX_CRUISE))

        self.costmap = CppSmartHybridCostmap(
            grid_bounds=(GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX),
            resolution=GRID_RES,
            r_inflation_m=self.r_infl,
        )
        if hasattr(self.costmap, "set_advanced_params"):
            self.costmap.set_advanced_params(
                persistence_frames=int(genes.get("costmap_persistence", COSTMAP_PERSISTENCE_FRAMES)),
                fix_yaw_sign=int(genes.get("fix_yaw_sign", 1)),
                use_true_aperture=int(genes.get("use_true_aperture", GAP_USE_TRUE_APERTURE)),
                w_gap_center=float(genes.get("w_gap_center", WEIGHT_GAP_CENTER)),
            )

        self.gap_scout = ToFGapScout(
            w_gap_distance=float(genes.get("w_gap_distance", 3.50)),
            w_gap_width=float(genes.get("w_gap_width", 2.00)),
            w_align_steer=float(genes.get("w_gap_steer", 0.80)),
            w_align_straight=float(genes.get("w_gap_straight", 0.40)),
            w_side_clearance=float(genes.get("w_side_clearance", 2.80)),
            gap_dist_exp_k=float(genes.get("gap_dist_exp_k", 0.60)),
            gap_dist_thresh_m=float(genes.get("gap_dist_thresh_m", 0.95)),
            alt_min_separation_m=float(genes.get("alt_min_separation_m", 0.25)),
            alt_aperture_bias=float(genes.get("alt_aperture_bias", 1.80)),
            scout_obs_width_m=float(genes.get("scout_obs_width_m", 0.22)),
            scout_block_dist_m=float(genes.get("scout_block_dist_m", self.target_lookahead_m)),
            w_gap_clear=float(genes.get("w_gap_clear", genes.get("alt_lookahead_depth_weight", 3.00))),
        )

        self.planner = CurvilinearSLMPC(
            costmap_engine=self.costmap,
            horizon_N=int(genes.get("horizon_N", 30)),
            dt=float(genes.get("dt", 0.016667)),
            q_progress=float(genes.get("q_progress", 8.00)),
            q_cte=float(genes.get("q_cte", 18.00)),
            q_heading=float(genes.get("q_heading", 8.00)),
            r_steer=float(genes.get("r_steer", 5.00)),
            r_accel=float(genes.get("r_accel", 2.00)),
            r_steer_rate=float(genes.get("r_steer_rate", 10.00)),
            mu_friction=self.mu_val,
            v_corner_factor=float(genes.get("v_corner_factor", 0.95)),
            v_cruise_max=self.v_cruise_max,
            v_cruise_min=self.v_cruise_min,
            alt_switch_hysteresis=float(genes.get("alt_switch_hysteresis", 15.00)),
            alt_min_separation_m=float(genes.get("alt_min_separation_m", 0.25)),
            alt_aperture_bias=float(genes.get("alt_aperture_bias", 1.80)),
            w_gap_clear=float(genes.get("w_gap_clear", 5.00)),
            center_bias=float(genes.get("center_bias", MPC_CENTER_BIAS)),
            track_width_cap_m=float(genes.get("track_width_cap_m", 1.25)),
            corner_outer_bias=float(genes.get("corner_outer_bias", 0.0)),
            inner_corner_margin_m=float(genes.get("inner_corner_margin_m", 0.0)),
            blind_turn_kappa=float(genes.get("blind_turn_kappa", 0.0)),
            brake_decel_scale=float(genes.get("brake_decel_scale", 1.0)),
            narrow_corridor_v_gain=float(genes.get("narrow_corridor_v_gain", 0.0)),
            use_corridor_flow=int(genes.get("use_corridor_flow", 0)),
            fix_yaw_sign=int(genes.get("fix_yaw_sign", 1)),
            mpc_v_nom_min=float(genes.get("mpc_v_nom_min", 1.20)),
            mpc_max_iter=int(genes.get("mpc_max_iter", MPC_MAX_ITER)),
        )
        self.planner.costmap = self.costmap

        rec_steer = min(MAX_STEERING_RAD, float(genes.get("recovery_steer_angle_rad", RECOVERY_STEER_ANGLE_RAD)))
        r_min_turn = WHEELBASE / max(0.01, math.tan(abs(rec_steer)))
        rec_speed = float(math.sqrt(max(0.01, self.mu_val * 9.81 * r_min_turn)))
        if "recovery_reverse_speed" in genes:
            rec_speed = float(genes["recovery_reverse_speed"])
        rec_fwd_speed = float(math.sqrt(max(0.01, 0.90 * self.mu_val * 9.81 * r_min_turn)))

        self.recovery_manager = StuckRecoveryManager(
            reverse_speed_m_s=rec_speed,
            corrective_forward_speed_m_s=rec_fwd_speed,
            steer_angle_rad=rec_steer,
            clearance_exit_m=float(genes.get("recovery_clearance_exit_m", RECOVERY_CLEARANCE_EXIT_M)),
            any_tof_exit_m=float(genes.get("recovery_any_tof_exit_m", RECOVERY_ANY_TOF_EXIT_M)),
            servo_slew_rad_s=SERVO_MAX_SPEED_RAD_S,
            brake_decel_max=BRAKE_DECEL_MAX,
            accel_fwd_max=ACCEL_FWD_MAX,
            arc_lookahead_m=float(genes.get("recovery_arc_lookahead_m", RECOVERY_ARC_LOOKAHEAD_M)),
            mu_friction=self.mu_val,
        )

    def set_dynamic_mu(self, new_mu: float) -> None:
        """Dynamically updates the effective tire-road friction coefficient."""
        self.mu_val = float(new_mu)
        if hasattr(self.planner, "set_dynamic_mu"):
            self.planner.set_dynamic_mu(new_mu)
        if hasattr(self.recovery_manager, "mu_friction"):
            self.recovery_manager.mu_friction = float(new_mu)
            r_min_turn = WHEELBASE / max(0.01, math.tan(max(0.01, abs(self.recovery_manager.steer_angle))))
            self.recovery_manager.corrective_forward_speed = float(
                math.sqrt(max(0.01, 0.90 * self.mu_val * 9.81 * r_min_turn))
            )

    def reset(self, start_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        """Resets all internal filters, costmap, pose tracker, and recovery state machines."""
        self.costmap.clear()
        self.pose_tracker.reset(start_pose[0], start_pose[1], start_pose[2])
        self.recovery_manager.reset()
        self.watchdog.reset()
        self.ramp_detector.reset()
        self.last_aww_status = None
        if hasattr(self.planner, "reset_active_target"):
            self.planner.reset_active_target()
        self.step_counter = 0
        self.active_steer = 0.0
        self.cached_target_steer = 0.0
        self.cached_target_speed = 0.0
        self.rate_limited_speed = 0.0
        self.was_recovering = False
        self.mode_desc = "SL_MPC"
        self.cached_f_tof = TOF_MAX_RANGE_M

    def step(
        self,
        tof_readings: Union[Dict[str, float], List[float]],
        v_est: float,
        steer_actual_rad: float,
        gyro_z_rad_s: float,
        dt: float = 0.016667,
        sim_time_s: float = 0.0,
        ground_truth_pose: Optional[Tuple[float, float, float]] = None,
        full_telemetry: bool = True,
        pitch_rad: float = 0.0,
        roll_rad: float = 0.0,
        imu_yaw_rad: Optional[float] = None,
    ) -> Tuple[float, float, Dict[str, Any]]:
        """
        Executes one full autonomous perception and trajectory optimization step (typically @ 60 Hz).

        Args:
            tof_readings: 16-channel ToF ranges (either as dict name->meters or list of 16 floats).
            v_est: Current fused forward velocity estimate (m/s).
            steer_actual_rad: Current active steering angle of the front wheels (radians).
            gyro_z_rad_s: Current body yaw rate around Z-axis (rad/s).
            dt: Timestep duration in seconds (default 1/60s = 0.016667s).
            sim_time_s: Current wall or simulation timestamp in seconds.
            ground_truth_pose: Optional (x, y, yaw) from simulator supervisor for global tracking.
            full_telemetry: If False, skips constructing high-allocation trajectory arrays (lean fast-path).

        Returns:
            Tuple[float, float, Dict[str, Any]]:
              - cmd_speed_mps: Commanded forward linear velocity (m/s).
              - cmd_steer_rad: Commanded virtual center steering angle (radians).
              - telemetry: Comprehensive or lean telemetry dictionary for logging/GUI.
        """
        t_start = time.perf_counter()
        self.step_counter += 1

        # 1. Perception Pre-Processing & 30mm Blind-Zone Clamping (In-Place Mutation)
        # Note: values <= 0.0 represent offline/missing sensors and are preserved as 0.0
        # so the costmap leaves their rays as unknown space rather than a 30mm obstacle.
        clean_tofs = self._clean_tofs
        if isinstance(tof_readings, (list, tuple)):
            for i, name in enumerate(SENSOR_NAMES):
                val = float(tof_readings[i]) if i < len(tof_readings) else TOF_MAX_RANGE_M
                if val <= 0.0 or math.isnan(val):
                    clean_tofs[name] = 0.0
                else:
                    clean_tofs[name] = max(MIN_TOF_PHYSICAL_M, min(TOF_MAX_RANGE_M, val))
        else:
            for name in SENSOR_NAMES:
                val = float(tof_readings.get(name, TOF_MAX_RANGE_M))
                if val <= 0.0 or math.isnan(val):
                    clean_tofs[name] = 0.0
                else:
                    clean_tofs[name] = max(MIN_TOF_PHYSICAL_M, min(TOF_MAX_RANGE_M, val))

        _f_vals = [
            clean_tofs.get("tof_front_left_4.1deg", TOF_MAX_RANGE_M),
            clean_tofs.get("tof_front_right_4.1deg", TOF_MAX_RANGE_M),
        ]
        _valid_f = [v for v in _f_vals if v > 0.0]
        f_tof = min(_valid_f) if _valid_f else TOF_MAX_RANGE_M
        self.cached_f_tof = f_tof

        # 2. Dead-Reckoning Odometry Update
        self.pose_tracker.update_odometry(
            v_est=v_est, yaw_rate=gyro_z_rad_s, dt=dt, imu_yaw_rad=imu_yaw_rad
        )
        nav_pose = ground_truth_pose if ground_truth_pose is not None else (self.pose_tracker.x, self.pose_tracker.y, self.pose_tracker.yaw)

        # 3. C++ Smart Hybrid Costmap Processing
        t_map_0 = time.perf_counter()
        need_unknown = (getattr(self.watchdog, "mode", "DISABLED") != "DISABLED")
        prob_map, hazard_mask, map_ms = self.costmap.process_tof_frame(
            clean_tofs, need_prob_map=full_telemetry, need_unknown_mask=need_unknown
        )
        t_map_ms = (time.perf_counter() - t_map_0) * 1000.0

        # 4. 3-Step Composite SE(2) Forward Lookahead Anchor Projection
        is_reversing = (self.recovery_manager.state != StuckRecoveryState.NORMAL_DRIVE)
        (dx_3, dy_3, delta_yaw_3), v_3, steer_3 = self.pose_tracker.compute_projected_anchor(
            v_current=v_est,
            steer_current=steer_actual_rad,
            steer_target=self.cached_target_steer,
            dt=dt,
            is_reversing=is_reversing,
        )

        # 5. Gap Scouting from Projected Anchor
        t_scout_0 = time.perf_counter()
        xg, yg, width, theta_g = self.gap_scout.scout_gap(
            tof_readings=clean_tofs,
            hazard_mask=hazard_mask,
            costmap_obj=self.costmap,
            target_lookahead_m=self.target_lookahead_m,
            steer_current=steer_3,
            prob_map=prob_map if full_telemetry else None,
            anchor_x=dx_3,
            anchor_y=dy_3,
            anchor_yaw=delta_yaw_3,
        )
        t_scout_ms = (time.perf_counter() - t_scout_0) * 1000.0
        self.last_gap_info = (xg, yg, width, theta_g)

        # 6. Curvilinear SL-MPC Trajectory Optimization
        t_plan_0 = time.perf_counter()
        plan_res = self.planner.plan(
            xg=xg,
            yg=yg,
            theta_g=theta_g,
            aperture_width=width,
            v_current=v_3,
            steer_current=steer_3,
            yaw_rate_current=gyro_z_rad_s,
            extra_targets=(None if self.disable_alt_solver else getattr(self.gap_scout, "last_targets", None)),
            x0=dx_3,
            y0=dy_3,
            psi0=delta_yaw_3,
            t_frame_start=t_start,
            pitch_rad=pitch_rad,
            full_telemetry=full_telemetry,
        )
        t_plan_ms = (time.perf_counter() - t_plan_0) * 1000.0
        self.last_plan_res = plan_res

        raw_target_steer = float(plan_res.get("cmd_steer_rad", 0.0))
        raw_target_speed = float(plan_res.get("cmd_speed_mps", self.v_cruise_min))

        min_fwd_dist = f_tof

        # 7. Candidate Gap Scout Target for Early Exit (primary target only — what SL-MPC will execute)
        candidate_gaps = []
        if min_fwd_dist >= 0.50:
            raw_targets = getattr(self.gap_scout, "last_targets", []) or []
            if raw_targets:
                tgt = raw_targets[0]
                if isinstance(tgt, (list, tuple)) and len(tgt) >= 2:
                    gx_c, gy_c = float(tgt[0]), float(tgt[1])
                    tgt_corr_yaw = getattr(self.recovery_manager, "target_corrective_yaw", None)
                    gap_aligned = True
                    if tgt_corr_yaw is not None and math.hypot(gx_c, gy_c) > 1e-3:
                        gap_world_yaw = wrap_to_pi(nav_pose[2] - math.atan2(gx_c, gy_c))
                        gap_aligned = abs(wrap_to_pi(tgt_corr_yaw - gap_world_yaw)) <= math.radians(35.0)
                    if gap_aligned:
                        candidate_gaps.append((gx_c, gy_c))

        # 8. Anti-Wrong-Way Watchdog (Detection before recovery arbitration; recording deferred until post-recovery speed/state settle)
        if getattr(self.watchdog, "mode", "DISABLED") != "DISABLED":
            aww_status = self.watchdog.update(
                x=nav_pose[0],
                y=nav_pose[1],
                yaw=nav_pose[2],
                v_est=v_est,
                yaw_rate=gyro_z_rad_s,
                cached_tofs=clean_tofs,
                now_s=sim_time_s,
                hazard_mask=hazard_mask,
                state=self.recovery_manager.state,
                candidate_gaps=candidate_gaps,
                costmap_obj=self.costmap,
                unknown_mask=self.costmap.unknown_mask,
                dt=dt,
                record_breadcrumbs=False,
            )
            self.last_aww_status = aww_status

            if aww_status.pending_wrong_way and not aww_status.is_wrong_way and self.recovery_manager.state == StuckRecoveryState.NORMAL_DRIVE:
                raw_target_speed = min(raw_target_speed, 0.0)

            if aww_status.is_wrong_way and self.recovery_manager.state == StuckRecoveryState.NORMAL_DRIVE:
                tgt_yaw = aww_status.target_recovery_yaw if aww_status.target_recovery_yaw is not None else wrap_to_pi(nav_pose[2] + math.pi)
                yaw_diff = wrap_to_pi(tgt_yaw - nav_pose[2])
                arc_cl = self.recovery_manager.evaluate_costmap_escape_arcs(hazard_mask, self.costmap)
                is_l_pin, is_r_pin = self.recovery_manager.evaluate_pinned_state(hazard_mask, self.costmap, clean_tofs)
                open_side = self.recovery_manager.resolve_open_side(
                    arc_clearance=arc_cl,
                    cached_tofs=clean_tofs,
                    heading_err=yaw_diff,
                    is_left_pinned=is_l_pin,
                    is_right_pinned=is_r_pin,
                )
                self.recovery_manager.trigger_corrective_180(
                    target_yaw=tgt_yaw,
                    steer_sign=open_side,
                    current_yaw=nav_pose[2],
                    arc_clearance=arc_cl,
                    cached_tofs=clean_tofs,
                    front_clearance_m=min_fwd_dist,
                    hazard_mask=hazard_mask,
                    costmap_obj=self.costmap,
                    is_left_pinned=is_l_pin,
                    is_right_pinned=is_r_pin,
                )
        else:
            self.last_aww_status = None

        # 9. Stuck Detection & Shield Reverse Evaluation
        _shield_cur = plan_res.get("shield", {})
        is_stall = (min_fwd_dist < 0.20 and v_est < 0.35 and raw_target_speed > 0.5)
        is_wall_contact = (min_fwd_dist <= 0.05)
        shield_reverse = bool(_shield_cur.get("reverse_suggested", False))

        is_aww_wrong_way = bool(self.last_aww_status and self.last_aww_status.is_wrong_way)
        target_breadcrumb_yaw = (
            self.last_aww_status.target_recovery_yaw
            if (self.last_aww_status and self.last_aww_status.target_recovery_yaw is not None)
            else self.watchdog.get_reverse_breadcrumb_yaw(nav_pose[0], nav_pose[1], nav_pose[2])
        )

        if (shield_reverse or is_wall_contact or is_stall) and self.recovery_manager.state == StuckRecoveryState.NORMAL_DRIVE:
            _arc_cl = self.recovery_manager.evaluate_costmap_escape_arcs(hazard_mask, self.costmap)
            _is_l_pin, _is_r_pin = self.recovery_manager.evaluate_pinned_state(hazard_mask, self.costmap, clean_tofs)
            _side = self.recovery_manager.resolve_escape_steer_dir(
                arc_clearance=_arc_cl, heading_err=float(theta_g),
                is_left_pinned=_is_l_pin, is_right_pinned=_is_r_pin,
            )
            self.recovery_manager.trigger_emergency_recovery(
                heading_err=float(theta_g),
                steer_dir=_side,
                current_v=v_est,
                current_steer=steer_actual_rad,
                instant_reverse=True,
                target_breadcrumb_yaw=target_breadcrumb_yaw,
                current_yaw=nav_pose[2],
                front_clearance_m=min_fwd_dist,
                hazard_mask=hazard_mask,
                costmap_obj=self.costmap,
                cached_tofs=clean_tofs,
                is_left_pinned=_is_l_pin,
                is_right_pinned=_is_r_pin,
                is_wrong_way=is_aww_wrong_way,
            )

        # 10. Execute Recovery State Machine
        is_early_exit = bool(
            self.last_aww_status
            and self.last_aww_status.early_exit_found
            and self.recovery_manager.state == StuckRecoveryState.CORRECTIVE_180
            and min_fwd_dist >= 0.80
        )
        is_recovering, rec_v, rec_steer = self.recovery_manager.update(
            dt=dt,
            current_v=v_est,
            commanded_v=raw_target_speed,
            cached_tofs=clean_tofs,
            heading_err=float(theta_g),
            active_steer=steer_actual_rad,
            hazard_mask=hazard_mask,
            costmap_obj=self.costmap,
            current_yaw=nav_pose[2],
            target_breadcrumb_yaw=target_breadcrumb_yaw,
            early_exit_available=is_early_exit,
            is_wrong_way=is_aww_wrong_way,
        )

        if is_recovering:
            if not self.was_recovering:
                self.rate_limited_speed = min(self.rate_limited_speed, max(0.0, v_est)) if rec_v >= 0.0 else min(self.rate_limited_speed, v_est)
            target_speed = rec_v
            target_steer = rec_steer
            self.active_steer = rec_steer
            rec_state = getattr(self.recovery_manager.state, 'name', str(self.recovery_manager.state))
            if rec_state == "CORRECTIVE_180":
                sub = getattr(self.recovery_manager, "corrective_substate", "FORWARD")
                self.mode_desc = f"PIVOT_180_{sub}"
            elif rec_state == "PIVOT_REVERSE":
                self.mode_desc = "PIVOT_REVERSE"
            elif rec_state == "DECEL_AND_CENTER":
                self.mode_desc = "DECEL_CENTER"
            else:
                self.mode_desc = f"ESCAPE_{rec_state}"
            self.was_recovering = True
            if hasattr(self.planner, "reset_active_target"):
                self.planner.reset_active_target()
        else:
            if self.was_recovering:
                self.was_recovering = False
                self.active_steer = steer_actual_rad

            target_speed = raw_target_speed
            target_steer = raw_target_steer
            is_shield_active = bool(_shield_cur.get("active", False))

            if is_shield_active:
                self.active_steer = max(-MAX_STEERING_RAD, min(MAX_STEERING_RAD, target_steer))
                c_name = _shield_cur.get("chosen_name", "ACTIVE").replace("_", " ").upper()
                self.mode_desc = f"SHIELD_{c_name}"
            else:
                steer_delta = target_steer - self.active_steer
                # Progressive distance & offset aggressiveness
                if min_fwd_dist >= self.aggr_dist_far:
                    dist_scale = self.aggr_min_scale
                elif min_fwd_dist <= self.aggr_dist_close:
                    dist_scale = 1.0
                else:
                    frac = (self.aggr_dist_far - min_fwd_dist) / max(0.001, (self.aggr_dist_far - self.aggr_dist_close))
                    dist_scale = float(self.aggr_min_scale + (frac ** 1.5) * (1.0 - self.aggr_min_scale))

                err_mag = abs(steer_delta)
                if err_mag <= self.steer_deadband:
                    err_scale = 0.90
                elif err_mag >= 0.05:
                    err_scale = 1.0
                else:
                    err_frac = (err_mag - self.steer_deadband) / max(0.001, (0.05 - self.steer_deadband))
                    err_scale = float(0.90 + err_frac * (1.0 - 0.90))

                steer_aggressiveness = float(max(0.85, min(1.0, dist_scale * err_scale)))
                self.active_steer = max(
                    -MAX_STEERING_RAD,
                    min(MAX_STEERING_RAD, self.active_steer + steer_aggressiveness * steer_delta),
                )
                self.mode_desc = "SL_MPC"

        # 10b. Ramp Detection & Speed Governance (Unified Hardware & Sim Safety)
        ramp_res = self.ramp_detector.update(pitch_rad=pitch_rad, roll_rad=roll_rad, dt=dt)
        if ramp_res["is_on_ramp"]:
            ramp_cap = float(ramp_res["speed_limit"])
            if target_speed > 0.0:
                target_speed = min(target_speed, ramp_cap)
            elif target_speed < 0.0:
                target_speed = max(target_speed, -ramp_cap)
            if "RAMP" not in self.mode_desc:
                self.mode_desc = f"RAMP_{self.mode_desc}"

        # 10c. Unified Dynamic Speed Rate Limiter (Smooth takeoff, no 0 -> 400 RPM step shock)
        if is_recovering:
            # Directional physical resync: only pull toward v_est when the robot has already slowed past the command
            target_v = target_speed
            if target_v < self.rate_limited_speed and v_est < self.rate_limited_speed:
                self.rate_limited_speed = max(target_v, v_est)
            elif target_v > self.rate_limited_speed and v_est > self.rate_limited_speed:
                self.rate_limited_speed = min(target_v, v_est)
            d_limit = 1.35 * min(BRAKE_DECEL_MAX, BRAKE_DECEL_RATIO * self.mu_val * GRAVITY_G)
            speed_err = target_v - self.rate_limited_speed
            max_dv = d_limit * dt
            if abs(speed_err) <= max_dv:
                self.rate_limited_speed = target_v
            else:
                self.rate_limited_speed += math.copysign(max_dv, speed_err)
        else:
            target_v = target_speed
            v_curr = v_est

            # Loose 60 Hz guardrail clamp; 400 Hz ESP32 electronic_diff governs physical traction ramp
            a_limit = 2.2 * min(ACCEL_FWD_MAX, ACCEL_FWD_RATIO * self.mu_val * GRAVITY_G)
            d_limit = 1.5 * min(BRAKE_DECEL_MAX, BRAKE_DECEL_RATIO * self.mu_val * GRAVITY_G)

            # Cap forward/reverse command lead above physical speed without pinning at +0.14 when v_est < 0
            v_curr_fwd = max(0.0, v_curr)
            v_curr_rev = min(0.0, v_curr)
            if self.rate_limited_speed >= 0.0:
                if self.rate_limited_speed > (v_curr_fwd + 0.85):
                    self.rate_limited_speed = v_curr_fwd + 0.85
            else:
                if self.rate_limited_speed < (v_curr_rev - 0.85):
                    self.rate_limited_speed = v_curr_rev - 0.85

            speed_err = target_v - self.rate_limited_speed
            is_accel = (self.rate_limited_speed >= 0.0 and speed_err > 0.0) or (self.rate_limited_speed <= 0.0 and speed_err < 0.0)
            max_dv = (a_limit if is_accel else d_limit) * dt

            if abs(speed_err) <= max_dv:
                self.rate_limited_speed = target_v
            else:
                self.rate_limited_speed += math.copysign(max_dv, speed_err)

        target_speed = self.rate_limited_speed
        self.cached_target_speed = target_speed
        self.cached_target_steer = self.active_steer

        # 10d. Breadcrumb Recording / Freeze State (post-recovery & post-command arbitration)
        if getattr(self.watchdog, "mode", "DISABLED") != "DISABLED":
            is_aww_frozen = (
                is_recovering
                or self.recovery_manager.state != StuckRecoveryState.NORMAL_DRIVE
                or v_est < 0.0
                or target_speed <= 0.0
                or raw_target_speed <= 0.0
                or shield_reverse
                or is_wall_contact
                or is_stall
                or abs(gyro_z_rad_s) > 3.5
                or bool(
                    self.last_aww_status
                    and (self.last_aww_status.pending_wrong_way or self.last_aww_status.is_wrong_way)
                )
            )
            self.watchdog.record_breadcrumb(
                x=nav_pose[0],
                y=nav_pose[1],
                yaw=nav_pose[2],
                v_est=v_est,
                yaw_rate=gyro_z_rad_s,
                now_s=sim_time_s,
                state=self.recovery_manager.state,
                freeze_recording=is_aww_frozen,
                freeze_old_state=is_aww_frozen,
                dt=dt,
            )

        t_total_ms = (time.perf_counter() - t_start) * 1000.0

        # 11. Compile Telemetry Dictionary
        if full_telemetry:
            if self.last_aww_status is not None and getattr(self.watchdog, "mode", "DISABLED") != "DISABLED":
                trail_pts = [
                    [round(float(p.x), 3), round(float(p.y), 3)]
                    for p in list(self.watchdog.breadcrumb_watchdog.breadcrumbs)[::2]
                ][-60:]
                sightline_rays = [
                    {
                        "xv": round(float(r["xv"]), 3),
                        "yv": round(float(r["yv"]), 3),
                        "clear": bool(r["clear"]),
                        "opposing": bool(r["opposing"]),
                        "heading_diff_deg": round(float(r["heading_diff_deg"]), 1),
                    }
                    for r in (self.last_aww_status.sightline_rays or [])
                ]
                raw_tgt_yaw = self.last_aww_status.target_recovery_yaw
                if raw_tgt_yaw is None:
                    raw_tgt_yaw = getattr(self.recovery_manager, "target_corrective_yaw", None)
                aww_target_yaw = round(float(raw_tgt_yaw), 3) if raw_tgt_yaw is not None else 0.0
            else:
                trail_pts = []
                sightline_rays = []
                aww_target_yaw = 0.0

            telemetry = {
                "time_step": self.step_counter,
                "sim_time": round(float(sim_time_s), 3),
                "mode": self.mode_desc,
                "planner": "SL_MPC",
                "status": plan_res.get("status", "OK"),
                "v_cmd": round(float(target_speed), 2),
                "v_tgt": round(float(raw_target_speed), 2),
                "v_est": round(float(v_est), 2),
                "steer_cmd_deg": round(math.degrees(self.active_steer), 1),
                "steer_deg": round(math.degrees(self.active_steer), 1),
                "steer_actual_deg": round(math.degrees(steer_actual_rad), 1),
                "pose": [round(float(nav_pose[0]), 3), round(float(nav_pose[1]), 3), round(float(nav_pose[2]), 3)],
                "proj_anchor": [round(float(dx_3), 3), round(float(dy_3), 3), round(float(delta_yaw_3), 3)],
                "proj_v": round(float(v_3), 2),
                "proj_steer_deg": round(math.degrees(steer_3), 1),
                "f_tof": round(float(f_tof), 3),
                "tof": {k: round(float(v), 3) for k, v in clean_tofs.items()},
                "tof_ranges": [round(float(clean_tofs.get(k, TOF_MAX_RANGE_M)), 3) for k in SENSOR_NAMES],
                "gap": [round(float(xg), 3), round(float(yg), 3), round(float(width), 3)],
                "targets": getattr(self.gap_scout, "last_targets", []),
                "best_line": plan_res.get("best_line", []),
                "candidate_lines": plan_res.get("candidate_lines", []),
                "guide_line": plan_res.get("guide_line", []),
                "corridor_left": plan_res.get("corridor_left", []),
                "corridor_right": plan_res.get("corridor_right", []),
                "shield": plan_res.get("shield", {}),
                "plan_ms": round(float(t_plan_ms), 2),
                "map_ms": round(float(t_map_ms), 2),
                "scout_ms": round(float(t_scout_ms), 2),
                "opt_prim_ms": round(float(plan_res.get("opt_prim_ms", 0.0)), 2),
                "opt_alt_ms": round(float(plan_res.get("opt_alt_ms", 0.0)), 2),
                "osqp_iters": int(plan_res.get("osqp_iters", 0)),
                "osqp_iters_alt": int(plan_res.get("osqp_iters_alt", 0)),
                "iters_prim": int(plan_res.get("osqp_iters", 0)),
                "iters_alt": int(plan_res.get("osqp_iters_alt", 0)),
                "status_prim": plan_res.get("status_prim"),
                "status_alt": plan_res.get("status_alt"),
                "cost_primary": round(float(plan_res["cost_primary"]), 3) if plan_res.get("cost_primary") is not None else None,
                "cost_alt": round(float(plan_res["cost_alt"]), 3) if plan_res.get("cost_alt") is not None else None,
                "depth_credit": round(float(plan_res.get("depth_credit", 0.0)), 3),
                "winner_id": int(plan_res.get("winner_id", 0)),
                "active_target": [round(float(v), 3) for v in self.planner._active_target] if getattr(self.planner, "_active_target", None) is not None else None,
                "step_ms": round(float(t_total_ms), 2),
                "recovery_state": str(self.recovery_manager.state),
                "is_wrong_way": bool(self.last_aww_status.is_wrong_way) if self.last_aww_status else False,
                "target_yaw": aww_target_yaw,
                "trail": trail_pts,
                "sightlines": sightline_rays,
                "is_on_ramp": bool(ramp_res["is_on_ramp"]),
                "ramp_speed_limit": float(ramp_res["speed_limit"]),
                "pitch_deg": float(ramp_res["pitch_deg"]),
                "roll_deg": float(ramp_res["roll_deg"]),
            }
        else:
            telemetry = {
                "time_step": self.step_counter,
                "sim_time": round(float(sim_time_s), 3),
                "mode": self.mode_desc,
                "planner": "SL_MPC",
                "status": plan_res.get("status", "OK"),
                "v_cmd": round(float(target_speed), 2),
                "v_tgt": round(float(raw_target_speed), 2),
                "v_est": round(float(v_est), 2),
                "steer_cmd_deg": round(math.degrees(self.active_steer), 1),
                "steer_deg": round(math.degrees(self.active_steer), 1),
                "steer_actual_deg": round(math.degrees(steer_actual_rad), 1),
                "f_tof": round(float(f_tof), 3),
                "plan_ms": round(float(t_plan_ms), 2),
                "map_ms": round(float(t_map_ms), 2),
                "scout_ms": round(float(t_scout_ms), 2),
                "opt_prim_ms": round(float(plan_res.get("opt_prim_ms", 0.0)), 2),
                "opt_alt_ms": round(float(plan_res.get("opt_alt_ms", 0.0)), 2),
                "osqp_iters": int(plan_res.get("osqp_iters", 0)),
                "osqp_iters_alt": int(plan_res.get("osqp_iters_alt", 0)),
                "iters_prim": int(plan_res.get("osqp_iters", 0)),
                "iters_alt": int(plan_res.get("osqp_iters_alt", 0)),
                "status_prim": plan_res.get("status_prim"),
                "status_alt": plan_res.get("status_alt"),
                "cost_primary": round(float(plan_res["cost_primary"]), 3) if plan_res.get("cost_primary") is not None else None,
                "cost_alt": round(float(plan_res["cost_alt"]), 3) if plan_res.get("cost_alt") is not None else None,
                "depth_credit": round(float(plan_res.get("depth_credit", 0.0)), 3),
                "winner_id": int(plan_res.get("winner_id", 0)),
                "active_target": [round(float(v), 3) for v in self.planner._active_target] if getattr(self.planner, "_active_target", None) is not None else None,
                "step_ms": round(float(t_total_ms), 2),
                "recovery_state": str(self.recovery_manager.state),
                "is_wrong_way": bool(self.last_aww_status.is_wrong_way) if self.last_aww_status else False,
                "is_on_ramp": bool(ramp_res["is_on_ramp"]),
                "ramp_speed_limit": float(ramp_res["speed_limit"]),
                "pitch_deg": float(ramp_res["pitch_deg"]),
                "roll_deg": float(ramp_res["roll_deg"]),
            }

        return (target_speed, self.active_steer, telemetry)


=== FILE: brain/cpp_smart_hybrid_costmap.py ===
"""
================================================================================
  PYTHON CTYPES WRAPPER FOR C++ SMART HYBRID COSTMAP (FRONT-ONLY TOF RIG)
  - 16 Front ToF Sensors (8 Left + 8 Right cross-pointing)
  - 64 Cross-beam circle-circle triangulation pairs
  - Continuous Bayesian angular decay
  - Native AVX2 machine-code compilation
  - Sub-0.10 ms execution time
================================================================================
"""

import ctypes
import os
import sys
from typing import Dict, List, Optional, Tuple
import numpy as np

# Load the compiled shared library (platform-specific extension)
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
DLL_PATH = None
_ext = ".dll" if sys.platform == "win32" else ".so"
candidates = [
    os.path.join(CURR_DIR, f"smart_hybrid_costmap_front{_ext}"),
    os.path.join(os.getcwd(), f"smart_hybrid_costmap_front{_ext}"),
    os.path.abspath(os.path.join(CURR_DIR, "..", "2_pi4", f"smart_hybrid_costmap_front{_ext}")),
    os.path.abspath(os.path.join(CURR_DIR, "..", "..", "..", "..", "..", "2_pi4", f"smart_hybrid_costmap_front{_ext}")),
]
for _cand in candidates:
    if os.path.exists(_cand):
        DLL_PATH = _cand
        break

if not DLL_PATH or not os.path.exists(DLL_PATH):
    raise FileNotFoundError(f"Cannot find compiled DLL/SO at any of {candidates}. Run compile_dll.py or build_so.sh first.")

_lib = ctypes.CDLL(DLL_PATH)

# Configure C-API function signatures
_lib.costmap_create.argtypes = [
    ctypes.c_float, ctypes.c_float,  # x_min, x_max
    ctypes.c_float, ctypes.c_float,  # y_min, y_max
    ctypes.c_float, ctypes.c_float,  # res, r_inflation_m
]
_lib.costmap_create.restype = ctypes.c_void_p

_lib.costmap_destroy.argtypes = [ctypes.c_void_p]
_lib.costmap_destroy.restype = None

if hasattr(_lib, "costmap_clear"):
    _lib.costmap_clear.argtypes = [ctypes.c_void_p]
    _lib.costmap_clear.restype = None

_lib.costmap_process_frame.argtypes = [
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_uint8),
]
_lib.costmap_process_frame.restype = None

_lib.costmap_probe_path.argtypes = [
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.c_int,
]
_lib.costmap_probe_path.restype = ctypes.c_int

_lib.costmap_get_width.argtypes = [ctypes.c_void_p]
_lib.costmap_get_width.restype = ctypes.c_int

_lib.costmap_get_height.argtypes = [ctypes.c_void_p]
_lib.costmap_get_height.restype = ctypes.c_int

if hasattr(_lib, "costmap_set_advanced_params"):
    _lib.costmap_set_advanced_params.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_float,
    ]
    _lib.costmap_set_advanced_params.restype = None

if hasattr(_lib, "costmap_get_unknown_mask"):
    _lib.costmap_get_unknown_mask.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8)]
    _lib.costmap_get_unknown_mask.restype = None

if hasattr(_lib, "costmap_get_perf"):
    _lib.costmap_get_perf.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float)]
    _lib.costmap_get_perf.restype = None

if hasattr(_lib, "costmap_scout_gaps"):
    _lib.costmap_scout_gaps.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),  # ranges_16
        ctypes.c_float, ctypes.c_float,  # max_lookahead_m, steer_current
        ctypes.c_float, ctypes.c_float,  # w_gap_distance, w_gap_width
        ctypes.c_float, ctypes.c_float,  # w_align_steer, w_align_straight
        ctypes.c_float,                  # w_side_clearance
        ctypes.c_float, ctypes.c_float,  # gap_dist_exp_k, gap_dist_thresh_m
        ctypes.c_float, ctypes.c_float,  # alt_min_separation_m, alt_aperture_bias
        ctypes.c_float, ctypes.c_float,  # scout_obs_width_m, scout_block_dist_m
        ctypes.c_float, ctypes.c_float,  # scout_straight_pen_k, scout_straight_exp_k
        ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), # out_targets_x, y, ap
        ctypes.c_int,                    # max_targets
        ctypes.POINTER(ctypes.c_uint8),  # custom_hazard_mask (optional)
        ctypes.POINTER(ctypes.c_float),  # out_targets_clear (optional)
    ]
    _lib.costmap_scout_gaps.restype = ctypes.c_int

if hasattr(_lib, "costmap_scout_gaps_projected"):
    _lib.costmap_scout_gaps_projected.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),  # ranges_16
        ctypes.c_float, ctypes.c_float,  # max_lookahead_m, steer_current
        ctypes.c_float, ctypes.c_float,  # w_gap_distance, w_gap_width
        ctypes.c_float, ctypes.c_float,  # w_align_steer, w_align_straight
        ctypes.c_float,                  # w_side_clearance
        ctypes.c_float, ctypes.c_float,  # gap_dist_exp_k, gap_dist_thresh_m
        ctypes.c_float, ctypes.c_float,  # alt_min_separation_m, alt_aperture_bias
        ctypes.c_float, ctypes.c_float,  # scout_obs_width_m, scout_block_dist_m
        ctypes.c_float, ctypes.c_float,  # scout_straight_pen_k, scout_straight_exp_k
        ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), # out_targets_x, y, ap
        ctypes.c_int,                    # max_targets
        ctypes.POINTER(ctypes.c_uint8),  # custom_hazard_mask (optional)
        ctypes.POINTER(ctypes.c_float),  # out_targets_clear (optional)
        ctypes.c_float, ctypes.c_float, ctypes.c_float, # anchor_x, anchor_y, anchor_yaw
    ]
    _lib.costmap_scout_gaps_projected.restype = ctypes.c_int

if hasattr(_lib, "costmap_probe_path"):
    _lib.costmap_probe_path.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),  # xs
        ctypes.POINTER(ctypes.c_float),  # ys
        ctypes.c_int,                    # count
    ]
    _lib.costmap_probe_path.restype = ctypes.c_int

if hasattr(_lib, "costmap_probe_path_custom"):
    _lib.costmap_probe_path_custom.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),  # xs
        ctypes.POINTER(ctypes.c_float),  # ys
        ctypes.c_int,                    # count
        ctypes.POINTER(ctypes.c_uint8),  # custom_hazard_mask
    ]
    _lib.costmap_probe_path_custom.restype = ctypes.c_int

# Standard sensor names order matching C++ array and only_front_tofs.proto
SENSOR_NAMES = [
    # Front-Left (0..7)
    "tof_front_left_4.1deg",
    "tof_front_left_15.4deg",
    "tof_front_left_26.6deg",
    "tof_front_left_37.9deg",
    "tof_front_left_49.1deg",
    "tof_front_left_60.4deg",
    "tof_front_left_71.6deg",
    "tof_front_left_82.9deg",
    # Front-Right (8..15)
    "tof_front_right_4.1deg",
    "tof_front_right_15.4deg",
    "tof_front_right_26.6deg",
    "tof_front_right_37.9deg",
    "tof_front_right_49.1deg",
    "tof_front_right_60.4deg",
    "tof_front_right_71.6deg",
    "tof_front_right_82.9deg",
]


class CppSmartHybridCostmap:
    """High-performance C++ Smart Hybrid Costmap for 16-channel Front-Only ToF rig."""

    def __init__(
        self,
        grid_bounds: Tuple[float, float, float, float] = (-1.25, 1.25, 0.0, 2.60),
        resolution: float = 0.015,
        r_inflation_m: float = 0.075,
    ):
        self.x_min, self.x_max, self.y_min, self.y_max = grid_bounds
        self.res = float(resolution)
        self.r_inflation_m = float(r_inflation_m)

        self._handle = _lib.costmap_create(
            self.x_min, self.x_max,
            self.y_min, self.y_max,
            self.res, self.r_inflation_m,
        )
        if not self._handle:
            raise RuntimeError("Failed to create C++ costmap instance")

        self.width = _lib.costmap_get_width(self._handle)
        self.height = _lib.costmap_get_height(self._handle)
        self.shape = (self.height, self.width)
        self.total_cells = self.width * self.height

        # Coordinate meshgrid matching C++ index order (row-major: Y is rows, X is cols)
        xs = np.linspace(self.x_min, self.x_max, self.width, dtype=np.float32)
        ys = np.linspace(self.y_min, self.y_max, self.height, dtype=np.float32)
        self.X, self.Y = np.meshgrid(xs, ys)

        # Preallocated C-contiguous NumPy output buffers
        self.prob_map = np.zeros((self.height, self.width), dtype=np.float32)
        self.hazard_mask = np.zeros((self.height, self.width), dtype=np.uint8)
        self.unknown_mask = np.zeros((self.height, self.width), dtype=np.uint8)
        self._prob_ptr = self.prob_map.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        self._haz_ptr = self.hazard_mask.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
        self._unk_ptr = self.unknown_mask.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
        self._lib = _lib
        self.raw_ranges_c = (ctypes.c_float * 16)()

        # Preallocated C-types outputs for fast C++ gap scouting
        self._out_act_x = ctypes.c_float(0.0)
        self._out_act_y = ctypes.c_float(0.0)
        self._out_targets_x = (ctypes.c_float * 4)()
        self._out_targets_y = (ctypes.c_float * 4)()
        self._out_targets_ap = (ctypes.c_float * 4)()
        self._out_targets_clear = (ctypes.c_float * 4)()
        self._perf_buf = (ctypes.c_float * 6)()
        self.last_perf = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def __del__(self):
        if hasattr(self, "_handle") and self._handle:
            _lib.costmap_destroy(self._handle)
            self._handle = None

    def clear(self) -> None:
        """Clears probability, hazard output, and persistence buffers."""
        if hasattr(self, "_handle") and self._handle and hasattr(_lib, "costmap_clear"):
            _lib.costmap_clear(self._handle)
        self.prob_map.fill(0.0)
        self.hazard_mask.fill(0)
        self.unknown_mask.fill(1)

    def set_advanced_params(
        self,
        persistence_frames: int = 0,
        fix_yaw_sign: int = 1,
        use_true_aperture: int = 1,
        w_gap_center: float = 0.832,
    ) -> None:
        """Configures persistence, yaw-sign convention, true gap width, and corridor centering."""
        if hasattr(self, "_handle") and self._handle and hasattr(_lib, "costmap_set_advanced_params"):
            _lib.costmap_set_advanced_params(
                self._handle,
                int(persistence_frames),
                int(bool(fix_yaw_sign)),
                int(bool(use_true_aperture)),
                float(w_gap_center),
            )

    def update(self, readings: any, current_steer_rad: float = 0.0) -> Tuple[np.ndarray, np.ndarray, float]:
        """Convenience method accepting either list/array of 16 ranges or dict."""
        if isinstance(readings, (list, tuple, np.ndarray)):
            r_dict = {name: float(readings[i]) for i, name in enumerate(SENSOR_NAMES) if i < len(readings)}
            return self.process_tof_frame(r_dict)
        elif isinstance(readings, dict):
            return self.process_tof_frame(readings)
        return self.prob_map, self.hazard_mask, 0.0

    @staticmethod
    def _extract_range(readings: Dict[str, float], name: str, idx: int) -> float:
        if name in readings:
            return float(readings[name])
        alt = name.replace("15.4", "15.3").replace("37.9", "37.8").replace("60.4", "60.3")
        if alt in readings:
            return float(readings[alt])
        alt2 = name.replace("15.3", "15.4").replace("37.8", "37.9").replace("60.3", "60.4")
        if alt2 in readings:
            return float(readings[alt2])
        # Legacy 4-quadrant fallback
        legacy_fl = ["tof_fl_0", "tof_fl_1", "tof_fl_2", "tof_fl_3"]
        legacy_fr = ["tof_fr_0", "tof_fr_1", "tof_fr_2", "tof_fr_3"]
        if idx < 8:
            leg_name = legacy_fl[idx // 2]
            if leg_name in readings:
                return float(readings[leg_name])
        else:
            leg_name = legacy_fr[(idx - 8) // 2]
            if leg_name in readings:
                return float(readings[leg_name])
        return 2.40

    def process_tof_frame(
        self, readings: Dict[str, float], noise_pct: float = 0.0, need_prob_map: bool = True, need_unknown_mask: bool = True
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """Drop-in 50/60 Hz Webots controller execution method returning (prob_map, hazard_mask, exec_ms)."""
        import time
        t0 = time.perf_counter()

        for i, name in enumerate(SENSOR_NAMES):
            raw_z = self._extract_range(readings, name, i)
            if noise_pct > 0.0 and raw_z < 2.399:
                noise = np.random.uniform(-noise_pct, noise_pct) * raw_z
                z = float(np.clip(raw_z + noise, 0.001, 2.40))
            else:
                z = raw_z
            self.raw_ranges_c[i] = z

        _lib.costmap_process_frame(
            self._handle,
            self.raw_ranges_c,
            self._prob_ptr if need_prob_map else None,
            self._haz_ptr,
        )
        if need_prob_map or need_unknown_mask:
            if hasattr(_lib, "costmap_get_unknown_mask"):
                _lib.costmap_get_unknown_mask(
                    self._handle,
                    self._unk_ptr,
                )
            elif need_prob_map:
                self.unknown_mask = ((self.prob_map >= 0.38) & (self.prob_map <= 0.65) & (self.hazard_mask == 0)).astype(np.uint8)
            else:
                self.unknown_mask.fill(0)

        if hasattr(_lib, "costmap_get_perf"):
            _lib.costmap_get_perf(self._handle, self._perf_buf)
            self.last_perf = (
                float(self._perf_buf[0]),
                float(self._perf_buf[1]),
                float(self._perf_buf[2]),
                float(self._perf_buf[3]),
                float(self._perf_buf[4]),
                float(self._perf_buf[5]),
            )

        exec_ms = (time.perf_counter() - t0) * 1000.0
        return self.prob_map, self.hazard_mask, exec_ms

    def process_frame(
        self, readings: Dict[str, float]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Runs the complete C++ costmap update."""
        for i, name in enumerate(SENSOR_NAMES):
            self.raw_ranges_c[i] = self._extract_range(readings, name, i)

        _lib.costmap_process_frame(
            self._handle,
            self.raw_ranges_c,
            self.prob_map.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            self.hazard_mask.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
        )
        if hasattr(_lib, "costmap_get_unknown_mask"):
            _lib.costmap_get_unknown_mask(
                self._handle,
                self.unknown_mask.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
            )
        else:
            self.unknown_mask = ((self.prob_map >= 0.38) & (self.prob_map <= 0.65) & (self.hazard_mask == 0)).astype(np.uint8)

        return self.prob_map, self.hazard_mask

    def build_costmap(
        self, readings: Dict[str, float], noise_pct: float = 0.0
    ) -> np.ndarray:
        """Compatibility wrapper for benchmark evaluation suites."""
        if noise_pct > 0.0:
            noisy = {}
            for k, v in readings.items():
                if v < 2.399:
                    noise = np.random.uniform(-noise_pct, noise_pct) * v
                    noisy[k] = float(np.clip(v + noise, 0.001, 2.40))
                else:
                    noisy[k] = v
            p_map, _ = self.process_frame(noisy)
        else:
            p_map, _ = self.process_frame(readings)

        return p_map

    def probe_path(self, xs: np.ndarray, ys: np.ndarray, hazard_mask: Optional[np.ndarray] = None) -> bool:
        """Instant collision check for an entire trajectory in local frame."""
        xs_c = np.ascontiguousarray(xs, dtype=np.float32)
        ys_c = np.ascontiguousarray(ys, dtype=np.float32)
        n = len(xs_c)

        if hazard_mask is not None and hasattr(_lib, "costmap_probe_path_custom"):
            haz_c = np.ascontiguousarray(hazard_mask, dtype=np.uint8)
            haz_ptr = haz_c.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
            res = _lib.costmap_probe_path_custom(
                self._handle,
                xs_c.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                ys_c.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                n,
                haz_ptr,
            )
        else:
            res = _lib.costmap_probe_path(
                self._handle,
                xs_c.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                ys_c.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                n,
            )
        return bool(res == 1)

    def scout_gaps_native(
        self,
        max_lookahead_m: float = 1.40,
        steer_current: float = 0.0,
        w_gap_distance: float = 3.50,
        w_gap_width: float = 2.00,
        w_align_steer: float = 0.80,
        w_align_straight: float = 0.40,
        w_side_clearance: float = 2.80,
        gap_dist_exp_k: float = 0.60,
        gap_dist_thresh_m: float = 0.95,
        gap_progress_margin_m: Optional[float] = None,
        alt_min_separation_m: float = 0.25,
        alt_aperture_bias: float = 1.80,
        scout_obs_width_m: float = 0.22,
        scout_block_dist_m: float = 1.05,
        scout_straight_pen_k: float = 2.50,
        scout_straight_exp_k: float = 1.50,
        top_k: int = 2,
        hazard_mask: Optional[np.ndarray] = None,
        anchor_x: float = 0.0,
        anchor_y: float = 0.0,
        anchor_yaw: float = 0.0,
    ) -> List[Tuple[float, float, float]]:
        """
        Executes C++ native gap scouting across the active hazard_mask.
        Returns: targets_list [(tx, ty, aperture), ...]
        """
        haz_ptr = None
        if hazard_mask is not None and hazard_mask is not self.hazard_mask:
            haz_c = np.ascontiguousarray(hazard_mask, dtype=np.uint8)
            haz_ptr = haz_c.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))

        eff_thresh = float(1.00 + gap_progress_margin_m) if gap_progress_margin_m is not None else float(gap_dist_thresh_m)

        if hasattr(_lib, "costmap_scout_gaps_projected"):
            n_found = _lib.costmap_scout_gaps_projected(
                self._handle,
                self.raw_ranges_c,
                float(max_lookahead_m),
                float(steer_current),
                float(w_gap_distance),
                float(w_gap_width),
                float(w_align_steer),
                float(w_align_straight),
                float(w_side_clearance),
                float(gap_dist_exp_k),
                float(eff_thresh),
                float(alt_min_separation_m),
                float(alt_aperture_bias),
                float(scout_obs_width_m),
                float(scout_block_dist_m),
                float(scout_straight_pen_k),
                float(scout_straight_exp_k),
                self._out_targets_x,
                self._out_targets_y,
                self._out_targets_ap,
                int(top_k),
                haz_ptr,
                self._out_targets_clear,
                float(anchor_x),
                float(anchor_y),
                float(anchor_yaw),
            )
        else:
            n_found = _lib.costmap_scout_gaps(
                self._handle,
                self.raw_ranges_c,
                float(max_lookahead_m),
                float(steer_current),
                float(w_gap_distance),
                float(w_gap_width),
                float(w_align_steer),
                float(w_align_straight),
                float(w_side_clearance),
                float(gap_dist_exp_k),
                float(eff_thresh),
                float(alt_min_separation_m),
                float(alt_aperture_bias),
                float(scout_obs_width_m),
                float(scout_block_dist_m),
                float(scout_straight_pen_k),
                float(scout_straight_exp_k),
                self._out_targets_x,
                self._out_targets_y,
                self._out_targets_ap,
                int(top_k),
                haz_ptr,
                self._out_targets_clear,
            )

        targets = []
        for i in range(max(1, min(n_found, top_k))):
            targets.append((
                float(self._out_targets_x[i]),
                float(self._out_targets_y[i]),
                float(self._out_targets_ap[i]),
                bool(self._out_targets_clear[i] > 0.5),
            ))

        return targets


=== FILE: brain/mcu_telemetry_protocol.h ===
#ifndef MCU_TELEMETRY_PROTOCOL_H
#define MCU_TELEMETRY_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// FOLKRACE LIGHTWEIGHT BINARY UART TELEMETRY PROTOCOL
// =============================================================================
// Designed for ultra-low latency & zero-copy direct memory transmission.
// The MCU doesn't do ANY string formatting or conversion:
//   Serial.write((const uint8_t *)&packet, sizeof(packet));
//
// Transmission overhead:
//   - Chassis Dynamics Packet: ~54 bytes @ 100 Hz = 5.4 KB/s (< 6% of 921.6k Baud)
//   - ToF Distance Packet:    ~76 bytes @ 60 Hz  = 4.5 KB/s (< 5% of 921.6k Baud)
//   - Downlink Motion Command: ~12 bytes @ 60 Hz = 0.7 KB/s (< 1% of 921.6k Baud)
// =============================================================================

#define MCU_UART_SYNC_1          0xAA
#define MCU_UART_SYNC_2          0x55

#define PI_UART_SYNC_1           0x55
#define PI_UART_SYNC_2           0xAA

#define MSG_ID_CHASSIS_TELEM     0x01
#define MSG_ID_TOF_TELEM         0x02
#define MSG_ID_MOTION_CMD        0x10

#define FLAG_OVERRIDE_ACTIVE     (1 << 0)
#define FLAG_BENCH_FAKE_DATA     (1 << 1)
#define FLAG_IMU_ONLINE          (1 << 2)
#define FLAG_STM32_FRONT_ONLINE  (1 << 3)
#define FLAG_STM32_REAR_ONLINE   (1 << 4)

#pragma pack(push, 1)

// =============================================================================
// 1. CHASSIS & DYNAMICS PACKET (ESP32 Main -> Pi 4)
// Total Size: 54 Bytes
// =============================================================================
typedef struct {
    uint8_t  sync[2];             // 0xAA, 0x55
    uint8_t  msg_type;            // 0x01 (MSG_ID_CHASSIS_TELEM)
    uint8_t  payload_len;         // 49 bytes payload
    uint32_t seq;                 // Monotonically increasing sequence number
    uint32_t timestamp_us;        // Microseconds timestamp

    // IMU Attitude & Accelerometer
    int16_t  yaw_deg_x10;         // -180.0 to +180.0 deg (scaled x10, e.g. 125 = 12.5 deg)
    int16_t  pitch_deg_x10;       // -90.0 to +90.0 deg (scaled x10)
    int16_t  roll_deg_x10;        // -180.0 to +180.0 deg (scaled x10)
    int16_t  gyro_z_mrad_s;       // Yaw rate in milliradians/sec (rad/s * 1000)
    int16_t  accel_fwd_cm_s2;     // Forward linear acceleration (cm/s^2, 100 = 1.0 m/s^2)
    int16_t  accel_lat_cm_s2;     // Lateral linear acceleration (cm/s^2)

    // Kinematics & Actuator Setpoints
    int16_t  steer_cmd_deg_x10;   // Commanded center steering angle (deg * 10)
    int16_t  steer_act_deg_x10;   // Actual steering angle feedback (deg * 10)
    uint16_t servo_pulse_us;      // Active PWM pulse width in microseconds (1000..2000)
    int16_t  v_cmd_mm_s;          // Commanded forward speed in mm/s (e.g. 3000 = 3.0 m/s)
    int16_t  v_est_mm_s;          // Fused vehicle velocity in mm/s

    // 4-Wheel Motor Setpoints & Actual Measured RPMs
    int16_t  fl_rpm_cmd;          // Front-Left target RPM
    int16_t  fr_rpm_cmd;          // Front-Right target RPM
    int16_t  rl_rpm_cmd;          // Rear-Left target RPM
    int16_t  rr_rpm_cmd;          // Rear-Right target RPM
    int16_t  fl_rpm_act;          // Front-Left actual measured RPM (from STM32)
    int16_t  fr_rpm_act;          // Front-Right actual measured RPM (from STM32)
    int16_t  rl_rpm_act;          // Rear-Left actual measured RPM (from STM32)
    int16_t  rr_rpm_act;          // Rear-Right actual measured RPM (from STM32)

    // Slip Dynamics & Subsystem Status
    uint8_t  slip_state;          // 0=NORMAL, 1=UNDERSTEER, 2=OVERSTEER, 3=SPIN
    int8_t   slip_ratio_pct;      // Slip ratio in percent (-100% to +100%)
    uint8_t  flags;               // Hardware status bitmask (FLAG_*)
    uint8_t  checksum;            // XOR over all preceding bytes [0 .. 52]
} ChassisTelemetryPacket;


// =============================================================================
// 2. TOF MULTI-ZONE PERCEPTION PACKET (ESP32 ToF Hub -> Pi 4)
// Total Size: 74 Bytes
// =============================================================================
typedef struct {
    uint8_t  sync[2];             // 0xAA, 0x55
    uint8_t  msg_type;            // 0x02 (MSG_ID_TOF_TELEM)
    uint8_t  sensor_id;           // Sensor channel ID (1..5)
    uint32_t seq;                 // Frame counter
    uint32_t timestamp_us;        // Timestamp in microseconds

    // 16 Zones Filtered Readings
    uint16_t dist_mm[16];         // Target distance per zone in mm (0..4000)
    uint8_t  status[16];          // Target detection status (5,6,9 = valid)
    uint8_t  spad[16];            // SPAD count / signal quality per zone
    int8_t   silicon_temp;        // Sensor silicon temperature in deg C
    uint8_t  checksum;            // XOR over bytes [0 .. 72]
} ToFTelemetryPacket;


// =============================================================================
// 3. MOTION COMMAND PACKET (Pi 4 -> ESP32 Main)
// Total Size: 13 Bytes
// =============================================================================
typedef struct {
    uint8_t  sync[2];             // 0x55, 0xAA
    uint8_t  msg_type;            // 0x10 (MSG_ID_MOTION_CMD)
    uint32_t seq;                 // Command sequence counter
    int16_t  target_v_mm_s;       // Target vehicle speed in mm/s (-5000 .. +5000)
    int16_t  target_steer_mrad;   // Target steering angle in milliradians (-650 .. +650)
    uint8_t  run_mode;            // 0=IDLE, 1=DRIVE, 2=EMERGENCY_BRAKE, 3=RECOVERY
    uint8_t  checksum;            // XOR over bytes [0 .. 11]
} MotionCommandPacket;

#pragma pack(pop)

// Fast XOR Checksum Calculator (takes < 20 CPU cycles on Cortex-M4 / ESP32)
static inline uint8_t mcu_calc_checksum(const uint8_t *data, size_t len) {
    uint8_t chk = 0;
    for (size_t i = 0; i < len; i++) {
        chk ^= data[i];
    }
    return chk;
}

#ifdef __cplusplus
}
#endif

#endif // MCU_TELEMETRY_PROTOCOL_H



=== FILE: brain/planner_config.py ===
"""
================================================================================
  AUTONOMOUS TOF PATH PLANNERS: UNIFIED CONFIGURATION & TUNING PARAMETERS
================================================================================
  Single source of truth for:
    1. Physical Vehicle Dimensions, Kinematics & Actuator Limits
    2. Tire Friction & Vehicle Dynamic Traction Envelopes
    3. Sensors, Costmap & Timing (60 Hz ToF & Pi Planning Loop)
    4. Gap Scouting, Aperture Sizing & Selection Weights
    5. Curvilinear Slack-Constrained SL-MPC (Native C++ / OSQP)
    6. Non-Blocking Stuck Detection & Reverse Recovery Manager
    7. Anti-Wrong-Way Watchdog & Early Gap Exit Sentry
================================================================================
"""

import os
import sys
import json
import math
from typing import Any, Dict, List, Optional, Tuple

# Ensure repository root is on sys.path for brain.vehicle_params access
_CURR_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_CURR_DIR, ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from brain.vehicle_params import (
    REAR_AXLE_OFFSET_M,
    FRONT_AXLE_OFFSET_M,
    WHEELBASE,
    TRACK_REAR,
    TRACK_FRONT,
    TRACK_KINGPIN,
    WHEEL_RADIUS,
    VEHICLE_MASS,
    ACKERMANN_PERCENT,
    MAX_SERVO_RAD,
    MAX_STEERING_RAD,
    SERVO_MAX_SPEED_RAD_S,
    MOTOR_DIR,
    V_MAX_WHEEL,
    V_MAX_CRUISE,
    TIRE_MU,
    GRAVITY_G,
    A_MAX_BASE,
    ACCEL_FWD_RATIO,
    BRAKE_DECEL_RATIO,
    LAT_ACCEL_RATIO,
    ACCEL_FWD_MAX,
    BRAKE_DECEL_MAX,
    LAT_ACCEL_MAX,
    R_MIN_TURN,
    V_MIN_CRUISE,
    SLIP_GATE_SPEED_MPS,
    SLIP_THRESH_SPIN_RAD_S,
    TOF_HZ,
    PI_LOOP_HZ,
    DT_PI_LOOP,
    CONTROL_LOOP_FREQ_HZ,
    CONTROL_LOOP_DT_S,
    SIM_TIME_STEP_MS,
    UART_BAUD,
    GRID_X_MAX,
    GRID_X_MIN,
    GRID_Y_MIN,
    GRID_Y_MAX,
    GRID_RES,
    R_INFLATION_M,
)

# ==============================================================================
# ACTIVE PLANNER CONFIGURATION
# ==============================================================================
PLANNER_TYPE: str = "SL_MPC"


# ==============================================================================
# 1. PHYSICAL VEHICLE, SENSORS & TIMING
# (Unified and synchronized from brain/vehicle_config.json via brain.vehicle_params)
# ==============================================================================

# Simulation Physics & Actuator Step Timing
TIME_STEP_MS: float = float(SIM_TIME_STEP_MS)  # Webots basicTimeStep in ms (matches world file: 8.333333 ms = 120.0 Hz)
DT: float = TIME_STEP_MS / 1000.0              # Physics & fast actuator/IMU step duration (0.00833333 s = 1/120 s)
SIM_HZ: float = 1000.0 / TIME_STEP_MS          # Simulation physics, 4WD motors, steering, and IMU rate (120.0 Hz)

PLANNER_DECIMATION: int = 1                    # Run planner every physics step in simulation
TOF_TIME_STEP_MS: float = TIME_STEP_MS * PLANNER_DECIMATION  # Sim ToF sampling interval in ms
DT_PLANNER: float = DT_PI_LOOP                 # Canonical planner cycle duration (0.0166667 s = 60.0 Hz)
PLANNER_HZ: float = PI_LOOP_HZ                 # ToF scan processing & trajectory planning rate (60.0 Hz)

# Realistic Hardware Latency & Lookahead Compensation (Time-Based)
TOF_LATENCY_S: float = 0.01667          # 16.67 ms (nominal 1 frame @ 60 Hz ToF integration & transport delay)
ACTUATOR_LATENCY_S: float = 0.00800     # 8.0 ms (UART transmission + servo physical response lag)
TOTAL_LATENCY_S: float = TOF_LATENCY_S + ACTUATOR_LATENCY_S  # 24.67 ms total latency

# Discrete step counts at current DT (120 Hz in simulation = 8.333 ms):
TOF_LATENCY_STEPS: int = max(1, int(round(TOF_LATENCY_S / DT)))        # 2 steps = 16.67 ms
ACTUATOR_LATENCY_STEPS: int = max(1, int(round(ACTUATOR_LATENCY_S / DT))) # 1 step = 8.33 ms
TOTAL_LATENCY_STEPS: int = TOF_LATENCY_STEPS + ACTUATOR_LATENCY_STEPS      # 3 steps = 25.0 ms

# 3.3 Sensor Device Names & Measurement Limits (16-Channel Front Rig: only_front_tofs.proto)
TOF_SENSOR_NAMES: List[str] = [
    # Front-Left Array (cross-pointing Right, 8 sensors)
    "tof_front_left_4.1deg",
    "tof_front_left_15.4deg",
    "tof_front_left_26.6deg",
    "tof_front_left_37.9deg",
    "tof_front_left_49.1deg",
    "tof_front_left_60.4deg",
    "tof_front_left_71.6deg",
    "tof_front_left_82.9deg",
    # Front-Right Array (cross-pointing Left, 8 sensors)
    "tof_front_right_4.1deg",
    "tof_front_right_15.4deg",
    "tof_front_right_26.6deg",
    "tof_front_right_37.9deg",
    "tof_front_right_49.1deg",
    "tof_front_right_60.4deg",
    "tof_front_right_71.6deg",
    "tof_front_right_82.9deg",
]
TOF_MAX_RANGE_M: float = 2.40          # meters maximum valid ToF range
TOF_MIN_RANGE_M: float = 0.001         # meters minimum valid ToF range

# Wheel Encoders
ENCODER_NAMES: Dict[str, str] = {
    "FL": "encoder FL",
    "FR": "encoder FR",
    "RL": "encoder RL",
    "RR": "encoder RR",
}

# 3.4 Dynamic Moving Obstacles (On Racing Line)
MOVING_OBSTACLE_NAMES: List[str] = ["MOVING_OBSTACLE", "MOVING_OBSTACLE_2", "MOVING_OBSTACLE_3"]
MOVING_OBSTACLE_SPEED_MPS: float = 1.00     # m/s pace along racing line (default: 1.00 m/s)
MOVING_OBSTACLE_DIRECTION: int = 1          # Direction along trajectory: +1 (Forward) or -1 (Reverse)
MOVING_OBSTACLE_Z_ELEVATION: float = 0.075  # meters elevation above track ground


# ==============================================================================
# 4. GAP SCOUTING & SELECTION WEIGHTS
# ==============================================================================

# 4.1 Gap Aperture & Lookahead Constraints
GAP_MIN_WIDTH_M: float = 0.0           # meters minimum passable aperture width
GAP_MAX_LOOKAHEAD_M: float = 2.30      # meters maximum lookahead distance for gap target
GAP_MIN_LOOKAHEAD_M: float = 0.20      # meters minimum lookahead distance

# 4.2 Candidate Evaluation Weights (GA Champion)
# Score = W_DIST * dist_factor + W_WIDTH * (width/max_w) - W_DEV_STEER * |theta - steer| - W_DEV_STRAIGHT * |theta|
WEIGHT_GAP_DISTANCE: float = 7.026     # Preference for longer/further clear gaps (GA Champion)
WEIGHT_GAP_WIDTH: float = 2.047        # Preference for wider open apertures
WEIGHT_ALIGN_STEER: float = 1.681      # Preference for turning toward where servo is already aiming
WEIGHT_ALIGN_STRAIGHT: float = 0.998   # Preference for going straight when openings are equal
WEIGHT_SIDE_CLEARANCE: float = 2.547   # Preference for steering away from close walls / side asymmetry

# 4.3 Exponential Distance Preference (1.0m Walled Track Geometry)
# Sightlines exceeding GAP_DIST_THRESH_M receive exponential distance reward
TRACK_CORRIDOR_WIDTH_M: float = 1.00   # meters physical track corridor width
GAP_DIST_THRESH_M: float = 0.95        # meters threshold distance to engage exponential gap reward
GAP_DIST_EXP_K: float = 0.717          # Exponential growth scaling factor for distance beyond threshold

# 4.4 Dual-Path Candidate Selection & Smart Challenger Arbitration
ALT_MIN_SEPARATION_M: float = 0.052    # meters minimum lateral separation between primary and challenger targets
ALT_APERTURE_BIAS: float = 1.553       # Multiplier on aperture width reward for challenger target
COSTMAP_PERSISTENCE_FRAMES: int = 0    # Body-frame obstacle persistence frames (0 = zero ghost smear)
GAP_USE_TRUE_APERTURE: int = 1         # 1 = real 180-ray angular gap width, 0 = legacy 0.30*dist
WEIGHT_GAP_CENTER: float = 0.832       # Corridor edge-clearance centering weight in gap scout


# ==============================================================================
# 5. CURVILINEAR SLACK-CONSTRAINED SL-MPC (OSQP / C++ NATIVE)
# ==============================================================================

# 5.1 Prediction Horizon & Timestep
MPC_HORIZON_N: int = 30                # Prediction horizon steps (fixed at 30 in native QP sparsity table)
MPC_DT: float = 1.0 / 60.0             # seconds per horizon step (exact 60.0 Hz planning loop: ~0.016667 s)
MPC_MAX_ITER: int = 25                 # Primary OSQP iteration cap (converged, ~0.47 ms mean latency)

# 5.2 Cost Function Weights (Quadratic & Linear OSQP Penalties, GA Champion)
MPC_Q_PROGRESS: float = 23.108         # Weight encouraging forward arc-length progress
MPC_Q_CTE: float = 54.342              # Weight penalizing cross-track deviation from guide line
MPC_Q_HEADING: float = 9.685           # Weight penalizing heading error
MPC_R_STEER: float = 4.944             # Weight penalizing steering angle magnitude
MPC_R_ACCEL: float = 3.507             # Weight penalizing longitudinal acceleration/braking effort
MPC_R_STEER_RATE: float = 12.310       # Weight penalizing steering rate slew
MPC_CENTER_BIAS: float = 0.0           # Corridor centering blend in [0.0, 1.0] (0=guide line, 1=corridor midpoint)

# 5.3 Traction Circle & Dynamic Speeds
MPC_MU_FRICTION: float = TIRE_MU       # Tire friction coefficient governing grip envelope (0.86)
MPC_V_CORNER_FACTOR: float = 0.843     # Friction circle cornering speed scaling factor
MPC_V_CRUISE_MAX: float = V_MAX_CRUISE # m/s top cruising speed ceiling (5.0 m/s)
MPC_V_CRUISE_MIN: float = V_MIN_CRUISE # m/s cornering / crawl speed floor (~1.20 m/s)

# 5.4 Challenger Aliases
MPC_ALT_SWITCH_HYSTERESIS: float = 24.484                     # Cost reduction needed for challenger overthrow
MPC_ALT_MIN_SEPARATION_M: float = ALT_MIN_SEPARATION_M        # Minimum lateral separation between paths (m)
MPC_ALT_APERTURE_BIAS: float = ALT_APERTURE_BIAS              # Challenger aperture width bias multiplier
MPC_W_GAP_CLEAR: float = 10.0                                 # Lookahead depth credit & clear horizon weight (m)

# 5.5 Track-Aware Cornering & Corridor Flow Parameters (Neutral Defaults = Baseline)
MPC_USE_CORRIDOR_FLOW: int = 0                                # 1 = corridor-aware blockage & narrow-gap flow, 0 = baseline
MPC_TRACK_WIDTH_CAP_M: float = 1.25                           # One-sided track-width corridor clamp (m; >=1.25 off)
MPC_CORNER_OUTER_BIAS: float = 0.0                            # Soft outside-entry bias inside free corridor on turns
MPC_INNER_CORNER_MARGIN_M: float = 0.0                        # Inner-apex kinematic turn-radius clearance margin (m)
MPC_BLIND_TURN_KAPPA: float = 0.0                             # Blind 90-deg corner curvature floor (1/m)
MPC_BRAKE_DECEL_SCALE: float = 1.0                            # Backward braking deceleration multiplier
MPC_NARROW_CORRIDOR_V_GAIN: float = 0.0                       # Speed moderation in narrow (<0.48m) corridors


# ==============================================================================
# 6. NON-BLOCKING STUCK & REVERSE RECOVERY MANAGER
# ==============================================================================

# 6.1 Recovery Kinematics & Speed Limits
RECOVERY_STEER_ANGLE_RAD: float = MAX_STEERING_RAD     # radians (~32.22 deg) steering lock during reverse arc
RECOVERY_REVERSE_SPEED_MPS: float = V_MIN_CRUISE       # m/s dynamic cornering speed floor (~1.33 m/s)
RECOVERY_CORRECTIVE_180_SPEED_MPS: float = V_MIN_CRUISE  # m/s full traction-circle U-turn speed (~1.33 m/s)
RECOVERY_SERVO_SLEW_RAD_S: float = SERVO_MAX_SPEED_RAD_S  # rad/s centering slew rate (15.4 rad/s at 8.4V)
RECOVERY_ARC_LOOKAHEAD_M: float = 0.146                # meters length of kinematic escape arcs projected on costmap

# 6.2 Stall Detection Triggers & Failsafe Timeouts
RECOVERY_CLEARANCE_EXIT_M: float = 0.25      # meters (25 cm) minimum forward clearance to conclude reverse
RECOVERY_REVERSE_STROKE_MAX_S: float = 1.0  # seconds maximum reverse stroke failsafe
RECOVERY_TOTAL_TIMEOUT_S: float = 1.2       # seconds maximum total recovery duration before forced exit
RECOVERY_EXIT_MIN_TIME_S: float = 0.0        # seconds minimum recovery stroke time before clearance check
RECOVERY_180_TIMEOUT_S: float = 2.5          # seconds timeout for forward-only corrective 180 turn
RECOVERY_180_TIMEOUT_WITH_REV_S: float = 3.5 # seconds timeout for corrective 180 when reverse support engaged
RECOVERY_180_REV_CLEAR_M: float = 0.38       # meters forward clearance required to exit CORRECTIVE_180[REVERSE]
RECOVERY_180_REV_MIN_S: float = 0.1361       # seconds minimum reverse duration in CORRECTIVE_180[REVERSE]
RECOVERY_180_ALIGN_DEG: float = 30.7898      # degrees target alignment tolerance for CORRECTIVE_180 exit
RECOVERY_180_EARLY_EXIT_TURN_DEG: float = 95.0  # degrees minimum net turn before allowing early exit

# 6.3 Multi-Criteria Exit Conditions
RECOVERY_ANY_TOF_EXIT_M: float = 0.60        # meters (60 cm) minimum clearance on at least one ToF to conclude reverse


# ==============================================================================
# 7. ANTI-WRONG-WAY WATCHDOG & EARLY GAP SENTRY
# ==============================================================================

# 7.1 Watchdog Mode & Breadcrumb History
ANTI_WRONG_WAY_MODE: str = "OPTION_1_BREADCRUMB"
AWW_BREADCRUMB_LOOKBACK_M: float = 8.0         # meters baseline historical path retained
AWW_BREADCRUMB_DURATION_SCALE: float = 0.35    # scale factor for breadcrumb fade (0.35x * 8.0m = 2.8m effective window)
AWW_BREADCRUMB_SPACING_M: float = 0.05         # meters minimum spatial spacing between breadcrumb nodes
AWW_BREADCRUMB_MAX_TIMEOUT_S: float = 6.0      # seconds maximum timeout before breadcrumb history expires
AWW_CONFIRM_TIME_S: float = 0.0543             # seconds continuous confirmation (~3-4 frames @ 60 Hz)
AWW_POST_RECOVERY_COOLDOWN_S: float = 0.3703   # seconds cooldown after CORRECTIVE_180 before re-arming
AWW_MIN_CONFIRMED_BREADCRUMBS: int = 3         # minimum confirmed opposing LOS breadcrumbs required
AWW_MAX_DETECT_DIST_M: float = 2.223           # meters forward LOS search envelope (within 2.4m ToF / 2.6m costmap)
AWW_MAX_LAT_XV_M: float = 0.538                # meters lateral in-corridor bound |lat_bc| for wrong-way LOS matching

# 7.2 Narrow-Cone Early Gap Exit Sentry
AWW_EARLY_EXIT_MIN_GAP_M: float = 1.4          # meters minimum scouted gap distance to qualify as long-path early exit
AWW_EARLY_EXIT_CONE_DEG: float = 30.0          # degrees total narrow cone width (±15 deg) oriented along candidate gap vector
AWW_EARLY_EXIT_OPPOSING_DEG: float = 110.0     # degrees heading divergence threshold for opposing breadcrumbs in gap corridor
AWW_EARLY_EXIT_MIN_BC_DIST_M: float = 0.15     # meters minimum distance from robot to evaluate opposing breadcrumbs in gap corridor (starts 15cm out)
AWW_EARLY_EXIT_CORRIDOR_HALF_WIDTH_M: float = 0.35  # meters max lateral corridor half-width for early-exit capsule
AWW_EARLY_EXIT_MIN_HALF_WIDTH_M: float = 0.08       # meters min close-range lateral half-width for early-exit capsule


# ==============================================================================
# 8. DRIFT & TRACTION CONTROL (REAR OPTICAL FLOW + ACTIVE FRONT STEERING)
# ==============================================================================

ENABLE_OPTICAL_FLOW_SENSOR: bool = False
ENABLE_TRACTION_DRIFT_CONTROL: bool = False

# 8.1 Speed Activation Gates & Hysteresis (m/s)
# Armed at or above dynamic cornering floor V_MIN_CRUISE (~1.204 m/s)
V_ARM_DRIFT_MPS: float = V_MIN_CRUISE          # ~1.204 m/s entry threshold
V_DISARM_CRAWL_MPS: float = 0.60               # Disarm only if speed drops below this AND tires have gripped
V_CRAWL_FLOOR_MPS: float = 0.40                # Floor of the linear fade ramp (zero countersteer at or below)

# 8.2 Slip Angle Thresholds (Radians)
BETA_SCRUB_RAD: float = math.radians(3.5)      # ~0.061 rad (linear tire limit)
BETA_DRIFT_RAD: float = math.radians(7.5)      # ~0.131 rad (saturated drift entry)
BETA_SPIN_RAD: float = math.radians(18.0)      # ~0.314 rad (critical spinout danger)

# 8.3 Active Countersteering & Anti-Snap Parameters
K_BETA: float = 1.10                           # Proportional countersteer gain on rear slip angle
K_YAW_RATE: float = 0.06                       # Damping against measured body yaw rate (rad/s -> rad steer)
K_YAW_DOT: float = 0.015                       # Derivative damping against yaw snap acceleration
K_RECOVERY_RELAX: float = 0.45                 # Damping relaxation when slip angle is collapsing (0.45 * K_BETA)
ANTI_SNAP_HOLD_TIME_S: float = 0.080           # 80 ms stateful hold latch for Schmitt trigger
D_BETA_LP_ALPHA: float = 0.43                  # Low-pass filter smoothing factor on slip rate (~15 Hz cutoff at 125 Hz)

# 8.4 Front-Axle Braking Lateral Reserve
FRONT_LATERAL_RESERVE_MIN: float = 0.40        # Minimum lateral friction fraction reserved for front steering during rear coast

# 8.5 Dynamic MU Scaling (Tire Kinetic Friction Reduction)
ENABLE_DYNAMIC_MU_SCALING: bool = False
MU_SLIDE_RATIO: float = 0.85                   # Minimum friction ratio (1.0 - 0.15 = 0.85 at max slip)
MU_RECOVERY_RATE: float = 2.0                  # Rate at which effective MU recovers back to nominal (1/s)

# 8.6 Continuous Rear Axle Drive Alpha (0.0 = pure ground-roll coasting, 1.0 = full 4WD drive)
ALPHA_SLEW_DROP_RATE: float = 4.0              # Max alpha decay rate (units/s) to prevent friction-circle shock
ALPHA_SLEW_RISE_RATE: float = 2.5              # Max alpha recovery rate (units/s) to prevent breaking traction again


# ==============================================================================
# 9. RAMP DETECTION & SPEED GOVERNOR
# ==============================================================================

ENABLE_RAMP_DETECTION: bool = True
RAMP_SPEED_LIMIT_MPS: float = 2.5       # Cruising speed limit (m/s) when robot is on a ramp
RAMP_ENTER_PITCH_DEG: float = 8.0       # Pitch angle (deg) threshold to trigger ramp mode
RAMP_ENTER_TILT_DEG: float = 9.0        # Combined tilt hypot(roll, pitch) (deg) threshold
RAMP_EXIT_PITCH_DEG: float = 5.0        # Pitch hysteresis exit floor (deg)
RAMP_EXIT_TILT_DEG: float = 6.0         # Tilt hysteresis exit floor (deg)
RAMP_HOLD_TIME_S: float = 0.30          # Hold latch duration (s) across ramp apex / transition


# ==============================================================================
# 10. BEST GENOME AUTO-LOADING (SHARED BY PI 4, BRAIN & WEBOTS PLANNERS)
# ==============================================================================

BEST_GENOME_SOURCE_PATH: Optional[str] = None


def _find_best_genome_path() -> Optional[str]:
    """Searches local and repository paths for best_planner_genome.json."""
    candidates: List[str] = [
        os.path.join(_CURR_DIR, "best_planner_genome.json"),
        os.path.join(os.path.dirname(_CURR_DIR), "best_planner_genome.json"),
    ]
    probe = _CURR_DIR
    for _ in range(6):
        candidates.append(
            os.path.join(probe, "5_webots", "optimization", "ga_planner_optimizer", "best_planner_genome.json")
        )
        parent = os.path.dirname(probe)
        if parent == probe:
            break
        probe = parent

    for path in candidates:
        if os.path.isfile(path):
            return path
    return None


def load_best_genome_genes() -> Tuple[Dict[str, Any], Optional[str]]:
    """Loads the 'genes' dictionary from best_planner_genome.json if present."""
    path = _find_best_genome_path()
    if not path:
        return {}, None
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        genes = data.get("genes") or data.get("best_genome") or {}
        if isinstance(genes, dict) and genes:
            return genes, path
    except Exception as e:
        print(f"[planner_config] Warning: Failed to load {path}: {e}", flush=True)
    return {}, None


def _apply_best_genome_overrides() -> None:
    """Overrides static planner defaults with values from best_planner_genome.json."""
    global BEST_GENOME_SOURCE_PATH
    genes, path = load_best_genome_genes()
    if not genes:
        return
    BEST_GENOME_SOURCE_PATH = path

    mapping = {
        "r_inflation_m": ("R_INFLATION_M", float),
        "target_lookahead_m": ("GAP_MAX_LOOKAHEAD_M", float),
        "w_gap_distance": ("WEIGHT_GAP_DISTANCE", float),
        "w_gap_width": ("WEIGHT_GAP_WIDTH", float),
        "w_gap_steer": ("WEIGHT_ALIGN_STEER", float),
        "w_gap_straight": ("WEIGHT_ALIGN_STRAIGHT", float),
        "w_side_clearance": ("WEIGHT_SIDE_CLEARANCE", float),
        "gap_dist_thresh_m": ("GAP_DIST_THRESH_M", float),
        "gap_dist_exp_k": ("GAP_DIST_EXP_K", float),
        "alt_min_separation_m": ("ALT_MIN_SEPARATION_M", float),
        "alt_aperture_bias": ("ALT_APERTURE_BIAS", float),
        "costmap_persistence": ("COSTMAP_PERSISTENCE_FRAMES", int),
        "use_true_aperture": ("GAP_USE_TRUE_APERTURE", int),
        "w_gap_center": ("WEIGHT_GAP_CENTER", float),
        "mpc_max_iter": ("MPC_MAX_ITER", int),
        "horizon_N": ("MPC_HORIZON_N", int),
        "dt": ("MPC_DT", float),
        "q_progress": ("MPC_Q_PROGRESS", float),
        "q_cte": ("MPC_Q_CTE", float),
        "q_heading": ("MPC_Q_HEADING", float),
        "r_steer": ("MPC_R_STEER", float),
        "r_accel": ("MPC_R_ACCEL", float),
        "r_steer_rate": ("MPC_R_STEER_RATE", float),
        "center_bias": ("MPC_CENTER_BIAS", float),
        "mu_friction": ("MPC_MU_FRICTION", float),
        "v_corner_factor": ("MPC_V_CORNER_FACTOR", float),
        "v_cruise_max": ("MPC_V_CRUISE_MAX", float),
        "v_cruise_min": ("MPC_V_CRUISE_MIN", float),
        "alt_switch_hysteresis": ("MPC_ALT_SWITCH_HYSTERESIS", float),
        "w_gap_clear": ("MPC_W_GAP_CLEAR", float),
        "use_corridor_flow": ("MPC_USE_CORRIDOR_FLOW", int),
        "track_width_cap_m": ("MPC_TRACK_WIDTH_CAP_M", float),
        "corner_outer_bias": ("MPC_CORNER_OUTER_BIAS", float),
        "inner_corner_margin_m": ("MPC_INNER_CORNER_MARGIN_M", float),
        "blind_turn_kappa": ("MPC_BLIND_TURN_KAPPA", float),
        "brake_decel_scale": ("MPC_BRAKE_DECEL_SCALE", float),
        "narrow_corridor_v_gain": ("MPC_NARROW_CORRIDOR_V_GAIN", float),
        "recovery_steer_angle_rad": ("RECOVERY_STEER_ANGLE_RAD", float),
        "recovery_clearance_exit_m": ("RECOVERY_CLEARANCE_EXIT_M", float),
        "recovery_arc_lookahead_m": ("RECOVERY_ARC_LOOKAHEAD_M", float),
    }

    g = globals()
    for gene_key, (const_name, cast) in mapping.items():
        if gene_key in genes:
            g[const_name] = cast(genes[gene_key])

    # Keep alias constants synchronized
    g["MPC_ALT_MIN_SEPARATION_M"] = g["ALT_MIN_SEPARATION_M"]
    g["MPC_ALT_APERTURE_BIAS"] = g["ALT_APERTURE_BIAS"]


_apply_best_genome_overrides()


=== FILE: brain/planner_sl_mpc.py ===
"""
================================================================================
  C++ NATIVE ENGINE: CURVILINEAR SLACK-CONSTRAINED SL-MPC
  - Native AVX2 C++ implementation via ctypes and embedded OSQP solver
  - High-performance replacement for Python prototype
  - Sub-millisecond compute budget (~0.3 ms vs ~26 ms Python)
================================================================================
"""

import ctypes
import math
import os
import sys
import time
from typing import Dict, List, Optional, Tuple, Union
import numpy as np

try:
    from .planner_config import (
        MPC_HORIZON_N,
        MPC_DT,
        MPC_MAX_ITER,
        MPC_Q_PROGRESS,
        MPC_Q_CTE,
        MPC_Q_HEADING,
        MPC_R_STEER,
        MPC_R_ACCEL,
        MPC_R_STEER_RATE,
        MPC_CENTER_BIAS,
        MPC_MU_FRICTION,
        MPC_V_CORNER_FACTOR,
        MPC_V_CRUISE_MAX,
        MPC_V_CRUISE_MIN,
        MPC_ALT_SWITCH_HYSTERESIS,
        MPC_ALT_MIN_SEPARATION_M,
        MPC_ALT_APERTURE_BIAS,
        MPC_W_GAP_CLEAR,
        MPC_USE_CORRIDOR_FLOW,
        MPC_TRACK_WIDTH_CAP_M,
        MPC_CORNER_OUTER_BIAS,
        MPC_INNER_CORNER_MARGIN_M,
        MPC_BLIND_TURN_KAPPA,
        MPC_BRAKE_DECEL_SCALE,
        MPC_NARROW_CORRIDOR_V_GAIN,
        DT_PLANNER,
    )
except ImportError:
    from planner_config import (
        MPC_HORIZON_N,
        MPC_DT,
        MPC_MAX_ITER,
        MPC_Q_PROGRESS,
        MPC_Q_CTE,
        MPC_Q_HEADING,
        MPC_R_STEER,
        MPC_R_ACCEL,
        MPC_R_STEER_RATE,
        MPC_CENTER_BIAS,
        MPC_MU_FRICTION,
        MPC_V_CORNER_FACTOR,
        MPC_V_CRUISE_MAX,
        MPC_V_CRUISE_MIN,
        MPC_ALT_SWITCH_HYSTERESIS,
        MPC_ALT_MIN_SEPARATION_M,
        MPC_ALT_APERTURE_BIAS,
        MPC_W_GAP_CLEAR,
        MPC_USE_CORRIDOR_FLOW,
        MPC_TRACK_WIDTH_CAP_M,
        MPC_CORNER_OUTER_BIAS,
        MPC_INNER_CORNER_MARGIN_M,
        MPC_BLIND_TURN_KAPPA,
        MPC_BRAKE_DECEL_SCALE,
        MPC_NARROW_CORRIDOR_V_GAIN,
        DT_PLANNER,
    )

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
DLL_PATH = None
_ext = ".dll" if sys.platform == "win32" else ".so"
candidates = [
    os.path.join(CURR_DIR, f"curvilinear_sl_mpc{_ext}"),
    os.path.join(os.getcwd(), f"curvilinear_sl_mpc{_ext}"),
    os.path.abspath(os.path.join(CURR_DIR, "..", "2_pi4", f"curvilinear_sl_mpc{_ext}")),
    os.path.abspath(os.path.join(CURR_DIR, "..", "..", "..", "..", "..", "2_pi4", f"curvilinear_sl_mpc{_ext}")),
]
for _cand in candidates:
    if os.path.exists(_cand):
        DLL_PATH = _cand
        break

CPP_AVAILABLE = False
_lib = None

if os.path.exists(DLL_PATH):
    try:
        _lib = ctypes.CDLL(DLL_PATH)
        CPP_AVAILABLE = True
    except Exception as e:
        print(f"[Warning] Failed to load curvilinear_sl_mpc.dll: {e}", flush=True)

PROBE_CALLBACK_TYPE = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.c_int,
)

if CPP_AVAILABLE and _lib is not None:
    _lib.sl_mpc_create.argtypes = [
        ctypes.c_void_p, ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_double,
        ctypes.c_double, ctypes.c_double, ctypes.c_double,
        ctypes.c_double, ctypes.c_double, ctypes.c_double,
        ctypes.c_double, ctypes.c_double,
        ctypes.c_double, ctypes.c_double,
    ]
    _lib.sl_mpc_create.restype = ctypes.c_void_p

    _lib.sl_mpc_destroy.argtypes = [ctypes.c_void_p]
    _lib.sl_mpc_destroy.restype = None

    _lib.sl_mpc_set_costmap.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
    _lib.sl_mpc_set_costmap.restype = None

    if hasattr(_lib, "sl_mpc_set_mu"):
        _lib.sl_mpc_set_mu.argtypes = [ctypes.c_void_p, ctypes.c_double]
        _lib.sl_mpc_set_mu.restype = None

    if hasattr(_lib, "sl_mpc_set_center_bias"):
        _lib.sl_mpc_set_center_bias.argtypes = [ctypes.c_void_p, ctypes.c_double]
        _lib.sl_mpc_set_center_bias.restype = None

    if hasattr(_lib, "sl_mpc_set_racing_params"):
        _lib.sl_mpc_set_racing_params.argtypes = [
            ctypes.c_void_p,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_int,
        ]
        _lib.sl_mpc_set_racing_params.restype = None

    if hasattr(_lib, "sl_mpc_set_context"):
        _lib.sl_mpc_set_context.argtypes = [
            ctypes.c_void_p,
            ctypes.c_double,
            ctypes.c_int,
        ]
        _lib.sl_mpc_set_context.restype = None

    if hasattr(_lib, "sl_mpc_set_advanced_params"):
        _lib.sl_mpc_set_advanced_params.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_double,
        ]
        _lib.sl_mpc_set_advanced_params.restype = None

    if hasattr(_lib, "sl_mpc_get_shield_active"):
        _lib.sl_mpc_get_shield_active.argtypes = [ctypes.c_void_p]
        _lib.sl_mpc_get_shield_active.restype = ctypes.c_int

    if hasattr(_lib, "sl_mpc_reset"):
        _lib.sl_mpc_reset.argtypes = [ctypes.c_void_p]
        _lib.sl_mpc_reset.restype = None

    if hasattr(_lib, "sl_mpc_set_max_iter"):
        _lib.sl_mpc_set_max_iter.argtypes = [ctypes.c_void_p, ctypes.c_int]
        _lib.sl_mpc_set_max_iter.restype = None

    _lib.sl_mpc_plan.argtypes = [
        ctypes.c_void_p,
        ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double,
        ctypes.c_double, ctypes.c_double, ctypes.c_double,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
    ]
    _lib.sl_mpc_plan.restype = ctypes.c_int

    if hasattr(_lib, "sl_mpc_plan_projected"):
        _lib.sl_mpc_plan_projected.argtypes = [
            ctypes.c_void_p,
            ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double,
            ctypes.c_double, ctypes.c_double, ctypes.c_double,
            ctypes.c_double, ctypes.c_double, ctypes.c_double,
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
        ]
        _lib.sl_mpc_plan_projected.restype = ctypes.c_int

    _lib.sl_mpc_get_best_line.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.c_int,
    ]
    _lib.sl_mpc_get_best_line.restype = ctypes.c_int

    _lib.sl_mpc_get_guide_line.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.c_int,
    ]
    _lib.sl_mpc_get_guide_line.restype = ctypes.c_int

    _lib.sl_mpc_get_corridor.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.c_int,
    ]
    _lib.sl_mpc_get_corridor.restype = ctypes.c_int

    _lib.sl_mpc_get_shield_branches.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
    ]
    _lib.sl_mpc_get_shield_branches.restype = ctypes.c_int


class CppCurvilinearSLMPC:
    """
    High-Performance Native C++ Curvilinear SL-MPC Planner.
    100% Drop-In Compatible with CurvilinearSLMPC.
    """

    def __init__(
        self,
        costmap_engine: any = None,
        horizon_N: int = MPC_HORIZON_N,
        dt: float = MPC_DT,
        q_progress: float = MPC_Q_PROGRESS,
        q_cte: float = MPC_Q_CTE,
        q_heading: float = MPC_Q_HEADING,
        r_steer: float = MPC_R_STEER,
        r_accel: float = MPC_R_ACCEL,
        r_steer_rate: float = MPC_R_STEER_RATE,
        mu_friction: float = MPC_MU_FRICTION,
        v_corner_factor: float = MPC_V_CORNER_FACTOR,
        v_cruise_max: float = MPC_V_CRUISE_MAX,
        v_cruise_min: float = MPC_V_CRUISE_MIN,
        alt_switch_hysteresis: float = MPC_ALT_SWITCH_HYSTERESIS,
        alt_min_separation_m: float = MPC_ALT_MIN_SEPARATION_M,
        alt_aperture_bias: float = MPC_ALT_APERTURE_BIAS,
        w_gap_clear: float = MPC_W_GAP_CLEAR,
        center_bias: float = MPC_CENTER_BIAS,
        track_width_cap_m: float = MPC_TRACK_WIDTH_CAP_M,
        corner_outer_bias: float = MPC_CORNER_OUTER_BIAS,
        inner_corner_margin_m: float = MPC_INNER_CORNER_MARGIN_M,
        blind_turn_kappa: float = MPC_BLIND_TURN_KAPPA,
        brake_decel_scale: float = MPC_BRAKE_DECEL_SCALE,
        narrow_corridor_v_gain: float = MPC_NARROW_CORRIDOR_V_GAIN,
        use_corridor_flow: int = MPC_USE_CORRIDOR_FLOW,
        fix_yaw_sign: int = 1,
        mpc_v_nom_min: float = 1.20,
        mpc_max_iter: int = MPC_MAX_ITER,
        **kwargs,
    ):
        self._costmap = costmap_engine
        self.N = int(horizon_N)
        self.dt = float(dt)
        self.q_progress = float(q_progress)
        self.q_cte = float(q_cte)
        self.q_heading = float(q_heading)
        self.r_steer = float(r_steer)
        self.r_accel = float(r_accel)
        self.r_steer_rate = float(r_steer_rate)
        self.mu_friction = float(mu_friction)
        self.v_corner_factor = float(v_corner_factor)
        self.v_cruise_max = float(v_cruise_max)
        self.v_cruise_min = float(v_cruise_min)
        self.alt_switch_hysteresis = float(alt_switch_hysteresis)
        self.alt_min_separation_m = float(alt_min_separation_m)
        self.alt_aperture_bias = float(alt_aperture_bias)
        self.w_gap_clear = float(kwargs.get("alt_lookahead_depth_weight", w_gap_clear))
        self.center_bias = max(0.0, min(1.0, float(center_bias)))
        self.track_width_cap_m = max(0.0, float(track_width_cap_m))
        self.corner_outer_bias = max(0.0, min(1.0, float(corner_outer_bias)))
        self.inner_corner_margin_m = max(0.0, min(0.25, float(inner_corner_margin_m)))
        self.blind_turn_kappa = max(0.0, min(3.0, float(blind_turn_kappa)))
        self.brake_decel_scale = max(0.4, min(1.5, float(brake_decel_scale)))
        self.narrow_corridor_v_gain = max(0.0, min(1.0, float(narrow_corridor_v_gain)))
        self.use_corridor_flow = int(bool(use_corridor_flow))
        self.fix_yaw_sign = int(bool(fix_yaw_sign))
        self.mpc_v_nom_min = max(0.80, min(3.50, float(mpc_v_nom_min)))
        self.mpc_max_iter = max(6, min(35, int(mpc_max_iter)))
        self.dt_actuator = DT_PLANNER
        self.nx = 7
        self.nu = 2
        self._last_winner = 0
        self._active_target: Optional[Tuple[float, float]] = None
        self._last_alt_target: Optional[Tuple[float, float]] = None

        # Preallocated ctypes outputs
        self._out_steer = ctypes.c_double(0.0)
        self._out_speed = ctypes.c_double(0.0)
        self._out_best_cost = ctypes.c_double(0.0)
        self._out_osqp_iters = ctypes.c_int(0)
        self._out_status_code = ctypes.c_int(0)

        self._max_pts = 64
        self._buf_xs1 = (ctypes.c_double * self._max_pts)()
        self._buf_ys1 = (ctypes.c_double * self._max_pts)()
        self._buf_xs2 = (ctypes.c_double * self._max_pts)()
        self._buf_ys2 = (ctypes.c_double * self._max_pts)()
        self._buf_xs3 = (ctypes.c_double * self._max_pts)()
        self._buf_ys3 = (ctypes.c_double * self._max_pts)()
        self._buf_xs4 = (ctypes.c_double * self._max_pts)()
        self._buf_ys4 = (ctypes.c_double * self._max_pts)()

        self._buf_xs5 = (ctypes.c_double * self._max_pts)()
        self._buf_ys5 = (ctypes.c_double * self._max_pts)()
        self._buf_xs6 = (ctypes.c_double * self._max_pts)()
        self._buf_ys6 = (ctypes.c_double * self._max_pts)()

        self._shield_k1_l_cnt = ctypes.c_int(0)
        self._shield_k1_l_safe = ctypes.c_int(0)
        self._shield_k1_r_cnt = ctypes.c_int(0)
        self._shield_k1_r_safe = ctypes.c_int(0)

        self._shield_k2_l_cnt = ctypes.c_int(0)
        self._shield_k2_l_safe = ctypes.c_int(0)
        self._shield_k2_r_cnt = ctypes.c_int(0)
        self._shield_k2_r_safe = ctypes.c_int(0)

        self._shield_straight_cnt = ctypes.c_int(0)
        self._shield_straight_safe = ctypes.c_int(0)
        self._shield_straight_zero_cnt = ctypes.c_int(0)
        self._shield_straight_zero_safe = ctypes.c_int(0)

        self._shield_active = ctypes.c_int(0)
        self._shield_chosen = ctypes.c_int(-1)

        cm_handle, probe_fn_ptr = self._resolve_costmap()

        # Handle 1: Primary Racing Line solver
        self._handle = _lib.sl_mpc_create(
            cm_handle,
            probe_fn_ptr,
            int(self.N),
            float(self.dt),
            float(self.q_progress),
            float(self.q_cte),
            float(self.q_heading),
            float(self.r_steer),
            float(self.r_accel),
            float(self.r_steer_rate),
            float(self.mu_friction),
            float(self.v_corner_factor),
            float(self.v_cruise_max),
            float(self.v_cruise_min),
        )

        # Handle 2: Smart Challenger Alternative Line solver
        self._handle_alt = _lib.sl_mpc_create(
            cm_handle,
            probe_fn_ptr,
            int(self.N),
            float(self.dt),
            float(self.q_progress),
            float(self.q_cte),
            float(self.q_heading),
            float(self.r_steer),
            float(self.r_accel),
            float(self.r_steer_rate),
            float(self.mu_friction),
            float(self.v_corner_factor),
            float(self.v_cruise_max),
            float(self.v_cruise_min),
        )

        if not self._handle:
            raise RuntimeError("Failed to create C++ CurvilinearSLMPCNative instance")

        # Deterministic 60 Hz iteration caps (prevents tail latency overruns on ARM Cortex-A72)
        if hasattr(_lib, "sl_mpc_set_max_iter"):
            if self._handle:
                _lib.sl_mpc_set_max_iter(self._handle, 14)
            if self._handle_alt:
                _lib.sl_mpc_set_max_iter(self._handle_alt, 6)

        self.set_center_bias(self.center_bias)
        self.set_racing_params(
            track_width_cap_m=self.track_width_cap_m,
            corner_outer_bias=self.corner_outer_bias,
            inner_corner_margin_m=self.inner_corner_margin_m,
            blind_turn_kappa=self.blind_turn_kappa,
            brake_decel_scale=self.brake_decel_scale,
            narrow_corridor_v_gain=self.narrow_corridor_v_gain,
            use_corridor_flow=self.use_corridor_flow,
        )
        if hasattr(_lib, "sl_mpc_set_advanced_params"):
            for h in (self._handle, self._handle_alt):
                if h:
                    _lib.sl_mpc_set_advanced_params(
                        h, ctypes.c_int(self.fix_yaw_sign), ctypes.c_double(self.mpc_v_nom_min)
                    )

    def reset_active_target(self) -> None:
        """Resets the currently tracked active corridor/target line and OSQP solver warm-start."""
        self._active_target = None
        self._last_alt_target = None
        self._last_winner = 0
        if _lib is not None and hasattr(_lib, "sl_mpc_reset"):
            if hasattr(self, "_handle") and self._handle:
                _lib.sl_mpc_reset(self._handle)
            if hasattr(self, "_handle_alt") and self._handle_alt:
                _lib.sl_mpc_reset(self._handle_alt)

    def set_dynamic_mu(self, new_mu: float) -> None:
        """
        Dynamically updates the tire friction coefficient governing the grip envelope
        and OSQP solver constraints in the native C++ Curvilinear SL-MPC planner.
        """
        self.mu_friction = float(new_mu)
        if _lib is not None and hasattr(_lib, "sl_mpc_set_mu"):
            if hasattr(self, "_handle") and self._handle:
                _lib.sl_mpc_set_mu(self._handle, ctypes.c_double(self.mu_friction))
            if hasattr(self, "_handle_alt") and self._handle_alt:
                _lib.sl_mpc_set_mu(self._handle_alt, ctypes.c_double(self.mu_friction))

    def set_center_bias(self, center_bias: float) -> None:
        """Updates the corridor centering blend [0.0, 1.0] on both primary and challenger C++ solvers."""
        self.center_bias = max(0.0, min(1.0, float(center_bias)))
        if _lib is not None and hasattr(_lib, "sl_mpc_set_center_bias"):
            if hasattr(self, "_handle") and self._handle:
                _lib.sl_mpc_set_center_bias(self._handle, ctypes.c_double(self.center_bias))
            if hasattr(self, "_handle_alt") and self._handle_alt:
                _lib.sl_mpc_set_center_bias(self._handle_alt, ctypes.c_double(self.center_bias))

    def set_racing_params(
        self,
        track_width_cap_m: float = 0.0,
        corner_outer_bias: float = 0.0,
        inner_corner_margin_m: float = 0.0,
        blind_turn_kappa: float = 0.0,
        brake_decel_scale: float = 1.0,
        narrow_corridor_v_gain: float = 0.0,
        use_corridor_flow: int = 0,
    ) -> None:
        """Updates racing-line and corridor-flow parameters on both primary and challenger solvers."""
        self.track_width_cap_m = max(0.0, float(track_width_cap_m))
        self.corner_outer_bias = max(0.0, min(1.0, float(corner_outer_bias)))
        self.inner_corner_margin_m = max(0.0, min(0.25, float(inner_corner_margin_m)))
        self.blind_turn_kappa = max(0.0, min(3.0, float(blind_turn_kappa)))
        self.brake_decel_scale = max(0.4, min(1.5, float(brake_decel_scale)))
        self.narrow_corridor_v_gain = max(0.0, min(1.0, float(narrow_corridor_v_gain)))
        self.use_corridor_flow = int(bool(use_corridor_flow))
        if _lib is not None and hasattr(_lib, "sl_mpc_set_racing_params"):
            for h in (getattr(self, "_handle", None), getattr(self, "_handle_alt", None)):
                if h:
                    _lib.sl_mpc_set_racing_params(
                        h,
                        ctypes.c_double(self.track_width_cap_m),
                        ctypes.c_double(self.corner_outer_bias),
                        ctypes.c_double(self.inner_corner_margin_m),
                        ctypes.c_double(self.blind_turn_kappa),
                        ctypes.c_double(self.brake_decel_scale),
                        ctypes.c_double(self.narrow_corridor_v_gain),
                        ctypes.c_int(self.use_corridor_flow),
                    )

    @property
    def alt_lookahead_depth_weight(self) -> float:
        return self.w_gap_clear

    @alt_lookahead_depth_weight.setter
    def alt_lookahead_depth_weight(self, val: float):
        self.w_gap_clear = float(val)

    @property
    def costmap(self):
        return getattr(self, "_costmap", None)

    @costmap.setter
    def costmap(self, cm):
        self._costmap = cm
        cm_handle, probe_fn_ptr = self._resolve_costmap()
        if hasattr(self, "_handle") and self._handle:
            _lib.sl_mpc_set_costmap(self._handle, cm_handle, probe_fn_ptr)
        if hasattr(self, "_handle_alt") and self._handle_alt:
            _lib.sl_mpc_set_costmap(self._handle_alt, cm_handle, probe_fn_ptr)

    def _resolve_costmap(self):
        """Extracts native costmap pointer and direct C-ABI probe function pointer."""
        if self._costmap is None:
            return None, None

        if hasattr(self._costmap, "_handle") and self._costmap._handle:
            try:
                import cpp_smart_hybrid_costmap
                lib_cm = cpp_smart_hybrid_costmap._lib
                probe_addr = ctypes.cast(lib_cm.costmap_probe_path, ctypes.c_void_p).value
                return ctypes.c_void_p(self._costmap._handle), ctypes.c_void_p(probe_addr)
            except Exception as e:
                print(f"[Warning] SL-MPC native costmap probe unavailable ({e}); "
                      f"falling back to SLOW Python probe callback", flush=True)

        if hasattr(self._costmap, "probe_path"):
            def _py_probe_cb(h, xs_ptr, ys_ptr, count):
                xs_arr = np.ctypeslib.as_array(xs_ptr, shape=(count,))
                ys_arr = np.ctypeslib.as_array(ys_ptr, shape=(count,))
                return 1 if self._costmap.probe_path(xs_arr, ys_arr) else 0

            self._cb_ref = PROBE_CALLBACK_TYPE(_py_probe_cb)
            fn_addr = ctypes.cast(self._cb_ref, ctypes.c_void_p).value
            return None, ctypes.c_void_p(fn_addr)

        return None, None

    def plan(
        self,
        xg: float,
        yg: float,
        theta_g: float,
        aperture_width: float,
        v_current: float,
        steer_current: float,
        yaw_rate_current: float,
        extra_targets: Optional[List[Tuple[float, float, float]]] = None,
        x0: float = 0.0,
        y0: float = 0.0,
        psi0: float = 0.0,
        t_frame_start: Optional[float] = None,
        frame_budget_ms: float = 15.5,
        pitch_rad: float = 0.0,
        full_telemetry: bool = True,
    ) -> Dict[str, any]:
        """
        Solves the Curvilinear SL-MPC QP using high-speed native C++ core.
        Tracks active line identity, associates perception candidates to Incumbent vs Challenger,
        and arbitrates using alt_switch_hysteresis cost margin.
        """
        # ----------------------------------------------------------------------
        # 0. CANDIDATE ASSOCIATION (INCUMBENT VS CHALLENGER)
        # ----------------------------------------------------------------------
        targets = []
        if extra_targets and len(extra_targets) > 0:
            targets = list(extra_targets)
        elif xg is not None and yg is not None:
            targets = [(float(xg), float(yg), float(aperture_width))]

        if not targets:
            targets = [(0.0, 1.20, 0.40)]

        prim_target = targets[0]
        alt_target = targets[1] if len(targets) >= 2 else None

        # Associate incoming stateless candidates with tracked active target
        if self._active_target is not None and len(targets) >= 2:
            act_x, act_y = self._active_target
            d0 = math.hypot(targets[0][0] - act_x, targets[0][1] - act_y)
            d1 = math.hypot(targets[1][0] - act_x, targets[1][1] - act_y)

            is_clear_0 = bool(targets[0][3]) if len(targets[0]) >= 4 else (math.hypot(targets[0][0], targets[0][1]) >= 2.20)
            is_clear_1 = bool(targets[1][3]) if len(targets[1]) >= 4 else (math.hypot(targets[1][0], targets[1][1]) >= 2.20)

            # If target 1 is closer to our currently followed line, target 1 is Incumbent
            # UNLESS target 1 is blocked by an obstacle while target 0 is completely clear (1.8m opening).
            if d1 < d0 and d1 <= 0.45:
                if (not is_clear_1) and is_clear_0:
                    prim_target = targets[0]
                    alt_target = targets[1]
                else:
                    prim_target = targets[1]
                    alt_target = targets[0]
            elif d0 <= 0.45:
                if (not is_clear_0) and is_clear_1:
                    prim_target = targets[1]
                    alt_target = targets[0]
                else:
                    prim_target = targets[0]
                    alt_target = targets[1]

        tx_prim, ty_prim, ap_prim = prim_target[:3]
        is_clear_prim = bool(prim_target[3]) if len(prim_target) >= 4 else (math.hypot(tx_prim, ty_prim) >= 2.20)
        theta_prim = math.atan2(tx_prim - x0, max(0.20, ty_prim - y0))

        has_opposite_gap = 0
        if alt_target is not None:
            dx_p = tx_prim - x0
            dx_a = alt_target[0] - x0
            is_clear_a = bool(alt_target[3]) if len(alt_target) >= 4 else (math.hypot(alt_target[0], alt_target[1]) >= 1.40)
            if (dx_p * dx_a < -0.005) and (is_clear_a or math.hypot(alt_target[0] - x0, alt_target[1] - y0) >= 1.10):
                has_opposite_gap = 1

        if _lib is not None and hasattr(_lib, "sl_mpc_set_context"):
            if self._handle:
                _lib.sl_mpc_set_context(self._handle, ctypes.c_double(float(pitch_rad)), ctypes.c_int(has_opposite_gap))
            if hasattr(self, "_handle_alt") and self._handle_alt:
                _lib.sl_mpc_set_context(self._handle_alt, ctypes.c_double(float(pitch_rad)), ctypes.c_int(has_opposite_gap))

        t_opt_0 = time.perf_counter()
        t_opt_cpu_0 = time.process_time()

        # ----------------------------------------------------------------------
        # 1. SOLVE INCUMBENT (PRIMARY) RACING LINE
        # ----------------------------------------------------------------------
        # Dynamic Primary OSQP Iteration Scaling based on whole-pipeline budget elapsed so far:
        # Prevents tail-latency overruns when perception (costmap + gap scout) took longer.
        if hasattr(_lib, "sl_mpc_set_max_iter") and self._handle:
            if t_frame_start is not None:
                elapsed_prior_ms = (time.perf_counter() - t_frame_start) * 1000.0
                target_deadline_ms = min(float(frame_budget_ms), 13.2)
                budget_for_prim_ms = target_deadline_ms - elapsed_prior_ms - 0.5  # Reserve 0.5ms for shield
                if budget_for_prim_ms >= 8.0:
                    max_iter_prim = self.mpc_max_iter
                elif budget_for_prim_ms >= 4.5:
                    max_iter_prim = min(10, self.mpc_max_iter)
                else:
                    max_iter_prim = 6
            else:
                max_iter_prim = self.mpc_max_iter
            _lib.sl_mpc_set_max_iter(self._handle, max_iter_prim)

        t_prim_0 = time.perf_counter()
        if hasattr(_lib, "sl_mpc_plan_projected"):
            success_prim = _lib.sl_mpc_plan_projected(
                self._handle,
                float(tx_prim), float(ty_prim), float(theta_prim), float(ap_prim),
                float(v_current), float(steer_current), float(yaw_rate_current),
                float(x0), float(y0), float(psi0),
                ctypes.byref(self._out_steer),
                ctypes.byref(self._out_speed),
                ctypes.byref(self._out_best_cost),
                ctypes.byref(self._out_osqp_iters),
                ctypes.byref(self._out_status_code),
            )
        else:
            success_prim = _lib.sl_mpc_plan(
                self._handle,
                float(tx_prim), float(ty_prim), float(theta_prim), float(ap_prim),
                float(v_current), float(steer_current), float(yaw_rate_current),
                ctypes.byref(self._out_steer),
                ctypes.byref(self._out_speed),
                ctypes.byref(self._out_best_cost),
                ctypes.byref(self._out_osqp_iters),
                ctypes.byref(self._out_status_code),
            )

        status_prim = self._out_status_code.value
        cmd_steer_prim = float(self._out_steer.value)
        cmd_speed_prim = float(self._out_speed.value)
        cost_prim = float(self._out_best_cost.value)
        iters_prim = int(self._out_osqp_iters.value)

        if full_telemetry:
            n_best_prim = _lib.sl_mpc_get_best_line(self._handle, self._buf_xs1, self._buf_ys1, self._max_pts)
            best_line_prim = [[float(self._buf_xs1[i]), float(self._buf_ys1[i])] for i in range(n_best_prim)]
        else:
            best_line_prim = []
        guide_line_prim = []
        t_prim_ms = (time.perf_counter() - t_prim_0) * 1000.0

        # ----------------------------------------------------------------------
        # 2. SOLVE SMART CHALLENGER ALTERNATIVE LINE (if available)
        # ----------------------------------------------------------------------
        has_challenger = False
        status_alt = -1
        cmd_steer_alt = 0.0
        cmd_speed_alt = self.v_cruise_min
        cost_alt = 1e9
        best_line_alt = []
        t_alt_ms = 0.0
        iters_alt = 0

        # Budget-conscious challenger solve with at least 1 iteration:
        # When distinct alternative target exists (has_sep):
        # - Capped iteration budget (8 iters) if remaining frame budget >= 5.5 ms.
        # - Fallback to single iteration (max_iter = 1) when budget is tight (>= 3.2 ms).
        # - Hard skip challenger if remaining budget < 3.2 ms to guarantee strict 60 Hz compliance.
        if alt_target is not None and hasattr(self, "_handle_alt") and self._handle_alt:
            tx_alt, ty_alt, ap_alt = alt_target[:3]
            is_clear_alt = bool(alt_target[3]) if len(alt_target) >= 4 else (math.hypot(tx_alt, ty_alt) >= 2.20)
            min_sep = getattr(self, "alt_min_separation_m", 0.06)
            has_sep = (math.hypot(tx_alt - tx_prim, ty_alt - ty_prim) >= min_sep or abs(tx_alt - tx_prim) >= (min_sep * 0.7))

            if has_sep:
                # Strict Whole-Pipeline Frame Budget Enforcement (Guaranteed < 16.6 ms worst-case):
                # Measure total time spent across the ENTIRE single-frame pipeline so far:
                # (perception + motion latency comp + costmap update + gap scout + primary MPC solve)
                t_check = time.perf_counter()
                elapsed_so_far_ms = ((t_check - t_frame_start) * 1000.0) if t_frame_start is not None else t_prim_ms
                target_deadline_ms = min(float(frame_budget_ms), 13.2)
                budget_remaining_ms = target_deadline_ms - elapsed_so_far_ms

                # Hard budget allocation (calibrated to ARM64 Cortex-A72 execution profile):
                # - High budget (10 iters): ONLY IF budget_remaining >= 6.5 ms
                # - Single iter (1 iter, ~1.2 ms + 0.5 ms shield): IF budget_remaining >= 4.8 ms
                # - Under 4.8 ms: HARD SKIP challenger to guarantee worst-case frame strictly < 16.6 ms!
                if budget_remaining_ms >= 6.5:
                    has_challenger = True
                    max_iter_alt = 10
                elif budget_remaining_ms >= 4.8:
                    has_challenger = True
                    max_iter_alt = 1
                else:
                    has_challenger = False

                if has_challenger:
                    if hasattr(_lib, "sl_mpc_reset"):
                        if (
                            self._last_alt_target is None
                            or math.hypot(tx_alt - self._last_alt_target[0], ty_alt - self._last_alt_target[1]) > 0.45
                        ):
                            _lib.sl_mpc_reset(self._handle_alt)
                    self._last_alt_target = (float(tx_alt), float(ty_alt))
                    if hasattr(_lib, "sl_mpc_set_max_iter"):
                        _lib.sl_mpc_set_max_iter(self._handle_alt, max_iter_alt)
                    t_alt_0 = time.perf_counter()
                    theta_alt = math.atan2(tx_alt - x0, max(0.20, ty_alt - y0))
                    if hasattr(_lib, "sl_mpc_plan_projected"):
                        _lib.sl_mpc_plan_projected(
                            self._handle_alt,
                            float(tx_alt), float(ty_alt), float(theta_alt), float(ap_alt),
                            float(v_current), float(steer_current), float(yaw_rate_current),
                            float(x0), float(y0), float(psi0),
                            ctypes.byref(self._out_steer),
                            ctypes.byref(self._out_speed),
                            ctypes.byref(self._out_best_cost),
                            ctypes.byref(self._out_osqp_iters),
                            ctypes.byref(self._out_status_code),
                        )
                    else:
                        _lib.sl_mpc_plan(
                            self._handle_alt,
                            float(tx_alt), float(ty_alt), float(theta_alt), float(ap_alt),
                            float(v_current), float(steer_current), float(yaw_rate_current),
                            ctypes.byref(self._out_steer),
                            ctypes.byref(self._out_speed),
                            ctypes.byref(self._out_best_cost),
                            ctypes.byref(self._out_osqp_iters),
                            ctypes.byref(self._out_status_code),
                        )
                    status_alt = self._out_status_code.value
                    iters_alt = int(self._out_osqp_iters.value)
                    cmd_steer_alt = float(self._out_steer.value)
                    cmd_speed_alt = float(self._out_speed.value)
                    cost_alt = float(self._out_best_cost.value)
                    if full_telemetry:
                        n_best_alt = _lib.sl_mpc_get_best_line(self._handle_alt, self._buf_xs1, self._buf_ys1, self._max_pts)
                        best_line_alt = [[float(self._buf_xs1[i]), float(self._buf_ys1[i])] for i in range(n_best_alt)]
                    t_alt_ms = (time.perf_counter() - t_alt_0) * 1000.0

        # ----------------------------------------------------------------------
        # 3. ARBITRATION BETWEEN INCUMBENT AND CHALLENGER
        # ----------------------------------------------------------------------
        candidate_lines = (
            ([best_line_prim, best_line_alt] if (has_challenger and best_line_alt) else [best_line_prim])
            if full_telemetry else []
        )

        winner = 0
        depth_credit = 0.0
        if has_challenger:
            primary_ok = (status_prim == 0)
            alt_ok = (status_alt == 0)
            if not primary_ok and alt_ok:
                # Primary line blocked/infeasible, challenger clear: immediate safety switch
                winner = 1
            elif primary_ok and not alt_ok:
                # Challenger infeasible, primary safe: retain primary
                winner = 0
            elif is_clear_alt and not is_clear_prim:
                # Challenger reaches unobstructed open horizon while primary terminates in an obstacle:
                # The clear opening decisively takes precedence over the obstructed path!
                winner = 1
            elif is_clear_prim and not is_clear_alt:
                # Primary is unobstructed while challenger hits an obstacle:
                # Retain primary unobstructed racing line!
                winner = 0
            elif primary_ok and alt_ok:
                # Both are clear or both are obstructed: arbitrate using MPC dynamic cost with hysteresis and depth credit
                d_prim = math.hypot(tx_prim, ty_prim)
                d_alt = math.hypot(tx_alt, ty_alt)
                depth_diff = d_alt - d_prim

                # Base depth credit proportional to lookahead advantage
                depth_credit = self.w_gap_clear * depth_diff

                # Challenger must beat Incumbent by hysteresis cost margin minus lookahead depth credit
                if (cost_alt - depth_credit + self.alt_switch_hysteresis) < cost_prim:
                    winner = 1
                else:
                    winner = 0

        # Update active tracked target to the winning line's target
        if winner == 1 and alt_target is not None:
            self._active_target = (alt_target[0], alt_target[1])
        else:
            self._active_target = (prim_target[0], prim_target[1])
        self._last_winner = winner

        win_handle = self._handle_alt if (winner == 1 and hasattr(self, "_handle_alt") and self._handle_alt) else self._handle

        if full_telemetry:
            n_guide = _lib.sl_mpc_get_guide_line(win_handle, self._buf_xs1, self._buf_ys1, self._max_pts)
            guide_line_win = [[float(self._buf_xs1[i]), float(self._buf_ys1[i])] for i in range(n_guide)] if n_guide > 0 else guide_line_prim

            n_corr = _lib.sl_mpc_get_corridor(win_handle, self._buf_xs1, self._buf_ys1, self._buf_xs2, self._buf_ys2, self._max_pts)
            corridor_left = [[float(self._buf_xs1[i]), float(self._buf_ys1[i])] for i in range(n_corr)]
            corridor_right = [[float(self._buf_xs2[i]), float(self._buf_ys2[i])] for i in range(n_corr)]

            # Extract Emergency Shield evasion branches (Step 0 Left/Right, Step 2 Left/Right, MPC Straight, Zero-Steer Straight)
            t_shield_0 = time.perf_counter()
            _lib.sl_mpc_get_shield_branches(
                win_handle,
                self._buf_xs1, self._buf_ys1, ctypes.byref(self._shield_k1_l_cnt), ctypes.byref(self._shield_k1_l_safe),
                self._buf_xs2, self._buf_ys2, ctypes.byref(self._shield_k1_r_cnt), ctypes.byref(self._shield_k1_r_safe),
                self._buf_xs3, self._buf_ys3, ctypes.byref(self._shield_k2_l_cnt), ctypes.byref(self._shield_k2_l_safe),
                self._buf_xs4, self._buf_ys4, ctypes.byref(self._shield_k2_r_cnt), ctypes.byref(self._shield_k2_r_safe),
                self._buf_xs5, self._buf_ys5, ctypes.byref(self._shield_straight_cnt), ctypes.byref(self._shield_straight_safe),
                self._buf_xs6, self._buf_ys6, ctypes.byref(self._shield_straight_zero_cnt), ctypes.byref(self._shield_straight_zero_safe),
                ctypes.byref(self._shield_active), ctypes.byref(self._shield_chosen)
            )

            n_k1_l = int(self._shield_k1_l_cnt.value)
            n_k1_r = int(self._shield_k1_r_cnt.value)
            n_k2_l = int(self._shield_k2_l_cnt.value)
            n_k2_r = int(self._shield_k2_r_cnt.value)
            n_str  = int(self._shield_straight_cnt.value)
            n_str_zero = int(self._shield_straight_zero_cnt.value)

            k1_left_branch  = [[float(self._buf_xs1[i]), float(self._buf_ys1[i])] for i in range(n_k1_l)] if n_k1_l > 0 else []
            k1_right_branch = [[float(self._buf_xs2[i]), float(self._buf_ys2[i])] for i in range(n_k1_r)] if n_k1_r > 0 else []
            k2_left_branch  = [[float(self._buf_xs3[i]), float(self._buf_ys3[i])] for i in range(n_k2_l)] if n_k2_l > 0 else []
            k2_right_branch = [[float(self._buf_xs4[i]), float(self._buf_ys4[i])] for i in range(n_k2_r)] if n_k2_r > 0 else []
            straight_branch = [[float(self._buf_xs5[i]), float(self._buf_ys5[i])] for i in range(n_str)]  if n_str  > 0 else []
            straight_zero_branch = [[float(self._buf_xs6[i]), float(self._buf_ys6[i])] for i in range(n_str_zero)] if n_str_zero > 0 else []
        else:
            guide_line_win = []
            corridor_left = []
            corridor_right = []
            t_shield_0 = time.perf_counter()
            _lib.sl_mpc_get_shield_branches(
                win_handle,
                None, None, None, None,
                None, None, None, None,
                None, None, None, None,
                None, None, None, None,
                None, None, None, None,
                None, None, None, None,
                ctypes.byref(self._shield_active), ctypes.byref(self._shield_chosen)
            )

        chosen_id = int(self._shield_chosen.value)
        chosen_name = None
        if chosen_id == 0:
            chosen_name = "k2_left"
        elif chosen_id == 1:
            chosen_name = "k2_right"
        elif chosen_id == 2:
            chosen_name = "k1_left"
        elif chosen_id == 3:
            chosen_name = "k1_right"
        elif chosen_id == 4:
            chosen_name = "reverse"  # All evasion paths blocked — recovery manager should reverse

        if full_telemetry:
            shield_info = {
                "active": bool(self._shield_active.value),
                "chosen_id": chosen_id,
                "chosen_name": chosen_name,
                "reverse_suggested": (chosen_id == 4),  # True when all 5-step evasion arcs are blocked
                "k1_left": k1_left_branch,
                "k1_left_safe": bool(self._shield_k1_l_safe.value),
                "k1_right": k1_right_branch,
                "k1_right_safe": bool(self._shield_k1_r_safe.value),
                "k2_left": k2_left_branch,
                "k2_left_safe": bool(self._shield_k2_l_safe.value),
                "k2_right": k2_right_branch,
                "k2_right_safe": bool(self._shield_k2_r_safe.value),
                "straight": straight_branch,
                "straight_safe": bool(self._shield_straight_safe.value),
                "straight_mpc": straight_branch,
                "straight_mpc_safe": bool(self._shield_straight_safe.value),
                "straight_zero": straight_zero_branch,
                "straight_zero_safe": bool(self._shield_straight_zero_safe.value),
            }
        else:
            shield_info = {
                "active": bool(self._shield_active.value),
                "chosen_id": chosen_id,
                "chosen_name": chosen_name,
                "reverse_suggested": (chosen_id == 4),
            }
        t_shield_ms = (time.perf_counter() - t_shield_0) * 1000.0

        # ----------------------------------------------------------------------
        # 4. PACKAGE ACTIVE CONTROL COMMANDS
        # ----------------------------------------------------------------------
        shield_is_active = bool(self._shield_active.value)
        if winner == 1:
            best_line = best_line_alt
            cmd_steer = cmd_steer_alt
            cmd_speed = cmd_speed_alt
            status_desc = "CHALLENGER_OVERTHROW"
            active_cost = cost_alt
        elif status_prim == 0:
            best_line = best_line_prim
            cmd_steer = cmd_steer_prim
            cmd_speed = cmd_speed_prim
            status_desc = "OPTIMAL_OSQP_MPC"
            active_cost = cost_prim
        else:
            best_line = best_line_prim
            cmd_steer = cmd_steer_prim
            cmd_speed = cmd_speed_prim if shield_is_active else self.v_cruise_min
            status_desc = "FALLBACK_SL_MPC"
            active_cost = cost_prim

        t_opt_total_ms = (time.perf_counter() - t_opt_0) * 1000.0
        t_opt_cpu_ms = (time.process_time() - t_opt_cpu_0) * 1000.0

        return {
            "cmd_steer_rad": cmd_steer,
            "cmd_speed_mps": cmd_speed,
            "osqp_status": "solved" if (status_prim == 0 or status_alt == 0) else "fallback",
            "osqp_iters": iters_prim,
            "osqp_iters_alt": iters_alt,
            "status_prim": status_prim,
            "status_alt": status_alt if has_challenger else None,
            "best_cost": active_cost,
            "cost_primary": cost_prim,
            "cost_alt": cost_alt if has_challenger else None,
            "depth_credit": depth_credit,
            "winner_id": winner,
            "status": status_desc,
            "best_line": best_line,
            "guide_line": guide_line_win,
            "candidate_lines": candidate_lines,
            "corridor_left": corridor_left,
            "corridor_right": corridor_right,
            "shield": shield_info,
            "opt_ms": round(t_opt_total_ms, 2),
            "opt_cpu_ms": round(t_opt_cpu_ms, 2),
            "opt_prim_ms": round(t_prim_ms, 2),
            "opt_alt_ms": round(t_alt_ms, 2),
            "opt_shield_ms": round(t_shield_ms, 2),
        }

    def __del__(self):
        if _lib is not None:
            if hasattr(self, "_handle") and self._handle:
                try:
                    _lib.sl_mpc_destroy(self._handle)
                except Exception:
                    pass
                self._handle = None
            if hasattr(self, "_handle_alt") and self._handle_alt:
                try:
                    _lib.sl_mpc_destroy(self._handle_alt)
                except Exception:
                    pass
                self._handle_alt = None


# Primary export alias
CurvilinearSLMPC = CppCurvilinearSLMPC

__all__ = [
    "CurvilinearSLMPC",
    "CppCurvilinearSLMPC",
    "CPP_AVAILABLE",
]


=== FILE: brain/pose_tracker.py ===
"""
================================================================================
  FOLKRACE AUTONOMOUS BRAIN: POSE TRACKER & SE(2) LOOKAHEAD PROJECTION
================================================================================
  Maintains rolling dead-reckoning odometry and executes the 3-step SE(2)
  forward lookahead projection to cancel ToF sensor integration delay
  and actuator execution lag.
================================================================================
"""

import math
import collections
from typing import Tuple, List, Optional

try:
    from .planner_config import (
        WHEELBASE,
        MAX_STEERING_RAD,
        SERVO_MAX_SPEED_RAD_S,
        TOF_LATENCY_STEPS,
        ACTUATOR_LATENCY_STEPS,
        TOF_LATENCY_S,
        ACTUATOR_LATENCY_S,
    )
except ImportError:
    from planner_config import (
        WHEELBASE,
        MAX_STEERING_RAD,
        SERVO_MAX_SPEED_RAD_S,
        TOF_LATENCY_STEPS,
        ACTUATOR_LATENCY_STEPS,
        TOF_LATENCY_S,
        ACTUATOR_LATENCY_S,
    )


def wrap_to_pi(angle: float) -> float:
    """Wraps angle in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class PoseTracker:
    """
    Maintains a rolling ring buffer of fused odometry poses and projects
    the vehicle's future anchor pose in SE(2) coordinates to compensate
    for perception latency and actuator response delay.
    """

    def __init__(
        self,
        wheelbase: float = WHEELBASE,
        tof_latency_steps: int = TOF_LATENCY_STEPS,
        actuator_latency_steps: int = ACTUATOR_LATENCY_STEPS,
        max_steer_rad: float = MAX_STEERING_RAD,
        servo_max_speed: float = SERVO_MAX_SPEED_RAD_S,
        tof_latency_s: float = TOF_LATENCY_S,
        actuator_latency_s: float = ACTUATOR_LATENCY_S,
    ):
        self.wheelbase = wheelbase
        self.tof_latency_s = float(tof_latency_s)
        self.actuator_latency_s = float(actuator_latency_s)
        self.tof_latency_steps = max(1, int(tof_latency_steps))
        self.actuator_latency_steps = max(1, int(actuator_latency_steps))
        self.max_steer_rad = max_steer_rad
        self.servo_max_speed = servo_max_speed

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.total_distance = 0.0
        self._imu_yaw_offset: Optional[float] = None

        # Ring buffer holding past poses (x, y, yaw)
        history_len = self.tof_latency_steps + 1
        self.pose_history = collections.deque(
            [(0.0, 0.0, 0.0) for _ in range(history_len)],
            maxlen=history_len,
        )

        self.proj_anchor = (0.0, 0.0, 0.0)
        self.proj_v = 0.0
        self.proj_steer = 0.0

    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        """Resets the dead-reckoning state and initializes the history buffer."""
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.total_distance = 0.0
        self._imu_yaw_offset = None
        history_len = self.tof_latency_steps + 1
        self.pose_history = collections.deque(
            [(self.x, self.y, self.yaw) for _ in range(history_len)],
            maxlen=history_len,
        )
        self.proj_anchor = (0.0, 0.0, 0.0)
        self.proj_v = 0.0
        self.proj_steer = 0.0

    def update_odometry(
        self,
        v_est: float,
        yaw_rate: float,
        dt: float,
        imu_yaw_rad: Optional[float] = None,
    ) -> Tuple[float, float, float]:
        """
        Integrates forward velocity and yaw rate into local Cartesian coordinates.
        Robot Body convention: +Y is Forward, +X is Right, +Z is Up (turning Left is CCW / positive).
        Uses BNO08x fused yaw directly when available (prevents 60 Hz gyro aliasing during collisions),
        falling back to second-order midpoint gyro integration otherwise.
        Dynamically adapts past pose history buffer to active timestep dt (time-based parity).
        """
        if math.isnan(v_est) or math.isinf(v_est):
            v_est = 0.0
        if math.isnan(yaw_rate) or math.isinf(yaw_rate):
            yaw_rate = 0.0

        # Dynamically adapt history depth based on active dt (time-based lookahead parity)
        if dt > 0.001:
            req_steps = max(1, int(round(self.tof_latency_s / dt)))
            if req_steps != self.tof_latency_steps:
                self.tof_latency_steps = req_steps
                target_len = req_steps + 1
                if self.pose_history.maxlen != target_len:
                    current_poses = list(self.pose_history)
                    while len(current_poses) < target_len:
                        current_poses.insert(0, current_poses[0] if current_poses else (self.x, self.y, self.yaw))
                    self.pose_history = collections.deque(current_poses[-target_len:], maxlen=target_len)

        if imu_yaw_rad is not None and not (math.isnan(imu_yaw_rad) or math.isinf(imu_yaw_rad)):
            if self._imu_yaw_offset is None:
                self._imu_yaw_offset = wrap_to_pi(self.yaw - float(imu_yaw_rad))
            new_yaw = wrap_to_pi(float(imu_yaw_rad) + self._imu_yaw_offset)
            yaw_mid = wrap_to_pi(self.yaw + 0.5 * wrap_to_pi(new_yaw - self.yaw))
        else:
            yaw_mid = self.yaw + 0.5 * yaw_rate * dt
            new_yaw = wrap_to_pi(self.yaw + yaw_rate * dt)

        dx = -v_est * math.sin(yaw_mid) * dt
        dy =  v_est * math.cos(yaw_mid) * dt

        self.x += dx
        self.y += dy
        self.yaw = new_yaw
        self.total_distance += abs(v_est) * dt

        self.pose_history.append((self.x, self.y, self.yaw))
        return (self.x, self.y, self.yaw)

    def compute_projected_anchor(
        self,
        v_current: float,
        steer_current: float,
        steer_target: float,
        dt: float,
        is_reversing: bool = False,
    ) -> Tuple[Tuple[float, float, float], float, float]:
        """
        Computes 3-step composite SE(2) lookahead projection in sensor frame F_sens:
          - Past Motion: step k-2 -> step k (ToF integration latency, ~16 ms)
          - Future Motion: step k -> step k+1 (actuator execution lag, ~8 ms)

        Returns:
          ((dx_anchor, dy_anchor, delta_yaw_anchor), projected_v, projected_steer)
        """
        # 1. Past Motion in sensor coordinate frame
        x_past, y_past, yaw_past = self.pose_history[0]
        x_curr, y_curr, yaw_curr = self.pose_history[-1]

        delta_yaw_past = wrap_to_pi(yaw_curr - yaw_past)
        dx_w = x_curr - x_past
        dy_w = y_curr - y_past
        cos_p = math.cos(yaw_past)
        sin_p = math.sin(yaw_past)
        dx_past = cos_p * dx_w + sin_p * dy_w
        dy_past = -sin_p * dx_w + cos_p * dy_w

        # 2. Future Motion via single-track bicycle model (time-based actuator latency)
        v_k = v_current if is_reversing else max(0.0, v_current)
        dt_act = self.actuator_latency_s if hasattr(self, "actuator_latency_s") else (self.actuator_latency_steps * dt)
        delta_yaw_future = (v_k / self.wheelbase) * math.tan(steer_current) * dt_act
        yaw_future_mid = 0.5 * delta_yaw_future
        dx_future = -v_k * math.sin(yaw_future_mid) * dt_act
        dy_future =  v_k * math.cos(yaw_future_mid) * dt_act

        # 3. Composite 3-step projected anchor pose in F_sens
        delta_yaw_3 = wrap_to_pi(delta_yaw_past + delta_yaw_future)
        cos_dp = math.cos(delta_yaw_past)
        sin_dp = math.sin(delta_yaw_past)
        dx_3 = dx_past + (cos_dp * dx_future - sin_dp * dy_future)
        dy_3 = dy_past + (sin_dp * dx_future + cos_dp * dy_future)

        # 4. Projected actuator state (speed v_3 and steering delta_3)
        # When unwinding out of a corner, project across full round-trip latency so planner
        # doesn't throttle exit acceleration on stale steering feedback.
        is_unwinding = (abs(steer_target) < abs(steer_current)) or (steer_target * steer_current < 0.0)
        dt_steer = max(dt_act, dt + dt_act) if is_unwinding else dt_act
        max_steer_delta = self.servo_max_speed * dt_steer
        delta_steer = max(-max_steer_delta, min(max_steer_delta, steer_target - steer_current))
        steer_3 = max(-self.max_steer_rad, min(self.max_steer_rad, steer_current + delta_steer))

        self.proj_anchor = (dx_3, dy_3, delta_yaw_3)
        self.proj_v = v_k
        self.proj_steer = steer_3

        return (self.proj_anchor, self.proj_v, self.proj_steer)



=== FILE: brain/ramp_detector.py ===
"""
================================================================================
  RAMP DETECTION & SPEED GOVERNOR MODULE
================================================================================
  Detects when the vehicle is ascending, traversing, or descending a ramp /
  elevation incline based on IMU pitch and tilt angles.
  When a ramp is detected, enforces a safe cruising speed limit (e.g. 2.0 m/s)
  with hysteresis and an apex transition hold latch.
================================================================================
"""

import math
from typing import Dict, Any

try:
    from .planner_config import (
        ENABLE_RAMP_DETECTION,
        RAMP_SPEED_LIMIT_MPS,
        RAMP_ENTER_PITCH_DEG,
        RAMP_ENTER_TILT_DEG,
        RAMP_EXIT_PITCH_DEG,
        RAMP_EXIT_TILT_DEG,
        RAMP_HOLD_TIME_S,
    )
except ImportError:
    from planner_config import (
        ENABLE_RAMP_DETECTION,
        RAMP_SPEED_LIMIT_MPS,
        RAMP_ENTER_PITCH_DEG,
        RAMP_ENTER_TILT_DEG,
        RAMP_EXIT_PITCH_DEG,
        RAMP_EXIT_TILT_DEG,
        RAMP_HOLD_TIME_S,
    )


class RampDetector:
    """
    Ramp detection and speed governance based on filtered IMU attitude.
    Replaces rollover detection to prevent unwanted countersteering and power cut
    on ramps, instead capping speed to a safe traversing limit.
    """

    def __init__(
        self,
        enabled: bool = ENABLE_RAMP_DETECTION,
        speed_limit_mps: float = RAMP_SPEED_LIMIT_MPS,
        enter_pitch_deg: float = RAMP_ENTER_PITCH_DEG,
        enter_tilt_deg: float = RAMP_ENTER_TILT_DEG,
        exit_pitch_deg: float = RAMP_EXIT_PITCH_DEG,
        exit_tilt_deg: float = RAMP_EXIT_TILT_DEG,
        hold_time_s: float = RAMP_HOLD_TIME_S,
        filter_alpha: float = 0.25,
    ):
        self.enabled = enabled
        self.speed_limit_mps = speed_limit_mps
        self.enter_pitch_deg = enter_pitch_deg
        self.enter_tilt_deg = enter_tilt_deg
        self.exit_pitch_deg = exit_pitch_deg
        self.exit_tilt_deg = exit_tilt_deg
        self.hold_time_s = hold_time_s
        self.filter_alpha = filter_alpha

        # Internal state
        self.is_on_ramp: bool = False
        self.hold_timer: float = 0.0
        self.filtered_pitch: float = 0.0
        self.filtered_roll: float = 0.0
        self._initialized: bool = False

    def reset(self) -> None:
        """Resets detector state and timers."""
        self.is_on_ramp = False
        self.hold_timer = 0.0
        self.filtered_pitch = 0.0
        self.filtered_roll = 0.0
        self._initialized = False

    def update(
        self,
        pitch_rad: float,
        roll_rad: float,
        dt: float = 0.008,
    ) -> Dict[str, Any]:
        """
        Evaluates current attitude for ramp traversal.

        Args:
            pitch_rad: Vehicle pitch angle in radians (+ = nose up, - = nose down)
            roll_rad: Vehicle roll angle in radians (+ = lean right, - = lean left)
            dt: Timestep duration (seconds)

        Returns:
            Dictionary with ramp detection state, speed limit, and attitude angles.
        """
        if not self._initialized:
            self.filtered_pitch = pitch_rad
            self.filtered_roll = roll_rad
            self._initialized = True
        else:
            # Low-pass filter to reject high-frequency suspension vibration / bumps
            a = self.filter_alpha
            self.filtered_pitch = a * pitch_rad + (1.0 - a) * self.filtered_pitch
            self.filtered_roll = a * roll_rad + (1.0 - a) * self.filtered_roll

        pitch_deg = abs(math.degrees(self.filtered_pitch))
        roll_deg = abs(math.degrees(self.filtered_roll))
        tilt_deg = math.degrees(math.hypot(self.filtered_pitch, self.filtered_roll))

        if not self.enabled:
            return {
                "is_on_ramp": False,
                "speed_limit": self.speed_limit_mps,
                "pitch_deg": round(pitch_deg, 2),
                "roll_deg": round(roll_deg, 2),
                "tilt_deg": round(tilt_deg, 2),
                "hold_timer": 0.0,
            }

        # Check entry trigger: pitch or tilt exceeds entry threshold
        is_incline = (pitch_deg >= self.enter_pitch_deg) or (tilt_deg >= self.enter_tilt_deg)

        if is_incline:
            self.is_on_ramp = True
            self.hold_timer = self.hold_time_s
        elif self.hold_timer > 0.0:
            # Latch hold: maintain ramp mode while transitioning across flat apex or leaving ramp
            self.hold_timer = max(0.0, self.hold_timer - dt)
            if self.hold_timer <= 0.0:
                # Check exit hysteresis
                if (pitch_deg < self.exit_pitch_deg) and (tilt_deg < self.exit_tilt_deg):
                    self.is_on_ramp = False
                else:
                    # Still above exit hysteresis floor, re-arm hold timer
                    self.hold_timer = self.hold_time_s
        else:
            # Check exit threshold
            if (pitch_deg < self.exit_pitch_deg) and (tilt_deg < self.exit_tilt_deg):
                self.is_on_ramp = False

        return {
            "is_on_ramp": self.is_on_ramp,
            "speed_limit": self.speed_limit_mps,
            "pitch_deg": round(pitch_deg, 2),
            "roll_deg": round(roll_deg, 2),
            "tilt_deg": round(tilt_deg, 2),
            "hold_timer": round(self.hold_timer, 3),
        }


# Backwards compatibility alias
AntiRolloverController = RampDetector


=== FILE: brain/stuck_recovery_manager.py ===
"""
================================================================================
  INTELLIGENT AUTONOMOUS REVERSE, DECEL & CENTER RECOVERY CONTROLLER
  Streamlined 2-Phase Recovery:
  1. PIVOT_REVERSE: Reverse arc with escape steering until ToF clearance opens up.
  2. DECEL_AND_CENTER: Decelerates to 0 m/s concurrently while slewing servo back
     to 0 radians at physical servo speed (PTK 9497 8.4V @ 15.4 rad/s).
  3. Clean Handshake to Curvilinear SL-MPC with zero residual steering error.
================================================================================
"""

import math
from typing import Any, Dict, NamedTuple, Optional, Tuple
import numpy as np

try:
    from .planner_config import (
        WHEELBASE,
        MAX_STEERING_RAD,
        GRAVITY_G,
        MPC_MU_FRICTION,
        RECOVERY_REVERSE_SPEED_MPS,
        RECOVERY_CORRECTIVE_180_SPEED_MPS,
        RECOVERY_STEER_ANGLE_RAD,
        RECOVERY_REVERSE_STROKE_MAX_S,
        RECOVERY_TOTAL_TIMEOUT_S,
        RECOVERY_180_TIMEOUT_S,
        RECOVERY_180_TIMEOUT_WITH_REV_S,
        RECOVERY_180_REV_CLEAR_M,
        RECOVERY_180_REV_MIN_S,
        RECOVERY_180_ALIGN_DEG,
        RECOVERY_180_EARLY_EXIT_TURN_DEG,
        RECOVERY_CLEARANCE_EXIT_M,
        RECOVERY_ANY_TOF_EXIT_M,
        RECOVERY_EXIT_MIN_TIME_S,
        RECOVERY_SERVO_SLEW_RAD_S,
        RECOVERY_ARC_LOOKAHEAD_M,
        TOF_MAX_RANGE_M,
        V_MIN_CRUISE,
    )
except ImportError:
    from planner_config import (
        WHEELBASE,
        MAX_STEERING_RAD,
        GRAVITY_G,
        MPC_MU_FRICTION,
        RECOVERY_REVERSE_SPEED_MPS,
        RECOVERY_CORRECTIVE_180_SPEED_MPS,
        RECOVERY_STEER_ANGLE_RAD,
        RECOVERY_REVERSE_STROKE_MAX_S,
        RECOVERY_TOTAL_TIMEOUT_S,
        RECOVERY_180_TIMEOUT_S,
        RECOVERY_180_TIMEOUT_WITH_REV_S,
        RECOVERY_180_REV_CLEAR_M,
        RECOVERY_180_REV_MIN_S,
        RECOVERY_180_ALIGN_DEG,
        RECOVERY_180_EARLY_EXIT_TURN_DEG,
        RECOVERY_CLEARANCE_EXIT_M,
        RECOVERY_ANY_TOF_EXIT_M,
        RECOVERY_EXIT_MIN_TIME_S,
        RECOVERY_SERVO_SLEW_RAD_S,
        RECOVERY_ARC_LOOKAHEAD_M,
        TOF_MAX_RANGE_M,
        V_MIN_CRUISE,
    )


def wrap_to_pi(angle_rad: float) -> float:
    """Wraps an angle into [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


class TofSummary(NamedTuple):
    min_l: float
    min_r: float
    avg_l: float
    avg_r: float
    d_straight: float
    max_any: float
    count: int


class StuckRecoveryState:
    NORMAL_DRIVE = "DRIVE"
    PIVOT_REVERSE = "PIVOT_REVERSE"
    DECEL_AND_CENTER = "DECEL_AND_CENTER"
    CORRECTIVE_180 = "CORRECTIVE_180"
    WIGGLE_ESCAPE = "WIGGLE_ESCAPE"


class StuckRecoveryManager:
    """
    Intelligent Autonomous Recovery Controller:
    - Evaluates forward kinematic escape arcs (Left/Right max steering lock & Center)
      projected directly onto the inflated costmap (hazard_mask).
    - Reverses away from obstacle along escape curvature until front clearance opens up.
    - Decelerates reverse motion smoothly to 0 m/s while concurrently centering the front servo.
    - Hands control cleanly back to the path planner with 0 steering offset and 0 yaw rate shock.
    """

    def __init__(
        self,
        reverse_speed_m_s: float = RECOVERY_REVERSE_SPEED_MPS,
        forward_speed_m_s: float = V_MIN_CRUISE,
        corrective_forward_speed_m_s: float = RECOVERY_CORRECTIVE_180_SPEED_MPS,
        steer_angle_rad: float = RECOVERY_STEER_ANGLE_RAD,
        clearance_exit_m: float = RECOVERY_CLEARANCE_EXIT_M,
        any_tof_exit_m: float = RECOVERY_ANY_TOF_EXIT_M,
        servo_slew_rad_s: float = RECOVERY_SERVO_SLEW_RAD_S,
        wheelbase_m: float = WHEELBASE,
        max_steer_rad: float = MAX_STEERING_RAD,
        arc_lookahead_m: float = RECOVERY_ARC_LOOKAHEAD_M,
        mu_friction: float = MPC_MU_FRICTION,
        **kwargs,
    ):
        self.state = StuckRecoveryState.NORMAL_DRIVE
        self.reverse_speed = float(reverse_speed_m_s)
        self.forward_speed = float(forward_speed_m_s)
        self.corrective_forward_speed = float(corrective_forward_speed_m_s)
        self.steer_angle = float(steer_angle_rad)
        self.clearance_exit_m = float(clearance_exit_m)
        self.any_tof_exit_m = float(any_tof_exit_m)
        self.servo_slew_rad_s = float(servo_slew_rad_s)
        self.mu_friction = float(mu_friction)

        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        self.preferred_yaw_sign = 1.0

        # State tracking for smooth ramping
        self.current_cmd_v = 0.0
        self.current_cmd_steer = 0.0

        # Anti-Wrong-Way CORRECTIVE_180 tracking & Reverse Support
        self.target_corrective_yaw: Optional[float] = None
        self.last_corrective_yaw: float = 0.0
        self.corrective_accumulated_turn: float = 0.0
        self.corrective_net_turn: float = 0.0
        self.corrective_used_reverse: bool = False
        self.corrective_substate: str = "FORWARD"  # "FORWARD" or "REVERSE"
        self.corrective_steer_dir: float = 1.0     # +1.0 (Left) or -1.0 (Right) towards more open side
        self.corrective_reverse_timer: float = 0.0
        self.forward_grace_timer: float = 0.0
        self._rev_best_d_straight: float = 0.0
        self._rev_stall_timer: float = 0.0
        self._rev_phys_stall_timer: float = 0.0
        self._rev_stroke_elapsed: float = 0.0
        self._wiggle_step: int = 0
        self._wiggle_phase_timer: float = 0.0
        self._wiggle_stall_timer: float = 0.0
        self._wiggle_steer_sign: float = 1.0
        self._wiggle_max_steps: int = 6
        self._wiggle_phase_duration_s: float = 0.30
        self._wiggle_attempts: int = 0
        self.corrective_rev_clear_m: float = float(kwargs.get("corrective_rev_clear_m", RECOVERY_180_REV_CLEAR_M))
        self.corrective_rev_min_s: float = float(kwargs.get("corrective_rev_min_s", RECOVERY_180_REV_MIN_S))
        self.corrective_align_rad: float = math.radians(float(kwargs.get("corrective_align_deg", RECOVERY_180_ALIGN_DEG)))
        self.corrective_early_exit_turn_rad: float = math.radians(float(kwargs.get("corrective_early_exit_turn_deg", RECOVERY_180_EARLY_EXIT_TURN_DEG)))

        # --- KINEMATIC ESCAPE ARCS PRECOMPUTATION ---
        self.wheelbase = float(wheelbase_m)
        self.max_steer_rad = float(max_steer_rad)
        self.r_min_turn = self.wheelbase / max(1e-3, math.tan(self.max_steer_rad))
        self.origin_y = 0.0   # Robot center origin (0.0, 0.0)
        self.bumper_y = 0.095 # Front bumper mount Y in vehicle frame

        # State tracking for pinned evaluation
        self.is_left_pinned: bool = False
        self.is_right_pinned: bool = False

        self.arc_lookahead_m = max(0.65, float(arc_lookahead_m))
        self.arc_step_ds = 0.015  # 15mm steps
        n_pts = max(3, int(self.arc_lookahead_m / self.arc_step_ds))
        self._arc_s = np.linspace(0.0, self.arc_lookahead_m, n_pts, dtype=np.float32)

        # Left Turn Arc (full left lock: sweeps left -X from robot center)
        self._arc_pts_left = np.zeros((n_pts, 2), dtype=np.float32)
        self._arc_pts_left[:, 0] = -self.r_min_turn * (1.0 - np.cos(self._arc_s / self.r_min_turn))
        self._arc_pts_left[:, 1] = self.origin_y + self.r_min_turn * np.sin(self._arc_s / self.r_min_turn)

        # Right Turn Arc (full right lock: sweeps right +X from robot center)
        self._arc_pts_right = np.zeros((n_pts, 2), dtype=np.float32)
        self._arc_pts_right[:, 0] = self.r_min_turn * (1.0 - np.cos(self._arc_s / self.r_min_turn))
        self._arc_pts_right[:, 1] = self.origin_y + self.r_min_turn * np.sin(self._arc_s / self.r_min_turn)

        # Center Straight Sightline (from robot center)
        self._arc_pts_center = np.zeros((n_pts, 2), dtype=np.float32)
        self._arc_pts_center[:, 0] = 0.0
        self._arc_pts_center[:, 1] = self.origin_y + self._arc_s

    # --- STATE TRANSITION & COMMAND HELPERS ---
    def reset(self) -> None:
        """Resets all recovery state machine timers, flags, and corrective tracking."""
        self.state = StuckRecoveryState.NORMAL_DRIVE
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        self.preferred_yaw_sign = 1.0
        self.current_cmd_v = 0.0
        self.current_cmd_steer = 0.0

        self.target_corrective_yaw = None
        self.last_corrective_yaw = 0.0
        self.corrective_accumulated_turn = 0.0
        self.corrective_net_turn = 0.0
        self.corrective_used_reverse = False
        self.corrective_substate = "FORWARD"
        self.corrective_steer_dir = 1.0
        self.corrective_reverse_timer = 0.0
        self.forward_grace_timer = 0.0
        self._rev_best_d_straight = 0.0
        self._rev_stall_timer = 0.0
        self._rev_phys_stall_timer = 0.0
        self._rev_stroke_elapsed = 0.0
        self._wiggle_step = 0
        self._wiggle_phase_timer = 0.0
        self._wiggle_stall_timer = 0.0
        self._wiggle_steer_sign = 1.0
        self._wiggle_attempts = 0

        self.is_left_pinned = False
        self.is_right_pinned = False

    def _finish_recovery(self) -> Tuple[bool, float, float]:
        self.state = StuckRecoveryState.NORMAL_DRIVE
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        self.current_cmd_v = 0.0
        self.current_cmd_steer = 0.0
        return False, 0.0, 0.0

    def finish_recovery_early(self) -> Tuple[bool, float, float]:
        """Immediately terminates active recovery maneuvers when a safe early exit path is scouted."""
        return self._finish_recovery()

    def _enter_decel_and_center(self) -> Tuple[bool, float, float]:
        self.state = StuckRecoveryState.DECEL_AND_CENTER
        self.stroke_timer = 0.0
        self._rev_phys_stall_timer = 0.0
        self._rev_stroke_elapsed = 0.0
        self.current_cmd_v = 0.0
        self.current_cmd_steer = 0.0
        return True, 0.0, 0.0

    def _wiggle_cmd_for_step(self, step: int, front_dist_m: float = 0.25) -> Tuple[float, float]:
        """Returns (velocity, steer) for the given wiggle phase index."""
        is_fwd = (step % 2 == 0)
        pair_idx = step // 2
        pair_sign = self._wiggle_steer_sign if (pair_idx % 2 == 0) else -self._wiggle_steer_sign
        step_sign = pair_sign if is_fwd else -pair_sign
        if is_fwd:
            fwd_cap = 0.65 if front_dist_m < 0.12 else 0.95
            wiggle_v = min(self.forward_speed, fwd_cap)
        else:
            wiggle_v = -self.reverse_speed
        return wiggle_v, step_sign * self.steer_angle

    def _enter_wiggle_escape(self, initial_steer_sign: Optional[float] = None) -> Tuple[bool, float, float]:
        """Transitions to alternating forward/reverse wiggle escape when reverse stalls."""
        self.state = StuckRecoveryState.WIGGLE_ESCAPE
        self.stroke_timer = 0.0
        self._wiggle_step = 0
        self._wiggle_phase_timer = 0.0
        self._wiggle_stall_timer = 0.0
        self._rev_phys_stall_timer = 0.0
        self._rev_stroke_elapsed = 0.0
        self._wiggle_attempts += 1

        if initial_steer_sign is not None:
            self._wiggle_steer_sign = 1.0 if initial_steer_sign >= 0.0 else -1.0
        else:
            base_sign = -self.preferred_yaw_sign
            self._wiggle_steer_sign = base_sign if (self._wiggle_attempts % 2 == 1) else -base_sign

        # Flip preferred_yaw_sign so subsequent reverse attempts don't repeat the same wedged arc
        self.preferred_yaw_sign = -self.preferred_yaw_sign
        v_cmd, steer_cmd = self._wiggle_cmd_for_step(0)
        return self._cmd(v_cmd, steer_cmd)

    def _cmd(self, v: float, steer: float) -> Tuple[bool, float, float]:
        self.current_cmd_v = float(v)
        self.current_cmd_steer = float(steer)
        return True, self.current_cmd_v, self.current_cmd_steer

    # --- UNIFIED SENSOR PARSING ---
    @staticmethod
    def _is_sensor_left(key: str) -> bool:
        """Determines if a sensor observes the vehicle's left side."""
        k = key.lower()
        if "tof_front_left_4.1deg" in k:
            return True
        if "tof_front_right_4.1deg" in k:
            return False
        # Cross-pointing lateral sensors: front_right points LEFT (-X)
        if "tof_front_right_" in k:
            return True
        if "tof_front_left_" in k:
            return False
        if "side_left" in k or "tof_fl" in k:
            return True
        if "side_right" in k or "tof_fr" in k:
            return False
        return "left" in k

    @staticmethod
    def _is_sensor_right(key: str) -> bool:
        """Determines if a sensor observes the vehicle's right side."""
        k = key.lower()
        if "tof_front_right_4.1deg" in k:
            return True
        if "tof_front_left_4.1deg" in k:
            return False
        # Cross-pointing lateral sensors: front_left points RIGHT (+X)
        if "tof_front_left_" in k:
            return True
        if "tof_front_right_" in k:
            return False
        if "side_right" in k or "tof_fr" in k:
            return True
        if "side_left" in k or "tof_fl" in k:
            return False
        return "right" in k

    @classmethod
    def parse_cached_tofs(cls, cached_tofs: Optional[Any]) -> TofSummary:
        """Centralized parsing method for cached ToF sensor readings (ignoring <= 0.0 offline sentinels)."""
        if isinstance(cached_tofs, TofSummary):
            return cached_tofs
        if not cached_tofs:
            return TofSummary(TOF_MAX_RANGE_M, TOF_MAX_RANGE_M, TOF_MAX_RANGE_M, TOF_MAX_RANGE_M, TOF_MAX_RANGE_M, 0.0, 0)

        def _is_valid(val: Any) -> bool:
            try:
                f = float(val)
                return f > 0.0 and not math.isnan(f) and not math.isinf(f)
            except (TypeError, ValueError):
                return False

        if isinstance(cached_tofs, dict):
            f_vals = [
                float(cached_tofs[k])
                for k in ("tof_front_left_4.1deg", "tof_front_right_4.1deg")
                if k in cached_tofs and _is_valid(cached_tofs[k])
            ]
            d_str = min(f_vals) if f_vals else TOF_MAX_RANGE_M
            l_v = [float(v) for k, v in cached_tofs.items() if cls._is_sensor_left(k) and _is_valid(v)]
            r_v = [float(v) for k, v in cached_tofs.items() if cls._is_sensor_right(k) and _is_valid(v)]
            all_v = [float(v) for v in cached_tofs.values() if _is_valid(v)]
        else:
            vals = [float(v) for v in cached_tofs]
            half = len(vals) // 2
            l_v = [v for v in vals[:half] if _is_valid(v)]
            r_v = [v for v in vals[half:] if _is_valid(v)]
            all_v = [v for v in vals if _is_valid(v)]
            d_str = min(all_v) if all_v else TOF_MAX_RANGE_M

        min_l = min(l_v) if l_v else TOF_MAX_RANGE_M
        min_r = min(r_v) if r_v else TOF_MAX_RANGE_M
        avg_l = sum(l_v) / len(l_v) if l_v else TOF_MAX_RANGE_M
        avg_r = sum(r_v) / len(r_v) if r_v else TOF_MAX_RANGE_M
        max_any = max(all_v) if all_v else 0.0
        return TofSummary(min_l, min_r, avg_l, avg_r, d_str, max_any, len(all_v))

    def evaluate_costmap_escape_arcs(
        self,
        hazard_mask: Optional[np.ndarray],
        costmap_obj: Optional[Any],
    ) -> Tuple[float, float, float]:
        """Projects maximum steering lock arcs and center sightline onto the inflated costmap."""
        if hazard_mask is None or costmap_obj is None:
            return (TOF_MAX_RANGE_M, TOF_MAX_RANGE_M, TOF_MAX_RANGE_M)

        res = getattr(costmap_obj, "res", 0.015)
        x_min = getattr(costmap_obj, "x_min", -1.25)
        y_min = getattr(costmap_obj, "y_min", 0.00)
        h, w = hazard_mask.shape

        def _trace(pts: np.ndarray) -> float:
            cols = np.round((pts[:, 0] - x_min) / res).astype(np.int32)
            rows = np.round((pts[:, 1] - y_min) / res).astype(np.int32)
            valid = (pts[:, 1] >= self.bumper_y) & (0 <= rows) & (rows < h) & (0 <= cols) & (cols < w)
            hit_mask = np.zeros(len(pts), dtype=bool)
            hit_mask[valid] = hazard_mask[rows[valid], cols[valid]] > 0
            hits = np.flatnonzero(hit_mask)
            if len(hits) > 0:
                return float(self._arc_s[max(0, hits[0] - 1)])
            return float(self._arc_s[-1])

        return (_trace(self._arc_pts_left), _trace(self._arc_pts_right), _trace(self._arc_pts_center))

    def evaluate_pinned_state(
        self,
        hazard_mask: Optional[np.ndarray],
        costmap_obj: Optional[Any],
        cached_tofs: Optional[Any] = None,
        steer_dir: Optional[float] = None,
    ) -> Tuple[bool, bool]:
        """Sparse check if robot is pinned against obstacles in the inflated costmap."""
        tof = self.parse_cached_tofs(cached_tofs)
        is_l = (tof.min_l <= 0.035)
        is_r = (tof.min_r <= 0.035)

        if hazard_mask is not None and costmap_obj is not None:
            res = getattr(costmap_obj, "res", 0.015)
            x_min = getattr(costmap_obj, "x_min", -1.25)
            y_min = getattr(costmap_obj, "y_min", 0.00)
            h, w = hazard_mask.shape
            r = int(round((0.10 - y_min) / res))
            if 0 <= r < h:
                if steer_dir is None or steer_dir < 0.0:
                    cols = [int(round((x - x_min) / res)) for x in (-0.020, -0.035, -0.050)]
                    is_l = is_l or any(0 <= c < w and hazard_mask[r, c] > 0 for c in cols)
                if steer_dir is None or steer_dir > 0.0:
                    cols = [int(round((x - x_min) / res)) for x in (0.020, 0.035, 0.050)]
                    is_r = is_r or any(0 <= c < w and hazard_mask[r, c] > 0 for c in cols)

        self.is_left_pinned = bool(is_l)
        self.is_right_pinned = bool(is_r)
        return (self.is_left_pinned, self.is_right_pinned)

    # --- STEERING ARBITRATION ROUTINES ---
    @staticmethod
    def arbitrate_open_direction(
        arc_clearance: Tuple[float, float, float],
        cached_tofs: Optional[Any] = None,
        heading_err: float = 0.0,
        is_left_pinned: bool = False,
        is_right_pinned: bool = False,
        similar_eps_m: float = 0.030,
        default_dir: float = 1.0,
    ) -> float:
        """Unified direction arbitration (+1.0 Left, -1.0 Right)."""
        diff_arc = arc_clearance[0] - arc_clearance[1]
        if abs(diff_arc) > similar_eps_m:
            return 1.0 if diff_arc > 0.0 else -1.0

        if cached_tofs:
            tof = StuckRecoveryManager.parse_cached_tofs(cached_tofs)
            if tof.min_l < 0.12 or tof.min_r < 0.12:
                return 1.0 if tof.min_l > tof.min_r else -1.0
            diff_avg = tof.avg_l - tof.avg_r
            if abs(diff_avg) > similar_eps_m:
                return 1.0 if diff_avg > 0.0 else -1.0
            diff_min = tof.min_l - tof.min_r
            if abs(diff_min) > similar_eps_m:
                return 1.0 if diff_min > 0.0 else -1.0

        if is_left_pinned and not is_right_pinned:
            return -1.0
        if is_right_pinned and not is_left_pinned:
            return 1.0

        if math.radians(10.0) < abs(heading_err) < math.radians(170.0):
            return 1.0 if heading_err > 0.0 else -1.0

        return default_dir

    @staticmethod
    def resolve_escape_steer_dir(
        arc_clearance: Tuple[float, float, float],
        heading_err: float = 0.0,
        similar_eps_m: float = 0.020,
        is_left_pinned: bool = False,
        is_right_pinned: bool = False,
        **kwargs,
    ) -> float:
        """Arbitrates reverse escape steering (swings front nose away from obstacles in reverse)."""
        if is_left_pinned and not is_right_pinned:
            return 1.0
        if is_right_pinned and not is_left_pinned:
            return -1.0

        diff_arc = arc_clearance[0] - arc_clearance[1]
        if abs(diff_arc) > similar_eps_m:
            return 1.0 if diff_arc > 0.0 else -1.0

        if math.radians(10.0) < abs(heading_err) < math.radians(170.0):
            return 1.0 if heading_err > 0.0 else -1.0

        return -1.0

    @classmethod
    def resolve_open_side(
        cls,
        arc_clearance: Tuple[float, float, float],
        cached_tofs: Optional[Any] = None,
        heading_err: float = 0.0,
        similar_eps_m: float = 0.030,
        is_left_pinned: bool = False,
        is_right_pinned: bool = False,
        default_dir: float = 1.0,
        **kwargs,
    ) -> float:
        """Determines the clearer side for a forward U-turn (+1.0 Left, -1.0 Right).
        Must choose the side with MORE free space / clearance to complete the 180 spin.
        """
        # 1. Pinned obstacles override: steer AWAY from the pinned wall!
        if is_left_pinned and not is_right_pinned:
            return -1.0
        if is_right_pinned and not is_left_pinned:
            return 1.0

        # 2. Costmap Arc Clearances: Left (+1.0) vs Right (-1.0)
        diff_arc = arc_clearance[0] - arc_clearance[1]
        if abs(diff_arc) > similar_eps_m:
            return 1.0 if diff_arc > 0.0 else -1.0

        # 3. Lateral ToF Clearances: Left (+1.0) vs Right (-1.0)
        if cached_tofs:
            tof = cls.parse_cached_tofs(cached_tofs)
            if tof.min_l < 0.15 or tof.min_r < 0.15:
                return 1.0 if tof.min_l > tof.min_r else -1.0
            diff_avg = tof.avg_l - tof.avg_r
            if abs(diff_avg) > similar_eps_m:
                return 1.0 if diff_avg > 0.0 else -1.0
            diff_min = tof.min_l - tof.min_r
            if abs(diff_min) > similar_eps_m:
                return 1.0 if diff_min > 0.0 else -1.0

        # 4. Heading error bias (if clearances are symmetric)
        if math.radians(10.0) < abs(heading_err) < math.radians(170.0):
            return 1.0 if heading_err > 0.0 else -1.0

        return default_dir

    def compute_reverse_steer(
        self,
        min_l_tof: float = TOF_MAX_RANGE_M,
        min_r_tof: float = TOF_MAX_RANGE_M,
        current_yaw: float = 0.0,
        target_yaw: Optional[float] = None,
        current_v: float = 0.0,
        preferred_sign: Optional[float] = None,
        is_left_pinned: Optional[bool] = None,
        is_right_pinned: Optional[bool] = None,
        hazard_mask: Optional[np.ndarray] = None,
        costmap_obj: Optional[Any] = None,
        **kwargs,
    ) -> float:
        """Single Source of Truth for Reverse Steering Arbitration."""
        if target_yaw is not None:
            heading_err = wrap_to_pi(target_yaw - current_yaw)
            if abs(heading_err) > math.radians(10.0):
                intended_steer_dir = -1.0 if heading_err > 0.0 else 1.0
            else:
                pref = preferred_sign if preferred_sign is not None else self.preferred_yaw_sign
                intended_steer_dir = -pref
        else:
            pref = preferred_sign if preferred_sign is not None else self.preferred_yaw_sign
            intended_steer_dir = -pref

        if is_left_pinned is None or is_right_pinned is None:
            if hazard_mask is not None and costmap_obj is not None:
                l_pin, r_pin = self.evaluate_pinned_state(hazard_mask, costmap_obj)
                is_left_pinned = l_pin if is_left_pinned is None else is_left_pinned
                is_right_pinned = r_pin if is_right_pinned is None else is_right_pinned
            else:
                is_left_pinned = (min_l_tof <= 0.035) if is_left_pinned is None else is_left_pinned
                is_right_pinned = (min_r_tof <= 0.035) if is_right_pinned is None else is_right_pinned

        if is_left_pinned and is_right_pinned:
            reverse_steer = 0.0
        elif intended_steer_dir < 0.0:
            reverse_steer = +self.steer_angle if is_left_pinned else -self.steer_angle
        else:
            reverse_steer = -self.steer_angle if is_right_pinned else +self.steer_angle

        # Above 0.5 m/s forward speed, continue steering the opposite of reverse steer,
        # capped by the tire friction circle so braking + steering stays within grip.
        if current_v >= 0.5:
            if reverse_steer == 0.0:
                return 0.0
            a_lat_max = max(0.1, self.mu_friction) * GRAVITY_G
            friction_steer_cap = math.atan((a_lat_max * self.wheelbase) / (current_v * current_v))
            steer_mag = min(self.steer_angle, friction_steer_cap)
            return -math.copysign(steer_mag, reverse_steer)

        return reverse_steer

    def _step_reverse_stroke(
        self,
        tof: TofSummary,
        current_yaw: float,
        target_yaw: Optional[float],
        current_v: float,
        preferred_sign: float,
        is_l_pinned: bool,
        is_r_pinned: bool,
        hazard_mask: Optional[np.ndarray],
        costmap_obj: Optional[Any],
    ) -> Tuple[bool, float, float]:
        """Unified execution step for reverse motion across PIVOT_REVERSE and CORRECTIVE_180."""
        steer = self.compute_reverse_steer(
            min_l_tof=tof.min_l,
            min_r_tof=tof.min_r,
            current_yaw=current_yaw,
            target_yaw=target_yaw,
            current_v=current_v,
            preferred_sign=preferred_sign,
            is_left_pinned=is_l_pinned,
            is_right_pinned=is_r_pinned,
            hazard_mask=hazard_mask,
            costmap_obj=costmap_obj,
        )
        return self._cmd(-self.reverse_speed, steer)

    def trigger_emergency_recovery(
        self,
        heading_err: float = 0.0,
        steer_dir: Optional[float] = None,
        current_v: float = 0.0,
        current_steer: float = 0.0,
        instant_reverse: bool = True,
        target_breadcrumb_yaw: Optional[float] = None,
        current_yaw: float = 0.0,
        front_clearance_m: float = 0.0,
        hazard_mask: Optional[np.ndarray] = None,
        costmap_obj: Optional[Any] = None,
        cached_tofs: Optional[Any] = None,
        is_left_pinned: Optional[bool] = None,
        is_right_pinned: Optional[bool] = None,
        is_wrong_way: Optional[bool] = None,
        **kwargs,
    ):
        """Immediately engages recovery maneuver.
        If facing wrong way and front is open, executes a forward U-turn (CORRECTIVE_180).
        Otherwise, engages reverse recovery (PIVOT_REVERSE).
        """
        tof = self.parse_cached_tofs(cached_tofs)
        d_front = tof.d_straight if tof.count > 0 else float(front_clearance_m)

        if is_left_pinned is None or is_right_pinned is None:
            l_pin, r_pin = self.evaluate_pinned_state(hazard_mask, costmap_obj, cached_tofs)
            is_left_pinned = l_pin if is_left_pinned is None else is_left_pinned
            is_right_pinned = r_pin if is_right_pinned is None else is_right_pinned

        # Determine if facing wrong way (authoritatively provided by breadcrumb watchdog)
        is_facing_wrong_way = False
        yaw_diff = 0.0
        if target_breadcrumb_yaw is not None:
            yaw_diff = wrap_to_pi(target_breadcrumb_yaw - current_yaw)
            is_facing_wrong_way = bool(is_wrong_way) if is_wrong_way is not None else (abs(yaw_diff) > math.radians(75.0))

        # Front is considered open if clearance >= 0.22m and not pinned on both sides
        is_front_open = (d_front >= 0.22) and not (is_left_pinned and is_right_pinned)

        if is_facing_wrong_way and is_front_open:
            # Front is open: simply do a U-turn (CORRECTIVE_180 forward) towards the open side!
            arc_cl = self.evaluate_costmap_escape_arcs(hazard_mask, costmap_obj)
            open_side = steer_dir if steer_dir is not None else self.resolve_open_side(
                arc_clearance=arc_cl,
                cached_tofs=cached_tofs,
                heading_err=yaw_diff,
            )
            tgt_yaw = target_breadcrumb_yaw if target_breadcrumb_yaw is not None else wrap_to_pi(current_yaw + math.pi)
            self.trigger_corrective_180(
                target_yaw=tgt_yaw,
                steer_sign=open_side,
                current_yaw=current_yaw,
                arc_clearance=arc_cl,
                cached_tofs=cached_tofs,
                front_clearance_m=d_front,
                hazard_mask=hazard_mask,
                costmap_obj=costmap_obj,
                is_left_pinned=is_left_pinned,
                is_right_pinned=is_right_pinned,
            )
            return

        self.state = StuckRecoveryState.PIVOT_REVERSE
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        self._rev_phys_stall_timer = 0.0
        self._rev_stroke_elapsed = 0.0

        if steer_dir is not None:
            self.preferred_yaw_sign = 1.0 if steer_dir > 0 else -1.0
        elif is_left_pinned and not is_right_pinned:
            self.preferred_yaw_sign = 1.0
        elif is_right_pinned and not is_left_pinned:
            self.preferred_yaw_sign = -1.0
        elif abs(heading_err) > math.radians(25.0):
            self.preferred_yaw_sign = 1.0 if heading_err > 0 else -1.0
        else:
            self.preferred_yaw_sign = -1.0

        target_steer = self.compute_reverse_steer(
            current_yaw=current_yaw,
            target_yaw=target_breadcrumb_yaw,
            current_v=current_v,
            preferred_sign=self.preferred_yaw_sign,
            is_left_pinned=is_left_pinned,
            is_right_pinned=is_right_pinned,
            hazard_mask=hazard_mask,
            costmap_obj=costmap_obj,
        )
        self.current_cmd_v = -self.reverse_speed if instant_reverse else float(current_v)
        self.current_cmd_steer = target_steer if instant_reverse else float(current_steer)

    def trigger_corrective_180(
        self,
        target_yaw: Optional[float] = None,
        steer_sign: Optional[float] = None,
        current_yaw: float = 0.0,
        arc_clearance: Optional[Tuple[float, float, float]] = None,
        cached_tofs: Optional[Any] = None,
        front_clearance_m: float = 1.0,
        hazard_mask: Optional[np.ndarray] = None,
        costmap_obj: Optional[Any] = None,
        is_left_pinned: Optional[bool] = None,
        is_right_pinned: Optional[bool] = None,
    ):
        """Engages Anti-Wrong-Way corrective turn."""
        tof = self.parse_cached_tofs(cached_tofs)
        d_front = tof.d_straight if tof.count > 0 else float(front_clearance_m)

        # If already in PIVOT_REVERSE, only ignore if front is blocked (< 0.22m).
        # If front is open, allow transition from reverse to forward U-turn!
        if self.state == StuckRecoveryState.PIVOT_REVERSE and d_front < 0.22:
            return

        self.state = StuckRecoveryState.CORRECTIVE_180
        self.stroke_timer = 0.0
        self.total_recovery_timer = 0.0
        self.target_corrective_yaw = target_yaw
        self.last_corrective_yaw = current_yaw
        self.corrective_accumulated_turn = 0.0
        self.corrective_net_turn = 0.0
        self.corrective_reverse_timer = 0.0
        self._rev_best_d_straight = float(d_front)
        self._rev_stall_timer = 0.0
        self._rev_phys_stall_timer = 0.0
        self._rev_stroke_elapsed = 0.0

        if steer_sign is None:
            cl = arc_clearance or (0.5, 0.5, 0.5)
            y_diff = wrap_to_pi(target_yaw - current_yaw) if target_yaw is not None else 0.0
            steer_sign = self.resolve_open_side(cl, cached_tofs=cached_tofs, heading_err=y_diff)

        self.corrective_steer_dir = 1.0 if steer_sign >= 0.0 else -1.0
        self.preferred_yaw_sign = self.corrective_steer_dir

        if d_front < 0.22:
            self.corrective_used_reverse = True
            self.corrective_substate = "REVERSE"
            target_steer = self.compute_reverse_steer(
                min_l_tof=tof.min_l,
                min_r_tof=tof.min_r,
                current_yaw=current_yaw,
                target_yaw=self.target_corrective_yaw,
                current_v=-self.reverse_speed,
                preferred_sign=self.corrective_steer_dir,
                is_left_pinned=is_left_pinned,
                is_right_pinned=is_right_pinned,
                hazard_mask=hazard_mask,
                costmap_obj=costmap_obj,
            )
            self._cmd(-self.reverse_speed, target_steer)
        else:
            self.corrective_used_reverse = False
            self.corrective_substate = "FORWARD"
            self.forward_grace_timer = 0.35
            self._cmd(self.corrective_forward_speed, self.corrective_steer_dir * self.steer_angle)

    def engage_corrective_reverse(self, force: bool = False):
        """Engages reverse support phase during CORRECTIVE_180 when an obstacle is encountered."""
        if self.state == StuckRecoveryState.CORRECTIVE_180 and self.corrective_substate != "REVERSE":
            if force or self.forward_grace_timer <= 0.0:
                self.corrective_substate = "REVERSE"
                self.corrective_reverse_timer = 0.0
                self._rev_best_d_straight = 0.0
                self._rev_stall_timer = 0.0
                self._rev_phys_stall_timer = 0.0
                self._rev_stroke_elapsed = 0.0

    def update(
        self,
        dt: float,
        current_v: float,
        commanded_v: float = 0.0,
        heading_err: float = 0.0,
        active_steer: float = 0.0,
        hazard_mask: Optional[np.ndarray] = None,
        costmap_obj: Optional[Any] = None,
        cached_tofs: Optional[Any] = None,
        current_yaw: float = 0.0,
        target_breadcrumb_yaw: Optional[float] = None,
        early_exit_available: bool = False,
        is_wrong_way: Optional[bool] = None,
        **kwargs,
    ) -> Tuple[bool, float, float]:
        """Updates recovery state machine."""
        if self.state == StuckRecoveryState.NORMAL_DRIVE:
            return False, 0.0, 0.0

        self.stroke_timer += dt
        self.total_recovery_timer += dt
        if self.forward_grace_timer > 0.0:
            self.forward_grace_timer -= dt
            if current_v >= 0.05:
                self.forward_grace_timer = 0.0

        # PHASE 3: DECELERATE SPEED TO 0 WHILE CENTERING SERVO
        if self.state == StuckRecoveryState.DECEL_AND_CENTER:
            is_centered = (abs(active_steer) <= 0.04) and (abs(self.current_cmd_steer) <= 0.02)
            is_stopped = (abs(current_v) <= 0.15)
            if (is_centered and is_stopped) or (self.stroke_timer >= 0.25):
                return self._finish_recovery()
            return self._cmd(0.0, 0.0)

        # Parse ToF sensor summary and evaluate clearances
        tof = self.parse_cached_tofs(cached_tofs)
        is_l_pinned, is_r_pinned = self.evaluate_pinned_state(hazard_mask, costmap_obj, tof)
        front_blocked = (tof.d_straight < 0.15)
        is_unpinned = (not is_l_pinned and not is_r_pinned)
        front_cleared = (tof.d_straight >= self.clearance_exit_m and not front_blocked and is_unpinned)
        phys_unpinned = (tof.min_l > 0.035 and tof.min_r > 0.035)
        corrective_front_cleared = (tof.d_straight >= self.clearance_exit_m and not front_blocked and phys_unpinned)
        any_tof_cleared = (tof.max_any >= self.any_tof_exit_m) or (tof.count <= 2 and tof.max_any >= self.clearance_exit_m)

        # Track signed net turn across BOTH REVERSE and FORWARD substates of CORRECTIVE_180
        yaw_aligned = False
        is_turn_sufficient = False
        if self.state == StuckRecoveryState.CORRECTIVE_180:
            d_yaw_signed = wrap_to_pi(current_yaw - self.last_corrective_yaw)
            self.corrective_net_turn += d_yaw_signed
            self.corrective_accumulated_turn = abs(self.corrective_net_turn)
            self.last_corrective_yaw = current_yaw

            yaw_aligned = (
                self.target_corrective_yaw is not None
                and abs(wrap_to_pi(self.target_corrective_yaw - current_yaw)) <= self.corrective_align_rad
            )
            if self.target_corrective_yaw is not None:
                is_turn_sufficient = yaw_aligned and abs(self.corrective_net_turn) >= math.radians(60.0)
            else:
                is_turn_sufficient = abs(self.corrective_net_turn) >= math.radians(145.0)

        # WIGGLE ESCAPE: Alternating forward/reverse pulses with alternating steering
        if self.state == StuckRecoveryState.WIGGLE_ESCAPE:
            self._wiggle_phase_timer += dt
            is_moving_freely = (
                abs(current_v) >= 0.25
                and tof.d_straight >= self.clearance_exit_m
                and not (is_l_pinned and is_r_pinned)
            )
            is_open_exit = (front_cleared and any_tof_cleared and self._wiggle_phase_timer >= 0.10)
            if is_open_exit or is_moving_freely:
                self._wiggle_attempts = 0
                if current_v < -0.15:
                    return self._enter_decel_and_center()
                return self._finish_recovery()

            is_fwd = (self._wiggle_step % 2 == 0)
            phase_dur = (0.22 if tof.d_straight < 0.12 else self._wiggle_phase_duration_s) if is_fwd else 0.35
            if self._wiggle_phase_timer >= 0.16:
                phase_stalled = (current_v < 0.10) if is_fwd else (current_v > -0.10)
                if phase_stalled:
                    self._wiggle_stall_timer += dt
                else:
                    self._wiggle_stall_timer = 0.0
            else:
                self._wiggle_stall_timer = 0.0

            if self._wiggle_phase_timer >= phase_dur or self._wiggle_stall_timer >= 0.14:
                self._wiggle_step += 1
                self._wiggle_phase_timer = 0.0
                self._wiggle_stall_timer = 0.0
                if self._wiggle_step >= self._wiggle_max_steps or self.total_recovery_timer >= 3.2:
                    return self._enter_decel_and_center()

            v_cmd, steer_cmd = self._wiggle_cmd_for_step(self._wiggle_step, front_dist_m=tof.d_straight)
            return self._cmd(v_cmd, steer_cmd)

        # REVERSE STROKE EXECUTION (Shared by PIVOT_REVERSE and CORRECTIVE_180[REVERSE])
        if self.state == StuckRecoveryState.PIVOT_REVERSE or (
            self.state == StuckRecoveryState.CORRECTIVE_180 and self.corrective_substate == "REVERSE"
        ):
            # Only start reverse stall timer once forward braking has completed (current_v <= 0.05)
            if current_v <= 0.05:
                self._rev_stroke_elapsed += dt
            if self._rev_stroke_elapsed >= 0.30 and current_v > -0.10:
                self._rev_phys_stall_timer += dt
            else:
                self._rev_phys_stall_timer = 0.0
            is_rev_phys_stalled = (self._rev_phys_stall_timer >= 0.25)

            if self.state == StuckRecoveryState.PIVOT_REVERSE:
                # Check if facing wrong way with open front -> transition to U-turn!
                is_facing_ww = False
                y_diff = 0.0
                if target_breadcrumb_yaw is not None:
                    y_diff = wrap_to_pi(target_breadcrumb_yaw - current_yaw)
                    is_facing_ww = bool(is_wrong_way) if is_wrong_way is not None else (abs(y_diff) > math.radians(75.0))

                if is_facing_ww and tof.d_straight >= 0.22 and not (is_l_pinned and is_r_pinned):
                    arc_cl = self.evaluate_costmap_escape_arcs(hazard_mask, costmap_obj)
                    tgt_yaw = target_breadcrumb_yaw if target_breadcrumb_yaw is not None else wrap_to_pi(current_yaw + math.pi)
                    open_side = self.resolve_open_side(arc_cl, cached_tofs=tof, heading_err=y_diff)
                    self.trigger_corrective_180(
                        target_yaw=tgt_yaw,
                        steer_sign=open_side,
                        current_yaw=current_yaw,
                        arc_clearance=arc_cl,
                        cached_tofs=tof,
                        front_clearance_m=tof.d_straight,
                        hazard_mask=hazard_mask,
                        costmap_obj=costmap_obj,
                        is_left_pinned=is_l_pinned,
                        is_right_pinned=is_r_pinned,
                    )
                    return self._cmd(self.corrective_forward_speed, self.corrective_steer_dir * self.steer_angle)

                if self.total_recovery_timer >= RECOVERY_TOTAL_TIMEOUT_S and tof.d_straight >= self.clearance_exit_m:
                    return self._finish_recovery()
                if self.stroke_timer >= RECOVERY_EXIT_MIN_TIME_S and front_cleared and any_tof_cleared:
                    self._wiggle_attempts = 0
                    return self._enter_decel_and_center()
                if is_rev_phys_stalled:
                    return self._enter_wiggle_escape()
                if self.stroke_timer >= RECOVERY_REVERSE_STROKE_MAX_S:
                    if tof.d_straight >= self.clearance_exit_m:
                        return self._enter_decel_and_center()
                    return self._enter_wiggle_escape()
                t_yaw = target_breadcrumb_yaw
                p_sign = self.preferred_yaw_sign
            else:
                self.corrective_used_reverse = True
                self.corrective_reverse_timer += dt
                if tof.d_straight > self._rev_best_d_straight + 0.035:
                    self._rev_best_d_straight = tof.d_straight
                    self._rev_stall_timer = 0.0
                else:
                    self._rev_stall_timer += dt

                # If reverse stroke completed alignment and clearance opened up, exit cleanly
                if is_turn_sufficient and corrective_front_cleared and any_tof_cleared:
                    return self._enter_decel_and_center()
                if self.total_recovery_timer >= RECOVERY_180_TIMEOUT_WITH_REV_S:
                    return self._enter_decel_and_center()

                if is_rev_phys_stalled and tof.d_straight < 0.18:
                    return self._enter_wiggle_escape(initial_steer_sign=-self.corrective_steer_dir)

                has_min_reverse_time = (self.corrective_reverse_timer >= max(RECOVERY_EXIT_MIN_TIME_S, self.corrective_rev_min_s))
                reverse_timeout = (self.corrective_reverse_timer >= RECOVERY_REVERSE_STROKE_MAX_S)
                rev_clear_target = max(self.clearance_exit_m, self.corrective_rev_clear_m)
                rev_stalled_with_room = (
                    self.corrective_reverse_timer >= 0.30
                    and self._rev_stall_timer >= 0.18
                    and tof.d_straight >= 0.22
                )
                if (has_min_reverse_time and (tof.d_straight >= rev_clear_target or rev_stalled_with_room or is_rev_phys_stalled)) or reverse_timeout:
                    self.corrective_substate = "FORWARD"
                    self.stroke_timer = 0.0
                    self.forward_grace_timer = 0.35
                    d_esc_l, d_esc_r, _ = self.evaluate_costmap_escape_arcs(hazard_mask, costmap_obj)

                    chosen_dir = self.corrective_steer_dir
                    if self.target_corrective_yaw is not None:
                        rem_err = wrap_to_pi(self.target_corrective_yaw - current_yaw)
                        if abs(rem_err) < math.radians(135.0) and abs(rem_err) > math.radians(10.0):
                            desired = 1.0 if rem_err > 0.0 else -1.0
                            desired_blocked = (desired > 0.0 and (is_l_pinned or (d_esc_l < 0.18 and d_esc_r >= 0.28))) or (
                                desired < 0.0 and (is_r_pinned or (d_esc_r < 0.18 and d_esc_l >= 0.28))
                            )
                            if not desired_blocked:
                                chosen_dir = desired
                            elif abs(d_esc_l - d_esc_r) > 0.04:
                                chosen_dir = 1.0 if d_esc_l > d_esc_r else -1.0
                        elif abs(d_esc_l - d_esc_r) > 0.04:
                            chosen_dir = 1.0 if d_esc_l > d_esc_r else -1.0
                    elif abs(d_esc_l - d_esc_r) > 0.04:
                        chosen_dir = 1.0 if d_esc_l > d_esc_r else -1.0

                    self.corrective_steer_dir = chosen_dir
                    self.preferred_yaw_sign = self.corrective_steer_dir
                    return self._cmd(self.corrective_forward_speed, self.corrective_steer_dir * self.steer_angle)
                t_yaw = self.target_corrective_yaw
                p_sign = self.corrective_steer_dir
                # Do not override reverse U-turn direction with corner-ToF pinning (prevents corner limit cycles)
                return self._step_reverse_stroke(
                    tof, current_yaw, t_yaw, current_v, p_sign, False, False, hazard_mask, costmap_obj
                )

            return self._step_reverse_stroke(
                tof, current_yaw, t_yaw, current_v, p_sign, is_l_pinned, is_r_pinned, hazard_mask, costmap_obj
            )

        # PHASE 2: CORRECTIVE 180 AUTO-CORRECTION ARC (FORWARD SUBSTATE)
        if self.state == StuckRecoveryState.CORRECTIVE_180:
            # If a safe long-path early exit is found, front is open, and chassis has turned enough, terminate U-turn
            if self.target_corrective_yaw is not None:
                can_early_exit = abs(wrap_to_pi(self.target_corrective_yaw - current_yaw)) <= math.radians(35.0)
            else:
                can_early_exit = abs(self.corrective_net_turn) >= self.corrective_early_exit_turn_rad
            if (early_exit_available or kwargs.get("early_exit_available", False)) and can_early_exit and tof.d_straight >= 0.80 and self.corrective_substate == "FORWARD":
                return self._finish_recovery()

            if is_turn_sufficient and corrective_front_cleared and any_tof_cleared:
                return self._finish_recovery()

            max_180_timeout = RECOVERY_180_TIMEOUT_WITH_REV_S if self.corrective_used_reverse else RECOVERY_180_TIMEOUT_S
            if (self.total_recovery_timer >= 1.2 and tof.d_straight >= 0.20 and is_turn_sufficient) or self.total_recovery_timer >= max_180_timeout:
                return self._finish_recovery()

            # Proactive stopping-distance & corner-clearance reverse trigger so we stop BEFORE wall impact
            d_proactive_rev = max(0.16, min(0.28, 0.15 + 0.14 * max(0.0, current_v)))
            outer_corner_dist = tof.min_r if self.corrective_steer_dir > 0.0 else tof.min_l
            inner_corner_dist = tof.min_l if self.corrective_steer_dir > 0.0 else tof.min_r
            is_front_blocked = (tof.d_straight < 0.16)
            is_proactive_blocked = (self.forward_grace_timer <= 0.0 and tof.d_straight < d_proactive_rev)
            is_corner_imminent = (
                self.forward_grace_timer <= 0.0
                and (outer_corner_dist < 0.12 or inner_corner_dist < 0.08)
                and tof.d_straight < 0.35
            )
            is_stalled = (self.forward_grace_timer <= 0.0 and abs(current_v) < 0.10 and self.stroke_timer > 0.25)
            is_wall_collision = (
                is_front_blocked
                or is_proactive_blocked
                or (tof.d_straight < 0.20 and (tof.min_l <= 0.04 or tof.min_r <= 0.04))
                or is_corner_imminent
            )
            if is_wall_collision:
                self.engage_corrective_reverse(force=True)
            elif is_stalled:
                self.engage_corrective_reverse(force=False)

            if self.corrective_substate == "REVERSE":
                return self._step_reverse_stroke(
                    tof, current_yaw, self.target_corrective_yaw, current_v,
                    self.corrective_steer_dir, False, False, hazard_mask, costmap_obj
                )

            # Only taper forward U-turn speed in the final braking zone (< 0.30m)
            if tof.d_straight < 0.30:
                taper_v = max(0.65, 3.5 * (tof.d_straight - 0.12))
                fwd_cmd_v = min(self.corrective_forward_speed, taper_v)
            else:
                fwd_cmd_v = self.corrective_forward_speed
            return self._cmd(fwd_cmd_v, self.corrective_steer_dir * self.steer_angle)

        return False, 0.0, 0.0



=== FILE: brain/tof_gap_scout.py ===
"""
================================================================================
  16-CHANNEL TOF & 15mm COSTMAP MULTI-TARGET GAP SCOUT MODULE

  Key Upgrades:
  1. RAYTRACING ON INFLATED COSTMAP (Hazard Mask):
     Directly casts radial lines across the front FOV through the expanded
     hazard grid (15mm resolution, dilated for vehicle clearance).
     For each line, computes the exact unobstructed distance to the first obstacle.
  2. MULTI-TARGET DISCOVERY:
     Finds the top local maxima (distinct furthest clear targets) so the vehicle
     is not stuck to a single centerline point.
  3. APERTURE & HEADING ESTIMATION:
     Provides primary target (xg, yg, width, theta) plus alternative targets
     so the trajectory planners evaluate candidates toward multiple openings.
================================================================================
"""

import math
from typing import Dict, List, Optional, Tuple
import numpy as np

try:
    from .planner_config import (
        GAP_MIN_WIDTH_M,
        GAP_MIN_LOOKAHEAD_M,
        GAP_MAX_LOOKAHEAD_M,
        GRID_X_MIN,
        GRID_X_MAX,
        GRID_Y_MIN,
        GRID_Y_MAX,
        GRID_RES,
        TOF_MAX_RANGE_M,
        R_INFLATION_M,
        WEIGHT_GAP_DISTANCE,
        WEIGHT_GAP_WIDTH,
        WEIGHT_ALIGN_STEER,
        WEIGHT_ALIGN_STRAIGHT,
        WEIGHT_SIDE_CLEARANCE,
        TRACK_CORRIDOR_WIDTH_M,
        GAP_DIST_THRESH_M,
        GAP_DIST_EXP_K,
        ALT_MIN_SEPARATION_M,
        ALT_APERTURE_BIAS,
    )
except ImportError:
    from planner_config import (
        GAP_MIN_WIDTH_M,
        GAP_MIN_LOOKAHEAD_M,
        GAP_MAX_LOOKAHEAD_M,
        GRID_X_MIN,
        GRID_X_MAX,
        GRID_Y_MIN,
        GRID_Y_MAX,
        GRID_RES,
        TOF_MAX_RANGE_M,
        R_INFLATION_M,
        WEIGHT_GAP_DISTANCE,
        WEIGHT_GAP_WIDTH,
        WEIGHT_ALIGN_STEER,
        WEIGHT_ALIGN_STRAIGHT,
        WEIGHT_SIDE_CLEARANCE,
        TRACK_CORRIDOR_WIDTH_M,
        GAP_DIST_THRESH_M,
        GAP_DIST_EXP_K,
        ALT_MIN_SEPARATION_M,
        ALT_APERTURE_BIAS,
    )

# 16-Channel Physical ToF Sensor Mounting Poses
# Format: (sensor_name, mount_x, mount_y, heading_rad, max_range_m)
FRONT_TOF_SENSOR_RIG = [
    # Front-Left Array (cross-pointing Right, rotated 3 deg outwards)
    # Leftmost ToF (pointing straight and right, zones 1.1 deg to 34.9 deg)
    ("tof_front_left_4.1deg",  -0.058, 0.092, 1.5512, TOF_MAX_RANGE_M),
    ("tof_front_left_15.4deg", -0.058, 0.092, 1.3548, TOF_MAX_RANGE_M),
    ("tof_front_left_26.6deg", -0.058, 0.092, 1.1585, TOF_MAX_RANGE_M),
    ("tof_front_left_37.9deg", -0.058, 0.092, 0.9621, TOF_MAX_RANGE_M),
    # Left ToF (pointing right, zones 46.1 deg to 79.9 deg)
    ("tof_front_left_49.1deg", -0.057, 0.090, 0.7658, TOF_MAX_RANGE_M),
    ("tof_front_left_60.4deg", -0.057, 0.090, 0.5694, TOF_MAX_RANGE_M),
    ("tof_front_left_71.6deg", -0.057, 0.090, 0.3731, TOF_MAX_RANGE_M),
    ("tof_front_left_82.9deg", -0.057, 0.090, 0.1767, TOF_MAX_RANGE_M),
    # Front-Right Array (cross-pointing Left, mirrored along Y axis, rotated 3 deg outwards)
    # Rightmost ToF (pointing straight and left, zones 1.1 deg to 34.9 deg)
    ("tof_front_right_4.1deg",   0.058, 0.092, 1.5904, TOF_MAX_RANGE_M),
    ("tof_front_right_15.4deg",  0.058, 0.092, 1.7868, TOF_MAX_RANGE_M),
    ("tof_front_right_26.6deg",  0.058, 0.092, 1.9831, TOF_MAX_RANGE_M),
    ("tof_front_right_37.9deg",  0.058, 0.092, 2.1795, TOF_MAX_RANGE_M),
    # Right ToF (pointing left, zones 46.1 deg to 79.9 deg)
    ("tof_front_right_49.1deg",  0.057, 0.090, 2.3758, TOF_MAX_RANGE_M),
    ("tof_front_right_60.4deg",  0.057, 0.090, 2.5722, TOF_MAX_RANGE_M),
    ("tof_front_right_71.6deg",  0.057, 0.090, 2.7685, TOF_MAX_RANGE_M),
    ("tof_front_right_82.9deg",  0.057, 0.090, 2.9649, TOF_MAX_RANGE_M),
]


# 16-Channel ToF Pod Geometry:
# 8 zones per corner, each zone has 11.25 deg FOV (8 * 11.25 deg = 90.0 deg total FOV per pod).
# Zone midpoints: 1.125 deg, 12.375 deg, ..., 79.875 deg.
# Zone outer bounds: from -4.5 deg (-0.0785 rad) to +85.5 deg (+1.4923 rad). Outer edges meet at 171 deg.
_NUM_CORNER_RAYS = 90

# Left Pod (-0.0575m, 0.091m, cross-pointing Right): -4.5 deg to +85.5 deg rel to +Y
_LEFT_REL_ANGLES = np.linspace(math.radians(-4.5), math.radians(85.5), _NUM_CORNER_RAYS)
_LEFT_AZIMUTHS = (math.pi / 2.0) - _LEFT_REL_ANGLES
_LEFT_COS = np.cos(_LEFT_AZIMUTHS)
_LEFT_SIN = np.sin(_LEFT_AZIMUTHS)

# Right Pod (+0.0575m, 0.091m, cross-pointing Left): +4.5 deg to -85.5 deg rel to +Y (i.e. -4.5 deg to +85.5 deg leftwards)
_RIGHT_REL_ANGLES = np.linspace(math.radians(4.5), -math.radians(85.5), _NUM_CORNER_RAYS)
_RIGHT_AZIMUTHS = (math.pi / 2.0) - _RIGHT_REL_ANGLES
_RIGHT_COS = np.cos(_RIGHT_AZIMUTHS)
_RIGHT_SIN = np.sin(_RIGHT_AZIMUTHS)


class ToFGapScout:
    """
    Scouts furthest points and navigable openings by combining direct ToF sensor
    beam sightlines (LUT) with C++ inflated obstacle raycasting.
    """

    def __init__(
        self,
        w_gap_distance: float = WEIGHT_GAP_DISTANCE,
        w_gap_width: float = WEIGHT_GAP_WIDTH,
        w_align_steer: float = WEIGHT_ALIGN_STEER,
        w_align_straight: float = WEIGHT_ALIGN_STRAIGHT,
        w_side_clearance: float = WEIGHT_SIDE_CLEARANCE,
        gap_dist_exp_k: float = GAP_DIST_EXP_K,
        gap_dist_thresh_m: float = GAP_DIST_THRESH_M,
        gap_progress_margin_m: Optional[float] = None,
        alt_min_separation_m: float = ALT_MIN_SEPARATION_M,
        alt_aperture_bias: float = ALT_APERTURE_BIAS,
        scout_obs_width_m: float = 0.22,
        scout_block_dist_m: Optional[float] = None,
        scout_straight_pen_k: float = 2.50,
        scout_straight_exp_k: float = 1.50,
        ray_origin_offset_m: float = 0.0575,
        w_gap_clear: float = 3.00,
        **kwargs,
    ):
        self.w_gap_distance = float(w_gap_distance)
        self.w_gap_width = float(w_gap_width)
        self.w_align_steer = float(w_align_steer)
        self.w_align_straight = float(w_align_straight)
        self.w_side_clearance = float(w_side_clearance)
        self.gap_dist_exp_k = float(gap_dist_exp_k)
        if gap_progress_margin_m is not None:
            self.gap_dist_thresh_m = float(TRACK_CORRIDOR_WIDTH_M + gap_progress_margin_m)
        else:
            self.gap_dist_thresh_m = float(gap_dist_thresh_m)
        self.alt_min_separation_m = float(alt_min_separation_m)
        self.alt_aperture_bias = float(alt_aperture_bias)
        self.scout_obs_width_m = float(scout_obs_width_m)
        self.scout_block_dist_m = float(scout_block_dist_m) if scout_block_dist_m is not None else None
        self.scout_straight_pen_k = float(scout_straight_pen_k)
        self.scout_straight_exp_k = float(scout_straight_exp_k)
        self.ray_origin_offset_m = float(ray_origin_offset_m)
        self.w_gap_clear = float(kwargs.get("alt_lookahead_depth_weight", w_gap_clear))
        self.last_xg: float = 0.0
        self.last_yg: float = 1.20
        self.last_aperture: float = 0.50
        self.last_targets: List[Tuple[float, float, float]] = []

    def scout_gap(
        self,
        tof_readings: Dict[str, float],
        hazard_mask: Optional[np.ndarray] = None,
        costmap_obj: Optional[any] = None,
        target_lookahead_m: float = 2.30,
        steer_current: float = 0.0,
        prob_map: Optional[np.ndarray] = None,
        anchor_x: float = 0.0,
        anchor_y: float = 0.0,
        anchor_yaw: float = 0.0,
    ) -> Tuple[float, float, float, float]:
        """
        Returns primary (x_g, y_g, aperture_width_m, recovery_heading_rad).
        Also stores multi-target options in self.last_targets: [(xg, yg, score), ...].
        """
        targets = self.find_furthest_targets(
            tof_readings=tof_readings,
            hazard_mask=hazard_mask,
            costmap_obj=costmap_obj,
            max_lookahead_m=target_lookahead_m,
            top_k=2,
            steer_current=steer_current,
            anchor_x=anchor_x,
            anchor_y=anchor_y,
            anchor_yaw=anchor_yaw,
        )

        if not targets:
            xg_prim = anchor_x
            yg_prim = anchor_y + max(GAP_MIN_LOOKAHEAD_M, min(target_lookahead_m, GAP_MAX_LOOKAHEAD_M))
            aperture = 0.45
        else:
            xg_prim, yg_prim, aperture = targets[0][:3]

        # Raw physical target coordinates directly without EMA smoothing or midpoint blending
        self.last_xg = xg_prim
        self.last_yg = yg_prim
        self.last_aperture = aperture
        self.last_targets = targets

        raw_rel = math.atan2(xg_prim - anchor_x, max(0.20, yg_prim - anchor_y)) + anchor_yaw
        theta_g = math.atan2(math.sin(raw_rel), math.cos(raw_rel))
        return xg_prim, yg_prim, aperture, theta_g

    def find_furthest_targets(
        self,
        tof_readings: Optional[Dict[str, float]] = None,
        hazard_mask: Optional[np.ndarray] = None,
        costmap_obj: Optional[any] = None,
        max_lookahead_m: float = 1.40,
        top_k: int = 3,
        steer_current: float = 0.0,
        prob_map: Optional[np.ndarray] = None,
        anchor_x: float = 0.0,
        anchor_y: float = 0.0,
        anchor_yaw: float = 0.0,
    ) -> List[Tuple[float, float, float]]:
        """
        Extracts furthest targets by evaluating real ToF beam sightlines and dense radial
        rays through the C++ inflated costmap (hazard_mask).
        Guarantees targets lie on physical clear space at the true lookahead distance
        without colliding with inflated obstacle boundaries.
        Returns list of (x, y, aperture_width).
        """
        # Fast C++ Native Gap Scout Path (0.05 ms vs 5 ms)
        if costmap_obj is not None and hasattr(costmap_obj, "scout_gaps_native") and hasattr(costmap_obj, "_handle") and costmap_obj._handle:
            targets = costmap_obj.scout_gaps_native(
                max_lookahead_m=max_lookahead_m,
                steer_current=steer_current,
                w_gap_distance=self.w_gap_distance,
                w_gap_width=self.w_gap_width,
                w_align_steer=self.w_align_steer,
                w_align_straight=self.w_align_straight,
                w_side_clearance=self.w_side_clearance,
                gap_dist_exp_k=self.gap_dist_exp_k,
                gap_dist_thresh_m=self.gap_dist_thresh_m,
                alt_min_separation_m=self.alt_min_separation_m,
                alt_aperture_bias=self.alt_aperture_bias,
                scout_obs_width_m=self.scout_obs_width_m,
                scout_block_dist_m=self.scout_block_dist_m if self.scout_block_dist_m is not None else max_lookahead_m,
                scout_straight_pen_k=self.scout_straight_pen_k,
                scout_straight_exp_k=self.scout_straight_exp_k,
                top_k=top_k,
                hazard_mask=hazard_mask,
                anchor_x=anchor_x,
                anchor_y=anchor_y,
                anchor_yaw=anchor_yaw,
            )
            return targets

        if hazard_mask is None or costmap_obj is None:
            return [(0.0, max_lookahead_m, 0.40)]

        res = costmap_obj.res
        x_min = costmap_obj.x_min
        y_min = costmap_obj.y_min
        h, w = hazard_mask.shape
        step_ds = max(res * 0.8, 0.012)

        candidates = []

        # ----------------------------------------------------------------------
        # 1. EVALUATE 16 PHYSICAL TOF BEAM SIGHTLINES (LUT + MEASURED RANGES)
        # ----------------------------------------------------------------------
        if tof_readings:
            active_rig = FRONT_TOF_SENSOR_RIG
            for s_name, sx, sy, heading, max_range in active_rig:
                raw_z = tof_readings.get(s_name, max_range)
                z = min(max_range, max(0.10, raw_z))
                cos_h = math.cos(heading)
                sin_h = math.sin(heading)

                clear_dist = z
                hit_obs = False
                n_steps = int(z / step_ds)
                for s in range(1, n_steps + 1):
                    d = s * step_ds
                    px = sx + d * cos_h
                    py = sy + d * sin_h
                    col = int(round((px - x_min) / res))
                    row = int(round((py - y_min) / res))
                    if row < 0 or row >= h or col < 0 or col >= w:
                        clear_dist = max(0.0, d - step_ds)
                        break
                    if hazard_mask[row, col] > 0:
                        clear_dist = max(0.0, d - step_ds)
                        hit_obs = True
                        break

                # Target at maximum clear beam distance (backed off by 7.5cm safety margin ONLY if obstacle hit)
                eff_dist = max(0.05, clear_dist - 0.075) if hit_obs else clear_dist
                tx = sx + eff_dist * cos_h
                ty = max(0.08, sy + eff_dist * sin_h)
                d_ax = tx - anchor_x
                d_ay = max(0.05, ty - anchor_y)
                total_dist = math.hypot(d_ax, d_ay)

                # Heading in steering coordinates relative to anchor (positive = left)
                raw_rel = math.atan2(d_ax, d_ay) + anchor_yaw
                rel_angle = math.atan2(math.sin(raw_rel), math.cos(raw_rel))
                steer_equiv = -rel_angle

                # Aperture estimation around this beam
                aperture_m = max(GAP_MIN_WIDTH_M, min(1.20, total_dist * 0.35))
                is_clear = (not hit_obs) and (raw_z >= (max_range - 0.01) or total_dist >= (max_lookahead_m - 0.05))
                candidates.append({
                    "tx": tx, "ty": ty, "dist": total_dist,
                    "aperture": aperture_m, "steer_equiv": steer_equiv,
                    "rel_angle": rel_angle,
                    "is_clear": is_clear,
                })

        # ----------------------------------------------------------------------
        # 2. EVALUATE DENSE RADIAL RAYS (180 TOTAL: 90 PER FRONT CORNER POD)
        # Left Corner (-0.07m, 0.095m):  90 rays spanning -89 deg to 0 deg (Left to Center)
        # Right Corner (+0.07m, 0.095m): 90 rays spanning 0 deg to +89 deg (Center to Right)
        # ----------------------------------------------------------------------
        max_r = min(max_lookahead_m, GAP_MAX_LOOKAHEAD_M)
        n_steps = int(max_r / step_ds)

        corner_fans = [
            (-self.ray_origin_offset_m, 0.091, _LEFT_COS, _LEFT_SIN),
            (self.ray_origin_offset_m, 0.091, _RIGHT_COS, _RIGHT_SIN),
        ]

        for ox, oy, cos_arr, sin_arr in corner_fans:
            for i in range(_NUM_CORNER_RAYS):
                cos_az = float(cos_arr[i])
                sin_az = float(sin_arr[i])
                clear_dist = max_r
                hit_obs = False

                for s in range(1, n_steps + 1):
                    d = s * step_ds
                    px = ox + d * cos_az
                    py = oy + d * sin_az
                    col = int(round((px - x_min) / res))
                    row = int(round((py - y_min) / res))
                    if row < 0 or row >= h or col < 0 or col >= w:
                        clear_dist = max(0.0, d - step_ds)
                        break
                    if hazard_mask[row, col] > 0:
                        clear_dist = max(0.0, d - step_ds)
                        hit_obs = True
                        break

                eff_dist = max(0.05, clear_dist - 0.06) if hit_obs else clear_dist
                # Physical target point in vehicle local coordinates (origin at vehicle center / rear axle)
                tx = ox + eff_dist * cos_az
                ty = max(0.08, oy + eff_dist * sin_az)
                d_ax = tx - anchor_x
                d_ay = max(0.05, ty - anchor_y)
                total_dist = math.hypot(d_ax, d_ay)

                # True relative angle from anchor to target (tx, ty)
                raw_rel = math.atan2(d_ax, d_ay) + anchor_yaw
                rel_angle = math.atan2(math.sin(raw_rel), math.cos(raw_rel))
                steer_equiv = -rel_angle
                aperture_m = max(GAP_MIN_WIDTH_M, min(1.20, total_dist * 0.30))

                candidates.append({
                    "tx": tx, "ty": ty, "dist": total_dist,
                    "aperture": aperture_m, "steer_equiv": steer_equiv,
                    "rel_angle": rel_angle,
                    "is_clear": not hit_obs,
                })

        if not candidates:
            return [(anchor_x, anchor_y + max_lookahead_m, 0.40, True)]

        max_clear_dist = max([c["dist"] for c in candidates])
        has_far_path = max_clear_dist > 0.85
        min_allowed_dist = max(0.30, max_clear_dist * 0.45) if has_far_path else 0.30

        # ----------------------------------------------------------------------
        # 3. SCORE CANDIDATE TARGETS
        # ----------------------------------------------------------------------
        # Check forward clearance directly along vehicle forward axis from anchor
        d_center = max_clear_dist
        if hazard_mask is not None:
            n_center_steps = int(max_r / step_ds)
            cos_fwd = -math.sin(anchor_yaw)
            sin_fwd =  math.cos(anchor_yaw)
            for s in range(1, n_center_steps + 1):
                d = s * step_ds
                px = anchor_x + d * cos_fwd
                py = anchor_y + 0.091 + d * sin_fwd
                col = int(round((px - x_min) / res))
                row = int(round((py - y_min) / res))
                if row < 0 or row >= h or col < 0 or col >= w or hazard_mask[row, col] > 0:
                    d_center = max(0.0, d - step_ds)
                    break
            else:
                d_center = max_r
        else:
            center_cands = [c["dist"] for c in candidates if abs(c["rel_angle"]) < math.radians(6)]
            d_center = max(center_cands) if center_cands else max_clear_dist

        # Compute side clearance asymmetry from lateral and front ToF sensors (8 per side):
        side_diff = 0.0
        if tof_readings:
            if any("front" in k for k in tof_readings):
                # 16-channel array:
                # Left-side openness: left front 4.1deg + right pod cross-pointing left 15.4deg to 82.9deg
                left_keys = ["tof_front_left_4.1deg"] + [f"tof_front_right_{ang}deg" for ang in [15.4, 26.6, 37.9, 49.1, 60.4, 71.6, 82.9]]
                # Right-side openness: right front 4.1deg + left pod cross-pointing right 15.4deg to 82.9deg
                right_keys = ["tof_front_right_4.1deg"] + [f"tof_front_left_{ang}deg" for ang in [15.4, 26.6, 37.9, 49.1, 60.4, 71.6, 82.9]]
            else:
                left_keys = ["tof_fl_3", "tof_fr_0", "tof_fr_1", "tof_fr_2", "tof_bl_0", "tof_bl_1", "tof_bl_2", "tof_bl_3"]
                right_keys = ["tof_fr_3", "tof_fl_0", "tof_fl_1", "tof_fl_2", "tof_br_0", "tof_br_1", "tof_br_2", "tof_br_3"]

            l_vals = [tof_readings.get(k, TOF_MAX_RANGE_M) for k in left_keys]
            r_vals = [tof_readings.get(k, TOF_MAX_RANGE_M) for k in right_keys]
            l_total = sum(l_vals) / len(l_vals) if l_vals else TOF_MAX_RANGE_M
            r_total = sum(r_vals) / len(r_vals) if r_vals else TOF_MAX_RANGE_M
            denom = max(0.25, 0.5 * (l_total + r_total))
            side_diff = (l_total - r_total) / denom

        w_straight = self.w_align_straight
        straight_block_penalty = 0.0
        theta_shadow = 0.0
        eff_block_dist = self.scout_block_dist_m if self.scout_block_dist_m is not None else max_r
        if d_center < eff_block_dist:
            theta_shadow = math.atan2(self.scout_obs_width_m * 0.5, max(d_center, 0.05))
            straight_block_penalty = self.scout_straight_pen_k * max(0.0, (eff_block_dist - d_center) / eff_block_dist)

        # Relax steer alignment penalty when facing an obstacle or turn
        w_steer = self.w_align_steer if (abs(steer_current) > 0.08 and d_center >= 1.0) else 0.20

        scored_candidates = []
        for c in candidates:
            if c["dist"] < min_allowed_dist:
                continue

            d_cand = c["dist"]
            d_thresh = self.gap_dist_thresh_m
            dist_base = d_cand / max(max_clear_dist, 0.10)
            if d_cand > d_thresh:
                # Slight exponential boost for distances exceeding threshold (open track corridor towards progress)
                excess = d_cand - d_thresh
                dist_factor = dist_base * math.exp(self.gap_dist_exp_k * excess)
            else:
                dist_factor = dist_base

            width_norm = c["aperture"] / 1.00
            steer_diff = abs(c["steer_equiv"] - steer_current)
            straight_diff = abs(c["rel_angle"])

            # Neighboring ToF Zone Width Bonus (0.3x lowest cost / best neighboring zone clearance)
            neighbor_bonus = 0.0
            if tof_readings:
                # Find matching physical zone index for candidate angle
                rel_deg = math.degrees(c["rel_angle"])
                active_rig = FRONT_TOF_SENSOR_RIG
                if rel_deg >= 0.0:
                    z_idx = min(7, max(0, int(round((rel_deg - 1.125) / 11.25))))
                else:
                    z_idx = min(15, max(8, 8 + int(round((-rel_deg - 1.125) / 11.25))))
                
                left_neighbor_dist = tof_readings.get(active_rig[z_idx - 1][0], 0.0) if (z_idx > 0 and z_idx != 8) else 0.0
                right_neighbor_dist = tof_readings.get(active_rig[z_idx + 1][0], 0.0) if (z_idx < 15 and z_idx != 7) else 0.0
                min_neighbor_dist = max(left_neighbor_dist, right_neighbor_dist)
                neighbor_norm = min(TOF_MAX_RANGE_M, min_neighbor_dist) / TOF_MAX_RANGE_M
                neighbor_bonus = 0.30 * self.w_gap_width * neighbor_norm

            # Side clearance alignment bonus:
            # c["rel_angle"] < 0 is left. -sin(rel_angle) > 0 for left targets.
            # If side_diff > 0 (left has longer reading), left targets receive a strong bonus!
            side_bonus = self.w_side_clearance * side_diff * (-math.sin(c["rel_angle"]))

            # Penalize center/straight targets within dynamic angular shadow cone
            center_penalty = 0.0
            if straight_block_penalty > 0.0 and theta_shadow > 0.001 and abs(c["rel_angle"]) < theta_shadow:
                angle_ratio = 1.0 - (abs(c["rel_angle"]) / theta_shadow)
                center_penalty = straight_block_penalty * math.exp(self.scout_straight_exp_k * angle_ratio)

            clear_bonus = self.w_gap_clear if c.get("is_clear", False) else 0.0

            score = (
                self.w_gap_distance * dist_factor
                + self.w_gap_width * width_norm
                + neighbor_bonus
                + side_bonus
                + clear_bonus
                - center_penalty
                - w_steer * steer_diff
                - w_straight * straight_diff
            )
            scored_candidates.append((score, c))

        scored_candidates.sort(key=lambda item: item[0], reverse=True)

        # ----------------------------------------------------------------------
        # 4. SMART DUAL-PATH SELECTION (Primary Racing Line + 1 Smart Challenger)
        # ----------------------------------------------------------------------
        targets = []
        if scored_candidates:
            best_score, c0 = scored_candidates[0]
            tx0, ty0, ap0 = c0["tx"], c0["ty"], c0["aperture"]
            targets.append((tx0, ty0, ap0, bool(c0.get("is_clear", False))))

            if top_k >= 2:
                # Target 1: Smart Challenger Line
                # Select the best distinct corridor opening on the opposite side of
                # the obstacle / centerline or separated by at least alt_min_separation_m.
                # The challenger is scored with alt_aperture_bias weighting safe clearance.
                challenger_candidates = []
                for _, c in scored_candidates:
                    tx, ty, ap = c["tx"], c["ty"], c["aperture"]
                    lat_sep = abs(tx - tx0)
                    ang_sep = abs(c["rel_angle"] - c0["rel_angle"])
                    is_opposite_side = (tx0 < -0.04 and tx > 0.04) or (tx0 > 0.04 and tx < -0.04)
                    min_ang_sep = math.radians(24.0)

                    # Must be separated by minimum distance AND not be an immediately adjacent zone
                    if (lat_sep >= self.alt_min_separation_m or is_opposite_side) and (ang_sep >= min_ang_sep or is_opposite_side):
                        # Compute challenger score with aperture bias
                        d_cand = c["dist"]
                        d_thresh = self.gap_dist_thresh_m
                        dist_base = d_cand / max(max_clear_dist, 0.10)
                        if d_cand > d_thresh:
                            dist_factor = dist_base * math.exp(self.gap_dist_exp_k * (d_cand - d_thresh))
                        else:
                            dist_factor = dist_base

                        width_norm = ap / 1.00
                        steer_diff = abs(c["steer_equiv"] - steer_current)
                        straight_diff = abs(c["rel_angle"])
                        side_bonus_alt = (self.w_side_clearance * 0.45) * side_diff * (-math.sin(c["rel_angle"]))
                        clear_bonus_alt = (self.w_gap_clear * self.alt_aperture_bias) if c.get("is_clear", False) else 0.0
                        center_penalty = 0.0
                        if straight_block_penalty > 0.0 and theta_shadow > 0.001 and abs(c["rel_angle"]) < theta_shadow:
                            angle_ratio = 1.0 - (abs(c["rel_angle"]) / theta_shadow)
                            center_penalty = straight_block_penalty * math.exp(self.scout_straight_exp_k * angle_ratio)

                        score_alt = (
                            self.w_gap_distance * dist_factor
                            + (self.w_gap_width * self.alt_aperture_bias) * width_norm
                            + side_bonus_alt
                            + clear_bonus_alt
                            - center_penalty
                            - w_steer * steer_diff
                            - w_straight * straight_diff
                        )
                        challenger_candidates.append((score_alt, c))

                if challenger_candidates:
                    challenger_candidates.sort(key=lambda item: item[0], reverse=True)
                    best_challenger = challenger_candidates[0][1]
                    targets.append((best_challenger["tx"], best_challenger["ty"], best_challenger["aperture"], bool(best_challenger.get("is_clear", False))))
                else:
                    # Fallback: next distinct candidate by spatial separation >= 15 cm
                    for _, c in scored_candidates:
                        tx, ty, ap = c["tx"], c["ty"], c["aperture"]
                        if math.hypot(tx - tx0, ty - ty0) >= 0.15:
                            targets.append((tx, ty, ap, bool(c.get("is_clear", False))))
                            break

                # Additional targets if top_k > 2 requested
                if top_k > 2:
                    for _, c in scored_candidates:
                        tx, ty, ap = c["tx"], c["ty"], c["aperture"]
                        if any(math.hypot(tx - ex, ty - ey) < 0.18 for (ex, ey, *_) in targets):
                            continue
                        targets.append((tx, ty, ap, bool(c.get("is_clear", False))))
                        if len(targets) >= top_k:
                            break

        # Fallback if no target met threshold
        if not targets:
            best_c = max(candidates, key=lambda x: x["dist"])
            targets = [(best_c["tx"], best_c["ty"], 0.40, bool(best_c.get("is_clear", False)))]

        return targets



=== FILE: brain/vehicle_config.json ===
{
  "vehicle": {
    "rear_axle_offset_m": -0.0700,
    "front_axle_offset_m": 0.0625,
    "track_rear_m": 0.1130,
    "track_front_m": 0.0980,
    "track_kingpin_m": 0.0620,
    "wheel_radius_m": 0.0275,
    "vehicle_mass_kg": 1.000,
    "ackermann_percent": 0.45,
    "max_servo_rad": 0.610865,
    "max_steering_rad": 0.562285,
    "servo_max_speed_rad_s": 22.00,
    "motor_dir": 1.0,
    "v_max_wheel_mps": 3.84,
    "v_max_cruise_mps": 3.84
  },
  "dynamics": {
    "tire_mu": 0.80,
    "gravity_g": 9.80665,
    "accel_fwd_ratio": 0.95,
    "brake_decel_ratio": 1.00,
    "lat_accel_ratio": 1.00,
    "slip_gate_speed_mps": 0.30,
    "slip_thresh_spin_rad_s": 0.45
  },
  "servo": {
    "pwm_pin": 21,
    "pwm_channel": 0,
    "pwm_freq_hz": 333,
    "pwm_resolution": 14,
    "pulse_min_us": 1239.0,
    "pulse_neutral_us": 1510.0,
    "pulse_max_us": 1781.0
  },
  "timing": {
    "tof_hz": 60.0,
    "pi_loop_hz": 60.0,
    "control_loop_freq_hz": 400,
    "sim_time_step_ms": 8.333333,
    "uart_baud": 921600
  },
  "costmap": {
    "grid_x_max": 1.25,
    "grid_x_min": -1.25,
    "grid_y_min": 0.00,
    "grid_y_max": 2.60,
    "grid_res": 0.015,
    "r_inflation_m": 0.075
  }
}

=== FILE: brain/vehicle_params.py ===
"""
AUTO-GENERATED FILE -- DO NOT EDIT DIRECTLY.
Generated by tools/sync_params.py from brain/vehicle_config.json
"""
import math

# ============================================================================== #
# 1. PHYSICAL VEHICLE & ACTUATOR SPECIFICATIONS                                  #
# ============================================================================== #
REAR_AXLE_OFFSET_M: float = -0.07
FRONT_AXLE_OFFSET_M: float = 0.0625
WHEELBASE: float = 0.1325
TRACK_REAR: float = 0.113
TRACK_FRONT: float = 0.098
TRACK_KINGPIN: float = 0.062
WHEEL_RADIUS: float = 0.0275
VEHICLE_MASS: float = 1.0

ACKERMANN_PERCENT: float = 0.45
MAX_SERVO_RAD: float = 0.610865
MAX_STEERING_RAD: float = 0.562285
SERVO_MAX_SPEED_RAD_S: float = 22.0

MOTOR_DIR: float = 1.0
V_MAX_WHEEL: float = 3.84
V_MAX_CRUISE: float = 3.84

# ============================================================================== #
# 2. TIRE & VEHICLE DYNAMICS LIMITS                                              #
# ============================================================================== #
TIRE_MU: float = 0.8
GRAVITY_G: float = 9.80665
A_MAX_BASE: float = 7.84532
ACCEL_FWD_RATIO: float = 0.95
BRAKE_DECEL_RATIO: float = 1.0
LAT_ACCEL_RATIO: float = 1.0
ACCEL_FWD_MAX: float = 7.453054
BRAKE_DECEL_MAX: float = 7.84532
LAT_ACCEL_MAX: float = 7.84532
R_MIN_TURN: float = 0.210271646984119
V_MIN_CRUISE: float = 1.2843863739223678
SLIP_GATE_SPEED_MPS: float = 0.3
SLIP_THRESH_SPIN_RAD_S: float = 0.45

# ============================================================================== #
# 3. DIGITAL STEERING SERVO PWM CONFIGURATION                                    #
# ============================================================================== #
SERVO_PWM_PIN: int = 21
SERVO_PWM_CHANNEL: int = 0
SERVO_PWM_FREQ_HZ: int = 333
SERVO_PWM_RESOLUTION: int = 14
SERVO_PULSE_MIN_US: float = 1239.0
SERVO_PULSE_NEUTRAL_US: float = 1510.0
SERVO_PULSE_MAX_US: float = 1781.0

# ============================================================================== #
# 4. TIMING & FREQUENCIES                                                        #
# ============================================================================== #
TOF_HZ: float = 60.0
PI_LOOP_HZ: float = 60.0
DT_PI_LOOP: float = 0.016666666666666666
CONTROL_LOOP_FREQ_HZ: int = 400
CONTROL_LOOP_DT_S: float = 0.0025
SIM_TIME_STEP_MS: float = 8.333333
UART_BAUD: int = 921600

# ============================================================================== #
# 5. COSTMAP BOUNDS & RESOLUTION                                                 #
# ============================================================================== #
GRID_X_MAX: float = 1.25
GRID_X_MIN: float = -1.25
GRID_Y_MIN: float = 0.0
GRID_Y_MAX: float = 2.6
GRID_RES: float = 0.015
R_INFLATION_M: float = 0.075



=== FILE: 2_pi4/README.md ===
# Raspberry Pi 4 — Autonomous Racing Brain & Companion Computer

The **Raspberry Pi 4 Model B** acts as the high-level brain and companion computer of the Folkrace vehicle. It is responsible for multi-zone Time-of-Flight (ToF) perception processing, dynamic obstacle costmap generation, Curvilinear SL-MPC trajectory optimization, and asynchronous live telemetry streaming.

> [!IMPORTANT]
> **Zero LiDAR Requirement**: The physical racing stack operates exclusively on solid-state Time-of-Flight perception. No rotating LiDAR, SLAM scan-matching, or Cartographer package is used on the active vehicle.

---

## 1. Multi-Core System Architecture

To prevent visualization, logging, or network I/O from causing jitter in the real-time path planner, workload execution is split across dedicated CPU cores:

```mermaid
flowchart LR
    subgraph PI4_CORES["Raspberry Pi 4 Quad-Core Broadcom BCM2711"]
        direction TB
        subgraph CORE_PLAN["Core 0 / 1: Real-Time Planning Pipeline"]
            TOF_IN["16-Zone ToF Input Parser<br/>(from ESP32 ToF @ 921.6k Baud)"]
            COSTMAP["Dynamic Polar/Cartesian Costmap<br/>(Ray Clearing & Obstacle Stamping)"]
            SL_MPC["Curvilinear SL-MPC Solver<br/>(C++ OSQP Quadratic Program @ 20-50 Hz)"]
            CMD_OUT["Velocity & Steer Packet Dispatch<br/>(to ESP32 Main @ 921.6k Baud)"]
            
            TOF_IN --> COSTMAP --> SL_MPC --> CMD_OUT
        end

        subgraph CORE_DEBUG["Core 2 / 3: Telemetry & Debugging Server"]
            IMU_IN["IMU Telemetry Parser<br/>(from ESP32 Main @ 100 Hz)"]
            WEB_SRV["Asynchronous Web / WebSocket Server<br/>(Live Tactical Radar, Thought Matrix, HUD)"]
            LOGGER["High-Rate Telemetry Flight Recorder"]

            IMU_IN --> WEB_SRV
            SL_MPC -.->|"State & Trajectory"| WEB_SRV
            WEB_SRV --> LOGGER
        end
    end
```

### Core Allocation:
* **Core 0 & 1 (Deterministic Control & Planning)**:
  * Ingests 16-zone distance readings from the ESP32 ToF Hub over high-speed UART.
  * Updates the dynamic vehicle costmap with spatial clustering and safe ray clearing.
  * Solves the **Curvilinear SL-MPC** (Successive Linearization Model Predictive Control) optimization problem formulated as a Quadratic Program (QP) and solved via OSQP in C++.
  * Generates optimal forward velocity setpoints and steering angle commands, transmitting them to the ESP32 Main chassis controller.
* **Core 2 & 3 (Diagnostics & Telemetry)**:
  * Hosts the local debugging server (HTTP / WebSockets / UDP) for real-time live telemetry display.
  * Displays the 2D tactical radar, vehicle heading, slip alerts, and MPC prediction horizon on mobile/laptop browsers.
  * High-speed logging to flash storage without interrupting the real-time planner.

---

## 2. Hardware Interfaces & Data Flows

### A. ESP32 ToF Hub Link (Perception Ingestion)
* **Protocol**: High-speed binary packet stream over UART @ 921,600 baud.
* **Payload**: 16 filtered distance readings (in millimeters) originating from the 4x VL53L8CX multizone sensors.
* **Update Rate**: 30 to 60 Hz.
* **Purpose**: Feeds the local costmap, detecting track barriers, corners, dynamic opponents, and opening gaps.

### B. ESP32 Main Link (Dynamics & IMU Master)
* **Protocol**: Bidirectional high-speed UART @ 921,600 baud.
* **Downlink (Pi 4 -> ESP32 Main)**:
  * Commanded vehicle speed `v` (in m/s).
  * Commanded Ackermann virtual steering angle `delta` (in radians).
  * System operational flags (run, pause, emergency brake, recovery mode).
* **Uplink (ESP32 Main -> Pi 4)**:
  * 100 Hz binary IMU packet (quaternions `w, x, y, z`, 3-axis accelerometer, 3-axis gyroscope).
  * Individual wheel shaft RPMs and electronic differential status.
  * Traction status, slip detection flags, and watchdog counters.

---

## 3. Path Planning: Curvilinear SL-MPC

The path planning engine is based on Curvilinear Successive Linearization Model Predictive Control:

* **State Space**: Formulated in the Frenet frame relative to track progress:
  * `s`: Arc length distance along the track reference line.
  * `e_y`: Lateral deviation from the track center.
  * `e_psi`: Heading angle error relative to the track tangent.
  * `v`: Longitudinal vehicle velocity.
* **Constraints**:
  * Track boundary limits derived from the 16-zone ToF distance measurements.
  * Maximum front wheel steering angle limit (plus or minus 30 degrees / 0.5236 rad).
  * Steering rate limits for smooth servo actuation.
  * Maximum wheel RPM limits and lateral acceleration friction circle bounds (`a_y <= mu * g`).
* **Obstacle Avoidance**:
  * Dynamic obstacle envelopes are projected into the Frenet frame.
  * Hard and soft clearance buffers ensure collision-free trajectories while maximizing apex speed.

---

## 4. Raspberry Pi 4 Pin Assignment

The Pi 4 communicates with the microcontrollers using its onboard hardware UART peripherals on the 40-pin GPIO header:

| Header Pin | Broadcom GPIO | Hardware Function | Connected Subsystem | Role |
| :--- | :--- | :--- | :--- | :--- |
| **Pin 8** | GPIO 14 | UART0 TX | Host PC / Debug Console | Primary Linux Serial Console |
| **Pin 10** | GPIO 15 | UART0 RX | Host PC / Debug Console | Primary Linux Serial Console |
| **Pin 32** | GPIO 12 | UART5 TX | ESP32 Main UART RX | Command down-link (v, delta) |
| **Pin 33** | GPIO 13 | UART5 RX | ESP32 Main UART TX | High-speed IMU & chassis uplink |
| **Pin 36** | GPIO 16 | UART3 RX | ESP32 ToF Hub UART TX | Ingestion of 16-zone ToF distance stream |
| **Pin 37** | GPIO 26 | GPIO Out | Hardware Sync / Reset | Optional hardware reset line |
| **Pin 6, 14, 20** | GND | Ground | Common Ground | Ground plane reference |

---

## 5. Software Execution & Tooling

To run the dual-core path planning and debug server on the Pi 4:

* **Dual-Core System Launcher (Automatic Hardware Core Pinning)**:
  ```bash
  python3 2_pi4/run_folkrace_system.py
  ```
  * Pins Path Planner strictly to **CPU Core 1**.
  * Pins Debug Server & SQLite Black Box strictly to **CPU Core 2**.
  * Starts Web Mission Control GUI at `http://robot:8080`.

* **Standalone Path Planner (Core 1 Dedicated)**:
  ```bash
  python3 2_pi4/path_planner_runner.py
  ```

* **Standalone Debug Server & Black Box Recorder (Core 2 Dedicated)**:
  ```bash
  python3 2_pi4/debug_server.py
  ```

* **Host PC Studio Launcher**:
  ```bash
  python 1_launchers/4_pi4_debug_studio.py
  ```

* **Offline 60 Hz Flight Log Replay**:
  ```bash
  python 1_launchers/5_local_flight_replay.py
  ```

* **Webots Champion Preview & Telemetry**:
  ```bash
  python 1_launchers/1_champion_preview.py
  ```
* **Offline Planner Validation Suite**:
  ```bash
  python 5_webots/simulation/my_project/controllers/tof_path_planner/test_planners_offline.py
  ```
* **Track Studio (Web Design UI)**:
  ```bash
  python 1_launchers/10_track_studio.py
  ```


=== FILE: 2_pi4/auto_servo_center.py ===
#!/usr/bin/env python3
"""
================================================================================
  FOLKRACE PI 4: AUTOMATIC SERVO CENTER CALIBRATION
================================================================================
  Closed-loop servo neutral trim detector:
    1. Runs pre-flight checks (planner idle, MCU/IMU/STM32 online, fresh data,
       no fake data, and sufficient front ToF clearance).
    2. Disables ESP32 GYRO countersteer and Rear TV during the straight-line test
       (restored unconditionally in finally/stop) and locks steering center.
    3. Measures stationary IMU gyro_z bias with movement & noise checks.
    4. Drives forward and backward at half-MU acceleration & low cruise speed,
       actively braking to a verified standstill after each leg and matching the
       return distance to the exact forward distance (including brake rollout).
    5. Rejects IMU bump outliers with MAD filtering and cancels residual gyro
       bias via exact unequal-speed forward/backward curvature averaging:
         kappa_err = (omega_fwd - omega_bwd) / (v_fwd + v_bwd)
    6. Uses a safeguarded secant / bracketed Illinois root-finder (no double
       integrator) to converge center_us within tolerance (<= 0.8 deg/m).
    7. Persists converged center to servo_trim.json and ESP32 NVS (TRIM_SAVE).
================================================================================
"""

import math
import os
import statistics
import sys
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(CURR_DIR)
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

try:
    from brain.vehicle_params import (
        GRAVITY_G,
        MAX_STEERING_RAD,
        SERVO_PULSE_MAX_US,
        SERVO_PULSE_MIN_US,
        SERVO_PULSE_NEUTRAL_US,
        TIRE_MU,
        WHEELBASE,
    )
except Exception:
    TIRE_MU = 0.80
    GRAVITY_G = 9.80665
    WHEELBASE = 0.1325
    MAX_STEERING_RAD = 0.562285
    SERVO_PULSE_NEUTRAL_US = 1510.0
    SERVO_PULSE_MIN_US = 1239.0
    SERVO_PULSE_MAX_US = 1781.0


BASE_SERVO_NEUTRAL_US = float(SERVO_PULSE_NEUTRAL_US)
US_PER_RAD = (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / (2.0 * MAX_STEERING_RAD)
VALID_TOF_STATUSES = {5, 6, 9}


def _median(vals: List[float]) -> float:
    return float(statistics.median(vals))


class _TelemetryTracker:
    """Deduplicates telemetry frames and extracts wrap-safe elapsed dt."""

    def __init__(
        self,
        get_telemetry: Callable[[], Dict[str, Any]],
        stale_timeout_s: float,
        now_fn: Callable[[], float] = time.monotonic,
    ):
        self._get_telemetry = get_telemetry
        self._stale_timeout_s = stale_timeout_s
        self._now = now_fn
        self._prev_key: Any = None
        self._has_key = False
        self._prev_mcu_us: Optional[int] = None
        self._last_fresh_wall_t = self._now()

    @staticmethod
    def _packet_key(pkt: Dict[str, Any]) -> Any:
        if pkt.get("seq") is not None or pkt.get("mcu_timestamp_us") is not None:
            return (pkt.get("seq"), pkt.get("mcu_timestamp_us"))
        if pkt.get("timestamp") is not None:
            return ("ts", pkt.get("timestamp"))
        return None

    def poll(self, now: Optional[float] = None) -> Tuple[Dict[str, Any], bool, float, bool]:
        """Returns (pkt, is_fresh, dt_s, is_stale)."""
        t_now = self._now() if now is None else now
        pkt = self._get_telemetry() or {}
        key = self._packet_key(pkt)

        if key is None:
            dt = max(0.002, min(0.10, t_now - self._last_fresh_wall_t))
            self._last_fresh_wall_t = t_now
            return pkt, True, dt, False

        if not self._has_key or key != self._prev_key:
            dt = max(0.002, min(0.10, t_now - self._last_fresh_wall_t))
            curr_mcu = pkt.get("mcu_timestamp_us")
            if self._has_key and curr_mcu is not None and self._prev_mcu_us is not None:
                delta_us = (int(curr_mcu) - int(self._prev_mcu_us)) & 0xFFFFFFFF
                dt_mcu = delta_us * 1e-6
                if 0.002 <= dt_mcu <= 0.10:
                    dt = dt_mcu
            self._prev_key = key
            self._has_key = True
            self._prev_mcu_us = int(curr_mcu) if curr_mcu is not None else None
            self._last_fresh_wall_t = t_now
            return pkt, True, dt, False

        is_stale = (t_now - self._last_fresh_wall_t) > self._stale_timeout_s
        return pkt, False, 0.0, is_stale


class AutoServoCenterCalibrator:
    """Runs the forward/backward automatic servo centering calibration loop."""

    DRIVE_DISTANCE_M = 1.00
    CRUISE_SPEED_MPS = 0.60
    HALF_MU_ACCEL_MPS2 = 0.5 * TIRE_MU * GRAVITY_G
    RAMP_ACCEL_MPS2 = min(1.50, HALF_MU_ACCEL_MPS2)
    BRAKE_DECEL_MPS2 = 1.40

    BIAS_SAMPLE_SEC = 0.50
    BIAS_POLL_SEC = 0.015
    SETTLE_STOP_SEC = 0.20
    STANDSTILL_SPEED_MPS = 0.025
    STANDSTILL_HOLD_SEC = 0.20
    BRAKE_TIMEOUT_SEC = 1.5
    BRAKE_FALLBACK_SEC = 0.25
    POST_TRIM_WAIT_SEC = 0.12
    CONTROL_DT_SEC = 0.02
    LEG_TIMEOUT_SEC = 6.0
    STALL_TIMEOUT_SEC = 1.5
    TELEM_STALE_SEC = 0.35

    MIN_STEADY_SPEED_MPS = 0.25
    CONVERGE_DRIFT_DEG_PER_M = 0.8
    MAX_ITERATIONS = 6

    MAX_STEP_US = 80.0
    MIN_CENTER_US = max(1100.0, SERVO_PULSE_MIN_US - 50.0)
    MAX_CENTER_US = min(1900.0, SERVO_PULSE_MAX_US + 50.0)
    MIN_FRONT_TOF_M = 0.25

    def __init__(
        self,
        get_telemetry: Callable[[], Dict[str, Any]],
        send_cmd: Callable[[bytes], bool],
        get_center_us: Callable[[], float],
        save_center_us: Callable[[float], float],
    ):
        self._get_telemetry = get_telemetry
        self._send_cmd = send_cmd
        self._get_center_us = get_center_us
        self._save_center_us = save_center_us

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._tof_required: bool = False

        self._status: Dict[str, Any] = {
            "running": False,
            "phase": "IDLE",
            "iteration": 0,
            "max_iterations": self.MAX_ITERATIONS,
            "distance_m": 0.0,
            "target_distance_m": self.DRIVE_DISTANCE_M,
            "cruise_speed_mps": self.CRUISE_SPEED_MPS,
            "tolerance_deg_per_m": self.CONVERGE_DRIFT_DEG_PER_M,
            "gyro_offset_rad_s": 0.0,
            "gyro_bias_rad_s": 0.0,
            "resid_bias_rad_s": 0.0,
            "drift_deg_per_m": 0.0,
            "fwd_drift_deg_per_m": 0.0,
            "bwd_drift_deg_per_m": 0.0,
            "servo_center_us": float(get_center_us()),
            "last_step_us": 0.0,
            "message": "Ready",
            "history": [],
        }

    def is_running(self) -> bool:
        with self._lock:
            return bool(self._status["running"])

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            out = dict(self._status)
            out["history"] = list(self._status["history"])
            return out

    def _now(self) -> float:
        return time.monotonic()

    def _sleep(self, seconds: float) -> None:
        time.sleep(seconds)

    def _update_status(self, **kwargs: Any) -> None:
        with self._lock:
            self._status.update(kwargs)

    @staticmethod
    def _read_speed_est(pkt: Dict[str, Any]) -> float:
        v_raw = pkt.get("v_est")
        if v_raw is None:
            v_raw = pkt.get("v_real")
        return float(v_raw or 0.0)

    def _min_front_tof_m(self, pkt: Dict[str, Any]) -> Optional[float]:
        if not pkt.get("mcu_tof_connected"):
            return None
        tof = pkt.get("tof_ranges") or pkt.get("tof_ranges_raw") or []
        if len(tof) < 12:
            return None
        statuses = pkt.get("tof_target0_status")
        front_vals: List[float] = []
        for idx in (0, 1, 2, 3, 8, 9, 10, 11):
            val = tof[idx]
            if val is None:
                continue
            fval = float(val)
            if 0.035 < fval < 4.5:
                front_vals.append(fval)
            elif (
                0.02 < fval <= 0.035
                and isinstance(statuses, (list, tuple))
                and idx < len(statuses)
                and int(statuses[idx]) in VALID_TOF_STATUSES
            ):
                front_vals.append(fval)
        return min(front_vals) if front_vals else None

    def _check_front_obstacle(self, pkt: Dict[str, Any], speed_mps: float) -> Optional[float]:
        min_front = self._min_front_tof_m(pkt)
        if min_front is None:
            return None
        stop_dist = max(
            self.MIN_FRONT_TOF_M,
            (speed_mps * speed_mps) / (2.0 * self.BRAKE_DECEL_MPS2) + 0.18,
        )
        return min_front if min_front < stop_dist else None

    def _preflight_check(self, pkt: Dict[str, Any], drive_dist_m: float) -> Optional[str]:
        if pkt.get("planner_running"):
            return "Stop Path Planner before running Auto Servo Center"
        if pkt.get("mode") == "ESTOP":
            return "Emergency stop active — clear ESTOP or RESET before Auto Servo Center"

        pkt_age = pkt.get("_pkt_age_s")
        if pkt_age is not None and float(pkt_age) > 0.5:
            return "MCU telemetry offline or stale — connect ESP32 Main before starting"

        if "mcu_main_connected" in pkt and pkt.get("mcu_main_connected") is False:
            return "ESP32 Main offline — check USB connection"

        flags = pkt.get("flags") or {}
        if isinstance(flags, dict):
            if flags.get("fake_data"):
                return "Disable Fake Data (Bench) mode before running Auto Servo Center"
            if "imu_online" in flags or "imu" in flags:
                imu_ok = flags.get("imu_online") if "imu_online" in flags else flags.get("imu")
                if not imu_ok:
                    return "BNO086 IMU offline — cannot measure yaw drift"
            if "stm32_front" in flags or "stm32_rear" in flags:
                if not flags.get("stm32_front") and not flags.get("stm32_rear"):
                    return "STM32 motor controllers offline — cannot drive wheels"

        min_front = self._min_front_tof_m(pkt)
        if min_front is not None:
            req_clearance = max(self.MIN_FRONT_TOF_M, min(drive_dist_m + 0.15, 2.20))
            if min_front < req_clearance:
                return f"Front obstacle too close ({min_front:.2f}m < {req_clearance:.2f}m required)"
        return None

    def start(
        self,
        max_iterations: Optional[int] = None,
        cruise_speed: Optional[float] = None,
        drive_distance_m: Optional[float] = None,
        tolerance_deg_per_m: Optional[float] = None,
        trim_offset_us: Optional[float] = None,
    ) -> Dict[str, Any]:
        with self._lock:
            if self._status["running"]:
                return {"started": False, "error": "Auto servo centering already running"}

            max_iters = int(max_iterations) if max_iterations is not None else self.MAX_ITERATIONS
            max_iters = max(2, min(12, max_iters))

            speed = float(cruise_speed) if cruise_speed is not None else self.CRUISE_SPEED_MPS
            speed = max(0.35, min(1.00, speed))

            drive_dist = float(drive_distance_m) if drive_distance_m is not None else self.DRIVE_DISTANCE_M
            drive_dist = max(0.50, min(2.00, drive_dist))

            tol_deg_m = float(tolerance_deg_per_m) if tolerance_deg_per_m is not None else self.CONVERGE_DRIFT_DEG_PER_M
            tol_deg_m = max(0.3, min(3.0, tol_deg_m))

            trim_off = float(trim_offset_us) if trim_offset_us is not None else 0.0
            trim_off = max(-60.0, min(60.0, trim_off))

            telem = self._get_telemetry() or {}
            err = self._preflight_check(telem, drive_dist)
            if err:
                return {"started": False, "error": err}

            self._tof_required = bool(telem.get("mcu_tof_connected"))
            self._stop_event.clear()
            base_center = float(self._get_center_us())
            initial_center = round(max(self.MIN_CENTER_US, min(self.MAX_CENTER_US, base_center + trim_off)), 1)

            self._status.update(
                {
                    "running": True,
                    "phase": "STARTING",
                    "iteration": 0,
                    "max_iterations": max_iters,
                    "distance_m": 0.0,
                    "target_distance_m": round(drive_dist, 2),
                    "cruise_speed_mps": round(speed, 2),
                    "tolerance_deg_per_m": round(tol_deg_m, 2),
                    "gyro_offset_rad_s": 0.0,
                    "gyro_bias_rad_s": 0.0,
                    "resid_bias_rad_s": 0.0,
                    "drift_deg_per_m": 0.0,
                    "fwd_drift_deg_per_m": 0.0,
                    "bwd_drift_deg_per_m": 0.0,
                    "servo_center_us": initial_center,
                    "last_step_us": 0.0,
                    "message": "Starting auto servo center...",
                    "history": [],
                }
            )
            self._thread = threading.Thread(
                target=self._run_worker,
                args=(initial_center, max_iters, speed, drive_dist, tol_deg_m),
                daemon=True,
                name="AutoServoCenter",
            )
            self._thread.start()
            return {"started": True, "servo_center_us": initial_center}

    def stop(self, reason: str = "Stopped by user") -> None:
        self._stop_event.set()
        if self.is_running():
            self._update_status(phase="ABORTED", message=reason)
        self._send_cmd(b"V 0.0\n")
        self._send_cmd(b"GYRO 1\n")
        self._send_cmd(b"TV 1\n")
        self._send_cmd(b"RESET\n")

    def _apply_trim(self, center_us: float, lock_steer: bool = False) -> float:
        clamped = round(max(self.MIN_CENTER_US, min(self.MAX_CENTER_US, center_us)), 1)
        trim_offset = max(-500.0, min(500.0, clamped - BASE_SERVO_NEUTRAL_US))
        if lock_steer:
            self._send_cmd(b"A 90.0\n")
        self._send_cmd(f"TRIM {trim_offset:.1f}\n".encode("utf-8"))
        return clamped

    def _measure_static_gyro_bias(self) -> Dict[str, Any]:
        self._update_status(phase="BIAS_CAL", message="Measuring stationary IMU gyro bias...")
        tracker = _TelemetryTracker(self._get_telemetry, self.TELEM_STALE_SEC, self._now)
        samples: List[float] = []
        t_end = self._now() + self.BIAS_SAMPLE_SEC

        while self._now() < t_end and not self._stop_event.is_set():
            pkt, is_fresh, _, is_stale = tracker.poll()
            if pkt.get("mode") == "ESTOP":
                return {"ok": False, "error": "Emergency stop active"}
            if is_stale:
                return {"ok": False, "error": "Telemetry stalled during gyro bias calibration"}
            if abs(self._read_speed_est(pkt)) > 0.05:
                return {"ok": False, "error": "Vehicle moving during stationary gyro bias calibration"}
            if is_fresh:
                samples.append(float(pkt.get("gyro_z") or 0.0))
            self._sleep(self.BIAS_POLL_SEC)

        if self._stop_event.is_set():
            return {"ok": False, "error": "Stopped by user"}
        if len(samples) < 4:
            return {"ok": False, "error": "Insufficient fresh IMU samples during gyro bias calibration"}

        ordered = sorted(samples)
        if len(ordered) >= 8:
            trim_n = max(1, len(ordered) // 10)
            trimmed = ordered[trim_n:-trim_n]
        else:
            trimmed = ordered

        mean_val = sum(trimmed) / len(trimmed)
        if len(trimmed) >= 2:
            var = sum((x - mean_val) ** 2 for x in trimmed) / len(trimmed)
            if math.sqrt(var) > 0.08:
                return {"ok": False, "error": "IMU gyro noisy or bumped during stationary bias calibration"}

        return {"ok": True, "bias": mean_val}

    def _active_brake_stop(self, direction: int, phase_name: str, base_dist_m: float, target_dist_m: float) -> Dict[str, Any]:
        self._update_status(phase=phase_name, message="Active braking to standstill...")
        self._send_cmd(b"BRAKE\n")

        tracker = _TelemetryTracker(self._get_telemetry, self.TELEM_STALE_SEC, self._now)
        brake_dist_m = 0.0
        t_start = self._now()
        last_brake_tx = t_start
        standstill_since: Optional[float] = None
        sent_v0_fallback = False

        while not self._stop_event.is_set():
            now = self._now()
            pkt, is_fresh, dt, is_stale = tracker.poll(now)
            if pkt.get("mode") == "ESTOP":
                return {"ok": False, "error": "Emergency stop active"}
            if is_stale:
                return {"ok": False, "error": f"Telemetry stalled during {phase_name}"}

            if (now - t_start) > self.BRAKE_FALLBACK_SEC and not sent_v0_fallback:
                v_cmd_pkt = pkt.get("v_cmd")
                if v_cmd_pkt is not None and abs(float(v_cmd_pkt)) > 0.02:
                    self._send_cmd(b"V 0.0\n")
                    sent_v0_fallback = True

            if not sent_v0_fallback and (now - last_brake_tx) >= 0.10:
                self._send_cmd(b"BRAKE\n")
                last_brake_tx = now

            v_est = self._read_speed_est(pkt)
            if is_fresh:
                v_signed = direction * v_est
                brake_dist_m += max(0.0, v_signed) * dt
                self._update_status(distance_m=round(base_dist_m + brake_dist_m, 3))

                if abs(v_est) <= self.STANDSTILL_SPEED_MPS:
                    if standstill_since is None:
                        standstill_since = now
                    elif (now - standstill_since) >= self.STANDSTILL_HOLD_SEC:
                        return {"ok": True, "brake_dist_m": brake_dist_m}
                else:
                    standstill_since = None

            if (now - t_start) > self.BRAKE_TIMEOUT_SEC:
                self._send_cmd(b"V 0.0\n")
                return {
                    "ok": False,
                    "error": f"{phase_name} failed to reach standstill within {self.BRAKE_TIMEOUT_SEC:.1f}s (v={v_est:.2f} m/s)",
                }

            self._sleep(self.CONTROL_DT_SEC)

        return {"ok": False, "error": "Stopped by user"}

    @staticmethod
    def _filter_steady_samples(samples: List[Tuple[float, float, float]]) -> Tuple[float, float, float]:
        """Applies MAD outlier rejection on gyro_z samples and returns (omega_avg, v_avg, dt_sum)."""
        omegas = [s[0] for s in samples]
        med = _median(omegas)
        abs_devs = [abs(w - med) for w in omegas]
        mad = _median(abs_devs)
        thresh = max(0.08, 3.5 * 1.4826 * mad)
        inliers = [s for s in samples if abs(s[0] - med) <= thresh]
        if not inliers:
            inliers = samples
        dt_sum = sum(s[2] for s in inliers)
        if dt_sum <= 0.0:
            return 0.0, 0.0, 0.0
        omega_avg = sum(s[0] * s[2] for s in inliers) / dt_sum
        v_avg = sum(s[1] * s[2] for s in inliers) / dt_sum
        return omega_avg, v_avg, dt_sum

    def _drive_leg(self, direction: int, speed_mps: float, target_dist_m: float, gyro_bias: float) -> Dict[str, Any]:
        """
        Drives forward (direction=+1) or backward (direction=-1) with trapezoidal
        half-MU ramp, active braking, MAD outlier rejection, and return-distance tracking.
        """
        phase = "FORWARD" if direction > 0 else "BACKWARD"
        self._update_status(
            phase=phase,
            distance_m=0.0,
            target_distance_m=round(target_dist_m, 2),
            message=f"Driving {'forward' if direction > 0 else 'backward'} {target_dist_m:.2f}m @ {speed_mps:.2f} m/s...",
        )

        tracker = _TelemetryTracker(self._get_telemetry, self.TELEM_STALE_SEC, self._now)
        dist_m = 0.0
        v_cmd = 0.0
        t_start = self._now()
        t_prev_cmd = t_start
        last_motion_t = t_start
        steady_samples: List[Tuple[float, float, float]] = []

        min_steady_v = max(self.MIN_STEADY_SPEED_MPS, 0.78 * speed_mps)
        steady_dist_min = min(0.20, 0.22 * target_dist_m)
        steady_rem_min = min(0.18, 0.20 * target_dist_m)

        while not self._stop_event.is_set():
            now = self._now()
            cmd_dt = max(0.002, min(0.10, now - t_prev_cmd))
            t_prev_cmd = now

            pkt, is_fresh, dt, is_stale = tracker.poll(now)
            if pkt.get("mode") == "ESTOP":
                return {"ok": False, "error": "Emergency stop active"}
            if is_stale:
                return {"ok": False, "error": f"Telemetry stalled during {phase}"}

            if direction > 0:
                if self._tof_required and (not pkt.get("mcu_tof_connected") or pkt.get("tof_front_ok") is False):
                    self._send_cmd(b"BRAKE\n")
                    return {"ok": False, "error": "Front ToF link lost during forward drive"}
                obs_dist = self._check_front_obstacle(pkt, max(abs(v_cmd), abs(self._read_speed_est(pkt))))
                if obs_dist is not None:
                    self._send_cmd(b"BRAKE\n")
                    return {"ok": False, "error": f"Obstacle detected ahead ({obs_dist:.2f}m)"}

            v_est = self._read_speed_est(pkt)
            v_signed = direction * v_est
            gyro_z = float(pkt.get("gyro_z") or 0.0) - gyro_bias
            slip_ratio = float(pkt.get("slip_ratio") or 0.0)

            if is_fresh:
                dist_m += max(0.0, v_signed) * dt
                self._update_status(distance_m=round(dist_m, 3))

            if abs(v_est) >= 0.05:
                last_motion_t = now
            elif (now - t_start) > 0.6 and (now - last_motion_t) > self.STALL_TIMEOUT_SEC:
                return {"ok": False, "error": "No wheel motion detected (check motors/battery/encoders)"}

            rem_m = max(0.0, target_dist_m - dist_m)
            v_brake_cap = math.sqrt(max(0.0, 2.0 * self.BRAKE_DECEL_MPS2 * rem_m))
            if dist_m >= target_dist_m or rem_m <= 0.025 or v_brake_cap <= 0.08:
                break
            if (now - t_start) > self.LEG_TIMEOUT_SEC:
                return {"ok": False, "error": f"{phase} leg timed out at {dist_m:.2f}m"}

            v_target = direction * min(speed_mps, max(0.12, v_brake_cap))
            is_accel = (abs(v_target) > abs(v_cmd)) and (v_target * v_cmd >= 0.0)
            max_dv = (self.RAMP_ACCEL_MPS2 if is_accel else self.BRAKE_DECEL_MPS2) * cmd_dt
            if v_target > v_cmd:
                v_cmd = min(v_target, v_cmd + max_dv)
            else:
                v_cmd = max(v_target, v_cmd - max_dv)

            self._send_cmd(f"V {v_cmd:.2f}\n".encode("utf-8"))

            if (
                is_fresh
                and v_signed >= min_steady_v
                and abs(v_signed - speed_mps) <= 0.20
                and dist_m >= steady_dist_min
                and rem_m >= steady_rem_min
                and slip_ratio <= 0.35
            ):
                steady_samples.append((gyro_z, v_signed, dt))

            self._sleep(self.CONTROL_DT_SEC)

        if self._stop_event.is_set():
            return {"ok": False, "error": "Stopped by user"}

        brake_phase = "BRAKE_FWD" if direction > 0 else "BRAKE_BWD"
        brake_res = self._active_brake_stop(direction, brake_phase, dist_m, target_dist_m)
        if not brake_res["ok"]:
            return brake_res

        total_dist_m = dist_m + float(brake_res["brake_dist_m"])
        if len(steady_samples) < 5:
            return {"ok": False, "error": f"Insufficient steady-state samples during {phase}"}

        omega_avg, v_avg, steady_dt_sum = self._filter_steady_samples(steady_samples)
        if steady_dt_sum < 0.20 or v_avg < self.MIN_STEADY_SPEED_MPS:
            return {"ok": False, "error": f"Insufficient steady-state cruise duration during {phase}"}

        return {
            "ok": True,
            "omega_avg": omega_avg,
            "v_avg": v_avg,
            "dist_m": dist_m,
            "total_dist_m": total_dist_m,
        }

    @classmethod
    def _compute_next_step_us(
        cls,
        center_us: float,
        kappa_err: float,
        prev_u: Optional[float],
        prev_k: Optional[float],
        u_pos: Optional[float],
        k_pos: Optional[float],
        u_neg: Optional[float],
        k_neg: Optional[float],
        w_pos: float,
        w_neg: float,
    ) -> Tuple[float, str]:
        """Safeguarded Illinois bracketed / secant / kinematic step (u_pos < u_neg)."""
        s_nom = -1.0 / (WHEELBASE * US_PER_RAD)

        if (
            u_pos is not None
            and u_neg is not None
            and k_pos is not None
            and k_neg is not None
            and u_pos < u_neg
            and k_pos > 0.0
            and k_neg < 0.0
        ):
            kp = w_pos * k_pos
            kn = w_neg * k_neg
            frac = kp / max(1e-9, kp - kn)
            frac = max(0.15, min(0.85, frac))
            u_next = u_pos + frac * (u_neg - u_pos)
            step_us = u_next - center_us
            method = "bracket"
        elif prev_u is not None and prev_k is not None and abs(center_us - prev_u) >= 3.0:
            slope = (kappa_err - prev_k) / (center_us - prev_u)
            if slope >= -0.20 * abs(s_nom):
                step_us = 0.85 * math.atan(kappa_err * WHEELBASE) * US_PER_RAD
                method = "kinematic"
            else:
                clamped_slope = max(-2.5 * abs(s_nom), min(-0.35 * abs(s_nom), slope))
                step_us = -0.85 * (kappa_err / clamped_slope)
                method = "secant"
        else:
            step_us = 0.85 * math.atan(kappa_err * WHEELBASE) * US_PER_RAD
            method = "kinematic"

        step_us = max(-cls.MAX_STEP_US, min(cls.MAX_STEP_US, step_us))
        if abs(step_us) < 0.5:
            step_us = 0.5 if kappa_err > 0.0 else -0.5
        return step_us, method

    def _run_worker(
        self,
        initial_center_us: float,
        max_iters: int,
        cruise_speed: float,
        drive_distance_m: float,
        tol_deg_per_m: float,
    ) -> None:
        center_us = initial_center_us
        history: List[Dict[str, Any]] = []

        prev_u: Optional[float] = None
        prev_k: Optional[float] = None
        u_pos: Optional[float] = None
        k_pos: Optional[float] = None
        u_neg: Optional[float] = None
        k_neg: Optional[float] = None
        w_pos = 1.0
        w_neg = 1.0
        last_side: Optional[str] = None
        same_side_count = 0

        try:
            self._send_cmd(b"MOTORS\n")
            self._send_cmd(b"GYRO 0\n")
            self._send_cmd(b"TV 0\n")
            center_us = self._apply_trim(center_us, lock_steer=True)
            self._sleep(self.POST_TRIM_WAIT_SEC)

            bias_res = self._measure_static_gyro_bias()
            if not bias_res["ok"]:
                if not self._stop_event.is_set():
                    self._update_status(phase="ERROR", message=bias_res["error"])
                return
            gyro_bias = float(bias_res["bias"])
            self._update_status(gyro_bias_rad_s=round(gyro_bias, 4))

            converged = False
            for it in range(1, max_iters + 1):
                if self._stop_event.is_set():
                    break

                self._update_status(iteration=it, servo_center_us=center_us)

                fwd = self._drive_leg(+1, cruise_speed, drive_distance_m, gyro_bias)
                if not fwd["ok"]:
                    if not self._stop_event.is_set():
                        self._update_status(phase="ERROR", message=fwd["error"])
                    return

                bwd_target_dist = max(0.35, min(drive_distance_m * 1.35, float(fwd["total_dist_m"])))
                bwd = self._drive_leg(-1, cruise_speed, bwd_target_dist, gyro_bias)
                if not bwd["ok"]:
                    if not self._stop_event.is_set():
                        self._update_status(phase="ERROR", message=bwd["error"])
                    return

                v_fwd = max(0.20, float(fwd["v_avg"]))
                v_bwd = max(0.20, float(bwd["v_avg"]))
                omega_fwd = float(fwd["omega_avg"])
                omega_bwd = float(bwd["omega_avg"])

                # Exact unequal-speed gyro-bias cancellation:
                # Forward (+v_fwd):  omega_fwd = +kappa_err * v_fwd + b_resid
                # Backward (-v_bwd): omega_bwd = -kappa_err * v_bwd + b_resid
                kappa_err = (omega_fwd - omega_bwd) / (v_fwd + v_bwd)
                b_resid = (v_bwd * omega_fwd + v_fwd * omega_bwd) / (v_fwd + v_bwd)

                drift_fwd_deg_per_m = math.degrees(omega_fwd / v_fwd)
                drift_bwd_deg_per_m = math.degrees(-omega_bwd / v_bwd)
                drift_deg_per_m = math.degrees(kappa_err)
                gyro_equiv = kappa_err * cruise_speed

                entry = {
                    "iter": it,
                    "omega_fwd": round(omega_fwd, 4),
                    "omega_bwd": round(omega_bwd, 4),
                    "v_fwd": round(v_fwd, 3),
                    "v_bwd": round(v_bwd, 3),
                    "fwd_drift_deg_per_m": round(drift_fwd_deg_per_m, 2),
                    "bwd_drift_deg_per_m": round(drift_bwd_deg_per_m, 2),
                    "drift_deg_per_m": round(drift_deg_per_m, 2),
                    "resid_bias_rad_s": round(b_resid, 4),
                    "center_us": round(center_us, 1),
                    "step_us": 0.0,
                    "method": "converged",
                }

                if abs(drift_deg_per_m) <= tol_deg_per_m:
                    history.append(entry)
                    saved = self._save_center_us(center_us)
                    self._apply_trim(saved)
                    self._sleep(0.05)
                    self._send_cmd(b"TRIM_SAVE\n")
                    self._sleep(0.05)
                    self._send_cmd(b"TRIM_GET\n")
                    self._update_status(
                        phase="CONVERGED",
                        gyro_offset_rad_s=round(gyro_equiv, 4),
                        resid_bias_rad_s=round(b_resid, 4),
                        drift_deg_per_m=round(drift_deg_per_m, 2),
                        fwd_drift_deg_per_m=round(drift_fwd_deg_per_m, 2),
                        bwd_drift_deg_per_m=round(drift_bwd_deg_per_m, 2),
                        servo_center_us=saved,
                        last_step_us=0.0,
                        history=history,
                        message=f"Converged in {it} tries! Center = {saved:.1f} us (drift {drift_deg_per_m:+.2f} deg/m)",
                    )
                    converged = True
                    break

                # Update root-finding bracket before computing step
                if kappa_err > 0.0:
                    u_pos, k_pos = center_us, kappa_err
                    if u_neg is not None and u_pos >= u_neg:
                        u_neg, k_neg = None, None
                        w_pos, w_neg = 1.0, 1.0
                        last_side, same_side_count = None, 0
                    else:
                        same_side_count = (same_side_count + 1) if last_side == "pos" else 1
                        last_side = "pos"
                        w_pos = 1.0
                        if same_side_count >= 2:
                            w_neg *= 0.5
                else:
                    u_neg, k_neg = center_us, kappa_err
                    if u_pos is not None and u_pos >= u_neg:
                        u_pos, k_pos = None, None
                        w_pos, w_neg = 1.0, 1.0
                        last_side, same_side_count = None, 0
                    else:
                        same_side_count = (same_side_count + 1) if last_side == "neg" else 1
                        last_side = "neg"
                        w_neg = 1.0
                        if same_side_count >= 2:
                            w_pos *= 0.5

                step_us, method = self._compute_next_step_us(
                    center_us=center_us,
                    kappa_err=kappa_err,
                    prev_u=prev_u,
                    prev_k=prev_k,
                    u_pos=u_pos,
                    k_pos=k_pos,
                    u_neg=u_neg,
                    k_neg=k_neg,
                    w_pos=w_pos,
                    w_neg=w_neg,
                )

                next_center = round(max(self.MIN_CENTER_US, min(self.MAX_CENTER_US, center_us + step_us)), 1)
                if abs(next_center - center_us) < 0.05:
                    entry["method"] = "limit"
                    history.append(entry)
                    self._update_status(
                        phase="ERROR",
                        history=history,
                        message=f"Servo trim reached hard limit ({center_us:.1f} us) before converging",
                    )
                    return

                prev_u = center_us
                prev_k = kappa_err
                applied_step = round(next_center - center_us, 1)
                entry["step_us"] = applied_step
                entry["method"] = method
                history.append(entry)

                center_us = self._apply_trim(next_center)
                self._update_status(
                    phase="ADJUSTING",
                    gyro_offset_rad_s=round(gyro_equiv, 4),
                    resid_bias_rad_s=round(b_resid, 4),
                    drift_deg_per_m=round(drift_deg_per_m, 2),
                    fwd_drift_deg_per_m=round(drift_fwd_deg_per_m, 2),
                    bwd_drift_deg_per_m=round(drift_bwd_deg_per_m, 2),
                    servo_center_us=center_us,
                    last_step_us=applied_step,
                    history=history,
                    message=(
                        f"Iter {it}/{max_iters} ({method}): drift {drift_deg_per_m:+.2f} deg/m "
                        f"-> trim {applied_step:+.1f} us (center {center_us:.1f} us)"
                    ),
                )
                self._sleep(self.POST_TRIM_WAIT_SEC)

            if not converged and not self._stop_event.is_set() and history:
                best = min(history, key=lambda h: abs(h["drift_deg_per_m"]))
                init_drift = abs(history[0]["drift_deg_per_m"])
                improved = abs(best["drift_deg_per_m"]) < init_drift
                within_soft_band = abs(best["drift_deg_per_m"]) <= (3.0 * tol_deg_per_m)

                if improved or within_soft_band:
                    saved = self._save_center_us(best["center_us"])
                    self._apply_trim(saved)
                    self._sleep(0.05)
                    self._send_cmd(b"TRIM_SAVE\n")
                    self._sleep(0.05)
                    self._send_cmd(b"TRIM_GET\n")
                    msg = (
                        f"Completed {max_iters} tries without meeting +/-{tol_deg_per_m:.1f} deg/m. "
                        f"Saved best = {saved:.1f} us (drift {best['drift_deg_per_m']:+.2f} deg/m)"
                    )
                    final_center = saved
                else:
                    self._apply_trim(initial_center_us)
                    final_center = initial_center_us
                    msg = (
                        f"Completed {max_iters} tries without improvement (best drift "
                        f"{best['drift_deg_per_m']:+.2f} deg/m). Restored initial {initial_center_us:.1f} us"
                    )

                self._update_status(
                    phase="DONE_BEST",
                    drift_deg_per_m=best["drift_deg_per_m"],
                    fwd_drift_deg_per_m=best["fwd_drift_deg_per_m"],
                    bwd_drift_deg_per_m=best["bwd_drift_deg_per_m"],
                    servo_center_us=final_center,
                    history=history,
                    message=msg,
                )
        except Exception as exc:
            self._update_status(phase="ERROR", message=f"Auto-center error: {exc}")
        finally:
            try:
                if not self._stop_event.is_set() and abs(self._read_speed_est(self._get_telemetry() or {})) > 0.05:
                    self._send_cmd(b"BRAKE\n")
                    last_brake_tx = self._now()
                    t_wait = last_brake_tx + 0.35
                    while self._now() < t_wait and not self._stop_event.is_set():
                        if abs(self._read_speed_est(self._get_telemetry() or {})) <= 0.05:
                            break
                        now_w = self._now()
                        if (now_w - last_brake_tx) >= 0.10:
                            self._send_cmd(b"BRAKE\n")
                            last_brake_tx = now_w
                        self._sleep(0.02)
            except Exception:
                pass
            self._send_cmd(b"V 0.0\n")
            self._send_cmd(b"GYRO 1\n")
            self._send_cmd(b"TV 1\n")
            self._send_cmd(b"RESET\n")
            self._update_status(running=False)


=== FILE: 2_pi4/benchmark_sl_mpc.py ===
#!/usr/bin/env python3
"""
================================================================================
  FOLKRACE PI 4: REAL-HARDWARE CURVILINEAR SL-MPC BENCHMARK
================================================================================
  Hardware Policy:
    * Strictly restricted to physical Raspberry Pi 4 Model B hardware (ARM64).
    * Execution on host workstation (Windows / x86_64) is disallowed.
    * When invoked from host workstation, it checks SSH connection to 'robot'.
      - If connected: syncs repository stack and runs benchmark remotely on Pi 4.
      - If disconnected: fails immediately and requests physical connection.
================================================================================
"""

import os
import sys
import time
import csv
import subprocess
import platform
from pathlib import Path
from typing import Dict, List, Optional, Tuple

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
if CURR_DIR not in sys.path:
    sys.path.insert(0, CURR_DIR)

WORKSPACE_ROOT = os.path.abspath(os.path.join(CURR_DIR, ".."))
if WORKSPACE_ROOT not in sys.path:
    sys.path.insert(0, WORKSPACE_ROOT)

PLANNER_DIR = os.path.join(WORKSPACE_ROOT, "5_webots", "simulation", "my_project", "controllers", "tof_path_planner")
if os.path.exists(PLANNER_DIR) and PLANNER_DIR not in sys.path:
    sys.path.insert(0, PLANNER_DIR)

SSH_HOST = os.environ.get("PI_SSH_HOST", "robot")
REMOTE_DIR = "/home/admin/folkrace"


def is_raspberry_pi() -> bool:
    """Verifies whether the current process is executing on physical Raspberry Pi hardware."""
    if sys.platform != "linux":
        return False

    # 1. Check device-tree model name
    for path in ("/sys/firmware/devicetree/base/model", "/proc/device-tree/model"):
        if os.path.exists(path):
            try:
                with open(path, "r", encoding="utf-8", errors="ignore") as f:
                    model = f.read()
                    if "Raspberry Pi" in model:
                        return True
            except Exception:
                pass

    # 2. Check cpuinfo hardware identifier
    if os.path.exists("/proc/cpuinfo"):
        try:
            with open("/proc/cpuinfo", "r", encoding="utf-8", errors="ignore") as f:
                content = f.read()
                if any(x in content for x in ("BCM2711", "BCM2835", "Raspberry Pi")):
                    return True
        except Exception:
            pass

    return False


def check_pi_ssh(host: str = SSH_HOST, timeout_s: int = 3) -> bool:
    """Checks if Raspberry Pi is reachable over SSH."""
    cmd = ["ssh", "-o", "BatchMode=yes", "-o", f"ConnectTimeout={timeout_s}", host, "echo ok"]
    try:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout_s + 1, text=True)
        return res.returncode == 0 and "ok" in res.stdout
    except Exception:
        return False


def run_remote_on_pi(host: str = SSH_HOST):
    """Synchronizes files and runs benchmark remotely on the Raspberry Pi 4 over SSH."""
    print("=" * 70)
    print("  [REMOTE DELEGATION] Running benchmark on Raspberry Pi 4...")
    print(f"  Target: {host} | Remote Directory: {REMOTE_DIR}")
    print("=" * 70)

    # 1. Create remote target directory
    print("[1/3] Preparing remote environment...")
    subprocess.run(["ssh", host, f"mkdir -p {REMOTE_DIR}/brain {REMOTE_DIR}/data"], check=True)

    # 2. Sync files
    print("[2/3] Syncing controller scripts, vehicle params, and dataset to Pi 4...")
    files_to_sync = [
        os.path.join(CURR_DIR, "benchmark_sl_mpc.py"),
        os.path.join(CURR_DIR, "build_so.sh"),
        os.path.join(PLANNER_DIR, "planner_config.py"),
        os.path.join(PLANNER_DIR, "planner_sl_mpc.py"),
        os.path.join(PLANNER_DIR, "cpp_smart_hybrid_costmap.py"),
        os.path.join(PLANNER_DIR, "tof_gap_scout.py"),
        os.path.join(PLANNER_DIR, "ramp_detector.py"),
    ]
    scp_cmd = ["scp"] + [f for f in files_to_sync if os.path.exists(f)] + [f"{host}:{REMOTE_DIR}/"]
    subprocess.run(scp_cmd, check=True)

    # Sync brain/ vehicle config & params
    brain_dir = os.path.join(WORKSPACE_ROOT, "brain")
    brain_sync = [
        os.path.join(brain_dir, name)
        for name in ("__init__.py", "vehicle_config.json", "vehicle_params.py", "mcu_telemetry_protocol.h")
        if os.path.isfile(os.path.join(brain_dir, name))
    ]
    if brain_sync:
        subprocess.run(["scp"] + brain_sync + [f"{host}:{REMOTE_DIR}/brain/"], check=True)

    # Sync dataset if available
    dataset_src = os.path.join(PLANNER_DIR, "data", "datasets", "tof_readings_cube_10cm.csv")
    if os.path.exists(dataset_src):
        subprocess.run(["scp", dataset_src, f"{host}:{REMOTE_DIR}/data/"], check=True)

    # 3. Execute remote benchmark
    print("[3/3] Executing benchmark natively on Raspberry Pi 4...")
    print("-" * 70)
    remote_exec = f"cd {REMOTE_DIR} && python3 benchmark_sl_mpc.py"
    res = subprocess.run(["ssh", "-t", host, remote_exec])
    sys.exit(res.returncode)


def print_disconnected_error(host: str):
    """Prints a clear error message when Pi 4 is unreachable."""
    print("=" * 70)
    print("  [FATAL ERROR] RASPBERRY PI 4 IS NOT CONNECTED")
    print("=" * 70)
    print(f"  2_pi4/benchmark_sl_mpc.py is restricted to physical Raspberry Pi 4 hardware.")
    print(f"  Current machine: {sys.platform} ({platform.machine()}) -- Execution rejected.")
    print()
    print(f"  SSH Target : {host} (admin@{host})")
    print(f"  Status     : UNREACHABLE / DISCONNECTED")
    print()
    print("  ACTIONS REQUIRED:")
    print("  1. Connect Raspberry Pi 4 to power and network (Ethernet / Wi-Fi).")
    print("  2. Verify Pi 4 is accessible: ping robot or ssh robot")
    print("  3. Re-run this benchmark once the Pi 4 connection is established.")
    print("=" * 70)
    sys.exit(1)


# ==============================================================================
# PHYSICAL PI 4 BENCHMARK EXECUTION
# ==============================================================================

def load_dataset(csv_path: str, sensor_names: List[str]):
    """Loads 16-channel ToF frames from recorded CSV file."""
    frames = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame = {}
            for k in sensor_names:
                if k in row:
                    frame[k] = float(row[k])
            if len(frame) == 16:
                frames.append(frame)
    return frames


def run_physical_pi_benchmark():
    import numpy as np
    from cpp_smart_hybrid_costmap import CppSmartHybridCostmap, SENSOR_NAMES
    from tof_gap_scout import ToFGapScout
    from planner_sl_mpc import CurvilinearSLMPC, CPP_AVAILABLE
    from planner_config import (
        GRID_X_MIN,
        GRID_X_MAX,
        GRID_Y_MIN,
        GRID_Y_MAX,
        GRID_RES,
        R_INFLATION_M,
    )

    print("=" * 70)
    print("  FOLKRACE PI 4: PHYSICAL HARDWARE CURVILINEAR SL-MPC BENCHMARK")
    print(f"  Host Architecture : {platform.machine()} | Kernel: {platform.release()}")
    print(f"  C++ Native SL-MPC Engine Available: {CPP_AVAILABLE}")
    print("=" * 70)

    if not CPP_AVAILABLE:
        print("[WARNING] C++ native engine not built on Pi 4. Compiling now via build_so.sh...")
        build_sh = os.path.join(CURR_DIR, "build_so.sh")
        if os.path.exists(build_sh):
            subprocess.run(["bash", build_sh], check=True)
            print("[INFO] Build complete, restarting benchmark...")
            os.execv(sys.executable, [sys.executable] + sys.argv)
        else:
            print("[ERROR] build_so.sh not found.")
            sys.exit(1)

    dataset_candidates = [
        os.path.join(CURR_DIR, "data", "tof_readings_cube_10cm.csv"),
        os.path.join(CURR_DIR, "tof_readings_cube_10cm.csv"),
        os.path.join(PLANNER_DIR, "data", "datasets", "tof_readings_cube_10cm.csv"),
    ]
    dataset_path = None
    for cand in dataset_candidates:
        if os.path.exists(cand):
            dataset_path = cand
            break

    if not dataset_path:
        print(f"[ERR] Dataset not found in any of: {dataset_candidates}")
        sys.exit(1)

    frames = load_dataset(dataset_path, SENSOR_NAMES)
    print(f"[INFO] Loaded {len(frames)} ToF frames from {os.path.basename(dataset_path)}")

    costmap = CppSmartHybridCostmap(
        grid_bounds=(GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX),
        resolution=GRID_RES,
        r_inflation_m=R_INFLATION_M,
    )
    gap_scout = ToFGapScout()
    planner = CurvilinearSLMPC(costmap_engine=costmap)
    planner.costmap = costmap

    v_curr = 2.50
    steer_curr = 0.0
    yaw_rate_curr = 0.0
    target_lookahead = 2.40

    # Warmup phase (50 iterations)
    print("\n[INFO] Running 50 warmup iterations on BCM2711 CPU...")
    for i in range(min(50, len(frames))):
        tof = frames[i]
        prob_map, hazard_mask, _ = costmap.process_tof_frame(tof)
        xg, yg, width, theta_g = gap_scout.scout_gap(
            tof_readings=tof,
            hazard_mask=hazard_mask,
            costmap_obj=costmap,
            target_lookahead_m=target_lookahead,
            steer_current=steer_curr,
            prob_map=prob_map,
        )
        _ = planner.plan(
            xg=xg,
            yg=yg,
            theta_g=theta_g,
            aperture_width=width,
            v_current=v_curr,
            steer_current=steer_curr,
            yaw_rate_current=yaw_rate_curr,
            extra_targets=getattr(gap_scout, "last_targets", None),
            t_frame_start=time.perf_counter(),
            frame_budget_ms=15.5,
        )

    # Benchmark Measurement across all frames
    print(f"[INFO] Benchmarking {len(frames)} frames...")
    costmap_times = []
    scout_times = []
    sl_mpc_times = []
    total_pipeline_times = []
    osqp_iterations = []
    cmd_speeds = []
    cmd_steers = []

    for i, tof in enumerate(frames):
        t_start_total = time.perf_counter_ns()

        t0 = time.perf_counter_ns()
        prob_map, hazard_mask, _ = costmap.process_tof_frame(tof)
        t_costmap_us = (time.perf_counter_ns() - t0) / 1000.0

        t0 = time.perf_counter_ns()
        xg, yg, width, theta_g = gap_scout.scout_gap(
            tof_readings=tof,
            hazard_mask=hazard_mask,
            costmap_obj=costmap,
            target_lookahead_m=target_lookahead,
            steer_current=steer_curr,
            prob_map=prob_map,
        )
        t_scout_us = (time.perf_counter_ns() - t0) / 1000.0

        t0 = time.perf_counter_ns()
        plan_res = planner.plan(
            xg=xg,
            yg=yg,
            theta_g=theta_g,
            aperture_width=width,
            v_current=v_curr,
            steer_current=steer_curr,
            yaw_rate_current=yaw_rate_curr,
            extra_targets=getattr(gap_scout, "last_targets", None),
            t_frame_start=t_start_total / 1e9,
            frame_budget_ms=15.5,
        )
        t_sl_mpc_us = (time.perf_counter_ns() - t0) / 1000.0

        t_total_us = (time.perf_counter_ns() - t_start_total) / 1000.0

        costmap_times.append(t_costmap_us)
        scout_times.append(t_scout_us)
        sl_mpc_times.append(t_sl_mpc_us)
        total_pipeline_times.append(t_total_us)

        osqp_iterations.append(plan_res.get("osqp_iters", 0))
        cmd_speeds.append(plan_res.get("cmd_speed_mps", 0.0))
        cmd_steers.append(plan_res.get("cmd_steer_rad", 0.0))

        cmd_steer = plan_res.get("cmd_steer_rad", 0.0)
        steer_curr = steer_curr + 0.3 * (cmd_steer - steer_curr)
        v_curr = max(0.5, min(5.0, plan_res.get("cmd_speed_mps", v_curr)))

    def stats_ms(arr_us):
        arr_ms = np.array(arr_us) / 1000.0
        return {
            "mean": np.mean(arr_ms),
            "median": np.median(arr_ms),
            "min": np.min(arr_ms),
            "max": np.max(arr_ms),
            "p95": np.percentile(arr_ms, 95),
            "p99": np.percentile(arr_ms, 99),
        }

    c_stats = stats_ms(costmap_times)
    s_stats = stats_ms(scout_times)
    m_stats = stats_ms(sl_mpc_times)
    t_stats = stats_ms(total_pipeline_times)

    print("\n" + "=" * 70)
    print("  PHYSICAL RASPBERRY PI 4 BENCHMARK RESULTS")
    print("=" * 70)
    print(f"{'Component':<32} {'Mean':<8} {'Median':<8} {'P95':<8} {'P99':<8} {'Max':<8}")
    print("-" * 70)
    print(f"{'1. C++ Costmap Update':<32} {c_stats['mean']:>6.3f}ms {c_stats['median']:>6.3f}ms {c_stats['p95']:>6.3f}ms {c_stats['p99']:>6.3f}ms {c_stats['max']:>6.3f}ms")
    print(f"{'2. ToF Gap Scout':<32} {s_stats['mean']:>6.3f}ms {s_stats['median']:>6.3f}ms {s_stats['p95']:>6.3f}ms {s_stats['p99']:>6.3f}ms {s_stats['max']:>6.3f}ms")
    print(f"{'3. C++ Curvilinear SL-MPC (OSQP)':<32} {m_stats['mean']:>6.3f}ms {m_stats['median']:>6.3f}ms {m_stats['p95']:>6.3f}ms {m_stats['p99']:>6.3f}ms {m_stats['max']:>6.3f}ms")
    print("-" * 70)
    print(f"{'TOTAL PLANNING PIPELINE':<32} {t_stats['mean']:>6.3f}ms {t_stats['median']:>6.3f}ms {t_stats['p95']:>6.3f}ms {t_stats['p99']:>6.3f}ms {t_stats['max']:>6.3f}ms")
    print("=" * 70)

    avg_hz = 1000.0 / t_stats['mean']
    p99_hz = 1000.0 / t_stats['p99']
    min_hz = 1000.0 / t_stats['max']
    sl_mpc_only_hz = 1000.0 / m_stats['mean']

    print(f"\n[RATES & FREQUENCIES (Physical Hardware)]")
    print(f"  * Curvilinear SL-MPC Alone:     {sl_mpc_only_hz:>6.1f} Hz (mean {m_stats['mean']:.3f} ms)")
    print(f"  * Full End-to-End Pipeline:    {avg_hz:>6.1f} Hz (mean {t_stats['mean']:.3f} ms)")
    print(f"  * 99th Percentile Guaranteed:  {p99_hz:>6.1f} Hz ({t_stats['p99']:.3f} ms)")
    print(f"  * Worst-Case Single Frame:     {min_hz:>6.1f} Hz ({t_stats['max']:.3f} ms)")

    print(f"\n[SOLVER METRICS]")
    print(f"  * OSQP Mean Iterations:         {np.mean(osqp_iterations):.1f}")
    print(f"  * OSQP Max Iterations:          {np.max(osqp_iterations)}")
    print(f"  * Mean Commanded Speed:         {np.mean(cmd_speeds):.2f} m/s")
    print(f"  * Steering Output Range:        [{np.min(cmd_steers):.3f}, {np.max(cmd_steers):.3f}] rad")
    print("\n" + "=" * 70)
    print("  PHYSICAL PI 4 BENCHMARK COMPLETED SUCCESSFULLY")
    print("=" * 70)


def main():
    if is_raspberry_pi():
        # Running directly on physical Raspberry Pi hardware
        run_physical_pi_benchmark()
    else:
        # Running from host workstation: check SSH connectivity
        print(f"[pi4_benchmark] Host is {sys.platform} ({platform.machine()}). Enforcing Pi 4 execution policy...")
        if check_pi_ssh(SSH_HOST):
            run_remote_on_pi(SSH_HOST)
        else:
            print_disconnected_error(SSH_HOST)


if __name__ == "__main__":
    main()


