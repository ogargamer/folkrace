# Google Cartographer 2D SLAM & Hardware Pipeline Backup

**Backup Date**: August 18, 2026  
**Status**: FULLY STABLE & VERIFIED (0.1 mm/s stationary drift, ~100 Hz IMU stream, 10 Hz MS200 LiDAR)  
**Location**: `c:\Users\nurmi\Documents\Da Winna\backup_cartographer_complete_stable_2026_08_18/`

---

## 1. System Architecture & Data Flow

```
[ BNO08X IMU ] ──(Hardware SPI: CS=10, INT=9)──> [ esp32-S3 Master ]
                                                           │
                                                           │ (Hardware UART2 @ 921600 Baud)
                                                           │ Binary 42-Byte Frames (100 Hz)
                                                           ▼
[ MS200 LiDAR ] ──(UART4 @ 230400)──> [ Raspberry Pi 5 (ROS 2 Jazzy) ]
                                             ├─ bno08x_imu_node.py (/imu @ 100 Hz)
                                             ├─ ms200_laser_node.py (/scan @ 10 Hz)
                                             ├─ static_transform_publisher (base_link -> laser_frame, imu_link)
                                             ├─ cartographer_node (2D SLAM Engine)
                                             ├─ cartographer_occupancy_grid_node (/map @ 2 Hz, 2cm resolution)
                                             └─ slam_map_server.py (Tornado MultiThreaded WebSocket @ Port 8888)
                                                           │
                                                           │ (WebSocket + HTTP)
                                                           ▼
                                               [ Real-Time Web Dashboard ]
                                                 http://10.43.8.42:8888
```

---

## 2. Directory Structure of this Backup

```
backup_cartographer_complete_stable_2026_08_18/
├── pi4_cartographer/
│   ├── bno08x_imu_node.py         # ROS 2 IMU node (921600 Baud UART reader with bytearray accumulator)
│   ├── cartographer_2d.lua        # Google Cartographer 2D configuration (Ceres weights: occupied=10, trans=15, rot=50)
│   ├── cartographer_2d.launch.py  # ROS 2 launch description
│   ├── ms200_laser_node.py        # MS200 LiDAR ROS 2 LaserScan publisher (230400 Baud, 360-degree binning)
│   ├── run_cartographer_slam.py   # Multi-process supervisor launching TFs, LiDAR, IMU, Cartographer, Map Grid & Server
│   ├── slam_map_server.py         # Multi-threaded Tornado server with EMA smoothing and telemetry broadcasting
│   ├── ros2_carto_diagnostics.py  # Autonomous 10-second real-time drift measurement & health testing tool
│   └── static/
│       ├── slam_view.html         # Cyberpunk/glassmorphism SLAM user interface
│       ├── slam_view.css          # Modern dark-mode styling and sensor telemetry grid
│       └── slam_view.js           # 60 FPS Canvas rendering engine with pan, zoom, trajectory and LiDAR sweeps
├── esp32_firmware/
│   ├── platformio.ini             # PlatformIO build configuration for esp32s3_master
│   ├── src/
│   │   ├── main.cpp               # Master loop with 10ms (100Hz) high-precision UART2 IMU stream
│   │   ├── imu_bno08x.cpp         # Hardware SPI driver for SparkFun BNO080 library (Game Rotation + Accel + Gyro)
│   │   ├── stm32_link.cpp         # Non-blocking 400Hz BLDC UART link (guards against disconnected stm32 stalls)
│   │   ├── servo_driver.cpp       # Servo actuation
│   │   ├── slip_control.cpp       # Traction & slip control
│   │   └── suspension.cpp         # Active suspension control
│   └── include/
│       ├── config.h               # Pin assignments & baud rate constants
│       ├── imu_bno08x.h           # IMU data structures and method declarations
│       └── stm32_link.h           # stm32 packet definitions
└── PC_webots/tools/launchers/
    ├── launch_cartographer_gui.py # One-click SSH reboot recovery, auto-sync, process cleanup & browser launcher
    └── start_slam.py              # Lightweight standalone launcher
```

---

## 3. Verified Performance & Drift Benchmarks

Captured on Raspberry Pi 5 via `ros2_carto_diagnostics.py`:

```text
================================================================
          CARTOGRAPHER ROS 2 REAL-TIME DIAGNOSTIC REPORT
================================================================
Sample Duration: 10.0s
LiDAR /scan:  100 msgs (10.0 Hz) | Points: 334/360 | Range: 0.12m - 3.66m
IMU /imu:     965 msgs (96.5 Hz ~ 100 Hz Rock-Solid)
  Accel: (-0.28, 0.48, 9.82) m/s²
  Gyro:  (0.0, 0.0, 0.0) °/s
Map Updates:  20

--- ROBOT POSE DRIFT ANALYSIS (Cartographer map -> base_link) ---
Total Pose Updates: 111 (11.1 Hz)
Start Pose: X = -0.0014 m, Y = 0.0012 m, Yaw =  0.00°
End Pose:   X = -0.0021 m, Y = 0.0022 m, Yaw = -0.01°
Net Drift:  Distance = 0.0012 m (1.2 mm over 10 seconds)
Drift Rate: 0.01 cm/s (0.1 mm/s)
Yaw Drift:  0.05°

>> STATUS: [PASS] Perfectly stationary (< 0.01 cm/s) <<
================================================================
```

---

## 4. How to Restore

### To restore Pi 5 SLAM files:
Run `launch_cartographer_gui.py` on your PC — it automatically syncs all files in `pi4/cartographer/` over SFTP to `/home/admin/cartographer/` on the Pi 5.

### To restore esp32 firmware:
Connect esp32 via USB and execute:
```powershell
C:\Users\nurmi\.platformio\penv\Scripts\platformio.exe run -e esp32s3_master --target upload
```

### To start SLAM anytime:
```powershell
py -3 launch_cartographer_gui.py
```
Open **`http://10.43.8.42:8888`** in any browser.
