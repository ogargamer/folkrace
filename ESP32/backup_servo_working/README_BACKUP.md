# Working Servo Driver & Test Harness Backup

This directory contains a complete, verified backup of the working **ES08MDII Servo Control Subsystem** targeting **GPIO 21** on the **ESP32-S3**.

---

## Backup Contents

- `servo_driver.h`: Header file defining `ESP32MCPWMDriver` (using Espressif `driver/mcpwm.h`) and fallback `ESP32ServoDriver` (LEDC).
- `servo_driver.cpp`: Implementation utilizing native Espressif MCPWM silicon hardware for direct microsecond pulse generation.
- `main.cpp`: Standalone test application with 50Hz continuous sweeping, calibration sequence, and interactive serial commands at 921600 baud.

---

## Hardware Configuration

- **Target Pin**: GPIO 21 (Right Header, 5th pin from bottom on ESP32-S3 DevKitC-1).
- **PWM Peripheral**: MCPWM Unit 0, Timer 0 (50 Hz).
- **Pulse Limits**: `500 µs` (0°) to `2500 µs` (180°), Neutral `1500 µs` (90°).
- **Mandatory Power Rule**: External PSU GND must be connected to ESP32 GND (Common Ground).
