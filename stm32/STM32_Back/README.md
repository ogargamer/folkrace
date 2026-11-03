# STM32 Back Axle Motor Controller (STM32_Back)

## Overview
Controls the Rear/Back Axle GM3506 direct-drive brushless motors:
- **Motor 1 (M1)**: Rear-Left (RL)
- **Motor 2 (M2)**: Rear-Right (RR)

## Host ESP32 Connection
- **UART Port on ESP32**: UART2
- **ESP32 TX (GPIO 15)** -> STM32 Back RX (PA10)
- **ESP32 RX (GPIO 16)** <- STM32 Back TX (PA9)
- **Baud Rate**: 921,600 baud, 8N1

## FOC Calibration
- Stores rear axle calibration results in STM32_Back/gui/lut_calibrations.json and STM32_Back/motor_config_*.json.
- Flash persistence saves to STM32 Flash Page 63 on boot.
