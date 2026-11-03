# STM32 Front Axle Motor Controller (STM32_Front)

## Overview
Controls the Front Axle GM3506 direct-drive brushless motors:
- **Motor 1 (M1)**: Front-Left (FL)
- **Motor 2 (M2)**: Front-Right (FR)

## Host ESP32 Connection
- **UART Port on ESP32**: UART1
- **ESP32 TX (GPIO 17)** -> STM32 Front RX (PA10)
- **ESP32 RX (GPIO 18)** <- STM32 Front TX (PA9)
- **Baud Rate**: 921,600 baud, 8N1

## FOC Calibration
- Stores front axle calibration results in STM32_Front/gui/lut_calibrations.json and STM32_Front/motor_config_*.json.
- Flash persistence saves to STM32 Flash Page 63 on boot.
