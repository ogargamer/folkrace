# SimpleFOC MT6816 High-Speed Verified Build (v3 Backup - Updated)
Updated: 2026-08-20

## Summary
This snapshot captures the fully optimized, high-speed SimpleFOC closed-loop motor control suite with MT6816 SPI encoder support, SpaceVectorPWM modulation, BEMF feed-forward compensation, and verified 600+ RPM stability.

## Included Files
- `main.cpp`: Complete STM32G474 PlatformIO firmware (`encoder_test` target).
- `test_mt6818_ui.py`: Real-time Python GUI with high-speed 921600 baud streaming, batch log streaming, live PID adjustment, and 2-min auto-tune sweep.

## Core Firmware Configurations
- **Baud Rate**: 921600
- **Modulation**: SpaceVectorPWM (+15.5% linear voltage headroom over SinePWM)
- **SPI Interface**: SPI1 Master (PA5 SCK, PA7 MOSI/SDAT, PB4 MISO, PA3 CS) @ 4 MHz Mode 3
- **Motor**: GM3506 Gimbal Motor (11 pole pairs, 12.0V Supply, KV = 130 RPM/V)
- **Driver**: 3-PWM Inverter (PA8, PA9, PA10, PB3 EN) @ 25 kHz
- **Velocity Limit**: 150.0 rad/s (1432 RPM)
- **PID Defaults**:
  - P = 0.1200
  - I = 1.2000
  - D = 0.0010
  - LPF Tf = 0.0150 s (Low-latency filter for 110 Hz electrical frequency at 600+ RPM)
  - Voltage Limit = 12.0 V
  - Output Ramp = 1000.0 V/s
  - Align Voltage = 3.0 V
- **Stall Watchdog**: Trips if target >= 5 RPM but actual < 1 RPM for > 2.0 s.
