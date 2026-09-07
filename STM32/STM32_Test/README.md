# STM32 Motor Controller Firmware

Firmware for the STM32G474CBT6 4-axis BLDC driver using PlatformIO and the SimpleFOC framework.

## Folder Organization

*   `platformio.ini`: PlatformIO configuration, including target board `genericSTM32G474CB` and compiler configuration.
*   `include/config.h`: Peripheral pinout definitions, limits, and UART packet structures.
*   `include/motor_axis.h` & `src/motor_axis.cpp`: High-level wrapper class for a single motor axis control loop.
*   `include/esp32_interface.h` & `src/esp32_interface.cpp`: Serial packet interface. Decodes command packets and responds with telemetry.
*   `include/safety.h` & `src/safety.cpp`: System safety watchdog, targets deceleration ramp, and active-low Status LED control.
*   `src/main.cpp`: Main setup and loop functions, routing ISR interrupts and managing execution frequencies.

## Clock & Peripheral Optimization
*   **Clock Configuration**: Runs clock-multiplied PLL (170MHz) off the internal HSI oscillator. LSE/HSE oscillators are disabled to allow PC14, PF0, and PF1 to function as digital inputs.
*   **JTAG Disabling**: Default JTAG registers are overwritten by setting pinModes for PA15, PB3, and PB4 inside `setup()`, freeing them for GPIO/Timer alternate functions while keeping SWD (`PA13`/`PA14`) active.
