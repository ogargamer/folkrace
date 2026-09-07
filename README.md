# 4-Axis BLDC Motor Controller System

This project contains the firmware for a dual-MCU BLDC control system driving 4 independent BLDC motor axes (GM3506) with real-time feedback loop.

## Folder Organization

*   **[STM32_Controller/](file:///C:/Users/nurmi/Documents/Da%20Winna/STM32_Controller)**: Low-level C++ motor controller firmware written in PlatformIO/Arduino. Runs the unconstrained FOC commutation loops (>10kHz) and a throttled 1kHz kinematic loop on an STM32G474CBT6.
*   **[ESP32_Master/](file:///C:/Users/nurmi/Documents/Da%20Winna/ESP32_Master)**: Master kinematics and high-level control firmware running on an ESP32. Communicates with the STM32 at 400Hz using a high-speed duplex UART link at 921600 baud.

## System Pin Mapping (STM32G474CBT6 LQFP48)

| Pin | Function / Role | Peripheral | Channel / Mode | Notes |
|:---|:---|:---|:---|:---|
| **PA0** | Motor 2 PWM A | TIM2 | TIM2_CH1 | |
| **PA1** | Motor 2 PWM B | TIM2 | TIM2_CH2 | |
| **PA2** | Motor 2 PWM C | TIM2 | TIM2_CH3 | |
| **PA3** | Encoder 4 Ch A | GPIO | EXTI3 | |
| **PA4** | Motor 2 Enable | GPIO | Output | Active-High |
| **PA5** | Motor 1 Enable | GPIO | Output | Active-High |
| **PA6** | Encoder 3 Ch B | GPIO | EXTI6 | |
| **PA7** | Encoder 3 Ch A | GPIO | EXTI7 | |
| **PA8** | Motor 1 PWM A | TIM1 | TIM1_CH1 | |
| **PA9** | Motor 1 PWM B | TIM1 | TIM1_CH2 | |
| **PA10** | Motor 1 PWM C | TIM1 | TIM1_CH3 | |
| **PA11** | Motor 4 PWM A | TIM4 | TIM4_CH1 | |
| **PA12** | Motor 4 PWM B | TIM4 | TIM4_CH2 | |
| **PA13** | SWDIO | SYS | SWDIO | SWD Debug Data |
| **PA14** | SWCLK | SYS | SWCLK | SWD Debug Clock |
| **PA15** | Encoder 4 Ch B | GPIO | EXTI15 | Reclaimed JTAG pin |
| **PB0** | Motor 3 PWM C | TIM3 | TIM3_CH3 | |
| **PB1** | Motor 4 Enable | GPIO | Output | Active-High |
| **PB2** | Encoder 1 Ch A | GPIO | EXTI2 | |
| **PB3** | Free Pin | GPIO | - | Reclaimed JTAG pin (Unused) |
| **PB4** | Motor 3 PWM A | TIM3 | TIM3_CH1 | Reclaimed JTAG pin |
| **PB5** | Motor 3 PWM B | TIM3 | TIM3_CH2 | |
| **PB6** | ESP32 TX | USART1 | USART1_TX | Communications |
| **PB7** | ESP32 RX | USART1 | USART1_RX | Communications |
| **PB8** | Motor 4 PWM C | TIM4 | TIM4_CH3 | |
| **PB9** | Motor 3 Enable | GPIO | Output | Active-High |
| **PC13** | Status LED | GPIO | Output | Active-Low (Low current pin) |
| **PC14** | Encoder 2 Ch A | GPIO | EXTI14 | Reclaimed LSE pin |
| **PC15** | Free Pin | GPIO | - | Reclaimed LSE pin (Unused) |
| **PF0** | Encoder 2 Ch B | GPIO | EXTI0 | Reclaimed HSE pin |
| **PF1** | Encoder 1 Ch B | GPIO | EXTI1 | Reclaimed HSE pin |
