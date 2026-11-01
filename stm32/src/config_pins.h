#ifndef CONFIG_PINS_H
#define CONFIG_PINS_H

#include <Arduino.h>

// ============================================================================
// DA WINNA — HARDWARE PINOUT & SYSTEM CONFIGURATION (STM32G474 LQFP48)
// ============================================================================
// Configured for Rear Axle (M3/M4 & Enc3/Enc4) on 2x10 Dual-Row Header
// Preserves Native USB (PA11/PA12) and BOOT0 (PB8) for USB DFU / Telemetry
// ============================================================================

// ----------------------------------------------------------------------------
// Shared SPI1 Bus (MT6816 Dual Magnetic Angle Encoders)
// ----------------------------------------------------------------------------
#ifndef PIN_SPI1_SCK
#define PIN_SPI1_SCK    PB3   // Clock (ENC1 SCK)
#endif

#ifndef PIN_SPI1_MISO
#define PIN_SPI1_MISO   PB4   // Master In (ENC1 MISO) - AF5
#endif

#ifndef PIN_SPI1_MOSI
#define PIN_SPI1_MOSI   PB5   // Master Out (ENC1 MOSI)
#endif

// Dedicated Chip Select Lines
#ifndef PIN_M1_CS
#define PIN_M1_CS       PA15  // Encoder 1 CS (SPI1)
#endif

#ifndef PIN_M2_CS
#define PIN_M2_CS       PB11  // Encoder 2 CS (SPI2 - held HIGH/deselected)
#endif

// ----------------------------------------------------------------------------
// Motor 1 / Rear Left (TIM3 @ 25 kHz Center-Aligned PWM)
// ----------------------------------------------------------------------------
#ifndef M1_PWM_A
#define M1_PWM_A        PA6   // TIM3_CH1 (Row 7, Right Inner)
#endif

#ifndef M1_PWM_B
#define M1_PWM_B        PA4   // TIM3_CH2 (Row 8, Right Inner)
#endif

#ifndef M1_PWM_C
#define M1_PWM_C        PB0   // TIM3_CH3 (Row 5, Right Outer)
#endif

#ifndef M1_EN
#define M1_EN           PB11  // M3 Gate Driver Enable (Row 3, Left Inner)
#endif

// ----------------------------------------------------------------------------
// Motor 2 / Rear Right (TIM1 @ 25 kHz Center-Aligned PWM)
// ----------------------------------------------------------------------------
#ifndef M2_PWM_A
#define M2_PWM_A        PA8   // TIM1_CH1 (Row 9, Right Inner)
#endif

#ifndef M2_PWM_B
#define M2_PWM_B        PA9   // TIM1_CH2 (Row 8, Right Outer)
#endif

#ifndef M2_PWM_C
#define M2_PWM_C        PA10  // TIM1_CH3 (Row 6, Right Inner)
#endif

#ifndef M2_EN
#define M2_EN           PB12  // M4 Gate Driver Enable (Row 3, Left Outer)
#endif

// ----------------------------------------------------------------------------
// Onboard Status LED & Diagnostics
// ----------------------------------------------------------------------------
#ifndef STATUS_LED
#define STATUS_LED      PC13  // Onboard User LED
#endif

// Ground Truth MT6816 PWM Diagnostics
#ifndef PIN_PWM_DIAG
#define PIN_PWM_DIAG    PA0   // Optional diagnostic PWM pin
#endif

// ----------------------------------------------------------------------------
// Motor & Control Engineering Parameters (GM3506 Optimized @ 12V)
// ----------------------------------------------------------------------------
#define MOTOR_POLE_PAIRS            11        // 24N22P Gimbal Motor
#define MOTOR_SUPPLY_VOLTAGE        12.0f     // 12V DC Supply
#define MOTOR_VOLTAGE_LIMIT         (12.0f / 1.7320508f) // 6.93V SpaceVectorPWM Linear Headroom
#define MOTOR_ALIGN_VOLTAGE         3.0f      // FOC Electrical Alignment Voltage
#define MOTOR_KV_RATING             130.0f    // GM3506 Back-EMF Feed-Forward
#define MOTOR_VELOCITY_LIMIT        150.0f    // 150 rad/s = 1432 RPM

// Stepped 128-Point Calibration Constants
#define CALIB_ALIGN_VOLTAGE         3.5f      // Stepped calibration movement voltage (V)
#define CALIB_HOLD_VOLTAGE          2.4f      // Thermal relief hold voltage during sample (V)
#define CALIB_TOTAL_STEPS           128       // 128-point LUT resolution
#define CALIB_OVERSAMPLE_COUNT      32        // Samples per step for noise rejection
#define LUT_ANOMALY_THRESHOLD_RAD   0.035f    // > 2.0 deg mech (~22 deg elec) = pole slip / stall
#define LUT_PKPK_LIMIT_RAD          0.080f    // > 4.6 deg mech = bad sweep / binding / severe runout

// Real-Time Control Loop Frequencies
#define FOC_COMMUTATION_HZ          20000     // 20.0 kHz Hard Real-Time Commutation (50.0 us)
#define MOTION_DOWNSAMPLE_RATIO     10        // Velocity PID downsampled to 2.0 kHz (500 us)
#define TELEMETRY_RATE_HZ           25        // 25 Hz Binary Telemetry Stream (40 ms)
#define SPI_SPEED_HZ                2656250   // 2.656 MHz (170 MHz / 64 prescaler) for high EMI noise immunity

// Pre-Loaded Zero Electrical Angles (Standalone Autonomous Startup Jerk Free)
#ifndef M1_DEFAULT_ZERO_ELECTRIC_ANGLE
#define M1_DEFAULT_ZERO_ELECTRIC_ANGLE 0.0f
#endif

#ifndef M2_DEFAULT_ZERO_ELECTRIC_ANGLE
#define M2_DEFAULT_ZERO_ELECTRIC_ANGLE 0.0f
#endif

#endif // CONFIG_PINS_H
