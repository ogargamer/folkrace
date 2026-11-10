#ifndef CONFIG_PINS_H
#define CONFIG_PINS_H

#include <Arduino.h>

// ============================================================================
// FOLKRACE — HARDWARE PINOUT & SYSTEM CONFIGURATION (STM32G474 LQFP48)
// ============================================================================
// Hardware Pinout matching exact pin solver / hardware assignments:
// M1: TIM2 (PA5, PA1, PA2), M1_EN: PA3
// M2: TIM3 (PA6, PA4, PB0), M2_EN: PA7
// Enc1: SPI1 (PB3 SCK, PB4 MISO, PB5 MOSI), CS: PA15
// Enc2: SPI2 (PB13 SCK, PB14 MISO, PB15 MOSI), CS: PB11
// Telemetry: USART1 (PA9 TX, PA10 RX)
// Native USB: PA11 (DM), PA12 (DP)
// ============================================================================

// ----------------------------------------------------------------------------
// SPI1 Bus (MT6816 Encoder 1)
// ----------------------------------------------------------------------------
#ifndef PIN_SPI1_SCK
#define PIN_SPI1_SCK PB3 // ENC1_SCK (SPI1 Clock)
#endif

#ifndef PIN_SPI1_MISO
#define PIN_SPI1_MISO PB4 // ENC1_MISO (SPI1 MISO)
#endif

#ifndef PIN_SPI1_MOSI
#define PIN_SPI1_MOSI PB5 // ENC1_MOSI (SPI1 MOSI)
#endif

#ifndef PIN_M1_CS
#define PIN_M1_CS PA15 // ENC1_CS (SPI1 Chip Select)
#endif

// ----------------------------------------------------------------------------
// SPI2 Bus (MT6816 Encoder 2 - Independent Hardware SPI2 on AF5)
// ----------------------------------------------------------------------------
// CRITICAL PIN MAPPING (STM32G474 AF5):
//   PB13 = SPI2_SCK  (Clock Output to MT6816 CLK)
//   PB14 = SPI2_MISO (Data Input from MT6816 DO)
//   PB15 = SPI2_MOSI (Data Output to MT6816 DI)
//   PB11 = Software Chip Select (CSN)
// STM32duino constructor argument order is: SPIClass(MOSI, MISO, SCK)
//   SPIClass SPI_2(PIN_SPI2_MOSI, PIN_SPI2_MISO, PIN_SPI2_SCK);
// ----------------------------------------------------------------------------
#ifndef PIN_SPI2_SCK
#define PIN_SPI2_SCK PB13 // ENC2_SCK (SPI2 Clock, AF5)
#endif

#ifndef PIN_SPI2_MISO
#define PIN_SPI2_MISO PB14 // ENC2_MISO (SPI2 MISO, AF5)
#endif

#ifndef PIN_SPI2_MOSI
#define PIN_SPI2_MOSI PB15 // ENC2_MOSI (SPI2 MOSI, AF5)
#endif

#ifndef PIN_M2_CS
#define PIN_M2_CS PB11 // ENC2_CS (SPI2 Chip Select)
#endif

// ----------------------------------------------------------------------------
// Motor 1 / TIM2 @ 25 kHz Center-Aligned PWM
// ----------------------------------------------------------------------------
#ifndef M1_PWM_A
#define M1_PWM_A PA5 // M1_PWM1 (TIM2 Phase A)
#endif

#ifndef M1_PWM_B
#define M1_PWM_B PA1 // M1_PWM2 (TIM2 Phase B)
#endif

#ifndef M1_PWM_C
#define M1_PWM_C PA2 // M1_PWM3 (TIM2 Phase C)
#endif

#ifndef M1_EN
#define M1_EN PA3 // M1_EN (Motor 1 Driver Enable)
#endif

// ----------------------------------------------------------------------------
// Motor 2 / TIM3 @ 25 kHz Center-Aligned PWM
// ----------------------------------------------------------------------------
#ifndef M2_PWM_A
#define M2_PWM_A PB0 // M2_PWM1 (TIM3 Phase A / CH1)
#endif

#ifndef M2_PWM_B
#define M2_PWM_B PA4 // M2_PWM2 (TIM3 Phase B / CH2)
#endif

#ifndef M2_PWM_C
#define M2_PWM_C PA6 // M2_PWM3 (TIM3 Phase C / CH3)
#endif

#ifndef M2_EN
#define M2_EN PA7 // M2_EN (Motor 2 Driver Enable)
#endif

// ----------------------------------------------------------------------------
// Onboard Status LED & Diagnostics
// ----------------------------------------------------------------------------
#ifndef STATUS_LED
#define STATUS_LED PA8 // Onboard User LED
#endif

#ifndef USER_BUTTON
#define USER_BUTTON PC13 // Onboard User Button (KEY)
#endif

// Ground Truth MT6816 PWM Diagnostics
#ifndef PIN_PWM_DIAG
#define PIN_PWM_DIAG PA0 // Optional diagnostic PWM pin
#endif

// ----------------------------------------------------------------------------
// Motor & Control Engineering Parameters (GM3506 Optimized @ 12V)
// ----------------------------------------------------------------------------
#define MOTOR_POLE_PAIRS 11        // 24N22P Gimbal Motor
#define MOTOR_SUPPLY_VOLTAGE 12.0f // 12V DC Supply
#define MOTOR_VOLTAGE_LIMIT 12.00f // 12.00V Full Inverter Headroom
#define MOTOR_ALIGN_VOLTAGE 3.0f   // FOC Electrical Alignment Voltage
#define MOTOR_KV_RATING 130.0f     // GM3506 Back-EMF Feed-Forward
#define MOTOR_VELOCITY_LIMIT                                                   \
  (2000.0f * 0.104719755f) // 2000 RPM = 209.44 rad/s headroom for 1800 RPM

// Default Motion Control & Sensor Parameters
#define DEFAULT_PID_P 0.10f
#define DEFAULT_PID_I 1.50f
#define DEFAULT_PID_D 0.00000f
#define DEFAULT_PID_RAMP 100.0f
#define DEFAULT_LPF_TF 0.0005f
#define DEFAULT_LAG_COMP_US 35.0f
#define DEFAULT_DWT_WINDOW_MS 5.0f

// Direction Convention Configuration
// Set to 1 to invert raw encoder counting direction globally in firmware so
// physical clockwise rotation increases angle & velocity (CW = positive)
#ifndef INVERT_ENCODER_DIRECTION
#define INVERT_ENCODER_DIRECTION 0
#endif

// Stepped 128-Point Calibration Constants (Matching ESP32 Suite)
#define CALIB_ALIGN_VOLTAGE 4.00f // Stepped calibration movement voltage (V)
#define CALIB_HOLD_VOLTAGE                                                     \
  3.20f // Adequate holding voltage during sample to overcome cogging (V)
#define CALIB_BOOST_VOLTAGE                                                    \
  4.50f // Automatic torque boost voltage on stall/stiction (V)
#define CALIB_TOTAL_STEPS 128           // 128-point LUT resolution
#define CALIB_SWEEP_DAMP_SETTLE_MS 65   // Settle before oversampling (ms)
#define CALIB_SWEEP_OVERSAMPLE_COUNT 96 // Samples per step for noise rejection
#define CALIB_ALIGN_OVERSAMPLE_COUNT 96 // Samples per pole for electrical zero
#define CALIB_COOLDOWN_PAUSE_SEC                                               \
  5 // Dedicated thermal cooldown duration (seconds)
#define LUT_ANOMALY_THRESHOLD_RAD                                              \
  0.035f // > 2.0 deg mech (~22 deg elec) = pole slip / stall
#define LUT_PKPK_LIMIT_RAD                                                     \
  0.060f // > 3.44 deg mech = bad sweep / binding / severe runout

// ----------------------------------------------------------------------------
// Industry-Grade Stall & Thermal Protection Parameters
// ----------------------------------------------------------------------------
#define STALL_MIN_EFFORT_VOLTS                                                 \
  2.50f // Minimum |Uq| required to qualify as stall (V)
#define STALL_SATURATION_VOLTS 10.50f // Near-saturation voltage threshold (V)
#define STALL_VELOCITY_FLOOR_RPM                                               \
  3.0f // Speed below which rotor is considered stationary (RPM)
#define STALL_VELOCITY_LAG_RPM                                                 \
  30.0f // Speed error threshold under saturation (RPM)
#define STALL_I2T_CONT_VOLTS 3.50f // Continuous thermal voltage threshold (V)
#define STALL_I2T_TRIP_ENERGY                                                  \
  60.0f // Integrated energy capacity (V^2 * s) [~450ms at 12V dead stall]
#define STALL_I2T_LEAK_RATE                                                    \
  25.0f // Energy dissipation rate when effort drops (V^2 / s)
#define STALL_FOLDBACK_VOLTS                                                   \
  2.00f // Safe continuous holding voltage during foldback (V)
#define STALL_MAX_RECOVERY_TRIES                                               \
  3                              // Dither unwedge attempts before hard shutdown
#define STALL_DITHER_PULSE_MS 80 // Duration of reverse recovery pulse (ms)

// Real-Time Control Loop Frequencies
#define FOC_COMMUTATION_HZ                                                     \
  20000 // 20.0 kHz Hard Real-Time Commutation (50.0 us)
#define MOTION_DOWNSAMPLE_RATIO                                                \
  4 // 5.0 kHz Staggered Velocity PID (every 200.0 us per motor)
#define TELEMETRY_RATE_HZ 25 // 25 Hz Binary Telemetry Stream (40 ms)
#define SPI_SPEED_HZ                                                           \
  10625000 // 10.625 MHz (170 MHz / 16 prescaler) for high-speed low-latency
           // reads

// Pre-Loaded Zero Electrical Angles (Standalone Autonomous Startup Jerk Free)
#ifndef M1_DEFAULT_ZERO_ELECTRIC_ANGLE
#define M1_DEFAULT_ZERO_ELECTRIC_ANGLE 0.0f
#endif

#ifndef M2_DEFAULT_ZERO_ELECTRIC_ANGLE
#define M2_DEFAULT_ZERO_ELECTRIC_ANGLE 0.0f
#endif

#endif // CONFIG_PINS_H
