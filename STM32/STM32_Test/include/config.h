#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Global System Speed
#define SYSTEM_CLOCK_FREQ 170000000L

// Motor Driver Configuration
#define MOTOR_POLE_PAIRS 11
#define DRIVER_VOLTAGE_LIMIT 12.5f
#define MOTOR_VOLTAGE_LIMIT 12.5f
#define MOTOR_CURRENT_LIMIT 0.5f

// Motor Driver Pin Definitions (from original test setup)
#define MOTOR_PWM_A PA0
#define MOTOR_PWM_B PA1
#define MOTOR_PWM_C PA2
#define MOTOR_EN    PA5

// Encoder Pins (Quadrature A/B)
#define ENC_A  PB4
#define ENC_B  PB5

// Heartbeat LED
#define STATUS_LED PC13 // Correct Active-Low LED pin

// Standalone Test Buttons
#define BUTTON_SPEED PB3 // Cycles speed presets: 0 -> 50 RPM
#define USER_BUTTON PB12 // Triggers FOC calibration / Starts closed-loop

#endif // CONFIG_H
