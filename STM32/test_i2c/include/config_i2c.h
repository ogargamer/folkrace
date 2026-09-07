#ifndef CONFIG_I2C_H
#define CONFIG_I2C_H

#include <Arduino.h>

// Global System Speed
#define SYSTEM_CLOCK_FREQ 170000000L

// Motor Driver Configuration
#define MOTOR_POLE_PAIRS     11
#define DRIVER_VOLTAGE_LIMIT 12.5f
#define MOTOR_VOLTAGE_LIMIT  12.5f
#define MOTOR_CURRENT_LIMIT  0.5f

// Motor Driver Pin Definitions
#define MOTOR_PWM_A PA0
#define MOTOR_PWM_B PA1
#define MOTOR_PWM_C PA2
#define MOTOR_EN    PA5

// Encoder Pins — Hardware I2C1 (MT6701 1MHz Fast Mode Plus)
// PB9 = I2C1_SDA | PB8 = I2C1_SCL
#define ENC_SDA PB9
#define ENC_SCL PB8

// Heartbeat LED
#define STATUS_LED PC13  // Active-Low

// Standalone Test Buttons
#define BUTTON_SPEED PB3
#define USER_BUTTON  PB12

#endif // CONFIG_I2C_H
