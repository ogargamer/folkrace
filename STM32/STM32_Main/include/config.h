#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Global System Speed
#define SYSTEM_CLOCK_FREQ 170000000L

// Motor Driver Configuration
#define MOTOR_POLE_PAIRS 11
#define DRIVER_VOLTAGE_LIMIT 12.0f
#define MOTOR_VOLTAGE_LIMIT 6.0f
#define MOTOR_CURRENT_LIMIT 1.5f

// Communication Baud Rate
#define USART_BAUD_RATE 921600

// Motor 1 (TIM1) Pin Definitions
#define M1_PWM_A PA8
#define M1_PWM_B PA9
#define M1_PWM_C PA10
#define M1_EN    PA5

// Motor 2 (TIM2) Pin Definitions
#define M2_PWM_A PA0
#define M2_PWM_B PA1
#define M2_PWM_C PA2
#define M2_EN    PA4

// Motor 3 (TIM3) Pin Definitions
#define M3_PWM_A PB4
#define M3_PWM_B PB5
#define M3_PWM_C PB0
#define M3_EN    PB9

// Motor 4 (TIM4) Pin Definitions
#define M4_PWM_A PA11
#define M4_PWM_B PA12
#define M4_PWM_C PB8
#define M4_EN    PB1

// Encoder Pin Definitions
#define ENC1_A PB2
#define ENC1_B PB10

#define ENC2_A PC14
#define ENC2_B PB11

#define ENC3_A PA7
#define ENC3_B PA6

#define ENC4_A PA3
#define ENC4_B PA15

// Master Communications (USART1)
#define USART_TX PB6
#define USART_RX PB7

// Heartbeat LED
#define STATUS_LED PC13

// Leftover Pins / Custom Hardware
#define USER_BUTTON PB12

// Communication Packet Structures
struct __attribute__((__packed__)) MotorCommandPacket {
    float m1_target; // Target RPM
    float m2_target; // Target RPM
    float m3_target; // Target RPM
    float m4_target; // Target RPM
    uint8_t checksum; // XOR of the 16 preceding payload bytes
};

struct __attribute__((__packed__)) MotorFeedbackPacket {
    uint8_t sync; // 0x55 Sync Byte
    float m1_rpm;
    float m2_rpm;
    float m3_rpm;
    float m4_rpm;
    uint8_t checksum; // XOR of the 16 preceding payload bytes
};

#endif // CONFIG_H
