#ifndef ESP32_CONFIG_H
#define ESP32_CONFIG_H

#include <Arduino.h>

// =============================================================================
// FOLKRACE Master ESP32-S3 Hardware Pinout Configuration
// Generated / Verified by pin_solver.py
// =============================================================================

// --- BNO086 9-DOF IMU (SPI2 Master) ---
#define IMU_SPI_SCK_PIN          12
#define IMU_SPI_MOSI_PIN         11
#define IMU_SPI_MISO_PIN         13
#define IMU_SPI_CS_PIN           10
#define IMU_INT_PIN              9
#define IMU_RST_PIN              14

// --- Ackermann Steering Servo (LEDC PWM) ---
#define SERVO_STEER_PWM_PIN      21  // 333 Hz Digital Ackermann Steering Servo
#define SERVO_ACKERMANN_PWM_PIN  SERVO_STEER_PWM_PIN

// --- STM32 Motor Controller #1 (UART1) ---
#define STM32_1_UART_BAUD        921600
#define STM32_1_UART_TX_PIN      17  // ESP32 TX -> STM32 #1 RX (PB7)
#define STM32_1_UART_RX_PIN      18  // ESP32 RX <- STM32 #1 TX (PB6)

// Legacy aliases for single-link backward compatibility
#define STM32_UART_BAUD          STM32_1_UART_BAUD
#define STM32_UART_TX_PIN        STM32_1_UART_TX_PIN
#define STM32_UART_RX_PIN        STM32_1_UART_RX_PIN

// --- STM32 Motor Controller #2 (UART2) ---
#define STM32_2_UART_BAUD        921600
#define STM32_2_UART_TX_PIN      15  // ESP32 TX -> STM32 #2 RX (PB7)
#define STM32_2_UART_RX_PIN      16  // ESP32 RX <- STM32 #2 TX (PB6)

// --- Raspberry Pi 4 Telemetry Highway (UART0) ---
#define PI_UART_BAUD             921600
#define PI_UART_TX_PIN           43  // ESP32 TX -> Pi 4 RX (GPIO 15)
#define PI_UART_RX_PIN           44  // ESP32 RX <- Pi 4 TX (GPIO 14)
#define PI_DRDY_PIN              1   // Hardware Batch Ready Pulse -> Pi 4 GPIO

// Control Loop Frequency (400 Hz)
#define CONTROL_LOOP_FREQ_HZ 400
#define CONTROL_LOOP_PERIOD_US (1000000 / CONTROL_LOOP_FREQ_HZ)

// Protocol Sync Bytes
#define COMMAND_SYNC_BYTE  0xAA
#define FEEDBACK_SYNC_BYTE 0x55

// Communication Packet Structures (matching STM32 esp32_interface.h)
struct __attribute__((__packed__)) MotorCommandPacket {
    float m1_target; // Target RPM
    float m2_target; // Target RPM
    float m3_target; // Target RPM
    float m4_target; // Target RPM
    uint8_t checksum; // XOR of preceding 16 bytes
};

struct __attribute__((__packed__)) MotorFeedbackPacket {
    uint8_t sync;     // 0x55 Sync Byte
    float m1_rpm;     // Actual shaft RPM
    float m2_rpm;
    float m3_rpm;
    float m4_rpm;
    uint8_t checksum; // XOR of preceding 16 bytes
};

#endif // ESP32_CONFIG_H
