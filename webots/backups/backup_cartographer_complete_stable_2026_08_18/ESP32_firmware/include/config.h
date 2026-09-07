#ifndef ESP32_CONFIG_H
#define ESP32_CONFIG_H

#include <Arduino.h>

// Baud rate for STM32 UART link
#define STM32_UART_BAUD 921600

// ESP32-S3 Hardware Serial Pins connected to STM32 (PB6 TX, PB7 RX)
#define STM32_UART_TX_PIN 17  // ESP32 TX -> STM32 RX (PB7)
#define STM32_UART_RX_PIN 18  // ESP32 RX <- STM32 TX (PB6)

// Pi 5 High-Speed UART2 Link (Pi 5 Pins 12 & 35 /dev/ttyAMA2)
#define PI_UART_BAUD 921600
#define PI_UART_TX_PIN 47  // ESP32 TX -> Pi 5 Pin 35 (GPIO 19 RX)
#define PI_UART_RX_PIN 21  // ESP32 RX <- Pi 5 Pin 12 (GPIO 18 TX)

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
