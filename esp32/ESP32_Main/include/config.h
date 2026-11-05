#ifndef ESP32_CONFIG_H
#define ESP32_CONFIG_H

#include <Arduino.h>
#include "vehicle_params.h"

// =============================================================================
// FOLKRACE Master ESP32-S3 Hardware Pinout Configuration
// Generated / Verified by pin_solver.py
// MCU: ESP32-S3-DevKitC-1 (N16R8 Octal Flash/PSRAM)
// =============================================================================

// --- BNO086 9-DOF IMU (SPI2 Master) ---
#define IMU_SPI_SCK_PIN          12  // Header J1 Pin 18 (SCK)
#define IMU_SPI_MOSI_PIN         11  // Header J1 Pin 17 (MOSI)
#define IMU_SPI_MISO_PIN         13  // Header J1 Pin 19 (MISO)
#define IMU_SPI_CS_PIN           10  // Header J1 Pin 16 (CS Active LOW)
#define IMU_INT_PIN              9   // Header J1 Pin 15 (INT Active LOW)
#define IMU_RST_PIN              14  // Header J1 Pin 20 (RST Active LOW)

// --- Single Ackermann Steering Servo (LEDC PWM) ---
// (SERVO_STEER_PWM_PIN, FREQ, RESOLUTION, and PULSE widths defined in vehicle_params.h)
#define SERVO_ACKERMANN_PWM_PIN  SERVO_STEER_PWM_PIN

// --- STM32 Motor Controller #1: Front Axle (M1 FL, M2 FR) ---
#define STM32_1_UART_NUM         1
#define STM32_1_UART_BAUD        921600
#define STM32_1_UART_TX_PIN      17  // ESP32 TX -> STM32 Front PA10 RX
#define STM32_1_UART_RX_PIN      18  // ESP32 RX <- STM32 Front PA9 TX

// --- STM32 Motor Controller #2: Rear Axle (M1 RL, M2 RR) ---
#define STM32_2_UART_NUM         2
#define STM32_2_UART_BAUD        921600
#define STM32_2_UART_TX_PIN      15  // ESP32 TX -> STM32 Rear PA10 RX
#define STM32_2_UART_RX_PIN      16  // ESP32 RX <- STM32 Rear PA9 TX

// --- Raspberry Pi 4 Telemetry Highway (UART0) ---
#define PI_UART_NUM              0
// (PI_UART_BAUD defined in vehicle_params.h)
#define PI_UART_TX_PIN           43  // ESP32 TX -> Pi 4 RX (GPIO 15)
#define PI_UART_RX_PIN           44  // ESP32 RX <- Pi 4 TX (GPIO 14)
#define PI_DRDY_PIN              1   // Hardware Batch Ready Pulse -> Pi 4 GPIO

// Note: Real-time control loop frequencies, vehicle physical geometry, kinematics,
// servo pulses, slip thresholds, and friction parameters are synchronized from
// config/vehicle_config.json via include/vehicle_params.h.

// =============================================================================
// Protocol Sync Bytes & Frame Structures
// =============================================================================
#define COMMAND_SYNC_BYTE        0xAA
#define FEEDBACK_SYNC_BYTE       0x55

// STM32 Dual-Motor Command Packet (transmitted to each STM32 at 400 Hz)
struct __attribute__((__packed__)) STM32MotorCommandPacket {
    uint8_t sync;      // 0xAA
    float m1_rpm;      // Motor 1 Target RPM
    float m2_rpm;      // Motor 2 Target RPM
    uint8_t checksum;  // XOR checksum over payload bytes
};

// 26-Byte Binary Telemetry Frame (received from STM32 FOC controller)
struct __attribute__((__packed__)) STM32TelemetryPacket {
    uint8_t  header[2];      // 0xAA, 0x55
    uint16_t m1_raw_angle;   // 0 .. 16383 (Raw 14-bit encoder)
    int16_t  m1_actual_rpm;  // Actual RPM x10
    int16_t  m1_target_rpm;  // Target RPM x10
    int16_t  m1_uq_mv;       // Phase voltage Uq (mV)
    uint8_t  m1_status;      // Bitfield
    uint16_t m2_raw_angle;   // 0 .. 16383
    int16_t  m2_actual_rpm;  // Actual RPM x10
    int16_t  m2_target_rpm;  // Target RPM x10
    int16_t  m2_uq_mv;       // Phase voltage Uq (mV)
    uint8_t  m2_status;      // Bitfield
    uint16_t foc_khz_x10;    // 200 = 20.0 kHz
    uint8_t  isr_lat_us;     // Peak ISR latency in us
    uint8_t  spikes_count;   // Jitter spikes
    uint8_t  calib_progress; // 0..100%
    uint8_t  checksum;       // XOR over bytes 2..24
};

#endif // ESP32_CONFIG_H
