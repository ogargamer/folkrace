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

// --- Raspberry Pi 4 Telemetry Link (Direct USB Serial Cable) ---
#define PI_UART_BAUD             921600
#define PI_USE_USB_SERIAL        1
// Zero header wires required: Host link runs over onboard USB Serial directly to Pi 4 USB port.

// Note: Real-time control loop frequencies, vehicle physical geometry, kinematics,
// servo pulses, slip thresholds, and friction parameters are synchronized from
// config/vehicle_config.json via include/vehicle_params.h.

// =============================================================================
// STM32 FOC Dual-Motor Real-Time 400 Hz Protocol
// =============================================================================
#define STM32_CMD_SYNC_1           0xAA
#define STM32_CMD_SYNC_2           0x55
#define STM32_FEEDBACK_SYNC_1      0xAA
#define STM32_FEEDBACK_SYNC_2      0x55

enum STM32CommandMode : uint8_t {
    STM32_CMD_DRIVE      = 0x00, // Regular 400Hz velocity command (param1=m1_rpm, param2=m2_rpm)
    STM32_CMD_ESTOP      = 0x01, // Immediate E-stop: cut PWM, zero voltage, disable drivers
    STM32_CMD_ENABLE     = 0x02, // Enable motor drivers
    STM32_CMD_CALIB_M1   = 0x10, // Start Motor 1 Calibration (param1=align_voltage)
    STM32_CMD_CALIB_M2   = 0x11, // Start Motor 2 Calibration (param1=align_voltage)
    STM32_CMD_SET_PID    = 0x20, // Set velocity PID (param1=Kp, param2=Ki)
    STM32_CMD_SAVE_FLASH = 0x30, // Save active calibration & gains to STM32 Flash
    STM32_CMD_ENTER_DFU  = 0xDF  // Reboot into ROM DFU bootloader
};

// 12-Byte Robust Command Packet (transmitted from ESP32 to each STM32 at 400 Hz)
struct __attribute__((__packed__)) STM32MotorCommandPacket {
    uint8_t header[2];  // 0xAA, 0x55 (STM32_CMD_SYNC_1, STM32_CMD_SYNC_2)
    uint8_t cmd_mode;   // STM32CommandMode
    float   param1;     // m1 target RPM / Kp / align_voltage
    float   param2;     // m2 target RPM / Ki
    uint8_t checksum;   // XOR of bytes 2..10 (cmd_mode, param1, param2)
};
static_assert(sizeof(STM32MotorCommandPacket) == 12, "STM32MotorCommandPacket must be 12 bytes");

// Status Flags Bitfield (from STM32)
#define STM32_STATUS_M1_ENABLED     (1 << 0)
#define STM32_STATUS_M2_ENABLED     (1 << 1)
#define STM32_STATUS_CALIB_ACTIVE   (1 << 2)
#define STM32_STATUS_STALL_WARNING  (1 << 3)
#define STM32_STATUS_LATCHED_FAULT  (1 << 4)
#define STM32_STATUS_ENC1_OK        (1 << 5)
#define STM32_STATUS_ENC2_OK        (1 << 6)

// 13-Byte High-Speed Feedback Packet (received from STM32 in 400 Hz ping-pong reply)
struct __attribute__((__packed__)) STM32FeedbackPacket {
    uint8_t header[2];      // 0xAA, 0x55
    uint8_t status_flags;   // Bitfield (enabled, calib, stall, fault, enc_ok)
    float   m1_actual_rpm;  // Actual shaft RPM
    float   m2_actual_rpm;  // Actual shaft RPM
    uint8_t calib_progress; // 0..100%
    uint8_t checksum;       // XOR over bytes 2..11 (status_flags, m1, m2, calib_progress)
};
static_assert(sizeof(STM32FeedbackPacket) == 13, "STM32FeedbackPacket must be 13 bytes");

#endif // ESP32_CONFIG_H
