#ifndef ESP32_CONFIG_H
#define ESP32_CONFIG_H

#include <Arduino.h>

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
#define SERVO_STEER_PWM_PIN      21  // Header J3 Pin 18 (333 Hz Digital Servo)
#define SERVO_ACKERMANN_PWM_PIN  SERVO_STEER_PWM_PIN
#define SERVO_PWM_CHANNEL        0   // LEDC Channel 0
#define SERVO_PWM_FREQ_HZ        333 // 333 Hz Refresh Rate (Period = 3003 us)
#define SERVO_PWM_RESOLUTION     14  // 14-bit resolution (0..16383, ~0.183 us/count)
#define SERVO_PULSE_MIN_US       500.0f   // Full Left endpoint (0 deg)
#define SERVO_PULSE_NEUTRAL_US   1520.0f  // Neutral Center (90 deg / 0 rad)
#define SERVO_PULSE_MAX_US       2500.0f  // Full Right endpoint (180 deg)

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
#define PI_UART_BAUD             921600
#define PI_UART_TX_PIN           43  // ESP32 TX -> Pi 4 RX (GPIO 15)
#define PI_UART_RX_PIN           44  // ESP32 RX <- Pi 4 TX (GPIO 14)
#define PI_DRDY_PIN              1   // Hardware Batch Ready Pulse -> Pi 4 GPIO

// =============================================================================
// Real-Time Control Loop Frequency & Timing (400 Hz deterministic)
// =============================================================================
#define CONTROL_LOOP_FREQ_HZ     400
#define CONTROL_LOOP_PERIOD_US   2500  // 2500 us (2.5 ms)
#define CONTROL_LOOP_DT_S        0.0025f

// =============================================================================
// Vehicle Physical Geometry & Kinematic Limits (Matching Webots Simulation)
// =============================================================================
#define VEHICLE_WHEELBASE_M      0.1325f  // L = 132.5 mm
#define VEHICLE_TRACK_REAR_M     0.1000f  // W_r = 100.0 mm
#define VEHICLE_TRACK_FRONT_M    0.0822f  // W_f = 82.2 mm
#define VEHICLE_TRACK_KINGPIN_M  0.0620f  // W_k = 62.0 mm
#define VEHICLE_WHEEL_RADIUS_M   0.0275f  // R_w = 27.5 mm
#define VEHICLE_MAX_STEER_RAD    0.5236f  // Max Virtual Center Angle (+/- 30 deg)
#define VEHICLE_ACKERMANN_PCT    0.40f    // 40% Ackermann Geometry
#define VEHICLE_MOTOR_DIR        -1.0f    // Webots thrust sign (-1.0 pushes forward)
#define VEHICLE_MAX_WHEEL_SPEED  5.0f     // Top speed ceiling (m/s)
#define VEHICLE_SERVO_MAX_RATE   16.89f   // Slew rate limit (rad/s, 0.062 s / 60 deg @ 8.4V = ~968 deg/s)

// Slip Detection Parameters
#define SLIP_GATE_SPEED_MPS      0.30f    // Minimum speed to evaluate slip (m/s)
#define SLIP_THRESH_SPIN_RAD_S   0.45f    // Base yaw rate divergence threshold

// Friction & Traction Dynamic Envelope
#define DEFAULT_ROAD_MU          0.86f    // Baseline surface friction coefficient
#define ACCEL_FWD_MAX            8.015f   // Maximum forward motor acceleration (m/s^2)
#define ACCEL_FWD_RATIO          0.95f    // Acceleration fraction of mu * g
#define BRAKE_DECEL_MAX          12.0f    // Maximum braking deceleration (m/s^2)
#define BRAKE_DECEL_RATIO        1.20f    // Braking fraction of mu * g

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
