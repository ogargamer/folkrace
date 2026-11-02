#ifndef TOF_PINOUT_H
#define TOF_PINOUT_H

#include <Arduino.h>

// =============================================================================
// FOLKRACE — QUAD VL53L8CX TOF SENSOR HUB (DEDICATED MISO ARCHITECTURE)
// Target Board: ESP32-S3-DevKitC-1 (N16R8 Octal PSRAM/Flash)
// Base Reference: feat/tof-s4-fallback (hardware-tested & verified 60 Hz)
//
// Topology:
//   - Shared SPI Bus: SCK (GPIO 13), MOSI (GPIO 14)
//   - Dedicated Per-Sensor MISO: Dynamically attached via spiAttachMISO()
//     (Eliminates VL53L8CX breakout tri-state bus contention)
//   - Consistent 4-Pin Quad Ordering per Sensor: [ MISO -> LPN -> INT -> CS ]
//     * ToF 1 (Front Left) : J1 Pins 4-7   [ GPIO 4, 5, 6, 7 ]
//     * ToF 2 (Front Right): J1 Pins 15-18 [ GPIO 9, 10, 11, 12 ]
//     * ToF 3 (Side Left)  : J3 Pins 4-7   [ GPIO 1, 2, 42, 41 ]
//     * ToF 4 (Side Right) : J3 Pins 8-10, 17 [ GPIO 40, 39, 38, 47 ]
//   - Host Telemetry Link: UART0 [ TX: GPIO 43, RX: GPIO 44 ]
// =============================================================================

#define NUM_TOFS 4

// =============================================================================
// SHARED SPI MASTER BUS (Header J1 Pins 19 & 20)
// =============================================================================
#define TOF_SHARED_SCK_PIN   13   // J1 Pin 19: Shared SPI Clock
#define TOF_SHARED_MOSI_PIN  14   // J1 Pin 20: Shared SPI MOSI

// =============================================================================
// PER-SENSOR QUAD PIN DEFINITIONS: [ MISO -> LPN -> INT -> CS ]
// =============================================================================

// --- ToF 1 (Front Left / Sensor 1) — Header J1 Pins 4 to 7 ---
#define TOF1_MISO_PIN        4    // J1 Pin 4: Dedicated MISO
#define TOF1_LPN_PIN         5    // J1 Pin 5: Power Enable / Reset
#define TOF1_INT_PIN         6    // J1 Pin 6: Data Ready Interrupt
#define TOF1_CS_PIN          7    // J1 Pin 7: Chip Select (Active LOW)

// --- ToF 2 (Front Right / Sensor 2) — Header J1 Pins 15 to 18 ---
#define TOF2_MISO_PIN        9    // J1 Pin 15: Dedicated MISO
#define TOF2_LPN_PIN         10   // J1 Pin 16: Power Enable / Reset
#define TOF2_INT_PIN         11   // J1 Pin 17: Data Ready Interrupt
#define TOF2_CS_PIN          12   // J1 Pin 18: Chip Select (Active LOW)

// --- ToF 3 (Side Left / Sensor 3) — Header J3 Pins 4 to 7 ---
#define TOF3_MISO_PIN        1    // J3 Pin 4: Dedicated MISO
#define TOF3_LPN_PIN         2    // J3 Pin 5: Power Enable / Reset
#define TOF3_INT_PIN         42   // J3 Pin 6: Data Ready Interrupt
#define TOF3_CS_PIN          41   // J3 Pin 7: Chip Select (Active LOW)

// --- ToF 4 (Side Right / Sensor 4) — Header J3 Pins 8 to 10, 17 ---
#define TOF4_MISO_PIN        40   // J3 Pin 8: Dedicated MISO
#define TOF4_LPN_PIN         39   // J3 Pin 9: Power Enable / Reset
#define TOF4_INT_PIN         38   // J3 Pin 10: Data Ready Interrupt
#define TOF4_CS_PIN          47   // J3 Pin 17: Chip Select (Active LOW)

// =============================================================================
// HOST TELEMETRY & CONSOLE LINK (To Raspberry Pi 5 / PC)
// Header J3 Pins 2-3
// =============================================================================
#define HOST_UART_TX_PIN     43   // J3 Pin 2: UART0 TX -> Pi 5 / PC
#define HOST_UART_RX_PIN     44   // J3 Pin 3: UART0 RX <- Pi 5 / PC
#define HOST_UART_BAUD       921600

#endif // TOF_PINOUT_H
