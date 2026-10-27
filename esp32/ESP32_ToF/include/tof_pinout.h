#ifndef TOF_PINOUT_H
#define TOF_PINOUT_H

#include <Arduino.h>

// =============================================================================
// DA WINNA — DUAL-SPI QUAD VL53L8CX HARDWARE PIN CONFIGURATION
// Target Board: ESP32-S3-DevKitC-1 (N16R8 Octal PSRAM/Flash)
//
// Topology:
//   - SPI Channel 1 (SPI2 / FSPI): Dedicated to ToF 1 & ToF 2 (Front Left / Right)
//   - SPI Channel 2 (SPI3 / HSPI): Dedicated to ToF 3 & ToF 4 (Side Left / Right)
// =============================================================================

#define NUM_TOFS 4

// =============================================================================
// CHANNEL 1: SPI2 (FSPI) — TOF 1 & TOF 2 (FRONT SECTOR)
// =============================================================================
// Shared Bus Pins for Channel 1
#define TOF_SPI2_SCK_PIN     12   // FSPI CLK  (IO MUX native high-speed)
#define TOF_SPI2_MOSI_PIN    11   // FSPI MOSI (IO MUX native high-speed)
#define TOF_SPI2_MISO_SHARED 13   // FSPI MISO (used if MISO is shared)

// --- ToF 1 (Front Left / Zone 1) ---
#define TOF1_CS_PIN          10   // Chip Select (Active LOW)
#define TOF1_INT_PIN         5    // Data Ready Interrupt (Falling Edge)
#define TOF1_LPN_PIN         6    // Low Power / Hardware Enable (Active HIGH)
#define TOF1_MISO_PIN        13   // Native FSPI MISO line

// --- ToF 2 (Front Right / Zone 2) ---
#define TOF2_CS_PIN          9    // Chip Select (Active LOW)
#define TOF2_INT_PIN         14   // Data Ready Interrupt (Falling Edge)
#define TOF2_LPN_PIN         8    // Low Power / Hardware Enable (Active HIGH)
#define TOF2_MISO_PIN        4    // Dedicated MISO (isolates breakout tri-state issues; or bridge to 13)


// =============================================================================
// CHANNEL 2: SPI3 (HSPI) — TOF 3 & TOF 4 (SIDE / CORNER SECTOR)
// =============================================================================
// Shared Bus Pins for Channel 2
#define TOF_SPI3_SCK_PIN     39   // HSPI CLK  (IO MUX native high-speed)
#define TOF_SPI3_MOSI_PIN    40   // HSPI MOSI (IO MUX native high-speed)
#define TOF_SPI3_MISO_SHARED 41   // HSPI MISO (used if MISO is shared)

// --- ToF 3 (Side Left / Zone 3) ---
#define TOF3_CS_PIN          7    // Chip Select (Active LOW)
#define TOF3_INT_PIN         15   // Data Ready Interrupt (Falling Edge)
#define TOF3_LPN_PIN         16   // Low Power / Hardware Enable (Active HIGH)
#define TOF3_MISO_PIN        41   // Native HSPI MISO line

// --- ToF 4 (Side Right / Zone 4) ---
#define TOF4_CS_PIN          17   // Chip Select (Active LOW)
#define TOF4_INT_PIN         18   // Data Ready Interrupt (Falling Edge)
#define TOF4_LPN_PIN         21   // Low Power / Hardware Enable (Active HIGH)
#define TOF4_MISO_PIN        38   // Dedicated MISO (isolates breakout tri-state issues; or bridge to 41)


// =============================================================================
// HOST TELEMETRY & CONSOLE LINK (To Raspberry Pi 5 / PC)
// =============================================================================
#define HOST_UART_TX_PIN     43   // UART0 TX (CH343P USB-to-UART / Pi 5 Serial)
#define HOST_UART_RX_PIN     44   // UART0 RX (CH343P USB-to-UART / Pi 5 Serial)
#define HOST_UART_BAUD       921600

// Optional Hardware Data-Ready Interrupt Pulse to Raspberry Pi 5
#define PI_DRDY_NOTIFY_PIN   2    // 10us HIGH pulse when full 4-sensor batch ready

#endif // TOF_PINOUT_H
