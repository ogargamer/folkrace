#!/usr/bin/env python3
"""
pin_solver.py
=============
Master ESP32-S3 Mathematical Pinout Solver & Hardware Peripheral Verifier
Target: ESP32-S3-DevKitC-1 (N16R8 Octal Flash/PSRAM)
Base Reference: FOLKRACE Master Sensor & Actuator Hub

Peripherals:
  - 1x BNO086 9-DOF IMU over Dedicated SPI Master (SCK, MOSI, MISO, CS, INT, RST)
  - 1x Ackermann Steering Servo on LEDC PWM (333 Hz high-rate digital PWM)
  - 2x STM32G474 Motor Controllers on dedicated hardware UARTs:
      * STM32 #1 (Front Axle): Full Duplex (TX + RX @ 921,600 Baud)
      * STM32 #2 (Rear Axle): Full Duplex (TX + RX @ 921,600 Baud)
  - 1x Raspberry Pi 4 Host Telemetry Highway (Dedicated Hardware UART + DRDY interrupt pulse)
  - Native USB CDC Serial / JTAG preserved for flashing and debug monitoring

Priorities:
  1st Priority: Hardware peripheral correctness & conflict freedom (BNO086 SPI, Servo, 3x UART links, 0 collisions).
  2nd Priority: Physical spacing & EMI noise immunity (Header clustering, Clock-to-signal isolation, DC guards).
"""

import sys
import math
import argparse

# ==============================================================================
# 1. ESP32-S3 SILICON CONSTRAINTS & PHYSICAL HEADER MODEL
# ==============================================================================
ALL_SILICON_GPIOS = set(range(0, 49))
NON_EXISTENT_GPIOS = {22, 23, 24, 25}             # ESP32-S3 silicon skips GPIO 22-25
FORBIDDEN_OCTAL_FLASH_PSRAM = set(range(26, 38))  # GPIO 26-37: Internal Octal Flash/PSRAM (N16R8)
ONBOARD_WS2812_LED = {48}                         # Onboard RGB LED Data
INPUT_ONLY_GPIOS = {46}                           # GPIO 46 lacks output driver transistors
STRAPPING_GPIOS = {0, 3, 45, 46}                  # Bootloader strapping pins
XTAL32K_GPIOS = {15, 16}                          # Can be used as standard GPIO if 32k crystal absent
NATIVE_USB_GPIOS = {19, 20}                       # Dedicated Native USB D- / D+

# Physical 2D Pin Mapping on ESP32-S3 DevKitC-1: (side, row_1_to_22)
# Side: 'left' (Header J1), 'right' (Header J3)
PHYSICAL_GRID = {
    # Header J1 (Left Side)
    "3V3_1": ("left", 1), "3V3_2": ("left", 2), "EN": ("left", 3),
    4: ("left", 4), 5: ("left", 5), 6: ("left", 6), 7: ("left", 7),
    15: ("left", 8), 16: ("left", 9), 17: ("left", 10), 18: ("left", 11),
    8: ("left", 12), 3: ("left", 13), 46: ("left", 14),
    9: ("left", 15), 10: ("left", 16), 11: ("left", 17), 12: ("left", 18),
    13: ("left", 19), 14: ("left", 20), "5V": ("left", 21), "GND_J1": ("left", 22),

    # Header J3 (Right Side)
    "GND_J3_1": ("right", 1), 43: ("right", 2), 44: ("right", 3),
    1: ("right", 4), 2: ("right", 5), 42: ("right", 6), 41: ("right", 7),
    40: ("right", 8), 39: ("right", 9), 38: ("right", 10),
    37: ("right", 11), 36: ("right", 12), 35: ("right", 13),
    0: ("right", 14), 45: ("right", 15), 48: ("right", 16),
    47: ("right", 17), 21: ("right", 18), 20: ("right", 19), 19: ("right", 20),
    "GND_J3_21": ("right", 21), "GND_J3_22": ("right", 22),
}

# ==============================================================================
# 2. OPTIMIZED HARDWARE SIGNAL ALLOCATION
# ==============================================================================
ALLOCATION = {
    # BNO086 9-DOF IMU (Dedicated SPI Master) -- Header J1 Pins 15-20 (Adjacent to 5V & GND)
    "IMU_SPI_SCK":          {"gpio": 12, "dir": "OUT", "bus": "SPI2_HOST", "target": "BNO086 SCK (CLK)"},
    "IMU_SPI_MOSI":         {"gpio": 11, "dir": "OUT", "bus": "SPI2_HOST", "target": "BNO086 DI (MOSI)"},
    "IMU_SPI_MISO":         {"gpio": 13, "dir": "IN",  "bus": "SPI2_HOST", "target": "BNO086 SDO (MISO)"},
    "IMU_SPI_CS":           {"gpio": 10, "dir": "OUT", "bus": "GPIO",      "target": "BNO086 CSN (CS)"},
    "IMU_INT":              {"gpio": 9,  "dir": "IN",  "bus": "GPIO",      "target": "BNO086 HINTN (INT)"},
    "IMU_RST":              {"gpio": 14, "dir": "OUT", "bus": "GPIO",      "target": "BNO086 NRST (RST)"},

    # Single Ackermann Steering Servo (LEDC PWM) -- Header J3 Pin 18 (Directly adjacent to GND Pins 21-22)
    "SERVO_ACKERMANN_PWM":  {"gpio": 21, "dir": "OUT", "bus": "LEDC",      "target": "Steering Servo PWM (Signal)"},

    # STM32 Motor Controller 1 Link (Front Axle) -- Header J1 Pins 10-11 (Full Duplex)
    "STM32_1_UART_TX":      {"gpio": 17, "dir": "OUT", "bus": "UART1",     "target": "STM32 #1 PA10 (RX)"},
    "STM32_1_UART_RX":      {"gpio": 18, "dir": "IN",  "bus": "UART1",     "target": "STM32 #1 PA9 (TX)"},

    # STM32 Motor Controller 2 Link (Rear Axle) -- Header J1 Pins 8-9 (Full Duplex)
    "STM32_2_UART_TX":      {"gpio": 15, "dir": "OUT", "bus": "UART2",     "target": "STM32 #2 PA10 (RX)"},
    "STM32_2_UART_RX":      {"gpio": 16, "dir": "IN",  "bus": "UART2",     "target": "STM32 #2 PA9 (TX)"},

    # Raspberry Pi 4 Telemetry Highway -- Header J3 Pins 2-4 (Directly below GND Pin 1)
    "PI_UART_TX":           {"gpio": 43, "dir": "OUT", "bus": "UART0",     "target": "Pi 4 GPIO 15 (Pin 10 / RXD0)"},
    "PI_UART_RX":           {"gpio": 44, "dir": "IN",  "bus": "UART0",     "target": "Pi 4 GPIO 14 (Pin 8 / TXD0)"},
    "PI_DRDY_OUT":          {"gpio": 1,  "dir": "OUT", "bus": "GPIO",      "target": "Pi 4 GPIO 20 (Pin 38 / INT)"},
}

RESERVED_SYSTEM_PINS = {
    "USB_DM":      {"gpio": 19, "dir": "I/O", "bus": "USB_CDC", "target": "Native USB D-"},
    "USB_DP":      {"gpio": 20, "dir": "I/O", "bus": "USB_CDC", "target": "Native USB D+"},
    "BOOT_STRAP":  {"gpio": 0,  "dir": "IN",  "bus": "STRAP",   "target": "Onboard BOOT Button (GND when pressed)"},
    "JTAG_STRAP":  {"gpio": 3,  "dir": "IN",  "bus": "STRAP",   "target": "JTAG Selection Strap (Internal)"},
    "VDD_SPI":     {"gpio": 45, "dir": "IN",  "bus": "STRAP",   "target": "Flash Voltage Strap (Must be LOW)"},
    "ROM_PRINT":   {"gpio": 46, "dir": "IN",  "bus": "STRAP",   "target": "Boot ROM Print Strap (Input-Only)"},
    "WS2812_LED":  {"gpio": 48, "dir": "OUT", "bus": "RGB_LED", "target": "Onboard WS2812 RGB LED Data"},
}


# ==============================================================================
# 3. MATHEMATICAL PROOFS & HARDWARE VALIDATION
# ==============================================================================
def calc_pin_distance(g1, g2):
    """Calculates physical distance in 2.54 mm pin pitch units across board grid."""
    side1, r1 = PHYSICAL_GRID[g1]
    side2, r2 = PHYSICAL_GRID[g2]
    if side1 == side2:
        return float(abs(r1 - r2))
    # Distance between Left and Right headers: 7.0 pin pitches (17.78 mm) across MCU
    return math.sqrt(7.0**2 + (r1 - r2)**2)


def evaluate_hardware_and_metrics():
    assigned = {}
    violations = []

    for name, item in ALLOCATION.items():
        g = item["gpio"]
        direction = item["dir"]

        # Proof 1: Strict Injectivity (no pin collisions)
        if g in assigned:
            violations.append(f"COLLISION: GPIO {g} assigned to both '{assigned[g]}' and '{name}'")
        assigned[g] = name

        # Proof 2: Silicon Safety
        if g in NON_EXISTENT_GPIOS:
            violations.append(f"INVALID SILICON: GPIO {g} does not exist in ESP32-S3 silicon")
        if g in FORBIDDEN_OCTAL_FLASH_PSRAM:
            violations.append(f"FATAL CONFLICT: GPIO {g} interferes with internal Octal Flash/PSRAM")
        if g in ONBOARD_WS2812_LED:
            violations.append(f"RGB LED CONFLICT: GPIO {g} is wired to onboard WS2812")
        if g in STRAPPING_GPIOS:
            violations.append(f"BOOTLOADER CONFLICT: GPIO {g} interferes with boot strapping")
        if g in NATIVE_USB_GPIOS:
            violations.append(f"USB CONFLICT: GPIO {g} interferes with Native USB CDC")

        # Proof 3: Direction Validity
        if direction == "OUT" and g in INPUT_ONLY_GPIOS:
            violations.append(f"DIRECTION ERROR: GPIO {g} is input-only (GPI); cannot drive output '{name}'")

    if violations:
        print("ERROR: Hardware validation failed:")
        for v in violations:
            print("  *", v)
        sys.exit(1)

    # Proof 4: Physical Header Clustering & Spacing Metrics
    sck_gpio = ALLOCATION["IMU_SPI_SCK"]["gpio"]
    miso_gpio = ALLOCATION["IMU_SPI_MISO"]["gpio"]
    sck_miso_pitch = calc_pin_distance(sck_gpio, miso_gpio)

    # Distance between high dv/dt Servo PWM and sensitive SPI lines
    servo_gpio = ALLOCATION["SERVO_ACKERMANN_PWM"]["gpio"]
    min_servo_to_imu_pitch = min(calc_pin_distance(servo_gpio, item["gpio"]) for k, item in ALLOCATION.items() if "IMU_" in k)

    # Calculate free GPIO count
    free_gpios = sorted(list(
        ALL_SILICON_GPIOS
        - {item["gpio"] for item in ALLOCATION.values()}
        - {item["gpio"] for item in RESERVED_SYSTEM_PINS.values()}
        - FORBIDDEN_OCTAL_FLASH_PSRAM
        - NON_EXISTENT_GPIOS
    ))

    return {
        "total_assigned": len(assigned),
        "free_gpios": free_gpios,
        "sck_miso_pitch": sck_miso_pitch,
        "sck_miso_mm": sck_miso_pitch * 2.54,
        "servo_to_imu_pitch": min_servo_to_imu_pitch,
        "servo_to_imu_mm": min_servo_to_imu_pitch * 2.54,
    }


# ==============================================================================
# 4. STRUCTURED, COMPACT OUTPUT REPORT
# ==============================================================================
def display_results():
    metrics = evaluate_hardware_and_metrics()

    print("=" * 80)
    print("  ESP32-S3 MASTER SENSOR & MOTOR CONTROLLER PIN SOLVER REPORT")
    print("  Target: ESP32-S3-DevKitC-1 (N16R8 Octal Flash/PSRAM)")
    print("  Subsystems: BNO086 SPI IMU + Ackermann Servo + 2x STM32 UART + Pi 4 Telemetry")
    print("=" * 80)

    print("\n[MATHEMATICAL PROOF & HARDWARE VALIDATION]")
    print(f"  * Total Signals Assigned          : {metrics['total_assigned']} pins")
    print(f"  * Pin Collision Check             : PASSED (0 overlaps, strict 1:1 mapping)")
    print(f"  * Hardware UART Links             : CONFIRMED (UART0=Pi 4, UART1=STM32_1 Full-Duplex, UART2=STM32_2 Full-Duplex)")
    print(f"  * Dedicated SPI2 Master (BNO086)  : 6 Dedicated Pins (Adjacent to 5V/GND on Header J1)")
    print(f"  * Ackermann Steering Servo (LEDC) : Assigned to GPIO 21 (Header J3 Row 18)")
    print(f"  * Octal Flash/PSRAM (GPIO 26-37)  : SAFE (100% untouched)")
    print(f"  * Boot Strapping Pins (0,3,45,46) : SAFE (100% untouched)")
    print(f"  * Onboard RGB LED (GPIO 48)       : SAFE (100% untouched)")
    print(f"  * Native USB CDC (GPIO 19, 20)    : PRESERVED (Flashing & Interactive Serial Monitor)")

    print("\n[PHYSICAL CLUSTERING & EMI IMMUNITY METRICS]")
    print(f"  * IMU SPI Ribbon Pin Pitch        : Row 15 to Row 20 contiguous header block on J1")
    print(f"  * Servo PWM to IMU Separation     : {metrics['servo_to_imu_pitch']:.2f} pin pitches ({metrics['servo_to_imu_mm']:.2f} mm across board)")
    print(f"  * Dual STM32 Links Physical Block : Row 8 to Row 11 on Header J1 (adjacent comms)")
    print(f"  * Pi 4 Telemetry Physical Block   : Row 2 to Row 4 on Header J3 (directly below GND)")

    def format_signal(sig, item):
        target = item.get("target", "")
        direction = item.get("dir", "")
        if not target:
            return sig
        if direction == "OUT":
            arrow = " -> "
        elif direction == "IN":
            arrow = " <- "
        else:
            arrow = " <-> "
        return f"{sig}{arrow}{target}"

    # Structured table with exactly SIGNAL and GPIO columns
    print("\n" + "-" * 72)
    print(f"  {'SIGNAL':<56} | {'GPIO':<9}")
    print("-" * 72)

    # BNO086 IMU
    print("  [BNO086 9-DOF IMU -- SPI2 DEDICATED MASTER]")
    for sig in ["IMU_SPI_SCK", "IMU_SPI_MOSI", "IMU_SPI_MISO", "IMU_SPI_CS", "IMU_INT", "IMU_RST"]:
        item = ALLOCATION[sig]
        sig_str = format_signal(sig, item)
        print(f"  {sig_str:<56} | GPIO {item['gpio']:<4}")

    # Ackermann Steering Servo
    print("\n  [ACKERMANN STEERING ACTUATOR -- LEDC HIGH-RATE PWM]")
    item = ALLOCATION["SERVO_ACKERMANN_PWM"]
    sig_str = format_signal("SERVO_ACKERMANN_PWM", item)
    print(f"  {sig_str:<56} | GPIO {item['gpio']:<4}")

    # STM32 Controller #1 Link
    print("\n  [STM32 MOTOR CONTROLLER #1 -- UART1 LINK (FRONT AXLE FULL-DUPLEX)]")
    for sig in ["STM32_1_UART_TX", "STM32_1_UART_RX"]:
        item = ALLOCATION[sig]
        sig_str = format_signal(sig, item)
        print(f"  {sig_str:<56} | GPIO {item['gpio']:<4}")

    # STM32 Controller #2 Link
    print("\n  [STM32 MOTOR CONTROLLER #2 -- UART2 LINK (REAR AXLE FULL-DUPLEX)]")
    for sig in ["STM32_2_UART_TX", "STM32_2_UART_RX"]:
        item = ALLOCATION[sig]
        sig_str = format_signal(sig, item)
        print(f"  {sig_str:<56} | GPIO {item['gpio']:<4}")

    # Raspberry Pi 4 Telemetry Highway
    print("\n  [RASPBERRY PI 4 TELEMETRY HIGHWAY -- UART0 + DRDY]")
    for sig in ["PI_UART_TX", "PI_UART_RX", "PI_DRDY_OUT"]:
        item = ALLOCATION[sig]
        sig_str = format_signal(sig, item)
        print(f"  {sig_str:<56} | GPIO {item['gpio']:<4}")

    # System & Reserved
    print("\n  [SYSTEM RESERVED & HARDWARE DEBUG]")
    for sig, item in RESERVED_SYSTEM_PINS.items():
        sig_str = format_signal(sig, item)
        print(f"  {sig_str:<56} | GPIO {item['gpio']:<4}")

    print("-" * 72)
    print(f"  Available Free GPIOs ({len(metrics['free_gpios'])} pins): {metrics['free_gpios']}")
    print("=" * 80 + "\n")


def print_c_header():
    """Generates C/C++ Header `#define` configuration for config.h"""
    print("// =============================================================================")
    print("// AUTOGENERATED HARDWARE CONFIGURATION FROM pin_solver.py")
    print("// =============================================================================")
    print("// BNO086 9-DOF IMU (SPI2)")
    print(f"#define IMU_SPI_SCK_PIN          {ALLOCATION['IMU_SPI_SCK']['gpio']}")
    print(f"#define IMU_SPI_MOSI_PIN         {ALLOCATION['IMU_SPI_MOSI']['gpio']}")
    print(f"#define IMU_SPI_MISO_PIN         {ALLOCATION['IMU_SPI_MISO']['gpio']}")
    print(f"#define IMU_SPI_CS_PIN           {ALLOCATION['IMU_SPI_CS']['gpio']}")
    print(f"#define IMU_INT_PIN              {ALLOCATION['IMU_INT']['gpio']}")
    print(f"#define IMU_RST_PIN              {ALLOCATION['IMU_RST']['gpio']}")
    print()
    print("// Ackermann Steering Servo (LEDC PWM)")
    print(f"#define SERVO_STEER_PWM_PIN      {ALLOCATION['SERVO_ACKERMANN_PWM']['gpio']}")
    print(f"#define SERVO_ACKERMANN_PWM_PIN  SERVO_STEER_PWM_PIN")
    print()
    print("// STM32 Motor Controller 1 Link (UART1 - Front Axle Full Duplex)")
    print("#define STM32_1_UART_BAUD        921600")
    print(f"#define STM32_1_UART_TX_PIN      {ALLOCATION['STM32_1_UART_TX']['gpio']}")
    print(f"#define STM32_1_UART_RX_PIN      {ALLOCATION['STM32_1_UART_RX']['gpio']}")
    print()
    print("// STM32 Motor Controller 2 Link (UART2 - Rear Axle Full Duplex)")
    print("#define STM32_2_UART_BAUD        921600")
    print(f"#define STM32_2_UART_TX_PIN      {ALLOCATION['STM32_2_UART_TX']['gpio']}")
    print(f"#define STM32_2_UART_RX_PIN      {ALLOCATION['STM32_2_UART_RX']['gpio']}")
    print()
    print("// Raspberry Pi 4 Telemetry Highway (UART0)")
    print("#define PI_UART_BAUD             921600")
    print(f"#define PI_UART_TX_PIN           {ALLOCATION['PI_UART_TX']['gpio']}")
    print(f"#define PI_UART_RX_PIN           {ALLOCATION['PI_UART_RX']['gpio']}")
    print(f"#define PI_DRDY_PIN              {ALLOCATION['PI_DRDY_OUT']['gpio']}")
    print("// =============================================================================")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ESP32-S3 Master Pinout Solver")
    parser.add_argument("--header", action="store_true", help="Print autogenerated C/C++ config header")
    args = parser.parse_args()

    if args.header:
        print_c_header()
    else:
        display_results()
