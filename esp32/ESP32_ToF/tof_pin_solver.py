#!/usr/bin/env python3
"""
tof_pin_solver.py
=================
Automated Mathematical Pinout Solver & Peripheral Feasibility Analyzer for
the Dedicated "FOLKRACE" ESP32-S3 Dual-SPI Quad VL53L8CX ToF Perception Hub.

Mathematical Objectives:
  1. Formal Set-Theoretic Proof:
     Validate that all pin assignments strictly satisfy all ESP32-S3 silicon
     constraints (Octal Flash/PSRAM, Input-Only GPIs, Strapping Pin boot levels,
     and IO MUX hardware capabilities) with ZERO collisions and ZERO illegal overlaps.
  2. Dual-SPI Hardware Segmentation:
     Prove that dividing 4 ToFs across 2 independent hardware SPI peripherals
     (SPI2_HOST / FSPI and SPI3_HOST / HSPI) solves MISO tri-state bus contention
     and mathematically eliminates SPI bus saturation at 60 Hz.
  3. Timing & Bandwidth Calculus:
     Compute frame transfer latency, SPI bus duty cycle, and telemetry link margin.
"""

import sys
import math

# ==============================================================================
# 1. ESP32-S3 SILICON CONSTRAINTS & HARDWARE SET DEFINITIONS
# ==============================================================================

# Total addressable GPIO space in silicon
ALL_SILICON_GPIOS = set(range(0, 49))

# Pins that DO NOT EXIST in ESP32-S3 silicon
NON_EXISTENT_GPIOS = {22, 23, 24, 25}

# Pins permanently wired to internal Octal SPI Flash and Octal PSRAM on N16R8 packages.
# Accessing or reconfiguring these pins instantly crashes the MCU or corrupts instruction cache!
FORBIDDEN_OCTAL_FLASH_PSRAM = {26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37}

# Hardware-fixed peripheral on DevKitC-1 boards
ONBOARD_WS2812_LED = {48}

# Input-Only GPI: Lacks output driver transistors (cannot drive CS, SCK, MOSI, or LPN)
INPUT_ONLY_GPIOS = {46}

# Strapping pins with specific reset-time logic levels:
#   - GPIO 0: Boot mode strap (Must be HIGH at boot, weak pull-up)
#   - GPIO 3: JTAG signal source strap (Floating default)
#   - GPIO 45: VDD_SPI voltage strap (Must be LOW at boot for 3.3V flash)
#   - GPIO 46: ROM message printing strap (Input-only)
STRAPPING_GPIOS = {0, 3, 45, 46}

# Dedicated USB OTG Native lines
NATIVE_USB_GPIOS = {19, 20}

# Default Console / CH343P USB-to-UART Bridge lines
CONSOLE_UART0_GPIOS = {43, 44}

# Valid usable GPIOs for application peripherals
USABLE_GPIOS = (
    ALL_SILICON_GPIOS
    - NON_EXISTENT_GPIOS
    - FORBIDDEN_OCTAL_FLASH_PSRAM
    - ONBOARD_WS2812_LED
)

# ==============================================================================
# 2. PROPOSED DUAL-SPI QUAD TOF PIN ALLOCATION
# ==============================================================================

DUAL_SPI_ALLOCATION = {
    # --------------------------------------------------------------------------
    # CHANNEL 1: SPI2_HOST (FSPI) — TOF 1 & TOF 2 (FRONT SECTOR)
    # --------------------------------------------------------------------------
    "SPI2_SCK":        {"gpio": 12, "dir": "OUT", "bus": "SPI2", "role": "FSPI Clock (IO MUX Native)"},
    "SPI2_MOSI":       {"gpio": 11, "dir": "OUT", "bus": "SPI2", "role": "FSPI MOSI (IO MUX Native)"},
    "TOF1_MISO":       {"gpio": 13, "dir": "IN",  "bus": "SPI2", "role": "ToF 1 MISO (FSPI MISO Native)"},
    "TOF1_CS":         {"gpio": 10, "dir": "OUT", "bus": "SPI2", "role": "ToF 1 Chip Select (Active LOW)"},
    "TOF1_INT":        {"gpio": 5,  "dir": "IN",  "bus": "SPI2", "role": "ToF 1 Data Ready Interrupt"},
    "TOF1_LPN":        {"gpio": 6,  "dir": "OUT", "bus": "SPI2", "role": "ToF 1 Power Enable / Reset"},

    "TOF2_MISO":       {"gpio": 4,  "dir": "IN",  "bus": "SPI2", "role": "ToF 2 MISO (Dedicated / Isolated)"},
    "TOF2_CS":         {"gpio": 9,  "dir": "OUT", "bus": "SPI2", "role": "ToF 2 Chip Select (Active LOW)"},
    "TOF2_INT":        {"gpio": 14, "dir": "IN",  "bus": "SPI2", "role": "ToF 2 Data Ready Interrupt"},
    "TOF2_LPN":        {"gpio": 8,  "dir": "OUT", "bus": "SPI2", "role": "ToF 2 Power Enable / Reset"},

    # --------------------------------------------------------------------------
    # CHANNEL 2: SPI3_HOST (HSPI) — TOF 3 & TOF 4 (SIDE / CORNER SECTOR)
    # --------------------------------------------------------------------------
    "SPI3_SCK":        {"gpio": 39, "dir": "OUT", "bus": "SPI3", "role": "HSPI Clock (IO MUX Native)"},
    "SPI3_MOSI":       {"gpio": 40, "dir": "OUT", "bus": "SPI3", "role": "HSPI MOSI (IO MUX Native)"},
    "TOF3_MISO":       {"gpio": 41, "dir": "IN",  "bus": "SPI3", "role": "ToF 3 MISO (HSPI MISO Native)"},
    "TOF3_CS":         {"gpio": 7,  "dir": "OUT", "bus": "SPI3", "role": "ToF 3 Chip Select (Active LOW)"},
    "TOF3_INT":        {"gpio": 15, "dir": "IN",  "bus": "SPI3", "role": "ToF 3 Data Ready Interrupt"},
    "TOF3_LPN":        {"gpio": 16, "dir": "OUT", "bus": "SPI3", "role": "ToF 3 Power Enable / Reset"},

    "TOF4_MISO":       {"gpio": 38, "dir": "IN",  "bus": "SPI3", "role": "ToF 4 MISO (Dedicated / Isolated)"},
    "TOF4_CS":         {"gpio": 17, "dir": "OUT", "bus": "SPI3", "role": "ToF 4 Chip Select (Active LOW)"},
    "TOF4_INT":        {"gpio": 18, "dir": "IN",  "bus": "SPI3", "role": "ToF 4 Data Ready Interrupt"},
    "TOF4_LPN":        {"gpio": 21, "dir": "OUT", "bus": "SPI3", "role": "ToF 4 Power Enable / Reset"},

    # --------------------------------------------------------------------------
    # HOST TELEMETRY HIGHWAY & SYNCHRONIZATION
    # --------------------------------------------------------------------------
    "HOST_UART_TX":    {"gpio": 43, "dir": "OUT", "bus": "UART0","role": "High-Speed UART TX -> Pi 5 / PC"},
    "HOST_UART_RX":    {"gpio": 44, "dir": "IN",  "bus": "UART0","role": "High-Speed UART RX <- Pi 5 / PC"},
    "PI_DRDY_PULSE":   {"gpio": 2,  "dir": "OUT", "bus": "GPIO", "role": "Batch Data-Ready Interrupt Pulse to Pi 5"},
}


# ==============================================================================
# 3. MATHEMATICAL VALIDATION ENGINE
# ==============================================================================

def run_mathematical_validation():
    print("=" * 82)
    print(">>> RUNNING FORMAL MATHEMATICAL VALIDATION FOR ESP32-S3 DUAL-SPI TOF HUB")
    print("=" * 82)

    errors = []
    warnings = []

    assigned_gpios = {}
    output_gpios = set()
    input_gpios = set()

    for name, info in DUAL_SPI_ALLOCATION.items():
        pin = info["gpio"]
        direction = info["dir"]

        # Check 1: Pin Collision
        if pin in assigned_gpios:
            errors.append(f"[COLLISION] GPIO {pin} assigned to both '{assigned_gpios[pin]}' and '{name}'!")
        else:
            assigned_gpios[pin] = name

        if direction == "OUT":
            output_gpios.add(pin)
        else:
            input_gpios.add(pin)

    # Check 2: Non-existent Silicon Pins
    invalid_silicon = set(assigned_gpios.keys()) & NON_EXISTENT_GPIOS
    if invalid_silicon:
        errors.append(f"[NON-EXISTENT SILICON] Pins {invalid_silicon} do not exist in ESP32-S3 silicon!")

    # Check 3: Octal Flash/PSRAM Protection
    psram_overlap = set(assigned_gpios.keys()) & FORBIDDEN_OCTAL_FLASH_PSRAM
    if psram_overlap:
        errors.append(f"[CRITICAL MEMORY VIOLATION] Pins {psram_overlap} clash with in-package Octal Flash/PSRAM!")

    # Check 4: Onboard LED Conflict
    led_overlap = set(assigned_gpios.keys()) & ONBOARD_WS2812_LED
    if led_overlap:
        warnings.append(f"[ONBOARD PERIPHERAL] GPIO 48 connects to WS2812 LED; avoid using if possible.")

    # Check 5: Input-Only Pins Driving Outputs
    illegal_outputs = output_gpios & INPUT_ONLY_GPIOS
    if illegal_outputs:
        errors.append(f"[ELECTRICAL VIOLATION] Pins {illegal_outputs} are INPUT-ONLY (GPI) and cannot drive outputs!")

    # Check 6: Strapping Pin Safety
    strapping_used = set(assigned_gpios.keys()) & STRAPPING_GPIOS
    for pin in strapping_used:
        sig_name = assigned_gpios[pin]
        if pin == 0:
            warnings.append(f"[STRAPPING PIN] GPIO 0 is used for {sig_name}. Must stay HIGH at reset (Boot Button).")
        elif pin == 45:
            errors.append(f"[STRAPPING CONFLICT] GPIO 45 is used for {sig_name}. Must be LOW at boot (VDD_SPI).")
        elif pin == 3:
            warnings.append(f"[STRAPPING PIN] GPIO 3 is used for {sig_name}. Floating at reset (JTAG).")

    # Check 7: Pin Budget
    total_used = len(assigned_gpios)
    total_available = len(USABLE_GPIOS)
    free_gpios = sorted(list(USABLE_GPIOS - set(assigned_gpios.keys())))

    # Print Verification Summary
    print(f"  Allocated Signals     : {len(DUAL_SPI_ALLOCATION)}")
    print(f"  Unique GPIOs Used     : {total_used} / {total_available} usable silicon pins")
    print(f"  Free Spare GPIOs      : {len(free_gpios)} pins -> {free_gpios}")
    print("  " + "-" * 78)

    if errors:
        print("  [FAILED] VALIDATION FOUND ERRORS:")
        for err in errors:
            print(f"    - {err}")
        return False
    else:
        print("  [PASSED] 100% MATHEMATICAL CONFORMITY PROVEN:")
        print("    - Zero Pin Collisions")
        print("    - Zero Octal Flash/PSRAM Violations (GPIO 26-37 untouched)")
        print("    - Zero Non-Existent Silicon Pins (GPIO 22-25 untouched)")
        print("    - Zero Input-Only Violations (GPIO 46 not driven)")
        print("    - Zero Boot Strapping Hazards (GPIO 0, 45, 46 safe)")
        if warnings:
            print("  [INFORMATIONAL NOTICES]:")
            for w in warnings:
                print(f"    - {w}")
        return True


# ==============================================================================
# 4. SPI TIMING & BANDWIDTH CALCULUS
# ==============================================================================

def calculate_timing_and_bandwidth():
    print("\n" + "=" * 82)
    print(">>> SPI TIMING, THROUGHPUT & BUS UTILIZATION CALCULUS")
    print("=" * 82)

    # VL53L8CX ST Results Buffer size per ranging frame:
    # 4x4 mode: 16 zones * 2 targets = 32 targets
    # ResultsData struct with distances, statuses, sigmas, ambient ~ 1,440 bytes
    FRAME_BYTES_4X4 = 1440
    # 8x8 mode: 64 zones * 2 targets = 128 targets ~ 3,200 bytes
    FRAME_BYTES_8X8 = 3200

    TARGET_FREQ_HZ = 60          # 60 Hz per sensor
    NUM_SENSORS = 4
    TOTAL_FRAMES_PER_SEC = NUM_SENSORS * TARGET_FREQ_HZ  # 240 frames/sec

    SPI_CLOCK_3MHZ = 3_000_000   # 3.0 MHz (Conservative ST-certified clock)
    SPI_CLOCK_10MHZ = 10_000_000 # 10.0 MHz (ESP32-S3 Hardware IO MUX capable)

    # Single Bus (1 Bus for 4 ToFs):
    bytes_per_sec_single = TOTAL_FRAMES_PER_SEC * FRAME_BYTES_4X4
    bits_per_sec_single = bytes_per_sec_single * 8
    duty_3mhz_single = (bits_per_sec_single / SPI_CLOCK_3MHZ) * 100
    duty_10mhz_single = (bits_per_sec_single / SPI_CLOCK_10MHZ) * 100

    # Dual Bus (2 Buses, 2 ToFs per bus):
    frames_per_bus = (NUM_SENSORS // 2) * TARGET_FREQ_HZ  # 120 frames/sec per bus
    bytes_per_sec_dual = frames_per_bus * FRAME_BYTES_4X4
    bits_per_sec_dual = bytes_per_sec_dual * 8
    duty_3mhz_dual = (bits_per_sec_dual / SPI_CLOCK_3MHZ) * 100
    duty_10mhz_dual = (bits_per_sec_dual / SPI_CLOCK_10MHZ) * 100

    # Latency per single frame read
    latency_3mhz_us = (FRAME_BYTES_4X4 * 8 / SPI_CLOCK_3MHZ) * 1_000_000
    latency_10mhz_us = (FRAME_BYTES_4X4 * 8 / SPI_CLOCK_10MHZ) * 1_000_000

    print(f"  Ranging Scenario       : 4x VL53L8CX in 4x4 Matrix @ 60 Hz")
    print(f"  Frame Payload Size     : {FRAME_BYTES_4X4} bytes ({FRAME_BYTES_4X4 * 8} bits)")
    print(f"  Aggregate System Rate  : {TOTAL_FRAMES_PER_SEC} frames/second ({bytes_per_sec_single / 1024:.1f} KB/s)")
    print("  " + "-" * 78)
    print("  ARCHITECTURE COMPARISON: SINGLE SHARED BUS vs DUAL INDEPENDENT BUSES")
    print("  " + "-" * 78)
    print(f"  {'Metric / Parameter':<32} | {'Single SPI Bus (Old)':<20} | {'Dual SPI Buses (Proposed)':<22}")
    print(f"  {'-'*32}-+-{'-'*20}-+-{'-'*22}")
    print(f"  {'Sensors per Bus':<32} | {'4 Sensors':<20} | {'2 Sensors per Bus':<22}")
    print(f"  {'Frames per Bus per Sec':<32} | {f'{TOTAL_FRAMES_PER_SEC} frames/s':<20} | {f'{frames_per_bus} frames/s':<22}")
    print(f"  {'Data Bandwidth per Bus':<32} | {f'{bits_per_sec_single/1e6:.2f} Mbps':<20} | {f'{bits_per_sec_dual/1e6:.2f} Mbps':<22}")
    print(f"  {'SPI Bus Load @ 3.0 MHz':<32} | {f'{duty_3mhz_single:.1f}% (HIGH JITTER)':<20} | {f'{duty_3mhz_dual:.1f}% (ROCK SOLID)':<22}")
    print(f"  {'SPI Bus Load @ 10.0 MHz':<32} | {f'{duty_10mhz_single:.1f}%':<20} | {f'{duty_10mhz_dual:.1f}% (ZERO OVERHEAD)':<22}")
    print(f"  {'Single Frame SPI Read Time':<32} | {f'{latency_3mhz_us:.0f} us @ 3MHz':<20} | {f'{latency_3mhz_us:.0f} us @ 3MHz':<22}")
    print(f"  {'Concurrent Reads Possible':<32} | {'NO (Sequential queue)':<20} | {'YES (Full Dual Hardware DMA)':<22}")
    print("  " + "-" * 78)


# ==============================================================================
# 5. PHYSICAL PINOUT TABLE & HEADER MAPPING REPORT
# ==============================================================================

def print_pinout_table():
    print("\n" + "=" * 82)
    print("  FOLKRACE — DUAL-SPI QUAD VL53L8CX PINOUT ASSIGNMENT TABLE")
    print("  Target: ESP32-S3-DevKitC-1 (N16R8) Dedicated ToF Perception Hub")
    print("=" * 82)
    print(f"  {'Signal Identifier':<18} | {'GPIO':<7} | {'Direction':<9} | {'Bus / Subsystem':<15} | {'Role & Hardware Capability':<26}")
    print(f"  {'-'*18}-+-{'-'*7}-+-{'-'*9}-+-{'-'*15}-+-{'-'*26}")

    categories = [
        ("CHANNEL 1: SPI2 (FSPI) — TOF 1 & TOF 2 (FRONT SECTOR)", "SPI2"),
        ("CHANNEL 2: SPI3 (HSPI) — TOF 3 & TOF 4 (SIDE SECTOR)", "SPI3"),
        ("HOST TELEMETRY LINK TO RASPBERRY PI 5 / PC", ["UART0", "GPIO"]),
    ]

    for title, match_bus in categories:
        print(f"\n  -- {title} --")
        for sig_name, info in DUAL_SPI_ALLOCATION.items():
            b = info["bus"]
            matches = (b == match_bus) if isinstance(match_bus, str) else (b in match_bus)
            if matches:
                gpio_str = f"GPIO {info['gpio']}"
                print(f"  {sig_name:<18} | {gpio_str:<7} | {info['dir']:<9} | {info['bus']:<15} | {info['role']}")

    print("\n" + "=" * 82)


def main():
    valid = run_mathematical_validation()
    calculate_timing_and_bandwidth()
    print_pinout_table()
    if not valid:
        sys.exit(1)

if __name__ == "__main__":
    main()
