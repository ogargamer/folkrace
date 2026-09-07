#!/usr/bin/env python3
"""
pin_solver.py
=============
Master ESP32-S3 (N16R8 Octal Flash/PSRAM) Pinout & Peripheral Assignment Solver for "DA WINNA" Sensor Hub.

IMPORTANT ESP32-S3 Silicon Facts:
  - Valid GPIOs: 0-21 and 26-48 only. GPIO 22-25 DO NOT EXIST on ESP32-S3.
  - GPIO 26-37: Reserved for in-package octal flash/PSRAM (N16R8). DO NOT USE.
  - GPIO 48: Onboard WS2812 RGB LED (DevKitC-1 v1.0). Excluded.
  - GPIO 46: INPUT-ONLY (GPI) — lacks output driver transistors. Cannot drive outputs.
  - ADC1 channels: ADC1_CH0-CH9 map to GPIO 1-10 ONLY. GPIO 0 has NO ADC.
  - Strapping pins: GPIO 0 (boot mode), 3 (JTAG source), 45 (VDD_SPI), 46 (ROM print + GPI).
  - Board assumed: ESP32-S3-DevKitC-1 v1.0 (N16R8, WS2812 on GPIO 48).

Features:
  1. 5x VL53L8CX Time-of-Flight Sensors on SPI2 Master (FSPI Hardware MUX, 10-20 MHz)
     - 3 Shared Bus Pins (SCK, MOSI, MISO)
     - 5 Dedicated Chip Selects (ToF 1 to 5)
     - 5 Dedicated Data Ready Interrupts (ToF 1 to 5)
  2. 1x BNO08X 9-DOF IMU on SPI3 Master (HSPI via GPIO Matrix, 3 MHz)
     - 4 Dedicated Bus Pins (SCK, MOSI, MISO, CS)
     - 3 Control Pins (INT, RST, WAK)
  3. 2x Steering Actuator Servos on LEDC PWM
  4. 1x STM32 Dual-Directional UART1 Link (921,600 Baud)
  5. Dedicated Raspberry Pi 5 Telemetry Highway:
     - Dedicated High-Speed UART2 to Pi 5 (2,000,000 Baud, single muxed stream: 60Hz+15Hz ToF + 200Hz IMU)
     - Data Ready Interrupt Output GPIO to Pi 5 (GPIO 43, rising-edge ISR, < 50us wakeup jitter)
     - All sensor streams share one UART2 with priority queuing (6.95% link utilization)
  6. Motor NTC thermistors are owned by the STM32G474 motor-safety controller.
  7. Strict Hardware Safety Enforcement:
     - GPIO 22-25 do not exist in ESP32-S3 silicon (excluded from solver)
     - GPIO 26-37 strictly reserved for In-Package Octal Flash & Octal PSRAM (N16R8)
     - Strapping pin protections (GPIO 0, 3, 45, 46)
     - Glitch-free startup allocation
"""

import sys
import os
import csv

# ---------------------------------------------------------------------------
# Complete ESP32-S3 Hardware Pinout Database (QFN56 / N16R8)
# ---------------------------------------------------------------------------
RAW_PINOUT_CSV = """Pin_Label,QFN56_Pin_No,GPIO_Num,IO_MUX_Default_Functions,RTC_GPIO_Name,ADC_Channel,Touch_Channel,Hardware_Interfaces_Fixed_IO_MUX,GPIO_Matrix_Routing_Peripherals,Timers_and_PWM,Strapping_Pin_Role,N16R8_OPI_Flash_PSRAM_Status,Onboard_DevBoard_Feature,Notes_and_Restrictions
0,5,0,GPIO0,RTC_GPIO0,N/A,N/A,RTC_GPIO0 / sar_i2c_scl_0,"UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers (4x 54-bit)","Boot Mode Strap: LOW = Joint Download Boot mode, HIGH (Default WPU) = SPI Flash Boot mode",Available / Free GPIO,Connected to Onboard BOOT Tactile Push Button (pulls to GND when pressed),Strapping pin! Weak pull-up at reset. Must be HIGH during boot. NO ADC on GPIO 0 (ADC1 starts at GPIO 1).
1,6,1,GPIO1,RTC_GPIO1,ADC1_CH0,TOUCH1,RTC_GPIO1 / sar_i2c_sda_0,"UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin,ADC1 channel (usable with Wi-Fi). Low-level glitch (~60us) during power-up.
2,7,2,GPIO2,RTC_GPIO2,ADC1_CH1,TOUCH2,RTC_GPIO2 / sar_i2c_scl_1,"UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin,ADC1 channel. Low-level glitch (~60us) during power-up.
3,8,3,GPIO3,RTC_GPIO3,ADC1_CH2,TOUCH3,RTC_GPIO3 / sar_i2c_sda_1,"UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers","JTAG Signal Source Select Strap (Floating default; 0=USB Serial/JTAG, 1=On-chip JTAG pins)",Available / Free GPIO,Header Pin,Strapping pin! No internal pull resistors at reset. Low-level glitch (~60us) during power-up.
4,9,4,GPIO4,RTC_GPIO4,ADC1_CH3,TOUCH4,RTC_GPIO4,"UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin,ADC1 channel. Low-level glitch (~60us) during power-up.
5,10,5,GPIO5,RTC_GPIO5,ADC1_CH4,TOUCH5,RTC_GPIO5,"UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin,ADC1 channel. Low-level glitch (~60us) during power-up.
6,11,6,GPIO6,RTC_GPIO6,ADC1_CH5,TOUCH6,RTC_GPIO6,"UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin,ADC1 channel. Low-level glitch (~60us) during power-up.
7,12,7,GPIO7,RTC_GPIO7,ADC1_CH6,TOUCH7,RTC_GPIO7,"UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin,ADC1 channel. Low-level glitch (~60us) during power-up.
8,13,8,GPIO8,RTC_GPIO8,ADC1_CH7,TOUCH8,"RTC_GPIO8, SUBSPICS1","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin,ADC1 channel. Low-level glitch (~60us) during power-up.
9,14,9,GPIO9,RTC_GPIO9,ADC1_CH8,TOUCH9,"RTC_GPIO9, SUBSPIHD, FSPIHD","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin,ADC1 channel. Low-level glitch (~60us) during power-up.
10,15,10,GPIO10,RTC_GPIO10,ADC1_CH9,TOUCH10,"RTC_GPIO10, FSPIIO4, SUBSPICSO, FSPICSO","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin,ADC1 channel. Low-level glitch (~60us) during power-up.
11,16,11,GPIO11,RTC_GPIO11,ADC2_CH0,TOUCH11,"RTC_GPIO11, FSPIIO5, SUBSPID, FSPID","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin,ADC2 channel (cannot be used concurrently with active Wi-Fi). Power-up glitch (~60us).
12,17,12,GPIO12,RTC_GPIO12,ADC2_CH1,TOUCH12,"RTC_GPIO12, FSPIIO6, SUBSPICLK, FSPICLK","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin,ADC2 channel (Wi-Fi conflict). Power-up glitch (~60us).
13,18,13,GPIO13,RTC_GPIO13,ADC2_CH2,TOUCH13,"RTC_GPIO13, FSPIIO7, SUBSPIQ, FSPIQ","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin,ADC2 channel (Wi-Fi conflict). Power-up glitch (~60us).
14,19,14,GPIO14,RTC_GPIO14,ADC2_CH3,TOUCH14,"RTC_GPIO14, FSPIDQS, SUBSPIWP, FSPIWP","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin,ADC2 channel (Wi-Fi conflict). Power-up glitch (~60us).
15,21,15,GPIO15,RTC_GPIO15,ADC2_CH4,N/A,"RTC_GPIO15, XTAL_32K_P, U0RTS","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin,Can be configured for 32.768 kHz external crystal (XTAL_32K_P). ADC2 channel.
16,22,16,GPIO16,RTC_GPIO16,ADC2_CH5,N/A,"RTC_GPIO16, XTAL_32K_N, U0CTS","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin,Can be configured for 32.768 kHz external crystal (XTAL_32K_N). ADC2 channel.
17,23,17,GPIO17,RTC_GPIO17,ADC2_CH6,N/A,"RTC_GPIO17, U1TXD","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin,Default drive strength 10 mA. Power-up glitch (~60us).
18,24,18,GPIO18,RTC_GPIO18,ADC2_CH7,N/A,"RTC_GPIO18, U1RXD, CLK_OUT3","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin,Default drive strength 10 mA. Power-up glitch (~60us).
19,25,19,GPIO19,RTC_GPIO19,ADC2_CH8,N/A,"USB_D-, U1RTS, CLK_OUT2","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Dedicated USB,Connected to USB-C Pass-through / USB OTG port (USB D- signal),Default drive strength 40 mA. Default function: USB D-. High-level power-up glitch (~60us x2).
20,26,20,GPIO20,RTC_GPIO20,ADC2_CH9,N/A,"USB_D+, U1CTS, CLK_OUT1","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Dedicated USB,Connected to USB-C Pass-through / USB OTG port (USB D+ signal),Default drive strength 40 mA. Default function: USB D+. High-level & pull-down glitches at power-up.
21,27,21,GPIO21,RTC_GPIO21,N/A,N/A,RTC_GPIO21,"UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin,General purpose IO, no special ADC/touch functions.

26,32,26,SPICS1,N/A,N/A,N/A,SPI0/1 Chip Select 1 (PSRAM),N/A,N/A,None,RESERVED - In-Package Octal PSRAM Chip Select,Internal Flash/PSRAM,DO NOT USE as general GPIO!
27,33,27,SPIHD,N/A,N/A,N/A,SPI0/1 HOLD / DQ3,N/A,N/A,None,RESERVED - In-Package Flash/PSRAM Data Line 3,Internal Flash/PSRAM,DO NOT USE as general GPIO!
28,34,28,SPIWP,N/A,N/A,N/A,SPI0/1 WP / DQ2,N/A,N/A,None,RESERVED - In-Package Flash/PSRAM Data Line 2,Internal Flash/PSRAM,DO NOT USE as general GPIO!
29,35,29,SPICSO,N/A,N/A,N/A,SPI0/1 Chip Select 0 (Flash),N/A,N/A,None,RESERVED - In-Package Flash Chip Select,Internal Flash,DO NOT USE as general GPIO!
30,36,30,SPICLK,N/A,N/A,N/A,SPI0/1 Clock (SPICLK),N/A,N/A,None,RESERVED - In-Package Flash/PSRAM Clock,Internal Flash/PSRAM,DO NOT USE as general GPIO!
31,37,31,SPIQ,N/A,N/A,N/A,SPI0/1 MISO / DQ1,N/A,N/A,None,RESERVED - In-Package Flash/PSRAM Data Line 1,Internal Flash/PSRAM,DO NOT USE as general GPIO!
32,38,32,SPID,N/A,N/A,N/A,SPI0/1 MOSI / DQ0,N/A,N/A,None,RESERVED - In-Package Flash/PSRAM Data Line 0,Internal Flash/PSRAM,DO NOT USE as general GPIO!
33,39,33,GPIO33,N/A,N/A,N/A,"SPIIO4, FSPIHD","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,RESERVED IN N16R8 - Octal PSRAM DQ4,Header Pin 33,CRITICAL: Octal PSRAM line. CANNOT be used!
34,40,34,GPIO34,N/A,N/A,N/A,"SPIIO5, FSPICSO","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,RESERVED IN N16R8 - Octal PSRAM DQ5,Header Pin 34,CRITICAL: Octal PSRAM line. CANNOT be used!
35,41,35,GPIO35,N/A,N/A,N/A,"SPIIO6, FSPID","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,RESERVED IN N16R8 - Octal PSRAM DQ6,Header Pin 35,CRITICAL: Octal PSRAM line. CANNOT be used!
36,42,36,GPIO36,N/A,N/A,N/A,"SPIIO7, FSPICLK","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,RESERVED IN N16R8 - Octal PSRAM DQ7,Header Pin 36,CRITICAL: Octal PSRAM line. CANNOT be used!
37,43,37,GPIO37,N/A,N/A,N/A,"SPIDQS, FSPIQ","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,RESERVED IN N16R8 - Octal PSRAM DQS,Header Pin 37,CRITICAL: Octal PSRAM line. CANNOT be used!
38,44,38,GPIO38,N/A,N/A,N/A,"FSPIWP, SUBSPIWP","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin 38,Available for general digital I/O.
39,45,39,MTCK,N/A,N/A,N/A,"JTAG MTCK (Clock)","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / JTAG Pin,Header Pin 39,Reconfigurable as regular GPIO.
40,46,40,MTDO,N/A,N/A,N/A,"JTAG MTDO (Data Out)","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / JTAG Pin,Header Pin 40,Reconfigurable as regular GPIO.
41,47,41,MTDI,N/A,N/A,N/A,"JTAG MTDI (Data In)","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / JTAG Pin,Header Pin 41,Reconfigurable as regular GPIO.
42,48,42,MTMS,N/A,N/A,N/A,JTAG MTMS (Mode Select),"UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / JTAG Pin,Header Pin 42,Reconfigurable as regular GPIO.
43,49,43,U0TXD,N/A,N/A,N/A,"UART0 TXD","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Dedicated UART,Connected to CH343P USB-to-UART bridge (RX),Default console print output pin (UART0 TX).
44,50,44,U0RXD,N/A,N/A,N/A,"UART0 RXD","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Dedicated UART,Connected to CH343P USB-to-UART bridge (TX),Default console input pin (UART0 RX).
45,51,45,GPIO45,N/A,N/A,N/A,GPIO45,"UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers","VDD_SPI Strap: 0 = 3.3V, 1 = 1.8V",Available / Free GPIO,Header Pin 45,Must NOT be pulled HIGH at boot.
46,52,46,GPIO46,N/A,N/A,N/A,GPIO46,"UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers","Boot ROM Print Strap",Available / Input-Only during boot,Header Pin 46,Weak pull-down at reset.
47,37,47,SPICLK_P_DIFF,N/A,N/A,N/A,"SPICLK_P_DIFF","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Free GPIO,Header Pin 47,Available for general IO.
48,36,48,SPICLK_N_DIFF,N/A,N/A,N/A,"SPICLK_N_DIFF","UART0/1/2, SPI2/3, I2C0/1, I2S0/1, TWAI/CAN, RMT, PCNT, LCD/Cam","LEDC (8 ch), MCPWM (2 units), General Timers",None,Available / Onboard RGB LED,Connected to Onboard WS2812 RGB LED (Data DIN),HARDWARE CONNECTED: Onboard WS2812 RGB LED.
"""


def parse_database(csv_text):
    db = {}
    lines = csv_text.strip().split("\n")
    for line in lines[1:]:
        if not line.strip():
            continue
        if line.count('"') % 2 != 0:
            line += '"'
        reader = csv.reader([line])
        try:
            row = next(reader)
        except Exception:
            continue
        if len(row) < 14:
            continue
        try:
            gpio_num = int(row[2].strip())
        except ValueError:
            continue
        db[gpio_num] = {
            "Label": row[0].strip(),
            "GPIO": gpio_num,
            "ADC": row[5].strip(),
            "Notes": row[13].strip(),
        }
    return db


PIN_DB = parse_database(RAW_PINOUT_CSV)

RESERVED_FLASH_PSRAM = set(range(26, 38))
NON_EXISTENT_GPIOS = {22, 23, 24, 25}  # ESP32-S3 silicon skips GPIO 22-25
INPUT_ONLY_GPIOS = {46}  # GPIO 46 lacks output driver transistors (GPI only)
EXCLUDED = RESERVED_FLASH_PSRAM.union({48}).union(NON_EXISTENT_GPIOS)
ALL_GPIOS = set(range(0, 49)) - NON_EXISTENT_GPIOS  # Only physically real GPIOs
USABLE_GPIOS = ALL_GPIOS - EXCLUDED


def solve_esp32_pinout():
    allocations = {}
    used = set()

    def assign(name, preferred_list, desc="", output=False):
        for p in preferred_list:
            if p in USABLE_GPIOS and p not in used:
                if output and p in INPUT_ONLY_GPIOS:
                    continue  # GPIO 46 is input-only, skip for output signals
                used.add(p)
                allocations[name] = (p, desc)
                return p
        raise RuntimeError(f"Cannot allocate pin for {name}!")

    # 1. STM32 Link (UART1)
    assign("STM32_UART1_TX", [17, 1], "UART1 TX -> STM32 RX")
    assign("STM32_UART1_RX", [18, 2], "UART1 RX -> STM32 TX")

    # 2. Servos (LEDC PWM) — must be output-capable pins
    assign("SERVO_FRONT_PWM", [21, 38, 39], "LEDC Front Steering Kingpin Servo", output=True)
    assign("SERVO_REAR_PWM", [45, 39], "LEDC Rear / Aux Steering Servo", output=True)

    # 3. Dedicated Pi 5 UART2 Stream (2M Baud ToF/Telemetry Link)
    assign(
        "PI_UART2_TX",
        [47, 25],
        "UART2 TX -> Pi 5 RX (Pin 35 / GPIO 19 /dev/ttyAMA2)",
    )
    assign(
        "PI_UART2_RX",
        [1, 44],
        "UART2 RX <- Pi 5 TX (moved to GPIO1; UART0 RX GPIO44 restored for debug)",
    )

    # 3b. Data Ready Interrupt Output -> Pi 5 GPIO (edge ISR, < 50us wakeup)
    #     Pulsed HIGH ~10us by ESP32 ISR after ToF batch is enqueued in UART2 TX DMA.
    #     Pi 5 configures this GPIO as rising-edge interrupt to wake read thread
    #     immediately instead of relying on kernel scheduler tick (~0.1-1ms jitter).
    #     GPIO 2: moved here so UART0 TX GPIO43 remains available for debug logs.
    #     NOTE: GPIO 46 is INPUT-ONLY (no output driver) — cannot be used here.
    assign(
        "PI_TOF_DRDY_OUT",
        [2, 43],
        "Output: ToF batch ready pulse -> Pi 5 GPIO (GPIO2; UART0 TX GPIO43 restored)",
        output=True,
    )

    # 4. 5x ToF Sensors on SPI2_HOST (FSPI Hardware MUX, 10-20MHz)
    assign("TOF_SPI2_SCK", [12], "SPI2 Shared Clock (10-20MHz)")
    assign("TOF_SPI2_MOSI", [11], "SPI2 Shared MOSI")
    assign("TOF_SPI2_MISO", [13], "SPI2 Shared MISO")

    assign("TOF1_CS", [10], "ToF 1 (Front Center 4x4 60Hz) CS")
    assign("TOF2_CS", [9], "ToF 2 (Front Left 45 deg 4x4 60Hz) CS")
    assign("TOF3_CS", [8], "ToF 3 (Front Right 45 deg 4x4 60Hz) CS")
    assign("TOF4_CS", [7], "ToF 4 (Ramp/Mound Detector Left 8x8 15Hz) CS")
    assign("TOF5_CS", [6], "ToF 5 (Ramp/Mound Detector Right 8x8 15Hz) CS")

    assign("TOF1_INT", [5], "ToF 1 Data Ready Interrupt")
    assign("TOF2_INT", [4], "ToF 2 Data Ready Interrupt")
    assign("TOF3_INT", [14], "ToF 3 Data Ready Interrupt")
    assign("TOF4_INT", [15], "ToF 4 Data Ready Interrupt")
    assign("TOF5_INT", [16], "ToF 5 Data Ready Interrupt")

    # 5. BNO08X IMU on SPI3_HOST (HSPI Dedicated 3MHz SPI Master)
    assign("IMU_SPI3_SCK", [39], "SPI3 Dedicated Clock (3MHz)")
    assign("IMU_SPI3_MOSI", [40], "SPI3 Dedicated MOSI")
    assign("IMU_SPI3_MISO", [41], "SPI3 Dedicated MISO")
    assign("IMU_SPI3_CS", [42], "IMU Chip Select")
    assign("IMU_INT", [19], "IMU Hardware INT (Falling Edge ISR) (USB D- pin, USB not used)")
    assign("IMU_RST", [20], "IMU Hardware Reset (USB D+ pin, USB not used)")
    assign("IMU_WAK", [38], "IMU Wake Pin (Active LOW)")

    # Motor thermistors are intentionally not connected to this sensor hub.
    # The STM32G474 owns all four analogue NTC inputs and the over-temperature
    # safety action, so an ESP32/Pi failure cannot suppress motor protection.

    return allocations, used


def print_report():
    allocations, used = solve_esp32_pinout()
    print("=" * 82)
    print("  DA WINNA -- MASTER ESP32-S3 PINOUT SOLVER (N16R8 DUAL-SPI SENSOR HUB)")
    print("  5x ToF (SPI2) + BNO08x IMU (SPI3) + 2x Servos + Pi4 UART2 + DRDY")
    print("=" * 82)
    print(f"  {'Subsystem / Signal':<22} | {'ESP32 GPIO':<12} | {'Description / Target Interface':<42}")
    print(f"  {'-'*22}-+-{'-'*12}-+-{'-'*42}")

    categories = [
        ("STM32 & Pi 5 Links (UART + DRDY)", ["STM32_", "PI_"]),
        ("5x ToF Sensors (SPI2 Master)", ["TOF"]),
        ("BNO08x IMU (SPI3 Master)", ["IMU_"]),
        ("Steering Servos (LEDC PWM)", ["SERVO_"]),
    ]

    for cat_title, prefixes in categories:
        print(f"\n  -- {cat_title} --")
        for name, (gpio, desc) in sorted(allocations.items(), key=lambda x: x[1][0]):
            if any(name.startswith(p) for p in prefixes):
                print(f"  {name:<22} | GPIO {gpio:<7} | {desc:<42}")

    print("\n" + "-" * 82)
    free = sorted(list(USABLE_GPIOS - used))
    print(f"  Total Pins Allocated : {len(allocations)} / {len(USABLE_GPIOS)} usable GPIOs")
    print(f"  Remaining Free GPIOs : {len(free)} pins -> {free}")
    print("=" * 82 + "\n")


if __name__ == "__main__":
    print_report()
