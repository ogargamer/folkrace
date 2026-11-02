#!/usr/bin/env python3
"""
pin_solver.py
=============
Master STM32G474 Pinout & Peripheral Assignment Solver for FOLKRACE Dual/Quad-BLDC Controller.

Features:
  1. 4-Motor PWM Inverter Control (12 PWM pins: 3 channels/timer across TIM1..5, TIM8)
  2. Telemetry UART Highway (USART1, USART2, USART3, LPUART1)
  3. Dual-SPI Split Bus Architecture (SPI1 Front Axle + SPI2 Rear Axle, 6 SPI signals + 4 CS pins)
  4. 4x Motor Temperature NTC Thermistors (4 dedicated ADC analog input channels)
  5. Inverter Enable Options:
     - Option A: 4 Individual Enables (M1_EN, M2_EN, M3_EN, M4_EN) -> SWD flashing
     - Option B: 2 Axle Enables (Front_EN for M1+M2, Rear_EN for M3+M4) -> Native USB + SWD + BOOT0 preserved!
     - Option C: 1 Master Global Enable (ALL_EN) -> Native USB + SWD + BOOT0 preserved!
  6. Flashing & Boot Pin Isolation:
     - BOOT0 (PB8) isolation to prevent jumper toggling during flashing
     - SWD (PA13/PA14) hardware debug & zero-button flashing preservation
     - Native USB (PA11/PA12) preservation
  7. Hardware Pin Traps & Onboard Peripherals:
     - PA8: Onboard Blue LED hardwired on WeAct / STM32G474 dev boards (causes parasitic loading on Phase A if used for motor PWM)
"""

import sys
import os
import csv
import itertools

# ---------------------------------------------------------------------------
# Physical Hardware Pinout Database for STM32G474 (LQFP48)
# ---------------------------------------------------------------------------
RAW_PINOUT_CSV = """Board Pin,MCU Pin,TIM,HRTIM,USART/UART,SPI/I2S,I2C,Analog,Other
A0,PA0,"TIM2_CH1, TIM5_CH1, TIM8_BKIN, TIM8_ETR, TIM2_ETR",,,USART2_CTS,,"ADC12_IN1, COMP1_INM, COMP3_INP, COMP1_OUT",RTC_TAMP2, WKUP1
A1,PA1,"TIM2_CH2, TIM5_CH2, TIM15_CH1N",,,USART2_RTS_DE,,"ADC12_IN2, COMP1_INP, OPAMP1_VINP, OPAMP3_VINP, OPAMP6_VINM",RTC_REFIN
A2,PA2,"TIM2_CH3, TIM5_CH3, TIM15_CH1",,,"USART2_TX, LPUART1_TX",,,"ADC1_IN3, COMP2_INM, OPAMP1_VOUT, COMP2_OUT",QUADSPI1_BK1_NCS, UCPD1_FRSTX, WKUP4/LSCO
A3,PA3,"TIM2_CH4, TIM5_CH4, TIM15_CH2",,,"USART2_RX, LPUART1_RX",,,"ADC1_IN4, COMP2_INP, OPAMP1_VINM, OPAMP1_VINP, OPAMP5_VINM",SAI1_CK1, QUADSPI1_CLK, SAI1_MCLK_A
A4,PA4,TIM3_CH2,,,USART2_CK,"SPI1_NSS, SPI3_NSS, I2S3_WS",,"ADC2_IN17, DAC1_OUT1, COMP1_INM",SAI1_FS_B
A5,PA5,"TIM2_CH1, TIM2_ETR",,,,SPI1_SCK,,"ADC2_IN13, DAC1_OUT2, COMP2_INM, OPAMP2_VINM",UCPD1_FRSTX
A6,PA6,"TIM16_CH1, TIM3_CH1, TIM8_BKIN, TIM1_BKIN",,,LPUART1_CTS,SPI1_MISO,,"ADC2_IN3, DAC2_OUT1, OPAMP2_VOUT, COMP1_OUT",QUADSPI1_BK1_IO3
A7,PA7,"TIM17_CH1, TIM3_CH2, TIM8_CH1N, TIM1_CH1N",,,,SPI1_MOSI,,"ADC2_IN4, COMP2_INP, OPAMP1_VINP, OPAMP2_VINP, COMP2_OUT",QUADSPI1_BK1_IO2, UCPD1_FRSTX
A8,PA8,"TIM1_CH1, TIM4_ETR",HRTIM1_CHA1,USART1_CK,I2S2_MCK,"I2C3_SCL, I2C2_SDA","ADC5_IN1, OPAMP5_VOUT, COMP7_OUT",MCO, FDCAN3_RX, SAI1_CK2, SAI1_SCK_A, Onboard Blue LED
A9,PA9,"TIM1_CH2, TIM15_BKIN, TIM2_CH3",HRTIM1_CHA2,USART1_TX,I2S3_MCK,"I2C3_SMBA, I2C2_SCL","ADC5_IN2, COMP5_OUT",SAI1_FS_A, UCPD1_DBCC1
A10,PA10,"TIM17_BKIN, TIM1_CH3, TIM2_CH4, TIM8_BKIN",HRTIM1_CHB1,USART1_RX,SPI2_MISO,I2C2_SMBA,COMP6_OUT,"USB_CRS_SYNC, SAI1_D1, SAI1_SD_A, UCPD1_DBCC2
A11,PA11,"TIM1_CH1N, TIM4_CH1, TIM1_CH4, TIM1_BKIN2",HRTIM1_CHB2,USART1_CTS,"SPI2_MOSI, I2S2_SD",,COMP1_OUT,"FDCAN1_RX, USB_DM
A12,PA12,"TIM16_CH1, TIM1_CH2N, TIM4_CH2, TIM1_ETR",HRTIM1_FLT1,USART1_RTS_DE,I2SCKIN,,COMP2_OUT,"FDCAN1_TX, USB_DP
DIO (A13),PA13,TIM16_CH1N, TIM4_CH3,,,USART3_CTS,,"I2C4_SCL, I2C1_SCL",,SWDIO-JTMS, IR_OUT, SAI1_SD_B
CLK (A14),PA14,"TIM8_CH2, TIM1_BKIN",,,USART2_TX,,"I2C4_SMBA, I2C1_SDA",,SWCLK-JTCK, LPTIM1_OUT, SAI1_FS_B
A15,PA15,"TIM2_CH1, TIM8_CH1, TIM1_BKIN, TIM2_ETR",HRTIM1_FLT2,"USART2_RX, UART4_RTS_DE","SPI1_NSS, SPI3_NSS, I2S3_WS",I2C1_SCL,,JTDI, FDCAN3_TX
B0,PB0,"TIM3_CH3, TIM8_CH2N, TIM1_CH2N",HRTIM1_FLT5,,,,,"ADC3_IN12, ADC1_IN15, COMP4_INP, OPAMP2_VINP, OPAMP3_VINP",QUADSPI1_BK1_IO1, UCPD1_FRSTX
B1,PB1,"TIM3_CH4, TIM8_CH3N, TIM1_CH3N",HRTIM1_SCOUT,,LPUART1_RTS_DE,,,"ADC3_IN1, ADC1_IN12, COMP1_INP, OPAMP3_VOUT, OPAMP6_VINM, COMP4_OUT",QUADSPI1_BK1_IO0
B2,PB2,"TIM5_CH1, TIM20_CH1",HRTIM1_SCIN,,,I2C3_SMBA,"ADC2_IN12, COMP4_INM, OPAMP3_VINM","RTC_OUT2, LPTIM1_OUT, QUADSPI1_BK2_IO1
B3,PB3,"TIM2_CH2, TIM4_ETR, TIM8_CH1N, TIM3_ETR","HRTIM1_SCOUT, HRTIM1_EEV9",USART2_TX,"SPI1_SCK, SPI3_SCK, I2S3_CK",,,JTDO-TRACESWO, UCPD1_CRS_SYNC, FDCAN3_RX, SAI1_SCK_B
B4,PB4,"TIM16_CH1, TIM3_CH1, TIM8_CH2N, TIM17_BKIN",HRTIM1_EEV7,"USART2_RX, UART5_RTS_DE","SPI1_MISO, SPI3_MISO",,,JTRST, FDCAN3_TX, SAI1_MCLK_B, UCPD1_CC2
B5,PB5,"TIM16_BKIN, TIM3_CH2, TIM8_CH3N, TIM17_CH1",HRTIM1_EEV6,"USART2_CK, UART5_CTS","SPI1_MOSI, SPI3_MOSI, I2S3_SD","I2C1_SMBA, I2C3_SDA",,FDCAN2_RX, LPTIM1_IN1, SAI1_SD_B
B6,PB6,"TIM16_CH1N, TIM4_CH1, TIM8_CH1, TIM8_ETR, TIM8_BKIN2","HRTIM1_SCIN, HRTIM1_EEV4",USART1_TX,,,COMP4_OUT,"FDCAN2_TX, LPTIM1_ETR, SAI1_FS_B, UCPD1_CC1
B7,PB7,"TIM17_CH1N, TIM4_CH2, TIM8_BKIN, TIM3_CH4",HRTIM1_EEV3,"USART1_RX, UART4_CTS,","I2C4_SDA, I2C1_SDA",COMP3_OUT,"LPTIM1_IN2, FMC_NL, PVD_IN
BOOT0 (B8),PB8,"TIM16_CH1, TIM4_CH3, TIM8_CH2, TIM1_BKIN",HRTIM1_EEV8,USART3_RX,,I2C1_SCL,COMP1_OUT,"BOOT0, SAI1_CK1, FDCAN1_RX, SAI1_MCLK_A
B9,PB9,"TIM17_CH1, TIM4_CH4, TIM8_CH3, TIM1_CH3N",HRTIM1_EEV5,USART3_TX,,I2C1_SDA,COMP2_OUT,"SAI1_D2, IR_OUT, FDCAN1_TX, SAI1_FS_A
B10,PB10,"TIM2_CH3, TIM1_BKIN",HRTIM1_FLT3,"USART3_TX, LPUART1_RX",,,,"COMP5_INM, OPAMP3_VINM, OPAMP4_VINM","QUADSPI1_CLK, SAI1_SCK_A
B11,PB11,TIM2_CH4,HRTIM1_FLT4,"USART3_RX, LPUART1_TX",,,,"ADC12_IN14, COMP6_INP, OPAMP4_VINP, OPAMP6_VOUT",QUADSPI1_BK1_NCS
B12,PB12,TIM5_ETR, TIM1_BKIN,HRTIM1_CHC1,"USART3_CK, LPUART1_RTS_DE","SPI2_NSS, I2S2_WS",I2C2_SMBA,"ADC4_IN3, ADC1_IN11, COMP7_INM, OPAMP4_VOUT, OPAMP6_VINP",FDCAN2_RX
B13,PB13,TIM1_CH1N,HRTIM1_CHC2,"USART3_CTS, LPUART1_CTS","SPI2_SCK, I2S2_CK",,,"ADC3_IN5, COMP5_INP, OPAMP3_VINP, OPAMP4_VINP, OPAMP6_VINP",FDCAN2_TX
B14,PB14,"TIM15_CH1, TIM1_CH2N",HRTIM1_CHD1,USART3_RTS_DE,SPI2_MISO,,"ADC4_IN4, ADC1_IN5, COMP7_INP, OPAMP2_VINP, OPAMP5_VINP, COMP4_OUT",
B15,PB15,"TIM15_CH2, TIM15_CH1N, TIM1_CH3N",HRTIM1_CHD2,,,"SPI2_MOSI, I2S2_SD",,"ADC4_IN5, ADC2_IN15, COMP6_INM, OPAMP5_VINM, COMP3_OUT",RTC_REFIN
C13,PC13,"TIM1_BKIN, TIM1_CH1N, TIM8_CH4N",,,,,,,WKUP2, RTC_TAMP1, RTC_TS, RTC_OUT1
C14,PC14,,,,,,,,OSC32_IN
C15,PC15,,,,,,,,OSC32_OUT
NRST,PG10,TIM1_CH1,,LPUART1_RX,,,"ADC12_IN6, COMP3_INM","NRST, MCO, LPTIM1_IN1"
"""


def parse_pinout_database(csv_text):
    database = {}
    lines = csv_text.strip().split("\n")

    for line in lines:
        line = line.strip()
        if not line or line.startswith("Board Pin"):
            continue

        if line.count('"') % 2 != 0:
            line += '"'

        reader = csv.reader([line])
        try:
            row = next(reader)
        except Exception:
            continue

        if len(row) < 2:
            continue

        board_pin = row[0].strip()
        mcu_pin = row[1].strip()

        pin_info = {
            "TIM": [],
            "UART": [],
            "SPI": [],
            "I2C": [],
            "Analog": [],
            "Other": [],
            "BoardPin": board_pin,
        }

        for cell in row[2:]:
            cell = cell.strip()
            if not cell:
                continue

            sub_items = [item.strip() for item in cell.split(",") if item.strip()]
            for item in sub_items:
                item = item.replace('"', "").strip()
                if not item:
                    continue

                if item.startswith("TIM") or item.startswith("HRTIM"):
                    pin_info["TIM"].append(item)
                elif any(x in item for x in ["USART", "UART", "LPUART"]):
                    pin_info["UART"].append(item)
                elif item.startswith("SPI") or item.startswith("I2S"):
                    pin_info["SPI"].append(item)
                elif item.startswith("I2C"):
                    pin_info["I2C"].append(item)
                elif any(
                    item.startswith(x) for x in ["ADC", "DAC", "OPAMP", "COMP"]
                ):
                    pin_info["Analog"].append(item)
                else:
                    pin_info["Other"].append(item)

        database[mcu_pin] = pin_info
    return database


PIN_FUNCTIONS = parse_pinout_database(RAW_PINOUT_CSV)
PINS = list(PIN_FUNCTIONS.keys())

MOTOR_TIMER_POOL = ["TIM1", "TIM2", "TIM3", "TIM4", "TIM5", "TIM8"]
SPI_PERIPHERALS = ["SPI1", "SPI2", "SPI3"]
UARTS = ["USART1", "USART2", "USART3", "LPUART1"]
MOTOR_THERMISTOR_OWNER = "STM32G474"
MOTOR_THERMISTOR_COUNT = 4


# ---------------------------------------------------------------------------
# Matching & Solver Helpers
# ---------------------------------------------------------------------------
def get_avail_pins(ex_pc=False, ex_usb=False, ex_dbg=True, ex_boot0=False, ex_nrst=True):
    out = []
    for pin in PINS:
        if ex_pc and pin in ["PC14", "PC15"]:
            continue
        if ex_usb and pin in ["PA11", "PA12"]:
            continue
        if ex_dbg and pin in ["PA13", "PA14"]:
            continue
        if ex_boot0 and pin in ["PB8"]:
            continue
        if ex_nrst and pin in ["PG10"]:
            continue
        out.append(pin)
    return out


def find_spi_triplets(avail_pins):
    triplets = {}
    for spi in SPI_PERIPHERALS:
        sck = [p for p in avail_pins if f"{spi}_SCK" in PIN_FUNCTIONS[p]["SPI"]]
        mosi = [
            p for p in avail_pins if f"{spi}_MOSI" in PIN_FUNCTIONS[p]["SPI"]
        ]
        miso = [
            p for p in avail_pins if f"{spi}_MISO" in PIN_FUNCTIONS[p]["SPI"]
        ]
        t = [
            (s, mo, mi)
            for s in sck
            for mo in mosi
            for mi in miso
            if len({s, mo, mi}) == 3
        ]
        if t:
            triplets[spi] = t
    return triplets


def kuhn(adj, roles):
    """Kuhn bipartite matching algorithm."""
    m = {}

    def dfs(u, vis):
        for v in adj[u]:
            if v not in vis:
                vis.add(v)
                if v not in m or dfs(m[v], vis):
                    m[v] = u
                    m[u] = v
                    return True
        return False

    for u in roles:
        dfs(u, set())
    return m


# ---------------------------------------------------------------------------
# Master Dual-SPI & Thermistor System Solver (NO LED)
# ---------------------------------------------------------------------------
def solve_dual_spi_system(motor_timers, uart, avail, num_enables=4):
    """
    Solves full pin allocation for:
      - 4 Motors x 3 PWM phases (12 pins)
      - 1 UART TX/RX pair (2 pins)
      - Dual-SPI Split Buses (SPI1 Front + SPI2 Rear, 6 pins + 4 CS pins)
      - 4x Motor NTC Thermistors owned by the STM32G474 (4 ADC channels)
      - Motor Enables (num_enables: 4 individual, 2 axle-split, or 1 master)
      - (NO LED required)
    """
    pwm_roles = [f"M{i+1}_{c}" for i in range(4) for c in ["CH1", "CH2", "CH3"]]
    uart_roles = ["UART_TX", "UART_RX"]
    all_roles = pwm_roles + uart_roles

    adj = {}
    for i, tim in enumerate(motor_timers):
        for ch in ["CH1", "CH2", "CH3"]:
            r = f"M{i+1}_{ch}"
            adj[r] = [
                p for p in avail if f"{tim}_{ch}" in PIN_FUNCTIONS[p]["TIM"]
            ]
    adj["UART_TX"] = [
        p for p in avail if f"{uart}_TX" in PIN_FUNCTIONS[p]["UART"]
    ]
    adj["UART_RX"] = [
        p for p in avail if f"{uart}_RX" in PIN_FUNCTIONS[p]["UART"]
    ]

    m = kuhn(adj, all_roles)
    if sum(1 for r in all_roles if r in m) != len(all_roles):
        return False, {}

    asgn = {r: m[r] for r in all_roles}
    used = set(asgn.values())
    free = [p for p in avail if p not in used]

    spi_map = find_spi_triplets(free)
    if not spi_map:
        return False, {}

    spi_pairs = [("SPI1", "SPI2"), ("SPI1", "SPI3"), ("SPI2", "SPI3")]
    for spi_a, spi_b in spi_pairs:
        if spi_a in spi_map and spi_b in spi_map:
            for trip_a in spi_map[spi_a]:
                pins_a = set(trip_a)
                for trip_b in spi_map[spi_b]:
                    pins_b = set(trip_b)
                    if pins_a.isdisjoint(pins_b):
                        free_after_spi = [
                            p
                            for p in free
                            if p not in pins_a and p not in pins_b
                        ]
                        adc_candidates = [
                            p
                            for p in free_after_spi
                            if any(
                                "ADC" in a
                                for a in PIN_FUNCTIONS[p]["Analog"]
                            )
                        ]
                        if len(adc_candidates) < MOTOR_THERMISTOR_COUNT:
                            continue

                        for th_combo in itertools.combinations(
                            adc_candidates, MOTOR_THERMISTOR_COUNT
                        ):
                            th_set = set(th_combo)
                            free_after_th = [
                                p for p in free_after_spi if p not in th_set
                            ]

                            # Need 4 CS pins + num_enables GPIOs
                            if len(free_after_th) >= (4 + num_enables):
                                cs_pins = free_after_th[:4]
                                enables = free_after_th[4 : 4 + num_enables]

                                asgn["SPI_MODE"] = "2X_SPI"
                                asgn["NUM_ENABLES"] = num_enables
                                asgn["SPI_A_NAME"] = spi_a
                                asgn["SPI_A_SCK"] = trip_a[0]
                                asgn["SPI_A_MOSI"] = trip_a[1]
                                asgn["SPI_A_MISO"] = trip_a[2]

                                asgn["SPI_B_NAME"] = spi_b
                                asgn["SPI_B_SCK"] = trip_b[0]
                                asgn["SPI_B_MOSI"] = trip_b[1]
                                asgn["SPI_B_MISO"] = trip_b[2]

                                asgn["ENC1_CS"] = cs_pins[0]
                                asgn["ENC2_CS"] = cs_pins[1]
                                asgn["ENC3_CS"] = cs_pins[2]
                                asgn["ENC4_CS"] = cs_pins[3]

                                for idx, en in enumerate(enables):
                                    asgn[f"EN_{idx+1}"] = en
                                for idx, th in enumerate(th_combo):
                                    adc_str = [
                                        a
                                        for a in PIN_FUNCTIONS[th]["Analog"]
                                        if "ADC" in a
                                    ][0]
                                    asgn[f"TH_M{idx+1}"] = th
                                    asgn[f"TH_M{idx+1}_ADC"] = adc_str

                                return True, asgn

    return False, {}


# ---------------------------------------------------------------------------
# Formatted Report Generator
# ---------------------------------------------------------------------------
def print_solution_report(title, timers, uart, asgn):
    quad_names = [
        "M1 (Front Left)",
        "M2 (Front Right)",
        "M3 (Rear Left)",
        "M4 (Rear Right)",
    ]
    num_en = asgn["NUM_ENABLES"]

    print("\n" + "=" * 82)
    print(f"  [SUCCESS] {title}")
    print("=" * 82)
    print(f"  Motor PWM Timers : {list(timers)}")
    print(f"  Telemetry UART   : {uart} (TX: {asgn['UART_TX']}, RX: {asgn['UART_RX']})")
    print(
        f"  Front SPI1 Bus   : {asgn['SPI_A_NAME']} (SCK: {asgn['SPI_A_SCK']}, MOSI: {asgn['SPI_A_MOSI']}, MISO: {asgn['SPI_A_MISO']})"
    )
    print(
        f"  Rear SPI2 Bus    : {asgn['SPI_B_NAME']} (SCK: {asgn['SPI_B_SCK']}, MOSI: {asgn['SPI_B_MOSI']}, MISO: {asgn['SPI_B_MISO']})"
    )
    print("-" * 82)
    print(
        f"  {'Quadrant / Motor':<20} | {'PWM Phases (CH1,2,3)':<20} | {'Inverter EN':<13} | {'Encoder CS':<12} | {'Thermistor (ADC)':<18}"
    )
    print(
        f"  {'-'*20}-+-{'-'*20}-+-{'-'*13}-+-{'-'*12}-+-{'-'*18}"
    )

    for i in range(1, 5):
        q = quad_names[i - 1]
        pwm_str = f"{asgn[f'M{i}_CH1']},{asgn[f'M{i}_CH2']},{asgn[f'M{i}_CH3']}"
        if num_en == 4:
            en_str = f"{asgn[f'EN_{i}']} ({PIN_FUNCTIONS[asgn[f'EN_{i}']]['BoardPin']})"
        elif num_en == 2:
            en_str = f"Axle {1 if i <= 2 else 2}: {asgn['EN_1' if i <= 2 else 'EN_2']}"
        else:
            en_str = f"Global: {asgn['EN_1']}"

        cs_str = f"{asgn[f'ENC{i}_CS']} ({PIN_FUNCTIONS[asgn[f'ENC{i}_CS']]['BoardPin']})"
        th_str = f"{asgn[f'TH_M{i}']} ({asgn[f'TH_M{i}_ADC']})"
        print(f"  {q:<20} | {pwm_str:<20} | {en_str:<13} | {cs_str:<12} | {th_str:<18}")

    print("=" * 82)


def run_solver():
    print("=" * 82)
    print("  FOLKRACE -- MASTER STM32G474 DUAL-SPI PINOUT SOLVER (NO LED)")
    print("  4x Motors (12 PWM) + 1x UART + Dual SPI (6 pins) + 4x Thermistors (ADC) + CS + EN")
    print(f"  Thermistor owner: {MOTOR_THERMISTOR_OWNER} ({MOTOR_THERMISTOR_COUNT} direct ADC channels)")
    print("=" * 82)

    # -----------------------------------------------------------------------
    # CONFIG 1: 2 Axle Enables (Front EN + Rear EN) -> Native USB + SWD + BOOT0 Preserved
    # -----------------------------------------------------------------------
    print("\n>>> [CONFIG 1] DUAL SPI + 2 AXLE ENABLES (FRONT + REAR EN) + NATIVE USB WORKING...")
    print("    Hardware Preserved: Native USB (PA11/12), SWD Debug (PA13/14), BOOT0 (PB8)")
    avail_cfg1 = get_avail_pins(ex_pc=False, ex_usb=True, ex_dbg=True, ex_boot0=True)
    sol_cfg1 = []
    for timers in itertools.permutations(MOTOR_TIMER_POOL, 4):
        for uart in UARTS:
            ok, asgn = solve_dual_spi_system(timers, uart, avail_cfg1, num_enables=2)
            if ok:
                sol_cfg1.append((timers, uart, asgn))

    print(f"    Total Valid Solutions Found: {len(sol_cfg1)}")
    if sol_cfg1:
        t, u, a = sol_cfg1[0]
        print_solution_report("DUAL-SPI + 2 AXLE ENABLES (USB & BOOT0 WORKING)", t, u, a)

    # -----------------------------------------------------------------------
    # CONFIG 2: 4 Individual Enables -> SWD Flashing (PA11/12 as GPIOs, BOOT0 Preserved)
    # -----------------------------------------------------------------------
    print("\n>>> [CONFIG 2] DUAL SPI + 4 INDIVIDUAL ENABLES (M1..M4 EN) + BOOT0 PRESERVED...")
    print("    Hardware Preserved: SWD Debug/Flashing (PA13/14), BOOT0 (PB8) | Flashing: ST-Link / SWD")
    avail_cfg2 = get_avail_pins(ex_pc=False, ex_usb=False, ex_dbg=True, ex_boot0=True)
    sol_cfg2 = []
    for timers in itertools.permutations(MOTOR_TIMER_POOL, 4):
        for uart in UARTS:
            ok, asgn = solve_dual_spi_system(timers, uart, avail_cfg2, num_enables=4)
            if ok:
                sol_cfg2.append((timers, uart, asgn))

    print(f"    Total Valid Solutions Found: {len(sol_cfg2)}")
    if sol_cfg2:
        t, u, a = sol_cfg2[0]
        print_solution_report("DUAL-SPI + 4 INDIVIDUAL ENABLES (SWD FLASHING)", t, u, a)


if __name__ == "__main__":
    run_solver()
